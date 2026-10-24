package com.example.slamcapture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.media.ImageReader
import android.os.Handler
import android.util.Log
import android.util.Range
import android.view.Surface

/**
 * 相机：只负责打开相机和把帧交出去，不判断何时落盘。
 *
 * 帧的唯一出口是 [onFrame]。
 *
 * 相机只有「开」和「关」两种状态；[ready] 和 [focusLockArmed] 只在 [start]、[stop]
 * 以及 [releaseResources] 中复位。要重新对焦就再次 [beginSettle]。
 */
class CameraCapture(
    private val context: Context,
    private val handler: Handler
) {
    companion object {
        const val TAG = "CameraCapture"

        // ORB-SLAM3 的常用工作分辨率；不要提高到 1280x720，否则特征提取跟不上 30fps。
        const val W = 640
        const val H = 480

        // 多后摄设备可能自动选错，可在这里写死相机 id（例如 "0"）；null 表示自动选择。
        // 可空类型不能使用 const。
        val CAMERA_ID_OVERRIDE: String? = null
    }

    private val manager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var reader: ImageReader? = null
    private var fpsRange: Range<Int>? = null
    private var surface: Surface? = null

    /**
     * 帧出口，运行在相机线程上，不能阻塞。
     * 无论能否落盘都会调用，以便预览显示当前状态。
     */
    @Volatile
    var onFrame: ((gray: ByteArray, tsNs: Long) -> Unit)? = null

    /**
     * 相机输出是否稳定到可以信任。
     *
     * 当前等价于对焦已收敛或已锁定；读不到有效对焦距离时保持连续对焦，也算就绪。
     * 只由相机线程写。
     */
    @Volatile
    var ready = false
        private set

    /** 错误说明，供状态栏显示；正常为 null。 */
    @Volatile
    var lastError: String? = null
        private set

    /** 把 Camera2 错误码转换为可读文本。 */
    // 这些常量定义在 CameraDevice.StateCallback 上，不在 CameraDevice 上。
    private fun cameraErrorText(error: Int): String = when (error) {
        CameraDevice.StateCallback.ERROR_CAMERA_IN_USE ->
            "相机被占用 —— 上一个页面可能还没放开，再试一次"
        CameraDevice.StateCallback.ERROR_MAX_CAMERAS_IN_USE -> "相机数量已达上限"
        CameraDevice.StateCallback.ERROR_CAMERA_DISABLED -> "相机被系统策略禁用"
        CameraDevice.StateCallback.ERROR_CAMERA_DEVICE -> "相机设备出错"
        CameraDevice.StateCallback.ERROR_CAMERA_SERVICE -> "相机服务出错"
        else -> "相机打开失败 error=$error"
    }

    /**
     * 相机代数：奇数为运行中，偶数为已停止。
     *
     * 每个回调捕获创建时的代数，不匹配就关闭自己的资源并返回，不写入字段。
     * 只在相机线程读写。
     */
    private var epoch = 0L

    /** 本轮是否要求锁定对焦。[beginSettle] 置 true，[releaseResources] 置 false。 */
    private var focusLockArmed = false

    /** 镜头是否支持自动对焦。 */
    private var supportsAf = false

    // ---------------- 开合 ----------------

    /**
     * 打开相机并开始出帧。与 [stop] 共用相机线程队列，保证执行顺序。
     */
    @SuppressLint("MissingPermission")
    fun start() = handler.post {
        if (epoch % 2L != 0L) return@post      // 已经在跑
        epoch++
        val myEpoch = epoch

        lastError = null
        val cameraId = pickCameraId()
        if (cameraId == null) {
            Log.e(TAG, "找不到可用的后置相机")
            lastError = "找不到可用的后置相机"
            epoch++                            // 回到偶数 = 已停
            return@post
        }

        Log.i(TAG, "使用相机 id=$cameraId")
        fpsRange = pickFpsRange(cameraId)
        ready = false
        focusLockArmed = false

        supportsAf = manager.getCameraCharacteristics(cameraId)
            .get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES)
            ?.any { it != CaptureRequest.CONTROL_AF_MODE_OFF } == true
        Log.i(TAG, "支持自动对焦 = $supportsAf")

        val r = ImageReader.newInstance(W, H, ImageFormat.YUV_420_888, 4)
        r.setOnImageAvailableListener({ rd -> onImageReady(rd, myEpoch) }, handler)
        reader = r
        surface = r.surface

        try {
            manager.openCamera(cameraId, deviceCallback(myEpoch), handler)
        } catch (e: Exception) {
            // 无权限时 openCamera 会抛运行时 SecurityException，需要在这里捕获。
            Log.e(TAG, "openCamera 失败", e)
            lastError = "打不开相机：${e.javaClass.simpleName}"
            releaseResources()
            epoch++                            // 回到偶数 = 已停，下次 start() 还能重来
        }
    }

    /**
     * 关闭相机。排在相机线程上，保证在 [start] 之后执行。
     * 所有引用清空，避免再次打开时抛 IllegalStateException。
     */
    fun stop() = handler.post {
        if (epoch % 2L == 0L) return@post       // 已经停了
        releaseResources()
        epoch++                                 // 偶数 = 已停
    }

    /** 关闭并清空所有资源，只在相机线程调用。 */
    private fun releaseResources() {
        try { session?.stopRepeating() } catch (_: Exception) {}
        try { session?.close() } catch (_: Exception) {}
        try { device?.close() } catch (_: Exception) {}
        // 先移除监听再关闭 reader，避免在途回调调用 acquireLatestImage 时抛出异常。
        try {
            reader?.setOnImageAvailableListener(null as ImageReader.OnImageAvailableListener?, null)
        } catch (_: Exception) {}
        try { reader?.close() } catch (_: Exception) {}
        session = null; device = null; reader = null; surface = null
        ready = false
        focusLockArmed = false
    }

    /**
     * 取一帧交给 [onFrame]，运行在相机线程上。
     * [myEpoch] 用于识别属于上一代的迟到回调。
     */
    private fun onImageReady(rd: ImageReader, myEpoch: Long) {
        if (myEpoch != epoch) return

        // acquireLatestImage 会丢弃落后的帧。
        val image = try {
            rd.acquireLatestImage()
        } catch (e: Exception) {
            // reader 已关闭但回调仍在路上，属于正常竞态。
            return
        } ?: return

        try {
            // 没有接收者时也要关闭图像。
            val cb = onFrame ?: return
            cb(extractGray(image), image.timestamp)
        } catch (e: Exception) {
            Log.e(TAG, "处理帧失败", e)
        } finally {
            image.close()   // 不关闭会耗尽 ImageReader 缓冲
        }
    }

    private fun deviceCallback(myEpoch: Long) = object : CameraDevice.StateCallback() {
        override fun onOpened(d: CameraDevice) {
            if (myEpoch != epoch) {
                // 这一代已结束，必须关闭这个设备。
                Log.w(TAG, "onOpened 迟到（这一代相机已结束），直接关掉")
                try { d.close() } catch (_: Exception) {}
                return
            }
            val surf = reader?.surface
            if (surf == null) {
                try { d.close() } catch (_: Exception) {}
                return
            }
            device = d
            createSession(d, surf, myEpoch)
        }

        override fun onDisconnected(d: CameraDevice) {
            if (myEpoch != epoch) {
                try { d.close() } catch (_: Exception) {}
                return
            }
            Log.w(TAG, "相机断开")
            lastError = "相机被断开（可能被别的 App 抢走了）"
            try { d.close() } catch (_: Exception) {}
            // 设备失效，清理资源并标记为已停止。
            releaseResources()
            epoch++
        }

        override fun onError(d: CameraDevice, error: Int) {
            if (myEpoch != epoch) {
                try { d.close() } catch (_: Exception) {}
                return
            }
            Log.e(TAG, "相机打开失败 error=$error")
            lastError = cameraErrorText(error)
            try { d.close() } catch (_: Exception) {}
            releaseResources()
            epoch++
        }
    }

    private fun createSession(d: CameraDevice, target: Surface, myEpoch: Long) {
        d.createCaptureSession(listOf(target), object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(s: CameraCaptureSession) {
                if (myEpoch != epoch) {
                    try { s.close() } catch (_: Exception) {}
                    return
                }
                session = s
                // 先使用连续对焦，收敛后由 [focusCallback] 锁定。
                requestSettle(myEpoch)
                Log.i(TAG, "相机已启动 ${W}x${H} fps范围=$fpsRange 支持AF=$supportsAf")
            }

            override fun onConfigureFailed(s: CameraCaptureSession) {
                Log.e(TAG, "会话配置失败——可能这台设备不支持 ${W}x${H} 的 YUV 输出")
                lastError = "会话配置失败（${W}x${H} YUV 不支持？）"
            }
        }, handler)
    }

    // ---------------- 收敛（对焦） ----------------

    /**
     * 进入已准备状态时调用：开始连续对焦，收敛后由 [focusCallback] 锁定镜组。
     * 排在相机线程上，与回调串行执行。
     */
    fun beginSettle() = handler.post {
        // 相机未开启时不处理。
        if (epoch % 2L == 0L) return@post

        // 必须在 return 前设置：会话尚未建好时，请求由 onConfigured 发出。
        // 定焦镜头直接视为就绪，否则「开始录制」永远不会变绿。
        ready = !supportsAf
        focusLockArmed = true
        requestSettle(epoch)
    }

    /** 发送连续对焦请求并监听 AF_STATE；会话未建好时不处理。 */
    private fun requestSettle(myEpoch: Long) {
        val d = device ?: return
        val s = session ?: return
        val surf = surface ?: return
        try {
            s.setRepeatingRequest(
                buildRequest(d, surf, focusDistance = null), focusCallback(myEpoch), handler
            )
        } catch (e: Exception) {
            Log.e(TAG, "发连续对焦请求失败", e)
        }
    }

    /**
     * 监听 CONTROL_AF_STATE，对焦完成后切换为锁定请求。
     * 一直无法收敛时保持连续对焦，避免锁在错误位置。
     * 每次创建新回调并捕获 [epoch]，防止陈旧回调修改 [ready]。
     */
    private fun focusCallback(myEpoch: Long) = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            s: CameraCaptureSession,
            request: CaptureRequest,
            result: TotalCaptureResult
        ) {
            if (myEpoch != epoch) return
            // 未要求收敛或已经收敛时直接返回；ready 同时用作幂等标记。
            if (!focusLockArmed || ready) return
            val st = result.get(CaptureResult.CONTROL_AF_STATE) ?: return
            val focused = st == CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED ||
                          st == CaptureResult.CONTROL_AF_STATE_PASSIVE_FOCUSED
            if (!focused) return

            val d = device ?: return
            val surf = surface ?: return

            // 读取对焦完成时的镜组位置。
            val dist = result.get(CaptureResult.LENS_FOCUS_DISTANCE)

            if (dist == null || dist <= 0f) {
                // 读不到有效距离（0 表示未知或无穷远）时不能切到 AF_MODE_OFF，
                // 否则镜组会回到停泊位并导致画面模糊。
                ready = true
                Log.w(TAG, "AF_STATE=$st，但读不到有效对焦距离($dist)，保持连续对焦")
                return
            }

            try {
                s.setRepeatingRequest(buildRequest(d, surf, dist), null, handler)
                ready = true
                Log.i(TAG, "对焦完成 AF_STATE=$st，镜组锁定于 %.3f 屈光度 (= %.2f m)"
                    .format(dist, 1f / dist))
            } catch (e: Exception) {
                // 锁定失败时继续使用连续对焦。
                ready = true
                Log.e(TAG, "锁定对焦失败，继续用连续对焦（画面仍清晰，但焦距不固定）", e)
            }
        }
    }

    /**
     * 从 YUV_420_888 的 Y 平面提取灰度。
     * Y 平面本身即灰度，不需要色彩转换；rowStride 可能大于 W，需要逐行拷贝。
     */
    private fun extractGray(image: android.media.Image): ByteArray {
        val plane = image.planes[0]
        val buf = plane.buffer
        val rowStride = plane.rowStride
        val pixelStride = plane.pixelStride
        buf.rewind()

        val gray = ByteArray(W * H)

        if (rowStride == W && pixelStride == 1) {
            // 无 padding，一次拷贝完成
            buf.get(gray)
        } else {
            // 逐行跳过 padding
            val row = ByteArray(rowStride)
            for (y in 0 until H) {
                val offset = y * rowStride
                if (offset >= buf.limit()) break
                buf.position(offset)
                val n = minOf(rowStride, buf.remaining())
                buf.get(row, 0, n)
                System.arraycopy(row, 0, gray, y * W, minOf(W, n))
            }
        }
        return gray
    }

    /**
     * 构造采集请求。
     *
     * focusDistance 为 null 时使用连续对焦；否则使用 AF_MODE_OFF 并显式设置
     * LENS_FOCUS_DISTANCE（单位屈光度），将镜组固定到指定位置。
     * 只设置 AF_MODE_OFF 会让部分设备把镜组复位到停泊位。
     */
    private fun buildRequest(d: CameraDevice, target: Surface, focusDistance: Float?): CaptureRequest {
        return d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
            addTarget(target)

            // EIS 会裁切并改变画面
            set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
            // OIS 会改变有效内参
            set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
                CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF)

            if (!supportsAf) {
                // 定焦镜头
                set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
            } else if (focusDistance == null) {
                set(CaptureRequest.CONTROL_AF_MODE,
                    CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
            } else {
                set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
                set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDistance)
            }

            // 暂不锁定 AE/AWB，避免室内欠曝。
            set(CaptureRequest.CONTROL_AE_LOCK, false)
            set(CaptureRequest.CONTROL_AWB_LOCK, false)

            // 固定帧率
            fpsRange?.let { set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
        }.build()
    }

    // ---------------- 设备探测 ----------------

    /** 输出所有相机的能力信息。 */
    fun logCapabilities() {
        val chars = manager.cameraIdList.map { it to manager.getCameraCharacteristics(it) }
        var backCount = 0
        for ((id, c) in chars) {
            val facing = c.get(CameraCharacteristics.LENS_FACING)
            val size = c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)
            val map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            val yuvSizes = map?.getOutputSizes(ImageFormat.YUV_420_888)?.joinToString(",")
            val fps = c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)?.joinToString(",")
            val has480p = map?.getOutputSizes(ImageFormat.YUV_420_888)
                ?.any { it.width == W && it.height == H } ?: false

            // 由焦距和传感器宽度计算水平视场角，用于区分主摄和超广角。
            val focals = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
            val phys = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val minFocus = c.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE)
            val fov = if (focals != null && focals.isNotEmpty() && phys != null && phys.width > 0f) {
                "%.1f°".format(Math.toDegrees(2.0 * Math.atan(phys.width / (2.0 * focals[0]))))
            } else "?"

            Log.i(TAG, "── 相机 id=$id facing=$facing sensor=$size")
            Log.i(TAG, "   YUV尺寸: $yuvSizes")
            Log.i(TAG, "   支持${W}x${H}: $has480p")
            Log.i(TAG, "   FPS范围: $fps")
            Log.i(TAG, "   焦距: ${focals?.joinToString(",")} mm  传感器宽: ${phys?.width} mm")
            Log.i(TAG, "   水平视场角: $fov   最近对焦距离: ${minFocus} (屈光度, 越大越近)")
            if (facing == CameraCharacteristics.LENS_FACING_BACK) backCount++
        }
        Log.i(TAG, "后置相机数量 = $backCount")
    }

    private fun pickCameraId(): String? {
        CAMERA_ID_OVERRIDE?.let { return it }
        // 取第一个后置相机；多摄设备不一定对应主摄。
        return manager.cameraIdList.firstOrNull { id ->
            manager.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING) ==
                CameraCharacteristics.LENS_FACING_BACK
        }
    }

    private fun pickFpsRange(cameraId: String): Range<Int>? {
        val c = manager.getCameraCharacteristics(cameraId)
        val ranges = c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            ?: return null
        // 优先固定 30fps
        ranges.firstOrNull { it.lower == 30 && it.upper == 30 }?.let { return it }
        // 其次选覆盖 30fps 且下界最大的范围
        ranges.filter { it.upper >= 30 && it.lower <= 30 }.maxByOrNull { it.lower }
            ?.let { return it }
        // 兜底
        return ranges.maxByOrNull { it.upper }
    }
}
