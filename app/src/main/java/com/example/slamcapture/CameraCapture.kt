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
import android.util.Size
import android.view.Surface
import java.util.concurrent.atomic.AtomicBoolean

class CameraCapture(
    private val context: Context,
    private val writer: SessionWriter,
    private val handler: Handler
) {
    companion object {
        const val TAG = "CameraCapture"

        // ORB-SLAM3 的典型工作分辨率。不要改成 1280×720——
        // ORB 特征提取会成为瓶颈，手机上跑不到 30fps。
        const val W = 640
        const val H = 480

        // 如果你的手机有多个后摄（主摄/超广角/长焦），自动选择可能选错。
        // 跑一次看日志里打印的 id，然后在这里写死（例如 "0"）。null = 自动选。
        //
        // 注意不能写 const：Kotlin 的 const val 只允许基本类型和非空 String，
        // 可空类型（String?）用 const 会编译失败。
        val CAMERA_ID_OVERRIDE: String? = null
    }

    private val manager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var reader: ImageReader? = null
    private var fpsRange: Range<Int>? = null
    private var surface: Surface? = null

    // 两个超时兜底的句柄。stop() 里必须撤掉——否则会话关了之后它们还会响，
    // 把一个已经停掉的相机重新标成"就绪"。原来只有一个对焦超时时就漏了这茬。
    private var focusTimeout: Runnable? = null
    private var imuTimeout: Runnable? = null

    /** 这颗镜头支不支持自动对焦（定焦摄像头为 false）。 */
    private var supportsAf = false

    /** 对焦是否已经锁定完毕。锁定后焦距固定，可以开始信任标定了。 */
    @Volatile var focusLocked = false
        private set

    /**
     * 是否已经开始真正落盘。两个条件**同时**满足才开。
     *
     * 1. 对焦完成（[focusReady]）——不然开头几帧是糊的。糊帧不是"没用"，
     *    是**有害**：ORB-SLAM3 的初始地图就是拿最前面这几帧建的，喂糊帧
     *    进去要么初始化失败，要么建出一个烂地图，后面全歪。
     *
     * 2. IMU 已经出数（[imuReady]）——ORB-SLAM3 假定 IMU 比相机先开始。
     *    本机 IMU 挂在传感器协处理器后面，注册后要 3 秒多才出数；如果相机
     *    抢先开始录，两个流的时间关系就反了，见 [ImuCollector.onFirstSample]。
     *
     * 两个条件**谁后到谁触发**，都不阻塞。
     */
    @Volatile private var focusReady = false
    @Volatile private var imuReady = false
    private val capturing = AtomicBoolean(false)

    /** 等对焦的超时上限。超过它就不再等，带着当前画面开始采集。 */
    private val focusWaitMs = 2500L

    //等 IMU 首样本的超时上限。
    private val imuWaitMs = 8000L

    @SuppressLint("MissingPermission")
    fun start() {
        val cameraId = pickCameraId() ?: run {
            Log.e(TAG, "找不到可用的后置相机")
            return
        }
        Log.i(TAG, "使用相机 id=$cameraId")
        fpsRange = pickFpsRange(cameraId)
        focusLocked = false
        focusReady = false
        capturing.set(false)

        supportsAf = manager.getCameraCharacteristics(cameraId)
            .get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES)
            ?.any { it != CaptureRequest.CONTROL_AF_MODE_OFF } == true
        Log.i(TAG, "支持自动对焦 = $supportsAf")

        val r = ImageReader.newInstance(W, H, ImageFormat.YUV_420_888, 4)
        r.setOnImageAvailableListener({ rd ->
            // acquireLatestImage 会自动丢掉落后的帧——这正是我们要的：
            // 宁可丢帧，也不要处理过期数据
            val image = rd.acquireLatestImage() ?: return@setOnImageAvailableListener
            try {
                // 对焦完成、IMU 出数，两件事都齐了之前，照常取走并 close，
                // 但**不送编码**。
                //
                // 为什么不是干脆不 acquire：不消费的话 4 个 buffer 很快填满，
                // 整条相机管线会停住，AE/AWB 也跟着停止收敛——等你要开始
                // 采集时画面反而更糟。必须保持管线畅通，只是在门口过滤掉。
                //
                // 为什么要过滤：糊帧不是"没用"，是**有害**；IMU 没出数就开录，
                // 还会让两个流的时间关系反过来。两者都见 capturing 的注释。
                if (!capturing.get()) return@setOnImageAvailableListener

                val gray = extractGray(image)
                writer.offerFrame(SessionWriter.FrameItem(image.timestamp, gray, W, H))
            } catch (e: Exception) {
                Log.e(TAG, "处理帧失败", e)
            } finally {
                image.close()   // 必须 close，否则 ImageReader 很快就耗尽缓冲
            }
        }, handler)
        reader = r
        surface = r.surface

        manager.openCamera(cameraId, object : CameraDevice.StateCallback() {
            override fun onOpened(d: CameraDevice) {
                device = d
                createSession(d, r.surface)
            }
            override fun onDisconnected(d: CameraDevice) {
                Log.w(TAG, "相机断开")
                d.close(); device = null
            }
            override fun onError(d: CameraDevice, error: Int) {
                Log.e(TAG, "相机打开失败 error=$error")
                d.close(); device = null
            }
        }, handler)
    }

    /**
     * 从 YUV_420_888 的 Y 平面提取灰度。
     *
     * Y 平面**就是**灰度图，不需要任何色彩空间转换。
     * 唯一的坑是 rowStride：硬件为了对齐，每行末尾可能有 padding，
     * 所以 rowStride 常常大于 W，必须逐行拷。
     */
    private fun extractGray(image: android.media.Image): ByteArray {
        val plane = image.planes[0]
        val buf = plane.buffer
        val rowStride = plane.rowStride
        val pixelStride = plane.pixelStride
        buf.rewind()

        val gray = ByteArray(W * H)

        if (rowStride == W && pixelStride == 1) {
            // 快路径：没有 padding，一次拷完
            buf.get(gray)
        } else {
            // 慢路径：逐行跳过 padding
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
     * 构造一个采集请求。
     *
     * [focusDistance] = null → 连续对焦，先把焦点找准。
     * [focusDistance] = d    → `AF_MODE_OFF` **加上**显式的
     *                          `LENS_FOCUS_DISTANCE = d`，把镜组驱动到指定位
     *                          置并钉住。单位是**屈光度**（1/米）。
     *
     * ------------------------------------------------------------------
     * 这里踩了本工程最贵的一个坑，值得完整记下来。
     *
     * SLAM 要求焦距固定（对焦呼吸会改变内参），所以最终要关掉 AF。但
     * **只写 `AF_MODE_OFF` 是不够的**——在小米/红米这类机器上，HAL 收到
     * `AF_MODE_OFF` 会把镜组**驱动回默认停泊位**（通常是最近对焦或无穷
     * 远），而不是"停在当前位置"。
     *
     * 现象很有迷惑性：切过去之后前 7 帧（约 230ms）还是清晰的，第 8 帧
     * 起突然糊掉，然后整段一直糊——因为镜组一直在"滑向停车位"。只看
     * 头几帧会以为是好的。
     *
     * 正确做法：`AF_MODE_OFF` **必须和 `LENS_FOCUS_DISTANCE` 一起给**，
     * 把"停车"变成"开到指定位置"。前提是镜头支持手动对焦，即
     * `LENS_INFO_MINIMUM_FOCUS_DISTANCE > 0`。
     *
     * 另外**不要**再发 `AF_TRIGGER_CANCEL`，那同样会把镜组推回默认位置。
     * ------------------------------------------------------------------
     */
    private fun buildRequest(d: CameraDevice, target: Surface, focusDistance: Float?): CaptureRequest {
        return d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
            addTarget(target)

            // EIS 会裁切并变形画面，直接毁掉标定
            set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
            // OIS 同理，而且会改变有效内参
            set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
                CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF)

            if (!supportsAf) {
                // 定焦镜头，没什么可做的
                set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
            } else if (focusDistance == null) {
                set(CaptureRequest.CONTROL_AF_MODE,
                    CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
            } else {
                set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
                set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDistance)
            }

            // 锁死 AE 在室内会欠曝。先让 AE 平滑跟随，链路跑通后再考虑锁。
            set(CaptureRequest.CONTROL_AE_LOCK, false)
            set(CaptureRequest.CONTROL_AWB_LOCK, false)

            // 固定帧率
            fpsRange?.let { set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
        }.build()
    }

    /**
     * 盯着 `CONTROL_AF_STATE`，一旦对焦完成就换成锁定版请求。
     *
     * 如果一直等不到（光线太暗、画面无纹理），就保持连续对焦继续跑——
     * 连续对焦的图是清晰的，只是焦距会随距离微调，比"锁在一个错误的
     * 位置、整幅图都是糊的"要好得多。
     */
    private val focusCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            s: CameraCaptureSession,
            request: CaptureRequest,
            result: TotalCaptureResult
        ) {
            if (focusLocked) return
            val st = result.get(CaptureResult.CONTROL_AF_STATE) ?: return
            val focused = st == CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED ||
                          st == CaptureResult.CONTROL_AF_STATE_PASSIVE_FOCUSED
            if (!focused) return

            val d = device ?: return
            val surf = surface ?: return

            // 把对焦达成那一刻的镜组位置读出来，之后用它把镜组钉在原地。
            val dist = result.get(CaptureResult.LENS_FOCUS_DISTANCE)

            if (dist == null || dist <= 0f) {
                // 读不到有效距离（0 表示未知，或者镜组在无穷远）。
                //
                // 这时**千万不能**切 AF_MODE_OFF——那正是会让镜组停车、
                // 把画面搞糊的那一步。宁可保持连续对焦：图一直是清晰的，
                // 代价只是焦距会随拍摄距离轻微呼吸。
                focusLocked = true
                Log.w(TAG, "AF_STATE=$st，但读不到有效对焦距离($dist)，保持连续对焦")
                markFocusReady("连续对焦（没有可锁的对焦距离）")
                return
            }

            try {
                s.setRepeatingRequest(buildRequest(d, surf, dist), null, handler)
                focusLocked = true
                Log.i(TAG, "对焦完成 AF_STATE=$st，镜组锁定于 %.3f 屈光度 (= %.2f m)"
                    .format(dist, 1f / dist))
                markFocusReady("对焦已锁定")
            } catch (e: Exception) {
                Log.e(TAG, "锁定对焦失败，继续用连续对焦（画面仍清晰，但焦距不固定）", e)
                markFocusReady("锁定失败，带连续对焦开始")
            }
        }
    }

    /**
     * 两个条件都齐了就开闸。幂等——两条路（对焦 / IMU）谁后到谁触发，
     * 各自的超时兜底也会触发，一共四个调用点。
     */
    private fun maybeBegin(trigger: String) {
        if (!focusReady || !imuReady) return
        if (!capturing.compareAndSet(false, true)) return
        Log.i(TAG, "开始落盘（由「$trigger」触发；对焦与 IMU 均已就绪）")
    }

    /** 对焦就绪：要么真的锁定了，要么等超时了。只标记，不阻塞。 */
    private fun markFocusReady(reason: String) {
        if (focusReady) return
        focusReady = true
        Log.i(TAG, "对焦就绪（$reason）")
        maybeBegin(reason)
    }

    /**
     * IMU 就绪。由 [ImuCollector.onFirstSample] 在第一个样本落盘后回调，
     * 跑在 IMU 的 handler 线程上——这里只碰 volatile 标志和 AtomicBoolean，
     * 不能有阻塞操作。
     */
    fun markImuReady() = markImuReady("IMU 首样本")

    private fun markImuReady(reason: String) {
        if (imuReady) return
        imuReady = true
        Log.i(TAG, "IMU 就绪（$reason）")
        maybeBegin(reason)
    }

    private fun createSession(d: CameraDevice, target: Surface) {
        d.createCaptureSession(listOf(target), object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(s: CameraCaptureSession) {
                session = s
                try {
                    // 先跑连续对焦版。对焦完成后 focusCallback 会自己换成锁定版。
                    s.setRepeatingRequest(
                        buildRequest(d, target, focusDistance = null), focusCallback, handler
                    )
                    Log.i(TAG, "相机已启动 ${W}x${H} fps范围=$fpsRange 支持AF=$supportsAf，"
                        + "等对焦与 IMU 都就绪后开始落盘")

                    // 兜底 1：万一 AF 一直不报 FOCUSED（光线太暗、画面无纹理、
                    // 或者干脆是定焦镜头），不能无限等下去。超时就开始采集——
                    // 连续对焦的画面是清晰的，只是焦距不固定，比空等强。
                    focusTimeout = Runnable {
                        if (!focusLocked) markFocusReady("等对焦超时 ${focusWaitMs}ms")
                    }.also { handler.postDelayed(it, focusWaitMs) }

                    // 兜底 2：IMU 迟迟不出数。本机正常约 3.3 s（传感器挂在
                    // 协处理器后面），给足 8 s。超时也开——但录出来的数据
                    // ORB-SLAM3 会因为 IMU 晚于相机而报错退出，所以这条 ERROR
                    // 是排查时的关键线索。
                    imuTimeout = Runnable {
                        if (!imuReady) {
                            Log.e(TAG, "等 IMU 首样本超时 ${imuWaitMs}ms——"
                                + "本次录制两个流的时间关系可能不满足 ORB-SLAM3 的前提")
                            markImuReady("等 IMU 超时")
                        }
                    }.also { handler.postDelayed(it, imuWaitMs) }
                } catch (e: Exception) {
                    Log.e(TAG, "setRepeatingRequest 失败", e)
                }
            }
            override fun onConfigureFailed(s: CameraCaptureSession) {
                Log.e(TAG, "会话配置失败——可能这台设备不支持 ${W}x${H} 的 YUV 输出")
            }
        }, handler)
    }

    fun stop() {
        focusTimeout?.let { handler.removeCallbacks(it) }
        imuTimeout?.let { handler.removeCallbacks(it) }
        focusTimeout = null
        imuTimeout = null

        try { session?.stopRepeating() } catch (_: Exception) {}
        try { session?.close() } catch (_: Exception) {}
        try { device?.close() } catch (_: Exception) {}
        try { reader?.close() } catch (_: Exception) {}
        session = null; device = null; reader = null; surface = null
        focusLocked = false
        focusReady = false
        capturing.set(false)
    }

    // ---------------- 设备探测 ----------------

    /** 列出所有相机的信息。第一次跑一定要看这段日志。 */
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

            // 焦距 + 传感器物理尺寸 → 水平视场角。
            // 这是区分主摄和超广角最可靠的依据：主摄约 65-80°，超广角 100-120°。
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
        // 取第一个后置相机。多摄手机上这通常是主摄，但不保证——
        // 所以 logCapabilities 打印的信息很重要。
        return manager.cameraIdList.firstOrNull { id ->
            manager.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING) ==
                CameraCharacteristics.LENS_FACING_BACK
        }
    }

    private fun pickFpsRange(cameraId: String): Range<Int>? {
        val c = manager.getCameraCharacteristics(cameraId)
        val ranges = c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            ?: return null
        // 优先：固定 30fps
        ranges.firstOrNull { it.lower == 30 && it.upper == 30 }?.let { return it }
        // 其次：覆盖 30 的范围里下界最大的（下界越大越稳，越不容易在光照变化时掉帧率）
        ranges.filter { it.upper >= 30 && it.lower <= 30 }.maxByOrNull { it.lower }
            ?.let { return it }
        // 兜底
        return ranges.maxByOrNull { it.upper }
    }
}
