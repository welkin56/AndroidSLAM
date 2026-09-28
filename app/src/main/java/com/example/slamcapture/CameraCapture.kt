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
 * 相机。**只负责「开相机」和「把帧交出去」两件事。**
 *
 * 它不知道 writer、不知道 IMU、不判断"什么时候该开始落盘"——那些是编排层
 * （[MainActivity]）的事。2026-09-18 那次只把 `imuReady` 的写入从 `start()`/`stop()`
 * 里拿掉，字段和判断还留在这儿，结果「什么时候开始落盘」要读四个地方才能拼出来。
 * 这次连根搬走。
 *
 * 每个帧的出口只有 [onFrame] 一个。预览挂在它上面；将来 SLAM 那一版也挂这儿，
 * 这个文件不用再动。
 *
 * ------------------------------------------------------------------
 * **一条必须守住的不变式：本类没有"自由预览"态。**
 *
 * 相机只有「开」和「关」两种状态，[ready] 和 [focusLockArmed] 只在 [start] / [stop]
 * （以及出错时那两处 [releaseResources]）里复位，别处一律不碰。以前有个 `endSettle()`
 * 想在相机开着的情况下把锁定解开、回到连续对焦，那会造出第三种状态——每个回调都得
 * 重新回答"我现在属于哪一代"。现在没有它了：**要重新对焦就重新 [beginSettle]，
 * 要重新开始就 [stop] 再 [start]**。将来谁想加"不关相机就回 IDLE"的快捷路径，
 * 先看见这条。
 * ------------------------------------------------------------------
 */
class CameraCapture(
    private val context: Context,
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

    /**
     * 每帧的出口。跑在**相机线程**上，实现里不能有阻塞操作。
     *
     * 注意它**不受"能不能落盘"约束**——对焦还没收敛、IMU 还没出数时也照给。
     * 预览要能看见"现在还是糊的"，那正是采集时该看的东西；能不能落盘是编排层
     * 的判断，跟这个出口无关。
     */
    @Volatile
    var onFrame: ((gray: ByteArray, tsNs: Long) -> Unit)? = null

    /**
     * **相机输出是否已经稳定到可以信任。**
     *
     * 今天这个判据等价于「对焦已收敛」：要么镜组真的锁上了，要么读不到有效对焦
     * 距离、决定保持连续对焦（连续对焦的图是清晰的，只是焦距会随拍摄距离呼吸）。
     *
     * 名字刻意**不叫** `focusLocked`——这是对外的判据，将来若锁白平衡/曝光，它们
     * 同样属于"相机输出稳没稳"，并进这个属性就行，不用再改一次名。内部真正只管
     * 对焦的东西（[requestSettle]、[focusLockArmed]）保持 `focus` 前缀。
     *
     * 只在相机线程上写；外面读它是为了决定能不能开始落盘。
     */
    @Volatile
    var ready = false
        private set

    /**
     * 出错的说明，给状态栏用。正常时为 null。
     *
     * **没有它，出错的表现就是一块黑屏加一个永远红的按钮，没有任何解释**——
     * 那正是最该避免的「查不出来的状态」。
     */
    @Volatile
    var lastError: String? = null
        private set

    /**
     * 把 Camera2 的错误码翻成人话 —— 光一句 `error=1` 谁也看不懂。
     *
     * `ERROR_CAMERA_IN_USE` 那条是最常见的：**上一个页面刚放开相机、系统还没真正释放**。
     * 从 SLAM 页按 Back 再立刻进录制页就会撞上它（两个页面各持一份 [CameraCapture]，
     * 关是异步的）。**症状是黑屏，出路是再按一次「开始准备」。**
     */
    // ⚠️ 这几个常量挂在 `CameraDevice.StateCallback` 上，**不是** `CameraDevice` ——
    // Kotlin 不许通过外部类访问嵌套类的静态成员，写错了编不过（已用 javap 核过）。
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
     * 相机这一代的编号。**奇数 = 正在跑，偶数 = 已停。**
     *
     * 以前这里是个 `started` 布尔量，它挡不住这个序列：
     * `开始准备 →(5ms) 取消准备 →(3ms) 开始准备`。第一次 `openCamera` 的 `onOpened`
     * 迟到，读到的 `started` 是**第二次刚置的 true**，于是把过期设备写进 [device]；
     * 紧接着第二次 open 拿到 `ERROR_CAMERA_IN_USE`，`onError` 里那行无条件的
     * `device = null` 把唯一活着的设备抹掉。结果：预览照常出图，但对焦永远收敛不了，
     * [device] 恒为 null，相机被永久占死，只能重启进程。
     *
     * 所以每个回调都捕获自己出生时的那一代 [epoch]，跑起来先比一次，**不相等就只
     * `close()` 自己那份资源然后退出，绝不写字段**。用奇偶编码运行状态（而不是
     * 另开一个布尔）是有意的：「这一代还算不算数」和「相机在不在跑」从根上就是同一个
     * 问题，分成两个字段迟早会互相矛盾。
     *
     * 只在相机线程上读写——所有回调都排在同一个 handler 上。
     */
    private var epoch = 0L

    /**
     * 这一轮「收敛」有没有被要求锁定对焦。
     *
     * [beginSettle] 置 true，[releaseResources] 置 false。**没有"自由预览"态**，
     * 所以它在相机开着的整段时间里一直是 true。
     */
    private var focusLockArmed = false

    /** 这颗镜头支不支持自动对焦（定焦摄像头为 false）。 */
    private var supportsAf = false

    // ---------------- 开合 ----------------

    /**
     * 打开相机并开始出帧。
     *
     * **整段排在相机线程上**，和 [stop] 共用一条队列，所以快速的前后台切换
     * （onStop → onStart）不会出现"刚开的相机被上一轮的拆除动作关掉"。
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
            // 没权限时 openCamera 抛的是**运行期** SecurityException，而且是在
            // 相机线程的 runnable 里抛——不接就是进程直接死。
            // （方法上的 @SuppressLint 只是关掉静态检查，拦不住它。）
            Log.e(TAG, "openCamera 失败", e)
            lastError = "打不开相机：${e.javaClass.simpleName}"
            releaseResources()
            epoch++                            // 回到偶数 = 已停，下次 start() 还能重来
        }
    }

    /**
     * 关掉相机。同样排在相机线程上，所以一定跑在 [start] 之后。
     *
     * 引用一律清成 null：留着已关闭的 CameraDevice 的话，紧接着开相机就是
     * `IllegalStateException`。
     */
    fun stop() = handler.post {
        if (epoch % 2L == 0L) return@post       // 已经停了
        releaseResources()
        epoch++                                 // 偶数 = 已停
    }

    /** 关掉并清空所有资源。**只在相机线程上调用。** */
    private fun releaseResources() {
        try { session?.stopRepeating() } catch (_: Exception) {}
        try { session?.close() } catch (_: Exception) {}
        try { device?.close() } catch (_: Exception) {}
        // 先摘掉监听再关 reader：关掉之后任何一条在途回调都会让 acquireLatestImage
        // 抛 IllegalStateException，逃出 Handler 就是进程直接死。
        try {
            reader?.setOnImageAvailableListener(null as ImageReader.OnImageAvailableListener?, null)
        } catch (_: Exception) {}
        try { reader?.close() } catch (_: Exception) {}
        session = null; device = null; reader = null; surface = null
        ready = false
        focusLockArmed = false
    }

    /**
     * 取一帧交给 [onFrame]。**跑在相机线程上。**
     *
     * [myEpoch] 是这个监听器出生时的代号：reader 关掉之后还会飞来一些回调，
     * 用代号一眼就能认出它们属于上一代。
     */
    private fun onImageReady(rd: ImageReader, myEpoch: Long) {
        if (myEpoch != epoch) return

        // acquireLatestImage 会自动丢掉落后的帧——这正是我们要的：
        // 宁可丢帧，也不要处理过期数据。
        val image = try {
            rd.acquireLatestImage()
        } catch (e: Exception) {
            // reader 已经关了，但回调还在路上。这是正常竞态，不是错误。
            return
        } ?: return

        try {
            // 没人要这一帧就白白取走 close 掉。
            val cb = onFrame ?: return
            cb(extractGray(image), image.timestamp)
        } catch (e: Exception) {
            Log.e(TAG, "处理帧失败", e)
        } finally {
            image.close()   // 必须 close，否则 ImageReader 很快就耗尽缓冲
        }
    }

    private fun deviceCallback(myEpoch: Long) = object : CameraDevice.StateCallback() {
        override fun onOpened(d: CameraDevice) {
            if (myEpoch != epoch) {
                // 这一代已经结束了。这个 device 没人要了，**必须在这里关掉**：
                // 关了 reader 之后配置会话会失败，但"配置失败"不等于"设备被释放"。
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
            // 设备没了，这一代作废：清干净并回到"已停"，否则之后每次 start()
            // 都会以为相机还开着，静默什么都不做。
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
                // 先跑连续对焦版，把焦点找准。锁定由 [focusCallback] 在
                // AF_STATE 收敛之后完成。
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
     * 进「已准备」时调用：**开始收敛**——重新跑连续对焦，对上之后由 [focusCallback]
     * 把镜组钉死。
     *
     * 排在相机线程上，和 [focusCallback] 天然串行。这也顺带解决了"读到已关闭的
     * CameraDevice"：`device` 只在相机线程上被清。
     */
    fun beginSettle() = handler.post {
        // 相机没开着（start 失败、或还没跑完就出了错）就没什么可收敛的。
        // 这一行挡掉的是"给一个已死的相机把 ready 置成 true"——那会让绿色按钮说谎。
        if (epoch % 2L == 0L) return@post

        // 这两行必须在 return 之前：会话可能还没建好（onConfigured 尚未回调），
        // 那种情况下请求由 onConfigured 发出去，它注册的 [focusCallback] 会读到
        // 这里刚设好的 focusLockArmed。
        //
        // 定焦镜头没什么可收敛的（AF_STATE 恒为 INACTIVE，focusCallback 永远不置位），
        // 直接算就绪——否则「开始录制」永远不会变绿，用户只能一直强制开始。
        ready = !supportsAf
        focusLockArmed = true
        requestSettle(epoch)
    }

    /** 发一次「连续对焦 + 盯着 AF_STATE」的请求。会话没建好就什么都不做。 */
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
     * 盯着 `CONTROL_AF_STATE`，一旦对焦完成就换成锁定版请求。
     *
     * 如果一直等不到（光线太暗、画面无纹理），就保持连续对焦继续跑——
     * 连续对焦的图是清晰的，只是焦距会随距离微调，比"锁在一个错误的
     * 位置、整幅图都是糊的"要好得多。
     *
     * 每次请求新造一个，好把出生时的 [epoch] 一起捕获进来：上一代的会话关掉之后
     * 仍可能有回调在路上，由一个陈旧的回调把 [ready] 置 true 就是绿色按钮说谎。
     */
    private fun focusCallback(myEpoch: Long) = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            s: CameraCaptureSession,
            request: CaptureRequest,
            result: TotalCaptureResult
        ) {
            if (myEpoch != epoch) return
            // 还没被要求收敛，或者已经收敛完了。两个条件都写在前面：
            // ready 兼任幂等闩锁，避免反复切请求。
            if (!focusLockArmed || ready) return
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
                // 锁不上也别卡住：连续对焦的画面是清晰的，只是焦距不固定。
                ready = true
                Log.e(TAG, "锁定对焦失败，继续用连续对焦（画面仍清晰，但焦距不固定）", e)
            }
        }
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
