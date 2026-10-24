package com.example.slamcapture

import android.Manifest
import android.app.AlertDialog
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.io.File
import java.util.concurrent.Executors

/**
 * 编排层：判断何时开始落盘。相机是否稳定、IMU 是否出数由各模块报告，在这里汇总。
 *
 * 状态机：
 *
 *     IDLE --[开始准备]--> ARMED <-> RECORDING
 *       ^                   |
 *       +---[取消准备]-------+
 *
 * beginPrepare 注册 IMU 并打开相机，teardown 关闭两者。停止录制后回到 ARMED，
 * 相机、对焦和 IMU 保持就绪，下一段可以直接开始。
 */
class CaptureActivity : AppCompatActivity() {

    companion object {
        const val TAG = "CaptureActivity"

        /**
         * 开始录制后延迟这么久再落相机帧，让 IMU 先写入一段。
         * 时间戳与 SensorEvent.timestamp 使用同一个 CLOCK_BOOTTIME 基准。
         */
        const val PREROLL_NS = 300_000_000L

        /** 状态栏刷新间隔。 */
        const val TICK_MS = 250L

        private const val REQ_CAMERA = 1
        private val RED = Color.parseColor("#C62828")
        private val GREEN = Color.parseColor("#2E7D32")

        /** 中性态（白底）下的按钮文字颜色。 */
        private val DARK_TEXT = Color.parseColor("#DE000000")
    }

    private enum class Phase { IDLE, ARMED, RECORDING }

    private lateinit var btnA: Button
    private lateinit var btnB: Button
    private lateinit var tvStatus: TextView
    private lateinit var preview: PreviewView

    private val uiHandler = Handler(Looper.getMainLooper())

    /** 后台删除目录，避免主线程 ANR。 */
    private val ioExecutor = Executors.newSingleThreadExecutor()

    private lateinit var camThread: HandlerThread

    /**
     * 相机对象在 onCreate 创建，onDestroy 时随线程销毁。
     * Handler 必须与其同生命周期，否则 postDelayed 会静默失效。
     */
    private lateinit var cam: CameraCapture

    // ---------------- 编排层状态。相机模块只报告 cam.ready，不读取这些字段 ----------------

    @Volatile
    private var phase = Phase.IDLE

    /** 落盘目标，只在 RECORDING 时非 null；相机线程读取，需要 volatile。 */
    @Volatile
    private var writer: SessionWriter? = null

    /** 早于此时刻的相机帧不落盘。 */
    @Volatile
    private var framesFromTsNs = 0L

    /** IMU 收集器，每次准备新建一个，teardown 时释放。 */
    private var imu: ImuCollector? = null
    private var imuThread: HandlerThread? = null

    /**
     * 就绪判据未成立但用户仍按下开始时的标记。
     * 只在 [beginRecording] 中设置一次，用于识别强制开始。
     */
    private var camGaveUp = false
    private var imuGaveUp = false

    /** 上一次保留下来的 session 目录，给状态栏显示。 */
    private var lastSessionDir: File? = null

    private var dialog: AlertDialog? = null

    // ---------------- 帧出口 ----------------

    /**
     * 每帧出口，运行在相机线程上，不能阻塞。
     * 落盘和预览各有独立的显式检查。
     */
    private fun onCameraFrame(gray: ByteArray, tsNs: Long) {
        val w = writer
        // 先读 writer 再读 phase，因此 SessionWriter.offerFrame 中的 running 检查是必需的。
        if (phase == Phase.RECORDING && w != null && tsNs >= framesFromTsNs) {
            w.offerFrame(SessionWriter.FrameItem(tsNs, gray, CameraCapture.W, CameraCapture.H))
        }

        // 预览始终接收帧；单独 try，避免预览异常影响采集。
        try {
            preview.offer(gray, tsNs)
        } catch (e: Throwable) {
            Log.e(TAG, "预览丢帧", e)
        }
    }

    // ---------------- 生命周期 ----------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_capture)

        btnA = findViewById(R.id.btnA)
        btnB = findViewById(R.id.btnB)
        tvStatus = findViewById(R.id.tvStatus)
        preview = findViewById(R.id.preview)

        // 相机线程与 Activity 同生命周期，start/stop 按队列顺序执行。
        val ct = HandlerThread("camera").also { it.start() }
        camThread = ct
        cam = CameraCapture(this, Handler(ct.looper))
        cam.onFrame = ::onCameraFrame
        cam.logCapabilities()   // 打印设备能力，第一次跑一定要看 logcat

        btnA.setOnClickListener { onButtonA() }
        btnB.setOnClickListener { onButtonB() }
        render()
    }

    /**
     * 相机和传感器由「开始准备」打开，不在前台状态切换时自动启动。
     */
    override fun onStart() {
        super.onStart()
    }

    override fun onStop() {
        super.onStop()
        // 切后台时拆除硬件；正在录制的文件保留。
        teardown("切后台")
    }

    override fun onDestroy() {
        super.onDestroy()
        teardown("退出")
        cam.onFrame = null
        uiHandler.removeCallbacksAndMessages(null)
        camThread.quitSafely()   // 执行完已排队的 stop()
        ioExecutor.shutdown()
    }

    // ---------------- 两个按钮 ----------------

    /** A 按钮：退一层。 */
    private fun onButtonA() {
        when (phase) {
            Phase.IDLE -> beginPrepare()
            Phase.ARMED -> teardown("取消准备")
            Phase.RECORDING -> confirmDiscard()
        }
    }

    private fun onButtonB() {
        when (phase) {
            Phase.IDLE -> Unit              // 按钮不可用
            Phase.ARMED -> beginRecording()
            Phase.RECORDING -> stopRecording(discard = false)
        }
    }

    // ---------------- 准备 ----------------

    /**
     * IDLE 到 ARMED：注册 IMU 并打开相机（含对焦初始化）。
     * 多个入口共用，先检查 [phase]。
     */
    private fun beginPrepare() {
        if (phase != Phase.IDLE) return

        // 无权限时不做任何修改；授权回调会再次调用本函数。
        if (!hasCameraPermission()) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), REQ_CAMERA)
            return
        }

        // 先启动 IMU；传感器预热期间可同时调整机位和等待对焦。
        val t = HandlerThread("imu").also { it.start() }
        val c = ImuCollector(this)
        imuThread = t
        imu = c
        c.start(Handler(t.looper))

        camGaveUp = false
        imuGaveUp = false
        preview.setActive(true)
        phase = Phase.ARMED

        cam.start()
        cam.beginSettle()

        render()
        uiHandler.removeCallbacks(ticker)
        uiHandler.post(ticker)
    }

    // ---------------- 录制 ----------------

    /**
     * ARMED 到 RECORDING。
     * 两个就绪判据只用于识别强制开始；IMU 领先由 [framesFromTsNs] 保证。
     */
    private fun beginRecording() {
        if (phase != Phase.ARMED) return

        camGaveUp = !cam.ready
        imuGaveUp = imu?.ready != true
        if (camGaveUp) Log.w(TAG, "强制开始：对焦未收敛，将带着连续对焦落盘")
        if (imuGaveUp) {
            // IMU 未出数的唯一提示，不要删除。
            Log.e(TAG, "强制开始：IMU 未出数——这一段的前若干秒相机帧将没有 IMU 覆盖")
        }

        val dir = newSessionDir()
        val w = SessionWriter(dir)
        w.forced = camGaveUp || imuGaveUp
        w.start()
        if (w.ioError != null) {
            Log.e(TAG, "建 session 文件失败：${w.ioError}")
            tvStatus.text = "无法创建输出文件：${w.ioError}\n$dir"
            return
        }

        writer = w
        imu?.attach(w)   // 从这一刻起 IMU 行开始落盘

        // IMU 尚未出数时不需要预热，直接落相机帧。
        framesFromTsNs = SystemClock.elapsedRealtimeNanos() + if (imuGaveUp) 0L else PREROLL_NS

        phase = Phase.RECORDING
        Log.i(TAG, "开始录制 $dir  预热 ${(framesFromTsNs - SystemClock.elapsedRealtimeNanos())
            .coerceAtLeast(0) / 1_000_000} ms")
        render()
    }

    /**
     * RECORDING 到 ARMED。不关闭相机、传感器注册或对焦，下一段可以立即开始。
     */
    private fun stopRecording(discard: Boolean) {
        if (phase != Phase.RECORDING) return

        phase = Phase.ARMED          // 先切相位：相机线程从下一帧起不再落盘
        imu?.detach()                // 传感器继续跑，只是不记录了

        val w = writer
        writer = null
        w?.stop()

        if (w == null) {
            render()
            return
        }

        if (w.framesWritten.get() == 0L) {
            // 保留目录便于排查一帧都没写入的情况。
            Log.e(TAG, "这一段一帧都没落盘（时间戳对不上或写盘失败），目录保留：" + w.sessionDir)
        }
        Log.i(TAG, "结束录制  相机 ${w.framesWritten.get()} 帧 / IMU ${w.imuWritten.get()} 条"
            + "  首帧=${w.firstFrameTsNs} 首IMU=${w.firstImuTsNs}")

        if (!discard) {
            lastSessionDir = w.sessionDir
        } else if (w.threadAlive) {
            // writer 线程仍在运行，删除会与写入并发，因此保留目录。
            Log.e(TAG, "writer 线程未退出，放弃删除：${w.sessionDir}")
            lastSessionDir = w.sessionDir
        } else {
            discardDir(w.sessionDir)
            lastSessionDir = null
        }
        render()
    }

    private fun confirmDiscard() {
        if (phase != Phase.RECORDING) return
        val frames = writer?.framesWritten?.get() ?: 0L
        dialog = AlertDialog.Builder(this)
            .setTitle("放弃这一段？")
            .setMessage("已经落盘的 $frames 帧会被删掉。\n相机和 IMU 继续保持热状态。")
            .setPositiveButton("删掉") { _, _ ->
                dialog = null
                stopRecording(discard = true)
            }
            .setNegativeButton("继续录", null)
            .setOnDismissListener { dialog = null }
            .show()
    }

    // ---------------- 拆除 ----------------

    /**
     * 回到 IDLE，幂等，由取消准备、onStop 和 onDestroy 共用。
     * cam.stop 是异步的，迟到的帧由 [phase] 和 SessionWriter 的 running 检查拦住。
     */
    private fun teardown(reason: String) {
        if (phase == Phase.IDLE) return
        Log.i(TAG, "拆掉准备状态（$reason）")

        if (phase == Phase.RECORDING) stopRecording(discard = false)

        preview.setActive(false)   // 预览闸先关，屏幕立刻黑
        phase = Phase.IDLE

        dialog?.dismiss()
        dialog = null
        uiHandler.removeCallbacks(ticker)

        imu?.stop()
        imuThread?.quitSafely()
        // 必须置 null，否则下次 ARMED 会读到已停止的 collector。
        imu = null
        imuThread = null

        cam.stop()
        camGaveUp = false
        imuGaveUp = false
        render()
    }

    // ---------------- 状态栏 ----------------

    private val ticker = object : Runnable {
        override fun run() {
            if (phase == Phase.IDLE) return
            render()
            uiHandler.postDelayed(this, TICK_MS)
        }
    }

    /** 刷新按钮和状态栏；显示内容由 [phase] 推导。 */
    private fun render() {
        val p = phase
        val w = writer
        val camOk = cam.ready
        val imuOk = imu?.ready == true

        // ---- 按钮 ----
        // 文字色和背景成对设置，中性态用深色字，着色态用白色字。
        when (p) {
            Phase.IDLE -> {
                btnA.text = "开始准备"
                btnA.backgroundTintList = null
                btnA.setTextColor(DARK_TEXT)
                btnB.text = "开始录制"
                btnB.isEnabled = false
                btnB.backgroundTintList = null
                btnB.setTextColor(DARK_TEXT)
            }
            Phase.ARMED -> {
                btnA.text = "取消准备"
                btnA.backgroundTintList = null
                btnA.setTextColor(DARK_TEXT)
                // 进入准备后按钮立即可用；就绪前显示为红色。
                btnB.isEnabled = true
                if (camOk && imuOk) {
                    btnB.text = "开始录制"
                    btnB.backgroundTintList = ColorStateList.valueOf(GREEN)
                } else {
                    btnB.text = "强制开始"
                    btnB.backgroundTintList = ColorStateList.valueOf(RED)
                }
                btnB.setTextColor(Color.WHITE)
            }
            Phase.RECORDING -> {
                btnA.text = "取消录制"
                btnA.backgroundTintList = null
                btnA.setTextColor(DARK_TEXT)
                btnB.text = "停止录制"
                btnB.isEnabled = true
                // 停止不是危险操作；取消录制另有确认弹窗。
                btnB.backgroundTintList = null
                btnB.setTextColor(DARK_TEXT)
            }
        }

        // ---- 状态栏 ----
        val head = when (p) {
            Phase.IDLE -> "点「开始准备」打开相机和 IMU。"
            Phase.ARMED -> when {
                camOk && imuOk -> "● 就绪，可以开始录制"
                !imuOk && !camOk -> "… 等 IMU 出数（本机约 3 秒），且对焦未收敛"
                !imuOk -> "… 等 IMU 出数（本机约 3 秒）"
                else -> "… 对焦未收敛（画面太暗或无纹理时它不会自己放行）"
            }
            Phase.RECORDING -> when {
                SystemClock.elapsedRealtimeNanos() < framesFromTsNs -> "… IMU 预热"
                // 以实际写入结果为准。
                w?.firstImuTsNs == 0L -> "… 这一段 IMU 还没落盘（相机帧暂时对不上 IMU）"
                w?.forced == true -> "● 录制中（强制开始）"
                else -> "● 录制中"
            }
        }

        val body = buildString {
            if (w != null) append(w.stats()) else append("已准备，未记录")
            lastSessionDir?.let { append("\n上一次：${it.absolutePath}") }
            cam.lastError?.let { append("\n相机：$it") }
        }
        tvStatus.text = "$head\n\n$body"
    }

    // ---------------- 权限 ----------------

    private fun hasCameraPermission() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != REQ_CAMERA) return
        if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            // 权限对话框不会触发 onStop，这里继续完成开始准备。
            beginPrepare()
        } else {
            tvStatus.text = "相机权限被拒绝。\n请到 设置 → 应用 → SlamCapture → 权限 里手动开启。"
        }
    }

    // ---------------- 文件 ----------------

    private fun newSessionDir(): File {
        // 外部存储不可用时退到内部存储；不能把 null 传给 File 构造器。
        val base = getExternalFilesDir(null) ?: filesDir
        val dir = File(File(base, "sessions"), "session_${System.currentTimeMillis()}")
        dir.mkdirs()
        return dir
    }

    /**
     * 删除一个 session 目录。
     * 先原子重命名标记为删除，再在后台删除。
     */
    private fun discardDir(dir: File) {
        val parent = dir.parentFile ?: return
        val tmp = File(parent, dir.name + ".deleting")
        if (!dir.renameTo(tmp)) {
            Log.e(TAG, "重命名失败，退而直接后台删：$dir")
            ioExecutor.execute { dir.deleteRecursively() }
            return
        }
        ioExecutor.execute {
            if (!tmp.deleteRecursively()) Log.e(TAG, "删除未完全成功：$tmp")
        }
    }
}
