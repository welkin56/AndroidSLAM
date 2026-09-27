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
import java.io.IOException
import java.util.concurrent.Executors

/**
 * 编排层。它**拥有**「什么时候开始落盘」这个判断，因为那是跨模块的：
 * 相机稳不稳、IMU 出没出数，两个模块各自只报告自己的事实，答案只能在这里合。
 *
 * ------------------------------------------------------------------
 * 状态机：
 *
 *     IDLE ──[开始准备]──► ARMED ⇄ RECORDING
 *       ▲                   │
 *       └───[取消准备]───────┘
 *
 * **「准备」是两条线的开关**：[beginPrepare] 注册 IMU 并打开相机（含对焦初始化），
 * [teardown] 两样都关掉。IDLE 时两样硬件都不占。代价是「取消准备 → 开始准备」要
 * 重付约 3.3 s 的传感器冷启动——接受，因为那是偶发路径，而常见路径上的每一次
 * 「停止录制」都不额外付任何东西。
 *
 * **「停止录制」回 ARMED 而不是 IDLE**：标定是「同一套机位连着录好几段」。回到
 * ARMED 意味着下一段按下就是录（相机还开着、对焦还锁着、IMU 还热着）。这正是
 * 「此时仍在采集 imu 和相机，只是不进行记录」。
 * ------------------------------------------------------------------
 *
 * 门控的策略**不是定时器，是用户**。「等多久就不等了」从 `postDelayed` 变成了
 * 「按下那个红色按钮」，所以这里再没有超时任务，[camGaveUp]/[imuGaveUp] 只在主线程、
 * 按钮回调里置位一次。
 */
class MainActivity : AppCompatActivity() {

    companion object {
        const val TAG = "MainActivity"

        /**
         * 开始录制后先白等这么久再落帧：让 IMU 先写进去一小段。
         *
         * **这是"IMU 领先相机"的全部保证**——相机帧在这之前一律丢弃，所以不管 IMU
         * 什么时候开始写，它都领先。对比的是 `Image.getTimestamp()`（曝光起始时刻），
         * 和 `SensorEvent.timestamp` 是同一个 CLOCK_BOOTTIME 基准，不需要任何换算。
         */
        const val PREROLL_NS = 300_000_000L

        /** 状态栏刷新间隔。 */
        const val TICK_MS = 250L

        /**
         * 词袋文件名。**145 MB，不进 APK** —— 用 `tools/push-vocab.sh` 推到外部文件目录，
         * 和 session 同一个根（理由同采集端：`adb push/pull` 都不需要 root）。
         */
        const val VOCAB_NAME = "ORBvoc.txt"

        /** 相机配置。权威副本在 `assets/`（STAGE5 §2.2），启动时落到私有目录再给 C++。 */
        private const val SETTINGS_NAME = "phone.yaml"

        // SlamBridge.nativeState() 的取值，和 C++ 侧 slam::State 一一对应。
        private const val SLAM_IDLE = 0
        private const val SLAM_LOADING = 1
        private const val SLAM_READY = 2
        private const val SLAM_FAILED = 3

        private const val REQ_CAMERA = 1
        private val RED = Color.parseColor("#C62828")
        private val GREEN = Color.parseColor("#2E7D32")

        /**
         * 中性态（backgroundTintList = null → 白底）的按钮文字。
         * 主题默认给按钮的是白字，白底上直接看不见——文字色必须和背景一起推出来。
         */
        private val DARK_TEXT = Color.parseColor("#DE000000")
    }

    private enum class Phase { IDLE, ARMED, RECORDING }

    private lateinit var btnA: Button
    private lateinit var btnB: Button
    private lateinit var btnSlam: Button
    private lateinit var tvStatus: TextView
    private lateinit var preview: PreviewView

    private val uiHandler = Handler(Looper.getMainLooper())

    /** 删目录用。10 分钟 30 fps 是约 1.8 万个 PNG，主线程删必然 ANR。 */
    private val ioExecutor = Executors.newSingleThreadExecutor()

    private lateinit var camThread: HandlerThread

    /**
     * 相机对象**全程只有一个**，`onCreate` 建、`onDestroy` 才随线程一起走。
     *
     * 这决定了它持有的 Handler 也必须是长命的：如果 handler 线程跟着录制起落，
     * 相机会握着一个绑在已死 Looper 上的 Handler，`postDelayed` 会**静默**不执行。
     */
    private lateinit var cam: CameraCapture

    // ---------------- 编排层的状态。相机模块不碰这些，它只报告自己的事实（cam.ready）。----------

    @Volatile
    private var phase = Phase.IDLE

    /** 落盘目标。**只在 RECORDING 非 null。** 相机线程会读它，所以必须 volatile。 */
    @Volatile
    private var writer: SessionWriter? = null

    /** 早于这个时刻的相机帧不落盘（预热）。 */
    @Volatile
    private var framesFromTsNs = 0L

    /** IMU 收集器。**一次「准备」一个**，[teardown] 时丢掉。 */
    private var imu: ImuCollector? = null
    private var imuThread: HandlerThread? = null

    /**
     * 兜底策略：这一段的就绪判据没成立，而用户还是按了按钮。
     *
     * **注意这两个不是"就绪"，是"强行开始"。** 它们只在 [beginRecording] 里、主线程上
     * 置位一次，不在别处。老代码把它们和就绪混成一个标志
     * （`markImuReady("等 IMU 超时")` 会在 IMU **没出数**时把 `imuReady` 置 true），
     * 状态就在说谎——那正是竞态的温床。
     */
    private var camGaveUp = false
    private var imuGaveUp = false

    /** 上一次保留下来的 session 目录，给状态栏显示。 */
    private var lastSessionDir: File? = null

    private var dialog: AlertDialog? = null

    // ---------------- SLAM（M4：只到「能构造 System」为止）----------------

    /** 按过「启动 SLAM」的时刻，0 = 没按过。只用来算「已用多久」。 */
    private var slamStartedAtNs = 0L

    /** 缓存的构造状态。**只在主线程写**（[onButtonSlam] 和 [ticker]）。 */
    private var slamState = SLAM_IDLE

    // ---------------- 帧出口 ----------------

    /**
     * 每帧的出口。**跑在相机线程上**，实现里不能有阻塞操作。
     *
     * 两路各自有一道显式闸，都不靠时序：落盘的闸是 [phase] + [writer] + [framesFromTsNs]，
     * 预览的闸在 [PreviewView] 里。
     */
    private fun onCameraFrame(gray: ByteArray, tsNs: Long) {
        val w = writer
        // 先读 writer 再读 phase，和拆除时的写序相反，所以 SessionWriter.offerFrame
        // 里那道 running 检查是必需的。
        if (phase == Phase.RECORDING && w != null && tsNs >= framesFromTsNs) {
            w.offerFrame(SessionWriter.FrameItem(tsNs, gray, CameraCapture.W, CameraCapture.H))
        }

        // 预览**任何时候都给**。糊帧对预览是有用的信息——那正是准备时该看的东西。
        // 单独 try：预览出岔子不能把采集帧吃掉。
        try {
            preview.offer(gray, tsNs)
        } catch (e: Throwable) {
            Log.e(TAG, "预览丢帧", e)
        }
    }

    // ---------------- 生命周期 ----------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        btnA = findViewById(R.id.btnA)
        btnB = findViewById(R.id.btnB)
        btnSlam = findViewById(R.id.btnSlam)
        tvStatus = findViewById(R.id.tvStatus)
        preview = findViewById(R.id.preview)

        // 相机线程和相机对象都活得和 Activity 一样久，这样 start/stop 只是一条
        // 队列上的两个消息，顺序天然确定（前后台快速来回不会互相踩）。
        val ct = HandlerThread("camera").also { it.start() }
        camThread = ct
        cam = CameraCapture(this, Handler(ct.looper))
        cam.onFrame = ::onCameraFrame
        cam.logCapabilities()   // 打印设备能力，第一次跑一定要看 logcat

        btnA.setOnClickListener { onButtonA() }
        btnB.setOnClickListener { onButtonB() }
        btnSlam.setOnClickListener { onButtonSlam() }

        // 摸一下 SlamBridge，让 .so 在启动时就加载：JNI_OnLoad 里装着日志重定向，
        // 而 liborb_slam3.so 是它的依赖，静态初始化也在这一刻跑。
        // 用 nativeHello 不用 nativeLogSelfTest——前者只回一个字符串，不灌七行噪音。
        // 这同时是 STAGE5 §7 排查阶梯的第一级：「桥还通不通」。
        Log.i(TAG, "native 探针：" + SlamBridge.nativeHello())

        render()
    }

    /**
     * **这里什么都不做。** 相机和传感器都由「开始准备」打开，不由前台状态打开——
     * 让它们在 IDLE 也活着就是空耗电。切回来时要重新点一次「开始准备」。
     */
    override fun onStart() {
        super.onStart()
    }

    override fun onStop() {
        super.onStop()
        // 切后台就拆。后台采集会被系统限制帧率，数据不可信。
        // 正在录的话文件一律保住——录到一半也是真数据。
        teardown("切后台")
    }

    override fun onDestroy() {
        super.onDestroy()
        teardown("退出")
        cam.onFrame = null
        uiHandler.removeCallbacksAndMessages(null)
        camThread.quitSafely()   // quitSafely 会把已经排队的 stop() 跑完
        ioExecutor.shutdown()
    }

    // ---------------- 两个按钮 ----------------

    /** A 按钮 = 「退一层」。两级的代价和层级成正比。 */
    private fun onButtonA() {
        when (phase) {
            Phase.IDLE -> beginPrepare()
            Phase.ARMED -> teardown("取消准备")
            Phase.RECORDING -> confirmDiscard()
        }
    }

    private fun onButtonB() {
        when (phase) {
            Phase.IDLE -> Unit              // 按钮本来就是灰的，点不进来
            Phase.ARMED -> beginRecording()
            Phase.RECORDING -> stopRecording(discard = false)
        }
    }

    // ---------------- SLAM ----------------

    /**
     * 按「启动 SLAM」。**只生效一次**——`System` 构造一次要十几到几十秒，不能重来。
     *
     * [SlamBridge.nativeInit] 立刻返回，真正的构造在 C++ 那边的线程上跑，所以这里
     * 不会卡住界面。我们只负责：把路径准备好，**并且在调它之前把最常见的死法挡在门外**。
     */
    private fun onButtonSlam() {
        if (slamStartedAtNs != 0L) return
        // 相机权限不是 SLAM 的前提（M4 不喂帧），但词袋和 yaml 是。
        val vocab = resolveVocabPath() ?: return
        val settings = resolveSettingsPath() ?: return

        slamStartedAtNs = SystemClock.elapsedRealtimeNanos()
        slamState = SLAM_LOADING
        SlamBridge.nativeInit(vocab, settings, null)
        render()
        uiQueueTicker()
    }

    /**
     * 词袋路径。145 MB，没进 APK，靠 `tools/push-vocab.sh` 推到外部文件目录。
     *
     * 返回 null = 已经往状态栏写了可读的提示，调用方直接返回、**不要调 nativeInit**。
     */
    private fun resolveVocabPath(): String? {
        val f = File(getExternalFilesDir(null) ?: filesDir, VOCAB_NAME)
        return f.takeIf { it.isFile && it.length() > 0 && it.canRead() }?.absolutePath
            ?: refuse("词袋", f) {
                "跑一次 tools/push-vocab.sh 把它推上来（145 MB，不进 APK）。"
            }
    }

    /**
     * yaml 路径。`assets/phone.yaml` 是权威副本（STAGE5 §2.2），但 C++ 那边要的是
     * **真实文件路径**（`cv::FileStorage` 不吃 AssetManager），所以先落到私有目录。
     */
    private fun resolveSettingsPath(): String? {
        val dst = File(filesDir, SETTINGS_NAME)
        try {
            assets.open(SETTINGS_NAME).use { input ->
                dst.outputStream().use { output -> input.copyTo(output) }
            }
        } catch (e: IOException) {
            Log.e(TAG, "释放 $SETTINGS_NAME 失败", e)
            tvStatus.text = "打不开 assets/$SETTINGS_NAME：$e"
            return null
        }
        return dst.takeIf { it.length() > 0 && it.canRead() }?.absolutePath
            ?: refuse("配置", dst) { "assets/$SETTINGS_NAME 是空的，检查打包。" }
    }

    /**
     * **第一道防线**（STAGE5 §3.2）。路径不对时 ORB-SLAM3 用 `exit(-1)` 报错，
     * 表现是「App 整个退回桌面、logcat 干干净净」——能在这里挡住的就别让它跑进去。
     *
     * 挡不住的也得知道它挡不住：yaml **存在但缺一个必填键**照样会 exit(-1)，
     * 那种只能靠 M2 接进 logcat 的 `cerr`。两道的分工见改动记录。
     */
    private fun refuse(what: String, f: File, hint: () -> String): String? {
        Log.e(TAG, "找不到$what：${f.absolutePath}")
        tvStatus.text = "找不到$what：\n${f.absolutePath}\n\n${hint()}"
        return null
    }

    // ---------------- 准备 ----------------

    /**
     * IDLE → ARMED：**同时打开两条线**，IMU 注册 + 相机（含对焦初始化）。
     *
     * 所有入口统一先查一遍 [phase]：授权回调、按钮都可能重复触发。
     */
    private fun beginPrepare() {
        if (phase != Phase.IDLE) return

        // 没有权限时**什么都别动**，包括 phase。授权回调里会重新走一遍这个函数。
        if (!hasCameraPermission()) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), REQ_CAMERA)
            return
        }

        // IMU 先起步。本机传感器从 registerListener 到出数要 3 秒多，这段时间
        // 正好用来调机位、等对焦——两条线的等待完全重叠。
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
        uiQueueTicker()
    }

    // ---------------- 录制 ----------------

    /**
     * ARMED → RECORDING。
     *
     * **两个就绪判据都只为"识别出这是强制开始"而存在**——它们不制造保证，只是让
     * 降级这件事被记下来。真正的保证是 [framesFromTsNs]：相机帧被硬按住 300 ms。
     */
    private fun beginRecording() {
        if (phase != Phase.ARMED) return

        camGaveUp = !cam.ready
        imuGaveUp = imu?.ready != true
        if (camGaveUp) Log.w(TAG, "强制开始：对焦未收敛，将带着连续对焦落盘")
        if (imuGaveUp) {
            // 这条 ERROR 是「IMU 没出数」唯一的线索。删掉它就会出现
            // 「UI 显示录制中、cam0 一帧没有、没有任何报错」这种查不出来的状态。
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

        // 预热：相机帧要等到 +300ms 才开始落。
        // **IMU 都没出数就别等了**——预热存在的唯一理由就是让 IMU 领先，没有 IMU
        // 可领先的时候那 300 ms 是纯浪费，还会白白丢掉这一段开头能用的画面。
        framesFromTsNs = SystemClock.elapsedRealtimeNanos() + if (imuGaveUp) 0L else PREROLL_NS

        phase = Phase.RECORDING
        Log.i(TAG, "开始录制 $dir  预热 ${(framesFromTsNs - SystemClock.elapsedRealtimeNanos())
            .coerceAtLeast(0) / 1_000_000} ms")
        render()
    }

    /**
     * RECORDING → ARMED。
     *
     * **不碰相机、不碰传感器注册、不碰对焦。** 这正是「停下来之后还能立刻录下一段」
     * 的原因：下一段按下就是录，IMU 还是热的，不用再等 3.3 秒。
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
            // 宁可留下证据也不要静默删掉——「二次录制永远不落帧」那类 bug
            // 就是这么被淹掉的。
            Log.e(TAG, "这一段一帧都没落盘（时间戳对不上或写盘失败），目录保留：" + w.sessionDir)
        }
        Log.i(TAG, "结束录制  相机 ${w.framesWritten.get()} 帧 / IMU ${w.imuWritten.get()} 条"
            + "  首帧=${w.firstFrameTsNs} 首IMU=${w.firstImuTsNs}")

        if (!discard) {
            lastSessionDir = w.sessionDir
        } else if (w.threadAlive) {
            // stop() 的 join 超时了，writer 线程还活着。这时删目录会和它的写并发，
            // writeFrame 会一路 FileNotFoundException。宁可把目录留下。
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
     * 回到 IDLE。**幂等**，「取消准备」/ `onStop` / `onDestroy` 共用。
     *
     * 注意这里**没有**「先断生产者再关消费者」那种漂亮顺序可言：[cam.stop] 是 post 到
     * 相机线程的（异步），其余全是主线程同步跑的。真正挡住迟到帧的是 [phase] 那道闸，
     * 加上 `SessionWriter.offerFrame` 里的 `running` 检查。
     */
    private fun teardown(reason: String) {
        if (phase == Phase.IDLE) return
        Log.i(TAG, "拆掉准备状态（$reason）")

        if (phase == Phase.RECORDING) stopRecording(discard = false)

        preview.setActive(false)   // 预览闸先关，屏幕立刻黑
        phase = Phase.IDLE

        dialog?.dismiss()
        dialog = null
        // 不是 removeCallbacks —— SLAM 可能还在加载，而它的进度靠这个 ticker 刷。
        // 重新排一遍，让 ticker 自己决定该不该继续。
        uiQueueTicker()

        imu?.stop()
        imuThread?.quitSafely()
        // 必须置 null：不置的话下次 ARMED 读到的是上一轮那个已经停掉的 collector。
        imu = null
        imuThread = null

        cam.stop()
        camGaveUp = false
        imuGaveUp = false
        render()
    }

    // ---------------- 状态栏 ----------------

    /**
     * 每 [TICK_MS] 刷一次。**什么时候继续跑由它自己判断**：采集端不在 IDLE，
     * 或者 SLAM 还在加载。
     *
     * ⚠️ 「SLAM 还在加载」这一条是必需的：那一段发生在 [Phase.IDLE] 期间，
     * 而这里原先遇到 IDLE 就直接 return —— 那样「词袋加载中…」永远不刷新。
     */
    private val ticker = object : Runnable {
        override fun run() {
            if (slamStartedAtNs != 0L) slamState = SlamBridge.nativeState()
            val slamBusy = slamStartedAtNs != 0L && slamState == SLAM_LOADING
            if (phase == Phase.IDLE && !slamBusy) return
            render()
            uiHandler.postDelayed(this, TICK_MS)
        }
    }

    /** 重新起一遍 ticker。重复调用无害——它每轮自己决定还继不继续。 */
    private fun uiQueueTicker() {
        uiHandler.removeCallbacks(ticker)
        uiHandler.post(ticker)
    }

    /** 刷按钮和状态栏。**按钮的颜色和文字都是从 [phase] 推出来的**，不另存一份。 */
    private fun render() {
        val p = phase
        val w = writer
        val camOk = cam.ready
        val imuOk = imu?.ready == true

        // ---- SLAM（M4 只有「启动」，没有停止/重定位）----
        // 失败之后也不放开：C++ 那边的 init 只生效一次（CAS on kIdle），再按也没用。
        btnSlam.isEnabled = slamStartedAtNs == 0L
        btnSlam.text = if (slamStartedAtNs == 0L) "启动 SLAM" else "SLAM 已启动"

        // ---- 按钮 ----
        // 文字色和背景**成对**设置：中性态（tint=null，白底）配深字，
        // 着色态（绿/红）配白字。只改背景不改字，就会复现「白底白字看不见」。
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
                // 一进准备就可用、就是红的。用户随时可以决定不等了——
                // 不做"灰按钮等就绪"那种中间态，那会让人以为要等它自己变。
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
                // 停止不是危险操作，恢复默认外观；危险的是左边那个「取消录制」，
                // 它自带一道确认弹窗。
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
                // 权威事实：这一段到底写进去没有。事前那个 ready 判断不作数了，看文件。
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

        // SLAM 那一行单独放最上面：它是另一条线（APP_DESIGN §6 的四个正交开关之一）。
        // 「加载中」带已用秒数 —— 文档里一直写的是猜的「十几到几十秒」，实测数字靠它读。
        val slamHead = when {
            slamStartedAtNs == 0L -> "SLAM：未启动"
            slamState == SLAM_LOADING -> {
                val sec = (SystemClock.elapsedRealtimeNanos() - slamStartedAtNs) / 1_000_000_000
                "SLAM：词袋加载中…（已 ${sec}s，十几到几十秒属正常）"
            }
            slamState == SLAM_READY -> "SLAM：就绪"
            slamState == SLAM_FAILED -> "SLAM：构造失败 —— 看 logcat（tag slam）"
            else -> "SLAM：状态未知（$slamState）"
        }

        tvStatus.text = "$slamHead\n\n$head\n\n$body"
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
            // 运行时权限对话框**不触发 onStop**，所以按 A 时那一趟出去了就是出去了，
            // 这里得把「开始准备」接着做完。
            beginPrepare()
        } else {
            tvStatus.text = "相机权限被拒绝。\n请到 设置 → 应用 → SlamCapture → 权限 里手动开启。"
        }
    }

    // ---------------- 文件 ----------------

    private fun newSessionDir(): File {
        // getExternalFilesDir 可能返回 null（外部存储没挂载）。这时退到内部存储，
        // **绝不能**用 File(null, ...)——那会落到进程的 CWD，在 Android 上就是 "/"。
        val base = getExternalFilesDir(null) ?: filesDir
        val dir = File(File(base, "sessions"), "session_${System.currentTimeMillis()}")
        dir.mkdirs()
        return dir
    }

    /**
     * 删掉一个 session 目录。
     *
     * 先 rename —— 那一瞬间目录名就告诉所有人"这个不算数了"，而且是原子操作；
     * 真正删掉上万个 PNG 是后台的事。
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
