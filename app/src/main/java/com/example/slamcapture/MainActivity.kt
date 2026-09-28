package com.example.slamcapture

import android.Manifest
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.GravityCompat
import androidx.drawerlayout.widget.DrawerLayout
import java.io.File
import java.io.IOException

/**
 * SLAM 主界面。**跑 SLAM 的那一屏。**
 *
 * 排版和录制页**故意不同**：这里是「状态 → 画面 → 按钮」，录制页是「画面 → 状态 → 按钮」。
 * 原因：这一页你盯着的是「**现在能不能用**」（词袋加载完没有、跟踪丢没丢），画面反而是辅助。
 *
 * 相机和 IMU **这一页自己持有**，和 [CaptureActivity] 各开各的 —— 两个模式互斥，
 * 理由是对焦所有权和性能（APP_DESIGN §6.3）。
 *
 * 那个主按钮是**三段**（和录制页的两段式对齐）：
 *
 * ```
 * 未准备 ──[准备 SLAM]──► 加载中（词袋 38 s）──► 就绪
 *                                                │
 *                                      [启动 SLAM] ▼
 *                                              运行中 ──[结束]──► 就绪
 * ```
 *
 * **准备** = 构造 `System`（贵且慢，38 秒）；**启动** = 开相机 + IMU + 开始吃帧。
 * 分开的理由和录制页那个一样：**贵的那一步不该被反复付**。
 *
 * ⚠️ **没有「取消准备」** —— `System` 构造没法取消，而且造好之后也没有安全的销毁时机
 * （见 [onButtonSlam] 上面那段）。所以这个按钮是**单行道**。
 */
class MainActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "MainActivity"

        /** 入口页选中的已保存地图（绝对路径）。**没有这个 extra 就是「新建地图」**。 */
        const val EXTRA_MAP_FILE = "map_file"

        private const val VOCAB_NAME = "ORBvoc.txt"
        private const val SETTINGS_NAME = "phone.yaml"

        // SlamBridge.nativeState() 的取值，和 C++ 侧 slam::State 一一对应。
        private const val SLAM_IDLE = 0
        private const val SLAM_LOADING = 1
        private const val SLAM_READY = 2
        private const val SLAM_FAILED = 3

        // ORB-SLAM3 的 eTrackingState（Tracking.h:121）。
        private const val TRK_NOT_READY = -1
        private const val TRK_NO_IMAGES = 0
        private const val TRK_NOT_INITIALIZED = 1
        private const val TRK_OK = 2
        private const val TRK_RECENTLY_LOST = 3
        private const val TRK_LOST = 4

        private const val TICK_MS = 250L
        private const val REQ_CAMERA = 1

        private val DARK_TEXT = Color.parseColor("#DE000000")
        private val RED = Color.parseColor("#C62828")
    }

    private lateinit var btnSlam: Button
    private lateinit var btnReloc: Button
    private lateinit var btnRecord: Button
    private lateinit var tvStatus: TextView
    private lateinit var preview: PreviewView
    private lateinit var drawer: DrawerLayout

    private val uiHandler = Handler(Looper.getMainLooper())

    // ---------------- 页面状态 ----------------

    /** 入口页选中的已保存地图。**null = 新建地图**。不加 enum，它本身就是那个参数。 */
    private var mapFile: String? = null

    /** C++ 那边的 `System` 状态。**只在主线程写**。 */
    private var slamState = SLAM_IDLE
    private var slamElapsedMs = 0L

    /**
     * 「正在吃帧」—— 相机和 IMU 开着、`nativeStart()` 调过了。
     *
     * 和 [slamState] 是**两件事**：`System` 造好之后一直活着（`slamState == SLAM_READY`），
     * 而吃不吃帧由用户按按钮决定。这正是「停止再启动不用重等 38 秒」的原因。
     */
    private var slamRunning = false

    // ---------------- 硬件（本页自己持有）----------------

    private lateinit var camThread: HandlerThread
    private lateinit var cam: CameraCapture
    private var imu: ImuCollector? = null
    private var imuThread: HandlerThread? = null

    // ---------------- 落盘（抽屉里的开关）----------------

    /** 只在录制中非 null。相机线程和 IMU 线程都会读，必须 volatile。 */
    @Volatile
    private var writer: SessionWriter? = null
    private var recording = false
    private var lastSessionDir: File? = null

    // ---------------- 位姿 ----------------

    /** 复用同一个数组 —— JNI 里构造 Java 数组代价高（§6 约定 3）。 */
    private val poseBuf = FloatArray(16)
    private var poseValid = false
    private var trkState = TRK_NOT_READY
    private var lastTrajectory: File? = null

    // ---------------- 生命周期 ----------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        btnSlam = findViewById(R.id.btnSlam)
        btnReloc = findViewById(R.id.btnReloc)
        btnRecord = findViewById(R.id.btnRecord)
        tvStatus = findViewById(R.id.tvStatus)
        preview = findViewById(R.id.preview)
        drawer = findViewById(R.id.drawer)

        // 相机线程和相机对象活得和 Activity 一样久（和录制页同一个理由：
        // start/stop 只是一条队列上的两个消息，顺序天然确定）。
        val ct = HandlerThread("camera").also { it.start() }
        camThread = ct
        cam = CameraCapture(this, Handler(ct.looper))
        cam.onFrame = ::onCameraFrame
        cam.logCapabilities()

        btnSlam.setOnClickListener { onButtonSlam() }
        findViewById<Button>(R.id.btnMenu).setOnClickListener {
            drawer.openDrawer(GravityCompat.END)
        }
        btnRecord.setOnClickListener { toggleRecording() }
        // 「重定位」现在点不动（恒灰）。判据是「有没有旧图」（§2.4），还没做。
        btnReloc.setOnClickListener { }

        mapFile = intent.getStringExtra(EXTRA_MAP_FILE)
        Log.i(TAG, "地图来源：" + (mapFile ?: "新建地图"))

        // 摸一下 SlamBridge，让 .so 在启动时就加载（JNI_OnLoad 里装着日志重定向）。
        // 用 nativeHello 不用 nativeLogSelfTest——前者只回一个字符串，不灌七行噪音。
        Log.i(TAG, "native 探针：" + SlamBridge.nativeHello())

        // 读一次构造状态：可能是**上一个 Activity 实例**留下的（从入口页再进来一次）。
        // System 是进程级的，Kotlin 这边每次都要重新问，不能自己记。
        slamState = SlamBridge.nativeState()
        if (slamState == SLAM_LOADING) uiQueueTicker()

        render()
    }

    override fun onStop() {
        super.onStop()
        // 切后台等同「停止」（§7.3）。回来要重新按「启动」—— 后台相机会被限帧，
        // 而且手机可能移动了，上一帧的位姿不再成立。
        stopHardware("切后台")
    }

    override fun onDestroy() {
        super.onDestroy()
        stopHardware("退出")
        cam.onFrame = null
        uiHandler.removeCallbacksAndMessages(null)
        camThread.quitSafely()
    }

    // ---------------- 「准备 / 启动 / 结束 SLAM」 ----------------
    //
    // 和录制页那个两段式对齐：
    //
    //     未准备 ──[准备 SLAM]──► 加载中（词袋 38 s）──► 就绪
    //                                                    │
    //                                          [启动 SLAM] ▼
    //                                                  运行中 ──[结束]──► 就绪
    //
    // ⚠️ **没有「取消准备」** —— 和录制页不一样。`System` 的构造没法中途取消，
    // 而且造好之后**也没有安全的销毁时机**（它没有析构函数、`Shutdown()` 又不 join 线程，
    // delete 就是 use-after-free）。所以这个按钮是**单行道**：准备过就回不去「未准备」。
    // 这不是偷懒，是把 ORB-SLAM3 的事实如实反映到界面上。

    private fun onButtonSlam() {
        // —— 运行中：结束（结束会话，那张图成为「旧图」，§2.3）——
        if (slamRunning) {
            stopHardware("用户结束")
            render()
            return
        }
        // 加载中 / 失败：不响应
        if (slamState == SLAM_LOADING || slamState == SLAM_FAILED) return

        if (!hasCameraPermission()) {
            ActivityCompat.requestPermissions(
                this, arrayOf(Manifest.permission.CAMERA), REQ_CAMERA)
            return
        }

        // —— System 已经造好了：只开硬件，不重等那 38 秒（§5 决策一）——
        if (slamState == SLAM_READY) {
            openHardware()
            render()
            return
        }

        // —— 第一次：构造 System ——
        val vocab = resolveVocabPath() ?: return
        val settings = resolveSettingsPath() ?: return
        // 选了图就把路径一路传下去。C++ 那边还不支持载入旧图（会打一行「收到但没实现」），
        // 但通道先通着，免得等到 M8 才发现送不过去。
        SlamBridge.nativeInit(vocab, settings, mapFile)
        slamState = SlamBridge.nativeState()   // 正常是 kLoading
        render()
        uiQueueTicker()
    }

    /**
     * 开相机 + IMU + `nativeStart()`。
     *
     * **等 `System` 就绪之后才调** —— 词袋加载那 38 秒里灌帧没人消费，队列满了纯属白丢。
     */
    private fun openHardware() {
        if (slamRunning || !hasCameraPermission()) return

        // IMU 先起步（本机从注册到出数要 3 秒多，正好和相机启动重叠）。
        val t = HandlerThread("imu").also { it.start() }
        val c = ImuCollector(this)
        imuThread = t
        imu = c
        c.onSample = { ts, wx, wy, wz, ax, ay, az ->
            // **跑在 IMU 线程上，里面绝不能阻塞**（铁律 1）。
            try {
                SlamBridge.nativePushImu(ts, wx, wy, wz, ax, ay, az)
            } catch (e: Throwable) {
                Log.e(TAG, "喂 IMU 失败", e)
            }
        }
        c.start(Handler(t.looper))

        preview.setActive(true)
        cam.start()
        cam.beginSettle()   // 收敛对焦然后锁死 —— 锁焦的后果是 fx 只在那个距离上等于 435.2

        SlamBridge.nativeStart()
        slamRunning = true
        Log.i(TAG, "SLAM 开始吃帧")
        // ⚠️ 必须自己把 ticker 叫起来：它在词袋加载完的那一刻就停了（busy 变 false），
        // 而位姿是靠它刷的。不叫的话界面会停在「已就绪」一动不动。
        uiQueueTicker()
    }

    /**
     * 关硬件。**幂等**，「停止」/ 切后台 / 退出共用。
     *
     * ⚠️ **不销毁 `System`** —— 那要重付 38 秒。C++ 那边 `slam::stop()` 只是停止吃帧，
     * 顺便清空积压的队列（否则下次启动会吃到停止前的旧帧）。
     */
    private fun stopHardware(reason: String) {
        if (!slamRunning) return
        Log.i(TAG, "SLAM 停止吃帧（$reason）")

        SlamBridge.nativeStop()   // 先让它别吃了，再拆硬件
        slamRunning = false

        stopRecording()           // 正在录的话文件一律保留（录到一半也是真数据）

        // 轨迹**只在用户主动停止 / 退出时导出**，切后台不导。
        // 因为 C++ 那边的位姿历史是**跨「停止/启动」累积**的（进程死才清），
        // 所以切后台不导不会丢东西 —— 反而能攒成一份完整的。
        if (reason != "切后台") exportTrajectory()

        cam.stop()
        imu?.stop()
        imuThread?.quitSafely()
        // 必须置 null：不置的话下次读到的是上一轮那个已经停掉的 collector。
        imu = null
        imuThread = null
        preview.setActive(false)
    }

    // ---------------- 帧出口 ----------------

    /**
     * 每帧的出口。**跑在相机线程上**，实现里不能有阻塞操作。
     *
     * **三个消费者，各自一道显式闸、各自包 try** —— 一路出岔子不能把另外两路吃掉。
     */
    private fun onCameraFrame(gray: ByteArray, tsNs: Long) {
        // ① 喂 SLAM。一条 JNI 调用 + 一次拷贝就返回。
        if (slamRunning) {
            try {
                SlamBridge.nativePushImage(tsNs, gray, CameraCapture.W, CameraCapture.H)
            } catch (e: Throwable) {
                Log.e(TAG, "喂帧失败", e)
            }
        }

        // ② 落盘（抽屉里的开关）。
        val w = writer
        if (recording && w != null) {
            try {
                w.offerFrame(SessionWriter.FrameItem(tsNs, gray, CameraCapture.W, CameraCapture.H))
            } catch (e: Throwable) {
                Log.e(TAG, "落盘丢帧", e)
            }
        }

        // ③ 预览。
        try {
            preview.offer(gray, tsNs)
        } catch (e: Throwable) {
            Log.e(TAG, "预览丢帧", e)
        }
    }

    // ---------------- 落盘开关（抽屉里）----------------

    private fun toggleRecording() {
        if (recording) stopRecording() else startRecording()
    }

    private fun startRecording() {
        if (!slamRunning) {
            Log.w(TAG, "SLAM 没在跑，不能录")
            return
        }
        if (writer != null) return

        val dir = newSessionDir()
        val w = SessionWriter(dir)
        // ⚠️ 标出来：这样录的和「录制模式」录的才分得清 —— 格式一模一样，
        // 而这段的帧率是被落盘拖低过的，用途也不同（见 SessionWriter.withSlam）。
        w.withSlam = true
        w.start()
        if (w.ioError != null) {
            Log.e(TAG, "建 session 文件失败：${w.ioError}")
            tvStatus.text = "无法创建输出文件：${w.ioError}\n$dir"
            return
        }
        writer = w
        recording = true
        imu?.attach(w)
        Log.i(TAG, "开始录制（带 SLAM）$dir")
        render()
    }

    private fun stopRecording() {
        if (!recording) return
        recording = false
        imu?.detach()
        val w = writer
        writer = null
        w?.stop()
        lastSessionDir = w?.sessionDir
        if (w != null) {
            Log.i(TAG, "停止录制  相机 ${w.framesWritten.get()} 帧 / IMU ${w.imuWritten.get()} 条")
        }
        render()
    }

    // ---------------- 位姿与轨迹 ----------------

    private fun exportTrajectory() {
        val dir = getExternalFilesDir(null) ?: filesDir
        val f = File(dir, "trajectory_${System.currentTimeMillis()}.txt")
        try {
            SlamBridge.nativeDumpTrajectory(f.absolutePath)
            lastTrajectory = f
        } catch (e: Throwable) {
            Log.e(TAG, "导出轨迹失败", e)
        }
    }

    // ---------------- 状态栏 ----------------

    /**
     * **加载中要跑，运行中也要跑。**
     *
     * 两个都必需：加载中是为了刷那 38 秒的进度；运行中是为了刷位姿和跟踪状态。
     * ⚠️ 而「刚变成 `kReady` 的那一刻」两件事同时发生 —— 开硬件也是在那一刻做的，
     * 所以这里不能一看到不是 `kLoading` 就 return。
     */
    private val ticker = object : Runnable {
        override fun run() {
            if (slamState == SLAM_LOADING) {
                slamState = SlamBridge.nativeState()
                slamElapsedMs = SlamBridge.nativeElapsedMs()
                // ⚠️ **不要在这儿开硬件。** 「就绪」和「启动」是两个阶段，
                // 中间隔着用户那一下 —— 自动开会把三段式黏成两段（改标签时漏改过一版）。
            }
            if (slamRunning) {
                poseValid = SlamBridge.nativeGetPose(poseBuf)
                trkState = SlamBridge.nativeGetTrackingState()
            }

            // ⚠️ **先刷，再判断要不要继续。** 反过来写的话，「词袋加载完」那一轮会
            // 读到 READY、算出 busy=false、然后在 render **之前**就 return ——
            // 界面永远停在「加载中…（已 40s）」，时间不走、按钮也不变。
            // **最后一轮刷新被自己吃掉了。**
            render()

            if (slamState == SLAM_LOADING || slamRunning) {
                uiHandler.postDelayed(this, TICK_MS)
            }
        }
    }

    private fun uiQueueTicker() {
        uiHandler.removeCallbacks(ticker)
        uiHandler.post(ticker)
    }

    private fun render() {
        // ---- 按钮 ----
        // 三个标签对应三个阶段。**加载中和构造失败都不可点** —— 前者没法取消，后者重试也没用
        // （C++ 那边的 `init` 只生效一次）。
        btnSlam.isEnabled = slamRunning || slamState == SLAM_IDLE || slamState == SLAM_READY
        btnSlam.text = when {
            slamRunning -> "结束"
            slamState == SLAM_LOADING -> "加载中…"
            slamState == SLAM_READY -> "启动 SLAM"
            slamState == SLAM_FAILED -> "构造失败"
            else -> "准备 SLAM"
        }
        btnSlam.backgroundTintList =
            if (slamRunning) ColorStateList.valueOf(RED) else null
        btnSlam.setTextColor(if (slamRunning) Color.WHITE else DARK_TEXT)

        // 「重定位」的灰 / 亮只看有没有旧图（§2.4）。还没有 referenceMap，所以恒灰。
        btnReloc.isEnabled = false

        btnRecord.isEnabled = slamRunning
        btnRecord.text = if (recording) "停止录制" else "开始录制"

        // ---- 状态栏 ----
        val mapLabel = mapFile?.let { File(it).name } ?: "新建地图"
        // `System` 造好之后有两种情形，先算出来 —— 塞进 `when` 的 else 分支里会写出
        // 「一个 when 里两个 else」那种读起来像错的东西。
        val afterReady = if (slamRunning) {
            trackingLine()
        } else {
            "SLAM：已就绪（没在吃帧）\n\n点「启动 SLAM」开始 —— 相机和 IMU 这一刻才打开。"
        }
        val head = when (slamState) {
            SLAM_IDLE ->
                "SLAM：未准备（$mapLabel）\n\n点「准备 SLAM」构造 System，约 38 秒。"
            SLAM_LOADING -> "SLAM：词袋加载中…（已 ${slamElapsedMs / 1000}s，本机约 38 秒）"
            SLAM_FAILED -> "SLAM：构造失败 —— 看 logcat（tag slam）"
            else -> afterReady
        }

        val body = buildString {
            if (slamRunning && poseValid) {
                append(
                    "位置 m: %.3f, %.3f, %.3f".format(poseBuf[12], poseBuf[13], poseBuf[14])
                )
            }
            val w = writer
            if (recording && w != null) {
                append("\n录制中：相机 ${w.framesWritten.get()} 帧 / IMU ${w.imuWritten.get()} 条")
            }
            lastSessionDir?.let { append("\n上一次录制：${it.absolutePath}") }
            lastTrajectory?.let { append("\n轨迹：${it.absolutePath}") }
        }
        tvStatus.text = if (body.isEmpty()) head else "$head\n\n$body"
    }

    /**
     * 跟踪状态那一行。
     *
     * `NOT_INITIALIZED` 那一条要把话说明白：**惯性初始化需要运动激励，静止举着手机
     * 永远初始化不了** —— 用户会以为坏了（STAGE7 §2.3 的引导文案）。
     */
    private fun trackingLine(): String = when (trkState) {
        TRK_OK -> "● 跟踪中（OK）"
        TRK_RECENTLY_LOST -> "… 跟踪不稳，放慢动作"
        TRK_LOST -> "… 已丢失，对着刚才扫过的地方"
        TRK_NOT_INITIALIZED -> "… 惯性初始化中 —— 请缓慢走动，静止举着不会初始化"
        TRK_NO_IMAGES -> "… 还没收到画面"
        else -> "… 还没准备好（跟踪状态 $trkState）"
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
            // 把刚才那一下就接着做完。
            onButtonSlam()
        } else {
            tvStatus.text = "相机权限被拒绝。\n请到 设置 → 应用 → SlamCapture → 权限 里手动开启。"
        }
    }

    // ---------------- 文件 ----------------

    /**
     * 词袋路径。145 MB，没进 APK，靠 `tools/push-vocab.sh` 推到外部文件目录。
     *
     * 返回 null = 已经往状态栏写了可读的提示，调用方直接返回、**不要调 nativeInit**。
     */
    private fun resolveVocabPath(): String? {
        val f = File(getExternalFilesDir(null) ?: filesDir, VOCAB_NAME)
        return f.takeIf { it.isFile && it.length() > 0 && it.canRead() }?.absolutePath
            ?: refuse("词袋", f) { "跑一次 tools/push-vocab.sh 把它推上来（145 MB，不进 APK）。" }
    }

    /**
     * yaml 路径。`assets/phone.yaml` 是权威副本，但 C++ 那边要的是**真实文件路径**
     * （`cv::FileStorage` 不吃 AssetManager），所以先落到私有目录。
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
     * 表现是「App 整个退回桌面、logcat 干干净净」—— 能在这里挡住的就别让它跑进去。
     *
     * 挡不住的也得知道它挡不住：yaml **存在但缺一个必填键**照样会 exit(-1)，
     * 那种只能靠 M2 接进 logcat 的 `cerr`。
     */
    private fun refuse(what: String, f: File, hint: () -> String): String? {
        Log.e(TAG, "找不到$what：${f.absolutePath}")
        tvStatus.text = "找不到$what：\n${f.absolutePath}\n\n${hint()}"
        return null
    }

    private fun newSessionDir(): File {
        // getExternalFilesDir 可能返回 null（外部存储没挂载）。这时退到内部存储，
        // **绝不能**用 File(null, ...)——那会落到进程的 CWD，在 Android 上就是 "/"。
        val base = getExternalFilesDir(null) ?: filesDir
        val dir = File(File(base, "sessions"), "session_${System.currentTimeMillis()}")
        dir.mkdirs()
        return dir
    }
}
