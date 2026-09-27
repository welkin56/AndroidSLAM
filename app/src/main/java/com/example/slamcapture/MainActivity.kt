package com.example.slamcapture

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.GravityCompat
import androidx.drawerlayout.widget.DrawerLayout
import java.io.File
import java.io.IOException

/**
 * SLAM 主界面。
 *
 * 排版和录制页**故意不同**：这里是「状态 → 画面 → 按钮」，录制页是「画面 → 状态 → 按钮」。
 * 原因：这一页你盯着的是「**现在能不能用**」（词袋加载完没有、跟踪丢没丢），画面反而是辅助；
 * 录制页盯着的是画面（构图、对上焦没有）。
 *
 * ⚠️ 这一步还是**空壳**：中间的画面是黑的（相机接线是 M5）、
 * 「重定位」恒灰（还没有旧图）、抽屉里只有一条占位。
 *
 * 录制**不在这里** —— 它和 SLAM 是两个互斥的模式，各开各的相机（见 [CaptureActivity]
 * 和 APP_DESIGN §6/§7.1）。
 */
class MainActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "MainActivity"

        /**
         * 入口页选中的已保存地图（绝对路径）。**没有这个 extra 就是「新建地图」**。
         *
         * 「新建」和「载入旧图」是**同一个参数的两个取值**，不是两种模式（APP_DESIGN §2.4）。
         */
        const val EXTRA_MAP_FILE = "map_file"

        /** 词袋文件名。**145 MB，不进 APK** —— 用 `tools/push-vocab.sh` 推到外部文件目录。 */
        private const val VOCAB_NAME = "ORBvoc.txt"

        /** 相机配置。权威副本在 `assets/`（STAGE5 §2.2），用时落到私有目录再给 C++。 */
        private const val SETTINGS_NAME = "phone.yaml"

        // SlamBridge.nativeState() 的取值，和 C++ 侧 slam::State 一一对应。
        private const val SLAM_IDLE = 0
        private const val SLAM_LOADING = 1
        private const val SLAM_READY = 2
        private const val SLAM_FAILED = 3

        private const val TICK_MS = 250L
    }

    private lateinit var btnSlam: Button
    private lateinit var btnReloc: Button
    private lateinit var tvStatus: TextView
    private lateinit var drawer: DrawerLayout

    private val uiHandler = Handler(Looper.getMainLooper())

    /** 入口页选中的已保存地图。**null = 新建地图**。不加 enum，它本身就是那个参数。 */
    private var mapFile: String? = null

    /** 缓存的构造状态。**只在主线程写**（[onButtonSlam] 和 [ticker]）。 */
    private var slamState = SLAM_IDLE

    /**
     * 词袋已经加载了多久（毫秒），**由 C++ 报**。
     *
     * 不用 Kotlin 自己掐表：构造是进程级的事实，而计时是 Activity 级的状态 ——
     * 从入口页再进来一次，计时归零而 `System` 还在造，界面就卡在「加载中」转圈。
     */
    private var slamElapsedMs = 0L

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        btnSlam = findViewById(R.id.btnSlam)
        btnReloc = findViewById(R.id.btnReloc)
        tvStatus = findViewById(R.id.tvStatus)
        drawer = findViewById(R.id.drawer)

        btnSlam.setOnClickListener { onButtonSlam() }
        findViewById<Button>(R.id.btnMenu).setOnClickListener {
            drawer.openDrawer(GravityCompat.END)
        }
        // 「重定位」现在点不动（按钮恒灰）。M8 之后判据是「有没有旧图」（§2.4）。
        btnReloc.setOnClickListener { }

        mapFile = intent.getStringExtra(EXTRA_MAP_FILE)
        Log.i(TAG, "地图来源：" + (mapFile ?: "新建地图"))

        // 摸一下 SlamBridge，让 .so 在启动时就加载：JNI_OnLoad 里装着日志重定向，
        // 而 liborb_slam3.so 是它的依赖，静态初始化也在这一刻跑。
        // 用 nativeHello 不用 nativeLogSelfTest——前者只回一个字符串，不灌七行噪音。
        // 这同时是 STAGE5 §7 排查阶梯的第一级：「桥还通不通」。
        Log.i(TAG, "native 探针：" + SlamBridge.nativeHello())

        // 读一次构造状态：可能是**上一个 Activity 实例**留下的（从入口页再进来一次）。
        // System 是进程级的，Kotlin 这边每次都要重新问，不能自己记。
        slamState = SlamBridge.nativeState()
        if (slamState == SLAM_LOADING) uiQueueTicker()

        render()
    }

    override fun onDestroy() {
        super.onDestroy()
        uiHandler.removeCallbacksAndMessages(null)
    }

    // ---------------- 启动 SLAM ----------------

    /**
     * **只生效一次** —— `System` 构造一次要三十几秒，不能重来。
     *
     * [SlamBridge.nativeInit] 立刻返回，真正的构造在 C++ 那边的线程上跑，所以这里
     * 不会卡住界面。我们只负责：把路径准备好，**并且在调它之前把最常见的死法挡在门外**。
     *
     * ⚠️ 这里**不开相机、不喂帧** —— 那是 M5。现在按下去只是把 `System` 造出来。
     */
    private fun onButtonSlam() {
        if (slamState != SLAM_IDLE) return
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

    // ---------------- 状态栏 ----------------

    /**
     * **只在词袋加载期间跑**，加载完就停 —— 没有别的要刷的东西。
     *
     * 「加载中」那 38 秒必须有它：不主动刷的话「已 Xs」会一直停在 0。
     */
    private val ticker = object : Runnable {
        override fun run() {
            if (slamState != SLAM_LOADING) return
            slamState = SlamBridge.nativeState()
            slamElapsedMs = SlamBridge.nativeElapsedMs()
            render()
            uiHandler.postDelayed(this, TICK_MS)
        }
    }

    private fun uiQueueTicker() {
        uiHandler.removeCallbacks(ticker)
        uiHandler.post(ticker)
    }

    private fun render() {
        btnSlam.isEnabled = slamState == SLAM_IDLE
        btnSlam.text = if (slamState == SLAM_IDLE) "启动 SLAM" else "SLAM 已启动"
        // 「重定位」的灰 / 亮只看有没有旧图（§2.4）。这一步还没有 referenceMap，所以恒灰。
        btnReloc.isEnabled = false

        val mapLabel = mapFile?.let { File(it).name } ?: "新建地图"
        // 「约 38 秒」是**实测值**（2026-09-27，Xiaomi PJD110）—— 文档里原先写的是猜的「十几到几十秒」。
        tvStatus.text = when (slamState) {
            SLAM_IDLE -> "SLAM：未启动（$mapLabel）\n\n点「启动 SLAM」构造 System，第一次约 38 秒。"
            SLAM_LOADING -> "SLAM：词袋加载中…（已 ${slamElapsedMs / 1000}s，本机约 38 秒）"
            SLAM_READY -> "SLAM：就绪"
            SLAM_FAILED -> "SLAM：构造失败 —— 看 logcat（tag slam）"
            else -> "SLAM：状态未知（$slamState）"
        }
    }
}
