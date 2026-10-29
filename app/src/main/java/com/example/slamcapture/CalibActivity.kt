package com.example.slamcapture

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs

/**
 * 标定页：对着棋盘格移动，攒够视角后在本机算内参，存进 [CalibStore]。
 *
 * 视角全在 native 内存里，不落盘、不需要 PC。一档 = 一个对焦屈光度：
 * 对焦锁在哪儿，这一行就属于哪个屈光度，之后按它插值。
 */
class CalibActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "CalibActivity"

        /** 内角点数：A4 上是 7×5 个方格、每格 30 mm。 */
        private const val COLS = 6
        private const val ROWS = 4

        /** 攒够这么多视角就自动拟合。 */
        private const val TARGET_VIEWS = 30

        /** 每 5 帧检一次：SB 检测器在 640×480 上要几十毫秒，不能每帧都做。 */
        private const val DETECT_EVERY = 5

        private const val TICK_MS = 300L
        private const val REQ_CAMERA = 1

        /** 画面外圈（最外 30%）至少要占到的角点比例。畸变靠这些点约束。 */
        private const val OUTER_TARGET = 0.10f
    }

    private enum class Phase { IDLE, ARMED, CAPTURING }

    private lateinit var preview: PreviewView
    private lateinit var tvStatus: TextView
    private lateinit var btnA: Button
    private lateinit var btnB: Button
    private lateinit var btnRefocus: Button
    private lateinit var patternWrap: View
    private lateinit var patternView: PatternView

    private lateinit var camThread: HandlerThread
    private lateinit var cam: CameraCapture

    private val uiHandler = Handler(Looper.getMainLooper())
    private val calibWorker = Executors.newSingleThreadExecutor()

    /** 同一时刻只允许一帧在检测：检测慢于抽帧间隔时，无界队列会一直堆积。 */
    private val detecting = AtomicBoolean(false)

    /** 落盘只在 UI 线程改，相机线程只读。 */
    @Volatile
    private var phase = Phase.IDLE
    @Volatile
    private var capturing = false

    private var focusLocked = false
    private var lockedDiopter: Float? = null
    private var frameSeq = 0
    private var finished = false

    private var uiMessage: String? = null
    /** 前 9 个是九宫格，第 10 个是画面外圈占比。 */
    private val covBuf = FloatArray(10)
    private val dispBuf = FloatArray(9)

    /** 否决提示用的方位名，顺序与 [displayCoverage] 一致。 */
    private val cellNames = arrayOf("左上", "上", "右上", "左", "中间", "右", "左下", "下", "右下")

    /**
     * 每格缺覆盖时该做什么。板子通常钉在墙上/屏幕上不动，所以要挪的是手机：
     * 手机往哪走，板子就在画面里往反方向去。
     */
    private val cellHints = arrayOf(
        "手机往右下挪，让板子落在画面左上角",
        "手机往下挪，让板子落在画面上方",
        "手机往左下挪，让板子落在画面右上角",
        "手机往右挪，让板子落在画面左侧",
        "让板子回到画面中间",
        "手机往左挪，让板子落在画面右侧",
        "手机往右上挪，让板子落在画面左下角",
        "手机往上挪，让板子落在画面下方",
        "手机往左上挪，让板子落在画面右下角",
    )
    private val resBuf = FloatArray(11)

    // ---------------- 生命周期 ----------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_calib)
        applySystemBarAppearance()

        preview = findViewById(R.id.preview)
        tvStatus = findViewById(R.id.tvStatus)
        btnA = findViewById(R.id.btnA)
        btnB = findViewById(R.id.btnB)
        btnRefocus = findViewById(R.id.btnRefocus)
        patternWrap = findViewById(R.id.patternWrap)
        patternView = findViewById(R.id.patternView)
        patternView.cols = COLS
        patternView.rows = ROWS

        val ct = HandlerThread("camera").also { it.start() }
        camThread = ct
        cam = CameraCapture(this, Handler(ct.looper))
        cam.onFrame = ::onFrame

        btnA.setOnClickListener { onButtonA() }
        btnB.setOnClickListener { onButtonB() }
        btnRefocus.setOnClickListener { onRefocus() }
        findViewById<Button>(R.id.btnPattern).setOnClickListener { showPattern(true) }
        findViewById<Button>(R.id.btnClosePattern).setOnClickListener { showPattern(false) }
        findViewById<Button>(R.id.btnSharePattern).setOnClickListener { sharePattern() }

        render()
    }

    override fun onStop() {
        super.onStop()
        teardown()
    }

    override fun onDestroy() {
        super.onDestroy()
        teardown()
        cam.onFrame = null
        uiHandler.removeCallbacksAndMessages(null)
        camThread.quitSafely()
        calibWorker.shutdown()
    }

    // ---------------- 帧出口 ----------------

    /** 运行在相机线程上，不能阻塞。 */
    private fun onFrame(gray: ByteArray, tsNs: Long) {
        try {
            preview.offer(gray, tsNs)
        } catch (e: Throwable) {
            Log.e(TAG, "预览丢帧", e)
        }
        if (!capturing) return
        if (++frameSeq % DETECT_EVERY != 0) return
        // 上一帧还在检就直接丢：排队只会让延迟越拖越长。拷贝也一样，只在真要检时才做。
        if (!detecting.compareAndSet(false, true)) return
        val copy = gray.copyOf()
        calibWorker.execute {
            try {
                if (capturing && SlamBridge.nativeCalibAddFrame(copy) > 0) {
                    uiHandler.post { onViewAccepted() }
                }
            } finally {
                detecting.set(false)
            }
        }
    }

    private fun onViewAccepted() {
        if (!capturing || finished) return
        render()
        if (SlamBridge.nativeCalibViews() >= TARGET_VIEWS) finishCalibration()
    }

    // ---------------- 按钮 ----------------

    private fun onButtonA() {
        if (phase == Phase.IDLE) beginCamera() else teardown()
    }

    private fun onButtonB() {
        when {
            phase == Phase.CAPTURING -> finishCalibration()
            phase == Phase.ARMED && !focusLocked -> lockFocus()
            phase == Phase.ARMED && focusLocked -> startCapture()
        }
    }

    private fun beginCamera() {
        uiMessage = null
        if (!hasCameraPermission()) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), REQ_CAMERA)
            return
        }
        preview.setActive(true)
        phase = Phase.ARMED
        focusLocked = false
        lockedDiopter = null
        cam.pinnedDiopter = null   // 上一次锁过的位置不要带到这一次
        cam.start()
        // 这里只跑连续对焦，不自动锁：标定要人看着画面清楚才锁。
        cam.startContinuousFocus()
        render()
        uiHandler.removeCallbacks(ticker)
        uiHandler.post(ticker)
    }

    private fun lockFocus() {
        val d = cam.lockFocusNow()
        if (d == null) {
            uiMessage = "还读不到对焦距离，等一两秒再点"
            render()
            return
        }
        lockedDiopter = d
        focusLocked = true
        uiMessage = null
        render()
    }

    /** 锁错位置了：放开镜组回到连续对焦，重新对好再锁。 */
    private fun onRefocus() {
        cam.releaseFocus()
        focusLocked = false
        lockedDiopter = null
        uiMessage = null
        render()
    }

    private fun startCapture() {
        val d = lockedDiopter ?: return
        SlamBridge.nativeCalibStart(COLS, ROWS, CameraCapture.W, CameraCapture.H)
        finished = false
        frameSeq = 0
        capturing = true
        phase = Phase.CAPTURING
        uiMessage = "距离不变；挪手机让板子依次出现在四角，每位停 1 秒"
        Log.i(TAG, "开始采集：对焦 %.3f D".format(d))
        render()
    }

    private fun finishCalibration() {
        if (finished) return
        finished = true
        capturing = false
        // 采集结束就回到 ARMED：不管弹窗选什么，按钮都要能再点。
        phase = Phase.ARMED
        val ok = SlamBridge.nativeCalibFinish(resBuf)
        val cov = displayCoverage()
        val n = resBuf[9].toInt()
        val rms = resBuf[8]

        if (!ok) {
            retry("视角不够（$n 个），再扫一会儿")
            return
        }

        // 否决规则写在这里：App 里没人替你看输出，算得不对就只能拦住。
        // 门槛看外圈，不看九格 —— 九格被长宽比放大，中间那列天然偏高。
        val outer = covBuf[9]
        if (outer < OUTER_TARGET || rms > 1.0f || n < 20) {
            val why = buildString {
                if (outer < OUTER_TARGET) {
                    append("画面外圈只占到 %.0f%%（目标 %.0f%%）：板子还没到过画面边上"
                        .format(outer * 100, OUTER_TARGET * 100))
                    (0 until 9).minByOrNull { cov[it] }?.let {
                        append("。先把板子放到画面").append(cellNames[it])
                    }
                }
                if (rms > 1.0f) {
                    if (isNotEmpty()) append("；")
                    append("重投影误差 %.2f px 偏大".format(rms))
                    append("（多半是拍糊了或板子不平）")
                }
                if (n < 20) {
                    if (isNotEmpty()) append("；")
                    append("只有 $n 个视角")
                }
            }
            retry(why)
            return
        }

        val d = lockedDiopter ?: return
        val dropped = resBuf[10].toInt()
        MaterialAlertDialogBuilder(this)
            .setTitle("标定完成")
            .setMessage(
                "对焦 %.2f D（约 %.2f m）\nfx %.2f   fy %.2f\ncx %.2f   cy %.2f\n"
                    .format(d, 1f / d, resBuf[0], resBuf[1], resBuf[2], resBuf[3]) +
                    "k1 %.5f\n重投影误差 %.3f px · %d 个视角".format(resBuf[4], rms, n) +
                    if (dropped > 0) "\n（剔除了 $dropped 个偏离的视角）" else ""
            )
            .setPositiveButton("保存") { _, _ -> saveRow(d, n, rms) }
            .setNegativeButton("不要", null)
            .show()
    }

    private fun retry(why: String) {
        MaterialAlertDialogBuilder(this)
            .setTitle("这次不行")
            .setMessage("$why\n\n重来一次？")
            .setPositiveButton("重来") { _, _ ->
                SlamBridge.nativeCalibReset()
                startCapture()
            }
            .setNegativeButton("先这样", null)
            .show()
    }

    private fun saveRow(d: Float, views: Int, rms: Float) {
        val info = cam.cameraInfo()
        val old = CalibStore.load(this)
        val prof = old ?: CalibProfile(
            model = Build.MODEL,
            cameraId = info?.first ?: "",
            sensorOrientation = info?.second ?: 0,
            imuFrequency = 0f,
            rows = emptyList(),
        )
        val row = CalibRow(
            d = d,
            fx = resBuf[0], fy = resBuf[1], cx = resBuf[2], cy = resBuf[3],
            k1 = resBuf[4], k2 = resBuf[5], p1 = resBuf[6], p2 = resBuf[7],
            source = "auto", rms = rms, views = views,
            date = SimpleDateFormat("yyyy-MM-dd", Locale.US).format(Date()),
        )
        // 同一档（±0.15 D）复标就覆盖，不另起一行。
        val rows = (prof.rows.filter { abs(it.d - d) > 0.15f } + row).sortedBy { it.d }
        if (CalibStore.save(this, prof.copy(rows = rows))) {
            Log.i(TAG, "已保存档位 %.3f D fx=%.3f".format(d, row.fx))
            finish()
        } else {
            uiMessage = "保存失败，看日志"
            render()
        }
    }

    // ---------------- 状态 ----------------

    private val ticker = object : Runnable {
        override fun run() {
            if (phase == Phase.IDLE) return
            render()
            uiHandler.postDelayed(this, TICK_MS)
        }
    }

    /**
     * 把 native 的九宫格转成屏幕方位：行 = 画面上下，列 = 画面左右。
     *
     * native 的下标是 `gy*3+gx`，gx 是图像 x（预览旋转后对应屏幕上下），
     * gy 是图像 y（对应屏幕左右，且左右镜像）—— 直接打印出来会和屏幕差 90°。
     */
    private fun displayCoverage(): FloatArray {
        SlamBridge.nativeCalibCoverage(covBuf)
        for (r in 0 until 3) {
            for (c in 0 until 3) dispBuf[r * 3 + c] = covBuf[(2 - c) * 3 + r]
        }
        return dispBuf
    }

    private fun render() {
        btnA.text = if (phase == Phase.IDLE) "打开相机" else "关闭相机"
        btnB.text = when {
            phase == Phase.CAPTURING -> "停止并拟合"
            focusLocked -> "开始采集"
            else -> "锁定对焦"
        }
        btnB.isEnabled = phase != Phase.IDLE
        btnRefocus.visibility =
            if (phase == Phase.ARMED && focusLocked) View.VISIBLE else View.GONE

        val sb = StringBuilder()
        uiMessage?.let { sb.append(it).append("\n\n") }

        when (phase) {
            Phase.IDLE -> sb.append("点「打开相机」，对准标定板。")
            Phase.ARMED -> {
                if (focusLocked) {
                    sb.append("● 对焦已锁定")
                    lockedDiopter?.let { d ->
                        sb.append("  %.2f D（约 %.2f m）".format(d, 1f / d))
                    }
                    sb.append("\n板子整块入镜，再点「开始采集」。")
                } else {
                    sb.append("… 对准板子后点「锁定对焦」")
                    // 这个数字一直在跳就说明连续对焦在工作。
                    cam.latestFocusDistance()?.let { d ->
                        sb.append("\n实时对焦 %.2f D（约 %.2f m）".format(d, 1f / d))
                    }
                    sb.append("\n等画面清楚了再锁。")
                }
            }
            Phase.CAPTURING -> {
                val n = SlamBridge.nativeCalibViews()
                sb.append("视角 $n/$TARGET_VIEWS")
                lockedDiopter?.let { sb.append("   对焦 %.2f D".format(it)) }
                val cov = displayCoverage()
                val outer = covBuf[9]
                sb.append("\n外圈 %.0f%%（目标 ≥%.0f%%）"
                    .format(outer * 100, OUTER_TARGET * 100))
                sb.append("\n九格参考")   // 长宽比决定中间列天然偏高，不拿它当门槛
                for (r in 0 until 3) {
                    sb.append("\n     ")
                    for (c in 0 until 3) sb.append("%3.0f%% ".format(cov[r * 3 + c] * 100))
                }
                sb.append('\n')
                if (outer >= OUTER_TARGET) {
                    sb.append("外圈够了，接着拍")
                } else {
                    // 一次只提示一处，否则不知道先动哪个。
                    sb.append("下一步：").append(cellHints[(0 until 9).minByOrNull { cov[it] }!!])
                }
            }
        }
        tvStatus.text = sb.toString()
    }

    // ---------------- 标定板 ----------------

    private fun showPattern(on: Boolean) {
        patternWrap.visibility = if (on) View.VISIBLE else View.GONE
    }

    private fun sharePattern() {
        val dir = File(getExternalFilesDir(null) ?: filesDir, "share").apply { mkdirs() }
        val f = File(dir, "chessboard-%dx%d.png".format(COLS, ROWS))
        try {
            f.outputStream().use {
                patternView.render(2000).compress(Bitmap.CompressFormat.PNG, 100, it)
            }
        } catch (e: Exception) {
            Log.e(TAG, "生成标定板失败", e)
            uiMessage = "生成标定板失败：${e.javaClass.simpleName}"
            render()
            return
        }
        val uri = FileProvider.getUriForFile(this, "$packageName.share", f)
        val send = Intent(Intent.ACTION_SEND).apply {
            type = "image/png"
            putExtra(Intent.EXTRA_STREAM, uri)
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
        startActivity(Intent.createChooser(send, "分享标定板（去打印，或发到另一块屏幕）"))
    }

    // ---------------- 拆除 ----------------

    private fun teardown() {
        if (phase == Phase.IDLE) return
        capturing = false
        finished = false
        focusLocked = false
        lockedDiopter = null
        phase = Phase.IDLE
        uiHandler.removeCallbacks(ticker)
        preview.setActive(false)
        cam.stop()
        showPattern(false)
        render()
    }

    // ---------------- 权限 ----------------

    private fun hasCameraPermission() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != REQ_CAMERA) return
        if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            beginCamera()
        } else {
            uiMessage = "相机权限被拒绝。\n请到 设置 → 应用 → SlamCapture → 权限 里开启。"
            render()
        }
    }
}
