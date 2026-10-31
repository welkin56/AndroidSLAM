package com.example.slamcapture

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PointF
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
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
 * 标定页：分步引导采集。
 *
 * 画面上画一个虚线四边形，形状随步骤变（正对是矩形，倾斜是梯形）；摆成那个形状后自动采
 * 3 帧、进下一步，共 9 步。倾斜步负责约束焦距 —— 正对时焦距与物距在单应里退化。
 *
 * 视角全在 native 内存里。一档 = 一个对焦屈光度 = 一行 [CalibRow]。
 */
class CalibActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "CalibActivity"

        /** 内角点数：A4 上是 7×5 个方格、每格 30 mm。 */
        private const val COLS = CalibGuide.COLS
        private const val ROWS = CalibGuide.ROWS

        /** 每 5 帧检一次：SB 检测器在 640×480 上要几十毫秒，不能每帧都做。 */
        private const val DETECT_EVERY = 5

        /** 连续这么多次「对上」才提交，摊掉单帧抖动。 */
        private const val HOLD_FRAMES = 3

        /** 同一步采几帧，必须和 calib.cpp 的 kPerStep 一致。 */
        private const val PER_STEP = 3

        /** 至少这么多视角才拟合，必须和 calib.cpp 的 kMinViews 一致。 */
        private const val MIN_VIEWS = 12

        private const val TICK_MS = 300L
        private const val REQ_CAMERA = 1

        /** 某一步卡住这么久就开始提示，多半是环境有问题。 */
        private const val STEP_TIMEOUT_MS = 45_000L

        /** 画面外圈（最外 30%）至少占到的角点比例。9 步走完约为 0.17。 */
        private const val OUTER_TARGET = 0.08f

        /** fx / fy 相对标准差超过它，说明焦距没被定住，这组数不能用来测量。 */
        private const val SD_MAX = 0.03f

        /**
         * 标定板的实际纸面尺寸（米）。默认 A4 满幅：7×5 个方格、每格 30 mm。
         *
         * 内参精度取决于每格占多少像素，与厘米数无关 —— 用显示器全屏显示时，
         * 量一下屏幕上的实际宽高填这里，就能在更远处标定。
         */
        private const val BOARD_LONG_M = 0.210f
        private const val BOARD_SHORT_M = 0.150f
    }

    private enum class Phase { IDLE, ARMED, GUIDING, FITTING }

    /** 当前步的目标：步号 + 目标点阵四边形（图像坐标，8 个 float）。整体替换引用。 */
    private class Target(val step: Int, val lattice: FloatArray)

    /** 给绘制用的一帧快照。worker 每次新建，主线程只读。 */
    private class GuideFrame(
        val quad: FloatArray?,
        val posPx: Float,
        val sizeErr: Float,
        val tiltErr: Float,
        val pass: Boolean,
        /** 连续对上几次（未提交的）。 */
        val hold: Int,
        /** 本步已提交的帧数。 */
        val commits: Int,
    )

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

    /** 换步时整体替换。worker 靠它拿目标框，绘制靠它算形状。 */
    @Volatile
    private var target: Target? = null

    /** 最近一次 probe 的结果，供绘制与状态行。 */
    @Volatile
    private var guide: GuideFrame? = null

    /** 本步开始的时刻，用来判断「卡住了」。 */
    private var stepStartMs = 0L

    // ---- 以下只在 calibWorker 上跑；[startCapture]/[teardown] 在 capturing=false 之后复位 ----
    /** 连续对上几次。 */
    @Volatile
    private var hold = 0
    /** [hold] 属于哪一步；换步就清零。 */
    @Volatile
    private var holdStep = -1
    /** 本步已提交的帧数；到 [PER_STEP] 才换步。 */
    private var stepCommits = 0
    private val detBuf = FloatArray(8)
    private val errBuf = CalibGuide.Err()

    private var focusLocked = false
    private var lockedDiopter: Float? = null
    private var frameSeq = 0
    private var finished = false

    private var uiMessage: String? = null
    /** 前 9 个是九宫格，第 10 个是画面外圈占比。只用于被拒时的说明和日志。 */
    private val covBuf = FloatArray(10)
    private val resBuf = FloatArray(13)

    // ---- 引导层绘制 ----
    // 全部成员化：drawGuide 在 onDraw 里跑，每帧 new 会 GC 抖动。
    // Paint 只能等 onCreate 之后建 —— 字段初始化时 resources 还是 null。
    private var density = 1f
    private lateinit var gpPaper: Paint
    private lateinit var gpLattice: Paint
    private lateinit var gpDot: Paint
    private lateinit var gpCorner: Paint
    private lateinit var gpLead: Paint
    private lateinit var gpDetOk: Paint
    private lateinit var gpDetBad: Paint
    private lateinit var gpProgOn: Paint
    private lateinit var gpProgOff: Paint
    private val gPath = Path()
    private val gPt = PointF()
    private val gPt2 = PointF()
    private val gPaper = FloatArray(8)
    private val gLat = FloatArray(8)
    private val gDots = FloatArray(CalibGuide.COLS * CalibGuide.ROWS * 2)
    /** 上面三个数组是按哪一步算的；-1 表示还没算。 */
    private var gStep = -1

    /**
     * 引导层的画笔。颜色用字面量而不是 `?attr/`：这一层画在相机画面上，
     * 必须与画面强对比，不能跟主题深浅变。
     */
    private fun initGuidePaints() {
        density = resources.displayMetrics.density
        gpPaper = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = 2f * density
            color = 0xCCFFFFFF.toInt()
            pathEffect = DashPathEffect(floatArrayOf(11f * density, 7f * density), 0f)
        }
        gpLattice = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = 1.5f * density
            color = 0x55FFFFFF
            pathEffect = DashPathEffect(floatArrayOf(3f * density, 6f * density), 0f)
        }
        gpDot = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
            color = 0x88FFFFFF.toInt()
        }
        gpCorner = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
            color = 0xFFFFFFFF.toInt()
        }
        gpLead = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = 1f * density
            color = 0xFFFFC107.toInt()   // 琥珀，和测量侧的吸附圈同一个色
            pathEffect = DashPathEffect(floatArrayOf(8f * density, 6f * density), 0f)
        }
        gpDetOk = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = 2f * density
            color = 0xFF00E676.toInt()
        }
        gpDetBad = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = 2f * density
            color = 0xFF00E5FF.toInt()   // 和测量折线同一个青
        }
        gpProgOn = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
            color = 0xFF00E676.toInt()
        }
        gpProgOff = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
            color = 0x66FFFFFF
        }
    }

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

        // 画笔要等 attachBaseContext 之后才能建。
        initGuidePaints()
        CalibGuide.setBoard(BOARD_LONG_M, BOARD_SHORT_M)
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
        val t = target ?: return
        // 上一帧还在检就直接丢，排队只会让延迟越拖越长。
        if (!detecting.compareAndSet(false, true)) return
        val copy = gray.copyOf()
        calibWorker.execute {
            var f: GuideFrame? = null
            var advanced = false
            try {
                if (capturing) {
                    val r = processFrame(copy, t)
                    f = r.first
                    advanced = r.second
                }
            } catch (e: Throwable) {
                Log.e(TAG, "probe 异常", e)
            } finally {
                detecting.set(false)
            }
            val frame = f
            uiHandler.post { onProbeResult(frame, advanced) }
        }
    }

    /**
     * 只在 [calibWorker] 上跑：probe → 比对 → 够格就 commit。
     *
     * probe 和 commit 必须同线程 —— native 只保留最近一次 probe 的角点。
     */
    private fun processFrame(copy: ByteArray, t: Target): Pair<GuideFrame?, Boolean> {
        if (t.step != holdStep) {
            holdStep = t.step
            hold = 0
            stepCommits = 0
        }
        if (SlamBridge.nativeCalibProbe(copy, detBuf) <= 0) {
            hold = 0
            return GuideFrame(null, 0f, 0f, 0f, false, 0, stepCommits) to false
        }

        CalibGuide.measure(detBuf, t.lattice, errBuf)
        // 第一帧严格，连中之后放宽，否则手抖会一直在快满那里反复。
        val relax = if (hold > 0) 1.5f else 1f
        val pass = errBuf.pass(relax, CalibGuide.STEPS[t.step])

        var committed = false
        if (pass && stepCommits < PER_STEP) {
            hold++
            if (hold >= HOLD_FRAMES && SlamBridge.nativeCalibCommit(t.step) > 0) {
                stepCommits++
                hold = 0   // 下一帧重新数满 HOLD_FRAMES，避免连拍同一瞬间
                committed = true
            }
        } else if (!pass) {
            hold = 0
        }

        // 本步收满才换步；收不满就继续等，不再一次提交就跳过一整步。
        val done = committed && stepCommits >= PER_STEP
        return Pair(
            GuideFrame(detBuf.clone(), errBuf.posPx, errBuf.sizeErr, errBuf.tiltErr,
                pass, hold, stepCommits),
            done,
        )
    }

    /** 主线程：更新引导层，必要时换步或收尾。 */
    private fun onProbeResult(f: GuideFrame?, advanced: Boolean) {
        if (phase != Phase.GUIDING) return
        guide = f
        preview.invalidate()
        if (advanced) {
            val next = (target?.step ?: 0) + 1
            if (next >= CalibGuide.STEPS.size) finishCalibration() else publishStep(next)
        }
        render()
    }

    /** 换到第 [k] 步：算目标框、重置计时。进度清零由 worker 发现步号变了自己做。 */
    private fun publishStep(k: Int) {
        val step = CalibGuide.STEPS[k]
        target = Target(k, CalibGuide.latticeQuad(step))
        guide = null
        stepStartMs = SystemClock.elapsedRealtime()
        preview.invalidate()
        render()
    }

    // ---------------- 按钮 ----------------

    private fun onButtonA() {
        when (phase) {
            Phase.IDLE -> beginCamera()
            Phase.FITTING -> Unit   // 拟合中相机已关，别在这里又开起来
            else -> teardown()
        }
    }

    private fun onButtonB() {
        when {
            phase == Phase.FITTING -> Unit
            phase == Phase.GUIDING -> askAbort()
            phase == Phase.ARMED && !focusLocked -> lockFocus()
            phase == Phase.ARMED && focusLocked -> startCapture()
        }
    }

    /**
     * 开相机。[pin] 非空时直接把镜组命令到那个屈光度（拟合失败后重来用），
     * 不再走连续对焦 —— 否则镜组会飘到别处，重来的第一次标定就换了档。
     */
    private fun beginCamera(pin: Float? = null) {
        uiMessage = null
        if (!hasCameraPermission()) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), REQ_CAMERA)
            return
        }
        preview.setActive(true)
        phase = Phase.ARMED
        focusLocked = false
        lockedDiopter = null
        // 必须在 start() 之前设：start() 会按它下发第一帧的对焦命令。
        cam.pinnedDiopter = pin
        cam.start()
        if (pin == null) {
            // 这里只跑连续对焦，不自动锁：标定要人看着画面清楚才锁。
            cam.startContinuousFocus()
        }
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
        hold = 0
        holdStep = -1
        stepCommits = 0
        gStep = -1
        capturing = true
        phase = Phase.GUIDING
        uiMessage = null
        preview.guidePainter = ::drawGuide
        // 框的尺寸固定（板长 200 px），所以标定物距和锁焦距离是两个独立的数。
        Log.i(TAG, "开始引导采集：对焦 %.3f D（%.2f m）· 目标框物距 %.3f m · 失焦 %.2f D"
            .format(d, 1f / d, CalibGuide.workingZ, CalibGuide.defocusD(d)))
        logSteps()
        publishStep(0)
    }

    /** 把 9 步的目标四边形打进日志，作为倾角符号的回归基线。 */
    private fun logSteps() {
        val q = FloatArray(8)
        for (i in CalibGuide.STEPS.indices) {
            val s = CalibGuide.STEPS[i]
            CalibGuide.quad(s, true, q)
            Log.i(TAG, "步骤%d %s u[%.0f..%.0f] v[%.0f..%.0f] 角点 %s".format(
                i + 1, s.title,
                minOf(q[0], q[2], q[4], q[6]), maxOf(q[0], q[2], q[4], q[6]),
                minOf(q[1], q[3], q[5], q[7]), maxOf(q[1], q[3], q[5], q[7]),
                (0 until 8).joinToString(" ") { "%.0f".format(q[it]) }
            ))
        }
    }

    /**
     * 采集结束：拟合放到 worker 上跑。
     *
     * `finish()` 持锁几百毫秒，所以要在调用前把 phase 置成 ARMED，
     * 否则 ticker 的 render() 会被那把锁挡住。
     */
    private fun finishCalibration() {
        if (finished) return
        finished = true
        capturing = false
        // 先关相机再拟合：拟合要几百毫秒且不需要画面，开着相机只是白占带宽和 CPU。
        // 不用 setActive(false) —— 留着最后一帧冻在屏幕上，比黑屏好认。
        cam.stop()
        preview.guidePainter = null
        target = null
        guide = null
        phase = Phase.FITTING
        uiMessage = "正在拟合 %d 个视角…".format(SlamBridge.nativeCalibViews())
        render()
        calibWorker.execute {
            val ok = SlamBridge.nativeCalibFinish(resBuf)
            SlamBridge.nativeCalibCoverage(covBuf)
            uiHandler.post { showFitResult(ok) }
        }
    }

    private fun showFitResult(ok: Boolean) {
        // 拟合期间退出（onStop 拆了相机）就不再弹窗。
        if (phase != Phase.FITTING) return
        uiMessage = null
        phase = Phase.IDLE   // 相机已经关了，回到未打开状态
        render()
        val n = resBuf[9].toInt()
        val rms = resBuf[8]
        val outer = covBuf[9]

        if (!ok) {
            Log.w(TAG, "拟合失败：视角 $n 个")
            val why = if (n < MIN_VIEWS) "只采到 $n 个视角（至少要 $MIN_VIEWS 个），再走一遍"
            else "拟合失败（$n 个视角），多半是板子不平或拍糊了"
            retry(why)
            return
        }

        // App 里没人替你看输出，算得不对只能拦住。
        // 视角数由 MIN_VIEWS（拟合门槛）和 fx 标准差把关，不再另设一个更高的数 ——
        // 两个门槛打架时会出现「拟合成功却存不进去」。
        if (outer < OUTER_TARGET || rms > 1.0f) {
            val why = buildString {
                if (rms > 1.0f) {
                    append("重投影误差 %.2f px 偏大".format(rms))
                    append("（多半是拍糊了或板子不平）")
                }
                if (outer < OUTER_TARGET) {
                    if (isNotEmpty()) append("；")
                    append("画面外圈只占 %.0f%%（目标 %.0f%%）"
                        .format(outer * 100, OUTER_TARGET * 100))
                }
            }
            retry(why)
            return
        }

        val d = lockedDiopter ?: return
        val dropped = resBuf[10].toInt()
        val sdMax = maxOf(resBuf[11], resBuf[12])
        val shaky = sdMax > SD_MAX

        // 是覆盖还是新增要说清，不然会以为「同一档又标一遍应该没变化」。
        val prof = CalibStore.load(this)
        val sameD = prof?.rows?.firstOrNull { abs(it.d - d) <= SAME_D_EPS }
        val afterCount = (prof?.rows?.size ?: 0) + (if (sameD != null) 0 else 1)

        MaterialAlertDialogBuilder(this)
            .setTitle(if (shaky) "标定完成（焦距不稳）" else "标定完成")
            .setMessage(
                "对焦 %.2f D（约 %.2f m）\nfx %.2f   fy %.2f\ncx %.2f   cy %.2f\n"
                    .format(d, 1f / d, resBuf[0], resBuf[1], resBuf[2], resBuf[3]) +
                    "k1 %.5f\n重投影误差 %.3f px · %d 个视角".format(resBuf[4], rms, n) +
                    "\n板子每格 %.0f px  ·  fx 标准差 %.2f%% / fy %.2f%%"
                        .format(CalibGuide.squarePx(), resBuf[11] * 100, resBuf[12] * 100) +
                    (if (dropped > 0) "\n（剔除了 $dropped 个偏离的视角）" else "") +
                    (
                        if (sameD != null) "\n（覆盖 %.2f D 那一档，共 %d 个档位）"
                            .format(sameD.d, afterCount)
                        else "\n（新增一档，共 %d 个档位）".format(afterCount)
                        ) +
                    (
                        if (shaky) "\n\n⚠ 焦距没被定住（标准差 %.1f%%）。这组数测出来会系统性偏，" +
                            "建议重标 —— 第 6~9 步要真的斜着看板子，或在更近处标。" .format(sdMax * 100)
                        else ""
                        )
            )
            .setPositiveButton("保存") { _, _ -> saveRow(d, n, rms) }
            .setNegativeButton("不要", null)
            .show()
    }

    /** 采集途中放弃：视角全部作废，回到「开始采集」之前。 */
    private fun askAbort() {
        MaterialAlertDialogBuilder(this)
            .setTitle("放弃这次采集？")
            .setMessage("已经采了 %d 个视角，放弃后要从第 1 步重走。"
                .format(SlamBridge.nativeCalibViews()))
            .setPositiveButton("放弃") { _, _ -> abortCapture() }
            .setNegativeButton("继续", null)
            .show()
    }

    private fun abortCapture() {
        capturing = false
        finished = false
        SlamBridge.nativeCalibReset()
        preview.guidePainter = null
        target = null
        guide = null
        hold = 0
        holdStep = -1
        stepCommits = 0
        phase = Phase.ARMED
        uiMessage = "已放弃，重新点「开始采集」"
        render()
    }

    /** 拟合失败后重来：相机在拟合前已经关了，先开回来并回到同一个屈光度。 */
    private fun retry(why: String) {
        MaterialAlertDialogBuilder(this)
            .setTitle("这次不行")
            .setMessage("$why\n\n重来一次？")
            .setPositiveButton("重来") { _, _ ->
                val d = lockedDiopter
                SlamBridge.nativeCalibReset()
                beginCamera(d)
                if (d != null && phase == Phase.ARMED) {
                    lockedDiopter = d
                    focusLocked = true
                    startCapture()
                } else {
                    uiMessage = "相机已重开，对准板子重新锁焦后再点「开始采集」"
                    render()
                }
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
            // 到分钟：档位列表按最近时间排序，只到天时同一天两次排不出先后。
            // 老的 yyyy-MM-dd 仍能参与字符串排序（视为当天 00:00），不用迁移。
            date = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US).format(Date()),
        )
        // 只有屈光度完全相同才覆盖；±0.15 D 那档会把重复标定的样本压成一行。
        val rows = (prof.rows.filter { abs(it.d - d) > SAME_D_EPS } + row).sortedBy { it.d }
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
            if (phase == Phase.IDLE || phase == Phase.FITTING) return
            render()
            uiHandler.postDelayed(this, TICK_MS)
        }
    }

    private fun render() {
        val busy = phase == Phase.FITTING
        // 拟合前相机就被关掉了，所以那时按钮显示「打开相机」且不可点。
        btnA.text = if (phase == Phase.IDLE || busy) "打开相机" else "关闭相机"
        btnA.isEnabled = !busy
        btnB.text = when {
            busy -> "拟合中…"
            phase == Phase.GUIDING -> "放弃采集"
            focusLocked -> "开始采集"
            else -> "锁定对焦"
        }
        btnB.isEnabled = phase != Phase.IDLE && !busy
        btnRefocus.visibility =
            if (phase == Phase.ARMED && focusLocked) View.VISIBLE else View.GONE

        val sb = StringBuilder()
        uiMessage?.let { sb.append(it).append("\n\n") }

        when (phase) {
            Phase.IDLE -> sb.append("点「打开相机」，对准标定板。")
            Phase.FITTING -> sb.append("相机已关闭。")
            Phase.ARMED -> {
                val d = lockedDiopter
                if (focusLocked && d != null) {
                    sb.append("● 对焦已锁定  %.2f D（约 %.2f m）".format(d, 1f / d))
                    sb.append("\n点「开始采集」后把板子挪进白框（约 %.2f m 处），对焦不用再管。"
                        .format(CalibGuide.workingZ))
                    val df = CalibGuide.defocusD(d)
                    if (df > 2f) {
                        sb.append("\n⚠ 板子离焦 %.1f D（约 %.0f px 模糊）：能标，角点会差一点。"
                            .format(df, df * 1.5f))
                    }
                    sb.append("\n之后只挪位置和角度，按白框走完 ")
                    sb.append(CalibGuide.STEPS.size).append(" 步。")
                } else {
                    sb.append("… 先对准板子点「锁定对焦」—— 锁在你实际测量的距离上")
                    // 这个数字一直在跳就说明连续对焦在工作。
                    cam.latestFocusDistance()?.let { d ->
                        sb.append("\n实时对焦 %.2f D（约 %.2f m）".format(d, 1f / d))
                    }
                }
            }
            Phase.GUIDING -> renderGuiding(sb)
        }
        tvStatus.text = sb.toString()
    }

    /** 引导中的四行：第几步 / 该做什么 / 现在差在哪 / 进度。 */
    private fun renderGuiding(sb: StringBuilder) {
        val t = target
        if (t == null) {
            sb.append("准备中…")
            return
        }
        val g = guide
        val st = CalibGuide.STEPS[t.step]

        sb.append("第 ").append(t.step + 1).append('/').append(CalibGuide.STEPS.size)
            .append(" 步 · ").append(st.title).append('\n')
        sb.append(st.hint).append('\n')
        sb.append(statusLine(g, t.step)).append('\n')

        sb.append("已采 %d/%d · 本步 %d/%d"
            .format(SlamBridge.nativeCalibViews(),
                CalibGuide.STEPS.size * PER_STEP,
                g?.commits ?: 0, PER_STEP))
        lockedDiopter?.let { sb.append(" · 对焦 %.2f D".format(it)) }
    }

    /**
     * 状态行：一次只说一件事，否则不知道先修哪个。
     * 顺序 = 看不看得到 → 大小 → 位置 → 形状（只在倾斜步）。
     */
    private fun statusLine(g: GuideFrame?, stepIndex: Int): String {
        if (g?.quad == null) return "放板子：整块放进白框，别挡、别逆光"
        if (g.hold > 0) {
            return if (g.hold >= HOLD_FRAMES) "已采满，换下一步"
            else "稳住了，还差 ${HOLD_FRAMES - g.hold} 帧"
        }
        if (g.pass) return "对上了，别动"

        val step = CalibGuide.STEPS[stepIndex]
        val base = when {
            g.sizeErr > CalibGuide.SIZE_TOL -> "大小不对：把板子拿近或拿远一点（别改对焦）"
            g.posPx > CalibGuide.POS_TOL -> "位置没对上：把绿框套进白框"
            step.checkTilt && g.tiltErr > step.tiltTol -> "形状不对：按上面那句把板子转一下"
            else -> "微调中…"
        }
        val waited = SystemClock.elapsedRealtime() - stepStartMs
        return if (waited > STEP_TIMEOUT_MS) {
            "$base（已停 %.0f 秒：位置 %.0f px%s）".format(
                waited / 1000f, g.posPx,
                if (step.checkTilt) "、形状 %.3f / 目标 %.2f".format(g.tiltErr, step.tiltTol) else "")
        } else base
    }

    // ---------------- 引导层 ----------------

    /**
     * 画在预览上：目标框（白虚线，形状随步骤变）+ 实测框（青/绿）+ 进度点。
     *
     * 只准用 [PreviewView.imageToScreen]，不自己算 letterbox —— 重算一次就和画面错位。
     */
    private fun drawGuide(c: Canvas) {
        val t = target ?: return
        if (preview.width <= 0) return
        val st = CalibGuide.STEPS[t.step]

        // 形状只在换步时算一次。
        if (gStep != t.step) {
            CalibGuide.quad(st, false, gPaper)
            CalibGuide.quad(st, true, gLat)
            CalibGuide.latticePoints(st, gDots)
            gStep = t.step
        }

        // 1) 纸面框：对齐纸张外沿
        drawQuad(c, gPaper, gpPaper)
        // 2) 点阵框：24 个内角点铺开的范围
        drawQuad(c, gLat, gpLattice)
        // 3) 24 个点：告诉用户角点该落在哪
        var i = 0
        while (i < gDots.size) {
            preview.imageToScreen(gDots[i], gDots[i + 1], gPt)
            c.drawCircle(gPt.x, gPt.y, 1.6f * density, gpDot)
            i += 2
        }
        // 4) 四个角标
        for (k in 0 until 4) {
            preview.imageToScreen(gPaper[k * 2], gPaper[k * 2 + 1], gPt)
            c.drawCircle(gPt.x, gPt.y, 3.5f * density, gpCorner)
        }

        val g = guide
        val q = g?.quad
        if (g != null && q != null) {
            // 5) 质心牵引线：位置偏得多时给个方向感
            if (g.posPx > CalibGuide.POS_TOL) {
                preview.imageToScreen(centroidX(q), centroidY(q), gPt)
                preview.imageToScreen(centroidX(gLat), centroidY(gLat), gPt2)
                c.drawLine(gPt.x, gPt.y, gPt2.x, gPt2.y, gpLead)
            }
            // 6) 实测框
            drawQuad(c, q, if (g.pass) gpDetOk else gpDetBad)
        }

        // 7) 进度：本步已采几个，画在纸面框上方
        var top = Float.MAX_VALUE
        var mid = 0f
        for (k in 0 until 4) {
            preview.imageToScreen(gPaper[k * 2], gPaper[k * 2 + 1], gPt)
            if (gPt.y < top) top = gPt.y
            mid += gPt.x * 0.25f
        }
        val y = (top - 14f * density).coerceAtLeast(12f * density)
        val doneCount = g?.commits ?: 0
        for (k in 0 until PER_STEP) {
            val x = mid + (k - (PER_STEP - 1) * 0.5f) * 14f * density
            c.drawCircle(x, y, 4f * density, if (k < doneCount) gpProgOn else gpProgOff)
        }
    }

    private fun drawQuad(c: Canvas, q: FloatArray, p: Paint) {
        gPath.reset()
        for (k in 0 until 4) {
            preview.imageToScreen(q[k * 2], q[k * 2 + 1], gPt)
            if (k == 0) gPath.moveTo(gPt.x, gPt.y) else gPath.lineTo(gPt.x, gPt.y)
        }
        gPath.close()
        c.drawPath(gPath, p)
    }

    private fun centroidX(q: FloatArray) = (q[0] + q[2] + q[4] + q[6]) * 0.25f
    private fun centroidY(q: FloatArray) = (q[1] + q[3] + q[5] + q[7]) * 0.25f

    // ---------------- 标定板 ----------------

    private fun showPattern(on: Boolean) {
        // 采集中打开会盖住引导层，看起来像卡住。
        if (on && phase == Phase.GUIDING) {
            uiMessage = "采集中不能看板子，会挡住引导框"
            render()
            return
        }
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
        val wasFitting = phase == Phase.FITTING
        capturing = false
        finished = false
        focusLocked = false
        lockedDiopter = null
        phase = Phase.IDLE
        // 引导层和待拟合的视角一起清掉。
        preview.guidePainter = null
        target = null
        guide = null
        hold = 0
        holdStep = -1
        stepCommits = 0
        gStep = -1
        // 拟合正持着 native 的锁，这里别去抢：视图留着没用，下一次 start() 会清。
        if (!wasFitting) SlamBridge.nativeCalibReset()
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
