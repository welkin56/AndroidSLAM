package com.example.slamcapture

import android.content.Context
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import android.view.WindowManager
import android.widget.Button
import android.widget.TextView
import androidx.activity.OnBackPressedCallback
import androidx.appcompat.app.AppCompatActivity
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.android.material.progressindicator.LinearProgressIndicator
import java.util.Locale
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * IMU 噪声标定：静置录一段，算 Allan 方差，得到四个噪声密度。
 *
 * 为什么值得单独一页：这是唯一还依赖 PC 的标定项（以前跑 tools/calibration/allan_variance.py）。
 * 流程和相机标定没有交集——不开相机、要静置二十分钟、需要一个计时器，所以不塞进 CalibActivity。
 *
 * 状态机：
 *
 *     IDLE --[开始采集]--> RECORDING --[完成并计算]--> COMPUTING --> REVIEW
 *       ^                     |                                        |
 *       +--[放弃]-------------+                        [保存并生效/放弃]--+
 *
 * 两个时间门槛：20 分钟白噪声就够（τ 上限 40 s，覆盖 -1/2 段），60 分钟才有机会
 * 进到游走项的 +1/2 段（τ 上限 120 s）。后者是经验值，最终仍以拟合斜率是否接近 +0.5 为准。
 */
class ImuCalibActivity : AppCompatActivity() {

    private enum class Phase { IDLE, RECORDING, COMPUTING, REVIEW }

    companion object {
        private const val TAG = "ImuCalib"

        private const val TICK_MS = 250L

        /** 脚本的硬性下限，不到它连算都不算。 */
        private const val MIN_SAMPLES = 10_000

        /** 脚本的警告线：10 分钟以下白噪声也偏糙。低于它只给「仍然保存」。 */
        private const val MIN_USABLE_SEC = 600.0

        /** 白噪声的建议下限与游走项的经验门槛。 */
        private const val RECOMMEND_SEC = 1200.0
        private const val WALK_SEC = 3600.0

        /** 游走项写入要求：斜率与目标 +0.5 的偏离。 */
        private const val WALK_SLOPE_TOL = 0.15

        /** 累积上限，约 75 分钟 @400 Hz。到顶就停止累积，不让 Java heap 被撑爆。 */
        private const val MAX_SAMPLES = 1_800_000

        private const val CH = 6

        /** 静置判据的窗宽：约 1 秒 @400 Hz。200 Hz 的机器上是 2 秒，判据只会更保守。 */
        private const val STILL_WIN = 400
        private const val GYRO_MEAN_MAX = 0.02f   // rad/s ≈ 1.15 °/s
        private const val GYRO_STD_MAX = 0.01f
        private const val ACC_STD_MAX = 0.05f     // m/s²
        private const val ACC_MEAN_TOL = 0.30f    // 与 9.81 的偏差

        /** 相邻样本间隔超过它就算一次中断。名义 2.5 ms 的十倍。 */
        private const val GAP_NS = 25_000_000L
    }

    // ---------------- 视图 ----------------

    private lateinit var tvGuide: TextView
    private lateinit var tvStatus: TextView
    private lateinit var progress: LinearProgressIndicator
    private lateinit var btnPrimary: Button
    private lateinit var btnSecondary: Button

    // ---------------- 线程 ----------------

    private var imuThread: HandlerThread? = null
    private var imuHandler: Handler? = null
    private var imu: ImuCollector? = null

    /** 计算放单线程池：6 轴 × 50 个 τ 要几秒，不能占主线程。 */
    private val computeExec = Executors.newSingleThreadExecutor()
    private val computeDone = AtomicInteger(0)
    private val computeTotal = AtomicInteger(1)

    private val uiHandler = Handler(android.os.Looper.getMainLooper())

    // ---------------- 状态 ----------------

    private var phase = Phase.IDLE

    /**
     * 分块样本，每块 `BLOCK × 6` 个 float 交错（gyro x/y/z、acc x/y/z）。
     *
     * 用分块而不是一条连续大数组：连续数组每次扩容要同时持有新旧两份，60 分钟时峰值
     * 会到 67 MB（稳态 40 MB）；分块每加一块只分配 1.57 MB，没有那个双份峰值。
     */
    @Volatile
    private var blocks: Array<FloatArray> = emptyArray()

    /** 已分配的样本容量，等于 `blocks.size * BLOCK`。 */
    @Volatile
    private var cap = 0

    @Volatile
    private var count = 0

    @Volatile
    private var firstTs = 0L

    @Volatile
    private var lastTs = 0L

    /** 累积到上限后置真，界面提示停止。 */
    @Volatile
    private var capped = false

    // 静置统计：1 秒窗的环形缓冲。只存模长，两个通道各一份。
    private val winW = FloatArray(STILL_WIN)
    private val winA = FloatArray(STILL_WIN)
    private var winPos = 0
    private var winFill = 0

    @Volatile
    private var winTotal = 0

    @Volatile
    private var winStill = 0

    @Volatile
    private var gapCount = 0L

    @Volatile
    private var pairCount = 0L

    /** 上一次算完但还没保存的结果。 */
    private var computed: Computed? = null

    /** 回到 IDLE 时要保留的一句话（放弃、切后台等）。render 优先显示它。 */
    private var hint: String? = null

    private data class Computed(
        val res: AllanVariance.Result,
        val n: Int,
        val dt: Double,
        val durationS: Double,
        val stillRatio: Double,
        val gapRatio: Double,
    )

    // ---------------- 生命周期 ----------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_imu_calib)

        // 20~60 分钟静置，息屏就会走 onStop 把传感器拆掉，那一段白录。
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        tvGuide = findViewById(R.id.tvGuide)
        tvStatus = findViewById(R.id.tvStatus)
        progress = findViewById(R.id.progress)
        btnPrimary = findViewById(R.id.btnPrimary)
        btnSecondary = findViewById(R.id.btnSecondary)

        btnPrimary.setOnClickListener { onPrimary() }
        btnSecondary.setOnClickListener { onSecondary() }

        onBackPressedDispatcher.addCallback(this, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                when (phase) {
                    Phase.RECORDING -> confirmDiscard()
                    Phase.IDLE, Phase.REVIEW -> finish()
                    Phase.COMPUTING -> {
                        // 计算只有几秒，等它自己结束比中断更简单，也不会留下半个结果。
                        tvGuide.text = "正在计算，请稍候…"
                    }
                }
            }
        })

        render()
    }

    override fun onStop() {
        super.onStop()
        // 数据只在内存里，切后台就等于放弃。明说，别让用户以为还在录。
        if (phase == Phase.RECORDING) {
            stopSensors()
            resetData()
            phase = Phase.IDLE
            hint = "切到后台，这一段已放弃（数据只在内存里）。"
            render()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        uiHandler.removeCallbacksAndMessages(null)
        computeExec.shutdownNow()
        stopSensors()
    }

    // ---------------- 按钮 ----------------

    private fun onPrimary() {
        when (phase) {
            Phase.IDLE -> startRecording()
            Phase.RECORDING -> finishRecording()
            Phase.COMPUTING -> Unit
            Phase.REVIEW -> save()
        }
    }

    private fun onSecondary() {
        when (phase) {
            Phase.IDLE -> finish()
            Phase.RECORDING -> confirmDiscard()
            Phase.COMPUTING -> Unit
            Phase.REVIEW -> {
                computed = null
                resetData()
                phase = Phase.IDLE
                hint = "已放弃这次结果，没有写入。"
                render()
            }
        }
    }

    private fun confirmDiscard() {
        MaterialAlertDialogBuilder(this)
            .setTitle("放弃这一段？")
            .setMessage("已录 ${timeText(elapsedSec())}，数据只在内存里，放弃后无法恢复。")
            .setNegativeButton("继续录", null)
            .setPositiveButton("放弃") { _, _ ->
                stopSensors()
                resetData()
                phase = Phase.IDLE
                hint = "已放弃。"
                render()
            }
            .show()
    }

    // ---------------- 采集 ----------------

    private fun startRecording() {
        resetData()
        hint = null
        val t = HandlerThread("imu-calib").also { it.start() }
        imuThread = t
        val h = Handler(t.looper)
        imuHandler = h

        val c = ImuCollector(this)
        c.onSample = { ts, wx, wy, wz, ax, ay, az ->
            addSample(ts, wx, wy, wz, ax, ay, az)
        }
        imu = c
        c.start(h)

        phase = Phase.RECORDING
        tvGuide.text = "保持静置。建议 20 分钟；60 分钟以上才能标游走项。"
        render()
        uiHandler.postDelayed(ticker, TICK_MS)
    }

    /** 只接受 IMU 线程调用。不能阻塞：扩容和统计都只做算术。 */
    private fun addSample(ts: Long, wx: Float, wy: Float, wz: Float,
                          ax: Float, ay: Float, az: Float) {
        val n = count
        if (n >= MAX_SAMPLES) {
            capped = true
            return
        }
        if (n == cap) addBlock()

        if (n == 0) {
            firstTs = ts
        } else {
            pairCount++
            if (ts - lastTs > GAP_NS) gapCount++
        }
        lastTs = ts

        val b = blocks[n shr AllanVariance.BLOCK_SHIFT]
        val off = (n and (AllanVariance.BLOCK - 1)) * CH
        b[off] = wx
        b[off + 1] = wy
        b[off + 2] = wz
        b[off + 3] = ax
        b[off + 4] = ay
        b[off + 5] = az
        count = n + 1

        accumulateStill(wx, wy, wz, ax, ay, az)
    }

    /** 追加一块。整块换引用而不是原地扩容，主线程随时读到的都是一个完整数组。 */
    private fun addBlock() {
        blocks = blocks + FloatArray(AllanVariance.BLOCK * CH)
        cap += AllanVariance.BLOCK
    }

    /**
     * 1 秒窗的静置判据。用窗而不是单样本：单样本只看得到噪声，窗才看得见转动与抖动。
     * 只用来打分和提示，不裁剪数据——脚本就是裸 overlapping，裁了就和 PC 版不可比。
     */
    private fun accumulateStill(wx: Float, wy: Float, wz: Float,
                                ax: Float, ay: Float, az: Float) {
        winW[winPos] = sqrt(wx * wx + wy * wy + wz * wz)
        winA[winPos] = sqrt(ax * ax + ay * ay + az * az)
        winPos = (winPos + 1) % STILL_WIN
        if (winFill < STILL_WIN) {
            winFill++
            return
        }

        var sw = 0f
        var sa = 0f
        for (i in 0 until STILL_WIN) {
            sw += winW[i]
            sa += winA[i]
        }
        val mw = sw / STILL_WIN
        val ma = sa / STILL_WIN
        var vw = 0f
        var va = 0f
        for (i in 0 until STILL_WIN) {
            val dw = winW[i] - mw
            vw += dw * dw
            val da = winA[i] - ma
            va += da * da
        }
        val sdW = sqrt(vw / STILL_WIN)
        val sdA = sqrt(va / STILL_WIN)

        val still = mw < GYRO_MEAN_MAX && sdW < GYRO_STD_MAX &&
            sdA < ACC_STD_MAX && abs(ma - 9.81f) < ACC_MEAN_TOL
        winTotal++
        if (still) winStill++
    }

    private fun stopSensors() {
        val c = imu ?: return
        imu = null
        // 先断出口：之后到达的回调会在 ImuCollector 里直接 return。
        c.onSample = null
        // 再等一跳：unregisterListener 不保证在途回调已经返回，不等就会边算边长。
        val h = imuHandler
        if (h != null) {
            val latch = CountDownLatch(1)
            h.post { latch.countDown() }
            try {
                latch.await(500, TimeUnit.MILLISECONDS)
            } catch (e: InterruptedException) {
                Log.w(TAG, "等待 IMU 线程收尾被中断", e)
            }
        }
        c.stop()
        imuHandler = null
        imuThread?.quitSafely()
        imuThread = null
    }

    private fun resetData() {
        uiHandler.removeCallbacks(ticker)
        blocks = emptyArray()
        cap = 0
        count = 0
        firstTs = 0L
        lastTs = 0L
        capped = false
        winPos = 0
        winFill = 0
        winTotal = 0
        winStill = 0
        gapCount = 0L
        pairCount = 0L
    }

    // ---------------- 计算 ----------------

    private fun finishRecording() {
        if (phase != Phase.RECORDING) return
        val n = count
        if (n < MIN_SAMPLES) return

        phase = Phase.COMPUTING
        stopSensors()

        val dt = (lastTs - firstTs).toDouble() / 1e9 / (n - 1)
        if (!(dt > 0.0)) {
            // 时间戳没前进（传感器卡死或时钟异常），τ/dt 会溢出，直接判这一段作废。
            resetData()
            phase = Phase.IDLE
            hint = "样本时间戳没有前进，这一段无法计算。"
            render()
            return
        }
        val durationS = (lastTs - firstTs).toDouble() / 1e9
        val still = if (winTotal > 0) winStill.toDouble() / winTotal else 1.0
        val gap = if (pairCount > 0) gapCount.toDouble() / pairCount else 0.0
        val data = blocks

        computeDone.set(0)
        computeTotal.set(6 * 60)
        render()

        computeExec.execute {
            val res = try {
                AllanVariance.compute(data, n, dt) { done, total ->
                    computeDone.set(done)
                    computeTotal.set(total)
                }
            } catch (e: Exception) {
                Log.e(TAG, "Allan 计算失败", e)
                null
            }
            runOnUiThread {
                if (isFinishing || isDestroyed) return@runOnUiThread
                if (res == null) {
                    computed = null
                    phase = Phase.REVIEW
                    tvStatus.text = "计算失败，详见 logcat。"
                    render()
                    return@runOnUiThread
                }
                computed = Computed(res, n, dt, durationS, still, gap)
                phase = Phase.REVIEW
                logDetail(res, durationS)
                renderResult()
                render()
            }
        }
    }

    // ---------------- 渲染 ----------------

    private val ticker = object : Runnable {
        override fun run() {
            if (phase != Phase.RECORDING) return
            render()
            uiHandler.postDelayed(this, TICK_MS)
        }
    }

    private fun render() {
        when (phase) {
            Phase.IDLE -> {
                tvGuide.text = hint ?: "把手机平放在硬桌面，别垫软布、别扶着。"
                tvStatus.text = idLabel() +
                    "\n20 分钟可标白噪声；60 分钟以上才能标游走项。"
                progress.visibility = LinearProgressIndicator.GONE
                btnPrimary.text = "开始采集"
                btnPrimary.isEnabled = true
                btnSecondary.text = "返回"
                btnSecondary.isEnabled = true
            }

            Phase.RECORDING -> {
                val sec = elapsedSec()
                val n = count
                val rate = if (sec > 1.0) n / sec else 0.0
                val moving = winTotal > 0 && winStill.toDouble() / winTotal < 0.9
                tvGuide.text = if (capped) {
                    "已达累积上限，请点「完成并计算」。"
                } else if (moving) {
                    "检测到移动，请放回桌面。"
                } else {
                    "保持静置。建议 20 分钟；60 分钟以上才能标游走项。"
                }
                tvStatus.text = String.format(
                    Locale.US,
                    "%s\n建议 ≥ %s · 游走 ≥ %s\n\n样本 %,d @ %.1f Hz\n静置 %.1f%%   间隔异常 %.2f%%",
                    clock(sec), clock(RECOMMEND_SEC), clock(WALK_SEC),
                    n, rate, stillRatio() * 100.0, gapRatio() * 100.0,
                )
                progress.visibility = LinearProgressIndicator.GONE
                btnPrimary.text = if (n < MIN_SAMPLES) "样本不足（<10,000）" else "完成并计算"
                btnPrimary.isEnabled = n >= MIN_SAMPLES
                btnSecondary.text = "放弃"
                btnSecondary.isEnabled = true
            }

            Phase.COMPUTING -> {
                val done = computeDone.get()
                val total = computeTotal.get().coerceAtLeast(1)
                val axis = (done * 6 / total + 1).coerceIn(1, 6)
                val tau = done % 50
                tvGuide.text = "正在计算…"
                tvStatus.text = String.format(
                    Locale.US, "计算中  %d/6 轴 · τ %d/50", axis, tau,
                )
                progress.visibility = LinearProgressIndicator.VISIBLE
                btnPrimary.text = "计算中…"
                btnPrimary.isEnabled = false
                btnSecondary.text = "请稍候"
                btnSecondary.isEnabled = false
            }

            Phase.REVIEW -> {
                progress.visibility = LinearProgressIndicator.GONE
                val m = computed
                if (m == null) {
                    tvGuide.text = "标定失败，没有写入。"
                    btnPrimary.text = "标定失败"
                    btnPrimary.isEnabled = false
                    btnSecondary.text = "返回"
                    btnSecondary.isEnabled = true
                    return
                }
                if (tvStatus.text.isNullOrEmpty()) renderResult()
                val usable = m.durationS >= MIN_USABLE_SEC && m.res.noiseGyro.mean.isFinite() &&
                    m.res.noiseAcc.mean.isFinite() && m.res.noiseGyro.mean > 0.0 &&
                    m.res.noiseAcc.mean > 0.0
                btnPrimary.text = if (usable) "保存并生效" else "仍然保存"
                btnPrimary.isEnabled = m.res.noiseGyro.mean.isFinite() &&
                    m.res.noiseAcc.mean.isFinite()
                btnSecondary.text = "放弃"
                btnSecondary.isEnabled = true
            }
        }
    }

    /** REVIEW 的四行结果。只在算完时写一次，ticker 不覆盖它。 */
    private fun renderResult() {
        val m = computed ?: return
        val r = m.res
        val rate = if (m.dt > 0) 1.0 / m.dt else 0.0
        val walkOk = walkWritable(m)

        tvStatus.text = buildString {
            append(
                String.format(
                    Locale.US,
                    "%,d 条 @ %.1f Hz · %.1f min · 静置 %.1f%%\n\n",
                    m.n, rate, m.durationS / 60.0, m.stillRatio * 100.0,
                )
            )
            append(line("NoiseGyro", r.noiseGyro.mean, qualityLabel(r.noiseGyro, m.durationS, m.stillRatio)))
            append(line("NoiseAcc ", r.noiseAcc.mean, qualityLabel(r.noiseAcc, m.durationS, m.stillRatio)))
            append(line("GyroWalk ", r.gyroWalk.mean, if (walkOk) "本次实测" else "未到上升段 · 不写入"))
            append(line("AccWalk  ", r.accWalk.mean, if (walkOk) "本次实测" else "未到上升段 · 不写入"))
            append("\n")
            when {
                m.durationS < MIN_USABLE_SEC ->
                    append("时间过短，结果不建议使用。")
                m.durationS < RECOMMEND_SEC ->
                    append("已达脚本下限，建议录到 20 分钟以上。")
                walkOk ->
                    append("四项都可写：本次时长够，游走斜率也达标。")
                m.durationS < WALK_SEC ->
                    append(
                        String.format(
                            Locale.US,
                            "游走项要 60 分钟以上；本次 %.1f 分钟，曲线停在平顶段，不写入。",
                            m.durationS / 60.0,
                        )
                    )
                else ->
                    append("时长够但游走斜率没接近 +0.5，仍不写入（换算出的 K 取决于窗口位置）。")
            }
            if (m.gapRatio > 0.001) {
                append(String.format(Locale.US, "\n采样中断 %.2f%%，dt 可能失真。", m.gapRatio * 100.0))
            }
        }
        tvGuide.text = "确认后写入。游走项不满足条件时保留原值。"
    }

    private fun line(name: String, value: Double, label: String): String =
        String.format(Locale.US, "%-10s %.3e   ［%s］\n", name, value, label)

    /**
     * 每轴斜率打进 logcat。
     *
     * 界面上只有折算后的「好/一般/糙」，看不出移植对不对；六轴斜率是否都接近目标
     * 才是算法正确性最强的信号 —— 移植走样时不可能刚好都对上。
     */
    private fun logDetail(r: AllanVariance.Result, durationS: Double) {
        val ax = arrayOf("x", "y", "z")
        for (i in 0 until 3) {
            Log.i(
                TAG,
                String.format(
                    Locale.US, "gyro.%s noise C=%.4e slope=%+.3f | walk C=%.4e slope=%+.3f",
                    ax[i], r.noiseGyro.values[i], r.noiseGyro.slopes[i],
                    r.gyroWalk.values[i], r.gyroWalk.slopes[i],
                ),
            )
        }
        for (i in 0 until 3) {
            Log.i(
                TAG,
                String.format(
                    Locale.US, "acc.%s  noise C=%.4e slope=%+.3f | walk C=%.4e slope=%+.3f",
                    ax[i], r.noiseAcc.values[i], r.noiseAcc.slopes[i],
                    r.accWalk.values[i], r.accWalk.slopes[i],
                ),
            )
        }
        Log.i(
            TAG,
            String.format(
                Locale.US,
                "均值 gyro=%.4e acc=%.4e | 离散 %.1f%% / %.1f%% | τ %.0fms~%.1fs | %.1f min | 目标 -0.50 / +0.50",
                r.noiseGyro.mean, r.noiseAcc.mean,
                r.noiseGyro.spread * 100, r.noiseAcc.spread * 100,
                r.tauMin * 1e3, r.tauMax, durationS / 60.0,
            ),
        )
    }

    /** 四维里取最差的一档。斜率阈值由窗口跨度反推：3.16^δ−1 就是系数不确定度。 */
    private fun qualityLabel(fit: AllanVariance.AxisFit, durationS: Double, still: Double): String {
        if (!fit.mean.isFinite() || fit.mean <= 0.0) return "无效"
        var worst = 0   // 0=好 1=一般 2=糙

        val dev = fit.worstSlopeDev
        worst = maxOf(worst, when {
            !dev.isFinite() -> 2
            dev <= 0.05 -> 0
            dev <= 0.10 -> 1
            else -> 2
        })

        val spread = fit.spread
        worst = maxOf(worst, when {
            !spread.isFinite() -> 2
            spread <= 0.10 -> 0
            spread <= 0.25 -> 1
            else -> 2
        })

        worst = maxOf(worst, when {
            durationS >= RECOMMEND_SEC -> 0
            durationS >= MIN_USABLE_SEC -> 1
            else -> 2
        })

        worst = maxOf(worst, when {
            still >= 0.98 -> 0
            still >= 0.90 -> 1
            else -> 2
        })

        return when (worst) {
            0 -> "好"
            1 -> "一般"
            else -> "糙"
        }
    }

    /**
     * profile.json 里的 quality：两路白噪声取最差一档。
     *
     * 不能只按时长分档——时长够但轴间离散大、静置差，一样不可用。
     * 写进文件的值要与界面上显示的是同一条判据，否则以后读到会以为「rough 只是时间短」。
     */
    private fun overallQuality(m: Computed): String {
        fun rank(s: String) = when (s) {
            "好" -> 0
            "一般" -> 1
            else -> 2
        }
        val g = rank(qualityLabel(m.res.noiseGyro, m.durationS, m.stillRatio))
        val a = rank(qualityLabel(m.res.noiseAcc, m.durationS, m.stillRatio))
        return when (maxOf(g, a)) {
            0 -> "ok"
            1 -> "rough"
            else -> "bad"
        }
    }

    /** 游走项写入条件：时长够，且两个轴的斜率都接近 +0.5。 */    private fun walkWritable(m: Computed): Boolean {
        val r = m.res
        if (m.durationS < WALK_SEC) return false
        if (!r.gyroWalk.mean.isFinite() || !r.accWalk.mean.isFinite()) return false
        if (r.gyroWalk.mean <= 0.0 || r.accWalk.mean <= 0.0) return false
        return r.gyroWalk.worstSlopeDev.isFinite() && r.gyroWalk.worstSlopeDev <= WALK_SLOPE_TOL &&
            r.accWalk.worstSlopeDev.isFinite() && r.accWalk.worstSlopeDev <= WALK_SLOPE_TOL
    }

    // ---------------- 保存与注入 ----------------

    private fun save() {
        val m = computed ?: return
        val r = m.res
        if (!r.noiseGyro.mean.isFinite() || !r.noiseAcc.mean.isFinite()) return

        val walkOk = walkWritable(m)
        val tmpl = templateWalk()
        val old = CalibStore.load(this)
        val info = cameraInfo()

        // 采样率是这次录制的副产品，比 402 那个常数准。超出合理范围就不写，
        // 免得拿降级到 200 Hz 的数据把 yaml 带偏。
        val rate = if (m.dt > 0) 1.0 / m.dt else 0.0
        val freq = if (rate in 100.0..1000.0) rate.toFloat() else (old?.imuFrequency ?: 0f)

        val noise = ImuNoise(
            noiseGyro = r.noiseGyro.mean,
            noiseAcc = r.noiseAcc.mean,
            gyroWalk = if (walkOk) r.gyroWalk.mean else tmpl.first,
            accWalk = if (walkOk) r.accWalk.mean else tmpl.second,
            walkUpdated = walkOk,
            samples = m.n,
            durationS = m.durationS,
            rateHz = rate,
            stillRatio = m.stillRatio,
            gapRatio = m.gapRatio,
            quality = overallQuality(m),
            recorded = CalibStore.stamp(),
        )

        val base = old ?: CalibProfile(
            model = Build.MODEL,
            cameraId = info?.first ?: "",
            sensorOrientation = info?.second ?: 0,
            imuFrequency = 0f,
            rows = emptyList(),
        )
        old?.imuNoise?.let {
            Log.i(
                TAG,
                String.format(
                    Locale.US,
                    "覆盖旧噪声：ng %.3e→%.3e  na %.3e→%.3e  ngw %.3e→%.3e  naw %.3e→%.3e",
                    it.noiseGyro, noise.noiseGyro, it.noiseAcc, noise.noiseAcc,
                    it.gyroWalk, noise.gyroWalk, it.accWalk, noise.accWalk,
                ),
            )
        }

        if (!CalibStore.save(this, base.copy(imuFrequency = freq, imuNoise = noise))) {
            tvGuide.text = "写入 profile.json 失败，详见 logcat。"
            render()
            return
        }

        val applied = try {
            SlamBridge.nativeSetImuNoise(
                noise.noiseGyro.toFloat(), noise.noiseAcc.toFloat(),
                noise.gyroWalk.toFloat(), noise.accWalk.toFloat(),
            )
        } catch (e: Throwable) {
            Log.e(TAG, "注入 native 失败", e)
            false
        }

        val msg = if (applied) {
            "已写入并对当前进程生效。"
        } else {
            "已写入"
        }
        if (!walkOk) {
            Log.i(TAG, "游走项未写入，保留模板值 ${tmpl.first} / ${tmpl.second}")
        }
        Log.i(TAG, msg)

        MaterialAlertDialogBuilder(this)
            .setTitle("已保存")
            .setMessage(msg)
            .setPositiveButton("完成") { _, _ -> finish() }
            .setCancelable(false)
            .show()
    }

    /** 模板里当前生效的 walk 值。不写它的时候要存这个数，否则下次读的人会以为是本次测的。 */
    private fun templateWalk(): Pair<Double, Double> = try {
        val txt = assets.open("phone.yaml").bufferedReader().use { it.readText() }
        fun v(key: String): Double =
            Regex("(?m)^\\Q$key\\E:\\s*([-+0-9.eE]+)").find(txt)
                ?.groupValues?.get(1)?.toDoubleOrNull() ?: 0.0
        v("IMU.GyroWalk") to v("IMU.AccWalk")
    } catch (e: Exception) {
        Log.e(TAG, "读模板 walk 值失败", e)
        0.0 to 0.0
    }

    /** 主摄 id 与传感器方向。不打开相机，只读 characteristics。 */
    private fun cameraInfo(): Pair<String, Int>? = try {
        val mgr = getSystemService(Context.CAMERA_SERVICE) as CameraManager
        val id = mgr.cameraIdList.firstOrNull {
            mgr.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) ==
                CameraCharacteristics.LENS_FACING_BACK
        }
        if (id == null) null else {
            val o = mgr.getCameraCharacteristics(id)
                .get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 0
            id to o
        }
    } catch (e: Exception) {
        Log.e(TAG, "读相机信息失败", e)
        null
    }

    // ---------------- 小工具 ----------------

    private fun elapsedSec(): Double {
        val f = firstTs
        if (f == 0L) return 0.0
        return (SystemClock.elapsedRealtimeNanos() - f) / 1e9
    }

    private fun stillRatio(): Double = if (winTotal > 0) winStill.toDouble() / winTotal else 1.0

    private fun gapRatio(): Double = if (pairCount > 0) gapCount.toDouble() / pairCount else 0.0

    private fun clock(sec: Double): String {
        val s = sec.toInt().coerceAtLeast(0)
        return String.format(Locale.US, "%02d:%02d", s / 60, s % 60)
    }

    private fun timeText(sec: Double): String {
        val s = sec.toInt().coerceAtLeast(0)
        return if (s >= 60) String.format(Locale.US, "%d 分 %d 秒", s / 60, s % 60) else "$s 秒"
    }

    /** IDLE 页顶部：上一次标定的时间与质量，让人知道当前值是哪来的。 */
    private fun idLabel(): String {
        val n = CalibStore.load(this)?.imuNoise ?: return "还没有标定过，现在用的是模板值。"
        val day = if (n.recorded.length >= 10) n.recorded.substring(0, 10) else n.recorded
        val walk = if (n.walkUpdated) "四项" else "白噪声"
        return String.format(
            Locale.US, "上次标定 %s · %.1f 分钟 · %s · 静置 %.0f%%",
            day, n.durationS / 60.0, walk, n.stillRatio * 100.0,
        )
    }
}
