package com.example.slamcapture

import kotlin.math.abs
import kotlin.math.log10
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.round
import kotlin.math.sqrt

/**
 * Allan 方差标定：从静置的 IMU 序列算四个噪声密度。
 *
 * 与 `tools/calibration/allan_variance.py` 逐行对应，任何一步不同都会让两边对不上：
 * **不去均值、不去趋势、不剔异常值**。输出是连续噪声密度（每 √Hz），
 * 与 phone.yaml 的注释同单位，进 native 前才乘除 √freq。
 */
object AllanVariance {

    /**
     * 样本分块大小。调用方按这个粒度累积，本对象按它取样本。
     *
     * 必须是 2 的幂：块索引用移位、块内偏移用掩码，省掉 1.8M×6 次整数除法。
     * 65536 条一块 = 1.57 MB（6 通道交错），60 分钟约 23 块。
     */
    const val BLOCK_SHIFT = 16
    const val BLOCK = 1 shl BLOCK_SHIFT

    /** σ(τ)=K·√(τ/3) 的换算系数。游走项的 C 要乘它。 */
    private val SQRT3 = sqrt(3.0)

    /** τ 网格的上下界：5 个采样点到总长的 1/30，中间取 50 点。 */
    private const val TAU_POINTS = 50
    private const val M_LO = 5

    /** 滑窗宽度（点数）。窗口跨度约 3.16×，|斜率−目标| ≤ 0.05 对应系数误差 ≤ 6%。 */
    private const val FIT_WIN = 8

    /**
     * 一组三轴拟合的汇总。
     *
     * [spread] 是轴间离散度 `(max−min)/mean`，[worstSlopeDev] 是三轴里最偏离目标的那个斜率。
     * 任一轴出 NaN 时 [mean] 与 [spread] 都是 NaN——和 `np.mean` 一样不做 nan 忽略。
     */
    data class AxisFit(
        val values: DoubleArray,
        val slopes: DoubleArray,
        val mean: Double,
        val spread: Double,
        val worstSlopeDev: Double,
    )

    data class Result(
        val noiseGyro: AxisFit,
        val noiseAcc: AxisFit,
        val gyroWalk: AxisFit,
        val accWalk: AxisFit,
        val tauMin: Double,
        val tauMax: Double,
        val tauCount: Int,
    )

    /**
     * @param blocks 分块样本，每块 [BLOCK]×6 个 float 交错存放（gyro x/y/z, acc x/y/z）
     * @param n 有效样本数，必须 ≤ `blocks.size * BLOCK`
     * @param dt 平均采样周期（秒）
     * @param onProgress 进度回调 (已完成单元数, 总单元数)，总数为 6 轴 × τ 点数
     */
    fun compute(
        blocks: Array<FloatArray>,
        n: Int,
        dt: Double,
        onProgress: ((Int, Int) -> Unit)? = null,
    ): Result {
        require(blocks.isNotEmpty()) { "没有样本" }
        require(n >= 2) { "样本太少" }
        require(dt > 0.0) { "采样周期必须为正" }
        require(n <= blocks.size * BLOCK) { "样本数超出块容量" }

        val mHi = max(n / 30, 20)
        val taus = tauGrid(M_LO, mHi, dt)
        val total = 6 * taus.size
        var done = 0

        // csum 逐轴复用：长度 n+1 的 Double，20 分钟约 3.9 MB。
        val csum = DoubleArray(n + 1)

        /** 算一轴的 Allan 偏差。每次新分配结果，因为下一轴会覆盖 [csum]。 */
        fun adevOf(channel: Int): DoubleArray {
            csum[0] = 0.0
            var k = 0
            while (k < n) {
                csum[k + 1] = csum[k] + sample(blocks, channel, k)
                k++
            }
            return overlappingAdev(csum, n, dt, taus) {
                done++
                onProgress?.invoke(done, total)
            }
        }

        // 六条 adev 先全部算完再拟合：白噪声与游走共用同一条曲线，只换拟合目标。
        val adevs = Array(6) { adevOf(it) }

        /** [channels] 是三个轴的通道号；[walk] 为真时按 σ=K√(τ/3) 乘 √3。 */
        fun axisFit(channels: IntArray, target: Double, walk: Boolean): AxisFit {
            val values = DoubleArray(3)
            val slopes = DoubleArray(3)
            for (a in 0 until 3) {
                // 白噪声与游走共用同一条 adev，只换拟合目标——脚本对同一轴算了两遍，
                // 结果一样，只是慢一倍。
                val f = fitPowerLaw(taus, adevs[channels[a]], target)
                values[a] = if (walk && f.c.isFinite()) f.c * SQRT3 else f.c
                slopes[a] = f.slope
            }
            val mean = values.average()
            var mn = Double.POSITIVE_INFINITY
            var mx = Double.NEGATIVE_INFINITY
            for (v in values) {
                if (v < mn) mn = v
                if (v > mx) mx = v
            }
            val spread = if (mean > 0.0 && mean.isFinite()) (mx - mn) / mean else Double.NaN
            val worst = if (slopes.any { it.isNaN() }) Double.NaN
            else slopes.maxOf { abs(it - target) }
            return AxisFit(values, slopes, mean, spread, worst)
        }

        val g = intArrayOf(0, 1, 2)
        val a = intArrayOf(3, 4, 5)
        return Result(
            noiseGyro = axisFit(g, -0.5, walk = false),
            noiseAcc = axisFit(a, -0.5, walk = false),
            gyroWalk = axisFit(g, +0.5, walk = true),
            accWalk = axisFit(a, +0.5, walk = true),
            tauMin = taus.first(),
            tauMax = taus.last(),
            tauCount = taus.size,
        )
    }

    /**
     * 取第 [ch] 通道第 [i] 个样本。
     *
     * 块大小是 2 的幂，所以块索引用移位、块内偏移用掩码 —— 这个函数每次计算要跑
     * 一百多万乘六次，整数除法在这里是实打实的开销。
     */
    private inline fun sample(blocks: Array<FloatArray>, ch: Int, i: Int): Float =
        blocks[i shr BLOCK_SHIFT][(i and (BLOCK - 1)) * 6 + ch]

    /**
     * τ 网格：logspace 取 50 点，对齐到 dt 的整数倍后去重。
     *
     * 去重在整数 m 上做——τ = m·dt 对 m 单射，比浮点比较可靠。
     */
    private fun tauGrid(mLo: Int, mHi: Int, dt: Double): DoubleArray {
        val lo = log10(mLo * dt)
        val hi = log10(mHi * dt)
        val ms = sortedSetOf<Int>()
        for (i in 0 until TAU_POINTS) {
            val v = lo + (hi - lo) * i / (TAU_POINTS - 1).toDouble()
            val m = round(10.0.pow(v) / dt).toInt()
            if (m >= 1) ms.add(m)
        }
        return DoubleArray(ms.size) { i -> ms.elementAt(i) * dt }
    }

    /**
     * overlapping Allan deviation。σ²(τ) = mean[(ȳ_{j+m} − ȳ_j)²] / 2，ȳ 是 m 点重叠窗均值。
     *
     * 代数展开后不需要落地 win / diff 两个 n 长的临时数组：
     * `diff[j] = (csum[j+2m] − 2·csum[j+m] + csum[j]) / m`。
     * [csum] 已就绪，长度 n+1；不满足 `n ≥ 2m+1` 的 τ 留 NaN，不进拟合。
     */
    private fun overlappingAdev(
        csum: DoubleArray,
        n: Int,
        dt: Double,
        taus: DoubleArray,
        onStep: () -> Unit,
    ): DoubleArray {
        val out = DoubleArray(taus.size)
        for (i in taus.indices) {
            val m = max(round(taus[i] / dt).toInt(), 2)
            if (n >= 2 * m + 1) {
                val cnt = n - 2 * m + 1
                var acc = 0.0
                var k = 0
                while (k < cnt) {
                    val d = csum[k + 2 * m] - 2.0 * csum[k + m] + csum[k]
                    acc += d * d
                    k++
                }
                val mf = m.toDouble()
                out[i] = sqrt(acc / cnt / (mf * mf) / 2.0)
            } else {
                out[i] = Double.NaN
            }
            onStep()
        }
        return out
    }

    /** 一次幂律拟合的结果。C 的定义：σ(τ*) = C·τ*^target。 */
    private class Fit(val c: Double, val slope: Double, val tauStar: Double)

    /**
     * 在 log-log 上滑窗线性拟合，取斜率最接近 [target] 的窗口（同分取靠前的）。
     *
     * C 用窗口两端 τ 的**几何中点**处的 σ 反算，而不是用实测斜率外推——
     * 这样斜率有偏差时 C 也不跟着漂。至少 4 个有效点才拟合，否则返回 NaN。
     */
    private fun fitPowerLaw(taus: DoubleArray, adev: DoubleArray, target: Double): Fit {
        var cnt = 0
        val lt = DoubleArray(taus.size)
        val la = DoubleArray(taus.size)
        val tv = DoubleArray(taus.size)
        for (i in taus.indices) {
            val v = adev[i]
            if (v.isFinite() && v > 0.0) {
                tv[cnt] = taus[i]
                lt[cnt] = log10(taus[i])
                la[cnt] = log10(v)
                cnt++
            }
        }
        if (cnt < 4) return Fit(Double.NaN, Double.NaN, Double.NaN)

        val win = min(FIT_WIN, cnt)
        var bestScore = Double.MAX_VALUE
        var bestSlope = Double.NaN
        var bestFrom = 0
        for (i in 0..(cnt - win)) {
            val s = slopeOf(lt, la, i, win)
            val score = abs(s - target)
            if (score < bestScore) {
                bestScore = score
                bestSlope = s
                bestFrom = i
            }
        }

        val tauLo = tv[bestFrom]
        val tauHi = tv[bestFrom + win - 1]
        val tauStar = sqrt(tauLo * tauHi)
        val sigmaStar = 10.0.pow(interpLog(log10(tauStar), lt, la, cnt))
        return Fit(sigmaStar * tauStar.pow(-target), bestSlope, tauStar)
    }

    /** 最小二乘斜率（`np.polyfit(deg=1)` 的闭式解）。只要斜率，截距不参与结果。 */
    private fun slopeOf(x: DoubleArray, y: DoubleArray, from: Int, len: Int): Double {
        var sx = 0.0
        var sy = 0.0
        var sxx = 0.0
        var sxy = 0.0
        for (k in from until from + len) {
            sx += x[k]
            sy += y[k]
            sxx += x[k] * x[k]
            sxy += x[k] * y[k]
        }
        val den = len * sxx - sx * sx
        return if (den == 0.0) 0.0 else (len * sxy - sx * sy) / den
    }

    /** `np.interp` 的对数域版本：[xs] 升序、前 [n] 项有效，区间外夹到端点。 */
    private fun interpLog(x: Double, xs: DoubleArray, ys: DoubleArray, n: Int): Double {
        if (x <= xs[0]) return ys[0]
        if (x >= xs[n - 1]) return ys[n - 1]
        var i = 0
        while (i < n - 2 && xs[i + 1] < x) i++
        val dx = xs[i + 1] - xs[i]
        if (dx == 0.0) return ys[i]
        return ys[i] + (ys[i + 1] - ys[i]) * (x - xs[i]) / dx
    }
}
