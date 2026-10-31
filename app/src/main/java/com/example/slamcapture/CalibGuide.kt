package com.example.slamcapture

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 引导流程里的一步：板子该出现在画面哪个位置、摆成什么形状。
 *
 * [su] / [sv] 是板子中心的相对位置，0 为居中、±1 为贴到画面边：
 * - [su] 沿图像 x，+1 = 屏幕下方
 * - [sv] 沿图像 y，+1 = 屏幕左方
 *
 * [tiltX] / [tiltY] 是弧度，指板子绕哪根相机轴转：[tiltX] 绕图像 x（左右边不等长），
 * [tiltY] 绕图像 y（上下边不等长）。用户不用理解，框自己会画成梯形。
 */
data class GuideStep(
    val title: String,
    val hint: String,
    val su: Float,
    val sv: Float,
    val tiltX: Float,
    val tiltY: Float,
    val tiltTol: Float,
    /** 是否校验形状。正对步只看位置和大小 —— 画面边缘的畸变会让边比本身就不准。 */
    val checkTilt: Boolean,
)

/**
 * 引导几何：把一步的目标姿态投成画面上的四边形，并判断实测板子有没有对上。
 *
 * 纯 Kotlin、不依赖 android。坐标都是图像坐标（640×480 传感器横向，长边沿图像 y），
 * 上屏交给 [PreviewView.imageToScreen]，这里不算 letterbox 也不旋转。
 */
object CalibGuide {

    /** 模板内参 `phone.yaml` 的 `Camera1.fx`。影响目标框的像素尺寸和梯形斜率。 */
    const val FOCAL = 440f

    private const val CX0 = 318.4f
    private const val CY0 = 239.9f

    private const val IMG_W = 640f
    private const val IMG_H = 480f

    /** 纸面离画面边缘留的余量。 */
    private const val MARGIN_PX = 8f

    /**
     * 板子的纸面尺寸（米）。默认 A4 满幅：7×5 个方格、每格 30 mm。
     *
     * 内参标定与物理尺寸无关（物点以「格」为单位），但**画面里的像素尺寸**有关：
     * 板子越大、或越近，每格像素越多。用显示器全屏显示时要按屏幕上的实际尺寸填。
     */
    var boardLong = 0.210f
        private set
    var boardShort = 0.150f
        private set

    fun setBoard(long: Float, short: Float) {
        if (long > 0.05f && short > 0.02f) {
            boardLong = long
            boardShort = short
        }
    }

    /** 24 个内角点铺开的范围：长边跨 COLS 格、短边跨 ROWS 格。 */
    private fun latticeLong() = boardLong * COLS / (COLS + 1)
    private fun latticeShort() = boardShort * ROWS / (ROWS + 1)

    /** 质心容差（px）。 */
    const val POS_TOL = 14f

    /** 尺度容差（RMS 半径的相对差）。 */
    const val SIZE_TOL = 0.15f

    /** 倾斜容差（对边长度比的差）。0.04 对应侧倾通过区间约 ±7°。 */
    private const val TILT_TOL = 0.04f

    private const val DEG = 0.017453292f

    /** 倾斜步的目标角。再大景深吃不消：对焦锁死，板面两端会一近一远。 */
    private const val TILT_DEG = 30f * DEG

    /** 内角点阵列，和 `CalibActivity` 的 `COLS/ROWS` 一致。 */
    const val COLS = 6
    const val ROWS = 4

    /**
     * 纸面长边在画面里的目标长度（px）。
     *
     * 定死它就把「标定距离」和「对焦距离」解耦了：用户移动板子去套框，
     * 而对焦锁在他实际使用的位置。板子不在焦点上只是虚一点，不影响内参的含义。
     */
    private const val TARGET_BOARD_PX = 200f

    /** 目标物距：由板子尺寸和目标像素长决定，与对焦无关。 */
    val workingZ: Float get() = FOCAL * boardLong / TARGET_BOARD_PX

    /** 板子每格在框里占多少像素。 */
    fun squarePx(): Float = TARGET_BOARD_PX / (COLS + 1)

    /**
     * 板子偏离焦点的屈光度数。约 1.5 px/D 的模糊，>2 D 就该提醒角点会变差。
     */
    fun defocusD(focusD: Float): Float = abs(1f / workingZ - focusD)

    /**
     * 9 步：前 5 步把板子推到画面四边（约束畸变），后 4 步倾斜（约束焦距）。
     * 目标坐标的数值表见 guided-calib-capture 的改动记录。
     */
    val STEPS: List<GuideStep> = listOf(
        GuideStep("正对 · 居中", "白框对齐纸边，四个角标贴在最外圈的四个内角点上",
            0f, 0f, 0f, 0f, TILT_TOL, false),
        GuideStep("正对 · 左上", "整块板子挪到画面左上，纸边贴住白框",
            -1f, 1f, 0f, 0f, TILT_TOL, false),
        GuideStep("正对 · 右上", "整块板子挪到画面右上",
            -1f, -1f, 0f, 0f, TILT_TOL, false),
        GuideStep("正对 · 左下", "整块板子挪到画面左下",
            1f, 1f, 0f, 0f, TILT_TOL, false),
        GuideStep("正对 · 右下", "整块板子挪到画面右下",
            1f, -1f, 0f, 0f, TILT_TOL, false),
        GuideStep("侧倾 · 左", "人往左挪半步：白框左边短、右边长",
            0f, 0f, -TILT_DEG, 0f, TILT_TOL, true),
        GuideStep("侧倾 · 右", "人往右挪半步：白框左边长、右边短",
            0f, 0f, TILT_DEG, 0f, TILT_TOL, true),
        GuideStep("俯仰 · 上", "镜头往上抬：白框下边长、上边短",
            0f, 0f, 0f, TILT_DEG, TILT_TOL, true),
        GuideStep("俯仰 · 下", "镜头往下压：白框上边长、下边短",
            0f, 0f, 0f, -TILT_DEG, TILT_TOL, true),
    )

    /** 四个角的符号序，对应局部坐标 (b, a) 的四种组合。 */
    private val SIGNS = floatArrayOf(-1f, -1f, 1f, -1f, 1f, 1f, -1f, 1f)

    /**
     * 投影一步的目标四边形，写进 [out] 的 8 个 float（图像坐标）。
     *
     * [lattice] 选点阵框（24 个内角点）还是纸面框（给用户对纸边）。
     * 输出顺序同 native 的 extremeQuad。
     */
    fun quad(step: GuideStep, lattice: Boolean, out: FloatArray) {
        val longHalf = (if (lattice) latticeLong() else boardLong) * 0.5f
        val shortHalf = (if (lattice) latticeShort() else boardShort) * 0.5f

        val cu = centreU(step)
        val cv = centreV(step)

        val raw = FloatArray(8)
        for (i in 0 until 4) {
            project(step, cu, cv, longHalf * SIGNS[i * 2 + 1], shortHalf * SIGNS[i * 2], raw, i * 2)
        }
        sortExtreme(raw, out)
    }

    /** 一步的目标点阵框，给调用方缓存用。 */
    fun latticeQuad(step: GuideStep): FloatArray = FloatArray(8).also { quad(step, true, it) }

    /** 24 个内角点的目标位置，[out] 需 48 个 float，长边优先（6 列 × 4 行）。 */
    fun latticePoints(step: GuideStep, out: FloatArray) {
        val cu = centreU(step)
        val cv = centreV(step)
        var k = 0
        for (i in 0 until COLS) {
            val a = -latticeLong() * 0.5f + i * (latticeLong() / (COLS - 1))
            for (j in 0 until ROWS) {
                val b = -latticeShort() * 0.5f + j * (latticeShort() / (ROWS - 1))
                project(step, cu, cv, a, b, out, k * 2)
                k++
            }
        }
    }

    /** 板子中心的图像 x。贴边步让**纸面**外沿对齐到 [MARGIN_PX]。 */
    private fun centreU(step: GuideStep): Float {
        val half = FOCAL * (boardShort * 0.5f) / workingZ
        return IMG_W * 0.5f + step.su * (IMG_W * 0.5f - MARGIN_PX - half)
    }

    private fun centreV(step: GuideStep): Float {
        val half = FOCAL * (boardLong * 0.5f) / workingZ
        return IMG_H * 0.5f + step.sv * (IMG_H * 0.5f - MARGIN_PX - half)
    }

    /**
     * 唯一的投影实现。a 沿图像 y（板子长边）、b 沿图像 x（短边），单位米。
     *
     * 先绕图像 y 偏 ψ、再绕图像 x 偏 φ。tiltX 让左右两条边不等长，tiltY 让上下不等长。
     */
    private fun project(
        step: GuideStep, cu: Float, cv: Float, a: Float, b: Float, out: FloatArray, off: Int,
    ) {
        val cp = cos(step.tiltX)
        val sp = sin(step.tiltX)
        val cs = cos(step.tiltY)
        val ss = sin(step.tiltY)

        val x0 = (cu - CX0) * workingZ / FOCAL
        val y0 = (cv - CY0) * workingZ / FOCAL

        val px = x0 + b * cs
        val py = y0 + a * cp + b * sp * ss
        val pz = workingZ - a * sp - b * cs * ss

        out[off] = FOCAL * px / pz + CX0
        out[off + 1] = FOCAL * py / pz + CY0
    }

    /**
     * 按四个方向取极值重排。
     *
     * 不能依赖角点索引 —— 板子翻面首角点就变，取错不报错、只会静默采到错帧。
     */
    private fun sortExtreme(q: FloatArray, out: FloatArray) {
        for (i in 0 until 4) {
            val du = if (i == 0 || i == 3) -1f else 1f
            val dv = if (i == 0 || i == 1) -1f else 1f
            var best = 0
            var bestDot = -Float.MAX_VALUE
            for (j in 0 until 4) {
                val d = q[j * 2] * du + q[j * 2 + 1] * dv
                if (d > bestDot) {
                    bestDot = d
                    best = j
                }
            }
            out[i * 2] = q[best * 2]
            out[i * 2 + 1] = q[best * 2 + 1]
        }
    }

    /** 实测与目标的三个误差分量。复用实例，避免每帧分配。 */
    class Err {
        var posPx = 0f
        var sizeErr = 0f
        var tiltErr = 0f

        /** 连中之后 [relax] 放宽（抗手抖），第一帧用 1.0。 */
        fun pass(relax: Float, step: GuideStep): Boolean =
            posPx <= POS_TOL * relax &&
                sizeErr <= SIZE_TOL * relax &&
                (!step.checkTilt || tiltErr <= step.tiltTol * relax)
    }

    /** 比较实测与目标。三个量正交，因为平均角点距离被位置差主导、感觉不到倾斜。 */
    fun measure(det: FloatArray, target: FloatArray, out: Err) {
        val dcx = (det[0] + det[2] + det[4] + det[6]) * 0.25f
        val dcy = (det[1] + det[3] + det[5] + det[7]) * 0.25f
        val tcx = (target[0] + target[2] + target[4] + target[6]) * 0.25f
        val tcy = (target[1] + target[3] + target[5] + target[7]) * 0.25f

        out.posPx = hypot(dcx - tcx, dcy - tcy)

        val dr = radius(det, dcx, dcy)
        val tr = radius(target, tcx, tcy)
        out.sizeErr = if (tr > 1e-3f) abs(dr / tr - 1f) else 1f

        // 对边比：下/上、左/右。反向倾斜时必然 < 1，这是挡住反方向动作的依据。
        val dBT = ratio(det, 1, 2, 3, 0)
        val tBT = ratio(target, 1, 2, 3, 0)
        val dLR = ratio(det, 2, 3, 0, 1)
        val tLR = ratio(target, 2, 3, 0, 1)
        out.tiltErr =
            if (dBT < 0f || dLR < 0f || tBT < 0f || tLR < 0f) 1f
            else hypot(dBT - tBT, dLR - tLR)
    }

    private fun radius(q: FloatArray, cx: Float, cy: Float): Float {
        var s = 0f
        for (i in 0 until 4) {
            val dx = q[i * 2] - cx
            val dy = q[i * 2 + 1] - cy
            s += dx * dx + dy * dy
        }
        return sqrt(s * 0.25f)
    }

    /** |p_i p_j| / |p_k p_l|；分母退化时返回 −1。 */
    private fun ratio(q: FloatArray, i: Int, j: Int, k: Int, l: Int): Float {
        val a = hypot(q[i * 2] - q[j * 2], q[i * 2 + 1] - q[j * 2 + 1])
        val b = hypot(q[k * 2] - q[l * 2], q[k * 2 + 1] - q[l * 2 + 1])
        return if (b > 1e-3f) a / b else -1f
    }
}
