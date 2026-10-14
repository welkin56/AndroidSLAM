package com.example.slamcapture

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PointF
import android.graphics.RectF
import android.graphics.Typeface
import android.util.AttributeSet
import android.view.View

/**
 * 2D 地图**画中画**：浮在预览右上角的小窗，画走过的轨迹 + 当前位置。
 *
 * 坐标系：**世界 XY 平面俯视**（Mono-Inertial 重力对齐 ⟹ **Z 是竖直**，所以地面就是 XY）。
 * 屏幕 x 取世界 x、屏幕 y 取世界 **-y**（屏幕 y 向下）。
 *
 * ------------------------------------------------------------------
 * ⚠️ **world→view 的变换只有 [worldToView] 一处** —— 和 [PreviewView.toScreen] 同一条纪律。
 *    散在多处的话，点会互相错位，而看起来像「SLAM 不准」。
 * ------------------------------------------------------------------
 *
 * 线程：**全在主线程**。[submit] 由 ticker 调，[onDraw] 由系统调。快照不可变、整引用交换。
 */
class MapView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    /**
     * 一刻的画面内容。**不可变**，主线程整体换引用。
     *
     * `traj` 是**自己的一份拷贝**（调用方的缓冲会复用），所以提交完之后随便改不影响这儿。
     * `hint` 非空时只显示那句话（没轨迹可画）—— **不能沉默**，空窗也要说清为什么空。
     */
    class Snapshot(traj: FloatArray, val trajN: Int, val curX: Float, val curY: Float,
                   val hasCur: Boolean, val hint: String?,
                   val dirX: Float = 0f, val dirY: Float = 0f) {
        /** 自己留一份 —— 调用方那个缓冲下一拍就复用了。 */
        val traj: FloatArray = traj.copyOf()
    }

    /** 当前显示的快照。主线程写、主线程读。 */
    @Volatile
    private var snap: Snapshot? = null

    private val density = resources.displayMetrics.density

    /** 轨迹线。**不用青色** —— 那是测量那边标记/折线的颜色，同屏会混。 */
    private val trajPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = Color.WHITE
        strokeJoin = Paint.Join.ROUND
        strokeCap = Paint.Cap.ROUND
    }

    /** 当前位置：琥珀实心 + 白描边（琥珀 = 「系统标的那个点」，和十字旁边的吸附圈一个语系）。 */
    private val curFill = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = 0xFFFFC107.toInt()
    }
    private val curEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.5f * density
        color = Color.WHITE
    }

    private val bgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = 0x66000000
    }
    private val edgePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1f * density
        color = 0x33FFFFFF
    }

    /** 提示文字。和小窗一样宽，所以字号压到能塞下几个汉字。 */
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 12f * resources.displayMetrics.scaledDensity
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
        textAlign = Paint.Align.CENTER
    }

    /** 自动缩放的结果，每帧现算。 */
    private var scale = 1f
    private var cxWorld = 0f
    private var cyWorld = 0f

    private val bgRect = RectF()
    private val path = Path()
    private val pt = PointF()

    /** 换一帧内容。**主线程**。 */
    fun submit(s: Snapshot) {
        snap = s
        invalidate()
    }

    /**
     * 世界 → 小窗内的屏幕坐标。**唯一的变换**。
     *
     * 屏幕 y 向下、世界 y 向上 ⟹ 第二项取负号。
     */
    private fun worldToView(x: Float, y: Float, out: PointF) {
        out.set(width / 2f + (x - cxWorld) * scale, height / 2f - (y - cyWorld) * scale)
    }

    /**
     * 按内容算 bbox 和缩放。**每帧现算、不存状态** —— 地图自己会跟着你走而缩放。
     *
     * 最小半幅 0.5m：刚起步只有一两个点时 bbox 接近零，不兜住的话缩放会炸到天上。
     */
    private fun fit(s: Snapshot) {
        var minX = Float.MAX_VALUE
        var minY = Float.MAX_VALUE
        var maxX = -Float.MAX_VALUE
        var maxY = -Float.MAX_VALUE
        for (i in 0 until s.trajN) {
            val x = s.traj[i * 3]
            val y = s.traj[i * 3 + 1]
            if (x < minX) minX = x
            if (x > maxX) maxX = x
            if (y < minY) minY = y
            if (y > maxY) maxY = y
        }
        if (s.hasCur) {
            if (s.curX < minX) minX = s.curX
            if (s.curX > maxX) maxX = s.curX
            if (s.curY < minY) minY = s.curY
            if (s.curY > maxY) maxY = s.curY
        }
        cxWorld = (minX + maxX) / 2f
        cyWorld = (minY + maxY) / 2f
        // +10% 余量，别让线贴着边框
        val halfW = maxOf((maxX - minX) / 2f, 0.5f) * 1.1f
        val halfH = maxOf((maxY - minY) / 2f, 0.5f) * 1.1f
        scale = minOf(width / (2f * halfW), height / (2f * halfH))
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val s = snap ?: return

        val r = 6f * density
        bgRect.set(0f, 0f, width.toFloat(), height.toFloat())
        canvas.drawRoundRect(bgRect, r, r, bgPaint)
        canvas.drawRoundRect(bgRect, r, r, edgePaint)

        if (s.hint != null) {
            drawOutlinedText(canvas, s.hint, width / 2f, height / 2f + textPaint.textSize / 3f)
            return
        }

        fit(s)

        // ⚠️ `Path` 是**累积**的 —— 不 reset 的话每一帧都往里再加一条折线，
        //    画面上会越描越粗、而且旧轨迹永远擦不掉。
        path.reset()
        if (s.trajN >= 2) {
            for (i in 0 until s.trajN) {
                worldToView(s.traj[i * 3], s.traj[i * 3 + 1], pt)
                if (i == 0) path.moveTo(pt.x, pt.y) else path.lineTo(pt.x, pt.y)
            }
            canvas.drawPath(path, trajPaint)
        }
        if (s.hasCur) {
            worldToView(s.curX, s.curY, pt)

            // 朝向三角：贴在点外面，尖端指相机光轴在地面上的投影方向。
            // (0,0) 只在**真的正下方**出现（那时方位角无定义）⟹ 不画。
            // ⚠️ 屏幕 y 向下、世界 y 向上 ⟹ 第二项取负，和 [worldToView] 同一个约定。
            if (s.dirX != 0f || s.dirY != 0f) {
                val sx = s.dirX
                val sy = -s.dirY
                val tip = 10f * density
                val half = 4.5f * density
                path.reset()   // 复用轨迹那条 —— 轨迹已经画完了，reset 掉再用
                path.moveTo(pt.x + sx * tip, pt.y + sy * tip)
                path.lineTo(pt.x - sy * half, pt.y + sx * half)
                path.lineTo(pt.x + sy * half, pt.y - sx * half)
                path.close()
                canvas.drawPath(path, curFill)
            }

            canvas.drawCircle(pt.x, pt.y, 4f * density, curFill)
            canvas.drawCircle(pt.x, pt.y, 4f * density, curEdge)
        }
    }

    /** 黑描边 + 白填充画两遍 —— 亮暗背景上都看得清。同 [PreviewView] 那套。 */
    private fun drawOutlinedText(canvas: Canvas, text: String, x: Float, y: Float) {
        textPaint.style = Paint.Style.STROKE
        textPaint.strokeWidth = 3f * density
        textPaint.color = Color.BLACK
        canvas.drawText(text, x, y, textPaint)
        textPaint.style = Paint.Style.FILL
        textPaint.color = Color.WHITE
        canvas.drawText(text, x, y, textPaint)
    }
}
