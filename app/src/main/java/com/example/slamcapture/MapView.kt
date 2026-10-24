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
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import kotlin.math.abs

/**
 * 2D 地图画中画：显示地图点云的一层横切、轨迹（白色为本次，蓝色为此前各次）
 * 以及当前位置。
 *
 * 坐标系为世界 XY 平面俯视；Mono-Inertial 下 Z 是竖直方向，屏幕 y 取世界 -y。
 *
 * 比例只在按 + / - 时改变，中心跟随当前位置，不做自动 fit。
 * 世界到视图的变换只在 [worldToView] 中实现。
 *
 * 所有方法在主线程调用，快照不可变、整体替换引用。
 */
class MapView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    /**
     * 一帧的画面内容，不可变，整体替换引用。
     * traj 和 pts 各自复制一份，调用方的缓冲之后修改不会影响这里。
     * hint 非空时只显示提示文字。
     */
    class Snapshot(traj: FloatArray, val trajN: Int,
                   val curX: Float, val curY: Float, val curZ: Float, val hasCur: Boolean,
                   val hint: String?,
                   val dirX: Float = 0f, val dirY: Float = 0f,
                   pts: FloatArray = EMPTY, val ptsN: Int = 0) {
        /** 保留一份拷贝，调用方的缓冲之后会复用。 */
        val traj: FloatArray = traj.copyOf()

        /** 只复制实际使用的部分。 */
        val pts: FloatArray = if (ptsN > 0) pts.copyOf(ptsN * 3) else EMPTY
    }

    companion object {
        private val EMPTY = FloatArray(0)

        /**
         * 缩放档位，单位米/dp（相邻约 1.5 倍）；乘以 density 得到米/像素。
         * 首尾分别为放大和缩小的下限。
         */
        private val ZOOM_M_PER_DP = listOf(
            0.0018f, 0.0027f, 0.004f, 0.006f, 0.009f, 0.013f, 0.02f, 0.03f,
            0.05f, 0.075f, 0.1f, 0.15f, 0.25f, 0.4f, 0.65f, 1.0f)

        /** 默认档 0.1 m/dp；从表中查下标，避免改表后错位。 */
        private val ZOOM_DEFAULT = ZOOM_M_PER_DP.indexOf(0.1f)

        /** 切片厚度范围（全宽，米）。 */
        const val SLICE_MIN = 0.2f
        const val SLICE_MAX = 4.0f
        const val SLICE_DEFAULT = 1.0f

        /** 切片中心高度变化超过此值时直接对齐，不进行平滑。 */
        private const val Z_SNAP = 1.5f

        /** 切片中心低通系数，每次移动剩余偏差的 15%。 */
        private const val Z_LOWPASS = 0.15f
    }

    /** 当前快照，主线程读写。 */
    @Volatile
    private var snap: Snapshot? = null

    private val density = resources.displayMetrics.density

    // ---------------- 视图状态，不随快照变化 ----------------

    private var zoomIdx = ZOOM_DEFAULT

    /** 每 dp 对应的米数，是比例的唯一来源。 */
    private var mPerDp = ZOOM_M_PER_DP[zoomIdx]

    /** 每世界米对应的像素数。 */
    private val pxPerMeter: Float get() = 1f / (mPerDp * density)

    /** 视图中心的世界坐标；跟随当前位置，无位姿时冻结。 */
    private var cx = 0f
    private var cy = 0f

    /** 查看器模式：false 时不跟随当前位置，改由 fitToBounds 和拖动决定。 */
    private var autoCenter = true

    /** 查看器模式：使用连续缩放；小地图仍使用档位表。 */
    private var continuousZoom = false

    /** 全屏查看器：不绘制圆角底板和描边。 */
    private var fullBleed = false

    /** 切片厚度（米，全宽）。 */
    private var sliceRange = SLICE_DEFAULT

    /** 切片中心的低通结果（世界 z），避免手抖导致边缘点反复进出。 */
    private var zS = 0f
    private var zReady = false

    /** 点云画布缓冲，每点两个屏幕坐标；按需要增长。 */
    private var cloudScratch = FloatArray(0)

    // ---------------- 查看器手势：拖动和双指缩放 ----------------
    //
    // 只有 autoCenter 为 false 时消费触摸；小地图不接收手势。

    /** 上一次拖动的锚点（指针平均位置）。 */
    private var lastTouchX = 0f
    private var lastTouchY = 0f

    /** 本次计算出的指针平均位置，与 lastTouch 比较。 */
    private var dragFocusX = 0f
    private var dragFocusY = 0f

    /** 捏合过程中上一次的焦点。 */
    private var lastFocusX = 0f
    private var lastFocusY = 0f

    private val scaleDetector = ScaleGestureDetector(context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScaleBegin(detector: ScaleGestureDetector): Boolean {
                lastFocusX = detector.focusX
                lastFocusY = detector.focusY
                return true
            }

            override fun onScale(detector: ScaleGestureDetector): Boolean {
                val focusX = detector.focusX
                val focusY = detector.focusY
                val old = mPerDp
                val next = (old / detector.scaleFactor)
                    .coerceIn(ZOOM_M_PER_DP.first(), ZOOM_M_PER_DP.last())
                val oldPpm = 1f / (old * density)
                val newPpm = 1f / (next * density)
                // 保持两指焦点下的世界点不动；焦点移动时同时平移。
                val wx = cx + (lastFocusX - width / 2f) / oldPpm
                val wy = cy - (lastFocusY - height / 2f) / oldPpm
                mPerDp = next
                cx = wx - (focusX - width / 2f) / newPpm
                cy = wy + (focusY - height / 2f) / newPpm
                lastFocusX = focusX
                lastFocusY = focusY
                invalidate()
                return true
            }
        })

    // ---------------- 画笔 ----------------

    /**
     * 点云画笔，使用单一颜色。
     * drawPoints 以 strokeWidth 作为点直径，需要 Cap.ROUND 才是圆点。
     */
    private val cloudPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        strokeWidth = 1.8f * density
        color = 0xB0CFD8DC.toInt()
    }

    /** 轨迹线。避免使用青色，以免与测量折线混淆。 */
    private val trajPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = Color.WHITE
        strokeJoin = Paint.Join.ROUND
        strokeCap = Paint.Cap.ROUND
    }

    /** 历史轨迹：蓝色，比实时轨迹细。 */
    private val histPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.4f * density
        color = 0xCC64B5F6.toInt()
        strokeJoin = Paint.Join.ROUND
        strokeCap = Paint.Cap.ROUND
    }

    /** 当前位置：琥珀色实心加白色描边。 */
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

    /** 路径点，绿色。 */
    private val wpFill = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = 0xFF66BB6A.toInt()
    }
    private val wpEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.2f * density
        color = Color.WHITE
    }
    private val wpText = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 9f * resources.displayMetrics.scaledDensity
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
        textAlign = Paint.Align.CENTER
    }

    private val edgePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1f * density
        color = 0x33FFFFFF
    }

    /** 提示文字画笔。 */
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 12f * resources.displayMetrics.scaledDensity
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
        textAlign = Paint.Align.CENTER
    }

    private val bgRect = RectF()
    private val path = Path()
    private val pt = PointF()

    // ---------------- 路径点，独立于 Snapshot ----------------
    //
    // 快照每帧重建，路径点只是偶尔变化，因此单独保存，由 revision 驱动更新。

    /** 世界坐标 x y，每点两个。 */
    private var wpXY = FloatArray(0)
    private var wpName = arrayOfNulls<String>(0)
    private var wpN = 0

    // ---------------- 外部命令（主线程）----------------

    /** 替换一帧内容，主线程。 */
    fun submit(s: Snapshot) {
        snap = s
        invalidate()
    }

    /**
     * 替换路径点，主线程调用。xy 每点两个世界坐标，names[i] 是第 i 个的名字。
     * 只传入有效且位于当前图的点；调用方按 revision 决定何时重建。
     */
    fun setWaypoints(xy: FloatArray, n: Int, names: Array<String?>) {
        if (wpXY.size < n * 2) wpXY = FloatArray(n * 2)
        System.arraycopy(xy, 0, wpXY, 0, n * 2)
        if (wpName.size < n) wpName = arrayOfNulls(n)
        for (i in 0 until n) wpName[i] = names.getOrNull(i)
        wpN = n
        invalidate()
    }

    /** 清空路径点图层，主线程调用。 */
    fun clearWaypoints() {
        if (wpN == 0) return
        wpN = 0
        invalidate()
    }

    // ---------------- 历史轨迹，独立于 Snapshot ----------------
    //
    // 同一张图此前各趟的轨迹，已换算到当前图坐标系。

    /** 每条轨迹 3n 个 float（世界坐标 x y z），由本视图持有。 */
    private var hist: List<FloatArray> = emptyList()

    /** 替换历史轨迹，主线程调用。数组归本视图所有。 */
    fun setHistory(traces: List<FloatArray>) {
        hist = traces
        invalidate()
    }

    /** 清空历史轨迹层，主线程调用。 */
    fun clearHistory() {
        if (hist.isEmpty()) return
        hist = emptyList()
        invalidate()
    }

    /** 放大一档；连续模式下按 1.5 倍。 */
    fun zoomIn() {
        if (continuousZoom) {
            mPerDp = (mPerDp / 1.5f).coerceAtLeast(ZOOM_M_PER_DP.first())
            invalidate()
            return
        }
        if (zoomIdx == 0) return
        mPerDp = ZOOM_M_PER_DP[--zoomIdx]
        invalidate()
    }

    /** 缩小一档；连续模式下按 1.5 倍。 */
    fun zoomOut() {
        if (continuousZoom) {
            mPerDp = (mPerDp * 1.5f).coerceAtMost(ZOOM_M_PER_DP.last())
            invalidate()
            return
        }
        if (zoomIdx >= ZOOM_M_PER_DP.size - 1) return
        mPerDp = ZOOM_M_PER_DP[++zoomIdx]
        invalidate()
    }

    /** 查看器模式：关闭跟随当前位置。 */
    fun setAutoCenter(on: Boolean) {
        autoCenter = on
    }

    /** 查看器模式：启用连续缩放。 */
    fun setContinuousZoom(on: Boolean) {
        continuousZoom = on
    }

    /** 当前切片中心的世界 z。 */
    fun sliceCenter(): Float = zS

    /** 由查看器直接指定切片中心。 */
    fun setSliceCenter(z: Float) {
        zS = z
        zReady = true
        invalidate()
    }

    /** 将当前高度送入切片中心低通，返回低通后的值。 */
    fun trackSliceCenter(z: Float): Float {
        updateSliceCenter(z)
        return zS
    }

    /** 当前切片厚度（米，全宽）。 */
    fun sliceRange(): Float = sliceRange

    /** 全屏查看器：跳过圆角底板和描边。 */
    fun setFullBleed(on: Boolean) {
        fullBleed = on
    }

    /**
     * 缩放视图以装下指定世界包围盒，并保留 10% 余量。
     * 使用连续比例，结果限制在档位表范围内。
     */
    fun fitToBounds(minX: Float, minY: Float, maxX: Float, maxY: Float) {
        if (width <= 0 || height <= 0) return
        val w = (maxX - minX).coerceAtLeast(1e-3f)
        val h = (maxY - minY).coerceAtLeast(1e-3f)
        val need = maxOf(
            w * 1.1f / (width / density),
            h * 1.1f / (height / density)
        )
        mPerDp = need.coerceIn(ZOOM_M_PER_DP.first(), ZOOM_M_PER_DP.last())
        cx = (minX + maxX) * 0.5f
        cy = (minY + maxY) * 0.5f
        invalidate()
    }

    /**
     * 查看器的拖动和双指缩放。
     * 小地图（autoCenter 为 true）不消费触摸。
     */
    override fun onTouchEvent(event: MotionEvent): Boolean {
        if (autoCenter) return false

        scaleDetector.onTouchEvent(event)
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN,
            MotionEvent.ACTION_POINTER_DOWN -> {
                // 手指数变化时更新锚点，避免画面跳动。
                updateDragFocus(event, -1)
                lastTouchX = dragFocusX
                lastTouchY = dragFocusY
            }
            MotionEvent.ACTION_POINTER_UP -> {
                // 必须排除抬起的手指，否则位移会从错误位置计算。
                updateDragFocus(event, event.actionIndex)
                lastTouchX = dragFocusX
                lastTouchY = dragFocusY
            }
            MotionEvent.ACTION_MOVE -> {
                if (!scaleDetector.isInProgress) {
                    // 双指时跟随中点，与 ScaleGestureDetector 的焦点一致。
                    updateDragFocus(event, -1)
                    val dx = dragFocusX - lastTouchX
                    val dy = dragFocusY - lastTouchY
                    cx -= dx / pxPerMeter
                    cy += dy / pxPerMeter
                    lastTouchX = dragFocusX
                    lastTouchY = dragFocusY
                    invalidate()
                }
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> Unit
        }
        return true
    }

    /** 将当前指针平均位置写入 dragFocusX/Y；skip 为要排除的指针下标，-1 表示全部参与。 */
    private fun updateDragFocus(event: MotionEvent, skip: Int) {
        var sx = 0f
        var sy = 0f
        var n = 0
        for (i in 0 until event.pointerCount) {
            if (i == skip) continue
            sx += event.getX(i)
            sy += event.getY(i)
            n++
        }
        if (n > 0) {
            dragFocusX = sx / n
            dragFocusY = sy / n
        }
    }

    /** 当前缩放档位下标，用于持久化。 */
    fun zoomIndex(): Int = zoomIdx

    /** 恢复缩放档位；越界值（如 -1）忽略。 */
    fun setZoomIndex(i: Int) {
        if (i < 0 || i >= ZOOM_M_PER_DP.size || i == zoomIdx) return
        zoomIdx = i
        mPerDp = ZOOM_M_PER_DP[i]
        invalidate()
    }

    /** 设置切片厚度（米，全宽），越界值会被夹取。 */
    fun setSliceRange(m: Float) {
        sliceRange = m.coerceIn(SLICE_MIN, SLICE_MAX)
        invalidate()
    }

    // ---------------- 画 ----------------

    /**
     * 世界坐标到视图屏幕坐标的唯一变换。
     * 屏幕 y 向下、世界 y 向上，因此第二项取负号。
     */
    private fun worldToView(x: Float, y: Float, out: PointF) {
        out.set(width / 2f + (x - cx) * pxPerMeter, height / 2f - (y - cy) * pxPerMeter)
    }

    /** 推进切片中心；大跳变时直接对齐，避免缓慢滑动。 */
    private fun updateSliceCenter(z: Float) {
        if (!zReady || abs(z - zS) > Z_SNAP) zS = z
        else zS += (z - zS) * Z_LOWPASS
        zReady = true
    }

    /**
     * 只绘制当前高度附近的点，形成墙和家具的横切轮廓。
     * 单次遍历写入 scratch，最后统一 drawPoints。
     */
    private fun drawCloud(canvas: Canvas, s: Snapshot) {
        if (s.ptsN <= 0) return
        // 按需增长，之后不再缩小，稳定后零分配。
        if (cloudScratch.size < s.ptsN * 2) cloudScratch = FloatArray(s.ptsN * 2)

        val lo = zS - sliceRange * 0.5f
        val hi = zS + sliceRange * 0.5f

        var cnt = 0
        for (i in 0 until s.ptsN) {
            val z = s.pts[i * 3 + 2]
            if (z < lo || z > hi) continue
            worldToView(s.pts[i * 3], s.pts[i * 3 + 1], pt)
            if (pt.x < 0f || pt.x >= width || pt.y < 0f || pt.y >= height) continue
            cloudScratch[cnt * 2] = pt.x
            cloudScratch[cnt * 2 + 1] = pt.y
            cnt++
        }
        if (cnt > 0) canvas.drawPoints(cloudScratch, 0, cnt * 2, cloudPaint)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val s = snap ?: return

        if (!fullBleed) {
            val r = 6f * density
            bgRect.set(0f, 0f, width.toFloat(), height.toFloat())
            canvas.drawRoundRect(bgRect, r, r, bgPaint)
            canvas.drawRoundRect(bgRect, r, r, edgePaint)
        }

        if (s.hint != null) {
            drawOutlinedText(canvas, s.hint, width / 2f, height / 2f + textPaint.textSize / 3f)
            return
        }

        // 中心跟随当前位置，无位姿时保持上一次的值。
        // 查看器不跟随，由 fit 和拖动决定中心。
        if (s.hasCur && autoCenter) {
            cx = s.curX
            cy = s.curY
            updateSliceCenter(s.curZ)
        }

        drawCloud(canvas, s)
        drawHistory(canvas)

        // Path 会累积，必须 reset，否则每帧都会重复添加轨迹。
        path.reset()
        if (s.trajN >= 2) {
            for (i in 0 until s.trajN) {
                worldToView(s.traj[i * 3], s.traj[i * 3 + 1], pt)
                if (i == 0) path.moveTo(pt.x, pt.y) else path.lineTo(pt.x, pt.y)
            }
            canvas.drawPath(path, trajPaint)
        }

        // 路径点在轨迹之后、当前位置之前绘制。
        drawWaypoints(canvas)

        if (s.hasCur) {
            worldToView(s.curX, s.curY, pt)
            // 朝向三角：尖端指向相机光轴在地面上的投影方向。
            // (0,0) 表示正下方，此时不画；屏幕 y 与世界 y 方向相反。
            if (s.dirX != 0f || s.dirY != 0f) {
                val sx = s.dirX
                val sy = -s.dirY
                val tip = 10f * density
                val half = 4.5f * density
                path.reset()   // 复用轨迹的 Path
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

    /** 历史轨迹共用一个 Path，每条一个子路径，只取 XY。 */
    private fun drawHistory(canvas: Canvas) {
        if (hist.isEmpty()) return
        path.reset()
        var any = false
        for (t in hist) {
            val n = t.size / 3
            if (n < 2) continue
            for (i in 0 until n) {
                worldToView(t[i * 3], t[i * 3 + 1], pt)
                if (i == 0) path.moveTo(pt.x, pt.y) else path.lineTo(pt.x, pt.y)
            }
            any = true
        }
        if (any) canvas.drawPath(path, histPaint)
    }

    /** 绘制路径点：绿色菱形和名字，只绘制视口内的点。 */
    private fun drawWaypoints(canvas: Canvas) {
        if (wpN <= 0) return
        val half = 3.2f * density
        for (i in 0 until wpN) {
            worldToView(wpXY[i * 2], wpXY[i * 2 + 1], pt)
            if (pt.x < -half || pt.x > width + half || pt.y < -half || pt.y > height + half) continue

            path.reset()
            path.moveTo(pt.x, pt.y - half * 1.4f)
            path.lineTo(pt.x + half, pt.y)
            path.lineTo(pt.x, pt.y + half * 1.4f)
            path.lineTo(pt.x - half, pt.y)
            path.close()
            canvas.drawPath(path, wpFill)
            canvas.drawPath(path, wpEdge)

            val name = wpName[i]
            if (!name.isNullOrEmpty()) {
                drawOutlinedText(canvas, name.take(10), pt.x, pt.y - half * 2.2f, wpText)
            }
        }
    }

    /** 黑描边加白填充，保证亮暗背景都可见。 */
    private fun drawOutlinedText(canvas: Canvas, text: String, x: Float, y: Float,
                                 paint: Paint = textPaint) {
        val oldStyle = paint.style
        val oldColor = paint.color
        val oldStroke = paint.strokeWidth
        paint.style = Paint.Style.STROKE
        paint.strokeWidth = 3f * density
        paint.color = Color.BLACK
        canvas.drawText(text, x, y, paint)
        paint.style = Paint.Style.FILL
        paint.color = Color.WHITE
        canvas.drawText(text, x, y, paint)
        paint.style = oldStyle
        paint.color = oldColor
        paint.strokeWidth = oldStroke
    }
}
