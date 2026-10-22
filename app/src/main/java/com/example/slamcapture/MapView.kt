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
import kotlin.math.abs

/**
 * 2D 地图**画中画**：浮在预览右上角的小窗，画**地图点云的一层横切** + 走过的轨迹
 * （白色是本趟、蓝色是这张图以前各趟）+ 当前位置。
 *
 * 坐标系：**世界 XY 平面俯视**（Mono-Inertial 重力对齐 ⟹ **Z 是竖直**，所以地面就是 XY）。
 * 屏幕 x 取世界 x、屏幕 y 取世界 **-y**（屏幕 y 向下）。
 *
 * ------------------------------------------------------------------
 * ⚠️ **比例固定，只有按 `+` / `−` 才变**（[zoomIdx]），中心跟着当前位置。
 *    不做自动 fit：那会让画面每帧自己缩放/平移，看着像「它为什么突然放大了」；
 *    而且地图点里几个远处的离群点就能把 bbox 撑爆、把轨迹压成一小团。
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
     * `traj` / `pts` 都是**自己的一份拷贝**（调用方的缓冲下一拍就复用），提交完之后
     * 那边随便改不影响这儿。
     *
     * `hint` 非空时只显示那句话（没东西可画）—— **不能沉默**，空窗也要说清为什么空。
     */
    class Snapshot(traj: FloatArray, val trajN: Int,
                   val curX: Float, val curY: Float, val curZ: Float, val hasCur: Boolean,
                   val hint: String?,
                   val dirX: Float = 0f, val dirY: Float = 0f,
                   pts: FloatArray = EMPTY, val ptsN: Int = 0) {
        /** 自己留一份 —— 调用方那个缓冲下一拍就复用了。 */
        val traj: FloatArray = traj.copyOf()

        /** 同上，但**只拷用到的部分**（容量由 C++ 那侧的上限决定，实际常只有几千）。 */
        val pts: FloatArray = if (ptsN > 0) pts.copyOf(ptsN * 3) else EMPTY
    }

    companion object {
        private val EMPTY = FloatArray(0)

        /**
         * 缩放档位，**米 / dp**（相邻约 ×1.5）。乘 `density` 才是米 / 像素。
         *
         * 一头一尾就是放大 / 缩小的底线：140dp 的小窗从**约 0.25 m** 一直看到 **140 m**。
         * 最里那几档是给「凑近看墙角 / 看轨迹拐弯」用的 —— 那里点云会很稀，那是稀疏地图的本相。
         */
        private val ZOOM_M_PER_DP = listOf(
            0.0018f, 0.0027f, 0.004f, 0.006f, 0.009f, 0.013f, 0.02f, 0.03f,
            0.05f, 0.075f, 0.1f, 0.15f, 0.25f, 0.4f, 0.65f, 1.0f)

        /** 默认档 = `0.1 m/dp`（一屏约 14 m，室内正好）。**从表里找**，免得改表时下标对不上。 */
        private val ZOOM_DEFAULT = ZOOM_M_PER_DP.indexOf(0.1f)

        /** 切片厚度的范围（**全宽**，米）。抽屉里那个滑块。 */
        const val SLICE_MIN = 0.2f
        const val SLICE_MAX = 4.0f
        const val SLICE_DEFAULT = 1.0f

        /** 切片中心的高度变化超过这么多，就**直接对齐**而不是慢慢滑（起步/重置/重定位）。 */
        private const val Z_SNAP = 1.5f

        /** 切片中心的低通系数（每拍挪剩余的 15%，4Hz 下 τ ≈ 1.6 s）。 */
        private const val Z_LOWPASS = 0.15f
    }

    /** 当前显示的快照。主线程写、主线程读。 */
    @Volatile
    private var snap: Snapshot? = null

    private val density = resources.displayMetrics.density

    // ---------------- 视图状态（**不随快照走**）----------------

    private var zoomIdx = ZOOM_DEFAULT

    /** 一个 dp 是多少米。**比例的唯一来源**。 */
    private var mPerDp = ZOOM_M_PER_DP[zoomIdx]

    /** 一个世界米是多少像素。**唯一的变换参数之一**。 */
    private val pxPerMeter: Float get() = 1f / (mPerDp * density)

    /** 视图中心（世界坐标）。**跟着当前位置走**；没有位姿时冻结。 */
    private var cx = 0f
    private var cy = 0f

    /** 切片厚度（米，全宽）。抽屉里的滑块。 */
    private var sliceRange = SLICE_DEFAULT

    /** 切片中心的**低通**结果（世界 z）。⚠️ 不用当前高度直接当中心 —— 手抖会让
     *  切片边缘的点反复进出，一片点在那儿闪。 */
    private var zS = 0f
    private var zReady = false

    /**
     * 点云画布缓冲（每点两个屏幕坐标），复用。**按需要长大** ——
     * 上限在 C++ 那侧（`kMapPointsMax`），这儿不去写一个和它对着的魔法数字。
     */
    private var cloudScratch = FloatArray(0)

    // ---------------- 画笔 ----------------

    /**
     * 点云。**单色** —— 白给了轨迹、琥珀给了当前位置、青给了测量，点云要第四个色。
     * `drawPoints` 拿 `strokeWidth` 当点的直径，`Cap.ROUND` 才是圆点（默认 BUTT 是方块）。
     */
    private val cloudPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        strokeWidth = 1.8f * density
        color = 0xB0CFD8DC.toInt()
    }

    /** 轨迹线。**不用青色** —— 那是测量那边标记/折线的颜色，同屏会混。 */
    private val trajPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = Color.WHITE
        strokeJoin = Paint.Join.ROUND
        strokeCap = Paint.Cap.ROUND
    }

    /** 历史轨迹：蓝色、比实时那条细，压在白线下面。 */
    private val histPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.4f * density
        color = 0xCC64B5F6.toInt()
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

    /** 路径点。**绿色** —— 和点云/轨迹/当前位置/测量都不同色。 */
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

    /** 提示文字。和小窗一样宽，所以字号压到能塞下几个汉字。 */
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 12f * resources.displayMetrics.scaledDensity
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
        textAlign = Paint.Align.CENTER
    }

    private val bgRect = RectF()
    private val path = Path()
    private val pt = PointF()

    // ---------------- 路径点（**独立于 Snapshot**）----------------
    //
    // 4 Hz 的快照每拍都重造，路径点却是「偶尔变一次」—— 所以单独一组字段，
    // 由 revision 驱动：[setWaypoints] 只在真的变了的时候被调。
    /** 世界坐标 `x y`（每点两个）。 */
    private var wpXY = FloatArray(0)
    private var wpName = arrayOfNulls<String>(0)
    private var wpN = 0

    // ---------------- 外部命令（主线程）----------------

    /** 换一帧内容。**主线程**。 */
    fun submit(s: Snapshot) {
        snap = s
        invalidate()
    }

    /**
     * 换一组路径点（**主线程**）。`xy` 每点两个世界坐标，`names[i]` 是第 i 个的名字。
     *
     * 只放**有效且在当前图**的那些 —— 别的图在另一个坐标系里，画上去必然错位。
     * 调用方按 revision 判断要不要重建，所以这里不做比较。
     */
    fun setWaypoints(xy: FloatArray, n: Int, names: Array<String?>) {
        if (wpXY.size < n * 2) wpXY = FloatArray(n * 2)
        System.arraycopy(xy, 0, wpXY, 0, n * 2)
        if (wpName.size < n) wpName = arrayOfNulls(n)
        for (i in 0 until n) wpName[i] = names.getOrNull(i)
        wpN = n
        invalidate()
    }

    /** 清空路径点图层（**主线程**）。关掉显示开关时用。 */
    fun clearWaypoints() {
        if (wpN == 0) return
        wpN = 0
        invalidate()
    }

    // ---------------- 历史轨迹（**独立于 Snapshot**）----------------
    //
    // 同一张图以前各趟的轨迹，已经从磁盘读成**当前图坐标系**里的世界坐标。
    // 只在换了一组文件 / 变换时才调，每拍只是重画。

    /** 每条 `3n` 个 float（world x y z）。调用方每次新建数组，这里直接持有。 */
    private var hist: List<FloatArray> = emptyList()

    /** 换一组历史轨迹（**主线程**）。数组归本视图所有，调用方之后别再改。 */
    fun setHistory(traces: List<FloatArray>) {
        hist = traces
        invalidate()
    }

    /** 清空历史轨迹层（**主线程**）。关掉开关 / 没有可画的时用。 */
    fun clearHistory() {
        if (hist.isEmpty()) return
        hist = emptyList()
        invalidate()
    }

    /** 放大一档（看到的世界变小）。到头了就什么也不做。 */
    fun zoomIn() {
        if (zoomIdx == 0) return
        mPerDp = ZOOM_M_PER_DP[--zoomIdx]
        invalidate()
    }

    /** 缩小一档（看到的世界变大）。到头了就什么也不做。 */
    fun zoomOut() {
        if (zoomIdx >= ZOOM_M_PER_DP.size - 1) return
        mPerDp = ZOOM_M_PER_DP[++zoomIdx]
        invalidate()
    }

    /** 当前缩放档位（`ZOOM_M_PER_DP` 的下标）。给持久化用。 */
    fun zoomIndex(): Int = zoomIdx

    /** 恢复缩放档位。**越界的直接忽略**（`-1` = 还没存过）。 */
    fun setZoomIndex(i: Int) {
        if (i < 0 || i >= ZOOM_M_PER_DP.size || i == zoomIdx) return
        zoomIdx = i
        mPerDp = ZOOM_M_PER_DP[i]
        invalidate()
    }

    /** 切片厚度（**米，全宽**）。抽屉那个滑块调它，值已经被那边夹过，这儿再夹一次兜底。 */
    fun setSliceRange(m: Float) {
        sliceRange = m.coerceIn(SLICE_MIN, SLICE_MAX)
        invalidate()
    }

    // ---------------- 画 ----------------

    /**
     * 世界 → 小窗内的屏幕坐标。**唯一的变换**。
     *
     * 屏幕 y 向下、世界 y 向上 ⟹ 第二项取负号。
     */
    private fun worldToView(x: Float, y: Float, out: PointF) {
        out.set(width / 2f + (x - cx) * pxPerMeter, height / 2f - (y - cy) * pxPerMeter)
    }

    /** 推进切片中心。⚠️ 大跳变（起步/重置/重定位换了世界系）**直接对齐**，不然会一路滑过来。 */
    private fun updateSliceCenter(z: Float) {
        if (!zReady || abs(z - zS) > Z_SNAP) zS = z
        else zS += (z - zS) * Z_LOWPASS
        zReady = true
    }

    /**
     * 点云：**只画当前高度附近这一带**，于是画面上剩下的就是墙和家具的横切轮廓。
     *
     * 一遍过 —— 同层且在窗口内的点直接转屏幕坐标写进 scratch，最后一次 `drawPoints`。
     * 切片是单色的，所以**不需要按颜色分桶**。
     */
    private fun drawCloud(canvas: Canvas, s: Snapshot) {
        if (s.ptsN <= 0) return
        // 按这次的点数长，长过就不再缩 ⟹ 稳定之后零分配。装得下所有点，写的时候不用再判越界。
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

        val r = 6f * density
        bgRect.set(0f, 0f, width.toFloat(), height.toFloat())
        canvas.drawRoundRect(bgRect, r, r, bgPaint)
        canvas.drawRoundRect(bgRect, r, r, edgePaint)

        if (s.hint != null) {
            drawOutlinedText(canvas, s.hint, width / 2f, height / 2f + textPaint.textSize / 3f)
            return
        }

        // 中心跟着当前位置。没有位姿时**冻住上一次的**，别跳回原点。
        if (s.hasCur) {
            cx = s.curX
            cy = s.curY
            updateSliceCenter(s.curZ)
        }

        drawCloud(canvas, s)
        drawHistory(canvas)

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

        // 路径点画在轨迹之后、当前位置之前 —— 「我在哪」永远压最上层。
        drawWaypoints(canvas)

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

    /**
     * 历史轨迹：所有条共用一个 `Path`（每条一个子路径），一次画完。
     * 和实时轨迹一样只取 XY —— 小窗是**世界俯视**。
     */
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

    /**
     * 路径点：绿色菱形 + 名字。
     *
     * 只画视口里的 —— 缩小到最外那档时，地图外的点全画一遍没意义。
     * 名字放不下（互相压）时不画文字，菱形本身还在。
     */
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

    /** 黑描边 + 白填充画两遍 —— 亮暗背景上都看得清。同 [PreviewView] 那套。 */
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
