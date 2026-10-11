package com.example.slamcapture

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.PointF
import android.graphics.RectF
import android.graphics.Typeface
import android.util.AttributeSet
import android.view.View

/**
 * 把相机帧画到屏幕上。
 *
 * 数据来自**已经在跑的那路 ImageReader 的 Y 平面**——就是 [CameraCapture.extractGray]
 * 交给落盘的那个数组，逐字节相同。**不要额外挂 camera surface**：多一路输出会让
 * 相机重新协商 stream 配置，可能换 sensor mode 或裁切，fx 就变了，整个标定作废。
 *
 * ------------------------------------------------------------------
 * 线程：`offer` 在**相机线程**上被调用，[onDraw] 在主线程。两者之间只传一个
 * **只读的 ByteArray 引用**（每帧新分配、之后没有任何人改它），所以不需要锁。
 *
 * 这一点是有意的：相机帧本来就比屏幕刷新快，展开一次够画好几遍。把展开放到
 * 主线程，相机线程就只做"换个引用 + 请求重绘"两个动作，**负载和没有预览时一样**。
 * 否则预览会拖慢采集，改变录到的帧集合，甚至让 AE/AWB 停止收敛（管线被堵住）。
 * ------------------------------------------------------------------
 *
 * **方向**：缓冲区永远是传感器原生方向（640x480 横着的），不管 Activity 锁哪个
 * 方向、手机怎么拿。实测这台上 marker 的"向下"方向指向画面 +x，所以要**顺时针
 * 转 90°** 画出去才是正的。旋转交给 Canvas —— 像素循环那边就能保持线性写入，
 * 否则每帧 30 万次跨 2.5KB 步长的写全是 cache miss。
 */
class PreviewView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private companion object {
        const val W = CameraCapture.W
        const val H = CameraCapture.H

        /** 预览限流到 ~15fps。展开 30 万个像素是主线程的活，不必跟满 30fps。 */
        const val MIN_INTERVAL_NS = 66_000_000L
    }

    /** [Overlay.state] 的四档，从「还没法测」到「测得很准」。 */
    enum class MeasureState {
        /**
         * 跟踪还没到 `OK`。
         *
         * ⚠️ 这是 **SLAM 的状态**，不是测量的状态 —— C++ 那边只回后面三档，这一档由 Kotlin
         * 按跟踪状态叠上去。不分开的话，「还没准备好」会被显示成「对着有纹理的地方」，
         * 而用户会去擦镜头（STAGE6 §2.7：沉默和「测不了」必须能区分）。
         */
        NO_TRACK,

        /** 十字附近没有可用的地图点。 */
        NONE,

        /** 有读数，但离十字偏远 / 是平面兜底出来的。 */
        FAIR,

        /** 吸附得很近。 */
        GOOD;

        /** 有数可读 —— 决定要不要画小圈、显数字。 */
        val hasReading: Boolean get() = this == FAIR || this == GOOD
    }

    /** 已经钉住的一个点。`low` = 它当初是平面兜底来的（不太准），画成空心圈。 */
    data class Mark(val px: Float, val py: Float, val low: Boolean)

    /** 一段折线。四个坐标都是**图像像素**，`label` 是它的长度。 */
    data class Seg(val x0: Float, val y0: Float, val x1: Float, val y1: Float, val label: String)

    /**
     * 十字那一层要画的东西。**不可变**，主线程整体换引用（[overlay] 是 `@Volatile`）——
     * 分开写几个 float 的话，[onDraw] 会读到半新半旧的一帧。
     *
     * `px`/`py` 是**图像像素**坐标（不是屏幕），屏幕位置由 [toScreen] 反推。
     * `marks`/`segs` 只放**投得出来**的点 —— 相机后方和屏幕外的在这里已经被滤掉了。
     * `live` 是「上一个点 → 十字」那段还在拉的橡皮筋，画成虚线。
     */
    data class Overlay(
        val state: MeasureState,
        val distM: Float,
        val px: Float,
        val py: Float,
        val marks: List<Mark> = emptyList(),
        val segs: List<Seg> = emptyList(),
        val live: Seg? = null,
    )

    /** 最新一帧的灰度。相机线程写引用，主线程读引用。数组本身只读。 */
    @Volatile
    private var pending: ByteArray? = null

    /**
     * 现在该不该出画面。主线程写，相机线程读。
     *
     * 两层闸都要有，缺一不可：
     *   - `offer()` 里挡掉新来的帧；
     *   - `onDraw()` 里挡掉**已经在路上**的帧。
     *
     * 因为 [CameraCapture.stop] 是 post 到相机线程的（异步），而 [setActive] 是主线程
     * 同步跑的，「清完屏之后还会来几十到几百毫秒的帧」是**必然**，不是小概率。
     * 只有第一层的话，刚清完屏的下一帧就把它又画回来了。
     */
    @Volatile
    private var active = false

    /** 上一次已经展开过的那个数组（按引用比，避免同一帧重复展开）。只在主线程碰。 */
    private var drawn: ByteArray? = null

    private val argb = IntArray(W * H)

    /** 灰度 → 不透明 ARGB 的查表。比每像素算一遍移位快得多。 */
    private val lut = IntArray(256) { v ->
        (0xFF shl 24) or (v shl 16) or (v shl 8) or v
    }

    private val bmp = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888)

    /** 画面在屏幕上的落点（letterbox 居中），以及缩放系数。只在主线程碰。 */
    private val dst = RectF()
    private var scale = 1f

    /** 上次收帧的时刻。只在相机线程碰。 */
    private var lastAcceptNs = 0L

    /**
     * **每画新的一帧**时调一次，拿这一刻要画的叠加层。主线程。
     */
    var overlaySupplier: (() -> Overlay?)? = null

    /** 本帧要画的叠加层，由 [overlaySupplier] 在新帧上填。 */
    private var overlay: Overlay? = null

    private val density = resources.displayMetrics.density
    private val textSize = 16f * resources.displayMetrics.scaledDensity

    private val crossPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = Color.WHITE
    }
    private val snapPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = 0xFFFFC107.toInt()   // 琥珀色，和十字的白区分开
    }
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = this@PreviewView.textSize
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
    }

    // ---- 多点折线。青色 = 「我自己画的东西」，和十字的白、吸附圈的琥珀区分开 ----

    /** 折线本身，以及**低置信度**的标记点（描边 = 空心圈）。 */
    private val markPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = 0xFF00E5FF.toInt()
    }

    /** 高置信度的标记点（实心）。 */
    private val markSolidPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = 0xFF00E5FF.toInt()
    }

    /** 「上一个点 → 十字」那段橡皮筋，虚线以表明**还没定**。 */
    private val livePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = 0xFF00E5FF.toInt()
        pathEffect = DashPathEffect(floatArrayOf(10f * density, 8f * density), 0f)
    }

    /** 段标签比十字旁边那个读数小一号 —— 它会有好几条，不能抢主读数的位置。 */
    private val segTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 13f * resources.displayMetrics.scaledDensity
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
    }

    /** [toScreen] 的复用输出。**不能同时拿两个** —— 要用两个点的地方得传两个不同的。 */
    private val ptA = PointF()
    private val ptB = PointF()

    /**
     * 图像像素 → 控件的屏幕坐标。
     *
     * 把 [onDraw] 里那三步代进去：`translate(dst.right, dst.top)` → `rotate(90)` → `scale(s)`
     * ⟹ 先缩放成 `(px·s, py·s)`，绕原点转 90° 变成 `(−py·s, px·s)`，再平移。
     *
     * ⚠️ **只有这一处做这个变换。** 写错的话标记会和场景错位，而**看起来像「SLAM 不准」**。
     */
    private fun toScreen(px: Float, py: Float, out: PointF) {
        out.set(dst.right - py * scale, dst.top + px * scale)
    }

    /** 黑描边 + 白填充画两遍 —— 亮背景上也看得清。别用 `setShadowLayer`（硬件加速下不一定生效）。 */
    private fun drawOutlinedText(canvas: Canvas, text: String, x: Float, y: Float, p: Paint) {
        p.style = Paint.Style.STROKE
        p.strokeWidth = 3f * density
        p.color = Color.BLACK
        canvas.drawText(text, x, y, p)
        p.style = Paint.Style.FILL
        p.color = Color.WHITE
        canvas.drawText(text, x, y, p)
    }

    /**
     * 开/关画面。主线程调用。
     *
     * 关的时候**必须显式清屏**：`onDraw` 在没帧时只是 `return`，而"不画"不等于
     * "擦掉"——上一帧会原封不动冻在屏幕上。光把 [pending] 置 null 是擦不掉的。
     */
    fun setActive(on: Boolean) {
        active = on
        if (on) {
            // 别让上一轮留下的限流时刻吃掉重开后的第一帧。
            lastAcceptNs = 0L
        } else {
            pending = null
            drawn = null
            invalidate()
        }
    }

    /**
     * 送一帧进来。**相机线程调用**，所以这里不能有任何耗时或阻塞的操作——
     * 主体只是换个引用 + 请求重绘，真正的展开留到 [onDraw]。
     *
     * 忙就丢：丢掉一帧预览没有任何代价。
     */
    fun offer(gray: ByteArray, tsNs: Long) {
        if (!active) return
        val now = System.nanoTime()
        if (now - lastAcceptNs < MIN_INTERVAL_NS) return
        lastAcceptNs = now
        pending = gray
        postInvalidateOnAnimation()
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        // 缓冲区是 640x480 横着的，顺时针转 90° 画出去之后占 480x640。
        // 按比例 letterbox 居中，**绝不拉伸**——拉伸会让人误判画面。
        val imgW = H.toFloat()
        val imgH = W.toFloat()
        scale = minOf(w / imgW, h / imgH)
        val dw = imgW * scale
        val dh = imgH * scale
        dst.set((w - dw) / 2f, (h - dh) / 2f, (w + dw) / 2f, (h + dh) / 2f)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        // 第二层闸：清屏之后即使还有帧溜进来也不画，否则刚清掉的画面会被画回来。
        val g = if (active) pending else null
        if (g == null) {
            drawn = null
            return
        }

        if (g !== drawn) {
            for (i in argb.indices) argb[i] = lut[g[i].toInt() and 0xFF]
            bmp.setPixels(argb, 0, W, 0, 0, W, H)   // 复用缓冲，不分配
            drawn = g
            // 叠加层跟着**新帧**更新，不跟着 ticker —— 见 overlaySupplier 上面那段。
            overlay = overlaySupplier?.invoke()
        }

        canvas.save()
        canvas.translate(dst.right, dst.top)
        canvas.rotate(90f)
        canvas.scale(scale, scale)
        canvas.drawBitmap(bmp, 0f, 0f, null)
        canvas.restore()

        overlay?.let { drawMeasure(canvas, it) }
    }

    /**
     * 画十字 + 吸附点 + 读数。
     *
     * 只在有画面的时候调 —— 跟着 [onDraw] 上面那道早退走，所以**没启动 SLAM 就没有十字**。
     */
    private fun drawMeasure(canvas: Canvas, ov: Overlay) {
        // ---- 已钉住的折线。先画线（在最下面），标记点和十字压在上面 ----
        for (s in ov.segs) drawSeg(canvas, s, markPaint)
        ov.live?.let { drawSeg(canvas, it, livePaint) }  // 还在拉的橡皮筋，虚线
        for (m in ov.marks) {
            toScreen(m.px, m.py, ptA)
            canvas.drawCircle(ptA.x, ptA.y, 7f * density,
                              if (m.low) markPaint else markSolidPaint)
        }

        // ---- 十字 ----
        // ⚠️ **不需要任何变换**：letterbox 是居中的，而 90° 旋转绕的就是中心，
        // 所以图像中心就是控件中心。STAGE6 §3 那一整节的存在就是为了这一句。
        val cx = width / 2f
        val cy = height / 2f
        val r = 14f * density
        canvas.drawLine(cx - r, cy, cx + r, cy, crossPaint)
        canvas.drawLine(cx, cy - r, cx, cy + r, crossPaint)

        // 会吸到哪儿。⚠️ STAGE6 §2.5：**必须画出来** —— 否则按下去之前你不知道会定到哪个点。
        if (ov.state.hasReading && ov.px >= 0f) {
            toScreen(ov.px, ov.py, ptA)
            canvas.drawCircle(ptA.x, ptA.y, 6f * density, snapPaint)
        }

        val label = when (ov.state) {
            MeasureState.NO_TRACK -> "等待跟踪…"
            MeasureState.NONE -> "对准有纹理的地方"
            MeasureState.FAIR -> "≈ %.2f m（不太准）".format(ov.distM)
            MeasureState.GOOD -> "%.2f m".format(ov.distM)
        }
        drawOutlinedText(canvas, label, cx - textPaint.measureText(label) / 2f,
                         cy + r + textSize, textPaint)
    }

    /** 一段折线 + 挂在中点的长度标签。 */
    private fun drawSeg(canvas: Canvas, s: Seg, line: Paint) {
        toScreen(s.x0, s.y0, ptA)
        toScreen(s.x1, s.y1, ptB)
        canvas.drawLine(ptA.x, ptA.y, ptB.x, ptB.y, line)
        drawOutlinedText(canvas, s.label,
                         (ptA.x + ptB.x) / 2f - segTextPaint.measureText(s.label) / 2f,
                         (ptA.y + ptB.y) / 2f, segTextPaint)
    }
}
