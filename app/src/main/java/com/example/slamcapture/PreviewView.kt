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
import android.util.Log
import android.view.View

/**
 * 把相机帧绘制到屏幕上。
 *
 * 数据来自现有 ImageReader 的 Y 平面，与 [CameraCapture.extractGray] 交给落盘的数组相同。
 * 不要额外挂 camera surface，那会触发 stream 重新协商并可能改变内参。
 *
 * 线程：offer 在相机线程调用，onDraw 在主线程；两者之间只传只读的 ByteArray 引用，
 * 像素展开留在主线程。
 *
 * 缓冲区是传感器原生方向（640x480 横向），绘制时顺时针旋转 90°。
 */
class PreviewView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private companion object {
        const val TAG = "PreviewView"

        const val W = CameraCapture.W
        const val H = CameraCapture.H

        /** 预览限流到约 15fps。 */
        const val MIN_INTERVAL_NS = 66_000_000L
    }

    /** [Overlay.state] 的四档，从「还无法测量」到「测量准确」。 */
    enum class MeasureState {
        /**
         * 跟踪尚未到 OK。
         * 这是 SLAM 状态而非测量状态，由 Kotlin 侧叠加到 C++ 返回的三档之上。
         */
        NO_TRACK,

        /** 十字附近没有可用的地图点。 */
        NONE,

        /** 有读数，但离十字较远或来自平面兜底。 */
        FAIR,

        /** 吸附距离很近。 */
        GOOD;

        /** 是否有可显示读数。 */
        val hasReading: Boolean get() = this == FAIR || this == GOOD
    }

    /** 已钉住的一个点；low 表示来自平面兜底，画成空心圈。 */
    data class Mark(val px: Float, val py: Float, val low: Boolean)

    /** 一段折线，四个坐标都是图像像素，label 是长度文本。 */
    data class Seg(val x0: Float, val y0: Float, val x1: Float, val y1: Float, val label: String)

    /**
     * 叠加层内容。不可变，整体替换引用，避免 onDraw 读到半新半旧的数据。
     *
     * px/py 是图像像素坐标，屏幕位置由 [imageToScreen] 换算。
     * marks 和 segs 只包含可投影的点。live 是最后一个点到十字的橡皮筋。
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

    /** 最新一帧灰度。相机线程写引用，主线程读引用；数组内容只读。 */
    @Volatile
    private var pending: ByteArray? = null

    /**
     * 是否显示画面。主线程写，相机线程读。
     *
     * offer 和 onDraw 两道检查都需要：关闭相机是异步的，清屏后仍可能有在途帧。
     */
    @Volatile
    private var active = false

    /** 上次已展开的数组；按引用比较，避免重复展开。只在主线程使用。 */
    private var drawn: ByteArray? = null

    private val argb = IntArray(W * H)

    /** 灰度到不透明 ARGB 的查表。 */
    private val lut = IntArray(256) { v ->
        (0xFF shl 24) or (v shl 16) or (v shl 8) or v
    }

    private val bmp = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888)

    /** 画面在屏幕上的落点和缩放系数，letterbox 居中。只在主线程使用。 */
    private val dst = RectF()
    private var scale = 1f

    /** 上次收帧的时刻。只在相机线程碰。 */
    private var lastAcceptNs = 0L

    /** 每绘制一帧新画面时调用，返回该帧的叠加层。主线程。 */
    var overlaySupplier: (() -> Overlay?)? = null

    /** 本帧的叠加层，由 [overlaySupplier] 在新帧到来时填充。 */
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
        fontFeatureSettings = "tnum"   // 等宽数字，读数变化时不左右跳
    }

    /** 读数底衬：画面亮暗不定，纯白字会糊掉。 */
    private val pillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = 0xB3000000.toInt()
    }
    private val pillRect = RectF()

    // ---- 多点折线，青色。与十字的白色、吸附圈的琥珀色区分 ----

    /** 折线以及低置信度标记点（空心圈）。 */
    private val markPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = 0xFF00E5FF.toInt()
    }

    /** 高置信度标记点（实心）。 */
    private val markSolidPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = 0xFF00E5FF.toInt()
    }

    /** 最后一个点到十字的橡皮筋，用虚线表示尚未确定。 */
    private val livePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f * density
        color = 0xFF00E5FF.toInt()
        pathEffect = DashPathEffect(floatArrayOf(10f * density, 8f * density), 0f)
    }

    /** 段标签字号比主读数小。 */
    private val segTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 13f * resources.displayMetrics.scaledDensity
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
        fontFeatureSettings = "tnum"
    }

    /** [imageToScreen] 的复用输出；同时需要两个点时使用不同实例。 */
    private val ptA = PointF()
    private val ptB = PointF()

    /**
     * 画面之外的附加层（标定引导框之类），画在测量叠加之后。
     * 只准用 [imageToScreen] 定位：dst/scale 重算一次就会和画面错位。
     */
    var guidePainter: ((Canvas) -> Unit)? = null

    /** 画面在屏幕上的落点，letterbox 之后的那块。没布局完成时是空矩形。 */
    fun imageRect(out: RectF) {
        out.set(dst)
    }

    /**
     * 图像像素到屏幕坐标。主线程。
     *
     * 对应 onDraw 中的 translate、rotate(90) 和 scale 三步。
     * 这个变换只在这里实现，写错会让标记与画面错位。
     */
    fun imageToScreen(px: Float, py: Float, out: PointF) {
        out.set(dst.right - py * scale, dst.top + px * scale)
    }

    /** 圆角深色底 + 白字；画面亮暗都能读。cx、baseline 是文字中心与基线。 */
    private fun drawPillText(canvas: Canvas, text: String, cx: Float, baseline: Float, p: Paint) {
        p.style = Paint.Style.FILL
        p.color = Color.WHITE
        val tw = p.measureText(text)
        val padH = 10f * density
        val padV = 5f * density
        val fm = p.fontMetrics
        val h = fm.descent - fm.ascent + 2 * padV
        val top = baseline + fm.ascent - padV
        pillRect.set(cx - tw / 2f - padH, top, cx + tw / 2f + padH, top + h)
        canvas.drawRoundRect(pillRect, h / 2f, h / 2f, pillPaint)
        canvas.drawText(text, cx - tw / 2f, baseline, p)
    }

    /**
     * 开关画面，主线程调用。
     * 关闭时必须清屏并请求重绘，否则上一帧会留在屏幕上。
     */
    fun setActive(on: Boolean) {
        active = on
        if (on) {
            // 重置限流时刻，避免丢掉重开后的第一帧。
            lastAcceptNs = 0L
        } else {
            pending = null
            drawn = null
            invalidate()
        }
    }

    /**
     * 送入一帧，相机线程调用，不能阻塞。这里只换引用并请求重绘，像素展开在 [onDraw]。
     * 忙时直接丢弃。
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
        // 640x480 横向缓冲旋转 90° 后占 480x640，按比例居中显示，不拉伸。
        val imgW = H.toFloat()
        val imgH = W.toFloat()
        scale = minOf(w / imgW, h / imgH)
        val dw = imgW * scale
        val dh = imgH * scale
        dst.set((w - dw) / 2f, (h - dh) / 2f, (w + dw) / 2f, (h + dh) / 2f)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        // 第二道检查：清屏后到达的在途帧不绘制。
        if (!active) {
            drawn = null
            return
        }
        val g = pending
        // 没新帧就重画上一帧：换步时单独 invalidate() 也要能出图。
        if (g == null && drawn == null) return

        if (g != null && g !== drawn) {
            for (i in argb.indices) argb[i] = lut[g[i].toInt() and 0xFF]
            bmp.setPixels(argb, 0, W, 0, 0, W, H)   // 复用缓冲，不分配
            drawn = g
            // 叠加层随新帧更新，不随 ticker 更新。
            overlay = overlaySupplier?.invoke()
        }

        canvas.save()
        canvas.translate(dst.right, dst.top)
        canvas.rotate(90f)
        canvas.scale(scale, scale)
        canvas.drawBitmap(bmp, 0f, 0f, null)
        canvas.restore()

        overlay?.let { drawMeasure(canvas, it) }

        // 引导层最后画，压在画面和测量叠加之上。
        guidePainter?.let { painter ->
            try {
                painter(canvas)
            } catch (e: Throwable) {
                Log.e(TAG, "引导层绘制失败，已停用", e)
                guidePainter = null
            }
        }
    }

    /** 绘制十字、吸附点和读数；只在有画面时调用。 */
    private fun drawMeasure(canvas: Canvas, ov: Overlay) {
        // ---- 已钉住的折线：先画线，标记点和十字在上层 ----
        for (s in ov.segs) drawSeg(canvas, s, markPaint)
        ov.live?.let { drawSeg(canvas, it, livePaint) }  // 尚未确定的橡皮筋
        for (m in ov.marks) {
            imageToScreen(m.px, m.py, ptA)
            canvas.drawCircle(ptA.x, ptA.y, 7f * density,
                              if (m.low) markPaint else markSolidPaint)
        }

        // ---- 十字 ----
        // 图像中心就是控件中心：letterbox 居中，旋转也绕中心。
        val cx = width / 2f
        val cy = height / 2f
        val r = 14f * density
        canvas.drawLine(cx - r, cy, cx + r, cy, crossPaint)
        canvas.drawLine(cx, cy - r, cx, cy + r, crossPaint)

        // 画出可能吸附的位置，让用户提前知道会钉到哪个点。
        if (ov.state.hasReading && ov.px >= 0f) {
            imageToScreen(ov.px, ov.py, ptA)
            canvas.drawCircle(ptA.x, ptA.y, 6f * density, snapPaint)
        }

        val label = when (ov.state) {
            MeasureState.NO_TRACK -> "等待跟踪…"
            MeasureState.NONE -> "对准有纹理的地方"
            MeasureState.FAIR -> "≈ %.2f m（不太准）".format(ov.distM)
            MeasureState.GOOD -> "%.2f m".format(ov.distM)
        }
        drawPillText(canvas, label, cx, cy + r + textSize + 6f * density, textPaint)
    }

    /** 绘制一段折线及其中点的长度标签。 */
    private fun drawSeg(canvas: Canvas, s: Seg, line: Paint) {
        imageToScreen(s.x0, s.y0, ptA)
        imageToScreen(s.x1, s.y1, ptB)
        canvas.drawLine(ptA.x, ptA.y, ptB.x, ptB.y, line)
        drawPillText(canvas, s.label,
                     (ptA.x + ptB.x) / 2f,
                     (ptA.y + ptB.y) / 2f + segTextPaint.textSize * 0.35f,
                     segTextPaint)
    }
}
