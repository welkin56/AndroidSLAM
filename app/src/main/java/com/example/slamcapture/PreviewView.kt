package com.example.slamcapture

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
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

    /**
     * 十字那一层要画的东西。**不可变**，主线程整体换引用（[overlay] 是 `@Volatile`）——
     * 分开写几个 float 的话，[onDraw] 会读到半新半旧的一帧。
     *
     * `px`/`py` 是**图像像素**坐标（不是屏幕），屏幕位置由 [onDraw] 里那套变换反推。
     */
    data class Overlay(val state: MeasureState, val distM: Float, val px: Float, val py: Float)

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

    /** 测量叠加层的内容。主线程写（ticker），主线程读（[onDraw]）。 */
    @Volatile
    var overlay: Overlay? = null

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
        // ⚠️ **不需要任何变换**：letterbox 是居中的，而 90° 旋转绕的就是中心，
        // 所以图像中心就是控件中心。STAGE6 §3 那一整节的存在就是为了这一句。
        val cx = width / 2f
        val cy = height / 2f
        val r = 14f * density
        canvas.drawLine(cx - r, cy, cx + r, cy, crossPaint)
        canvas.drawLine(cx, cy - r, cx, cy + r, crossPaint)

        // 会吸到哪儿。⚠️ STAGE6 §2.5：**必须画出来** —— 否则按下去之前你不知道会定到哪个点。
        if (ov.state.hasReading && ov.px >= 0f) {
            // 图像像素 (px, py) → 屏幕。把上面三步代进去：
            //   translate(dst.right, dst.top) → rotate(90) → scale(s)
            //   (px, py) 先缩放成 (px·s, py·s)，绕原点转 90° 变成 (-py·s, px·s)，再平移。
            canvas.drawCircle(
                dst.right - ov.py * scale, dst.top + ov.px * scale, 6f * density, snapPaint
            )
        }

        val label = when (ov.state) {
            MeasureState.NO_TRACK -> "等待跟踪…"
            MeasureState.NONE -> "对准有纹理的地方"
            MeasureState.FAIR -> "≈ %.2f m（不太准）".format(ov.distM)
            MeasureState.GOOD -> "%.2f m".format(ov.distM)
        }

        // 画两遍：先黑描边再白填充，亮背景上也看得清。
        // ⚠️ **不用 `setShadowLayer`** —— 硬件加速下它不一定生效，而且要给这个 View
        // 单独关掉硬件加速，代价比多画一遍大。
        val tx = cx - textPaint.measureText(label) / 2f
        val ty = cy + r + textSize
        textPaint.style = Paint.Style.STROKE
        textPaint.strokeWidth = 3f * density
        textPaint.color = Color.BLACK
        canvas.drawText(label, tx, ty, textPaint)
        textPaint.style = Paint.Style.FILL
        textPaint.color = Color.WHITE
        canvas.drawText(label, tx, ty, textPaint)
    }
}
