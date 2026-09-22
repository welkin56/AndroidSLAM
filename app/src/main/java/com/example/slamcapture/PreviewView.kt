package com.example.slamcapture

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.RectF
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
    }
}
