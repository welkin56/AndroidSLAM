package com.example.slamcapture

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View

/**
 * 全屏棋盘格标定板：(cols+1)×(rows+1) 个方格，四周留一格白边 ——
 * 检测器要靠白边把板子和背景分开。
 *
 * 方格的实际物理尺寸不影响内参，所以屏幕大小随意；但每一格必须是正方的，
 * 屏幕像素是方的，画出来就满足。
 */
class PatternView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    /** 内角点数（不是方格数）。 */
    var cols = 6
    var rows = 4

    private val paint = Paint()

    /** 在 [w]×[h] 的矩形里居中画一块板。 */
    private fun drawBoard(canvas: Canvas, left: Float, top: Float, side: Float) {
        val sqW = cols + 1
        val sqH = rows + 1
        canvas.drawColor(Color.WHITE)
        paint.color = Color.BLACK
        for (r in 0 until sqH) {
            for (c in 0 until sqW) {
                if ((r + c) % 2 == 0) continue
                canvas.drawRect(
                    left + c * side, top + r * side,
                    left + (c + 1) * side, top + (r + 1) * side, paint
                )
            }
        }
    }

    override fun onDraw(canvas: Canvas) {
        if (width == 0 || height == 0) return
        val sqW = cols + 1
        val sqH = rows + 1
        val side = minOf(width.toFloat() / sqW, height.toFloat() / sqH) * 0.92f
        val left = (width - sqW * side) / 2f
        val top = (height - sqH * side) / 2f
        drawBoard(canvas, left, top, side)
    }

    /** 导出可以直接分享/打印的图（每边多留一格白边）。 */
    fun render(widthPx: Int = 2000): Bitmap {
        val sqW = cols + 1
        val sqH = rows + 1
        val side = widthPx.toFloat() / (sqW + 2)
        val w = (sqW * side + 2 * side).toInt()
        val h = (sqH * side + 2 * side).toInt()
        val bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        drawBoard(Canvas(bmp), side, side, side)
        return bmp
    }
}
