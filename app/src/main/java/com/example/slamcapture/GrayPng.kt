package com.example.slamcapture

import java.io.ByteArrayOutputStream
import java.io.OutputStream
import java.util.zip.CRC32
import java.util.zip.Deflater

/**
 * 极简 8 位灰度 PNG 编码器，手写实现，不走 Bitmap。
 *
 * Bitmap 没有 1 字节灰度格式：ALPHA_8 不被 Skia 的 PNG 编码器支持，
 * ARGB_8888 又是 4 字节/像素。直接写灰度可以减少 4 倍数据量。
 *
 * PNG 结构（灰度 8 位，无隔行）：
 *   签名(8 字节) + IHDR + IDAT + IEND
 * 每个 chunk = 长度(4) + 类型(4) + 数据 + CRC32(4)，均为大端。
 * IDAT 是 zlib 流，内容逐行为：[滤波器字节] [该行像素]，滤波器固定为 0。
 */
object GrayPng {

    private val SIGNATURE = byteArrayOf(
        0x89.toByte(), 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A
    )

    /**
     * 把 [gray]（长度至少 width*height，逐行排列）编码成灰度 PNG 写入 [out]。
     * 出错时抛异常，调用方需要处理。
     */
    fun encode(gray: ByteArray, width: Int, height: Int, out: OutputStream) {
        require(width > 0 && height > 0) { "尺寸非法 ${width}x$height" }
        require(gray.size >= width * height) {
            "灰度缓冲太小：${gray.size} < ${width * height}"
        }

        out.write(SIGNATURE)
        writeChunk(out, "IHDR", ihdr(width, height))
        writeChunk(out, "IDAT", idat(gray, width, height))
        writeChunk(out, "IEND", ByteArray(0))
        out.flush()
    }

    /** IHDR：宽、高、位深 8、色彩类型 0(灰度)、压缩 0、滤波 0、隔行 0 */
    private fun ihdr(w: Int, h: Int): ByteArray {
        val b = ByteArray(13)
        putInt(b, 0, w)
        putInt(b, 4, h)
        b[8] = 8   // bit depth
        b[9] = 0   // 色彩类型 0：灰度
        b[10] = 0  // compression method = deflate
        b[11] = 0  // filter method = 0
        b[12] = 0  // interlace = none
        return b
    }

    private fun idat(gray: ByteArray, w: Int, h: Int): ByteArray {
        // ① 组原始数据：每行前面插一个滤波器字节 0
        val raw = ByteArray(h * (1 + w))
        for (y in 0 until h) {
            val dst = y * (1 + w)
            raw[dst] = 0
            System.arraycopy(gray, y * w, raw, dst + 1, w)
        }

        // ② zlib 压缩。Deflater 默认输出带 zlib 头和 adler32 尾的流，符合 PNG 要求。
        //    使用 BEST_SPEED，优先保证帧率。
        val deflater = Deflater(Deflater.BEST_SPEED)
        try {
            deflater.setInput(raw)
            deflater.finish()

            val buf = ByteArray(64 * 1024)
            val out = ByteArrayOutputStream(raw.size / 4 + 1024)
            while (!deflater.finished()) {
                val n = deflater.deflate(buf)
                if (n > 0) out.write(buf, 0, n)
                else if (deflater.needsInput()) break  // 防止死循环
            }
            return out.toByteArray()
        } finally {
            deflater.end()
        }
    }

    /** 写一个 chunk：长度 + 类型 + 数据 + CRC32(类型+数据) */
    private fun writeChunk(out: OutputStream, type: String, data: ByteArray) {
        val len = ByteArray(4); putInt(len, 0, data.size)
        val typeBytes = type.toByteArray(Charsets.US_ASCII)

        out.write(len)
        out.write(typeBytes)
        out.write(data)

        val crc = CRC32()
        crc.update(typeBytes)
        crc.update(data)
        val crcBytes = ByteArray(4)
        putInt(crcBytes, 0, crc.value.toInt())   // 取低 32 位
        out.write(crcBytes)
    }

    /** 大端写入 32 位整数 */
    private fun putInt(b: ByteArray, off: Int, v: Int) {
        b[off] = (v ushr 24).toByte()
        b[off + 1] = (v ushr 16).toByte()
        b[off + 2] = (v ushr 8).toByte()
        b[off + 3] = v.toByte()
    }
}
