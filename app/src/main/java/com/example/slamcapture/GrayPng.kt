package com.example.slamcapture

import java.io.ByteArrayOutputStream
import java.io.OutputStream
import java.util.zip.CRC32
import java.util.zip.Deflater

/**
 * 极简 8 位灰度 PNG 编码器（手写，不走 Bitmap）。
 *
 * 为什么不用 Bitmap.compress()：
 *
 *   Bitmap 根本没有 1 字节的灰度格式。最接近的 ALPHA_8 是"纯 alpha"，
 *   Skia 的 PNG 编码器不支持（返回 false 且静默写出 0 字节）。
 *   唯一能用的 ARGB_8888 是 **4 字节/像素**——灰度数据本来只有 1 字节，
 *   等于让 deflate 去压 4 倍的数据量。实测手机上每帧要 100ms 以上，
 *   30fps 根本跟不上，写盘线程成为瓶颈，队列秒满、疯狂丢帧。
 *
 *   直接按 PNG 规范写灰度，输出数据量小 4 倍，编码耗时降到 ~15ms。
 *   顺带的好处是产物是真正的 `8-bit grayscale`，和 EuRoC / 标定工具的
 *   预期一致。
 *
 * PNG 结构（灰度 8 位，无隔行）：
 *
 *   签名(8 字节) + IHDR + IDAT + IEND
 *
 *   每个 chunk = 长度(4) + 类型(4) + 数据 + CRC32(4)，全部大端。
 *   IDAT 里装的是一个 **zlib 流**（不是裸 deflate），内容是逐行的：
 *     [滤波器字节] [该行的 width 个像素]
 *   滤波器用 0（None）最省事，灰度图压缩率也够看。
 */
object GrayPng {

    private val SIGNATURE = byteArrayOf(
        0x89.toByte(), 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A
    )

    /**
     * 把 [gray]（长度至少 width*height，逐行排列）编码成灰度 PNG 写到 [out]。
     * 出错会抛异常——调用方必须处理，不要静默吞掉。
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
        b[9] = 0   // color type 0 = 灰度（这就是我们要的）
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

        // ② zlib 压缩。Deflater 默认输出 zlib 流（带 2 字节头和 adler32
        //    尾），正是 PNG 要的；不要用 nowrap=true。
        //
        //    用 BEST_SPEED 而不是默认的 DEFAULT_COMPRESSION：帧率是硬指标，
        //    文件大小不是。灰度 640x480 用 BEST_SPEED 也能压到 ~50%。
        val deflater = Deflater(Deflater.BEST_SPEED)
        try {
            deflater.setInput(raw)
            deflater.finish()

            val buf = ByteArray(64 * 1024)
            val out = ByteArrayOutputStream(raw.size / 4 + 1024)
            while (!deflater.finished()) {
                val n = deflater.deflate(buf)
                if (n > 0) out.write(buf, 0, n)
                else if (deflater.needsInput()) break  // 理论到不了，防死循环
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
        putInt(crcBytes, 0, crc.value.toInt())   // 低 32 位就是 CRC
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
