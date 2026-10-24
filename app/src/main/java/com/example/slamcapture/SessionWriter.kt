package com.example.slamcapture

import android.util.Log
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.atomic.AtomicLong

/**
 * 把采集到的帧和 IMU 样本按 EuRoC 格式落盘。
 *
 * 目录布局：
 *   sessionDir/cam0/data/<时间戳ns>.png
 *   sessionDir/cam0/data.csv
 *   sessionDir/imu0/data.csv
 */
class SessionWriter(val sessionDir: File) {

    companion object {
        private const val TAG = "SessionWriter"
        // 队列深度，64 帧约 2 秒；在崩溃丢数据和偶发卡顿之间取平衡。
        private const val FRAME_QUEUE_SIZE = 64
        private const val IMU_QUEUE_SIZE = 8192
    }

    class FrameItem(val tsNs: Long, val gray: ByteArray, val w: Int, val h: Int)

    class ImuItem(
        val tsNs: Long,
        val wx: Float, val wy: Float, val wz: Float,
        val ax: Float, val ay: Float, val az: Float
    )

    private val camDir = File(sessionDir, "cam0/data")
    private val imuDir = File(sessionDir, "imu0")

    private val frameQueue = ArrayBlockingQueue<FrameItem>(FRAME_QUEUE_SIZE)
    private val imuQueue = ArrayBlockingQueue<ImuItem>(IMU_QUEUE_SIZE)

    @Volatile private var running = false
    private var thread: Thread? = null

    private lateinit var camCsv: BufferedWriter
    private lateinit var imuCsv: BufferedWriter

    val framesWritten = AtomicLong()
    val framesDropped = AtomicLong()
    val framesFailed = AtomicLong()
    val imuWritten = AtomicLong()
    val imuDropped = AtomicLong()

    /** IMU 配对时加速度与陀螺仪时间戳的最大偏差（纳秒）。 */
    val imuMaxPairGapNs = AtomicLong()

    /** 配对偏差累计值与次数，用于计算均值。 */
    private val imuGapSumNs = AtomicLong()
    private val imuGapCount = AtomicLong()

    /** 配对偏差超过 5 ms 的次数。比最大值更能反映是否存在系统性问题。 */
    val imuGapOver5ms = AtomicLong()

    fun recordPairGap(gapNs: Long) {
        imuGapSumNs.addAndGet(gapNs)
        imuGapCount.incrementAndGet()
        imuMaxPairGapNs.updateAndGet { maxOf(it, gapNs) }
        if (gapNs > 5_000_000L) imuGapOver5ms.incrementAndGet()
    }

    /** 配对偏差均值（毫秒）。 */
    fun imuMeanPairGapMs(): Double {
        val n = imuGapCount.get()
        return if (n > 0) imuGapSumNs.get().toDouble() / n / 1e6 else 0.0
    }

    var startedAtNs: Long = 0
        private set

    /** 这一段是否是强制开始：对焦未收敛或 IMU 尚未出数。会写入 meta.json。 */
    @Volatile
    var forced = false

    /** 这一段是否在 SLAM 运行时录制。两种来源格式相同，需要额外标记以便区分。 */
    @Volatile
    var withSlam = false

    /** 第一帧落盘的时刻；0 表示没有写入。只由 writer 线程写。 */
    @Volatile
    var firstFrameTsNs = 0L
        private set

    /**
     * 第一条 IMU 落盘的时刻；0 表示没有写入。只由 writer 线程写。
     * 与 [firstFrameTsNs] 比较可得到两个流的实际覆盖关系，正常情况下 IMU 早约 300 ms。
     */
    @Volatile
    var firstImuTsNs = 0L
        private set

    /** 建文件失败的原因，正常为 null。 */
    @Volatile
    var ioError: String? = null
        private set

    /** writer 线程是否仍在运行。stop() 的 join 超时后不能再删除目录。 */
    val threadAlive: Boolean get() = thread?.isAlive == true

    /**
     * 建文件并启动 writer 线程。
     * IO 异常在这里处理，不抛给主线程；失败时 [ioError] 非 null。
     */
    fun start() {
        try {
            camDir.mkdirs()
            imuDir.mkdirs()

            camCsv = BufferedWriter(OutputStreamWriter(FileOutputStream(File(sessionDir, "cam0/data.csv"))))
            imuCsv = BufferedWriter(OutputStreamWriter(FileOutputStream(File(imuDir, "data.csv"))))

            // EuRoC 格式的注释行。ORB-SLAM3 会跳过以 '#' 开头的行。
            camCsv.write("#timestamp [ns],filename\n")
            imuCsv.write(
                "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1]," +
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
            )
        } catch (e: Exception) {
            Log.e(TAG, "建 session 文件失败（磁盘满或外部存储不可写）", e)
            ioError = e.message ?: e.javaClass.simpleName
            closeFiles()
            return
        }

        startedAtNs = System.nanoTime()
        running = true
        thread = Thread({ loop() }, "session-writer").also { it.start() }
    }

    /**
     * 采集线程调用。队列满时丢帧并计数，不阻塞采集。
     * 拆除期间送来的在途帧会因 running 为 false 而计入丢弃。
     */
    fun offerFrame(item: FrameItem) {
        if (!running || !frameQueue.offer(item)) framesDropped.incrementAndGet()
    }

    /** 与 [offerFrame] 相同的丢弃计数规则。 */
    fun offerImu(item: ImuItem) {
        if (!running || !imuQueue.offer(item)) imuDropped.incrementAndGet()
    }

    private fun loop() {
        while (running || frameQueue.isNotEmpty() || imuQueue.isNotEmpty()) {
            var didWork = false

            // IMU 频率高（~200Hz），批量排空
            var n = 0
            while (n < 256) {
                val item = imuQueue.poll() ?: break
                writeImu(item)
                n++
                didWork = true
            }

            frameQueue.poll()?.let {
                writeFrame(it)
                didWork = true
            }

            if (!didWork) Thread.sleep(2)
        }
        closeQuietly()
    }

    private fun writeFrame(f: FrameItem) {
        try {
            val name = "${f.tsNs}.png"
            val file = File(camDir, name)

            // 直接写灰度 PNG，无法使用 Bitmap 的灰度编码，见 GrayPng.kt。
            FileOutputStream(file).use { out ->
                GrayPng.encode(f.gray, f.w, f.h, out)
            }

            // 磁盘写满或编码异常可能留下空文件；计入失败，不写入 CSV。
            if (file.length() == 0L) {
                file.delete()
                framesFailed.incrementAndGet()
                Log.e(TAG, "PNG 写出为空 ts=${f.tsNs}（已删除空文件）")
                return
            }

            camCsv.write("${f.tsNs},$name\n")
            val written = framesWritten.incrementAndGet()
            if (firstFrameTsNs == 0L) firstFrameTsNs = f.tsNs
            if (written % 30 == 0L) camCsv.flush()  // 每秒刷一次
        } catch (e: Exception) {
            framesFailed.incrementAndGet()
            Log.e(TAG, "写帧失败 ts=${f.tsNs}", e)
        }
    }

    private fun writeImu(i: ImuItem) {
        try {
            imuCsv.write("${i.tsNs},${i.wx},${i.wy},${i.wz},${i.ax},${i.ay},${i.az}\n")
            val written = imuWritten.incrementAndGet()
            if (firstImuTsNs == 0L) firstImuTsNs = i.tsNs
            if (written % 200 == 0L) imuCsv.flush()
        } catch (e: Exception) {
            Log.e(TAG, "写 IMU 失败", e)
        }
    }

    /** 关闭两个 CSV；允许部分文件尚未成功打开。 */
    private fun closeFiles() {
        try { if (::camCsv.isInitialized) { camCsv.flush(); camCsv.close() } } catch (_: Exception) {}
        try { if (::imuCsv.isInitialized) { imuCsv.flush(); imuCsv.close() } } catch (_: Exception) {}
    }

    private fun closeQuietly() {
        closeFiles()

        // 写入元数据
        try {
            val elapsedS = (System.nanoTime() - startedAtNs) / 1e9
            File(sessionDir, "meta.json").writeText(
                """
                {
                  "duration_s": $elapsedS,
                  "forced": $forced,
                  "with_slam": $withSlam,
                  "first_frame_ts_ns": $firstFrameTsNs,
                  "first_imu_ts_ns": $firstImuTsNs,
                  "frames_written": ${framesWritten.get()},
                  "frames_dropped": ${framesDropped.get()},
                  "imu_written": ${imuWritten.get()},
                  "imu_dropped": ${imuDropped.get()},
                  "imu_max_pair_gap_ns": ${imuMaxPairGapNs.get()},
                  "imu_mean_pair_gap_ns": ${(imuMeanPairGapMs() * 1e6).toLong()},
                  "imu_gap_over_5ms": ${imuGapOver5ms.get()},
                  "calibrated": false,
                  "note": "未标定。内参和相机-IMU外参见阶段四。"
                }
                """.trimIndent()
            )
        } catch (e: Exception) {
            Log.e(TAG, "写 meta.json 失败", e)
        }
    }

    fun stop() {
        running = false
        thread?.join(5000)
    }

    fun stats(): String {
        val fw = framesWritten.get()
        val fd = framesDropped.get()
        val total = fw + fd
        val dropPct = if (total > 0) fd * 100.0 / total else 0.0
        val elapsed = (System.nanoTime() - startedAtNs) / 1e9
        val fps = if (elapsed > 0) fw / elapsed else 0.0

        return buildString {
            append("已运行   : %.1f s\n".format(elapsed))
            append("相机     : %d 帧  掉 %d  丢帧率 %.2f%%\n".format(fw, fd, dropPct))
            // 正常应为 0；不为 0 说明 PNG 编码失败。
            append("编码失败 : %d 帧\n".format(framesFailed.get()))
            append("实际帧率 : %.1f fps  (目标 30)\n".format(fps))
            append("IMU      : %d 条  丢 %d\n".format(imuWritten.get(), imuDropped.get()))
            append("IMU 频率 : %.1f Hz\n".format(if (elapsed > 0) imuWritten.get() / elapsed else 0.0))
            // 均值是主体，最大值仅作参考。
            append("配对偏差 : 均值 %.2f ms   最大 %.2f ms   超5ms %d 次\n"
                .format(imuMeanPairGapMs(), imuMaxPairGapNs.get() / 1e6, imuGapOver5ms.get()))
        }
    }
}
