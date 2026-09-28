package com.example.slamcapture

import android.util.Log
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.atomic.AtomicLong

/**
 * 把采集到的帧和 IMU 样本落盘成 EuRoC 数据集格式。
 *
 * 目录布局：
 *   sessionDir/cam0/data/<时间戳ns>.png
 *   sessionDir/cam0/data.csv
 *   sessionDir/imu0/data.csv
 */
class SessionWriter(val sessionDir: File) {

    companion object {
        private const val TAG = "SessionWriter"
        // 队列深度 = 允许积压的帧数。64 帧 ≈ 2 秒。太大则崩溃时丢一大片，
        // 太小则写入线程偶发卡顿就掉帧。
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

    /** IMU 配对时，加速度和陀螺仪时间戳的最大偏差（纳秒）。 */
    val imuMaxPairGapNs = AtomicLong()

    /** 配对偏差的累计值与次数，用来算**均值**。 */
    private val imuGapSumNs = AtomicLong()
    private val imuGapCount = AtomicLong()

    /**
     * 配对偏差超过 5ms 的次数。
     *
     * 这个才是真正要看的指标：最大值只是 12000 次里的一个尖峰（线程被
     * 调度走了、GC 了一下都会造成），完全没有代表性。如果这一项接近 0，
     * 说明偏差只是偶发；如果占比可观，才是系统性问题。
     */
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

    /**
     * 这一段是不是「强制开始」的——对焦没收敛、或者 IMU 还没出数，用户按了红按钮。
     * 由编排层在 [start] 之前设好，写进 meta.json，**事后审查时一眼能认出可疑数据**。
     */
    @Volatile
    var forced = false

    /**
     * 这一段是不是**在 SLAM 跑着的时候录的**（抽屉里那个开关）。
     *
     * ⚠️ 必须标出来：两种来源录出来的文件**格式完全一样**，事后光看目录分不出。
     * 而它们的用途不同 —— 带 SLAM 的那段是给「同段数据手机跑 vs PC 跑」做对照用的，
     * 帧率被落盘拖低过；录制模式录的才是干净的采集数据。
     */
    @Volatile
    var withSlam = false

    /** 第一帧落盘的时刻。0 = 一帧都没落。writer 线程独占写。 */
    @Volatile
    var firstFrameTsNs = 0L
        private set

    /**
     * 第一条 IMU 落盘的时刻。0 = 一条都没落。writer 线程独占写。
     *
     * 这是「**这一段到底写进去没有**」的权威事实——和 ImuCollector.ready 那个事前
     * 判断是两回事。事前判断永远可能过时（传感器刚好哑了、写盘刚好失败），
     * 所以两个都要：一个用来当场决定要不要放行，一个用来事后核对。
     *
     * 和 [firstFrameTsNs] 一比，就是这一段两个流的实际覆盖关系。
     * 按设计 IMU 应该**小于**相机、且差约 300 ms。
     */
    @Volatile
    var firstImuTsNs = 0L
        private set

    /** 建文件失败的原因，正常为 null。编排层看它决定要不要继续。 */
    @Volatile
    var ioError: String? = null
        private set

    /** writer 线程还在不在。**`stop()` 的 join 超时之后，不能再去删目录。** */
    val threadAlive: Boolean get() = thread?.isAlive == true

    /**
     * 建文件并启动 writer 线程。
     *
     * **IO 异常在这里收掉，不抛给调用方。** 磁盘满、外部存储没挂载都会走到这儿，
     * 而调用方是主线程——让它接这个异常只会崩掉 App。失败时 [ioError] 非 null。
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
     * 采集线程调用。队列满了就丢帧并计数——绝不阻塞采集。
     *
     * `running` 为 false 时送来的，是拆除途中的在途帧。[MainActivity] 读 `writer`
     * 和 `phase` 的顺序（先 writer 后 phase）与本类拆除时的写序（先 phase 后 writer）
     * **正好相反**，所以相机线程可能读到非 null 的 writer 加一个陈旧的 RECORDING。
     * 这个窗口只有毫秒级，后果也就是丢一帧——但老代码在这条路上是**静默丢弃且不计数**，
     * 于是 `frames_written + frames_dropped` 不等于相机实际出的帧数，统计口径骗人。
     */
    fun offerFrame(item: FrameItem) {
        if (!running || !frameQueue.offer(item)) framesDropped.incrementAndGet()
    }

    /** 同上，[offerImu] 侧的那道检查。 */
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

            // 直接手写灰度 PNG，不走 Bitmap。
            //
            // 早期版本用 Bitmap.compress()，踩了两个坑：
            //   ① ALPHA_8（"纯 alpha"，没有颜色通道）不被 Skia 的 PNG 编码器
            //      支持，compress() 返回 false 且一个字节都不写，完全静默，
            //      结果是文件被创建但大小是 0。
            //   ② 改用 ARGB_8888 之后能出图了，但 4 字节/像素让每帧编码耗时
            //      超过 100ms，写盘线程跟不上 30fps，丢帧率飙到 65%。
            //
            // 手写编码器是 1 字节/像素，数据量小 4 倍，而且产物是真正的
            // 8-bit grayscale。详见 GrayPng.kt。
            FileOutputStream(file).use { out ->
                GrayPng.encode(f.gray, f.w, f.h, out)
            }

            // 仍然保留这道校验：文件系统写满、编码器内部异常等都可能留下
            // 空文件。宁可计一个失败，也不要往 CSV 里记一条指向空图的记录。
            if (file.length() == 0L) {
                file.delete()
                framesFailed.incrementAndGet()
                Log.e(TAG, "PNG 写出为空 ts=${f.tsNs}（已删除空文件）")
                return
            }

            camCsv.write("${f.tsNs},$name\n")
            val written = framesWritten.incrementAndGet()
            if (firstFrameTsNs == 0L) firstFrameTsNs = f.tsNs
            if (written % 30 == 0L) camCsv.flush()  // 每秒刷一次，崩溃时少丢数据
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

    /**
     * 关掉两个 CSV。**对"可能根本没打开成功"是安全的**——[start] 失败时
     * 会在中途抛出来，那时只有一部分文件被打开。
     */
    private fun closeFiles() {
        try { if (::camCsv.isInitialized) { camCsv.flush(); camCsv.close() } } catch (_: Exception) {}
        try { if (::imuCsv.isInitialized) { imuCsv.flush(); imuCsv.close() } } catch (_: Exception) {}
    }

    private fun closeQuietly() {
        closeFiles()

        // 写一份元数据，阶段三/四用得上
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
            // 这一项必须为 0。不为 0 说明 PNG 编码失败，磁盘上会有 0 字节文件。
            append("编码失败 : %d 帧\n".format(framesFailed.get()))
            append("实际帧率 : %.1f fps  (目标 30)\n".format(fps))
            append("IMU      : %d 条  丢 %d\n".format(imuWritten.get(), imuDropped.get()))
            append("IMU 频率 : %.1f Hz\n".format(if (elapsed > 0) imuWritten.get() / elapsed else 0.0))
            // 均值才是主体，最大值只做参考。超 5ms 的次数接近 0 就说明
            // 那个最大值只是偶发尖峰，不是系统性问题。
            append("配对偏差 : 均值 %.2f ms   最大 %.2f ms   超5ms %d 次\n"
                .format(imuMeanPairGapMs(), imuMaxPairGapNs.get() / 1e6, imuGapOver5ms.get()))
        }
    }
}
