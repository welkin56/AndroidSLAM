package com.example.slamcapture

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.SystemClock
import android.util.Log
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs

/**
 * 采集加速度计和陀螺仪。
 *
 * 单位与 ORB-SLAM3 一致：加速度为 m/s²，角速度为 rad/s，不需要转换。
 * SensorEvent.timestamp 与 Camera 的 Image.getTimestamp() 都基于 CLOCK_BOOTTIME，
 * 可以直接使用。
 *
 * 「传感器是否在运行」与「是否在记录」是两件事，分别由 [start]/[stop]
 * 和 [attach]/[detach] 控制。
 */
class ImuCollector(private val context: Context) {

    companion object {
        const val TAG = "ImuCollector"

        /**
         * 超过这么久没有新样本就认为传感器停止出数。
         * 本机采样约 200 Hz，500 ms 留有充足余量。
         */
        private const val STALE_NS = 500_000_000L
    }

    /**
     * 传感器当前是否在出数：最近 [STALE_NS] 内收到过样本。
     *
     * 这是实时状态，不是历史标记。[detach] 后仍为 true，传感器停止出数后自动变为 false。
     * 它只表示传感器活着，不表示这一段已经写入文件；后者看 [SessionWriter.firstImuTsNs]。
     *
     * 由 IMU 线程写，主线程读。
     */
    val ready: Boolean
        get() {
            val last = lastSampleNs.get()
            return last != 0L && SystemClock.elapsedRealtimeNanos() - last < STALE_NS
        }

    /** 最近一次样本的 [SensorEvent.timestamp]；0 表示从未出数。 */
    private val lastSampleNs = AtomicLong(0L)

    /** 是否已经 [start] 过。每个实例只能用一次，防止重复注册监听器。 */
    private val started = AtomicBoolean(false)

    /** 当前落盘目标；null 表示传感器在运行但不记录。主线程写，IMU 线程读。 */
    @Volatile
    private var writer: SessionWriter? = null

    /**
     * 配对完成的样本出口：时间戳、角速度、加速度。
     *
     * 与 [writer] 相互独立，可同时存在。运行在 IMU 线程上，实现中不能阻塞。
     */
    @Volatile
    var onSample: ((tsNs: Long, wx: Float, wy: Float, wz: Float,
                    ax: Float, ay: Float, az: Float) -> Unit)? = null

/** [start] 的时刻，用于测量冷启动耗时。 */
    @Volatile
    private var registeredAtNs = 0L

    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    /** 一次加速度读数。不可变对象。 */
    private class AccelSample(val ts: Long, val x: Float, val y: Float, val z: Float)

    // 保留最近两次加速度读数。陀螺仪事件到来时选择时间更近的一个配对，
    // 而不是固定使用最新值，以减小配对偏差。
    @Volatile private var newestAccel: AccelSample? = null
    @Volatile private var prevAccel: AccelSample? = null

    private val accelListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.values.size < 3) return
            prevAccel = newestAccel
            newestAccel = AccelSample(
                event.timestamp, event.values[0], event.values[1], event.values[2]
            )

            // 首次出数时记录冷启动耗时。
            if (lastSampleNs.getAndSet(event.timestamp) == 0L) {
                val t0 = registeredAtNs
                val ms = if (t0 > 0) (event.timestamp - t0) / 1_000_000 else -1
                Log.i(TAG, "IMU 已出数，冷启动耗时 $ms ms")
            }
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    private val gyroListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.values.size < 3) return
            val n = newestAccel ?: return            // 还没有加速度样本
            val w = writer
            val cb = onSample
            if (w == null && cb == null) return      // 既不落盘也无人订阅
            val p = prevAccel

            val t = event.timestamp
            val s = if (p != null && abs(t - p.ts) < abs(t - n.ts)) p else n

            // 两个出口使用同一份配对结果。
            cb?.invoke(t, event.values[0], event.values[1], event.values[2], s.x, s.y, s.z)
            if (w != null) {
                w.recordPairGap(abs(t - s.ts))
                w.offerImu(
                    SessionWriter.ImuItem(
                        t,
                        event.values[0], event.values[1], event.values[2],
                        s.x, s.y, s.z
                    )
                )
            }
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    /** 注册传感器。由「开始准备」调用，每个实例只能调用一次。 */
    fun start(handler: Handler) {
        if (!started.compareAndSet(false, true)) {
            Log.e(TAG, "同一个 ImuCollector 被 start 了两次——每次「准备」都该新建一个。"
                + "这里拒绝执行，否则监听器会重复注册、冷启动计时会错乱。")
            return
        }

        // 使用已校准的传感器：系统已完成 bias 和温度补偿。
        val accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        if (accel == null || gyro == null) {
            Log.e(TAG, "设备缺少 IMU 传感器！accel=$accel gyro=$gyro")
            return
        }

        Log.i(TAG, "加速度计: ${accel.name}  最大量程 ${accel.maximumRange} m/s²")
        Log.i(TAG, "陀螺仪  : ${gyro.name}  最大量程 ${gyro.maximumRange} rad/s")

        registeredAtNs = SystemClock.elapsedRealtimeNanos()

        // SENSOR_DELAY_FASTEST 不限速，并使用后台线程 handler。
        // Android 12 起高速采样需要 HIGH_SAMPLING_RATE_SENSORS 权限；
        // 拿不到时退到 200 Hz。
        try {
            sm.registerListener(accelListener, accel, SensorManager.SENSOR_DELAY_FASTEST, handler)
            sm.registerListener(gyroListener, gyro, SensorManager.SENSOR_DELAY_FASTEST, handler)
        } catch (e: SecurityException) {
            Log.w(TAG, "拿不到高速采样权限，退到 200Hz 运行", e)
            sm.registerListener(accelListener, accel, 5000, handler)  // 5000 微秒 = 200Hz
            sm.registerListener(gyroListener, gyro, 5000, handler)
        }
    }

    /** 挂上落盘目标。传感器保持运行，[ready] 不受影响。 */
    fun attach(w: SessionWriter) {
        writer = w
    }

    /** 清除落盘目标。传感器继续运行，之后可以再次 [attach]。 */
    fun detach() {
        writer = null
    }

    /**
     * 注销传感器。由「取消准备」调用。
     * 注销后 [ready] 会在 [STALE_NS] 内自动变为 false。
     */
    fun stop() {
        sm.unregisterListener(accelListener)
        sm.unregisterListener(gyroListener)
        writer = null
        Log.i(TAG, "IMU 已停止")
    }
}
