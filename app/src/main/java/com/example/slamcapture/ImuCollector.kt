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
 * 单位：Android 和 ORB-SLAM3 完全一致，不需要任何转换。
 *   TYPE_ACCELEROMETER  → m/s²
 *   TYPE_GYROSCOPE      → rad/s
 *
 * 时间戳：SensorEvent.timestamp 是纳秒，基准 = SystemClock.elapsedRealtimeNanos()
 *         （CLOCK_BOOTTIME）。相机那边 Image.getTimestamp() 是同一个基准。
 *         **所以直接用，不要做任何转换。**
 *
 * ------------------------------------------------------------------
 * **「传感器在不在跑」和「在不在记录」是两件事**，由 [start]/[stop] 和
 * [attach]/[detach] 分别控制。
 *
 * 因为一次「准备」里可以连着录好几段，每段一个 [SessionWriter]：传感器整段时间
 * 都开着（不用反复付那 3.3 秒冷启动），而只有真正在录的那一段才往文件里写。
 * 这正是编排层那句「仍在采集 imu 和相机，只是不进行记录」的实现。
 * ------------------------------------------------------------------
 */
class ImuCollector(private val context: Context) {

    companion object {
        const val TAG = "ImuCollector"

        /**
         * 多久没有新样本就认为传感器不再出数了。
         *
         * 本机加速度计在 200 Hz 上下（周期 5 ms），500 ms 是一百个采样周期的余量——
         * 正常抖动绝无可能触发，只有真的停了才会。
         */
        private const val STALE_NS = 500_000_000L
    }

    /**
     * **传感器现在是不是在出数** —— 也就是「此刻 [attach]，IMU 的行会立刻开始落盘」。
     *
     * 判据是「最近 [STALE_NS] 内收到过样本」，一个**活的**事实，不是「历史上出过数」。
     * 这个区别是故意的：
     *
     *   - 停止录制（[detach]）之后它仍然为 true —— 因为传感器确实还在跑，**事实没变**。
     *     所以这里不需要、也不存在"复位"这回事：没有哪个历史遗留值需要被清掉。
     *   - 传感器中途哑掉，半秒后它自己变成 false。做成闩锁的话就会一直停在 true 上，
     *     而编排层据此判断「这一段 IMU 会领先相机」——那就成了假话。
     *   - 新建的实例还没出数，自然是 false。
     *
     * ------------------------------------------------------------------
     * **它不制造"IMU 领先相机"这个保证，只报告前提成不成立。** 真正的保证在编排层：
     * 相机帧在 `开始录制 + 300 ms` 之前一律丢弃。传感器活着 ⟹ 行在 ≤5 ms 内开始落，
     * ⟹ 领先约 295 ms。所以这个字段的用途是让「强制开始」那条路能被**识别**出来，
     * 而不是让正常那条路成立。
     *
     * 另外：它说的是"传感器活没活"，不是"这一段写了没有"。ARMED 期间它已经是 true
     * 而 [writer] 还是 null，一行都没写——这是有意的，要不要写由编排层决定。
     * 「这一段到底写进去没有」是另一个事实，**由 [SessionWriter] 拥有**
     * （`firstImuTsNs`）。录制中要看那个。
     * ------------------------------------------------------------------
     *
     * 由 IMU 的 handler 线程写，编排层在主线程读（4 次/秒 + 每次按下开始录制）。
     */
    val ready: Boolean
        get() {
            val last = lastSampleNs.get()
            return last != 0L && SystemClock.elapsedRealtimeNanos() - last < STALE_NS
        }

    /** 最近一次样本的 [SensorEvent.timestamp]。0 = 从来没出过数。 */
    private val lastSampleNs = AtomicLong(0L)

    /**
     * 这个实例有没有被 [start] 过。**它不是状态标志，是「只能 start 一次」的守卫。**
     *
     * 每次「准备」都新建一个实例，所以第二次 [start] 一定是"复用一个已经停掉的对象"
     * ——监听器会重复注册、[lastSampleNs] 的冷启动计时也会错乱。与其在文档里劝，
     * 不如让它在运行时就响。
     */
    private val started = AtomicBoolean(false)

    /**
     * 当前的落盘目标。**null = 传感器在跑，但不记录。**
     *
     * 主线程写（attach/detach），IMU 的 handler 线程读。
     */
    @Volatile
    private var writer: SessionWriter? = null

    /**
     * 每个**搓好对**的样本的出口：一个时间戳 + 角速度 + 加速度。
     *
     * 和 [writer] **各管各的**，可以同时有，也可以只有一边 —— 一个是落盘，一个是喂给 SLAM。
     * **跑在 IMU 的 handler 线程上**，实现里不能阻塞。
     *
     * 为什么是一整个样本而不是「陀螺、加速度各一路」：ORB-SLAM3 的 `IMU::Point`
     * 就是这个形状，而配对逻辑本来就在下面那个陀螺回调里。详见 `SlamBridge.nativePushImu`。
     */
    @Volatile
    var onSample: ((tsNs: Long, wx: Float, wy: Float, wz: Float,
                    ax: Float, ay: Float, az: Float) -> Unit)? = null

    /** [start] 的那一刻，用来量冷启动耗时。 */
    @Volatile
    private var registeredAtNs = 0L

    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    /** 一次加速度读数。不可变对象，放在 @Volatile 引用里天然线程安全。 */
    private class AccelSample(val ts: Long, val x: Float, val y: Float, val z: Float)

    // 保留最近**两次**加速度读数。
    //
    // 陀螺仪事件到来时，挑时间上更近的那个配对，而不是无脑用最新的：
    // 最新那次可能刚发生在很久以前（最坏差一个完整采样周期），而它的
    // 前一次反而离当前时刻更近。保留两次，最坏偏差直接减半。
    @Volatile private var newestAccel: AccelSample? = null
    @Volatile private var prevAccel: AccelSample? = null

    private val accelListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.values.size < 3) return
            prevAccel = newestAccel
            newestAccel = AccelSample(
                event.timestamp, event.values[0], event.values[1], event.values[2]
            )

            // 出数了。这条日志量的就是那 3.3 秒冷启动——老版本量的是"首个样本已落盘"，
            // 现在那两个时刻是分开的，分开量才看得出问题。
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
            val n = newestAccel ?: return            // 还没收到过加速度，等下一拍
            val w = writer
            val cb = onSample
            if (w == null && cb == null) return      // 既不落盘也没人要：传感器照跑
            val p = prevAccel

            val t = event.timestamp
            val s = if (p != null && abs(t - p.ts) < abs(t - n.ts)) p else n

            // 两个出口都在**同一个地方** —— 这样它们拿到的都是搓好对的完整样本。
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

    /** 注册传感器。由「开始准备」调用。**一个实例只能调一次**，见 [started]。 */
    fun start(handler: Handler) {
        if (!started.compareAndSet(false, true)) {
            Log.e(TAG, "同一个 ImuCollector 被 start 了两次——每次「准备」都该新建一个。"
                + "这里拒绝执行，否则监听器会重复注册、冷启动计时会错乱。")
            return
        }

        // 用**已校准**的版本：系统已经做了 bias 和温度补偿，对 ORB-SLAM3 足够。
        // UNCALIBRATED 变体精度更高，但要自己处理 bias 和坐标系，先用普通版。
        val accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        if (accel == null || gyro == null) {
            Log.e(TAG, "设备缺少 IMU 传感器！accel=$accel gyro=$gyro")
            return
        }

        Log.i(TAG, "加速度计: ${accel.name}  最大量程 ${accel.maximumRange} m/s²")
        Log.i(TAG, "陀螺仪  : ${gyro.name}  最大量程 ${gyro.maximumRange} rad/s")

        registeredAtNs = SystemClock.elapsedRealtimeNanos()

        // SENSOR_DELAY_FASTEST = 0 微秒（不限速）+ 后台线程 handler，避免主线程抖动。
        //
        // 注意：Android 12 (API 31) 起，采样率高于 200Hz 需要
        // HIGH_SAMPLING_RATE_SENSORS 权限（已在 Manifest 声明），否则
        // registerListener 直接抛 SecurityException。
        // 这里再兜一层：万一拿不到，退到 200Hz 继续跑，而不是让整个 App 崩掉。
        try {
            sm.registerListener(accelListener, accel, SensorManager.SENSOR_DELAY_FASTEST, handler)
            sm.registerListener(gyroListener, gyro, SensorManager.SENSOR_DELAY_FASTEST, handler)
        } catch (e: SecurityException) {
            Log.w(TAG, "拿不到高速采样权限，退到 200Hz 运行", e)
            sm.registerListener(accelListener, accel, 5000, handler)  // 5000 微秒 = 200Hz
            sm.registerListener(gyroListener, gyro, 5000, handler)
        }
    }

    /** 开始记录。**传感器本来就在跑**，这只是把落盘目标挂上。[ready] 不受影响。 */
    fun attach(w: SessionWriter) {
        writer = w
    }

    /** 停止记录。**传感器继续跑**，下一段录制随时可以再 [attach]。[ready] 不受影响。 */
    fun detach() {
        writer = null
    }

    /**
     * 注销传感器。由「取消准备」调用。
     *
     * 调用之后 [ready] 会在 [STALE_NS] 之内自己变成 false——不需要谁去复位它，
     * 因为它的定义就是"最近还收到过样本"，而注销之后不会再有样本了。
     */
    fun stop() {
        sm.unregisterListener(accelListener)
        sm.unregisterListener(gyroListener)
        writer = null
        Log.i(TAG, "IMU 已停止")
    }
}
