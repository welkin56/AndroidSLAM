package com.example.slamcapture

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.util.Log
import java.util.concurrent.atomic.AtomicBoolean
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
 */
class ImuCollector(private val context: Context, private val writer: SessionWriter) {

    companion object { const val TAG = "ImuCollector" }

    /**
     * 第一个**真正写进文件**的 IMU 样本到来时回调一次。
     *
     * 相机在等这个信号，相机不能在 IMU 出数前就录帧。
     *
     * 回调点在 writer.offerImu() **之后**，不是 onSensorChanged 一进来。
     * 因为上面那句 `val n = newestAccel ?: return`：加速度计还没出数时，陀螺仪
     * 样本根本不会落盘。这个信号要代表「文件真的拿到第一行」。
     *
     * 回调在 IMU 的 handler 线程上执行，实现里不要做阻塞操作。
     */
    @Volatile var onFirstSample: (() -> Unit)? = null

    private val firstSampleSeen = AtomicBoolean(false)

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
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    private val gyroListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.values.size < 3) return
            val n = newestAccel ?: return   // 还没收到过加速度，等下一拍
            val p = prevAccel

            val t = event.timestamp
            val s = if (p != null && abs(t - p.ts) < abs(t - n.ts)) p else n

            writer.recordPairGap(abs(t - s.ts))
            writer.offerImu(
                SessionWriter.ImuItem(
                    t,
                    event.values[0], event.values[1], event.values[2],
                    s.x, s.y, s.z
                )
            )

            // 文件已经拿到第一行，通知在等的人（相机）。只放一次。
            if (firstSampleSeen.compareAndSet(false, true)) {
                Log.i(TAG, "首个 IMU 样本已落盘 t=$t，通知相机")
                onFirstSample?.invoke()
            }
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    fun start(handler: Handler) {
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

    fun stop() {
        sm.unregisterListener(accelListener)
        sm.unregisterListener(gyroListener)
        Log.i(TAG, "IMU 已停止")
    }
}
