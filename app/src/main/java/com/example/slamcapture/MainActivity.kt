package com.example.slamcapture

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.io.File

class MainActivity : AppCompatActivity() {

    companion object { const val TAG = "MainActivity" }

    private lateinit var btnToggle: Button
    private lateinit var tvStatus: TextView

    private var writer: SessionWriter? = null
    private var camera: CameraCapture? = null
    private var imu: ImuCollector? = null

    private var camThread: HandlerThread? = null
    private var imuThread: HandlerThread? = null

    private var running = false
    private val uiHandler = Handler(Looper.getMainLooper())

    private val statsRunnable = object : Runnable {
        override fun run() {
            writer?.let { tvStatus.text = it.stats() + "\n\n" + it.sessionDir.absolutePath }
            if (running) uiHandler.postDelayed(this, 500)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        btnToggle = findViewById(R.id.btnToggle)
        tvStatus = findViewById(R.id.tvStatus)

        btnToggle.setOnClickListener {
            if (running) stopCapture() else ensurePermissionAndStart()
        }
    }

    // ---------------- 权限 ----------------

    private fun ensurePermissionAndStart() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            == PackageManager.PERMISSION_GRANTED
        ) {
            startCapture()
        } else {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 1)
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == 1 && grantResults.isNotEmpty()
            && grantResults[0] == PackageManager.PERMISSION_GRANTED
        ) {
            startCapture()
        } else {
            tvStatus.text = "相机权限被拒绝。\n请到 设置 → 应用 → SlamCapture → 权限 里手动开启。"
        }
    }

    // ---------------- 采集生命周期 ----------------

    private fun startCapture() {
        val sessionDir = File(
            File(getExternalFilesDir(null), "sessions"),
            "session_${System.currentTimeMillis()}"
        )
        sessionDir.mkdirs()
        Log.i(TAG, "输出目录: $sessionDir")

        val w = SessionWriter(sessionDir).also { it.start() }
        writer = w

        // 采集和写盘都在独立线程上。主线程只负责画界面。
        val ct = HandlerThread("camera").also { it.start() }
        val it2 = HandlerThread("imu").also { it.start() }
        camThread = ct
        imuThread = it2

        val cam = CameraCapture(this, w, Handler(ct.looper))
        val imuCollector = ImuCollector(this, w)

        // 相机要等 IMU 真的开始写数据之后才能录帧。ORB-SLAM3 假定 IMU 比
        // 相机先开始，反过来的话它的加载器会读越界内存，而且不一定崩——
        // 可能只是每帧配到 0 个 IMU 测量，最后给一条莫名其妙的烂轨迹。
        //
        // CameraCapture 内部是「对焦就绪 + IMU 就绪」两个条件都齐了才开闸，
        // 所以这里不需要谁等谁：两条线各跑各的，谁后到谁触发。
        imuCollector.onFirstSample = { cam.markImuReady() }

        camera = cam
        imu = imuCollector

        // 顺序有讲究：先注册 IMU。本机传感器要 3 秒多才出数，那一刻钟从
        // registerListener 开始算——先把它点着，相机开设备、配对焦的时间
        // 就白赚在这段里，两条线的等待完全重叠。
        imuCollector.start(Handler(it2.looper))

        cam.logCapabilities()   // 打印设备能力，第一次跑一定要看 logcat
        cam.start()

        running = true
        btnToggle.text = "停止采集"
        uiHandler.post(statsRunnable)
    }

    private fun stopCapture() {
        running = false
        uiHandler.removeCallbacks(statsRunnable)

        camera?.stop()
        imu?.stop()
        camThread?.quitSafely()
        imuThread?.quitSafely()

        val w = writer
        w?.stop()

        tvStatus.text = "已停止\n\n" + (w?.stats() ?: "") + "\n\n" + (w?.sessionDir?.absolutePath ?: "")
        Log.i(TAG, "采集结束")

        writer = null; camera = null; imu = null
        camThread = null; imuThread = null
    }

    override fun onStop() {
        super.onStop()
        // 切后台就停。后台采集会被系统限制帧率，数据不可信。
        if (running) stopCapture()
    }

    override fun onDestroy() {
        super.onDestroy()
        uiHandler.removeCallbacksAndMessages(null)
    }
}
