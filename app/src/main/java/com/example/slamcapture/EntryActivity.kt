package com.example.slamcapture

import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * 入口页：选定「地图从哪来」，然后进主界面。
 *
 * **这一页只记选择，不加载任何东西**（APP_DESIGN §2.1）—— 词袋、atlas、相机一个都不碰，
 * 所以它可以瞬间返回、改主意。真正的载图发生在第一次「启动 SLAM」，
 * 因为 `LoadAtlas` 在 `System` 的构造函数里，构造就发生在启动那一刻（§2.2）。
 *
 * 启动主界面时**不 finish()** —— Back 要能回到这儿改主意。
 *
 * ⚠️ **保存还没做（M8）**，所以磁盘上不会有 `.osa`，列表恒为空。
 * 这一页现在的作用是把「选择」这条通道立起来：它一路传到
 * [SlamBridge.nativeInit] 的第三个参数（`null` = 新建地图）。
 */
class EntryActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "EntryActivity"

        /** 地图文件放哪。和采集 session 同一个根 —— `adb pull/push` 都不需要 root。 */
        private const val MAPS_DIR = "maps"
        private const val EXT = ".osa"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_entry)

        findViewById<Button>(R.id.btnNewMap).setOnClickListener {
            Log.i(TAG, "选了「新建地图」")
            open(null)
        }

        // 录制模式和「选地图」是两条不同的轴：它不碰地图，直接进采集页。
        // 两个模式互斥（各开各的相机），见 APP_DESIGN §6 / §7.1。
        findViewById<Button>(R.id.btnRecordMode).setOnClickListener {
            Log.i(TAG, "进录制模式")
            startActivity(Intent(this, CaptureActivity::class.java))
        }

        // 列表不在这儿扫 —— 见 onStart 的注释。onStart 在 onCreate 之后必定会跑。
    }

    /**
     * 扫列表**放在这里而不是 [onCreate]**。
     *
     * 因为启主界面时**不 `finish()`**（Back 要能回来），这个 Activity 只会被 stop、
     * 不会被销毁 —— 于是 `onCreate` **只跑一次**，从主界面退回来时列表是**陈旧的**。
     * 将来存了一张图再退回来，新图不会出现。放 `onStart` 就没这个问题。
     */
    override fun onStart() {
        super.onStart()
        refreshMapList()
    }

    private fun refreshMapList() {
        val list = findViewById<ListView>(R.id.mapList)
        val empty = findViewById<TextView>(R.id.tvEmpty)
        val maps = listMaps()

        if (maps.isEmpty()) {
            list.visibility = View.GONE
            empty.visibility = View.VISIBLE
        } else {
            empty.visibility = View.GONE
            list.visibility = View.VISIBLE
            list.adapter =
                ArrayAdapter(this, android.R.layout.simple_list_item_1, maps.map(::describe))
            list.setOnItemClickListener { _, _, pos, _ ->
                Log.i(TAG, "选了地图：${maps[pos].absolutePath}")
                open(maps[pos].absolutePath)
            }
        }
    }

    /**
     * 扫 [mapsDir] 下的 `.osa`。目录不存在（一次都没存过）时 `listFiles` 返回 null。
     *
     * ⚠️ 这里**只能显示文件层面的东西**（名字、时间）。「这张图有多少关键帧」显示不了 ——
     * 那要新增 JNI 查询接口，而 STAGE5 §6 的接口表里没有。
     */
    private fun listMaps(): List<File> =
        (mapsDir().listFiles { f -> f.isFile && f.name.endsWith(EXT) } ?: emptyArray())
            .sortedByDescending { it.lastModified() }

    private fun mapsDir(): File = File(getExternalFilesDir(null) ?: filesDir, MAPS_DIR)

    private fun describe(f: File): String {
        val when_ = SimpleDateFormat("MM-dd HH:mm", Locale.getDefault()).format(Date(f.lastModified()))
        return "${f.name.removeSuffix(EXT)}\n$when_"
    }

    /**
     * @param mapFile 已保存地图的绝对路径；**null = 新建地图**。
     *   「新建」和「载入旧图」是同一个参数的两个取值，不是两种模式（APP_DESIGN §2.4）。
     */
    private fun open(mapFile: String?) {
        startActivity(
            Intent(this, MainActivity::class.java)
                .putExtra(MainActivity.EXTRA_MAP_FILE, mapFile)
        )
    }
}
