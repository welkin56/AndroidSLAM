package com.example.slamcapture

import android.content.Intent
import android.os.Bundle
import android.text.format.Formatter
import android.util.Log
import android.view.View
import android.view.ViewGroup
import android.widget.BaseAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * 入口页：选择地图来源后进入主界面。
 *
 * 这一页只记录选择，不加载词袋、atlas 或相机。真正载图发生在第一次启动 SLAM 时。
 * 启动主界面时不 finish()，以便按返回键回来重新选择。
 * 选择结果传给 [SlamBridge.nativeInit] 的第三个参数，null 表示新建地图。
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
        applySystemBarAppearance()

        findViewById<Button>(R.id.btnNewMap).setOnClickListener {
            Log.i(TAG, "选了「新建地图」")
            open(null)
        }

        // 录制模式不涉及地图，直接进入采集页；两个模式各开各的相机。
        findViewById<Button>(R.id.btnRecordMode).setOnClickListener {
            Log.i(TAG, "进录制模式")
            startActivity(Intent(this, CaptureActivity::class.java))
        }

        // 列表在 onStart 中扫描。
    }

    /**
     * 列表在每次回到本页时重扫。
     * 启动主界面时不 finish()，本 Activity 只会被 stop，onCreate 不会再次执行。
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
            list.adapter = MapAdapter(maps)
        }
    }

    /** 一行：名字 + 修改时间与大小。文件名与目录名重复时只显示目录名。 */
    private inner class MapAdapter(private val items: List<File>) : BaseAdapter() {
        override fun getCount() = items.size
        override fun getItem(position: Int) = items[position]
        override fun getItemId(position: Int) = items[position].absolutePath.hashCode().toLong()

        override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
            val v = convertView ?: layoutInflater.inflate(R.layout.item_map, parent, false)
            val f = items[position]
            val dir = f.parentFile?.name ?: ""
            val base = f.name.removeSuffix(EXT)
            v.findViewById<TextView>(R.id.tvMapTitle).text =
                if (base == dir) dir else "$dir / $base"
            val when_ = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.getDefault())
                .format(Date(f.lastModified()))
            val size = Formatter.formatShortFileSize(this@EntryActivity, f.length())
            v.findViewById<TextView>(R.id.tvMapSubtitle).text = "$when_ · $size"
            // 点击交给卡片：ListView 的 listSelector 是矩形，会盖住圆角。
            v.findViewById<View>(R.id.mapCard).setOnClickListener {
                Log.i(TAG, "选了地图：${f.absolutePath}")
                open(f.absolutePath)
            }
            return v
        }
    }

    /**
     * 扫描 maps/<图目录>/<名字>.osa。
     *
     * 一张地图对应一个目录，其中还包含该图的轨迹。一个目录里可能有多份 .osa，
     * 每次另存都会新写一份，不覆盖。只能显示文件层面的信息（名字、时间）。
     */
    private fun listMaps(): List<File> {
        val dirs = mapsDir().listFiles { f -> f.isDirectory } ?: return emptyList()
        return dirs
            .flatMap { d -> (d.listFiles { f -> f.isFile && f.name.endsWith(EXT) } ?: emptyArray()).toList() }
            .sortedByDescending { it.lastModified() }
    }

    private fun mapsDir(): File = File(getExternalFilesDir(null) ?: filesDir, MAPS_DIR)

    /**
     * @param mapFile 已保存地图的绝对路径；null 表示新建地图。
     */
    private fun open(mapFile: String?) {
        startActivity(
            Intent(this, MainActivity::class.java)
                .putExtra(MainActivity.EXTRA_MAP_FILE, mapFile)
        )
    }
}
