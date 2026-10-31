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
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.google.android.material.dialog.MaterialAlertDialogBuilder
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

        // 标定同理：不载图、不开 SLAM，只算内参。先进档位列表，在那里管理/新建各屈光度。
        findViewById<Button>(R.id.btnCalib).setOnClickListener {
            Log.i(TAG, "进标定档位列表")
            startActivity(Intent(this, CalibListActivity::class.java))
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
        val hint = findViewById<TextView>(R.id.tvListHint)
        val maps = listMaps()

        hint.visibility = if (maps.isEmpty()) View.GONE else View.VISIBLE
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
            v.findViewById<TextView>(R.id.tvMapTitle).text = mapTitle(f)
            val when_ = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.getDefault())
                .format(Date(f.lastModified()))
            val size = Formatter.formatShortFileSize(this@EntryActivity, f.length())
            v.findViewById<TextView>(R.id.tvMapSubtitle).text = "$when_ · $size"
            // 点击交给卡片：ListView 的 listSelector 是矩形，会盖住圆角。
            v.findViewById<View>(R.id.mapCard).apply {
                setOnClickListener {
                    Log.i(TAG, "选了地图：${f.absolutePath}")
                    open(f.absolutePath)
                }
                setOnLongClickListener {
                    showMapActions(f)
                    true
                }
            }
            return v
        }
    }

    /** 文件名与目录名重复时只显示目录名。 */
    private fun mapTitle(f: File): String {
        val dir = f.parentFile?.name ?: ""
        val base = f.name.removeSuffix(EXT)
        return if (base == dir) dir else "$dir / $base"
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

    /** 长按菜单。改名和删除都会动磁盘，先让用户挑。 */
    private fun showMapActions(osa: File) {
        MaterialAlertDialogBuilder(this)
            .setTitle(mapTitle(osa))
            .setItems(arrayOf("重命名", "删除")) { _, which ->
                if (which == 0) askRenameMap(osa) else askDeleteMap(osa)
            }
            .setNegativeButton("取消", null)
            .show()
    }

    /**
     * 列表上的名字就是会话目录名，改名同时改目录和目录里唯一的 .osa。
     * 目录里有多份 .osa 时只改目录，文件名留作区分。
     */
    private fun askRenameMap(osa: File) {
        val dir = osa.parentFile ?: return
        textInputDialog("重命名地图", dir.name) { raw ->
            val name = TrajectoryStore.cleanMapName(raw)
            if (name.isEmpty()) {
                Toast.makeText(this, "名字不能为空", Toast.LENGTH_SHORT).show()
                return@textInputDialog
            }
            renameMap(dir, name)
        }
    }

    private fun renameMap(dir: File, name: String) {
        if (name == dir.name) return
        val target = TrajectoryStore.uniqueMapDir(dir.parentFile ?: return, name)
        val osas = dir.listFiles { f -> f.isFile && f.name.endsWith(EXT) } ?: emptyArray()
        if (!dir.renameTo(target)) {
            Log.e(TAG, "目录改名失败：${dir.absolutePath} → ${target.absolutePath}")
            Toast.makeText(this, "重命名失败", Toast.LENGTH_LONG).show()
            return
        }
        Log.i(TAG, "地图改名：${dir.name} → ${target.name}")
        // 旧 File 已经指向不存在的路径，得从改名后的目录里重新取。
        if (osas.size == 1 && osas[0].name.removeSuffix(EXT) != target.name) {
            val f = File(target, osas[0].name)
            if (!f.renameTo(File(target, target.name + EXT)))
                Log.e(TAG, "地图文件改名失败：${f.absolutePath}")
        }
        refreshMapList()
    }

    /**
     * 删整个会话目录。一行地图背后是一个目录：地图本体、按图归档的轨迹、录像都在里面。
     * 只删 .osa 会留下首页再也看不到的孤儿数据，所以这里连目录一起删。
     */
    private fun askDeleteMap(osa: File) {
        val dir = osa.parentFile ?: return
        val others = (dir.listFiles { f -> f.isFile && f.name.endsWith(EXT) }?.size ?: 1) - 1
        val msg = buildString {
            append(mapTitle(osa)).append("\n\n").append(dir.absolutePath).append("\n\n")
            append(summarize(dir))
            if (others > 0) append("\n\n目录里另外 $others 份地图也会一起删掉。")
            append("\n\n删除后不可恢复。")
        }
        MaterialAlertDialogBuilder(this)
            .setTitle("删除地图？")
            .setMessage(msg)
            .setPositiveButton("删除") { _, _ ->
                Log.i(TAG, "删除地图目录：${dir.absolutePath}")
                if (deleteRecursively(dir)) refreshMapList()
                else Toast.makeText(this, "删除失败，详情见日志", Toast.LENGTH_LONG).show()
            }
            .setNegativeButton("取消", null)
            .show()
    }

    /** 目录内容的粗统计：地图份数、轨迹条数、录像文件数与总大小。 */
    private fun summarize(dir: File): String {
        var osa = 0
        var traces = 0
        var recs = 0
        var bytes = 0L
        dir.walkTopDown().filter { it.isFile }.forEach { f ->
            bytes += f.length()
            val rel = f.relativeTo(dir).path
            when {
                rel.endsWith(EXT) -> osa++
                // 归档在 map-<id>/ 的与尚未归档的 traj/ 里的都是轨迹文本。
                rel.endsWith(".txt") && (rel.startsWith("traj/") || rel.startsWith("map-")) -> traces++
                rel.startsWith("rec/") -> recs++
            }
        }
        val parts = mutableListOf("$osa 份地图")
        if (traces > 0) parts += "$traces 条轨迹"
        if (recs > 0) parts += "$recs 个录像文件"
        parts += Formatter.formatShortFileSize(this, bytes)
        return parts.joinToString(" · ")
    }

    private fun deleteRecursively(f: File): Boolean {
        if (f.isDirectory) f.listFiles()?.forEach { deleteRecursively(it) }
        return f.delete()
    }

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
