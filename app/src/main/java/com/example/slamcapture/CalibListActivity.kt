package com.example.slamcapture

import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.view.View
import android.view.ViewGroup
import android.widget.BaseAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.google.android.material.color.MaterialColors
import com.google.android.material.dialog.MaterialAlertDialogBuilder

/**
 * 标定档位列表：看、排序、删掉各屈光度下的内参。
 *
 * 新建走老的 [CalibActivity]（棋盘格采集）；它 `finish()` 回到这里，本页在
 * [onStart] 里重扫 `profile.json` 就能看到新行 —— 和入口页刷地图列表同一个范式。
 *
 * 列表用 `ListView + BaseAdapter`（仓库里唯一在用的列表写法），不引 RecyclerView。
 * 排序只在内存里做：`CalibStore` 读写都会按 `d` 升序重排，写回的顺序没有意义。
 */
class CalibListActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "CalibList"
    }

    private enum class SortKey { DIOPTER, TIME }

    private lateinit var list: ListView
    private lateinit var tvEmpty: TextView
    private lateinit var tvMeta: TextView
    private lateinit var tvSection: TextView

    /** 排序芯片。用 TextView + bg_chip，和 activity_main 的状态芯片同一套形状。 */
    private lateinit var chipSortDiopter: TextView
    private lateinit var chipSortTime: TextView

    private var prof: CalibProfile? = null

    /**
     * 显示顺序，元素是 [CalibProfile.rows] 的**下标**。
     *
     * 存下标而不是 `CalibRow` 对象：它是 data class，两行数值完全相同时 `equals` 相等，
     * 按对象删会连带删掉另一行 —— 而「同档标了两次」正是这次要支持的情形。
     */
    private var order: List<Int> = emptyList()

    private var sortKey = SortKey.DIOPTER

    /** 屈光度默认升序（对齐磁盘顺序和插值顺序），时间默认最新在前。 */
    private var asc = true

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_calib_list)
        applySystemBarAppearance()

        list = findViewById(R.id.calibList)
        tvEmpty = findViewById(R.id.tvEmpty)
        tvMeta = findViewById(R.id.tvMeta)
        tvSection = findViewById(R.id.tvSection)
        chipSortDiopter = findViewById(R.id.chipSortDiopter)
        chipSortTime = findViewById(R.id.chipSortTime)

        chipSortDiopter.setOnClickListener { pickSort(SortKey.DIOPTER) }
        chipSortTime.setOnClickListener { pickSort(SortKey.TIME) }
        findViewById<Button>(R.id.btnNewCalib).setOnClickListener {
            Log.i(TAG, "进标定采集页")
            startActivity(Intent(this, CalibActivity::class.java))
        }

        render()
    }

    override fun onStart() {
        super.onStart()
        reload()
    }

    // ---------------- 数据 ----------------

    private fun reload() {
        prof = CalibStore.load(this)
        reorder()
        render()
    }

    /** 点当前那个键 = 翻转方向；点另一个 = 换键并回到它自己的默认方向。 */
    private fun pickSort(key: SortKey) {
        if (key == sortKey) {
            asc = !asc
        } else {
            sortKey = key
            asc = key == SortKey.DIOPTER
        }
        reorder()
        render()
    }

    private fun reorder() {
        val rows = prof?.rows.orEmpty()
        // sortedBy 是稳定排序：键相同时保持磁盘顺序（按 d 升序），行为可预期。
        val base = when (sortKey) {
            SortKey.DIOPTER -> rows.indices.sortedBy { rows[it].d }
            SortKey.TIME -> rows.indices.sortedBy { rows[it].date }
        }
        order = if (asc) base else base.reversed()
    }

    // ---------------- 渲染 ----------------

    private fun render() {
        val p = prof
        val rows = p?.rows.orEmpty()

        // 副标题只放设备信息，不重复档位数量 —— 那个放在分区标题上。
        tvMeta.text = if (p == null) {
            "还没有标定表 —— 现在用的是模板内参"
        } else buildString {
            append(p.model.takeIf { it.isNotBlank() } ?: "未知机型")
            p.cameraId.takeIf { it.isNotBlank() }?.let { append(" · 相机 ").append(it) }
            if (p.imuFrequency > 0f) append(" · IMU ").append("%.1f Hz".format(p.imuFrequency))
        }
        tvSection.text = if (rows.isEmpty()) "档位" else "档位 · ${rows.size}"

        // 方向只写在选中的那颗上：两颗都带箭头会让人以为它们各自有方向。
        val arrow = if (asc) " ↑" else " ↓"
        val byD = sortKey == SortKey.DIOPTER
        chipSortDiopter.text = if (byD) "屈光度$arrow" else "屈光度"
        chipSortTime.text = if (!byD) "最近时间$arrow" else "最近时间"
        applyChip(chipSortDiopter, byD)
        applyChip(chipSortTime, !byD)

        if (rows.isEmpty()) {
            list.visibility = View.GONE
            list.adapter = null
            tvEmpty.visibility = View.VISIBLE
        } else {
            tvEmpty.visibility = View.GONE
            list.visibility = View.VISIBLE
            list.adapter = RowAdapter(rows)
        }
    }

    /**
     * 芯片的选中态：填主色 + 文字反白；未选中回到 bg_chip 的描边底。
     *
     * 颜色走 MaterialColors 而不是写死 `@color`，因为主题跟随动态色（见 themes.xml 的注释）。
     * padding 在换背景之后重设一遍：`setBackgroundResource` 会按 drawable 的 padding 覆盖掉
     * 布局里那份 —— shape 没有 padding，理论上不会变，但显式设一次省得以后再踩。
     */
    private fun applyChip(v: TextView, on: Boolean) {
        v.setBackgroundResource(if (on) R.drawable.bg_chip_selected else R.drawable.bg_chip)
        v.setPadding(dp(10), dp(5), dp(10), dp(5))
        v.setTextColor(
            MaterialColors.getColor(
                v,
                if (on) com.google.android.material.R.attr.colorOnPrimary
                else com.google.android.material.R.attr.colorOnSurfaceVariant,
            )
        )
    }

    private fun dp(v: Int): Int = (v * resources.displayMetrics.density).toInt()

    /** 一行：屈光度（+ 等效距离）/ fx · RMS · 日期。 */
    private inner class RowAdapter(private val rows: List<CalibRow>) : BaseAdapter() {
        override fun getCount() = order.size
        override fun getItem(position: Int) = rows[order[position]]
        override fun getItemId(position: Int) = order[position].toLong()

        override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
            val v = convertView ?: layoutInflater.inflate(R.layout.item_calib_row, parent, false)
            val i = order[position]
            val r = rows[i]
            v.findViewById<TextView>(R.id.tvRowTitle).text =
                "%.2f D（≈ %s）".format(r.d, distance(r.d))
            // 挤成一行是 36 个等宽字符 ≈ 259dp，卡片里只剩 260dp 左右，窄屏必然截断。
            // 拆成两段：数字在左，时间右对齐成一列。RMS 的单位省了，px 是这个量的常识。
            v.findViewById<TextView>(R.id.tvRowSub).text =
                "fx %.1f · RMS %.2f".format(r.fx, r.rms)
            v.findViewById<TextView>(R.id.tvRowDate).text = shortDate(r.date)
            // 点击交给卡片：ListView 的 listSelector 是矩形，会盖住 16dp 圆角。
            v.findViewById<View>(R.id.calibCard).apply {
                setOnClickListener { showDetail(i) }
                setOnLongClickListener { askDelete(i); true }
            }
            return v
        }
    }

    // ---------------- 行操作 ----------------

    /** 点一行：把列表上看不全的参数摊开。 */
    private fun showDetail(i: Int) {
        val r = prof?.rows?.getOrNull(i) ?: return
        MaterialAlertDialogBuilder(this)
            .setTitle("%.2f D（≈ %s）".format(r.d, distance(r.d)))
            .setMessage(
                buildString {
                    append("fx %.2f    fy %.2f\n".format(r.fx, r.fy))
                    append("cx %.2f    cy %.2f\n".format(r.cx, r.cy))
                    append("k1 %.5f   k2 %.5f\n".format(r.k1, r.k2))
                    append("p1 %.5f   p2 %.5f\n\n".format(r.p1, r.p2))
                    append("重投影误差 %.3f px · %d 个视角\n".format(r.rms, r.views))
                    append("来源 %s · %s".format(CalibStore.sourceLabel(r), r.date))
                }
            )
            .setPositiveButton("关闭", null)
            .setNeutralButton("删除") { _, _ -> askDelete(i) }
            .show()
    }

    private fun askDelete(i: Int) {
        val r = prof?.rows?.getOrNull(i) ?: return
        MaterialAlertDialogBuilder(this)
            .setTitle("删除这一档？")
            .setMessage(
                "%.2f D（≈ %s）\nfx %.1f · RMS %.2f px\n%s\n\n删除后不可恢复。"
                    .format(r.d, distance(r.d), r.fx, r.rms, r.date)
            )
            .setPositiveButton("删除") { _, _ -> doDelete(i) }
            .setNegativeButton("取消", null)
            .show()
    }

    private fun doDelete(i: Int) {
        val p = prof ?: return
        if (i !in p.rows.indices) return
        val rows = p.rows.toMutableList().apply { removeAt(i) }
        if (CalibStore.save(this, p.copy(rows = rows))) {
            Log.i(TAG, "删除档位 index=$i，剩 ${rows.size} 个")
            reload()
        } else {
            Toast.makeText(this, "删除失败，详情见日志", Toast.LENGTH_LONG).show()
        }
    }

    // ---------------- 格式化 ----------------

    /** 屈光度 → 等效距离。接近 0 是对焦到无穷远。 */
    private fun distance(d: Float): String =
        if (d <= 0.05f) "∞" else "%.2f m".format(1f / d)

    /** `yyyy-MM-dd HH:mm` → `MM-dd HH:mm`；老的只到天的行原样显示。 */
    private fun shortDate(s: String): String = if (s.length >= 16) s.substring(5) else s
}
