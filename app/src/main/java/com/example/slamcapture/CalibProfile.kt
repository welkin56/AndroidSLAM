package com.example.slamcapture

import android.content.Context
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.util.Log
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * 屈光度相差小于它就算「同一档」：复标会覆盖那一行，不另起一行。
 *
 * 不能用浮点 `==`：两次锁焦的读数几乎不会逐位相同，写相等等于把覆盖功能删掉。
 * 0.01 D 在 1 m 处约 1 cm，作为「同一次标定」的判定够严；比它远的差异是真实档位。
 */
const val SAME_D_EPS = 0.01f

/**
 * 一个对焦屈光度下的内参。
 *
 * [d] 是 `LENS_FOCUS_DISTANCE` 的读数（屈光度，越大越近），也是这张表唯一的索引。
 * 屈光度和 fx 近似线性（镜组位移 δ = f²·d），所以两次之间直接插值就够。
 */
data class CalibRow(
    val d: Float,
    val fx: Float,
    val fy: Float,
    val cx: Float,
    val cy: Float,
    val k1: Float,
    val k2: Float,
    val p1: Float,
    val p2: Float,
    /** auto | manual | factory | estimated */
    val source: String = "auto",
    val rms: Float = 0f,
    val views: Int = 0,
    val date: String = "",
)

/** 一台设备 + 一个输出分辨率的内参表。行按 [CalibRow.d] 升序。 */
data class CalibProfile(
    val model: String,
    val cameraId: String,
    val sensorOrientation: Int,
    val imuFrequency: Float,
    val rows: List<CalibRow>,
)

/**
 * 内参表的读写与插值。磁盘位置：[外部文件目录]/calib/profile.json，和 maps/ 同级、可 adb pull。
 *
 * 一张地图只记一套内参（写在地图目录的 session.json 里），运行时由 MainActivity 决定用哪套：
 * 新图从这张表插值，旧图直接读记录。
 */
object CalibStore {

    private const val TAG = "CalibStore"
    private const val DIR = "calib"
    private const val FILE = "profile.json"

    fun dir(ctx: Context): File = File(ctx.getExternalFilesDir(null) ?: ctx.filesDir, DIR)
    fun file(ctx: Context): File = File(dir(ctx), FILE)

    // ---------------- 读写 ----------------

    fun load(ctx: Context): CalibProfile? {
        val f = file(ctx)
        if (!f.isFile) return null
        return try {
            val o = JSONObject(f.readText())
            val arr = o.optJSONArray("rows") ?: JSONArray()
            val rows = (0 until arr.length()).mapNotNull { i ->
                arr.optJSONObject(i)?.let(::rowFromJson)
            }.sortedBy { it.d }
            CalibProfile(
                model = o.optString("model", ""),
                cameraId = o.optString("camera_id", ""),
                sensorOrientation = o.optInt("sensor_orientation", 0),
                imuFrequency = o.optDouble("imu_frequency_hz", 0.0).toFloat(),
                rows = rows,
            )
        } catch (e: Exception) {
            Log.e(TAG, "读 profile 失败：${f.absolutePath}", e)
            null
        }
    }

    fun save(ctx: Context, p: CalibProfile): Boolean {
        val f = file(ctx)
        return try {
            f.parentFile?.mkdirs()
            val arr = JSONArray()
            p.rows.sortedBy { it.d }.forEach { arr.put(rowToJson(it)) }
            val o = JSONObject()
                .put("version", 1)
                .put("model", p.model)
                .put("camera_id", p.cameraId)
                .put("sensor_orientation", p.sensorOrientation)
                .put("imu_frequency_hz", p.imuFrequency.toDouble())
                .put("rows", arr)
            f.writeText(o.toString(2))
            Log.i(TAG, "写入 profile：${f.absolutePath}（${p.rows.size} 行）")
            true
        } catch (e: Exception) {
            Log.e(TAG, "写 profile 失败：${f.absolutePath}", e)
            false
        }
    }

    private fun rowToJson(r: CalibRow) = JSONObject()
        .put("d", r.d.toDouble())
        .put("fx", r.fx.toDouble()).put("fy", r.fy.toDouble())
        .put("cx", r.cx.toDouble()).put("cy", r.cy.toDouble())
        .put("k1", r.k1.toDouble()).put("k2", r.k2.toDouble())
        .put("p1", r.p1.toDouble()).put("p2", r.p2.toDouble())
        .put("source", r.source)
        .put("rms", r.rms.toDouble())
        .put("views", r.views)
        .put("date", r.date)

    private fun rowFromJson(o: JSONObject) = CalibRow(
        d = o.optDouble("d", 0.0).toFloat(),
        fx = o.optDouble("fx", 0.0).toFloat(),
        fy = o.optDouble("fy", 0.0).toFloat(),
        cx = o.optDouble("cx", 0.0).toFloat(),
        cy = o.optDouble("cy", 0.0).toFloat(),
        k1 = o.optDouble("k1", 0.0).toFloat(),
        k2 = o.optDouble("k2", 0.0).toFloat(),
        p1 = o.optDouble("p1", 0.0).toFloat(),
        p2 = o.optDouble("p2", 0.0).toFloat(),
        source = o.optString("source", "auto"),
        rms = o.optDouble("rms", 0.0).toFloat(),
        views = o.optInt("views", 0),
        date = o.optString("date", ""),
    )

    // ---------------- 插值 ----------------

    /**
     * 按屈光度取内参：相邻两行线性插值，两端外夹到最近一行（这时等价于"取最近"）。
     * k2/p1/p2 量小又难辨识，直接取最近那一行的整份，不插。
     */
    fun interpolate(rows: List<CalibRow>, d: Float): CalibRow? {
        if (rows.isEmpty()) return null
        val s = rows.sortedBy { it.d }
        if (s.size == 1 || d <= s.first().d) return s.first()
        if (d >= s.last().d) return s.last()

        val i = s.indexOfLast { it.d <= d }.coerceAtLeast(0)
        val a = s[i]
        val b = s[i + 1]
        val t = if (b.d == a.d) 0f else (d - a.d) / (b.d - a.d)
        fun lerp(x: Float, y: Float) = x + (y - x) * t
        val near = if (t < 0.5f) a else b
        return a.copy(
            d = d,
            fx = lerp(a.fx, b.fx), fy = lerp(a.fy, b.fy),
            cx = lerp(a.cx, b.cx), cy = lerp(a.cy, b.cy),
            k1 = lerp(a.k1, b.k1),
            k2 = near.k2, p1 = near.p1, p2 = near.p2,
            source = near.source,
            rms = near.rms, views = near.views, date = near.date,
        )
    }

    /** 表格没覆盖到的屈光度差多少（D）。用于判断"超出标定范围"要不要提示。 */
    fun distanceToTable(rows: List<CalibRow>, d: Float): Float {
        if (rows.isEmpty()) return Float.MAX_VALUE
        return rows.minOf { kotlin.math.abs(it.d - d) }
    }

    // ---------------- 兜底来源 ----------------

    /**
     * 模板里的值（`assets/phone.yaml` 的 `Camera1.*`），没有任何 profile 时用。
     * 记为 estimated —— 它在 9-20 那版标定里是实测值，但换了设备就不成立。
     */
    fun fallback(ctx: Context): CalibRow? = try {
        val txt = ctx.assets.open("phone.yaml").bufferedReader().use { it.readText() }
        fun v(key: String): Float? =
            Regex("(?m)^\\Q$key\\E:\\s*([-+0-9.eE]+)").find(txt)
                ?.groupValues?.get(1)?.toFloatOrNull()

        val fx = v("Camera1.fx")
        val fy = v("Camera1.fy")
        val cx = v("Camera1.cx")
        val cy = v("Camera1.cy")
        if (fx != null && fy != null && cx != null && cy != null) {
            CalibRow(
                d = DEFAULT_D, fx = fx, fy = fy, cx = cx, cy = cy,
                k1 = v("Camera1.k1") ?: 0f, k2 = v("Camera1.k2") ?: 0f,
                p1 = v("Camera1.p1") ?: 0f, p2 = v("Camera1.p2") ?: 0f,
                source = "estimated",
            )
        } else null
    } catch (e: Exception) {
        Log.e(TAG, "读 assets/phone.yaml 失败", e)
        null
    }

    /**
     * 出厂内参：`LENS_INTRINSIC_CALIBRATION` 是 active array 像素坐标，按输出尺寸缩放。
     *
     * 畸变留 0 —— `LENS_DISTORTION` 的系数顺序没有可靠来源可核，宁可不用，也不猜。
     * 所以这一档只适合"没有板子、又想先跑起来"的情况，界面上要标出来。
     *
     * 本机这个值是焦距/传感器尺寸算出来的名义值（缩放回 640 是 422.7，比实测低 3.8%），
     * 不是单机标定，别指望它比模板值准。
     */
    fun factory(ctx: Context, cameraId: String): CalibRow? = try {
        val mgr = ctx.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        val c = mgr.getCameraCharacteristics(cameraId)
        val calib = c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)
        val active = c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)
        if (calib == null || calib.size < 4 || active == null ||
            active.width() <= 0 || active.height() <= 0
        ) null else {
            val sx = CameraCapture.W.toFloat() / active.width()
            val sy = CameraCapture.H.toFloat() / active.height()
            CalibRow(
                d = DEFAULT_D,
                fx = calib[0] * sx, fy = calib[1] * sy,
                cx = calib[2] * sx, cy = calib[3] * sy,
                k1 = 0f, k2 = 0f, p1 = 0f, p2 = 0f,
                source = "factory",
            )
        }
    } catch (e: Exception) {
        Log.e(TAG, "读出厂内参失败", e)
        null
    }

    // ---------------- 地图目录里的记录 ----------------

    /**
     * 把这一趟用的内参写进 <地图目录>/session.json 的 `intrinsics` 段。
     * 打开同一张图时直接用它，不再看当前对焦 —— 换台设备、换个光线都不该改变这张图的内参。
     */
    fun writeMapIntrinsics(dir: File, row: CalibRow, model: String, cameraId: String): Boolean {
        return try {
            val f = File(dir, "session.json")
            val o = if (f.isFile) JSONObject(f.readText()) else JSONObject()
            o.put(
                "intrinsics", JSONObject()
                    .put("diopter", row.d.toDouble())
                    .put("fx", row.fx.toDouble()).put("fy", row.fy.toDouble())
                    .put("cx", row.cx.toDouble()).put("cy", row.cy.toDouble())
                    .put("k1", row.k1.toDouble()).put("k2", row.k2.toDouble())
                    .put("p1", row.p1.toDouble()).put("p2", row.p2.toDouble())
                    .put("source", row.source)
                    .put("model", model)
                    .put("camera_id", cameraId)
                    .put("recorded", stamp())
            )
            f.writeText(o.toString(2))
            Log.i(TAG, "地图内参已记录：%s fx=%.3f".format(f.absolutePath, row.fx))
            true
        } catch (e: Exception) {
            Log.e(TAG, "写地图内参失败：${dir.absolutePath}", e)
            false
        }
    }

    fun readMapIntrinsics(dir: File): CalibRow? = try {
        val f = File(dir, "session.json")
        if (!f.isFile) null else {
            val o = JSONObject(f.readText()).optJSONObject("intrinsics")
            if (o == null) null else CalibRow(
                d = o.optDouble("diopter", DEFAULT_D.toDouble()).toFloat(),
                fx = o.optDouble("fx", 0.0).toFloat(),
                fy = o.optDouble("fy", 0.0).toFloat(),
                cx = o.optDouble("cx", 0.0).toFloat(),
                cy = o.optDouble("cy", 0.0).toFloat(),
                k1 = o.optDouble("k1", 0.0).toFloat(),
                k2 = o.optDouble("k2", 0.0).toFloat(),
                p1 = o.optDouble("p1", 0.0).toFloat(),
                p2 = o.optDouble("p2", 0.0).toFloat(),
                source = o.optString("source", "auto"),
            ).takeIf { it.fx > 0f }
        }
    } catch (e: Exception) {
        Log.e(TAG, "读地图内参失败：${dir.absolutePath}", e)
        null
    }

    /** 来源的一句话说明，界面直接显示。 */
    fun sourceLabel(row: CalibRow): String = when (row.source) {
        "auto" -> "实测"
        "manual" -> "手填"
        "factory" -> "出厂值"
        else -> "未标定"
    }

    private fun stamp(): String =
        SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss", Locale.US).format(Date())

    /** 没有屈光度信息时的占位档位（1 m）。 */
    private const val DEFAULT_D = 1.0f
}
