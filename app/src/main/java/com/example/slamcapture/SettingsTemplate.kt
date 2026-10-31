package com.example.slamcapture

import java.util.Locale

/**
 * 把 `assets/phone.yaml` 改成这一趟要用的 settings。
 *
 * **内参不写进 yaml**：真正生效的是 `SlamBridge.nativeSetIntrinsics()`，在 `nativeStart()` 前注入。
 * 这里只改两样 System 构造期就吃掉、之后没有安全改法的东西：相机–IMU 外参和 IMU 频率。
 */
object SettingsTemplate {

    /**
     * `IMU.T_b_c1` 的 4x4（行主序，`p_body = T * p_cam`）。
     *
     * 前三列的每一列是相机某个轴在**设备**坐标系里的方向：
     * 相机 X = 图像向右、Y = 图像向下、Z = 朝场景。相机 Z 恒为设备 -Z（后摄朝外）。
     * 行列式恒为 +1。90° 那行和原 phone.yaml 里手推的结果逐位一致。
     */
    fun tbc(sensorOrientation: Int): FloatArray {
        val cx: FloatArray
        val cy: FloatArray
        when (((sensorOrientation % 360) + 360) % 360) {
            0 -> { cx = floatArrayOf(1f, 0f, 0f); cy = floatArrayOf(0f, -1f, 0f) }
            90 -> { cx = floatArrayOf(0f, -1f, 0f); cy = floatArrayOf(-1f, 0f, 0f) }
            180 -> { cx = floatArrayOf(-1f, 0f, 0f); cy = floatArrayOf(0f, 1f, 0f) }
            else -> { cx = floatArrayOf(0f, 1f, 0f); cy = floatArrayOf(1f, 0f, 0f) }
        }
        val cz = floatArrayOf(0f, 0f, -1f)
        return floatArrayOf(
            cx[0], cy[0], cz[0], 0f,
            cx[1], cy[1], cz[1], 0f,
            cx[2], cy[2], cz[2], 0f,
            0f, 0f, 0f, 1f,
        )
    }

    /**
     * 替换模板里的 `IMU.T_b_c1` 数据块和 `IMU.Frequency`，其余原样保留（注释也在）。
     * 找不到对应块时返回原文本 —— 宁可沿用模板，也不要生成一份坏的 yaml。
     */
    fun patch(template: String, tbc: FloatArray, imuFrequency: Float): String {
        val out = template.lines().toMutableList()

        val head = out.indexOfFirst { it.startsWith("IMU.T_b_c1:") }
        if (head >= 0) {
            val dataAt = (head until out.size).firstOrNull {
                out[it].trimStart().startsWith("data:")
            } ?: -1
            val endAt = if (dataAt < 0) -1 else (dataAt until out.size).firstOrNull {
                out[it].contains("]")
            } ?: -1
            if (dataAt >= 0 && endAt >= 0) {
                val rows = (0 until 4).map { r ->
                    val values = (0 until 4).joinToString(", ") { c ->
                        String.format(Locale.US, "%.1f", tbc[r * 4 + c])
                    }
                    val open = if (r == 0) "   data: [ " else "          "
                    val close = if (r == 3) " ]" else ","
                    open + values + close
                }
                out.subList(dataAt, endAt + 1).clear()
                out.addAll(dataAt, rows)
            }
        }

        for (i in out.indices) {
            if (out[i].startsWith("IMU.Frequency:")) {
                out[i] = "IMU.Frequency: " + String.format(Locale.US, "%.1f", imuFrequency)
            }
        }
        return out.joinToString("\n")
    }
}
