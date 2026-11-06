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
     * 替换模板里的 `IMU.T_b_c1` 数据块、`IMU.Frequency`，以及（有标定值时）四个噪声键。
     * 其余原样保留（注释也在）。找不到对应块时返回原文本 —— 宁可沿用模板，也不要生成一份坏的 yaml。
     *
     * [noise] 为 null 时四个噪声键一字不动：没标过就用模板值。
     */
    fun patch(
        template: String,
        tbc: FloatArray,
        imuFrequency: Float,
        noise: ImuNoise? = null,
    ): String {
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

        replaceValue(out, "IMU.Frequency", String.format(Locale.US, "%.1f", imuFrequency))
        noise?.let { writeNoise(out, it) }
        return out.joinToString("\n")
    }

    /** 只替换 `key:` 后面的数值，行尾的 `# 注释` 保持不动。找不到该键就什么都不做。 */
    private fun replaceValue(lines: MutableList<String>, key: String, value: String) {
        for (i in lines.indices) {
            if (!lines[i].startsWith("$key:")) continue
            val hashAt = lines[i].indexOf('#')
            val tail = if (hashAt >= 0) "   " + lines[i].substring(hashAt) else ""
            lines[i] = "$key: $value$tail"
            return
        }
    }

    /**
     * 四个噪声键整行重写，注释跟着值一起换。
     *
     * 不保留模板里的旧注释：那上面写着上一次标定的日期和三轴均值，值换了注释不换，
     * 读到的人会以为这行还是那次的结果。
     */
    private fun writeNoise(lines: MutableList<String>, n: ImuNoise) {
        val day = n.recorded.take(10).ifEmpty { "未知日期" }
        val from = "App Allan 标定 $day"
        setNoiseLine(
            lines, "IMU.NoiseGyro", n.noiseGyro, "rad/s/√Hz",
            String.format(Locale.US, "%s（本次实测 %.3e）", from, n.noiseGyro),
        )
        setNoiseLine(
            lines, "IMU.NoiseAcc", n.noiseAcc, "m/s²/√Hz",
            String.format(Locale.US, "%s（本次实测 %.3e）", from, n.noiseAcc),
        )
        // 游走项只在本次真测出来时才换。短录制的曲线停在平顶段，
        // 那时换算出的 K 取决于窗口位置，写进去等于编一个数。
        if (n.walkUpdated) {
            setNoiseLine(lines, "IMU.GyroWalk", n.gyroWalk, "rad/s²/√Hz", "$from（本次实测）")
            setNoiseLine(lines, "IMU.AccWalk", n.accWalk, "m/s³/√Hz", "$from（本次实测）")
        }
    }

    /** 整行替换；[value] 非有限或 ≤ 0 时跳过，避免写出 inf/0 让惯性初始化卡住。 */
    private fun setNoiseLine(
        lines: MutableList<String>,
        key: String,
        value: Double,
        unit: String,
        note: String,
    ) {
        if (!value.isFinite() || value <= 0.0) return
        val idx = lines.indexOfFirst { it.startsWith("$key:") }
        if (idx < 0) return
        val text = String.format(Locale.US, "%s: %.3e", key, value)
        lines[idx] = String.format(Locale.US, "%-28s# %-12s %s", text, unit, note).trimEnd()
    }
}
