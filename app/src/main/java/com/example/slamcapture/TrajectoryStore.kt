package com.example.slamcapture

import java.io.File

/**
 * 磁盘上「图与文件夹」的归属规则，只在这里定义。
 *
 * map-<id>/ 里的轨迹都在同一坐标系（该图合并后的坐标系），读取时可直接叠加。
 * traj/ 里是尚未归档的轨迹。
 */
object TrajectoryStore {

    fun mapDir(session: File, mapId: Long): File = File(session, "map-$mapId")

    fun trajDir(session: File): File = File(session, "traj")

    fun recDir(session: File): File = File(session, "rec")

    /**
     * 能与 [curMapId] 直接叠加的轨迹，按文件名排序。
     * traj/ 中的轨迹不参与，因为它们不属于任何已保存的图。
     */
    fun tracesFor(session: File, curMapId: Long): List<File> =
        mapDir(session, curMapId)
            .listFiles { f -> f.isFile && f.name.startsWith("traj-") && f.name.endsWith(".txt") }
            ?.sorted()
            ?: emptyList()
}
