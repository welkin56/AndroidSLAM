package com.example.slamcapture

import java.io.File

/**
 * 磁盘上「图 ⇔ 文件夹」的归属规则，只在这里定义。
 *
 * `map-<id>/` 里的轨迹**全部在同一个坐标系**（那张图合并之后的坐标系）—— 这是唯一的
 * 不变量，读的时候直接叠，不用查任何表。`traj/` 里的是还没归档的（存图没成功的那趟）。
 */
object TrajectoryStore {

    fun mapDir(session: File, mapId: Long): File = File(session, "map-$mapId")

    fun trajDir(session: File): File = File(session, "traj")

    fun recDir(session: File): File = File(session, "rec")

    /**
     * 能和 [curMapId] 直接叠的轨迹（按文件名排序）。
     *
     * `traj/` 里的**不参与** —— 它们不属于任何存下来的图，坐标系和这张图不一定一致。
     */
    fun tracesFor(session: File, curMapId: Long): List<File> =
        mapDir(session, curMapId)
            .listFiles { f -> f.isFile && f.name.startsWith("traj-") && f.name.endsWith(".txt") }
            ?.sorted()
            ?: emptyList()
}
