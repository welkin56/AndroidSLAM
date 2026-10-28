package com.example.slamcapture

/**
 * JNI 声明层：只定义 Kotlin 与 C++ 的调用契约，不写业务。
 *
 * C++ 符号必须是 Java_com_example_slamcapture_SlamBridge_<方法名>。
 * C++ 的 cout/printf 输出到 logcat（tag slam）。
 */
object SlamBridge {

    init {
        System.loadLibrary("slam_jni")
    }

    // M4：构造 ORB_SLAM3::System

    /**
     * 只生效一次，重复调用会被忽略。同步只保存路径并置为加载中，构造在后台线程完成。
     *
     * @param loadFromFile 要载入的旧图绝对路径（不带 .osa）；null 表示新建地图。
     */
    external fun nativeInit(vocabPath: String, settingsPath: String, loadFromFile: String?)

    /** 构造状态：0 未开始 / 1 加载中 / 2 就绪 / 3 失败。 */
    external fun nativeState(): Int

    external fun nativeIsReady(): Boolean

    /** 已加载的毫秒数，不在加载中时返回 0。由 C++ 计时，Activity 重建不会归零。 */
    external fun nativeElapsedMs(): Long

    // M5：喂帧、喂 IMU、位姿

    /** 每帧调用一次，运行在相机线程上，不能阻塞；实现只拷贝进队列。 */
    external fun nativePushImage(tsNs: Long, gray: ByteArray, w: Int, h: Int)

    /**
     * 一个完整样本调用一次：角速度在前、加速度在后。
     * 与 ImuItem 的顺序相反，写反不会报错，但会导致跟踪异常。
     */
    external fun nativePushImu(tsNs: Long, wx: Float, wy: Float, wz: Float,
                               ax: Float, ay: Float, az: Float)

    /**
     * 注入相机内参。**必须在 [nativeStart] 之前**，跑着帧时调用会被忽略。
     * 一张地图只用一套内参，切换内参要重开一张图。
     */
    external fun nativeSetIntrinsics(fx: Float, fy: Float, cx: Float, cy: Float,
                                     k1: Float, k2: Float, p1: Float, p2: Float)

    /** 开始吃帧，不销毁 System。 */
    external fun nativeStart()

    /** 停止吃帧并清空队列，System 保留。 */
    external fun nativeStop()

    /** 4x4 列主序；返回 false 表示还没有位姿。out 至少 16 个元素。 */
    external fun nativeGetPose(out: FloatArray): Boolean

    /** eTrackingState：-1 未就绪 / 0 无帧 / 1 惯性初始化 / 2 OK / 3 刚丢 / 4 完全丢。 */
    external fun nativeGetTrackingState(): Int

    /** 本次运行的地图重置次数，[nativeStart] 归零。 */
    external fun nativeResetCount(): Int

    /** 当前轨迹的点数。 */
    external fun nativeTrajSize(): Int

    /**
     * 2D 地图轨迹：out 收 3n 个 float（世界坐标 x y z），返回点数。
     * 按 stride 抽稀，覆盖全程并包含最新点。
     */
    external fun nativeGetTrajectory(out: FloatArray): Int

    /**
     * 2D 地图点云：out 收 3n 个 float（世界坐标 x y z），返回点数。
     * 数据是 SLAM 线程每秒重建的快照，上限 5000 点。
     */
    external fun nativeGetMapPoints(out: FloatArray): Int

    /** 小地图开关。关闭时 SLAM 线程不重建点云快照。 */
    external fun nativeSetMapCloudEnabled(on: Boolean)

    // 存图 / 载图

    /**
     * 把当前地图另存到 path + ".osa"（不含扩展名），只在 SLAM 停止后调用。
     * 只发请求，序列化在后台完成；进度看 [nativeSaveState]。
     */
    external fun nativeRequestSaveMap(path: String)

    /** 保存状态：0 空闲 / 1 排队 / 2 保存中 / 3 完成 / 4 失败。 */
    external fun nativeSaveState(): Int

    /**
     * 把 atlas 恢复到上一次保存的状态：优先最近一次成功存图，其次构造时载入的图，
     * 都没有则清成空图。一趟结束但未保存时调用，进度看 [nativeRestoreState]。
     */
    external fun nativeRequestRestoreMap()

    /** 恢复状态：0 空闲 / 1 排队 / 2 恢复中 / 3 完成 / 4 失败。 */
    external fun nativeRestoreState(): Int

    /**
     * 把 atlas 换成 [path] 指定的图（不含扩展名，空串表示空图）。
     * 入口页重新选择时调用，只发请求；进度看 [nativeRestoreState]。
     */
    external fun nativeRequestLoadAtlas(path: String)

    /** atlas 当前对应的路径（不含 .osa）；空串表示新建或空图。 */
    external fun nativeLoadedMap(): String

    /** atlas 规模。out 至少 3 个，依次为图数、关键帧数、地图点数。 */
    external fun nativeAtlasStats(out: IntArray)

    /**
     * atlas 中有关键帧的图（按 id 升序）。三组出参一一对应，返回实际写入数。
     * 数组装不下时少给；返回值等于数组长度时应扩容重读。
     */
    external fun nativeMapList(ids: LongArray, kfs: IntArray, mps: IntArray): Int

    /**
     * 指定图的点云（该图自己的坐标系）。out 收 3n 个 float，返回点数。
     * 按 stride 抽稀到 out.size / 3，只在选图和刷新时调用。
     */
    external fun nativeMapCloud(mapId: Long, out: FloatArray): Int

    /**
     * 当前轨迹此刻所属的图，-1 表示没有。
     * 与 [nativeLastTrajMapId] 不同：这里是实时值。
     */
    external fun nativeTrajectoryMapId(): Long

    /** 构造失败细分：0 没失败 / 1 载图失败 / 2 其他。 */
    external fun nativeFailureCode(): Int

    /** 把本趟位姿历史写成 TUM 格式文件。 */
    external fun nativeDumpTrajectory(path: String)

    // M6：画面中心的距离读数

    /** 实时测距开关。关闭时只停止计算，不影响 SLAM 跟踪。 */
    external fun nativeSetMeasuring(on: Boolean)

    /** out 至少 7 个：[0] 距离、[1..3] 世界 xyz、[4][5] 图像 xy、[6] 附近特征点数。 */
    external fun nativeGetMeasure(out: FloatArray): Int

    // 多点折线
    //
    // 折线保存在 C++ 侧，因为需要保存 MapPoint*；坐标会随 BA 变化。

    /**
     * 把当前读数钉成一个点，没有有效读数时返回 false。
     * snapTo >= 0 时复制第 snapTo 个点的身份；-1 表示使用十字当前读数。
     */
    external fun nativeAddPinned(snapTo: Int): Boolean

    external fun nativeUndoPinned()

    external fun nativeClearPinned()

    /** 断开折线：下一个点另起一条，不再拉橡皮筋。 */
    external fun nativeClosePinnedChain()

    /**
     * 读折线；刷新坐标、投影和长度计算都在 C++ 完成。
     *
     * out 至少 8n + 1，返回实际写入点数（装不下时少给）。
     * 布局：
     *   out[0] = n
     *   每点 3 个：px py flags
     *   每段 5 个：x0 y0 x1 y1 len（len 小于 0 表示不画）
     *   最后 5 个：x0 y0 x1 y1 liveLen（小于 0 表示没有橡皮筋）
     * flags：bit0 在相机前方 / bit1 之后断开 / bit2 不太准。
     */
    external fun nativeGetPinned(out: FloatArray): Int

    // 路径点
    //
    // 锚定在关键帧上并保存相对位姿，地图变化后会自动跟随。

    /** 在当前位置记录路径点；发请求即返回，下一帧在 SLAM 线程执行。 */
    external fun nativeRequestAddWaypoint(name: String)

    /** 加路径点状态：0 没试过 / 1 排队 / 2 成功 / 3 失败。 */
    external fun nativeWaypointState(): Int

    /** 失败原因。成功时是空串。 */
    external fun nativeWaypointMsg(): String

    external fun nativeRenameWaypoint(id: Int, name: String)

    external fun nativeDeleteWaypoint(id: Int)

    /** 每次变化加一，界面据此重建图层。 */
    external fun nativeWaypointRevision(): Long

    /** 路径点总条数，包含失效的。 */
    external fun nativeWaypointCount(): Int

    /** 下一个路径点 id，单调递增，删除后也不会复用。 */
    external fun nativeNextWaypointId(): Int

    /**
     * 读取路径点。pos 每点 3 个 float；meta 每点 id, mapId, kfId, flags
     * （bit0 坐标有效 / bit1 在当前图）；names 接收名字。
     * 返回实际写入条数，装不下时少给。名字元素可以为 null。
     */
    external fun nativeGetWaypoints(pos: FloatArray, meta: LongArray, names: Array<String?>): Int

    // 轨迹归属 / 合并链

    /** 本趟最后一段轨迹所属的图（-1 表示没有）。应在 nativeStop() 之后立即读取。 */
    external fun nativeLastTrajMapId(): Long

    /** 本趟合并链：每条 10 个 float，survivor absorbed s qx qy qz qw tx ty tz。 */
    external fun nativeLastMerges(out: FloatArray): Int

    /**
     * 把轨迹文件按合并变换写入 [dst]。格式为 8 列：time tx ty tz qx qy qz qw（Twc）。
     * 写入副本，调用方负责全部成功后再统一重命名。
     */
    external fun nativeTransformTrajFile(src: String, dst: String,
                                         s: Float, qx: Float, qy: Float, qz: Float, qw: Float,
                                         tx: Float, ty: Float, tz: Float): Boolean

    /**
     * 历史轨迹显示所需的坐标系信息：每条 9 个 float，
     * id, s, qx, qy, qz, qw, tx, ty, tz。第一条的 id 是显示用图（没有图时返回 0）。
     * 多跳合并已经合成；out 至少 9 乘以期望条数。
     */
    external fun nativeDisplayTransforms(out: FloatArray): Int

    /**
     * 读取已归档轨迹并按上述变换换算，out 收 3n 个 float（世界 xyz），返回点数。
     * 按 stride 抽稀到 out.size / 3。
     */
    external fun nativeReadTrajFile(path: String,
                                    s: Float, qx: Float, qy: Float, qz: Float, qw: Float,
                                    tx: Float, ty: Float, tz: Float, out: FloatArray): Int
}
