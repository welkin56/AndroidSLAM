package com.example.slamcapture

/**
 * JNI 声明层：只做「Kotlin ↔ C++」的契约，不写业务。
 *
 * C++ 侧符号必须是 `Java_com_example_slamcapture_SlamBridge_<方法名>`。
 * C++ 那边的 `cout` / `printf` 去处是 **logcat（tag `slam`）**，不是 Run 窗口。
 */
object SlamBridge {

    init {
        System.loadLibrary("slam_jni")
    }

    // —— M4：构造 ORB_SLAM3::System ——

    /**
     * **只生效一次**，重复调用会被忽略（构造要几十秒，不能重来）。
     *
     * 只做「存路径 + 置加载中」就返回，真正的构造在别的线程上 —— 否则「启动」按钮卡死。
     *
     * 两条路径必须存在：路径不对时 ORB-SLAM3 用 `exit(-1)`，表现是 App 直接没了、
     * logcat 里什么都没有。
     *
     * @param loadFromFile 载入哪张旧图（绝对路径，**不带 `.osa`**）。null = 新建地图。
     */
    external fun nativeInit(vocabPath: String, settingsPath: String, loadFromFile: String?)

    /** 构造状态：0=没开始 1=加载中 2=就绪 3=失败。四态是必需的：「失败」得能显示出来。 */
    external fun nativeState(): Int

    external fun nativeIsReady(): Boolean

    /** 词袋加载了多久（毫秒），不在加载中时返回 0。**别自己掐表** —— Activity 重建会归零，而构造是进程级的。 */
    external fun nativeElapsedMs(): Long

    // —— M5：喂帧、喂 IMU、位姿 ——

    /** 每帧一次。**跑在相机线程上，里面绝不能阻塞** —— 实现里只是拷进队列。 */
    external fun nativePushImage(tsNs: Long, gray: ByteArray, w: Int, h: Int)

    /**
     * 一个完整样本调一次：**角速度在前、加速度在后**。
     *
     * 和 `ImuItem` 的顺序相反，分开传很容易写反 —— 而**写反不报错、只是跟踪整个乱掉**。
     */
    external fun nativePushImu(tsNs: Long, wx: Float, wy: Float, wz: Float,
                               ax: Float, ay: Float, az: Float)

    /** 开始吃帧。**不销毁 `System`** —— 停止之后再启动是立刻可用的。 */
    external fun nativeStart()

    /** 停止吃帧，并清空积压的队列（否则下次启动会吃到旧帧）。`System` 保留。 */
    external fun nativeStop()

    /** 4x4 **列主序**。返回 false = 还没有位姿。`out` 必须 ≥ 16 个元素。 */
    external fun nativeGetPose(out: FloatArray): Boolean

    /** ORB-SLAM3 的 `eTrackingState`：-1 没准备好 / 0 一帧没进来 / 1 惯性初始化中 / 2 OK / 3 刚丢 / 4 全丢。 */
    external fun nativeGetTrackingState(): Int

    /** 本次运行「地图被重置」的次数。[nativeStart] 归零。 */
    external fun nativeResetCount(): Int

    /** 当前轨迹有多少条。 */
    external fun nativeTrajSize(): Int

    /**
     * 2D 地图的轨迹：`out` 收 `3n` 个 float（世界米 `x y z`），返回点数。
     *
     * **stride 抽稀，不是截断**（覆盖全程、恒含最新点）—— 要的是形状，**别扩容取全**。
     */
    external fun nativeGetTrajectory(out: FloatArray): Int

    /**
     * 2D 地图的点云：`out` 收 `3n` 个 float（世界米 `x y z`），返回点数。
     *
     * 是 SLAM 线程**每秒重建一次的快照**，不是现读地图（全图枚举要抱地图的锁）。
     * 和 [nativeGetTrajectory] 一样抽稀（上限 5000），**同样别扩容取全**。
     */
    external fun nativeGetMapPoints(out: FloatArray): Int

    /** 小地图开关。关掉时 SLAM 线程连点云快照都不重建；重开时下一个空隙立刻重建。 */
    external fun nativeSetMapCloudEnabled(on: Boolean)

    // —— 存图 / 载图 ——

    /**
     * 把当前地图另存到 `path + ".osa"`（**不含扩展名**）。**只在 SLAM 停止之后调。**
     *
     * 只发请求就返回 —— 序列化整张 atlas 要几秒，卡主线程就是 ANR。进度看 [nativeSaveState]。
     */
    external fun nativeRequestSaveMap(path: String)

    /** 保存状态：0 空闲 / 1 排队 / 2 保存中 / 3 完成 / 4 失败。 */
    external fun nativeSaveState(): Int

    /**
     * 把 atlas 恢复成「上一次保存」的样子 —— 目标：最近一次成功存图的文件；
     * 没存过就用构造时载入的那张；都没有（新建地图）就清成一张空图。
     *
     * 一趟结束但没保存时调。**只发请求就返回**，进度看 [nativeRestoreState]。
     */
    external fun nativeRequestRestoreMap()

    /** 恢复状态：0 空闲 / 1 排队 / 2 恢复中 / 3 完成 / 4 失败。 */
    external fun nativeRestoreState(): Int

    /**
     * 把 atlas 换成 [path] 指向的那张（**不含扩展名**；空串 = 空图）。
     *
     * 入口页每次重新选都要调：`System` 只能构造一次，不换的话「新建地图」只是
     * 又开一张当前图，旧 atlas 原样留着。**只发请求**，进度同样看 [nativeRestoreState]。
     */
    external fun nativeRequestLoadAtlas(path: String)

    /** atlas 现在装的是哪一份（不含扩展名）。空串 = 新建 / 空图。 */
    external fun nativeLoadedMap(): String

    /**
     * atlas 有多大。`out` 要 ≥ 3，收 `[图数, 关键帧数, 地图点数]`。
     *
     * 载图之后这是界面上**唯一**看得出「图进来了」的地方 —— 载进来的图不是当前图，
     * 小窗、轨迹、测距都看不到它。
     */
    external fun nativeAtlasStats(out: IntArray)

    /** 构造失败的细分：0 没失败 / 1 载图失败 / 2 别的。 */
    external fun nativeFailureCode(): Int

    /** 把这一趟的位姿历史写成文件（TUM 格式）。**一次运行一份** —— [nativeStart] 会清掉上一趟。 */
    external fun nativeDumpTrajectory(path: String)

    // —— M6：画面中心的距离读数 ——

    /** 抽屉里那个开关。关掉时 C++ 连算都不算，但**不影响 SLAM 跟踪**。 */
    external fun nativeSetMeasuring(on: Boolean)

    /** `out` **必须 ≥ 7**（短了返回「不可测」）：`[0]`距离 `[1..3]`世界xyz `[4][5]`图像xy `[6]`附近特征点数。 */
    external fun nativeGetMeasure(out: FloatArray): Int

    // —— 多点折线 ——
    //
    // 折线整个存在 C++ 侧，因为它要存 `MapPoint*`。存坐标快照不行：BA 一直在挪地图点，
    // 走一圈之后快照就和地图差开 —— 标记飘到空中，**长度数字也跟着错**。

    /**
     * 把当前读数钉成一个点。没有有效读数返回 false。
     *
     * `snapTo >= 0` = 复制第 `snapTo` 个点的**身份**；-1 = 用十字当前的读数。
     */
    external fun nativeAddPinned(snapTo: Int): Boolean

    external fun nativeUndoPinned()

    external fun nativeClearPinned()

    /** 「断开」：最后一个点之后不再拉橡皮筋，下一个点会另起一条折线。 */
    external fun nativeClosePinnedChain()

    /**
     * 读折线。**刷新坐标、投影、算长度全在 C++ 做完**，这一侧只管画。
     *
     * `out` 要 ≥ `8n + 1`，返回**实际写了几个点**（可能小于真实点数 —— 那时扩容后下一帧再来）。
     *
     * ```
     * out[0] = n
     * 每点 3 个：px py flags
     * 每段 5 个：x0 y0 x1 y1 len        len < 0 表示这条不画
     * 最后 5 个：x0 y0 x1 y1 liveLen    liveLen < 0 表示没有橡皮筋
     * ```
     *
     * `flags` 的位：bit0 在相机前方 / bit1 之后断开 / bit2 不太准。
     */
    external fun nativeGetPinned(out: FloatArray): Int
}
