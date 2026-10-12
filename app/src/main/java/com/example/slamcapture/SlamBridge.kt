package com.example.slamcapture

/**
 * JNI 声明层：只做「Kotlin ↔ C++」的契约，不写业务。
 *
 * C++ 侧符号必须是 `Java_com_example_slamcapture_SlamBridge_<方法名>`；
 * `external fun` = 声明「实现在 native 里」，Kotlin 编译器只登记签名不产码。
 * `loadLibrary("slam_jni")` 对应 libslam_jni.so，类首次使用时装载一次。
 *
 * ⚠️ C++ 那边的 `cout` / `cerr` / `printf` 去处是 **logcat（tag `slam`）**，
 * **不是 Android Studio 的 Run 窗口** —— 重定向装在 `JNI_OnLoad` 里，见 jni_bridge.cpp。
 */
object SlamBridge {

    init {
        System.loadLibrary("slam_jni")
    }

    // —— M4：构造 ORB_SLAM3::System ——

    /**
     * **只生效一次**，重复调用会被忽略（构造一次要十几到几十秒，不能重来）。
     *
     * 同步只做「存路径 + 置加载中」，真正的构造在 C++ 那边的线程上跑，所以这个调用
     * **立刻返回**——否则「启动」按钮会被卡死十几到几十秒。
     *
     * 两条路径都必须指向**真实存在的文件**：`System` 会把它们交给 `cv::FileStorage`
     * 和 `loadFromTextFile`，路径不对时 ORB-SLAM3 用 `exit(-1)` 报错，
     * 表现是 App 干净地消失、logcat 里什么也没有。
     *
     * @param loadFromFile 载入哪张旧图。**M4 还不支持**，传 null。
     */
    external fun nativeInit(vocabPath: String, settingsPath: String, loadFromFile: String?)

    /**
     * 构造状态：0=没开始 1=加载中 2=就绪 3=失败。
     *
     * 用四态而不是布尔：「失败」必须能显示出来，否则界面就是「点了一下，一直转圈」。
     */
    external fun nativeState(): Int

    external fun nativeIsReady(): Boolean

    /**
     * 词袋已经加载了多久（毫秒）。不在加载中时返回 0。
     *
     * **不要用 Kotlin 自己掐表** —— 构造是进程级的事实，而 Activity 重建时（比如从入口页
     * 再进来一次）Kotlin 那边的计时会归零，`System` 却还在造，界面就成了「刚点下然后
     * 一直转圈」。谁干的活谁报时间。
     */
    external fun nativeElapsedMs(): Long

    // —— M5：喂帧、喂 IMU、位姿 ——

    /**
     * 每帧一次。**跑在相机线程上，里面绝不能阻塞**（铁律 1）—— 实现里只是拷进队列。
     *
     * **不做零拷贝**（STAGE5 §5）：`ByteArray` 有三个持有者（落盘 / 预览 / SLAM），
     * 复用一个 buffer 会让它们读到被覆盖的帧。
     */
    external fun nativePushImage(tsNs: Long, gray: ByteArray, w: Int, h: Int)

    /**
     * 每来一个完整样本调一次：**一个时间戳 + 角速度 + 加速度**。
     *
     * ⚠️ 这不是 §6 写的那种「陀螺/加速度分两次调、由 C++ 配对」。改的理由：
     * `IMU::Point` 要的就是完整样本，而配对逻辑 [`ImuCollector`] 本来就有（还带配对偏差统计）。
     * 顺带关掉一个**静默**的坑 —— 那个结构体的参数顺序是「加速度在前」，
     * 和我们的 `ImuItem` 正好相反，分开传很容易写反，而**写反不报错、只是跟踪整个乱掉**。
     */
    external fun nativePushImu(tsNs: Long, wx: Float, wy: Float, wz: Float,
                               ax: Float, ay: Float, az: Float)

    /** 开始吃帧。**不销毁 `System`** —— 所以「停止」之后再「启动」是立刻可用的。 */
    external fun nativeStart()

    /** 停止吃帧，并**清空积压的队列**（否则下次启动会吃到停止前的旧帧）。`System` 保留。 */
    external fun nativeStop()

    /** 4x4 **列主序**。返回 false = 还没有位姿。**传进来的数组必须 ≥ 16 个元素。** */
    external fun nativeGetPose(out: FloatArray): Boolean

    /** ORB-SLAM3 的 `eTrackingState`：-1 还没准备好 / 0 一帧没进来 / 1 惯性初始化中 / 2 OK / 3 刚丢 / 4 完全丢。 */
    external fun nativeGetTrackingState(): Int

    /** 本次运行里「地图被重置」的次数。[nativeStart] 归零。给 UI 显示用。 */
    external fun nativeResetCount(): Int

    /** 当前轨迹有多少条。**只含跟踪 `OK` 的帧** —— 没定位的帧位姿是单位阵，不算数。 */
    external fun nativeTrajSize(): Int

    /**
     * 把这一趟的位姿历史写成文件（TUM 格式，和 PC 的 `CameraTrajectory.txt` 同款）。
     * **一次运行一份** —— [nativeStart] 会把上一趟的清掉。
     */
    external fun nativeDumpTrajectory(path: String)

    // —— M6：画面中心的距离读数 ——

    /** 抽屉里那个开关。关掉时 C++ 那边连算都不算，但**不影响 SLAM 跟踪**。 */
    external fun nativeSetMeasuring(on: Boolean)

    /** ⚠️ `out` **必须 ≥ 7**（短了直接返回「不可测」）：`[0]`距离 `[1..3]`世界xyz `[4][5]`图像xy `[6]`附近特征点数。 */
    external fun nativeGetMeasure(out: FloatArray): Int

    // —— 多点折线 ——
    //
    // ⚠️ **折线整个存在 C++ 侧**，因为它要存 `MapPoint*`。存坐标快照是不行的：
    // BA 一直在挪地图点，走一圈之后快照就和地图差开了 —— 标记飘到空中，
    // **而且长度数字也跟着错**。刷新 / 投影 / 算长度一次给完，这一侧只负责画。

    /**
     * 把当前读数钉成一个点。没有有效读数返回 false。
     *
     * `snapTo >= 0` = **复制第 `snapTo` 个点的身份**（吸到已有点用）；
     * -1 = 用十字当前的读数。
     */
    external fun nativeAddPinned(snapTo: Int): Boolean

    external fun nativeUndoPinned()

    external fun nativeClearPinned()

    /** 「断开」：最后一个点之后不再拉橡皮筋，下一个点会另起一条折线。 */
    external fun nativeClosePinnedChain()

    /**
     * 读折线：**刷新坐标、投影、算长度全在这一侧做完**，Kotlin 只管画。
     *
     * `out` 要 ≥ `8n + 1`，**返回实际写了几个点**（可能小于真实点数 —— 那时扩容后下一帧再来）。
     *
     * ```
     * out[0] = n
     * 每点 3 个：px py flags
     * 每段 5 个：x0 y0 x1 y1 len      ← **已经是画面上的两个端点**；len < 0 ⟹ 不画
     * 最后 5 个：x0 y0 x1 y1 liveLen  ← 橡皮筋（最后一个点 → 十字）；liveLen < 0 ⟹ 没有
     * ```
     *
     * `flags` 的位：bit0 在相机前方 / bit1 之后断开 / bit2 不太准。
     *
     * ⚠️ 线段两端**有时不是真实投影** —— 端点在相机后面时是「沿线段插值到相机平面前一点点」
     * 再投的，于是线朝**正确方向**延伸出屏幕、被画布裁掉。细节见 `slam_engine.cpp`。
     */
    external fun nativeGetPinned(out: FloatArray): Int
}
