package com.example.slamcapture

/**
 * JNI 声明层：只做「Kotlin ↔ C++」的契约，不写业务。
 *
 * C++ 侧符号必须是 `Java_com_example_slamcapture_SlamBridge_<方法名>`；
 * `external fun` = 声明「实现在 native 里」，Kotlin 编译器只登记签名不产码。
 * `loadLibrary("slam_jni")` 对应 libslam_jni.so，类首次使用时装载一次。
 */
object SlamBridge {

    init {
        System.loadLibrary("slam_jni")
    }

    /** M1 自检用：让 C++ 回一句话，证明桥是通的。 */
    external fun nativeHello(): String

    /**
     * M2 自检用：让 C++ 往 cout / cerr / printf 各写一行。
     *
     * 它们的去处是 logcat（tag `slam`），**不是 Android Studio 的 Run 窗口**——
     * 重定向装在 `JNI_OnLoad` 里，见 jni_bridge.cpp。
     */
    external fun nativeLogSelfTest()

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
}
