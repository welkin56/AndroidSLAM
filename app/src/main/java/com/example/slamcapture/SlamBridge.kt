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
}
