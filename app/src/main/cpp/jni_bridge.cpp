// M1 空壳桥：只验证「Kotlin ↔ C++ 这条路是通的」，不碰 ORB-SLAM3。
//
// 符号命名 = Java_<包名点换下划线>_<类名>_<方法名>。与 SlamBridge.kt 的声明
// 对不上时 Kotlin 侧抛 UnsatisfiedLinkError —— 排错第一板斧就是逐段核这个名。
#include <jni.h>

extern "C" JNIEXPORT jint JNI_OnLoad(JavaVM* /*vm*/, void* /*reserved*/) {
    // M2 会在这里加 stdout/stderr → logcat 的重定向。
    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_slamcapture_SlamBridge_nativeHello(JNIEnv* env, jobject /*thiz*/) {
    return env->NewStringUTF("hello from libslam_jni.so (M1)");
}
