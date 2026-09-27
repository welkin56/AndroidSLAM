// JNI 桥：把 Kotlin 的调用翻译成 C++ 的调用。这一层只做「翻译 + 转发」，不放业务。
//
// 符号命名 = Java_<包名点换下划线>_<类名>_<方法名>。与 SlamBridge.kt 的声明对不上
// 时 Kotlin 侧抛 UnsatisfiedLinkError —— 排错第一板斧就是逐段核这个名。
#include <jni.h>

#include <cstdio>
#include <iostream>
#include <string>

#include "log_redirect.h"

// .so 一加载就装日志重定向。ORB-SLAM3 的 cout/cerr 默认进 /dev/null，不接出来它
// 就是个哑巴，而它的失败方式（exit(-1)）在 logcat 里连栈都没有。
extern "C" JNIEXPORT jint JNI_OnLoad(JavaVM* /*vm*/, void* /*reserved*/) {
    slamlog::install();
    return JNI_VERSION_1_6;
}

// —— M1 的探针：桥还通不通。（STAGE5 §7 的排查顺序第一句就问它）——
extern "C" JNIEXPORT jstring JNICALL
Java_com_example_slamcapture_SlamBridge_nativeHello(JNIEnv* env, jobject /*thiz*/) {
    return env->NewStringUTF("hello from libslam_jni.so (M1)");
}

// —— M2 自检：三条路各打一次，外加一条超长行验证分块 ——
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeLogSelfTest(JNIEnv* /*env*/, jobject /*thiz*/) {
    std::cout << "[M2] cout 一行（endl 换行并 flush）" << std::endl;
    std::cerr << "[M2] cerr 一行（无缓冲，立刻出来）" << std::endl;
    printf("[M2] printf 一行\n");

    // 超长行：验证分块生效——尾巴应该能完整看到，而不是断在半截。
    std::string longLine = "[M2] 超长行开始>";
    while (longLine.size() < 9000) longLine += "0123456789";
    longLine += "<超长行结束";
    std::cout << longLine << std::endl;

    // 收尾语先打；下面那行故意不带换行，要晚约 0.2 s 才出现，晚一点是对的。
    std::cerr << "[M2] 自检结束（下面那行不带换行，晚约 0.2 s 出现才是对的）" << std::endl;
    std::cout << "[M2] 这一行故意不带换行" << std::flush;
}
