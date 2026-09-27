// JNI 桥：把 Kotlin 的调用翻译成 C++ 的调用。这一层只做「翻译 + 转发」，不放业务。
//
// 符号命名 = Java_<包名点换下划线>_<类名>_<方法名>。与 SlamBridge.kt 的声明对不上
// 时 Kotlin 侧抛 UnsatisfiedLinkError —— 排错第一板斧就是逐段核这个名。
#include <jni.h>

#include <android/log.h>

#include <cstdio>
#include <iostream>
#include <string>

#include "log_redirect.h"
#include "slam_engine.h"

namespace {

// jstring → std::string。路径都是 ASCII，GetStringUTFChars 拿到的够用。
// ⚠️ 放匿名 namespace 里：这一层编出来的辅助符号不能漏进 .so 的导出表（见 M2 的改动记录）。
std::string toStdString(JNIEnv* env, jstring s) {
    if (s == nullptr) return {};
    const char* p = env->GetStringUTFChars(s, nullptr);
    if (p == nullptr) return {};   // OOM
    std::string out(p);
    env->ReleaseStringUTFChars(s, p);
    return out;
}

}  // namespace


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

// —— M4：构造 ORB_SLAM3::System ——
//
// 三个函数体都包 try/catch：C++ 异常穿过 JNI 边界 = 进程直接死，Java 侧连栈都没有。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeInit(JNIEnv* env, jobject /*thiz*/,
                                                   jstring vocabPath, jstring settingsPath,
                                                   jstring loadFromFile) {
    try {
        slam::init(toStdString(env, vocabPath), toStdString(env, settingsPath),
                   toStdString(env, loadFromFile));
    } catch (...) {
        // 这里吞掉不好，但「异常过界」更坏。状态查询那边会显示成「没开始」。
        __android_log_write(ANDROID_LOG_ERROR, "slam", "[M4] nativeInit 抛异常");
    }
}

// 0=没开始 1=加载中 2=就绪 3=失败。用四态而不是布尔：「失败」必须能显示出来，
// 否则界面就是「点了一下，一直转圈」。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeState(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jint>(slam::state());
    } catch (...) {
        return static_cast<jint>(slam::State::kFailed);
    }
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_example_slamcapture_SlamBridge_nativeIsReady(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return slam::state() == slam::State::kReady ? JNI_TRUE : JNI_FALSE;
    } catch (...) {
        return JNI_FALSE;
    }
}
