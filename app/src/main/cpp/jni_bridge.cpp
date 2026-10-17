// JNI 桥：把 Kotlin 的调用翻译成 C++ 的调用。这一层只做「翻译 + 转发」，不放业务。
//
// 符号命名 = Java_<包名点换下划线>_<类名>_<方法名>。与 SlamBridge.kt 的声明对不上
// 时 Kotlin 侧抛 UnsatisfiedLinkError —— 排错第一板斧就是逐段核这个名。
#include <jni.h>

#include <android/log.h>

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

// 已经加载了多久（毫秒）。不在加载中时返回 0。
// 由 C++ 报而不是 Kotlin 掐表：构造是进程级的事实，Activity 重建时 Kotlin 那边的计时会归零。
extern "C" JNIEXPORT jlong JNICALL
Java_com_example_slamcapture_SlamBridge_nativeElapsedMs(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jlong>(slam::elapsedMs());
    } catch (...) {
        return 0;
    }
}

// —— M5：喂帧、喂 IMU、位姿 ——
//
// ⚠️ `nativePushImage` 和 `nativePushImu` **跑在相机线程 / 传感器线程上**，
// 里面绝不能有阻塞操作（铁律 1）。两个都只是把数据拷进队列就返回。
//
// **先不做零拷贝**（STAGE5 §5）：`ByteArray` 现在有三个持有者（落盘 / 预览 / SLAM），
// 复用 buffer 会让它们读到被覆盖的帧。JNI_ABORT = 只读，不回写。

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativePushImage(JNIEnv* env, jobject /*thiz*/,
                                                        jlong tsNs, jbyteArray gray,
                                                        jint w, jint h) {
    try {
        jbyte* p = env->GetByteArrayElements(gray, nullptr);
        if (p == nullptr) return;   // OOM
        slam::pushImage(static_cast<long long>(tsNs),
                        reinterpret_cast<const unsigned char*>(p), w, h);
        env->ReleaseByteArrayElements(gray, p, JNI_ABORT);
    } catch (...) {
        __android_log_write(ANDROID_LOG_ERROR, "slam", "[M5] nativePushImage 抛异常");
    }
}

// ⚠️ **一个完整样本**（一个时间戳 + 角速度 + 加速度），不是分两次调。
// 配对由 Kotlin 侧的 `ImuCollector` 做（它本来就在做，还带配对偏差的统计）。
// 见 slam_engine.h 里那段说明 —— 顺带关掉了「加速度和角速度写反」那个静默的坑。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativePushImu(JNIEnv* /*env*/, jobject /*thiz*/,
                                                      jlong tsNs, jfloat wx, jfloat wy, jfloat wz,
                                                      jfloat ax, jfloat ay, jfloat az) {
    try {
        slam::pushImu(static_cast<long long>(tsNs), wx, wy, wz, ax, ay, az);
    } catch (...) {
        __android_log_write(ANDROID_LOG_ERROR, "slam", "[M5] nativePushImu 抛异常");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeStart(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        slam::start();
    } catch (...) {
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeStop(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        slam::stop();
    } catch (...) {
    }
}

// 4x4 列主序，用出参（§6 约定 3）。返回 false = 还没有位姿。
extern "C" JNIEXPORT jboolean JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetPose(JNIEnv* env, jobject /*thiz*/,
                                                      jfloatArray out) {
    try {
        // ⚠️ 必须查长度：Kotlin 那边传短了的话，写 16 个 float 会踩坏堆。
        if (env->GetArrayLength(out) < 16) return JNI_FALSE;
        jfloat* p = env->GetFloatArrayElements(out, nullptr);
        if (p == nullptr) return JNI_FALSE;
        const bool ok = slam::pose(p);
        // ok 时回写（mode 0），不 ok 时 JNI_ABORT 不白拷一趟。
        env->ReleaseFloatArrayElements(out, p, ok ? 0 : JNI_ABORT);
        return ok ? JNI_TRUE : JNI_FALSE;
    } catch (...) {
        return JNI_FALSE;
    }
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetTrackingState(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jint>(slam::trackingState());
    } catch (...) {
        return -1;
    }
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeResetCount(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jint>(slam::resetCount());
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeTrajSize(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jint>(slam::trajSize());
    } catch (...) {
        return 0;
    }
}

// 2D 地图要的轨迹：stride 抽稀的 xyz 三元组。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetTrajectory(JNIEnv* env, jobject /*thiz*/,
                                                            jfloatArray out) {
    try {
        jfloat* p = env->GetFloatArrayElements(out, nullptr);
        if (p == nullptr) return 0;   // OOM
        const int n = slam::getTrajectory(p, env->GetArrayLength(out));
        env->ReleaseFloatArrayElements(out, p, 0);   // 写回
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

// 2D 地图的点云：`out` 收 `3n` 个 float（世界米 xyz），返回点数。
// 和 `nativeGetTrajectory` 一样是**抽稀**契约 —— 固定缓冲，别扩容取全。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetMapPoints(JNIEnv* env, jobject /*thiz*/,
                                                           jfloatArray out) {
    try {
        jfloat* p = env->GetFloatArrayElements(out, nullptr);
        if (p == nullptr) return 0;   // OOM
        const int n = slam::getMapPoints(p, env->GetArrayLength(out));
        env->ReleaseFloatArrayElements(out, p, 0);   // 写回
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

// 抽屉里那个「小地图」开关。关掉时 SLAM 线程连点云快照都不重建 ——
// 那次枚举抱着 `Map::mMutexMap`，藏起来时白付不起。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeSetMapCloudEnabled(JNIEnv* /*env*/, jobject /*thiz*/,
                                                                 jboolean on) {
    try {
        slam::setMapCloudEnabled(on == JNI_TRUE);
    } catch (...) {
    }
}

// 存图：**只发请求**。真正的序列化在 SLAM 线程上做（几秒，会把主线程卡死）。
// `path` 不含 `.osa` 后缀，上游自己加。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeRequestSaveMap(JNIEnv* env, jobject /*thiz*/,
                                                             jstring path) {
    try {
        slam::requestSaveMap(toStdString(env, path));
    } catch (...) {
    }
}

// 保存状态：0 空闲 / 1 排队 / 2 保存中 / 3 完成 / 4 失败。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeSaveState(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return slam::saveState();
    } catch (...) {
        return 0;
    }
}

// 把 atlas 恢复成「上一次保存」的样子：**只发请求**。
// 要重读 .osa（秒级）而且会阻塞等 LocalMapping / LoopClosing，只能在 SLAM 线程上做。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeRequestRestoreMap(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        slam::requestRestoreMap();
    } catch (...) {
    }
}

// 恢复状态：0 空闲 / 1 排队 / 2 恢复中 / 3 完成 / 4 失败。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeRestoreState(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return slam::restoreState();
    } catch (...) {
        return 0;
    }
}

// atlas 有多大。`out` 要 ≥ 3，收 `[图数, 关键帧数, 地图点数]`。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeAtlasStats(JNIEnv* env, jobject /*thiz*/,
                                                         jintArray out) {
    try {
        if (env->GetArrayLength(out) < 3) return;
        jint* p = env->GetIntArrayElements(out, nullptr);
        if (p == nullptr) return;
        int v[3] = {0, 0, 0};
        slam::atlasStats(v);
        p[0] = v[0];
        p[1] = v[1];
        p[2] = v[2];
        env->ReleaseIntArrayElements(out, p, 0);
    } catch (...) {
    }
}

// 构造失败的细分：0 没失败 / 1 载图失败 / 2 别的。给界面说清是哪一种。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeFailureCode(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return slam::failureCode();
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeDumpTrajectory(JNIEnv* env, jobject /*thiz*/,
                                                             jstring path) {
    try {
        slam::dumpTrajectory(toStdString(env, path));
    } catch (...) {
        __android_log_write(ANDROID_LOG_ERROR, "slam", "[M5] nativeDumpTrajectory 抛异常");
    }
}

// 抽屉里那个「实时测距」开关。关掉时连算都不算，但**不影响 SLAM 本身**。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeSetMeasuring(JNIEnv* /*env*/, jobject /*thiz*/,
                                                           jboolean on) {
    try {
        slam::setMeasuring(on == JNI_TRUE);
    } catch (...) {
    }
}

// —— M6：画面中心的距离读数 ——
//
// 出参写法，和 `nativeGetPose` 同款（理由见 slam_engine.h 里那句「§6 约定 3」）。
// `out` 要 ≥ 7：[0]=距离m [1..3]=世界 xyz [4]/[5]=图像像素 xy [6]=附近特征点数。
// 返回 0/1/2（不可测/勉强/好）。⚠️ **长度不足必须挡掉** —— 写 7 个 float 会踩坏堆。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetMeasure(JNIEnv* env, jobject /*thiz*/,
                                                         jfloatArray out) {
    try {
        if (env->GetArrayLength(out) < 7) return 0;
        jfloat* p = env->GetFloatArrayElements(out, nullptr);
        if (p == nullptr) return 0;   // OOM
        const int q = slam::getMeasure(p);
        env->ReleaseFloatArrayElements(out, p, 0);
        return q;
    } catch (...) {
        return 0;
    }
}

// 「多点折线」。刷新、投影、算长度**全在 C++ 一次给完** —— 因为折线存的是 `MapPoint*`，
// 那个东西只在 C++ 侧拿得到。见 slam_engine.h 那一整段。

extern "C" JNIEXPORT jboolean JNICALL
Java_com_example_slamcapture_SlamBridge_nativeAddPinned(JNIEnv* /*env*/, jobject /*thiz*/,
                                                        jint snapTo) {
    try {
        return slam::addPinned(static_cast<int>(snapTo)) ? JNI_TRUE : JNI_FALSE;
    } catch (...) {
        return JNI_FALSE;
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeUndoPinned(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        slam::undoPinned();
    } catch (...) {
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeClearPinned(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        slam::clearPinned();
    } catch (...) {
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeClosePinnedChain(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        slam::closePinnedChain();
    } catch (...) {
    }
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetPinned(JNIEnv* env, jobject /*thiz*/,
                                                        jfloatArray out) {
    try {
        jfloat* p = env->GetFloatArrayElements(out, nullptr);
        if (p == nullptr) return 0;   // OOM
        const int n = slam::getPinned(p, env->GetArrayLength(out));
        env->ReleaseFloatArrayElements(out, p, 0);   // 写回
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}
