// JNI 桥：把 Kotlin 调用转发给 C++。这一层只做翻译，不放业务。
//
// 符号名必须是 Java_<包名>_<类名>_<方法名>，与 SlamBridge.kt 对不上会抛 UnsatisfiedLinkError。
#include <jni.h>

#include <android/log.h>

#include <algorithm>
#include <string>
#include <cstring>
#include <vector>

#include "log_redirect.h"
#include "slam_engine.h"

namespace {

// jstring 转 std::string。路径都是 ASCII。
// 放在匿名 namespace 中，避免辅助符号导出到 .so。
std::string toStdString(JNIEnv* env, jstring s) {
    if (s == nullptr) return {};
    const char* p = env->GetStringUTFChars(s, nullptr);
    if (p == nullptr) return {};   // OOM
    std::string out(p);
    env->ReleaseStringUTFChars(s, p);
    return out;
}

}  // namespace


// .so 加载时安装日志重定向。ORB-SLAM3 的 cout/cerr 默认被丢弃。
extern "C" JNIEXPORT jint JNI_OnLoad(JavaVM* /*vm*/, void* /*reserved*/) {
    slamlog::install();
    return JNI_VERSION_1_6;
}

// M4：构造 ORB_SLAM3::System。
// 所有函数体都包 try/catch：C++ 异常穿过 JNI 边界会直接终止进程。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeInit(JNIEnv* env, jobject /*thiz*/,
                                                   jstring vocabPath, jstring settingsPath,
                                                   jstring loadFromFile) {
    try {
        slam::init(toStdString(env, vocabPath), toStdString(env, settingsPath),
                   toStdString(env, loadFromFile));
    } catch (...) {
        // 吞掉异常不好，但异常穿过 JNI 边界更糟；状态查询会显示为未开始。
        __android_log_write(ANDROID_LOG_ERROR, "slam", "[M4] nativeInit 抛异常");
    }
}

// 0 没开始 / 1 加载中 / 2 就绪 / 3 失败。失败必须能显示出来。
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

// 已加载的毫秒数，不在加载中时返回 0。
// 由 C++ 计时，避免 Activity 重建时归零。
extern "C" JNIEXPORT jlong JNICALL
Java_com_example_slamcapture_SlamBridge_nativeElapsedMs(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jlong>(slam::elapsedMs());
    } catch (...) {
        return 0;
    }
}

// M5：喂帧、喂 IMU、位姿。
//
// PushImage 和 PushImu 跑在相机线程和传感器线程上，不能阻塞，
// 只把数据拷进队列就返回。JNI_ABORT 表示只读，不回写。

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

// 一个完整样本：时间戳、角速度、加速度。配对由 Kotlin 侧的 ImuCollector 完成。
// 参数顺序详见 slam_engine.h。
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

// 只允许在 nativeStart() 之前调用；跑着帧时会被忽略。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeSetIntrinsics(
        JNIEnv* /*env*/, jobject /*thiz*/,
        jfloat fx, jfloat fy, jfloat cx, jfloat cy,
        jfloat k1, jfloat k2, jfloat p1, jfloat p2) {
    try {
        slam::setIntrinsics(fx, fy, cx, cy, k1, k2, p1, p2);
    } catch (...) {
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

// 4x4 列主序，用出参传回。返回 false 表示还没有位姿。
extern "C" JNIEXPORT jboolean JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetPose(JNIEnv* env, jobject /*thiz*/,
                                                      jfloatArray out) {
    try {
        // Kotlin 传的数组必须够长，否则写 16 个 float 会破坏堆。
        if (env->GetArrayLength(out) < 16) return JNI_FALSE;
        jfloat* p = env->GetFloatArrayElements(out, nullptr);
        if (p == nullptr) return JNI_FALSE;
        const bool ok = slam::pose(p);
        // 成功时回写，失败时用 JNI_ABORT 跳过拷贝。
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

// 2D 地图轨迹：按 stride 抽稀的 xyz 三元组。
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

// 2D 地图点云：out 收 3n 个 float（世界坐标 xyz），返回点数。
// 与 nativeGetTrajectory 一样按 stride 抽稀。
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

// 小地图开关。关闭时 SLAM 线程不重建点云快照。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeSetMapCloudEnabled(JNIEnv* /*env*/, jobject /*thiz*/,
                                                                 jboolean on) {
    try {
        slam::setMapCloudEnabled(on == JNI_TRUE);
    } catch (...) {
    }
}

// 存图：只发请求，序列化在 SLAM 线程完成。path 不含 .osa 后缀。
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

// 把 atlas 恢复到上一次保存的状态：只发请求，重读 .osa 在 SLAM 线程完成。
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

// 换源：把 atlas 换成 path 指定的图（不含 .osa，空串表示空图），只发请求。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeRequestLoadAtlas(JNIEnv* env, jobject /*thiz*/,
                                                               jstring path) {
    try {
        slam::requestLoadAtlas(toStdString(env, path));
    } catch (...) {
    }
}

// atlas 当前对应的路径（不含 .osa），空串表示新建或空图。
extern "C" JNIEXPORT jstring JNICALL
Java_com_example_slamcapture_SlamBridge_nativeLoadedMap(JNIEnv* env, jobject /*thiz*/) {
    try {
        const std::string p = slam::loadedMapPath();
        return env->NewStringUTF(p.c_str());
    } catch (...) {
        return env->NewStringUTF("");
    }
}

// atlas 规模。out 至少 3 个，依次为图数、关键帧数、地图点数。
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

// 构造失败细分：0 没失败 / 1 载图失败 / 2 其他。
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

// 实时测距开关。关闭时只停止计算，不影响 SLAM。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeSetMeasuring(JNIEnv* /*env*/, jobject /*thiz*/,
                                                           jboolean on) {
    try {
        slam::setMeasuring(on == JNI_TRUE);
    } catch (...) {
    }
}

// M6：画面中心的距离读数。
//
// out 至少 7 个：[0] 距离（米）、[1..3] 世界 xyz、[4][5] 图像像素、[6] 附近特征点数。
// 返回 0/1/2（不可测/勉强/好）。数组长度不足时直接挡掉。
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

// 多点折线。刷新、投影、算长度都在 C++ 一次完成，因为折线保存的是 MapPoint*。

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

// 路径点。名字通过一个 String[] 一次传回，避免 N 次 JNI 往返。
extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeRequestAddWaypoint(JNIEnv* env, jobject /*thiz*/,
                                                                 jstring name) {
    try {
        slam::requestAddWaypoint(toStdString(env, name));
    } catch (...) {
    }
}

// 状态：0 没试过 / 1 排队 / 2 成功 / 3 失败。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeWaypointState(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jint>(slam::waypointState());
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_slamcapture_SlamBridge_nativeWaypointMsg(JNIEnv* env, jobject /*thiz*/) {
    try {
        return env->NewStringUTF(slam::waypointMsg().c_str());
    } catch (...) {
        return env->NewStringUTF("");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeRenameWaypoint(JNIEnv* env, jobject /*thiz*/,
                                                             jint id, jstring name) {
    try {
        slam::renameWaypoint(static_cast<int>(id), toStdString(env, name));
    } catch (...) {
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_slamcapture_SlamBridge_nativeDeleteWaypoint(JNIEnv* /*env*/, jobject /*thiz*/,
                                                             jint id) {
    try {
        slam::deleteWaypoint(static_cast<int>(id));
    } catch (...) {
    }
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_example_slamcapture_SlamBridge_nativeWaypointRevision(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jlong>(slam::waypointRevision());
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeWaypointCount(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jint>(slam::waypointCount());
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeNextWaypointId(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jint>(slam::nextWaypointId());
    } catch (...) {
        return 1;
    }
}

// pos 每点 3 个 float，meta 每点 4 个 long long，names 收名字。
// 返回实际写入条数；装不下时少给。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeGetWaypoints(JNIEnv* env, jobject /*thiz*/,
                                                           jfloatArray pos, jlongArray meta,
                                                           jobjectArray names) {
    try {
        jfloat* p = env->GetFloatArrayElements(pos, nullptr);
        if (p == nullptr) return 0;
        jlong* m = env->GetLongArrayElements(meta, nullptr);
        if (m == nullptr) {
            env->ReleaseFloatArrayElements(pos, p, JNI_ABORT);
            return 0;
        }

        const jsize nameCap = names == nullptr ? 0 : env->GetArrayLength(names);
        // jlong 在 Android 上是 long，引擎使用 long long，需通过临时缓冲转换。
        const jsize metaLen = env->GetArrayLength(meta);
        std::vector<long long> metaBuf(static_cast<size_t>(metaLen));
        std::vector<std::string> nameBuf(static_cast<size_t>(nameCap > 0 ? nameCap : 0));
        std::memcpy(metaBuf.data(), m, sizeof(long long) * static_cast<size_t>(metaLen));

        const int n = slam::getWaypoints(p, env->GetArrayLength(pos),
                                         metaBuf.data(), static_cast<int>(metaLen),
                                         nameBuf.empty() ? nullptr : nameBuf.data(),
                                         static_cast<int>(nameBuf.size()));

        // 引擎写的是 metaBuf，必须拷回 m 后再 Release，否则回写的是原始缓冲。
        std::memcpy(m, metaBuf.data(), sizeof(long long) * static_cast<size_t>(metaLen));

        env->ReleaseFloatArrayElements(pos, p, 0);
        env->ReleaseLongArrayElements(meta, m, 0);

        // 逐条创建字符串放回；装不下的不写，Kotlin 侧看到空串。
        for (int i = 0; i < n && i < nameCap; ++i) {
            jstring s = env->NewStringUTF(nameBuf[static_cast<size_t>(i)].c_str());
            if (s == nullptr) break;
            env->SetObjectArrayElement(names, i, s);
            env->DeleteLocalRef(s);
        }
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

// 轨迹归属、合并链、轨迹变换。

// 值在 nativeStop() 中快照，停止后应立即读取。
extern "C" JNIEXPORT jlong JNICALL
Java_com_example_slamcapture_SlamBridge_nativeLastTrajMapId(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jlong>(slam::lastTrajMapId());
    } catch (...) {
        return -1;
    }
}

// 每条 10 个 float：survivor absorbed s qx qy qz qw tx ty tz。返回实际条数。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeLastMerges(JNIEnv* env, jobject /*thiz*/,
                                                         jfloatArray out) {
    try {
        if (out == nullptr) return 0;
        jfloat* o = env->GetFloatArrayElements(out, nullptr);
        if (o == nullptr) return 0;
        const int n = slam::lastMerges(o, env->GetArrayLength(out));
        env->ReleaseFloatArrayElements(out, o, 0);
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_example_slamcapture_SlamBridge_nativeTransformTrajFile(
    JNIEnv* env, jobject /*thiz*/, jstring src, jstring dst,
    jfloat s, jfloat qx, jfloat qy, jfloat qz, jfloat qw,
    jfloat tx, jfloat ty, jfloat tz) {
    try {
        return slam::transformTrajFile(toStdString(env, src), toStdString(env, dst),
                                       s, qx, qy, qz, qw, tx, ty, tz)
                   ? JNI_TRUE
                   : JNI_FALSE;
    } catch (...) {
        return JNI_FALSE;
    }
}

// 每条 9 个 float：id s qx qy qz qw tx ty tz；第一条固定为 displayMapId。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeDisplayTransforms(JNIEnv* env, jobject /*thiz*/,
                                                                jfloatArray out) {
    try {
        if (out == nullptr) return 0;
        jfloat* o = env->GetFloatArrayElements(out, nullptr);
        if (o == nullptr) return 0;
        const int n = slam::displayTransforms(o, env->GetArrayLength(out));
        env->ReleaseFloatArrayElements(out, o, 0);
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

// out 收 3n 个 float，返回点数。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeReadTrajFile(
    JNIEnv* env, jobject /*thiz*/, jstring path,
    jfloat s, jfloat qx, jfloat qy, jfloat qz, jfloat qw,
    jfloat tx, jfloat ty, jfloat tz, jfloatArray out) {
    try {
        if (out == nullptr) return 0;
        jfloat* o = env->GetFloatArrayElements(out, nullptr);
        if (o == nullptr) return 0;
        const int n = slam::readTrajFile(toStdString(env, path), s, qx, qy, qz, qw, tx, ty, tz,
                                         o, env->GetArrayLength(out));
        env->ReleaseFloatArrayElements(out, o, 0);
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

// 三组出参一一对应；只返回 KF 大于 0 的图。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeMapList(
    JNIEnv* env, jobject /*thiz*/, jlongArray ids, jintArray kfs, jintArray mps) {
    try {
        if (ids == nullptr || kfs == nullptr || mps == nullptr) return 0;
        const jsize len = std::min(env->GetArrayLength(ids),
                                   std::min(env->GetArrayLength(kfs), env->GetArrayLength(mps)));
        if (len <= 0) return 0;

        // jlong 在 Android 上是 long，引擎使用 long long，需转换后回写。
        std::vector<long long> idBuf(static_cast<size_t>(len));
        std::vector<int> kfBuf(static_cast<size_t>(len));
        std::vector<int> mpBuf(static_cast<size_t>(len));
        const int n = slam::mapList(idBuf.data(), kfBuf.data(), mpBuf.data(),
                                    static_cast<int>(len));
        if (n <= 0) return 0;

        std::vector<jlong> outIds(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) outIds[i] = static_cast<jlong>(idBuf[static_cast<size_t>(i)]);
        env->SetLongArrayRegion(ids, 0, n, outIds.data());
        env->SetIntArrayRegion(kfs, 0, n, kfBuf.data());
        env->SetIntArrayRegion(mps, 0, n, mpBuf.data());
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

// out 收 3n 个 float，返回点数。
extern "C" JNIEXPORT jint JNICALL
Java_com_example_slamcapture_SlamBridge_nativeMapCloud(
    JNIEnv* env, jobject /*thiz*/, jlong mapId, jfloatArray out) {
    try {
        if (out == nullptr) return 0;
        jfloat* o = env->GetFloatArrayElements(out, nullptr);
        if (o == nullptr) return 0;
        const int n = slam::mapCloud(static_cast<long long>(mapId), o,
                                     env->GetArrayLength(out));
        env->ReleaseFloatArrayElements(out, o, 0);
        return static_cast<jint>(n);
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_example_slamcapture_SlamBridge_nativeTrajectoryMapId(JNIEnv* /*env*/, jobject /*thiz*/) {
    try {
        return static_cast<jlong>(slam::trajectoryMapId());
    } catch (...) {
        return -1;
    }
}
