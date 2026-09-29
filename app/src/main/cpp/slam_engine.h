// 构造并持有 `ORB_SLAM3::System`，喂它帧和 IMU，把位姿交出去。
// Kotlin 侧只问状态和位姿，不碰任何句柄 —— §6 那条约定：「图」的概念完全不跨 JNI。
#pragma once

#include <string>

namespace slam {

// 用 int 报给 Kotlin。四态是必需的：「失败」得能显示出来，
// 否则 UI 就是「点了一下，一直转圈」。
enum class State {
    kIdle = 0,     // 还没调过 init
    kLoading = 1,  // 词袋在加载（实测本机约 38 s）
    kReady = 2,    // System 构造好了
    kFailed = 3,   // 构造抛异常了（⚠️ 走 exit(-1) 的那种根本走不到这里）
};

// 只生效一次，重复调用直接忽略。**同步只做「存路径 + 置 kLoading」，立刻返回** ——
// 真正的构造在别的线程上跑，否则「启动」按钮会被卡死三十几秒。
void init(const std::string& vocabPath, const std::string& settingsPath,
          const std::string& loadFromFile);

State state();

// 「已经加载了多久」，毫秒；不在加载中时返回 0。
//
// 为什么要由 C++ 报而不是 Kotlin 自己掐表：**构造是进程级的事实，掐表是 Activity 级的状态**。
// Activity 重建时 Kotlin 那边的计时会归零，而 System 还在造 —— 界面就显示成「刚点下」然后一直转圈。
long long elapsedMs();

// ---------------- 喂数据（相机线程 / 传感器线程调，**必须立刻返回**）----------------
//
// 队列满时**丢最旧的**，绝不阻塞、绝不积压 —— 那是 §0 的验收判据
// 「SLAM 跟不上时丢帧不积压」。

void pushImage(long long tsNs, const unsigned char* gray, int w, int h);

// ⚠️ **一次一个完整样本**（一个时间戳 + 角速度 + 加速度）。
//
// 这偏离了 STAGE5 §6 写的 `nativePushImu(ts, v0, v1, v2, isAccel)`（分两次调、C++ 侧配对）。
// 改的理由：`IMU::Point` 要的就是完整样本，配对跑不掉，而 **`ImuCollector` 已经在做这个配对了**
// （陀螺事件 + 时间最近的加速度），还带配对偏差的统计。在 C++ 里再配一遍 = 抄一份更差的。
// 顺便这也关掉了一个静默的坑：`IMU::Point(acc…, gyro…, t)` 的参数顺序和我们的 `ImuItem` **相反**，
// 分开传的话很容易写反，而**写反不报错、只是跟踪乱掉**。
void pushImu(long long tsNs, float wx, float wy, float wz,
             float ax, float ay, float az);

// ---------------- 开关（主线程调）----------------

/** 开始吃帧。**不销毁 `System`** —— 销毁就要重付那 38 秒（§5 决策一）。 */
void start();

/** 停止吃帧，并**把两个队列清空**（否则下次启动会吃到停止前积压的旧帧）。`System` 保留。 */
void stop();

// ---------------- 查询（主线程调）----------------

/** 4x4 **列主序**；返回 false = 还没有位姿。用出参是因为 JNI 里构造 Java 数组代价高（§6 约定 3）。 */
bool pose(float out16[16]);

/** ORB-SLAM3 的 `eTrackingState`：-1 还没准备好 / 0 一帧都没进来 / 1 惯性初始化中 / 2 OK / 3 刚丢 / 4 完全丢。 */
int trackingState();

/** 本次运行里「地图被重置」的次数。`start()` 归零。给 UI 显示用。 */
int resetCount();

/** 当前轨迹有多少条。只含跟踪 `OK` 的帧。 */
int trajSize();

// ---------------- 轨迹 ----------------

/**
 * 把这一趟的位姿历史写成 CSV（每行 `秒, tx,ty,tz, qx,qy,qz,qw`，TUM 格式，**3D**）。
 *
 * 位姿历史留在 C++ 而不是 Kotlin：每帧调一次 JNI 再写文件，那是每帧一次 JNI + 一次 I/O ——
 * 在相机回调里干重活是铁律 1 禁止的。**留在 C++，停止/退出时一次性导出。**
 */
void dumpTrajectory(const std::string& path);

}  // namespace slam
