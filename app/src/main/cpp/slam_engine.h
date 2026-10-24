// 构造并持有 `ORB_SLAM3::System`，喂它帧和 IMU，把位姿交出去。
// Kotlin 侧只问状态和位姿，不碰任何句柄 —— §6 那条约定：「图」的概念完全不跨 JNI。
#pragma once

#include <string>

#include "measure_center.h"  // slam::CenterMeasure

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

/**
 * 导出轨迹给 2D 地图画：`out` 收 `3n` 个 float（世界米，`x y z`），返回点数。
 *
 * ⚠️ **stride 抽稀，不是截断** —— 要的是形状不是精度：覆盖全程、恒含最新点。
 * **别学 `readPinned` 扩容取全**（那份契约要的是精度，正好相反）。
 */
int getTrajectory(float* out, int cap);

// ---------------- 测距（画面中心）----------------
//
// 十字固定在画面中心，所以**没有「测哪儿」这个参数** —— 每帧顺手算一次就够，
// 不需要 UI 往返。平滑也在这一侧做（只有 SLAM 线程碰那个环形缓冲）。

/** 开不开测距（**主线程调**，抽屉里那个开关）。关掉时连算都不算，但**不影响 SLAM**。 */
void setMeasuring(bool on);

/** 发布一帧的测量结果（**SLAM 线程调**）。内部做中值平滑，并按变化打日志。 */
void publishMeasure(const CenterMeasure& m);

/**
 * 读最近一次测量（**主线程调**）。
 *
 * `out` 要 ≥ 7：`[0]` 距离（米）、`[1..3]` 目标世界坐标、`[4]`/`[5]` 目标图像像素坐标、
 * `[6]` 十字附近的**特征点**数（不管有没有地图点）。返回 `MeasureQuality`。
 */
int getMeasure(float out7[7]);

// ---------------- 多点折线 ----------------
// 刷新 + 投影 + 算长度**一次给完**，Kotlin 只负责画。
// 全部**主线程调**。

/**
 * 把当前读数钉成一个点。没有有效读数返回 false。
 *
 * ⚠️ `snapTo >= 0` 时**复制第 `snapTo` 个点的身份**（`mp` / `pw` / `low` 全抄），
 * 而不是用十字当前命中的那个 —— 那个只是**碰巧很近的另一个地图点**，之后会各漂各的。
 */
bool addPinned(int snapTo);

void undoPinned();
void clearPinned();

/** 「断开」：最后一个点之后不再拉橡皮筋，下一个点会另起一条折线。 */
void closePinnedChain();

/**
 * 读折线：**刷新坐标 → 投影 → 算长度**，一次给完。
 *
 * `out` 要 ≥ `4n + 1`：每点 4 个 `px py flags lenToNext`，最后一个是 `liveLen`
 * （最后一个点 → 十字；没读数或已断开时是 -1）。
 *
 * `flags` 的位：bit0 在相机前方 / bit1 新折线起点 / bit2 之后断开 / bit3 不太准（空心圈）。
 *
 * **返回「实际写了几个点」**（`≤ cap/4`）。容量不够时这一帧少画一个，
 * 调用方看到写满就扩容 —— **不能返回真实点数**，那调用方会去读没写过的内存。
 */
int getPinned(float* out, int cap);

// ---------------- 路径点 ----------------

/** 在当前位置记一个路径点（**主线程调**）。发请求就返回，下一帧在 SLAM 线程执行。 */
void requestAddWaypoint(const std::string& name);

/** 加路径点的结果（**主线程调**）：0 没试过 / 1 排队 / 2 成功 / 3 失败。 */
int waypointState();

/** 失败原因（**主线程调**）。成功时是空串。 */
std::string waypointMsg();

/** 改名 / 删除（**主线程调**）。不碰地图，就近生效。 */
void renameWaypoint(int id, const std::string& name);
void deleteWaypoint(int id);

/** 变一次加一。界面只在它变了之后重建叠加层。 */
long long waypointRevision();

/**
 * 读路径点世界坐标。`pos` 每点 3 个 float，`meta` 每点 `id, mapId, kfId, flags`
 * （bit0 坐标有效 / bit1 在当前图）。
 *
 * **装不下就少给，返回实际写了几条** —— 和 [getPinned] 同一个契约。
 */
int getWaypoints(float* pos, int posCap, long long* meta, int metaCap,
                 std::string* names, int nameCap);

/** 路径点总条数（**主线程调**）。 */
int waypointCount();

/** 下一个路径点 id —— 默认名字用它的编号，单调递增（**主线程调**）。 */
int nextWaypointId();

// ---------------- 存图 ----------------

/**
 * 把当前地图另存到 `path + ".osa"`（**主线程调**）。**只在 SLAM 停止之后调**。
 *
 * 只是发个请求就返回：真正的序列化在 SLAM 线程上做（主线程跑会和跟踪抢地图），
 * 而且慢，可能要几秒。进度看 [saveState]。
 */
void requestSaveMap(const std::string& path);

/** 保存状态（**主线程调**）：0 空闲 / 1 排队 / 2 保存中 / 3 完成 / 4 失败。 */
int saveState();

/**
 * 把 atlas 恢复成「上一次保存」的样子（**主线程调**）。
 *
 * 目标：最近一次成功存图的文件；没存过就用构造时载入的那张；两个都没有（新建地图）
 * ⟹ 清成一张空图。真正的活在 SLAM 线程空闲时做（要重读 .osa，秒级）。
 *
 * 一趟结束但**没保存**时调 —— 否则这趟建出来的图会留在 atlas 里，
 * 下一趟的回环检测还能匹配到它，把已经不要的数据合并回来。
 */
void requestRestoreMap();

/** 恢复状态（**主线程调**）：0 空闲 / 1 排队 / 2 恢复中 / 3 完成 / 4 失败。 */
int restoreState();

/**
 * 把 atlas **换成** `path` 指向的那张（**主线程调**）。空串 = 清成一张空图。
 *
 * 入口页每次重新选（新建地图 / 换一张图）时调：`System` 只能构造一次，不换它会变成
 * 「又开一张当前图，旧 atlas 原样留着」。成功之后这个路径成为新的「恢复」基准。
 *
 * 和 [requestRestoreMap] 共用一条队列和同一个 [restoreState]。
 */
void requestLoadAtlas(const std::string& path);

/** atlas 现在装的是哪一份（不含 `.osa`）。空串 = 新建 / 空图。**主线程调**。 */
std::string loadedMapPath();

/**
 * atlas 里现在有几张图、一共多少关键帧、多少地图点（**主线程调**）。`out` 要 ≥ 3。
 *
 * 载图之后**这是界面上唯一看得出「图进来了」的地方** —— 载进来的图不是当前图，
 * 2D 小窗、轨迹、测距都看不到它。
 */
void atlasStats(int out3[3]);

/** 构造失败的细分（**主线程调**）：0 没失败 / 1 载图失败 / 2 别的。给界面说清是哪一种。 */
int failureCode();

// ---------------- 2D 地图的点云 ----------------

/**
 * 读一次地图点云快照（**主线程调**）。`out` 收 `3n` 个 float（世界米 `x y z`），返回点数。
 *
 * 数据是 **SLAM 线程每秒重建一次的快照**，不是现读地图 —— 全图枚举要抱 `Map::mMutexMap`，
 * 而那个锁 LocalMapping 每建一个点都要拿。留给这一侧的只有 memcpy ✓
 *
 * **stride 抽稀，不是截断**（全量上限 5000，装得下就不抽）。和 [getTrajectory]
 * 同一个契约：**别学 `readPinned` 扩容取全**（那个要的是精度，正好相反）。
 */
int getMapPoints(float* out, int cap);

/**
 * 开不开点云快照（**主线程调**，抽屉里那个开关）。关掉时 SLAM 线程整个不重建 ——
 * 重建那几毫秒**抱着 `mMutexMap`**，藏起来时白付不起。重开时下一个空隙立刻重建 ✓
 */
void setMapCloudEnabled(bool on);

// ---------------- 轨迹 ----------------

/**
 * 把这一趟的位姿历史写成 CSV（每行 `秒, tx,ty,tz, qx,qy,qz,qw`，TUM 格式，**3D**）。
 *
 * 位姿历史留在 C++ 而不是 Kotlin：每帧调一次 JNI 再写文件，那是每帧一次 JNI + 一次 I/O ——
 * 在相机回调里干重活是铁律 1 禁止的。**留在 C++，停止/退出时一次性导出。**
 */
void dumpTrajectory(const std::string& path);

/**
 * 本趟最后一段轨迹属于哪张图（**主线程调**）。-1 = 没有。
 *
 * 值在 `stop()` 里快照 —— `nativeStop()` 之后 worker 可能立刻消费挂着的「恢复 atlas」
 * 请求，那会把整个 atlas 换掉；事后再问拿到的就是空图。
 */
long long lastTrajMapId();

/** 本趟的合并链（**主线程调**）：每条 10 个 float，按发生顺序。装不下就少给。 */
int lastMerges(float* out, int cap);

/**
 * 当前轨迹**此刻**属于哪张图（**主线程调**）。-1 = 没有。
 *
 * 和 [lastTrajMapId] 的区别：这个是实时值，跑着的时候也能问（拿 `mMutexTraj`）；
 * 那个是 `stop()` 的快照。地图查看器用它判「当前位置画在哪张图上」。
 */
long long trajectoryMapId();

/**
 * atlas 里有关键帧的图（**主线程调**，按 id 升序）。三条出参一一对应，返回写了几张。
 * 0 关键帧的空图不返回。
 */
int mapList(long long* ids, int* kfs, int* mps, int cap);

/**
 * 任意一张图的点云（**主线程调**），该图自己的坐标系。
 * `out` 收 `3n` 个 float，stride 抽稀到 `cap/3` 点。只在选图/刷新时调。
 */
int mapCloud(long long mapId, float* out, int cap);

/**
 * 界面显示用的「文件夹 id → 当前坐标系」变换（**主线程调**）。
 *
 * 每条 9 个 float：`id s qx qy qz qw tx ty tz`，**第一条固定是 displayMapId**。
 * 没有图时返回 0。id 和变换一次给完 —— 分两次问的话中间夹一次合并会对不上。
 */
int displayTransforms(float* out, int cap);

/**
 * 读一个已归档的轨迹文件、变换到显示坐标系（**主线程调**）。
 *
 * 每行 `time tx ty tz qx qy qz qw`（`Twc`）；只取位置：
 * `tw' = s·(q_on·tw) + t_on`。**stride 抽稀**到 `cap/3` 点，写 `3n` 个 xyz。
 */
int readTrajFile(const std::string& path,
                 float s, float qx, float qy, float qz, float qw,
                 float tx, float ty, float tz,
                 float* out, int cap);

/**
 * 把一个轨迹文件按合并变换写到 `dst`（**主线程调**）。成功返回 true。
 *
 * `T_on` 的 `p' = s·R·p + t` 作用在 `Twc` 上 ⟹ `q' = q_on⊗q`、`t' = s·R_on·t + t_on`。
 * 写的是 `dst`（不是原地改）：调用方负责「全部写完再统一 rename」。
 */
bool transformTrajFile(const std::string& src, const std::string& dst,
                       float s, float qx, float qy, float qz, float qw,
                       float tx, float ty, float tz);

}  // namespace slam
