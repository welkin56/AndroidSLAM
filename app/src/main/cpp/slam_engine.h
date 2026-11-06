// 持有 ORB_SLAM3::System：喂帧和 IMU，输出位姿。
// Kotlin 侧只查状态和位姿，不接触任何句柄。
#pragma once

#include <string>

#include "measure_center.h"  // slam::CenterMeasure

namespace slam {

// 用 int 报给 Kotlin。四态是必需的，失败也要能显示。
enum class State {
    kIdle = 0,     // 还没调过 init
    kLoading = 1,  // 词袋加载中（本机约 38 s）
    kReady = 2,    // System 构造完成
    kFailed = 3,   // 构造抛异常（走 exit(-1) 的路径到不了这里）
};

// 只生效一次。同步只保存路径并置 kLoading，构造在后台线程进行。
void init(const std::string& vocabPath, const std::string& settingsPath,
          const std::string& loadFromFile);

State state();

// 已加载的毫秒数，不在加载中时返回 0。由 C++ 计时，Activity 重建不会归零。
long long elapsedMs();

// ---------------- 喂数据（相机线程 / 传感器线程调，必须立刻返回）----------------
//
// 队列满时丢最旧的，不阻塞。

void pushImage(long long tsNs, const unsigned char* gray, int w, int h);

// 一次传入一个完整样本：时间戳、角速度、加速度。
// 与 IMU::Point 的参数顺序一致；配对由 Kotlin 侧的 ImuCollector 完成。
void pushImu(long long tsNs, float wx, float wy, float wz,
             float ax, float ay, float az);

// ---------------- 开关（主线程调）----------------

/**
 * 注入相机内参（主线程调）。只允许在 start() 之前：帧没在跑时上游没有并发读。
 * 同时刷新测距侧缓存的投影内参。
 */
void setIntrinsics(float fx, float fy, float cx, float cy,
                   float k1, float k2, float p1, float p2);

/**
 * 注入 IMU 噪声密度（主线程调），与 setIntrinsics 同一前置条件：只在 start() 之前。
 * 返回 false 表示 System 没构造好或正在吃帧，调用方决定要不要提示。
 */
bool setImuNoise(float ng, float na, float ngw, float naw);

/** 开始吃帧，不销毁 System。 */
void start();

/** 停止吃帧并清空两个队列，System 保留。 */
void stop();

// ---------------- 查询（主线程调）----------------

/** 4x4 列主序；返回 false 表示还没有位姿。 */
bool pose(float out16[16]);

/** eTrackingState：-1 未就绪 / 0 无帧 / 1 惯性初始化 / 2 OK / 3 刚丢 / 4 完全丢。 */
int trackingState();

/** 本次运行的地图重置次数，start() 归零。 */
int resetCount();

/** 当前轨迹点数，只含跟踪 OK 的帧。 */
int trajSize();

/**
 * 导出轨迹：out 收 3n 个 float（世界米 x y z），返回点数。
 * 按 stride 抽稀，覆盖全程并包含最新点。
 */
int getTrajectory(float* out, int cap);

// ---------------- 测距（画面中心）----------------
//
// 十字固定在画面中心，不需要传入测量位置；平滑也在这一侧完成。

/** 开关测距（主线程调）。关闭时只停止计算，不影响 SLAM。 */
void setMeasuring(bool on);

/** 发布一帧测量结果（SLAM 线程调），内部做中值平滑和日志限流。 */
void publishMeasure(const CenterMeasure& m);

/**
 * 读最近一次测量（主线程调）。out 至少 7 个：
 * [0] 距离（米）、[1..3] 目标世界坐标、[4][5] 目标像素坐标、[6] 附近特征点数。
 * 返回 MeasureQuality。
 */
int getMeasure(float out7[7]);

// ---------------- 多点折线 ----------------
// 刷新、投影、算长度一次给完，Kotlin 只负责绘制。全部主线程调。

/**
 * 把当前读数钉成一个点；没有有效读数返回 false。
 * snapTo >= 0 时复制第 snapTo 个点的身份，而不是用十字当前命中的点。
 */
bool addPinned(int snapTo);

void undoPinned();
void clearPinned();

/** 断开折线：下一个点另起一条，不再拉橡皮筋。 */
void closePinnedChain();

/**
 * 读折线：刷新坐标、投影、算长度一次给完。
 * out 至少 4n+1：每点 4 个 px py flags lenToNext，最后是 liveLen
 * （-1 表示没有读数或已断开）。
 * flags：bit0 在相机前方 / bit1 新折线起点 / bit2 之后断开 / bit3 不太准。
 * 返回实际写入的点数（<= cap/4），装不下时少给。
 */
int getPinned(float* out, int cap);

// ---------------- 路径点 ----------------

/** 在当前位置记录路径点（主线程调），请求发出后由 SLAM 线程在下一帧执行。 */
void requestAddWaypoint(const std::string& name);

/** 加路径点状态：0 没试过 / 1 排队 / 2 成功 / 3 失败。 */
int waypointState();

/** 失败原因，成功时为空串。 */
std::string waypointMsg();

/** 改名或删除（主线程调）。 */
void renameWaypoint(int id, const std::string& name);
void deleteWaypoint(int id);

/** 每次变化加一，界面据此重建叠加层。 */
long long waypointRevision();

/**
 * 读路径点世界坐标。pos 每点 3 个 float；meta 每点 id, mapId, kfId, flags
 * （bit0 坐标有效 / bit1 在当前图）。装不下就少给，返回实际条数。
 */
int getWaypoints(float* pos, int posCap, long long* meta, int metaCap,
                 std::string* names, int nameCap);

/** 路径点总条数（主线程调）。 */
int waypointCount();

/** 下一个路径点 id，默认名字用它编号（主线程调）。 */
int nextWaypointId();

// ---------------- 存图 ----------------

/**
 * 把当前地图另存到 path + ".osa"（主线程调，只在 SLAM 停止后调）。
 * 只发请求，序列化在 SLAM 线程完成；进度看 saveState()。
 */
void requestSaveMap(const std::string& path);

/** 0 空闲 / 1 排队 / 2 保存中 / 3 完成 / 4 失败。 */
int saveState();

/**
 * 把 atlas 恢复到上一次保存的状态（主线程调）：
 * 有成功存图就用它，否则用构造时载入的图，都没有则清成空图。
 * 一趟结束但没保存时调用，避免旧图留在 atlas 里被后续回环匹配到。
 */
void requestRestoreMap();

/** 0 空闲 / 1 排队 / 2 恢复中 / 3 完成 / 4 失败。 */
int restoreState();

/**
 * 把 atlas 换成 path 指向的图（主线程调）。空串表示清成空图。
 * 入口页每次重新选择时调用；成功后该路径成为新的恢复基准。
 * 和 requestRestoreMap 共用队列与 restoreState。
 */
void requestLoadAtlas(const std::string& path);

/** 当前 atlas 对应的路径（不含 .osa），空串表示新建/空图。 */
std::string loadedMapPath();

/** atlas 的图数、关键帧数、地图点数，out 至少 3 个。 */
void atlasStats(int out3[3]);

/** 构造失败细分：0 没失败 / 1 载图失败 / 2 其他。 */
int failureCode();

// ---------------- 2D 地图的点云 ----------------

/**
 * 读一次地图点云快照（主线程调）。out 收 3n 个 float，返回点数。
 * 数据是 SLAM 线程每秒重建的快照；按 stride 抽稀（上限 5000）。
 */
int getMapPoints(float* out, int cap);

/** 开关点云快照（主线程调）；关闭时 SLAM 线程不重建。 */
void setMapCloudEnabled(bool on);

// ---------------- 轨迹 ----------------

/**
 * 把本趟位姿写成 CSV（每行 秒, tx,ty,tz, qx,qy,qz,qw，TUM 格式）。
 * 位姿历史保存在 C++，停止或退出时一次性导出。
 */
void dumpTrajectory(const std::string& path);

/**
 * 本趟最后一段轨迹所属的图（主线程调），-1 表示没有。
 * 值在 stop() 中快照，因为之后 atlas 可能被恢复请求换掉。
 */
long long lastTrajMapId();

/** 本趟合并链（主线程调）：每条 10 个 float，按发生顺序。 */
int lastMerges(float* out, int cap);

/**
 * 当前轨迹此刻所属的图（主线程调），-1 表示没有。
 * 与 lastTrajMapId 的区别是实时读取。
 */
long long trajectoryMapId();

/** atlas 中有关键帧的图（主线程调，按 id 升序），返回实际写入的图数。 */
int mapList(long long* ids, int* kfs, int* mps, int cap);

/** 指定图的点云（主线程调，使用该图自己的坐标系）。out 收 3n 个 float。 */
int mapCloud(long long mapId, float* out, int cap);

/**
 * 界面显示用的「文件夹 id 到当前坐标系」变换（主线程调）。
 * 每条 9 个 float：id s qx qy qz qw tx ty tz；第一条固定为 displayMapId。
 */
int displayTransforms(float* out, int cap);

/**
 * 读取已归档轨迹并变换到显示坐标系（主线程调）。
 * 文件每行 time tx ty tz qx qy qz qw（Twc），只取位置：
 * tw' = s*(q_on*tw) + t_on。按 stride 抽稀到 cap/3 点。
 */
int readTrajFile(const std::string& path,
                 float s, float qx, float qy, float qz, float qw,
                 float tx, float ty, float tz,
                 float* out, int cap);

/**
 * 把轨迹文件按合并变换写入 dst（主线程调），成功返回 true。
 * 写副本而不是原地修改，调用方负责全部成功后再统一改名。
 */
bool transformTrajFile(const std::string& src, const std::string& dst,
                       float s, float qx, float qy, float qz, float qw,
                       float tx, float ty, float tz);

}  // namespace slam
