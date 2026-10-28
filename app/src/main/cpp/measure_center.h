// 画面中心处的距离测量。十字固定在中心，所以没有「测哪里」参数。
//
// 有跨帧状态（CenterMeasureState）；接口不出现 cv:: 和 Eigen，
// 因为 slam_engine.cpp 不链 OpenCV。
#pragma once

#include <string>

namespace ORB_SLAM3 {
class System;
class MapPoint;
}  // namespace ORB_SLAM3

namespace slam {

/** 可测性分档。0 也要在界面上给出说明。 */
enum MeasureQuality {
    kNone = 0,
    kFair = 1,
    kGood = 2,
};

struct CenterMeasure {
    int quality = kNone;
    int method = 0;      // 1 = 地图点，2 = 局部平面
    float distance = 0;  // 相机中心到目标，米
    float wx = 0, wy = 0, wz = 0;  // method == 2 时无效

    /** 命中的地图点，method == 1 时非空。 */
    ORB_SLAM3::MapPoint* mp = nullptr;

    float px = -1, py = -1;        // 命中处的图像像素，供界面画圈

    int source = 0;      // 1 新选 / 2 沿用锁定，日志用
    int candidates = 0;  // 当帧宽锥内的候选数，日志用
    int support = 0;     // 环内支持条目数（不去重），日志用
    int texture = 0;     // 宽锥内特征点总数，日志用
    float zSpan = 0;     // 当帧候选深度的最大相对跨度，日志用
};

/** 环容量，同时也是时间窗：满了覆盖最旧的。 */
constexpr int kSeenCap = 256;

/** 跨帧保留的状态。 */
struct CenterMeasureState {
    /** 环内一条记录：最近见过的一个地图点。不去重。 */
    struct Seen {
        ORB_SLAM3::MapPoint* mp;
        float pw[3];  // 世界坐标，每帧用当前位姿现算角度和深度
    };
    Seen seen[kSeenCap] = {};
    int seenHead = 0;  // 下一次写入位置

    /** 光轴锁定：记录锁定的是环里的哪个点。 */
    ORB_SLAM3::MapPoint* locked = nullptr;
    float lockedPx = -1, lockedPy = -1;  // 最后一次已知像素，供画圈

    /** 深度锁定：上一帧到锁定点的距离，以及上一帧相机世界坐标。 */
    float lastD = 0;
    float lastCam[3] = {0, 0, 0};
};

/**
 * 在画面中心测一次，跑在 SLAM 线程上，紧跟在 orbTrack 之后。
 *
 * t_cw 是 4x4 列主序的 Tcw；st 不能为空。
 */
CenterMeasure centerMeasure(ORB_SLAM3::System* sys, int w, int h, const float t_cw[16],
                            CenterMeasureState* st);

// ---------------- 投影：把已钉住的世界点画回画面 ----------------

/** 读一次内参，幂等。 */
void initIntrinsics(const std::string& settingsPath);

/** 取内参给调用方自行投影。initIntrinsics 未成功时返回 false。 */
bool cameraIntrinsics(float* fx, float* fy, float* cx, float* cy);

/** 覆盖缓存的内参。App 侧注入真实内参后必须调一次，否则投影还用 yaml 里的占位值。 */
void updateIntrinsics(float fx, float fy, float cx, float cy);

}  // namespace slam
