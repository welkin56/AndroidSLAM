// 「十字指的地方有多远」。十字固定在画面中心，所以**没有「测哪儿」这个参数**。
//
// ⚠️ 但它不是纯函数 —— 要跨帧记住「最近见过哪些地图点」，见 CenterMeasureState。
// ⚠️ 接口里不能出现 `cv::` 和 Eigen：`slam_engine.cpp` 会 include 这个头，而那个 target
//    刻意不链 OpenCV（理由见 orb_track.h）。
#pragma once

#include <string>

namespace ORB_SLAM3 {
class System;
class MapPoint;
}  // namespace ORB_SLAM3

namespace slam {

/** 可测性分档。**0 必须显示成一句话**，不能沉默（STAGE6 §2.7）。 */
enum MeasureQuality {
    kNone = 0,
    kFair = 1,
    kGood = 2,
};

struct CenterMeasure {
    int quality = kNone;
    int method = 0;      // 1 = 地图点，2 = 局部平面
    float distance = 0;  // 相机心 → 目标，米
    float wx = 0, wy = 0, wz = 0;  // `method == 2` 时无效

    /**
     * 命中的地图点，`method == 1` 时非空。
     */
    ORB_SLAM3::MapPoint* mp = nullptr;

    float px = -1, py = -1;        // 命中处的图像像素，给 UI 画小圈

    int source = 0;    // 1 新选 / 2 沿用锁定。日志用
    int candidates = 0;  // 当帧宽锥内的候选数。日志用
    int support = 0;     // 环里够支持的**条目数**（不去重，所以是时间加权）。日志用
    int texture = 0;     // 宽锥内的**特征点**总数（不管有没有地图点）。日志用
    float zSpan = 0;     // 当帧候选深度的最大相对跨度。日志用
};

/** 环容量。**它同时就是时间窗** —— 老化完全靠「满了覆盖最旧的」。 */
constexpr int kSeenCap = 256;

/** 跨帧要留住的东西。 */
struct CenterMeasureState {
    /** 环里的一条：一个**最近见过**的地图点。**不去重** —— 同一点出现几次就有几条。 */
    struct Seen {
        ORB_SLAM3::MapPoint* mp;
        float pw[3];  // 世界坐标。每帧用当帧位姿现算角度和深度
    };
    Seen seen[kSeenCap] = {};
    int seenHead = 0;  // 下一次写哪

    /** 光轴锁定：只是「环里的哪一个」，坐标在环里。 */
    ORB_SLAM3::MapPoint* locked = nullptr;
    float lockedPx = -1, lockedPy = -1;  // 它最后一次已知的像素（画小圈）

    /** 深度锁定：上一帧到锁定点的距离，和上一帧相机在世界系里的位置。 */
    float lastD = 0;
    float lastCam[3] = {0, 0, 0};
};

/**
 * 在画面中心测一次。**跑在 SLAM 线程上**，紧跟 `orbTrack` 之后。
 *
 * `t_cw` 是 4x4 **列主序**的 `Tcw`。`st` **必须非空** —— 支持度和锁定都靠它那个环。
 */
CenterMeasure centerMeasure(ORB_SLAM3::System* sys, int w, int h, const float t_cw[16],
                            CenterMeasureState* st);

// ---------------- 投影：把已钉住的世界点画回画面 ----------------
// 只管**画**，不管距离 —— 点之间的距离是纯世界坐标的算式，和位姿无关。

/** 读一次内参。幂等。 */
void initIntrinsics(const std::string& settingsPath);

/**
 * 取内参给调用方自己算投影。没读到（`initIntrinsics` 没跑成）返回 false。
 *
 * ⚠️ **为什么不是「世界坐标 → 像素」一个函数包掉**：一条线段的方向**要由两端共同决定**，
 * 不能只看一个点 —— 点的屏幕位置只由视线方向决定，**深度不影响**，所以单看它推不出
 * 线段该往哪走。投影因此搬到了 `slam_engine.cpp` 里**逐线段**做。
 */
bool cameraIntrinsics(float* fx, float* fy, float* cx, float* cy);

}  // namespace slam
