// 「十字指的地方有多远」。十字固定在画面中心，所以**没有「测哪儿」这个参数**。
//
// ⚠️ 但它不是纯函数 —— 要跨帧记住「盯住的是哪个点」，见 CenterMeasureState。
// ⚠️ 接口里不能出现 `cv::` 和 Eigen：`slam_engine.cpp` 会 include 这个头，而那个 target
//    刻意不链 OpenCV（理由见 orb_track.h）。
#pragma once

namespace ORB_SLAM3 {
class System;
class MapPoint;
}  // namespace ORB_SLAM3

namespace slam {

/** 可测性分档。**0 必须显示成一句话**，不能沉默（STAGE6 §2.7）。 */
enum MeasureQuality {
    kNone = 0,  // 十字附近没有可用的地图点
    kFair = 1,  // 测到了，但离十字偏远，或者是平面兜底出来的
    kGood = 2,  // 吸附得很近
};

struct CenterMeasure {
    int quality = kNone;
    int method = 0;      // 1 = 地图点，2 = 局部平面
    float distance = 0;  // 相机心 → 目标，米
    float wx = 0, wy = 0, wz = 0;  // `method == 2` 时无效（不知道方向就变不成点）
    float px = -1, py = -1;        // 命中处的图像像素，给 UI 画小圈
    int source = 0;                // 1 新选 / 2 沿用锁定。日志用
    int candidates = 0;            // 吸附半径内几个候选。日志用
    int support = 0;               // 其中深度和它接近的（「陪着它」的）有几个。日志用
    float zSpan = 0;               // 候选深度的最大相对跨度。日志用
};

/** 跨帧要留住的东西 —— 三条闸门里两条要用它，见 .cpp 的 ①②④。 */
struct CenterMeasureState {
    // ① 光轴锁定：盯住的点。它的世界坐标不会变
    ORB_SLAM3::MapPoint* locked = nullptr;
    float px = -1, py = -1;  // 它最后一次已知的图像像素
    // ④ 深度锁定：上一帧到它的距离，和上一帧相机在世界系里的位置
    float lastD = 0;
    float lastCam[3] = {0, 0, 0};
    int missFrames = 0;  // 它连续多少帧没被匹配上
};

/**
 * 在画面中心测一次。**跑在 SLAM 线程上**，紧跟 `orbTrack` 之后。
 *
 * `t_cw` 是 4x4 **列主序**的 `Tcw`。`st` 可以传 `nullptr`（退化成每帧重新选、不锁定）。
 */
CenterMeasure centerMeasure(ORB_SLAM3::System* sys, int w, int h, const float t_cw[16],
                            CenterMeasureState* st);

}  // namespace slam
