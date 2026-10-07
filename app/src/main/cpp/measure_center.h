// 「十字指的地方有多远」。
//
// 十字**固定在画面中心**，所以这一层不需要任何 UI 输入 —— 它是「每来一帧就顺手算一次」
// 的纯函数。不需要往返、不需要加锁、不需要存状态。
//
// ⚠️ 接口里**不能出现 `cv::` 和 Eigen**：`slam_engine.cpp` 会 include 这个头，而那个
// target 刻意不链 OpenCV（理由见 orb_track.h 开头）。所以位姿走 float[16]，
// 结果走纯 float 的结构体。实现放在 .cpp 里，那个文件编进 orb_slam3。
#pragma once

namespace ORB_SLAM3 { class System; }

namespace slam {

/** 可测性分档。**0 必须显示成一句话** —— 沉默和「测不了」要能区分（STAGE6 §2.7）。 */
enum MeasureQuality {
    kNone = 0,  // 十字附近没有可用的地图点
    kFair = 1,  // 测到了，但离十字偏远，或者是平面兜底出来的
    kGood = 2,  // 吸附得很近
};

/** 一次测量。 */
struct CenterMeasure {
    int quality = kNone;
    int method = 0;  // 1 = 最近的 MapPoint，2 = 局部平面
    float distance = 0;  // 相机心 → 目标，米

    /** 目标的世界坐标。**`method == 2` 时无效**（沿光轴只知道深度，要变成点得有内参）。 */
    float wx = 0, wy = 0, wz = 0;

    /** 命中处的图像像素坐标，给 UI 画「会吸到哪」那个小圈。没命中时是负数。 */
    float px = -1, py = -1;
};

/**
 * 在画面中心测一次。**跑在 SLAM 线程上**，紧跟 `orbTrack` 之后。
 *
 * `t_cw` 是 4x4 **列主序**的 `Tcw`（世界 → 相机），就是 `TrackMonocular` 返回的那个。
 * 没准备好 / 跟踪不 OK 时返回 `quality == kNone`。
 */
CenterMeasure centerMeasure(ORB_SLAM3::System* sys, int w, int h, const float t_cw[16]);

}  // namespace slam
