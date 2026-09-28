// 「一帧裸字节 → 一次 TrackMonocular」的一层薄包装。
//
// **为什么需要它：** `slam_engine.cpp` 属于 `slam_jni` 这个 target，而那个 target
// **不链 OpenCV** —— 这是刻意的。DBoW2 / orb_slam3 里已经静态链了一份 OpenCV，
// 再来一份会让同一个进程里出现**两套 `cv::` 的全局状态**。
//
// 真机上为此崩过一次：`Frame::ExtractORB` → `ORBextractor` → `cv::GaussianBlur`
// → **`cv::parallel_for_` 被解析到了 libDBoW2.so 那一份** —— 那是 OpenCV 里带
// **全局线程池**的东西，两个副本的状态互相踩。完整调用链见
// docs/changes/2026-09-28-m5-slam-on-phone.md。
//
// 所以「构造 `cv::Mat`」这件事必须留在链着 OpenCV 的那一侧 —— 就是这个文件。
// 它编进 `orb_slam3`，`slam_engine.cpp` 只当它是个普通的 C++ 函数。
#pragma once

#include <vector>

#include "ImuTypes.h"   // ORB_SLAM3::IMU::Point

namespace ORB_SLAM3 { class System; }

namespace slam {

/** 一次跟踪的结果。字段和 `SaveTrajectoryEuRoC` 写的 8 列一一对应。 */
struct TrackResult {
    bool valid = false;
    double t = 0;
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
    /** 4x4 **列主序**。 */
    float pose16[16] = {0};
};

/**
 * 调一次 `System::TrackMonocular`。
 *
 * `gray` 是 `w*h` 的连续灰度缓冲，**只要在这个调用期内有效**（里面会 clone）。
 */
bool orbTrack(ORB_SLAM3::System* sys, const unsigned char* gray, int w, int h,
              double timestamp, const std::vector<ORB_SLAM3::IMU::Point>& imu,
              TrackResult* out);

}  // namespace slam
