// 一帧裸字节到一次 TrackMonocular 的薄封装。
//
// slam_engine.cpp 所在的 target 不链 OpenCV：orb_slam3 已经静态链了一份，
// 同一进程里出现两套 cv:: 全局状态会互相踩。构造 cv::Mat 的事留给这一侧。
#pragma once

#include <vector>

#include "ImuTypes.h"   // ORB_SLAM3::IMU::Point

namespace ORB_SLAM3 { class System; }

namespace slam {

/** 一次跟踪的结果。字段对应 SaveTrajectoryEuRoC 的 8 列。 */
struct TrackResult {
    bool valid = false;
    double t = 0;
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
    /** 4x4 列主序（Twc）。 */
    float pose16[16] = {0};

    /** 本帧的 eTrackingState，每帧都填。 */
    int trackingState = -1;

    /** 当前图的关键帧数。掉到 0 表示发生过重置。 */
    unsigned long kfCount = 0;

    /**
     * 4x4 列主序的 Tcw（世界到相机），与 pose16 方向相反。
     * System 拿不到当前帧位姿，测距需要的这份只能从这里带出去。
     */
    float t_cw16[16] = {0};
};

/**
 * 调一次 System::TrackMonocular。
 *
 * gray 是 w*h 的连续灰度缓冲，只需在调用期间有效（内部会 clone）。
 */
bool orbTrack(ORB_SLAM3::System* sys, const unsigned char* gray, int w, int h,
              double timestamp, const std::vector<ORB_SLAM3::IMU::Point>& imu,
              TrackResult* out);

}  // namespace slam
