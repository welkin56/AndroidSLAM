// 见 orb_track.h。存在的唯一理由：让不链 OpenCV 的那一层也能喂帧。
#include "orb_track.h"

#include <cstring>

#include <opencv2/core.hpp>

#include "System.h"

namespace slam {

namespace {
// ORB_SLAM3 eTrackingState::OK。
constexpr int kTrackingOk = 2;
}  // namespace

bool orbTrack(ORB_SLAM3::System* sys, const unsigned char* gray, int w, int h,
              double timestamp, const std::vector<ORB_SLAM3::IMU::Point>& imu,
              TrackResult* out) {
    if (sys == nullptr || gray == nullptr || out == nullptr || w <= 0 || h <= 0) return false;

    // 灰度 640x480 连续缓冲，Kotlin 侧的 extractGray() 已经拍平成 w*h。
    // cv::Mat 不持有这块内存，TrackMonocular 内部会 clone。
    cv::Mat im(h, w, CV_8UC1, const_cast<unsigned char*>(gray));

    const Sophus::SE3f Tcw = sys->TrackMonocular(im, timestamp, imu);

    out->trackingState = sys->GetTrackingState();
    out->kfCount = sys->KeyFramesInMap();

    // Tcw（世界到相机），与下面保存的 Twc 方向相反。测距需要它，见 orb_track.h。
    {
        const Eigen::Matrix4f m_cw = Tcw.matrix();
        std::memcpy(out->t_cw16, m_cw.data(), sizeof(out->t_cw16));
    }

    // 只认 OK 的帧。其他状态下返回单位阵，allFinite() 照样通过，
    // 存下来会得到一堆原点位姿。
    if (out->trackingState != kTrackingOk) return false;

    // 过滤 NaN/Inf。Sophus::SE3f 没有 empty()。
    if (!Tcw.matrix().allFinite()) return false;

    // 存 Twc（Tcw 的逆），与 SaveTrajectoryEuRoC 的输出格式一致，便于和 PC 端对比。
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Matrix4f m = Twc.matrix();
    std::memcpy(out->pose16, m.data(), sizeof(out->pose16));

    const Eigen::Vector3f t3 = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();
    out->t = timestamp;
    out->x = t3.x(); out->y = t3.y(); out->z = t3.z();
    out->qx = q.x(); out->qy = q.y(); out->qz = q.z(); out->qw = q.w();
    out->valid = true;
    return true;
}

}  // namespace slam
