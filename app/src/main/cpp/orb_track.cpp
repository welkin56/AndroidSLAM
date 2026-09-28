// 见 orb_track.h —— 这个文件存在的唯一理由是「让不链 OpenCV 的那一层也能喂帧」。
#include "orb_track.h"

#include <cstring>

#include <opencv2/core.hpp>

#include "System.h"

namespace slam {

bool orbTrack(ORB_SLAM3::System* sys, const unsigned char* gray, int w, int h,
              double timestamp, const std::vector<ORB_SLAM3::IMU::Point>& imu,
              TrackResult* out) {
    if (sys == nullptr || gray == nullptr || out == nullptr || w <= 0 || h <= 0) return false;

    // 灰度 640x480 连续缓冲。Kotlin 侧 `extractGray()` 已经拍平成 w*h，
    // 没有 rowStride 的问题（§7 报错速查第 10 条）。
    // `cv::Mat` 不持有这块内存，`TrackMonocular` 进来第一件事就是 `im.clone()`。
    cv::Mat im(h, w, CV_8UC1, const_cast<unsigned char*>(gray));

    const Sophus::SE3f Tcw = sys->TrackMonocular(im, timestamp, imu);

    // `Sophus::SE3f` 没有 `empty()`。用 allFinite 判 —— 初始化之前 `Frame::mTcw`
    // 是没赋过值的 Eigen 对象，可能是 NaN/Inf。
    if (!Tcw.matrix().allFinite()) return false;

    // ⚠️ 存 **Twc**（= Tcw.inverse()），和 `SaveTrajectoryEuRoC`（`System.cc:616-622`）一致 ——
    // 那样和 PC 跑出来的 CameraTrajectory.txt 就是同款格式，直接 diff。
    // 存成 Tcw 的话对照会看着像旋转错位，白查一轮。
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
