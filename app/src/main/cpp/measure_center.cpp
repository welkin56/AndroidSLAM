// 见 measure_center.h。两条路径：最近的 MapPoint（主力），局部平面（兜底）。
//
// 上游一个字都不用改 —— 包括路径 2 用的内参：相机坐标系里的平面，它的 `1/Z`
// 在**像素坐标**上是仿射的，所以最小二乘不需要 K。推导见 planeDepthAt。
#include "measure_center.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Dense>

#include "MapPoint.h"
#include "System.h"

namespace slam {
namespace {

// ORB_SLAM3 `eTrackingState::OK`（Tracking.h:121）。
constexpr int kTrackingOk = 2;

// 吸附半径 / 平面兜底的邻域半径，都是**占图像宽度**的比例。
//
// STAGE6 §2.5 / §4 给的是归一化数（0.02 / 0.15），换成像素要乘 fx，而 `System`
// 没暴露内参。这里拿图像宽度顶替 fx（本机 fx≈435、w=640，差 1.5 倍以内）——
// 反正是手感参数，而且这样是分辨率成比例的，换分辨率不会走样。
constexpr float kSnapFrac = 0.02f;
constexpr float kConeFrac = 0.15f;

/** 吸附得够近才算「好」，否则只能算「勉强」。 */
constexpr float kGoodFrac = 0.4f;

/** 平面拟合的内点判据：相对深度误差超过这个比例就剔掉。 */
constexpr float kPlaneRelTol = 0.05f;
constexpr size_t kMinPlanePoints = 5;

/** 相机后方的点不算数。 */
constexpr float kMinZ = 0.05f;

struct Center {
    float u, v;
};

/**
 * 十字对应的**去畸变后**像素坐标。
 *
 * ⚠️ 现在 `k1/k2/p1/p2` 全是 0 ⟹ 去畸变是恒等的，图像中心就是去畸变坐标系的中心。
 * 一旦标定出非零畸变，这两个坐标系就不再重合（`mvKeysUn` 是去畸变后的，预览显示的是原图），
 * 那时**只有这一处要改**：把 (w/2, h/2) 过一遍畸变模型。见 STAGE6 §3。
 */
inline Center centerPx(int w, int h) { return {w * 0.5f, h * 0.5f}; }

/** 离十字最近的、还活着的地图点。下标和 `mvKeysUn` / `mvpMapPoints` 一一对应。 */
bool nearestMapPoint(const std::vector<cv::KeyPoint>& kps,
                     const std::vector<ORB_SLAM3::MapPoint*>& mps, const Center& c,
                     float radiusPx, int* outIdx, float* outDpx) {
    // ⚠️ 两个数组长度不等就会越界。上游是等长的，但这里取 min 兜一道 ——
    // 越界在这里是**静默**的（读到的是错位的点，不是崩）。
    const size_t n = std::min(kps.size(), mps.size());
    const float r2 = radiusPx * radiusPx;

    // ⚠️ **先按像素筛，再查 `isBad()`**，顺序不能反。
    // `isBad()` 要同时拿 `mMutexFeatures` 和 `mMutexPos` 两把锁（MapPoint.cc），
    // 而 `LocalMapping` / `LoopClosing` 也在抢 —— 500 个点逐个查就是 500 次双锁，
    // 平均值不大，但**尾延迟**难看（偶尔被挡一下就是毫秒）。
    // 先筛之后半径内通常只剩两三个点，锁的次数从 500 降到个位数。
    bool found = false;
    float best2 = 0;
    int bi = -1;
    for (size_t i = 0; i < n; ++i) {
        if (mps[i] == nullptr) continue;

        const float dx = kps[i].pt.x - c.u;
        const float dy = kps[i].pt.y - c.v;
        const float d2 = dx * dx + dy * dy;
        if (d2 > r2) continue;

        if (found && d2 >= best2) continue;   // 有个更近的候选了，连锁都不用拿

        if (mps[i]->isBad()) continue;
        found = true;
        best2 = d2;
        bi = static_cast<int>(i);
    }

    if (!found) return false;
    *outIdx = bi;
    *outDpx = std::sqrt(best2);
    return true;
}

/**
 * 用局部地图点拟合一个平面，再取**十字处沿光轴的深度**。
 *
 * 不需要内参，靠的是这个恒等式：相机坐标系里的一个平面
 *
 *     n·X = d,   X = Z·(x, y, 1),   x = (u-cx)/fx,  y = (v-cy)/fy
 *
 * 代入展开就是 `1/Z = a·u + b·v + c` —— **在像素坐标上是仿射的**，
 * 内参和平面参数全被 a/b/c 吸收了。所以直接对 `1/Z` 做最小二乘就行，
 * 最后在十字处取值。
 *
 * 比三点 RANSAC 稳：先用全部点拟合一遍，把相对残差大的剔掉再拟合一次 ——
 * 邻域里同时有地板和桌子时，第一遍能把「另一张面」暴露出来。
 */
bool planeDepthAt(const std::vector<cv::KeyPoint>& kps,
                  const std::vector<ORB_SLAM3::MapPoint*>& mps, const Eigen::Matrix4f& Tcw,
                  const Center& c, float coneRadiusPx, float* outZ) {
    const size_t n = std::min(kps.size(), mps.size());

    std::vector<Eigen::Vector3f> uvz;  // (u, v, 1/Z)
    uvz.reserve(64);
    for (size_t i = 0; i < n; ++i) {
        ORB_SLAM3::MapPoint* mp = mps[i];
        if (mp == nullptr) continue;

        // 同 nearestMapPoint：**先按像素筛，再拿 `isBad()` 的锁**。
        const float dx = kps[i].pt.x - c.u;
        const float dy = kps[i].pt.y - c.v;
        if (dx * dx + dy * dy > coneRadiusPx * coneRadiusPx) continue;
        if (mp->isBad()) continue;

        // `GetWorldPos()` 自己加锁（MapPoint.cc:198），取到就用，别留着。
        const float z = (Tcw * mp->GetWorldPos().homogeneous()).z();
        if (z < kMinZ) continue;

        uvz.emplace_back(kps[i].pt.x, kps[i].pt.y, 1.0f / z);
    }
    if (uvz.size() < kMinPlanePoints) return false;

    // 解 [u v 1] · (a,b,c) = 1/Z 的正规方程。返回 false = 点退化（比如全挤在一条像素线上）。
    auto fit = [](const std::vector<Eigen::Vector3f>& pts, Eigen::Vector3f* abc) {
        Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();
        for (const Eigen::Vector3f& p : pts) {
            const Eigen::Vector3f r(p.x(), p.y(), 1.0f);
            A += r * r.transpose();
            b += r * p.z();
        }
        const Eigen::FullPivLU<Eigen::Matrix3f> lu(A);
        if (!lu.isInvertible()) return false;
        *abc = lu.solve(b);
        return true;
    };

    Eigen::Vector3f abc;
    if (!fit(uvz, &abc)) return false;

    std::vector<Eigen::Vector3f> inliers;
    inliers.reserve(uvz.size());
    for (const Eigen::Vector3f& p : uvz) {
        const float inv = abc.x() * p.x() + abc.y() * p.y() + abc.z();
        if (inv <= 1e-6f) continue;  // 拟合出「负深度」，明显是外点
        // 相对深度误差 |Z_i - Z_fit| / Z_i。在 1/Z 上用乘的比在 Z 上用减的稳。
        if (std::fabs(inv - p.z()) / inv > kPlaneRelTol) continue;
        inliers.push_back(p);
    }

    // 内点不过半 ⟹ 邻域里多半有**两张面**（地板 + 桌子），混合平面的值不能用。
    if (inliers.size() < kMinPlanePoints || inliers.size() * 2 < uvz.size()) return false;
    if (!fit(inliers, &abc)) return false;

    const float inv = abc.x() * c.u + abc.y() * c.v + abc.z();
    if (inv <= 1e-6f) return false;
    *outZ = 1.0f / inv;
    return true;
}

}  // namespace

CenterMeasure centerMeasure(ORB_SLAM3::System* sys, int w, int h, const float t_cw[16]) {
    CenterMeasure out;
    if (sys == nullptr || w <= 0 || h <= 0) return out;

    // 只有跟踪 OK 的帧才谈得上「这里有多远」。其他状态返回 kNone —— 调用方照样要发布，
    // 否则 UI 会一直留着上一帧的数字。
    if (sys->GetTrackingState() != kTrackingOk) return out;

    // 两个都是**按值返回的拷贝**，而且和写入方锁同一个 `mMutexState`
    // （写 System.cc:321-323，读 :1327 / :1333）—— 所以任意线程都能调。
    const std::vector<cv::KeyPoint> kps = sys->GetTrackedKeyPointsUn();
    const std::vector<ORB_SLAM3::MapPoint*> mps = sys->GetTrackedMapPoints();
    if (kps.empty() || mps.empty()) return out;

    Eigen::Matrix4f Tcw;
    std::memcpy(Tcw.data(), t_cw, sizeof(float) * 16);

    const Center c = centerPx(w, h);
    const float fw = static_cast<float>(w);

    // ---- 路径 1：最近的活地图点（主力）----
    int idx = -1;
    float dpx = 0;
    if (nearestMapPoint(kps, mps, c, kSnapFrac * fw, &idx, &dpx)) {
        const Eigen::Vector3f pw = mps[idx]->GetWorldPos();
        const Eigen::Vector3f pc = (Tcw * pw.homogeneous()).head<3>();
        if (pc.z() > kMinZ) {
            out.quality = dpx <= kGoodFrac * kSnapFrac * fw ? kGood : kFair;
            out.method = 1;
            out.distance = pc.norm();  // 斜距，和激光测距仪报的是同一种量
            out.wx = pw.x();
            out.wy = pw.y();
            out.wz = pw.z();
            out.px = kps[idx].pt.x;
            out.py = kps[idx].pt.y;
            return out;
        }
    }

    // ---- 路径 2：局部平面兜底 ----
    float z = 0;
    if (planeDepthAt(kps, mps, Tcw, c, kConeFrac * fw, &z)) {
        out.quality = kFair;  // 兜底出来的，一律不当「好」
        out.method = 2;
        out.distance = z;
        // ⚠️ **不给世界坐标**：沿光轴只知道深度，要变成点得知道十字的归一化方向，那要内参。
        // STAGE6 §2.3 的「两点距离」将来要用它 —— 到时候再加 `System::GetK()`。
        out.px = c.u;
        out.py = c.v;
    }
    return out;
}

}  // namespace slam
