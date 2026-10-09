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

// 三个半径都是**占图像宽度**的比例。STAGE6 给的是归一化数（要乘 fx），而 `System`
// 没暴露内参 —— 这里拿图像宽度顶替 fx（本机 fx≈435、w=640，差 1.5 倍以内），
// 反正是手感参数，而且这样分辨率成比例，换分辨率不会走样。
constexpr float kSnapFrac = 0.02f;         // 锚点：要「正好指着」，所以窄
constexpr float kSupportConeFrac = 0.05f;  // 支持度：要数「这一片是不是实心的」，所以宽
constexpr float kConeFrac = 0.15f;         // 平面兜底的邻域

/** 角度够小才算「好」，否则只能算「勉强」。按角度比，所以是个比值。 */
constexpr float kGoodFrac = 0.4f;

/** 吸附半径对应的角度：像素半径是 `kSnapFrac·w`，把 `w` 当 `fx` 用 ⟹ `θ = kSnapFrac`。 */
constexpr float kSnapAngleRad = kSnapFrac;

/** 解锁的角度，**比吸附宽一倍** —— 滞回的释放必须比获取宽，否则选中的点自己锁不住。 */
constexpr float kLockAngleRad = 2.0f * kSnapAngleRad;

/** 支持度的**纵深**容差（同一帧内）：深度差在这个比例以内，算「陪着 anchor」。 */
constexpr float kSupportDepthFrac = 0.20f;

/** 至少这么多个点陪着，anchor 才可信；不够就降级平面法。 */
constexpr int kMinSupport = 3;

/** 距离的帧间跳变 > `max(kDepthJumpMin, kDepthJumpGain × 相机位移)` ⟹ 锁定点出问题了。 */
constexpr float kDepthJumpGain = 3.0f;
constexpr float kDepthJumpMin = 0.10f;

/** 锁定点连续这么多帧没被匹配，就判它已经不在视野里（物体走了 / 被挡住）。 */
constexpr int kLockMaxMiss = 30;

/** 半径内深度跨度超过这个比例 = 同时有前后两层，读数只能算「勉强」。 */
constexpr float kAmbiguousSpan = 0.3f;

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

/** 一个候选：某个特征点匹配上了地图点，而且落在**支持度那个宽锥**里。 */
struct Candidate {
    ORB_SLAM3::MapPoint* mp = nullptr;
    float u = 0, v = 0;  // 特征点的图像像素
    float dpx = 0;       // 到十字的像素距离
    Eigen::Vector3f pw;  // 世界坐标
    Eigen::Vector3f pc;  // 相机坐标系
};

/** 收集宽锥里**所有**可用候选 —— 不只是最近那个，支持度要数个数。 */
void gatherCandidates(const std::vector<cv::KeyPoint>& kps,
                      const std::vector<ORB_SLAM3::MapPoint*>& mps, const Eigen::Matrix4f& Tcw,
                      const Center& c, float radiusPx, std::vector<Candidate>* out) {
    // ⚠️ 两个数组长度不等就会越界，而且是**静默**的（读到的是错位的点，不是崩）。
    const size_t n = std::min(kps.size(), mps.size());
    const float r2 = radiusPx * radiusPx;

    for (size_t i = 0; i < n; ++i) {
        ORB_SLAM3::MapPoint* mp = mps[i];
        if (mp == nullptr) continue;

        // ⚠️ **先按像素筛，再查 `isBad()`**，顺序不能反。`isBad()` 要拿两把锁，
        // 而 LocalMapping / LoopClosing 也在抢 —— 500 个点逐个查，**尾延迟**会难看。
        const float dx = kps[i].pt.x - c.u;
        const float dy = kps[i].pt.y - c.v;
        const float d2 = dx * dx + dy * dy;
        if (d2 > r2) continue;
        if (mp->isBad()) continue;

        const Eigen::Vector3f pw = mp->GetWorldPos();
        const Eigen::Vector3f pc = (Tcw * pw.homogeneous()).head<3>();
        if (pc.z() < kMinZ) continue;

        out->push_back({mp, kps[i].pt.x, kps[i].pt.y, std::sqrt(d2), pw, pc});
    }
}

/** 它偏离光轴多少弧度。**不需要内参** —— 所以没被匹配的帧也能算。 */
inline float axisAngle(const Eigen::Vector3f& pc) {
    return std::atan2(std::sqrt(pc.x() * pc.x() + pc.y() * pc.y()), pc.z());
}

/** 深度在 `z ± kSupportDepthFrac·z` 以内的候选有几个 —— 也就是有多少个点「陪着它」。 */
int countSupport(const std::vector<Candidate>& cand, float z) {
    const float tol = z * kSupportDepthFrac;
    int n = 0;
    for (const Candidate& k : cand) {
        if (std::fabs(k.pc.z() - z) <= tol) ++n;
    }
    return n;
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

CenterMeasure centerMeasure(ORB_SLAM3::System* sys, int w, int h, const float t_cw[16],
                            CenterMeasureState* st) {
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

    // 相机在世界系里的位置 = −Rᵀ·t。深度锁定要拿它算「帧间位移的上限」。
    const Eigen::Vector3f cam = -Tcw.block<3, 3>(0, 0).transpose() * Tcw.block<3, 1>(0, 3);

    std::vector<Candidate> cand;
    cand.reserve(32);
    gatherCandidates(kps, mps, Tcw, c, kSupportConeFrac * fw, &cand);

    out.candidates = static_cast<int>(cand.size());
    if (cand.size() > 1) {
        float zmin = cand[0].pc.z(), zmax = zmin;
        for (const Candidate& k : cand) {
            zmin = std::min(zmin, k.pc.z());
            zmax = std::max(zmax, k.pc.z());
        }
        out.zSpan = zmin > 0 ? (zmax - zmin) / zmin : 0.0f;
    }

    // ================= ① 光轴锁定：还指着那个点吗 =================
    bool have = false;
    Eigen::Vector3f pc;
    ORB_SLAM3::MapPoint* mp = nullptr;
    if (st != nullptr && st->locked != nullptr && !st->locked->isBad()) {
        const Eigen::Vector3f q =
            (Tcw * st->locked->GetWorldPos().homogeneous()).head<3>();
        if (q.z() > kMinZ && axisAngle(q) <= kLockAngleRad) {
            have = true;
            pc = q;
            mp = st->locked;
            out.source = 2;
        }
    }

    // ================= ② 深度锁定：这个读数该不该这么变 =================
    if (have) {
        const float dx = cam.x() - st->lastCam[0];
        const float dy = cam.y() - st->lastCam[1];
        const float dz = cam.z() - st->lastCam[2];
        const float limit = std::max(kDepthJumpMin, kDepthJumpGain * std::sqrt(dx * dx + dy * dy + dz * dz));
        if (std::fabs(pc.norm() - st->lastD) > limit) {
            st->locked = nullptr;
            st->missFrames = 0;
            return out;
        }
    }
    // 连续太久没被匹配 ⟹ 它多半已经不在视野里（物体走了 / 被挡住）。
    // 30 帧足够长，不会误伤「闪一下」的瞬时未匹配 —— 那正是锁定要救的。
    if (have) {
        bool matched = false;
        for (const Candidate& k : cand) {
            if (k.mp == mp) { matched = true; break; }
        }
        st->missFrames = matched ? 0 : st->missFrames + 1;
        if (st->missFrames >= kLockMaxMiss) {
            have = false;
            st->locked = nullptr;
            st->missFrames = 0;
        }
    }

    // ================= ③ 没锁住 → 选新目标，但要够「支持」 =================
    // 支持度问的是「这个点**现在**周围有没有一簇点陪着它」—— 孤立的坏点没有。
    if (!have) {
        const Candidate* anchor = nullptr;
        for (const Candidate& k : cand) {
            if (k.dpx > kSnapFrac * fw) continue;  // 锚点只要「正好指着」的
            if (anchor == nullptr || k.dpx < anchor->dpx) anchor = &k;
        }
        if (anchor != nullptr) {
            const int support = countSupport(cand, anchor->pc.z());
            out.support = support;
            if (support >= kMinSupport) {
                pc = anchor->pc;
                mp = anchor->mp;
                have = true;
                out.source = 1;
            }
            // 不够支持 ⟹ 不信它，留给 ⑤ 降级平面法
        }
    }

    // ================= ④ 出结果 =================
    if (have) {
        out.method = 1;
        out.distance = pc.norm();  // 斜距，和激光测距仪报的是同一种量
        const Eigen::Vector3f pw = mp->GetWorldPos();
        out.wx = pw.x();
        out.wy = pw.y();
        out.wz = pw.z();
        out.support = countSupport(cand, pc.z());  // 沿用时也报，日志里好/坏才可比

        out.quality = axisAngle(pc) <= kGoodFrac * kSnapAngleRad ? kGood : kFair;
        // 同一帧里就有前后两层 ⟹ 读数本来就歧义，降档，不给一个自信的错误数字。
        if (out.zSpan > kAmbiguousSpan) out.quality = kFair;

        // 像素：这一帧匹配到就用真实的，否则沿用上一次的（转动时会变旧，见遗留）。
        float u = -1, v = -1;
        for (const Candidate& k : cand) {
            if (k.mp == mp) { u = k.u; v = k.v; break; }
        }
        if (u < 0 && st != nullptr) { u = st->px; v = st->py; }
        out.px = u;
        out.py = v;

        if (st != nullptr) {
            st->locked = mp;
            st->px = u;
            st->py = v;
            st->lastD = out.distance;  // 给下一帧的深度锁定当基准
            st->lastCam[0] = cam.x();
            st->lastCam[1] = cam.y();
            st->lastCam[2] = cam.z();
            st->missFrames = 0;
        }
        return out;
    }

    // ================= ⑤ 平面兜底 =================
    // 走到这儿 = 没锁住，且（没候选 / anchor 不够支持 / 锁失效）。
    // ⚠️ **平面结果不锁定**：它本来就是「地图点不可信」时的降级，没有身份；
    // 锁住等于把一个已知不太准的值粘住，而一旦有够支持的点出现又要切回来 —— 那才是跳变。
    // ⚠️ 也**不更新 `lastD` / `lastCam`** —— 它和「到某个地图点的距离」不是同一个量，
    // 拿它当基准比下一帧会立刻误判成跳变。
    if (st != nullptr) {
        st->locked = nullptr;
        st->missFrames = 0;
    }
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
