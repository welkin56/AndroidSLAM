// 见 measure_center.h。两条路径：最近的 MapPoint（主力），局部平面（兜底）。
#include "measure_center.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "MapPoint.h"
#include "Settings.h"
#include "System.h"

namespace slam {
namespace {

// ORB_SLAM3 eTrackingState::OK。
constexpr int kTrackingOk = 2;

// 入环筛和平面兜底的半径，均按图像宽度取比例。
constexpr float kSupportConeFrac = 0.047f;  // 入环筛约 30 px
constexpr float kConeFrac = 0.15f;          // 平面兜底邻域约 96 px

/** 锚点优先半径，20 px 内才算「正好指着」；找不到则退到整个入环锥并降一档质量。 */
constexpr float kSnapRadiusPx = 20.0f;

/** 角度小于该比例算「好」，否则算「勉强」。 */
constexpr float kGoodFrac = 0.4f;

/**
 * 两根锥用弧度：环里的点是几十帧前入的，没有当帧像素位置，只能按角度判断。
 * 数值由像素半径除以 fx（435.2）得到，不能除以图像宽度。
 */
constexpr float kSupportConeRad = 0.0689f;  // 约 30 px

/** 解锁角度（约 30 px）。 */
constexpr float kLockAngleRad = 0.0689f;    // 约 30 px

/** 深度差在此比例内算支持锚点。 */
constexpr float kSupportDepthFrac = 0.20f;

/** 至少这么多条目才算够支持。环不去重，改 kSeenCap 时需要重定此值。 */
constexpr int kMinSupport = 10;

/** 距离的帧间跳变超过 max(kDepthJumpMin, kDepthJumpGain * 相机位移) 时，判定锁定点失效。 */
constexpr float kDepthJumpGain = 3.0f;
constexpr float kDepthJumpMin = 0.10f;

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
 * 十字对应的去畸变像素坐标。
 * 当前畸变系数全为 0，所以就是图像中心；标定出非零畸变后需在此处做去畸变。
 */
inline Center centerPx(int w, int h) { return {w * 0.5f, h * 0.5f}; }

/** 一个候选：特征点匹配到地图点，且落在支持度宽锥内。 */
struct Candidate {
    ORB_SLAM3::MapPoint* mp = nullptr;
    float u = 0, v = 0;  // 特征点的图像像素
    float dpx = 0;       // 到十字的像素距离
    Eigen::Vector3f pw;  // 世界坐标
    Eigen::Vector3f pc;  // 相机坐标系
};

/** 收集宽锥内所有可用候选；支持度需要计数，不能只取最近的。 */
void gatherCandidates(const std::vector<cv::KeyPoint>& kps,
                      const std::vector<ORB_SLAM3::MapPoint*>& mps, const Eigen::Matrix4f& Tcw,
                      const Center& c, float radiusPx, std::vector<Candidate>* out) {
    // 两个数组长度取小，避免越界。
    const size_t n = std::min(kps.size(), mps.size());
    const float r2 = radiusPx * radiusPx;

    for (size_t i = 0; i < n; ++i) {
        ORB_SLAM3::MapPoint* mp = mps[i];
        if (mp == nullptr) continue;

        // 先按像素筛，再查 isBad()。后者要拿锁，和 LocalMapping 竞争，尽量少查。
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

/** 偏离光轴的弧度，不需要内参，因此未匹配的帧也能计算。 */
inline float axisAngle(const Eigen::Vector3f& pc) {
    return std::atan2(std::sqrt(pc.x() * pc.x() + pc.y() * pc.y()), pc.z());
}

/**
 * 用局部地图点拟合平面，再取十字处沿光轴的深度。
 *
 * 相机系平面 n*X = d 代入 X = Z*(x, y, 1) 后，1/Z 在像素坐标上是仿射的，
 * 所以直接对 1/Z 做最小二乘即可，不需要内参。
 * 先做一次全量拟合，剔除相对误差大的点后再拟合一次。
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

        // 同 gatherCandidates：先按像素筛，再查 isBad()。
        const float dx = kps[i].pt.x - c.u;
        const float dy = kps[i].pt.y - c.v;
        if (dx * dx + dy * dy > coneRadiusPx * coneRadiusPx) continue;
        if (mp->isBad()) continue;

        // GetWorldPos() 内部加锁。
        const float z = (Tcw * mp->GetWorldPos().homogeneous()).z();
        if (z < kMinZ) continue;

        uvz.emplace_back(kps[i].pt.x, kps[i].pt.y, 1.0f / z);
    }
    if (uvz.size() < kMinPlanePoints) return false;

    // 解 [u v 1] * (a,b,c) = 1/Z 的正规方程；点退化时返回 false。
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
        if (inv <= 1e-6f) continue;  // 拟合出负深度，是外点
        // 相对深度误差；在 1/Z 上计算比在 Z 上更稳定。
        if (std::fabs(inv - p.z()) / inv > kPlaneRelTol) continue;
        inliers.push_back(p);
    }

    // 内点不过半，邻域里多半有两张面，混合平面的值不可用。
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
    if (sys == nullptr || w <= 0 || h <= 0 || st == nullptr) return out;

    // 只有跟踪 OK 的帧才可测；其他状态返回 kNone，但调用方仍需发布，避免界面保留旧值。
    if (sys->GetTrackingState() != kTrackingOk) return out;

    // 两个接口都按值返回拷贝，内部使用同一把锁，任意线程可调。
    // kps 是本帧全部特征点，不只是匹配到地图点的那些。
    const std::vector<cv::KeyPoint> kps = sys->GetTrackedKeyPointsUn();
    const std::vector<ORB_SLAM3::MapPoint*> mps = sys->GetTrackedMapPoints();
    if (kps.empty() || mps.empty()) return out;

    Eigen::Matrix4f Tcw;
    std::memcpy(Tcw.data(), t_cw, sizeof(float) * 16);

    const Center c = centerPx(w, h);
    const float coneR = kSupportConeFrac * static_cast<float>(w);

    // 相机世界坐标 = -R^T * t，用于计算帧间位移上限。
    const Eigen::Vector3f cam = -Tcw.block<3, 3>(0, 0).transpose() * Tcw.block<3, 1>(0, 3);

    // ---- 纹理密度：宽锥内的特征点总数
    {
        const float r2 = coneR * coneR;
        int tex = 0;
        for (const cv::KeyPoint& kp : kps) {
            const float dx = kp.pt.x - c.u, dy = kp.pt.y - c.v;
            if (dx * dx + dy * dy <= r2) ++tex;
        }
        out.texture = tex;
    }

    // ---- 入环：当帧宽锥内的候选全部追加，不去重 ----
    std::vector<Candidate> cand;
    cand.reserve(32);
    gatherCandidates(kps, mps, Tcw, c, coneR, &cand);
    out.candidates = static_cast<int>(cand.size());
    if (cand.size() > 1) {
        float zmin = cand[0].pc.z(), zmax = zmin;
        for (const Candidate& k : cand) {
            zmin = std::min(zmin, k.pc.z());
            zmax = std::max(zmax, k.pc.z());
        }
        out.zSpan = zmin > 0 ? (zmax - zmin) / zmin : 0.0f;
    }

    for (const Candidate& k : cand) {
        // 已在环里的同一点顺手刷新世界坐标；BA 会持续调整地图点。
        for (int i = 0; i < kSeenCap; ++i) {
            if (st->seen[i].mp != k.mp) continue;
            st->seen[i].pw[0] = k.pw.x();
            st->seen[i].pw[1] = k.pw.y();
            st->seen[i].pw[2] = k.pw.z();
        }
        // 总是新增一条，满了覆盖最旧的；老化由容量决定。
        CenterMeasureState::Seen& s = st->seen[st->seenHead];
        s.mp = k.mp;
        s.pw[0] = k.pw.x();
        s.pw[1] = k.pw.y();
        s.pw[2] = k.pw.z();
        st->seenHead = (st->seenHead + 1) % kSeenCap;
    }

    // 用当帧位姿现算环里各点的相机系坐标。
    // 返回类型必须显式写：auto 会推成引用临时对象的 Eigen::Block 表达式。
    auto pcOf = [&](const float pw[3]) -> Eigen::Vector3f {
        return (Tcw * Eigen::Vector3f(pw[0], pw[1], pw[2]).homogeneous()).head<3>();
    };

    bool have = false;
    Eigen::Vector3f pc;
    ORB_SLAM3::MapPoint* mp = nullptr;

    // ============ ① 光轴锁定：是否仍指向同一个点 ============
    // 不要求这一帧匹配到它；世界坐标在环里，位姿是当帧的，仍能计算角度。
    if (st->locked != nullptr) {
        const float* pw = nullptr;
        for (int i = 0; i < kSeenCap; ++i) {
            if (st->seen[i].mp == st->locked) { pw = st->seen[i].pw; break; }
        }
        // 环里找不到说明已被挤出，自动解锁。
        if (pw != nullptr && !st->locked->isBad()) {
            const Eigen::Vector3f q = pcOf(pw);
            if (q.z() > kMinZ && axisAngle(q) <= kLockAngleRad) {
                have = true;
                pc = q;
                mp = st->locked;
                out.source = 2;
            }
        }
    }

    // ============ ② 深度锁定：检查读数变化是否合理 ============
    // 世界点固定，距离只取决于相机位姿，正常应随位移平滑变化。
    if (have) {
        const float dx = cam.x() - st->lastCam[0];
        const float dy = cam.y() - st->lastCam[1];
        const float dz = cam.z() - st->lastCam[2];
        const float limit =
            std::max(kDepthJumpMin, kDepthJumpGain * std::sqrt(dx * dx + dy * dy + dz * dz));
        if (std::fabs(pc.norm() - st->lastD) > limit) {
            st->locked = nullptr;
            // 跳变过大时直接不给数，也不走平面兜底，避免产生另一个跳变。
            return out;
        }
    }

    // 统计环里够支持的条目数。先按角度和深度筛，只对通过者查 isBad()。
    auto countSupport = [&](const Eigen::Vector3f& apc) {
        const float tol = apc.z() * kSupportDepthFrac;
        int n = 0;
        for (int i = 0; i < kSeenCap; ++i) {
            const CenterMeasureState::Seen& s = st->seen[i];
            if (s.mp == nullptr) continue;
            const Eigen::Vector3f q = pcOf(s.pw);
            if (q.z() <= kMinZ) continue;
            if (axisAngle(q) > kSupportConeRad) continue;    // 现在还在不在宽锥里
            if (std::fabs(q.z() - apc.z()) > tol) continue;  // 深度接近
            if (s.mp->isBad()) continue;                     // 到这一步才付锁
            ++n;
        }
        return n;
    };

    // ============ ③ 没锁住时，从当帧候选里选锚点 ============
    // 用像素距离而不是角度：候选带当帧真实像素位置，更准。
    bool snappedFar = false;  // 退到远处才找到，质量降一档
    if (!have) {
        const Candidate* anchor = nullptr;
        for (const Candidate& k : cand) {
            if (k.dpx > kSnapRadiusPx) continue;  // 先只找「正好指着」的
            if (anchor == nullptr || k.dpx < anchor->dpx) anchor = &k;
        }
        if (anchor == nullptr && !cand.empty()) {
            // 20 px 内没有候选，退到整个入环锥。
            snappedFar = true;
            for (const Candidate& k : cand) {
                if (anchor == nullptr || k.dpx < anchor->dpx) anchor = &k;
            }
        }

        if (anchor != nullptr) {
            const int support = countSupport(anchor->pc);
            out.support = support;
            if (support >= kMinSupport) {
                pc = anchor->pc;
                mp = anchor->mp;
                have = true;
                out.source = 1;
            }
            // 支持不够，留给平面兜底。
        }
    }

    // ============ ④ 输出结果 ============
    if (have) {
        out.method = 1;
        out.mp = mp;  // 折线靠它识别身份，世界坐标会变
        out.distance = pc.norm();  // 斜距
        // 用最新的世界坐标，环里的 pw 可能已过期。
        const Eigen::Vector3f pw = mp->GetWorldPos();
        out.wx = pw.x();
        out.wy = pw.y();
        out.wz = pw.z();
        if (out.source == 2) out.support = countSupport(pc);  // 沿用时也记录支持度

        // 按角度分档；沿用时没有当帧像素位置，只能用角度。
        out.quality = axisAngle(pc) <= kGoodFrac * kSupportConeRad ? kGood : kFair;
        // 当帧有前后两层时读数有歧义，降档。
        if (out.zSpan > kAmbiguousSpan) out.quality = kFair;
        // 退到远处才找到的，一律不算「好」。
        if (snappedFar) out.quality = kFair;

        // 本帧匹配到就用真实像素，否则沿用上一次的。
        float u = -1, v = -1;
        for (const Candidate& k : cand) {
            if (k.mp == mp) { u = k.u; v = k.v; break; }
        }
        if (u < 0) { u = st->lockedPx; v = st->lockedPy; }
        out.px = u;
        out.py = v;

        st->locked = mp;
        st->lockedPx = u;
        st->lockedPy = v;
        st->lastD = out.distance;  // 下一帧深度锁定的基准
        st->lastCam[0] = cam.x();
        st->lastCam[1] = cam.y();
        st->lastCam[2] = cam.z();
        return out;
    }

    // ============ ⑤ 平面兜底 ============
    // 平面结果没有身份，不锁定；也不更新 lastD/lastCam，因为量纲不同。
    st->locked = nullptr;
    float z = 0;
    if (planeDepthAt(kps, mps, Tcw, c, kConeFrac * static_cast<float>(w), &z)) {
        out.quality = kFair;  // 兜底结果一律不算「好」
        out.method = 2;
        out.distance = z;
        // 十字等于主点（见 centerPx），相机系视线方向为 (0,0,1)；改 centerPx 时需同步修改。
        const Eigen::Vector3f pw = Tcw.block<3, 3>(0, 0).transpose() * Eigen::Vector3f(0.f, 0.f, z) + cam;
        out.wx = pw.x();
        out.wy = pw.y();
        out.wz = pw.z();
        out.px = c.u;
        out.py = c.v;
    }
    return out;
}

// ---------------- 投影 ----------------

namespace {
/** 由 initIntrinsics 填充；0 表示未读到。 */
float gFx = 0, gFy = 0, gCx = 0, gCy = 0;
}  // namespace

void initIntrinsics(const std::string& settingsPath) {
    if (gFx != 0) return;

    printf("[M6] initIntrinsics: 开始读内参，settings=%s\n", settingsPath.c_str());
    fflush(stdout);

    // 走 Settings 的公开接口；Tracking::mK 是 protected，System 不提供 K。
    // 真值仍来自 phone.yaml，不在代码里抄常量。
    ORB_SLAM3::Settings s(settingsPath, static_cast<int>(ORB_SLAM3::System::IMU_MONOCULAR));
    ORB_SLAM3::GeometricCamera* cam = s.camera1();
    if (cam == nullptr) {
        printf("[M6] ⚠️ initIntrinsics 失败：camera1() 是空的 ⟹ 多点折线画不出来\n");
        fflush(stdout);
        return;
    }

    // 0..3 = fx, fy, cx, cy。
    gFx = cam->getParameter(0);
    gFy = cam->getParameter(1);
    gCx = cam->getParameter(2);
    gCy = cam->getParameter(3);

    printf("[M6] 投影内参 fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n", gFx, gFy, gCx, gCy);
    fflush(stdout);
}

bool cameraIntrinsics(float* fx, float* fy, float* cx, float* cy) {
    if (gFx == 0) return false;
    *fx = gFx;
    *fy = gFy;
    *cx = gCx;
    *cy = gCy;
    return true;
}

}  // namespace slam
