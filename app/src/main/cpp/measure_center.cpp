// 见 measure_center.h。两条路径：最近的 MapPoint（主力），局部平面（兜底）。
//
// 上游一个字都不用改 —— 包括路径 2 用的内参：相机坐标系里的平面，它的 `1/Z`
// 在**像素坐标**上是仿射的，所以最小二乘不需要 K。推导见 planeDepthAt。
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

// ORB_SLAM3 `eTrackingState::OK`（Tracking.h:121）。
constexpr int kTrackingOk = 2;

// 入环筛 / 平面兜底的半径：**占图像宽度**的比例，直接乘 w。
constexpr float kSupportConeFrac = 0.047f;  // 入环筛 ≈ 30 px
constexpr float kConeFrac = 0.15f;          // 平面兜底的邻域 ≈ 96 px

/**
 * 锚点的**优先**半径 —— 这么近才算「正好指着」。
 *
 * ⚠️ 找不到就**退到整个入环锥（30 px）**，并把质量降一档。
 * 不这么做的话，20 px 内恰好没图点时**一个数都不会有**（实测栽过）。
 */
constexpr float kSnapRadiusPx = 20.0f;

/** 角度够小才算「好」，否则只能算「勉强」。按角度比，所以是个比值。 */
constexpr float kGoodFrac = 0.4f;

/**
 * 下面两根锥用**弧度**：环里的点是几十帧前入的，**没有当帧像素位置**，只能按角度判。
 *
 * 数值是「像素半径 ÷ fx」来的（fx = 435.2，本机 `phone.yaml` 的 `Camera1.fx`）：
 *     30 px → 0.0689 rad
 * ⚠️ 要改就照这么除，**不能除 `w`** —— `w/fx` 差 1.47 倍（栽过）。
 */
constexpr float kSupportConeRad = 0.0689f;  // ≈30 px

/** 解锁的角度（≈30 px）。锁定点同样没有当帧像素位置，也只能用角度。 */
constexpr float kLockAngleRad = 0.0689f;    // ≈30 px

/** 深度差在这个比例以内，算「陪着 anchor」。 */
constexpr float kSupportDepthFrac = 0.20f;

/**
 * 至少这么多**条目**够支持才采纳。
 *
 * ⚠️ 环**不去重**，所以这个数是「每帧符合几条 × 环装得下几帧」——
 * 静止时好面 ~80、一闪而过的坏点 ~1。**改 `kSeenCap` 必须重定这个数。**
 */
constexpr int kMinSupport = 10;

/** 距离的帧间跳变 > `max(kDepthJumpMin, kDepthJumpGain × 相机位移)` ⟹ 锁定点出问题了。 */
constexpr float kDepthJumpGain = 3.0f;
constexpr float kDepthJumpMin = 0.10f;

/** 半径内深度跨度超过这个比例 = 同时有前后两层，读数只能算「勉强」。 */
constexpr float kAmbiguousSpan = 0.3f;

/** 平面拟合的内点判据：相对深度误差超过这个比例就剔掉。 */
constexpr float kPlaneRelTol = 0.05f;
constexpr size_t kMinPlanePoints = 5;

/** 相机后方的点不算数。 */
constexpr float kMinZ = 0.05f;

/** 投影出来的像素夹在这个范围里 —— 相机后方的点会算出天文数字，没必要真带出去。 */
constexpr float kProjClamp = 20000.f;

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
    if (sys == nullptr || w <= 0 || h <= 0 || st == nullptr) return out;

    // 只有跟踪 OK 的帧才谈得上「这里有多远」。其他状态返回 kNone —— 调用方照样要发布，
    // 否则 UI 会一直留着上一帧的数字。
    if (sys->GetTrackingState() != kTrackingOk) return out;

    // 两个都是**按值返回的拷贝**，而且和写入方锁同一个 `mMutexState`
    // （写 System.cc:321-323，读 :1327 / :1333）—— 所以任意线程都能调。
    // ⚠️ `kps` 是**这一帧全部特征点**（本机 1200 个），不只是匹配上地图点的那些。
    const std::vector<cv::KeyPoint> kps = sys->GetTrackedKeyPointsUn();
    const std::vector<ORB_SLAM3::MapPoint*> mps = sys->GetTrackedMapPoints();
    if (kps.empty() || mps.empty()) return out;

    Eigen::Matrix4f Tcw;
    std::memcpy(Tcw.data(), t_cw, sizeof(float) * 16);

    const Center c = centerPx(w, h);
    const float coneR = kSupportConeFrac * static_cast<float>(w);

    // 相机在世界系里的位置 = −Rᵀ·t。深度锁定要拿它算「帧间位移的上限」。
    const Eigen::Vector3f cam = -Tcw.block<3, 3>(0, 0).transpose() * Tcw.block<3, 1>(0, 3);

    // ---- 纹理密度：宽锥内的**特征点**总数
    {
        const float r2 = coneR * coneR;
        int tex = 0;
        for (const cv::KeyPoint& kp : kps) {
            const float dx = kp.pt.x - c.u, dy = kp.pt.y - c.v;
            if (dx * dx + dy * dy <= r2) ++tex;
        }
        out.texture = tex;
    }

    // ---- 入环：当帧宽锥内的候选**全部追加**（不去重）----
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
        // 已在环里的同 mp 条目：**顺手刷新世界坐标** —— BA 一直在动它，
        // 而入环只发生一次。这一步让 `pw` 只要「还被匹配上」就是新的。
        for (int i = 0; i < kSeenCap; ++i) {
            if (st->seen[i].mp != k.mp) continue;
            st->seen[i].pw[0] = k.pw.x();
            st->seen[i].pw[1] = k.pw.y();
            st->seen[i].pw[2] = k.pw.z();
        }
        // **总是新增一条**（不去重）；满了覆盖最旧的 —— 老化完全靠容量，没有帧号。
        CenterMeasureState::Seen& s = st->seen[st->seenHead];
        s.mp = k.mp;
        s.pw[0] = k.pw.x();
        s.pw[1] = k.pw.y();
        s.pw[2] = k.pw.z();
        st->seenHead = (st->seenHead + 1) % kSeenCap;
    }

    // 环里的点用**当帧位姿**现算相机系坐标 —— 角度和深度都从这里来。
    // ⚠️ **返回类型必须显式写**：`auto` 会推成 `Eigen::Block` 表达式，它引用着
    //    `homogeneous()` 出来的临时对象 —— 临时在 return 时销毁，读到的就是垃圾。
    auto pcOf = [&](const float pw[3]) -> Eigen::Vector3f {
        return (Tcw * Eigen::Vector3f(pw[0], pw[1], pw[2]).homogeneous()).head<3>();
    };

    bool have = false;
    Eigen::Vector3f pc;
    ORB_SLAM3::MapPoint* mp = nullptr;

    // ============ ① 光轴锁定：还指着那个点吗 ============
    // ⚠️ **不要求它这一帧被匹配** —— 世界坐标在环里、位姿是当帧的，角度照样算得出来。
    if (st->locked != nullptr) {
        const float* pw = nullptr;
        for (int i = 0; i < kSeenCap; ++i) {
            if (st->seen[i].mp == st->locked) { pw = st->seen[i].pw; break; }
        }
        // 环里找不到 = 已被容量挤出去 ⟹ **自动解锁**（这就是「多久没见就忘掉」，不用数帧）
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

    // ============ ② 深度锁定：这个读数该不该这么变 ============
    // 世界点固定 ⟹ 距离**只是相机位姿的函数**，正常只能随相机位移平滑变化。
    if (have) {
        const float dx = cam.x() - st->lastCam[0];
        const float dy = cam.y() - st->lastCam[1];
        const float dz = cam.z() - st->lastCam[2];
        const float limit =
            std::max(kDepthJumpMin, kDepthJumpGain * std::sqrt(dx * dx + dy * dy + dz * dz));
        if (std::fabs(pc.norm() - st->lastD) > limit) {
            st->locked = nullptr;
            // ⚠️ **直接不给数**，不走平面兜底 —— 兜底会给一个不同的数，看着更像跳变。
            return out;
        }
    }

    // 数「环里够支持的**条目数**」。⚠️ 先按角度和深度筛，**只对通过的那几条查 `isBad()`** ——
    // 逐条查就是 256 次双锁/帧，那正是当初放弃「遍历全图」的原因。
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

    // ============ ③ 没锁住 → 从**当帧候选**里选 anchor ============
    // 用**像素距离**（不是角度）—— 候选带当帧的真实像素位置，比角度准。
    // ⚠️ 别改成「从环里选」：环里的点没有当帧像素位置、只能用角度，
    //    而角度阈比入环阈窄一半 ⟹「30px 内有点、15px 内没有」会一个都选不出。
    bool snappedFar = false;  // 退了才找到的 ⟹ 质量降一档
    if (!have) {
        const Candidate* anchor = nullptr;
        for (const Candidate& k : cand) {
            if (k.dpx > kSnapRadiusPx) continue;  // 先只要「正好指着」的
            if (anchor == nullptr || k.dpx < anchor->dpx) anchor = &k;
        }
        if (anchor == nullptr && !cand.empty()) {
            // 「正好指着」范围内一个都没有 → 退到整个入环锥。
            // 宁可给一个不太准的数，也别「明明指着有东西的地方却什么都没有」。
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
            // 不够支持 ⟹ 不信这个点，留给 ⑤ 降级平面法
        }
    }

    // ============ ④ 出结果 ============
    if (have) {
        out.method = 1;
        out.mp = mp;  // 折线靠它认住身份（世界坐标会变，见 header）
        out.distance = pc.norm();  // 斜距，和激光测距仪报的是同一种量
        // ⚠️ 用 `GetWorldPos()` 而不是环里的 `pw` —— 前者是最新的（BA 一直在动点）。
        const Eigen::Vector3f pw = mp->GetWorldPos();
        out.wx = pw.x();
        out.wy = pw.y();
        out.wz = pw.z();
        if (out.source == 2) out.support = countSupport(pc);  // 沿用时也报，日志里好/坏才可比

        // 按角度分档（沿用时锁定点没有当帧像素位置，只能用角度 —— 和锚点选像素不同）。
        // `kGoodFrac = 0.4` ⟹ 「好」= 12 px 以内。
        out.quality = axisAngle(pc) <= kGoodFrac * kSupportConeRad ? kGood : kFair;
        // 同一帧里就有前后两层 ⟹ 读数本来就歧义，降档，不给一个自信的错误数字。
        if (out.zSpan > kAmbiguousSpan) out.quality = kFair;
        // 「正好指着」范围内没找到、退到远处才找到的 —— 一律不当「好」。
        if (snappedFar) out.quality = kFair;

        // 像素：这一帧匹配到就用真实的，否则沿用上一次的
        // （环里的点常常这一帧没被匹配 —— 那正是锁定要救的情况）。
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
        st->lastD = out.distance;  // 给下一帧的深度锁定当基准
        st->lastCam[0] = cam.x();
        st->lastCam[1] = cam.y();
        st->lastCam[2] = cam.z();
        return out;
    }

    // ============ ⑤ 平面兜底 ============
    // ⚠️ **平面结果不锁定**：它本来就是「地图点不可信」时的降级，没有身份；
    // 锁住等于把一个已知不太准的值粘住，而一旦有够支持的点出现又要切回来 —— 那才是跳变。
    // ⚠️ 也**不更新 `lastD` / `lastCam`** —— 它和「到某个地图点的距离」不是同一个量。
    st->locked = nullptr;
    float z = 0;
    if (planeDepthAt(kps, mps, Tcw, c, kConeFrac * static_cast<float>(w), &z)) {
        out.quality = kFair;  // 兜底出来的，一律不当「好」
        out.method = 2;
        out.distance = z;
        // 世界坐标**能给**：十字恒等于主点（见 centerPx），那条视线在相机系里恰好是 (0,0,1)。
        // ⚠️ 和 centerPx 同一个假设，改那个要一起改。
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
/** [initIntrinsics] 填一次。0 = 没有内参，[projectWorld] 全返回 -1。 */
float gFx = 0, gFy = 0, gCx = 0, gCy = 0;
}  // namespace

void initIntrinsics(const std::string& settingsPath) {
    if (gFx != 0) return;

    printf("[M6] initIntrinsics: 开始读内参，settings=%s\n", settingsPath.c_str());
    fflush(stdout);

    // ⚠️ 走 `Settings` 的公开接口 —— `Tracking::mK` 是 protected，`System` 不给 K。
    // 唯一真相仍是 phone.yaml，不在这儿抄常量。
    ORB_SLAM3::Settings s(settingsPath, static_cast<int>(ORB_SLAM3::System::IMU_MONOCULAR));
    ORB_SLAM3::GeometricCamera* cam = s.camera1();
    if (cam == nullptr) {
        printf("[M6] ⚠️ initIntrinsics 失败：camera1() 是空的 ⟹ 多点折线画不出来\n");
        fflush(stdout);
        return;
    }

    // 0..3 = fx, fy, cx, cy（顺序见 Tracking.cc:549-553）。
    gFx = cam->getParameter(0);
    gFy = cam->getParameter(1);
    gCx = cam->getParameter(2);
    gCy = cam->getParameter(3);

    printf("[M6] 投影内参 fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n", gFx, gFy, gCx, gCy);
    fflush(stdout);
}

int projectWorld(const float* worldXyz, int n, const float t_cw[16], float* out) {
    if (worldXyz == nullptr || out == nullptr || n <= 0) return 0;
    if (gFx == 0) return 0;

    // `t_cw` 是列主序：元素 (r,c) 在 [c*4+r]。pc = Rcw·pw + tcw。
    int hits = 0;
    for (int i = 0; i < n; ++i) {
        const float* w = worldXyz + 3 * i;
        const float x = t_cw[0] * w[0] + t_cw[4] * w[1] + t_cw[8] * w[2] + t_cw[12];
        const float y = t_cw[1] * w[0] + t_cw[5] * w[1] + t_cw[9] * w[2] + t_cw[13];
        const float z = t_cw[2] * w[0] + t_cw[6] * w[1] + t_cw[10] * w[2] + t_cw[14];

        const bool front = z >= kMinZ;
        // ⚠️ 后面的点把 z **夹成正的小量**，而不是丢掉 —— 这样它落到画面外、**朝那个方向**。
        // 转过手机时端点正好从前面穿到后面，夹出来的方向就是正确的渐近方向，
        // 于是「一端在相机后面」的线段仍然画得出来，而且是往对的方向。
        const float zc = front ? z : kMinZ;

        float* o = out + 3 * i;
        o[0] = std::min(std::max(gFx * x / zc + gCx, -kProjClamp), kProjClamp);
        o[1] = std::min(std::max(gFy * y / zc + gCy, -kProjClamp), kProjClamp);
        o[2] = front ? 1.f : 0.f;
        if (front) ++hits;
    }
    return hits;
}

}  // namespace slam
