// 见 measure_center.h。两条路径：最近的 MapPoint（主力），局部平面（兜底）。
#include "measure_center.h"

#include <algorithm>
#include <chrono>
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

/** 点–面约束的邻域半径，约 96 px。解析图固定 640x480，换分辨率要重算。 */
constexpr float kSurfRadiusPx = 96.0f;

/**
 * 点–面约束的门控。它要决定一个已有坐标改不改，所以要拦得住错的平面；
 * 但也不能严到永远过不了 —— 实测 30 px 锥内只有 2~3 个点。
 */
constexpr int kSurfMinInliers = 6;
constexpr float kSurfMaxResidual = 0.04f;   // 1/Z 域的相对残差 RMS
constexpr float kSurfMinCosInc = 0.3f;      // 入射角 < 72°

/** 平面深度和锚点自己的深度差超过这个比例，就分不清谁错了，不改。 */
constexpr float kSurfRelMax = 0.25f;

/**
 * 选层：先只保留与锚点深度相差这个比例以内的点，再拟合。
 *
 * 96 px 窗口在 2 m 处横跨 ±0.6 m，多表面窗口里全量最小二乘会拟出一个谁都不贴的
 * 「妥协平面」，随后内点剔除把所有点判成外点。必须大于 kSurfRelMax —— 选层筛得比
 * 最终门控还严时，门控永远轮不到。
 */
constexpr float kSurfLayerFrac = 0.30f;

/** 平面修正比的平滑系数，约 0.3/帧。 */
constexpr float kSurfRatioAlpha = 0.3f;

/** 面修正的统计周期。1 Hz 明细是抽样，算不出成功率，要单独累计。 */
constexpr long long kSurfStatMs = 5000;

/**
 * 是否把平面修正真正落到坐标上。false = 只算、只打日志，不改任何深度。
 *
 * 三轮真机的结论：门控能过（通过率 40~90%），但修正比自身有 ±2~8% 的散布，
 * 而中位数 0.993 说明**没有系统偏差可修** —— 用它改 p_kf 等于把平面估计的噪声
 * 灌进钉点，段长极差从 0.5% 涨到 8~23%。
 * 在拿到「开/关 + 已知长度」的对照之前，保留观测、不落盘。
 */
constexpr bool kSurfaceApply = false;

/** 内参，initIntrinsics / updateIntrinsics 填充；0 表示未读到。 */
float gFx = 0, gFy = 0, gCx = 0, gCy = 0;

long long monoMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

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

/** 一次局部平面拟合的结果。平面写成 n·X = 1，所以 1/Z 在像素上是仿射的。 */
struct PlaneFit {
    float invZ = 0;      // 目标像素处的 1/Z
    Eigen::Vector3f n;   // 平面法向（未归一化，n·X = 1 的 n）
    int window = 0;      // 窗口内可用点数，选层前
    int inliers = 0;
    int total = 0;       // 选层后参与拟合的点数，也是「内点过半」的分母
    int solved = 0;      // 1 = 第一次最小二乘解出来了；0 = 点不够或方程秩亏
    float residual = 0;  // 1/Z 域的相对残差 RMS
    float cosInc = 0;    // 入射角余弦，1 = 正对表面
};

/**
 * 在像素 (u,v) 周围拟合局部平面。
 *
 * 相机系平面 n*X = d 代入 X = Z*(x, y, 1) 后，1/Z 在像素坐标上是仿射的，
 * 所以直接对 1/Z 做最小二乘即可，不需要内参。
 * refZ > 0 时只保留与它相差 kSurfLayerFrac 以内的点，先收缩到一张面；传 0 表示不选层。
 * 先做一次全量拟合，剔除相对误差大的点后再拟合一次。
 */
bool fitPlaneAt(const std::vector<cv::KeyPoint>& kps,
                const std::vector<ORB_SLAM3::MapPoint*>& mps, const Eigen::Matrix4f& Tcw,
                float u, float v, float radiusPx, float refZ, PlaneFit* out) {
    const size_t n = std::min(kps.size(), mps.size());

    // (du, dv, 1/Z)，du/dv 是相对目标像素的偏移。
    // 用绝对像素建正规方程时法矩阵条件数约 2.6e7（u≈320、v≈240），float 的秩判据
    // 3·eps·maxpivot 会把最小特征值判成 0，FullPivLU::isInvertible() 直接 false —— 实测
    // 459 次拟合里 455 次卡在这里，内点恒为 0。平移之后条件数降到 1e3 量级。
    std::vector<Eigen::Vector3f> uvz;
    uvz.reserve(64);
    int window = 0;
    for (size_t i = 0; i < n; ++i) {
        ORB_SLAM3::MapPoint* mp = mps[i];
        if (mp == nullptr) continue;

        // 同 gatherCandidates：先按像素筛，再查 isBad()。
        const float dx = kps[i].pt.x - u;
        const float dy = kps[i].pt.y - v;
        if (dx * dx + dy * dy > radiusPx * radiusPx) continue;
        if (mp->isBad()) continue;

        // GetWorldPos() 内部加锁。
        const float z = (Tcw * mp->GetWorldPos().homogeneous()).z();
        if (z < kMinZ) continue;
        ++window;

        // 选层排在算出 z 之后：它只跟深度有关，而深度要加锁取，放在像素筛后能少拿一批锁。
        if (refZ > 0.f && std::fabs(z - refZ) > kSurfLayerFrac * refZ) continue;

        uvz.emplace_back(dx, dy, 1.0f / z);
    }
    // 两个计数都在早退之前赋值，否则拟合失败时日志只剩一句失败，看不出是密度还是选层。
    out->window = window;
    out->total = static_cast<int>(uvz.size());
    if (uvz.size() < static_cast<size_t>(kMinPlanePoints)) return false;

    // 解 [du dv 1] * (a,b,c) = 1/Z 的正规方程；点退化时返回 false。
    // 方程本身用 double：平移之后 float 也够，但稀疏窗口只有 5~8 个点时留一倍余量。
    auto fit = [](const std::vector<Eigen::Vector3f>& pts, Eigen::Vector3d* abc) {
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3f& p : pts) {
            const Eigen::Vector3d r(p.x(), p.y(), 1.0);
            A += r * r.transpose();
            b += r * p.z();
        }
        const Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
        if (!lu.isInvertible()) return false;
        *abc = lu.solve(b);
        return true;
    };

    Eigen::Vector3d abc;
    if (!fit(uvz, &abc)) return false;
    out->solved = 1;

    std::vector<Eigen::Vector3f> inliers;
    inliers.reserve(uvz.size());
    double sum2 = 0.0;
    for (const Eigen::Vector3f& p : uvz) {
        const double inv = abc.x() * p.x() + abc.y() * p.y() + abc.z();
        if (inv <= 1e-6) continue;  // 拟合出负深度，是外点
        // 相对深度误差；在 1/Z 上计算比在 Z 上更稳定。
        const double rel = std::fabs(inv - p.z()) / inv;
        if (rel > kPlaneRelTol) continue;
        inliers.push_back(p);
        sum2 += rel * rel;
    }

    // 先记下来：拟合失败时也要能看出「有几个点、几个内点」，否则诊断只剩一句失败。
    out->inliers = static_cast<int>(inliers.size());
    out->residual = inliers.empty()
                        ? 0.f
                        : static_cast<float>(std::sqrt(sum2 / static_cast<double>(inliers.size())));

    // 内点不过半，邻域里多半有两张面，混合平面的值不可用。
    if (inliers.size() < static_cast<size_t>(kMinPlanePoints) || inliers.size() * 2 < uvz.size())
        return false;
    if (!fit(inliers, &abc)) return false;

    // 相对坐标的原点就是目标像素，所以十字处的 1/Z 直接是常数项。
    const double inv = abc.z();
    if (inv <= 1e-6) return false;

    out->invZ = static_cast<float>(inv);

    // 把 1/Z = a*du + b*dv + c 还原成平面。du = u' − u：
    // 1 = a*fx*X + b*fy*Y + [a*(cx−u) + b*(cy−v) + c]*Z。
    out->n = Eigen::Vector3f(static_cast<float>(abc.x() * gFx), static_cast<float>(abc.y() * gFy),
                             static_cast<float>(abc.x() * (gCx - u) + abc.y() * (gCy - v) +
                                                 abc.z()));
    if (out->n.norm() > 1e-6f && gFx > 0.f) {
        const Eigen::Vector3f ray((u - gCx) / gFx, (v - gCy) / gFy, 1.0f);
        out->cosInc = std::fabs(out->n.dot(ray)) / (out->n.norm() * ray.norm());
    }
    return true;
}

/** 门控：只有这块邻域确实是一个正对着的平面，才允许它改一个已有的深度。 */
bool surfaceGated(const PlaneFit& f) {
    if (f.inliers < kSurfMinInliers) return false;
    if (f.residual > kSurfMaxResidual) return false;
    return f.cosInc >= kSurfMinCosInc;
}

/** 两个 tag 既当打印名，也当统计下标。 */
constexpr int kTagCross = 0;  // 十字
constexpr int kTagPin = 1;    // 钉点

inline int tagIndex(const char* tag) {
    return std::strcmp(tag, "钉点") == 0 ? kTagPin : kTagCross;
}
inline const char* tagName(int ti) { return ti == kTagPin ? "钉点" : "十字"; }

/** 面修正的分档计数，每 kSurfStatMs 打一行并清零。 */
struct SurfaceStat {
    int tries = 0;
    int pass = 0;
    int fitFail = 0;  // 拟合失败，含选层后点不够
    int gated = 0;    // 内点少 / 残差大 / 掠射
    int far = 0;      // 与锚点差太多
};
enum { kSurfPass = 0, kSurfFitFail, kSurfGated, kSurfFar };

// 只在 SLAM 线程上调（centerMeasure 与 surfaceAtPoint 同一条链），不需要加锁。
SurfaceStat gStat[2];

/**
 * 记一次门控结果。
 *
 * 1 Hz 明细是抽样，只能看出「过没过」，算不出成功率；这里逐次累计，够一个周期
 * 才打一行。门控一直不过时，这一行是唯一能说明「卡在哪一档」的东西。
 */
void noteSurface(int ti, int outcome) {
    static long long lastMs[2] = {0, 0};
    SurfaceStat& s = gStat[ti];
    ++s.tries;
    if (outcome == kSurfPass) ++s.pass;
    else if (outcome == kSurfFitFail) ++s.fitFail;
    else if (outcome == kSurfGated) ++s.gated;
    else ++s.far;

    const long long now = monoMs();
    if (now - lastMs[ti] < kSurfStatMs) return;
    lastMs[ti] = now;
    printf("[M6] 面统计[%s] 尝试%d 通过%d(%.0f%%) 拟合失败%d 门控%d 差太多%d\n", tagName(ti),
           s.tries, s.pass, 100.f * static_cast<float>(s.pass) / static_cast<float>(s.tries),
           s.fitFail, s.gated, s.far);
    fflush(stdout);
    s = SurfaceStat{};
}

/**
 * 门控明细，两个 tag 各自 1 Hz 限流。
 *
 * 门没过的时候，四个数里哪个在挡比「过没过」有用得多；拟合失败（点够但不在
 * 一个面上）也要打，否则它静默地什么也不发生，看起来和「全部通过」一样。
 * 两个 tag 共用一条限流时，调用多的那个会把另一个饿死。
 */
void logSurface(const PlaneFit& f, const char* tag, const char* why, float z, float zNew) {
    static long long lastMs[2] = {0, 0};
    const int ti = tagIndex(tag);
    const long long now = monoMs();
    if (now - lastMs[ti] < 1000) return;
    lastMs[ti] = now;
    // 窗和层要分开：「候选 12」现在有两种解释（密度不够 / 被选层筛光），一个数说不清。
    printf("[M6] 面[%s] 窗%d/层%d 内点%d 残差%.1f%% 入射%.2f 深度%.2f→%.2f %s\n", tag,
           f.window, f.total, f.inliers, f.residual * 100.f, f.cosInc, z, zNew, why);
    fflush(stdout);
}

/**
 * 沿射线把 pc 缩放到局部平面的深度，横向不动。
 *
 * 单目的单点深度沿射线有一个零空间，平面把射线截成一点，正好补掉它。
 * 缩放而不是只改 z，是因为方向必须由原射线决定；只改 z 会连方向一起改掉。
 */
bool correctDepth(const std::vector<cv::KeyPoint>& kps,
                  const std::vector<ORB_SLAM3::MapPoint*>& mps, const Eigen::Matrix4f& Tcw,
                  const char* tag, Eigen::Vector3f* pc, SurfaceDepth* st) {
    if (gFx <= 0.f || pc->z() <= kMinZ) return false;
    const int ti = tagIndex(tag);

    const float u = gFx * pc->x() / pc->z() + gCx;
    const float v = gFy * pc->y() / pc->z() + gCy;

    PlaneFit f;
    // 以锚点自己的深度选层：要修的就是它，以它为中心才能把修正量关在 ±kSurfLayerFrac 里。
    if (!fitPlaneAt(kps, mps, Tcw, u, v, kSurfRadiusPx, pc->z(), &f)) {
        // 三种失败要分开：点不够、方程秩亏、有解但内点不过半 —— 修法完全不同。
        const char* why = (static_cast<size_t>(f.total) < kMinPlanePoints) ? "点太少"
                          : !f.solved                                      ? "方程奇异"
                                                                           : "内点不足";
        logSurface(f, tag, why, pc->z(), pc->z());
        noteSurface(ti, kSurfFitFail);
        return false;
    }

    st->inliers = f.inliers;
    st->total = f.total;
    st->residual = f.residual;
    st->cosInc = f.cosInc;

    const float k = (1.0f / f.invZ) / pc->z();
    if (!surfaceGated(f)) {
        const char* why = (f.inliers < kSurfMinInliers) ? "内点少"
                          : (f.residual > kSurfMaxResidual) ? "残差大"
                                                            : "掠射";
        logSurface(f, tag, why, pc->z(), pc->z());
        noteSurface(ti, kSurfGated);
        return false;
    }
    // 差得太多说明平面锁到了别的表面上，那时分不清谁错，宁可不改。
    if (!(k > 0.f) || std::fabs(k - 1.0f) > kSurfRelMax) {
        logSurface(f, tag, "与锚点差太多", pc->z(), pc->z() * k);
        noteSurface(ti, kSurfFar);
        return false;
    }

    logSurface(f, tag, "通过", pc->z(), pc->z() * k);
    noteSurface(ti, kSurfPass);
    // 只观测模式：比值已经打进日志，坐标一律不动。
    if (!kSurfaceApply) return false;
    *pc *= k;
    st->ok = 1;
    st->ratio = k;
    return true;
}

/** 十字处的平面深度，给无锚点时的兜底用。 */
bool planeDepthAt(const std::vector<cv::KeyPoint>& kps,
                  const std::vector<ORB_SLAM3::MapPoint*>& mps, const Eigen::Matrix4f& Tcw,
                  const Center& c, float coneRadiusPx, float* outZ) {
    PlaneFit f;
    // 兜底没有锚点可参照，refZ 传 0 表示不选层，行为与加选层之前一致。
    if (!fitPlaneAt(kps, mps, Tcw, c.u, c.v, coneRadiusPx, 0.f, &f)) return false;
    *outZ = 1.0f / f.invZ;
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
            // 解锁就把修正比放掉，重新锁定时从 1 起，避免套用旧锚点的修正。
            st->ratioMp = nullptr;
            st->ratioEma = 1.0f;
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

        // 点–面约束：只沿射线改深度，横向（像素）不动。
        // 锁定和跳变判据继续用未修正的 pc，那套状态机不受这里影响。
        // correctDepth 会就地乘上未平滑的 k，所以用草稿接，值本身丢弃。
        Eigen::Vector3f pcFit = pc;
        SurfaceDepth sd;
        const bool corrected = correctDepth(kps, mps, Tcw, "十字", &pcFit, &sd);

        // 修正比按锚点平滑：单帧平面深度会抖，硬套上去读数跟着抖。
        // 换锚点从 1 起，避免把上一点的修正量带到新点上。
        if (st->ratioMp != mp) {
            st->ratioMp = mp;
            st->ratioEma = 1.0f;
        }
        // 门控没过时目标回到 1：没有本帧证据就慢慢退回未修正，而不是把旧修正一直供着。
        const float target = corrected ? sd.ratio : 1.0f;
        st->ratioEma += kSurfRatioAlpha * (target - st->ratioEma);

        const Eigen::Vector3f pcOut = pc * st->ratioEma;
        out.distance = pcOut.norm();  // 斜距
        // 和 distance 必须同源，否则界面上的点和数字对不上；
        // 没有修正时仍取地图点最新坐标（环里的 pw 可能已过期）。
        Eigen::Vector3f pw;
        if (std::fabs(st->ratioEma - 1.0f) > 1e-4f) {
            pw = Tcw.block<3, 3>(0, 0).transpose() * pcOut + cam;
        } else {
            pw = mp->GetWorldPos();
        }
        out.surface = (std::fabs(st->ratioEma - 1.0f) > 0.005f) ? 1 : 0;
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
        st->lastD = pc.norm();  // 深度锁定的基准，用未修正值保持原判据
        st->lastCam[0] = cam.x();
        st->lastCam[1] = cam.y();
        st->lastCam[2] = cam.z();
        return out;
    }

    // ============ ⑤ 平面兜底 ============
    // 平面结果没有身份，不锁定；也不更新 lastD/lastCam，因为量纲不同。
    st->locked = nullptr;
    // 同理放掉修正比：兜底这几帧没有锚点，旧修正不该带到下次锁定。
    st->ratioMp = nullptr;
    st->ratioEma = 1.0f;
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

void updateIntrinsics(float fx, float fy, float cx, float cy) {
    gFx = fx;
    gFy = fy;
    gCx = cx;
    gCy = cy;
    printf("[M6] 内参更新 fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n", gFx, gFy, gCx, gCy);
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

SurfaceDepth surfaceAtPoint(ORB_SLAM3::System* sys, const float t_cw[16],
                            float wx, float wy, float wz) {
    SurfaceDepth st;
    if (sys == nullptr || sys->GetTrackingState() != kTrackingOk) return st;

    const std::vector<cv::KeyPoint> kps = sys->GetTrackedKeyPointsUn();
    const std::vector<ORB_SLAM3::MapPoint*> mps = sys->GetTrackedMapPoints();
    if (kps.empty() || mps.empty()) return st;

    Eigen::Matrix4f Tcw;
    std::memcpy(Tcw.data(), t_cw, sizeof(float) * 16);

    Eigen::Vector3f pc =
        Tcw.block<3, 3>(0, 0) * Eigen::Vector3f(wx, wy, wz) + Tcw.block<3, 1>(0, 3);
    correctDepth(kps, mps, Tcw, "钉点", &pc, &st);
    return st;
}

}  // namespace slam
