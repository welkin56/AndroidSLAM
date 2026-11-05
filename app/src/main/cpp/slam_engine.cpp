// 见 slam_engine.h。
//
// System 从不 delete：它没有析构函数，Shutdown() 也不 join 线程，
// delete 会导致 use-after-free；让它活到进程结束还能省去词袋重载。
//
// 单例故意泄漏、线程 detach，避免进程退出时析构 joinable thread。
// 队列有界，满了丢最旧，不阻塞、不积压。
// 诊断输出走 printf，fd 1/2 已接入 logcat（tag slam）。
#include "slam_engine.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <pthread.h>

#include "ImuTypes.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"
#include "orb_track.h"

namespace slam {
namespace {

// 帧队列上限。SLAM 跟不上时丢帧，避免延迟累积。
constexpr size_t kMaxFrames = 4;

// IMU 队列上限（402 Hz 下约 5 秒）。
constexpr size_t kMaxImu = 2000;

// 等帧超时，让循环定期检查 running。
constexpr int kWaitMs = 100;

// 两趟之间在时间轴上的间隔。必须远小于 ORB-SLAM3 的 1 秒跳变阈值。
constexpr double kSessionGapS = 0.033;

// ORB_SLAM3 eTrackingState::OK。
constexpr int kTrackingOk = 2;

// 测距中值窗口。锁定后只需压毫米级位姿噪声，小窗口滞后更小。
constexpr size_t kMeasureRing = 3;

// 一趟最多记录的合并链条数（每条 10 个 float）。
constexpr int kLastMergesMax = 16;

// 连续测不到这么多帧才清空中值窗口，避免有/无抖动导致反复重开。
constexpr int kMeasureRingHold = 3;

// 测量日志限流间隔；printf 跑在 SLAM 线程上。
constexpr long long kMeasureLogMs = 300;

// 一直测不到时每隔这么久打一行；有读数时不打。
constexpr long long kMeasureStuckMs = 5000;

/** 导出给 2D 地图的轨迹点数上限。 */
constexpr int kTrajExportMax = 512;

/** 点云快照点数上限，超过后才按 stride 抽稀。 */
constexpr int kMapPointsMax = 5000;

/** 点云快照重建间隔。 */
constexpr long long kMapSnapMs = 1000;

// SLAM 线程栈大小。sizeof(Frame) 约 146 KB，且 GrabImageMonocular 中 Frame 是
// 栈上临时对象；Android 的 std::thread 默认只有 1 MB。
constexpr size_t kSlamThreadStack = 8 * 1024 * 1024;

void* workerEntry(void* arg);

struct Frame {
    long long tsNs = 0;
    int w = 0, h = 0;
    std::vector<unsigned char> gray;
};

struct Imu {
    long long tsNs = 0;
    float wx = 0, wy = 0, wz = 0;
    float ax = 0, ay = 0, az = 0;
};

// 轨迹读上游「相对参考关键帧」的表，见 System::GetFrameTrajectory。

struct Engine {
    std::atomic<State> state{State::kIdle};
    std::atomic<ORB_SLAM3::System*> sys{nullptr};
    /** 开始构造的时刻（steady_clock 毫秒），供 elapsedMs 使用。 */
    std::atomic<long long> startedAtMs{0};
    /** 是否吃帧。停止时不销毁 System。 */
    std::atomic<bool> running{false};

    /** 每次 start() 置位：worker 在第一帧把时间轴接到上一趟后面。 */
    std::atomic<bool> rebaseSession{false};

    // 启动线程前赋值，之后只读，不需要锁。
    std::string vocab;
    std::string settings;

    std::mutex qMutex;
    std::condition_variable qCv;
    std::deque<Frame> frames;
    std::deque<Imu> imus;

    std::mutex poseMutex;
    /** Twc（相机到世界），供 Kotlin 显示位置；[12..14] 是相机世界坐标。 */
    float pose[16] = {0};
    /** Tcw（世界到相机），投影用；与 pose 方向相反。 */
    float t_cw[16] = {0};
    bool poseValid = false;

    /** 上游轨迹的中转缓冲（每点 8 个 float），只在主线程使用。 */
    std::mutex trajMutex;
    std::vector<float> trajFull;

    /** 点云快照（每点 3 个 float，世界坐标），SLAM 线程写、主线程读。 */
    std::mutex mapSnapMutex;
    std::vector<float> mapSnap;
    /** 重建暂存；与 mapSnap 交换以复用内存，只在 SLAM 线程使用。 */
    std::vector<float> mapBuild;
    /** 上次重建时刻；主线程可能重置，所以是 atomic。 */
    std::atomic<long long> mapSnapAtMs{0};
    /** 小地图开关；关闭时 SLAM 线程不重建快照。 */
    std::atomic<bool> mapCloudOn{true};

    /** 实时测距开关。只影响是否计算，不影响 SLAM。 */
    std::atomic<bool> measuring{false};

    /** 最近一次距离读数（已平滑），主线程只读。 */
    std::mutex measureMutex;
    CenterMeasure measure;
    /** 中值窗口；与 measure 同一把锁，start() 会清空。 */
    std::deque<float> measureRing;
    /** 连续测不到的帧数，决定何时清空中值窗口。 */
    int measureMiss = 0;
    /** 跨帧状态，供光轴锁定和深度锁定使用。 */
    CenterMeasureState measureState;
    /** 上次测量日志时刻，用于限流。 */
    long long measureLogAtMs = 0;

    /** 地图重置次数，start() 归零。 */
    std::atomic<int> resetCount{0};

    /** 是否选择了旧图，用于区分构造失败的原因。 */
    bool loadWanted = false;

    /** 构造失败细分：0 没失败 / 1 载图失败 / 2 其他。 */
    std::atomic<int> failCode{0};

    /**
     * 保存地图请求。主线程发出，worker 在停止后空闲时执行。
     * 序列化整张 atlas 需要几秒，不能放在主线程。
     */
    std::mutex saveMutex;
    bool saveRequest = false;
    std::string savePath;   // 不含 .osa
    int saveState = 0;      // 0 空闲 / 1 排队 / 2 保存中 / 3 完成 / 4 失败
    /** 本次存图要落盘的合并记录数（请求时快照），成功后丢弃。 */
    int saveMergeCount = 0;

    /**
     * 换 atlas 请求。主线程发出，worker 空闲时执行。
     *
     * restoreHasTarget 为 false 表示恢复到 savedPath（没有则 loadPath，再没有则空图）；
     * 为 true 表示换到 restoreTarget（空串即空图）。
     * loadPath、savedPath、restoreTarget 都由这把锁保护。
     */
    std::mutex restoreMutex;
    bool restoreRequest = false;
    bool restoreHasTarget = false;
    std::string restoreTarget;
    int restoreState = 0;   // 0 空闲 / 1 排队 / 2 恢复中 / 3 完成 / 4 失败
    /** 构造时载入的 .osa 路径（不含扩展名），空表示新建地图。 */
    std::string loadPath;
    /** 最近一次成功存图的路径（不含 .osa）。 */
    std::string savedPath;

    /** 本趟开跑时的 atlas 规模，没变化则不需要恢复。 */
    std::atomic<int> baseMaps{0};
    std::atomic<int> baseKfs{0};

    /**
     * 折线上的一个点。坐标存在「锚点关键帧」的相机系里，每帧由关键帧位姿算出来。
     *
     * 同一条链共用一个锚点关键帧，所以整条链是一个刚体：局部 BA 改不动段长。
     * 关键帧位姿由它全部观测点一起约束，单个地图点被挪几十厘米时它几乎不动。
     */
    struct Pinned {
        ORB_SLAM3::MapPoint* mp = nullptr;   // 锚点来源，只记身份
        ORB_SLAM3::KeyFrame* kf = nullptr;   // 锚点关键帧；nullptr = 只能冻着
        ORB_SLAM3::Map* mapAtPin = nullptr;  // p_kf 是按哪张图的尺度算的
        float p_kf[3] = {0, 0, 0};           // 点在锚点关键帧相机系里的位置
        float sAtPin = 1.f;                  // 上面这组坐标是按哪个累积尺度算的
        float pw[3] = {0, 0, 0};             // 每帧由关键帧位姿算出的世界坐标
        bool low = false;                    // 平面兜底/勉强来的（画空心圈）
        bool startsChain = false;            // 它是一条新折线的起点
        bool closed = false;                 // 它之后「断开」了
    };
    /** 折线。与 measure 共用一把锁；只在主线程读写。 */
    std::vector<Pinned> pinned;
    /** 上一次段长心跳的时刻，限流用。 */
    long long lenLogAtMs = 0;

    /** 加路径点请求。主线程发出，worker 在下一帧之后执行一次。 */
    std::mutex wpMutex;
    std::string wpName;
    int wpState = 0;   // 0 没试过 / 1 排队 / 2 成功 / 3 失败
    std::string wpMsg;

    /** 本趟轨迹归属与合并链，在 stop() 中快照；主线程读写，不需要锁。 */
    long long lastTrajMapId = -1;
    float lastMerges[10 * 16];
    int nLastMerges = 0;
};

// 故意泄漏，见文件头第 2 条。
Engine& engine() {
    static Engine* e = new Engine;
    return *e;
}

long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void construct(Engine& e) {
    printf("[M5] 开始构造 System（IMU_MONOCULAR，bUseViewer=false）\n");
    fflush(stdout);
    const auto t0 = std::chrono::steady_clock::now();

    ORB_SLAM3::System* s = nullptr;
    try {
        // 必须显式传 false：默认值是 true，会创建 Viewer 对象并启动线程。
        s = new ORB_SLAM3::System(e.vocab, e.settings, ORB_SLAM3::System::IMU_MONOCULAR,
                                  /*bUseViewer=*/false);
    } catch (const std::exception& ex) {
        // 载图失败会以异常形式抛出。
        printf("[M5] System 构造抛异常：%s\n", ex.what());
        s = nullptr;
    } catch (...) {
        s = nullptr;
    }

    // ORB-SLAM3 的部分错误仍走 exit(-1)，不会进入上面的 catch。
    if (s == nullptr) {
        printf("[M5] System 构造失败\n");
        e.failCode.store(e.loadWanted ? 1 : 2);
        e.state.store(State::kFailed);
        {
            // worker 不会进入循环，清掉挂着的恢复请求，避免界面停在「载入地图」。
            std::lock_guard<std::mutex> lk(e.restoreMutex);
            e.restoreRequest = false;
            e.restoreState = 0;
        }
        return;
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    e.sys.store(s);
    e.state.store(State::kReady);

    // 投影需要内参。放在构造成功之后，失败时不必解析。
    initIntrinsics(e.settings);

    printf("[M5] System 构造完成，耗时 %lld ms\n", (long long)ms);
    fflush(stdout);

    // 路径点 sidecar 跟随 atlas；新建地图没有该文件属于正常情况。
    std::string lp;
    {
        std::lock_guard<std::mutex> lk(e.restoreMutex);
        lp = e.loadPath;
    }
    if (!lp.empty()) s->LoadWaypoints(lp);
}

void savePose(Engine& e, const TrackResult& r) {
    std::lock_guard<std::mutex> lk(e.poseMutex);
    std::memcpy(e.pose, r.pose16, sizeof(e.pose));
    std::memcpy(e.t_cw, r.t_cw16, sizeof(e.t_cw));
    e.poseValid = true;
}

/**
 * 重建点云快照（SLAM 线程）：枚举、抽稀、取坐标。
 */
void rebuildMapSnap(Engine& e, ORB_SLAM3::System* sys) {
    std::vector<ORB_SLAM3::MapPoint*> pts = sys->GetAllMapPoints();

    // 用持久缓冲构建，完成后与 mapSnap 交换，稳定后零分配。
    std::vector<float>& out = e.mapBuild;
    out.clear();
    const size_t n = pts.size();
    if (n > 0) {
        const size_t stride = std::max<size_t>(1, (n + kMapPointsMax - 1) / kMapPointsMax);
        out.reserve(std::min<size_t>(n / stride + 1, kMapPointsMax) * 3);
        // 只访问抽中的点，单点访问量与地图大小无关。
        for (size_t i = 0; i < n; i += stride) {
            ORB_SLAM3::MapPoint* p = pts[i];
            if (p == nullptr || p->isBad()) continue;
            const Eigen::Vector3f w = p->GetWorldPos();
            out.push_back(w.x());
            out.push_back(w.y());
            out.push_back(w.z());
        }
    }

    std::lock_guard<std::mutex> lk(e.mapSnapMutex);
    e.mapSnap.swap(out);
}

/** 到间隔才重建（SLAM 线程）；有帧等待时推迟。 */
void maybeRebuildMapSnap(Engine& e, ORB_SLAM3::System* sys) {
    if (!e.running.load() || !e.mapCloudOn.load()) return;
    const long long now = nowMs();
    if (now - e.mapSnapAtMs.load() < kMapSnapMs) return;
    {
        std::lock_guard<std::mutex> lk(e.qMutex);
        if (!e.frames.empty()) return;
    }
    e.mapSnapAtMs.store(now);
    rebuildMapSnap(e, sys);
}

/**
 * 执行挂着的加路径点请求（SLAM 线程）。
 * 不等待跟踪恢复：这一帧没有有效位姿就直接失败。
 */
void maybeAddWaypoint(Engine& e, ORB_SLAM3::System* sys, bool poseOk) {
    std::string name;
    {
        std::lock_guard<std::mutex> lk(e.wpMutex);
        if (e.wpState != 1) return;
        e.wpState = 2;
        name.swap(e.wpName);
        e.wpMsg.clear();
    }

    const bool ok = poseOk && sys->AddWaypoint(name);
    if (!ok) {
        std::lock_guard<std::mutex> lk(e.wpMutex);
        e.wpState = 3;
        if (!poseOk) {
            e.wpMsg = "没有有效位姿（跟踪没跟上）";
        } else {
            // 失败原因由 System 给出，避免把不同原因混成一句。
            const std::string why = sys->GetWaypointError();
            e.wpMsg = why.empty() ? "加不了路径点" : why;
        }
    }
}

/** 会话结束后一次性存图；只在没有帧时调用。 */
void maybeSaveMap(Engine& e, ORB_SLAM3::System* sys) {
    std::string path;
    int mergesAtSave = 0;
    {
        std::lock_guard<std::mutex> lk(e.saveMutex);
        if (!e.saveRequest) return;
        e.saveRequest = false;
        e.saveState = 2;   // 保存中
        path = e.savePath;
        mergesAtSave = e.saveMergeCount;
    }

    printf("[MAP] 开始存图 → %s.osa\n", path.c_str());
    fflush(stdout);
    bool ok = true;
    try {
        sys->SaveAtlasTo(path);
    } catch (...) {
        ok = false;
    }

    {
        std::lock_guard<std::mutex> lk(e.saveMutex);
        e.saveState = ok ? 3 : 4;
    }
    if (ok) {
        // 这些合并已随 .osa 落盘，留着会在下次存图时重复应用。
        sys->DropMergeRecords(mergesAtSave);
        {
            std::lock_guard<std::mutex> lk(e.restoreMutex);
            e.savedPath = path;   // 「上一次保存」指向这一份
        }
        // sidecar 失败不算存图失败，地图本身已经写好了。
        if (!sys->SaveWaypoints(path)) {
            printf("[MAP] 路径点 sidecar 写失败（地图已保存）\n");
            fflush(stdout);
        }
    }
    printf("[MAP] 存图%s\n", ok ? "完成" : "失败");
    fflush(stdout);
}

/**
 * 把 atlas 恢复到「上一次保存」的状态（SLAM 线程、空闲时）。
 *
 * 这一趟没存图时，建出来的图不应留在 atlas 中，否则下一趟的回环检测仍能匹配到它。
 * 返回 true 表示执行过恢复。
 */
bool maybeRestoreMap(Engine& e, ORB_SLAM3::System* sys) {
    std::string path;
    bool explicitTarget = false;
    {
        std::lock_guard<std::mutex> lk(e.restoreMutex);
        if (!e.restoreRequest) return false;
        e.restoreRequest = false;
        explicitTarget = e.restoreHasTarget;
        path = explicitTarget ? e.restoreTarget
                              : (e.savedPath.empty() ? e.loadPath : e.savedPath);
    }

    // 正在吃帧时先不处理，请求留到停止后执行。
    if (e.running.load()) {
        std::lock_guard<std::mutex> lk(e.restoreMutex);
        e.restoreRequest = true;
        return false;
    }

    // 存图优先：序列化期间 LocalMapping 被停住，此时 Reset 会一直等待。
    {
        std::lock_guard<std::mutex> lk(e.saveMutex);
        if (e.saveRequest || e.saveState == 2) {
            std::lock_guard<std::mutex> lk2(e.restoreMutex);
            e.restoreRequest = true;   // 下一轮再处理
            return false;
        }
    }

    // 这一趟没有新数据时，恢复没有意义；换源是用户明确要求，不适用此判断。
    if (!explicitTarget) {
        int st[3] = {0, 0, 0};
        sys->GetAtlasStats(&st[0], &st[1], &st[2]);
        if (st[0] == e.baseMaps.load() && st[1] == e.baseKfs.load()) {
            std::lock_guard<std::mutex> lk(e.restoreMutex);
            e.restoreState = 3;
            printf("[MAP] 恢复 atlas：这一趟没有新数据，跳过\n");
            fflush(stdout);
            return true;
        }
    }

    {
        std::lock_guard<std::mutex> lk(e.restoreMutex);
        e.restoreState = 2;   // 恢复中
    }
    printf("[MAP] %s → %s\n", explicitTarget ? "换源" : "恢复 atlas",
           path.empty() ? "(空图)" : (path + ".osa").c_str());
    fflush(stdout);

    const bool ok = sys->ReloadAtlasFrom(path);
    // 路径点跟随 atlas；ReloadAtlasFrom 已清空旧数据，这里按新源重新读取。
    if (ok) sys->LoadWaypoints(path);


    {
        std::lock_guard<std::mutex> lk(e.restoreMutex);
        if (!ok) {
            // 失败时 Tracking::Reset() 已执行，内存中是空图；同时清掉来源，
            // 避免 loadedMapPath() 仍报告未载入的图。
            e.loadPath.clear();
            e.savedPath.clear();
        } else if (explicitTarget) {
            // 换源成功后，后续恢复以新源为准。
            e.loadPath = path;
            e.savedPath.clear();
        }
    }

    // 坐标系已改变：位姿、点云快照、折线全失效。
    // 折线握着旧图的关键帧，旧图被换掉之后那些指针没有意义了。
    clearPinned();
    {
        std::lock_guard<std::mutex> lk(e.poseMutex);
        e.poseValid = false;
    }
    {
        std::lock_guard<std::mutex> lk(e.mapSnapMutex);
        e.mapSnap.clear();
    }
    e.mapSnapAtMs.store(0);   // 下一个空隙立即重建

    {
        std::lock_guard<std::mutex> lk(e.restoreMutex);
        // 期间来了新请求时保持「恢复中」，下一轮继续。
        if (!e.restoreRequest) e.restoreState = ok ? 3 : 4;
    }
    printf("[MAP] %s%s\n", explicitTarget ? "换源" : "恢复 atlas", ok ? "完成" : "失败");
    fflush(stdout);
    return true;
}

/** 段长心跳，定义在后面（要用测距那片工具函数）。 */
void logSegments(Engine& e, long long now);

// 常驻循环：先构造 System，再持续取帧、跟踪、保存位姿。
void worker(Engine& e) {
    construct(e);
    if (e.state.load() != State::kReady) return;

    ORB_SLAM3::System* sys = e.sys.load();
    bool haveBase = false;
    double base = 0.0;
    /** 上一帧喂给 SLAM 的时间（秒，本进程的时间轴），跨会话接续用。 */
    bool haveLastT = false;
    double lastT = 0.0;
    /** 本会话的时间偏移，让两趟之间的间隔看起来只有一帧。 */
    double sessionOffset = 0.0;
    // 见过非 0 的 KF 数后才算重置；刚构造完本来就没有关键帧。
    bool sawKfs = false;

    for (;;) {
        Frame f;
        bool idle = false;
        {
            std::unique_lock<std::mutex> lk(e.qMutex);
            e.qCv.wait_for(lk, std::chrono::milliseconds(kWaitMs),
                           [&] { return !e.frames.empty(); });
            if (e.frames.empty()) {
                idle = true;   // 超时且队列为空；不要在这里重建，重建会再拿 qMutex
            } else {
                f = std::move(e.frames.front());
                e.frames.pop_front();
            }
        }
        if (idle) {
            maybeRebuildMapSnap(e, sys);
            maybeSaveMap(e, sys);
            // 恢复会清空当前图，重置检测状态也要一起清。
            if (maybeRestoreMap(e, sys)) sawKfs = false;
            continue;
        }

        // 停止期间不吃帧；队列已由 stop() 清空。
        if (!e.running.load()) continue;

        // 纳秒转秒只在这里做，时间基准取本进程的第一帧。
        if (!haveBase) {
            base = static_cast<double>(f.tsNs) / 1e9;
            haveBase = true;
        }

        // 每趟重新起跑时把时间轴接在上一趟后面。不接的话，20 秒的真实间隔会触发
        // ORB-SLAM3 的 Timestamp jump 守卫（>1 秒），它会重建一张图。
        // 相机和 IMU 加同一个偏移，相对时序不变。
        if (e.rebaseSession.exchange(false)) {
            sessionOffset = haveLastT
                            ? (lastT + kSessionGapS) - (static_cast<double>(f.tsNs) / 1e9 - base)
                            : 0.0;
            if (haveLastT) {
                printf("[M5] 时间轴接续：偏移 %.3f s（上一趟最后一帧 t=%.3f）\n",
                       sessionOffset, lastT);
                fflush(stdout);
            }
        }

        const double t = static_cast<double>(f.tsNs) / 1e9 - base + sessionOffset;

        // 把上一帧到这一帧之间的 IMU 交给 ORB-SLAM3，
        // 精确切片由 TrackMonocular 内部按时间戳完成。
        std::vector<ORB_SLAM3::IMU::Point> imu;
        {
            std::lock_guard<std::mutex> lk(e.qMutex);
            while (!e.imus.empty() && e.imus.front().tsNs <= f.tsNs) {
                const Imu& m = e.imus.front();
                // 参数顺序是加速度在前、角速度在后；写反不会报错，但跟踪会乱。
                imu.emplace_back(m.ax, m.ay, m.az, m.wx, m.wy, m.wz,
                                 static_cast<double>(m.tsNs) / 1e9 - base + sessionOffset);
                e.imus.pop_front();
            }
        }

        // 走 orbTrack：本层不链 OpenCV，cv::Mat 的构造留在另一侧。
        TrackResult r;
        orbTrack(sys, f.gray.data(), f.w, f.h, t, imu, &r);
        lastT = t;
        haveLastT = true;

        // 测距关闭时不算；开启时每帧都发布，包括不可测的帧。
        // 测量读写 measureState，需要加锁；publishMeasure 之后单独取锁，不嵌套。
        if (e.measuring.load()) {
            CenterMeasure m;
            {
                std::lock_guard<std::mutex> lk(e.measureMutex);
                m = centerMeasure(sys, f.w, f.h, r.t_cw16, &e.measureState);
            }
            publishMeasure(m);
        }

        // 段长心跳不跟测距开关走：测距关掉之后折线仍然要画。
        if (r.valid) {
            std::lock_guard<std::mutex> lk(e.measureMutex);
            logSegments(e, nowMs());
        }

        // 重置检测每帧都要做。KF 数掉到 0 表示当前图被重置，
        // 坐标系已重新锚定，前一段轨迹不再可用，只保留最后一段。
        if (r.kfCount == 0) {
            if (sawKfs) {
                sawKfs = false;
                // 一次运行对应一份轨迹，与 start() 语义一致。
                sys->ClearFrameTrajectory();
                const int n = e.resetCount.fetch_add(1) + 1;
                printf("[M5] 地图被重置（第 %d 次）\n", n);
                fflush(stdout);
                // 点云是当前图的快照，同样清空。
                {
                    std::lock_guard<std::mutex> lk(e.mapSnapMutex);
                    e.mapSnap.clear();
                }
                e.mapSnapAtMs.store(0);   // 下一个空隙立即重建
            }
        } else {
            sawKfs = true;
        }

        // r.valid 仅在跟踪状态为 OK 时是 true。
        if (r.valid) savePose(e, r);

        maybeAddWaypoint(e, sys, r.valid);

        // 30fps 下 wait_for 的谓词总为真，超时分支不会执行，所以这里也要尝试重建。
        maybeRebuildMapSnap(e, sys);
    }
}

void* workerEntry(void* arg) {
    worker(*static_cast<Engine*>(arg));
    return nullptr;
}

}  // namespace

void init(const std::string& vocabPath, const std::string& settingsPath,
          const std::string& loadFromFile) {
    Engine& e = engine();

    State expected = State::kIdle;
    if (!e.state.compare_exchange_strong(expected, State::kLoading)) {
        return;  // 已经调用过或正在加载；构造只能进行一次。
    }

    // 载图由 System 构造时读取 yaml 的 System.LoadAtlasFromFile 决定，这里只记录是否带图。
    e.loadWanted = !loadFromFile.empty();
    {
        // Kotlin 传的是含 .osa 的完整文件名；恢复/换源会自行追加后缀，这里必须去掉。
        std::string lp = loadFromFile;
        const std::string kOsaExt = ".osa";
        if (lp.size() > kOsaExt.size() &&
            lp.compare(lp.size() - kOsaExt.size(), kOsaExt.size(), kOsaExt) == 0) {
            lp.resize(lp.size() - kOsaExt.size());
        }
        std::lock_guard<std::mutex> lk(e.restoreMutex);
        e.loadPath = lp;   // 没有存过图时恢复到这里
    }
    if (e.loadWanted) printf("[M5] 这次带旧图：%s\n", loadFromFile.c_str());

    e.vocab = vocabPath;
    e.settings = settingsPath;
    e.startedAtMs.store(nowMs());

    // 用 pthread 是为了设置栈大小；线程 detach 后活到进程结束。
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, kSlamThreadStack);
    pthread_t t;
    if (pthread_create(&t, &attr, workerEntry, &e) == 0) {
        pthread_detach(t);
    } else {
        printf("[M5] SLAM 线程起不来\n");
        e.state.store(State::kFailed);
    }
    pthread_attr_destroy(&attr);
}

State state() { return engine().state.load(); }

long long elapsedMs() {
    Engine& e = engine();
    if (e.state.load() != State::kLoading) return 0;
    return nowMs() - e.startedAtMs.load();
}

void pushImage(long long tsNs, const unsigned char* gray, int w, int h) {
    if (gray == nullptr || w <= 0 || h <= 0) return;
    Engine& e = engine();

    Frame f;
    f.tsNs = tsNs;
    f.w = w;
    f.h = h;
    f.gray.assign(gray, gray + static_cast<size_t>(w) * h);

    std::lock_guard<std::mutex> lk(e.qMutex);
    // 满了丢最旧的，保留最新帧。
    while (e.frames.size() >= kMaxFrames) e.frames.pop_front();
    e.frames.push_back(std::move(f));
    e.qCv.notify_one();
}

void pushImu(long long tsNs, float wx, float wy, float wz,
             float ax, float ay, float az) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.qMutex);
    if (e.imus.size() >= kMaxImu) e.imus.pop_front();
    e.imus.push_back(Imu{tsNs, wx, wy, wz, ax, ay, az});
}

void setIntrinsics(float fx, float fy, float cx, float cy,
                   float k1, float k2, float p1, float p2) {
    Engine& e = engine();
    ORB_SLAM3::System* sys = e.sys.load();
    if (sys == nullptr) {
        printf("[M5] setIntrinsics：System 还没构造好，忽略\n");
        fflush(stdout);
        return;
    }
    if (e.running.load()) {
        // 上游的 LocalMapping / LoopClosing 读这些量没有加锁，跑着改会出事。
        printf("[M5] setIntrinsics 在吃帧状态下被调用，忽略\n");
        fflush(stdout);
        return;
    }
    sys->SetCameraIntrinsics(fx, fy, cx, cy, k1, k2, p1, p2);
    updateIntrinsics(fx, fy, cx, cy);
    printf("[M5] 内参注入 fx=%.3f fy=%.3f cx=%.3f cy=%.3f k1=%.5f k2=%.5f\n",
           fx, fy, cx, cy, k1, k2);
    fflush(stdout);
}

void start() {
    Engine& e = engine();
    // 新一趟：让 worker 在第一帧把时间轴接在上一趟后面，避免触发跳变守卫。
    e.rebaseSession.store(true);
    // 一次运行对应一份轨迹；上游轨迹表从不清理，需要手动清。
    if (ORB_SLAM3::System* sys = e.sys.load()) {
        sys->ClearFrameTrajectory();
        // 记录开跑时的 atlas 规模，用于判断这一趟是否建出新数据。
        int st[3] = {0, 0, 0};
        sys->GetAtlasStats(&st[0], &st[1], &st[2]);
        e.baseMaps.store(st[0]);
        e.baseKfs.store(st[1]);
    }
    {
        // 上一趟的点云快照不能保留。
        std::lock_guard<std::mutex> lk(e.mapSnapMutex);
        e.mapSnap.clear();
    }
    e.mapSnapAtMs.store(0);   // 第一个空隙就重建
    {
        std::lock_guard<std::mutex> lk(e.poseMutex);
        e.poseValid = false;
    }
    {
        // 上一趟的读数不带到这一趟。
        std::lock_guard<std::mutex> lk(e.measureMutex);
        e.measure = CenterMeasure{};
        e.measureRing.clear();
    }
    e.resetCount.store(0);
    {
        // 上一趟未消费的加路径点请求不保留。
        std::lock_guard<std::mutex> lk(e.wpMutex);
        e.wpState = 0;
        e.wpName.clear();
        e.wpMsg.clear();
    }
    e.running.store(true);
}

void stop() {
    Engine& e = engine();

    // 先快照：running=false 后 worker 可能立刻消费恢复请求并换掉 atlas。
    if (ORB_SLAM3::System* sys = e.sys.load()) {
        e.lastTrajMapId = sys->GetTrajectoryMapId();
        // cap 的单位是 float（每条 10 个），不是条数。
        e.nLastMerges = sys->GetMergeRecords(e.lastMerges, kLastMergesMax * 10);
    }

    e.running.store(false);
    // 挂着的加路径点请求不执行，也不留到下一趟。
    {
        std::lock_guard<std::mutex> lk(e.wpMutex);
        if (e.wpState == 1) {
            e.wpState = 3;
            e.wpMsg = "已停止，没有添加";
            printf("[WP] 停止前挂着的加路径点请求已丢弃\n");
            fflush(stdout);
        }
    }
    // 清空队列，避免下次启动吃到停止前积压的旧帧。
    std::lock_guard<std::mutex> lk(e.qMutex);
    e.frames.clear();
    e.imus.clear();
}

bool pose(float out16[16]) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.poseMutex);
    if (!e.poseValid) return false;
    std::memcpy(out16, e.pose, sizeof(e.pose));
    return true;
}

int trackingState() {
    ORB_SLAM3::System* sys = engine().sys.load();
    // GetTrackingState() 内部加锁，可从任意线程调用。
    return sys == nullptr ? -1 : sys->GetTrackingState();
}

void setMeasuring(bool on) {
    Engine& e = engine();
    e.measuring.store(on);
    if (!on) {
        // 关闭时立即清空，避免界面显示一个不会更新的数字。
        std::lock_guard<std::mutex> lk(e.measureMutex);
        e.measure = CenterMeasure{};
        e.measureRing.clear();
        e.measureMiss = 0;
        e.measureState = CenterMeasureState{};
    }
}

void publishMeasure(const CenterMeasure& m) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);

    CenterMeasure out = m;
    if (m.quality == kNone) {
        // 立即变为不可测，但中值窗口保留几帧，见 kMeasureRingHold。
        if (++e.measureMiss >= kMeasureRingHold) e.measureRing.clear();
    } else {
        e.measureMiss = 0;
        e.measureRing.push_back(m.distance);
        while (e.measureRing.size() > kMeasureRing) e.measureRing.pop_front();
        // 取中值，对偶发误吸附更稳定。
        float v[kMeasureRing];
        size_t n = 0;
        for (float x : e.measureRing) v[n++] = x;
        std::sort(v, v + n);
        out.distance = v[n / 2];
    }

    // 只在可测性变化或读数变化超过 5 cm 时打日志，并做限流。
    const long long now = nowMs();
    const long long since = now - e.measureLogAtMs;
    const bool flipped = (out.quality == kNone) != (e.measure.quality == kNone);
    const bool moved = out.quality != kNone && e.measure.quality != kNone &&
                       std::fabs(out.distance - e.measure.distance) > 0.05f;
    if ((flipped || moved) && since >= kMeasureLogMs) {
        e.measureLogAtMs = now;
        if (out.quality == kNone) {
            printf("[M6] * 测不到  候选%d 支持%d 纹理%d 跨度%.0f%%\n", out.candidates,
                   out.support, out.texture, out.zSpan * 100.0f);
        } else {
            printf("[M6] * %.3f m  世界(%.3f, %.3f, %.3f)  像素(%.1f, %.1f)"
                   "  候选%d 支持%d 纹理%d 跨度%.0f%%  %s 路径%d 质量%d\n",
                   out.distance, out.wx, out.wy, out.wz, out.px, out.py,
                   out.candidates, out.support, out.texture, out.zSpan * 100.0f,
                   out.source == 1 ? "新选" : "沿用", out.method, out.quality);
        }
        fflush(stdout);
    } else if (out.quality == kNone && since >= kMeasureStuckMs) {
        // 一直测不到时也要留日志，否则无法区分开关未开、无候选和支持度不足。
        e.measureLogAtMs = now;
        printf("[M6]   测不到  候选%d 支持%d 纹理%d 跨度%.0f%%\n", out.candidates,
               out.support, out.texture, out.zSpan * 100.0f);
        fflush(stdout);
    }

    e.measure = out;
}

int getMeasure(float out7[7]) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);
    out7[0] = e.measure.distance;
    out7[1] = e.measure.wx;
    out7[2] = e.measure.wy;
    out7[3] = e.measure.wz;
    out7[4] = e.measure.px;
    out7[5] = e.measure.py;
    // 十字附近的特征点数，不管有没有地图点；界面据此区分「多停一会儿」和「换个地方」。
    out7[6] = static_cast<float>(e.measure.texture);
    return e.measure.quality;
}

// ---------------- 多点折线 ----------------

namespace {
/** 投影不可用的原因，仅在原因变化时打一行。 */
void logNoPose(int why) {
    static int last = 0;
    if (why == last) return;
    last = why;
    if (why == 1) printf("[M6] ⚠️ 投影不可用：跟踪不是 OK（折线整条不画）\n");
    if (why == 2) printf("[M6] ⚠️ 投影不可用：还没有位姿（折线整条不画）\n");
    fflush(stdout);
}

/** 折线点数上限。 */
constexpr int kMaxPinned = 128;

/** 相机后方的点插值到这个 z；只影响落点距离，不改变方向。 */
constexpr float kNearZ = 0.02f;

/** 像素夹取范围；贴近相机平面的点会得到极大值。 */
constexpr float kProjClamp = 20000.f;

/** 内参，首次 getPinned 时读取；只在主线程使用。 */
struct Intr {
    float fx = 0, fy = 0, cx = 0, cy = 0;
    bool ok = false;
};
Intr gIntr;

/** 世界到相机。列主序：元素 (r,c) 位于 [c*4+r]；pc = Rcw*pw + tcw。 */
inline Eigen::Vector3f toCam(const float t[16], const float* w) {
    return Eigen::Vector3f(
        t[0] * w[0] + t[4] * w[1] + t[8] * w[2] + t[12],
        t[1] * w[0] + t[5] * w[1] + t[9] * w[2] + t[13],
        t[2] * w[0] + t[6] * w[1] + t[10] * w[2] + t[14]);
}

/** 相机系点投影到像素；z 必须大于 0。 */
inline void project1(const Eigen::Vector3f& p, float* xy) {
    xy[0] = std::min(std::max(gIntr.fx * p.x() / p.z() + gIntr.cx, -kProjClamp), kProjClamp);
    xy[1] = std::min(std::max(gIntr.fy * p.y() / p.z() + gIntr.cy, -kProjClamp), kProjClamp);
}

/**
 * 把相机后方的端点沿线段插值到 z = kNearZ。
 * 方向必须由两端共同决定；直接把 z 夹成常数会得到错误方向。
 */
Eigen::Vector3f pullToFront(const Eigen::Vector3f& back, const Eigen::Vector3f& front) {
    const float dz = front.z() - back.z();
    if (dz <= 1e-6f) return back;   // 退化；下面会判掉
    return back + ((kNearZ - back.z()) / dz) * (front - back);
}

/** 一条线段在画面上的两个端点；两端都在相机后方时返回 false。 */
bool segPixels(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float* xy4) {
    const bool fa = a.z() > 0.f;
    const bool fb = b.z() > 0.f;
    if (!fa && !fb) return false;

    const Eigen::Vector3f pa = fa ? a : pullToFront(a, b);
    const Eigen::Vector3f pb = fb ? b : pullToFront(b, a);
    if (pa.z() <= 0.f || pb.z() <= 0.f) return false;

    project1(pa, xy4);
    project1(pb, xy4 + 2);
    return true;
}

float dist3(const float* a, const float* b) {
    const float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * 把钉点坐标刷成锚点关键帧位姿下的最新值。
 *
 * 整图尺度修正会把关键帧的平移乘 s，而 p_kf 是不变量。不跟着乘的话，
 * 每个点会整体平移 (s-1)*相机中心，两个点就朝不同方向散开。
 */
void refreshPinned(Engine::Pinned& p) {
    if (p.kf == nullptr || p.kf->isBad()) return;

    // 关键帧被挂到别的图上（合并）或图被清空时，两边的累积尺度没有可比性：
    // 算出来的比值会是垃圾，p_kf 会被乘飞。这种情况只冻住，不重标。
    ORB_SLAM3::Map* m = p.kf->GetMap();
    if (m != nullptr && m == p.mapAtPin) {
        const float s = m->GetMetricScale();
        if (s > 0.f && s != p.sAtPin) {
            const float k = s / p.sAtPin;
            p.p_kf[0] *= k;
            p.p_kf[1] *= k;
            p.p_kf[2] *= k;
            p.sAtPin = s;
        }
    } else if (p.mapAtPin != nullptr) {
        p.low = true;   // 换图了，这组坐标只能冻在旧尺度里
    }

    const Sophus::SE3f Twc = p.kf->GetPoseInverse();
    const Eigen::Vector3f w = Twc * Eigen::Vector3f(p.p_kf[0], p.p_kf[1], p.p_kf[2]);
    p.pw[0] = w.x();
    p.pw[1] = w.y();
    p.pw[2] = w.z();
}

/** 没有别的钉点再用这个关键帧时，放开「不许剔除」的保护。 */
void releaseKfIfUnused(const Engine& e, ORB_SLAM3::KeyFrame* kf) {
    if (kf == nullptr) return;
    for (const Engine::Pinned& p : e.pinned) {
        if (p.kf == kf) return;
    }
    // 图被重置后这个 KF 已经不属于任何 Map（Map::clear 会把 mpMap 置 NULL）。
    // 而 SetErase 在「待剔除」标记已置上时会直接进 SetBadFlag，它第一行就解引用
    // mpMap。那种情况下放开不放开保护都无所谓，跳过。
    if (kf->GetMap() == nullptr) return;
    kf->SetErase();
}

/**
 * 段长心跳（1 Hz）。钉点之后读数到底稳没稳住，只有这一行能证明；别的都是推理。
 * 锚点关键帧 id 一起打，用来判断两条链是不是已经分开了。
 */
void logSegments(Engine& e, long long now) {
    if (now - e.lenLogAtMs < 1000) return;
    e.lenLogAtMs = now;

    const int n = static_cast<int>(e.pinned.size());
    if (n <= 0) return;

    for (Engine::Pinned& p : e.pinned) refreshPinned(p);

    printf("[M6] 段长");
    for (int i = 0; i + 1 < n; ++i) {
        if (e.pinned[i + 1].startsChain) {
            printf(" -");
        } else {
            printf(" %.3f", dist3(e.pinned[i].pw, e.pinned[i + 1].pw));
        }
    }
    printf(" | 锚点");
    for (const Engine::Pinned& p : e.pinned) {
        printf(" %lu", p.kf == nullptr ? 0UL : p.kf->mnId);
    }
    ORB_SLAM3::Map* m = e.pinned[0].mapAtPin;
    printf(" | 图尺度 %.4f | 点 %d\n", m != nullptr ? m->GetMetricScale() : 0.f, n);
    fflush(stdout);
}

}  // namespace

bool addPinned(int snapTo) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);

    if (e.pinned.size() >= kMaxPinned) return false;

    Engine::Pinned p;
    if (snapTo >= 0) {
        // 吸附到已有点时复制它的身份，而不是使用十字当前命中的另一个点。
        if (snapTo >= static_cast<int>(e.pinned.size())) return false;
        const Engine::Pinned& src = e.pinned[snapTo];
        p.mp = src.mp;
        p.kf = src.kf;
        p.mapAtPin = src.mapAtPin;
        std::memcpy(p.p_kf, src.p_kf, sizeof(p.p_kf));
        std::memcpy(p.pw, src.pw, sizeof(p.pw));
        p.sAtPin = src.sAtPin;
        p.low = src.low;
    } else {
        if (e.measure.quality == kNone) return false;  // 没读数就没什么可钉的
        // 兜底读数没有地图点身份，坐标是按主点视线估算的；
        // 钉下去等于把平面误差带进段长，而段长要的是厘米级。
        if (e.measure.method != 1 || e.measure.mp == nullptr) return false;
        p.mp = e.measure.mp;
        p.pw[0] = e.measure.wx;
        p.pw[1] = e.measure.wy;
        p.pw[2] = e.measure.wz;

        // 同一条链共用一个锚点关键帧。两点各跟各的关键帧时，局部 BA 会把它们
        // 改出相对位移；共用一个，整条链就是一个刚体。
        ORB_SLAM3::KeyFrame* kf = nullptr;
        if (!e.pinned.empty() && !e.pinned.back().closed) kf = e.pinned.back().kf;
        const bool freshAnchor = (kf == nullptr);
        if (freshAnchor && p.mp != nullptr && !p.mp->isBad()) {
            ORB_SLAM3::KeyFrame* rk = p.mp->GetReferenceKeyFrame();
            if (rk != nullptr && !rk->isBad()) kf = rk;
        }
        if (kf != nullptr) {
            const Sophus::SE3f Tcw = kf->GetPose();
            const Eigen::Vector3f pc = Tcw * Eigen::Vector3f(p.pw[0], p.pw[1], p.pw[2]);
            if (pc.allFinite()) {
                p.kf = kf;
                p.p_kf[0] = pc.x();
                p.p_kf[1] = pc.y();
                p.p_kf[2] = pc.z();
                ORB_SLAM3::Map* m = kf->GetMap();
                p.mapAtPin = m;
                p.sAtPin = (m != nullptr) ? m->GetMetricScale() : 1.f;
                if (freshAnchor) kf->SetNotErase();
            }
        }
        // 没绑上关键帧的点只能冻着，走久了会偏，一律算「不太准」。
        p.low = (e.measure.quality != kGood) || p.kf == nullptr;
    }

    // 断开之后下一个点就是新折线的起点；由这一侧判断，Kotlin 不需要保存链状态。
    p.startsChain = e.pinned.empty() || e.pinned.back().closed;
    e.pinned.push_back(p);
    if (snapTo < 0) {
        printf("[M6] 钉点 #%d 锚点KF %lu 关键帧系(%.3f, %.3f, %.3f) 质量%d\n",
               static_cast<int>(e.pinned.size()), p.kf == nullptr ? 0UL : p.kf->mnId,
               p.p_kf[0], p.p_kf[1], p.p_kf[2], e.measure.quality);
        fflush(stdout);
    }
    return true;
}

void undoPinned() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);
    if (e.pinned.empty()) return;
    ORB_SLAM3::KeyFrame* kf = e.pinned.back().kf;
    e.pinned.pop_back();
    releaseKfIfUnused(e, kf);
}

void clearPinned() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);
    // 多个点共用同一个锚点关键帧，先去重再逐个放开。
    std::vector<ORB_SLAM3::KeyFrame*> kfs;
    for (const Engine::Pinned& p : e.pinned) {
        if (p.kf == nullptr) continue;
        if (std::find(kfs.begin(), kfs.end(), p.kf) == kfs.end()) kfs.push_back(p.kf);
    }
    e.pinned.clear();
    int stale = 0;
    for (ORB_SLAM3::KeyFrame* kf : kfs) {
        // 同 releaseKfIfUnused：图被重置后 mpMap 是 NULL，SetErase 会崩。
        if (kf->GetMap() == nullptr) {
            ++stale;
            continue;
        }
        kf->SetErase();
    }
    if (stale > 0) {
        printf("[M6] 清空折线：%d 个锚点KF 已随旧图失效，跳过解保护\n", stale);
        fflush(stdout);
    }
}

void closePinnedChain() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);
    if (!e.pinned.empty()) e.pinned.back().closed = true;
}

int getPinned(float* out, int cap) {
    // 布局：1 + 3n + 5(n-1) + 5 = 8n + 1；(cap-1)/8 是能装下的点数。
    if (out == nullptr) return 0;
    Engine& e = engine();

    if (!gIntr.ok) gIntr.ok = cameraIntrinsics(&gIntr.fx, &gIntr.fy, &gIntr.cx, &gIntr.cy);

    // poseMutex 和 measureMutex 不嵌套：先拷贝位姿并释放锁，再获取测量锁。
    // 必须使用 t_cw（Tcw）；误用 pose（Twc）会让所有点的 z 为负。
    float t[16];
    bool havePose = false;
    {
        std::lock_guard<std::mutex> lk(e.poseMutex);
        if (e.poseValid) {
            std::memcpy(t, e.t_cw, sizeof(t));
            havePose = true;
        }
    }
    ORB_SLAM3::System* sys = e.sys.load();
    if (!havePose || sys == nullptr || sys->GetTrackingState() != kTrackingOk) {
        // 跟踪不是 OK 时 t_cw 是上一帧的旧值，继续投影会让标记停在错误位置。
        havePose = false;
        logNoPose(sys == nullptr || sys->GetTrackingState() != kTrackingOk ? 1 : 2);
    }

    std::lock_guard<std::mutex> lk(e.measureMutex);
    const int n = std::min(static_cast<int>(e.pinned.size()),
                           std::min((cap - 1) / 8, kMaxPinned));
    if (n <= 0) return 0;
    const int segBase = 1 + 3 * n;

    // ① 计算每个点的相机系坐标并刷新世界坐标。
    //    坐标跟着锚点关键帧走；整图尺度修正时 p_kf 一起重标。
    Eigen::Vector3f pc[kMaxPinned];
    for (int i = 0; i < n; ++i) {
        Engine::Pinned& p = e.pinned[i];
        refreshPinned(p);
        pc[i] = havePose ? toCam(t, p.pw) : Eigen::Vector3f(0, 0, -1);
    }

    // ② 标记点和 flags（bit0 在相机前方 / bit1 之后断开 / bit2 不太准）
    for (int i = 0; i < n; ++i) {
        const Engine::Pinned& p = e.pinned[i];
        float* o = out + 1 + 3 * i;
        o[0] = -1.f;
        o[1] = -1.f;
        float flags = 0.f;
        if (pc[i].z() > 0.f) {
            project1(pc[i], o);
            flags += 1.f;
        }
        if (p.closed) flags += 2.f;
        if (p.low) flags += 4.f;
        o[2] = flags;
    }

    // ③ 每段的两个屏幕端点和长度；几何全部在这里算完。
    for (int i = 0; i < n - 1; ++i) {
        float* o = out + segBase + 5 * i;
        if (e.pinned[i + 1].startsChain || !segPixels(pc[i], pc[i + 1], o)) {
            o[0] = o[1] = o[2] = o[3] = 0.f;
            o[4] = -1.f;   // 小于 0 表示不画这一段
            continue;
        }
        o[4] = dist3(e.pinned[i].pw, e.pinned[i + 1].pw);
    }

    // ④ 橡皮筋：最后一个点到当前十字命中点；断开、无读数或两端都在后方时为 -1。
    // 没有读数时 measure 坐标是 0，不能拿来计算。
    float* lv = out + segBase + 5 * (n - 1);
    lv[0] = lv[1] = lv[2] = lv[3] = 0.f;
    lv[4] = -1.f;
    const Engine::Pinned& last = e.pinned[n - 1];
    if (havePose && !last.closed && e.measure.quality != kNone) {
        const float cur[3] = {e.measure.wx, e.measure.wy, e.measure.wz};
        if (segPixels(pc[n - 1], toCam(t, cur), lv)) lv[4] = dist3(last.pw, cur);
    }

    return n;
}

// ---------------- 路径点 ----------------

void requestAddWaypoint(const std::string& name) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.wpMutex);
    // 已经排着一个就不再排 —— 连按两下只该加一个点。
    if (e.wpState == 1) return;
    e.wpName = name;
    e.wpMsg.clear();
    e.wpState = 1;   // 排队
}

int waypointState() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.wpMutex);
    return e.wpState;
}

std::string waypointMsg() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.wpMutex);
    return e.wpMsg;
}

void renameWaypoint(int id, const std::string& name) {
    ORB_SLAM3::System* sys = engine().sys.load();
    if (sys != nullptr) sys->RenameWaypoint(id, name);
}

void deleteWaypoint(int id) {
    ORB_SLAM3::System* sys = engine().sys.load();
    if (sys != nullptr) sys->DeleteWaypoint(id);
}

long long waypointRevision() {
    ORB_SLAM3::System* sys = engine().sys.load();
    return sys == nullptr ? 0 : sys->GetWaypointRevision();
}

int getWaypoints(float* pos, int posCap, long long* meta, int metaCap,
                 std::string* names, int nameCap) {
    ORB_SLAM3::System* sys = engine().sys.load();
    if (sys == nullptr || pos == nullptr || meta == nullptr) return 0;

    const int n = sys->GetWaypoints(pos, posCap, meta, metaCap);
    // 名字按同一顺序填；装不下就留空串（坐标照给，只是这一拍不显示名字）。
    for (int i = 0; i < n && names != nullptr && i < nameCap; ++i)
        names[i] = sys->GetWaypointName(static_cast<int>(meta[4 * i]));
    return n;
}

int waypointCount() {
    ORB_SLAM3::System* sys = engine().sys.load();
    return sys == nullptr ? 0 : sys->GetWaypointCount();
}

int nextWaypointId() {
    ORB_SLAM3::System* sys = engine().sys.load();
    return sys == nullptr ? 1 : sys->GetNextWaypointId();
}

int resetCount() { return engine().resetCount.load(); }

int trajSize() {
    ORB_SLAM3::System* sys = engine().sys.load();
    return sys == nullptr ? 0 : sys->GetFrameTrajectorySize();
}

int getTrajectory(float* out, int cap) {
    if (out == nullptr || cap < 3) return 0;
    Engine& e = engine();
    ORB_SLAM3::System* sys = e.sys.load();
    if (sys == nullptr) return 0;

    const int maxPts = std::min(cap / 3, kTrajExportMax);
    if (maxPts <= 0) return 0;

    std::lock_guard<std::mutex> lk(e.trajMutex);
    if (static_cast<int>(e.trajFull.size()) < maxPts * 8) e.trajFull.resize(maxPts * 8);
    // 抽稀在上游做（那边才知道总帧数）；这边只把 8 个里的 xyz 挑出来。
    const int n = sys->GetFrameTrajectory(e.trajFull.data(), maxPts * 8);
    for (int i = 0; i < n; ++i) {
        out[3 * i] = e.trajFull[8 * i + 1];
        out[3 * i + 1] = e.trajFull[8 * i + 2];
        out[3 * i + 2] = e.trajFull[8 * i + 3];
    }
    return n;
}

int getMapPoints(float* out, int cap) {
    if (out == nullptr || cap < 3) return 0;
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.mapSnapMutex);
    const int n = std::min(static_cast<int>(e.mapSnap.size() / 3), cap / 3);
    if (n <= 0) return 0;
    std::memcpy(out, e.mapSnap.data(), sizeof(float) * 3 * static_cast<size_t>(n));
    return n;
}

void requestSaveMap(const std::string& path) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.saveMutex);
    // 排队中或正在保存时忽略重复请求。
    if (e.saveState == 1 || e.saveState == 2) return;
    e.savePath = path;
    // 记录请求这一刻的合并链数量；完成时再数会多丢记录。
    e.saveMergeCount = e.nLastMerges;
    e.saveRequest = true;
    e.saveState = 1;             // 排队
}

int saveState() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.saveMutex);
    return e.saveState;
}

void requestRestoreMap() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.restoreMutex);
    // 排队中或正在恢复时忽略重复请求。
    if (e.restoreState == 1 || e.restoreState == 2) return;
    e.restoreHasTarget = false;   // 恢复到上一次保存
    e.restoreRequest = true;
    e.restoreState = 1;   // 排队
}

void requestLoadAtlas(const std::string& path) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.restoreMutex);
    // 换源请求不丢弃，以最后一次选择为准；正在恢复时排队等待。
    e.restoreTarget = path;
    e.restoreHasTarget = true;
    e.restoreRequest = true;
    e.restoreState = 1;   // 排队
}

std::string loadedMapPath() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.restoreMutex);
    return e.savedPath.empty() ? e.loadPath : e.savedPath;
}

int restoreState() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.restoreMutex);
    return e.restoreState;
}

void atlasStats(int out3[3]) {
    out3[0] = 0;
    out3[1] = 0;
    out3[2] = 0;
    ORB_SLAM3::System* sys = engine().sys.load();
    if (sys != nullptr) sys->GetAtlasStats(&out3[0], &out3[1], &out3[2]);
}

int failureCode() { return engine().failCode.load(); }

void setMapCloudEnabled(bool on) {
    Engine& e = engine();
    if (e.mapCloudOn.exchange(on) == on) return;   // 状态未变
    if (on) e.mapSnapAtMs.store(0);                // 重开后下一个空隙立即重建
    printf("[MAP] 点云快照%s\n", on ? "启用" : "停用");
    fflush(stdout);
}

void dumpTrajectory(const std::string& path) {
    Engine& e = engine();
    ORB_SLAM3::System* sys = e.sys.load();
    if (sys == nullptr) return;

    int n = 0;
    {
        std::lock_guard<std::mutex> lk(e.trajMutex);
        for (int cap = 8 * 4096; ; cap *= 2) {
            e.trajFull.resize(cap);
            n = sys->GetFrameTrajectory(e.trajFull.data(), cap);
            if (n < cap / 8 || cap >= 8 * 400000) break;
        }
    }
    if (n <= 0) return;

    std::ofstream f(path);
    if (!f.is_open()) {
        printf("[M5] 轨迹写不出去：%s\n", path.c_str());
        return;
    }
    // 每行：时间(秒) tx ty tz qx qy qz qw，列与 SaveTrajectoryEuRoC 一致。
    f << std::setprecision(9);
    for (int i = 0; i < n; ++i) {
        const float* p = e.trajFull.data() + 8 * i;
        f << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " "
          << p[4] << " " << p[5] << " " << p[6] << " " << p[7] << "\n";
    }
    printf("[M5] 轨迹导出 %d 条 → %s\n", n, path.c_str());
    fflush(stdout);
}

long long lastTrajMapId() { return engine().lastTrajMapId; }

int lastMerges(float* out, int cap) {
    Engine& e = engine();
    if (out == nullptr || cap < 10) return 0;
    const int n = std::min(e.nLastMerges, cap / 10);
    if (n <= 0) return 0;
    std::memcpy(out, e.lastMerges, sizeof(float) * 10 * static_cast<size_t>(n));
    return n;
}

long long trajectoryMapId() {
    ORB_SLAM3::System* sys = engine().sys.load();
    return sys == nullptr ? -1 : sys->GetTrajectoryMapId();
}

int mapList(long long* ids, int* kfs, int* mps, int cap) {
    ORB_SLAM3::System* sys = engine().sys.load();
    return sys == nullptr ? 0 : sys->GetMapList(ids, kfs, mps, cap);
}

int mapCloud(long long mapId, float* out, int cap) {
    ORB_SLAM3::System* sys = engine().sys.load();
    return sys == nullptr ? 0 : sys->GetMapCloud(mapId, out, cap);
}

int displayTransforms(float* out, int cap) {
    if (out == nullptr || cap < 9) return 0;
    ORB_SLAM3::System* sys = engine().sys.load();
    if (sys == nullptr) return 0;
    return sys->GetDisplayTransforms(out, cap);
}

int readTrajFile(const std::string& path,
                 float s, float qx, float qy, float qz, float qw,
                 float tx, float ty, float tz,
                 float* out, int cap) {
    if (out == nullptr || cap < 3) return 0;
    std::ifstream in(path);
    if (!in.is_open()) {
        printf("[TRAJ] 显示：读不开 %s\n", path.c_str());
        return 0;
    }

    const Eigen::Quaternionf qOn = Eigen::Quaternionf(qw, qx, qy, qz).normalized();
    const Eigen::Vector3f tOn(tx, ty, tz);

    // 文件可能很长，先收集再抽稀，使 stride 覆盖全程。
    std::vector<Eigen::Vector3f> pts;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream ls(line);
        double ts, x, y, z, lqx, lqy, lqz, lqw;
        if (!(ls >> ts >> x >> y >> z >> lqx >> lqy >> lqz >> lqw)) continue;
        pts.push_back(s * (qOn * Eigen::Vector3f((float)x, (float)y, (float)z)) + tOn);
    }
    if (pts.empty()) return 0;

    const int maxPts = cap / 3;
    const size_t stride = std::max<size_t>(1, (pts.size() + maxPts - 1) / maxPts);
    int n = 0;
    for (size_t i = 0; i < pts.size() && n < maxPts; i += stride) {
        out[3 * n] = pts[i].x();
        out[3 * n + 1] = pts[i].y();
        out[3 * n + 2] = pts[i].z();
        ++n;
    }
    // 末点通常不在网格上，补上以免轨迹短一截。
    if (n > 0 && n < maxPts && (pts.size() - 1) % stride != 0) {
        const Eigen::Vector3f& p = pts.back();
        out[3 * n] = p.x();
        out[3 * n + 1] = p.y();
        out[3 * n + 2] = p.z();
        ++n;
    }
    return n;
}

bool transformTrajFile(const std::string& src, const std::string& dst,
                       float s, float qx, float qy, float qz, float qw,
                       float tx, float ty, float tz) {
    std::ifstream in(src);
    if (!in.is_open()) {
        printf("[TRAJ] 变换：读不开 %s\n", src.c_str());
        return false;
    }
    std::ofstream out(dst, std::ios::trunc);
    if (!out.is_open()) {
        printf("[TRAJ] 变换：写不开 %s\n", dst.c_str());
        return false;
    }

    const Eigen::Quaternionf qOn = Eigen::Quaternionf(qw, qx, qy, qz).normalized();
    const Eigen::Vector3f tOn(tx, ty, tz);

    // 格式与 dumpTrajectory 一致：time tx ty tz qx qy qz qw，均为 Twc。
    out << std::setprecision(9);
    std::string line;
    int bad = 0;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream ls(line);
        double ts, x, y, z, lqx, lqy, lqz, lqw;
        if (!(ls >> ts >> x >> y >> z >> lqx >> lqy >> lqz >> lqw)) {
            ++bad;
            continue;
        }
        const Eigen::Quaternionf q((float)lqw, (float)lqx, (float)lqy, (float)lqz);
        const Eigen::Vector3f tw =
            s * (qOn * Eigen::Vector3f((float)x, (float)y, (float)z)) + tOn;
        const Eigen::Quaternionf q2 = (qOn * q).normalized();
        out << ts << " " << tw.x() << " " << tw.y() << " " << tw.z() << " "
            << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << "\n";
    }
    out.close();
    if (!out.good()) {
        printf("[TRAJ] 变换：写失败 %s\n", dst.c_str());
        return false;
    }
    if (bad > 0) {
        // 坏行说明文件不完整，按失败处理，避免用半截结果覆盖原件。
        printf("[TRAJ] 变换：%s 有 %d 行解析不了\n", src.c_str(), bad);
        return false;
    }
    printf("[TRAJ] 变换完成 → %s\n", dst.c_str());
    fflush(stdout);
    return true;
}

}  // namespace slam
