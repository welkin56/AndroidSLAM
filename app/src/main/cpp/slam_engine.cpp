// 见 slam_engine.h。
//
// 三条纪律，都是 ORB-SLAM3 的形态逼出来的：
//
// 1. **绝不 delete System。** 它没有析构函数；`Shutdown()` 也不 join 线程 ——
//    `System.cc` 里等 LocalMapping/LoopClosing 结束的那段被整段注释掉了，只发个
//    `RequestFinish` 就返回。所以「析构」之后线程还在跑、还握着已释放的对象，delete 就是
//    use-after-free。「System 活到进程结束」既是省一次 145 MB 词袋重载，也是唯一安全的做法。
//
// 2. **单例故意泄漏、线程 detach。** 函数内静态对象会在进程退出时析构，而那时去析构一个
//    joinable 的 std::thread 会直接 std::terminate。
//
// 3. **队列有界、满丢最旧。** 不阻塞、不积压 —— 那是 §0 的验收判据
//    「SLAM 跟不上时丢帧不积压」。
//
// 诊断输出走 printf 就行 —— M2 已经把 fd 1/2 接进 logcat 了（tag `slam`）。
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
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <pthread.h>

#include "ImuTypes.h"
#include "MapPoint.h"
#include "System.h"
#include "orb_track.h"

namespace slam {
namespace {

// 帧队列上限。**故意小**：SLAM 跟不上时就得丢，攒着只会让延迟越滚越大。
constexpr size_t kMaxFrames = 4;

// IMU 队列上限（402 Hz 下约 5 秒）。正常每帧就排空一次，够不到这个数。
constexpr size_t kMaxImu = 2000;

// 等帧的超时。让循环能定期回头看看 running。
constexpr int kWaitMs = 100;

// ORB_SLAM3 `eTrackingState::OK`（Tracking.h:121）。和 measure_center.cpp 里那个同义。
constexpr int kTrackingOk = 2;

// 测距读数做中值的窗口。锁定之后它只需要压**毫米级**的位姿噪声，小窗口滞后更小。
constexpr size_t kMeasureRing = 3;

// 连续测不到这么多帧才真的把中值窗口扔掉。
// 一次就清的话，「有/无/有/无」的抖动会让中值反复重开 —— 等于没有平滑。
constexpr int kMeasureRingHold = 3;

// 测量日志的限流间隔。`printf` 是往管道写，卡一下就是毫秒级，而它跑在 SLAM 线程上。
constexpr long long kMeasureLogMs = 300;

// 「一直测不到」每隔这么久也打一行。**只在测不到时**，有读数不会打 ⟹ 不刷屏。
constexpr long long kMeasureStuckMs = 5000;

/** 导给 2D 地图的轨迹点数上限。画面上只有 140dp，再多也看不出来。 */
constexpr int kTrajExportMax = 512;

/** 点云快照的点数上限。室内规模装得下就**不抽稀**，超了才按这个台阶抽。 */
constexpr int kMapPointsMax = 5000;

/** 点云快照的重建间隔。地图点增得很慢，一秒一次足够。 */
constexpr long long kMapSnapMs = 1000;

// SLAM 线程的栈大小。**不能省**：`sizeof(Frame)` 是 146 KB，而
// `Tracking::GrabImageMonocular` 里 `mCurrentFrame = Frame(...)` 是**栈上的临时对象**。
// PC 上跑 `mono_inertial_euroc` 用的是主线程（8 MB），Android 的 `std::thread` 只给 1 MB。
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

// 轨迹读上游那份「相对参考关键帧」的表 —— 见 `System::GetFrameTrajectory`。

struct Engine {
    std::atomic<State> state{State::kIdle};
    std::atomic<ORB_SLAM3::System*> sys{nullptr};
    /** 开始构造的时刻（steady_clock 的毫秒，只在本进程内有意义）。给 elapsedMs 用。 */
    std::atomic<long long> startedAtMs{0};
    /** 吃不吃帧。和「System 在不在」是两件事 —— 停止不销毁 System（§5 决策一）。 */
    std::atomic<bool> running{false};

    // 只在启动线程之前赋值，之后只读 —— 所以不用锁。
    std::string vocab;
    std::string settings;

    std::mutex qMutex;
    std::condition_variable qCv;
    std::deque<Frame> frames;
    std::deque<Imu> imus;

    std::mutex poseMutex;
    /** **Twc**（相机 → 世界）。给 Kotlin 显示位置用 —— `[12..14]` 就是相机在世界里的坐标。 */
    float pose[16] = {0};
    /** **Tcw**（世界 → 相机）。投影用。⚠️ 和 `pose` **方向相反**，别混（混了的表现见 projectPoints）。 */
    float t_cw[16] = {0};
    bool poseValid = false;

    /** 拉上游那份轨迹的中转缓冲（8 个 float/点，上游的格式）。只在主线程碰。 */
    std::mutex trajMutex;
    std::vector<float> trajFull;

    /** 地图点云快照（3 个 float/点，世界米）。SLAM 线程重建、主线程读。 */
    std::mutex mapSnapMutex;
    std::vector<float> mapSnap;
    /** 重建时的暂存。和 `mapSnap` 换一下就把上一份回收成下一次的缓冲（零分配）。**只在 SLAM 线程碰。** */
    std::vector<float> mapBuild;
    /** 上次重建的时刻。**主线程要能在重开/重启时归零 ⟹ atomic。** */
    std::atomic<long long> mapSnapAtMs{0};
    /** 抽屉里那个「小地图」开关。关掉时 SLAM 线程整个不重建。 */
    std::atomic<bool> mapCloudOn{true};

    /** 抽屉里那个「实时测距」开关。只影响算不算，不影响 SLAM。 */
    std::atomic<bool> measuring{false};

    /** 最近一次的距离读数（已平滑）。主线程只读。 */
    std::mutex measureMutex;
    CenterMeasure measure;
    /** 中值窗口。和 `measure` 同一把锁 —— 因为 `start()` 要能从主线程把它清掉。 */
    std::deque<float> measureRing;
    /** 连续测不到的帧数。决定**什么时候**才真的扔掉中值窗口（见 kMeasureRingHold）。 */
    int measureMiss = 0;
    /** 跨帧的状态：三条闸门里两条要用它（光轴锁定、深度锁定）。和 `measure` 同一把锁。 */
    CenterMeasureState measureState;
    /** 上次打测量日志的时刻，用来限流。 */
    long long measureLogAtMs = 0;

    /** 「地图被重置」的次数。`start()` 归零。 */
    std::atomic<int> resetCount{0};

    /**
     * 折线上的一个点。**存 `MapPoint*` 而不是只存坐标** —— 坐标会变，身份不会。
     * 理由见 `slam_engine.h` 那一整段。
     */
    struct Pinned {
        ORB_SLAM3::MapPoint* mp = nullptr;  // nullptr = 平面兜底来的，没身份，只能冻着
        float pw[3] = {0, 0, 0};            // 兜底/失效时的坐标，也当每帧的刷新缓存
        bool low = false;                   // 平面兜底/勉强来的（画空心圈）
        bool startsChain = false;           // 它是一条新折线的起点
        bool closed = false;                // 它之后「断开」了
    };
    /** 折线。和 `measure` 用同一把锁 —— `addPinned` 要读它。**全部主线程碰。** */
    std::vector<Pinned> pinned;
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
        // bUseViewer 必须显式传 false：System.h 里它的默认值是 true。
        // Pangolin 虽然剥了，但传 true 照样会 new 一个 Viewer 对象 + 起一条 std::thread。
        s = new ORB_SLAM3::System(e.vocab, e.settings, ORB_SLAM3::System::IMU_MONOCULAR,
                                  /*bUseViewer=*/false);
    } catch (...) {
        s = nullptr;
    }

    // 注意：ORB-SLAM3 报错用的是 exit(-1)（System.cc 7 处 + Settings.cc 9 处），
    // 那条路根本走不到这里 —— 进程直接没了，logcat 里只留 M2 接出来的那行 cerr。
    if (s == nullptr) {
        printf("[M5] System 构造抛异常了\n");
        e.state.store(State::kFailed);
        return;
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    e.sys.store(s);
    e.state.store(State::kReady);

    // 投影要内参（幂等，从 settings 读，不抄常量）。**放 System 造好之后** ——
    // 一是不占 `init()` 那个主线程，二是构造失败时不用白付这一次解析。
    initIntrinsics(e.settings);

    printf("[M5] System 构造完成，耗时 %lld ms\n", (long long)ms);
    fflush(stdout);
}

void savePose(Engine& e, const TrackResult& r) {
    std::lock_guard<std::mutex> lk(e.poseMutex);
    std::memcpy(e.pose, r.pose16, sizeof(e.pose));
    std::memcpy(e.t_cw, r.t_cw16, sizeof(e.t_cw));
    e.poseValid = true;
}

/**
 * 重建点云快照（**SLAM 线程**）：枚举全图 → 抽稀 → 取坐标。
 */
void rebuildMapSnap(Engine& e, ORB_SLAM3::System* sys) {
    std::vector<ORB_SLAM3::MapPoint*> pts = sys->GetAllMapPoints();

    // 用持久缓冲建、建完和 `mapSnap` 对换 ⟹ 上一份回收成下一次的缓冲，稳定后零分配。
    std::vector<float>& out = e.mapBuild;
    out.clear();
    const size_t n = pts.size();
    if (n > 0) {
        const size_t stride = std::max<size_t>(1, (n + kMapPointsMax - 1) / kMapPointsMax);
        out.reserve(std::min<size_t>(n / stride + 1, kMapPointsMax) * 3);
        // 只碰**抽中的**那些 ⟹ 单点访问量（两次加锁 + 一次取坐标）与地图大小无关。
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

/** 到点了才重建（**SLAM 线程**）。**有帧等着就推迟** —— 别让一次枚举停在 LocalMapping 前面。 */
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

// 常驻循环：先构造，然后一直取帧、跟踪、存位姿。M4 那版是「构造完就退出」。
void worker(Engine& e) {
    construct(e);
    if (e.state.load() != State::kReady) return;

    ORB_SLAM3::System* sys = e.sys.load();
    bool haveBase = false;
    double base = 0.0;
    // 先见过非 0 才算重置 —— 刚构造完本来就一个 KF 都没有。
    bool sawKfs = false;

    for (;;) {
        Frame f;
        bool idle = false;
        {
            std::unique_lock<std::mutex> lk(e.qMutex);
            e.qCv.wait_for(lk, std::chrono::milliseconds(kWaitMs),
                           [&] { return !e.frames.empty(); });
            if (e.frames.empty()) {
                idle = true;   // 超时：队列空着（⚠️ 别在锁内调重建 —— 它要拿 qMutex）
            } else {
                f = std::move(e.frames.front());
                e.frames.pop_front();
            }
        }
        if (idle) {
            maybeRebuildMapSnap(e, sys);
            continue;
        }

        // 停止期间不吃帧。队列由 stop() 清空，所以这里直接丢掉也不会吃到旧帧。
        if (!e.running.load()) continue;

        // **纳秒 → 秒的换算只在这里做**（§6 约定 2）。基准取第一帧。
        if (!haveBase) {
            base = static_cast<double>(f.tsNs) / 1e9;
            haveBase = true;
        }
        const double t = static_cast<double>(f.tsNs) / 1e9 - base;

        // 把「上一帧到这一帧之间」的 IMU 交给它。
        // 精确切片由 ORB-SLAM3 自己做 —— `TrackMonocular` 内部先灌进 `mlQueueImuData`，
        // `Frame` 构造里的 `Tracking::PreintegrateIMU()` 再按时间戳 collectUntil。
        std::vector<ORB_SLAM3::IMU::Point> imu;
        {
            std::lock_guard<std::mutex> lk(e.qMutex);
            while (!e.imus.empty() && e.imus.front().tsNs <= f.tsNs) {
                const Imu& m = e.imus.front();
                // ⚠️⚠️ 顺序：**加速度在前、角速度在后** —— `IMU::Point` 的签名如此
                // （`ImuTypes.h:49`）。我们的 `ImuItem` 是反的（时间戳+角速度在前），
                // 照直觉对过去会把两者**静默写反**：不报错，只是跟踪整个乱掉。
                imu.emplace_back(m.ax, m.ay, m.az, m.wx, m.wy, m.wz,
                                 static_cast<double>(m.tsNs) / 1e9 - base);
                e.imus.pop_front();
            }
        }

        // ⚠️ 走 `orbTrack` 而不是直接 `sys->TrackMonocular`：这一层（`slam_jni`）
        // **刻意不链 OpenCV**，`cv::Mat` 的构造留在链着它的那一侧。见 orb_track.h。
        TrackResult r;
        orbTrack(sys, f.gray.data(), f.w, f.h, t, imu, &r);

        // 测距：开关关着就连算都不算。开着的每一帧都要发布 —— **包括不可测的帧**，
        // 测不到却留着上一帧的数字，比不显示更坏。
        // 测量要在锁下跑，因为它要读写 `measureState`（锁定住哪个点）。
        // `publishMeasure` 自己再拿一次锁 —— 两次是**先后**的，不嵌套，不会死锁。
        if (e.measuring.load()) {
            CenterMeasure m;
            {
                std::lock_guard<std::mutex> lk(e.measureMutex);
                m = centerMeasure(sys, f.w, f.h, r.t_cw16, &e.measureState);
            }
            publishMeasure(m);
        }

        // ⚠️ 重置检测**每帧都要做** —— 重置恰恰发生在不 OK 的时候。
        // 重置会清空当前图（KF 数掉到 0），而坐标系已经重新锚定，
        // 前一段轨迹拼不上来了 —— 清掉，只留最后一段。
        if (r.kfCount == 0) {
            if (sawKfs) {
                sawKfs = false;
                //（一次运行 = 一份轨迹，和 `start()` 同一个语义）。
                sys->ClearFrameTrajectory();
                const int n = e.resetCount.fetch_add(1) + 1;
                printf("[M5] 地图被重置（第 %d 次）\n", n);
                fflush(stdout);
                // 点云同理：它是「当前那张图」的快照，没有跨图判据。
                {
                    std::lock_guard<std::mutex> lk(e.mapSnapMutex);
                    e.mapSnap.clear();
                }
                e.mapSnapAtMs.store(0);   // 下一个空隙立刻重建
            }
        } else {
            sawKfs = true;
        }

        // `r.valid` 只在跟踪状态是 OK 时为 true（见 orb_track.cpp）。
        if (r.valid) savePose(e, r);

        // 刚处理完一帧、队列大概又空了 —— 同一个帧间空隙。
        // ⚠️ 必须在这儿也调一次：30fps 下 `wait_for` 的谓词每次都立刻为真，
        //    上面那个超时分支**根本不执行**。
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
        return;  // 已经调过（或正在加载）。构造一次要三十几秒，不能重来。
    }

    if (!loadFromFile.empty()) {
        // ⚠️ M5 还不支持载入旧图。而且这里有个结构性的麻烦：`System` 的构造函数**没有**
        // 「从哪个文件载」这个参数 —— 它读的是 yaml 里的 `System.LoadAtlasFromFile`
        // （System.cc:76-98）。所以要真支持，M8 得另想办法。
        printf("[M5] 收到了 loadFromFile 但还不支持载入旧图，忽略：%s\n", loadFromFile.c_str());
    }

    e.vocab = vocabPath;
    e.settings = settingsPath;
    e.startedAtMs.store(nowMs());

    // 用 pthread 而不是 std::thread —— 只为了能设栈大小（std::thread 没这个接口）。
    // 起来就 detach：它活到进程结束，没人 join 它。
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
    // 满了丢**最旧的**：晚到的帧才是有用的，攒着只会让延迟越滚越大。
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

void start() {
    Engine& e = engine();
    // 一次运行 = 一份轨迹。上游那张表从不清理，不清的话会跨 run 累加。
    if (ORB_SLAM3::System* sys = e.sys.load()) sys->ClearFrameTrajectory();
    {
        // 点云同理：上一趟的快照不能留到这一趟。
        std::lock_guard<std::mutex> lk(e.mapSnapMutex);
        e.mapSnap.clear();
    }
    e.mapSnapAtMs.store(0);   // 这一趟的第一个空隙就重建
    {
        std::lock_guard<std::mutex> lk(e.poseMutex);
        e.poseValid = false;
    }
    {
        // 上一趟的读数不留到这一趟（免得「刚启动就显示一个数」）。
        std::lock_guard<std::mutex> lk(e.measureMutex);
        e.measure = CenterMeasure{};
        e.measureRing.clear();
    }
    e.resetCount.store(0);
    e.running.store(true);
}

void stop() {
    Engine& e = engine();
    e.running.store(false);
    // **清空两个队列**：不然下次启动会把停止前积压的旧帧当成新帧喂进去，
    // 而那些帧的时间戳已经落后一大截，跟踪会当场乱掉。
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
    // `GetTrackingState()` 内部加锁（System.cc:1320），可以从任意线程调。
    return sys == nullptr ? -1 : sys->GetTrackingState();
}

void setMeasuring(bool on) {
    Engine& e = engine();
    e.measuring.store(on);
    if (!on) {
        // 关掉就当场清空。留着上一次的读数，UI 那边会显示一个**不会更新的数字**。
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
        // 发布上**立刻**变成不可测（留一个过期数字比不显示更坏），但**窗口先别扔** ——
        // 见 kMeasureRingHold 的理由。
        if (++e.measureMiss >= kMeasureRingHold) e.measureRing.clear();
    } else {
        e.measureMiss = 0;
        e.measureRing.push_back(m.distance);
        while (e.measureRing.size() > kMeasureRing) e.measureRing.pop_front();
        // 中值，不用均值 —— 对偶尔的误吸附鲁棒（STAGE6 §5 同款理由）。
        float v[kMeasureRing];
        size_t n = 0;
        for (float x : e.measureRing) v[n++] = x;
        std::sort(v, v + n);
        out.distance = v[n / 2];
    }

    // 只在「瞄准的东西变了」时打一行：可测性翻了面，或者读数动了 5 cm 以上。
    // 每帧都打会把 logcat 刷爆（99% 是重复的），而且管道写卡一下就是毫秒 —— 这是 SLAM 线程。
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
        // ⚠️ **「一直测不到」要留痕。** 不然它在日志里是一片空白 ——
        // 分不清是开关没开、没候选、还是支持度不够。**这个项目最常见的问题就是它。**
        // 只在**测不到**时打，有读数就走上面那条 ⟹ 正常使用时不会刷屏。
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
    // 十字附近有多少**特征点**（不管有没有地图点）。UI 拿它区分两种情况：
    //   有纹理但没建进图 → 「对着它多停一会儿」；没纹理 → 「换个地方」。
    out7[6] = static_cast<float>(e.measure.texture);
    return e.measure.quality;
}

// ---------------- 多点折线 ----------------

namespace {
/** 投影不可用（没位姿 / 跟踪不是 OK）—— **只在原因变化时打一行**，不刷屏。 */
void logNoPose(int why) {
    static int last = 0;
    if (why == last) return;
    last = why;
    if (why == 1) printf("[M6] ⚠️ 投影不可用：跟踪不是 OK（折线整条不画）\n");
    if (why == 2) printf("[M6] ⚠️ 投影不可用：还没有位姿（折线整条不画）\n");
    fflush(stdout);
}

/** 点的上限。也给 `addPinned` 当软上限。 */
constexpr int kMaxPinned = 128;

/** 相机后面的点插值到这儿。**取多少只决定它落多远，不改变方向。** */
constexpr float kNearZ = 0.02f;

/** 像素夹取 —— 贴着相机平面的点会算出天文数字。 */
constexpr float kProjClamp = 20000.f;

/** 内参。第一次 [getPinned] 时懒读。**只在主线程碰。** */
struct Intr {
    float fx = 0, fy = 0, cx = 0, cy = 0;
    bool ok = false;
};
Intr gIntr;

/** 世界 → 相机。列主序：元素 (r,c) 在 [c*4+r]；pc = Rcw·pw + tcw。 */
inline Eigen::Vector3f toCam(const float t[16], const float* w) {
    return Eigen::Vector3f(
        t[0] * w[0] + t[4] * w[1] + t[8] * w[2] + t[12],
        t[1] * w[0] + t[5] * w[1] + t[9] * w[2] + t[13],
        t[2] * w[0] + t[6] * w[1] + t[10] * w[2] + t[14]);
}

/** 相机系的一点 → 图像像素。**z 必须 > 0**。 */
inline void project1(const Eigen::Vector3f& p, float* xy) {
    xy[0] = std::min(std::max(gIntr.fx * p.x() / p.z() + gIntr.cx, -kProjClamp), kProjClamp);
    xy[1] = std::min(std::max(gIntr.fy * p.y() / p.z() + gIntr.cy, -kProjClamp), kProjClamp);
}

/**
 * 相机后面的 `back`，沿线段朝前面那头插值到 `z = kNearZ`。
 *
 * ⚠️ 方向**必须由两端共同决定**（点的屏幕位置只由视线方向决定，单看它推不出线段往哪走）。
 * 把 z 夹成一个常数是错的 —— 那等于宣称两端同深度，真机上画过横跨屏幕的线。
 */
Eigen::Vector3f pullToFront(const Eigen::Vector3f& back, const Eigen::Vector3f& front) {
    const float dz = front.z() - back.z();
    if (dz <= 1e-6f) return back;   // 退化；下面会判掉
    return back + ((kNearZ - back.z()) / dz) * (front - back);
}

/** 一条线段在画面上的两个端点。**两端都在相机后面 ⟹ false（这条不画）**。 */
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
}  // namespace

bool addPinned(int snapTo) {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);

    if (e.pinned.size() >= kMaxPinned) return false;

    Engine::Pinned p;
    if (snapTo >= 0) {
        // 吸到已有点：**复制它的身份**，不是用十字当前命中的那个
        //（那个只是碰巧很近的另一个地图点，之后会各漂各的）。
        if (snapTo >= static_cast<int>(e.pinned.size())) return false;
        const Engine::Pinned& src = e.pinned[snapTo];
        p.mp = src.mp;
        p.pw[0] = src.pw[0];
        p.pw[1] = src.pw[1];
        p.pw[2] = src.pw[2];
        p.low = src.low;
    } else {
        if (e.measure.quality == kNone) return false;  // 没读数就没什么可钉的
        p.mp = e.measure.mp;
        p.pw[0] = e.measure.wx;
        p.pw[1] = e.measure.wy;
        p.pw[2] = e.measure.wz;
        p.low = (e.measure.quality != kGood);
    }

    // 断开之后，下一个点就是**新一条折线**的起点。**这一侧自己判** ——
    // 于是 Kotlin 一条链状态都不用存，全读回去（两边不会对不上）。
    p.startsChain = e.pinned.empty() || e.pinned.back().closed;
    e.pinned.push_back(p);
    return true;
}

void undoPinned() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);
    if (!e.pinned.empty()) e.pinned.pop_back();
}

void clearPinned() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);
    e.pinned.clear();
}

void closePinnedChain() {
    Engine& e = engine();
    std::lock_guard<std::mutex> lk(e.measureMutex);
    if (!e.pinned.empty()) e.pinned.back().closed = true;
}

int getPinned(float* out, int cap) {
    // 布局：`1 + 3n + 5(n-1) + 5` = `8n + 1`；`(cap-1)/8` 是能装下的点数
    //（`-1` 留给 `liveLen` 那块，用它算上界才不越界）。
    if (out == nullptr) return 0;
    Engine& e = engine();

    if (!gIntr.ok) gIntr.ok = cameraIntrinsics(&gIntr.fx, &gIntr.fy, &gIntr.cx, &gIntr.cy);

    // ⚠️ `poseMutex` 和 `measureMutex` **不嵌套**：先把位姿拷出来、放锁，再拿测量那把。
    // ⚠️ **必须是 `t_cw`，不是 `pose`**（`pose` 是 Twc，方向相反）—— 拿它当 Tcw 用的话
    // 每个点的 z 都是负的，全部被判成「在相机后方」，一个标记都画不出来，而且一声不吭。
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
        // 跟踪不是 OK 时 `t_cw` 是**上一帧 OK 时的旧值** —— 拿它投影会把标记画到
        // 错的地方，看起来像「贴在屏幕上不动」。
        havePose = false;
        logNoPose(sys == nullptr || sys->GetTrackingState() != kTrackingOk ? 1 : 2);
    }

    std::lock_guard<std::mutex> lk(e.measureMutex);
    const int n = std::min(static_cast<int>(e.pinned.size()),
                           std::min((cap - 1) / 8, kMaxPinned));
    if (n <= 0) return 0;
    const int segBase = 1 + 3 * n;

    // ① 每个点的相机系坐标 + 刷新世界坐标。
    //    刷新是这一整套东西存在的理由：**BA 一直在挪地图点**，存快照的话走一圈就和场景错开。
    //    没身份（平面兜底来的）或已失效的，只能留上一份。
    Eigen::Vector3f pc[kMaxPinned];
    for (int i = 0; i < n; ++i) {
        Engine::Pinned& p = e.pinned[i];
        if (p.mp != nullptr && !p.mp->isBad()) {
            const Eigen::Vector3f w = p.mp->GetWorldPos();
            p.pw[0] = w.x();
            p.pw[1] = w.y();
            p.pw[2] = w.z();
        }
        pc[i] = havePose ? toCam(t, p.pw) : Eigen::Vector3f(0, 0, -1);
    }

    // ② 标记点 + flags（bit0 在相机前方 / bit1 之后断开 / bit2 不太准）
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

    // ③ 每段：**两个屏幕端点 + 长度**。几何全在这儿算完，Kotlin 只管画。
    for (int i = 0; i < n - 1; ++i) {
        float* o = out + segBase + 5 * i;
        if (e.pinned[i + 1].startsChain || !segPixels(pc[i], pc[i + 1], o)) {
            o[0] = o[1] = o[2] = o[3] = 0.f;
            o[4] = -1.f;   // < 0 ⟹ 这条不画
            continue;
        }
        o[4] = dist3(e.pinned[i].pw, e.pinned[i + 1].pw);
    }

    // ④ 橡皮筋：最后一个点 → 当前十字命中点。断开 / 没读数 / 两端都在后面 ⟹ -1。
    // ⚠️ 没读数时 `e.measure.wx/wy/wz` 是 0 —— 拿它算就是从最后一个点量到**世界原点**。
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

void setMapCloudEnabled(bool on) {
    Engine& e = engine();
    if (e.mapCloudOn.exchange(on) == on) return;   // 没变过，什么也不用做
    if (on) e.mapSnapAtMs.store(0);                // 重开：下一个空隙立刻重建，不等满 1 s
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
    // `时间(秒) tx ty tz qx qy qz qw`，空格分隔。列和 `SaveTrajectoryEuRoC` 一样，
    f << std::setprecision(9);
    for (int i = 0; i < n; ++i) {
        const float* p = e.trajFull.data() + 8 * i;
        f << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " "
          << p[4] << " " << p[5] << " " << p[6] << " " << p[7] << "\n";
    }
    printf("[M5] 轨迹导出 %d 条 → %s\n", n, path.c_str());
    fflush(stdout);
}

}  // namespace slam
