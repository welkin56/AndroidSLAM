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

#include <atomic>
#include <chrono>
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

/** 一条轨迹记录。字段和 `SaveTrajectoryEuRoC` 写的 8 列一一对应。 */
struct TrajPoint {
    double t = 0;
    float x = 0, y = 0, z = 0;
    float qx = 0, qy = 0, qz = 0, qw = 1;
};

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
    float pose[16] = {0};
    bool poseValid = false;

    std::mutex trajMutex;
    std::vector<TrajPoint> traj;
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
    printf("[M5] System 构造完成，耗时 %lld ms\n", (long long)ms);
    fflush(stdout);
}

void savePose(Engine& e, const TrackResult& r) {
    {
        std::lock_guard<std::mutex> lk(e.poseMutex);
        std::memcpy(e.pose, r.pose16, sizeof(e.pose));
        e.poseValid = true;
    }

    TrajPoint p;
    p.t = r.t;
    p.x = (float)r.x; p.y = (float)r.y; p.z = (float)r.z;
    p.qx = (float)r.qx; p.qy = (float)r.qy; p.qz = (float)r.qz; p.qw = (float)r.qw;
    std::lock_guard<std::mutex> lk(e.trajMutex);
    e.traj.push_back(p);
}

// 常驻循环：先构造，然后一直取帧、跟踪、存位姿。M4 那版是「构造完就退出」。
void worker(Engine& e) {
    construct(e);
    if (e.state.load() != State::kReady) return;

    ORB_SLAM3::System* sys = e.sys.load();
    bool haveBase = false;
    double base = 0.0;

    for (;;) {
        Frame f;
        {
            std::unique_lock<std::mutex> lk(e.qMutex);
            e.qCv.wait_for(lk, std::chrono::milliseconds(kWaitMs),
                           [&] { return !e.frames.empty(); });
            if (e.frames.empty()) continue;   // 超时：回头看看还有没有别的事
            f = std::move(e.frames.front());
            e.frames.pop_front();
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

        // 注意这里**没有**和 PC 那边的帧集完全对齐：PC 的 `SaveTrajectoryEuRoC` 是从
        // `mlRelativeFramePoses` 存的（只含被插入的帧），而我们是每帧存。
        // 对照比的是**形状和尺度**，条数对不齐没关系。
        if (r.valid) savePose(e, r);
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

void start() { engine().running.store(true); }

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

void dumpTrajectory(const std::string& path) {
    Engine& e = engine();
    std::vector<TrajPoint> copy;
    {
        std::lock_guard<std::mutex> lk(e.trajMutex);
        copy = e.traj;
    }

    std::ofstream f(path);
    if (!f.is_open()) {
        printf("[M5] 轨迹写不出去：%s\n", path.c_str());
        return;
    }
    // 格式和 `SaveTrajectoryEuRoC` 逐列对齐（`System.cc:622`）——
    // `时间(秒) tx ty tz qx qy qz qw`，空格分隔。**故意不用逗号**：
    // 这样和 PC 跑出来的 CameraTrajectory.txt 就是同款，对照时直接 diff。
    f << std::setprecision(9);
    for (const TrajPoint& p : copy) {
        f << p.t << " " << p.x << " " << p.y << " " << p.z << " "
          << p.qx << " " << p.qy << " " << p.qz << " " << p.qw << "\n";
    }
    printf("[M5] 轨迹导出 %zu 条 → %s\n", copy.size(), path.c_str());
    fflush(stdout);
}

}  // namespace slam
