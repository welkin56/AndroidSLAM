// 见 slam_engine.h。
//
// 两条纪律都是 ORB-SLAM3 的形态逼出来的：
//
// 1. **绝不 delete System。** 它没有析构函数；`Shutdown()` 也不 join 线程 ——
//    `System.cc` 里等 LocalMapping/LoopClosing 结束的那段被整段注释掉了，只发个
//    `RequestFinish` 就返回。所以「析构」之后线程还在跑、还握着已释放的对象，delete 就是
//    use-after-free。「System 活到进程结束」既是省一次 145 MB 词袋重载，也是唯一安全的做法。
//
// 2. **单例故意泄漏、线程 detach。** 函数内静态对象会在进程退出时析构，而那时去析构一个
//    joinable 的 std::thread 会直接 std::terminate。
//
// 诊断输出走 printf 就行 —— M2 已经把 fd 1/2 接进 logcat 了（tag `slam`），
// 和 ORB-SLAM3 自己的 cout/cerr 落在同一个通道里。
#include "slam_engine.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

#include "System.h"

namespace slam {
namespace {

struct Engine {
    std::atomic<State> state{State::kIdle};
    std::atomic<ORB_SLAM3::System*> sys{nullptr};
    /** 开始构造的时刻（steady_clock 的毫秒，只在本进程内有意义）。给 elapsedMs 用。 */
    std::atomic<long long> startedAtMs{0};

    // 只在启动线程之前赋值，之后只读 —— 所以不用锁。
    std::string vocab;
    std::string settings;
};

long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 故意泄漏，见文件头第 2 条。
Engine& engine() {
    static Engine* e = new Engine;
    return *e;
}

void construct(Engine& e) {
    printf("[M4] 开始构造 System（IMU_MONOCULAR，bUseViewer=false）\n");
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
    // 所以这条 kFailed 只覆盖「抛异常」这种少见情况。
    if (s == nullptr) {
        printf("[M4] System 构造抛异常了\n");
        e.state.store(State::kFailed);
        return;
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    e.sys.store(s);
    e.state.store(State::kReady);
    printf("[M4] System 构造完成，耗时 %lld ms\n", (long long)ms);
    fflush(stdout);
}

}  // namespace

void init(const std::string& vocabPath, const std::string& settingsPath,
          const std::string& loadFromFile) {
    Engine& e = engine();

    State expected = State::kIdle;
    if (!e.state.compare_exchange_strong(expected, State::kLoading)) {
        return;  // 已经调过（或正在加载）。构造一次要几十秒，不能重来。
    }

    if (!loadFromFile.empty()) {
        // ⚠️ M4 不支持载入旧图。而且这里有个结构性的麻烦：`System` 的构造函数**没有**
        // 「从哪个文件载」这个参数 —— 它读的是 yaml 里的 `System.LoadAtlasFromFile`
        // （System.cc:76-98）。所以要真支持，M8 得另想办法（改 yaml 或给 System 加字段）。
        printf("[M4] 收到了 loadFromFile 但 M4 还不支持载入旧图，忽略：%s\n", loadFromFile.c_str());
    }

    e.vocab = vocabPath;
    e.settings = settingsPath;
    e.startedAtMs.store(nowMs());

    // 起来就 detach：它活到进程结束，没人 join 它。
    std::thread(&construct, std::ref(e)).detach();
}

State state() { return engine().state.load(); }

long long elapsedMs() {
    Engine& e = engine();
    if (e.state.load() != State::kLoading) return 0;
    return nowMs() - e.startedAtMs.load();
}

}  // namespace slam
