// 把 C++ 世界的 stdout/stderr 接进 logcat（tag=slam）。
//
// 为什么必须做：ORB-SLAM3 的进度和错误全走 cout/cerr，而 Android 上这两个默认进
// /dev/null —— 不接出来它就是个哑巴。更坏的是它用 exit(-1) 报错（不走 Java 异常
// 路径），表现是「App 整个消失、logcat 干干净净」，没有日志就无从下手。
#include "log_redirect.h"

#include <android/log.h>
#include <poll.h>
#include <pthread.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

namespace slamlog {
namespace {

constexpr char kTag[] = "slam";

// logcat 单条上限约 4 KB（含 tag/pid 前缀），超了会被静默截断。留足余量。
// 它同时是攒行缓冲的上限（见 logThread）：两处用同一个数，切点才刚好重合。
constexpr size_t kMaxChunk = 3000;

// 攒了半行（还没见到换行符）时最多等这么久就吐出去，否则不带 endl 的那句永远看不见。
// 不能太短：cerr 是逐段 flush 的，`cerr << a << b << endl` 会分成好几次 write。
constexpr int kFlushMs = 200;

int gReadFd = -1;

// 按 kMaxChunk 切开再写。n == 0 时也写一条空日志，这样输出里的空行能保留。
void emit(const char* s, size_t n) {
    char buf[kMaxChunk + 1];
    size_t off = 0;
    do {
        size_t len = (n - off > kMaxChunk) ? kMaxChunk : n - off;
        memcpy(buf, s + off, len);
        buf[len] = '\0';
        __android_log_write(ANDROID_LOG_INFO, kTag, buf);
        off += len;
    } while (off < n);
}

// 按行重组：管道给的是字节流，一次 read 可能拿到半行或好几行。
//
// 用 __android_log_write 而不是 Java 的 Log：这是个 pthread，不是 JVM 创建的线程，
// 没有 JNIEnv 也没有 ClassLoader。__android_log_write 是纯 C 函数，绕开了这些。
void* logThread(void* /*arg*/) {
    std::string pending;
    char buf[1024];
    for (;;) {
        pollfd p{gReadFd, POLLIN, 0};
        int r = poll(&p, 1, kFlushMs);
        if (r < 0) {
            if (errno == EINTR) continue;
            return nullptr;                       // 不该发生；线程没了就退化成哑巴
        }
        if (r == 0) {                             // 超时：把没换行的残留吐出去
            if (!pending.empty()) {
                emit(pending.data(), pending.size());
                pending.clear();
            }
            continue;
        }
        ssize_t n = read(gReadFd, buf, sizeof buf);
        // 信号打断不算结束。线程一死管道就没人排空，写满 64 KB 之后任何往 stdout
        // 写的地方（包括 ORB-SLAM3）会永久阻塞。
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return nullptr;                // 写端全关（fd 1/2 一直在，走不到）
        pending.append(buf, static_cast<size_t>(n));
        // 切点有两个：见到换行，或者攒够一整块。
        size_t start = 0;
        for (size_t i = 0; i < pending.size(); ++i) {
            const size_t len = i - start + 1;
            if (pending[i] == '\n') {
                emit(pending.data() + start, len - 1);   // 换行本身不进日志，logcat 自己断行
                start = i + 1;
            } else if (len == kMaxChunk) {               // 攒够一整块还没换行，硬切
                emit(pending.data() + start, len);
                start = i + 1;
            }
        }
        pending.erase(0, start);
    }
}

}  // namespace

void install() {
    // 前提：0/1/2 是开着的（App 进程由 zygote 建好），所以 pipe 拿到两个 >= 3 的 fd。
    // 万一不是就放弃 —— 继续下去的话 dup2 会把读端也顶掉，变成「日志线程读自己的写端」。
    int fds[2];
    if (pipe(fds) != 0) {
        __android_log_write(ANDROID_LOG_ERROR, kTag, "[M2] 日志重定向没装上：pipe() 出错");
        return;
    }
    if (fds[0] <= STDERR_FILENO || fds[1] <= STDERR_FILENO) {
        close(fds[0]);
        close(fds[1]);
        __android_log_write(ANDROID_LOG_ERROR, kTag,
                            "[M2] 日志重定向没装上：pipe() 拿到的 fd 号不合格");
        return;
    }

    gReadFd = fds[0];
    dup2(fds[1], STDOUT_FILENO);
    dup2(fds[1], STDERR_FILENO);
    close(fds[1]);                       // 写端现在由 fd 1/2 持有，这个副本多余

    // 行缓冲。不设的话 stdout 会攒到几 KB 才吐一次，日志看起来是
    // 「卡住不动，然后突然一大坨」。stderr 本来就是无缓冲的，写出来表明意图。
    setvbuf(stdout, nullptr, _IOLBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    pthread_t t;
    if (pthread_create(&t, nullptr, logThread, nullptr) == 0) {
        pthread_detach(t);               // 活到进程结束，不用 join
    }

    // 走 printf，不走 __android_log_write —— 要证明的正是「fd 1 被掉包了」这件事。
    // logcat 里搜 tag `slam` 看到它，才说明后面 ORB-SLAM3 的 cout/cerr 也接得出来。
    printf("[M2] stdout/stderr 已接进 logcat，tag=%s\n", kTag);
}

}  // namespace slamlog
