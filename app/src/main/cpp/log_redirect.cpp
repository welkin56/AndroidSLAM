// 把 C++ 的 stdout/stderr 接入 logcat（tag=slam）。
//
// ORB-SLAM3 的日志走 cout/cerr，Android 默认丢弃；它的一部分错误还直接 exit(-1)，
// 没有这些日志就无法定位。
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

// logcat 单条约 4 KB，留出余量；同时作为攒行缓冲上限。
constexpr size_t kMaxChunk = 3000;

// 未换行的残留最多等这么久；cerr 可能分多次 write，间隔不能设得太短。
constexpr int kFlushMs = 200;

int gReadFd = -1;

// 按 kMaxChunk 切块写入；n == 0 时也写一条空日志，保留空行。
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

// 按行重组字节流；一次 read 可能拿到半行或几行。
// 本线程由 pthread 创建，没有 JNIEnv，只能用 __android_log_write。
void* logThread(void* /*arg*/) {
    std::string pending;
    char buf[1024];
    for (;;) {
        pollfd p{gReadFd, POLLIN, 0};
        int r = poll(&p, 1, kFlushMs);
        if (r < 0) {
            if (errno == EINTR) continue;
            return nullptr;                       // 不应发生；线程退出后日志会静默
        }
        if (r == 0) {                             // 超时：把没换行的残留吐出去
            if (!pending.empty()) {
                emit(pending.data(), pending.size());
                pending.clear();
            }
            continue;
        }
        ssize_t n = read(gReadFd, buf, sizeof buf);
        // 信号打断不算结束。线程退出后管道无人排空，写满 64 KB 会让所有写 stdout 的地方阻塞。
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return nullptr;                // 写端全关（fd 1/2 一直在，走不到）
        pending.append(buf, static_cast<size_t>(n));
        // 切点：换行，或攒够一整块。
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
    // 0/1/2 默认已打开，pipe 应拿到两个 >= 3 的 fd；
    // 否则放弃，避免 dup2 顶掉读端。
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
    close(fds[1]);                       // 写端已由 fd 1/2 持有

    // stdout 设为行缓冲，否则日志会攒成一大块才输出；stderr 本身无缓冲。
    setvbuf(stdout, nullptr, _IOLBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    pthread_t t;
    if (pthread_create(&t, nullptr, logThread, nullptr) == 0) {
        pthread_detach(t);               // 活到进程结束，不需要 join
    }

    // 用 printf 验证 fd 1 已被替换；logcat 中出现这行才说明后续 cout/cerr 也能接出。
    printf("[M2] stdout/stderr 已接进 logcat，tag=%s\n", kTag);
}

}  // namespace slamlog
