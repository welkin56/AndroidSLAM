# 2026-09-27 阶段五 M2：把 C++ 的 stdout/stderr 接进 logcat

## 动机

按 [STAGE5](../tutorial/STAGE5_JNI_BRIDGE.md) 的里程碑推进。**M2 必须排在
任何 ORB-SLAM3 相关工作之前**，理由在 §1.4 和 §3.2：

- ORB-SLAM3 的进度和错误**全部**走 `cout` / `cerr`，而 Android 上这两个默认进
  `/dev/null` —— 不接出来它就是个哑巴。
- 更坏的是它用 **`exit(-1)`** 报错（16 处），不走 Java 异常路径。表现是
  「点一下 App 整个退回桌面、**logcat 干干净净**」，而 §7 那套「看信号名 →
  `ndk-stack` 符号化」对它完全无效——**它不是崩溃，是正常退出**。

M1 的空壳 `.so` 已经进 APK 了（见
[2026-09-26-m1](2026-09-26-m1-native-build-and-negative-test.md)）。M2 是在它上面
加日志通道，不碰 ORB-SLAM3。

## 改动

- **`app/src/main/cpp/` 拆成三个文件**（见「决策」最后一条）：

  | 文件 | 职责 |
  |---|---|
  | `jni_bridge.cpp` | JNI 符号层。每个 `Java_...` 只做翻译 + 转发；`JNI_OnLoad` 里调一次 `slamlog::install()` |
  | `log_redirect.{h,cpp}` | 基础设施。`emit()`（分块写）、`logThread()`（按行重组）、`install()`（掉包 fd 1/2，然后 **`printf` 一句自证**） |
  | `CMakeLists.txt` | `add_library(slam_jni SHARED jni_bridge.cpp log_redirect.cpp)` |

  另加 `nativeLogSelfTest()`：`cout` / `cerr` / `printf` / 一条 9 KB 超长行各来一次。

- **`SlamBridge.kt`** —— 加 `external fun nativeLogSelfTest()`。M1 的
  `nativeHello()` **保留**：§7 的排查顺序第一句就是「M1 的字符串还打不打得出」，
  那个探针是排查阶梯的第一级，后面几个月都要用。

- **`MainActivity.kt`** —— `onCreate` 里加一行 `SlamBridge.nativeLogSelfTest()`。
  两个作用：**验证 M2**，以及**让 `.so` 在 App 启动时就加载** ——
  `JNI_OnLoad` 里装的重定向越早装上越好。M7 接线时这一行会被真正的
  `nativePushImage` 取代。

  > **2026-09-27 补记**：验收跑过之后，那一行**已经不在文件里**了 ——
  > 所以有一段时间 `SlamBridge` 在 Kotlin 侧零调用点，`.so` 也因此没被加载过。
  > M4 把调用点加了回来（改用 `nativeHello()`，不往 logcat 灌七行噪音），
  > 见 [2026-09-27-m4](2026-09-27-m4-construct-system.md)。
  > **下面「回滚」一节里那条「删掉 onCreate 里那一行」于是已经不需要执行了。**

- **`docs/tutorial/STAGE5_JNI_BRIDGE.md`** —— M2 一节重写：删掉原来的
  `JNI_OnLoad` 代码片段（已经实现进源文件了，教程不嵌实现），换成设计说明 +
  四个容易漏的细节表 + 真实的验收命令（`adb logcat -s slam`）；M3 的 CMakeLists
  里补上 `log_redirect.cpp`。

## 决策

- **采用：一根管道，stdout/stderr 合并，共用一个 tag `slam`，全部 INFO 级。**
  - **否决：两根管道 + 两个读线程**（stderr 走 ERROR 级，可以 `logcat *:E` 过滤）。
    能按级别过滤是真的好处，但**两个线程一跑，`cout` 和 `cerr` 的先后顺序就没了**。
    排「先干了什么、然后哪一步失败」时，时序比级别要紧得多；而且级别过滤这件事
    用 `grep` 也能近似做到。
- **采用：用 `__android_log_write`，不用 Java 的 `Log`。**
  读管道的是个 pthread，**不是 JVM 创建的线程**——没有 `JNIEnv`，也没有
  ClassLoader，碰 Java 那套要先 `AttachCurrentThread` 再缓存 `jclass` 的 global ref
  （§7 第 6 条）。`__android_log_write` 是纯 C 函数，把这一整类问题绕开了。
- **采用：超长行分块（3000 字节）。** logcat 单条上限约 4 KB（含 tag/pid 前缀），
  超了**静默截断**。旧教程说「看到莫名的截断不是出错」——现在不用忍了。
  代价是切点在字节上，理论上能切开一个多字节字符；日志以 ASCII 为主，接受。
- **采用：攒了半行就超时（200 ms）吐出去。**
  **否决：只认换行符。** 那样一句不带 `endl` 的进度输出会**永远**卡在缓冲区里，
  直到下一次有换行才连带出现——看起来就是「这行日志丢了」。
  **200 ms 这个数**：不能太短，`cerr` 是逐段 flush 的，`cerr << a << b << endl`
  会分成好几次 `write`，太短会把一句话切成好几条日志。
- **采用：`pipe()` 拿到的 fd 必须都 > 2，否则放弃重定向。**
  不检查的话，万一 fd 1 是关着的，`pipe()` 可能返回 `{1, 2}`，于是 `dup2` 会把
  **读端**也顶掉——变成「日志线程读自己的写端」，比哑巴还难查。
  宁可退化也不能这样坏。App 进程的 0/1/2 由 zygote 保证是开着的（通常是
  `/dev/null`），正常走不到这条分支。
- **采用：重定向装在 `JNI_OnLoad`，不等到第一次调用。** 加载期和静态初始化期的
  输出都要能看见（M3 之后 `liborb_slam3.so` 会作为依赖一起被 `dlopen`）。
- **采用：`read` 必须处理 `EINTR`。** 第一版没写，`read` 被信号打断返回 -1 就当成
  「写端关了」让读线程退出 —— 而**线程一死，管道没人排空，写满 64 KB 之后任何往
  stdout 写的地方都会永久阻塞**。那就不是「日志丢了」，是**把 ORB-SLAM3 卡死**。
  写日志的代码不能有杀死自己的失败模式。
- **⚠️ 修正（用户发现的）：攒行缓冲必须封顶，光靠超时兜不住。**
  用户问「输入一直不带换行，`pending` 不会无限增长吗」—— **问对了，会。**
  `poll` 一有数据就立刻返回，所以**超时只在输出有停顿的时候才轮得到**；写方要是
  不停顿地吐字节，200 ms 分支永远不执行，`pending` 就一路涨。原版只在「有 200 ms
  空档」时才安全，而空档取决于对方，不取决于我们。
  修法有个正好可用的巧合：**让 `pending` 的上限等于 `emit` 的分块大小
  （同一个 `kMaxChunk = 3000`）**，硬切点于是和分块点**完全重合** ——
  结果和「整行攒齐再切」一模一样，但内存被卡在 4 KB 以内。
  实测见验证第 2 条：2 MB 不带换行的数据，`pending` 峰值 4016 字节，
  而且是 699 条完整日志，**一个字节没丢**。
  现实里 ORB-SLAM3 输出全是 `endl` 结尾，够不到这个场景；但**我们自己**以后随手写个
  不带 `endl` 的 `cout` 就正好踩上 —— 而**日志工具失效的时候，你正在排查别的问题**。
- **采用（用户提出）：拆文件，但目录保持平铺。** 用户问「要不要拆成多个 .h/.cpp，
  归类为 include 和 src」。
  - **拆文件，是。** 判据是「**是不是两件可以独立讲清楚的事**」：JNI 符号层是机械的
    翻译转发；日志重定向跟 JNI 一点关系都没有（只是恰好被 `JNI_OnLoad` 调用）；
    M3 进来的 `slam_engine` 又是第三种。原来三层挤在一个文件里。
  - **拆 `include/` + `src/`，否。** 那套结构是给**库的使用者**准备的 —— 把公开头
    集中到一处、靠 `target_include_directories` 只暴露 API。**我们的 `.so` 没有 C++
    使用者**：唯一的消费者是 Kotlin，走 JNI，一个头文件都不 include。为一个不存在
    的需求付「每个文件多一层 `..`、四个文件跨两个目录翻」的代价，不值。
  - **顺带解决了一个真问题**：`emit` / `logThread` 原来是文件内的匿名 namespace，
    拆出去就**必须**变成外部符号。做法是套一层 `namespace slamlog`，于是导出的是
    `_ZN7slamlog7installEv` 一个名字，而不是 `emit` / `logThread` 这两个极其容易
    和 ORB-SLAM3 / Boost 撞车的短名字。实测见验证第 3 条。
- **采用：保留 M1 的 `nativeHello()`。** 它是「桥还通不通」的探针，删掉的话
  §7 的排查阶梯就断了一级。

## 验证

**1. 语法检查 + 符号名核对**（用 NDK 的 clang，不需要跑 Gradle）：

```
$ NDK=~/Android/Sdk/ndk/26.1.10909125/toolchains/llvm/prebuilt/linux-x86_64
$ CC="$NDK/bin/clang++ -c --target=aarch64-none-linux-android26 \
      --sysroot=$NDK/sysroot -std=c++17 -Wall -Wextra"
$ $CC -o /tmp/lr.o log_redirect.cpp && echo "log_redirect.cpp 编译通过，零告警"
log_redirect.cpp 编译通过，零告警
$ $CC -o /tmp/jb.o jni_bridge.cpp   && echo "jni_bridge.cpp  编译通过，零告警"
jni_bridge.cpp  编译通过，零告警
```

符号名核对 —— 这是**这一层最容易出错的地方**（§1.2 命名层）：符号必须和 Kotlin 侧
的 `object SlamBridge` + 包名逐字对上，对不上就是 `UnsatisfiedLinkError`。

```
$ "$NDK/bin/llvm-nm" /tmp/jb.o | grep -v " U " | grep -iv "_ZNSt\|GCC_except\|__clang\|DW.ref"
0000000000000000 T JNI_OnLoad
000000000000002c T Java_com_example_slamcapture_SlamBridge_nativeHello
000000000000005c T Java_com_example_slamcapture_SlamBridge_nativeLogSelfTest
0000000000000000 W _ZN7_JNIEnv12NewStringUTFEPKc
```

**只有该导出的三个是导出的**（大写 `T`），没有把 `slamlog::install` 漏出来当
`Java_...` 用，也没有多出别的辅助符号。

**2. 拆行逻辑：把 `logThread` 里那个循环原样抄成一个宿主程序跑**（`__android_log_write`
在 x86 上不存在，但**要验的是切点，不是 logcat**）：

```
整行 9017 字节
切成 4 条，各条长度：3000 3000 3000 17
接起来和原文一致：是
剩余 pending：0 字节（应为 0）

灌了 2 MB 无换行数据
pending 峰值：4016 字节（封顶前会涨到 2 MB）
产出了 699 条日志，最后一条 3000 字节

50 行普通输入 -> 50 条日志（应为 50），pending=0
```

第 1 条是**回归**：封顶之后切点没变，还是 3000/3000/3000/17，接起来和原文逐字节相同。
第 2 条是用户那个问题的答案：2 MB 不带换行，`pending` 峰值 **4016 字节**
（= 3000 封顶 + 一次 1024 字节的 `read`），699 条日志**一个字节没丢**。
第 3 条确认普通多行输入的行数没被这次改动影响。

> 这个测试用宿主 g++ 跑，验的是算法。**它不能替代第 3 条** —— 真机上的
> `read`/`poll` 行为、logcat 的分块上限、`JNI_OnLoad` 的时序都还是没验过的。

**3. 拆文件之后没有多泄漏符号**（拆出去的东西必须变成外部符号，所以这步得看一眼）：

```
$ "$NDK/bin/llvm-nm" /tmp/lr.o | grep -v " U " | grep -iv "_ZNSt\|GCC_except\|__clang\|DW.ref"
0000000000000424 t _ZN7slamlog12_GLOBAL__N_14emitEPKcm
0000000000000000 r _ZN7slamlog12_GLOBAL__N_14kTagE
0000000000000000 d _ZN7slamlog12_GLOBAL__N_17gReadFdE
0000000000000160 t _ZN7slamlog12_GLOBAL__N_19logThreadEPv
0000000000000000 T _ZN7slamlog7installEv
```

**`log_redirect.o` 只导出 `slamlog::install` 一个符号**，其余全是小写（`t`/`r`/`d` =
内部）。`emit`、`logThread` 在里面的匿名 namespace 里，出不去。

这一条不是洁癖：一个进程里所有 `.so` **共用一套动态链接的符号名字空间**，`emit`
这种短名字和 ORB-SLAM3 / DBoW2 / g2o / Boost 撞车的概率不低，撞上就是
**「崩在 A 库、根因在 B 库」**那种最难查的 bug。命名空间把名字拉长到
`_ZN7slamlog7installEv`，撞车概率降到可以忽略。

**4. 未验证 —— 设备上的实际输出。**

**没跑 `./gradlew assembleDebug`，也没装上手机看 logcat**（构建按约定由用户执行）。
所以下面这些**都还没被观测过**，只是「设计成这样」：

- `JNI_OnLoad` 那句「已接进 logcat」是否真的出现；
- `cout` / `cerr` / `printf` 三路是否都通；
- 9 KB 超长行是否分成了 4 条（3000/3000/3000/…）而不是断在半截；
- 不带换行的那行是否**晚约 0.2 s** 出现。

**要跑的**：

```bash
cd /home/welkin/SLAMAPP/android/SlamCapture && ./gradlew assembleDebug
# 装到手机后：
adb logcat -c && adb logcat -s slam
```

冷启动 App，预期按顺序看到：

```
[M2] stdout/stderr 已接进 logcat，tag=slam
[M2] cout 一行（endl 换行并 flush）
[M2] cerr 一行（无缓冲，立刻出来）
[M2] printf 一行
[M2] 超长行开始>0123456789……（4 条，接起来正好是整行，结尾是 <超长行结束）
[M2] 自检结束（下面那行不带换行，晚约 0.2 s 出现才是对的）
[M2] 这一行故意不带换行          ← 时间戳比上面那条晚约 0.2 s
```

**关键回归**：M1 的探针不能被这次改动弄坏 —— 把 `nativeLogSelfTest()` 那行换成
`Log.i(TAG, SlamBridge.nativeHello())`，仍然应该打出 `hello from libslam_jni.so (M1)`。

## 遗留

- ~~**`.so` 的静态初始化输出会漏掉。**~~ **已核（2026-09-27，M4 上设备时）**：
  `JNI_OnLoad` 是 VM 在 `dlopen` **完成之后**才调的，而 `libslam_jni.so` 的依赖
  `liborb_slam3.so` 的静态初始化在这之前就跑完了 —— **如果它在静态初始化里打印什么，
  那几句接不到。** 实测那条 `[M2] stdout/stderr 已接进 logcat` 就是 logcat 的第一条，
  前面什么都没有 —— **ORB-SLAM3 目前确实没有这种输出**。
  ⚠️ 这条限制还在，只是暂时不咬人：哪天它（或我们自己的某个 `.so`）加了静态初始化期的
  打印，就会静默丢掉。见 [2026-09-27-m4](2026-09-27-m4-construct-system.md)。
- **写满了会阻塞。** 管道容量约 64 KB，读线程活着就一直排空，正常不会满。
  `EINTR` 那条路已经堵上了，但**读线程仍可能因为别的不可恢复错误退出**
  （`poll` 返回非 EINTR 的负值、`read` 返回非 EINTR 的负值），那时 `cout` 会
  **阻塞**住调用它的线程。这是「宁可阻塞也不静默丢日志」的选择，但代价要说清楚。
  佐证在 logcat 里：读线程退出后再也没有 `slam` 的行。
- **`setvbuf(stdout, ...)` 在 stdout 已被写过时会失败**（C 标准规定）。`JNI_OnLoad`
  里没人写过，正常成立；万一失败也**没有报错**，只是退化成块缓冲。
- **分块切在字节边界上**，可能把一个多字节字符切开，那一条日志的末尾会出现乱码。
- **`logcat` 只保留环形缓冲区**（默认几 MB），跑得久了早先的日志会被冲掉。
  长时间跑 SLAM 要配合 `adb logcat -s slam > file` 落盘。
- **`nativeLogSelfTest()` 在 `onCreate` 里**，所以任何 JNI 侧的硬错误都会让
  App **一开就闪退**（M1 的负向验证已经确认过这个性质，是**故意的**）。
  M7 接线时这行会被 `nativePushImage` 取代。

## 回滚

**只是不想要自检**（保留日志重定向，这是最常见的一种）：

1. `MainActivity.kt` —— 删掉 `onCreate` 里的 `SlamBridge.nativeLogSelfTest()` 那一行。
2. `SlamBridge.kt` —— 删掉 `nativeLogSelfTest()` 的声明。
3. `jni_bridge.cpp` —— 删掉 `nativeLogSelfTest()` 这个函数。

只回滚第 1 步的话，症状是「日志还是出不来」，但不会崩 —— 因为 `JNI_OnLoad` 照样会
在首次访问 `SlamBridge` 时装上重定向。

**整个 M2 都不要**（回到 M1 状态），在第 1~3 步之外再做两件：

4. 删掉 `log_redirect.h` / `log_redirect.cpp`，`CMakeLists.txt` 的 `add_library`
   只留 `jni_bridge.cpp`。
5. `jni_bridge.cpp` 的 `JNI_OnLoad` 恢复成只 `return JNI_VERSION_1_6`。

**M2 和这次拆文件是两个独立的改动**：想留 M2 但退回单文件的话，把
`log_redirect.cpp` 的内容并回 `jni_bridge.cpp` 的匿名 namespace 即可，其余不动。

所有文件都在 `android/SlamCapture/` 这个仓库里，`git diff` 可退
（`log_redirect.{h,cpp}` 是新增文件，要用 `git clean` 或手动删）。
`docs/` 那边只有教程 M2 一节和本文件，同样可退。
