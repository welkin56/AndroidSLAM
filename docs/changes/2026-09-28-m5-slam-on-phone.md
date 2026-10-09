# 2026-09-28 阶段五 M5：让 SLAM 在手机上跑起来（设备侧）

> ⚠️ **这一步还没完**：PC 侧那个「标 1.6 米量尺度」的小工具还没做，
> 验收 ②③ 也没跑。**本文件只记设备侧这一半**，后面补齐了再追加。

> ⭐ **这一步踩了两个重大 bug，缺一不可**（真机 `SIGSEGV`，定位花了几轮）：
>
> | # | bug | 详述 | **单独的坑文档**（可复用） |
> |---|---|---|---|
> | 1 | **SLAM 线程只有 1 MB 栈**，而 `Frame` 一个临时对象就要 146 KB | §6 | [bugs/thread-stack-too-small.md](../bugs/thread-stack-too-small.md) |
> | 2 | **进程里有两份 OpenCV**，`cv::parallel_for_` 背后各带一套 TBB 全局状态 | §7 | [bugs/duplicated-static-lib-symbols.md](../bugs/duplicated-static-lib-symbols.md) |
>
> **两个都必需** —— 修任何一个都不够（做过 ablation：删掉 #1 立刻又崩）。
> 教训写在 §6 末尾：**「修了 A 之后还坏」只能说明 A 不充分，不能说明 A 不必要。**
>
> 下面 §6/§7 记的是**这一次**的定位过程；**这两个坑的通用形状、怎么认出来、
> 什么情况下还会撞上**，写在 `bugs/` 那两份里。

## 动机

M4 上设备验过（`System` 构造成功，词袋**实测 37.7 s**），「UI 外壳：三个页面」也验过了。
下一步是让 SLAM **真的在手机上边跑边算**。

**里程碑重新划过**（用户定的）：原 M5（喂帧）+ M6（喂 IMU）+ M7（对照 PC）
**合成一节**，`M6`/`M7` 编号不再使用。两条理由：

1. **IMU 一起接。** `IMU_MONOCULAR` 模式下跑纯视觉**不是它的设计用途** ——
   那个中间态（尺度任意、状态停在 `NOT_INITIALIZED`）不可信，拿它做验收会误导。
   而且相机和 IMU 本来就是**同一个「启动 SLAM」按钮的两半**。
2. **录制开关也进来**（用户问「不做边跑边录，到时候还验收吗」问出来的）。
   验收要的是「同一段数据」，不边跑边录，手机那次就什么都没留下 —— 而 M6 就得**再走一趟**，
   走一趟不是随手的事（惯性初始化要 15 秒运动激励）。**两趟换一个开关，不划算。**

顺带解决了用户提的一个界面问题：**「启动 SLAM」按完一直显示「SLAM 已启动」，没有停止**。
那是 M4 有意留的（`nativeStop` 还没做），这一步补上。

## 改动

### C++

**`cpp/slam_engine.{h,cpp}` —— 从「构造完就退出」长成一台引擎**

| 加的 | 干什么 |
|---|---|
| 帧队列（有界 4 帧） | 满了**丢最旧** —— §0 的验收判据「SLAM 跟不上时丢帧不积压」 |
| IMU 队列（上限 2000 ≈ 5 s） | 喂帧时按时间戳取一段交给 `TrackMonocular` 的第三个参数 |
| `std::mutex` + `float pose[16]` + `bool valid` | SLAM 线程写、主线程读 |
| `std::atomic<bool> running` | `start/stop` 只开关「吃不吃帧」 |
| 时间基准 | 第一帧的时间戳；**纳秒 → 秒的换算只在 SLAM 线程里做**（§6 约定 2） |
| 位姿历史 `std::vector<TrajPoint>` | 每帧 push 一条，停止时一次性导出 |

**线程从一次性改成常驻循环**：构造 → 一直取帧、跟踪、存位姿。

**`cpp/jni_bridge.cpp`**：加 7 个符号 —— `nativePushImage` / `nativePushImu` /
`nativeStart` / `nativeStop` / `nativeGetPose` / `nativeGetTrackingState` /
`nativeDumpTrajectory`。全部包 `try/catch(...)`。

`nativePushImage` 里用 `GetByteArrayElements` + `Release(..., JNI_ABORT)` ——
**没做零拷贝**（§5 写了理由：`ByteArray` 有三个持有者，复用 buffer 会读到被覆盖的帧）。

### Kotlin

| 文件 | 改了什么 |
|---|---|
| `SlamBridge.kt` | 加 7 个 `external fun` |
| `ImuCollector.kt` | 加 `onSample` 回调，**挂在和落盘同一个地方**（陀螺仪回调里配对那一处） |
| `MainActivity.kt` | **重写**：接相机 + IMU、「启动/停止」两态、抽屉里的录制开关、位姿和跟踪状态、轨迹导出 |
| `activity_main.xml` | 抽屉里加一个能点的「开始录制」+ 一条灰着的「保存地图（M8）」 |
| `SessionWriter.kt` | 加 `withSlam`，写进 `meta.json` |
| `CameraCapture.kt` | 加 `cameraErrorText()` —— 把错误码翻成人话（见决策八） |

### 文档

- **`STAGE5_JNI_BRIDGE.md`** —— M5/M6/M7 三节合成一节（M6 那节删掉），
  验收改成三档，并把几条「读源码得到的核查」写进去
- **`APP_DESIGN.md` §2.2** —— 抽屉里那个录制开关标成已实现
- 本文件 + `README.md` 索引
- 顺带：**「UI 外壳」那篇的「未验证」改成已验证**（用户跑过了）

## 决策

### 一、喂 IMU 用「一个完整样本」，**偏离了 §6 的签名**

§6 写的是 `nativePushImu(tsNs, v0, v1, v2, isAccel)` —— 陀螺和加速度分两次调、由 C++ 配对。
**改成** `nativePushImu(tsNs, wx, wy, wz, ax, ay, az)`，一次一个完整样本。

理由两条：

1. `IMU::Point` 要的就是完整样本（一个时间戳 + 加速度 + 角速度），**配对跑不掉**；
2. **`ImuCollector` 本来就在做这个配对** —— 它在陀螺回调里挑时间最近的加速度配成一条
   （`ImuCollector.kt` 的 `gyroListener`），而且带配对偏差的统计（`recordPairGap`）。
   在 C++ 里再配一遍 = 抄一份更差的。

**附带的收益**：它顺手关掉了「加速度和角速度写反」那个静默的坑（见决策二）。

### 二、⚠️ 记一个静默的坑：`IMU::Point` 的参数顺序

| | 顺序 |
|---|---|
| ORB-SLAM3（`ImuTypes.h:49`） | `IMU::Point(acc_x, acc_y, acc_z, ang_vel_x, ang_vel_y, ang_vel_z, t)` |
| 我们落盘用的 `ImuItem` | `(tsNs, wx, wy, wz, ax, ay, az)` |

**正好相反。** 照直觉对过去会写反，而**它不报任何错** —— 只是跟踪整个乱掉，
和「接线没接好」长得一模一样。所以验证清单里专门有一条症状对照。

### 三、轨迹存 **`Twc`**，不是 `Tcw`

去核了 `SaveTrajectoryEuRoC`（`System.cc:616-622`）：它构造 `Tcw` 之后**取逆**再写。
**和它对齐，验收 ③ 才能直接 diff** —— 写反了对照会看着像旋转错位，白查一轮。

格式也照它：`时间(秒) tx ty tz qx qy qz qw`，**空格分隔**（虽然文件名是 `.txt`，
`APP_DESIGN` §9 写的是 `.path.csv` —— 选空格是为了直接 diff，比逗号有用）。

### 四、位姿历史留在 C++，不每帧过一遍 JNI

每帧调一次 `nativeGetPose` 再写文件 = 每帧一次 JNI + 一次 I/O ——
**而那是在相机回调里干重活，正是铁律 1 禁止的**。留在 C++（10 分钟才 1.1 MB），
停止时一次性导出。

**这也是那个预留的「取快照」接口的一次小练习** —— 2D 地图以后要的是同一件事。

### 五、队列小、满丢最旧

帧队列只有 4 帧。「SLAM 跟不上时丢帧不积压」是 §0 的验收判据，
而攒着的代价是**延迟越滚越大**（喂进去的帧越来越旧）。丢最旧的 =
晚到的帧才是有用的。

### 六、`start/stop` 不销毁 `System`

§5 决策一。相机/IMU 是**耗电**的，跟着用户意图走；`System` 是**耗时**的（38 秒），
跟着进程走。所以「停止」之后再「启动」是**立刻**可用的。

`stop()` 里顺带**清空两个队列** —— 不清的话下次启动会吃到停止前积压的旧帧，
而那些帧的时间戳已经落后一大截，跟踪会当场乱掉。

### 七、等 `kReady` 之后再开相机和 IMU

词袋加载那 38 秒里灌帧没人消费，队列满了纯属白丢。所以 ticker 看到
`state == kReady` 的那一刻才开硬件。

**ticker 的退出条件因此要改**：原先「不是 `kLoading` 就 return」会在刚变
`kReady` 时停下 —— 而那一刻正是要开硬件、之后还要一直刷位姿的时候。

### 八、相机错误码翻成人话（用户问出来的）

用户问「**不结束 SLAM 直接返回，然后去开录制模式**」会怎样。查下来：

- **按 Back 就等于「停止」** ✓（`onStop` → `stopHardware`）
- **但关相机是两段异步叠的**：`CameraCapture.stop()` 是 `handler.post`，
  而 Camera2 的 `close()` **本身也是异步**的 —— 发出去 ≠ 硬件已放开
- 所以「按 Back → 立刻点录制模式」有**小概率**撞上 `ERROR_CAMERA_IN_USE`

**用户决定：不修，写进文档。** 结构性根治也做不到 —— Camera2 没有
「我已经放开了」这个信号可等。出路是再按一次「开始准备」。

**但顺手修了一条**：原来那条错误信息是 `相机打开失败 error=1`，**谁也看不懂** ——
而恰恰这个场景下需要知道「是被占用还是坏了」。现在翻成人话，并直接给出路。

> 修的时候自己踩了一个：那几个常量挂在 `CameraDevice.StateCallback` 上，
> 写成 `CameraDevice.ERROR_CAMERA_IN_USE` **编不过**（Kotlin 不许通过外部类访问
> 嵌套类的静态成员）。用 `javap` 翻 `android.jar` 才确认。
> 顺带发现 `ERROR_MAX_CAMERAS_IN_USE` **不含** `ERROR_CAMERA` 子串，第一次 grep 漏了它。

### 九、轨迹只在「主动停止 / 退出」时导出，切后台不导

C++ 那边的位姿历史是**跨「停止/启动」累积**的（进程死才清），
所以切后台不导不会丢东西 —— 反而能攒成一份完整的。

## 验证

### 1. C++ 交叉编译通过，13 个 JNI 符号逐字对上

```
$ ninja -j4        → 退出码 0，我们自己的文件零警告

$ llvm-nm -D libslam_jni.so | grep Java_...  →  13 个
diff <(C++ 导出) <(Kotlin 声明)  →  ✓ 一个不多一个不少
```

（过程中改了一处编译错：`Sophus::SE3f` **没有 `empty()`**。
而 PC 那个示例**根本不用返回值**（`SLAM.TrackMonocular(...)` 连变量都不接），
所以没有现成判据可抄 —— 改用 `Tcw.matrix().allFinite()`，
初始化之前 `Frame::mTcw` 是没赋过值的 Eigen 对象，可能是 NaN/Inf。）

### 2. 静态检查（Kotlin 没过编译器，所以逐项查）

- 六个 `.kt` 的 `{}` `()` 全平（第一次查出来多 3 个 `)`，追下去是我注释里的 `1)` `2)` `3)`）
- 每个 Activity 的 `R.id` ↔ 布局定义的 id 一个不多一个不少

> ⚠️ **静态检查漏了一个，是用户编译时抓到的**：
>
> 给 `CameraCapture.lastError` 加那个错误码翻译函数时，我只锚定了
> `var lastError: String? = null` **这一行**，**没看它后面还跟着一行 `private set`**
> —— 于是函数被插进属性声明和它的 setter 中间，`private set` 悬空：
>
> ```
> CameraCapture.kt:122:17 Expecting member declaration
> ```
>
> **这是本次会话同类错误的第二次**（前一次是替换代码块时把 `toString` 收尾的 `}` 一起吃掉了）。
> 两次根因一样：**只盯着要改的那几行，没看它后面粘着什么**。
> **括号平衡检查抓不到这个** —— 因为它两侧都平衡。

### 3. Gradle 构建通过、已装上设备（2026-09-28）

```
BUILD SUCCESSFUL in 28s
Installed on 1 device.
```

（唯一的 warning 是 `CameraCapture.kt:308` 的 `createCaptureSession` 弃用 ——
**原有的**，不是这次引入的。）

### 4. 上设备后用户抓到的两个 bug（2026-09-28）

**① 「准备 SLAM 之后什么都没做，它自己把相机开了」**

ticker 里有一句 `if (slamState == SLAM_READY) openHardware()` ——
把「就绪」和「启动」两个阶段黏成了一个。**改三段式标签时漏改了这里。**

**② 「加载完之后界面卡住：时间不走、按钮一直是『加载中…』」**

```kotlin
val busy = slamState == SLAM_LOADING || slamRunning
if (!busy) return          // ← 先判断
render()                   // ← 后刷新（永远轮不到）
```

「词袋加载完」那一轮读到 `READY` → `busy` 算成 `false` → **在 `render()` 之前就 return**。
**最后一轮刷新被自己吃掉了。**

> ⚠️ **这个 bug 从写 ticker 的那一版就在**，不是改标签引来的。
> 形状是通用的：**「循环退出判断」放在「本轮副作用」之前，最后一轮的副作用就会丢。**
> 凡是 `if (done) return; doSomething()` 这种顺序，都要问一句
> 「最后一轮那个 `doSomething` 还需要吗」。
>
> 连带修的第三个：`openHardware()` 之后没叫 `uiQueueTicker()` ——
> ticker 在就绪那一刻就停了，而位姿靠它刷，所以手动点「启动 SLAM」位姿也不动。

### 5. **还没做的**

- **验收 ②③**：PC 侧那个小工具**还没做**，1.6 米和对照都没跑
- **跟踪状态在 `NOT_INITIALIZED` ⇄ `OK` 之间反复** —— 见「遗留」，下次专门研究

### 6. 定位过程：从 SIGSEGV 到根因

> ⚠️ **这一节里的临时手段都已经清掉了。** 留在这里是因为**它们是怎么找到根因的** ——
> 下次再撞上「tombstone 只有 1 帧栈」的崩溃，照着走一遍就行。
> 崩溃处理器的代码附在本节末尾。

真机上撞了一次 **`SIGSEGV / SEGV_ACCERR`**，tombstone **只有 1 帧栈**，
符号化还落在 STL 内部（被链接器的同形函数合并误导，指向了一个单目下根本不会走的
双目函数）。查不出来，所以依次用了三招：

| # | 在哪 | 是什么 | 最后 |
|---|---|---|---|
| 1 | `slam_engine.cpp` 的 `kSlamThreadStack` + `init()` 里的 `pthread_create` | **SLAM 线程栈从默认 1 MB 改成 8 MB**；顺带从 `std::thread` 换成 `pthread`（`std::thread` 没有设栈的接口） | ✅ **必需**（做过 ablation，见下） |
| 2 | `slam_engine.cpp` 的 `worker()` 里 `[M5dbg]` 那两段 | 前 10 帧打印输入（`w/h/gray/t/imu`）和返回 | 已清 |
| 3 | `slam_engine.cpp` 的 `crashHandler` + `installCrashHandler` | 自己展开栈、打印 `xxx.so+0x偏移`，然后 `raise()` 重抛让系统照常出 tombstone | 已清（**代码见下**） |

**第 2 招排除了「数据喂错了」**：

```
[M5dbg] 帧 0  w=640 h=480 gray=307200 t=0.000000 imu=143 step=640
```

`gray` 正好是 640×480、`step` 对、IMU 143 条也对（相机比 IMU 晚启动约 0.36 s，
402 Hz × 0.36 ≈ 145）。**输入没问题**，所以问题在里面。

**第 3 招直接给出了 14 帧栈** —— 见 §7。

> **第 1 条：删了又加回来 —— 一次真正的 ablation。**
>
> 它当时是我最重的一个怀疑，理由也硬：`sizeof(Frame) = 145.9 KB`（其中
> `mGrid` + `mGridRight` 占 144 KB），而 `Tracking::GrabImageMonocular` 里写的是
> `mCurrentFrame = Frame(...)` —— **一个 146 KB 的临时对象，在栈上**；
> PC 上跑 `mono_inertial_euroc` 用的是**主线程（8 MB）**，而 Android 上
> `std::thread` 给的是**默认 1 MB**。
>
> **⚠️ 然后我把它删了，理由是一个推理错误**：我说「改成 8 MB 之后仍然崩，所以它
> 不是崩因」—— 但**那时两份 OpenCV 的问题还在**。两个原因叠着，**修任何一个都不够**；
> 「这个不充分」不等于「这个不必要」。我把这两件事混了。
>
> **删掉之后 App 立刻又崩** —— 于是它被加回来，**好了**。这一次是真的 ablation：
> 有它 → 好，没它 → 崩。
>
> **所以这次崩溃是【两个原因叠加】，缺一不可：**
> 1. **SLAM 线程只有 1 MB 栈**（`Frame` 一个临时对象就 146 KB）
> 2. **进程里有两份 OpenCV**（§7）
>
> 教训：**「修了 A 之后还坏」只能说明 A 不充分，不能说明 A 不必要。** 要判断必要性，
> 得在**其他变量都干净**的前提下单独摘掉它试。

#### 附录：崩溃处理器（备查）

下次再遇到「tombstone 只有 1 帧栈」就把它加回去。

**⚠️ 不能用 `backtrace()`** —— bionic 的 `<execinfo.h>` 里它标着 `__INTRODUCED_IN(33)`，
而 `minSdk` 是 26，**声明都看不到，编不过**。用编译器自带的 `_Unwind_Backtrace`
（所有 API 级都有）+ **`dladdr`** 拿 `.so` 名和加载基址：

```cpp
#include <dlfcn.h>
#include <signal.h>
#include <unistd.h>
#include <unwind.h>

struct UnwindState { void* frames[64]; int n = 0; int skip = 1; };

_Unwind_Reason_Code unwindCb(_Unwind_Context* ctx, void* arg) {
    auto* st = static_cast<UnwindState*>(arg);
    if (st->skip > 0) { --st->skip; return _URC_NO_REASON; }
    if (st->n >= 64) return _URC_END_OF_STACK;
    st->frames[st->n++] = reinterpret_cast<void*>(_Unwind_GetIP(ctx));
    return _URC_NO_REASON;
}

void crashHandler(int sig, siginfo_t* info, void* /*ctx*/) {
    UnwindState st;
    _Unwind_Backtrace(unwindCb, &st);
    printf("\n!!! 信号 %d  出错地址 %p  栈深 %d\n", sig, info ? info->si_addr : nullptr, st.n);
    fflush(stdout);

    char line[256];
    for (int i = 0; i < st.n; ++i) {
        Dl_info di;
        int len;
        if (dladdr(st.frames[i], &di) != 0 && di.dli_fname != nullptr) {
            const char* b = strrchr(di.dli_fname, '/');
            len = snprintf(line, sizeof line, "  #%02d  %s+0x%zx\n", i,
                           b ? b + 1 : di.dli_fname,
                           static_cast<size_t>(reinterpret_cast<uintptr_t>(st.frames[i]) -
                                               reinterpret_cast<uintptr_t>(di.dli_fbase)));
        } else {
            len = snprintf(line, sizeof line, "  #%02d  %p\n", i, st.frames[i]);
        }
        if (len > 0) write(STDOUT_FILENO, line, static_cast<size_t>(len));
    }

    // 恢复默认动作再重抛 —— 这样系统照常生成 tombstone，两边信息都不丢。
    signal(sig, SIG_DFL);
    raise(sig);
}
```

（`printf` / `snprintf` 严格说不是 async-signal-safe，但崩溃现场只有一次机会 ——
拿不到信息比这点风险更坏。`write` 是安全的。）

**输出的是 `xxx.so+0x偏移`**，拿未 strip 的 `.so` 反查（在
`app/build/intermediates/cxx/<类型>/<哈希>/obj/<abi>/` 下）：

```bash
NDK=~/Android/Sdk/ndk/26.1.10909125/toolchains/llvm/prebuilt/linux-x86_64/bin
"$NDK/llvm-addr2line" -f -C -i -e <那个 .so> 0x偏移
```

> **第 3 条的实现有个坑**：本来该用 `backtrace()`，但 **bionic 的 `<execinfo.h>` 里它标着
> `__INTRODUCED_IN(33)`** —— 而我们 `minSdk = 26`，**声明都看不到，编不过**。
> 改用编译器自带的 `_Unwind_Backtrace`（所有 API 级都有）+ **`dladdr`** 拿 `.so` 名和
> 加载基址，输出 `.so+偏移`，效果一样。

**为什么怀疑栈**：量出来 `sizeof(Frame) = 145.9 KB`（其中 `mGrid` + `mGridRight`
**占 144 KB**），而 `Tracking::GrabImageMonocular` 里写的是

```cpp
mCurrentFrame = Frame(...);     // ← 这是个【临时对象】，在【栈】上
```

**一次构造就要 146 KB 栈。** PC 上跑 `mono_inertial_euroc` 用的是**主线程（8 MB）**，
而 Android 上 `std::thread` 给的是**默认 1 MB** —— 这是个典型的
「PC 上好好的、Android 上就崩」。

**但改成 8 MB 之后仍然崩**（2026-09-28 21:07 那次），所以**栈没被证明是根因** ——
最后查明**根本不是**。**这三招现在都已经从代码里清掉了。**

**已经排除的**：喂进去的数据是好的 ——
`[M5dbg] 帧 0  w=640 h=480 gray=307200 t=0.000000 imu=143 step=640`
（`gray` 正好是 640×480，`step` 对，IMU 143 条也对：相机比 IMU 晚启动约 0.36 s）。

### 7. ⭐ 根因：**进程里有两份 OpenCV**（2026-09-28 深夜）

崩溃处理器给出的 14 帧栈直接指到了病灶：

```
#02  libDBoW2.so+0x1d6408   ← cv::parallel_for_，落在【DBoW2 那个库】里
#03  liborb_slam3.so+0x26bf9a0   cv::GaussianBlurFixedPoint
#04  liborb_slam3.so+0x26c1734   cv::GaussianBlur
#05  liborb_slam3.so+0x1ec9c0c   ORBextractor::operator()   ORBextractor.cc:1133
#06  liborb_slam3.so+0x22bd210   Frame::ExtractORB          Frame.cc:422
#07  liborb_slam3.so+0x22c09d0   Frame::Frame(...)          Frame.cc:311
#08  liborb_slam3.so+0x1e30570   Tracking::GrabImageMonocular
#09  liborb_slam3.so+0x1d3795c   System::TrackMonocular
#10  libslam_jni.so+0x13be54     slam::worker               slam_engine.cpp:311
```

**`cv::GaussianBlur` 从 `liborb_slam3.so` 调进去，执行到的是 `libDBoW2.so` 里那份
`cv::parallel_for_`。**

**为什么会有两份：** `DBoW2` 上游做成了独立的 `.so`，而它
`target_link_libraries(DBoW2 ${OpenCV_LIBS})` 链的是**静态** OpenCV ——
于是 `libDBoW2.so` 里**静态链了一份完整的 `cv::`**，而 `liborb_slam3.so` 里**也有一份**。
两份里都有 **`cv::parallel_for_`** —— 那是 OpenCV 的并行后端分发点，背后带全局状态，
两个副本的状态互相踩。

> 📌 **2026-09-29 更正一处归属：** 上面原文写的是「OpenCV 里带**全局线程池**的东西」。
> 复查发现**这份 OpenCV 4.6.0 编译时开了 TBB 后端**（`libopencv_core.a:parallel.cpp.o`
> 里有 `U createParallelBackendTBB`，而 OpenCV 源码里那行注册在 `#ifdef HAVE_TBB` 下），
> 所以那份全局状态是 **TBB 的**（`theMarket` / `theTLS` / `__TBB_InitOnce::count`，
> 实测在 `.so` 里有 16 个导出的 `B` 符号）。**OpenCV 自己的 `ThreadPool` 在最终 `.so`
> 里连符号名都没有。结论不变，归属改掉。**

> ⚠️ **这条推翻了 M3 里我自己的一个判断。** 当时发现 `libslam_jni.so` 里凭空多出
> 170 个 `cv::` 符号，我写的是「**只是体积问题、不是正确性问题**」。
> **它就是正确性问题** —— 而且代价是一次真机 SIGSEGV、三轮定位。

**改法（三层，缺一不可）：**

1. **`DBoW2` 从 `SHARED` 改成 `STATIC`** —— 它的代码进 `liborb_slam3.so`，
   不再单独成一个库。"两份 OpenCV"里的一份直接消失。
2. **`orb_slam3` 的链接全改 `PRIVATE`**（`DBoW2 g2o` 也进去）——
   **静态库的 `PRIVATE` 链接项照样会传递**（CMake 标成 `LINK_ONLY`），
   所以在 DBoW2 那一层挡不住，得在这一层断。
3. **新增 `orb_track.{h,cpp}`** —— 把「构造 `cv::Mat` + 调 `TrackMonocular`」包一层，
   **编进 `orb_slam3`**（那一侧链着 OpenCV）。这样 `slam_jni` 可以完全不碰 OpenCV：
   `target_include_directories(slam_jni PRIVATE ${OpenCV_INCLUDE_DIRS})` 只给**头文件**
   （`ImuTypes.h` 自己 include OpenCV，编的时候要能找到），但**一个符号都不链**。

**验证（这次是硬的）：**

```
$ llvm-nm -D --defined-only <每个 .so> | grep -c _ZN2cv
  liborb_slam3.so   1175 个    ← 唯一一份
  libslam_jni.so       0 个
  libg2o.so            0 个

$ llvm-nm -D --defined-only <每个 .so> | grep -c _ZN2cv13parallel_for_
  liborb_slam3.so      1 处
  libslam_jni.so       0 处

$ llvm-nm -u libslam_jni.so | grep -c _ZN2cv
  0 个未定义        ← 连引用都没有
```

**`libDBoW2.so` 也不再产出。**

## 遗留

- **⚠️ 跟踪状态在 `NOT_INITIALIZED` ⇄ `OK` 之间反复**（2026-09-28 跑通之后立刻发现的）。
  **不是「跟丢」那种一次性失败 —— 它来回跳。下次专门研究。**
  三条线索，**别再从头查**：

  1. **惯性初始化要运动激励**：静止举着永远初始化不了，需要前 15 秒持续平移
     （`Tracking::mTinit` 那条梯子 + VIBA2）。
  2. **session 要 60 秒以上** —— 阶段三查过：20.3 秒结构性不够，重采 65 秒后
     位姿 37 → 1730 帧、离散度 20 倍 → 0.8%。见
     [2026-09-19](2026-09-19-tracking-reset-diagnosis.md)。
  3. **⚠️ 首要嫌疑：「重置的自我循环」。** 阶段四查明的：`ResetIfRequested()` 会把
     `mTinit` **归零**，而它是**唯一能关掉清图开关的东西** —— 于是**越重置越爬不出来**。
     **「初始化完毕又掉回去」正是这个形状。** 见
     [2026-09-20](2026-09-20-stage4-calibration.md)。

  **M5 还引入了一个 PC 上没有的变量**：帧队列有界（4 帧、满丢最旧）。
  如果 SLAM 处理得比 30 fps 慢，大部分帧会被丢掉，而 **IMU 预积分是按帧间隔算的** ——
  帧率越低、窗口越大、误差越大 → 初始化反复失败。**这条必须实测。**

  **下次先做两件零风险的事**：① 在 `pushImage` 里数丢帧率 ② 在 logcat 里搜
  `Reseting` / `ResetActiveMap` 看有没有频繁重置。
  **别急着调 `mMaxFrames` / `mTinit` 的阈值** —— 那是拿猜去盖现象。

- **PC 侧的「标 1.6 米量尺度」小工具还没做。** 缺的是「像素 → 地图点」那半：
  要改 `mono_inertial_euroc.cc` 把每帧的 `GetTrackedKeyPointsUn()` +
  `GetTrackedMapPoints()` dump 出来，再复用 `tools/calibration/click_points.py` 的点选。
  **做完才能跑验收 ②③。**
- **相机接管的竞态不修**（决策八）。表现是录制页黑屏 + 一条「相机被占用」，
  出路是再按一次「开始准备」。要根治得加重试，现在不加。
- **「停止」期间 `System` 保留但永远不销毁** —— 这是 §5 决策一，不是漏了
- **两个页面的相机装配代码各有一份**（`MainActivity` 这份是照抄 `CaptureActivity` 的）。
  两边都跑起来之后再决定要不要抽公共类
- **2D 地图/轨迹** —— 设计在 `APP_DESIGN` §5；它要的「SLAM 线程上取深拷贝快照」接口
  **这一步也没实现**，只是位姿历史这个模式是同一个套路
- **内存**：跑 20 分钟，记 `Atlas::GetAllMaps().size()`、当前图 KF 数、进程 native 内存
- 保存时机 / `<名字>` / 换图要不要重启进程 —— 都是 M8 的

## 回滚

**整个设备侧都不要**：

1. `cpp/slam_engine.{h,cpp}` 回到 M4 那版（`git diff` 可退），
   `cpp/jni_bridge.cpp` 删掉 7 个新符号
2. `SlamBridge.kt` 删掉 7 个 `external fun`
3. `ImuCollector.kt` 删掉 `onSample` 和它在陀螺回调里的那一处调用，
   把 `val w = writer ?: return` 改回去
4. `MainActivity.kt` 用 `git checkout` 取回 M4 那版
5. `SessionWriter.kt` 删掉 `withSlam` 和 meta.json 里那一行
6. `activity_main.xml` / `CameraCapture.kt` 同上

**只回滚「抽屉里那个录制开关」**：删掉 `btnRecord` 的监听和 `startRecording`/
`stopRecording`/`toggleRecording`，其余不动 —— 喂帧和喂 IMU 不依赖它。
