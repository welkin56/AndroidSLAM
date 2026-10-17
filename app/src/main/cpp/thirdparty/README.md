# thirdparty —— 这个目录里为什么有这些东西

`app/src/main/cpp/` 下是我们自己写的 native 代码（JNI 桥、日志重定向、md5，
以后还有 `SlamEngine`）。这个子目录只放**别人写的**东西。

## 分法：什么自带，什么外装

判据借的是 ORB-SLAM3 自己的那套（见 `ORB_SLAM3/Thirdparty/` 的构成）：

| | 自带，进这个目录 | 外装，靠系统装或自己编 |
|---|---|---|
| 作者改过（是 fork） | ✅ | |
| 版本敏感、上游 API 易变 | ✅ | |
| 和项目紧耦合 | ✅ | |
| 体积小、编译快 | ✅ | |
| 通用、成熟、API 稳定 | | ✅ |
| 体积大、编译慢 | | ✅ |
| 系统包 / 官方发布物维护得好 | | ✅ |

## 自带的（进 git）

| 目录 | 是什么 | 版本 / 来源 |
|---|---|---|
| `orb_slam3/` | ORB-SLAM3 的 `src/` + `include/` | 从 `SLAMAPP/ORB_SLAM3`（本地仓库 `1e379cf`）复制 |
| `DBoW2/` | 词袋模型（回环检测用） | 同上，`Thirdparty/DBoW2` |
| `g2o/` | 图优化后端 | 同上，`Thirdparty/g2o` |
| `Sophus/` | 李群/李代数头文件库（只要头） | 同上，`Thirdparty/Sophus` |

后三个是 **ORB-SLAM3 作者自己 fork 过的**，不是上游原版，版本必须跟着走 ——
这就是它们自带、而 Eigen 不自带的区别。

## 外装的（**不进 git**）

判据是 `git` 管不管，不是**放在哪** —— 下面 OpenCV 和 Boost **目录就在 `thirdparty/` 里**，
只是被 `.gitignore` 挡住了。**换机器时把它们放回同样的位置就行，路径不用配。**

| 库 | 从哪来 | 为什么不自带 |
|---|---|---|
| **OpenCV** | **`thirdparty/OpenCV-android-sdk/`**（官方 Android SDK，872 MB） | 通用、成熟、官方有预编译发布物。**下载完直接丢进 `thirdparty/` 就行** —— `CMakeLists.txt` 默认就在那儿找。放别处用 `-DOPENCV_ANDROID_SDK=...` 指过去 |
| **Eigen** | `apt install libeigen3-dev` → `/usr/include/eigen3` | 纯头文件、API 稳定、和 ORB-SLAM3 在 PC 上的用法一致 |
| **Boost** | `./tools/setup-boost-android.sh` | 没有官方安卓发布物，得自己交叉编译；而且它是构建产物，绑死 NDK 版本。跑一次脚本，装到 `boost-android/`（已 gitignore） |

## 为什么 ORB-SLAM3 是复制的，不直接引用 `SLAMAPP/ORB_SLAM3/`

这样 `android/SlamCapture` 这个仓库自己能构建，不依赖仓库外的任何路径 ——
clone 下来就能编。`SLAMAPP/ORB_SLAM3/` 那边保持上游原样，不再改动。

代价是两份源码会分叉。**规则：以后只改这里这份**，`SLAMAPP/ORB_SLAM3/` 只当上游参照。
PC 上跑 `mono_inertial_euroc` 做对照（M7 的验收项）用的还是上游那份 ——
两边都没有改算法，只删了可视化，所以对照仍然成立。

## 对搬进来的东西做了哪些改动

**全都是为了在安卓上编得过，没有一处动到算法。** 每处都在源码里留了注释。

| # | 文件 | 改动 | 不改会怎样 |
|---|---|---|---|
| 1 | `orb_slam3/src/Viewer.cc`、`MapDrawer.cc`、`include/MapDrawer.h` | 削成空壳（保留类名和公有方法签名，方法体清空） | 安卓上没有 Pangolin |
| 2 | `orb_slam3/include/Map.h`、`src/System.cc` | 删掉 `#include <pangolin/pangolin.h>` | 同上 |
| 3 | `orb_slam3/include/Map.h` | `GLubyte* mThumbnail` → `unsigned char*` | `GLubyte` 是 Pangolin 带进来的，剥掉之后没了。它只是 `unsigned char` 的 typedef，那个成员从来没被真正用过 |
| 4 | `orb_slam3/include/*`、`src/*` | 35 处 `#include "Thirdparty/<库>/..."` → `"<库>/..."` | 三个依赖提到上一级了，前缀没了 |
| 5 | `orb_slam3/src/System.cc` | `#include <openssl/md5.h>` → `"md5.h"` | NDK 没有 OpenSSL，见 `app/src/main/cpp/md5.h` |
| 6 | `orb_slam3/src/ORBmatcher.cc`、`DBoW2/DBoW2/FORB.cpp` | `#include <stdint-gcc.h>` → `<stdint.h>` | 上游误留的 GCC 内部头，clang/libc++ 里没有 |
| 7 | `g2o/g2o/core/*.h`（5 个） | `<tr1/unordered_map>`、`<tr1/memory>` → C++11 标准库，`std::tr1::` → `std::` | TR1 是 C++11 之前的 GCC 专有头。clang 定义了 `__GNUC__`，所以它走了 GCC 那条分支 |
| 8 | `g2o/CMakeLists.txt` | `FIND_PACKAGE(Eigen3 ...)` → 用顶层传进来的 `SLAMAPP_EIGEN_INCLUDE` | 交叉编译时 CMake 的 package 查找被 NDK 锁在 sysroot 里，找不到系统装的 Eigen |
| 9 | `DBoW2/CMakeLists.txt`、`g2o/CMakeLists.txt` | 删掉 `-march=native` | 那是"按本机 CPU 生成指令"，本机 x86、目标 arm64，直接报 `unknown target CPU 'native'`。它只是优化 |
| 10 | `orb_slam3/src/Optimizer.cc` | `InertialOptimization(Map*, Matrix3d&, double&)` 里那句「顶点缺失」的报错串不再逐个 `->id()` | ⚠️ **这条和上面 9 条性质不同：它是上游的 bug，不是编译迁移。** |
| 11 | `orb_slam3/src/Optimizer.cc` | `InertialOptimization(...)`（多参数那个重载）里检测到 `!mpImuPreintegrated` 之后**补了 `continue`** | 同上 —— **上游 bug，和第 10 条同源** |
| 12 | `orb_slam3/include/Tracking.h` + `src/Tracking.cc` | 加一把 `mMutexTraj`，把往 `mlRelativeFramePoses` 那四张表 push 的地方罩住 | ⚠️ **不是上游的 bug，是我们多出来的用法**：上游只在 `Shutdown()` 之后读那四张表，所以不用锁；**我们要在跑的时候读** |
| 13 | `orb_slam3/include/System.h` + `src/System.cc` | **新增** `GetFrameTrajectory()` / `GetFrameTrajectorySize()` —— 把上游那段 VBA 拼装从「写文件」改成「写进 buffer」 | 同上，配套 |
| 14 | `orb_slam3/include/System.h` + `src/System.cc` | **新增** `GetAllMapPoints()` —— 转发 `Atlas::GetAllMapPoints()`，直接返回 `std::vector<MapPoint*>`（**不做抽稀、不碰坐标**，那是显示策略） | ⚠️ **不是上游的 bug**：上游没有这个入口，只有逐帧的 `GetTrackedMapPoints()`。**且必须用 `Atlas::GetAllMapPoints()`** —— `Atlas::GetCurrentMap()` 在坏图窗口里 `while(IsBad()) usleep(3000)`，而且**锁是抱着睡的** |
| 15 | `orb_slam3/src/Tracking.cc` | `GrabImageMonocular` 里，**一个特征点都没提出来的帧直接丢掉**（返回上一帧位姿，不调 `Track()`）| ⚠️ **上游 bug**：`Frame` 的图像 ctor 在「`mvKeys.empty()`」时**提前 return**，`mpMutexImu` 等成员从没赋过值 ⟹ 真机上 `PreintegrateIMU → setIntegrated()` 解引用未初始化的裸指针，**SIGSEGV 崩在 0x0**。详见 `docs/changes/2026-10-15-empty-frame-crash.md` |
| 16 | `orb_slam3/src/System.cc` | `SaveAtlas()` / `LoadAtlas()` **不再硬拼 `"./"` 前缀** | ⚠️ **不是上游的 bug，是安卓这条路上的硬伤**：我们给的是**绝对路径**，而 `"./" + "/data/…"` 会被解析成**相对当前目录**（安卓的 CWD 是 `/`，不可写）⟹ 存不进去、也载不进来。去掉前缀对**相对路径的行为完全一样**（`./x` ≡ `x`）|
| 17 | `orb_slam3/include/System.h` + `src/System.cc` | **新增** `SaveAtlasTo(path)` —— 公开包装，让文件名能**现取** | 上游的 `SaveAtlas` 是 private，而且路径只能来自**构造时读的 yaml** ⟹ 而 §9 要「另存、不覆盖」= 每次一个新名字，构造时根本还不知道 |
| 18 | `orb_slam3/src/System.cc` | 载图失败从 **`exit(-1)` 改成抛异常** | ⚠️ `exit(-1)` 在安卓上表现为「**App 干净地退回桌面、logcat 里只有一行**」⟹ 用户看到的是「点了启动，App 没了」而没有任何提示。抛出去，`slam_engine.cpp` 的 `construct()` 接得住 |
| 19 | `orb_slam3/include/System.h` + `src/System.cc` | **新增** `GetAtlasStats()` —— atlas 里有几张图 / 共多少关键帧 / 多少地图点 | 同 14，给界面显示用。**没有它，「图载进来了没有」在界面上完全看不出来**（载完之后当前图是新建的空图）|
| 20 | `orb_slam3/include/Atlas.h` + `src/Atlas.cc`、`include/System.h` + `src/System.cc` | **新增** `Atlas::AdoptFrom()` / `System::LoadAtlasInto()` / `System::ReloadAtlasFrom()` —— 在**进程内**把 atlas 换回某份 `.osa` 的状态 | ⚠️ 构造之后想换图，上游只有「换进程」一条路：`LoadAtlas()` 里 `ia >> mpAtlas` **换的是指针**，而 `Tracking` / `LocalMapping` / `LoopClosing` 都攥着旧指针。改成「读进现有对象、只搬内容」，对象身份不变，线程不用改指向 |
| 21 | `orb_slam3/src/Tracking.cc`（`Reset`）、`src/LoopClosing.cc`（`ResetIfRequested`） | `Reset` 清那四张轨迹表时拿上 `mMutexTraj`；`ResetIfRequested` 顺手清掉半途的回环 / 合并候选计数 | 上游 `Reset` 只在 `Shutdown` 之后调、`ResetIfRequested` 只在 `System::Reset()` 里用 ⟹ 它没考虑「主线程正在读轨迹」和「上一个 KF 的候选缓存」。我们这两条都碰上了 |

第 3、6、7 条是这次真编才暴露出来的 —— 光看代码不容易想到。

### 第 10 条说清楚

前 9 条都是「在安卓上编得过」的迁移改动。**第 10 条不是** —— 它是上游一个真实的崩溃：

```cpp
if(!VP1 || !VV1 || ... || !VS)
{
    Verbose::PrintMess("Error" + to_string(VP1->id()) + ", " + ...);   // ← 已经知道有 nullptr 了
    continue;                                                          //    还逐个解引用
}
```

**这句诊断信息，恰恰在它要诊断的那种情况下必然崩**（空指针上读偏移 8 的成员）。
真机上抓到的 tombstone 是 `SIGSEGV / fault addr 0x8`，
`addr2line` 定位到 `Optimizer.cc:3464` → `g2o::HyperGraph::Vertex::id()`。

**为什么这不违反「没动算法」**：那个分支本来就 `continue`，跳过这条边是上游的原意；
删掉的只是一句**永远不会被打印**（`Verbose::th` 是 QUIET）、**且必然崩溃**的字符串拼接。
**计算路径一个字都没变，所以 PC 对照仍然成立。**

⚠️ 它只是**不崩了**，没有解决「谁把顶点弄空了」（那是 `LocalMapping` 在初始化/重置期间的老问题）。
但上游本来就打算 `continue` 跳过那条边，所以不崩是对的。

**⚠️ `SLAMAPP/ORB_SLAM3/src/Optimizer.cc` 里同样这一行没改** —— 按上面的规则，
那边只当上游参照。PC 上如果也撞上这条分支，会崩在同一行。

### 第 11 条说清楚

**和第 10 条是同一个模子**：ORB-SLAM3 里有一类「**检测到 → 打印 → 照样用**」的写法。

```cpp
if(!pKFi->mpImuPreintegrated)
    std::cout << "Not preintegrated measurement" << std::endl;   // ← 检测到了

pKFi->mpImuPreintegrated->SetNewBias(...);                        // ← 然后照样解引用 ✗ 崩
```

真机 tombstone：`SIGSEGV / fault addr 0x8a8`，
`pthread_mutex_lock` ← `IMU::Preintegrated::SetNewBias` ← `Optimizer::InertialOptimization`
← `LocalMapping::InitializeIMU` ← `LocalMapping::Run`。

**修法**：补 `continue` —— 没有预积分就没法参与惯性优化，
**和上面几行按 `isBad()` / `mnId` 过滤是同一套逻辑** ✓

**⚠️ 触发条件是偶发的**：需要「某个关键帧没有 IMU 预积分」。
日志里伴随 `Empty IMU measurements vector!!!` 和 `Not preintegrated measurement`。
历次运行出现 0~1 次。

**⚠️ 还有三处同源但没崩过、也没修的**（用户决定「下次崩了再说」）：
`Optimizer.cc` 里三处 `new EdgeInertialGS(pKFi->mpImuPreintegrated)` ——
**顶点非空 ≠ 预积分非空**，所以那三处的空检查护不住它。**崩了就来这儿看。**

### 第 12 / 13 条说清楚

**这两条和上面 11 条性质不同 —— 不是上游 bug，是我们多出来的用法。**

上游已经有一套**正确**的相机轨迹：它不存绝对位姿，存的是「**每一帧相对它自己那个参考关键帧**」的
位姿（`Tracking::mlRelativeFramePoses` + `mlpReferences`），读的时候**用关键帧当前的位姿重算** ⟹
回环 / 全局 BA 改过地图之后**仍然一致**。

**我们原来是自己每帧抄一份绝对位姿** ✗ —— BA 一改地图坐标系，抄下来的就留在旧系里，
**轨迹跳一段**（真机上就是这么发现的），而且 `trajectory.txt` 也是同一份错的。

**两处改动：**

| | 改什么 | 为什么 |
|---|---|---|
| **12** | `Tracking` 加 `mMutexTraj`，罩住往那四张表 push 的地方 | 上游只在 `Shutdown()` **之后**读它们（`SaveTrajectoryEuRoC` 的注释就写着 "Call first Shutdown()"），那时没有线程在跑 ⟹ 它不用锁。**我们在跑的时候从主线程读**（2D 地图 4Hz）⟹ 必须自己加一把 |
| **13** | `System` 新增 `GetFrameTrajectory()` / `GetFrameTrajectorySize()` | 把上游 `SaveTrajectoryEuRoC` 那段拼装从「写文件」改成「写进 buffer」。`mpTracker` 是 `System` 自己的成员，所以**不用碰 `Tracking` 的成员可见性** |

**⚠️ 两处故意和上游不一样**（都在 .cc 里注释了）：

- **不乘 `pKF->mImuCalib.mTbc`** —— 那会转到 **body/IMU** 系，而我们的实时位姿是**相机系**；
- **不乘首帧偏移 `Twb`** —— 那会挪到 EuRoC 的**真值系**，而我们要的是**地图坐标系**。

两条都是为了「轨迹和当前位置在同一个系里」，否则两者会错开。
**副作用**：导出的 `trajectory.txt` 因此**和 PC 的输出不在同一个 frame** ⟹
**别逐行 diff，只能比形状和尺度**。

**另外加了两处防护**（上游没有）：生成树爬升加了 64 跳上界；`pMap` 不用
`mpAtlas->GetCurrentMap()`（那个在坏图窗口里是 `while(IsBad()) usleep(3000)`，从主线程调会**冻死界面**），
改用「最后一个有效参考帧属于哪张图」—— 语义一样、不会挂，**而且顺带让重置之后的老帧被自动滤掉**。

## 许可证

搬进来的东西各自带着自己的许可证，不要当成这个项目的代码：

- ORB-SLAM3、DBoW2、g2o —— **GPLv3**
- Sophus —— MIT

外装的那几个：OpenCV 是 Apache 2.0，Eigen 是 MPL2，Boost 是 BSL-1.0。
