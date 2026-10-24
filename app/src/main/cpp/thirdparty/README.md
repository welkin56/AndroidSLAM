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
| 22 | `orb_slam3/include/Map.h` + `src/Map.cc` | **新增** `Map::GetLatestKeyFrame()` —— 图集合里 `mnId` 最大的那个关键帧 | 路径点要一个**保证活着**的锚点。`Tracking::GetLastKeyFrame()` 不行：它可能还在 `LocalMapping` 队列里，而 `LocalMapping::Release()` 会**直接 delete** 排队的 KF（合并、回环、存图都会调 `Release()`）|
| 23 | `orb_slam3/include/System.h` + `src/System.cc` | **新增** 路径点：`AddWaypoint` / `Rename` / `Delete` / `Clear` / `GetWaypoints` / `SaveWaypoints` / `LoadWaypoints` | 锚在关键帧上、存相对位姿（`T_rel = Tcw_此刻 * Twc_锚点`），和轨迹同一套。BA / 回环 / 合并改了地图之后**自动跟着走** —— 合并时 `MergeLocal2` 是 `pKF->UpdateMap()` 搬对象，指针不变 |
| 24 | `orb_slam3/src/Optimizer.cc`（`MergeInertialBA`） | 优化集 `vpOptimizableKFs` 按对象去重；固定集 `lFixedKeyFrames` 去掉与两个优化集重叠的成员 | ⚠️ **上游 bug**：滑动窗口是「当前 KF 的 `mPrevKF` 链」+「合并目标 KF 的前后链」拼起来的，两条链会走到同一个 KF ⟹ 同一个 `mnId` 被 `addVertex` 两次。g2o 只打一行 `addVertex: FATAL` 就继续，**图已经坏了**，之后整张图不刷新、跟踪连续失败。只有「载入旧图 + 再合并」会稳定踩到（载入后两边的 `mPrevKF` 链是互通的） |
| 25 | `orb_slam3/src/LoopClosing.cc`（`MergeLocal2`） | 在 `ApplyScaledRotation` **之前**打一行 `[MAP] 合并：图 A（KF n）→ 图 B（KF m），scale s` | 合并原本**完全静默**。它同时改变坐标系（`p_new = s·R·p_old + t`）而 map id 不变，出问题时无从对齐时间线 |
| 26 | `orb_slam3/include/Atlas.h` + `src/Atlas.cc`、`src/LoopClosing.cc`、`include/System.h` + `src/System.cc` | **新增** 合并链：`Atlas::RecordMerge` / `GetMergeRecords` / `ClearMergeRecords`（仅内存），`System::GetTrajectoryMapId()` / `GetMergeRecords()` | ⚠️ **不是上游 bug，是我们多出来的用法**：磁盘上的轨迹按图分文件夹，而合并**只改坐标系、不改图的身份**（`ApplyScaledRotation` 把当前图搬到被吸收图的系里）。不记下 `T_on`，合并前写进磁盘的轨迹就永远停在旧坐标系，没法跟合并后的图叠。链在 `PostLoad()` / `ReloadAtlasFrom()` 清空 —— 换 atlas 之后旧的变换没有意义 |
| 27 | `orb_slam3/src/LocalMapping.cc`（`InitializeIMU` / `ScaleRefinement`） | 删 `mlNewKeyFrames` 里的 KF 之前**先看 `isBad()`**，只有真的标坏才 `delete` | ⚠️ **上游 bug，会留下悬空指针**：`SetBadFlag()` 对**地图的初始关键帧**直接 `return`（`mnId == GetInitKFid()`，保护原点），紧接着的 `delete` 却照样执行 ⟹ 初始 KF 被删掉、却还在 `mspKeyFrames` 里、还被几百个地图点观测着。之后 BA / 回环一碰就是 UB，存图时 `MapPreSave` 遍历观测会崩。详见 `docs/changes/2026-10-21-presave-crash-dangling-kf.md` |
| 28 | `orb_slam3/src/MapPoint.cc`（`PreSave` / `EraseObservation`） | `PreSave` 先把要删的观测收集起来、循环结束再 `EraseObservation`；`EraseObservation` 擦到空时不再解引用 `begin()` | ⚠️ **上游 bug**：边遍历 `mObservations` 边 erase ⟹ 下一次 `++it` 是 use-after-free（真机崩在 `__tree_next_iter`）。擦到空再取 `begin()->first` 也是 UB，正好是同一条路径 |
| 29 | `orb_slam3/include/Atlas.h` + `src/Atlas.cc`、`include/System.h` + `src/System.cc` | **新增** `Atlas::GetBestMapId` / `GetDisplayTransforms` / `DropMergeRecords`，`System::GetDisplayMapId` / `GetDisplayTransforms` / `DropMergeRecords` | ⚠️ **不是上游 bug，是我们多出来的用法**：小地图要叠历史轨迹，而合并只改坐标系、不改 map id。`GetDisplayTransforms` 按合并链算出「每个 `map-<id>/` 到当前系」的累计变换（被吸收的原样、幸存图那一族乘 `T_on`）。`DropMergeRecords` 让链只留**还没落盘**的合并 —— 否则下一次存图会把同一条 `T_on` 再乘一遍。和 26 同源，见 `docs/changes/2026-10-21-minimap-history-tracks.md` |
| 30 | `orb_slam3/src/Atlas.cc`（`PreSave`）、`src/Map.cc`（`PreSave`）、`src/MapPoint.cc`（构造函数 / `PreSave`） | `Atlas::PreSave` **先清空 `mvpBackupMaps` 再重建**，空图/坏图在加入列表前就剔掉；`Map::PreSave` 的过滤集只放非坏 KF；`mBackupRefKFId` 四个构造函数里初始化成 `-1`，`PreSave` 每次先归零 | ⚠️ **上游 bug，会写出自相矛盾的文件**：同一进程里连续存图时，`mvpBackupMaps` 会累积上一次的图；被合并掉的空图挂着**上一份**备份数组，而里面的 KF/MP 已经搬到别的图 ⟹ 文件里某张图的 KF 列表缺一批 id，连接/参考帧却指着它们。载入时 `PostLoad` 往 `mConnectedKeyFrameWeights` 塞 `nullptr`，`UpdateBestCovisibles()` 当场 SIGSEGV。详见 `docs/changes/2026-10-21-atlas-stale-backup-maps.md` |
| 31 | `orb_slam3/include/{Atlas,Map,KeyFrame,MapPoint}.h` + 对应 `.cc`、`src/System.cc` | **新增** `Atlas::CheckBackupConsistency()` → `Map::CheckBackupConsistency()` → `KeyFrame::CheckBackupConsistency()`，在**反序列化之后、`PostLoad()` 之前**检查每张图的引用是不是都落在自己身上；`KeyFrame::PostLoad` / `MapPoint::PostLoad` 查不到的 id 跳过、不再塞 `nullptr` | ⚠️ **配合 30**：修复前写出来的旧 `.osa` 已经坏了。没有自检的话它们会把整个 App 带崩；现在判载入失败，界面走「载图失败」那条路 |
| 32 | `orb_slam3/include/System.h` + `src/System.cc` | **新增** `System::GetMapList()` / `GetMapCloud()` —— atlas 里每张图的 id/KF/点数和**单张图的点云** | ⚠️ **不是上游 bug，是我们多出来的用法**：2D 地图查看器要一张一张翻 atlas 里的图，而 `Atlas::GetAllMapPoints()` 只给当前图。`GetMapCloud` 会对目标图拿 `mMutexMap` 拷指针表，所以只给「选图/刷新」这种低频动作调。详见 `docs/changes/2026-10-24-map-viewer.md` |

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
