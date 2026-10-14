# 2026-10-14 轨迹改成**读上游那份**（回环之后不再跳一段）

一句话：**别再自己抄绝对位姿了** —— 上游一直在记一份**正确的**（相对参考关键帧），
读时用关键帧**当前**的位姿重算 ⟹ 回环/BA 改过地图之后**仍然一致**。

## 动机

真机上，**优化之后轨迹突然跳一段** ✗（走闭环的时候最明显）

**根因**：`traj` 记的是**每帧当时的绝对位姿**，而回环/全局 BA **改写地图坐标系**
⟹ 旧点留在旧系、新点在新系 ⟹ 折线出现台阶。

**影响面是两处，不止小窗**：

| | |
|---|---|
| **2D 小窗** | 就是看到的那个跳 |
| **`trajectory.txt`** | 同一份 `traj` ⟹ 也是旧的 ✗ 而它是拿去和 PC 对照的 |

## 改动

| 文件 | 改什么 |
|---|---|
| `thirdparty/orb_slam3/include/Tracking.h` + `src/Tracking.cc` | 加 `mMutexTraj`，罩住往那四张表 push 的地方（**第 12 条补丁**）|
| `thirdparty/orb_slam3/include/System.h` + `src/System.cc` | 新增 `GetFrameTrajectory()` / `GetFrameTrajectorySize()`（**第 13 条**）|
| `thirdparty/README.md` | 记这两条 |
| `cpp/slam_engine.cpp` | **删掉** `TrajPoint` / `Engine::traj` / `savePose` 里记轨迹那半；`trajSize` / `getTrajectory` / `dumpTrajectory` 改走垫片 |

**给 Kotlin 的契约没变**（`getTrajectory` 还是 `3n` 个 xyz）—— 8 个 float 里只挑 xyz 出来 ✓

## 决策

### 一、⭐ 为什么抽稀成立：**「相对参考关键帧」是点对点独立的**（用户反复问的那点）

`Tracking.cc:2305`：

```cpp
Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
mlRelativeFramePoses.push_back(Tcr_);
mlpReferences.push_back(mCurrentFrame.mpReferenceKF);   // ← 每帧各记各的参考帧
```

**`mpReferenceKF` 挂在 `mCurrentFrame` 上**（类型是 `KeyFrame*` ✗ 不是 `Frame*`）⟹

```
Twc[i] = ( rel[i] · Trw_i · pKF_i->GetPose() )⁻¹
             ↑        ↑       ↑   三个下标全是 i，**和别的帧无关**
```

**所以能跳过中间的** ✓ 唯一的「累计」是**一帧内部**的生成树爬升（参考帧被剔时）✓ 不跨帧 ✓

> ⚠️ **如果它存的是「相对上一帧」**，那就必须从 0 一路累乘、**跳不了** ✓
> **而它不是** ✓ 而且那样也没用 —— 累乘出来的是「当场那张地图」的轨迹，优化之后照样歪 ✗

**代价只是走指针**（`std::list` 没有随机访问）：50k 次 `++` ≈ 0.1ms ✓
**拼装（矩阵乘 + 爬树）只做选中的 ≤maxPts 个** ✓ —— `continue` 就在 `compose` **之前** ✓

### 二、⭐ **必须加锁**（用户抓出来的）

`System::TrackMonocular` 里那个 `mMutexState` 是在 `GrabImageMonocular` **返回之后**才拿的：

```cpp
Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(...);   // ← 轨迹在这儿被 push
unique_lock<mutex> lock2(mMutexState);                   // ← 锁在这之后
```

⟹ **它不包括那四张表** ✗
**上游为什么没事**：它只在 `Shutdown()` 之后读（注释就写着 "Call first Shutdown()"）✓
**我们每帧从主线程读** ⟹ 一边 push 一边遍历 ⟹ 竞争 ✗
**⟹ 给那四张表自己一把锁**（`Tracking::mMutexTraj`）✓ 临界区只有四个 `push_back` ✓

### 三、**不肯自己存**（用户提的方案，值得记）

用户提过一个更「自主」的版本：**加个接口拿到参考帧，我们自己存 `(pKF, rel)`**。
**优点是真的**：自己存 `std::vector` ⟹ **O(1) 下标访问**，抽稀不用走指针 ✓

**但还是否了**，理由：

| | 读上游 | 自己存 |
|---|---|---|
| 拿参考帧 | 上游已在存 ✓ | ⚠️ **还是得加接口**（`mpReferenceKF` 是 protected）✗ |
| **参考帧被剔了** | **生成树爬升现成的** ✓ | ⚠️ **自己复刻**（`mTcp` + `GetParent()`）✗ |
| **重置后的老帧** | **跨图过滤现成的** ✓ | ⚠️ 自己判 ✗ |

**省下的是 0.1ms，付出的是复刻三段有坑的逻辑** —— 而「生成树爬升」那段上游自己都写得**没有上界**
（我加了 64 跳才敢用）✗ 复刻一遍就是再踩一遍 ✓

**而且两条路都要动上游** ⟹ 那条「不动上游」的理由两边都不成立 ✓

### 四、两处**故意**和上游不一样

| | 上游 | 我们 | 为什么 |
|---|---|---|---|
| **`mTbc`** | 乘（转到 body/IMU 系）| **不乘** | 我们的实时位姿（`poseBuf`）是**相机系** ⟹ 必须同系，否则轨迹和琥珀点错开 ✗ |
| **首帧归零 `Twb`** | 归（EuRoC 真值系）| **不归** | 我们要**地图坐标系** ✓ |

**副作用**：导出的 `trajectory.txt` 因此**和 PC 的输出不在同一个 frame** ⟹
**注释里原来那句「对照时直接 diff」是错的，改掉了** ✓（一直是错的，只是之前比的是形状和尺度）

### 五、⚠️ **不用 `GetCurrentMap()`** —— 它会挂死

上游那段用它过滤「参考帧还是不是当前图的」，而 `Atlas.cc:254` 是：

```cpp
while(mpCurrentMap->IsBad()) usleep(3000);   // 死循环
```

**主线程 4Hz 调，撞上就是整个 UI 冻死** ✗✗
**改成「最后一个有效参考帧属于哪张图」** ✓ 语义一样、不会挂 ✓
**附带好处**：重置之后老帧的参考帧在旧图上 ⟹ **自动被滤掉** ⟹ 我们**不用自己清轨迹** ✓
（`start()` 和 reset 分支里原来那两处 `traj.clear()` 跟着删了 ✓）

### 六、`cap` 的两种语义 —— 两份契约**正好相反**

| | `getPinned` | `getTrajectory` |
|---|---|---|
| 装不下怎么办 | **少给**（截断）| **抽稀**（覆盖全程、恒含最新点）|
| 想「取全」 | **扩容再读** | **传个够大的 `cap`** |

**同一个词两种契约** ✗ 所以 `dumpTrajectory` 是**把 cap 加倍到装得下**（要全保真 ✓），
而 `getTrajectory` 是**固定 512**（要形状 ✓）✓ 两边都写在注释里了 ✓

### 七、⚠️ 第一次上机：**乘法顺序写反了**（真机当场暴露）

上游是 `(*lit) * Trw` —— **`rel` 在最左边** ✓
我第一版写成了 `Trw * pKF->GetPose() * rel` ✗ **`rel` 跑到最右边** ✗

```
对的：  rel · Tcw_ref = (Tcw_cur · Twc_ref) · Tcw_ref = Tcw_cur  ✓
我写的：Tcw_ref · rel  = Tcw_ref · Tcw_cur · Twc_ref              ✗ 根本不是 Tcw_cur
```

**症状是「高频锯齿」** ✓ —— 因为结果左边多出一个 `Tcw_ref`，而**参考关键帧每 0.5 秒换一个**
⟹ 每换一次就跳一下 ⟹ 折线上一排锯齿 ✓

**一个错解释了「锯齿」和「位姿不对」两个症状** ✓
（这就是为什么要**先上机**再看图：这种错在编译和读代码时都看不出来 ✗）

### 八、⚠️ 计数跨 run 累加 —— 因为那张表**上游从不清理**

`trajSize()` 现在直接读 `mlRelativeFramePoses.size()` ✗ 而它是**进程级累积**的 ✗
⟹ 停一次再启动，计数**接着往上加** ✗

**「小地图看起来清了」是假的** —— 停 SLAM 之后 `slamRunning == false` ⟹ hint 是「SLAM 未启动」
⟹ `onDraw` 只画那句提示 ✓ **数据还在那儿** ✗ 一启动就会两条轨迹叠在一起 ✗

**修法**：加 `Tracking::ClearTrajectory()` / `System::ClearFrameTrajectory()`，
在 `start()` 里调 ✓ —— **一次运行 = 一份轨迹** ✓（和删掉的那句 `traj.clear()` 同一个语义 ✓）

## 验证

**1. 编译 + 装机** ✓

```
> Task :app:assembleDebug
BUILD SUCCESSFUL
Performing Streamed Install
Success
```

**2. ⚠️ 真机未验证。** 要看的：

| # | 做什么 | 期望 |
|---|---|---|
| ① | ⭐ **走一个带闭环的圈**（回到起点附近）| **轨迹不跳** ⟸ 就是这次修的 |
| ② | 回环前后各看一眼 | 轨迹**连续**，和当前位置始终接得上 |
| ③ | 导出 `trajectory.txt` | 形状和优化后一致（**不做和 PC 的逐行 diff**，见决策四）|
| ④ | 回归 | 小窗 hint / 方向三角 / 测距 overlay |
| ⑤ | **重置之后** | 计数归零、小窗清空 ⟸ **靠跨图过滤，不是靠手动清** |
| ⑥ | 状态栏「轨迹 N 点」 | 语义变了：现在含**跟丢的帧**（以前只含 OK 的）⟹ 数字会**略大** |
| ⑦ | ⭐ **停 SLAM → 再启动** | **计数从 0 重新开始**、小窗只有这一趟的轨迹 ⟸ 决策八 |
| ⑧ | 慢走一段直线 | **没有锯齿** ⟸ 决策七 |

**3. 第一次上机暴露的（已修）**

- **锯齿 + 位姿不对** ⟹ 决策七（乘法顺序）
- **计数跨 run 累加** ⟹ 决策八

## 遗留

- ⚠️ **`trajectory.txt` 和 PC 不在同一个 frame**（决策四）—— 注释改诚实了，但没修
- ⚠️ **跟丢的帧会推重复值** ⟹ 偶尔多一个零长度线段 ✓ 无害
- ⚠️ **间隔不严格均匀**（我们的队列会丢帧）⟹ 按序号抽稀是「近似均匀」✓ 按时间抽的话
  `mlFrameTimes` 就在数据里（而且已经带出去了）—— **现在不做**
- **`TrackResult` 里的 `t/qx..qw` 现在是死字段**（没人读了）—— `orb_track.cpp` 每帧还算一遍 ✗
  可以删，但那要动那个线程敏感的 `orb_track` ✓ **下次顺手清**
- **`mlRelativeFramePoses` 跨重置累积** —— 内存单调涨（和「ORB 从不释放」同一类账）✓
  但它比关键帧轻得多（SE3f，28 B/帧）✓

## 回滚

```
thirdparty/orb_slam3/{include/Tracking.h, src/Tracking.cc}
thirdparty/orb_slam3/{include/System.h, src/System.cc}
app/src/main/cpp/slam_engine.cpp
```
