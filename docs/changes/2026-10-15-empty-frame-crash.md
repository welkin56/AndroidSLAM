# 2026-10-15 真机崩溃：**0 个特征点的空帧**喂进了 `Track()`（上游 bug）

一句话：ORB 一帧都没提出特征点时，`Frame` 的构造函数**提前 return**，`mpMutexImu` 这个
**裸指针从没赋过值** ✗ —— 后面 `PreintegrateIMU → setIntegrated()` 解引用它，崩在 0x0。
修法：**这种帧直接丢掉**，别喂进 `Track()`。

## 动机

用户：「你看一下日志又崩溃了」。`logcat -b crash` 里**同一条栈三次**
（10-15 19:14:10 / 19:14:37 / 19:17:52，进程分别活了 265 / 289 / 221 秒）：

```
signal 11 (SIGSEGV), code 1 (SEGV_MAPERR), fault addr 0x0
Cause: null pointer dereference
    #00 pthread_mutex_lock+16                       ← x0 = 0
    #01 std::__ndk1::mutex::lock()+12
    #02 ORB_SLAM3::Frame::setIntegrated()+28
    #03 ORB_SLAM3::Tracking::PreintegrateIMU()+2036
    #04 ORB_SLAM3::Tracking::Track()+940
    #05 ORB_SLAM3::Tracking::GrabImageMonocular(...)
    #06 ORB_SLAM3::System::TrackMonocular(...)
    #07 slam::orbTrack(...)
```

⚠️ **和上次那个 NaN 崩溃（`EdgeInertialGS::computeError → GetRotInG → Sophus::defaultEnsure`）
不是同一个** —— 那个是 NaN，这个是空指针 ✗。

⚠️ **跑了几分钟才炸**（不是一启动就炸），这条时间线本身就是线索：静止时画面一直有纹理 ✓，
一为了惯性初始化**动起来**（快速平移/转头、或者扫过白墙和天花板），才出现**空帧** ✓。

## 改动

- `thirdparty/orb_slam3/src/Tracking.cc` —— `GrabImageMonocular` 里 `Track()` 之前加一道判断：
  帧的 `N == 0` 就打印一行并**返回上一帧的位姿**，不调 `Track()`。
- `thirdparty/README.md` —— **第 15 条补丁**（和 10/11 同一类：上游的未初始化 / 空指针）。

## 决策

### 一、根因：`mpMutexImu` 是**唯一**没被赋值的那个成员

```cpp
Frame::Frame(const cv::Mat &imGray, ...)          // Frame.cc:289（单目+IMU 用的就是它）
  : ..., mbHasPose(false), mbHasVelocity(false)   // ← 初始化列表里**没有** mpMutexImu
{
    ExtractORB(0, imGray, 0, 1000);
    N = mvKeys.size();
    if(mvKeys.empty())
        return;                                    // ← :320 一个特征点都没有，直接返回
    ...
    mpMutexImu = new std::mutex();                 // ← :381 在最末尾，永远走不到
}
```

```cpp
void Frame::setIntegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);     // ← 裸指针，未初始化就是栈垃圾
    mbImuPreintegrated = true;
}
```

`Frame.h:323` 是 `std::mutex *mpMutexImu;` —— **没有类内初值** ✗，四个图像 ctor 里
又都只在**末尾**才 `new` ✗，而它们**全都有** `if(mvKeys.empty()) return;` 这个早退 ✗。
⟹ 空帧的这个指针是**栈上的残留值**，恰好是 0 时就是 0x0 ✓ 和 tombstone 对得上 ✓。

⚠️ **只有内建类型/裸指针成员是垃圾** —— `mvKeysUn`/`mvuRight`/`mvDepth`/`mvpMapPoints`/
`mvbOutlier` 那些 `std::vector` 成员**不在**初始化列表里 ⟹ 走的是**默认构造**（空）✓
不是垃圾 ✓。所以崩的只可能是标量和裸指针 ✓。

### 二、⭐ 改**调用方**（丢掉空帧），不改 `Frame`

```cpp
    if(mCurrentFrame.N == 0)
    {
        Verbose::PrintMess("Empty frame (0 keypoints), skipped", Verbose::VERBOSITY_NORMAL);
        return mLastFrame.GetPose();
    }
```

**否决：把四条 `new` 挪到早退之前。** 要改四处 ✗，而且**下次上游再加一个早退就再漏一遍** ✗；
更要紧的是——那四个 ctor 里早退漏掉的**不止 `mpMutexImu`**（`Nleft`/`monoLeft`/`fx`/`mb`…
一堆标量同样没赋值 ✗），补一个只是把崩溃往后挪 ✓。

**否决：给 `setIntegrated()` 加空指针判断。** 那是**静默跳过** ✗ —— 这帧永远不算「已积分」，
换一种沉默而已 ✓（这个项目里「不能沉默」是条硬规矩）。

**否决：让 ctor 把空帧也构造完整。** 上游**故意**用早退表示「这帧是废的」✓ ——
替它补全等于改上游语义 ✗（而且 `Track()` 拿到 0 个点的帧照样没法跟踪 ✓）。

**采用：在调用方丢掉。** 0 个特征点的帧**本来就没法跟踪** ✓，喂进去只会被判跟丢 ✓。
丢掉它 ⟹ `PreintegrateIMU` **够不着**这个半成品 ✓，而且它**进不了 `mLastFrame`** ✓
（`mLastFrame = mCurrentFrame` 在 `Track()` 末尾 ✓）⟹ 垃圾连下一帧都污染不到 ✓

### 三、两个小决定

| | 怎么做 | 为什么 |
|---|---|---|
| 返回值 | `mLastFrame.GetPose()` **不是** `mCurrentFrame.GetPose()` | 半成品的 `mTcw` 同样是没赋过值的 ✓（`mLastFrame` 那个至少是上一帧真的位姿 ✓；第一帧时是 Sophus 的单位阵 ✓ 而跟踪状态是 `NO_IMAGES_YET` ⟹ 上层本来就不会用 ✓）|
| 更新 `lastID` 吗 | **不更新**（判断放在它**前面**）| `lastID` 是「**处理过**的最后一帧」的意思，而这一帧我们没处理 ✓。而且它参与 `(lastID - initID) < mMaxFrames` 那个初始化窗口的计数 ✓ |

### 四、⚠️ 这可能也解释了你那条「重置次数变多」

一个 0 特征的帧喂进 `Track()`，最可能是被判成**跟丢** ✓ —— 而跟丢多了会触发重置 ✗。
这只是**推测，未验证** ✓ 这次先把崩溃堵掉，看日志里那行 `Empty frame ... skipped`
出现得勤不勤 ✓ 就知道了。

## 验证

**编译 + 装机** ✓

```
> Task :app:installDebug
BUILD SUCCESSFUL in 1m 4s
```

**⚠️ 真机未验证。** 怎么复现 / 怎么确认：

| # | 做什么 | 期望 |
|---|---|---|
| ① | ⭐ 故意制造空帧：**对着白墙 / 天花板扫，同时快速甩动** | **不崩** ⟸ 就是这次修的 |
| ② | 同时看 `slam` tag | 有 `Empty frame (0 keypoints), skipped` ✓（`Verbose::th` 默认就是 `NORMAL`、yaml 里没改它 ⟹ 这行**打得出** ✓）|
| ③ | 正常走一圈 | 行为不变（这些帧本来就跟踪不了）✓ |
| ④ | 再看 `[M5] 地图被重置` 那行的时间分布 | 如果空帧确实会引发重置，次数应该**变少** ⟸ 决策四 |

## 遗留

- ⚠️ **只堵住了 `GrabImageMonocular` 这一条路。** 上游那四个图像 ctor 的早退**仍然**会产出
  半成品 `Frame` ✗ —— 将来谁在别处 `Frame f(imGray, ...)` 而不检查 `N`，同一个坑还在 ✓
- ⚠️ **空帧那些标量成员仍然是垃圾**（`Nleft`/`monoLeft`/`mnCloseMPs`/`mb`/`fx`/`fy`/`cx`/`cy`…）
  ✗ —— 这次没碰 ✓。它们不会被解引用（不崩 ✓），但**算出来的值可能是错的** ✓。
  真要对治，出路是给 `Frame` 的那批标量加类内初值 ✓ —— 那是比这条大一圈的改动 ✓
- **默认 ctor `Frame::Frame()` 同样没有 `mpMutexImu` 初值** ✗ —— 走查过：`mCurrentFrame` /
  `mLastFrame` 都是默认构造的，但**只有非空帧才会被赋值进去** ✓ ⟹ 到不了 `setIntegrated()` ✓
  （这次加的那道判断同时也保证了这一点 ✓）
- 上次那个 **NaN 崩溃**（`Sophus::defaultEnsure`）和这条无关 ✗ —— 还挂着 ✓

## 回滚

```
thirdparty/orb_slam3/src/Tracking.cc     ← 删掉 GrabImageMonocular 里那道 if(mCurrentFrame.N == 0)
thirdparty/README.md                     ← 删第 15 条
```

⚠️ 回滚之后崩溃会**照旧复现** ✓ —— 那是上游的行为，不是我们引入的 ✓。
