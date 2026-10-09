# ORB-SLAM3 Mono-Inertial 内部流程与重置判据

> **为什么写这篇**：调试阶段三时，日志里反复出现 `New Map created with N points`，
> 轨迹被反复重置吃掉。「什么决定了它要重置」这个问题，答案不在采集端、也不在配置里，
> 而在 ORB-SLAM3 自己的状态机里 —— 而且分散在四个文件的多处独立判断中。
>
> **本文的判据部分经过一次实测修正。** 第一版是纯读代码推出来的，跑了一次真实
> session 之后发现**主因判断错了**（把主因当成了次要路径，而且把「运动过多」
> 当成了「运动不足」）。修正过程见
> [../changes/2026-09-19-tracking-reset-diagnosis.md](../changes/2026-09-19-tracking-reset-diagnosis.md)，
> 实测证据见 §七。
>
> 行号对应本仓库的 `ORB_SLAM3/`，上游版本 **4452a3c**（`Update Dependencies.md`）。
> 换版本要重新核对行号。

---

## 一、线程模型：谁在主线程，谁不在

一切从 System.cc:189 那句注释开始：

```cpp
//Initialize the Tracking thread
//(it will live in the main thread of execution, the one that called this constructor)
```

**Tracking 没有自己的线程 —— 它就活在 `main` 里。** 名字叫 "Tracking thread"
但实际不开线程，这是最容易误解的一处。另外三个才有独立线程：

```
main 线程                    LocalMapping 线程      LoopClosing 线程      Viewer 线程
─────────                    ─────────────────      ────────────────      ───────────
main()                       Run()                  Run()                 Run()
 └ System SLAM(...)  ──开线程──→ [System.cc:197]     [System.cc:214]       [System.cc:233]
   └ new Tracking                                                          ↑
       活在主线程                                                    只有 bUseViewer
                                                                     为 true 才开
```

后果：**`main` 里那个喂帧循环就是 Tracking 的心跳**，每喂一帧同步跑完才返回。
跟踪慢会直接拖慢喂帧节奏，反之亦然。

---

## 二、从 `main` 到 `Track()` 的调用链

```
main()                                            Examples/.../mono_inertial_euroc.cc:140
 │
 ├─ ORB_SLAM3::System SLAM(vocab, settings, IMU_MONOCULAR, true)
 │    ├─ new Tracking(this, ...)                  ← 不开线程，活在这里
 │    ├─ new LocalMapping(...) + thread           System.cc:195-197
 │    ├─ new LoopClosing(...) + thread            System.cc:213-214
 │    └─ new Viewer(...)      + thread            System.cc:232-233
 │
 └─ while(还有帧)                                  mono_inertial_euroc.cc:215
     │
     └─ SLAM.TrackMonocular(im, tframe, vImuMeas)
         ├─ for(i_imu...) mpTracker->GrabImuData(vImuMeas[i_imu])    ← IMU 先进
         └─ mpTracker->GrabImageMonocular(imToFeed, timestamp)      Tracking.cc
             ├─ 灰度化 —— 我们是灰度图，channels()==1，这里跳过
             ├─ mCurrentFrame = Frame(...)      ★ ORB 特征提取在这里
             ├─ Track()                         ★★ 状态机在这一层
             └─ return mCurrentFrame.GetPose()
```

`Track()` 内部（Tracking.cc:1794）的分叉：

```
Track()
 │
 ├─ 开头先检查 LocalMapping 有没有置 mbBadImu        ← 路径④
 │
 ├─ mState == NOT_INITIALIZED ──→ MonocularInitialization()
 │                                     └─ CreateInitialMapMonocular()
 │                                          ├─ "New Map created with N points"
 │                                          └─ if(TrackedMapPoints(1)<50)   ← 路径③
 │
 └─ 否则（已经在跑）
     ├─ TrackWithMotionModel() / TrackReferenceKeyFrame() / Relocalization()
     └─ bOK = TrackLocalMap()                    Tracking.cc:2127
          ├─ false → cout << "Fail to track local map!"
          └─ 按情况置 RECENTLY_LOST 或 LOST
                └─ LOST → KeyFramesInMap() < 10 ? 清空 : 开新图    ← 路径②
```

---

## 三、IMU 数据的入口：跟着图像一起喂

**不是异步推送，是批量的。** `main` 在喂第 `ni` 帧之前，先把时间戳不晚于这一帧的
IMU 测量全部收进 `vImuMeas`
（mono_inertial_euroc.cc:198），
连同图像一起交给 `TrackMonocular`：

```
main:  while(imu时间戳 <= 当前帧时间戳) { vImuMeas.push_back(...); }
main:  SLAM.TrackMonocular(im, tframe, vImuMeas)
         ├─ GrabImuData()  逐条塞进 IMU 缓冲
         └─ GrabImageMonocular()
              └─ Frame(..., &mLastFrame, *mpImuCalib)   ← 预积分在这里做
```

这是 ORB-SLAM3 的**批处理式接口风格** —— 不是实时流，是「你准备好一帧 + 它要的
IMU，交给我」。将来做 JNI 实时版时这一层要重新设计。

---

## 四、平行线程：LocalMapping

`main` 跑 Tracking 的同时，LocalMapping 线程在背后处理交进来的关键帧：

```
LocalMapping::Run()
 │
 └─ while(1)
     ├─ CheckNewKeyFrames()  && !mbBadImu   ← ★ mbBadImu 一旦为真，整个循环停摆
     ├─ ProcessNewKeyFrame()
     ├─ MapPointCulling()
     ├─ CreateNewMapPoints()       ★ 三角化新点 —— 地图在这里长大
     ├─ SearchInNeighbors()
     │
     ├─ if(mbInertial && isImuInitialized())
     │      ├─ 检查位移 → 可能 "Not enough motion for initializing"   ← 路径④的种子
     │      └─ LocalInertialBA(...)    ★ 惯性 BA，VIBA 1 / VIBA 2 两级
     │
     └─ LocalBA()
```

**关键：IMU 初始化不归 Tracking 管，归 LocalMapping 管。**
所以跟踪阈值会「自己变松」，其实是**另一个线程**把 `isImuInitialized()` 翻成 `true`，
Tracking 只是读这个标志。

`InitializeIMU` 的入口门槛（LocalMapping.cc:1181-1211）：

```cpp
if (mbMonocular) { minTime = 2.0; nMinKF = 10; }

if(mpAtlas->KeyFramesInMap() < nMinKF)                    return;  // ① ≥10 个关键帧
if(mpCurrentKeyFrame->mTimeStamp - mFirstTs < minTime)    return;  // ② 跨度 ≥2.0 秒
```

---

## 五、重置的全部四条路径

**纠正**：重置不止一条路。第一版文档只找到了路径②，把它当成了主因 ——
实测下来它是**本次一次都没走到的**那条。

| # | 位置 | 条件 | 日志 |
|---|---|---|---|
| ① | Tracking.cc:2151 | 跟踪失败 **且** IMU 没初始化完 | `IMU is not or recently initialized. Reseting active map...` |
| ② | Tracking.cc:2019 | `mState==LOST` 且 `KeyFramesInMap()<10` | `A new map is started...` + `Reseting current map...` |
| ③ | Tracking.cc:2589 | 初始化验收 `TrackedMapPoints(1)<50` | `Wrong initialization, reseting...` |
| ④ | Tracking.cc:1805 | LocalMapping 置了 `mbBadImu` | `TRACK: Reset map because local mapper set the bad imu flag` |

### 路径① —— 本次的主因

```cpp
if(bOK)
    mState = OK;
else if (mState == OK)
{
    if (mSensor == System::IMU_MONOCULAR || ...)
    {
        if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
        {
            cout << "IMU is not or recently initialized. Reseting active map..." << endl;
            mpSystem->ResetActiveMap();        // ← 立即清空，不留余地
        }
        mState = RECENTLY_LOST;
    }
```

**只要跟踪失败一次，而 IMU 还没初始化完，地图当场被清空。**
注意条件是 `!isImuInitialized() || !GetIniertialBA2()` —— 就算第一级惯性 BA
（VIBA 1）完成了，第二级（VIBA 2）没完成，**照样清空**。

而且这条路径**绕过了所有宽限期**：直接 `ResetActiveMap()`，不经过 RECENTLY_LOST
的 5 秒等待，也不看 `KeyFramesInMap()`。

### 路径② —— 第一版当成主因的那条

```cpp
else if (mState == LOST)
{
    Verbose::PrintMess("A new map is started...", ...);
    if (pCurrentMap->KeyFramesInMap() < 10)
    {
        mpSystem->ResetActiveMap();
        Verbose::PrintMess("Reseting current map...", ...);
    }
    else
        CreateMapInAtlas();      // 建新图，旧的留在 atlas 里
}
```

判据是 `KeyFramesInMap() < 10`，**但要走很远才能到这**：
得先经过路径①（如果 IMU 没初始化就清空了，根本到不了这里），
或者 IMU 已初始化、进入 RECENTLY_LOST、耗满 `time_recently_lost` = **5.0 秒**
（Tracking.cc:48）才转 LOST。

**实测中这条一次都没走到。**

### 路径③ —— 初始化验收

```cpp
Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", ...);

if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50)
{
    Verbose::PrintMess("Wrong initialization, reseting...", ...);
    mpSystem->ResetActiveMap();
}
```

**注意顺序**：`New Map created` **先打印**，紧接着才验收。
所以那行日志**不等于初始化成功**，它只是说「图建出来了」。

### 路径④ —— LocalMapping 的 bad imu 标志

`mbBadImu` 一旦为真，两件事同时发生：

1. LocalMapping 主循环那个 `if(CheckNewKeyFrames() && !mbBadImu)` 直接跳过 ——
   **整个局部建图停摆**
2. Tracking 下一帧一进来就检查到，`cout << "TRACK: Reset map because local mapper
   set the bad imu flag"` 然后清空

**谁能置这个标志？只有一处 —— 见下一节。**

---

## 六、初始触发：不是「动得太快」，是「动得太少」

路径④的标志由 LocalMapping.cc:136 置位：

```cpp
if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
{
    float dist = (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()).norm()
               + (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter()).norm();

    if(dist > 0.05)
        mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
    if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
    {
        if((mTinit < 10.f) && (dist < 0.02))
        {
            cout << "Not enough motion for initializing. Reseting..." << endl;
            mbResetRequestedActiveMap = true;
            mbBadImu = true;                    // ← 置标志，交给路径④
        }
    }
```

判据拆开看：

| 变量 | 含义 | 阈值 |
|---|---|---|
| `dist` | 最近**两个关键帧间隔的相机中心位移之和** | 触发要 **< 0.02** |
| `mTinit` | 累计的「有效初始化时长」，只在 `dist > 0.05` 时才累加 | 触发要 **< 10 秒** |

**触发条件是「连续性位移太小」，不是「运动太快」。**

`mTinit` 只在 `dist > 0.05` 时累加，意味着**系统要求大约 10 秒的持续运动**
才能完成惯性初始化。而中途任何一次关键帧位移掉到 0.02 以下，就前功尽弃 ——
而且不是重来，是把地图整个清掉。

---

## 七、实测诊断（2026-09-19，session_1789703111468）

跑一次完整 session，抓全部日志。608 帧，20 秒。

### 日志里实际出现的消息

```
New Map created with 518 points          ← 第一次初始化成功
start VIBA 1
end VIBA 1
Not enough motion for initializing. Reseting...       ★ 真正的根因
TRACK: Reset map because local mapper set the bad imu flag
SYSTEM-> Reseting active map in monocular case
LM: Reseting current map in Local Mapping...
mnFirstFrameId = 31
239 Frames set to lost

First KF:75; Map init KF:0
New Map created with 84 points
Fail to track local map!
IMU is not or recently initialized. Reseting active map...   ← 之后 11 次都是这条
...（重复 11 次）
```

### 三条消息的出现次数

| 消息 | 次数 | 说明 |
|---|---|---|
| `Not enough motion for initializing.` | **1** | 只在第一轮，此后再没到过这一步 |
| `start VIBA 1` | **1** | 惯性 BA 只尝试过一次 |
| `Wrong initialization, reseting...` | **0** | 路径③一次没走 |
| `A new map is started...` | **0** | 路径②一次没走 |
| `IMU is not or recently initialized.` | **11** | 全是路径①的级联 |

### 最终结果

> ⚠️ **下面这个数字是一次「运气差的运行」，别拿它当基准。**
> 同一份输入重跑三次得到 311 / 312 / 15 帧位姿 —— 这个系统本身不可复现，
> 原因见 §八末尾。下面其余的分析（日志计数、IMU 实测、因果链）不受影响，
> 因为它们在各次运行之间是一致的。

```
Map 0 has 6 KFs
CameraTrajectory.txt      37 行
KeyFrameTrajectory.txt     6 行
```

608 帧里只有 37 帧有位姿，而且**全在最后**：位姿覆盖 t = 19.03 s ~ 20.23 s
（即第 571 ~ 607 帧）。也就是说前面 19 秒全废，只有最后一小段的地图活了下来。
（好的那次是 315 帧位姿、首帧出现在第 9.76 秒 —— 正好是 `mTinit` 过 10 的时刻。）

> 注意 `CameraTrajectory.txt` 里的时间戳是**纳秒**（EuRoC 输出格式），
> 要除以 1e9 才是秒。详见 §九。

### IMU 实测：开头两秒是静止的

| 秒 | 平均角速度 (rad/s) | 加速度标准差 (m/s²) |
|---|---|---|
| **0** | **0.0785** | **0.112** |
| **1** | **0.0789** | **0.079** |
| 2 | 0.3527 | 0.083 |
| 3 | 0.3963 | 0.133 |
| 5 | 0.9460 | 0.412 |
| 9 | 1.1778 | 0.383 |

0.079 rad/s ≈ **4.5 °/s**，是手抖的量级；后面 0.35~1.2 rad/s（20~70 °/s）
才是有意的运动。**第 2 秒才开始真正移动。**

时间线对得上：第一张地图的初始化在**帧 31（≈1.0 秒）** 完成
（日志 `mnFirstFrameId = 31`），紧接着 VIBA 就判了「运动不足」并清空 ——
而 IMU 数据表明这一刻之前一直是静止的。

### 因果链

```
开头 2 秒几乎不动
   ↓
地图建起来了（518 点），VIBA 1 跑完
   ↓
检查关键帧位移：dist < 0.02，且 mTinit < 10
   ↓
"Not enough motion for initializing" → mbBadImu = true
   ↓
LocalMapping 停摆 + Tracking 清空地图（路径④）
   ↓
新地图重建，但 IMU 永远初始化不完
   ↓
此后每次跟踪失败都撞上路径①：!isImuInitialized() → 立刻清空
   ↓
而且 !isImuInitialized() 时跟踪阈值是严的 50（不是松的 15）
   ↓
更容易失败 → 更容易清空 → 死循环（11 次）
```

**注意最后这个循环的核心**：`!isImuInitialized()` 这一个条件同时导致
「阈值严」和「一失败就清空」。这才是真正的死锁机制 ——
第一版文档里「两个 10 撞上」的说法是个漂亮的猜想，但**不是实际发生的事**。

---

## 八、对采集的指导

### 结论跟第一版相反

| | 第一版（纯推理） | 实测结论 |
|---|---|---|
| 开头 | 「必须**慢**」 | 「必须**动起来**，但不能太快」 |
| 主因 | 跟踪阈值被打破 | **惯性初始化拿不到足够的位移** |

### 真正要满足的是一条时间梯子，不是某一个瞬间

2026-09-19 补充：上面那张表只说到「要动」，**不够**。往下读源码发现，
ORB-SLAM3 单目惯性的启动是**一串挂在 `mTinit` 上的定时事件**：

| `mTinit` | 发生什么 | 代码 |
|---|---|---|
| > 5 s | VIBA 1 | LocalMapping.cc:205 |
| > 10 s | 「运动不足」检查**永久关闭** | LocalMapping.cc:138 |
| > 15 s | **VIBA 2** → `GetIniertialBA2()` 置位 | LocalMapping.cc:218 |
| > 25/35/45… s | 尺度精化（每次只有 0.5 s 窗口） | LocalMapping.cc:232 |

而清空地图的条件是 `!isImuInitialized() || !GetIniertialBA2()`
（Tracking.cc:2151）——
**在 VIBA 2 置位之前，任何一次跟踪失败都立刻清空地图，没有宽限期。**

关键在 `mTinit` 怎么涨（LocalMapping.cc:134）：
**只在每关键帧间隔「相机光心位移 > 5 cm」时才累加。** 所以它约等于
「你在持续走动的时间」，站着不动、原地转，它一点都不涨。

**实测佐证：**20.3 秒的 session，最好的一次首帧位姿出现在第 **9.76 秒**
（正是 `mTinit` 过 10 的时刻），而 `start VIBA 2` 四次运行里出现 **0 次** ——
`mTinit` 到结束都没到 15。**整个 20 秒里那个清空条件恒为真**，
所以稍微一点时序抖动就翻车。这也解释了为什么同一份数据跑三次，能出 311/312/15
三种结果。

### 采集规范

1. **录 60 秒以上。** 目标是让 `mTinit` 跑到 15 s 以上（VIBA 2 完成）。
   20 秒在当前设计下是**结构性不够**的 —— 不是运气问题。

2. **前 15 秒持续走动，一次都别停。** 这一段的每一秒都在往 `mTinit` 里攒。
   停一下不算「暂停」，是**整个地图被清掉、重新开始攒**。

3. **必须是平移，不能只原地转。** `dist` 量的是**相机光心位移**。
   举着手机转身，光心几乎不动，`mTinit` 一秒都不涨。正常步行的速度就够。

4. **但也别乱甩。** 运动过快会让 ORB 特征模糊、帧间匹配数掉下来，
   撞上跟踪失败。存在一个窗口：**持续、有明显平移、但不剧烈**。

5. **场景纹理要够。** 纹理稀疏的墙面会让跟踪一直在及格线附近，
   一点抖动就掉出去。挑有东西看的方向走。

6. **「停下来拍照」有前提。** 你的交互是「先建图 → 再停下拍照」——
   现在能给出确切约束了：**等 VIBA 2 跑完（约 15 秒有效运动）再停**。
   这条得落到 App 的提示里，否则用户一停下来就前功尽弃。

7. **一次失败之后基本救不回来。** 第一轮失败会置 `mbBadImu`，
   之后进入 §七 那个死循环。**最实际的办法是停掉重录。**

### 已按上面规范重采验证过：成立

`session_1789820470653`，65.1 秒 / 1870 帧，跑三遍：

| | 旧 20.3 s session | 新 65 s session |
|---|---|---|
| 位姿帧 | 311 / 312 / **15** | **1730 / 1730 / 1744** |
| 关键帧 | 74 | 239 ~ 246 |
| `start VIBA 2` | **0 次** | **1 次** ✓ |
| 首帧位姿 | 9.76 s | 4.2 s |
| 三次跑的离散度 | 差 20 倍 | 差 0.8% |

1870 帧里 1730 帧有位姿（92.5%），单地图、无重建。
**`start VIBA 2` 终于出现** —— 那正是 `GetIniertialBA2()` 置位、清空条件解除的时刻。

尺度旁证：路径长 30.1 / 32.6 / 30.5 m，包围盒对角线三次都是 3.2 m，
中位瞬时速度 0.53 m/s —— 端着手机慢走的速度，说明米制尺度大致是对的。
**但这是旁证不是真值**，验精度要另做（见下）。

### 下一步能做的验证

1. **验尺度（不需要标定，优先做）**：用卷尺量一段已知直线（比如 3 m），
   端着手机走一趟，比轨迹算出来的长度。这直接对应 App 的最终用途（测距），
   而且现在就能做。

2. **插桩**：把 `isImuInitialized()`、`mnMatchesInliers`、`KeyFramesInMap()`、
   `mTinit`、`dist` 实时打出来，盯一次采集就能看到初始化是一步步成的
   （或怎么失败的）。这是诊断，不是调参。

### 已知的方法论问题：运行结果不可复现

同一份输入、同一份配置，跑三次得到 311 / 312 / 15 帧位姿。
根源是 `LocalMapping`/`LoopClosing` 各跑一个线程，而 Tracking（主线程）读
`KeyFramesInMap()` / `mTinit` / `dist` 时它们正在并发增删关键帧；
`mbBadImu` 更是**一个线程写、另一个线程读，没加锁**。

**影响：改一个参数、跑一次、看结果的 A/B 对比方法在这个系统上不成立。**
要对比就得跑 N 次取中位数，或者先解决确定性。

**不过这个问题的严重程度取决于系统离悬崖有多远**：旧 20.3 s session 上差 20 倍，
新的 65 s session 上只差 0.8%（1730 / 1730 / 1744）。
也就是说它主要是「初始化没过关」的**症状**，不是独立的第三个 bug ——
但**没有归零**，做参数对比时仍然要跑多次。

---

## 九、数据格式核对：一次虚惊

诊断过程中一度以为发现了大问题，记在这里免得以后再查一遍。

**症状**：`CameraTrajectory.txt` 里的时间戳是 `34018334103837.000000`（14 位），
而三份数据文件里的时间戳也是 14 位（`33999304339253`）—— 看起来像是
**纳秒一路传进了 ORB-SLAM3 内部**，而 EuRoC 约定是秒，差 1e9 倍。

**结论：虚惊，单位链路是通的。**

### 三处事实

**1. 上游的 EuRoC 时间戳文件本身就是纳秒整数**
`Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt`：

```
1403636579763555584
1403636579813555456
```

而 `LoadImages` 里那句 `vTimeStamps.push_back(t/1e9);` 正是把 ns 转成秒。
**我们的 `timestamps.txt` 格式与上游完全一致，不用改。**

**2. `LoadImages` 一行读两个 token**

```cpp
stringstream ss;
ss << s;
vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");   // ← 整行当文件名
double t;
ss >> t;                                                        // ← 再读数字当时间戳
```

`ss.str()` 返回的是**整个流内容**，不是「已读部分」—— 所以第一行就把整行拿去当文件名了。
设计上依赖「文件名就是时间戳」，EuRoC 和我们的 App 都是这个约定，所以能用。

**3. `SaveTrajectoryEuRoC` 输出时乘回 1e9**

```cpp
f << setprecision(6) << 1e9*(*lT) << " " << setprecision(9) << twb(0) << " " ...
```

内部 `mTimeStamp` 是**秒**，写出来的 14 位数字是 **EuRoC 规定的输出格式**，
`evo` 之类的工具要的正是这个。

### 链路

```
文件(纳秒) → 加载时 /1e9 → 内部(秒) → 保存时 *1e9 → 文件(纳秒)
```

**要自己解析 `CameraTrajectory.txt` 时，记得先除以 1e9。**

---

## 相关

- 诊断的完整过程与结论修正：[../changes/2026-09-19-tracking-reset-diagnosis.md](../changes/2026-09-19-tracking-reset-diagnosis.md)
- 阶段三的调试经过与遗留问题：[../changes/2026-09-18-imu-camera-startup-sync.md](../changes/2026-09-18-imu-camera-startup-sync.md)
- 主教程阶段四（标定）：[ANDROID_SLAM_TUTORIAL.md](ANDROID_SLAM_TUTORIAL.md) §5
