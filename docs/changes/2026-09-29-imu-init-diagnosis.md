# 诊断：SLAM 反复重置 —— **是这次录得不好**，不是代码或配置

> **这一份没有代码改动**，是一次**排除法诊断**。记下来是因为它**排掉了一堆我本来打算去查的方向**，
> 那些路走一遍要花很久。用户的话：「这次跑的效果不太好啊」—— **对，诊断的结论就是这个。**

## 动机

M5 之后一直卡在「SLAM 反复重置、轨迹显示到 2 就重置」。
2026-09-29 用户跑了一轮，**开着新做的录制开关**，落下了 `maps/20260929-210256/`（27.7 s、819 帧）。
这是第一次有「带 SLAM 的完整 session」可以用来对着查。

## 改动

**没有改代码。** 只有两件杂事：

- 删了 `dataset/` 里三段标定原始数据（9-20 / 9-22 ×2，共 **1.13 GB**）——
  标定结果已经落成 `tools/calibration/phone_calibrated_2026-09-20.yaml`，`_calib/` 的 txt/png 留着。
  **dataset 从 2130 MB → 997 MB。**
- `CLAUDE.md`：Python 环境写错了（写的 `DeepLearning`，实际是 `slam`），改正。

## 决策

### 一、⭐ 排除掉的方向（这是本记录的主要价值）

| 我曾怀疑 | 怎么排除的 | 结论 |
|---|---|---|
| **采集链路有问题** | 量了这次 session：相机 **29.9 fps**、IMU **402 Hz**、**一次 >5ms 的间隙都没有**、画面 Laplacian 375~1274、ORB 角点 1184~1200 顶格 | ❌ 数据是干净的 |
| **手机运行时的问题**（丢帧队列 / 慢） | **PC 喂同一份数据，失败方式和手机一模一样**（44 次重置、3 个关键帧） | ❌ 和手机无关，**不该在手机上找** |
| **配置 / 环境坏了** | 同一台 PC、**同一份 `phone.yaml`**，跑 9-19 的老 session：**827 个位姿、298 个关键帧、VIBA1 ×3、VIBA2 ×1** | ❌ 环境是好的 |
| **IMU 噪声参数错了** | 用户提醒去看 Allan 那份记录：`8.8e-05` / `1.1e-03` 是三轴均值，斜率 -0.50，**且核实过 `mpImuCalib(Ng*sf,…)` 的换算约定 = yaml 填连续密度、直接填** | ❌ 值是对的（旧的 `1.7e-03` 才是错的） |
| **相机内参错了** | 换成 9-20 实测标定值做对照：轨迹点 **12 → 33**、重置 **44 → 39** | ⚠️ **有帮助但不够**，不是主因（见决策三） |
| **动作太快** | LK 光流量了逐帧像素位移：这次 **10.5 px/帧**，老数据 **6.9 px/帧** | ⚠️ 快一倍多，但**差得不够大**，解释不了「3 帧就丢」 |

### 二、⭐ 找到的机制：**一个自我锁死**

```cpp
// Tracking.cc:2142
if(bOK) mState = OK;
else if (mState == OK) {                       // ← 刚刚还是好的，现在跟丢了
    if (mSensor == System::IMU_MONOCULAR) {
        if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2()) {
            cout << "IMU is not or recently initialized. Reseting active map..." << endl;
            mpSystem->ResetActiveMap();        // ← 立刻清图
            return;
        }
```

链条：

```
单目初始化成功（"New Map created with 528 points"）
      ↓
Fail to track local map!   ← 几帧之内
      ↓
mState 离开 OK，而 IMU 还没初始化
      ↓
Tracking.cc:2149 → ResetActiveMap()      ← 27 秒里 44 次
      ↓
而 IMU 初始化要求 KeyFramesInMap() > 2 且 mTinit 累计 ≥ 10 s（VIBA2 要 15 s）
      ↓
地图每 0.6 秒就被清一次 ⇒ IMU 永远到不了门槛 ⇒ 回到第一步
```

**越重置越爬不出来。** 老 session 之所以能爬出来，是因为它最终攒够了关键帧 —— 这次攒不到。

### 三、相机内参：**该修，但不是根因**

`phone.yaml` 现在用的是**阶段三的估算值**，而它自己的注释就写着「阶段四标定完之后要整体替换」。
阶段四的标定**做过了**（9-20，ChArUco，RMS 重投影 0.2991 px），但**从没合进去**：

| | 现在用的 `phone.yaml` | 9-20 实测标定 |
|---|---|---|
| fx / fy | 435.2 / 435.2 | 439.227 / 439.377 |
| cx / cy | **320.0 / 240.0**（"取图像中心"） | 318.427 / 239.856 |
| k1 / k2 | **0 / 0**（"畸变未知，先用 0"） | 0.018222 / -0.005362 |
| p1 / p2 | 0 / 0 | -0.000274 / 0.000340 |

换成标定值之后轨迹点 **12 → 33（2.7 倍）**、重置 44 → 39 —— **说明内参确实在拖后腿，
但换完仍然失败，所以它不是根因。**

> ⚠️ **没有据此改 `phone.yaml`。** 用户的计划是先重录一段好的，标定值一起合 —— 顺便验一下
> 标定到底准不准（那正是 [INTRINSICS_VS_FOCUS.md](../tutorial/INTRINSICS_VS_FOCUS.md) 悬着的问题）。

### 四、结论：**重录**，按能成功的条件走

9-19 那次能跑通，说明这套东西是 work 的。差别在**录制方式**：

| 要 | 不要 |
|---|---|
| **持续平移**（走起来） | 原地转身 / 站着扫视 |
| **60 秒以上** | 30 秒以内 |
| 让画面有**视差** | 快速甩镜头 |
| 对着**有纵深**的东西 | 只对着一个平面（桌面/墙面） |

> **为什么必须「走」**：`LocalMapping.cc:138` 的 `mTinit` **只在相邻关键帧位移 > 5 cm 时才累加**，
> 而它要爬到 10 s（VIBA2 要 15 s）。站着转，`mTinit` 永远是 0，还会直接触发
> `Not enough motion for initializing. Reseting...`。

**顺带**：重录时对着那根 **1.6 米**的东西完整走一圈 → 尺度验证的素材一起有了。

## 验证

命令和真实输出，全部在 PC 上跑（数据是用户从设备拉下来的）：

### 1. 这次的 session 质量

```bash
$ awk -F, 'NR>1 && $1+0>0 {...}' cam0/data.csv     # 跳过表头
  818 帧 跨度 27.33 s  间隔 min 0.0301 mean 0.0334 max 0.0667  → 29.9 fps
$ awk -F, 'NR>1 && $1+0>0 {...}' imu0/data.csv
  11149 条  平均 2.48 ms  最大 2.5 ms  → 402 Hz
  超5ms 0 次 (0.0%)   超10ms 0   超20ms 0
$ ORB 角点 1184~1200（顶格）；Laplacian 方差 375.5 ~ 1273.8
```

### 2. PC 跑这次的 session（用现在的 `phone.yaml`）

```bash
$ ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt \
    .../assets/phone.yaml <session> <session>/timestamps.txt m5debug
  f_m5debug.txt   12 行
  kf_m5debug.txt   3 行
  "Fail to track local map!"                           44 次
  "IMU is not or recently initialized. Reseting..."    44 次
  "Timestamp jump"                                      0 次
  "start VIBA 1/2"                                      0 次
```

> **注意 `Timestamp jump` 是 0 次** —— 我 M5 时的假设（丢帧 → 帧间隔 >1s → 判跳变 → 重置）
> **是错的**。真正的话是 `IMU is not or recently initialized`。

### 3. 换成标定内参的对照

```bash
$ ... /tmp/settings/B_camcalib.yaml ... bcam
                          基线      换标定内参
  轨迹点数                 12        33
  关键帧数                  3         6
  Fail to track local map  44        39
  IMU not initialized→重置  44        39
```

### 4. 同 yaml 跑 9-19 的老 session（决定性对照）

```bash
$ ... .../assets/phone.yaml session_1789820470653 ... old
  轨迹点数 827   关键帧数 298
  start VIBA 1  3 次     start VIBA 2  1 次     ← IMU 初始化成功过
  Fail to track local map  26     IMU not initialized→重置  20
```

**同一台机器、同一份 yaml，老数据好、新数据坏 ⇒ 差别在那段数据，不在环境。**

### 5. 逐帧像素位移（LK 光流，每 10 帧测一次）

```
9-29 新（失败）: 中位 104.5 px / 0.33 s  → 折算 10.5 px/帧
9-19 旧（成功）: 中位  69.3 px / 0.33 s  → 折算  6.9 px/帧
```

## 遗留

- **重录一段**（决策四的条件）—— 这是下一步，用户做
- **把 9-20 的标定值合进 `phone.yaml`** —— 待定，和重录一起
- **⚠️ 一个顺手发现的小竞态**：`run.json` 写 `"traj_points": 1`，而 `trajectory.txt` **是 0 字节**。
  `exportTrajectory` 在 `nativeStop()` 之后立刻调，但 worker 线程可能还在处理手上那一帧 ——
  dump 完了它才 `savePose`。**不影响正确性**（就是最后一次 dump 少一条），但看着矛盾。
  要修的话：dump 前等一下 worker 静默，或者 dump 两次。
- **相机内参标定的可信度问题**（畸变四角覆盖不足、k2 符号会翻）——
  见 [INTRINSICS_VS_FOCUS.md](../tutorial/INTRINSICS_VS_FOCUS.md)，**仍未定论**
- **`resetCount` 已经上屏** —— 这次 `run.json` 记了 `map_resets: 7`，是它在手机上第一次有用

## 回滚

**没有代码改动，不需要回滚。** 唯一的破坏性动作是删数据（已确认过、`_calib/` 结果保留）。

CLAUDE.md 的改动要退就把 `slam` 改回 `DeepLearning` —— 但那是错的，别退。
