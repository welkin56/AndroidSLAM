# 2026-10-31 弱帧不立锚：关键帧插入推迟到质量够的帧

一句话：不改「插不插关键帧」，改「用哪一帧插」。内点不够的帧先不建 KF，等下一个够的帧；
已判定丢失的帧完全不建。**动的是 `thirdparty/orb_slam3/src/Tracking.cc` 和 `phone.yaml`，
不碰测量侧。**

## 动机

用户报：跑一段之后 SLAM 漂得很远，「小地图越跑越远，标记也跟着飘，之后关键点变少几乎没了」，
而且**漂了就回不来**，后续没法再标。原话是「一两帧比较糊的时候特征点减少，地图就飘走」。

一开始想做模糊度门控，被否掉了，理由成立：焦距锁定，靠近会糊但特征点未必少，
Laplacian 方差的阈值靠不住。改用一个直接描述位姿可信度的量 —— `mnMatchesInliers`
（`TrackLocalMap` 每帧算好，`Tracking.cc:3024-3045`），它数的是「有多少地图点真的支持
这个位姿」，跟特征总数无关。

## 排查结论：不是「插太多」，是「在错的地方立锚」

| 事实 | 位置 |
|---|---|
| `c4 = (15 < mnMatchesInliers < 75) \|\| mState==RECENTLY_LOST`，命中就插 | `Tracking.cc:3202` |
| `c1b` 恒真（`mMinFrames=0` + LocalMapper 空闲）、`c3` 是 0.5 s 无条件 | `Tracking.cc:3178`、`3191` |
| `InsertKFsWhenLost` 默认 true，`phone.yaml` 缺这个键 | `Tracking.cc:1326` / `Settings.cc:425` |
| `LocalInertialBA` 的固定锚 = 时间窗外那一帧，滑动、且是上一轮刚优化过的 KF | `Optimizer.cc:2437-2446` |
| `LocalBundleAdjustment` 只锁 `GetInitKFid()`，全图仅第一帧是硬锚 | `Optimizer.cc:1220` |

也就是说：**跟踪一变弱，上游就补一张关键帧**。意图是「缺图补图」，但它默认了这一帧位姿可信 ——
而这时位姿恰恰最不可信。这张 KF 进入 `mPrevKF` 链后成为后续 BA 的基准，误差只累加，没有回收机制。

## 改法：三档

| 帧质量 | 判据 | 动作 |
|---|---|---|
| 强 | `mnMatchesInliers ≥ 45` 且 `mState==OK` | 照上游规则（c1/c2/c3/c4 原样） |
| 弱 | `mn < 45` 且 `mState==OK` | **推迟**：不插，等下一个强帧；距上一 KF >1.0 s 仍无强帧则放行一次 |
| 丢失 | `mState != OK` | **不插**，无上限 |

两条边界：

- **1.0 s 放行**：`mpImuPreintegratedFromLastKF` 从上一 KF 开始积分，臂越长 `PredictStateIMU`
  误差越大（随 t²）。插一张不完美的锚好过让位姿靠纯惯性硬推。
- **丢失档不设上限**：从丢失恢复靠的是 `TrackLocalMap` 重新匹配上（`mn>10` 即回 `OK`，
  `Tracking.cc:3053`），不依赖新关键帧。

## 文件

| 文件 | 改动 |
|---|---|
| `app/src/main/assets/phone.yaml` | 新增 `IMU.InsertKFsWhenLost: 0`（键名**必须带 `IMU.` 前缀**） |
| `thirdparty/orb_slam3/src/Tracking.cc` | 匿名命名空间加 `kWeakInliers=45` / `kWeakHoldS=1.0`；`NeedNewKeyFrame()` 加两档判据；`CreateNewKeyFrame()` 末加一行 `[KF]` 日志 |

### 一个必须记住的坑：键名带前缀

这个 App 走的是 `Settings` 解析（`System.cc:85` → `newParameterLoader` → `Tracking.cc:624`），
不是 `ParseCamParamFile` 那条 FileStorage 老路。后者读的是**无前缀**的 `InsertKFsWhenLost`
（`Tracking.cc:1345`），前者读的是 `IMU.InsertKFsWhenLost`（`Settings.cc:425`）。
**写错前缀不会报错，只会静默按 `true` 走。**

## 日志

```
[KF] #412 内点 268 间隔 0.51s          ← 正常插入
[KF] 缓插 内点 31 距上一KF 0.33s        ← 弱帧被推迟
[KF] 丢失不立锚 内点 12 距上一KF 1.8s 状态 3   ← 丢失期间被挡
```

`缓插` 与 `丢失不立锚` 按 `mnId % 15` 限流（30 fps 下约 0.5 s 一行）。

## 待调参数

`kWeakInliers = 45` 是按「正常房间里 `mnMatchesInliers` 有几百」猜的**先验值**。
第一轮日志要先看 `[KF] #... 内点 N` 的分布：如果中位数只有 60~80，45 等于没设，需要下调；
如果中位数是 300，45 可能还偏松。**改一个常量即可。**

`kWeakHoldS = 1.0` 不建议先动。

## 明确没走的路

| 备选 | 否决理由 |
|---|---|
| **模糊度门控** | 锁焦后靠近会糊但特征未必少，阈值不可靠；用户也否了 |
| **弱帧不建新地图点**（跳过 `CreateNewMapPoints`） | 上游正是靠弱帧补新点救场，拦掉会**加速**丢失，方向相反 |
| **弱 KF 在 BA 里 `setFixed(true)`** | 等于把错位姿钉死当基准，比让它自由优化更糟 |
| **弱 KF 从 BA 时间窗里跳过**（`MergePrevious`） | 要动 `mPrevKF` 链和预积分合并；`KeyFrameCulling:1024-1031` 的配方只在**删除** KF 时成立 |
| **删关键帧** | `SetBadFlag` 会连带杀掉只有 2 个观测的地图点（`MapPoint.cc:202-209`）、动生成树、删 KFDB 条目；Tracking 的 `mpReferenceKF`/`mpLastKeyFrame` 会悬垂 |
| **`c3` 的 0.5 s 改 0.8 s** | 能减少 BA 轮次，但同时拉长 `PredictStateIMU` 外推臂，双刃。留作下一轮的旋钮，等第一轮日志 |

## 说清楚上限

**这不消除漂移。** 弱帧期间视觉约束本来就没有，位姿只能靠 IMU 外推，漂是物理。
这个改动做的是**不让漂移被写进地图** —— 地图还是原来那张，走回去还对得上，
上一轮 `p_kf` 锚定的钉点也不会被带跑。

要减少漂移本身得在别处：提高 `ORBextractor.nFeatures`（现 1200）、改善光照纹理、
或「只在跟踪强时允许钉点」的交互约束。都不在这一轮。

## 验证（未构建）

```bash
cd /home/welkin/SLAMAPP/SlamCapture
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb logcat -G 16M && adb logcat -c
adb logcat -v threadtime -s slam:V MainActivity:V AndroidRuntime:E > /tmp/slam-1031.log
```

判读顺序：

1. `[KF] #... 内点 N` 的分布 —— 先看这个，它决定 `kWeakInliers` 要不要改
2. `[KF] 缓插` 出现的时刻是否与画面变糊 / 走远对齐
3. 走一圈回来，`[MAP] 尺度修正` 的次数是否变少
4. 段长在两次尺度事件之间是否仍一动不动
5. `Fail to track local map` / `BAD LOOP` 是否减少

## 回滚

删掉 `NeedNewKeyFrame()` 里那个 `if(mpAtlas->...isImuInitialized() && ...)` 整块、
`CreateNewKeyFrame()` 末尾那行 `cout`、顶部匿名命名空间，再把 `phone.yaml` 的
`IMU.InsertKFsWhenLost` 删掉或改成 1。无格式变更、无磁盘影响。
