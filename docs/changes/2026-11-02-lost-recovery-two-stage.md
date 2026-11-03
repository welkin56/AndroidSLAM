# 2026-11-02 丢失后不重置：两级视觉恢复打破位姿死锁

## 动机

快速转视野时，内点在**一帧之内**从 57 掉到 2，然后整整 5 秒恒为 0，最后 `CreateMapInAtlas()` 重建地图 —— 坐标系重锚，已钉的测量点全部作废。

用户的观察是：「我一般是快速移动视野，虽然短时间关键点会很低，但之后会恢复的」。也就是说画面回来了，系统却回不来。

## 根因：自举死锁，不是「画面没回来」

匹配有两类，只有一类能被死锁困住：

| 匹配方式 | 依据 | 依赖位姿 | 谁在用 |
|---|---|---|---|
| 投影匹配 `SearchLocalPoints` | 按当前位姿把地图点投到画面，在 `th` 尺度像素内找 | **是** | `TrackLocalMap` |
| 描述子匹配 `SearchByBoW` | 同一 BoW 词袋节点内比描述子距离，**完全不看位置** | **否** | `TrackReferenceKeyFrame`、`Relocalization` |

`RECENTLY_LOST` 期间 IMU 子分支只做 `PredictStateIMU()`，然后走 `TrackLocalMap` —— **只有依赖位姿的那一类**。

于是：位姿偏 2° → 投影偏 15 px（`0.0349 × 426`）→ 正好卡在 `th=15` 的窗边；偏 3° 彻底出窗 → 搜不到点 → `mnMatchesInliers=0` → 位姿不更新（只在优化成功后写）→ 下一帧继续偏。

「内点恒为 0 而不是逐渐衰减」这个形态本身就说明是锁死，不是在漂。

两条支撑事实：

- **`TrackWithMotionModel()` 在 IMU 初始化后是空壳**（`2901-2906`：`PredictStateIMU(); return true;`），不做任何视觉匹配。所以 OK 状态下 `if(!bOK) bOK = TrackReferenceKeyFrame();`（1990）**永远不触发** —— 那条兜底是死代码。正常跟踪时也在靠投影匹配，只是位姿没偏所以没暴露。
- `Relocalization()` 三个调用点（2039/2079/2115）在 `IMU_MONOCULAR` 下一个都走不到。

## 改法：两级

在 `RECENTLY_LOST` 分支的超时判断之后、IMU 分支收尾之前插入。只在 `bOK && mCurrentFrame.isSet()` 时执行（超时帧 `bOK=false`，不试）。

| | 方法 | 覆盖场景 | 代价 | 失败有无副作用 |
|---|---|---|---|---|
| **级 1** | `TrackReferenceKeyFrame()`，每帧 | 偏了一点、画面还在参考帧视野内 | 3~5 ms | **无**（`nmatches<15` 的 return 在赋 `mvpMapPoints` 之前） |
| **级 2** | `Relocalization()`，每 3 帧 | 转到别处又回来 | 20~100 ms | **有**（先 `SetPose` 再判内点），必须回滚 |

级 2 只在级 1 失败后试。级 1 已经算过 BoW，级 2 的 `ComputeBoW` 命中缓存（`mBowVec.empty()` 为假），零额外成本。

两级都不成时统一回滚：位姿退回 `T_pred`、清空 `mvpMapPoints` / `mvbOutlier`、复原 `mnLastRelocFrameId`。

## 四个不敢省的地方

**① 不设 `mState = OK`，交给紧随其后的 `TrackLocalMap`。**
提前置 OK 会让它失败时掉进 `2181 else if (mState == OK)`：`!GetIniertialBA2()` 时直接 `ResetActiveMap()`，或者把 `mTimeStampLost` 刷成当前时间 —— **5 秒计时器被重置，丢失无限续命**。

**② 不设 `bOK = false`。**
`TrackLocalMap` 是唯一更新 `mnMatchesInliers`、唯一能把状态拉回 `OK` 的地方。置 false 等于丢掉唯一的验证机会。

**③ 必须用 `mCurrentFrame.isSet()` 守卫。**
`2024` 那句 `PredictStateIMU();` 丢弃了返回值。它可能返回 false（无 `mpPrevFrame`、或两支都不进）且不写位姿，此时 `GetPose()` 返回未初始化的 `mTcw`。拿它当 `T_pred` 回滚，会把 `mbHasPose` 从 false 强行变 true —— 而 `SetPose` 只置位、不能复位，这个转变不可复原。**这是本轮唯一一个会引入新错误状态的地方。**

**④ 内点必须带 `Observations()>0`。**
与 `TrackReferenceKeyFrame` 内部的口径一致，排除「只有一次观测、还没被 BA 确认」的点，否则刚三角化的点也会被算成内点。

## 阈值怎么来的

**级 1 门槛 15**：它的真实职责很窄 —— 补回 `TrackReferenceKeyFrame` 在 IMU 模式下 `2814` 无条件 `return true` 删掉的下限（上游非 IMU 分支是 `2817` 的 `nmatchesMap>=10`），并严一档。

门槛容易高估：`nGood` 只是**预筛**，不决定关键帧质量。链条是 `nGood` → 喂不喂 → `TrackLocalMap` 局部地图补匹配 → `mnMatchesInliers` → 回不回 OK → 建不建 KF。量级差一个数量级：`nGood` 10~40，`mnMatchesInliers` 历史 p50=212。

**夹角闸 级 1 = 30°、级 2 = 45°**：`T_pred` 在丢失 1~2 秒后自身就有 10~15° 不确定度，留一倍余量。级 2 触发时丢失更久、`T_pred` 更不可靠，所以放宽。它的作用是拦 BoW/PnP 误匹配 —— `PoseOptimization` 第三轮会关掉鲁棒核（`Optimizer.cc:1040-1041`），错解可以内部自洽，单看内点数拦不住。

**平移闸只用于级 2，且放到 3 m**：单目尺度不一定收敛好，卡太死会误杀。

## 一个已知的不对称：短丢失恢复不会立锚

`NeedNewKeyFrame` 的最终判据是 `((c1a||c1b||c1c) && c2) || c3 || c4`，其中 `c3 = dtKF >= 0.5` 是**独立项**，不走 `c2`。所以恢复当帧只要 `dtKF >= 0.5` 就会建 KF，不看内点多少。

唯一能挡的是「弱帧推迟」（`mnMatchesInliers < 45 && dtKF < 1.0`）⟹

- **丢失 < 1 秒就恢复的**：内点低 → 被挡住 → 不建 KF（安全档）
- **丢失 ≥ 1 秒才恢复的**：`dtKF >= 1.0` → 不挡 → 必建 KF

这类 KF 的位姿至少是**视觉修正过的**（不是纯 IMU 外推），比改动前「丢失期间完全不立锚」面对的情况要好。但如果日志显示内点只有十几就建 KF 并随后又丢，下一步要在 `NeedNewKeyFrame` 加「刚恢复 N 帧内不立锚」。

## 文件

- `thirdparty/orb_slam3/src/Tracking.cc`
  - 匿名命名空间：`kLostRefKFMinInliers` / `kLostRefKFMaxRotDeg` / `kLostRelocMaxRotDeg` / `kLostRelocMaxPosM` / `kLostRelocEvery`
  - `includes`：`<algorithm>`、`<cmath>`
  - `Tracking::RotationDeg()`、`Tracking::TryRecoverByReferenceKF()`、`Tracking::TryRecoverByRelocalization()`
  - `RECENTLY_LOST` 分支内的接入与统一回滚
  - `Relocalization()` 内 `MLPnPsolver` 的作用域回收
- `thirdparty/orb_slam3/include/Tracking.h` — 三个声明 + `TrackReferenceKeyFrame()` 多一个 `pInitPose` 参数
- `thirdparty/orb_slam3/src/MLPnPsolver.cpp` — 补 `~MLPnPsolver()` 的定义。头文件只声明不定义，原版从不 delete 所以链接器不需要这个符号；一旦开始回收就 `undefined symbol`。成员全是 STL/Eigen/标量，`= default` 够用。
- `thirdparty/orb_slam3/src/Optimizer.cc` — `PoseInertialOptimizationLastFrame` 里 `EdgePriorPoseImu` 的 `mpcpi` 空指针守卫（`thirdparty/README.md` 第 35 条）

## 明确没走的路

- **不改 `mnFramesToResetIMU`。** 它在 `Settings` 路径下从未被赋值，是个未定义的 `int`，却参与四处判据（含选哪个优化器）。这条已经查到根：`assets/phone.yaml:11` 有 `File.version: "1.0"`，于是 `System.cc:84-85` 走 `settings_ = new Settings(...)`，`Tracking.cc:94-96` 进入 `if(settings)` 分支，`114-123` 那段（唯一赋值点 `mnFramesToResetIMU = mMaxFrames;`）被整段跳过；`newParameterLoader`（`577-659`）只赋到 `mMaxFrames`。本轮不动，但它决定「视觉修正会不会被 IMU 边拽回去」。
- ~~不给级 1 写 `mnLastRelocFrameId`~~ → **11-03 已推翻并改掉**。当时的论证混淆了两个量：级 1 的 `nGood` 是 BoW 预筛结果（10~40），而 `TrackLocalMap` 的 `mnMatchesInliers` 历史 p50=212，50 那道坎挡不住真恢复。真机日志证明不写这一行的代价是确定的：级 1 的 8 次"成功"全部只活一帧。现在级 1 和级 2 一样写这一行，见「11-03 真机验证」。
- **不做 `mpReferenceKF` 的父链回退。** 它 bad 之后匹配质量本身不可靠（点观测已被 `SetBadFlag` 抹掉），宁可等级 2 全图。
- ~~不修 `MLPnPsolver` 的内存泄漏~~ → **收尾时已修**。`new MLPnPsolver` 全程不 delete，每个持有长度 `F.N` 的 `mvPoch3d` / `mvPc` / `mvSigma2` / `mvMaxError`，按候选数泄漏 60~100 KB/次。原版在 `IMU_MONOCULAR` 下 `Relocalization()` 没有调用点，所以是零成本；级 2 每 3 帧一次把它变成每次丢跟踪约 50 次。修法是在 `vpMLPnPsolvers` 声明后放一个作用域守卫，两个 `return` 出口都能覆盖，`resize` 出来的空槽是 `nullptr`，`delete nullptr` 安全。

## 11-03 真机验证：级 1 在抖，级 2 成功后崩

30 秒、故意制造跟踪不稳的运行（`/tmp/slam-1102b.log` + `crash-1102b.log`）。

**两级都通了，机制没走错。** 级 1 成功 8 次，夹角只有 1.0~6.5°；级 2 成功 1 次，夹角 0.45°、平移 0.29 m。`T_pred` 比预想的准，30°/45° 这两个闸其实一直在放行。

**但级 1 全是假成功。**

| 指标 | 值 |
|---|---|
| `TRACK_REF_KF` 被调用 | 131 次 |
| 级 1 成功 | 8 次（约 6%） |
| 成功时的内点 | 15、15、15、15、16、16……全卡在门槛上 |
| 恢复后存活 | `距丢失` 只有 0.033 / 0.033 / 0.199 / 0.267 / 0.367 / 0.533 秒 |

`0.033 秒` 就是一帧。系统在「这一帧判丢 → 下一帧勉强过线 → 再下一帧又丢」之间振荡，一次都没真回到 OK。原因就是上文那条当时存疑的「不给级 1 写 `mnLastRelocFrameId`」：`TrackLocalMap` 走惯性优化，拿 33 ms 的预积分残差把刚修正的位姿拽回漂移轨迹。

**级 2 成功 17 ms 后崩溃。** tombstone：

```
#00 EdgePriorPoseImu::EdgePriorPoseImu(ConstraintPoseImu*)+268   fault addr 0x0
#01 Optimizer::PoseInertialOptimizationLastFrame(Frame*, bool)+5668
#02 Tracking::TrackLocalMap()+804
```

`Optimizer.cc:5117` 的 `new EdgePriorPoseImu(pFp->mpcpi)`，而 `pFp->mpcpi` 是 null —— 上一行刚判过空，只打了一句永远不输出的 `PrintMess`（`VERBOSITY_NORMAL` 实际是 QUIET）。`mpcpi` 只有 `PoseInertialOptimizationLastKeyFrame` / `LastFrame` 末尾会生成，**纯 `PoseOptimization` 从不生成**；而级 2 恢复当帧因为 `mnLastRelocFrameId` 刚被设成当前帧，走的就是纯 PO。于是下一帧 `pFp` 指向一个 `mpcpi` 为 null 的帧，当场崩。已加守卫，见 `thirdparty/README.md` 第 35 条。

**顺带定死了 `mnFramesToResetIMU` 的值：是 0。** 最后一行 `RESETING FRAME!!!` 只在 `2268` 的 `mnId == mnLastRelocFrameId + mnFramesToResetIMU` 成立时打印，而 `mnLastRelocFrameId` 刚被设成当前帧 —— 只能是 0。所以它不是垃圾值，是"碰巧的 0"，但没有任何代码保证（见上文证据链）。

**另外**：启动后 5 秒内 `Reseting active map` 13 次、`地图被重置` 7 次，全在 IMU 初始化之前。这条连锁跟本次改动无关，是 `2234` 的 `!GetIniertialBA2()` → `ResetActiveMap()`。

## 验证

**未构建**（`wsl.exe` 被本机安全策略拦着）。

```bash
cd /home/welkin/SLAMAPP/SlamCapture
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb logcat -G 16M && adb logcat -c
adb logcat -v threadtime -s slam:V > /tmp/slam-1102.log
```

判读：

| 日志表现 | 结论 | 下一步 |
|---|---|---|
| `[TRK] 钥匙帧恢复` 后 1 秒内停 `Fail to track`，无后续 `地图被重置` | 级 1 够用 | 保留，调阈值 |
| 只有 `[TRK] 全图重定位` 成功 | 级 1 不够，级 2 补上了 | 考虑把级 1 门槛调低 |
| `[TRK]` 出现但内点只到 5-8 然后归零 | 惯性优化把修正拽回去了 | 下一轮改 `3013`，加「刚恢复」标志走纯视觉 |
| 只有 `匹配不足` / `拒收`，从没成功 | 匹配数根本不够 | 看 `拒收` 里的夹角分布：普遍 >45° 说明 `T_pred` 已漂废 |

改前基线（2026-10-31 实测）：4 分钟 14 次 `地图被重置`，其中 10 次是同一条 `IMU is not or recently initialized` 连锁。

## 遗留

- ~~惯性优化可能把视觉修正拽回去~~ → **11-03 实测确认，已修**：级 1 成功后也写 `mnLastRelocFrameId`，`3132` 让这一帧走纯 `PoseOptimization`。见「11-03 真机验证」。
- **级 1 的标记写了不回滚。** 级 1 判定通过就写 `mnLastRelocFrameId`，但如果随后 `TrackLocalMap` 没过（要求 50 内点），调用方的回滚只处理 `!recovered` 那条路径，标记会留在前进后的值上，导致接下来 `mMaxFrames` 帧的验收都被这条严格期压着。这算"惩罚"不算"错误"，先观察日志再决定要不要在 `TrackLocalMap` 失败时回滚。
- **`mnFramesToResetIMU` 未定义**（见「明确没走的路」）。
- **`mnLastFrameSeen` 的残留不可回滚。** 级 1 内部给 outlier 点设过 `mnLastFrameSeen = mnId`，回滚时清空了 `mvpMapPoints` 但这些点仍会被 `SearchLocalPoints` 跳过。量级 = 一次匹配的 outlier 数（个位数），可接受。
- **级 2 的节流值 3 是猜的。** 要实测掉帧情况再定。
