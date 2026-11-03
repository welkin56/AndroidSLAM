# 丢失期间禁用惯性优化：拦住 g2o 内部的 NaN abort

## 现象

两级恢复接上以后，崩溃从 `mpcpi` 空指针变成 Sophus 断言：

```
SO3::exp failed! omega: -nan -nan -nan
SIGABRT
EdgeInertial::computeError()
g2o::SparseOptimizer::optimize()
Optimizer::PoseInertialOptimizationLastFrame()
Tracking::TrackLocalMap()
```

11-03 20:46 这次日志把触发条件钉死了：

- 20:46:10.012 开始 `Fail to track local map!`。
- 级 1 连续给出 30、25、22、20、20、19、18 个内点，但每一次随后都没通过
  `TrackLocalMap`，所以 `mState` 一直是 `RECENTLY_LOST`，从未回到 `OK`。
- 约 1.3 秒后连续出现 6 次 `[OPT] 惯性优化发散（结果含 NaN），本帧状态不写回`。
- 第 7 次没有走到函数末尾的写回检查，直接在 `optimizer.optimize()` 内部 abort。

## 根因

`kRecoverVisualFrames = 30` 只按 `mnLastRelocFrameId` 计算恢复后的纯视觉窗口，
没有看 `mState`。级 1 每次尝试都会推进 `mnLastRelocFrameId`；停止匹配约 30 帧后，
窗口在「实际上还没恢复」的情况下到期，`TrackLocalMap()` 又切回
`PoseInertialOptimizationLastFrame()`。

这时位姿、速度、bias 和相邻两帧预积分并不构成一致状态，Gauss-Newton 可以发散。
末尾的有限性检查只能阻止 NaN **写回 Frame / KeyFrame**，无法阻止它在下一轮
`EdgeInertial::computeError()` 中进入 `SO3::exp`。

21:12 第二次日志推翻了“窗口到期”这个单点解释：

- 21:11:57.166 级 2 成功，`RESETING FRAME!!!`，状态回到 `OK`。
- 之后稳定跟踪整整 30 帧，21:11:58.164 还正常建了 `[KF] #124`。
- 21:11:58.195 第一次重进惯性优化，立刻 `[OPT] 惯性优化发散`。
- 级 1 随后救回，再稳定 30 帧，再发散；如此循环三次，最后一步在
  `optimizer.optimize()` 内部 abort。

所以窗口本身没有错，错在**纯视觉段不会生成 `Frame::mpcpi`**。恢复后第一次惯性优化
拿不到上一帧先验，上一帧位姿、速度、bias 全都在优化集里，却只有 33 ms 的
`EdgeInertial` 约束它们，方程欠定，Gauss-Newton 直接发散。延长窗口只会把同一个
必炸点往后推。

## 改动

### 1. 丢失 / 冷却期间只做纯视觉

`Tracking::TrackLocalMap()` 的优化器选择增加条件：

```cpp
if (mState != OK || 在恢复后的纯视觉窗口内 || 惯性优化冷却中)
    Optimizer::PoseOptimization(&mCurrentFrame);
else
    惯性优化;
```

惯性优化返回 `-1`（发散）时，本帧立即退回纯视觉，并进入 30 帧冷却。帧本身不会被
写坏：g2o 顶点持的是 `ImuCamPose` 副本，只有函数末尾才写回 `Frame`。

### 2. 优化改成逐步检查，拦住 abort

`PoseInertialOptimizationLastFrame()` 里原来的 `optimizer.optimize(10)` 改成十次
`optimize(1)`，每次之后检查位姿、速度、陀螺 bias、加计 bias 是否全有限。一旦不是，
立即返回 `-1`。这样 NaN 不会活到下一次 `computeActiveErrors()` 里被送进 `SO3::exp`。

### 3. 输入检查补齐

- `PreintegratedFinite()` 检查 `dT`、`dR/dV/dP`、全部 Jacobian、`C`、`Info`；
  `PredictStateIMU()` 两个分支都用它，避免 NaN 在 `GetDeltaRotation()` 内部 abort。
- `PoseInertialOptimizationLastFrame()` 入口补查两帧位姿是否已设置且有限、
  预积分 `dR/JRg` 是否有限。

之前的三层防护保留：

1. `PreintegrateIMU()` 丢弃非有限测量和非正 `tstep`。
2. 惯性优化入口检查 bias、速度、`dT`、`C`、`Info`；写回前再查结果。
3. `PredictStateIMU()` 使用前检查 bias 与预积分内部量。

同轮审查又修了三处状态/防护漏洞：

- 级 1/级 2 给出候选但 `TrackLocalMap()` 验收失败时，回滚 `mnLastRelocFrameId`。
- `RECENTLY_LOST` 的 IMU 分支接收 `PredictStateIMU()` 的返回值。
- `PoseInertialOptimizationLastFrame()` 缺前帧或预积分对象时退回纯视觉，
  不再“缺指针反而跳过检查”。

## 没做的事

- 不实现上游空着的 `ResetFrameIMU()`。当前修复保证不再 abort，但纯视觉段之后
  惯性优化仍可能每 30 帧失败一次并退回纯视觉。要让 IMU 真正无缝接回，需要用它
  保存的恢复段重新估计速度和 bias，或者显式构造一个弱先验，恢复 `mpcpi` 链。
- 不调整级 1 内点门槛，不动 `mnFramesToResetIMU`。

## 验证

**未构建。** 下一次日志重点看：

- 不再出现 Sophus / `SIGABRT`。
- 可能出现 `[OPT] 惯性优化第 N 轮第 M 步发散，本帧退回纯视觉`，之后应继续跟踪，
  不应掉成 `RECENTLY_LOST`。
- 若 `[OPT]` 每 30 帧一次但不影响跟踪，说明安全修复生效、IMU 状态重建仍待做。
- 若不再出现 `[OPT]`，说明当前场景下 30 帧纯视觉已足够让状态重新一致。
