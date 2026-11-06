# 2026-11-05 IMU 噪声标定搬进 App：静置录一段，当场算 Allan 方差

## 动机

用户原话：「还有什么能在手机 APP 上直接标定的也做进去，实际用户使用是不用电脑的」。

盘了一遍现有的标定项，只剩 IMU 四个噪声参数还在 PC 上——`tools/calibration/allan_variance.py`
要 Python + numpy + matplotlib，普通用户装不起来。其余项其实都已经在 App 内自动完成：

| 项 | 现在怎么做 |
|---|---|
| `fx fy cx cy k1` | 9 步引导标定页 |
| 对焦档位索引 | 自动读 `LENS_FOCUS_DISTANCE` |
| `IMU.T_b_c1` 的旋转 | `SettingsTemplate.tbc()` 按 `SENSOR_ORIENTATION` 生成 |
| `IMU.Frequency` | 从设备读出后 patch 进 settings |
| **四个噪声参数** | **只有这一项还在 PC 上** |

而 `phone.yaml` 里现在的 `GyroWalk 1.9e-05` / `AccWalk 3.0e-03` 其实是从 ORB-SLAM3 的
EuRoC 配置抄来的（`1.9393e-05` / `3.0000e-03`），描述的是 ADIS16448，不是这台手机。
顺带查出 `NoiseGyro` 当年抄的时候多写一个 0（EuRoC 是 `1.7e-4`，写成了 `1.7e-3`），
这就是 09-23 文档里「原值高估 19 倍」的真正来源——不是估高了，是抄错一个数量级。

## 改动

- `app/src/main/java/com/example/slamcapture/AllanVariance.kt` —— **新增**。Allan 方差与幂律拟合，
  与 `tools/calibration/allan_variance.py` 逐行对应。每轴只算一次 adev，白噪声和游走共用同一条
  曲线、只换拟合目标（脚本算了四遍，结果相同，慢一倍）。
- `app/src/main/java/com/example/slamcapture/ImuCalibActivity.kt` —— **新增**。`IDLE / RECORDING /
  COMPUTING / REVIEW` 四态；静置检测；计时；结果页四值 + 质量标记；保存确认与注入。
  每轴斜率打进 logcat（`gyro.x noise C=... slope=...`），界面只显示折算后的档位，
  看不出移植对不对；`profile.json` 的 `quality` 与界面用同一条判据（两路白噪声取最差）。
- `app/src/main/res/layout/activity_imu_calib.xml` —— **新增**。
- `CalibProfile.kt` —— 新增 `ImuNoise`，`CalibProfile` 加 `imuNoise: ImuNoise? = null`
  （命名参数构造，`CalibActivity` 那处不用改），`version` 1→2，读写 `imu_noise` 对象。
- `SettingsTemplate.kt` —— `patch()` 加 `noise` 参数；噪声四行整行重写（`%.3e`，注释随值重建）；
  `IMU.Frequency` 从「整行重写」改成「只换值、保留行尾注释」。原来的 `%.1f` 对 `8.8e-05`
  会写成 `0.0`。
- `MainActivity.kt` —— `resolveSettingsPath()` 把 `prof?.imuNoise` 传进 `patch`。
- `SlamBridge.kt` / `jni_bridge.cpp` / `slam_engine.h` / `slam_engine.cpp` —— `nativeSetImuNoise`
  与 `setImuNoise()`，后者照抄 `setIntrinsics` 的 `running` 门闩。
- `thirdparty/orb_slam3/{include,src}/System.*` —— `SetImuNoise` 纯转发。
- `thirdparty/orb_slam3/{include,src}/Tracking.*` —— `SetImuNoise`：`mpImuCalib->Set(Tbc, ng*sf, ...)`。
- `thirdparty/README.md` —— fork 改动清单加一行（`SetImuNoise`），并在「注意」里补一条：
  `Frame::mImuCalib` 是值拷贝、已有的 Preintegrated 不回溯；`Settings` 无 setter，
  所以「立即生效」只能走这条路径。
- `AndroidManifest.xml` / `activity_calib_list.xml` / `CalibListActivity.kt` —— 页面注册与入口。

## 决策

- **否决：塞进 `CalibActivity`。** 那个页面处处依赖相机预览和 9 步几何判据，加一条「不用相机、
  要静置 20 分钟」的分支会让状态机变复杂，而且两个流程的按钮语义完全不同。
- **否决：复用 `SessionWriter` 落盘。** 它固定建 `cam0/data` 目录、有自己的 writer 线程与有界队列，
  为纯内存标定建一套空相机目录、再把 26 MB 文本读回来解析是净亏。
- **否决：C++ 实现 Allan。** 纯 1-D 标量运算，走 JNI 只多两次大数组拷贝和一层调试盲区；
  本机 `wsl.exe` 被安全策略拦住，编译不了，而 Kotlin 能直接和 Python 并排逐行审。
- **否决：导出原始数据。** 用户明确不要。代价是没法拿同一份数据在 PC 上逐位对账，
  见「验证」一节的三条间接判据。
- **否决：给游走项开手动写入开关。** 四个数值都显示、只有两个可信，这个信息已经完整；
  给按钮反而要求用户理解「这个数不可信」才能正确用。
- **采用：分块累积，每块 65536 条 × 6 通道。** 中间一度改成「一条连续大数组 + 1.5 倍扩容」，
  只算了稳态 40 MB、没算扩容时新旧两份同时存在的峰值（60 分钟 67 MB、顶到上限 101 MB），
  已改回。分块每加一块只分配 1.57 MB，60 分钟 23 块 = 36 MB，无扩容峰值。
  代价是算法要按块取样本，用移位和掩码（65536 = 2^16）代替除法。
- **采用：游走项双条件**（时长 ≥ 60 分钟 **且** 斜率在 `0.5 ± 0.15` 内）。60 分钟是经验门槛，
  不是保证——09-23 用 18 分钟时四轴斜率是 `+0.08/+0.01/-0.05/-0.05`，没人验证过 60 分钟
  一定到 `+0.5`。时长达标但斜率不达标，一样保留原值。
- **采用：算完先问是否保存。** 时长不足或静置度差时，用户可以选择不写。
- **关键发现：`System` 是进程级的，`nativeInit` 只跑一次。** `nativeStop()` 只置 `running=false`，
  不销毁 `sys`；回到 `MainActivity` 走的是 `openHardware()`，不会重新构造 `Settings`。
  所以改 yaml 只在**下次冷启动**生效，「立即生效」必须走运行时注入，两条路都要。

## 验证

**算法等价性：已在 PC 上用纯 Python 验算，通过。**

```
$ python3 verify_allan.py
tau 点数 48  (12.42 ms ~ 1.65 s)
有效 tau 48 / 48
代数展开最大相对误差 5.057e-15
斜率：闭式解 -0.517833  vs 中心化 -0.517833  差 2.81e-14
Noise C=9.9065e-05  slope=-0.499  tau*=0.059 s
Walk  C=2.7984e-03  slope=-0.478  tau*=0.035 s
```

三条都对着：

1. 代数展开（`csum[j+2m] − 2csum[j+m] + csum[j]`，不落地 `win`/`diff` 两个 n 长数组）
   与原实现的相对误差 `5e-15`，是浮点极限；
2. 斜率闭式解与中心化实现差 `3e-14`，说明 `polyfit(deg=1)` 的移植没走样；
3. 合成数据（白噪声 + 慢漂移）拟合出 `slope=-0.499`，白噪声段识别正确。

顺带印证了一件事：合成数据里 walk 拟合出 **`-0.478` 而不是 `+0.5`**——τ 只到 1.65 s，
同样停在平顶段。这和真机 20 分钟的表现是同一个现象。

**真机验证（2026-11-06，静置 15.3 分钟）：**

| 项 | 实测 | 对 09-23 PC 标定 |
|---|---|---|
| `NoiseGyro` | `8.610e-05` | −2.1% |
| `NoiseAcc` | `1.057e-03` | −3.7% |
| 采样率 | 402.51 Hz | 一致（历史 402.5） |
| 静置 / 丢样 | 99.57% / 0 | — |
| 游走项 | `walk_updated: false`，yaml 两行原样 | 符合设计 |
| `files/phone.yaml` | 值与注释都换了、`%.3e` 正常、`IMU.Frequency 402.5` | — |
| `profile.json` | `imu_noise` 段结构完整，`imu_frequency_hz` 写实测 | — |

两个白噪声各差 2~4%，正好是标定本身的重复性量级，说明算法移植没走样。
15.3 分钟短于建议的 20 分钟，所以 `quality` 是 `rough`。

**这次没验到的：** 六轴斜率。结果页只显示折算档位，没把每轴 `slope` 打出来——已在
`logDetail()` 补上 logcat 输出，下次标定能看到。

**仍未验：**

| 项 | 期望 |
|---|---|
| 注入日志 | `[IMU] 噪声注入 ng=... na=...`（标定保存后立刻出现在 logcat） |
| 注入后初始化 | `nativeGetTrackingState()` 从 1 走到 2；被写成 0/NaN 会卡在 1 |
| 三轴离散 | Gyro ≈ 7%、Acc ≈ 25% |

## 遗留

- **原始数据不导出**（用户定）。若上面三条判据都不过，需要临时加一个「保存原始数据」开关，
  拿同批数据在 PC 上跑脚本定位。这是后路，不是计划内步骤。
- **60 分钟能不能真标出游走项，未实测。** 现有 `GyroWalk/AccWalk` 仍是 EuRoC 那台
  ADIS16448 的值。设计上已经允许试（时长 + 斜率双条件），但没跑过一次 60 分钟。
- **静置检测的三个盲区无法从数据内部识别**：手机垫软布（高频抖动被吸收）、桌子本身在晃、
  手扶在桌沿。只能靠文案和 `stillRatio` 提示。
- **累积上限 1,800,000 条**（约 75 分钟 @400 Hz），到顶停止累积并提示。内存账：
  28 块 × 1.57 MB = 42 MB，加计算时的 `csum`（`DoubleArray(n+1)`）14 MB，峰值 56 MB。
  60 分钟那个常用档位是 36 + 12 = 48 MB。是为了不让 Java heap 被撑爆，正常使用碰不到。
- `assets/phone.yaml:87-89` 那段「阶段四用 imu_utils 做 Allan 方差分析拿到真实值」是陈述性错误
  ——实际用的是自写脚本，`imu_utils` 从来没用过。本轮没改。

## 回滚

按顺序：

1. 删 `AllanVariance.kt`、`ImuCalibActivity.kt`、`activity_imu_calib.xml`、
   `docs/changes/2026-11-05-imu-allan-ondevice.md`。
2. `CalibProfile.kt`：删 `ImuNoise`、`imuNoise` 字段、`noiseToJson/noiseFromJson`，`version` 改回 1。
3. `SettingsTemplate.kt`：`patch()` 去掉 `noise` 参数与 `writeNoise/setNoiseLine`；
   `replaceValue` 可以留着（它只是比原来的整行重写更保守）。
4. `MainActivity.kt`：`patch(...)` 调用去掉第三个参数。
5. native 侧：撤 `SetImuNoise` 四处、`setImuNoise`、`nativeSetImuNoise`。
6. manifest / 列表按钮 / `CalibListActivity` 的入口。

只回滚「写入」而不回滚「计算」也可以：把 `SettingsTemplate.patch` 的 `noise` 参数传 null，
噪声参数就不再进 yaml，App 仍然能测出来给自己看。
