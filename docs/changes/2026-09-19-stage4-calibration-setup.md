# 2026-09-19 阶段四：相机标定工具链与流程

## 动机

阶段三的跟踪问题解决后（见 [另一篇](2026-09-19-tracking-reset-diagnosis.md)），
按原计划进入阶段四标定，目标是替换掉 `phone.yaml` 里的占位值
（主点取图像中心、畸变全 0）。

用户问「做全套要不要物理设备量杆臂」—— 借此把标定的范围和路线定清楚。

## 改动

- **新增** `tools/calibration/charuco_9x6.png` —— ChArUco 标定板（1920x1280，40 角点）
- **新增** `tools/calibration/calibrate_camera.py` —— 标定脚本
- **新增** `docs/tutorial/STAGE4_CALIBRATION.md` —— 完整流程
- **没有改动任何源码**，`phone.yaml` 也还没动（要等真实标定结果）

## 决策

**1. 否决 Kalibr —— 本机装不了。**
Kalibr 只支持 ROS 1；ROS 1 最后一个发行版 Noetic 只出到 Ubuntu 20.04。
本机是 **Ubuntu 24.04**，官方源只有 ROS 2，而 **Kalibr 不支持 ROS 2**。
要装只能走 Docker（`ethz-asl/kalibr` 带 `Dockerfile_ros1_20_04`），要 sudo + 几个 GB。

**但内参标定根本不需要它** —— `phone.yaml` 用的是 `Camera.type: "PinHole"`，
其畸变模型就是 `k1,k2,p1,p2`，和 `cv2.calibrateCamera` 的输出一一对应。

**2. 用 ChArUco 而不是普通棋盘格。**
角点靠 ArUco marker 定位，**板子只入镜一部分也能用**。手持采集时这个宽容度很值钱。
（`findChessboardCorners` 要求整板可见。）

**3. 用屏幕代替打印。**
本机没有打印机（`lp`/`lpr` 都没有）。**这对内参标定毫无影响** ——
`calibrateCamera` 对物体点的**尺度不变**，`fx`/`fy` 的单位是像素，与方格实际多大无关。
所以连尺子都不需要。

**4. 外参放到最后。**
旋转已有解析解（从 `SENSOR_ORIENTATION=90` 推导，行列式 +1，是对的），
只缺 1~3 cm 的平移。而它需要 Docker+Kalibr 或源码构建 Basalt ——
**收益最小、代价最大**。回复用户时纠正了自己一个说法：全套标定**不需要**物理量具，
工具是从数据里估外参的；卡住的是工具链不是设备。

**5. 之前把「我装不了」说成了「装不了」，已更正。**
CLAUDE.md 里原写「WSL2 + WSLg，无 sudo」。用户指出：不是不能用 sudo，
是要先经他确认。已改。**差点因为这条把整条 Kalibr 路误判成不可能。**

## 验证

**合成数据还原测试**（`/tmp/synth_calib_test.py`，临时脚本未保留）：
用**已知内参**把标定板投影渲染成 640x480 的 60 帧假采集，再让脚本去还原。

| | 真值 | 标定还原 |
|---|---|---|
| fx | 422.7 | **423.49** |
| fy | 422.7 | **423.51** |
| cx | 320.0 | **319.77** |
| cy | 240.0 | **239.81** |

误差 0.2% 以内，RMS 0.24 px。残差来自合成时 1920→640 的重采样，不是脚本的问题。
**整条链路（读 csv → ChArUco 检测 → 抽稀 → calibrateCamera → 出报告）验证通过。**

顺带确认覆盖检查会正确报警：合成的位姿没覆盖到画面顶行，
脚本如实打出 `0.0% 0.0% 1.5%`。

**未验证：真实采集上的表现。** 还没有真实标定数据。

## 遗留

- **真实标定一次都没跑过。** 上面验的是脚本，不是手机相机。
- **`phone.yaml` 还没改** —— 等真实结果。
- **IMU 噪声参数没做**（Allan 方差，需 1~2 小时静止采集），外参没做。
- **`tools/` 不在任何 git 仓库里** —— 和 `ORB_SLAM3/` 一样没有版本控制保护。
  目前就两个文件，觉得碍事的话可以再议。
- **尺度真值没验**：本想用卷尺量一段已知直线走来验证米制尺度，用户没有卷尺，跳过了。
  这个验证不需要标定就能做，等有尺子随时可补。

## 回滚

```bash
rm -rf ~/SLAMAPP/tools/calibration
rm ~/SLAMAPP/docs/tutorial/STAGE4_CALIBRATION.md
rm ~/SLAMAPP/docs/changes/2026-09-19-stage4-calibration-setup.md
```

没有任何源码被改，`phone.yaml` 保持原样。
