# 阶段四：传感器标定

阶段三结束时 `phone.yaml` 里几乎全是占位值（主点取图像中心、畸变全 0、
`T_b_c1` 平移全 0）。这份文档是**把它们换成实测值**的流程。

> **先读这一句**：阶段三的跟踪问题（地图反复重置）**根因不是标定**，
> 是采集的 session 太短。换成 65 秒后位姿从 37 帧涨到 1730 帧。
> 所以本文档解决的是**精度**，不是**能不能跑通**。
> 背景见 [ORB_SLAM3_TRACKING_FLOW.md](ORB_SLAM3_TRACKING_FLOW.md) §八。

---

## 一、为什么不用 Kalibr

原计划是 Kalibr，**在本机装不了**：

- Kalibr 只支持 ROS 1；ROS 1 的最后一个发行版 Noetic 只出到 **Ubuntu 20.04**，
  且已于 2025 年 5 月 EOL。
- 本机是 **Ubuntu 24.04**，官方源里只有 ROS 2（Jazzy），而 **Kalibr 不支持 ROS 2**。

要装只能绕道 Docker（`ethz-asl/kalibr` 仓库带 `Dockerfile_ros1_20_04`），
需要 sudo，几个 GB。**但内参标定根本不需要这套。**

| 目标 | 用什么 | 需要 sudo 吗 |
|---|---|---|
| **相机内参 + 畸变** | **OpenCV（`slam` 环境里已有）** | 不要 |
| IMU 噪声参数 | numpy / scipy 的 Allan 方差 | 不要 |
| 相机–IMU 外参 | Kalibr（Docker）或 Basalt（源码构建） | **要，且是大工程** |

关键在于 `phone.yaml` 用的是 `Camera.type: "PinHole"`，它的畸变模型就是
`k1, k2, p1, p2` —— 和 `cv2.calibrateCamera` 的输出**一一对应**。
Kalibr 在这件事上没有任何优势。

**外参放到最后。** 旋转已经有了：`phone.yaml` 里 `T_b_c1` 的旋转部分是从
`SENSOR_ORIENTATION = 90` 解析推导的，行列式 +1，是对的。缺的只有平移
（杆臂），而手机里相机模组和 IMU 芯片相距 1~3 cm。收益最小、代价最大，最后做。

---

## 二、环境

已经建好了，**不用再装**：

```bash
conda activate slam      # 或直接用 ~/software/miniconda3/envs/slam/bin/python3
```

里面有 `cv2 5.0.0`、`numpy`、`scipy`、`matplotlib`、`aruco`。

---

## 三、标定板

已经生成好了：[`tools/calibration/charuco_9x6.png`](../../tools/calibration/charuco_9x6.png)
（1920x1280，9x6 方格，40 个内角点，DICT_5X5_1000）。

**用 ChArUco 而不是普通棋盘格**：角点靠 ArUco marker 定位，
板子只入镜一部分也能用 —— 手持采集宽容得多。

### 怎么用

**在电脑显示器上全屏打开这张图**，然后端着手机对着屏幕拍。

> 没有打印机（本机没有 `lp`），所以用屏幕。**这完全没问题** ——
> 内参标定对物体点的**尺度不变**：`fx`/`fy` 的单位是像素，与方格实际多大无关。
> 所以不需要尺子，也不需要知道屏幕的物理尺寸。

屏幕亮度调到中等偏亮，**注意避开反光**（手机和屏幕之间别有直射光源）。

---

## 四、采集

用现有 App 录一次，和平时一样，只是这次镜头对着屏幕上的板子。

**要录成什么样：**

1. **板子要占画面较大比例** —— 大约 1/3 到 4/5 屏。太小了角点定位不准。
2. **换角度**，这是重点：
   - 正对，然后上下左右四个方向倾斜（每个方向 20°~40°）
   - 绕光轴转（横竖斜都来）
   - 远一点、近一点
3. **画面各个区域都要覆盖到** —— 板子移到画面的**四角**和**中央**。
   角落尤其重要：镜头边缘的畸变靠它们约束，只在中央拍的话畸变系数是不可信的。

   > ⚠️ **这是第一次标定唯一没做到的一条**，代价很具体：四角占比只有
   > 2.0 / 4.3 / 3.6 / 7.3%（门槛 5%），结果 `k2` 的符号在重标时都会翻。
   > 内参没受影响，但畸变相当于白标了。**重录时把主要精力放在走四个角上。**
4. **慢慢来，每个姿态停一下再动。** 运动模糊会让角点检测不准。
5. **整板尽量都入镜**（不是硬性要求，但入得多更准）。

**60~90 秒足够。** 录完用平时的流程拉到 PC。

> ⚠️ 采集时**不需要**满足阶段三那条「前 15 秒持续走动」的规范 ——
> 那不是 SLAM 在建图，只是录一段带板子的视频。别把两件事混起来。

---

## 五、跑标定

```bash
S=~/SLAMAPP/dataset/selfrun/sessions/<你的 session>
~/software/miniconda3/envs/slam/bin/python3 \
    ~/SLAMAPP/tools/calibration/calibrate_camera.py "$S"
```

它会：

- 逐帧找 ChArUco 角点，丢掉角点太少的帧
- 均匀抽稀（视频 30fps，相邻帧几乎一样；不抽稀的话"在哪儿停得久哪儿权重就大"）
- 跑 `cv2.calibrateCamera`
- 丢掉重投影误差 > 1px 的帧重标一次
- 输出覆盖图 `coverage.png` 和完整结果 `calibration_result.txt`

结果写在 session 旁边的 `<session>_calib/` 目录。

---

## 六、怎么看结果

### 1. RMS 重投影误差

**小于 0.5 px 算好，0.3 px 以内很好。** 大于 1 px 说明采集有问题，重录。

### 2. 覆盖图（`coverage.png`）

红点是检测到的角点位置。**九宫格各格都该有东西。**
如果角上全是空的，`k1`/`k2` 就是瞎猜的 —— 脚本会在终端里把这个比例打出来，
九格都该在 5% 以上。

**覆盖不好是重录的唯一理由，比 RMS 更能说明问题。**

### 3. 内参与真值的对照

脚本输出 fx/fy/cx/cy。对照原来的估算值：

| | 估算值 | 2026-09-20 实测 |
|---|---|---|
| fx / fy | 422.7 | **439.23 / 439.38** |
| cx | 320.0（图像中心） | **318.43** |
| cy | 240.0（图像中心） | **239.86** |
| k1 | 0（未知） | **+0.018222** |
| k2 | 0（未知） | **−0.005362** ⚠️ 见下 |

fx 差 +3.9%。估算值假设了整个传感器宽度都映射到整幅图像，实际 ISP 会裁切 ——
**以实测为准**。主点不在正中心（cx 偏左 1.6 px），这就是标定的收益之一。

> ⚠️ **k1/k2 的可信度比四个内参低一档。** 把检出帧随机取子集重标 6 次：
> `cx` 只差 ±0.7 px、`fx` ±2%（可信）；而 `k1` 差 1.9 倍，
> **`k2` 连符号都会翻（−0.0233 ~ +0.0013）= 没测出来**。
> 根因是采集时画面四个角没覆盖到。
>
> 另外拟合时必须**用 `CALIB_FIX_K3` 把 k3 钉住** —— 自由拟合 k1/k2/k3 会互相
> 抵消出 ~10% 畸变的荒谬解（手机镜头不可能有这么大）。钉住后 RMS 只差 1.2%。
> 脚本已经这么写了。

> ⚠️ **2026-09-28 补充：上面这些 fx / k 的可信度还有一个未排除的变量。**
> 标定时对焦在屏幕上（近距离），而**测量时对焦在地板/远处** —— 对焦会移动镜组，
> 可能改变有效焦距。三次标定的 fx 跨会话抖 3.9%，成因可能是这个，
> 也可能是"覆盖不足导致畸变挤进 fx"。
> **两个假设都还站着，判别实验没做。**
> 详见 [INTRINSICS_VS_FOCUS.md](INTRINSICS_VS_FOCUS.md)。
>
> 在此之前，本节的内参值仍按「当前最优」使用，但不要当成已定论。

---

## 七、填回 phone.yaml

编辑 `ORB_SLAM3/Examples/Monocular-Inertial/phone.yaml`，
把 `Camera1.fx` / `fy` / `cx` / `cy` / `k1` / `k2` / `p1` / `p2` 换掉。

> **已经做过了**（2026-09-20，值见上一节的对照表）。本节留着是因为重录之后要再做一次。

- **`k3` 不要填** —— PinHole 模型里没有这一项（日志里那句
  `Camera1.k3 optional parameter does not exist...` 就是在说这个）。
- 记得把那段「这是估算值」的注释一起改掉，否则下次读会误以为还是猜的。

改完重跑一次，对比跟踪质量：

```bash
cd ~/SLAMAPP/ORB_SLAM3
S=~/SLAMAPP/dataset/selfrun/sessions/session_1789820470653
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt \
    ./Examples/Monocular-Inertial/phone.yaml "$S" "$S/timestamps.txt" calib
```

> ⚠️ **这个系统单次运行不可复现**（同一份输入跑三次能出差 20 倍，详见
> [ORB_SLAM3_TRACKING_FLOW.md](ORB_SLAM3_TRACKING_FLOW.md) §八末）。
> **跑一次看结果不算数**，至少跑三次。

---

## 八、之后（还没做）

### IMU 噪声参数

`phone.yaml` 里的 `IMU.NoiseGyro` / `NoiseAcc` / `GyroWalk` / `AccWalk`
现在是照 BMI26x 数据手册估的。可以用 **Allan 方差**从实测数据算出来：
把手机**完全静止**放 1~2 小时（睡前放着就行），采一段，用 numpy/scipy 算。
不需要 ROS。

### 相机–IMU 外参

最后做，见第一节。需要 Docker + Kalibr，或者从源码建 Basalt。

---

## 相关

- [ORB_SLAM3_TRACKING_FLOW.md](ORB_SLAM3_TRACKING_FLOW.md) —— 跟踪流程与重置路径
- [`tools/calibration/`](../../tools/calibration/) —— 标定脚本与标定板
- [../changes/2026-09-19-tracking-reset-diagnosis.md](../changes/2026-09-19-tracking-reset-diagnosis.md) —— 为什么跟踪问题的根因不是标定
