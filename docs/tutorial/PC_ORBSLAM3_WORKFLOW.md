# PC 侧工作流：跑 ORB-SLAM3 并判读结果

> 覆盖主教程的 **阶段一**（用公开数据集确认通路）和 **阶段三**（用自采数据验证采集层）。
> 两者是同一件事——**拿数据、跑、判读**——只是数据来源不同，所以合在一份文档里。
>
> 设计原理见 [ANDROID_SLAM_TUTORIAL.md §1](ANDROID_SLAM_TUTORIAL.md#1-关键认知)，
> 线程模型与地图重置见 [ORB_SLAM3_TRACKING_FLOW.md](ORB_SLAM3_TRACKING_FLOW.md)。

---

## 1. 调用约定（先记住这一条）

```
mono_inertial_euroc <vocab> <settings> <seq_folder> <times_file> [file_name]
```

| 参数 | 说明 |
|---|---|
| `<vocab>` | `ORBvoc.txt`，139 MB |
| `<settings>` | 相机 + IMU 参数 yaml |
| `<seq_folder>` | 数据目录 |
| `<times_file>` | 时间戳文件，**每行一个纳秒整数** |
| `[file_name]` | **只影响输出文件名**，不影响算法 |

`[file_name]` 传了就输出 `kf_<name>.txt` / `f_<name>.txt`，
不传就输出 `CameraTrajectory.txt` / `KeyFrameTrajectory.txt`（**覆盖上一次**）。

**做对比时必须传它**，否则多次运行的结果互相覆盖，没法并排看。
（2026-09-19 核实：源码里 `bFileName` 只出现在保存轨迹那一处。）

---

## 2. 阶段一：用 EuRoC 确认通路

**目的不是跑出结果，是建立"什么是正常的轨迹"这个判断。** 拿一个已知答案
（EuRoC 带 ground truth）标定你的眼光，阶段三才有的比。

```bash
cd ~/SLAMAPP/dataset
mkdir -p EuRoC && cd EuRoC
# 从 ETH 官网下载 MH_01_easy.zip（约 1.5GB）
# https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
unzip MH_01_easy.zip
```

目录结构（**注意 `mav0/` 这一层**，本工程踩过：App 写出来的目录少一层）：

```
dataset/EuRoC/MH_01_easy/
└── mav0/
    ├── cam0/data/*.png
    ├── imu0/data.csv
    └── state_groundtruth_estimate0/data.csv
```

时间戳文件仓库里已经带了：

```bash
cd ~/SLAMAPP/ORB_SLAM3
./Examples/Monocular-Inertial/mono_inertial_euroc \
    ./Vocabulary/ORBvoc.txt \
    ./Examples/Monocular-Inertial/EuRoC.yaml \
    ~/SLAMAPP/dataset/EuRoC/MH_01_easy \
    ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt \
    euroc1
```

**判据**：轨迹与 ground truth 的**尺度对得上**（ATE RMSE 几十厘米量级，不是几米）。
尺度差几倍 → 多半是 IMU 噪声参数或时间戳口径。

---

## 3. 阶段三：用自采数据验证采集层

**这一步不能跳过。** 它的作用是把"算法问题"和"采集问题"彻底分开——
跳过它，等到 JNI 阶段出问题，你分不清是采集错了、桥接错了、还是 SLAM 参数错了，
**三个变量同时不确定，调试会变成玄学**。

### 3.1 装 App、抓日志、拉数据

```bash
APK=~/SLAMAPP/android/SlamCapture/app/build/outputs/apk/debug/app-debug.apk
adb install -r "$APK"

adb logcat -c
adb logcat -s MainActivity:V CameraCapture:V ImuCollector:V SessionWriter:V \
    | tee /tmp/slam_capture.log
```

录一段（**要求见 3.2**），然后拉回来：

```bash
DEST=~/SLAMAPP/dataset/selfrun/sessions
adb pull /sdcard/Android/data/com.example.slamcapture/files/sessions/. "$DEST"

S=$(ls -d "$DEST"/session_* | grep -v _calib | tail -1)
ls "$S"      # cam0/  imu0/  meta.json  timestamps.txt
cat "$S"/meta.json
```

**`meta.json` 要看的三项**：

| 字段 | 应该是 |
|---|---|
| `frames_dropped` / `imu_dropped` | 0 |
| `first_imu_ts_ns` vs `first_frame_ts_ns` | 前者**小于**后者，差约 3×10⁸（≈300 ms） |
| `forced` | `false` |

### 3.2 录制规范（这一条是拿几十次失败跑换来的）

| 要求 | 为什么 |
|---|---|
| **60 秒以上** | 惯性初始化是挂在 `mTinit` 上的时间梯子，VIBA2 需要约 15 秒的有效运动。20 秒的 session 结构性地不够 |
| **前 15 秒持续平移** | 激励不足会陷入「重置 → 更爬不出来」的自我循环 |
| **摆好装置再按录制** | 前 15 秒在摆弄装置 = 没有激励 |
| **变速走动** | 匀速平移对加速度激励贡献很小 |

### 3.3 `timestamps.txt` 要自己生成

`mono_inertial_euroc` 需要它，但**App 不写这个文件**（源码里没有任何引用）——
它是 PC 侧从 `cam0/data.csv` 的第一列取的：

```bash
cut -d, -f1 "$S/cam0/data.csv" | grep -v '^#' > "$S/timestamps.txt"
```

格式与上游 EuRoC 的完全一致（每行一个纳秒整数），`LoadImages` 直接吃。

### 3.4 跑

```bash
cd ~/SLAMAPP/ORB_SLAM3
S=$(ls -d ~/SLAMAPP/dataset/selfrun/sessions/session_* | grep -v _calib | tail -1)

./Examples/Monocular-Inertial/mono_inertial_euroc \
    ./Vocabulary/ORBvoc.txt \
    ./Examples/Monocular-Inertial/phone.yaml \
    "$S" "$S/timestamps.txt" selfrun1
```

> ⚠️ **这个系统单次运行不可复现。** 同一份输入跑三次能差出 2 倍以上
> （受控 A/B 实测：同配置下位姿 1730 vs 819，重置次数 7 vs 52）。
> **跑一次看结果不算数，至少跑三次**，看中位数和分布。详见
> [ORB_SLAM3_TRACKING_FLOW.md](ORB_SLAM3_TRACKING_FLOW.md)。

### 3.5 判据

| 现象 | 结论 |
|---|---|
| 跑完并出轨迹，关键帧数正常，8 次重置都能恢复 | ✅ 采集没问题，可以进入阶段五 |
| CPU 单核空转、不输出 | ❌ 老版 `while(!eof())` 死循环，见主教程 §9 |
| `IMU 与相机时间关系不满足前提` | ❌ 采集侧问题，回阶段二 |
| `Failed to load image at: ...` | ❌ `timestamps.txt` 与 `cam0/data/` 的文件名对不上，或 pull 没拉全 |
| 全程重置循环、跟踪帧数很少 | ❌ 多半是激励不足，回 3.2 |

**不只看"有没有出轨迹"，还要看轨迹体检**：关键帧数、位姿覆盖率、重置次数、
VIBA2 有没有跑过。

---

## 4. 用 evo 看轨迹

```bash
cd ~/SLAMAPP/ORB_SLAM3
evo_traj tum f_selfrun1.txt --plot                    # 看形状
evo_traj tum f_selfrun1.txt --ref groundtruth.csv -p --plot_mode=xyz   # 比尺度
```

用 `f_*.txt`（每帧轨迹）而不是 `kf_*.txt` 跟 evo 对比——点更密，曲线更连贯。

**evo 装了没**：在 conda 环境里，不要 `pip install`（见 CLAUDE.md 第三节）。

---

## 5. 输出文件与清理

跑完在工作目录（`ORB_SLAM3/`）得到：

| 文件 | 内容 | 给谁用 |
|---|---|---|
| `f_<name>.txt` | **每一帧**的位姿（丢跟踪的帧不写） | evo 对比用这个；`1661/1721` 那个覆盖率是指它 |
| `kf_<name>.txt` | **只看关键帧**的位姿 | 看关键帧数（`161 KF`）；`measure_marks.py` 读的也是它 |

格式都是 EuRoC/TUM 那套：`时间戳 x y z qx qy qz qw`。

> ⚠️ **`f` 和 `kf` 很容易记反。** `f` = frames（全部帧），`kf` = keyframes。
> 旧版主教程把这两个写反过——已修正。（核实来源：`System.cc` 里
> `SaveTrajectoryEuRoC` 遍历的是 `mlRelativeFramePoses`，即每帧。）

不传第五个参数时是 `CameraTrajectory.txt`（每帧）/ `KeyFrameTrajectory.txt`（关键帧），
**并且会覆盖上一次**。

> 顺带：第五个参数的存在性是按**参数个数的奇偶**判断的
> （`bFileName = ((argc-3) % 2 == 1)`，`mono_inertial_euroc.cc:53`）。
> 所以「4 个参数」和「5 个参数」是两种模式，多给一个就会被当成文件名。

```bash
# 每次开跑前清一下，避免把上一次的结果当成这次的
rm -f kf_*.txt f_*.txt CameraTrajectory.txt KeyFrameTrajectory.txt
```

**ORB-SLAM3 目录里不要 `git commit`**——它是上游 clone，提交会被后续 `git pull`
冲掉，改动留在工作区就好。
