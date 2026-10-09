# 2026-09-18 修复两处卡死：ORB-SLAM3 读越界 + App 侧 IMU 晚于相机

> 这篇是**补记**（2026-09-19 补写），记录阶段三调试中修掉的两个挂起 bug。
> 改动本身写在 09-18，记录写在 09-19，内容据当时的调试记录整理。

## 动机

阶段三的目标是「在 PC 上用 `mono_inertial_euroc` 跑手机采集的数据，验证采集本身没问题」。
第一次跑就卡住不退出了，而且**不报错**。查下来是两个独立的 bug，前后脚暴露：

**症状 1** —— 输出停在这里就不动了，进程一直活着、CPU 占满单核：

```
Error: Cannot open time stamps file: .../timestamps.txt
LOADED!
Loading IMU for sequence 0...LOADED!
```

**症状 2** —— 把症状 1 的路径问题绕过去之后，换一个 session 又不对了。
这次进程能跑完，但轨迹是垃圾：每帧配到 0 个 IMU 测量。

两个 bug 的共同点是**都不崩、不报错**，所以特别费时间。

## 改动

### bug A — C++ 侧读不存在的文件

- `ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc.cc`
  - 加了 `ResolveSeqRoot()`：自动判断目录里有没有 `mav0/` 这一层，两种布局都能吃。
    上游代码写死 `pathSeq + "/mav0/cam0/data"`，而 App 输出的目录**没有** `mav0/`。
  - 两处文件打开加了 `is_open()` 守卫，打开失败就打错误退出。

  真正的坑不在「打开失败」，而在**失败之后的行为**：`LoadIMU` 里是
  `while(!fImu.eof())`。文件打不开时 `getline` 失败，**置位的是 failbit 不是 eofbit**，
  所以 `eof()` 永远不为真 → 无限空转。这才是「不报错也不退出」的来源。

- 同一个文件里还有一处**越界读**，顺手一起修了（就是下面 bug B 的 C++ 侧）。

### bug B — C++ 侧读越界内存

- `ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc.cc`
  - 找 `first_imu` 的循环原来没有边界检查，改成「先判再减」，并且
    `fi == 0 || fi == nImu` 时报错退出。

  原版是 `while(...) ++fi;` 然后 `fi--;`。那句 `--` 默认了**循环至少跑过一次**，
  而循环一次都不跑的条件恰好就是「IMU 比相机晚开始」。
  退化后 `fi == -1`，`vector::operator[]` 收的是 `size_t`，-1 变成
  18446744073709551615 → 读堆外内存。**最坏的结果不是崩，是静默给一条烂轨迹。**

  注意一开始差点修漏：只判 `< 0` 是不够的。如果循环跑到 `fi == nImu`，
  递减后得到 `nImu-1`，它不是负数，越界只是**延后一次迭代**发生。
  两个退化都要判。

### bug B — App 侧：让 IMU 先于相机落盘

- `android/SlamCapture/app/src/main/java/com/example/slamcapture/ImuCollector.kt`
  - 加 `onFirstSample` 回调 + `firstSampleSeen: AtomicBoolean`，**在
    `writer.offerImu()` 之后**触发（不是 `onSensorChanged` 一进来就触发 ——
    加速度计还没出数时陀螺样本根本不落盘，信号必须代表「文件真拿到第一行」）。

- `android/SlamCapture/app/src/main/java/com/example/slamcapture/CameraCapture.kt`
  - 加 `focusReady` / `imuReady` 两个 volatile 标志 + `maybeBegin()`，两个都齐了才开闸。
  - 两个等待各配一个**可取消的**超时兜底（`focusTimeout` / `imuTimeout`），
    `stop()` 里必须 `removeCallbacks` —— 否则会话关了之后超时还会响，
    把一个已经停掉的相机重新标成「就绪」。

- `android/SlamCapture/app/src/main/java/com/example/slamcapture/MainActivity.kt`
  - 接线：`imuCollector.onFirstSample = { cam.markImuReady() }`
  - **先** `imuCollector.start()` **再** `cam.start()`：本机传感器要 3 秒多才出数，
    那一刻钟从 `registerListener` 起算，先点着 IMU，相机开设备、配对焦的时间白赚在这段里。

## 决策

- **否决：串行等待。** 我最初的想法是先起 IMU，等它就绪再起相机。否决理由：
  必须用一个固定等待时长，而实测 IMU 冷启动约 **3.3 s**、预热后只要 **11 ms**，
  差 300 倍。写死必然在两种情况下错一种。

- **否决：`CountDownLatch` 阻塞相机线程。** 我最初打算在相机 handler 上
  `awaitFirstSample(timeoutMs)`。否决理由：对焦回调**就跑在相机 handler 上**，
  阻塞它会让 `ImageReader` 的 4 个 buffer 填满，整条管线停住 —— 包括 AE/AWB 停止收敛。
  相机自己都还没准备好，等来的「就绪」是假的。
  → 改成回调 + 两个标志，**纯标记，不阻塞**。

- **采用：两条线并行，谁后到谁触发。** 四个触发点（对焦就绪 / IMU 就绪 / 两个超时）
  都调同一个幂等的 `maybeBegin()`。

- **用户纠正（重要）：** 我曾经在 `cam.start()` 里加了 `imuReady = false`，
  还在 `MainActivity` 里调换顺序来规避它造成的竞态。用户直接否掉：
  「为什么你 camstart 要把 imu 设为 false，你动 imu 东西干什么」。
  **他是对的** —— 我让相机拥有了 IMU 的状态，而且清零正好可能落在 IMU 回调之后，
  把刚置上的标记抹掉。正确做法是 `start()` 和 `stop()` 都**不碰** `imuReady`，
  全工程只有 `markImuReady()` 一个写入点。

## 验证

**bug B 的 C++ 守卫**（在旧 session 上跑，专门验证报错路径）：

```
$ ./Examples/Monocular-Inertial/mono_inertial_euroc .../ORBvoc.txt phone.yaml "$S" timestamps.txt
ERROR: sequence 0 的 IMU 与相机时间关系不满足 ORB-SLAM3 的前提（IMU 必须先于相机开始）。
       first image t = ...
       first IMU   t = ...
       IMU 相对首帧 = 1.74303 s（应当为负）
```

守卫生效，异常输入被拦下来了 ✓

**App 侧门控**（重采一次后量时间关系）：

```
$ ... 从 cam0/data.csv 和 imu0/data.csv 各取首行
imu0[0] - cam0[0] = -0.573 s      ← 负的，IMU 先开始，方向正确
608 帧 / 8424 IMU 样本
IMU 间隔恒 2.484 ms，无超过 10 ms 的缺口
相机 30.0 fps
```

**端到端**：`mono_inertial_euroc` 跑完、退出码 0、输出
`CameraTrajectory.txt` / `KeyFrameTrajectory.txt` ✓

**未验证**：跟踪质量。310/608 帧有位姿，第一帧位姿在采集开始后 9.93 s ——
前半段被反复重置吃掉了。这属于阶段四的范围（见下）。

## 遗留

- **跟踪质量差**，失败模式是 `New Map created with N points` 后立刻
  `Fail to track local map!`，是**视觉**跟踪失败，与 IMU 噪声无关。
  剩下的可疑项全是 Kalibr 会给出真值的：相机主点、畸变、`T_b_c1` 平移。
  用户明确指示：**不要在阶段三上调参**，交给阶段四标定。

- **App 不写 `timestamps.txt`。** 全工程搜不到这个名字 ——
  `cam0/data.csv` 才是 App 的产物（EuRoC 标准的 `#timestamp [ns],filename`），
  但 `mono_inertial_euroc` 第 4 个参数要的是**每行一个裸纳秒数**。
  现在靠手工生成，**每次采集都要重复**：
  ```bash
  tail -n +2 "$S/cam0/data.csv" | cut -d, -f1 > "$S/timestamps.txt"
  ```
  暂时绕过，早晚要让 App 直接写出来。

- **误导性的 `LOADED!`。** `LoadImages` 的 `is_open()` 守卫会提前 return，
  但 `main()` 无条件打印 `LOADED!`。日志里会看到「明明失败了却说加载成功」。

- **`CameraTrajectory.txt` / `KeyFrameTrajectory.txt` 目前是坏的那次跑出来的。**
  中间试过把 `IMU.NoiseGyro` 从 `1.7e-3` 改成 EuRoC 的 `1.7e-4`，
  **结果更糟**（重置 2 次 → 11 次，关键帧 74 → 4），已改回 `1.7e-3`，
  但它覆盖了先前较好的输出。重跑一次即可覆盖回来。

- **`phone.yaml` 没有版本控制**（当时 `docs/` 和 android 仓库都还没建）。
  这是整个链路的核心配置，只存在于工作区。

## 回滚

四处，顺序无关，但**要一起回滚** —— 只回滚一半会让时间关系重新错乱：

1. `ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc.cc` —
   删掉 `ResolveSeqRoot()` 和两处 `is_open()`，把 `first_imu` 循环改回
   `while(...) ++fi; fi--;`。
   （注意：这是在**上游 clone** 里改的，文件自带 `.git`，
   `git checkout -- <file>` 即可还原到这个文件的上游版本。但那样 bug A 也一并回来了。）
2. `ImuCollector.kt` — 删 `onFirstSample` / `firstSampleSeen` 及触发代码。
3. `CameraCapture.kt` — 删 `focusReady` / `imuReady` / `maybeBegin()` /
   `markFocusReady()` / `markImuReady()` 和两个超时，恢复原来的 `beginCapture()`。
4. `MainActivity.kt` — 删 `imuCollector.onFirstSample = ...` 那行。

回滚后 App 会退回「相机不等 IMU 就开始录帧」，在 IMU 冷启动那 3.3 秒里
录到的帧没有对应的 IMU 数据 —— 正是 bug B 的触发条件。

## 相关

- 上游改动留在 `ORB_SLAM3/` 里，该目录自带 git 仓库（`git diff` 可看）。
  **不要**在那里提交，容易被后续 `git pull` 冲掉。
- 阶段三的收尾状态与阶段四的计划，见 [../tutorial/ANDROID_SLAM_TUTORIAL.md](../tutorial/ANDROID_SLAM_TUTORIAL.md) §4、§5。
