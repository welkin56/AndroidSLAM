# 2026-09-25 教程重构：主教程只留方向，阶段细节下沉；Stage 5 重写

## 动机

用户在三处提了意见，最后合成一句：

> 我都想把主教程这些细节删了，主要是大方向指导，细节还得看各个 stage 的文档，
> 你改一下主教程，还有 stage5 的文档吧，stage5 文档修改或删掉重写都行

起因是我在「阶段四验收」时发现主教程烂了：

- **§0 现状停在阶段一**（「你跑的是 mono_tum，必须换 Mono-Inertial」——那件事
  三个阶段前就做完了）
- **§5 还在教 Kalibr + rosbag**，而 Kalibr 早在 09-19 就被否决（只支持 ROS 1，
  Ubuntu 24.04 装不了）
- **§3/§4 里混着大量实操命令**，和 STAGE2 那份专门文档重复
- 而 **Stage 5 那份「零基础版」其实是检查清单**——凡是需要动手写的骨架全写
  「照主教程 §6.x 抄」，而 §6 本身也是草图。两份加起来不足以让人走通。

## 改动

### 重写 `docs/tutorial/ANDROID_SLAM_TUTORIAL.md`（1081 → 约 300 行）

确立一条**分工判据**，写在文首：

> **这条信息在阶段的实现换掉之后还成立吗？**
> 成立（约束、原理、验收判据）→ 主教程；不成立（命令、路径、代码）→ 阶段文档。

每个阶段统一成四段：**目标 / 关键决策 / 验收判据 / →详情链接**。
保留了 §1 关键认知、§9 排查手册、§10 参考资料——它们不随实现变化。

### 新建 `docs/tutorial/PC_ORBSLAM3_WORKFLOW.md`

把原 §2（EuRoC）+ §4（自采数据）的实操搬过来。**两者合在一份**，因为它们
本来就是同一件事（拿数据、跑、判读），只是数据来源不同。

### 重写 `docs/tutorial/STAGE5_JNI_BRIDGE.md`

补上 README 早就承诺、但文件里根本没有的内容：**App/ART/JNI/NDK 地基**和
**M0~M7 分步里程碑**（每步都写了「过了的标志」）。

### 新建 `STAGE6_PHOTO_MEASUREMENT.md` / `STAGE7_UI_STATE_MACHINE.md`

原 §7/§8 的内容不能就这么删——它们是**已经想过的设计**（三条测距路径、
归一化坐标对齐、状态与可用性映射）。搬到各自的阶段文档，标注「设计说明，未开工」。

### `docs/README.md`

tutorial 列表按新结构重写，并写明分工原则。

## 决策

### 采用：主教程讲「为什么」，阶段文档讲「怎么做」

用户的原话就是判据。落地时用「实现换掉之后还成立吗」这条来分，比按「长短」或
「重要程度」分更可操作——后者每次都靠感觉。

### 否决：把 §7/§8 直接删掉

它们是还没开工阶段的设计稿，删了就是净损失。新建两份文档承接，
比「留在主教程里当特例」更一致——特例一旦开口，下次就不知道边界在哪了。

### 否决：保留 Stage 5 的「照主教程 §6 抄」写法

它把「教程」退化成了「索引」，而 §6 的内容本身是草图级别。
读者拿到两份草图仍然写不出工程。**这就是「删掉重写」那一条的触发原因。**

### 关键：这次顺手核出六个错，都是照着旧文档抄必然踩的

| 旧文档写的 | 实际 | 后果 |
|---|---|---|
| `include_directories(${ORB_ROOT}/Thirdparty/eigen)` | **该目录不存在**（PC 上是 `find_package(Eigen3)` → `/usr/include/eigen3`） | CMake 报找不到头文件 |
| `OpenCV_DIR = opencv/sdk/native/jni` | **本机没有**，`opencv/` 是源码 clone，无任何 `.so` | 第一个 `find_package` 就失败 |
| `Java_com_example_slam_SlamBridge_...` | 少了 `capture` | **必然 `UnsatisfiedLinkError`** |
| yaml 里写 `System.UseViewer: 0` | **不是 yaml 键**，是 `System` 构造函数第 4 个参数 | 被 `FileStorage` 静默忽略，Viewer 照建 → Android 上找不到 Pangolin |
| 「`KeyFrameTrajectory.txt` 关键帧位姿，给 evo 用这个」 | `f_` = **每帧**（1661），`kf_` = **关键帧**（161），说反了 | 拿关键帧轨迹算覆盖率会得到 96.5% 这种好笑的结果 |
| §5 全篇 Kalibr + rosbag | 09-19 已否决 | —— |

### 新发现三个会当场卡住的地雷（写进 STAGE5 §3）

1. **`-march=native` 硬编码在 DBoW2 和 g2o 的 CMakeLists 里**。
   交叉编译到 arm64 会直接报 `unknown target CPU 'native'`。
2. **`src/` 下 16 处 `exit(-1)`**（`System.cc` 7 处 / `Settings.cc` 9 处，另有一处在
   `include/Settings.h` 的内联模板里）。在 Android 上的表现是
   **点一下 App 整个退回桌面、logcat 干干净净**——没有 FATAL EXCEPTION、没有 Java 栈。
   旧文档那套「信号名 → ndk-stack」的排查路线**对它完全无效**，因为它不是崩溃。
3. **本版本词袋只有文本加载路径**（`System.cc` 只调 `loadFromTextFile`，139 MB），
   没有二进制快速路径。所以 `nativeInit` 必须异步，且 `System` 对象不该跟着
   「准备」销毁——否则切一次前后台要重载几十秒。

### 决策：SLAM 的生命周期挂 Activity，不挂「准备」

和 09-21 给 IMU 定的规则**看起来矛盾**，但判据是同一条——**看这个资源的代价类型**：

- IMU / 相机：代价是**耗电** → 跟着用户意图走（`准备` 开、`取消准备` 关）
- SLAM：代价是**时间 + 内存**（139 MB 词袋）→ 跟着进程走（`onCreate` / `onDestroy`）

写在 STAGE5 §5「决策一」里。

## 验证

这次是文档改动，但**所有事实性论断都对着源码和本机核过**，不是照抄旧文。

**1. 六个错的事实来源**

```
$ grep -rn "exit(-1)" src/ include/ | wc -l
16

$ grep -rn "march=native" Thirdparty/*/CMakeLists.txt
Thirdparty/g2o/CMakeLists.txt:57:SET(CMAKE_CXX_FLAGS_RELEASE "... -O3 -march=native")
Thirdparty/DBoW2/CMakeLists.txt:8:set(CMAKE_C_FLAGS "... -march=native ")

$ ls -d opencv/sdk/native/jni
ls: cannot access 'opencv/sdk/native/jni': No such file or directory

$ ls ORB_SLAM3/Thirdparty/
DBoW2  Sophus  g2o            ← 没有 eigen

$ grep -rn "UseViewer" src/ include/
src/System.cc:42:   const bool bUseViewer, ...
include/System.h:105: System(..., const bool bUseViewer = true, ...)
                       ← 是构造参数，不是 yaml 键
```

**2. `f_` / `kf_` 谁是谁**（用 09-23 的实测数字交叉验证）

```
$ sed -n '256,261p' Examples/Monocular-Inertial/mono_inertial_euroc.cc
    const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
```

而 `System.cc` 里 `SaveTrajectoryEuRoC` 遍历的是 `mlRelativeFramePoses`（**每帧**）。
对上 09-23 记录里的「161 KF / 1661 位姿」——`kf_`=161、`f_`=1661。**确认旧文写反了。**

顺带核出第五个参数是按**参数个数的奇偶**判断的（`mono_inertial_euroc.cc:53`
`bFileName = ((argc-3) % 2 == 1)`），已写进文档。

**3. 现场数据印证 09-21 的设计生效**

```
$ cat dataset/selfrun/sessions/session_1790161277904/meta.json
"first_frame_ts_ns": 492164394255186,
"first_imu_ts_ns":   492164083352221,     ← 早 310,903,965 ns ≈ 311 ms
"forced": false,
"frames_dropped": 0, "imu_dropped": 0,
```

和 [2026-09-23 尺度验证](2026-09-23-scale-verification.md) 里写的「IMU 领先 311ms」
完全一致——**「相机帧硬按住 300ms」那套预热在真实数据里是 work 的**。

**4. 未验证**

- **文档里没有任何一条构建/运行步骤被执行过**（NDK、CMake、OpenCV Android SDK
  都还没装）。M0~M7 是待执行的计划，不是已验证的过程。
- 阶段六、七的文档是**设计说明**，明确标了未开工。

## 遗留

- **阶段四的「阶段四由 Kalibr 给出真实值」注释还留在 `phone.yaml` 里**
  （`:84` 和 `:103`），而 Kalibr 已废弃、外参平移仍是 0、噪声块下面已是实测值。
  这次没动 `phone.yaml`（它不是文档），但**这两处注释现在是错的**，该改。
- **`meta.json` 里硬编码的 `"calibrated": false` 和 `"note": "未标定…阶段四"` 已经不成立。**
  是 `SessionWriter.kt` 里的一行常量，改动要重新编译 App，这次没动。
- **外参平移、畸变系数两项仍然没做**（阶段四遗留，不堵阶段五/六）。
- 阶段六/七是设计稿；`STAGE6` 里 §6「快照必须深拷贝」等几条是**推断，未验证**。

## 回滚

`docs/` 是独立仓库（分支 master），`git diff` 可退。涉及：

- 改写：`tutorial/ANDROID_SLAM_TUTORIAL.md`、`tutorial/STAGE5_JNI_BRIDGE.md`、`README.md`
- 新增：`tutorial/PC_ORBSLAM3_WORKFLOW.md`、`tutorial/STAGE6_PHOTO_MEASUREMENT.md`、
  `tutorial/STAGE7_UI_STATE_MACHINE.md`（`git clean` 或手动删）

**`android/SlamCapture/` 和 `ORB_SLAM3/` 一个字都没动。**
