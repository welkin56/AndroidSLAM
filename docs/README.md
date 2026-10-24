# SLAMAPP 文档

本项目的全部文档。分三类：

| 目录 | 是什么 | 怎么找 | 会变吗 |
|---|---|---|---|
| [tutorial/](tutorial/) | 教程与设计说明 —— 「这个项目现在是怎么搭的」 | 按主题 | **会改写**。每个阶段推进时对应章节要更新 |
| [changes/](changes/) | 改动记录 —— 「某天动了什么、为什么」 | 按日期 | **只增不改** |
| [bugs/](bugs/) | 踩过的坑 —— 「这类问题长什么样、怎么查」 | **按名字** | 只增不改，可以补充 |

**`changes/` 和 `bugs/` 的区别**：前者是**事件**（这一天改了什么），后者是**知识**
（这个坑的通用形状、怎么认出来、什么情况下还会撞上）。改动记录是给「审查这次改动」
看的；`bugs/` 是给**「又撞上一个怪现象」**的时候翻的。

教程描述**当前状态**会被覆盖，另外两类是**历史流水**不动。混在一起会互相干扰，所以拆开。

## tutorial/

**分工原则：主教程只讲方向，细节在各阶段文档里。**

判断一条信息该写在哪：**它在阶段的实现换掉之后还成立吗？**
成立（约束、原理、验收判据）→ 主教程；不成立（命令、文件路径、代码片段）→ 阶段文档。

- [ANDROID_SLAM_TUTORIAL.md](tutorial/ANDROID_SLAM_TUTORIAL.md) —— **主教程**。
  现状与终态、关键认知、阶段一~七各自的目标与验收判据、排查手册、参考资料。
  **每节的末尾给详情链接，需要动手时去对应的阶段文档。**
- [PC_ORBSLAM3_WORKFLOW.md](tutorial/PC_ORBSLAM3_WORKFLOW.md) —— 阶段一 + 阶段三：
  在 PC 上跑 ORB-SLAM3 的工作流（数据集、自采数据、`timestamps.txt`、判据、evo）。
  两者是同一件事，只是数据来源不同，所以合在一份里。
- [STAGE2_ANDROID_CAPTURE.md](tutorial/STAGE2_ANDROID_CAPTURE.md) —— 阶段二：Android
  采集层的零基础实操（环境搭建、每个源文件的职责与踩坑、无线调试、验收指标）
- [ORB_SLAM3_TRACKING_FLOW.md](tutorial/ORB_SLAM3_TRACKING_FLOW.md) —— 阶段三的认知部分：
  ORB-SLAM3 Mono-Inertial 的线程模型、从 `main()` 起的调用链、**地图重置的四条路径**
- [STAGE4_CALIBRATION.md](tutorial/STAGE4_CALIBRATION.md) —— 阶段四：相机与 IMU 标定
  的完整流程（为什么不用 Kalibr、标定板、采集要求、怎么看结果）
- [INTRINSICS_VS_FOCUS.md](tutorial/INTRINSICS_VS_FOCUS.md) —— ⚠️ **未定论的假设**：
  内参会不会随对焦距离变？含两个竞争假设、文献调研（dynamic intrinsics）、
  判别实验方案，以及一条"对焦距离可指定"的可操作性发现。
  **判别实验还没做**
- [STAGE5_JNI_BRIDGE.md](tutorial/STAGE5_JNI_BRIDGE.md) —— 阶段五：JNI 桥接的
  **零基础知识点**（App/ART/JNI/NDK 地基）+ 依赖从哪来 + **四个地雷** +
  分步里程碑 **M0~M8**（含 `SlamEngine` 的实现）
- [STAGE6_MEASUREMENT.md](tutorial/STAGE6_MEASUREMENT.md) —— 阶段六：**实时**尺寸测量。
  **第一版已实现**（到目标的距离、路径 1+2），**真机还没跑过**；两点距离、路径 3、
  多路径融合未做。含**三条测距路径**、吸附、可测性、精度与时效，
  以及**稠密化的候选路线与否决理由**（深度网络 / 深度补全 / 参照物）
- [MEASUREMENT_TUNING.md](tutorial/MEASUREMENT_TUNING.md) —— **测距调参表**：
  每个阈值在哪、管什么、调大调小的后果，以及**调参时看不到什么**（已知的日志缺口）。
  调读数之前先看这份
- [BUILD_MEMORY.md](tutorial/BUILD_MEMORY.md) —— **编译内存**：`installDebug` 为什么占满内存
  （ninja 不带 `-j`，按核数开满），现在的 job pool / 堆配置在哪，怎么验证、怎么调
- [STAGE7_UI_STATE_MACHINE.md](tutorial/STAGE7_UI_STATE_MACHINE.md) —— 阶段七：两个状态机
  ——采集端（已实现）和 SLAM 侧的测距可用性（未开始）
- [APP_DESIGN.md](tutorial/APP_DESIGN.md) —— **已定的 App 形态**：一条流水线 + 两个视图、
  入口页、切图（方案 A）、路径点与手动重定位、三个正交开关、生命周期、保存策略。
  **不属于任何单一阶段**，所以单独立一份
- [MAP_FEATURE_IDEAS.md](tutorial/MAP_FEATURE_IDEAS.md) —— ⚠️ **不是阶段**，是
  **还没打算实现**的功能设想（迷宫路线记录 / 随身尺寸测量 / SLAM 可观测性），
  含精度预算与「人工指示 vs 语义分割」这类取舍分析。
  写下来是为了固化结论，不是为了排期

## changes/

改动记录的索引在 [changes/README.md](changes/README.md)，**最新在前**。
新记录的格式见 [changes/_TEMPLATE.md](changes/_TEMPLATE.md)。

## bugs/

索引在 [bugs/README.md](bugs/README.md)。**一份一条，标题就是那个坑。**

- [thread-stack-too-small.md](bugs/thread-stack-too-small.md) —— Android 的 pthread
  默认 **1 MB** 栈，而 PC 主线程是 8 MB。深度 C++ 调用链 + 栈上大对象在 PC 上没事、
  在安卓上 `SIGSEGV`。**「在 PC 上好好的」在栈这一维上不成立。**
- [duplicated-static-lib-symbols.md](bugs/duplicated-static-lib-symbols.md) —— 同一个
  `.a` 被链进两个 `.so`，进程里就有两份它的全局状态（这次是
  `cv::parallel_for_` 背后的 TBB 并行引擎）。**「两份逐字节相同的代码」不是冗余，是 bug。**

## 相关的文档

不在本目录，但同样是文档：

- [`../CLAUDE.md`](../CLAUDE.md) —— 协作约定（给我看的，不是教程）
- [`..`](../) —— **本仓库的根**：Android 采集端源码（`app/`）+ `tools/`

> ⚠️ **本文里的源码引用**（`Tracking.cc:2019` 这种）指的是**上游 ORB-SLAM3**，
> 它**不在这个仓库里**。本仓库里有一份**同源码的副本**：
> `app/src/main/cpp/thirdparty/orb_slam3/` —— 要对照代码去那儿。
> （那份按 [thirdparty/README.md](../app/src/main/cpp/thirdparty/README.md) 做过几处移植改动，
> **行号可能和上游略有偏移**。）
