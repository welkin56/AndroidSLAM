# 2026-10-07 阶段六第一版：画面中心实时测距（路径 1 + 路径 2）

一句话：**十字固定在画面中心，每帧顺手算一次「那儿有多远」，读数画在十字旁边。**
测量代码住在链 OpenCV 的那一侧，`slam_jni` 依旧一个 `cv::` 都不碰。

## 动机

阶段五刚收尾 —— 手机能实时跑出**米制**位姿，初始化问题解决，尺度验过（3 对点均值 −1.74%）。
但它**只输出位姿**。用户的原话是：

> 「我的期望是解决初始化问题，手机跑出来和电脑效果差不多，**我指出一段 1.6 米的长度，
> 你用手机 SLAM 计算的结果大致相同**。」

这一步把「能跑」变成「有用」。

用户还追加了两个要求：

1. **「没有其他路径吗，其他应用是怎么做的比如 github 上，我希望覆盖尽可能广，精度尽可能高，
   没有其他方案就路径 1+2 吧」** —— 于是先做了一轮调研（结论见「决策」第一节）。
2. **「测距是一个功能开关和录像一样，此外，这些开关要有记忆性」** —— 于是它进了抽屉，
   而且和「录制落盘」一起记进 `SharedPreferences`。

## 改动

### C++ —— 测量本体

- **`cpp/measure_center.cpp`（新增）** —— 两条路径：
  - **路径 1**：离画面中心最近的、还活着的 MapPoint。距离取 `‖Tcw · p_world‖`（斜距，
    和激光测距仪报的是同一种量）。
  - **路径 2**（路径 1 落空时才跑）：邻域地图点拟合局部平面，再取十字处沿光轴的深度。
    用**相对残差剔除 + 重新拟合**代替 RANSAC —— 确定性的，而且比三点 RANSAC 稳。
- **`cpp/measure_center.h`（新增）** —— 接口里**只有 float 和 `System*`**，没有 `cv::`、没有 Eigen。
  这不是洁癖，是下面那条边界能成立的前提。
- **`cpp/CMakeLists.txt`** —— `measure_center.cpp` 加进 **`orb_slam3`** target（不是 `slam_jni`）。
  理由和 `orb_track.cpp` 一模一样：它碰 `cv::KeyPoint` 和 `MapPoint`。
- **`cpp/orb_track.{h,cpp}`** —— `TrackResult` 里**不再放测量结果**，改成带出
  **`float t_cw16[16]`**（列主序 `Tcw`）。`System` 上有 `GetTrackedKeyPointsUn()` /
  `GetTrackedMapPoints()`，却**拿不到当前帧的位姿** —— 那份 `Tcw` 只能从这儿出去。
- **`cpp/slam_engine.{h,cpp}`** ——
  - `Engine` 加 `measuring`（原子开关）、`measure`（发布结果）、`measureRing`（中值窗口）。
  - `publishMeasure()`：**中值平滑** + **按变化打日志**。测不到时**立刻**变成不可测，
    不留过期数字。
  - `getMeasure()`：主线程读，`out[0]=距离` `[1..3]=世界坐标` `[4]/[5]=图像像素`。
  - `worker()` 里测距**跟着开关走** —— 关掉时连 `centerMeasure` 都不调。
  - `start()` 清掉上一趟的读数。
- **`cpp/jni_bridge.cpp`** —— `nativeSetMeasuring` + `nativeGetMeasure`，都包 `try/catch`，
  后者照 `nativeGetPose` 的写法**查数组长度**。

### Kotlin

- **`SlamBridge.kt`** —— 两个新 `external fun`。
- **`PreviewView.kt`** ——
  - 加 `MeasureState` **嵌套枚举**（`NO_TRACK` / `NONE` / `FAIR` / `GOOD`）和 `Overlay` 数据类。
  - `onDraw` 末尾画：**十字**、**吸附小圈**、**读数文字**（先黑描边再白填充）。
- **`MainActivity.kt`** ——
  - 抽屉里加 **`swMeasure`「实时测距」** 开关，**两个开关都记进 `SharedPreferences`**。
  - ticker 里多取一次（**复用已有的 250 ms tick**），把「跟踪未就绪」那一档叠上去。
  - 状态栏加一行距离。
  - `stopHardware()` 里清掉叠加层。
- **`res/layout/activity_main.xml`** —— 抽屉里加 `swMeasure`。

### 文档

`STAGE6_MEASUREMENT.md`、`APP_DESIGN.md`、本记录 + 索引。

## 决策

### 一、先做了一轮调研：为什么是路径 1+2

用户要求「覆盖尽可能广、精度尽可能高」，并问别的应用怎么做。查下来：

| 那类应用 | 靠什么 |
|---|---|
| Google Measure / AR Ruler（Android） | **ARCore** 平面 hit-test + Depth API —— 项目硬约束排除 Google |
| iPhone 上的测距 App | **LiDAR** / ARKit —— 我们没有深度传感器 |
| **ImageMeter** / **Camera_Measure**（无 ARCore、无深度硬件） | **参照物**（A4 纸、硬币）—— 单张照片在数学上就恢复不出绝对尺度 |

**第三行是关键**：市面上不用 Google、不用深度传感器的测距 app，做的**不是三维重建**。
而我们这个方案真的有一张**米制三维地图**，缺的只是**稠密性**。

稠密化的候选，以及否决理由：

| 路线 | 覆盖 | 精度 @1.6 m | 手机实时 | 结论 |
|---|---|---|---|---|
| **路径 1+2** | ~65% | cm | ✅ | **做** |
| 单目米制深度网络（Metric3D v2 / Depth Anything V2 Metric） | 100% | ❌ MAE ≈ 2.6 m 量级，**近处最差** | ❌ **515 ms ~ 2.4 s/帧** | **否决**：精度差两个数量级、速度差一到两个数量级。它训在 KITTI/驾驶场景上，**室内 0.5–3 m 是分布外** |
| 稀疏深度补全（LingBot-Depth-*DC* / IU-DC / OMNI-DC） | ~100% | 理论最优 | ❌ ViT-Large ~300M，**无端侧先例** | **否决（暂时）**：方向对，但全是研究代码。**记进文档作为第二批候选** |
| 立体 / MVS | 高 | 好 | ❌ | 单目 + 手部微动，基线太小 |
| 参照物换算 | 100% | **最好** | 交互麻烦 | **不实现，记成「精度兜底」** —— SLAM 尺度不可信时它是唯一的独立校验 |
| 路径 3（描述子三角化） | +? | 差（基线短） | ✅ | **等本轮实测命中率再定** —— STAGE6 那个「40%」是估值，一次都没测过 |

> ⭐ **深度补全那条路的输入正是「RGB + 稀疏米制深度图」**，而那个稀疏深度图
> = 我们每帧的 `mvpMapPoints` 投影回图像。**本轮做出来的东西本来就是它的前置。**

来源：[Metric3D v2 LiteRT](https://github.com/google-ai-edge/litert-samples/pull/167)、
[Depth Anything V2 Metric](https://deepwiki.com/kaylorchen/Depth-Anything-V2/5-metric-depth-estimation)、
[ZipDepth](https://github.com/fabiotosi92/ZipDepth)、
[LingBot-Depth](https://github.com/robbyant/lingbot-depth)、
[IU-DC](https://github.com/YangDL-BEIHANG/Dense-mapping-from-sparse-visual-odometry)、
[ImageMeter 的测量原理](https://www.imagemeter.com/manual/measuring/basics/)。

### 二、⭐ 十字固定在中心 ⟹ **不需要任何 UI 输入**

这一条决定了整个实现形状。

十字恒对应图像 `(W/2, H/2)`，所以「测哪儿」**不是运行时参数**。推论：

> **测量可以在 SLAM 线程上每帧算一次，把结果发布成一个标量，UI 只读。**
> **零额外 JNI 调用、零锁竞争、零往返。**

平滑也顺理成章 —— 环形缓冲只活在 SLAM 线程上。这正是 09-26 选「十字固定」而不是
「点哪测哪」时埋下的收益。

### 三、路径 2 **不需要内参**（这一条改了原计划）

原计划写的是「中心射线与拟合平面求交」，那要 `K`。查过：**`Tracking::mK` 是
`protected`（`Tracking.h:290`），`System` 拿不到**，要走就得改上游。

**换成不需要内参的等价形式**。相机坐标系里的一个平面：

```
n·X = d,   X = Z·(x, y, 1),   x = (u-cx)/fx,  y = (v-cy)/fy
⟹ 1/Z = a·u + b·v + c        ← 在**像素坐标**上是仿射的
```

内参和平面参数**全被 a/b/c 吸收了**。所以对 `1/Z` 直接最小二乘，再在十字处取值就行。
**上游一个字都不用改。**

代价：路径 2 给不出**世界坐标**（沿光轴只知道深度，要变成点得知道归一化方向）。
`CenterMeasure::wx/wy/wz` 在 `method == 2` 时无效，已在头文件里标明。
验收用的那排砖走的是路径 1，不受影响；将来做 (b) 两点距离时要它，那时再加 `System::GetK()`。

### 四、否决过的方案

- **否决：** 把测量写在 `orb_track.cpp` 里（它手边就有 `Tcw`，最省事）。
  理由：**「开不开测距」是策略，属于 `Engine`**；`orb_track` 只该管「喂帧」。
  而且 `slam_jni` 本来就**不能**调 `orb_track` 里碰 `cv::` 的东西 —— 反过来才成立。
  所以改成 `orb_track` 带出 `Tcw`，由 `worker()` 决定要不要调 `centerMeasure`。
  改完的额外好处：**关掉测距时那两次 vector 拷贝也省了**。
- **否决：** 「跟踪未就绪」也由 C++ 报。
  理由：那是 **SLAM 的状态**，不是测量的状态。混在一起的话，跟踪没好时界面会显示
  「对准有纹理的地方」—— 用户会去擦镜头。
- **否决：** 状态栏显世界坐标。
  理由：路径 2 给不出，显成 `0.00, 0.00, 0.00` 是**误导**。要那三个数看 logcat。
- **否决：** 枚举常量放进 `PreviewView` 的 companion。
  理由：**编译不过** —— 那个 companion 是 `private` 的（`W`/`H`/`MIN_INTERVAL_NS` 有意私有）。
  改成嵌套枚举 `MeasureState`。顺带把 `state >= M_FAIR` 那种**依赖声明顺序**的写法
  换成 `state.hasReading`。
- **⚠️ 用户纠正：** 我先把测距**默认开**，用户要求**默认关**。
  理由（他给的）：它和录制一样是**用户拨开才生效**的功能，不替用户决定。
- **⚠️ 用户纠正：** 我用了 `sleep` 干等后台编译。
  理由（他给的）：「万一它快一点，我还要等你 sleep？」—— 后台任务完成会主动通知。

### 五、性能：`isBad()` 的锁要排在像素筛选**之后**

`MapPoint::isBad()` **要同时拿两把锁**（`mMutexFeatures` + `mMutexPos`），而
`LocalMapping` / `LoopClosing` 也在抢。第一版写的是「先判 `isBad()` 再算距离」——
那就是 **每帧 500 次双锁**。平均开销不大，但**尾延迟**难看。

改成**先按像素距离筛，只对落到半径内那两三个点查 `isBad()`**，锁的次数从 500 降到个位数，
语义完全一样。

用户问过「这个计算开销大吗」，当时的估算是 **15~25 µs/帧**，
对比单帧跟踪的 **26 ms** ⟹ **0.1% 量级**（`GetTrackedKeyPointsUn` + `GetTrackedMapPoints`
两次堆分配 + ~18 KB memcpy ≈ 2–5 µs；路径 1 扫 500 个点 ≈ 5 µs）。所以它**不跟着帧率走**。

## 验证

（下面都是实际跑过的。）

**1. 编译通过**

```
$ cd android/SlamCapture && ./gradlew assembleDebug
BUILD SUCCESSFUL in 7s
39 actionable tasks: 7 executed, 32 up-to-date
```

第一次是 **FAILED**，8 处同一个 Kotlin 错误（见「决策」第四节）：
```
e: MainActivity.kt:535:60 Cannot access 'Companion': it is private in 'PreviewView'
```
改成嵌套枚举后过了。

**2. 两个新 JNI 符号真的进了 `.so`**（符号名对不上是这套桥的头号失败模式）

```
$ llvm-nm -D --defined-only app/build/intermediates/cxx/Debug/70425w2u/obj/arm64-v8a/libslam_jni.so \
    | grep Java_com_example_slamcapture_SlamBridge
000000000006f888 T Java_com_example_slamcapture_SlamBridge_nativeGetMeasure
000000000006f85c T Java_com_example_slamcapture_SlamBridge_nativeSetMeasuring
（连同原有的 15 个，共 17 个）
```

**3. OpenCV 边界完好** ⭐ —— 这是新文件最可能破坏的东西

```
$ llvm-nm -D libslam_jni.so | grep centerMeasure
                 U _ZN4slam13centerMeasureEPN9ORB_SLAM36SystemEiiPKf      ← 未定义引用 ✓
$ llvm-nm -D --defined-only libslam_jni.so | grep -c '_ZN2cv'    → 0
$ llvm-nm -D libslam_jni.so | grep '_ZN2cv' | grep -c ' U '      → 0
✅ 干净：slam_jni 里一个 cv:: 都没有
```

`slam::centerMeasure(ORB_SLAM3::System*, int, int, float const*)` 定义在
`liborb_slam3.so` 里（`T`），`libslam_jni.so` 只持有**未定义引用** —— 正是设计意图。

**4. 未验证**

- ⚠️ **真机上一行都没跑过。** 没装、没开、没出过读数。四档验收（有读数 / 用那排 1.6 m 的砖
  验精度 / 对白墙必须给提示语 / 路径 2 兜底生效）**全部未做**。
- 实测命中率、路径 2 的实际覆盖率、读数抖动 —— **一个数都没有**。
- 所以「路径 1 还是不是主力」这个判断**目前仍然只有 STAGE6 那个估值（40%）**。

## 遗留

- ⚠️ **畸变耦合**（STAGE6 §3 点名要留对的）。`mvKeysUn` 是**去畸变后**的坐标，预览显示的是
  **带畸变**的原图。现在 `k1/k2/p1/p2` 全是 0 ⟹ 恒等，两者一致。
  **一旦标定出非零畸变，中心就不再是 `(W/2, H/2)`。**
  本轮没解决（拿不到 K 和畸变系数），但代码里收口在 `centerPx()` 一个函数里 ——
  **只有那一处要改**。
- **路径 2 给不出世界坐标**（见「决策」三）。要做 (b) 两点距离就得加 `System::GetK()`。
- **融合与置信度**（STAGE6 §5）没做：多路径取中值、离散度 > 5 cm 降级。现在只有顺序兜底。
- **路径 3**（描述子三角化）没做。
- **`System` 没暴露 `fx`**，吸附半径用 `0.02·w` 近似 `0.02·fx`（本机 fx≈435、w=640，
  差 1.5 倍以内）。是个手感参数，但要精确就得加 `System::Fx()`。
- **抖动**：读数在阈值附近会在「数字 / 提示语」之间闪。本轮不做滞回。
- **`(b)` 两点距离**没做。
- **`swMeasure` 默认关** —— 用户定的。

## 回滚

- **C++**：删 `measure_center.{h,cpp}`；`CMakeLists.txt` 去掉那一行；
  `orb_track.{h,cpp}` 去掉 `t_cw16`（改回不带）；`slam_engine.{h,cpp}` 去掉
  `measuring` / `measure` / `measureRing` / `setMeasuring` / `publishMeasure` / `getMeasure`
  和 `worker()` 里那一句；`jni_bridge.cpp` 删两个符号。
- **Kotlin**：`SlamBridge.kt` 删两个 `external fun`；
  `MainActivity.kt` 删 `swMeasure` 相关（字段、监听、恢复、`pollMeasure`、ticker 那行、
  状态栏那行、`stopHardware` 里那两行）和 `pref`/`PREF_*`；
  `PreviewView.kt` 删 `MeasureState` / `Overlay` / `overlay` / 三个 Paint / `drawMeasure`
  和 `onDraw` 末尾那一句；`activity_main.xml` 删 `swMeasure` 那一段。
- **`ORB_SLAM3/` 一个字都没动** ✓（这是路径 2 换成免内参形式换来的）。
- **布局以外没有资源改动**；`SharedPreferences` 里会留下两个没人读的键（无害）。
