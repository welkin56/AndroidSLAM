# 给 SLAM 页补上录制模式那道 300 ms IMU 预热闸

## 动机

**用户问出来的**：

> 原来录制模式不是保证了 imu 先写的吗，怎么这里又无法保证了

起因是新录的那段数据（2026-09-30，62 秒）被 PC 侧的守卫拦下了：

```
ERROR: IMU 与相机时间关系不满足 ORB-SLAM3 的前提（IMU 必须先于相机开始）
       first image t = 1077477431929144
       first IMU   t = 1077477479977117
       IMU 相对首帧 = +0.048 s（应当为负）
```

**IMU 比相机晚 48 ms。** 查下来**用户记得没错**：

| | 录制模式 `CaptureActivity` | SLAM 页 `MainActivity` |
|---|---|---|
| 帧出口的闸 | `tsNs >= framesFromTsNs` | **没有** |
| 值 | `PREROLL_NS = 300_000_000L` | — |
| 效果 | 相机帧被**硬按住 300 ms**，IMU 先写满 | 相机帧立刻喂进去 |

**M5 抄了录制模式的相机接线，但漏了这道闸。**

## 改动

**`MainActivity.kt`**，四处：

| 位置 | 改什么 |
|---|---|
| `companion object` | 加 `PREROLL_NS = 300_000_000L`，注释写明和录制模式同值 |
| 硬件段 | 加 `@Volatile var framesFromTsNs = 0L`（相机线程读、主线程写） |
| `openHardware()` | `c.start(...)` 之后设 `framesFromTsNs = SystemClock.elapsedRealtimeNanos() + PREROLL_NS` |
| `onCameraFrame()` | 加 `val warm = tsNs >= framesFromTsNs`，**①喂 SLAM 和 ②落盘各加一道 `warm`**；**③预览不加** |

**预览不受影响** —— 糊帧对预览是有用的信息，这跟录制模式的选择一致。

**①也加闸**（录制模式只闸了落盘那条）：SLAM 同样需要 IMU 先于相机，**这次守卫拦下的正是喂给 SLAM 的那份**。

## 决策

### 一、为什么 48 ms 不是根因，但还是要修

48 ms 只影响**头 2~3 帧**：`PreintegrateIMU()` 在第一帧没有 `mpPrevFrame` 直接返回，
第 2 帧起才需要 IMU 跨度。**解释不了手机上 18 次地图重置。**

**但它是真缺陷**，两个理由：
1. 录制模式有、SLAM 页没有 —— 是 M5 抄漏了，不是设计选择
2. **48 ms 不是常数** —— 它取决于 IMU 首样本什么时候到。IMU 从 `registerListener`
   到出数本机要 3 秒多，**下一次可能是更大的值**，而那时候就不只是头几帧的事了

### 二、⚠️ 同一天查出来的另一件事：**问题在手机运行时，不在数据**

把这段 62 秒的数据喂给 PC（`mono_inertial_euroc`，同一份 `phone.yaml`）：

| | PC（同一份数据） | 手机 |
|---|---|---|
| 轨迹点数 | **999** | **1** |
| 关键帧数 | **99** | ~0 |
| `start VIBA 1` | **2 次** | — |
| `start VIBA 2` | **1 次** | — |
| 地图重置 | 42 次，**但熬过去了** | 18 次，**没熬过** |

**PC 的报告是「熬过头 ~500 帧（≈17 秒）就起来了」**：

```
重置帧号: 4 36 78 93 127 133 ... 454 506 | 723 730 744 766 780 787 795 819 846
                     ↑ 前 34 次挤在头 506 帧      ↑ 之后稀疏了
VIBA1 在日志 523 行、VIBA2 在 670 行
```

**所以 09-29 那份记录的结论要修正**：那次 PC 也失败，我判定「是这段录得不好」；
**这次数据好了，PC 能跑通而手机不能 ⇒ 手机运行时有问题。**

> 09-29 那次 27 秒、逐帧位移 10.5 px；这次 62 秒、用户走了起来。**录制方式确实改善了数据**
> —— 但手机这边仍然爬不出来，说明**还有一个手机侧的变量**。

**首要嫌疑仍是 M5 引入的那个**：帧队列有界（4 帧、满丢最旧）。
如果手机 SLAM 的帧率远低于 30，实际喂进 ORB-SLAM3 的帧间隔会远大于 33 ms。

**但这还是推理，下一步要数字**（见「遗留」）。

## 验证

### 1. 静态检查 ✅

```bash
# 字符串模板 $函数名()、注释里一行两次 */、被调用但没定义的方法 —— 全为空
# 括号平衡 130/130、324/324
# PREROLL_NS 2 次 / framesFromTsNs 3 次 / SystemClock 2 次
```

### 2. ❌ 没有编译

按约定编译是用户跑的。**这次改的是事件路径（`onCameraFrame` 第一行）**，
逻辑上只多了一个比较，但**没过编译器**。

### 3. PC 对照的实验数据

见「决策二」的表。命令：

```bash
$ ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt \
    .../assets/phone.yaml <session> <session>/timestamps_trim.txt t930
```

> ⚠️ 用的是 `timestamps_trim.txt`（掐掉 IMU 之前的 2 帧）——
> **正是因为这道闸缺失，原始数据过不了我们自己的守卫**。这道闸补上之后，
> 新录的数据就不需要 trim 了。

**PC 最后段错误了（退出码 139）**，在产出 999 个位姿之后。没有查 —— 不影响上面的结论。

## 补充（同日，用户合并之后）：埋点

**`slam_engine.cpp` 三处：**

| 位置 | 加什么 |
|---|---|
| `Engine` | `framesIn` / `framesDropped` 两个原子计数 |
| `pushImage()` | 丢一帧就 `framesDropped++`；入队就 `framesIn++` |
| `worker()` | 给 `orbTrack` 掐表；**每 10 帧 / 距上次 >2 s / 本帧 >200 ms** 打一行 |

**输出：**

```
[M5q] 帧 30  入 180 丢 150 (83.3%)  队列 4  本帧 190 ms  间隔 0.198 s  状态 1  KF 0
```

**为什么这么打** —— **一行就够劈开两个原因**：

| 读出来 | 原因 | 修法 |
|---|---|---|
| **本帧耗时长**（190 ms） | SLAM 自己慢 | 减特征数 / 缩图 / 换配置 |
| **耗时短但间隔大** | 相机没喂够 / 队列丢太多 | 改队列策略 |

**这两个的修法完全不同**，所以在量到之前不动任何代码。
触发条件里那条 `>2 s` 是给低帧率留的 —— 真跑成 2 fps 时，纯按帧数打会一条都出不来。

### ⭐ 顺带：C++ 现在能自己过语法检查了

`app/.cxx/Debug/<哈希>/<abi>/compile_commands.json` 里有每个 TU 的**确切编译命令**，
拿它去掉 `-o` 换个 `-fsyntax-only` 就能**在交给编译器之前先查一遍**：

```bash
# 从 compile_commands.json 取 slam_engine.cpp 那条，去掉 -o *.o，加 -fsyntax-only
```

**这次结果：退出码 0**（只有 g2o / Eigen 原有的 deprecation warning）。

**这条值得单独记** —— 这一轮我被编译器抓了两次，都是**我看不出来的**：

| 报错 | 原因 |
|---|---|
| `Function invocation 'nowStamp()' expected` | 字符串模板写成 `"$nowStamp()"`（少了花括号） |
| `Expecting member declaration` | KDoc 里 `` `session-*/` `` 的 `*/` 把注释提前结束了 |

**两次都是 Kotlin —— 而 Kotlin 没有 compile_commands.json，所以那两处仍然只能靠编译器。**
C++ 侧从今天起可以先自检。

## 遗留

- **⏳ 埋点已加，等实测。** 重编装 + 走一趟 + 抓 logcat。
  logcat 里同时有 ORB-SLAM3 自己的 `mnFirstFrameId`（每次重置打一行），
  **帧号推进速度能独立反推帧率** —— 和 `[M5q]` 那行互相印证。
- **Kotlin 侧仍然没有自检手段**（没有 compile_commands.json 那种东西），
  所以 Kotlin 改动还是只能靠编译器
- **PC 那个段错误**没查（999 位姿之后）
- **IMU 配对偏差**：这次 `imu_mean_pair_gap_ns = 2.4 ms`（上次 1.0 ms），
  `imu_gap_over_5ms = 1164`。**但 IMU 样本本身的间隔是完美的**
  （24905 条，平均 2.48 ms，最大 2.5 ms，零缺口）—— 所以是**配对**的问题，不是丢样。
  `ImuCollector` 用「时间最近的加速度」配对，402 Hz 下半个周期是 1.25 ms，
  2.4 ms 的平均偏差偏大。**待查，优先级低于上面那条。**

## 回滚

删掉这四处即可：

1. `PREROLL_NS` 常量
2. `framesFromTsNs` 字段
3. `openHardware()` 里那一行赋值
4. `onCameraFrame()` 里的 `warm` 和两处 `&& warm`

**回滚之后会退回 M5 的行为** —— IMU 可能晚于相机开始，
PC 侧那道守卫会再次拒绝这类数据。
