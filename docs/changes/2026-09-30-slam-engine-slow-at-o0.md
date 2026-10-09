# ⭐ 根因：安卓侧 ORB-SLAM3 是 `-O0` 编的 —— 单帧要 0.6 秒

> **这是「SLAM 反复重置、初始化起不来」的根因。** 从 M5 到现在的所有排查，
> 这一条收口。埋点上线后用户走了一趟，数字直接指出来了。

## 动机

上一份记录（[2026-09-30-slam-page-preroll-gate](2026-09-30-slam-page-preroll-gate.md)）
把问题劈成了两半：**PC 同一份数据跑通（999 位姿、VIBA2 到了），手机跑不通**。
结论是「手机运行时有问题」，但**不知道是 SLAM 慢还是相机没喂够**。

埋点（`[M5q]` 那行）就是为了分这个。用户走了一趟，一次就分出来了。

## 改动

| 文件 | 改什么 |
|---|---|
| `android/.../cpp/CMakeLists.txt` | 加 **`add_compile_options(-O3)`**，位置在 `project()` 之后、**任何 `add_subdirectory` 之前** |
| `CLAUDE.md` | 加一条协作规则：**源码注释要短** |

## 决策

### 一、⭐ 数字指认的链条

一个完整的失败周期（实测原始行）：

```
[M5q] 帧  2  入  22 丢  16 (72.7%)  队列 1  本帧  603 ms  间隔 0.033 s  状态 1 KF 0
[M5q] 帧  5  入  82 丢  73 (89.0%)  队列 3  本帧 1507 ms  间隔 0.133 s  状态 2 KF 2
[M5q] 帧 13  入 200 丢 183 (91.5%)  队列 3  本帧 2021 ms  间隔 0.133 s  状态 2 KF 2
[M5q] 帧 14  入 201 丢 183 (91.0%)  队列 3  本帧   28 ms  间隔 2.030 s   ← 超 1.0 s
[M5q] 帧 20  入 282 丢 258 (91.5%)  队列 3  本帧   30 ms  间隔 1.134 s   ← 超 1.0 s
```

**71 行埋点的统计：**

| | min | 中位 | max |
|---|---|---|---|
| **本帧耗时** | 28 ms | **603 ms** | **2558 ms** |
| **帧间隔** | 0.033 s | 0.265 s | **2.567 s** |
| **丢帧率** | 72.7% | **93.7%** | 94.3% |

- **间隔 > 1.0 s 的行：6 / 71**
- **本帧 > 1000 ms 的行：18 / 71**（四分之一以上）
- 跟踪状态分布：`1`（NOT_INITIALIZED）48 次、`2`（OK）23 次

**这次 logcat 里 `Timestamp jump` 出现 18 次** —— 和 18 次 `mnFirstFrameId` 对得上。

**链条：**

```
单帧要 603 ms（中位）          ← 帧周期只有 33 ms，慢 18 倍
     ↓
队列永远是满的（3/4），中位丢 93.7%
     ↓
「处理过的帧」之间的间隔 = 处理耗时 = 0.27 ~ 2.57 s
     ↓
Tracking.cc:1828  if(mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0)
     ↓
"Timestamp jump detected" → ResetActiveMap()       ← 18 次
     ↓
IMU 初始化要 KeyFramesInMap()>2 且累计运动 10~15 s，地图每 1~2 秒被清一次
     ↓
**永远起不来**
```

> **⚠️ 顺带：我 M5 时的假设是对的。** 当时我猜的就是
> 「丢帧 → 帧间隔 >1s → 判时间戳跳变 → 重置」，09-29 那份记录里我把它推翻了
> （因为那次的 log 里 `Timestamp jump` 是 0 次）。**现在有了埋点，它出现了 18 次。**
> 09-29 那次没出现，是因为那次根本没跑到丢帧严重的阶段。

### 二、为什么是 `-O0`

**两条编译命令的对照：**

| | 参数 |
|---|---|
| **PC**（`ORB_SLAM3/CMakeLists.txt:10-11`） | **`-O3`** … `-march=native` |
| **安卓**（`compile_commands.json` 里 `Tracking.cc` 那条） | `-g -DANDROID -fdata-sections -ffunction-sections -funwind-tables -fstack-protector-strong -Wformat …` —— **一个 `-O` 都没有** |

**AGP 的 Debug 构建不传任何 `-O`，clang 默认就是 `-O0`。** 30 个文件（含 `Tracking.cc`、
`System.cc`）全在无优化下编译。

**而 `-O0` 对 Eigen 是灾难** —— Eigen 靠内联和表达式模板吃饭，不优化时每个小矩阵运算
都退化成函数调用 + 临时对象。`-O0` 对 `-O3 -march=native` 差 **50~100 倍**，正好对上。

### 三、为什么放在 `add_compile_options`

```cmake
add_compile_options(-O3)     # 在 project() 之后、任何 add_subdirectory 之前
```

- 它写进目录的 `COMPILE_OPTIONS` 属性，**在 flags 变量之后生效** → 一定盖得住
- 放在 `add_subdirectory(DBoW2)` / `(g2o)` 之前，**子目录一起受益**
- 备选是 `set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3")`，
  但那个依赖「我们的 `set` 跑在 AGP 的 `-D` 之后」，不如 `add_compile_options` 稳

### 四、为什么之前一直没发现

- **PC 上一直是 `-O3`**（上游 `CMakeLists.txt` 写死的），所以 PC 从来没问题
- **M5 只验了「能不能跑起来」，没量过速度** —— 验收判据里没有「帧率」这一项
- 而症状（反复重置）看起来像算法/数据问题，把我引到 IMU 初始化那条路上查了很久

**教训：跨平台移植时，「能不能跑」和「跑得动吗」是两个判据，缺一个就会漏掉这类问题。**

## 验证

### 1. 埋点实测 ✅（上面的表）

logcat 抓了 40 分钟窗口，71 行 `[M5q]`。

### 2. 编译命令 ✅

```bash
$ python3 -c "…读 compile_commands.json，打印 Tracking.cc 的完整命令…"
   # 逐 token 输出，确认没有 -O
```

对照 PC：`grep -nE '\-O[0-9s]' ORB_SLAM3/CMakeLists.txt` → 第 10/11 行是 `-O3`。

### 3. ✅ **`-O3` 上设备验过了 —— 判断成立**

| | `-O0` | **`-O3`** |
|---|---|---|
| 本帧耗时（中位） | 603 ms | **~26 ms**（↓23 倍） |
| 丢帧率 | 93.7% | **~6%** |
| `Timestamp jump` | 18 次 | **0 次** |
| 跟踪状态 | 大多 `NOT_INITIALIZED` | **一直 `OK`** |
| 关键帧 | 0~2 | **54** |
| `start VIBA` | 0 | **1**（IMU 初始化成功） |
| 队列 | 永远是满的（3/4） | 大多是 **0** |

**初始化正常了。**

### 4. CLAUDE.md 的规则改动

新增「源码注释要短」。**用户当天纠正的第三次**，所以这次进了 CLAUDE.md，不只在记忆里。

## 补充（同日）：⭐ 同一个根因的第二个面 —— 缺 `NDEBUG`

**`-O3` 之后初始化正常了，但走了约 28 秒闪退。** 崩溃信息：

```
Optimizer.cc:2837: static void ORB_SLAM3::Optimizer::LocalInertialBA(...):
    assertion "mit->second>=3" failed
Fatal signal 6 (SIGABRT)

#02  ORB_SLAM3::Optimizer::LocalInertialBA(...)
#03  ORB_SLAM3::LocalMapping::Run()
```

**是 `assert()` 失败，不是内存问题。**

```cpp
// Optimizer.cc:2835
for(map<int,int>::iterator mit=mVisEdges.begin(); mit!=mend; mit++)
{
    assert(mit->second>=3);       // 纯 debug 检查 —— 循环体什么都没干
}
```

**PC 是 `CMAKE_BUILD_TYPE=Release`（`ORB_SLAM3/CMakeLists.txt:4-6`），CMake 的 Release
默认带 `-DNDEBUG` → 这个 assert 被编掉了。** 所以 PC 从来没撞上。

**证据（`strings` 查最终二进制）：**

| `.so` | 字符串 `mit->second>=3` |
|---|---|
| PC `libORB_SLAM3.so` | **0** ← 被编掉了 |
| 安卓 `liborb_slam3.so` | **1** ← 还在，会 abort |

（对照：`TRACK: Reset map` 在 PC 那个 `.so` 里能读到（1 处），证明 `strings` 命令有效 ——
所以上面那个 0 是真的没有，不是读不到。）

### 决策：把两个面一起解决，改构建参数而不是改算法

```cmake
add_compile_options(-O3 -DNDEBUG)
```

> **`-O0` 和「assert 还开着」是同一个根因的两个面：Android 是 Debug 构建，PC 是 Release。**

**为什么不直接去掉/改掉 ORB-SLAM3 那个 assert**：

- 它是**上游的 debug 检查**，不是功能代码
- **PC 用「关闭 assert」的配置跑出了好轨迹（999 位姿）** —— 说明那个条件不满足
  并不破坏算法，只是上游自己没考虑到
- **跟着已经验证过的配置走，不自己动算法** —— 动算法是另一个量级的风险

### 遗留的一个认知

**`-O0` / `NDEBUG` 这两条都是「Android Debug vs PC Release」的差别。**
移植时**构建类型必须对齐**，否则会以最隐蔽的方式偏掉（性能差 23 倍、assert 打崩）。
**这类差异不会再报错、也不会在 PC 上复现。**

## 遗留

- **⏳ 待验证：`-DNDEBUG`**（`-O3` 已验）。重编 + 走一趟，预期不再 SIGABRT
- **⚠️ `[M5q]` 埋点长期留着会刷 logcat**（现在每秒一条）。等这轮验完，
  要么调稀（每 100 帧 / 只在异常时打），要么撤掉。**它的诊断使命已经完成了**
- **9-30 那趟数据还在**（`dataset/selfrun/m5run/20260930-133625/`）——
  它是**在 `-O0` 下录的**，但**数据本身没问题**（PC 用它跑出 999 位姿）。
  可以做对照：**同一份数据，看手机端前后的 `[M5q]` 差多少**
- **PC 那个段错误**（999 位姿之后，退出码 139）**没查** —— 不影响本结论，但别忘了
- **IMU 配对偏差**（平均 2.4 ms，`gap_over_5ms` 1164）—— 优先级更低
- **CLAUDE.md 里那条注释规则**是我写的，措辞如果不合意直接改

## 回滚

删掉 `cpp/CMakeLists.txt` 里那一行 `add_compile_options(-O3)` 即可 ——
但**那会退回单帧 600 ms**，不要退。

CLAUDE.md 那条注释规则要撤就删掉那三行。
