# 2026-10-08 删掉阶段五留下的两处日志脚手架

一句话：**`nativeHello` 和 `nativeLogSelfTest` 删了。** 顺带查清了一件事 ——
**这个 App 根本没有「谁在刷屏」**，日志量比想象的小得多。

## 动机

用户问了一句：**「对于一些日志是不是可以删掉，不让它输出了」**。

上下文是上一轮在讨论测距读数跳变的诊断，所以先得判断「吵」的到底是谁。
把全部 80 多处输出（Kotlin `Log.*` + C++ `printf`/`cout`）列了一遍之后，
**结论和直觉相反**，见「决策」第一节。

用户看完清单说：**「可以，只把建议删的删了吧」** —— 即只删表里标了「删」的两处。

## 改动

- **`cpp/jni_bridge.cpp`** ——
  - 删 `nativeHello`（M1 探针）和 `nativeLogSelfTest`（M2 自检）两个符号。
  - 删 `<cstdio>` / `<iostream>` 两个 include —— 它们只服务于 M2 那段自检。
- **`java/.../SlamBridge.kt`** ——
  - 删两个 `external fun` 声明。
  - **但把 M2 那条知识搬进了类注释**：
    「C++ 的 `cout`/`cerr`/`printf` 去处是 **logcat（tag `slam`）**，
    不是 Android Studio 的 Run 窗口」。删代码不等于删知识。
- **`java/.../MainActivity.kt`** ——
  - 删掉 `Log.i(TAG, "native 探针：" + SlamBridge.nativeHello())`。
  - ⚠️ 这一句还兼着一个**隐性作用**，见「决策」第三节 —— 所以把意图写成注释留给了
    紧随其后的 `nativeState()`。

## 决策

### 一、⭐ 调研结论：没有谁在刷屏（这条是本次记录的主要价值）

用户以为有日志洪水。查下来是**三方都安静**：

**1. ORB-SLAM3 的详细日志早就被关掉了。**

```cpp
// System.cc:239 —— 构造函数最后一行（构造函数 40~241 行）
Verbose::SetTh(Verbose::VERBOSITY_QUIET);
```

`PrintMess(str, lev)` 的实现就是 `if (lev <= th) cout << str << endl;`，
所以那 **90 处 `PrintMess`**（`Tracking.cc` 47 + `LoopClosing.cc` 20 +
`LocalMapping.cc` 3 + `System.cc` 1）**全部静音**。

**2. 剩下的裸 `cout` 在热路径上只在异常事件打印。**

把 `Track()` / `TrackLocalMap()` / `TrackWithMotionModel()` / `TrackReferenceKeyFrame()`
四个函数**整个区间**扫了一遍，只有这些：

```
"Fail to track local map!"                       ← 跟丢
"RESETING FRAME!!!" / "Timestamp jump detected"  ← 重置、时间戳跳变
"TRACK_REF_KF: Less than 15 matches!!"           ← 匹配不足
```

**一条逐帧的都没有。** 唯一像逐帧的 `ORBextractor.cc:1089/1166`（提取了多少特征点）
**是注释掉的**。

> ⚠️ **顺带纠正一个容易想当然的点**：`Verbose::th` **只挡 `PrintMess`**，
> 那 211 处裸 `cout <<` 它一处都拦不住。所以「把 verbosity 调低就安静了」是错的 ——
> 这次之所以安静，是因为裸 `cout` 本来就都挂在异常分支上。

**3. 我们自己的日志也全是「每事件一行」，没有逐帧的。**

### 二、所以真正能删的只有两处，而且删它们安全

`nativeHello` / `nativeLogSelfTest` 是**阶段五的脚手架**：

| | 当年验什么 | 验过了吗 |
|---|---|---|
| M1 `nativeHello` | 桥通不通（符号名、`loadLibrary`、JNI 签名） | ✅ `STAGE5_JNI_BRIDGE.md` 记着 |
| M2 `nativeLogSelfTest` | **静态初始化期的输出接不接得到** + logcat 4 KB 分块 | ✅ 同上 |

**一次性验证完成之后，验证工具就该撤。** 留着它们有实际代价：`SlamBridge` 的
导出符号表里挂着两个没人调的函数，下一个人读到这里要花时间判断「这还有用吗」。

### 三、⚠️ 删 `MainActivity` 那句探针，有个隐性作用要先接住

那句的原注释写着：

> 摸一下 SlamBridge，让 .so 在启动时就加载（JNI_OnLoad 里装着日志重定向）。

**这是真的** —— `SlamBridge` 是个 `object`，`.so` 在**第一次被碰**时才加载，
而日志重定向装在 `JNI_OnLoad` 里。删掉探针后，接力的是**紧随其后的 `nativeState()`**，
效果完全一样。

**但这变成了一处隐式依赖** —— 将来有人把 `nativeState()` 挪走或删掉，
重定向就会推迟到更晚，而症状是「构造 `System` 之前的输出没了」，很难往这儿想。
所以没让它变成隐式的：**在 `nativeState()` 上面写了一句注释说明它是那个「第一次碰」。**

### 四、没做的两件事（都提过，用户没要）

- **`cam.logCapabilities()` 没删** —— 它是日常最吵的一处（每次「启动 SLAM」打 8 行
  相机尺寸 / FPS / 焦距 / FOV），但**我当时的建议是「改成可关」而不是删**，
  所以严格按「只删建议删的」执行时它不在范围内。**仍待决定。**
- **日志没分成两个 tag** —— 根因是**我们的 `printf` 和 ORB-SLAM3 的 `cout`
  共用同一个 tag `slam`**（`log_redirect.cpp` 里 `kTag = "slam"`，而我们的输出也走 fd 1）。
  加一个 `slamapp` tag 会让 `[M6]` 那几行立刻好读得多，**对诊断读数的跳变更直接**。
  但用户这次只要「删」，所以只删。

## 验证

**1. 没有悬空引用**

```
$ grep -rn "nativeHello\|nativeLogSelfTest" app/src/
✓ 两个符号已经一处不剩
```

**2. 编译通过**

```
$ ./gradlew assembleDebug
BUILD SUCCESSFUL in 18s
```

**3. 导出符号从 17 个降到 15 个，旧符号没有残留**

```
$ llvm-nm -D --defined-only .../libslam_jni.so | grep -c Java_com_example_slamcapture_SlamBridge
15
$ llvm-nm -D --defined-only .../libslam_jni.so | grep -E "nativeHello|nativeLogSelfTest"
✓ 无
```

（上一版是 17 个 —— 见 [2026-10-07](2026-10-07-realtime-distance-measurement.md) 的验证第 2 条。
顺手也再次证明**符号名和 Kotlin 声明是成对改的**，没有单边。）

**4. 未验证**

- **真机没跑过。** 这次只删输出、不碰逻辑，风险极低，但按规矩照写：
  **没跑就是没跑。** 装一次看一眼 `logcat -s slam` 里确实没有 `[M1]`/`[M2]` 即可。

## 遗留

- **`cam.logCapabilities()` 还在每次「启动 SLAM」打 8 行**（上面「决策」四）。
- **两个 tag 没分开**（同上）。
- **`[M5] 收到了 loadFromFile 但还不支持载入旧图`** 留着没删 —— 它是个诚实的警告，
  而且现在没人传非 null，不会打。
- **`STAGE5_JNI_BRIDGE.md` 里 M1/M2 的描述没改** —— 那两节讲的是**当年怎么把桥搭起来**，
  是教程不是现状清单，删掉的脚手架不影响它们成立。

## 回滚

**一个提交的量级，`git revert` 就够。** 手动的话：
`jni_bridge.cpp` 恢复那两个函数体 + `<cstdio>` / `<iostream>`；
`SlamBridge.kt` 恢复两个 `external fun`；`MainActivity.kt` 恢复那行 `Log.i` 探针。
**没有配置、没有资源、没有 `ORB_SLAM3/` 的改动。**
