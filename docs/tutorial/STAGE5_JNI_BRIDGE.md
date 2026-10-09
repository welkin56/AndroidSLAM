# 阶段五实战：JNI 桥接（零基础版）

> 目标读者：没做过 Android 底层、没写过 JNI 的人。
>
> 主教程 [§6](ANDROID_SLAM_TUTORIAL.md#6-阶段五jni-桥接) 讲**方向与判据**。
> 本文是**落地手册**：地基知识、依赖从哪来、九个里程碑（M0~M8）、
> 以及四个会当场卡住你的地雷。
>
> **剥 Pangolin 清单和 CMakeLists 全文都在 M3 里** —— 主教程已经不放这些细节了。
>
> 骨架以本文为准，不需要去别处补。

---

## 0. 这一阶段做什么

```
CameraCapture.onFrame(灰度, ts) ──┐
                                  ├─→ SlamBridge(JNI) → SlamEngine(C++ 队列+线程+图)
ImuCollector(陀螺/加速度, ts) ─────┘                              │
                              ORB-SLAM3 TrackMonocular ◄─────────┘
                                        │
                              最新位姿/跟踪状态 → 状态栏
```

**`SlamEngine` 是本阶段的核心产出** —— 它不只是"队列 + 线程"，还持有
**「旧图」（`referenceMap`）** 和**切图**的能力。详见 §5 和
[APP_DESIGN.md](APP_DESIGN.md) §2.6。

**不做**：不动采集端逻辑（它的 fx、噪声参数、时间戳口径已经全部锁定，本阶段只**加调用**）；
不做尺寸测量（阶段六，见 [STAGE6_MEASUREMENT.md](STAGE6_MEASUREMENT.md)）；
不做性能优化（先跑通 —— **但刻意留了路，见 §8**）。

**验收判据**

| 项 | 判据 |
|---|---|
| 实时性 | 相机不掉帧；SLAM 跟不上时丢帧不积压 |
| 正确性 | **同段数据手机实时跑 vs PC 跑 `mono_inertial_euroc`，轨迹量级一致** |
| 稳定性 | 切后台干净停、回来不崩；连跑 10 分钟不崩 |

第二条是唯一的硬判据。前一条只能说明"没卡死"，说明不了"算得对"。

---

## 1. 地基：App、ART、JNI、NDK

这一节解释**为什么这层特别容易出事**。没写过 JNI 的人踩的坑，十有八九都源于
「两个世界」这件事没想清楚。

### 1.1 一个 App 进程里住着两个世界

Android App 的代码不是只有 Kotlin。**同一个进程里可以同时装着**：

- **Kotlin 字节码**，由 ART 执行（带 GC、带异常、带栈回溯）
- **native 机器码**（`.so`），由 CPU 直接执行（无 GC、无托管栈）

两者通过 JNI 互相调用。ORB-SLAM3 全部活在第二个世界里。

### 1.2 JNI 是三层约定

| 层 | 内容 | 破了会怎样 |
|---|---|---|
| **命名** | Kotlin 的 `nativePushImage` ↔ C++ 的 `Java_com_example_slamcapture_SlamBridge_nativePushImage` | `UnsatisfiedLinkError` |
| **类型** | `jlong`/`jint`/`jobject` 与 C++ 类型的对应；`JNIEnv*` 是**线程私有**的函数表 | 崩溃或静默读错数据 |
| **生命周期** | `jobject` 是**句柄不是指针**；local ref 数量有上限；跨线程要 `AttachCurrentThread` | 句柄表溢出、用错 env |

命名规则里最容易漏的一条：**包名里的下划线要转义成 `_1`**。
本项目包名是 `com.example.slamcapture`（无下划线），暂时碰不到，但值得知道。

> **本书的包名是 `com.example.slamcapture`。** 旧版主教程里写的
> `Java_com_example_slam_...` 少了 `capture`，照着抄必然 `UnsatisfiedLinkError`——已修正。

### 1.3 NDK 是什么

NDK = 一套**交叉编译工具链**：在 x86 的电脑上产出 ARM 手机能执行的 `.so`。
它自带 clang、链接器、系统头文件，以及 `ndk-stack` 这类排错工具。

CMake 是构建脚本工具，Gradle 调它来编你的 C++。

### 1.4 为什么这一层特别容易出事

**两个世界的错误模型不一样。** 这是本阶段所有怪事的根源：

| | Kotlin 世界 | native 世界 |
|---|---|---|
| 出错 | 抛异常，有人接着，有栈 | 返回错误码、abort、或**直接 exit** |
| 崩溃 | `FATAL EXCEPTION` + Java 栈 | SIGSEGV，要符号化才有栈 |
| 日志 | `Log.i` → logcat | `std::cout` → **默认丢进 `/dev/null`** |

由此引出三条**必须提前知道**的结论：

1. **C++ 异常绝不能穿过 JNI 边界。** 穿过去 = 进程直接死，Java 侧连栈都没有。
   每个 `extern "C"` 函数体都要包 `try { ... } catch (...) { ... }`。
2. **ORB-SLAM3 的失败方式在 Android 上是"App 整个消失"。** 它用 `exit(-1)` 报错
   （见 §3.2），而 `exit()` 不走 Java 的异常路径——logcat 里**没有 FATAL EXCEPTION、
   没有 Java 栈、干干净净**。
3. **不把 stdout 接出来，ORB-SLAM3 就是个哑巴。** 它所有的进度和错误都用
   `cout`/`cerr` 输出，而 Android 上这些默认进 `/dev/null`。

**所以 M2（把 stdout 接进 logcat）必须排在做任何 ORB-SLAM3 相关工作之前。**
跳过它去调 SLAM，等于蒙着眼睛。

---

## 2. 依赖从哪来

**动手前先把这张表填满。** 这一节是本阶段最容易低估的部分——教程旧版说
「用官方 OpenCV Android SDK 的预编译库」，但本机并没有那个 SDK。

| 需要 | 本机现状 | 怎么办 |
|---|---|---|
| NDK | ✅ `26.1.10909125` | 已在 `build.gradle.kts` 里写死 `ndkVersion` |
| CMake | ✅ `3.22.1` | 已装 |
| **Boost.Serialization** | ⚠️ **PC 上有，Android 上要自己编** | **见 §3.4** |
| OpenCV（Android） | ❌ **只有源码 clone** | 见 2.1 |
| Eigen | ✅ `/usr/include/eigen3` | 纯头文件，直接指过去，不用交叉编译 |
| DBoW2 / g2o / Sophus | ✅ `ORB_SLAM3/Thirdparty/` | 用，但要先打补丁（§3.1） |
| `ORBvoc.txt` | ✅ 139 MB | 不进 APK，见 §3.3 |
| `phone.yaml` | ✅ | 见 2.2 |

### 2.1 OpenCV：本机没有 Android SDK

`SLAMAPP/opencv/` 是**OpenCV 源码 clone**，里面能找到的 `OpenCVConfig.cmake`
只有 PC 版（`opencv/build/`）。**没有 `sdk/native/jni/`，没有任何 `.so`。**

两条路，选一条：

**A（推荐）：下载官方 OpenCV Android SDK。** 解压后得到
`OpenCV-android-sdk/sdk/native/{jni,libs}/`。里面是**已经编好的**各 ABI 的 `.so`，
省掉几十分钟的编译。

> **把这个目录整个丢进 `app/src/main/cpp/thirdparty/`**（和 `orb_slam3/`、`DBoW2/` 并排）。
> `CMakeLists.txt` 默认就在那儿找，**不用配任何路径变量**。
> 它不进 git（872 MB），换机器时按同样的位置放回去就行。

**B：从源码 clone 自己编 Android SDK。** 源码树里有 `platforms/android/build_sdk.py`，
它需要 Android SDK/NDK，跑起来比较慢。好处是版本和 PC 那份完全一致。

拿到之后，CMake 里接进去：

```cmake
set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/thirdparty/opencv/sdk/native/jni)
find_package(OpenCV REQUIRED)
```

> **先 `ls` 确认那个目录里真有 `OpenCVConfig.cmake`。** 没有就手工接：

```cmake
add_library(opencv_java4 SHARED IMPORTED)
set_target_properties(opencv_java4 PROPERTIES
    IMPORTED_LOCATION ${OPENCV_SDK}/sdk/native/libs/${ANDROID_ABI}/libopencv_java4.so)
include_directories(${OPENCV_SDK}/sdk/native/jni/include)
```

### 2.2 `phone.yaml` 的权威副本放哪

它目前躺在 `ORB_SLAM3/Examples/Monocular-Inertial/`，已在该仓库正常跟踪
（`1e379cf`，09-18 的「镜头配置文件」），可正常提交、有 git 可退。装进 APK
就要在 android 仓库里再放一份——于是有两份，改一份忘一份就会漂移。

**定成单向：`assets/` 那份是权威。** PC 上跑 `mono_inertial_euroc` 时从 `assets/` 拷过去。
否则以后改了外参、改了一份忘一份，两边会悄悄漂移。

---

## 3. 四个地雷

**这四条会直接让你卡住半天以上，动手前先看一眼。**

### 3.1 `-march=native` 会杀死交叉编译

DBoW2 和 g2o 的 `CMakeLists.txt` 里**硬编码了 `-march=native`**：

```
Thirdparty/DBoW2/CMakeLists.txt:8-9      -O3 -march=native
Thirdparty/g2o/CMakeLists.txt:57-58      -O3 -march=native
```

`-march=native` 的意思是"按**本机** CPU 生成指令"。本机是 x86，目标是 arm64——
交叉编译时它会直接报错（`unknown target CPU 'native'`），或者更坏：静默生成错误的指令。

**修法**：把这两处改成受 `ANDROID` 变量保护，或者干脆删掉。
删掉是安全的——`-march=native` 只是优化，不影响正确性。

> 主 `CMakeLists.txt` 里也有同样的写法（`-march=native`），但我们**不用它的**——
> 我们只把 `src/` 和两个 `Thirdparty` 拉进自己的 CMake，全文见 **M3**。

### 3.2 ORB-SLAM3 的失败方式是"进程静默消失"

本版本 `src/` 下有 **16 处 `exit(-1)`**（另有一处在 `include/Settings.h:132` 的内联模板里）：

```
src/System.cc:74,123,145,162,249,333,411
src/Settings.cc:41,51,65,75,89,99,113,137,272
```

比如 settings 文件打开失败：

```cpp
cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
if(!fsSettings.isOpened()) {
   cerr << "Failed to open settings file at: " << strSettingsFile << endl;
   exit(-1);
}
```

在 PC 上这没问题——你看得见 stderr，也看得见退出码。**在 Android 上：**

- stdout/stderr 默认进 `/dev/null` → `cerr` 那行**看不到**
- `exit()` 不走 Java 异常路径 → logcat 里**没有 FATAL EXCEPTION、没有 Java 栈**
- 表现：点「启动 SLAM」→ **App 整个退回桌面** → logcat 干干净净

**这一条会伪装成各种样子**（"一按就闪退"、"没反应"、"装不上"），
而 §7 那套「看信号名 → ndk-stack 符号化」的排查路线**对它完全无效**，因为它不是崩溃，
是正常退出。

**两道防线**：

1. **M2 把 stderr 接进 logcat**（§4），这样 `cerr` 那行至少看得见。
2. **调用 `nativeInit` 之前在 Kotlin 侧校验**：vocab 和 yaml 两个文件存在、yaml 能读到、
   大小不是 0。把最常见的几种 `exit(-1)` 挡在门外，并给出能看懂的提示。

### 3.3 词袋加载是一次慢操作，而且本版本没有快速路径

`ORBvoc.txt` 是 **139 MB 文本**。`System` 的构造里调用
`mpVocabulary->loadFromTextFile(...)`——**只有文本这一条路**：

```
src/System.cc:118, 140   mpVocabulary->loadFromTextFile(strVocFile);
```

有些 ORB-SLAM3 分支有二进制词袋的快速加载，**这个版本没有**（DBoW2 的
`TemplatedVocabulary` 里只有文本加载和基于 `cv::FileStorage` 的存取，
前者被 `System` 用了）。所以：

- **加载要按三十几秒预期**，不是"一瞬间"。**实测本机 37.7 s**（2026-09-27，Xiaomi PJD110）——
  文档里原先写的「十几到几十秒」是猜的，实测落在最上沿。见 M4 一节的 logcat。
- **它必须跑在 SLAM 线程上，不能同步阻塞 UI。**
- **不要每次「启动」都重建 `System` 对象。** 否则每次开关都要重等几十秒，
  用户体验会崩掉——见 §5 决策一（这件事由 `SlamEngine` 负责）。

### 3.4 Boost 是硬依赖，而 Android 上得自己编

**这条最容易漏，因为它不在 `Thirdparty/` 里，PC 上又是系统包自动装好的。**

先看证据——`libORB_SLAM3.so` 确实链着它：

```
$ ldd lib/libORB_SLAM3.so | grep boost
libboost_serialization.so.1.83.0 => /lib/x86_64-linux-gnu/libboost_serialization.so.1.83.0
```

而且**不是可选的**：那 13 个文件里的 `serialize()` 没有任何 `#ifdef` 保护
（`Atlas.h` / `Map.h` / `KeyFrame.h` / `MapPoint.h` / `KeyFrameDatabase.h` /
`CameraModels/*` / `LoopClosing.h` …），`CMakeLists.txt:124` 也是无条件
`-lboost_serialization`。

**所以哪怕你永远不存地图，链接 `libORB_SLAM3.so` 也需要它。** 这不是"顺手多一个依赖"
——Boost.Serialization 拖着一大片 Boost 头（preprocessor / mpl / type_traits…），
交叉编译是这条路上最麻烦的一步。

两条路：

- **自己编**：Boost 是纯 C++、没有平台相关代码，理论上 b2 配好 NDK toolchain 就能过。
  耗时且容易踩坑，但一次性。
- **找现成的 Android 预编译**：省事，但版本要对得上（PC 上那份是 **1.83**）。

**好消息**：既然躲不掉，**存地图就是白送的**——`SaveAtlas` / `LoadAtlas` 本来就在
这份 `.so` 里，不用再付代价。它的能力边界和两处必改见
[MAP_FEATURE_IDEAS.md](MAP_FEATURE_IDEAS.md#四贯穿事实之二地图持久化)。

---

## 4. M0~M8 里程碑

**一次只推进一步，每步都有明确的"过了"标志。** 跳步的代价在这个阶段特别大，
因为两个世界的错误混在一起时几乎无法定位。

### M0 环境就位

```bash
cd ~/Android/Sdk/cmdline-tools/latest/bin
./sdkmanager "ndk;26.1.10909125" "cmake;3.22.1"
```

**过了的标志**：`ls ~/Android/Sdk/ndk ~/Android/Sdk/cmake` 各出一个版本目录，
**并且现有的 `./gradlew assembleDebug` 仍然能过**（加 NDK 不该影响没有 native 的构建）。

> ✅ **本机已完成。** NDK `26.1.10909125`、CMake `3.22.1` 都装好了，
> `app/build.gradle.kts` 里也写死了 `ndkVersion`（不写的话 AGP 会去要它**自己默认**的
> 那个版本，和本机装的对不上就报 `No version of NDK matched`）。

### M1 空壳 `.so` 进 APK

只做**桥本身**，不碰 ORB-SLAM3。建 `app/src/main/cpp/`，写一个最小的
`jni_bridge.cpp`（一个 `JNI_OnLoad` + 一个返回字符串的函数），配好
`CMakeLists.txt` 和 Gradle 的 `externalNativeBuild`。

```kotlin
// app/build.gradle.kts
android {
    defaultConfig {
        ndk { abiFilters += "arm64-v8a" }
    }
    externalNativeBuild {
        cmake { path = file("src/main/cpp/CMakeLists.txt") }
    }
}
```

**过了的标志**：App 启动时 logcat 打出那句来自 C++ 的字符串。

> **这一步不能跳。** 它验证的是「Kotlin ↔ C++ 这条路是通的」。
> 如果连空 .so 都进不去，后面所有问题都会长得像 ORB-SLAM3 的问题。

### M2 把 stdout/stderr 接进 logcat

**在做任何 ORB-SLAM3 相关工作之前做这一步**（理由见 §1.4 和 §3.2）。
实现在 `app/src/main/cpp/log_redirect.cpp`，**调用点是 `jni_bridge.cpp` 的
`JNI_OnLoad`** —— `.so` 一加载就装上，不能拖到第一次调用，否则加载期和初始化期的
输出就丢了。

结构是「**一根管道 + 一个读线程**」：`pipe()` 之后把 fd 1/2 都 `dup2` 成写端，
读线程把字节流按行切开，逐行喂给 `__android_log_write`，tag 是 **`slam`**。

**stdout 和 stderr 合并成一个 tag、都用 INFO 级。** 分两个管道能按级别过滤，
但两个读线程一跑，`cout` 和 `cerr` 的先后顺序就没了 —— 而排「先干了什么、
然后哪一步失败」恰恰更要紧。

四个容易漏的细节：

| 细节 | 漏了会怎样 |
|---|---|
| `setvbuf(stdout, ..., _IOLBF, 0)` | 攒够几 KB 才吐一次，日志看着是「卡住不动，然后突然一大坨」 |
| 按行重组（`read` 只保证给你字节流） | 半行、或好几行粘成一条 |
| 超长行分块（约 3000 字节一块） | logcat 单条上限约 4 KB，**尾巴被静默砍掉** |
| 攒了半行就超时吐出（约 200 ms） | 不带 `endl` 的那句**永远**卡在缓冲区里看不见 |
| **攒行缓冲也按同一个 3000 封顶** | 见下 —— **光有超时兜不住** |

用 `__android_log_write` 而不是 Java 的 `Log`：读管道的是个 pthread，不是 JVM
创建的线程，**没有 `JNIEnv`、也没有 ClassLoader**（§1.2 第三层约定）。

**过了的标志**：`adb logcat -s slam` 里能看到 C++ 用 `cout` / `cerr` / `printf`
写的内容。Kotlin 侧调一次 `SlamBridge.nativeLogSelfTest()` 把这几条路连同分块一起验。

### M3 交叉编译出 `liborb_slam3.so`

> ✅ **本机已完成**（2026-09-27）。改动清单、每处的理由、以及「为什么是这个方案」
> 见 [changes/2026-09-27-m3](../changes/2026-09-27-m3-cross-compile-orb-slam3.md)。
> 下面这一节是**做法和踩过的坑**，代码本身在源文件里，不在这里重复。

**这一步是四件事拼起来的**，顺序不能乱：**剥 Pangolin、打 `-march=native` 补丁、
编 Boost、配 CMakeLists**。最后才是跑构建。

#### 先定依赖从哪来

判据借 ORB-SLAM3 自己在 `Thirdparty/` 上的分法，**完整表格在
`app/src/main/cpp/thirdparty/README.md`**：

- **自带**（进仓库）：ORB-SLAM3 本体、DBoW2、g2o、Sophus ——
  后面三个是作者 **fork 过的版本**，必须跟着代码走。
- **外装**（不进仓库）：OpenCV（官方 Android SDK）、Eigen（`apt install libeigen3-dev`）、
  Boost（自己交叉编译，见下）。

**源码自带、构建系统不共用。** Android 的构建从头到尾只由
`app/src/main/cpp/CMakeLists.txt` 驱动，它把 ORB-SLAM3 的 `src/*.cc` 按路径列出来自己编；
`ORB_SLAM3/CMakeLists.txt` 一次都不参与交叉编译。

#### 剥 Pangolin

Pangolin 是桌面 OpenGL 可视化库，Android 上没有。**依赖范围比想象中小：**

| 文件 | 依赖程度 | 处理 |
|---|---|---|
| `src/Viewer.cc` + `include/Viewer.h` | 真正使用 | **削成空壳**（保留类名和公有方法签名，方法体清空） |
| `src/MapDrawer.cc` + `include/MapDrawer.h` | 真正使用 | **削成空壳** |
| `include/Map.h` | **只是 `#include`，没实际用** | 删掉那一行 |
| `src/System.cc` | 创建 Viewer/MapDrawer 的地方 | 删 `#include <pangolin/pangolin.h>` |

**⚠️ 一个必须纠正的旧说法**：旧教程说「用 `System.UseViewer` 收口」，
**那不是 yaml 键**（见 M4）。传 `bUseViewer=false` **只是不创建 Viewer 线程/窗口**，
**但这些类的代码仍然要被编译和链接** —— `System.h` 里有 `mpMapDrawer` 这类成员指针，
`System.cc` 里有调用点。**所以 stub 必须做，不能靠那个参数省掉。**

**削空壳时两个细节**（都容易漏）：

- `Viewer::isStopped()` / `isFinished()` **要恒返回 true**。`Tracking::Reset` 里有
  `while(!mpViewer->isStopped()) usleep(3000);` —— 原版靠绘制线程把标志翻过去才能退出，
  线程没了之后只要返回一次 false，那里就是个死循环。
- `MapDrawer::SetCurrentCameraPose()` 可以直接空掉。它原本做的是「缓存 `Tcw.inverse()`
  给 Viewer 画相机位置」，而这个方法在 `Tracking` 里**每帧都调**。没有 Viewer 之后
  那个值没人读，连求逆和加锁都省了。

#### 真编了才会暴露的坑

下面这些光看代码不容易想到 —— **都是实际编译撞出来的**：

| 症状 | 原因 | 修法 |
|---|---|---|
| `unknown type name 'GLubyte'` | `GLubyte` 是 Pangolin 带进来的 | `Map.h` 里那个 `GLubyte* mThumbnail` 改成 `unsigned char*`（它只是 typedef，且该成员从没被真正用过） |
| `'stdint-gcc.h' file not found` | 上游误留的 **GCC 内部头**（`ORBmatcher.cc`、`DBoW2/FORB.cpp`） | 改成 `<stdint.h>` |
| g2o 里 `'tr1/unordered_map' file not found` | TR1 是 C++11 之前的 GCC 专有头。**clang 定义了 `__GNUC__`**，所以它走了 GCC 那条分支 | 5 个头文件改成 C++11 标准库，`std::tr1::` → `std::` |
| `error: unknown target CPU 'native'` | DBoW2 / g2o 里硬编码的 `-march=native`（§3.1） | 删掉 |

> **`-include stdint.h` 解决不了 `stdint-gcc.h` 那个错**（实测过）：
> `#include <stdint-gcc.h>` 是无条件指令，先塞别的头进去不会让那个不存在的文件出现。
> 那个办法治的是**另一件事**：很多 ORB-SLAM3 文件用了 `uint8_t` 却没 include `<stdint.h>`。

#### 编 Boost（Android 版）

见 §3.4。**不做必然链接失败** —— 它只在 M8 的存/载地图里用到，但**编译期就躲不掉**。

一条命令：

```bash
cd android/SlamCapture && ./tools/setup-boost-android.sh
```

**它不是源码，是构建产物**，所以 `gitignore` 掉了（绑死 NDK 版本和编译选项、150 MB、
其中只用到 `libboost_serialization` 一个）。Boost 没有官方安卓发布物，只能自己编。

#### CMakeLists

**全文在 `app/src/main/cpp/CMakeLists.txt`。** 几个不写就出事的地方：

| 约束 | 不写会怎样 |
|---|---|
| `project(... LANGUAGES C CXX)` | g2o 里有一个 `os_specific.c`，只开 CXX 会在 configure 阶段就报没有 C 编译器 |
| `set(CMAKE_CXX_STANDARD 14)` | 上游就是 C++14，而且 `LoopClosing.cc` 里有 `mnFullBAIdx++`（bool 自增，C++17 移除），往上抬编不过 |
| `CMAKE_EXPORT_COMPILE_COMMANDS` **写在文件最前** | 它是**创建 target 时**用来初始化属性的，写在末尾形同没写 |
| Boost 的 `.a` **不能**用 `find_library` 找 | NDK 把 `find_*` 的搜索根锁在 sysroot 里，`PATHS` 会被重新挂到 sysroot 下面，**装在哪都找不到** |
| Eigen 要用 `NO_CMAKE_FIND_ROOT_PATH` 找 | 同一个锁。不绕开就找不到系统装的 Eigen，g2o 里的 `FIND_PACKAGE(Eigen3)` 直接失败 |
| `OpenCV_CONFIG_SUBDIR /abi-${ANDROID_ABI}` | 它靠 `ANDROID_NDK_ABI_NAME` 定位，而 NDK 的 toolchain 只设 `ANDROID_ABI` |
| `find_package(OpenCV REQUIRED core imgproc ...)` 点名模块 | 不带限定的话，全量列表里会带上 `opencv_java` —— 一个 18 MB 的 `.so`，Java 绑定用不上 |

**过了的标志**：`liborb_slam3.so` 产出。

```bash
SO=$(find app/build -name "liborb_slam3.so" | head -1)
NM=$(find ~/Android/Sdk/ndk -name llvm-nm | head -1)
"$NM" -D "$SO" | grep -c " T "
```

> 用 `find` 而不是 `**` 通配符——`**` 要 bash 的 `globstar` 才生效，很容易在这类
> 命令上白费时间。

**这一阶段的报错大概率是"include 路径找不到"和"signature 对不上"**——
对着 §7 逐条清。

### M4 能构造 `System`

写 `SlamEngine::init()`，里面 `new System(vocab, settings, System::IMU_MONOCULAR, false)`。

**注意最后那个 `false`**：

> **`System.UseViewer` 不是 yaml 键。** `bUseViewer` 是 `System` **构造函数**的第 4 个参数
> （`include/System.h:105`，默认 `true`）。旧版教程把它写成 yaml 里的一行，那是无效的
> ——`FileStorage` 会忽略它，而 Viewer 照建不误。Android 上没有 Pangolin，必须从参数上关掉。

**这一里程碑会第一次撞上 §3.2 和 §3.3**：路径写错 → 进程静默消失；路径写对 →
等三十几秒的词袋加载（**实测 37.7 s**）。

#### 两个文件从哪来

`System` 要的是**两个真实文件路径**，接口上都是裸路径参数，谁生产它们、放在哪，
§6 的接口不管。实际落地：

| 文件 | 放哪 | 怎么到那 |
|---|---|---|
| `phone.yaml` | **`assets/` 那份是权威**（§2.2），启动时复制到 `filesDir/` 再用 | 打进 APK，几 KB |
| `ORBvoc.txt` | **`getExternalFilesDir()/`**（和采集 session 同一个根，`adb push` 免 root） | `tools/push-vocab.sh` 推上去 |

**词袋 145 MB 不进 APK** —— 它会让 APK 从 60 MB 涨到 200 MB+。这是调试期的做法；
后来的方案是「打进 assets 并 gzip，首次运行解压到 `filesDir`」，见改动记录的遗留。

#### 两道防线，分工要清楚

`exit(-1)` 在 `System.cc` 7 处 + `Settings.cc` 9 处 + `Settings.h` 1 处，**共 17 处**。

**第一道（Kotlin 侧预校验）**：调 `nativeInit` **之前**查两个文件存在、大小非 0、可读。
能挡住的是「路径错 / 文件不在」这一类。

**第二道（M2 的 logcat）**：yaml **存在但缺一个必填键**，第一道挡不住 —— 那种只能靠
ORB-SLAM3 自己 `cerr` 出来那一行（`Settings.h:132` 会说清是哪个键），然后进程消失。

> ⚠️ **yaml 里的 `Viewer.*` 那 10 个键全是 `required=true`**，即使 `bUseViewer=false`
> 也照样必读（`Settings.cc:169` 无条件调 `readViewer`）。少写一个就静默消失。
> 这是写 yaml 时最容易踩的一个。

**过了的标志**：`nativeInit` 返回后，logcat 里有 ORB-SLAM3 自己打印的启动信息
（这就是 M2 的价值），且 App 没有退出。

> 一定要单独做一步「**故意传一个错的 yaml 路径**」，亲眼看一次它是怎么消失的。
> 下次真遇到时才认得出。**更好的负向验证**是「yaml 存在、但删掉一个 `Viewer.*` 键」——
> 它能同时演示两道防线是怎么分工的：第一道放行，第二道把那行 `cerr` 打出来。

### M5 让 SLAM 在手机上跑起来，并证明它算得对

> ⚠️ **2026-09-28：原 M5（喂帧）+ M6（喂 IMU）+ M7（对照 PC）合成一步。**
> `M6` / `M7` 这两个编号不再使用。设备侧的活本来就是一件事，
> 而「量尺度」和「对照 PC」都在 PC 上做，没必要为它们单独占一个里程碑。

**喂帧和喂 IMU 一起做。** 理由：`IMU_MONOCULAR` 模式下跑纯视觉**不是它的设计用途** ——
那个中间态（尺度任意、状态停在 `NOT_INITIALIZED`）不可信，拿它做验收会误导。
而且相机和 IMU 本来就是**同一个「启动 SLAM」按钮的两半**。

#### 验收分三档，后两档在 PC 上

| 档 | 验什么 | 在哪 |
|---|---|---|
| ① | 跟踪从 `NOT_INITIALIZED` 走到 `OK`；位姿随运动平滑变化 | **手机** |
| ② | **标一段已知 1.6 米，用 SLAM 数据推算它** | **PC** |
| ③ | 手机实时的轨迹 vs PC 跑同一段的轨迹 | **PC** |

**所以手机上只要「跑 + 录 + 存轨迹」，量尺度的活在 PC 上一起做。**
（②③ 在 PC 是因为点选工具在那边，而且它们本来就要跑一遍 ORB-SLAM3 才能拿到地图点。）

> ⭐ **② 顺带验的是整条测距链路的雏形** —— 它要「在一帧画面上点两个点 → 查它们对应的
> 3D 地图点 → 算距离」，而那正是 [STAGE6](STAGE6_MEASUREMENT.md) 要做的事。

#### 边跑边录：落盘是「消费者」，不是「模式」

```
录制模式        相机 → 落盘
SLAM 页 + 开关  相机 → SLAM + 落盘
```

**同一份 `SessionWriter`，挂在两个不同的相机所有者下面。** 它自己不拥有相机，
**也不碰对焦**（不重新收敛，只消费帧）。

**这和「两个模式互斥」不矛盾** —— 那条针对的是「两个模式各自开相机」。
见 [APP_DESIGN](APP_DESIGN.md) §6.2 / §6.3。

**为什么需要它**：验收 ②③ 要的是「**同一段数据**」。不边跑边录，手机那次
就什么都没留下，PC 那边没有可跑的东西。

**代价**：开着录制时 SLAM 的帧率会掉（要写 PNG）。**这是刻意的，而且对照仍然成立** ——
录下来的就是手机喂进去的同一段序列。**但要实测「掉到多少 ORB-SLAM3 会跟丢」**，
这一条比尺度更早咬人，是本步的验收项之一。

> ⚠️ **这样录出来的 session 和「录制模式」录的格式完全一样**，事后分不出哪个是带着
> SLAM 跑的。`meta.json` 里要有标记（`with_slam`）—— **不加的话对不上账。**

> **否决过的替代方案：手机上做「回放」**（把 session 按时间戳喂给 SLAM）。
> 它也能拿到对照，而且额外是**离线调算法的唯一手段**（能复现同一段数据，才谈得上
> 「改一行、跑一遍」）。但代价大：一整套回放路径，而且**必须按时间戳节流**才可比
> （「尽快喂」会改掉 ORB-SLAM3 的行为）。**现在不做，要用再说。**

#### ⚠️ 一个静默的坑：`IMU::Point` 的参数顺序

| | 顺序 |
|---|---|
| ORB-SLAM3（`ImuTypes.h:49`） | `IMU::Point(acc_x, acc_y, acc_z, ang_vel_x, ang_vel_y, ang_vel_z, timestamp)` |
| 我们落盘用的 | `ImuItem(tsNs, wx, wy, wz, ax, ay, az)` |

**顺序正好相反。照直觉对过去会把加速度和角速度写反，而且不报任何错** ——
只是跟踪整个乱掉，和「接线没接好」长得一模一样。

**所以喂 IMU 走「一个完整样本」的形式**（一个时间戳 + 角速度 + 加速度），
配对交给采集端那个本来就在做这件事的模块 —— 见 §6 的接口说明。

#### 三条读源码得到的核查（动手前先知道）

**一、不喂 IMU 不会卡死**（虽然我们不用这条路）

```
Tracking.cc:1636   if(mlQueueImuData.size() == 0) { PrintMess("Not IMU data..."); setIntegrated(); return; }
Tracking.cc:1642   while(true) { ... }   ← 两个出口都挂在队列状态上，队列只减不增 ⟹ 必然退出
Tracking.cc:4011   while(!mCurrentFrame.imuIsPreintegrated())  ← 在 UpdateFrameIMU 里，
                                                                 是惯性初始化那条路，没 IMU 走不到
```

（`Tracking.cc:1664` 那行 `bSleep = true` 写在 `break` **后面**，是上游的死代码，无害。）

**二、切片的活是 ORB-SLAM3 干的**

`TrackMonocular(im, ts, vImuMeas)` 内部**先**把 `vImuMeas` 灌进 `mlQueueImuData`，
**然后** `Frame` 构造里的 `Tracking::PreintegrateIMU()` 自己按时间戳 `collectUntil`。
所以我们只要把「上一帧到这一帧之间的 IMU」交给它就行，**不用自己精确切片**。

**三、轨迹要存 `Twc`，不是 `Tcw`**

`SaveTrajectoryEuRoC`（`System.cc:616-622`）构造 `Tcw` 之后**取逆**再写。
和它对齐，③ 才能直接 diff —— **写反了对照会看着像旋转错位，白查一轮。**

#### 过了的标志

① 跟踪走到 `OK`、位姿随运动平滑变化，**按「停止」再「启动」不用重等词袋**；
② 那 1.6 米用 SLAM 数据推算出来对得上；③ 手机轨迹和 PC 轨迹的形状、尺度一致。


### M8 图管理：存 / 载 / 切图 / 重定位

**这一步才是 `SlamEngine` 真正区别于一个普通 wrapper 的地方。**
前面七步做完，"实时跑 SLAM"就有了；这一步做的是**产品形态**（[APP_DESIGN](APP_DESIGN.md)）。

四件事，按依赖顺序：

1. **存** —— `saveTo(path)`。要先把 `SaveAtlas` / `LoadAtlas` 里**写死的 `./`**
   改成传入的绝对路径（`System.cc:1411` / `:1450`；Android 上进程 CWD 是 `/`，
   不可写）。详见 [MAP_FEATURE_IDEAS](MAP_FEATURE_IDEAS.md#四贯穿事实之二地图持久化)
   的「四个坑」。
2. **载** —— `init(..., loadFromFile)`。注意是**整个 Atlas 进来**，
   而「旧图」要的是**一张 `Map`**：挑关键帧最多的那张。
3. **切图** —— `Atlas::SetCurrentMap()`，**自己加**（ORB-SLAM3 没有）。
   上层在这一刻**清空轨迹**（§5 决策四）。
4. **重定位** —— `relocalize()`，返回失败原因编码（§6）。
   **实现前先跑一遍 [MAP_FEATURE_IDEAS](MAP_FEATURE_IDEAS.md) §4「待实测」那三步**：
   那里会先告诉你「加载完当前图是空的、候选被 `pMap` 过滤挡住」这个推断**对不对**。
   **这一步是这条路能不能走的试金石，别跳过。**

**过了的标志**：走通「建图 → 存 → 重启 → 载入 → 走到路径点 → 按重定位 → 切过去
→ 轨迹从那里重新开始」这一整条。

> ⚠️ **这一步是 ORB-SLAM3 最不成熟的部分**（七个被推翻的判断全在这附近）。
> **小步走，每步都跑一次看行为**，不要靠读代码规划。

---

## 5. 接进 App

> **⚠️ 2026-09-26 更新**：整体形态已定，见 [APP_DESIGN.md](APP_DESIGN.md)。
> **相机和 IMU 不再挂在采集端的「准备」上**，而是挂在 **「启动 SLAM」** 上——
> 因为 SLAM 也需要它们，而且要常跑。下面这张表取代了原来的「采集端状态机挂接点」。

| 时机 | 做什么 |
|---|---|
| 入口页选完（新建 / 继续上次） | **什么都不做**，只记选择 |
| **按「启动 SLAM」（第一次）** | `nativeInit(...)` → 构造 `System`（**异步**，词袋十几到几十秒）→ 开相机 + IMU → `nativeStart()` |
| **按「启动 SLAM」（之后）** | 跳过 init，直接 开相机 + IMU + `nativeStart()` |
| `onCameraFrame()` | `nativePushImage(...)` |
| `gyroListener` / `accelListener` | `nativePushImu(...)` |
| **按「重定位」** | `nativeRelocalize()` → 成功则 `SlamEngine` 内部**切图**，上层**清空轨迹** |
| **按「停止」/ `onStop`** | 关相机 + IMU + `nativeStop()`。**`System` 保留** |
| **进程退出** | `nativeShutdown()` |

**落盘（录制）是另一条独立的线**，跟采集端那套 `IDLE ─开始准备→ ARMED ⇄ RECORDING`
状态机走，**一行都不用改**（见 [APP_DESIGN](APP_DESIGN.md) §6）。

### 决策一：`System` 对象活到进程结束，且这件事由 `SlamEngine` 负责

**IMU 和相机挂在「启动 SLAM」上**（停止就注销/关闭），因为它们的代价是**耗电**。

**SLAM 的代价结构不一样**：销毁 `System` 意味着下次要**重新加载 139 MB 词袋**
（§3.4/§3.3），几十秒。**所以它在第一次「启动」时构造一次，跨「停止」一直保留**，
直到进程退出。

> 判据是「**这个资源的代价类型是什么**」：
> **耗电的资源跟着用户意图走（启动/停止），耗时的资源跟着进程走。**

**这件事由 `SlamEngine` 持有** —— Kotlin 侧不用记着"别重建"。
这也是 §2.6 那个类的第一个理由。

### 决策二：`nativeInit` 必须异步

词袋加载十几到几十秒，**同步调用会把「启动」按钮卡死**。

做法：`nativeInit` 立刻返回，真正的构造在 SLAM 线程里做；Kotlin 侧轮询
`nativeIsReady()`（状态栏本来就有 ticker）。

### 决策三：落盘的绿色判据**不加** SLAM 这条线

落盘的判据是**相机就绪 && IMU 就绪**。SLAM 进来之后有了第三条线，但**不加进去**：

**落盘本身不需要 SLAM** —— 它只需要相机和 IMU，SLAM 是消费者。
先录下来、SLAM 慢一点跟上，反而是更稳的顺序。

**判据应该只包含"录得对不对"的东西**；SLAM 好不好是另一回事，
混进去会让一个慢操作阻塞一个快操作。

### 决策四：切图时上层要**清空轨迹**

重定位成功 → `SlamEngine` 内部把当前图换成旧图。**但上层必须同时清空轨迹记录** ——
切图前的轨迹点在**新图坐标系**里，切图后的是**旧图坐标系**，两份没有对齐关系
（那个关系只有 LoopClosing 回环合并时才有），硬接上去轨迹会跳。

**清零是唯一诚实的选择**，而且符合语义：按重定位的意思就是「从这里开始」。
详见 [APP_DESIGN](APP_DESIGN.md) §3。

### 关于帧数据怎么传

旧版教程说"用 DirectByteBuffer 零拷贝"，**这条和现有代码冲突，先别做**：

`extractGray()` 返回的 `ByteArray` 现在有**两个持有者**——`SessionWriter` 的队列
（异步写盘）和 `PreviewView.pending`（异步绘制）。**复用一个 buffer 会让这两路
读到已经被覆盖的帧。**

而且「复用 + C++ 端 `clone()`」是**两次拷贝**，比一次都不省。

**先用 `ByteArray` + `GetByteArrayElements` + `clone()`。** 300KB × 30fps = 9MB/s 的
memcpy，在这个阶段完全不值得优化。真要优化时的正解是：`extractGray` 每帧产出一个
DirectByteBuffer，三路消费者都从 buffer 读，C++ 侧不再 `clone()`。

---

## 6. 接口约定

```kotlin
// SlamBridge.kt
object SlamBridge {
    init { System.loadLibrary("slam_jni") }

    // —— 生命周期。init 只在第一次「启动」时调一次 ——
    external fun nativeInit(vocabPath: String, settingsPath: String, loadFromFile: String?)
    external fun nativeStart()
    external fun nativeStop()
    external fun nativeShutdown()

    // —— 重定位：接到 referenceMap 上 ——
    // 0=成功  1=没有旧图  2=DB 里没候选（你不在旧图覆盖的地方）
    // 3=有候选但匹配不上（场景/光线变了）
    external fun nativeRelocalize(): Int

    // —— 每帧 / 每样本 ——
    external fun nativePushImage(tsNs: Long, gray: ByteArray, w: Int, h: Int)
    external fun nativePushImu(tsNs: Long, v0: Float, v1: Float, v2: Float, isAccel: Boolean)

    // —— 查询 ——
    external fun nativeGetPose(out: FloatArray): Boolean    // 4x4 列主序
    external fun nativeGetTrackingState(): Int
    external fun nativeIsReady(): Boolean                   // System 构造完了没
}
```

C++ 侧的符号名必须严格是 `Java_com_example_slamcapture_SlamBridge_<方法名>`。

**四条必须遵守的约定：**

1. 每个 `extern "C"` 函数体包 `try/catch(...)`——**异常不过界**。
2. 时间戳原样传（纳秒，`CLOCK_BOOTTIME`），**到 SLAM 线程里再统一减基准转秒**。
   不要在 Kotlin 侧做任何换算。
3. `nativeGetPose` 用**出参**而不是返回值：JNI 里构造 Java 数组代价高，
   Kotlin 侧复用同一个 `FloatArray` 更省。
4. **「图」的概念完全不跨 JNI。** Kotlin 侧看不到 `Map` / `Atlas` / `referenceMap`，
   只看到上面这些动作。**这是个好信号** —— 说明这条边界划对了。
   （`nativeRelocalize` 返回的是**失败原因编码**，不是 Java 对象。）

### `SlamEngine` 侧的形状

```cpp
class SlamEngine {
    ORB_SLAM3::System* sys_;              // 活到进程结束
    ORB_SLAM3::Map*    referenceMap_;     // 可能为空
public:
    void init(vocab, settings, sensor, loadFromFile);   // 只调一次
    void start();                                        // 开始吃帧
    void stop();                                         // referenceMap_ = 当前图
    int  relocalize();                                   // 在 referenceMap_ 里找 + 切过去
    void saveTo(path);

    // ⚠️ 预留：在 SLAM 线程上取一次快照（轨迹点 + 地图点），**深拷贝**。
    // 现在不实现，但**形状要留在这儿** —— 见下面那条。
};
```

**它最实在的价值是「切图」** —— `Atlas::SetCurrentMap()` 在 ORB-SLAM3 里
**不存在**（见 [MAP_FEATURE_IDEAS](MAP_FEATURE_IDEAS.md) §4 的「四个坑」），
要自己加，而这个类**同时拥有**这个能力。

#### ⚠️ 给 2D 地图预留的那个接口（2026-09-28）

**2D 地图/轨迹视图迟早要加**（设计已定，见 [APP_DESIGN](APP_DESIGN.md) §5），
而它**唯一的结构性依赖**是这个类上还不存在的一个方法：

> 取一次快照（轨迹点 + 地图点，或者关键帧位置），返回**深拷贝**。

两条硬约束，都不是"以后再说"能绕过去的：

1. **必须在 SLAM 线程上取** —— Map / KeyFrame / MapPoint 是 `Tracking` 和
   `LocalMapping` 正在写的东西，在别的线程读是**数据竞争**。
2. **必须深拷贝** —— 只拿指针回去画，下一轮优化就把它释放了，
   表现是**偶发崩溃或画到垃圾数据**。

**所以 `SlamEngine` 的形状里现在就给它留位置**（哪怕先留个空实现）。
理由：它和「测距」那个消费者是**同一个**接口（[STAGE6](STAGE6_MEASUREMENT.md) 已把两者的
需求分开写清楚了 —— 测距**不需要**全图快照，而 2D 地图需要）。
**后加这个接口 = 回头改 SLAM 线程的结构**；先留位置 = 只加一个方法。

**⚠️ 封装要薄**：只记账 + 转发，**不试图重新定义「图」的语义**。
ORB-SLAM3 的 Atlas/Map 那块本身就糙，**薄封装至少漏的时候一眼看得见**。

---

## 7. 报错速查

| # | 症状 | 原因 | 修法 |
|---|---|---|---|
| 1 | `UnsatisfiedLinkError: No implementation found` | C 函数名/签名与 Kotlin 不符 | 逐段核对包名；包名里的 `_` 要写成 `_1`；重载方法需要 `__签名` 后缀 |
| 2 | **点一下 App 就退回桌面，logcat 干净** | ORB-SLAM3 里的 `exit(-1)`（§3.2） | 先做 M2 接出 stderr；再在 Kotlin 侧预校验文件 |
| 3 | 启动就崩 `couldn't find libslam_jni.so` | ABI 混装（模拟器是 x86） | `abiFilters += "arm64-v8a"`，并且只用真机 |
| 4 | `error: unknown target CPU 'native'` | DBoW2/g2o 里的 `-march=native`（§3.1） | 删掉那两处 |
| 5 | `local reference table overflow` | 循环里 `NewStringUTF`/`FindClass` 不释放 | 循环外缓存 `NewGlobalRef` |
| 6 | native 线程里 `FindClass` 返回 null | 非 Java 创建的线程没有 ClassLoader | 启动时在 Java 线程缓存 `jclass` 为 GlobalRef |
| 7 | SIGSEGV 崩在 env 用法上 | 跨线程用了别人的 `JNIEnv*` | `JavaVM->AttachCurrentThread` 取本线程的 env |
| 8 | 进程瞬间死、Java 栈为空 | C++ 异常穿过了 JNI | 函数体整体 `try/catch(...)` |
| 9 | ANR / 相机掉帧 | JNI 里干重活 | 铁律 1：回调只进队列 |
| 10 | 图像错位/花屏 | 假设了 `rowStride == width` | 按 stride 构造 `cv::Mat` |
| 11 | 帧率锯齿、GC 抖动 | Java 侧每帧分配大数组 | 复用缓冲（但先看 §5 的警告） |
| 12 | 切后台回来位姿不对 | 停期间手机移动了，上一帧位姿不再成立 | **正常**。`onStop` 走「停止」，回来按「重定位」接回去（§5 挂接表） |
| 13 | 点「启动 SLAM」卡住不动 | `nativeInit` 同步阻塞了 UI | 异步化（§5 决策二） |
| 14 | **重定位总是返回「没候选」** | 候选被 `pMap` 过滤挡住——**当前图是空的** | **先切图再重定位**（M8 第 3 步）；这就是 `SetCurrentMap()` 必须自己加的原因 |

排查顺序：**先确认它在哪一层**——M1 的字符串还打不打得出？M2 的日志还在不在？
分清楚是「桥断了」还是「ORB-SLAM3 出错了」，再往下查。

```bash
SYM=$(find app/build -type d -name "arm64-v8a" -path "*obj*" | head -1)
adb logcat | ndk-stack -sym "$SYM"
```

`ndk-stack` 把 `.so` 里的崩溃地址翻译回 `文件:行号`。**但它只处理信号崩溃，
对 §3.2 的 `exit(-1)` 无效。**

---

## 8. 性能：现在不做优化，但**留了路**

**§0 说「不做性能优化（先跑通）」—— 但那些取舍是刻意的**，记在这里，
免得以后想优化时找不到当初为什么那么写。

### 现在的取舍（都是"先跑通"）

| 取舍 | 现在 | 代价 | 在哪 |
|---|---|---|---|
| 帧数据传递 | `ByteArray` + 拷贝（**两次**） | 9 MB/s memcpy | §5「关于帧数据怎么传」 |
| **特征提取** | ORB-SLAM3 自带的 **CPU** 实现 | **最大的一块**（多少要测） | 上游 `ORBextractor.cc` |
| 灰度转换 | **直接用 Y 平面，零转换** | ✅ 已经最优 | `extractGray()` |
| 喂帧 | 单线程队列 | 够用 | `SlamEngine` |

### 优化方向，按性价比

| 方向 | 针对 | 判断 |
|---|---|---|
| **NEON / DSP** | **ORB 特征提取**（FAST + 描述子） | **最值** —— 纯计算密集、可向量化，而且是大头 |
| 零拷贝 | 帧传递那两次 memcpy | 收益小（9 MB/s 对现代手机不算什么），**而且要先解决三路消费者** |
| **GPU** | 去畸变、灰度、图像金字塔 | **现在都不是瓶颈**（去畸变是恒等、灰度零转换） |

> ⚠️ **优化前先测，别凭直觉。** ORB-SLAM3 自带计时：
>
> ```cmake
> add_definitions(-DREGISTER_TIMES)     # CMakeLists 里加一行
> ```
>
> 开了之后 `Shutdown()` 会调 `PrintTimeStats()`（`System.cc:557`），
> **打印分阶段耗时分布**。**先看分布再决定优化谁** —— 猜错的代价是白干几天。
>
> （`REGISTER_TIMES` 是个**编译期宏**，不在 ORB-SLAM3 的 CMakeLists 里，要自己加。）

### 要留的三条路

1. **别把接口焊死。** `nativePushImage` 现在收 `ByteArray`，将来可能收 `ByteBuffer`
   或 GPU 纹理。**接口本身改起来不难，难的是三路消费者（落盘 / 预览 / SLAM）同时改。**
2. **`extractGray()` 是「图像进系统」的唯一入口** —— 将来换 GPU/DSP 从它切，
   别在别处再开第二条路。
3. **瓶颈大概率不在"传递"而在"计算"** —— 所以"先跑通用拷贝"这个取舍，
   大概率不会成为以后的障碍。

---

## 9. 相关

- 设计蓝图：[ANDROID_SLAM_TUTORIAL.md §6](ANDROID_SLAM_TUTORIAL.md#6-阶段五jni-桥接)
- TrackMonocular 调用链与 VIBA 口径：[ORB_SLAM3_TRACKING_FLOW.md](ORB_SLAM3_TRACKING_FLOW.md)
- 采集端状态机：[changes/2026-09-21](../changes/2026-09-21-camera-preview-and-gating.md)
- 下一步：[STAGE6_MEASUREMENT.md](STAGE6_MEASUREMENT.md)（实时尺寸测量）
- 产品形态（入口页 / 视图 / 切图 / 路径点）：[APP_DESIGN.md](APP_DESIGN.md)
  —— **wrapper 里要顺手留的桩不是"拍照快照"，而是「在 SLAM 线程上取一次
  关键帧/地图点的深拷贝」**（2D 地图和覆盖率都要它；测量用的
  `GetTrackedKeyPointsUn()` 是现成的，不用另做）
