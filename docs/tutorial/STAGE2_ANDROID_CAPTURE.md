# 阶段二实战：Android 采集层（零基础版）

> 目标读者：没用过 Android 开发、机器上什么套件都没有的人。
> 本文所有命令都在 **WSL2 里**执行，不需要 sudo，不碰系统目录。
> 你只需要一个能上网的环境 + 一台 Android 11 以上的手机。

---

## 0. 先搞清楚这一阶段做什么

### 做什么

```
相机 640×480 @30fps  ─┐
                      ├─→ 同一个时钟基准的纳秒时间戳 ─→ 落盘成文件
IMU 加速度 + 陀螺仪   ─┘
```

跑完之后你手上应该有一个目录，长这样：

```
session_1234567890/
├── cam0/
│   ├── data/1758000000000000000.png     ← 文件名就是时间戳（纳秒）
│   ├── data/1758000033333333333.png
│   └── data.csv                          ← 每行：时间戳,文件名
├── imu0/
│   └── data.csv                          ← 每行：时间戳,wx,wy,wz,ax,ay,az
└── meta.json
```

这个格式是 **EuRoC 数据集的原生格式**，阶段三可以直接喂给 ORB-SLAM3 的
`mono_inertial_euroc`，一行转换代码都不用写。这是刻意的设计。

### 不做什么（重要）

| 不做 | 为什么 |
|---|---|
| ❌ 不接 SLAM | 这是阶段五的事。这一阶段只验证"采集"本身 |
| ❌ 不写 JNI / C++ | 纯 Kotlin。教程 3.1 里那个 `app/src/main/cpp/` 目录**现在不要建** |
| ❌ 不用 NDK | 上面那条的推论 |
| ❌ 不用模拟器 | 模拟器没有真实相机、更没有真实 IMU，对本项目**毫无价值** |

**为什么这个顺序能救命**：采集层是整个项目最容易翻车的地方。如果一开始就把
SLAM 接上，出了问题是"相机坏了"还是"标定不对"还是"ORB-SLAM3 抽风"你分不清。
先只做采集，把变量砍到一个，问题就变得可诊断了。

### 阶段二的验收标准

跑完本文，你要能回答这三个问题：

1. 实际帧率是多少？稳不稳？（目标 30fps，掉帧 < 1%）
2. IMU 实际多少 Hz？和相机的时间戳对不对得上？
3. 相机画面亮度够不够？有没有过曝/欠曝？

---

## 1. 环境搭建

### 1.1 装 JDK 17

Android 的构建工具链要求 JDK 17。系统里没有，你也没有 sudo 的免密权限，
所以**下载压缩包解到你自己的家目录**，不碰 `/usr`。

```bash
mkdir -p ~/Android/downloads
cd ~/Android/downloads

# 从 Adoptium 下载 Temurin JDK 17（约 190MB）
curl -L --progress-bar -o jdk17.tar.gz \
  "https://api.adoptium.net/v3/binary/latest/17/ga/linux/x64/jdk/hotspot/normal/eclipse"

# 解压到 ~/Android/jdk-17
mkdir -p ~/Android/jdk-17
tar xzf jdk17.tar.gz -C ~/Android/jdk-17 --strip-components=1

# 验证
~/Android/jdk-17/bin/java -version
```

最后一条应该打印类似：

```
openjdk version "17.0.20" 2026-xx-xx
OpenJDK Runtime Environment Temurin-17.0.20+8 (build 17.0.20+8)
```

> **为什么是 17 而不是 21**：AGP 8.5 支持 17 和 21，但 17 是兼容性最稳的选择。
> Android 官方文档默认用它。

### 1.2 配置环境变量

**这一步必须在装 Gradle 之前做**——Gradle 自己是用 Java 写的，找不到 JDK
会直接报 `JAVA_HOME is not set`。

```bash
cat >> ~/.bashrc << 'EOF'

# ==== Android 开发环境 ====
export JAVA_HOME=$HOME/Android/jdk-17
export ANDROID_HOME=$HOME/Android/Sdk
export ANDROID_SDK_ROOT=$HOME/Android/Sdk
export PATH=$JAVA_HOME/bin:$ANDROID_HOME/cmdline-tools/latest/bin:$ANDROID_HOME/platform-tools:$PATH
EOF

source ~/.bashrc

# 验证
echo $JAVA_HOME
java -version
```

`java -version` 能打印出 17 就成了。

> **`ANDROID_HOME` 和 `platform-tools` 现在就加进 PATH 了**，虽然还没装。
> PATH 里指向不存在的目录是无害的，装完立刻就能用 `adb`，省得回头再改一次。

### 1.3 装 Android SDK 命令行工具

Android SDK 本体也是几百 MB，同样装到家目录。

```bash
cd ~/Android/downloads

# 官方命令行工具（约 130MB）
curl -L --progress-bar -o cmdline-tools.zip \
  "https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip"

# SDK 的目录结构有硬性要求：必须是 <sdk>/cmdline-tools/latest/ 这一层
mkdir -p ~/Android/Sdk/cmdline-tools
unzip -q cmdline-tools.zip -d /tmp/ct
mv /tmp/ct/cmdline-tools ~/Android/Sdk/cmdline-tools/latest
rm -rf /tmp/ct
```

> **如果 `unzip` 没装**（会提示 command not found），用 Python 代替：
> ```bash
> python3 -m zipfile -e cmdline-tools.zip /tmp/ct
> ```

> **目录名 `latest` 是硬要求**，不是习惯。`sdkmanager` 会按这个路径去找自己。
> 你可以同时装多个版本的 cmdline-tools，靠目录名区分。

### 1.4 装 Gradle

Android Studio 会自带 Gradle，但我们不用 IDE，所以要自己装一个。

```bash
cd ~/Android/downloads

curl -L --progress-bar -o gradle-8.7-bin.zip \
  "https://services.gradle.org/distributions/gradle-8.7-bin.zip"

unzip -q gradle-8.7-bin.zip -d ~/Android/
chmod +x ~/Android/gradle-8.7/bin/gradle

~/Android/gradle-8.7/bin/gradle --version
```

> **为什么要先装 Gradle**：官方推荐的用法是用 `gradlew`（Gradle Wrapper），
> 但 `gradlew` 依赖一个 `gradle-wrapper.jar` 二进制文件，这个文件平时是
> Android Studio 生成的。**没有 IDE 的时候，得先用真正的 gradle 生成一次
> wrapper**。这就是 1.4 存在的唯一理由——生成完就可以只认 `./gradlew` 了。

### 1.5 装 SDK 组件

```bash
# 先接受许可
sdkmanager --licenses
# 会连续问你几个 y/n，全部按 y 回车

# 装三个包
sdkmanager "platform-tools" "platforms;android-34" "build-tools;34.0.0"
```

三个包各自的作用：

| 包 | 作用 | 大小 |
|---|---|---|
| `platform-tools` | 提供 `adb`、`fastboot` | ~10MB |
| `platforms;android-34` | Android 14 的 API 定义（编译期需要） | ~60MB |
| `build-tools;34.0.0` | `aapt2`、`d8`、`apksigner` 等编译工具 | ~50MB |

### 1.6 验证环境

```bash
echo "JAVA_HOME   = $JAVA_HOME"
java -version
echo "ANDROID_HOME= $ANDROID_HOME"
adb version
sdkmanager --list_installed
```

`adb version` 能打印版本号就说明 1.5 成功了。

---

## 2. 建工程

### 2.1 目录结构

```bash
mkdir -p ~/SLAMAPP/android/SlamCapture/app/src/main/java/com/example/slamcapture
mkdir -p ~/SLAMAPP/android/SlamCapture/app/src/main/res/values
mkdir -p ~/SLAMAPP/android/SlamCapture/app/src/main/res/layout
cd ~/SLAMAPP/android/SlamCapture
```

最终长这样（**注意：没有 `cpp/` 目录**）：

```
SlamCapture/
├── settings.gradle.kts
├── build.gradle.kts
├── gradle.properties
├── local.properties
└── app/
    ├── build.gradle.kts
    └── src/main/
        ├── AndroidManifest.xml
        ├── java/com/example/slamcapture/
        │   ├── MainActivity.kt
        │   ├── CameraCapture.kt
        │   ├── ImuCollector.kt
        │   └── SessionWriter.kt
        └── res/
            ├── layout/activity_main.xml
            └── values/strings.xml
```

### 2.2 文件清单

工程里一共 13 个文件。**源码以 `~/SLAMAPP/android/SlamCapture/` 里的为准**，
下面只说明每个文件负责什么，不再贴代码。

| 文件 | 职责 |
|---|---|
| `settings.gradle.kts` | 声明模块和依赖仓库 |
| `build.gradle.kts` | 声明 AGP / Kotlin 插件版本 |
| `gradle.properties` | 构建参数（内存、AndroidX 开关） |
| `local.properties` | 指向 SDK 路径，与机器相关 |
| `app/build.gradle.kts` | 模块配置：compileSdk / minSdk / targetSdk 和依赖 |
| `app/src/main/AndroidManifest.xml` | 权限声明、Activity 注册、方向锁定 |
| `app/src/main/res/layout/activity_main.xml` | 一个按钮 + 一个状态文本框 |
| `app/src/main/res/values/strings.xml` | 应用名 |
| `GrayPng.kt` | 手写 8 位灰度 PNG 编码器（不走 Bitmap） |
| `SessionWriter.kt` | 后台落盘 |
| `CameraCapture.kt` | Camera2 采集 |
| `ImuCollector.kt` | IMU 采集 |
| `MainActivity.kt` | 权限、生命周期，把上面三个串起来 |

### 2.3 三件不能省的事

**① 两个权限声明**（都在 `AndroidManifest.xml`，漏了会直接崩）

```xml
<uses-permission android:name="android.permission.CAMERA" />
<uses-permission android:name="android.permission.HIGH_SAMPLING_RATE_SENSORS" />
```

第二条是 **Android 12（API 31）新增**的。想以超过 200Hz 采样传感器就必须
声明，而 `SENSOR_DELAY_FASTEST` 正好是 0 微秒（不限速）。漏了会在
`registerListener()` 抛 `SecurityException` 直接崩掉。

它是 **normal 级权限，安装时自动授予，不需要运行时申请**——加一行声明就够。

> 但 `CAMERA` 不一样，它是 dangerous 级，**必须代码里动态申请**，
> 见 §3.4。

**② Activity 锁方向**（`android:screenOrientation="locked"`）

Android 的 IMU 数据在**设备坐标系**下。屏幕从竖屏转成横屏，整个坐标系
就跟着转 90°，IMU 轴跳变会让 SLAM 当场崩溃。锁死是最省事的解法，
而且阶段四标定出来的相机-IMU 外参也是绑定在这个固定方向上的。

**③ 不申请存储权限**

数据写进 App 私有目录（`getExternalFilesDir()`），Android 10+ 不需要
任何权限，`adb pull` 也不用 root。

### 2.4 版本选择

| 项 | 值 | 理由 |
|---|---|---|
| JDK | 17 | AGP 8.x 要求 |
| Gradle | 8.7 | AGP 8.5 要求 8.7+ |
| AGP | 8.5.2 | 与 Gradle 8.7 匹配 |
| Kotlin | 1.9.24 | 与 AGP 8.5 匹配 |
| compileSdk / targetSdk | 34 | Android 14 |
| minSdk | 26 | Android 8.0。再往下 Camera2 的行为差异要额外处理 |

**这几个版本是相互绑定的，不要单独升某一个**，否则会出现
`Unsupported class file major version` 之类的错误。

---

## 3. 四个源文件的设计要点

代码在 `app/src/main/java/com/example/slamcapture/`。这一节只记录
**为什么这么写**，以及**踩过的坑**。

### 3.1 `SessionWriter.kt` —— 后台落盘

**职责**：把帧和 IMU 样本写成 EuRoC 数据集格式，让阶段三能不加转换地
直接喂给 ORB-SLAM3。

```
sessionDir/cam0/data/<时间戳ns>.png
sessionDir/cam0/data.csv     # 时间戳,文件名
sessionDir/imu0/data.csv     # 时间戳,wx,wy,wz,ax,ay,az
sessionDir/meta.json         # 统计信息
```

**关键设计：**

| 决策 | 原因 |
|---|---|
| 采集线程只管塞队列，另开线程写盘 | 写一张 PNG 要几毫秒，写在采集回调里必然掉帧 |
| `ArrayBlockingQueue.offer()` | 满了立刻返回 false，**不阻塞**。用 `put()` 会把磁盘速度绑到相机上 |
| 队列满就丢帧 + 计数 | 宁可丢帧，也不要让采集线程卡住 |
| 每 30 帧 `flush()` | 全放在 close 里 flush，App 被系统杀掉就全丢了 |

**这里踩过两个坑，都值得记住。**

#### 坑 ①：不要用 `Bitmap` 来编码

最初走的是 `Bitmap.compress(PNG)` 这条路，踩了两级：

**第一级** —— 用 `Bitmap.Config.ALPHA_8`。它是"纯 alpha"格式（只有透明度，
没有颜色通道），Skia 的 PNG 编码器**不支持**它，`compress()` 直接返回
`false` 且一个字节都不写。

**第二级** —— 换成 `ARGB_8888`（每像素 4 字节，R=G=B=灰度值）之后能出图了，
但每帧编码要 100ms 以上。灰度数据本来只有 **1 字节/像素**，等于让 deflate
去压 **4 倍**的数据量。写盘线程跟不上 30fps → 64 深的队列秒满 → 丢帧率
65%、实际帧率只有 **8.8fps**。

**最终解法：手写 8 位灰度 PNG 编码器**（`GrayPng.kt`）。

| | Bitmap / ARGB_8888 | 手写灰度 PNG |
|---|---|---|
| 每像素字节 | 4 | **1** |
| 每帧编码耗时 | > 100 ms | **~15 ms** |
| 产物格式 | 8-bit **RGBA** | 8-bit **grayscale** |
| 依赖 | Skia | 无 |

PNG 的灰度格式本身很简单，`GrayPng.kt` 一共一百多行：

```
签名(8字节) + IHDR + IDAT + IEND

每个 chunk = 长度(4) + 类型(4) + 数据 + CRC32(4)，全部大端
IDAT 里装的是一个 zlib 流，内容是逐行的：
  [滤波器字节(0)] [该行的 width 个像素]
```

三个容易写错的点：

| 点 | 正确做法 |
|---|---|
| IHDR 的 color type | 填 **0**（灰度）。填 6 是 RGBA，宽度就得按 4 倍算 |
| IDAT 的压缩格式 | PNG 要的是 **zlib 流**（带 2 字节头 + adler32 尾）。`Deflater` 默认就输出这个，**不要**设 `nowrap=true` |
| CRC32 覆盖范围 | **类型 + 数据**，不含前面那 4 字节长度 |

压缩级别用 `Deflater.BEST_SPEED` 而不是默认级别——**帧率是硬指标，
文件大小不是**。灰度 640×480 用 BEST_SPEED 也能压到约 50%。

#### 坑 ②：编码失败是**静默**的

`Bitmap.compress()` 不抛异常，只返回 `false`，且**一个字节都不写**。
如果不检查返回值，会得到最坏的结果：

> 文件被创建了、但大小是 0；CSV 里记录得好好的；界面上的帧数、帧率、
> 丢帧率全部正常。**只有图是空的。**

这种"指标全对但数据是废的"是最难发现的一类 bug，因为它看起来一切正常。

换成手写编码器之后异常会正常抛出，但**这道校验仍然保留**——磁盘写满、
编码器内部的边界情况都可能留下空文件：

```kotlin
FileOutputStream(file).use { out ->
    GrayPng.encode(f.gray, f.w, f.h, out)
}

if (file.length() == 0L) {
    file.delete()
    framesFailed.incrementAndGet()
    Log.e(TAG, "PNG 写出为空 ts=${f.tsNs}（已删除空文件）")
    return
}
```

界面上 `stats()` 里的 **`编码失败` 这一项必须为 0**。

### 3.2 `CameraCapture.kt` —— Camera2 采集

**职责**：从主摄以 640×480 固定 30fps 取 YUV，提取 Y 平面当灰度图。

**关键设计：**

| 决策 | 原因 |
|---|---|
| `acquireLatestImage()` | 自动丢弃积压的旧帧。用 `acquireNextImage()` 会排队，延迟越积越大 |
| `image.close()` 放 `finally` | 不 close 的话 4 个 buffer 瞬间耗尽，采集直接停 |
| Y 平面直接当灰度 | `YUV_420_888` 的 Y 平面**就是**灰度图，零转换代价 |
| 处理 `rowStride != W` | 硬件为对齐会在每行末尾加 padding，不处理图像会斜切 |
| `TEMPLATE_RECORD` | 适合连续采集。`TEMPLATE_PREVIEW` 会开自动对焦 |

**必须关闭的两项：**

```kotlin
// ① EIS 电子防抖——会裁切并变形画面，直接毁掉标定
set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
    CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
// ② OIS 光学防抖——会改变有效内参
set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
    CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF)
```

| 项 | 不关的后果 |
|---|---|
| EIS | 会裁切并变形画面，**直接毁掉标定** |
| OIS | 改变有效内参 |

#### 坑 ③：关 AF 不等于"冻结焦距"——这是全工程最贵的一个坑

SLAM 要求焦距固定（对焦呼吸会改变内参），所以最终要关掉 AF。但
**光写 `AF_MODE_OFF` 是不够的**：

> 在小米/红米这类机器上，HAL 收到 `AF_MODE_OFF` 会把镜组**驱动回默认
> 停泊位**（通常是最近对焦距离或无穷远），而**不是**"停在当前位置"。

这个坑的迷惑性在于**它的失败是延迟的**：

```
第 1-7 帧    方差 ~1080   ← 很锐，看着一切正常
第 8 帧起    方差 ~44     ← 突然糊，然后整段一直糊
```

前 7 帧清晰是因为 HAL 还没执行完，镜组**正在滑向停车位的路上**（这段
路程约 230ms）。如果只抽查开头，会以为成功了。

**正确做法：`AF_MODE_OFF` 必须和 `LENS_FOCUS_DISTANCE` 一起给。**

| 阶段 | 请求里设什么 |
|---|---|
| 采集刚开始 | `CONTROL_AF_MODE_CONTINUOUS_PICTURE`，让系统把焦点找准 |
| `AF_STATE` 变成 `FOCUSED_LOCKED`/`PASSIVE_FOCUSED` | 从 `CaptureResult` 读出 `LENS_FOCUS_DISTANCE`，然后 `AF_MODE_OFF` + `LENS_FOCUS_DISTANCE = 该值` |

`LENS_FOCUS_DISTANCE` 的单位是**屈光度**（1/米），是"把镜组开到指定
位置"而不是"让它自己停"。前提是镜头支持手动对焦，即
`LENS_INFO_MINIMUM_FOCUS_DISTANCE > 0`（本机 id=0 是 7.69，支持）。

另外**不要**发 `AF_TRIGGER_CANCEL`，它同样会把镜组推回默认位置。

**兜底路径**（都不能省）：

| 情况 | 处理 |
|---|---|
| 读到的距离 ≤ 0（未知/无穷远） | **不要**切 `AF_MODE_OFF`——那正是会糊的那一步。保持连续对焦 |
| 一直等不到对焦完成（暗、无纹理、定焦镜头） | 2.5 秒超时后保持连续对焦开始采集 |

连续对焦的画面一直是清晰的，代价只是焦距会随距离轻微呼吸。**这比"锁在
一个错误位置、整幅图都是糊的"好得多。**

日志里会打印锁定结果：

```
对焦完成 AF_STATE=2，镜组锁定于 0.231 屈光度 (= 4.33 m)
```

**验收标准：整段 620 帧的拉普拉斯方差最小值 > 300。** 糊掉的帧只有 20-70，
差距是数量级的，不会误判。

> ⚠️ **锁定的代价**：对焦距离是**按下开始采集那一刻**的场景定的。所以
> 开始采集时就要把镜头对准你要扫的东西，之后别跑到比它近得多的地方
> （本机锁在 4.33m，大约 2m 以内就会出景深）。

**AE / AWB 先不要锁。** 完全锁死在室内会欠曝，先用 `TEMPLATE_RECORD` 让
AE 平滑跟随，整条链路跑通后再考虑。

**帧率要固定。** 从 `CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES` 里找
`[30, 30]`；找不到就选覆盖 30 的范围里下界最大的那个（下界越大越稳，
越不容易在光照变化时掉帧率）。

**分辨率不要改大。** 640×480 是 ORB-SLAM3 的典型工作分辨率。
1280×720 会让 ORB 特征提取成为瓶颈，手机上跑不到 30fps。

#### ⚠️ 多摄手机必须确认选对了镜头

`logCapabilities()` 会打印每个相机的 id、朝向、传感器尺寸、支持的 YUV
分辨率、FPS 范围，以及**焦距 / 传感器物理尺寸 / 水平视场角 / 最近对焦距离**。
**第一次跑必须看这段日志。**

后摄有多颗时（主摄 / 超广角 / 长焦），代码里"取第一个朝后的相机"这个
策略**不保证选到主摄**。选成超广角的话，内参和视场角全错，阶段四的标定
和阶段六的测距直接废掉。

**光看传感器尺寸是分不出来的**——多摄手机的各颗镜头常常共用同一个
传感器规格（本机 id=0 和 id=2 都是 4096×3072）。**要看水平视场角**：

| 镜头 | 典型水平视场角 |
|---|---|
| 主摄 | 65° – 80° |
| 超广角 | 100° – 120° |
| 长焦 | 20° – 40° |

视场角由 `2·atan(传感器宽度 / (2·焦距))` 算出，日志里直接打印。
确认之后把 `CAMERA_ID_OVERRIDE` 写死，不要依赖自动选择。

### 3.3 `ImuCollector.kt` —— IMU 采集

**职责**：采集加速度计和陀螺仪，配成 EuRoC 格式的一行。

**单位不需要任何转换**——Android 和 ORB-SLAM3 完全一致：

| 传感器 | Android 单位 | ORB-SLAM3 期望 | 转换 |
|---|---|---|---|
| `TYPE_ACCELEROMETER` | m/s² | m/s² | ❌ 不用 |
| `TYPE_GYROSCOPE` | rad/s | rad/s | ❌ 不用 |

**用已校准的版本**，不要用 `_UNCALIBRATED` 变体。系统已经做了 bias 和温度
补偿，对 ORB-SLAM3 足够；UNCALIBRATED 精度更高，但要自己处理 bias 和
坐标系，先用普通版。

**后台线程 handler**：`SENSOR_DELAY_FASTEST` 采样频率很高，注册在主线程的
handler 上会被主线程其他任务阻塞，时间戳的规律性会变差。

#### 坑 ③：Android 12+ 的高速采样权限

`SENSOR_DELAY_FASTEST` = **0 微秒**（不限速）。Android 12 起，超过 200Hz
需要 `HIGH_SAMPLING_RATE_SENSORS` 权限，否则 `registerListener()` 直接抛
`SecurityException`。

Manifest 里声明是正解。代码里还兜了一层——**拿不到权限就退到 200Hz 继续跑，
而不是让整个 App 挂掉**：

```kotlin
try {
    sm.registerListener(accelListener, accel, SensorManager.SENSOR_DELAY_FASTEST, handler)
    sm.registerListener(gyroListener, gyro, SensorManager.SENSOR_DELAY_FASTEST, handler)
} catch (e: SecurityException) {
    Log.w(TAG, "拿不到高速采样权限，退到 200Hz 运行", e)
    sm.registerListener(accelListener, accel, 5000, handler)  // 5000 微秒 = 200Hz
    sm.registerListener(gyroListener, gyro, 5000, handler)
}
```

#### 配对问题（影响精度，必须理解）

EuRoC 格式一行要同时有陀螺仪和加速度的 **6 个值**，但 Android 的两个传感器
是**独立回调**的，永远不会同时到达。所以只能拿一次加速度读数配当前的
陀螺仪，凑成一行。

**用"最近的"，不是"最新的"。** 代码里保留最近**两次**加速度读数，陀螺仪
事件到来时挑时间上更近的那个配。最新那次可能刚发生在很久以前，而它的
前一次反而离当前时刻更近——保留两次，最坏偏差直接减半。

#### 别用"最大值"衡量这件事

第一版只记了 `imuMaxPairGapNs`，得到 12.4ms，看着像有问题。加上分布统计
才发现**均值只有 0.65ms**，超过 5ms 的只有 **0.28%**，那个 12.4ms 是 6000+
次里的**一个尖峰**（传感器批次投递的边界效应）。

> **最大值在几千个样本上就是"最倒霉的那一次"，完全没有代表性。**
> 用它做验收标准会一直误报。

所以现在记三个量：

| 指标 | 本机实测 | 说明 |
|---|---|---|
| **均值** `imu_mean_pair_gap_ns` | **0.65 – 0.82 ms** | **主要验收指标** |
| 超 5ms 次数 `imu_gap_over_5ms` | 19 – 33 次 (≈0.3%) | 尖峰占比，应 < 1% |
| 最大值 `imu_max_pair_gap_ns` | 12 – 15 ms | 仅供参考 |

> **验收标准：均值 < 1.5 ms，且超 5ms 的比例 < 1%。**

为什么均值 0.65ms 是"对"的：加速度和陀螺仪都是 402Hz（周期 2.48ms），
用"最近一次"配对时偏差在 ±1.24ms 内均匀分布，理论均值正好是周期的
四分之一 ≈ **0.62 ms**。实测贴在这个值上，说明配对逻辑没有系统性问题。

### 3.4 `MainActivity.kt` —— 串起来

**职责**：权限申请、生命周期，把上面三个组件串起来。

#### 运行时权限不能省

清单里的 `<uses-permission>` 只是**声明**。Android 6.0 起，`CAMERA` 这类
dangerous 权限必须在代码里**动态申请**：

```kotlin
if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
    == PackageManager.PERMISSION_GRANTED) {
    startCapture()
} else {
    ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 1)
}
```

然后在 `onRequestPermissionsResult()` 里拿到结果，用户点了"允许"之后
才真正启动采集。

#### `@SuppressLint("MissingPermission")` 和动态申请是**分工，不是二选一**

`CameraCapture.start()` 里调 `manager.openCamera()`，那里有个
`@SuppressLint("MissingPermission")`。它俩的关系是：

| | 管什么 | 少了的后果 |
|---|---|---|
| `@SuppressLint` | 让 **Lint 静态检查**闭嘴 | **编译不过** |
| `checkSelfPermission` | 保证运行时真的有权限 | **运行时崩溃** |

原因：`MissingPermission` 在 Android Lint 里是 **error 级**，会直接让编译
失败。而它只静态扫描**当前函数体**，看不到调用方做了什么——所以
`openCamera()` 那里必须就地压掉。真正保证不崩的是调用方那句
`checkSelfPermission`。

#### 线程模型

```
主线程            只画界面、处理按钮
  ├─ camera 线程  Camera2 回调 + 图像提取
  ├─ imu 线程     SensorEventListener
  └─ writer 线程  写 PNG 和 CSV（SessionWriter 内部自己起）
```

一共 4 个线程。**采集和写盘绝不能在主线程上**。

#### `onStop()` 里主动停止采集

App 切到后台后，Android 会限制相机帧率，采到的帧率数据不可信。
**宁可停掉，也不要脏数据。**

---

## 4. 编译

### 4.1 生成 Gradle Wrapper（只做一次）

```bash
cd ~/SLAMAPP/android/SlamCapture

~/Android/gradle-8.7/bin/gradle wrapper --gradle-version 8.7
```

成功的话会多出这些文件：

```
gradlew          ← 以后都用这个
gradlew.bat
gradle/wrapper/gradle-wrapper.jar
gradle/wrapper/gradle-wrapper.properties
```

```bash
chmod +x gradlew
./gradlew --version
```

> **`./gradlew` 和 `gradle` 的区别**：`gradlew` 会读
> `gradle/wrapper/gradle-wrapper.properties` 里写的版本号，自动下载对应版本。
> 这样工程就自带了自己的构建工具版本，换台机器也不会因为 Gradle 版本不同而
> 编不过。团队协作和 CI 都用它。生成一次之后，`~/Android/gradle-8.7` 就可以
> 不管了。

#### 4.1.1 如果 `gradle wrapper` 失败

可能遇到的报错：

```
Execution failed for task ':wrapper'.
> Test of distribution url https://services.gradle.org/distributions/gradle-8.7-bin.zip failed.
```

**原因**：`wrapper` 任务会先对那个 URL 做一次严格的**可达性探测**，
而且**只认 HTTP 200**。真实链路是：

```
services.gradle.org   ——307——→   github.com/gradle/gradle-distributions/...
    （只是跳转站，不存文件）              （文件真在这儿）
```

`services.gradle.org` 自己返回的是 `307` 跳转，真正提供文件的是 **GitHub**。
**如果 GitHub 在你的网络下不通，探测就会超时，任务失败。**

这**不是 Gradle 的 bug，也不是环境配错了**。判断方法：

```bash
# 探测的最终地址，不通的话这条会超时
curl -4 -sI --max-time 15 -o /dev/null -w "%{http_code}\n" \
  https://github.com/gradle/gradle-distributions/releases/download/v8.7.0/gradle-8.7-bin.zip
# 000 = 不通
```

> **为什么大多数人碰不到**：用 Android Studio 新建工程时，`gradlew` /
> `gradle-wrapper.jar` / `gradle-wrapper.properties` 是 IDE 自动生成的，
> 这条 URL 探测根本不会执行。只有纯命令行手动跑 `gradle wrapper` 才会遇到。

#### 解法一（推荐）：用国内镜像生成真 wrapper

把分发地址换成国内镜像，一步绕开 GitHub：

```bash
cd ~/SLAMAPP/android/SlamCapture

~/Android/gradle-8.7/bin/gradle wrapper \
  --gradle-version 8.7 \
  --gradle-distribution-url "https://mirrors.cloud.tencent.com/gradle/gradle-8.7-bin.zip"
```

验证镜像上的文件和官方一致（可选，但建议做一次）：

```bash
# 官方校验和
curl -4 -sL "https://services.gradle.org/distributions/gradle-8.7-bin.zip.sha256"
# 本地文件的校验和
sha256sum ~/Android/downloads/gradle-8.7-bin.zip
# 两行应该完全相同
```

> 官方和腾讯镜像上的 `gradle-8.7-bin.zip` 都是 134184980 字节，
> SHA-256 是 `544c35d6bd849ae8a5ed0bcea39ba677dc40f49df7d1835561582da2009b961d`。

生成成功后 `gradle/wrapper/gradle-wrapper.properties` 里的
`distributionUrl` 会指向腾讯镜像，以后 `./gradlew` 就一直从那儿拉，**永久绕开
GitHub**。

**备选镜像**（腾讯不通时换）：

- `https://mirrors.huaweicloud.com/gradle/gradle-8.7-bin.zip`

#### 解法二（保底）：用转发脚本，完全不联网

如果镜像也不通，那就别用 wrapper 了，直接在工程根目录建一个转发脚本：

```bash
cd ~/SLAMAPP/android/SlamCapture

cat > gradlew << 'EOF'
#!/bin/sh
exec "$HOME/Android/gradle-8.7/bin/gradle" "$@"
EOF

chmod +x gradlew
./gradlew --version
```

效果和真 wrapper 等价：**Gradle 版本被钉死在 8.7，命令依然是
`./gradlew assembleDebug`**，本文档后面所有命令都不用改。

> ⚠️ **不要用 `ln -s` 软链接**：`~/Android/gradle-8.7/bin/gradle` 这个启动脚本
> **不跟随符号链接**，它靠 `$0` 的目录去找自己的
> `lib/gradle-launcher-8.7.jar`。软链接会让 `$0` 变成 `./gradlew`，它就会去
> 工程目录里找 `lib/`，直接崩。用 `exec` 转发绝对路径才安全。

**代价**：工程不自带 Gradle 版本信息，换机器要手动改脚本里的路径。对单人自用
项目无所谓。

#### 好消息：编译本身不受影响

`assemblyDebug` 真正要访问的三个 Maven 仓库和 GitHub 无关，实测都很快：

| 仓库 | 状态 | 耗时 |
|---|---|---|
| `dl.google.com`（`google()`） | ✅ 200 | 0.58s |
| `repo1.maven.org`（`mavenCentral()`） | ✅ 200 | 0.37s |
| `plugins.gradle.org`（`gradlePluginPortal()`） | ✅ 200 | 0.39s |

**所以只有 wrapper 这一步受影响。** 这里如果也全部不通，才需要按 §4.3
配置阿里云镜像。

### 4.2 编译 Debug APK

```bash
cd ~/SLAMAPP/android/SlamCapture
./gradlew assembleDebug
```

**第一次会很久（5–15 分钟）**，因为要下载 AGP、Kotlin 编译器、AndroidX 等依赖。
后续增量编译在 10 秒内。

> 这期间会往 `~/.gradle` 写几百 MB 缓存。别中断。

成功的话输出：

```
BUILD SUCCESSFUL in 6m 12s
```

APK 在：

```
app/build/outputs/apk/debug/app-debug.apk
```

验证一下：

```bash
ls -lh app/build/outputs/apk/debug/app-debug.apk
```

### 4.3 如果编译失败

| 报错 | 原因 | 解法 |
|---|---|---|
| `SDK location not found` | `local.properties` 路径不对 | 检查里面的 `sdk.dir` 是不是 `/home/welkin/Android/Sdk` |
| `Unsupported class file major version` | Gradle 用错了 JDK | `echo $JAVA_HOME` 应该是 `~/Android/jdk-17` |
| `Could not resolve com.android.tools.build:gradle` | 网络问题 | 重试。国内网络可能需要配置镜像 |
| `Failed to install the following SDK components` | 少了 SDK 包 | 补跑 §1.5 的 `sdkmanager` |
| 卡在 `Downloading` 很久 | 正常，首次要下 300MB+ | 等 |

#### 如果 Maven 依赖拉不动（国内网络）

实测 `dl.google.com` / `repo1.maven.org` / `plugins.gradle.org` 三个仓库在
国内一般直连可用（都在 1 秒内）。但万一遇到 `Could not resolve`，
把阿里云镜像插到最前面即可。

改动 `settings.gradle.kts` 的两个 `repositories` 块：

```bash
cd ~/SLAMAPP/android/SlamCapture

cat > settings.gradle.kts << 'EOF'
pluginManagement {
    repositories {
        maven { url = uri("https://maven.aliyun.com/repository/gradle-plugin") }
        maven { url = uri("https://maven.aliyun.com/repository/google") }
        maven { url = uri("https://maven.aliyun.com/repository/public") }
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        maven { url = uri("https://maven.aliyun.com/repository/google") }
        maven { url = uri("https://maven.aliyun.com/repository/public") }
        google()
        mavenCentral()
    }
}
rootProject.name = "SlamCapture"
include(":app")
EOF
```

> **镜像放在前面，官方仓库保留在后面**：这样镜像命中就直接用，镜像上没有的
> artifact 会自动回落到官方源，不会因为镜像不全而编不过。
>
> 改完记得重新编译。已经下过的依赖会走 `~/.gradle` 缓存，不会重下。

---

## 5. 连手机（无线调试）

你的手机是 Android 11+，支持**无线调试**。这条路不需要数据线、不需要
管理员权限、不需要 USB 透传，是最省事的方案。

### 5.1 手机上开启开发者模式

1. **设置 → 关于手机 → 版本号**，**连续点 7 次**
2. 系统提示"您现在处于开发者模式"

> 有些机型（小米、OPPO）路径不同：小米是「设置 → 我的设备 → 全部参数 →
> 连续点 MIUI 版本 7 次」；OPPO/vivo 一般在「设置 → 关于手机 → 版本信息」。

### 5.2 开启无线调试

1. **设置 → 系统 → 开发者选项 → 无线调试**（打开开关）
2. 点进「无线调试」→ **使用配对码配对设备**
3. 屏幕上会显示：

```
Wi-Fi 配对码
12345678

IP 地址和端口
192.168.1.50:41234        ← 配对端口
```

4. **先别关这个弹窗**，回到 WSL2 执行配对

> 注意：「配对端口」和「调试端口」是**两个不同的端口**。配对弹窗显示的是
> 配对端口；无线调试主界面上显示的是调试端口。别搞混。

### 5.3 从 WSL2 配对

```bash
# 换成你手机上显示的 IP 和配对端口
adb pair 192.168.1.50:41234
# 会提示 Enter pairing code:，输入屏幕上那个 8 位码
```

成功后：

```bash
adb devices
```

应该是空的（配对 ≠ 连接）。

### 5.4 连接

回到手机上的「无线调试」主界面，看 **IP 地址和端口**（这次是调试端口）：

```bash
adb connect 192.168.1.50:37000

# 验证
adb devices
```

应该看到：

```
List of devices attached
192.168.1.50:37000	device
```

状态必须是 `device`。如果是 `unauthorized`，看手机屏幕有没有授权弹窗；
如果是 `offline`，`adb disconnect` 再重连。

> **Wi-Fi 换了或者手机重启后要重新配对吗？** 不用重新配对，但 IP 变了要重新
> `adb connect`。手机重启后无线调试会关掉，需要重新打开。

### 5.5 如果 WSL2 连不上手机（NAT 模式问题）

WSL2 默认是 NAT 网络，正常情况下能访问局域网。如果 `adb connect` 卡住：

**方案 A：改用镜像网络模式**（推荐，一劳永逸）

在 Windows 上创建/编辑 `C:\Users\<你的用户名>\.wslconfig`：

```ini
[wsl2]
networkingMode=mirrored
```

然后 Windows PowerShell 里 `wsl --shutdown`，重开 WSL2。

> 镜像模式下 WSL2 和 Windows 共用网络栈，局域网设备直接可达。

**方案 B：走 USB**

需要 `usbipd-win`，在 Windows 管理员 PowerShell 里操作，步骤多一些。
只有无线方案彻底不通时才考虑。

---

## 6. 装上手机并验证

### 6.1 安装

```bash
cd ~/SLAMAPP/android/SlamCapture
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

看到 `Success` 就成了。手机上会出现一个叫 **SlamCapture** 的图标。

### 6.2 看实时日志

**开一个单独的终端窗口**，跑：

```bash
adb logcat -c                                    # 先清空
adb logcat -s CameraCapture:V ImuCollector:V MainActivity:V SessionWriter:V
```

> `-s TAG:V` 表示只显示这几个 TAG 的日志。不加过滤的话 logcat 会被系统日志淹没。

然后在手机上：

1. 打开 SlamCapture
2. 点「开始采集」，同意相机权限
3. **拿着手机慢慢走动 30–60 秒**（要走动，静止的数据没有价值——IMU 需要
   运动激励才能标定和初始化）
4. 点「停止采集」

### 6.3 验证清单

**① 看设备能力日志**（`CameraCapture` TAG）

```
── 相机 id=0 facing=1 sensor=4096x3072
   YUV尺寸: 4096x3072,4000x3000,3840x2160,...,640x480,640x360,...
   支持640x480: true
   FPS范围: [10, 10],[10, 15],[15, 15],[10, 24],[24, 24],[10, 30],[30, 30]
── 相机 id=1 facing=0 sensor=3280x2464
...
后置相机数量 = 3
使用相机 id=0
```

要确认：

- ✅ `支持640x480: true` —— 不支持的话要改分辨率
- ✅ `使用相机 id=0` 是你想要的**主摄**（`facing=1` 是后置，`facing=0` 是前置）

> ⚠️ **有 3 个后摄时，`id=0` 不保证是主摄。**
> 上面这个例子里 id=0 和 id=2 的传感器尺寸**完全相同**（都是 4096×3072），
> 光看分辨率分不出谁是主摄、谁是超广角。
>
> **选错镜头的后果**：超广角和主摄的内参、畸变系数完全不同，
> 阶段四标定和阶段六测距**直接废掉**。
>
> **怎么确认**：把采到的某张 PNG 打开看——主摄是正常视角，
> 超广角明显更宽、边缘有桶形畸变。
> 如果 `id=0` 是超广角，把主摄的 id 填到
> `CameraCapture.CAMERA_ID_OVERRIDE`。

**② 看 IMU 日志**（`ImuCollector` TAG）

```
加速度计: bmi26x Accelerometer Non-wakeup  最大量程 156.9064 m/s²
陀螺仪  : bmi26x Gyroscope Non-wakeup  最大量程 34.90656 rad/s
```

量程够用（加速度 156 m/s² ≈ 16g，陀螺仪 34.9 rad/s ≈ 2000°/s）。

> 名字里带 **`Non-wakeup`** 表示这类传感器在设备休眠时会被挂起。
> 这类机型上 **「保持屏幕常亮」是硬性要求**，不是建议（见 §7）。

**③ 看界面上的统计数字**

停止后屏幕上会显示：

```
已运行   : 22.3 s
相机     : 620 帧  掉 0  丢帧率 0.00%
编码失败 : 0 帧
实际帧率 : 30.0 fps  (目标 30)
IMU      : 7660 条  丢 0
IMU 频率 : 402.0 Hz
配对偏差 : 均值 0.82 ms   最大 12.42 ms   超5ms 33 次
```

**验收标准：**

| 指标 | 合格 | 不合格怎么办 |
|---|---|---|
| 丢帧率 | **< 1%** | 队列调大；或换更快的存储；或检查有没有别的 App 占相机 |
| **编码失败** | **必须 = 0** | 不为 0 说明磁盘上有空文件，见 §3.1 坑 ② |
| 实际帧率 | **29–30 fps** | 检查 `fpsRange` 日志，可能设备不支持固定 30 |
| IMU 频率 | **100–500 Hz** | 太低说明 `SENSOR_DELAY_FASTEST` 被限流 |
| 配对偏差 **均值** | **< 1.5 ms** | 这才是主指标，见 §3.3 |
| 配对偏差 超5ms 占比 | **< 1%** | 占比高才说明某个传感器慢；单看最大值会误判 |

**还要看画面锐度**（这一项界面不显示，必须离线算，见 §6.7）。

### 6.4 取回数据

```bash
cd ~/SLAMAPP
mkdir -p dataset/selfrun

adb pull /sdcard/Android/data/com.example.slamcapture/files/sessions/ \
    dataset/selfrun/
```

> **`adb pull` 私有目录不需要 root**，这是我们从一开始就选
> `getExternalFilesDir()` 的原因之一。

### 6.5 检查 PNG 是否正常（重要！）

这一项**最容易漏看**，因为它的失败是静默的。

```bash
cd ~/SLAMAPP/dataset/selfrun/sessions/session_*/

ls -la cam0/data/ | head -5
file cam0/data/$(ls cam0/data/ | head -1)
```

**要看到两件事：**

```
-rw-r--r-- 1 ... 153600 1758...png          ← 大小不能是 0
1758...png: PNG image data, 640 x 480, 8-bit grayscale, non-interlaced
```

| 检查项 | 合格 |
|---|---|
| 文件大小 | **不能是 0** |
| `file` 输出 | `PNG image data, 640 x 480, 8-bit grayscale` |
| 界面上的 `编码失败` | **必须是 0** |

**必须是 `8-bit grayscale`。** 如果报 `8-bit/color RGBA`，说明这是早期版本
（走 `Bitmap`/`ARGB_8888`）留下的数据——那种数据本身没问题，但采的时候
丢帧率高达 65%。重采一份。

**如果文件全是 0 字节**，说明 PNG 编码失败，原因和修法见 §3.1 的「坑 ②」。


### 6.6 用 Python 快速体检

```bash
cd ~/SLAMAPP/dataset/selfrun/sessions/session_*/

python3 - << 'EOF'
import csv, os, sys

# ---- 相机 ----
rows = [r for r in csv.reader(open('cam0/data.csv')) if r and not r[0].startswith('#')]
ts = [int(r[0]) for r in rows]
print(f"相机帧数: {len(ts)}")
print(f"时长    : {(ts[-1]-ts[0])/1e9:.2f} s")
print(f"平均帧率: {len(ts)/((ts[-1]-ts[0])/1e9):.2f} fps")

dt = [(ts[i+1]-ts[i])/1e6 for i in range(len(ts)-1)]
dt_sorted = sorted(dt)
print(f"帧间隔  中位数 {dt_sorted[len(dt_sorted)//2]:.2f} ms  "
      f"最大 {max(dt):.2f} ms  最小 {min(dt):.2f} ms")
print(f"时间戳单调递增: {all(ts[i+1] > ts[i] for i in range(len(ts)-1))}")

# 掉帧：间隔 > 1.5 倍理论值的算掉
theory = 1000/30
n_gap = sum(1 for d in dt if d > theory*1.5)
print(f"明显卡顿(>1.5倍帧间隔): {n_gap} 次")

missing = [r[1] for r in rows if not os.path.exists('cam0/data/'+r[1])]
print(f"CSV 里有但文件不存在: {len(missing)}")

# ---- IMU ----
irows = [r for r in csv.reader(open('imu0/data.csv')) if r and not r[0].startswith('#')]
its = [int(r[0]) for r in irows]
print()
print(f"IMU 条数: {len(its)}")
print(f"IMU 频率: {len(its)/((its[-1]-its[0])/1e9):.2f} Hz")
print(f"IMU 时间戳单调递增: {all(its[i+1] > its[i] for i in range(len(its)-1))}")

# ---- 时钟对齐（最关键的一项）----
print()
print("=== 相机与 IMU 时间戳重合度 ===")
print(f"相机首帧 - IMU 首样本 = {(ts[0]-its[0])/1e6:.1f} ms")
print(f"相机末帧 - IMU 末样本 = {(ts[-1]-its[-1])/1e6:.1f} ms")
EOF
```

**这一段的验收标准：**

| 指标 | 合格 |
|---|---|
| 平均帧率 | 29–30 fps |
| 时间戳单调递增 | `True` |
| CSV 里有但文件不存在 | `0` |
| IMU 时间戳单调递增 | `True` |
| 相机↔IMU 首末时刻偏差 | **都在 100ms 以内**（说明两条流是同时启停的） |

最后一项如果差了几百毫秒甚至几秒，说明某一路启动慢了——这会让 ORB-SLAM3
的 IMU 初始化失败。去查 `onOpened` 和 `registerListener` 的日志时间差。

### 6.7 画面锐度体检（会抓出对焦 bug）

**这一项界面不显示，只能离线算，但它是唯一能抓住对焦问题的检查。**

用**拉普拉斯方差**：对灰度图做 3×3 拉普拉斯卷积，取结果的方差。图越锐，
高频越多，方差越大。

```bash
cd ~/SLAMAPP/dataset/selfrun/sessions/session_*
~/software/miniconda3/envs/DeepLearning/bin/python3 - << 'EOF'
import numpy as np, glob
from PIL import Image

def sharp(p):
    im = np.asarray(Image.open(p).convert('L'), dtype=np.float64)
    lap = (-4*im[1:-1,1:-1] + im[:-2,1:-1] + im[2:,1:-1]
           + im[1:-1,:-2] + im[1:-1,2:])
    return lap.var()

files = sorted(glob.glob("cam0/data/*.png"))
v = np.array([sharp(f) for f in files])
print(f"共 {len(files)} 帧")
print(f"中位 {np.median(v):.1f}  最小 {v.min():.1f}  最大 {v.max():.1f}")
print(f"低于 300 的帧: {(v < 300).sum()} / {len(v)}")

for i in range(0, len(v), 20):
    seg = v[i:i+20]
    print(f"  帧 {i+1:3d}-{min(i+20,len(v)):3d}  {seg.mean():7.1f}  "
          + '#' * int(min(seg.mean(), 2000) / 40))
EOF
```

**验收标准：最小方差 > 300，且没有整段塌到几十的区间。**

本机实测基线：

| 状态 | 方差 |
|---|---|
| 正常清晰 | **320 – 2000**（随场景纹理多少浮动） |
| 失焦（镜组停错位置） | **20 – 70** |

差了一个数量级，不会误判。

> ⚠️ **必须看整段曲线，不能只抽查开头。** 对焦 bug 的失败是延迟的：切换
> `AF_MODE_OFF` 之后前 7 帧（约 230ms）镜组还在路上，仍然是清晰的，第 8 帧
> 才开始糊。只看开头会得出"一切正常"的错误结论。详见 §3.2 坑 ③。

场景本身也会影响方差（大片平坦墙面天然就低）。所以**判据是"有没有整段
塌到几十"，不是"每一帧都要上千"**。

---

## 7. 采集时的注意事项

这几条不做的话，采到的数据是废的，但你事后才发现。

| 事项 | 原因 |
|---|---|
| **必须走动** | IMU 需要运动激励（平移 + 旋转）才能恢复尺度和重力方向。拿着手机站着不动，ORB-SLAM3 永远初始化不了 |
| **走动要慢且平滑** | 快速甩动会触发运动模糊，特征点全丢 |
| **不要转圈** | 纯旋转没有平移，提供不了尺度信息 |
| **场景要有纹理** | 对着白墙 ORB 提不出特征。要对着有家具、书籍、海报的环境 |
| **光照要均匀** | 对着窗户逆光会过曝，暗角落提不出特征 |
| **关闭电池优化** | 设置 → 应用 → SlamCapture → 电池 → 不受限制。否则系统会限流 |
| **保持屏幕常亮** | 屏幕灭掉后系统可能限制相机 |
| **不要使用变焦** | 数码变焦改变有效内参 |
| **一镜到底** | 不要中途暂停。断了就重新采一段 |

**推荐的第一次采集动作：**

> 手机横屏（或竖屏，看你锁定的方向），端稳，以正常步速沿着房间走一圈，
> 走的过程中缓慢左右转头看四周，全程约 60 秒。

---

## 8. 常见故障

### 编译相关

| 现象 | 原因 | 解法 |
|---|---|---|
| `./gradlew: Permission denied` | 没加执行权限 | `chmod +x gradlew` |
| `Could not find method namespace()` | 根 build.gradle 的 AGP 版本没生效 | 检查根 `build.gradle.kts` 是不是写对了 |
| 编译到 90% 卡住很久 | 正常，在跑 dex | 等 |

### 运行相关

| 现象 | 原因 | 解法 |
|---|---|---|
| 点开始后黑屏 / 无日志 | 相机权限被拒 | 设置里手动开 |
| `会话配置失败` | 设备不支持 640×480 的 YUV 输出 | 看 `YUV尺寸` 日志，换一个支持的分辨率（比如 640×360） |
| `相机打开失败 error=1` | 相机被别的 App 占用 | 关掉相机/微信/扫码类 App |
| 点开始后立刻退出 | 崩溃 | 见下面「如何看崩溃堆栈」 |
| 崩溃信息里有 `HIGH_SAMPLING_RATE_SENSORS` | Manifest 少了那个权限 | 见 §2.3，Android 12+ 必须声明 |
| 帧率只有 15fps | 光照不足，AE 自动降帧 | 补光。这是手机相机的常见行为 |
| 丢帧率高 | PNG 编码跟不上 | 见 §3.1 坑 ①。确认走的是 `GrayPng`，不是 `Bitmap` |
| **照片整幅均匀地糊** | 镜组被停在了错误位置 | 见 §3.2 坑 ③。日志要出现 `镜组锁定于 N 屈光度`。**注意看整段，别看开头 7 帧**，用 §6.7 算方差 |
| **PNG 文件全是 0 字节** | 编码静默失败 | 见 §3.1 坑 ② |
| IMU 频率只有 50Hz | 部分机型 `FASTEST` 被限流 | 正常范围，能接受；低于 50Hz 就有问题 |
| 切后台后数据断了 | `onStop()` 主动停了 | 设计如此，见 §3.4 的说明 |

### 如何看崩溃堆栈（重要）

**§6.2 那个 logcat 过滤器会把崩溃信息过滤掉。**

`adb logcat -s CameraCapture:V ImuCollector:V ...` 里的 `-s` 是「白名单」模式：
**除了列出来的 TAG，其它全部静音**。而崩溃堆栈是 `AndroidRuntime` 这个 TAG
打出来的——不在白名单里，所以你看得见 App 死了，看不见它为什么死。

正确的看法是分开抓 crash 缓冲区：

```bash
# 1. 清空，确保拿到的是这次的
adb logcat -b crash -c

# 2. 在手机上复现一次崩溃

# 3. 把堆栈 dump 出来
adb logcat -b crash -d
```

如果第 3 步是空的，用这个兜底：

```bash
adb logcat -d -s AndroidRuntime:E libc:F DEBUG:F
```

读堆栈的方法：**从下往上找第一个 `com.example.slamcapture` 的行**，那就是
你代码里出事的位置，后面括号里的 `MainActivity.kt:97` 就是行号。上面那些
`android.hardware.*` / `android.view.*` 是系统内部调用，不用管。

一个真实例子：

```
java.lang.SecurityException: To use the sampling rate of 0 microseconds,
app needs to declare the normal permission HIGH_SAMPLING_RATE_SENSORS.
    at android.hardware.SystemSensorManager$BaseEventQueue.enableSensor(...)   ← 系统
    at com.example.slamcapture.ImuCollector.start(ImuCollector.kt:83)          ← 你的代码，就是这行
    at com.example.slamcapture.MainActivity.startCapture(MainActivity.kt:97)
```

### 连接相关

| 现象 | 原因 | 解法 |
|---|---|---|
| `adb devices` 里是 `unauthorized` | 手机上的授权弹窗没点 | 看手机屏幕，点允许 |
| `adb connect` 卡住 | WSL2 NAT 到不了局域网 | 见 §5.5 |
| `adb devices` 里是 `offline` | 连接失效 | `adb disconnect` 后重连 |
| 手机重启后连不上 | 无线调试重置了 | 重新打开无线调试，换端口后 `adb connect` |

---

## 9. 完成后你应该有的东西

```
~/SLAMAPP/dataset/selfrun/sessions/session_<时间戳>/
├── cam0/
│   ├── data/           ← N 张 640×480 灰度 PNG
│   └── data.csv
├── imu0/
│   └── data.csv
└── meta.json
```

**并且能回答 §0 里那三个问题。** 能答上来，阶段二就算过了。

---

## 10. 下一步

阶段二结束后**先别急着标定**。按教程的顺序，下一步是 **阶段三：PC 验证**：

拿这份刚采的数据，喂给本机的 `mono_inertial_euroc` 跑一遍。

```bash
cd ~/SLAMAPP/ORB_SLAM3
./Examples/Monocular-Inertial/mono_inertial_euroc \
    ./Vocabulary/ORBvoc.txt \
    ./Examples/Monocular-Inertial/EuRoC.yaml \
    ~/SLAMAPP/dataset/selfrun/sessions/session_<时间戳> \
    <时间戳列表.txt> \
    selfrun
```

**注意**：这里用的是 EuRoC 的配置文件，内参和畸变都是**错的**（那是
EuRoC 相机的参数，不是你的手机）。所以这次跑的目的**不是要跑出好结果**，
而是回答两个问题：

1. 数据能不能被正确读入？（帧数对不对、时间戳解析对不对）
2. ORB 特征提取得出来吗？（画面够不够清晰）

**跑不出轨迹是正常的**，因为内参不对。这恰恰是阶段四标定要解决的问题——
而这次运行的意义在于，把"采集问题"和"标定问题"分开了：

- 如果**读入就报错**，或**特征点少得可怜** → 回去改采集层
- 如果**读入正常、特征点充足，只是轨迹飘** → 恭喜，采集层没问题，
  直接进阶段四

这就是教程里说"这一步不能跳过"的原因。

需要我把阶段三的步骤也写成文档的时候说一声。
