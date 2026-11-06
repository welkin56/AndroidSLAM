# SlamCapture

Android 上的单目惯性 SLAM 实时测距 App。

`Kotlin + CameraX + JNI + ORB-SLAM3 + OpenCV`，单目 + IMU，ARM64，Android 8.0+。

---

## 能做什么

| 功能 | 说明 |
|---|---|
| **实时测距** | 画面中心十字锁住一个地图点，直接读出相机到它的距离；同时给出世界坐标 |
| **钉点成链** | 把点钉住，测两点间的实际长度。同一条链共用一个锚点关键帧，段长是纯刚体量，不受位姿抖动影响 |
| **相机内参标定** | 9 步引导（5 正对 + 4 倾斜 30°），按对焦屈光度存成档位表，运行时插值注入 |
| **IMU 噪声标定** | Allan 方差，产出 `phone.yaml` 里的噪声参数 |
| **采集与复现** | 相机 + IMU 双流落盘，零丢帧；同一份数据可在 PC 上用 ORB-SLAM3 重跑对照 |

## 现状

**这不是一个完成品，是一个能用的实验平台。** 下面的数字都来自真机日志，不是目标值。

| 项 | 状态 |
|---|---|
| 单目惯性跟踪（正常视野） | 可用，地图尺度多次收敛到 ±2% |
| 实时测距 | 可用。同一次钉点内读数不动（50 秒零变化）；重复钉点组间差 3~7%，绝对值随图尺度浮动，见限制 1 |
| 相机内参标定 | 可用。同档连标 4 次，fx 相对标准差 0.73% |
| 长链测距（> 2 m） | 可用，但未做滑动重锚 |
| 长时间运行 | **不稳定**，见下 |
| 视野剧变后的恢复 | **不可用**，见下 |

### 已知限制

1. **系统尺度误差 −6.5%~+0.9%，随图浮动**。单目初始化的尺度每次随机，所以绝对值
   跨图不可比，同一张图内稳定。这是当前最大的误差项，需要「已知长度校准」一次定死。
2. **快速转视野会丢跟踪，5 秒后重建地图**。两级视觉恢复做过完整实现并真机验证，
   结果会触发「恢复一帧 → 下一帧又丢 → 清图」的连锁，比不改更差，已存档不合并
   （分支 `exp/lost-recovery`）。
3. **平面深度只做兜底，不做修正**。用局部平面改地图点深度的修正链实测段长极差从
   0.5% 恶化到 8~23%，已删除（分支 `exp/surface-depth` 存有完整实现）。十字没锁到
   地图点时仍会用平面估一个深度（标 `≈`），但这类读数**不能钉点** —— 它没有地图点
   身份，钉下去等于把估算误差带进段长。
4. **未做已知长度校准**。需要一个固定尺寸的基准物，属于产品决定 —— 这是限制 1 的解法。
5. **只在单机型上验证**（arm64-v8a）。内置的内参与 IMU 噪声是开发者这台机器的值，
   换机型不重标也能跑，但测距读数不可信 —— 需要重走一次标定（内参 + IMU 噪声）。
6. **词袋 `ORBvoc.txt` 145 MB 不进源码仓库**。debug 包要单独推送到设备（见构建步骤）；
   release 包内置，首次启动自动解压到应用私有目录。

## 技术栈

| 层 | 内容 |
|---|---|
| App | Kotlin，CameraX（`ImageAnalysis` 灰度流），`minSdk 26` |
| 桥接 | JNI，native 侧单线程 worker 串行消费帧，日志重定向到 logcat |
| SLAM | ORB-SLAM3，`IMU_MONOCULAR` 模式（`thirdparty/orb_slam3`，已针对 Android 裁剪） |
| 视觉 | OpenCV 4.x（`core` / `imgproc` / `features2d` / `calib3d`） |
| 优化 | g2o（惯性 BA） |

送入 SLAM 的图像固定 640×480。内参由 App 的标定档位按当前对焦
屈光度插值后经 JNI 注入 —— 详见 `docs/changes/2026-10-28-intrinsics-profile.md`。

## 构建

### 前置

- JDK 17、Android SDK 34、**NDK 26.1.10909125**
- `cmake` 3.22.1（Android SDK 自带）

两个依赖体积大，不进版本库，需自行准备：

```bash
# 1) OpenCV Android SDK 4.x，解压到下面这个路径
#    app/src/main/cpp/thirdparty/OpenCV-android-sdk/

# 2) Boost.Serialization（ORB-SLAM3 无条件链它，Boost 没有官方安卓包）
cd <repo> && ./tools/setup-boost-android.sh
```

### 编译与安装

```bash
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

### 词袋

`ORBvoc.txt`（139 MB）不进 git。先从 ORB-SLAM3 官方仓库的 `Vocabulary/` 取到，
然后用下面两种方式之一：

**debug 包：推到设备**

```bash
./tools/push-vocab.sh
```

**release 包：打进 APK**

```bash
./tools/prepare-release-vocab.sh   # 复制到 app/src/release/assets/（不进 git）
```

打出的 release 包首次启动会把词袋解压到 `files/` 下（约 5 秒，只做一次），
之后启动不再需要这一步。四处都没有时会明确提示缺什么。

## 目录结构

```
app/src/main/cpp/
  slam_engine.cpp      native 引擎：线程、帧队列、钉点、段长
  measure_center.cpp   测距核心：候选筛选、深度锁定、跳变判据、平面兜底
  orb_track.cpp        ORB-SLAM3 封装与位姿导出
  calib.cpp            内参标定：probe / commit / finish
  jni_bridge.cpp       JNI 入口
  thirdparty/          ORB-SLAM3 / DBoW2 / Sophus / g2o（含 fork 改动清单）

app/src/main/java/com/example/slamcapture/
  MainActivity.kt      主界面与状态机
  CalibActivity.kt     标定流程
  CalibGuide.kt        9 步引导的几何判据
  PreviewView.kt       预览与叠加层

docs/
  tutorial/            教程与设计说明（13 篇，讲「现在是怎么搭的」）
  changes/             改动记录（60+ 篇，讲「某天动了什么、为什么」）
  bugs/                踩过的坑（讲「这类问题长什么样、怎么查」）

tools/
  calibration/         标定与验证脚本（Python）
  setup-boost-android.sh
  push-vocab.sh        debug：把词袋推到设备
  prepare-release-vocab.sh  release：把词袋复制进 assets
```

## 文档

- **[docs/tutorial/ANDROID_SLAM_TUTORIAL.md](docs/tutorial/ANDROID_SLAM_TUTORIAL.md)** —— 主教程：
  七个阶段的目标、验收判据、排查手册
- **[docs/README.md](docs/README.md)** —— 三类文档的分工与索引
- **[app/src/main/cpp/thirdparty/README.md](app/src/main/cpp/thirdparty/README.md)** ——
  ORB-SLAM3 的 fork 改动清单（升级上游时要重放的改动），以及踩到的上游 bug
- **[docs/changes/](docs/changes/)** —— 每个决定的依据与真机实测数据，**包括失败和回滚**

## 许可

本项目**整体以 GNU GPL v3 发布**，全文见 [LICENSE](LICENSE)。

第三方组件与各自许可：

| 组件 | 许可 | 位置 |
|---|---|---|
| ORB-SLAM3 | GPLv3 | `app/src/main/cpp/thirdparty/orb_slam3/` |
| DBoW2 | GPLv3 | `app/src/main/cpp/thirdparty/DBoW2/` |
| Sophus | MIT | `app/src/main/cpp/thirdparty/Sophus/` |
| g2o | BSD | `app/src/main/cpp/thirdparty/g2o/` |
| OpenCV | Apache 2.0 | 需自行下载，不在此仓库 |
| Boost | Boost Software License | 需自行编译，不在此仓库 |

ORB-SLAM3 版权归 Universidad de Zaragoza 的 ORB-SLAM3 团队所有；本仓库对其的修改
是对 Android 平台的移植与适配，逐条记录在 `thirdparty/README.md` 里。
