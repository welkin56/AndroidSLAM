# thirdparty —— 这个目录里为什么有这些东西

`app/src/main/cpp/` 下是我们自己写的 native 代码（JNI 桥、日志重定向、md5，
以后还有 `SlamEngine`）。这个子目录只放**别人写的**东西。

## 分法：什么自带，什么外装

判据借的是 ORB-SLAM3 自己的那套（见 `ORB_SLAM3/Thirdparty/` 的构成）：

| | 自带，进这个目录 | 外装，靠系统装或自己编 |
|---|---|---|
| 作者改过（是 fork） | ✅ | |
| 版本敏感、上游 API 易变 | ✅ | |
| 和项目紧耦合 | ✅ | |
| 体积小、编译快 | ✅ | |
| 通用、成熟、API 稳定 | | ✅ |
| 体积大、编译慢 | | ✅ |
| 系统包 / 官方发布物维护得好 | | ✅ |

## 自带的（进 git）

| 目录 | 是什么 | 版本 / 来源 |
|---|---|---|
| `orb_slam3/` | ORB-SLAM3 的 `src/` + `include/` | 从 `SLAMAPP/ORB_SLAM3`（本地仓库 `1e379cf`）复制 |
| `DBoW2/` | 词袋模型（回环检测用） | 同上，`Thirdparty/DBoW2` |
| `g2o/` | 图优化后端 | 同上，`Thirdparty/g2o` |
| `Sophus/` | 李群/李代数头文件库（只要头） | 同上，`Thirdparty/Sophus` |

后三个是 **ORB-SLAM3 作者自己 fork 过的**，不是上游原版，版本必须跟着走 ——
这就是它们自带、而 Eigen 不自带的区别。

## 外装的（不进 git）

| 库 | 从哪来 | 为什么不自带 |
|---|---|---|
| **OpenCV** | `/home/welkin/SLAMAPP/OpenCV-android-sdk`（官方 Android SDK，872 MB） | 通用、成熟、官方有预编译发布物。换机器用 `-DOPENCV_ANDROID_SDK=...` 指过去 |
| **Eigen** | `apt install libeigen3-dev` → `/usr/include/eigen3` | 纯头文件、API 稳定、和 ORB-SLAM3 在 PC 上的用法一致 |
| **Boost** | `./tools/setup-boost-android.sh` | 没有官方安卓发布物，得自己交叉编译；而且它是构建产物，绑死 NDK 版本。跑一次脚本，装到 `boost-android/`（已 gitignore） |

## 为什么 ORB-SLAM3 是复制的，不直接引用 `SLAMAPP/ORB_SLAM3/`

这样 `android/SlamCapture` 这个仓库自己能构建，不依赖仓库外的任何路径 ——
clone 下来就能编。`SLAMAPP/ORB_SLAM3/` 那边保持上游原样，不再改动。

代价是两份源码会分叉。**规则：以后只改这里这份**，`SLAMAPP/ORB_SLAM3/` 只当上游参照。
PC 上跑 `mono_inertial_euroc` 做对照（M7 的验收项）用的还是上游那份 ——
两边都没有改算法，只删了可视化，所以对照仍然成立。

## 对搬进来的东西做了哪些改动

**全都是为了在安卓上编得过，没有一处动到算法。** 每处都在源码里留了注释。

| # | 文件 | 改动 | 不改会怎样 |
|---|---|---|---|
| 1 | `orb_slam3/src/Viewer.cc`、`MapDrawer.cc`、`include/MapDrawer.h` | 削成空壳（保留类名和公有方法签名，方法体清空） | 安卓上没有 Pangolin |
| 2 | `orb_slam3/include/Map.h`、`src/System.cc` | 删掉 `#include <pangolin/pangolin.h>` | 同上 |
| 3 | `orb_slam3/include/Map.h` | `GLubyte* mThumbnail` → `unsigned char*` | `GLubyte` 是 Pangolin 带进来的，剥掉之后没了。它只是 `unsigned char` 的 typedef，那个成员从来没被真正用过 |
| 4 | `orb_slam3/include/*`、`src/*` | 35 处 `#include "Thirdparty/<库>/..."` → `"<库>/..."` | 三个依赖提到上一级了，前缀没了 |
| 5 | `orb_slam3/src/System.cc` | `#include <openssl/md5.h>` → `"md5.h"` | NDK 没有 OpenSSL，见 `app/src/main/cpp/md5.h` |
| 6 | `orb_slam3/src/ORBmatcher.cc`、`DBoW2/DBoW2/FORB.cpp` | `#include <stdint-gcc.h>` → `<stdint.h>` | 上游误留的 GCC 内部头，clang/libc++ 里没有 |
| 7 | `g2o/g2o/core/*.h`（5 个） | `<tr1/unordered_map>`、`<tr1/memory>` → C++11 标准库，`std::tr1::` → `std::` | TR1 是 C++11 之前的 GCC 专有头。clang 定义了 `__GNUC__`，所以它走了 GCC 那条分支 |
| 8 | `g2o/CMakeLists.txt` | `FIND_PACKAGE(Eigen3 ...)` → 用顶层传进来的 `SLAMAPP_EIGEN_INCLUDE` | 交叉编译时 CMake 的 package 查找被 NDK 锁在 sysroot 里，找不到系统装的 Eigen |
| 9 | `DBoW2/CMakeLists.txt`、`g2o/CMakeLists.txt` | 删掉 `-march=native` | 那是"按本机 CPU 生成指令"，本机 x86、目标 arm64，直接报 `unknown target CPU 'native'`。它只是优化 |

第 3、6、7 条是这次真编才暴露出来的 —— 光看代码不容易想到。

## 许可证

搬进来的东西各自带着自己的许可证，不要当成这个项目的代码：

- ORB-SLAM3、DBoW2、g2o —— **GPLv3**
- Sophus —— MIT

外装的那几个：OpenCV 是 Apache 2.0，Eigen 是 MPL2，Boost 是 BSL-1.0。
