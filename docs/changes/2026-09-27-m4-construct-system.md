# 2026-09-27 阶段五 M4：能构造 `System`

## 动机

按 [STAGE5](../tutorial/STAGE5_JNI_BRIDGE.md) 的里程碑推进。M3 已经把
`liborb_slam3.so` 编出来、和 JNI 层一起进了 APK
（[见 2026-09-27-m3](2026-09-27-m3-cross-compile-orb-slam3.md)），
**M4 是第一次真正执行 ORB-SLAM3 的代码**。

验收判据只有一句话：`nativeInit` 返回后，logcat 里有 ORB-SLAM3 自己打印的启动信息，
且 App 没有退出。

**为什么这条判据必须靠日志**：ORB-SLAM3 用 `exit(-1)` 报错，一共 **17 处**
（`System.cc` 7 + `Settings.cc` 9 + `Settings.h` 1），而 `exit()` 不走 Java 异常路径 ——
表现是「App 整个退回桌面、logcat 干干净净」。这正是 M2 存在的理由。

**一条现状**：`SlamBridge` 此前在 Kotlin 侧**没有任何调用点**（M2 那行自检已不在文件里），
而 `object SlamBridge` 的 `init { System.loadLibrary(...) }` 是首次访问该类时才执行 ——
所以 `.so` 之前没被加载过。M4 的「启动 SLAM」把这条链接了回去。

## 改动

### C++

| 文件 | 改了什么 |
|---|---|
| `cpp/slam_engine.{h,cpp}` | **新增**。文件内单例 + 自己的线程 + `new System(...)`，只给三个自由函数 |
| `cpp/jni_bridge.cpp` | 加三个符号：`nativeInit` / `nativeState` / `nativeIsReady`；加一个匿名 namespace 里的 `toStdString` |
| `cpp/CMakeLists.txt` | `add_library(slam_jni ...)` 加 `slam_engine.cpp`（一行）；`target_link_libraries` 的签名从默认改成 `PUBLIC`/`PRIVATE`（见决策七） |

### Kotlin

| 文件 | 改了什么 |
|---|---|
| `SlamBridge.kt` | 加三个 `external fun` |
| `MainActivity.kt` | 加「启动 SLAM」按钮、路径解析、**预校验**、ticker 扩窗、状态栏加一行 |
| `res/layout/activity_main.xml` | 加那个按钮（独占一行，它和采集端那两个不同组语义） |

两个新方法值得点名：

- `resolveVocabPath()` —— 词袋在 `getExternalFilesDir()/ORBvoc.txt`
- `resolveSettingsPath()` —— 把 `assets/phone.yaml` 复制到 `filesDir/` 再返回真实路径
  （`cv::FileStorage` 不吃 AssetManager）

### 资源与脚本

- **`app/src/main/assets/phone.yaml`** —— 按 §2.2，assets 那份是权威副本。
  从 `ORB_SLAM3/Examples/Monocular-Inertial/phone.yaml` 逐字节拷过来
  （`cmp` 验过），**fx 是 435.2** —— 09-23 那次裁定写回的是 ORB_SLAM3 那份，
  而「assets 权威」定于 09-26，之前没有任何记录确认两边同步过，这次核了
- **`tools/push-vocab.sh`** —— 把 145 MB 的词袋 `adb push` 到
  `/sdcard/Android/data/com.example.slamcapture/files/`

### 文档

- **`APP_DESIGN.md` §6 / §7** —— 消解那处**真冲突**（见决策一）
- **`STAGE5_JNI_BRIDGE.md` M4** —— 补上「两个文件从哪来」和「两道防线怎么分工」
- 本文件 + `README.md` 索引

## 决策

### 一、相机 + IMU 归一个「或」语义的类管（用户提出）

`APP_DESIGN` §6 说「已有的采集端状态机**一行都不用改**」，还明确把
「**录但不建图** —— 重录一段做 A/B 对比」列为必须能表达的组合；而 §7.1 说相机+IMU
改绑「SLAM 运行中」。**两条不能同时成立** —— 录制要有帧，帧要相机，而「未启动」那张表
写死了「相机关、IMU 注销」。

用户给的解法：**一个类管相机和 IMU，两个申请者（SLAM / 录制），走「或」——
任一个需要就打开，都放手才关闭。**

这样两条都成立：§6 那个组合靠「录制自己申请」活下来，§7.1 的「SLAM 要常跑」也满足。
而且它顺手解决一个技术问题：`CameraCapture` 和 `ImuCollector` **都没有引用计数**
（前者 `epoch` 是幂等守卫，后者 `started` 是「只能 start 一次」的守卫），
各开各的必然打架。

**地图（`System`）不进这个类。** 判据是 §5 决策一那条原话 ——
**耗电的资源跟着用户意图走，耗时的资源跟着进程走**。两种代价类型混在一个类里，
那个类的生命周期就没有单一含义了。

**这一条 M4 只落到文档**，代码留给后面单独一步（见遗留）。

### 二、`nativeState()` 返回四态 int，不是 bool

0=没开始 1=加载中 2=就绪 3=失败。布尔装不下，而**「失败」必须能显示出来** ——
否则 UI 就是「点了一下，一直转圈」，那正是这个项目反复踩的「查不出来的状态」。

### 三、进度是「已用时」，不是百分比

`loadFromTextFile` 不提供任何进度回调，编不出来。所以状态栏写「已 Xs」。
**顺带这也是实测数字的来源** —— 文档里一直写的是猜的「十几到几十秒」。

### 四、**绝不 `delete System`**

它**没有析构函数**（`System.h` 全文无 `~System`），而 `Shutdown()` **也不 join 线程** ——
`System.cc` 里等 LocalMapping/LoopClosing 结束的 while 循环**被整段注释掉了**，
只发个 `RequestFinish` 就返回。所以「析构」之后线程还在跑、还握着已释放的对象，
`delete` 就是 use-after-free。

「`System` 活到进程结束」既是省一次 145 MB 词袋重载，**也是唯一安全的做法**。

### 五、单例故意泄漏 + 线程 detach

函数内静态对象会在进程退出时析构，而那时去析构一个 joinable 的 `std::thread`
会直接 `std::terminate`。所以 `static Engine* e = new Engine;` 故意泄漏。

### 六、`bUseViewer` 显式传 `false`

`System.h:105` 的默认值是 `true`。Pangolin 虽然剥了，但传 `true` 照样会
`new Viewer` + 起一条 `std::thread`（`System.cc:231-232`）。

### 七、❌ 走过一条错路：以为 `PRIVATE` 能消掉重复符号

`libslam_jni.so` 的 `.text` 是 253 KB，里面有 **170 个 `cv::` 符号**
（`imshow`、`namedWindow`、`CommandLineParser` 那一堆）而它本该只有几十 KB。

我先判成「`target_link_libraries` 用了默认签名（= PUBLIC），OpenCV 被传递给了
`slam_jni`」，改成 `PRIVATE` 之后**重编，符号数反而从 159 涨到 170，体积没变**。

真因在链接行上：**OpenCV 的 `.a` 照样出现在 `libslam_jni.so` 的链接行里**，
漏点是 vendored 的 `DBoW2/CMakeLists.txt` —— 它写着
`target_link_libraries(DBoW2 ${OpenCV_LIBS})`，用的是默认签名，从那条路传下去了，
在 `orb_slam3` 这一层挡不住。

`PRIVATE` 还是留下了（意图是对的），但**注释改成了实话**，免得下一个人照着错的理由去查。
真要清掉得改 DBoW2 那一行 —— 见遗留。

### 八、❌ 否决：重写 `System` 只留 IMU_MONOCULAR（用户提的）

用户问能不能把 `System` 瘦成只有 mono-inertial。**能做，但不是现在**，理由是数算不过来：

- `liborb_slam3.so` 在 APK 里 **42.55 MB**，而 `libg2o.so`(10.9M) / `libDBoW2.so`(3.2M) /
  `libboost_serialization.a`(1.06M) 都是**独立的**，不在这个数里。也就是说这 42.55 MB
  绝大部分是**静态链进去的 OpenCV**，ORB-SLAM3 自己的源码才 1.4 MB 文本
- `TrackStereo` / `TrackRGBD` 加起来两三百行，砍掉省不到 1 MB
- 代价是 fork 一个 1600 行的文件 —— 而我们刚花整场 M3 才让它编过
- 而且它和 vendored 那份 README 的规矩冲突：那里写的是「9 处移植改动**全是为了在安卓上
  编得过，没有一处动算法**」。重写 `System` 会让 M7 那条「PC 和手机跑**同一份**算法」
  的对照打折扣

**等 M4 跑通、手上有实测数据再决定。**

### 九、词袋：M4 走 `adb push`，assets+gzip 留到「实际使用时」

145 MB 进 APK 会让它从 60 MB 涨到 200 MB+。用户定的：调试阶段怎么方便怎么来，
实际使用时打进 assets 并 gzip、首次运行解压到 `filesDir`。

## 验证

### 1. C++ 全量交叉编译通过（用**真的** Boost，不是替身）

```
$ cmake -S app/src/main/cpp -B /tmp/m4build -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake \
        -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-26 -DCMAKE_BUILD_TYPE=Release
configure 退出码: 0

$ ninja -j4
退出码: 0
  5.61 MB  libDBoW2.so
 55.08 MB  libg2o.so
225.42 MB  liborb_slam3.so     ← 未 strip；进 APK 那份是 42.55 MB
  1.79 MB  libslam_jni.so
```

### 2. JNI 符号名逐个核对

这是最容易错的地方（§1.2 命名层）。C++ 导出的 5 个 `Java_` 符号与 `SlamBridge.kt`
的 5 条声明**逐字对上**：

```
C++ 侧：nativeHello  nativeInit  nativeIsReady  nativeLogSelfTest  nativeState
Kotlin：nativeHello  nativeInit  nativeIsReady  nativeLogSelfTest  nativeState
```

**`toStdString` 那个新增的辅助函数没有漏进导出表**（M2 那条教训的复查）。

### 3. 上设备：**M4 判据达成**

```
09-27 18:09:27.678  I slam: [M2] stdout/stderr 已接进 logcat，tag=slam
09-27 18:09:56.584  I slam: [M4] 开始构造 System（IMU_MONOCULAR，bUseViewer=false）
                    I slam: ORB-SLAM3 Copyright (C) 2017-2020 ...
                    I slam: Input sensor was set to: Monocular-Inertial
                    I slam: Loading settings from /data/user/0/com.example.slamcapture/files/phone.yaml
                    I slam:      -Camera 1 parameters (Pinhole): [ 435.2 435.2 320 240 ]
                    I slam: Loading ORB Vocabulary. This could take a while...
09-27 18:10:34.193  I slam: Vocabulary loaded!
                    I slam: Initialization of Atlas from scratch
                    I slam: Creation of new map with id: 0
09-27 18:10:34.254  I slam: [M4] System 构造完成，耗时 37669 ms
```

**App 没有退出。** 这一趟顺带验掉的：

| 验掉的 | 证据 |
|---|---|
| **M2 第一次真跑起来** | `.so` 之前从没被 `loadLibrary` 过（`SlamBridge` 零调用点），这是它头一次加载 |
| M2 的日志重定向真的管用 | 上面那些全是 ORB-SLAM3 自己的 `cout`，一行没漏 |
| JNI 三个新符号 | 调通了才有后面这些 |
| **yaml 从 assets 落到 filesDir 那条路** | `Loading settings from /data/user/0/.../files/phone.yaml` |
| **assets 那份的 fx 是对的** | `[ 435.2 435.2 320 240 ]` |
| **M2 那条遗留**（`liborb_slam3.so` 静态初始化期有没有输出） | **没有** —— `JNI_OnLoad` 那行就是 logcat 第一条，前面什么也没有 |

所有 `optional parameter does not exist` 都是 `required=false` 的键，`Settings` 打一行就跳过 ——
反过来说，**我们这份 yaml 必填键一个不缺**，这正是它没触发 `exit(-1)` 的原因。

#### ⭐ 实测：词袋加载 **37.7 秒**

```
18:09:56.589  Loading ORB Vocabulary. This could take a while...
18:10:34.193  Vocabulary loaded!
```

文档里一直写的是**猜的**「十几到几十秒」（§3.3 / §7.4）。实测 37.7 s，在最上沿。
**这个数直接决定界面**：38 秒的等待，「已 Xs」那个进度不是可有可无的。
已把 §3.3 和 APP_DESIGN §7.4 改成实测值。

### 4. **还没做的**

- **两条负向验证**（见 STAGE5 M4）：路径不存在 → 预校验挡住；
  yaml 缺一个 `Viewer.*` 键 → 预校验放行、M2 的 logcat 指出是哪个键、然后 App 消失。
  **这两条没跑过**，所以「两道防线的分工」目前还只是设计，没有实测背书。

## 遗留

- **相机 / IMU 的挂载重构**（把 `beginPrepare`/`teardown` 里开合硬件的部分搬进
  决策一那个类）。**M4 一行没动采集端**，所以这次改动不会影响已经跑通的采集流程。
- **UI 外壳**（入口页 + 主界面两按钮，空实现）。用户定的次序是
  **M4 → UI 外壳 → 硬件类 → M5**，理由是 M4 是这几步里唯一有未知数的。
- **assets + gzip 词袋**（用户说的「实际使用时」）
- **`libslam_jni.so` 里多出约 253 KB 的 OpenCV 代码**。真因见决策七，
  要清掉得把 vendored `DBoW2/CMakeLists.txt` 的
  `target_link_libraries(DBoW2 ${OpenCV_LIBS})` 改成 `PRIVATE`。
  **只是体积问题，不是正确性问题** —— 两份是同一份静态库编出来的，逐字节相同。
- **`SlamEngine` 的队列、`nativePushImage`、`nativeStart/Stop`**（M5）
- **`referenceMap` / `relocalize` / `saveTo`**（M8）
- **`loadFromFile` 这个参数现在是空转的。** 而且它和 ORB-SLAM3 的结构对不上：
  `System` 的构造函数**没有**「从哪个文件载」这个参数，它读的是 yaml 里的
  `System.LoadAtlasFromFile`（`System.cc:76-98`）。M8 要真支持，得另想办法
  （改 yaml 或给 `System` 加字段）
- **17 处 `exit(-1)` 一处没动。** 这是 M4 判据必须靠日志的原因，不是可以顺手清掉的东西
- **`phone.yaml` 的文件头注释还停在阶段三**（说「内参和外参都还没标定」），
  而里面的 fx 已经是 09-23 标定后的 435.2。那是 `ORB_SLAM3/` 那份的注释，没动

## 回滚

**整个 M4 都不要**（回到 M3 的状态）：

1. `rm app/src/main/cpp/slam_engine.{h,cpp}`
2. `jni_bridge.cpp` —— 删掉三个 `Java_..._nativeInit/nativeState/nativeIsReady`
   和那个匿名 namespace 里的 `toStdString`，以及 `#include "slam_engine.h"`
   和 `#include <android/log.h>`
3. `CMakeLists.txt` —— `slam_jni` 的源文件表去掉 `slam_engine.cpp`
4. `SlamBridge.kt` —— 删三个 `external fun`
5. `MainActivity.kt` / `activity_main.xml` —— 删「启动 SLAM」按钮那一行、SLAM 那几个字段、
   `onButtonSlam` 和两个 `resolveXxxPath`、`refuse`、`uiQueueTicker`；
   把 ticker 和 `teardown` 里的 `uiQueueTicker()` 改回 `uiHandler.removeCallbacks(ticker)`
6. `rm -r app/src/main/assets/`、`rm tools/push-vocab.sh`

**只回滚「词袋走 adb push」这一条**：把 `resolveVocabPath()` 改成指向
`filesDir`，改用别的投放方式 —— 其余不动。

文档那边：`APP_DESIGN` §6/§7 的改动、`STAGE5` 的 M4 一节、本文件，都在 `docs/` 仓库里，
可退。
