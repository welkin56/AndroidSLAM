# 2026-09-26 M1 首次 native 构建：写死 NDK 版本 + 负向验证 UnsatisfiedLinkError

## 动机

阶段五 M1 的目标是「空壳 `.so` 进 APK，证明 Kotlin ↔ C++ 这条路是通的」。
代码已经就位（`SlamBridge.kt` 的 `external fun nativeHello()` + `MainActivity.onCreate`
里的自检调用 + `jni_bridge.cpp` 的 `JNI_OnLoad` 骨架），缺的是验收。

两件事发生在这次验收里：

1. **首次 native 构建的 NDK 版本风险。** `app/build.gradle.kts` 里没写 `ndkVersion`，
   AGP 会去要它**自己默认**的那个版本；和本机装的（26.1.10909125）对不上就报
   `No version of NDK matched the requested version`。这是首次 native 构建最常见的
   失败点，而且是"配了环境却没生效"型的失败——最费时间。
2. **用户做了负向验证**（按 STAGE5 的建议故意把 C++ 函数名改错一个字母），
   观察到真实失败模式。

## 改动

### `android/SlamCapture/app/build.gradle.kts`

`android { }` 里加一行 `ndkVersion = "26.1.10909125"`。

意图**不是"修一个 bug"**，而是**消掉一个自由变量**：不写的话，
「本机装了什么 NDK」和「构建用哪个 NDK」是两件事，中间隔着 AGP 的默认值——
而那是个随 AGP 版本变动的隐藏常量。写死之后两者是同一个事实，不用猜。

### `android/SlamCapture/app/src/main/cpp/jni_bridge.cpp`

`nativeHelloo` → `nativeHello`（**改回**）。这是负向验证的收尾，
净效果为零——文件与验证前完全一致。

## 决策

- **采用：显式写死 `ndkVersion`。** 理由：它是"让两个事实合一"，不是"绕过报错"。
  代价是换 NDK 时要手动改这一行——但那正是我们想要的：换版本应该是个显式动作，
  不该被 AGP 的默认值悄悄决定。
- **否决：不写，等报错了再加。** 理由：这条错误的信息量很低（它只说"版本不对"，
  不说"你该用哪个"），而且出现在构建最早期，容易和"环境没装好"混淆。
- **采用：负向验证必须真跑一遍。** 理由见「验证」第 2 条——这次拿到的信息
  （ART 会把它找过的符号名打出来）是**读文档得不到的**，而它把
  "以后撞上 `UnsatisfiedLinkError` 怎么办"从"逐字比对"变成了"照抄日志"。

## 验证

**1. 构建与安装：用户实跑，通过。**

- `./gradlew assembleDebug` 成功（native 构建没有报 NDK 版本错）。
- APK 装到真机后能启动——**也就是说 `.so` 已进包、`System.loadLibrary("slam_jni")`
  已成功**。M1 的前两层（编出来、进包）由此间接确认。

> ⚠️ **未验证**：`ndkVersion` 那行是**预防性**的——用户没有报告过 NDK 版本错误，
> 所以"不加这行会不会报错"没有被观测。它当前的作用是消除不确定性，不是修复已发生的故障。
> 同样未验证的还有：`unzip -l` 那条不看设备就能跑的检查、以及 logcat 里
> `M1 桥自检: hello from libslam_jni.so (M1)` 那句是否真的打出来了——
> 用户报告的是负向验证的现象（见下），没贴正向的自检日志。

**2. 负向验证：用户实跑，得到预期结果。**

把 `jni_bridge.cpp` 里的 `nativeHello` 改成 `nativeHelloo`（**只改 C++，Kotlin 不动**），
重新构建安装后：

> 用户报告：**「改了名字之后 APP 都进不去闪退了」**

**这是预期的**，而且现象正确：

- 自检调用在 `MainActivity.onCreate` 里，`UnsatisfiedLinkError` 从 `onCreate`
  穿透出去 → `ActivityThread` 认定 Activity 创建失败 → 进程被杀。
- **所以「一开就闪退」不是「JNI 坏了」，是「自检代码正好写在启动路径上」。**
  同一行若写在按钮回调里，表现会是「App 正常打开、点那个按钮才崩」——
  同一个错误，两种完全不同的表象。

**这次拿到的两个可复用结论：**

1. **ART 的报错里带着它找过的符号名**：

   ```
   No implementation found for java.lang.String
       com.example.slamcapture.SlamBridge.nativeHello()
       (tried Java_com_example_slamcapture_SlamBridge_nativeHello
        and Java_com_example_slamcapture_SlamBridge_nativeHello__)
   ```

   以后撞上 `UnsatisfiedLinkError` **不用逐字比对猜哪里写错了，照抄日志里那个名字**。
   这是 JNI 报错里少见的友好之处。

2. **名字对不上这件事，编译期毫无反应。** 用户是靠人眼发现打错字的；
   换成 M5 喂帧那种上下文，同样的错误表现是「App 正常，就是点了没反应」，
   很难往名字上想。这正是把负向验证放在 M1（只有一个函数、上下文最干净）做的价值。

**3. 未验证**：M1 做得再顺也说明不了"ORB-SLAM3 能在 Android 上跑"。
按 STAGE5，M1 只验桥本身。

## 遗留

- **M2（stdout/stderr → logcat）没做。** 在它做完之前引入 ORB-SLAM3 会是个哑巴
  ——ORB-SLAM3 的进度和错误全走 `cout`/`cerr`，Android 上默认进 `/dev/null`。
- `MainActivity.kt:174` 那行 `M1 桥自检` 是**临时**的。M2 之后可以删，
  也可以留作"native 层已装载"的启动标记——待定。
- STAGE5 的 M1 一节现在只写了「过了的标志」一句话，**没有这次总结出的
  三层验法**（编出来 / 进包 / 跑通）和失败模式对照表。要不要补进去待定。

## 回滚

`android/SlamCapture/` 这个仓库里 `git diff` 可退。两处：

1. `app/build.gradle.kts`：删掉 `ndkVersion` 那一行及其注释。
2. `app/src/main/cpp/jni_bridge.cpp`：**无需动作**（已改回原样，净变化为零）。
