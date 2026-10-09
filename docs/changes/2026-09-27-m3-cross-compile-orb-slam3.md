# 2026-09-27 阶段五 M3：交叉编译 ORB-SLAM3

## 动机

按 [STAGE5](../tutorial/STAGE5_JNI_BRIDGE.md) 的里程碑推进。M1 的空壳 `.so` 进 APK 了，
M2 的日志通道通了（[见 2026-09-27-m2](2026-09-27-m2-stdout-to-logcat.md)），
现在把真的 ORB-SLAM3 编进来。

M3 的「过了的标志」只有一句话：**`liborb_slam3.so` 产出。**
不碰 `System` 的构造（M4），不喂帧（M5）。

## 改动

### 一、ORB-SLAM3 从「原地改」改成「vendored」

这是本阶段最大的结构决定，也是用户提出的：

- **之前**：直接改 `SLAMAPP/ORB_SLAM3/`（上游 clone，本地 git 仓库）。
- **现在**：复制一份到 `android/SlamCapture/app/src/main/cpp/thirdparty/orb_slam3/`，
  **`SLAMAPP/ORB_SLAM3/` 保持上游原样，一字节不动。**

注意区分两件事：**源码共享**和**构建系统共享**。Android 的构建从头到尾只由
`app/src/main/cpp/CMakeLists.txt` 驱动，它把 ORB-SLAM3 的 `src/*.cc` 按路径列出来自己编。
`ORB_SLAM3/CMakeLists.txt` 一次都不参与交叉编译（它 `find_package(Pangolin REQUIRED)`
找不到、`find_package(OpenCV 4.4)` 会找到宿主那份 x86 的、还会去编 `Examples/`）。

### 二、新增的文件

| 路径 | 是什么 |
|---|---|
| `cpp/thirdparty/orb_slam3/{src,include}/` | ORB-SLAM3 本体，28 个 TU |
| `cpp/thirdparty/{DBoW2,g2o,Sophus}/` | 从 `ORB_SLAM3/Thirdparty/` 复制，三个库都提上一级 |
| `cpp/md5.{h,cpp}` | 顶替 OpenSSL 的 MD5（见决策四） |
| `cpp/CMakeLists.txt` | 整个重写 |
| `cpp/thirdparty/README.md` | 依赖分法 + 9 处移植改动的清单 |
| `tools/setup-boost-android.sh` | 交叉编译安卓版 Boost |
| `.gitignore` | 加 `thirdparty/boost-android/` |

### 三、vendored 副本里的 9 处移植改动

**全是「在安卓上编得过」，没有一处动算法。** 完整表格在 `thirdparty/README.md`，
这里只记哪些是**真编了才暴露出来的**（光看代码不容易想到）：

- `Map.h` 的 `GLubyte* mThumbnail` —— `GLubyte` 是 Pangolin 带进来的，剥掉之后没了。
- `ORBmatcher.cc` 和 `DBoW2/FORB.cpp` 里的 `#include <stdint-gcc.h>` ——
  上游误留的 GCC 内部头。
- g2o 的 5 个头文件用 `<tr1/unordered_map>` / `<tr1/memory>` —— TR1 是 C++11 之前的
  GCC 专有头，而 clang 定义了 `__GNUC__`，所以它走了 GCC 那条分支。

## 决策

### 一、依赖分两类：自带 vs 外装（用户定的纲）

用户给出了判据，借的是 ORB-SLAM3 自己在 `Thirdparty/` 上的分法：

> **作者改过、版本敏感、紧耦合、体积小** → 自带，跟着代码走；
> **通用、成熟、体积大、系统包维护好** → 外装。

| 库 | 判定 | 落地 |
|---|---|---|
| DBoW2 / g2o / Sophus | 自带 | 是 ORB-SLAM3 作者 fork 过的版本，必须跟着代码走 |
| ORB-SLAM3 本体 | 自带 | |
| **Eigen** | 外装 | 系统 `apt libeigen3-dev`，和 ORB-SLAM3 在 PC 上的用法一致 |
| **OpenCV** | 外装 | 官方 Android SDK，指向 `SLAMAPP/OpenCV-android-sdk` |
| **Boost** | 外装 | 没有官方安卓发布物，自己编；且它是构建产物，gitignore |

**我在这上面判断错了两次，都被用户拦下来**：先把 Eigen 搬进 `thirdparty/`，
又把 OpenCV 的 185 MB 搬进仓库。两次都是拿「自包含」当理由，而忽略了
「自带的是 fork、外装的是成熟库」这条真正的线。

### 二、OpenCV 用哪一份：三条路走下来

| 方案 | 仓库要背 | 结果 |
|---|---|---|
| 静态全家桶（`staticlibs/` + `3rdparty/libs/`） | 167 M | 起先搬进去了，**被否** |
| 只留那个 18 M 的 `libopencv_java4.so` | 25 M | 想过，也**被否** |
| **指向本机那份官方 SDK** | 0 | **采用** |

第三条否掉前两条的理由不只是体积：`libopencv_java4.so` 那条路虽然实测导出 2487 个
`cv::` 符号、够用，但那意味着**要放弃 SDK 官方配置、改成手写 IMPORTED target**，
而最后还是要靠仓库外的路径。既然躲不掉外部路径，那就不该为「少一点点外部依赖」
去承担一个自己维护的链接配置。

### 三、Boost 必须编，而且 gitignore

`System.cc` 的 `SaveAtlas` / `LoadAtlas` 是两个开关（yaml 的 `System.SaveAtlasToFile` /
`System.LoadAtlasFromFile`）控制的，`phone.yaml` 里没有它们 —— **M3~M7 一次都执行不到，
M8 的「继续上次」全靠它**。

问过「能不能改源码去掉依赖」，答案是代价更高：12 个文件里的 `serialize()` 是成员模板，
不实例化就不产生代码，但**声明和 `#include <boost/...>` 必须在**，否则头文件本身编不过。
要用 `#ifdef` 全套包起来，M8 再原样拆回来。

**它是构建产物**（绑死 NDK 版本和编译选项、150 MB、其中只用到 `libboost_serialization`
一个），所以 `.gitignore` 掉，配一个 `tools/setup-boost-android.sh`。

**顺带发现的坑**：`boost_serialization` 那个 `.a` 不能用 `find_library` 找 ——
NDK 的 toolchain 把 `find_*` 的搜索根锁在 sysroot 里（`CMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY`），
`PATHS` 给的目录会被重新挂到 sysroot 下面，**装在哪都找不到**。改成直接写路径。

### 四、MD5 顶替 OpenSSL，而不是给它装个 OpenSSL

`System::CalculateCheckSum` 用 OpenSSL 的 MD5 给词袋文件算校验和（存/载地图时核对
自己是不是配着同一份词袋）。全项目就这一处。

安卓上没有 OpenSSL：NDK 不带，系统那个 `libcrypto` 是 BoringSSL、不属于 NDK 公开 API。
为了一个哈希函数去交叉编译整份 OpenSSL 不值当，所以自带一个 `md5.{h,cpp}`。

**函数名和签名照抄 OpenSSL**，所以 `System.cc` 只改了一行 include，
`MD5_Init/Update/Final` 那几行一个字没动。**摘要和 OpenSSL 逐字节相同**（实测见验证第 1 条），
PC 上存的地图和手机上存的能互相认。

**这个文件放哪，来回改了三次**：`thirdparty/openssl-md5/` → `cpp/md5_compat/` →
`cpp/md5_compat.{h,cpp}` → `cpp/md5.{h,cpp}`。用户的两次意见都对：
① 我们自己写的代码不该放进 `thirdparty/`（那个目录的定义是「别人写的」）；
② 不必为了维持 `<openssl/md5.h>` 那个假路径去套一层目录 —— 改 `System.cc` 一行更直白。

**否决：靠 include 路径把 `<openssl/md5.h>` 遮住。** 那样后来人会以为项目真依赖 OpenSSL。

### 五、用户提过「用编译参数给每个文件配一个头文件」，实测对这条无效

针对 `stdint-gcc.h` 那个错，用户记得以前用编译参数解决过。实测：

```
$ clang++ ... -c /tmp/t.cpp                → fatal error: 'stdint-gcc.h' file not found
$ clang++ ... -include stdint.h -c /tmp/t.cpp → fatal error: 'stdint-gcc.h' file not found
```

`#include <stdint-gcc.h>` 是无条件指令，先塞别的头进去不会让那个不存在的文件出现。
用户记的应该是**另一件事**：很多 ORB-SLAM3 文件用了 `uint8_t` 却没 include `<stdint.h>`，
`-include stdint.h` 正好补上那种。**同一地带的不同毛病。**

### 六、`CMAKE_EXPORT_COMPILE_COMMANDS` 的位置（M2 遗留的 bug）

这是 M2 那份 CMakeLists 里一直存在的错：它被写在文件**末尾**，而
**它是在每个 target 被创建时用来初始化 target 属性的**，所以对先创建的
`DBoW2` / `g2o` / `orb_slam3` 全部无效 —— 整个构建里没有任何 target 导出，
顶层 `compile_commands.json` 根本不生成。

M2 之所以看着是好的，是因为 **AGP 在命令行上自己传了** `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`。
独立跑 CMake 时就现原形了。**已挪到 `project()` 之前。**

## 验证

### 1. MD5 与 OpenSSL/md5sum 逐字节相同

```
$ ./md5test          # 我的实现
d41d8cd98f00b204e9800998ecf8427e     ← 空串
900150983cd24fb0d6963f7d28e17f72     ← "abc"
f96b697d7cb7938d525a2f31aaf161d0     ← "message digest"
c3fcd3d76192e4007dfb496cca67e13b     ← a-z
57edf4a22be3c955ac49da2e2107b67a     ← 80 位数字
30a83621ce5422fbdfdd539777458c78     ← 200 个 'x'，逐字节流式喂
dd3d2d60c2aae8edc7f7e2a5203b15f7     ← 按文件读（System.cc 的用法）

$ md5sum / printf | md5sum          # 系统说的答案
七条全部逐字符相同
```

### 2. 交叉编译：71/72 步通过

**方法**：真安卓版 Boost 还没编，用一个替身顶上 ——
`/tmp/fake-boost/include/boost` 软链到宿主的 boost 头（同为 1.83），
`lib/libboost_serialization.a` 是个**空归档**。这样所有 TU 都能编，只在最后链接时断掉。

> 第一次替身做错了（把整个 `/usr/include` 链进去），宿主的 glibc 头跑到安卓 sysroot
> 前面，报了 `bits/wordsize.h` not found。改成只暴露 `boost/` 一个子树。
> **失败的是替身，不是项目。**

```
$ cmake -S app/src/main/cpp -B /tmp/m3build -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake \
        -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-26 \
        -DBOOST_ROOT_ANDROID=/tmp/fake-boost
configure 退出码: 0
-- Found OpenCV: /home/welkin/SLAMAPP/OpenCV-android-sdk (found version "4.6.0")
                          found components: core imgproc imgcodecs features2d calib3d flann

$ ninja -j4
71/72 步完成；69 个 .o；libDBoW2.so (5.7 MB)、libg2o.so (56 MB) 产出
FAILED: liborb_slam3.so
```

**唯一的失败是链接，而且缺的符号清一色是 Boost：**

```
$ grep -oE "undefined symbol: [A-Za-z_:]+.*" m3build.log | sed -E 's/^(undefined symbol: [A-Za-z_]+::[A-Za-z_]+).*/\1/' | sort | uniq -c
     20 boost::archive
```

**20 个未定义符号，100% 是 `boost::archive`，没有一个别的。**
而且全部只从 `System.cc.o` 引用，落点是 `SaveAtlas` / `LoadAtlas`：

```
System.cc.o:(ORB_SLAM3::System::LoadAtlas(int))
System.cc.o:(ORB_SLAM3::System::SaveAtlas(int))
```

这正是预判的位置。**换句话说：除了那份真 Boost，`liborb_slam3.so` 已经没有别的缺口。**

### 3. 真 Boost 编好之后，Gradle 全量构建通过

`tools/setup-boost-android.sh` 跑通了（**脚本里唯一没把握的 `--user-config` 参数，
b2 认**）。产出 `libboost_serialization.a` 1.06 MB + `libboost_wserialization.a`
695 KB（后者是宽字符变体，`--with-serialization` 顺带编的，我们用不到、也没链）。

```
$ ./gradlew assembleDebug
BUILD SUCCESSFUL in 58s
39 actionable tasks: 9 executed, 30 up-to-date
```

APK 里的 native 库（`unzip -l app-debug.apk | grep lib/`）：

```
 3174016  lib/arm64-v8a/libDBoW2.so
10943448  lib/arm64-v8a/libg2o.so
44612840  lib/arm64-v8a/liborb_slam3.so
  582104  lib/arm64-v8a/libslam_jni.so
```

**逐个核过 `liborb_slam3.so`**：

```
Class: ELF64    Type: DYN    Machine: AArch64

$ llvm-nm -D liborb_slam3.so | grep -c ' T '
1486                                    ← 教程里那条验收命令

$ llvm-readelf -d liborb_slam3.so | grep NEEDED
libDBoW2.so  libg2o.so  libz.so  liblog.so  libandroid.so  libc.so  libdl.so  libm.so  libmediandk.so

$ llvm-nm -D --defined-only liborb_slam3.so | grep -c boost
10566
```

两点值得记：

- **`NEEDED` 里没有 boost** —— 它被**静态**链进了 `liborb_slam3.so`（10566 个 boost
  符号定义在里面）。这正是 `link=static` 要的效果：APK 里不用再多一个要打包的 `.so`。
- **`libandroid.so` 出现了** —— 来自静态链进来的 OpenCV。它是 NDK 公开库，正常。

APK 60 MB（native 部分约 59 MB，主要是 `liborb_slam3.so` 的 44.6 MB，
其中一大半是静态链进去的 OpenCV）。

### 4. 未验证

- **上设备**：`.so` 能否加载、`System.loadLibrary` 会不会报缺符号、
  M2 那条「静态初始化期的输出接不接得到」的遗留（`liborb_slam3.so` 现在是
  `libslam_jni.so` 的依赖，App 一启动它的静态初始化就会跑，正好验这个）。

至此 M3 的验收判据（**`liborb_slam3.so` 产出**）达成。

## 遗留

- **`SLAMAPP` 之外只剩两个路径依赖**：OpenCV SDK 的位置（`-DOPENCV_ANDROID_SDK=`）、
  系统 Eigen（`apt install libeigen3-dev`）。README 和 CMakeLists 里都写明了。
- **Boost 编好之后 `thirdparty/boost-android/` 占 185 MB**（其中绝大部分是全量的
  Boost 头文件，实际只用到几个）。它在 `.gitignore` 里，不进仓库。
- ~~**`setup-boost-android.sh` 编完不清临时文件。**~~ **已补**：脚本现在
  **默认保留**缓存（重编能省十几分钟），结束时打印位置、体积和一条删除命令；
  加 `--clean` 就直接删。本机那 1.1 GB 已经手动删掉了。
- **`.so` 有没有真上过设备还不知道。** 目前只验到「编出来、链对了、进了 APK」，
  没验「装到手机上 `System.loadLibrary` 不报错」。
- **g2o 的 CMake 构建会往源码树写 `thirdparty/g2o/config.h`**
  （`configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)`）。实测写回的字节和原来
  完全一样（两个 `#cmakedefine` 都不成立），所以 `git status` 干净。
  **但换 NDK 或打开 `G2O_USE_OPENMP` 时会真的改到那个文件。**
- **`SLAMAPP/ORB_SLAM3/` 和 vendored 副本从此会分叉。** 规则是「以后只改 vendored 那份」。
  M7 的对照验收里，PC 跑的是上游那份 —— 两边都没动算法，只删了可视化，所以对照成立。
- **C++ 标准锁在 C++14**（`CMAKE_CXX_STANDARD 14`）。上游就是 14，
  而且 `LoopClosing.cc` 里有 `mnFullBAIdx++`（bool 自增，C++17 移除）这类东西，
  往上抬会编不过。
- 编译警告（`AlignedBit is deprecated`、`-Wdangling-else` 等）**没清**，都是上游的，
  不影响正确性。

## 回滚

**只是不想编 ORB-SLAM3**（回到 M2 的状态）：

1. `CMakeLists.txt` —— 删掉 OpenCV / Boost / Eigen / DBoW2 / g2o / orb_slam3 那几段，
   `add_library(slam_jni ...)` 恢复成只有 `jni_bridge.cpp log_redirect.cpp`，
   `target_link_libraries(slam_jni log)` 去掉 `orb_slam3`。
2. `rm -rf app/src/main/cpp/thirdparty/`（含 vendored 的 ORB-SLAM3，是未跟踪文件）。
3. `rm app/src/main/cpp/md5.{h,cpp}`、`rm -r tools/`。
4. `.gitignore` 里去掉 boost-android 那条。

`ORB_SLAM3/` 那边**没有任何东西需要回滚** —— 这次一字节都没动它。

**想把 vendored 换回原地改**：把 `thirdparty/orb_slam3/` 的 `src`/`include` 指向
`SLAMAPP/ORB_SLAM3/`，把 9 处移植改动搬到上游那份。不建议 —— 两份会分叉，
而且上游那份同时是 PC 对照用的。
