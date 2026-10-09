# 2026-09-26 修好编辑器智能感知：把编译数据库导出到固定路径

## 动机

用户报告：**CMake 生成的智能感知文件找不到，编辑器读不到报错。**

排查结果：**文件其实是生成好的**，两处都有，内容一致，而且都指向正确的 NDK clang
（带 `--target=aarch64-none-linux-android26` 和 `--sysroot=`）：

```
app/.cxx/Debug/70425w2u/arm64-v8a/compile_commands.json   ← AGP 真正的构建目录
app/.cxx/tools/debug/arm64-v8a/compile_commands.json      ← AGP 给 IDE 的
```

**根因是两个叠在一起的事实：**

1. **用户用的是 `ms-vscode.cpptools`（官方 C/C++ 扩展）**，不是 clangd。
   它**不会自动去 `.cxx` 里找**，必须有 `compileCommands` 设置——而工作区里
   **一份 `c_cpp_properties.json` 都没有**。
2. 就算写了，**`compileCommands` 不支持通配符**，而 AGP 把它放在
   `app/.cxx/<构建类型>/<哈希>/<abi>/` 下——**那层哈希目录（`70425w2u`）
   会随构建配置变化**。配死路径就会在某次构建之后突然失效，
   表现是"智能感知莫名其妙读不到报错"，很难往这儿想。

## 改动

- **`app/src/main/cpp/CMakeLists.txt`** —— 加 `add_custom_target(export_compile_commands ALL ...)`，
  每次构建后用 `copy_if_different` 把 `${CMAKE_BINARY_DIR}/compile_commands.json`
  拷到**本文件旁边**（固定路径）。

  **用 `add_custom_target` 而不是 `configure_file`**：编译数据库是在 configure 阶段的
  **最后**才写出来的，configure 期间读它只会读到上一次的（或者根本没有）。

- **`/home/welkin/SLAMAPP/.vscode/c_cpp_properties.json`（新建）** —— 指向那个固定路径，
  外加 `compilerPath`（NDK clang）和 `intelliSenseMode: linux-clang-arm64`（交叉编译到 arm64）。

  **第二版又补了 `compilerArgs`**（见验证第 4 条）：`--target` 和 `--sysroot` 两个参数。
  **它们本来就在导出的 `compile_commands.json` 里** —— 但 cpptools **解析编译命令时不认
  `--sysroot`**（它把"系统头在哪"当成另一条路径，单独去问编译器）。所以要把这两个
  参数**再告诉它一次**，否则找不到 `jni.h`。

- **`.gitignore`** —— 加一行 `app/src/main/cpp/compile_commands.json`（生成物，不是源码）。

## 决策

- **采用：让构建自己导出到固定路径**，而不是每次哈希变了手动改配置。
  理由：那个哈希对用户是黑盒，**不知道它什么时候变**；而"某天智能感知突然不灵了"
  是个极难定位的症状。
- **采用：拷贝，不是软链。** 拷贝出来的是**真文件**，`.cxx` 被整个清掉也不影响。
- **否决：软链。** 同样要记着"哈希变了重建"，而且 `.cxx` 被 clean 掉时软链会变悬空
  ——**又是同一种"静默失效"**。
- **否决：直接用 `.cxx` 下那份。** 就是哈希问题本身。
- **否决：`configure_file`。** 时序不对：编译数据库在 configure **末尾**才写出。
- **⚠️ 修正：`add_custom_target(... ALL)` 不够，必须 `add_dependencies`。**
  我原来的想法是"`ALL` 就会跟着默认构建跑"——**实测推翻了**（见验证第 2 条）。
  **AGP 不走 `all`，只构建它要的那个 `.so`。**
- **否决：走 CMake Tools + `CMakePresets.json`**（就是 ORB-SLAM3 那套机制）。
  好处是**一个字都不用手写**（工具链、sysroot 全自动），换 NDK 版本也不用改配置。
  **否决理由**：`cmake.sourceDirectory` 是**工作区级**的，一个工作区只能有一个
  活动 CMake 工程，而 `SLAMAPP/` 下现在有**两个**（`ORB_SLAM3/` 和 android 的）。
  要解就得上**多根工作区**（`.code-workspace`），而那会让"整个 SLAMAPP 一眼看全"
  的现状变差（`ORB_SLAM3/`、`android/` 要么丢掉、要么在树里重复出现）。
  **用户评估后认为不值。**
- **采用：只补 `compilerArgs` 两行。** 这不是"手写新路径"——那两个参数**本来就在
  `compile_commands.json` 里**，只是 cpptools 解析编译命令时不认 `--sysroot`，
  要单独告诉它一次。

## 验证

**1. 根因的凭据**

```
$ find . -name compile_commands.json
./app/.cxx/Debug/70425w2u/arm64-v8a/compile_commands.json
./app/.cxx/tools/debug/arm64-v8a/compile_commands.json

$ head -c 600 app/.cxx/Debug/70425w2u/arm64-v8a/compile_commands.json
... "command": "/home/welkin/Android/Sdk/ndk/26.1.10909125/toolchains/llvm/prebuilt/
     linux-x86_64/bin/clang++ --target=aarch64-none-linux-android26 --sysroot=..."

$ ls ~/.vscode-server/extensions | grep -i cpp
ms-vscode.cpptools-1.34.4-linux-x64        ← 确认是官方扩展，不是 clangd

$ ls /home/welkin/SLAMAPP/.vscode/
settings.json        ← 此前没有 c_cpp_properties.json
```

**2. ⚠️ 第一版（只写 `ALL`）实测没用 —— 这条是本次最有价值的部分**

用户跑了构建，成功，但文件没出来：

```
$ ./gradlew assembleDebug
BUILD SUCCESSFUL in 7s
$ ls -l app/src/main/cpp/compile_commands.json
ls: cannot access 'app/src/main/cpp/compile_commands.json': No such file or directory
```

诊断（推翻了我的直觉——我以为 `ALL` 一定会跑）：

```
$ grep -c export_compile_commands app/.cxx/Debug/70425w2u/arm64-v8a/build.ninja
5                                        ← 目标确实生成进 build.ninja 了

$ grep -m5 "^default \|^build all" .../build.ninja
build all: phony .../obj/arm64-v8a/libslam_jni.so export_compile_commands
default all                              ← 而且确实挂在 all 里

$ cat .../.ninja_log
0    140  ... CMakeFiles/slam_jni.dir/jni_bridge.cpp.o
140  288  .../obj/arm64-v8a/libslam_jni.so     ← 只跑了这两个，导出目标一次没跑
```

**结论：AGP 构建的是 `libslam_jni.so` 这个具体目标，不走 `all`。**
所以 `ALL` 形同虚设 —— **必须挂成依赖**：
```cmake
add_dependencies(slam_jni export_compile_commands)
```

**3. `add_dependencies` 那一版生效了**

用户重跑构建后，导出的文件出现（20:18:13）：
```
$ ls -l app/src/main/cpp/compile_commands.json
-rw-r--r-- 1 welkin welkin 767 20:18:13 app/src/main/cpp/compile_commands.json
```

**4. ⚠️ 但智能感知还是报错 —— 而且这暴露了第二层问题**

```
检测到 #include 错误。请考虑更新 compile_commands.json 或 includePath。C/C++(1696)
无法打开 源 文件 "jni.h" C/C++(1696)
```

**根因：cpptools 解析编译命令时不认 `--sysroot`。** 它把"系统头在哪"当成
**另一条路径**（单独跑一次编译器去问），而问的时候不带 sysroot → 拿到的是
**宿主机**的目录 → 没有 `jni.h`。

`jni.h` 确实在编译命令指向的地方：
```
$ find $NDK/sysroot -name jni.h
.../toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include/jni.h
```

修法：`compilerArgs` 里显式给 `--target` 和 `--sysroot`。

**5. 顺带查清了"为什么 ORB-SLAM3 那边 CMake 之后自动就能识别"**

全局用户设置里有一行：
```json
"C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
```
`ms-vscode.cmake-tools` 扩展**装了**（1.24.42）。CMake Tools 配置 CMake 工程后，
把**已经解析好**的编译器/sysroot/include 喂给 cpptools。

**所以差别不是"发现机制"，是"宿主 vs 交叉"**：
- ORB_SLAM3 是**宿主编译** —— cpptools 用默认编译器问就是对的
- Android 是**交叉编译** —— 必须有人把 toolchain 告诉它

CMake Tools 没管 Android 这个工程，是因为它是 **Gradle 驱动的 CMake**，
没有人为它在 CMake Tools 里配 NDK toolchain。（要走那条路得加 `CMakePresets.json`
+ 处理"一个工作区两个 CMake 工程"的冲突 —— 用户评估后**不做**，代价不值。）

**6. 未验证**

- **`compilerArgs` 那一版没验证。** 没重载窗口、没跑 `C/C++: Log Diagnostics`。
- **一个未验证的副作用**：这份 `c_cpp_properties.json` 会不会让 ORB-SLAM3 现有的
  配置失效？判断是**不会**（`C_Cpp.default.*` 是"字段缺省时的默认值"，而文件里
  **没写 `configurationProvider`**，应该继续继承 cmake-tools），**但没实测**。
  用户表示不介意。

## 遗留

- **工作区目录的前提**：这份 `c_cpp_properties.json` 放在 `SLAMAPP/` 下。
  **如果 VSCode 开的窗口是 `android/SlamCapture`，这份配置不会被读到**，
  要拷一份到那边的 `.vscode/`。（判断依据是 `SLAMAPP/.vscode/settings.json` 已存在。）
- 目前编译数据库里**只有 `jni_bridge.cpp` 一个条目**（CMakeLists 里就这一个文件），
  它的唯一 include 是 `<jni.h>`（NDK 自带）。**ORB-SLAM3 的头文件要等 M3 才会进来。**

## 回滚

1. `CMakeLists.txt`：删掉 `export_compile_commands` 那个 target（`CMAKE_EXPORT_COMPILE_COMMANDS`
   那一行留着无害）。
2. `.vscode/c_cpp_properties.json`：删掉即可。
3. `.gitignore`：删掉那一行。

**`android/SlamCapture/` 是独立仓库，`git diff` 可退**（`.vscode/` 在 `SLAMAPP/` 下，
那里不是仓库）。
