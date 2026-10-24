# 编译内存

`./gradlew installDebug` 占满内存（15 GB 物理 + 8 GB swap）的成因和调法。

## 原因

内存大头不是 Gradle，是 **NDK 侧并行编译**。

- AGP 调 ninja 的命令由 CMake 生成，形如 `ninja -C <abi 目录> {targets}`，**不带 `-j`**。
  ninja 不带 `-j` 就按在线核数开满。
- WSL 里 `processors=32`，于是一次可能起 32 个 clang。ORB-SLAM3 的 `Optimizer.cc`、
  `Tracking.cc` 这类文件单个实例的峰值就有几百 MB 到 1 GB，32 个叠起来必然爆。
- Gradle 守护进程（`-Xmx4g`）和 Kotlin 守护进程（默认 `-Xmx2g`）再各占一块。

编译产物按 `abiFilters` 只有 `arm64-v8a` 一个 ABI，所以并发只来自 ninja 的 job 数，
不是多 ABI。

## 现在的配置

两处都已经在仓库里，不用每次手动设。

1. `app/src/main/cpp/CMakeLists.txt` 开头：

   ```cmake
   set(CMAKE_JOB_POOLS compile=8)
   set(CMAKE_JOB_POOL_COMPILE compile)
   ```

   给所有 target 的编译规则挂一个深度 8 的 ninja pool，全局共享这一个上限。
   **必须写在 `project()` 之前**，写到后面不生效（属性在 target 创建时初始化）。
   只限编译，不限链接。

2. `gradle.properties`：

   ```
   org.gradle.jvmargs=-Xmx2g -Dfile.encoding=UTF-8
   kotlin.daemon.jvmargs=-Xmx1536m
   ```

   给 ninja 腾出内存。本项目 Java/Kotlin 侧只有一个模块，2 GB 够用。

粗算峰值：8 × 约 1 GB + Gradle 2 GB + Kotlin 1.5 GB ≈ 12 GB，留有余量。

## 可选：直接减少 WSL 的核数

代价最大但最彻底的一刀。改 `C:\Users\<用户>\.wslconfig`：

```ini
[wsl2]
processors=8
```

然后 `wsl --shutdown`，重新进 WSL 生效。注意这会影响所有工具（不只是编译），
`nproc` 变成 8。仓库里的 pool 配置和它叠加时，取更小的那个。

调 `processors` 之前先看 `nproc`：如果已经是 8 或更少，就不用动它。

## 验证

改完 CMake 要重新 configure 才会重新生成 ninja 文件：

```
./gradlew :app:configureCMakeDebug[arm64-v8a]
grep -A1 '^pool compile' app/.cxx/Debug/*/arm64-v8a/CMakeFiles/rules.ninja
```

期望输出：

```
pool compile
  depth = 8
```

再看 `build.ninja` 里编译边带着 `pool = compile`，说明规则挂上了。

编译时另开一个终端看峰值：

```
while :; do free -m | awk '/Mem:/{print $3" MB used"}'; sleep 2; done
```

## 调大调小

- 内存还是紧 → 把 `compile=8` 改成 6 或 4；速度会线性变慢一点。
- 内存富裕、想更快 → 加到 12，前提是物理内存够。
- 改 `CMakeLists.txt` 里这两行不需要清 `.cxx`，重新 configure 即可。
- 改完仍 OOM，再考虑 `processors` 那条；两者都不行才需要加 swap。
