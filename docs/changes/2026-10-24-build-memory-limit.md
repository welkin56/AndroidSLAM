# 2026-10-24 限制 NDK 并行编译数，压 installDebug 的内存峰值

## 动机

`./gradlew installDebug` 把内存占满（15 GB 物理 + 8 GB swap）。用户问能不能把线程数调小。

## 改动

- `app/src/main/cpp/CMakeLists.txt` —— 在 `project()` 之前加两行：

  ```cmake
  set(CMAKE_JOB_POOLS compile=8)
  set(CMAKE_JOB_POOL_COMPILE compile)
  ```

  所有 target 的编译边挂同一个深度 8 的 ninja pool。

- `gradle.properties` —— `org.gradle.jvmargs` 从 `-Xmx4g` 降到 `-Xmx2g`，新增
  `kotlin.daemon.jvmargs=-Xmx1536m`。

- `docs/tutorial/BUILD_MEMORY.md` —— 成因、现在的配置、可选的 `.wslconfig` 改法、验证命令。

- `app/src/main/cpp/CMakeLists.txt` 的其余注释顺手按同一风格收敛：去掉 `⚠️`、`**`、反引号、
  阶段编号和真机崩溃经过，只留「这是什么 / 必须这么写的后果」。
  剥离注释后的代码与 `HEAD` 逐行一致，唯一新增的代码就是上面那两行 `set()`。

## 决策

- **采用：CMake job pool，而不是改 `.wslconfig` 的 `processors`。** 后者影响 WSL 里所有工具，
  且要重启 WSL 才生效；job pool 只作用于本项目编译，`processors` 留原样。
- **采用：只限编译、不限链接。** 链接是串行的、峰值远低于同时跑多个 `-O3` 的 clang。
  加 link pool 只会拖慢。
- **采用：Gradle 堆从 4g 降到 2g。** Java/Kotlin 侧只有 `app` 一个模块，4g 是 AGP 的通用建议值，
  这个项目用不到。省下来的两 G 正好给 clang。
- **否决：`org.gradle.parallel=false`。** 单模块项目，跨模块并行本来就不存在，关掉只有坏处。
- **否决：给每个 target 单独 `set_target_properties(... JOB_POOL_COMPILE ...)`。**
  ORB-SLAM3 / g2o / DBoW2 加起来几十个 target，逐个写既容易漏，也和上游 diff 变大。
  `CMAKE_JOB_POOL_COMPILE` 会被目录作用域继承，顶层写一次就够。
- **否决：用 `-j` 前缀包装 ninja。** AGP 的构建命令来自 CMake 生成的
  `buildTargetsCommand`（`ninja -C <dir> {targets}`），没有可注入参数的入口。

## 验证

**未验证：未执行 Gradle / NDK 构建**（按约定编译类步骤由用户执行）。

已单独用仓库里的 CMake 3.22.1 在临时工程上验证这两个变量的行为：

- 变量写在 `project()` **之前** → `CMakeFiles/rules.ninja` 出现 `pool compile` + `depth = N`，
  编译边带 `pool = compile`。
- 写在 `project()` **之后** → 不生成 pool。

在本仓库的确认方式（用户执行）：

```
./gradlew :app:configureCMakeDebug[arm64-v8a]
grep -A1 '^pool compile' app/.cxx/Debug/*/arm64-v8a/CMakeFiles/rules.ninja
```

## 遗留

- 8 是按「15 GB 内存 + 单个 .cc 峰值约 1 GB」估的，没有实测峰值。如果仍然 OOM，
  先降到 6 或 4，再考虑 `.wslconfig` 的 `processors`。
- 首次全量编译的耗时没对比过。

## 回滚

```
git checkout -- app/src/main/cpp/CMakeLists.txt gradle.properties
rm docs/tutorial/BUILD_MEMORY.md docs/changes/2026-10-24-build-memory-limit.md
```

删记录后记得一并删掉 `docs/changes/README.md` 和 `docs/README.md` 里对应的两行。
