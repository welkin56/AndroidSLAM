# 2026-11-06 release 内置词袋并配置签名

## 动机

仓库要 public，并且要发 release APK 给别人装。现在别人下载源码后必须自己下 139 MB
的词袋、再跑 `tools/push-vocab.sh` 用 adb 推上去才能启动——这对「只想装个 APK 试一下」
的人是硬门槛。同时 `build.gradle.kts` 里 `release` 没有 `signingConfig`，`assembleRelease`
产出的是 `app-release-unsigned.apk`，手机装不上。

目标：release 包自带词袋，首次启动自己解压；签名配好，`./gradlew :app:assembleRelease`
直接出可安装包。

## 改动

- `app/build.gradle.kts`
  - `signingConfigs` 加 `release`，材料从 `keystore.properties` 读。**必须写在 `buildTypes`
    之前**：`buildTypes.release` 里用 `signingConfigs.getByName("release")`，Kotlin DSL 是
    顺序执行的，块在后面会取不到名字。文件缺失时不创建这个 config，release 仍能构建（产
    unsigned 包），debug 完全不受影响。
  - `sourceSets.release.assets.srcDir("src/release/assets")`：词袋只进 release。放
    `src/main/assets` 的话 debug 每次改一行代码都要重打 139 MB。
  - `mergeReleaseAssets` 加前置校验：词袋缺失或小于 50 MB 就 `throw GradleException`。
    没有这道闸，忘放词袋会静默产出一个「装完提示你去跑 push 脚本」的包，发出去才发现。
    用 `tasks.matching` 而不是 `tasks.named`，任务名随 AGP 版本变时不至于让整份脚本解析失败。

- `app/src/main/java/com/example/slamcapture/MainActivity.kt`
  - 删 `resolveVocabPath()`，换成 `ensureVocabThenStart()`：先查
    `getExternalFilesDir()/ORBvoc.txt`（push 脚本推的就是这里，所以两条路共用一个判断），
    没有再回退到 assets。
  - `unpackVocab()` 在后台线程流式复制，先写 `.part` 再改名。**中途被杀不能留下半个
    `ORBvoc.txt`**——那种文件会被下次启动当成可用，直接交给 `DBoW2::loadFromTextFile`
    读到半截再崩在 native 里，比明确报错难查得多。
  - 解压期间 `slamState = SLAM_LOADING` + `vocabPreparing = true`，状态栏显示已写多少 MB。
    每 8 MB 刷一次界面：139 MB 按 1 MB 刷会有 139 次 `render()`，其中绝大多数用户看不出差别。
  - `startSystem(vocabPath)` 抽出原来的 `resolveSettingsPath()` + `nativeInit`，让同步和
    异步两条路复用。

- `tools/prepare-release-vocab.sh` — 新增。把词袋复制到 `app/src/release/assets/`。
  已存在且大小一致就跳过（139 MB 的 `cp` 要几秒）。

- `keystore.properties.example` — 新增。签名材料的格式说明，真实文件与 `.jks` 都在
  `.gitignore` 里。

- `.gitignore` — 加 `*.jks` / `*.keystore` / `keystore.properties` 和
  `app/src/release/assets/ORBvoc.txt`。

- `README.md` — 「词袋不进 APK」改成「不进源码仓库，debug 推、release 内置」。

## 决策

- **采用：release 专用 assets，不把词袋放 `src/main/assets`。** debug 是日常构建，
  不应该为一次性的发布动作付出每次打包 139 MB 的代价。

- **采用：解压到 `getExternalFilesDir()`，而不是源码里读流。** `DBoW2` 这个版本的
  `loadFromTextFile` 只接受文件路径（没有 `loadFromBinaryFile`，也没有 istream 重载），
  喂不了 assets 流。而解压目标选外部私有目录，是因为 `push-vocab.sh` 推的就是那里——
  同一个判断既能命中打包过的包，也能命中手动推过文件的 debug 包，不必分两条路。

- **否决：assets 里放 gzip 后的 `ORBvoc.txt.gz`。** APK 本身是 zip，明文 txt 已经被压到
  约 45 MB，gzip 一层省不下多少，反而多一次解压。

- **否决：升级 DBoW2 支持二进制词袋（43 MB）。** 能再省约 25 MB，但要改第三方库并重新
  验证加载路径。为 25 MB 动 SLAM 内核不值得。

- **否决：`nativeInit` 直接读 assets。** 需要在 C++ 侧拿 `AAssetManager`，还要给
  `DBoW2` 加流式重载。目前这条路只在 Java 侧多 60 行就能解决。

- **否决：解压时显示确定进度条。** 需要总量，而 `AssetManager.openFd()` 对压缩存储的
  asset 会抛异常（拿不到未压缩长度）。显示已写 MB 数已经够判断「有没有在动」。

## 验证

**未验证。** 本机 `wsl.exe` 被安全策略拦住，无法编译。

要看的：

| 项 | 期望 |
|---|---|
| `./tools/prepare-release-vocab.sh` | 输出复制的文件大小 139 MB 量级 |
| `./gradlew :app:assembleRelease` | 产出 `app-release.apk`（不是 `-unsigned`），体积约 105 MB |
| 首次启动 | 状态栏「正在解压词袋… 已写 N MB」，5 秒内完成，之后进入构造 |
| 第二次启动 | 不再出现解压提示（文件已存在） |
| `adb shell ls -l /sdcard/Android/data/com.example.slamcapture/files/` | 有 `ORBvoc.txt`，无 `.part` |
| 故意删掉 release assets 里的词袋再打 | `mergeReleaseAssets` 报 GradleException，提示跑 prepare 脚本 |
| debug 构建 | 体积、行为与改动前一致 |

## 遗留

- **debug 包仍然要 `push-vocab.sh`。** 这是刻意的（见「决策」第一条），但意味着
  `assembleDebug` 出来的包给别人装仍然要 adb 推词袋。
- **首次解压期间切后台的边界。** 解压完成后会检查 `isFinishing || isDestroyed`，但没检查
  「Activity 在后台但没销毁」——那种情况会继续构造 System（`nativeInit` 本身是异步的，
  不阻塞主线程），用户切回来直接是就绪态。没实测过，属可接受行为。
- **`versionCode`/`versionName` 没动**，还是 1 / "1.0"。第一次发布合适，以后每次发版
  记得手动加。
- **没配 CI。** `.jks` 在本地，GitHub Actions 里要另存 secrets。

## 回滚

1. 删 `app/src/release/assets/`（词袋副本），release 包回到「需要 push 脚本」的状态。
2. `MainActivity.kt`：`ensureVocabThenStart()` 换回同步的 `resolveVocabPath()` 即可，
   `startSystem()` 保持不动。
3. `build.gradle.kts`：删 `signingConfigs`、`sourceSets`、末尾的校验任务。
4. `.gitignore` / `README.md` / `tools/prepare-release-vocab.sh` 可直接删。
