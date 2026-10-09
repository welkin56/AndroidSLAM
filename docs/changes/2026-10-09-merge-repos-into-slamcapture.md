# 2026-10-09 三个仓库并成一个：`SlamCapture` 变成唯一仓库 + 工作区根

一句话：**`docs/` 和 `tools/` 并进 `SlamCapture`，`SlamCapture` 从 `android/` 下面提出来当工作区根。
以后一个工作区、一个仓库。**

## 动机

一个项目摊在三处：

```
SLAMAPP/                     ← 不是仓库
├── docs/                    ← 仓库
├── android/SlamCapture/     ← 仓库
├── ORB_SLAM3/ opencv/ Pangolin/
├── tools/                   ← ⚠️ 完全没版本控制
├── CLAUDE.md swapClaude.sh  ← ⚠️ 完全没版本控制
└── dataset/ OpenCV-android-sdk/
```

**用户提的三件事**：

1. 「把 `android/SlamCapture` 作为工作区文件夹单独拿出来，不要 Android 那个目录」
2. 「把 docs 仓库删除，一起放进 `SlamCapture` 管理，tools 也是」
3. 「**我不想要 docs 那些 git 历史了，直接删去**」

顺带的目的：**上 GitHub 私有仓库时只需要推一个**。

`ORB_SLAM3/` **完全不动** —— 用户说的：「他只是源码仓库，我完全不动它，
需要的都搬进 thirdparty 了」。所以它留在仓库外。

## 改动

**目录结构**（全部是 `mv`，没有改文件内容）：

```
SLAMAPP/
├── SlamCapture/          ← 唯一仓库 + 工作区根
│   ├── CLAUDE.md         ← 从 SLAMAPP/ 搬进来，**重写**
│   ├── .gitignore        ← 改了（见下）
│   ├── docs/             ← 从 SLAMAPP/docs 搬进来，`.git` 删掉
│   ├── tools/            ← 从 SLAMAPP/tools 搬进来
│   └── app/  gradle*/ …
├── ORB_SLAM3/ opencv/ Pangolin/ OpenCV-android-sdk/ dataset/
└── slamapp-docs-git-2026-10-09.tar.gz   ← 合并前 docs 的历史备份（754 KB）
```

**内容改动（4 类）**：

| 改什么 | 处数 | 怎么改 |
|---|---|---|
| `docs/*.md` 里指向**仓库外**的链接 | **31** | **删掉 `](...)`，留文字** |
| `docs/README.md` 的 `](../android/SlamCapture/)` | 1 | 改目标 → `](../)`，并**加一段说明源码副本在哪** |
| `.gitignore` | — | 裸模式改锚定 + 加 Python 产物 |
| `CLAUDE.md` | 整个文件 | **重写**（用户要求「不要那么啰嗦解释，只要写怎么做」） |

## 决策

### 一、链接**不做重定向，直接删**（用户提的）

我原来的方案是把 28 条 `../../ORB_SLAM3/src/*.cc#L*` 重定向到仓库内的副本
`app/src/main/cpp/thirdparty/orb_slam3/src/`。**用户否了**：

> 「链接能删掉吗，别链接了，**别人也看不了**」

**对** —— 指向仓库外的相对链接对别人**必然是断的**，重定向到 `thirdparty/` 也只是勉强可用。
而且**链接的显示文字里本来就带着行号**（`[Tracking.cc:2019](...)`），
**删掉 `](...)` 什么都不丢**：

```
[Tracking.cc:2019](../../ORB_SLAM3/src/Tracking.cc#L2019)   →   Tracking.cc:2019
```

**顺带的好处：不用再核「两份源码的行号一致不一致」** —— 那个校验本来是个隐患。

**代价**：`ORB_SLAM3/Examples/` 那两条（`phone.yaml`、`mono_inertial_euroc.cc`）
本来就没法重定向（`thirdparty/` 里只有 `src/` 和 `include/`），删链接反而一致。
**在 `docs/README.md` 补了一段**说明「源码引用指上游，本仓库的副本在 `thirdparty/orb_slam3/`」——
**补上「不链接」丢掉的指路作用。**

### 二、⚠️ **否决：用 `git subtree` 保 docs 的历史**

我原本建议 `git subtree add`（保住那 16 个提交）。**用户明确不要**：

> 「我不想要 docs 那些 git 历史了，直接删去」

**⚠️ 但删之前还是打了备份**（`SLAMAPP/slamapp-docs-git-2026-10-09.tar.gz`，754 KB）——
删 `.git` 不可逆，而这一秒的代价换掉「万一以后要追溯」的风险。
**备份放在 `SLAMAPP/` 下而不是 `~/`**（用户要求：「你把备份的东西挪到 SLAMAPP 目录不要乱放」）——
放在 `SlamCapture/` **外面**，所以不会进仓库。

### 三、⚠️ 搬 `tools/` 时**踩了一个坑**：嵌套成了 `tools/tools/`

`SlamCapture` 仓库**本来就有个 `tools/`**（`push-vocab.sh`、`setup-boost-android.sh`，已跟踪）。
`mv /home/welkin/SLAMAPP/tools SlamCapture/tools` 于是把标定目录塞进了那个已有的 `tools/` 里
→ **`tools/tools/calibration/`**。

**是 `git status --ignored` 里那行 `tools/tools/calibration/__pycache__/` 暴露的** ——
一个目录名里出现两次「tools」，很扎眼。

**修法**：`mv tools/tools/calibration ./ && rmdir tools/tools`。
**修完之后文档里那两条 `../../tools/calibration/...` 链接正好能解析**（它们本来就假设
`tools/` 在仓库根下）。**计划里漏了「目标目录已存在」这一条** —— 是这次唯一一处返工。

### 四、`CLAUDE.md` 重写（用户要求）

> 「重新写整个 CLAUDE.md，不要那么啰嗦解释，只要写怎么做就好了，
> 以后工作目录只在 SlamCapture 不会看到其他东西」

从 ~170 行压到 ~105 行。**删掉的**：每个约定的长段「为什么」、
「三个容易踩的点」那一节（`ORB_SLAM3/` 不在工作区里，不用反复讲）。

**改掉的两处事实**：

1. 目录导览 → 只描述 `SlamCapture/` 里面的；仓库外的东西**一行带过**。
2. ⚠️ **删掉了这条旧理由**：「改动记录放在 `docs/` 而不是 android 仓库里，
   是因为它要覆盖 C++ 那边的改动」—— **合并后这个理由不成立了**。
   换成明说：**改动记录只覆盖 `SlamCapture` 这个仓库**，
   `ORB_SLAM3/` 在仓库外、以后动它没有记录。
   （App 侧的 C++ 是 `thirdparty/orb_slam3/` 那份副本，**在仓库里，照常有记录**。）

### 五、`.gitignore` 的裸模式要锚定

原来写的是裸的 `build/` 和 `captures/` —— **裸模式匹配任意深度**。
以前只有 App 一个目录无所谓，现在仓库里多了 `docs/` 和 `tools/`，容易误伤。

```diff
-build/            →  /build/
                   +  /app/build/
-captures/         →  /captures/
                   +  __pycache__/
                   +  *.pyc
```

**改完立刻验了**：`git status` 里 `app/build` 下 **0 项**（没把 1.4 GB 构建产物放进来）。

### 六、⚠️ 搬目录**弄坏了 OpenCV 的默认路径**（第二个搬运坑，**编译时才暴露**）

`CMakeLists.txt` 里 OpenCV 的默认位置是**往上数 6 级**：

```cmake
get_filename_component(_opencv_default ${CPP_DIR}/../../../../../../OpenCV-android-sdk ABSOLUTE)
```

从 `app/src/main/cpp` 往上 6 级：

| | 6 级落到哪 | `OpenCV-android-sdk` 在不在 |
|---|---|---|
| **搬之前** `SLAMAPP/android/SlamCapture/…` | `SLAMAPP/` | ✓ |
| **搬之后** `SLAMAPP/SlamCapture/…` | `/home/welkin/` | ✗ **少了 `android/` 那一层** |

**编译直接报 `find_package(OpenCV) ... did not find one`。**

**光改级数还不够** —— 用户提了更好的做法：

> 「这东西放到 thirdparty 里面吧，以后别人要用直接丢进 thirdparty 里」

对。**把 872 MB 的 SDK 整个搬进 `thirdparty/`**，默认路径改成 `${TP_DIR}/OpenCV-android-sdk`。
好处是**路径自解释、换机器不用配变量** —— 别人下载完丢进去就能编。

**⚠️ 但还有个陷阱**：`OPENCV_ANDROID_SDK` 是 `set(... CACHE PATH ...)`，
`app/.cxx/` 里**存着旧路径**。**光改默认值不生效**，必须 `rm -rf app/.cxx` 让 CMake 重来。

> **教训：搬仓库时「往上数几级」的路径全会失效，而它们只在编译时才报。**
> 全仓库扫过，只有这一处是 6 级。

### 七、`CLAUDE.md` 入库

**建议入库**（它是项目文档、无敏感内容），这样 `docs/` 里那两条指向它的链接对别人也有效。
用户之前定的「不走版本控制」是因为**那时根目录不是仓库**，现在条件变了。
**最终由用户的 `git add` 决定。**

## 验证

**1. 两个 `.git` 确实分开的**（删之前核过，怕误删）

```
SlamCapture/.git   16 个提交   HEAD = d0a0c8a feat:测距加支持度、深度锁定和光轴锁定
docs/.git          16 个提交   HEAD = 20ebea2 测距功能实现
```

**2. 全仓库相对链接：210 条，断的 0 条** ⭐

```
$ 脚本扫 SlamCapture/**/*.md + CLAUDE.md，逐条检查目标文件是否存在
  ✅ 210 条真链接，断的 0 条
```

> ⚠️ **扫描器本身踩过一次**：第一版**不跳过代码块和行内代码**，
> 于是把改动记录里**举例用的链接语法**（`` `](../android/...)` `` 这种）当成真链接，
> 报了 14 条「断链」—— 其中 9 条还是 **OpenCV SDK 自带的 license 文档**里的（搬进仓库后才被扫到）。
> **跳过代码、排除 `thirdparty/OpenCV-android-sdk/` 之后才是真数：0 条。**
> 教训：**「验证工具报错」先怀疑工具**。

**3. 没有残留的仓库外链接**

```
$ grep -rnE '\]\(\.\./\.\./(ORB_SLAM3|android)/' --include='*.md' .
  ✓ 无
```

**4. `git status` 干净**（没把构建产物放进来）

```
 M .gitignore
?? CLAUDE.md
?? docs/
?? tools/calibration/
   app/build 下 0 项
```

（`tools/` 下那两个 `.sh` 是**原本就跟踪的**，只有 `calibration/` 是新加的。）

**5. 编译通过**（清 `.cxx` 全量重编）

```
$ rm -rf app/.cxx && ./gradlew assembleDebug
BUILD SUCCESSFUL in 1m 16s
$ ls -lh app/build/outputs/apk/debug/app-debug.apk   →  14M
$ llvm-nm -D libslam_jni.so | grep -c Java_..._SlamBridge   →  15（没少）
$ llvm-nm -D --defined-only libslam_jni.so | grep -c '_ZN2cv'  →  0（OpenCV 边界仍然干净）
```

**6. 未验证**

- ⚠️ **PC 侧工作流没跑过。** `tools/` 的路径变了（`SLAMAPP/tools` → `SlamCapture/tools`），
  `mono_inertial_euroc` 的调用路径也变了。**脚本里有没有写死的相对路径，没查。**
- **`.vscode/`** 是 `cp -a` 过去的（原地那份**没删**）—— 工作区确认换过去之后要删旧的。

## 遗留

- ⚠️ **改动记录不再覆盖 C++ 侧**（见「决策」四）。
- ⚠️ **docs 的 16 个提交没了**（用户要求）。备份在
  `SLAMAPP/slamapp-docs-git-2026-10-09.tar.gz`。**以后追溯只能靠文件内容。**
- ⚠️ **源码引用的行号**（`Tracking.cc:2019` 这种）是对着**上游**核的；
  仓库内副本 `thirdparty/orb_slam3/` 的行号**理论上一致，但没重新核过**。
  现在没有超链接了，所以最多是「读者对照时偏几行」。
- **`swapClaude.sh`** 留在 `SLAMAPP/` 没动。
- **`dataset/`、`OpenCV-android-sdk/`** 也留在外面 —— 上 GitHub 时不推（GitHub 单文件 100 MB 硬限，
  `ORB_SLAM3/` 还有 1.4 GB 的本上游数据集）。

## 回滚

**搬目录那一步之后、删 `.git` 之前**：`mv` 回去 + `rmdir` 就行，两个仓库和历史都没动。
**删 `.git` 之后**：docs 的历史只能从备份解回去：

```bash
mkdir -p /tmp/docsrestore && tar xzf SLAMAPP/slamapp-docs-git-2026-10-09.tar.gz -C /tmp/docsrestore
```

**内容改动**（链接、`.gitignore`、`CLAUDE.md`）单独回滚即可 —— 它们和搬目录无关。
