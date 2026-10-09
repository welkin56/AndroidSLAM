# 静态库被链进多个 `.so`

**同一个静态库（`.a`）被链进两个不同的 `.so`，进程里就有两份它的代码
—— 包括它的全局状态、静态单例、线程池。**

发现：2026-09-28（阶段五 M5，真机第一次跑 ORB-SLAM3）。

---

## 症状

- **`SIGSEGV`**，崩在某个第三方库的函数里
- 那个函数**看起来完全无辜** —— 这次是 `cv::parallel_for_`，一个内部循环工具
- **PC 上同一份代码跑几百次都没事**（PC 只有一份，因为只链了一个 `.so`）
- 调用链是「A 库调进去、执行到 B 库那份实现」

## 这次的具体形状

```
libslam_jni.so
 ├── NEEDED liborb_slam3.so   ← 静态链了一份完整 OpenCV
 └── NEEDED libDBoW2.so       ← 【也】静态链了一份完整 OpenCV  ← 病灶
```

`DBoW2` 上游做成了独立的 `.so`，而它 `target_link_libraries(DBoW2 ${OpenCV_LIBS})`
链的是**静态** OpenCV。于是 `libDBoW2.so` 里有一份 `cv::`，`liborb_slam3.so` 里**也有一份**。

两份里都有 **`cv::parallel_for_`** —— 那是 OpenCV 的**并行后端分发点**，
背后挂着**带全局状态**的并行引擎：

```
Frame::ExtractORB → ORBextractor → cv::GaussianBlur → cv::parallel_for_
                                                       ↑ 动态链接器选中了 libDBoW2.so 那一份
```

两个副本的全局状态互相踩 —— 崩。

### ⚠️ 那份全局状态是 TBB 的，不是 OpenCV 自己的线程池

> 这是 2026-09-29 复查时纠正的一处**归属错误** —— 原先这里写的是
> 「OpenCV 里带**全局线程池**的东西」。结论没错，归属错了。

这份 OpenCV 4.6.0（Android SDK 预编译）**编译时开了 TBB 后端**。凭据：

```
libopencv_core.a → parallel.cpp.o
  T _ZN2cv13parallel_for_...                              ← 定义
  U _ZN2cv8parallel24createParallelBackendTBBEv           ← 引用 TBB 后端
  U _ZN3tbb10interface78internal15task_arena_base...      ← 引用 TBB 本体
```

而 OpenCV 源码里这一行是**有保护的**：

```cpp
// modules/core/src/parallel/registry_parallel.impl.hpp
#ifdef HAVE_TBB
    DECLARE_STATIC_BACKEND("TBB", createParallelBackendTBB)
```

**能看到这个 `U` 引用 ⇒ `HAVE_TBB` 当时是定义的。** 而且 TBB 的代码确实进了最终产物 ——
`liborb_slam3.so` 里有 **16 个导出的 TBB 全局状态**（`market::theMarket`、
`governor::theTLS`、`__TBB_InitOnce::count` …）。

反过来，OpenCV 自己的 `ThreadPool` —— **在最终 `.so` 里连符号名都没有**
（`ThreadPool::instance()` 在静态库里是 `W`，被内联进各调用点后被回收了）。

## 为什么 PC 上不出现

**因为 PC 上两个 `.so` 对 OpenCV 都只是「引用」，没有一个是「自带」。**

```
PC                                              Android
/usr/local/lib/libopencv_core.so.406            sdk/native/staticlibs/.../libopencv_core.a
   ↑ NEEDED        ↑ NEEDED                          ↑ 抽      ↑ 抽
libDBoW2.so   libORB_SLAM3.so                   libDBoW2.so  liborb_slam3.so
   └────── 同一个 soname，ld.so 去重 ──────┘       └── 各自往自己代码段里塞一份 ──┘
        进程里 1 份 cv:: ⟹ 没事                        进程里 2 份 cv:: ⟹ 崩
```

实测（2026-09-29）：

| | PC | Android |
|---|---|---|
| OpenCV 形态 | `/usr/local/lib/libopencv_core.so.406`（源码编的，默认动态） | `sdk/native/staticlibs/arm64-v8a/libopencv_core.a` |
| `libDBoW2` 的 cv:: | 定义 **0**，未定义引用 **5** | 自带一份 |
| `libORB_SLAM3` 的 cv:: | 定义 **6**，未定义 **58** | 定义 **1229** |
| `readelf -d <so> \| grep NEEDED` | **有一串 opencv** | **一个都没有** |

**`NEEDED` 那一行就能分开这两种形态**：有 opencv = 引用别人；一个都没有 = 自己带了。

### ⚠️ PC 那 6 个定义全是弱符号

```
W _ZN2cv3Mat2atIfEERT_i      ← 模板实例化
W _ZN2cv7MatExprD1Ev         ← inline 析构
W _ZN2cv9BFMatcherD0Ev
```

**弱符号天然允许在多个 `.so` 里重复，而且它们不带全局状态 —— 重复无害。**
所以 PC 上「有 6 个 cv:: 定义」不等于「有这个问题」。

**这同时收窄了判据：出事的是「强定义（`T`）+ 带全局状态」的符号重复，
不是符号名重复本身。** Android 那 1229 里 1168 个是 `T`，`parallel_for_` 正是其中一个 `T`
（另一个是 `W` 的弱符号，不用管）。

### 推论：这不是「Android 特有」

是「**静态链接 OpenCV** 特有」。PC 上换静态 OpenCV 编，一样崩。

Android 之所以更容易撞：**OpenCV 官方 Android SDK 对 native C++ 只提供 `.a`**
（`sdk/native/libs/<abi>/` 里只有一个 `libopencv_java4.so`，那是给 Java API 的整体打包，
没有 per-module 的 `.so` 可链）。**不是我们选错了路，是这条路只有一条。**

## 怎么查出来

**数符号。** 这是决定性的、不用跑设备的判据：

```bash
NM=$(find ~/Android/Sdk/ndk -name llvm-nm | head -1)
for so in app/build/**/*.so; do
  printf "%-22s cv:: 符号 %s 个  parallel_for_ %s 处\n" "$(basename $so)" \
    "$("$NM" -D --defined-only "$so" | grep -c '_ZN2cv')" \
    "$("$NM" -D --defined-only "$so" | grep -c '_ZN2cv13parallel_for_')"
done
```

**同一个名字在多个 `.so` 里同时出现 ⇒ 就有这个问题。**

顺带两条：
- `llvm-nm -D --defined-only` 数的是**定义**（`T`），不是引用（`U`）
- 用**未 strip** 的 `.so`（在 `app/build/intermediates/cxx/<类型>/<哈希>/obj/<abi>/`）

**只看强定义**（跳过弱符号的噪声）—— 慌的时候用这条：

```bash
"$NM" -D --defined-only "$so" | grep '_ZN2cv' | grep -c ' T '
```

## 为什么工具链不报错

**因为「跨 `.so` 的符号重复」不在任何一层工具的职责范围内。**
逐层看，每层都恰好管不着：

**1. 编译期看不见「库」。** 编译只处理单个 `.cpp`。`.a` 只是 `.o` 的打包，里面
**没有任何元数据**记录「这些符号来自 OpenCV」。编 OpenCV 的时候更不知道「这份会被链两次」。

**2. 两次链接是彼此独立的两条命令。** `libDBoW2.so` 和 `liborb_slam3.so` 各是一次独立的
链接器调用。**重复定义检查只发生在一次链接内部** —— 没有任何一刻链接器同时看到两份。

**3. 静态库是「按需抽取」，不是整块塞。** 链接器只抽**当前未定义符号**所需的那些 `.o`
（加依赖闭包）。两次抽出来的本来就不是同一批 —— 这次 DBoW2 要 5 个，ORB-SLAM3 要几百个。
**各自抽，互不知情。**

**4. 同一个 `.a` 在一条命令行上写两遍，也不报错。** 这点最能说明问题：第二次遇到同一个归档，
链接器发现里面的符号**已经定义过了**，于是一个成员都不抽，安静跳过。
**「同一个 `.a` 出现两次」在链接器眼里完全合法** —— 我们这是同源副本，更不会报。

**5. `ld.so` 不但不报错，还刻意支持。** 同名符号在多个 `.so` 里有定义时，
它取**搜索顺序里第一个** —— 这叫 **symbol interposition**，是 `LD_PRELOAD`、
`malloc` hook 赖以工作的机制。**是特性，不是错误。**
所以 `parallel_for_` 被解析到 `libDBoW2.so` 那一份，全程零警告。

`-Wl,--no-undefined` 这类开关也抓不住：**符号是定义的，正是这一点让链接成功。
缺符号会报错，多符号不会。**

> **这是个跨二进制的、只在运行时才成形的图论问题，工具链里没有哪一层为它负责。**
> 所以判据只能是人肉数符号。
> **「链接通过了」在这个问题上不构成任何保证。**

## 修法（三层，缺一不可）

1. **让那个静态库只进一个 `.so`。** 本例：`DBoW2` 从 `SHARED` 改成 `STATIC` ——
   它的代码进 `liborb_slam3.so`，不再单独成库。
2. **把链接签名改成 `PRIVATE`。** ⚠️ **静态库的 `PRIVATE` 链接项照样会传递**
   （CMake 标成 `LINK_ONLY`），所以在库自己那一层挡不住 —— **得在消费它的那一层断**。
3. **让不该碰它的 target 一个符号都不链**，只给**头文件路径**。
   （`target_include_directories(... PRIVATE ${OpenCV_INCLUDE_DIRS})` ——
   头文件不产生符号引用，链了才会。）

### 目标状态

**修好之后**的实测（2026-09-29）：

| `.so` | cv:: **定义** | cv:: **未定义引用** | NEEDED 里有 opencv | `parallel_for_` |
|---|---|---|---|---|
| `liborb_slam3.so` | **1229**（1168 `T` + 60 `W` + 1 `B`） | **0** | 没有 ✓ | **1 处** |
| `libslam_jni.so` | 0 | **0** | 没有 ✓ | 0 |
| `libg2o.so` | 0 | 0 | 没有 ✓ | 0 |
| `libDBoW2.so` | — | — | — | **不再产出** |

### ⭐ 更利落的判据：看 `未定义引用` 是不是 0

上表 `liborb_slam3.so` 那一行的**组合**才是关键：

> **定义 1229 个 ＋ 未定义引用 0 个**

**「有定义、又零未定义引用」= 它自带一整份、完全不依赖外面 —— 这正是静态链接的指纹。**
反过来，PC 侧那份是「定义 6 个＋未定义 58 个」，一眼就是「在用别人的」。

这条比数定义个数省事，也**不需要知道里面有没有 TBB**：

```bash
# 谁是「自给自足」的（自带一整份）？
for so in *.so; do
  d=$("$NM" -D --defined-only "$so" | grep -c '_ZN2cv')
  u=$("$NM" -D "$so" | grep '_ZN2cv' | grep -c ' U ')
  [ "$d" -gt 0 ] && [ "$u" -eq 0 ] && echo "自带一份: $so"
done
# 输出两个以上 ⇒ 就是这个问题
```

`libg2o.so` 和 `libslam_jni.so` 是**定义 0、未定义 0** —— 它们是**真的不碰 OpenCV**，
不是「碰了但被消掉」，所以不用担心。

## ⚠️ 我在这件事上判断错过一次

早先（阶段五 M3）就发现 `libslam_jni.so` 里凭空多出 170 个 `cv::` 符号，
当时的结论是：

> 两份是同一份静态库编出来的，逐字节相同，所以**只是体积问题、不是正确性问题**。

**「逐字节相同」恰恰不是安慰 —— 它保证了两份的状态一模一样地各自演化。**
带全局状态的函数（线程池、单例、缓存）只要被两边各调一次，就一定互相踩。

**教训：「两份相同的代码」在多个 `.so` 里不是冗余，是 bug。**

## 什么情况下还会撞上

- 任何**被多个 `.so` 静态链接**的第三方库
- 尤其是**自己有全局状态**的：线程池（OpenCV / TBB / OpenMP）、
  单例、日志、内存分配器、随机数
- 链接时**不会报任何错** —— 两份都成功链上，问题只在运行时发作

**所以：每次加了新的 `.so`，都值得数一遍符号。**

## 相关

- 那次完整的定位过程：[changes/2026-09-28-m5-slam-on-phone.md](../changes/2026-09-28-m5-slam-on-phone.md) §7
- 和它**同时发作、缺一不可**的另一条：[thread-stack-too-small.md](thread-stack-too-small.md)
