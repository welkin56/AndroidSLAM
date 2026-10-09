# 线程栈太小

**Android 上 pthread 的默认栈是 1 MB，而 PC 上跑同样代码用的是主线程（8 MB）。**
调用链一深就撞墙 —— **在 PC 上永远不会复现。**

发现：2026-09-28（阶段五 M5，真机第一次跑 ORB-SLAM3）。

---

## 症状

- **`SIGSEGV / SEGV_ACCERR`**（注意是 `ACCERR` 不是空指针）
- **出错地址是页对齐的**，而且紧挨着 `sp` 上方
- **tombstone 只有 1 帧栈**，展开器在第一帧就放弃
- 符号化落在**某个 STL 内部**（`vector` 构造之类）—— **这是误导**，
  深调用里第一个受害者而已
- **PC 上同一份代码跑几百次都没事**

## 这次的具体数字

```
sizeof(ORB_SLAM3::Frame)    = 149232 字节 = 145.9 KB
  └ 其中 mGrid[64][48] + mGridRight[64][48] = 147456 字节 = 144 KB（占 99%）
sizeof(ORB_SLAM3::Tracking) = 449600 字节 = 439.1 KB
```

而 `Tracking::GrabImageMonocular` 里写的是：

```cpp
mCurrentFrame = Frame(...);      // ← 这是个【临时对象】，在【栈】上，一次 146 KB
```

1 MB 的栈，减掉调用链本身的消耗，**不够放这一个临时对象**。

> 怎么量出来的：在 PC 上写个 5 行的程序 `#include "Frame.h"`，
> 把 `sizeof` 当数组长度，然后从 `.o` 的符号表里读 —— 不用跑真机。
> **成员大小和架构无关**，PC 上量的就是安卓上的。

## 修法

**显式指定栈大小。`std::thread` 没有这个接口，得用 `pthread`：**

```cpp
#include <pthread.h>

constexpr size_t kStack = 8 * 1024 * 1024;   // 和 PC 的主线程对齐

pthread_attr_t attr;
pthread_attr_init(&attr);
pthread_attr_setstacksize(&attr, kStack);
pthread_t t;
if (pthread_create(&t, &attr, workerEntry, &e) == 0) pthread_detach(t);
pthread_attr_destroy(&attr);
```

**8 MB 对齐 PC** 就够了 —— 那是同一份代码在 PC 上验证过能跑的配置。代价只是虚拟内存。

## 什么情况下还会撞上

- 任何「**在 native 线程上跑一长串 C++**」的地方，尤其是**栈上有大对象**的
- `sizeof` 大得离谱的类：里面有大数组（本例）、大 `std::array`、或按值传的大结构体
- **PC 上验证通过 ≠ 安卓上没问题** —— 只要那条路在 PC 上走的是主线程

## 教训

> 「在 PC 上好好的」这句话，在**线程栈**这一维上**不成立** ——
> 因为 PC 的主线程有 8 MB，而你起的 native 线程只有 1 MB。
> **只要代码被挪进 `std::thread` / `pthread_create`，就得重新想一遍栈。**

## 相关

- 那次完整的定位过程：[changes/2026-09-28-m5-slam-on-phone.md](../changes/2026-09-28-m5-slam-on-phone.md) §6
- 和它**同时发作、缺一不可**的另一条：[duplicated-static-lib-symbols.md](duplicated-static-lib-symbols.md)
