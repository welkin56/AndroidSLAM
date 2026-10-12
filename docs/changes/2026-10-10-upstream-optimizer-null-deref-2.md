# 2026-10-10 上游又一个「检测到 → 打印 → 照样用」（`mpImuPreintegrated`）

一句话：**ORB-SLAM3 里第二处同源崩溃。检测到 `mpImuPreintegrated` 是空的，
打印一句 `Not preintegrated measurement`，然后下一行直接解引用它。**

> 第一处是 [2026-10-08 那份](2026-10-08-upstream-optimizer-null-deref.md)（g2o 顶点那处）。
> **同一个模子，不同的地方。** 两处都已修。

## 动机

真机跑「吸附」测试时闪退。tombstone：

```
SIGSEGV, fault addr 0x8a8, Cause: null pointer dereference
  #00  pthread_mutex_lock
  #01  std::mutex::lock()
  #02  IMU::Preintegrated::SetNewBias(Bias const&)     ← 在一个野指针上加锁
  #03  Optimizer::InertialOptimization(...)
  #04  LocalMapping::InitializeIMU(...)
  #05  LocalMapping::Run()
```

**崩溃前两行日志**（就是线索）：

```
19:49:07.651   Empty IMU measurements vector!!!
19:49:08.200   Not preintegrated measurement
19:49:09.474   崩
```

**⟹ 有帧没有 IMU ⟹ 它的 `mpImuPreintegrated` 是空的 ⟹ 上游拿它去 `SetNewBias` ⟹ 崩。**

## 改动

**`thirdparty/orb_slam3/src/Optimizer.cc`**（`InertialOptimization` 的多参数重载）：

```cpp
if(!pKFi->mpImuPreintegrated)
    std::cout << "Not preintegrated measurement" << std::endl;   // ← 检测到了

pKFi->mpImuPreintegrated->SetNewBias(...);                        // ← 然后照样解引用 ✗

// 改成：
if(!pKFi->mpImuPreintegrated)
    continue;                                    // 没有预积分就不参与惯性优化
```

**`thirdparty/README.md`** 改动表加**第 11 条** ✓ 并写明「和第 10 条同源」。

## 决策

### 一、和第一处是**同一个模子**

| | 检测到 | 然后 |
|---|---|---|
| **第 10 条** | `if(!VP1 \|\| !VV1 \|\| …)` | `PrintMess("Error" + VP1->id() + …)` ✗ |
| **第 11 条** | `if(!pKFi->mpImuPreintegrated)` | `setNewBias(...)` ✗ |

**⟹ ORB-SLAM3 里「检测到 → 打印 → 照样用」是**一类**写法** ✓ **不是孤例。**

### 二、为什么 `continue` 是对的，而且**只跳过 `SetNewBias` 不够**

**语义上**：没有预积分就没法参与惯性优化 ——
**和上面几行按 `isBad()` / `mnId` 过滤是同一套逻辑** ✓

**⚠️ 只跳 `SetNewBias` 不行** —— **下面几行还会用它**
（`new EdgeInertialGS(pKFi->mpImuPreintegrated)`）✗ **不跳过整个关键帧还是会崩** ✓

### 三、⚠️ **还有三处同源，没修**（用户决定「下次崩了再说」）

`Optimizer.cc` 里三处 `new EdgeInertialGS(pKFi->mpImuPreintegrated)` ✗
**顶点非空 ≠ 预积分非空** —— 所以那三处**已有的顶点空检查护不住它** ✗

**已写进 `thirdparty/README.md` 第 11 条**：「崩了就来这儿看」✓

**⚠️ 所以「以后不会再崩」这话不能说** —— 同一个触发条件再来，**那三处还是会崩**，只是位置不同。

## 验证

**1. 编译通过** ✓（`Optimizer.cc` 只动了 **4 行插入 / 1 行删除**）

**2. 真机跑了一趟，没崩** ✓

```
$ adb logcat -b crash -d | grep -c SIGSEGV   →  0
```

**3. 未验证**

- **那三处同源的没修**，**触发条件仍在**（`Empty IMU` 是偶发的，历次 0~1 次）
- **「为什么会有帧没有 IMU」没查** —— 那才是根因。这次只加了防御。

## 遗留

- 📌 **2026-10-12 追加：又崩了一次，但和这条不是同一个洞。**
  栈：`LocalMapping::InitializeIMU → Optimizer::InertialOptimization → g2o →
  **EdgeInertialGS::computeError** → **Preintegrated::GetDeltaRotation** →
  **Sophus::defaultEnsure**（抛）→ abort`。
  **⚠️ 上面那个 `continue` 就在这个 `EdgeInertialGS` 前面**（`Optimizer.cc:3139` vs `:3163`）
  —— **空预积分已经进不去了** ⟹ **这次不是空指针**。
  `defaultEnsure` 只在**值非法（NaN/Inf）**时抛 ⟹ **这次是 NaN**：要么来自
  `GetDeltaRotation` 的**重积分**，要么来自传进去的**偏置顶点估计**。**没查出来。**
  复现时又没崩，而且**主 logcat 缓冲滚掉了**，崩溃前的上下文没了 ⟹ 留到下次复现再查
  （要 **`adb logcat -b all -v time` 从头录**，别等崩了再抓）。
  见 [2026-10-12](2026-10-12-polyline-edge-direction.md)。
- ⚠️ **三处 `EdgeInertialGS` 的空洞**（见「决策」三）
- **`mTinit` 跨会话累加** —— `ScaleRefinement` 会在任意地图状态下触发（老账）
- **「谁把预积分/顶点弄空」没解决** —— 只加了防御

## 回滚

`Optimizer.cc` 把那个 `continue` 换回 `std::cout << "Not preintegrated measurement";`；
`thirdparty/README.md` 删掉第 11 条那一段。
