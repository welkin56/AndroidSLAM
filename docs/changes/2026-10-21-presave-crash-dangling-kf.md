# 2026-10-21 存图崩溃：初始关键帧被 delete 后还留在地图里

一句话：`LocalMapping::InitializeIMU()` 删 `mlNewKeyFrames` 里的 KF 时，
**初始关键帧会被 `SetBadFlag()` 拒绝**（它保护地图原点），但 `delete` 照样执行 ——
那张 KF 变成悬空指针，却还留在 `mspKeyFrames` 里、还被几百个地图点观测着。

## 动机

新布局第一次装机验证，跑完一趟点「保存地图」，App 秒崩：

```
10-21 19:07:32.117  I slam : [MAP] 开始存图 → .../20261021-190732.osa
10-21 19:07:32.161  F libc : Fatal signal 11 (SIGSEGV), code 1 (SEGV_MAPERR),
                            fault addr 0x21b10000000000
10-21 19:07:33.116  F DEBUG: #00 ... liborb_slam3.so
                            (ORB_SLAM3::MapPoint::PreSave(...)+332)
                          #01 ... Map::PreSave(...)
                          #02 ... Atlas::PreSave()
                          #03 ... System::SaveAtlas(int)
                          #04 ... System::SaveAtlasTo(...)
```

用 NDK 的 `llvm-addr2line` 对 **同 BuildId 的 .so** 符号化（BuildId 必须对得上，
否则地址会落到别的函数上）：

```
$ llvm-addr2line -f -C -i -e .../liborb_slam3.so 0x390e1c
std::__ndk1::__tree_next_iter<...>(...)
  .../__tree:208
ORB_SLAM3::MapPoint::PreSave(...)
  app/src/main/cpp/thirdparty/orb_slam3/src/MapPoint.cc:581
```

反汇编确认：崩在 `bl EraseObservation` 之后的 `++it` ——
erase 已经把当前节点释放了，`__tree_next_iter` 从**已释放的节点**里读孩子指针，
读到 `0x21b10000000000` 这种垃圾值。

**这不是新功能引入的**，是上游一个存在很久的 KF 生命周期 bug；这次跑在
「地图重置 + 惯性初始化」之后被踩中（日志里 `[M5] 地图被重置（第 1 次）`，
随后的 `First KF:3` 让 3 号成为新图的初始 KF）。

## 改动

**上游（补丁 27、28）**

- `LocalMapping::InitializeIMU()` / `ScaleRefinement()`：
  ```cpp
  (*lit)->SetBadFlag();
  if((*lit)->isBad()) delete *lit;   // ← 新增判断
  ```
- `MapPoint::PreSave()`：先把 `spKF` 里没有的观测 KeyFrame 收集到 `vpStale`，
  循环结束再逐个 `EraseObservation`（不能边遍历边 erase）。
- `MapPoint::EraseObservation()`：擦到 `mObservations` 为空时，`mpRefKF` 置 null，
  不再解引用 `begin()`（空 map 的 `begin() == end()`）。

## 决策

**① 为什么初始 KF 会进 `mlNewKeyFrames`？**
`Tracking::CreateInitialMapMonocular()` 既 `mpAtlas->AddKeyFrame(pKFini/pKFcur)`，
又 `mpLocalMapper->InsertKeyFrame(...)`。所以它们同时在**地图集合**和**待处理队列**里。

**② 为什么 `SetBadFlag()` 拒绝它？**
`KeyFrame::SetBadFlag()` 第一句是 `if(mnId == mpMap->GetInitKFid()) return;` ——
初始 KF 是地图原点的定义者，不能被剔除。上游的本意是「不标坏」，
但调用方紧接着的 `delete` 没有跟着这个语义走。

**③ 为什么不干脆不从队列里删它？**
`mlNewKeyFrames.clear()` 只是把它从**队列**里拿掉；它早就在地图里、
并且由 `CreateInitialMapMonocular` 建好了观测和连接，不需要再被 `ProcessNewKeyFrame` 处理。
「不 delete + 清队列」正好留下它该有的状态。

**④ 为什么还要改 `MapPoint::PreSave`？**
悬空 KF 的根源堵住了，但「边遍历 `mObservations` 边 `erase`」本身就是 UB ——
只要将来任何一条路径让观测里出现一个不在 `spKF` 的 KF（比如以后加别的图切换逻辑），
都会以同样的方式崩。收集后再删是零成本的写法。

**⑤ 为什么在 `EraseObservation` 里把 `mpRefKF` 置 null？**
`EraseObservation` 可能在擦掉最后一个观测时走到 `mpRefKF == pKF`；
原写法 `mObservations.begin()->first` 在空表上解引用 `end()`。置 null 之后，
`PreSave` 的 `spKF.find(nullptr)` 自然失败、不写备份 id，后续也不会有人拿它算位姿
（观测 ≤2 的点会被标坏）。

**否决：只改 `PreSave`、不动 `LocalMapping`。**
能挡住这次崩溃，但悬空 KF 还在图里 —— BA / 回环 / 剔除随手一碰就是别的 UB
（这次日志里那串 `*Loop detected` + `BAD LOOP!!!` 很可能就和它有关）。
根因必须堵在 `delete` 那里。

## 验证

**未验证**（还没重新编译 / 装机）。崩的是**存图路径**，现有验证表（
[2026-10-21-map-folders-for-trajectory.md](2026-10-21-map-folders-for-trajectory.md#验证)）
的 ①②⑤ 都会顺带覆盖它。

> **2026-10-22 补记**：同一进程三次存图（两次合并之后覆盖）全部 `[MAP] 存图完成`，
> 没有再出现 `MapPoint::PreSave` 的 SIGSEGV。

定向复现步骤：

| # | 做什么 | 期望 |
|---|---|---|
| ① | 启动 → 等 `First KF:...` 和 `start VIBA 1/end VIBA 1` 都出现 → 跑 30 秒以上（尽量触发一两次回环）→ 结束 → 存图 | 不再 SIGSEGV，日志出现 `[MAP] 存图完成` |
| ② | 先踩一次 `地图被重置（第 N 次）`，再跑一段 → 存图 | 同上（这次就是这条路径） |
| ③ | 存图后杀掉 App → 载入 `.osa` | 图能正常读回（悬空 KF 不会再被序列化进文件） |

## 遗留

- 这次只堵了 `InitializeIMU` / `ScaleRefinement` 两条路径。**其它 `delete` KeyFrame
  的地方**（`LocalMapping::Release()`）只删队列里的 KF，没有 `SetBadFlag`，
  性质不同 —— 目前没发现问题，但值得记住这个模式：**delete 之前要确认它真的不在图里**。
- `SetBadFlag()` 里 `mpMap` 在标坏之后**没有置空**（`mpMap->EraseKeyFrame(this)` 之后
  仍指向原图）。`PreSave` 靠 `isBad()` 兜住了，但其它只查 `GetMap()` 的地方会是坑。
  这次没改，怕牵动面太大。

## 回滚

```
thirdparty/orb_slam3/src/{LocalMapping.cc,MapPoint.cc}
thirdparty/README.md（补丁 27、28）
```
