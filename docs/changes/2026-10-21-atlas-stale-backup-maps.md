# 2026-10-21 存图写进了上一次的备份，载图 SIGSEGV

一句话：`Atlas::PreSave()` 往 `mvpBackupMaps` 里 **追加** 而不清空。同一进程里连续存图
（跑一趟→存→再跑一趟→再存）时，第二份文件会带上一张**已经被合并掉、内容却停在上一份
备份里**的图 —— 它引用的 KF 已经搬到别的图了。载入时 `PostLoad` 找不到这些 id，往连接表
里塞 `nullptr`，`UpdateBestCovisibles()` 解引用就崩。

## 动机

用户报「新建地图可以，载入地图不行；新建地图初始化后点载入地图，整个 App 都崩溃了」。

真机墓碑：

```
Fatal signal 11 (SIGSEGV), code 1 (SEGV_MAPERR), fault addr 0x107c
#00 pthread_mutex_lock+16
#01 std::__ndk1::mutex::lock()+12
#02 ORB_SLAM3::KeyFrame::UpdateBestCovisibles()+640
#03 ORB_SLAM3::KeyFrame::PostLoad(...)+3032
#04 ORB_SLAM3::Map::PostLoad(...)+908
#05 ORB_SLAM3::Atlas::PostLoad()+480
#06 ORB_SLAM3::System::LoadAtlas(int)+1540
```

`0x107c` 就是 `nullptr + offsetof(KeyFrame, mMutexConnections)` ——
`PostLoad` 往 `mConnectedKeyFrameWeights` 里插了一个 **null KeyFrame**。

往前翻日志，载入时成片地打：

```
ERROR: MP without KF reference 310; Num obs: 30
ERROR: MP without KF reference 328; Num obs: 12
...
```

文件里的 map 引用了一批不在这张图里的 KF id。查设备上的存档：只有一个会话
（`20261021-192709`），里面**两趟**都存过图（`map-1/` 下两份 trajectory json，
19:28:00 和 19:28:40），`.osa` 的 mtime 是第二趟的 19:28:43 —— 第二趟选的「覆盖」。

## 根因

`Atlas::PreSave()` 原来是这样：

```cpp
std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

for (Map* pMi : mvpBackupMaps) {
    if (pMi->GetAllKeyFrames().size() == 0) { SetMapBad(pMi); continue; }  // ← 只剔出 mspMaps
    pMi->PreSave(spCams);
}
```

- **`mvpBackupMaps` 没有 `clear()`**：第二趟存图时，列表里还留着第一趟的图。
- 空图是在 `std::copy` **之后**才被 `SetMapBad` 的 —— 它已经被塞进 `mvpBackupMaps`，
  照样会被序列化。
- 更致命的是：那张图（第一趟的 map 0）在第一趟存过图，身上挂着**第一趟的**
  `mvpBackupKeyFrames` / `mvpBackupMapPoints`。第二趟它被合并成 0 关键帧，KF/MP 已经
  `UpdateMap` 搬到 map 1 去了，但**备份数组没人清**。
- 于是写出去的文件里，map 0 的 KF 列表停在第一趟（没有 map 1 新建的那批 KF），
  而 KF 的连接、MP 的参考帧已经按第二趟的 `PreSave` 更新成了 map 1 的 id。

`PostLoad` 是**按图**建 id 表的（`Map::PostLoad` 里 `mpKeyFrameId` 只装这张图的 KF）⟹
map 0 查不到那些新 id ⟹ `mConnectedKeyFrameWeights[nullptr]` ⟹ 崩。

顺带查出同一类的两个上游隐患：

- `MapPoint::mBackupRefKFId` **构造函数没初始化**。`PreSave` 只在参考帧属于本图时才赋值，
  否则留下的是构造时的垃圾 —— 写进文件就是一个不存在的 id。
- `Map::PreSave` 把 `mspKeyFrames`（不过滤 `isBad()`）当过滤集传给 KF/MP 的 `PreSave`，
  而 `mvpBackupKeyFrames` 是**过滤**过坏帧的。一旦集合里残留一个坏帧，就会写出一个
  指向不存在帧的连接 id。

## 改动

**上游（补丁 30、31）**

- `Atlas::PreSave()`：**先 `mvpBackupMaps.clear()` 再重建**；空图 / 坏图在**加入列表前**
  就剔掉，绝不让它带着旧备份数组进文件。
- `Map::PreSave()`：过滤集 `spKFs` 只放非坏的 KF，MP 和 KF 的 `PreSave` 都用它。
- `MapPoint`：四个构造函数把 `mBackupRefKFId` 初始化成 `-1`；`PreSave` 每次先归零。
- **新增载入前自检**：`Atlas::CheckBackupConsistency()` → `Map::CheckBackupConsistency()` →
  `KeyFrame::CheckBackupConsistency()`。逐张图检查 MP 的参考帧、KF 的连接 / 父子 /
  children / loop / merge 边是不是都落在**本图**的 id 表里。不一致就判载入失败
  （构造路径抛异常 → 「载图失败」；进程内换源 → `restoreState=4` → 「载入地图失败」），
  不再 SIGSEGV。
- `KeyFrame::PostLoad()` / `MapPoint::PostLoad()`：查不到的 id **跳过**，不再往容器里塞
  `nullptr`；MP 的参考帧丢了就退到第一个观测帧。自检已经挡住了坏文件，这两处是兜底。

**我们这侧**

- 无 Kotlin / JNI 改动。

## 决策

**① 不修旧文件，直接判载入失败。**
那份 `.osa` 少了 13 个关键帧的引用，图是不完整的 —— 强行把 null 跳过去能让它「载入」，
但后续 IMU / 回环会踩到别的空指针。App 本来就有「载图失败」这条干净的路径，宁可让用户
重跑一趟，也不要放一张残缺的图进来。

**② 自检放在 `PostLoad()` 之前、`AdoptFrom()` 之前。**
`ReloadAtlasFrom` 会先把内容搬进现有 atlas 再 `PostLoad`，那时已经晚了。所以在构造路径
反序列化完就查 `mpAtlas`，在换源路径查还没被搬走的 `pLoaded`。

**③ 为什么不是只加 `clear()`。**
空图是在 copy **之后**才被识别出来的；只清空列表，它下一次还是会被 copy 进来，仍然带着
第一趟的旧备份数组。必须把「筛选」挪到「加入列表」之前。

**④ 为什么还要改 `Map::PreSave` 的过滤集。**
就算 `mspKeyFrames` 里不该有坏帧，写文件时也**只认真正会写进去的集合**。这是零成本的一致性：
过滤器指向的集合 = 文件里的集合，就不会有悬空 id。

**否决：把坏引用从备份数组里删掉再存。**
那是在保存路径上做修复，等于把「图里本来就不该有坏帧」这个前提悄悄抹掉；根因（坏帧怎么
留在集合里）还在，下次换个地方暴露。根因在 `PreSave` 的两个循环，改那里。

## 验证

**2026-10-22 真机跑到了修复前必崩的那条路径**：同一进程连续三次存图，后两次都在合并之后。

```
10-22 13:12:00.507  [MAP] 存图完成
10-22 13:12:30.442  [MAP] 合并：图 1（KF 34）→ 图 0（KF 139），scale 1
10-22 13:12:55.549  [MAP] 存图完成        ← 覆盖同一个 20261022-131200.osa
10-22 13:13:31.982  [MAP] 合并：图 2（KF 35）→ 图 1（KF 155），scale 1
10-22 13:13:48.936  [MAP] 存图完成        ← 第三次覆盖
```

- 全程没有 `MP without KF reference`，没有 SIGSEGV，`adb logcat -b crash` 为空。
- 最终目录只剩 `map-2/`（三条轨迹：`traj-131113` / `131211` / `131313`），
  `map-0/`、`map-1/` 已经按链并走，`traj/` 为空 —— 第二、三次存图写进去的只有
  **当前活着的图**，没有把上一次的备份数组再写一遍。
- `.osa` 35,480,040 字节，mtime 13:13:48（第三次存图）。

| # | 结果 |
|---|---|
| ① 同一进程连续存图（中间两次合并） | ✅ 见上，修复前这是必崩路径 |
| ② 杀掉 App → 冷启动载入新文件 | ⬜ **本次日志没有覆盖**：13:13:01 从入口页重选它时 `nativeLoadedMap()` 正好等于这份，`atlas 换源` 被跳过，没有真正读盘 |
| ③ 载入修复前的坏文件 | ⬜ 旧会话目录已经删了，`CheckBackupConsistency` 的失败路径没跑到 |
| ④ 一趟就存图（没合并） | ✅ 13:12:00 那次正常完成 |

⚠️ ②③ 还差一次定向验证：杀 App → 入口页选 `20261022-131200.osa` → 准备 SLAM。
修复后存的文件应该能正常载入。如果还有问题，把 `slam` 日志贴出来。

## 遗留

- **旧版本存出来的文件无法修复**（缺关键帧，不是靠跳过 null 能补的）。
- `Atlas::PreSave` 里 `SetMapBad` 会改 `mspMaps`，现在遍历的是副本 `vMaps`，安全；
  但这个函数整体仍然**没有加锁** —— 依赖「存图时 SLAM 已停」这条前提，和上游一致。
- `KeyFrame::SetBadFlag()` 里 `mpMap` 标坏后没有置空（见
  [2026-10-21-presave-crash-dangling-kf.md](2026-10-21-presave-crash-dangling-kf.md) 的遗留），
  这次没动。
- 同一进程里「存图 → 换源 → 再存」现在也走同一条重建逻辑，没有额外验证。

## 回滚

```
thirdparty/orb_slam3/{include/Atlas.h,src/Atlas.cc}
thirdparty/orb_slam3/{include/Map.h,src/Map.cc}
thirdparty/orb_slam3/{include/KeyFrame.h,src/KeyFrame.cc}
thirdparty/orb_slam3/{include/MapPoint.h,src/MapPoint.cc}
thirdparty/orb_slam3/src/System.cc
thirdparty/README.md（补丁 30、31）
```

`.osa` 格式没变；回滚后旧版本仍能读新版本存的文件（自检只是多一道读前检查）。
