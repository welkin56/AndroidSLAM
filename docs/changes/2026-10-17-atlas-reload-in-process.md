# 2026-10-17 进程内重新载入 atlas（System 手动初始化）

一句话：不用重启进程、不重付 38 秒词袋，把 `System` 的 atlas **换回某份 `.osa` 的状态**。
这次只做**机制**，界面上还没人调用它。

## 动机

`System` 跟着进程活到底（§5 决策一），而一趟运行建出来的图会一直留在 atlas 里。
**没保存的那趟也一样留** —— 倒排表里还有它的关键帧，下一趟的回环检测照样能把它们
当成候选，把已经不要的数据合并回来。

要解决这个，就得有「把 atlas 退回上一次保存的样子」的能力。而构造之后上游没有换图的路：

- `LoadAtlas()` 里是 `ia >> mpAtlas` —— **换的是指针**；
- `mpTracker` / `mpLocalMapper` / `mpLoopCloser` 各自攥着旧的 `Atlas*`，换了指针就是悬空；
- `System` 又不能销毁（`Shutdown()` 不 join 线程、也没有析构函数）。

## 改动

### 上游补丁 20：读进现有的 Atlas，不换指针

- `Atlas::AdoptFrom(Atlas* p)` —— 把反序列化出来的 `mvpBackupMaps` / `mvpCameras` /
  `mnLastInitKFidMap` 搬到现有对象里。静态 ID 计数器是 `serialize` 直接写的，已经全局生效。
- `System::LoadAtlasInto(path, type)` —— 和 `LoadAtlas` 同一套读法，唯一差别是最后
  `mpAtlas->AdoptFrom(pLoaded)` 而不是 `mpAtlas = pLoaded`；`delete pLoaded` 时它只剩空壳。
- `System::ReloadAtlasFrom(path)` —— `mpTracker->Reset()` → `LoadAtlasInto()` →
  `CreateNewMap()` + `SetInertialSensor()`。`path` 空 ⟹ Reset 留下的那张空图就是结果。

### 上游补丁 21：Reset 的两处假设，我们碰上了

- `Tracking::Reset()` 清那四张轨迹表时**没拿 `mMutexTraj`** —— 上游只在 `Shutdown()`
  之后调它，所以不要锁；我们的主线程随时在读（2D 小窗）。
- `LoopClosing::ResetIfRequested()` 只清队列，**不清半途的候选计数**。
  下一个 KF 会走 `mnLoopNumCoincidences > 0` 那条分支，拿**上一张图**的 KF 位姿去算 Sim3。

### 我们这侧

- `slam_engine` 加 `requestRestoreMap()` / `restoreState()`，worker 在空闲时消费。
  目标路径 = 最近一次**成功**存图的文件 → 没有就用构造时载入的那张 → 都没有就空图。
- 启动时记一份 atlas 账（图数 / 关键帧数）；恢复前对一下，**没变化就跳过**。
- JNI + `SlamBridge.kt` 两个口子。

## 决策

**① 用「搬内容」而不是「重新指向」。**
让三个线程改指向意味着要在它们各自的线程上做一次同步的重绑，而且它们内部的
`mpCurrentMap` / `mpLastMap` 之类也得跟着改 —— 收益为零，风险全在同步上。

**② Reset 用 `Tracking::Reset()`，不用 `System::Reset()`。**
后者只是置一个 `mbReset` 标志，**只有 `TrackMonocular` 会消费它**。
停止期间根本没有帧进来，标志会一直挂着。我们直接在 SLAM 线程上调 `mpTracker->Reset()`。

**③ 放弃「读失败就保持原样」。**
`Reset()` 已经把倒排表和当前图清掉了，回不去。失败就留一张空图 ——
系统还能继续建图，总比卡在一个半清空的状态好。返回值让界面能说实话。

**④ 跳过「这一趟没建出东西」的恢复。**
每次恢复都会重读文件、并且**漏掉旧的一份图**（`Map::clear()` / `Atlas::clearAtlas()`
都不 delete，上游的设计）。白恢复一次就是白漏一份，所以先比一下账。

## 验证

**未验证。** 这次没有编译、没有装机。要验的四条：

| # | 做什么 | 期望 |
|---|---|---|
| ① | 新建地图 → 走一段 → 结束 → 保存 | 和从前一样 |
| ② | 载入那张图 → 走一段 → 结束 → 保存 | 存出来的仍是「两张图」（本次不做合并），但能载回来 |
| ③ | ⭐ 载入图 A → 走一段 → 结束 → 不保存 | logcat `[MAP] 恢复 atlas → A.osa` + `恢复 atlas 完成`；状态栏的地图账回到载入时的数 |
| ④ | 恢复之后接着走 | 跟踪正常、不崩、重置计数不涨 |

## 遗留

- ⚠️ **每次真正恢复都会漏一份旧图**（KF / MP 都不 delete，上游的 `clearAtlas()` 就这样）。
  自用一天几趟没问题；如果以后变成频繁恢复，要单独处理删除。
- ⚠️ **恢复用的是构造时那份词袋**，校验和必须一致。换过词袋的 `.osa` 会读失败（留空图）。
- ⚠️ **恢复期间不要从主线程读 atlas / 轨迹** —— `PostLoad` 没拿 `mMutexAtlas`。
  这一次由界面侧（下一批）保证；C++ 这侧没有兜底锁。
- `mnFirstFrameId` 在载入多图的 atlas 上不会更新（上游只在 `GetAllMaps().size()==1`
  时更新）。它只在 `ResetActiveMap` 里数「丢了哪些帧」，影响可忽略，没动。

## 回滚

```
thirdparty/orb_slam3/include/Atlas.h + src/Atlas.cc      ← AdoptFrom
thirdparty/orb_slam3/include/System.h + src/System.cc    ← LoadAtlasInto / ReloadAtlasFrom
thirdparty/orb_slam3/src/Tracking.cc                     ← Reset 的锁
thirdparty/orb_slam3/src/LoopClosing.cc                  ← ResetIfRequested 的清计数
app/src/main/cpp/slam_engine.{h,cpp}                     ← 请求 / 状态 / 跳过判断
app/src/main/cpp/jni_bridge.cpp、SlamBridge.kt           ← 两个口子
```

⚠️ 只回滚 C++ 不回滚 `SlamBridge.kt` 的话，那两个 `external fun` 找不到实现，
**类加载时 `UnsatisfiedLinkError`**。
