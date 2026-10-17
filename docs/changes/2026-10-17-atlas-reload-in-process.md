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

---

## 追加：接上界面 —— 结束时不保存就把 atlas 退回去

### 改动

- `MainActivity.askMapSave()`：弹窗消失时看用户选没选「存」。
  **没选**（点了「不保存」，或者返回键 / 点外面关掉）⟹ `nativeRequestRestoreMap()`。
  三个肯定按钮（保存 / 覆盖 / 另存为）都置 `willSave = true`。
- 恢复期间：按钮文字「恢复地图…」并禁用；**不读 atlas 和轨迹**
  （`Atlas::PostLoad()` 没拿锁，主线程这时候问不得）；ticker 继续跑，直到状态落到完成/失败。
- 状态栏多一行：恢复中 / 已退回 / 失败。

### 决策

**① 触发点放在「弹窗消失」而不是「按了不保存」。**
返回键和点外面关掉是同一个语义（用户没说要存），用 `setOnDismissListener` 一个口子全接住。

**② 后台（切后台 / 退出）不触发恢复。**
那时没有「存不存」的问询，自动丢图会**静默丢数据**。脏状态留着，
等用户真正按「结束」走完那个弹窗再算账。

**③ 恢复期间禁「启动」。**
C++ 那侧也有一条兜底（`running` 为真就把请求留到下一轮），但那只是防线 ——
界面上不该让人按下启动、然后等着被恢复清掉。

### 验证

接上一节那张表，③④ 现在可以从界面走了：

| # | 做什么 | 期望 |
|---|---|---|
| ③ | ⭐ 载入图 A → 走一段 → 结束 → **不保存** | 按钮变「恢复地图…」；logcat `[MAP] 恢复 atlas → A.osa`；完成后状态栏「已丢弃这一趟」，地图账回到载入时的数 |
| ③b | 同上，但选「保存」 | **不**触发恢复；`[MAP] 存图完成` |
| ③c | 新建地图 → 走一段 → 结束 → 不保存 | 地图账回到 1 张 / 0 关键帧 |
| ④ | 恢复完接着按启动、走一段 | 跟踪正常、重置计数不涨、不崩 |

### 遗留

- ⚠️ **恢复期间视觉上是「卡一下」**：按钮灰着、地图账那行消失，直到完成。
  大图的 `.osa` 要读一两秒。
- ⚠️ **`exit` / 崩溃 / 被系统杀掉**都不会触发恢复 —— 只有走「结束 → 弹窗」这条正常路径。

---

## 追加：真机第一跑抓到的两个问题（2026-10-17 17:36）

### ① `.osa.osa` —— 路径拼了两遍（已修）

```
[MAP] 恢复 atlas → /storage/.../20261017-121243.osa.osa
Load file not found
```

`EXTRA_MAP_FILE` 传进来的是**完整文件名**（`EntryActivity` 扫的就是 `.osa`），
而 `reload` 和上游 `LoadAtlas` 一样**自己拼 `.osa`**。`slam::init()` 里统一去掉后缀。

⚠️ 顺带说明失败时的表现：`ReloadAtlasFrom` 之前已经 `Reset()` 过，所以
**atlas 被清成了一张空图** —— 这是设计好的兜底（§「决策 ③」），不是新 bug。

### ② IMU 预积分的残留（顺手补）

`CreateInitialMapMonocular` 里有这么一句：

```cpp
pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;
```

`Tracking::Reset()` 不清这个指针 ⟹ 新图的第一对 KF 会挂上一段**带着旧会话运动**的
预积分。上游没暴露是因为它不这么用；我们一重置就必中。现在 Reset 末尾换一个新的
（和 `MonocularInitialization` 里那段同一个写法）。

⚠️ `ResetActiveMap()` **没动** —— 它是「同一张图原地清空」，真机上一直在走，
行为和从前一致。要改得单独评估。

### 验证

已跑：载入图 → 走一段（自动合并了一次）→ 结束 → 不保存 → 请求恢复 → **失败**（①）。
修完这两条之后**未重新验证**。

---

## 追加：入口页每次重新选都要生效（2026-10-17 真机反馈）

### 问题

```
载入 A → 走一段 → 结束 → 不保存（atlas 退回 A）→ 退出到入口页 → 点「新建地图」
```

界面上写着「新建地图」，但 `System` 是进程级的：`nativeInit` 只会生效一次，
所以实际发生的是「在旧 atlas 上又开了一张当前图」—— **A 还在里面**，
回环检测照样能匹配到它。换一张图同理（载入只在构造那一刻做过）。

### 改动

- `slam_engine`：`requestLoadAtlas(path)` / `loadedMapPath()`。和「恢复」共用一条队列、
  同一个 `restoreState`，但语义分开：
  - `restoreHasTarget == false` → **恢复**：回 `savedPath` → `loadPath` → 空图；
  - `restoreHasTarget == true` → **换源**：入口页这次选的（空串 = 空图）。
  - 换源成功 ⟹ `loadPath = 新源`、`savedPath` 清空（以后的「恢复」以新源为准）。
  - 换源**不做**「这一趟没新数据就跳过」的判断 —— 用户明确要换，没数据也得换。
- `MainActivity.onCreate`：`System` 已经造好（READY / LOADING）时，比一下
  「入口这次选的」和 `nativeLoadedMap()`，**不同才换**。

### 决策

**① 「不同才换」而不是「每次都换」。**
每次都换的话，同一个 Activity 重建（进程被杀后恢复、返回键回入口再进来）会把
内存里还没存的东西静默丢掉。比较之后，重复选同一张就是空操作。

**② 四种组合走同一条路。**

| 从 → 到 | `have` | `want` |
|---|---|---|
| 载入 A → 载入 B | A | B |
| 载入 A → 新建 | A | `""` |
| 新建 → 载入 A | `""` | A |
| 载入 A → 载入 A | A | A（不换）|

**③ `loadedMapPath()` 报「存过的」优先。**
另存为之后，atlas 的内容对应新文件；再回入口选老图时应该重新载入老图，
而不是因为 `loadPath` 还是老的就不动。

### 验证

未验证。要看的日志：

```
atlas 换源：/…/A →            （点新建地图）
atlas 换源：/…/A → /…/B       （换另一张）
[MAP] 换源 → (空图) / …/B.osa
[MAP] 换源完成
```

### 补充两条边界

- **换源 / 恢复失败时 `loadPath` / `savedPath` 一起清空。** 失败时 `Tracking::Reset()`
  已经执行过了 ⟹ 内存里就是一张空图，来源不能再报着那张没装进来的图。
  清空之后，下次从入口再选同一张会**重试**（`have == ""` ≠ `want`）。
- **同一张图重复进入不换源。** 从入口页选一张和当前 atlas 同源的图是空操作，
  这和「系统重建 Activity」走的是同一条判断。代价：**退出（没按结束）→ 再选同一张**，
  内存里那段没保存的数据会留着 —— 和「切后台不算结束」是同一套语义。

---

## 真机验证（2026-10-17 17:50–17:53）

三条都跑通了，logcat tag `slam` / `MainActivity`：

```
17:51:31.075  [MAP] 恢复 atlas → …/20261017-121243.osa
17:51:31.232  [MAP] 恢复 atlas完成                       ← 157 ms

17:51:35.746  MainActivity: atlas 换源：…/20261017-121243 →     （入口点新建地图）
17:51:35.785  [MAP] 换源完成                             ← 37 ms

17:52:10.875  [MAP] 换源 → …/20261017-121243.osa         （入口换回载入）
17:52:11.040  [MAP] 换源完成                             ← 165 ms

17:52:12.326  MainActivity: 地图来源：…/20261017-121243.osa
（没有换源日志 ⟹ 同源不换 ✓）
```

耗时都在 200 ms 以内（.osa 只有 ~1 MB）。

### ⚠️ 顺带发现（**不是这批改动引起的**）

```
addVertex: FATAL, a vertex with ID … has already been registered with this graph
```

12 分钟刷了 13797 行，**每次都紧跟在 `*Merge detected` 之后**。17:40 那个进程从启动
到合并**一次 Reset / 换源都没做过**，照样刷 ⟹ 上游 `MergeInertialBA` 那条路的问题。
g2o 打 FATAL 后继续，不一定崩，但说明那次优化**少加了顶点**。

17:36 同样合并过一次却没刷 ⟹ 和数据有关。**单开一批查**，不在这批里动。

### 补充：构造失败时不要卡在「载入地图…」

入口页的换源请求是**排在队列里**的。如果 `System` 构造失败（`.osa` 读不了），
worker 直接退出 ⟹ 请求永远不会被消费，`restoreState` 停在 1，按钮文字会一直显示
「载入地图…」，盖住本该显示的「构造失败」。

- `construct()` 失败分支：清掉 `restoreRequest` / `restoreState`；
- 按钮文字：`SLAM_FAILED` 排在 `restoring` 前面。
