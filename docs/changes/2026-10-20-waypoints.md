# 2026-10-20 路径点

一句话：**记一个名字 + 「我此刻站在哪」**，存法和轨迹一致 —— 锚在关键帧上、存相对位姿。
BA / 回环 / 合并改了地图之后它自动跟着走；地图被重置就失效。

## 动机

用户的原话是「路径点要做成和轨迹一样的形式」。这句话在代码里有一个很具体的含义，
不是「长得像」，而是**存法一样**：

```cpp
// Tracking 每帧这么存（Tracking.cc:2327）
Tcr = Tcw_当前帧 * Twc_参考关键帧;
// 读的时候现乘，BA / 合并改过都自动反映进来
Tcw = Tcr * Twc_参考关键帧;
```

路径点如果直接存按下那一刻的 `x y z`，它会在**地图被优化之后自己漂走** ——
标记浮在半空、和墙对不上。这和当初折线为什么存 `MapPoint*` 而不是坐标是同一个教训
（[2026-10-12-pinned-points-follow-mappoints.md](2026-10-12-pinned-points-follow-mappoints.md)）。

## 改动

**上游（补丁 22、23）**

- `Map::GetLatestKeyFrame()`：图集合里 `mnId` 最大的 KF。
- `System` 上的路径点：`AddWaypoint` / `RenameWaypoint` / `DeleteWaypoint` / `ClearWaypoints` /
  `GetWaypoints` / `GetWaypointCount` / `GetWaypointName` / `GetWaypointRevision` /
  `SaveWaypoints` / `LoadWaypoints`。

**我们这侧**

- `slam_engine`：`requestAddWaypoint` / `waypointState` / `waypointMsg` / `renameWaypoint` /
  `deleteWaypoint` / `waypointRevision` / `getWaypoints` / `waypointCount`。
- `jni_bridge` + `SlamBridge.kt`：对应 8 个 JNI。
- `MapView`：独立的路径点图层（绿色菱形 + 名字），**不走 `Snapshot`**，由 revision 驱动。
- `MainActivity`：抽屉里「路径点」开关 / 「添加路径点」/「管理路径点」；状态栏一行计数。
- `activity_main.xml`：三个新控件。

## 决策

**① 锚在关键帧上，不存世界坐标。**
存坐标就是存「某一刻的结论」，而地图一直在被修正。锚 KF 存**相对位姿**，
读的时候现乘 —— 和轨迹同一套，回环、BA、合并全都自动跟上。

**② 锚点必须取「已经在图集合里」的 KF —— 这是本设计唯一的陷阱。**
上游对 KF 的回收分两种，后果完全不同：

| | 会发生什么 | 能不能锚 |
|---|---|---|
| 进了 `Map::mspKeyFrames` 的 KF | 只 `SetBadFlag()`，**从不 `delete`** | ✅ 能，被剔了就爬链 |
| 还在 `LocalMapping::mlNewKeyFrames` 排队的 | `Release()` 里 **`delete` 掉** | ❌ 悬空指针 |

`Release()` 在合并、回环闭合、**以及我们自己的 `SaveAtlasTo()`** 里都会被调。
而 `Tracking::GetLastKeyFrame()` 返回的恰恰可能是排队中的那个。
所以用 `Map::GetLatestKeyFrame()`，只在图集合里找。

**③ 锚点被剔了不失效，沿 `mTcp` 爬到活着的父帧。**
`SetBadFlag()` 在标坏那一刻把 `mTcp`（相对父帧的位姿）算好留在对象里，
所以爬链之后坐标**仍然准确**。这和 `GetFrameTrajectory()` 里那段爬链是同一份逻辑。
只有 `GetMap() == nullptr`（`Map::clear()`，即 `ResetActiveMap`）才算失效 ——
那时坐标系真的没了。用户问的就是这条，见下面的验证。

**④ 合并之后不用写任何钩子。**
`MergeLocal2()` 搬 KF 用的是 `pKFi->UpdateMap(pCurrentMap)` —— **对象指针不变**，
位姿已经被 `ApplyScaledRotation` 变换到新系。路径点的 `T_rel` 是相对的，
所以算出来自动就在新系里。

**⑤ sidecar 用 TSV，不用 JSON。**
`<base>.waypoints.tsv`。读写都在 C++，为固定 schema 手写一个 JSON 解析器不值当。
文件名改了 `APP_DESIGN §9` 里原来写的 `.markers.json`。

**⑥ 生命周期完全跟着 `.osa`。**
存图成功才写 sidecar；「不保存」退图时 atlas 换回旧的，sidecar 也跟着回滚；
换图时先清空再按新路径读。**「地图失效则路径点跟着失效」不需要额外机制。**

**⑦ 加路径点的请求下一帧就消费，不排队等跟踪恢复。**
排队的话，用户按完丢了几秒，会在**站错的地方**冒出一个路径点。这一帧不行就直接失败。

**⑧ 只画「有效 + 在当前图」的。**
别的图在另一个坐标系里，画到当前图的小地图上必然错位。这和「小地图只显示当前图的
点云和轨迹」是同一条纪律。

## 验证

**编译：C++ 通过，Kotlin 第一次失败（已修）。**

第一次 `./gradlew :app:assembleDebug` 的实际输出：

```
error: type mismatch: inferred type is Array<String?> but Array<String> was expected
            val n = SlamBridge.nativeGetWaypoints(wpPos, wpMeta, wpNames)
error: type mismatch: inferred type is Array<String?> but Array<String> was expected
        val n = SlamBridge.nativeGetWaypoints(pos, meta, names)
```

同一个错误报了两次（两处调用）。原因是我把 `nativeGetWaypoints` 的 `names` 声明成
`Array<String>`，而调用方用的是 `arrayOfNulls<String>(n)` ⟹ `Array<String?>`。JNI 那边
本来只认 `jobjectArray`，可空性签名不参与链接，所以直接改成 `Array<String?>`。
**native 那半边 19:57 就链接出 `libslam_jni.so` 了** —— 失败只发生在 Kotlin 阶段。

顺带修了一处没被编译器抓到的问题：关掉显示开关时原来调
`setWaypoints(wpPos, 0, wpNames)`，而 `wpPos` 是 stride-3、`setWaypoints` 按 stride-2 拷。
`n=0` 时无害，但语义是错的 —— 改成独立的 `MapView.clearWaypoints()`。

**装机验证未做**：

| # | 做什么 | 期望 |
|---|---|---|
| ① | 跑一段 → 加路径点（改名「门口」）→ 重命名 → 删一个 | 小窗绿色标记 + 名字；管理列表实时变 |
| ② | 关「路径点」开关 | 不画；管理界面仍能看；再打开能重新画出来 |
| ③ | 存图 → 杀掉 App → 载入这张图 | 路径点回来；不在当前图的显示「图 #x（不在当前图）」 |
| ④ | 载入后走一段，等自动 `Merge detected` | 合并后**原先不在当前图的路径点出现在正确位置** |
| ⑤ | 结束选「不保存」→ 再进同一张图 | 路径点退回上一次保存的状态 |
| ⑥ | ⭐ 走很久让 LocalMapping 剔掉一批 KF，再看路径点 | 位置**不跳**（爬链了） |
| ⑦ | 地图被重置（看 `地图已重置 N 次`）| 那条路径点标「已失效」，不画，不崩 |

## 第二轮：合并之后路径点「没了」、加不了点

真机反馈：「回环检测合并后就没了，之后也添加不了点了」。查下来是**三个独立的 bug**，
第 3 个是这一轮最严重的（合并会毁掉整张图）。

### bug 1 —— JNI 丢了 `meta`（添加就「失效」、也删不掉）

`nativeGetWaypoints` 把引擎结果写进了临时的 `metaBuf`，却拿**没动过的 `m`** 去
`ReleaseLongArrayElements`。于是 Kotlin 收到的 `meta` 全是 0：

- `flags` = 0 ⟹ 判定非「有效」⟹ 显示「已失效」，小地图不画；
- `id` = 0 ⟹ 删除时传 0，`DeleteWaypoint` 匹配不上任何一条 ⟹ 删不掉。

修：`Release` 之前 `memcpy(m, metaBuf.data(), ...)`。`pos` 一直是直接写进 `p` 的，
所以坐标本身没问题，只是界面不画。

### bug 2 —— 合并后标记停在旧位置

`refreshWaypoints()` 原来只在 `mnWaypointRevision` 变化时重读。合并走
`Map::ApplyScaledRotation()`，**整张图的位姿都变了而 revision 不变** ⟹ 小地图一直按旧坐标画，
点被画到视口外面，看着就是「没了」。

修：去掉 revision 早退，每拍重读。路径点只有几个，代价可忽略（轨迹本来也是每拍读）。

### bug 3 ⭐ —— `MergeInertialBA` 往 g2o 里加重复顶点

日志现场：

```
*Merge detected
Local Mapping STOP
addVertex: FATAL, a vertex with ID 28 has already been registered with this graph
addVertex: FATAL, a vertex with ID 466 ...
...（几百行）
Local Mapping RELEASE
Merge finished!
```

`MergeInertialBA` 的滑动窗口是**两条时间链拼起来的**：当前 KF 沿 `mPrevKF` 往前追，
合并目标 KF 沿 `mPrevKF` / `mNextKF` 前后追。两条链会走到**同一个 KF** 上，同一个
`mnId` 被 `addVertex` 两次。g2o 只打一行 FATAL 就继续，**图本身已经坏了** ——
优化结果不可信，合并后整张图不刷新、后续跟踪失败、连续重置。

修（[Optimizer.cc](app/src/main/cpp/thirdparty/orb_slam3/src/Optimizer.cc)）：

- `vpOptimizableKFs` 按对象去重（保留首次出现顺序），`N` 跟着改；
- `lFixedKeyFrames` 去掉与两个优化集重叠的成员。

另外在 `MergeLocal2()` 做变换**之前**加一行日志：
`[MAP] 合并：图 A（KF n）→ 图 B（KF m），scale s`。之前合并完全是静默的，
出问题时无从对齐时间线。

**这一条是上游 bug**，不是我们改出来的 —— 但只有「载入旧图 + 二次合并」这种用法
才会稳定踩到（两边的 `mPrevKF` 链在载入后是互通的）。

### 关于失败原因

`AddWaypoint` 增加了 `GetWaypointError()`，失败时会说清是「没有当前图」/
「图 7 还没有关键帧」/「当前帧没有位姿」，不再一律报「当前图还没有关键帧」。
同时把 `AddWaypoint` 里的 `GetCurrentMap()` 换成 `PeekCurrentMap()` —— 前者在图被
标坏的窗口里会抱着 atlas 锁 `usleep` 自旋，而这是 SLAM 线程。

### 验证（第二轮）

**合并修复已验证**（真机日志，2026-10-20 21:19）：

```
*Merge detected
[MAP] 合并：图 1（KF 46）→ 图 0（KF 183），scale 1
Merge finished!
[WP] 加路径点 #3 「路径点 3」 → 锚 KF 416（图 1）
```

- `addVertex: FATAL` **0 行**（修之前有几百行）；
- 合并之后不再出现 `Fail to track local map!`，也没有连续重置；
- 合并后仍能正常加路径点（#3、#4）。

### 顺带：默认名字改成用 id

日志暴露出重名：`加路径点 #3 「路径点 3」`、`加路径点 #4 「路径点 3」`。

原来默认名是 `现有条数 + 1`，**删过一个点之后条数回退**，新点就会和还没删的撞名。
改成 `GetNextWaypointId()` —— 返回下一个要分配的 id，单调递增，而且随 sidecar 存盘
（`LoadWaypoints` 里设成 `maxId + 1`）。


- **路径点的锚点是「图里最新那个 KF」**，不是「按下那一刻的帧」。两者可能差几十毫秒
  （下一帧就有一个更近的 KF）。对米级标记无意义。
- **`map id` 会消失**：覆盖保存过、或那张图被 `PreSave` 剔掉之后，sidecar 里的 `kfId`
  找不到了 ⟹ 读进来标「已失效」，名字保留，用户可手动删。不猜、不静默改锚点。
- 重置（`ResetActiveMap`）会让那条路径上的路径点**全部失效** —— 诚实的，但 UI 上会
  一次冒出一批「已失效」，需要用户手动清。
- 路径点**不进 `.osa`**，所以「只拷 .osa 到别的机器」会把路径点落下。这是刻意的
  （`.osa` 是 ORB-SLAM3 的格式，动它的序列化旧存档就读不了）。

## 回滚

```
thirdparty/orb_slam3/{include/Map.h,src/Map.cc,include/System.h,src/System.cc}
app/src/main/cpp/{slam_engine.h,slam_engine.cpp,jni_bridge.cpp}
app/src/main/java/com/example/slamcapture/{SlamBridge,MainActivity,MapView}.kt
app/src/main/res/layout/activity_main.xml
```

⚠️ Kotlin 和 JNI **必须一起回** —— 只回一层会在类加载时 `UnsatisfiedLinkError`。
`.waypoints.tsv` 是新增文件，删掉即可；`.osa` 格式没变。
