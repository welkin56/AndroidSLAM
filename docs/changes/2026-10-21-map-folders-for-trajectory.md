# 2026-10-21 轨迹按 atlas 图分文件夹

一句话：`run-N/trajectory.txt` 换成 **`map-<id>/traj-<时间>.txt`** —— 一个 atlas 里
每张图一个文件夹，**文件夹里的轨迹全部在同一坐标系**。合并时合并文件夹。

## 动机

一个 atlas 里可能有好几张图（载入旧图再跑、图里被重置、图与图合并），而原来的布局是
每趟一个 `run-N/`，**磁盘上完全看不出这段轨迹属于哪张图、跟别的趟在不在同一个系**。

关键事实：**合并会改坐标系，但不改图的身份**。

- `MergeLocal2()` 里 `pCurrentMap`（当前图，容器）活着，被吸收的旧图 KF/MP 全部
  `UpdateMap` 搬过来，自己变 0 关键帧、存盘时被 `PreSave()` 剔掉；
- `ApplyScaledRotation(T_on, s)` 作用在**当前图**上，`p' = s·R·p + t` ⟹ 合并之后
  容器图的坐标就是被吸收那张图的坐标；
- 所以「合并前写进磁盘的轨迹」停在旧坐标系里，要乘 `T_on` 才能和合并后的图叠。

## 改动

**上游（补丁 26）**

- `Atlas`：`RecordMerge` / `GetMergeRecords` / `ClearMergeRecords` —— 合并链**只存内存**
  （`survivor, absorbed, s, T_on`）。`PostLoad()` 末尾清空：链属于被换掉的那个 atlas。
- `LoopClosing::MergeLocal2()`：在 `ApplyScaledRotation` **之前**记一条。后面
  `numKFnew<10` 那条提前 `return` 时变换和 KF 搬迁已经做过了，所以必须放最前面。
- `System`：`GetTrajectoryMapId()`（判据和 `GetFrameTrajectory()` 逐字一致）、
  `GetMergeRecords()`；`ReloadAtlasFrom()` 入口清链（兜住载入失败那条路）。

**我们这侧**

- `slam_engine`：`stop()` 里当场快照 `lastTrajMapId` + 合并链；`transformTrajFile(src, dst, …)`。
- `jni_bridge` + `SlamBridge.kt`：3 个新 JNI。
- `TrajectoryStore.kt`（新）：`mapDir` / `trajDir` / `recDir` / `tracesFor()`。
- `MainActivity`：新目录布局、延迟归档、录像搬 `rec/`、合并链写进轨迹 json。
- `docs/tutorial/APP_DESIGN.md` §9、`thirdparty/README.md` 补丁 26。

## 决策

**① 合并链只存内存，不落盘。**
它只在「存图成功、重组文件夹」那一刻用一次；重组之后所有文件已经统一，
链本身没有价值。跨进程更不用 —— 被吸收的图存盘时就被剔掉了，
读进来的每张活着的图各自是独立坐标系。轨迹 json 里另抄一份 `merges` 留给人看。

**② 延迟归档：轨迹先写 `traj/`，存图成功才搬进 `map-<id>/`。**
这是审核时发现的最大问题。原计划在 `stop()` 就写进 `map-<id>/`，有两个洞：

- 选「不保存」/存失败时，那个文件夹里会留下一份**已合并坐标系**的轨迹，混在旧坐标系
  文件里，不变量当场破掉；
- `Map::nNextId` 随 `.osa` 存盘，**回滚之后同一个 id 会被下一张新图复用** ⟹ 上趟被
  丢弃的轨迹会被算进另一张图。

延迟归档把两个洞一起堵上：`map-<id>/` 只会在存盘成功时新建，而存过的 id 一定在
`.osa` 里、不会回头复用。副作用是「本趟文件已经在最终坐标系里」变成天然成立 ——
原先那个「重排时跳过本趟文件」的例外整条删掉了。

**③ 重组按合并链逐条做，不能拆成「先全变换再全搬」。**
第二条合并发生时，第一步已经从被吸收目录搬进来的文件也在 `map-<survivor>/` 里，
它们同样要乘第二步的 `T_on`。按链顺序走，每一步「幸存目录里现有的东西」正好在
当时的坐标系里，多跳自动正确。

**④ 文件夹名跟**存活的那张图**，不跟坐标系来源。**
合并后坐标系来自被吸收的图，但那个图已经 0 关键帧、会从 `.osa` 里消失；
跟着它命名 ⟹ 文件夹和 atlas 对不上。所以：被吸收的目录消失、内容搬到容器目录下。
载入图 7 之后新开一趟（容器是新建的图 8）合并 7→8 时，看起来就是 **7 改名成 8** ——
第二种才是常态。

**⑤ 存图完成用「保存中 → 完成」那一拍触发一次。**
`saveState` 是**粘性的**（3 会一直保持到下一次 `requestSaveMap`），原计划「看到 3 就
重排」会被之后任何一次 `render()`（点开关、拖滑块）再触发一遍 —— 坐标被变换很多次。

**⑥ 归档上下文在 `startMapSave` 时快照。**
存图是异步的，完成时用户可能已经开了下一趟，`runMapId` / `curTrajBase` 都重置了。
`PendingArchive` 存请求那一刻的三个字段。

**⑦ 元信息在 `stop()` 就写，不等「留不留」弹窗。**
存图可能在用户回答之前完成、把两个文件一起搬走；晚写就会一个在 `map/`、一个留在
`traj/`。相应地，删轨迹时用 `runTrajFile()` 同时查两个位置 —— 否则「不保存」会删一个
不存在的路径，文件反而留在图目录里。

**⑧ 变换先写 `.tmp`，全部成功再统一 `rename`；`renameTo` 的返回值必须查。**
原计划不查返回值就 `deleteRecursively()` 源目录 ⟹ 搬失败 = 删掉没搬走的文件。

**⑨ 不走运行时变换。** 读的时候直接叠 `map-<id>/` 里的全部文件，不做任何换算 ——
变换只在写入侧一处。合并后、存图前那个窗口里，被吸收目录的轨迹**本来就对**
（坐标系就是它的），容器目录的存量是旧的、不能叠；这一批不接 UI。

**否决的方案：**

- **不保存时把本趟轨迹按 `T_on` 逆变换回来**。理论上能保住文件夹不变量，但多一套
  「反向变换 + 只对某一趟生效」的特殊路径，而且解决不了 id 复用。
- **合并一发生就落盘**。违反「不保存就不动磁盘」：回滚之后磁盘和 atlas 对不上。
- **文件夹名改用「坐标系来源那张图」的 id**。那张图合并后就不在 `.osa` 里了。
- **给文件夹加稳定后缀（如首个 KF id）**。名称稳定了，但文件夹名不再等于 map id，
  读取侧要多一层映射；不值得。

## 验证

**App 未验证**（还没编译 / 装机 —— 编译由用户跑 `./gradlew :app:assembleDebug`）。

> 第一次装机验证跑到了存图那一步就崩了 —— **不是本功能的问题**，是上游的初始
> KF 悬空指针（见 [2026-10-21-presave-crash-dangling-kf.md](2026-10-21-presave-crash-dangling-kf.md)）。
> 修完之后这一份的验证表要重跑一遍。

**公式自检**（PC 上独立小程序，不是 App 构建）：把 `transformTrajFile` 的公式和
`Map::ApplyScaledRotation` 对关键帧的处理逐项对比，1000 组随机位姿 / 尺度：

```
$ g++ -O2 -I/usr/include/eigen3 /tmp/traj_xform_test.cpp -o /tmp/traj_xform_test && /tmp/traj_xform_test
max error = 5.94136e-06
```

`q' = q_on ⊗ q`、`t' = s·R_on·t + t_on` 是对的（误差在 float 精度内）。
注意这里只能验证**数学**，映射到哪一个方向（`T_on` 作用在当前图上）是从
`MergeLocal2` 的调用点读出来的。

| # | 做什么 | 期望 |
|---|---|---|
| ① | 跑一趟不合并，存图 | 出现 `map-<id>/traj-<时间>.txt`，`traj/` 里那个消失，没有 `run-1/` |
| ② | ⭐ 载入旧图 → 触发 `Merge detected` → 存图 | 旧图的 `map-x/` 并入本趟的 `map-y/`，`map-x/` 整个消失；文件数 = 两者之和 + 本趟 |
| ③ | ⭐ 同上，看合并前就在 `map-y/` 里的轨迹 | 坐标被 `T_on` 变换过；从 `map-x/` 搬进来的没变 |
| ④ | ⭐ 同上，看本趟那个 `traj-*.txt` | 没被变换（和内存里的轨迹逐点一致） |
| ⑤ | ⭐ 触发合并但选「不保存地图」 | `map-*` 完全没动；本趟文件留在 `traj/`；没有 `map-y/` |
| ⑥ | 存图失败（磁盘满 / 只读） | 同 ⑤ |
| ⑦ | 一趟里连续合并两次 | 按顺序重放；`map-3/` 收集到三方全部轨迹 |
| ⑧ | 存图成功后再等 10 秒 / 点几个开关 | 文件不再被二次变换 |
| ⑨ | ⑤ 之后再跑一趟 → 存图 | 上一趟的松文件没被算进新图的 `map-<id>/` |
| ⑩ | 勾了录像跑一趟 | 录像在 `rec/<时间>/session-M/`，和 `map-<id>/` 平级 |
| ⑪ | 另存为（勾 / 不勾录像） | 新目录有 `map-<id>/`、`traj/`；`rec/` 只在勾了时出现；本趟文件只变换一次 |
| ⑫ | 不保存这一趟轨迹 | 本趟两个文件被删，`traj/` 里别的文件不动 |
| ⑬ | 回归 | 路径点 / 轨迹 / 测距 / 折线 / 小地图 / 入口页换图 |

## 遗留

- **不做回放**：`tracesFor()` 有了，但还没接到小地图上。
- **合并窗口内的历史叠加**（下一批必须能做）：被吸收目录原样画，容器目录按内存里
  的链乘 `T_on`。⚠️ 现在的合并链只在 `stop()` 快照，跑着的时候显示侧拿不到当前链 ——
  下一批要加一个读当前 atlas 链的接口（`Atlas::GetMergeRecords` 本来就带锁）。
- **一趟中途 `CreateMapInAtlas()` 换过图**时，前面那段会被 `GetFrameTrajectory()` 静默
  滤掉，本趟只落进最后那张图的目录。要如实分段得让导出按图切 —— 另开一批。
- **`traj/` 会攒松文件**：一直不存图的趟都留在这里。以后可以加一键清理。
- **载入后浏览用哪张图**：`ReloadAtlasFrom()` 末尾会 `CreateNewMap()`，**current map
  是新的空图**。显示那批要按「图集里 id 最大且关键帧 > 0」取，不能用 current map。
- **别在同一个会话目录里混不同版本的 `.osa`**：`Map::nNextId` 随文件走，载入一份更旧的
  会把计数器拉回去，新图号可能和已有文件夹撞上。正常 App 流程不会出现。
- 非惯性 `MergeLocal()`（我们不跑）**不记合并**，而且它会 `ChangeId()` 让两张图共号 ——
  以后若改纯单目，文件夹方案要重新想。

## 回滚

```
thirdparty/orb_slam3/{include/Atlas.h,src/Atlas.cc,src/LoopClosing.cc,
                     include/System.h,src/System.cc}
app/src/main/cpp/{slam_engine.h,slam_engine.cpp,jni_bridge.cpp}
app/src/main/java/com/example/slamcapture/{SlamBridge,MainActivity,TrajectoryStore}.kt
docs/tutorial/APP_DESIGN.md, thirdparty/README.md
```

⚠️ Kotlin 和 JNI **必须一起回**（否则类加载时 `UnsatisfiedLinkError`）。
`.osa` 格式没变、没有新增格式文件。已按新布局落盘的目录要手工搬回 `run-N/`
才会有旧代码认得的布局（旧代码不读 `map-<id>/`，不会崩，只是看不见）。
