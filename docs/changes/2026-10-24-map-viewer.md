# 2026-10-24 2D 地图查看器：atlas 里每张图单独看

一句话：抽屉里多一个「地图查看」，全屏 2D 地图，**atlas 里有几个带关键帧的图就能切几个**——
即使那张图还没和当前图合并。一次只画一张（不同图没有公共坐标系），SLAM 在跑时
只有当前轨迹所属那张图上画当前位置。

## 动机

小地图只画当前图：`Atlas::GetAllMapPoints()` / `GetAllKeyFrames()` 都只转发
`mpCurrentMap`，点云快照也是当前图。载入一个 atlas 之后，旧图在内存里活得好好的，
但界面上完全看不到它们；历史轨迹层也只显示「当前轨迹所属那张图的合并家族」。
用户要一个「把小地图放大、能翻看 atlas 里每张图」的查看器。

## 改动

**上游（补丁 32）**

- `System::GetMapList(ids, kfs, mps, cap)`：遍历 `GetAllMaps()`，只返回 **KF>0** 的图，
  按 id 升序。
- `System::GetMapCloud(mapId, out, cap)`：找到那张图 → `Map::GetAllMapPoints()` →
  stride 抽稀到 `cap/3` 点 → 写 `3n` 个 `xyz`。**是这张图自己的坐标系**。

**引擎 / JNI**

- `slam_engine`：`mapList()` / `mapCloud()` / `trajectoryMapId()`（实时值，
  区别于 `stop()` 快照的 `lastTrajMapId`）。
- `jni_bridge` + `SlamBridge`：`nativeMapList` / `nativeMapCloud` / `nativeTrajectoryMapId`。

**MapView**

- `autoCenter`（默认 true）：false 时不跟当前位置改中心/切片；当前位置标记照画。
- `continuousZoom`（默认 false）：查看器用连续 `mPerDp`；小地图仍走档位表。
- `fitToBounds`：按 bbox 算连续比例 + 中心（选图 / 「适应」）。
- `setSliceCenter` / `sliceCenter` / `trackSliceCenter`：切片中心的外部读写；
  `trackSliceCenter` 复用同一套 `Z_SNAP` / `Z_LOWPASS`，查看器开着、小地图不重绘时
  也能跟着人走。
- 单指拖动平移、双指捏合（以两指焦点为中心，焦点移动顺带平移）。
  **不做双击、不做惯性**。
- `fullBleed`：全屏时不画圆角底板/描边。
- **拖动锚点 = 所有指针的平均位置**（`updateDragFocus`），`ACTION_POINTER_UP`
  排除抬起的那根。第一版用手指数变化时的 `getX(0)` 重新起算 —— 抬起的是 index 0 时，
  锚点留在了那根已经失效的手指上，剩下那根一动，位移就从几百像素外算起，
  整幅图瞬移。两指时平均位置正好等于 `ScaleGestureDetector` 的焦点，
  捏合启动/收尾两套锚对得上，也不会错位。

**MainActivity / 布局**

- 抽屉 `btnMapViewer`「地图查看」；`previewWrap` 里叠 `viewerWrap`（第二个 MapView +
  顶部 HUD + 右下角缩放键）。打开时隐藏状态行/底部按钮行，相机和 SLAM 继续跑。
- HUD：`图 #id · KF · 点 · N 条轨迹[ · 当前位置/上次位置/当前位姿在图 #x]`（点标题开
  地图列表）、`当前`、`刷新`、`适应`、`切片:高度/中位数`、`关闭`。
- 每张图：点云（选图/刷新时读一次）、`map-<id>/` 的历史轨迹（**单位变换**）、该图的
  有效路标点；本趟白线只在选中当前图时画；当前位置只在选中当前图 + 跑着 + 跟踪 OK 时画。
- 切片中心两种模式：**当前高度**（默认，跟小地图的低通 z）或**点云 z 中位数**
  （历史图没有位姿时更稳）。选择记在 SharedPreferences。
- 查看器开着时每 ~2 秒重读一次地图列表；选中的图没了就切到当前图（没有就切最大 id）。
- Back：查看器开着先收起；抽屉开着先关抽屉；否则维持原行为。
- `refreshWaypoints` 拆出 `readWaypointList()`：开关关着但查看器开着时也读，
  查看器按 `mapId` 筛自己那张图。

## 决策

**① 一次只画一张图，绝不叠。**
不同图没有公共坐标系；叠画（哪怕换颜色）是错的。地图选择器是功能的一部分。

**② 轨迹用单位变换读。**
`map-<id>/` 的不变量就是「文件在这张图自己的系里」。用 `nativeDisplayTransforms`
反而会把别的图/合并链的坐标套进来。

**③ 点云主线程枚举、只在切图/刷新时做。**
`System::GetMapCloud` 会拿 `mMutexMap` 拷一份指针表，几毫秒级；切图是低频动作。
走 worker 快照要再加「按 mapId 缓存 + 请求队列」，收益不值。耗时 >20ms 打一行 warning。

**④ 不复用 `mapSnap`。**
那个快照只覆盖当前图、1 Hz、worker 侧；查看器需要任意图，直接 `GetMapCloud`。

**⑤ `autoCenter=false` + fit + 拖动。**
查看旧图时没有位姿；跟当前位置会把整张图甩出屏幕。当前位置只是一个标记。

**⑥ 当前位置只画在它所属那张图上。**
别的图没有重定位关系，画上去就是错的；HUD 明说「当前位姿在图 #x」，给「当前」按钮跳过去。

**⑦ 切片中心两种模式可切，默认当前高度。**
当前高度 = 小地图同一套低通；中位数 = 这张图点云的 z 中位数。不做自动判断。

**⑧ 在 Activity 里叠一层，不新开 Activity。**
共用 ticker / 位姿 / 缓冲，不重复一遍 SLAM 状态机。

**否决：**
- **把 atlas 里所有图的点云同时画出来。** 没有公共系。
- **查看器跟当前位置自动居中。** 看历史图会甩出屏幕。
- **双击缩放 / 惯性滑动。** 用户没要，先不做。
- **为查看器改载入/合并逻辑。** 它正是为了「没合并也能看」，不需要动 Atlas/Tracking。

## 验证

**未验证**（还没编译装机 —— 编译由用户跑 `./gradlew :app:assembleDebug`）。

**先造一个多图 atlas**（现在设备上那份合并完只剩一张图）：

1. 入口页载入一张已保存的图 → 启动 SLAM；
2. 跑一段，**不要**让它和旧图合并（或者不走旧图覆盖的区域）→ 结束 → 保存；
3. `.osa` 里就有旧图 + 新图两张（`askMapSave` 会提示「N 张图」）。

| # | 做什么 | 期望 |
|---|---|---|
| ① | 打开查看器 | 列表里有旧图和新图；默认选中当前图 |
| ② | 切到**没合并的旧图** | 点云/历史轨迹/路标点都能看；**不画当前位置**，标题提示「当前位姿在图 #新」 |
| ③ | 点 `当前` | 切回新图，琥珀点 + 朝向三角出现 |
| ④ | SLAM 跑着时看当前图 | 位置点跟着动；白线在长；`刷新` 后点云更新；切片「高度」跟小地图同层，切「中位数」变成这张图点云的中位高度 |
| ⑤ | 停止 SLAM 后看旧图 | 图照常可看；没有当前位置点 |
| ⑥ | 单指拖 / 双指捏 / `+` `−` / `适应` | 拖动跟手；捏合以两指焦点为中心；按钮 ×1.5；`适应` 回到整图 bbox；双击没反应 |
| ⑥a | 捏合**中途先抬起一根**，剩下那根继续拖 | 不跳（第一版这里会瞬移，见「改动」最后一条） |
| ⑦ | 有一张 0 关键帧的空图 | 不出现在列表里 |
| ⑧ | 查看器开着时触发合并 / 存图 | 列表 2 秒内更新；被吸收的图消失，选中项自动切走 |
| ⑨ | `SLAM_IDLE` / `LOADING` / 恢复 atlas 期间打开 | 明确提示，不崩、不读半成品 atlas |
| ⑩ | 回归 | 小地图 / 历史轨迹 / 路径点 / 测距 / 折线 / 存图 / 缩放 |

日志：切图一行 `[VIEW] 图 #id 点云 N 点，Xms`；列表只在 id 集合变化时打
`[VIEW] atlas 图列表：…`；点云枚举 >20ms 打 warning。

## 遗留

- **点云是一次性快照**：当前图在跑的时候点会变，要按「刷新」重读；没有自动刷新。
- **主线程枚举**：当前图正被 LocalMapping 改时，切图可能等它一下（几毫秒级）。
- **一个大 atlas（很多图）**：列表每 2 秒刷新、每次切图读一次点云；没做按图缓存。
- **同一张图里后续的回环/BA 会挪地图**，历史轨迹文件不会跟着更新；查看器也不重新变换。
  和上一批历史轨迹同一条限制。
- **没做平移/缩放状态的持久化**：每次打开重新 `fit`。
- **没做图的重命名/删除**：查看器只读。
- **点云密度**：沿用 `kMapPointsMax = 5000` 的抽稀；很近的细节看不出来。

## 回滚

```
thirdparty/orb_slam3/{include/System.h,src/System.cc}
app/src/main/cpp/{slam_engine.h,slam_engine.cpp,jni_bridge.cpp}
app/src/main/java/com/example/slamcapture/{SlamBridge,MapView,MainActivity}.kt
app/src/main/res/layout/activity_main.xml
thirdparty/README.md（补丁 32）、docs/…
```

没有 `.osa` / 磁盘格式改动；回滚只影响界面和三个只读接口。
