# 2026-10-30 小地图：加一个「实时轨迹」开关

## 动机

用户提的需求：

> 我现在想加一个当前轨迹显示的按钮用来控制小地图是否显示当前实时的轨迹

小地图（148dp）上一共三层：本趟轨迹、点云、历史轨迹。历史轨迹早就有开关
（`swHistory`），本趟轨迹没有 —— 想看清点云形状或跟历史轨迹对比时，本趟那条线一直压在上面。

## 改动

- `app/src/main/res/layout/activity_main.xml` — 抽屉「标记」分组里、历史轨迹开关之上加
  `MaterialSwitch swTraj`「实时轨迹」，默认开。
- `app/src/main/java/com/example/slamcapture/MainActivity.kt`
  - `PREF_MAP_TRAJ` 记住状态；在注册监听前恢复，与其它开关一致。
  - 监听里只做「存偏好 + `render()`」：ticker 停了（SLAM 未运行）时也要立刻生效。
  - `refreshMap()`：关掉时 `n = 0`，**跳过 `nativeGetTrajectory`**，不读出来再丢。
  - 提示文案：「等待定位…」加 `swTraj.isChecked` 前置条件。

## 决策

- **采用：放抽屉里，不做小地图上的图标按钮。** 小窗只有 148dp，右下角已经有放大/缩小两个键，
  再塞一个会把可视面积吃掉；而且这个开关是「看的时候调一下」，不是走图时频繁按。
  用户也是这么选的。
- **采用：关掉时不读 native，而不是读完丢结果。** `getTrajectory()` 要拿
  `Engine::trajMutex`（`slam_engine.cpp:1170`），而小地图本来就是 4 Hz ticker 在调，
  能省一次加锁就省。代价是 `trajBuf` 里留着旧数据 —— `Snapshot.trajN` 为 0，不会画出来。
- **采用：主动隐藏不算「没内容」。** 原逻辑里「轨迹为空 + 点云为空 + 无历史」会弹
  「等待定位…」。用户自己把轨迹关了，小地图就该安静地空着，而不是报一个假状态。
  「SLAM 未启动」「跟踪未就绪」两条保留 —— 那是真状态，跟开关无关。
- **不采用：把开关做成 MapView 的 Snapshot 字段。** `trajN = 0` 已经是天然的「不画」，
  再加一个布尔是想表达同一件事的第二条路。

## 验证

**未构建、未上机。** 只确认了改动逻辑；编译和真机验证由用户执行（见下）。

```
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

预期：抽屉里「标记」分组多一个「实时轨迹」；关掉后小地图上本趟的那条线消失，
点云和当前位置箭头仍在；退出重进保持关闭；全屏地图查看器不受影响。

## 遗留

- 开关只管**小地图**。全屏地图查看器的当前轨迹层（`refreshMapViewer()`，口径是
  「这张图有没有数据」）没有跟着关 —— 那是另一个视图，等有需求再统一。
- 隐藏的是绘制，不是记录：轨迹照常写盘、照常进 `session.json`，所以关着跑一趟
  不会丢数据。

## 回滚

```
app/src/main/res/layout/activity_main.xml      # 删掉 swTraj 那段
app/src/main/java/com/example/slamcapture/MainActivity.kt
                                               # PREF_MAP_TRAJ、字段、findViewById、
                                               # 恢复与监听块、refreshMap 的三处判断
```

`PREF_MAP_TRAJ` 留在 SharedPreferences 里无害，不用清。
