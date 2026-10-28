# 2026-10-28 内参：按对焦屈光度索引的 profile + 启动前注入

一句话：内参不再写进 yaml，改由 App 的标定表按**对焦屈光度**插值后，在 `nativeStart()`
之前直接注入 ORB-SLAM3；一张地图记录一套内参，重新打开时直接用记录值。

这一批是 M1 + M2（标定页 M3 还没做），目标是让"换到别人的手机"这件事不再依赖 PC。

## 动机

三个具体问题叠在一起，导致部署到别的手机上基本不可能：

1. **部署的内参从没换过**：`assets/phone.yaml` 里一直是 `fx=435.2`、畸变全 0，
   而 9-20 实测是 `fx=439.227`、`k1=0.0182`。在 640×480 上 k1 缺失等于画面角落
   约 6 px 的系统偏移。
2. **`T_b_c1` 旋转是按本机 90° 写死的**：换一台 `SENSOR_ORIENTATION=270` 的手机，
   矩阵错两个轴，现象和"标定不准"一样，但其实是初始化都不对。
3. **fx 随对焦距离变，而锁在哪儿是 AF 决定的**：每次会话锁在不同屈光度；
   还出现过锁在 0.1 m、整段远景全糊的情况（原来只拦了 `dist <= 0`）。

## 改动

**profile（新文件 `CalibProfile.kt`）**

- 位置 `外部文件目录/calib/profile.json`，一行 = 一个屈光度：`d / fx / fy / cx / cy /
  k1..p2 / source / rms / views / date`。
- 取内参用**线性插值**（`fx ∝ 1 + f·d`，镜组位移与屈光度成正比），超出两端夹到最近一行
  —— 这时等价于"取最近"。`k2/p1/p2` 量小又难辨识，直接取最近那一行的整份。
- 兜底两级：`assets/phone.yaml` 的 `Camera1.*`（source=estimated）、
  `LENS_INTRINSIC_CALIBRATION`（source=factory，按 active array 缩放，畸变留 0）。

**settings（新文件 `SettingsTemplate.kt`）**

- yaml 只替换两处：`IMU.T_b_c1`（按 `SENSOR_ORIENTATION` 现算）和 `IMU.Frequency`。
- **`Camera1.*` 一个都不改**：内参真正生效的路径是注入，不是文件。
- 90° 那组矩阵和原来手推的逐位一致，本机行为不变。

**注入（上游补丁 33）**

- `System::SetCameraIntrinsics` → `Tracking::SetCameraIntrinsics`：
  改 `mpCamera` 的四个参数 + 重建 `mK / mK_ / mDistCoef` + 置 `Frame::mbInitialComputations`。
- 只允许在 `nativeStart()` 之前调：帧要 `slamRunning` 才喂，之前上游没有并发读。
- 同时刷新 `measure_center` 缓存的 `gFx…gCy`（那边有 `gFx != 0` 早退）。

**时序（`MainActivity`）**

```
「准备」  nativeInit（yaml 只带外参和 IMU 频率）
「启动」  开相机 + IMU → beginSettle
          旧图：读 session.json 的记录 → 注入 → nativeStart
          新图：等 focusDiopter（≤3 s，或用户点「锁定并开始」）→ 插值 → 注入 → nativeStart
```

- `openHardware()` 拆成 `openHardware` / `finishStart` / `startTracking` 三段，
  等对焦由现有 ticker 推进，按钮在这期间显示「锁定并开始」。
- 等对焦期间帧不喂（`slamRunning` 闸），不会有任何一帧用错内参。

**记录（一张图一套内参）**

- 地图目录的 `session.json` 多一段 `intrinsics`（屈光度 + 全部系数 + model/camera_id + 时间）。
- 打开旧图**直接读记录**，不信当前对焦；有屈光度就把镜组命令回那个位置。
- 「另存为」本来就复制 `session.json`，记录自动跟着走。

**对焦（`CameraCapture`）**

- `pinnedDiopter`：命令式固定屈光度；`focusDiopter` / `focusCommanded` / `focusNote` 对外可见。
- `latestDistance` 每帧都更新，`lockFocusNow()` / `releaseFocus()` 供手动锁定。
- 自动分支加边界：收敛值离内参表最近的档超过 1 D 时**不锁**、只提示。

**诊断**

- `logCapabilities` 补 `SENSOR_ORIENTATION`、`LENS_INTRINSIC_CALIBRATION`、
  `LENS_INFO_FOCUS_DISTANCE_CALIBRATION`、`CONTROL_AF_AVAILABLE_MODES`、active array。
- `meta.json` 加 `calib` 段（focus_mode / diopter / source / fx/fy/cx/cy/k1）。
- 诊断面板常驻「内参 …」和「对焦 …」两行。

## 决策

**① 内参不进 yaml，走注入。**
Settings 在构造时按 `Camera1.*` 先造一个 Pinhole，但它在 `nativeStart()` 之前不参与计算。
所以文件里写什么无所谓，真正生效的是注入那一刻 —— 内参只有一个真相（profile / 地图记录），
改内参不必重写文件，也不会因为文本替换写错一行而静默出错。

**② 否决：让 C++ 直接读 profile.json。**
那要在 native 侧再引一个 JSON 解析器，而 Kotlin 已经会解析、只要过 8 个 float。
多一份解析就多一处会不一致的地方。

**③ 一张图一套内参，不做运行中热切换。**
老 KeyFrame 的内参是 `const`、构造时拷贝，换掉共享相机会让同一张图混用两种内参：
每个残差自洽，但尺度会跳一次，而且这个错误不可见。所以只允许在开跑前注入。

**④ 否决：词袋缓存 + `System(ORBVocabulary*, …)` 重载。**
它能把重建从 38 s 降到 ~1 s，但只在"换图 / 重开 System"时用得上，和"一张图一套内参"
不冲突也不必需。本次不做。

**⑤ 用现有的「启动」对焦环节，不新增预检。**
原本打算在「准备」前开关一次相机读屈光度；改成在 `openHardware` 里等 ——
少一次开关相机，而且用户能看见"对焦中"并手动锁定。

**⑥ 出厂内参只取 fx/fy/cx/cy，畸变留 0。**
`LENS_DISTORTION` 的系数顺序没有可靠来源可核（没网、SDK 源码也没装）。
宁可不带畸变，也不猜一个可能写反的数组；界面标"出厂值"。

**⑦ 自动对焦不再无条件接受。**
原来只拦 `dist <= 0`，锁到 0.1 m 也照跑。现在收敛到表外 +1 D 就只提示、不锁。

## 验证

**2026-10-28 真机构建 + 跑过**（小米 PJD110）。IDE 诊断对这些文件为空：
`MainActivity.kt`、`CameraCapture.kt`、`CalibProfile.kt`、`SettingsTemplate.kt`、
`SessionWriter.kt`、`slam_engine.{h,cpp}`、`measure_center.{h,cpp}`、
`orb_slam3/{System.h,System.cc,Tracking.h,Tracking.cc}`、`jni_bridge.cpp`。

### 实际日志

第一趟（新建地图，没有 profile）：

```
CameraCapture: 传感器方向: 90  active array: Rect(0, 0 - 4096, 3072)
CameraCapture: 出厂内参 fx,fy,cx,cy,s: 2705.357,2705.3572,2048.0,1536.0,0.0
CameraCapture: 对焦距离标定: 1   AF模式: 0,1,2,3,4
CameraCapture: 对焦完成 AF_STATE=2，镜组锁定于 0.303 屈光度 (= 3.30 m)
MainActivity : 内参（模板兜底）：fx=439.227 ... d=0.30
slam         : [M6] 内参更新 fx=439.227 ...
slam         : [M5] 内参注入 fx=439.227 ...
CalibStore   : 地图内参已记录：…/maps/20261028-202329/session.json fx=439.227
```

第二趟（回入口页重新打开同一张图）：

```
MainActivity : 内参（地图记录）：fx=439.227 ... d=0.30
slam         : [M5] 内参注入 fx=439.227 ...      ← 在「SLAM 开始吃帧」之前
MainActivity : SLAM 开始吃帧
CameraCapture: 对焦固定于 0.303 屈光度（命令 0.303）
```

三件事被证实：

1. **设备认命令式焦距**：命令 0.303 → 回读 0.303，逐位相符（`对焦距离标定: 1` = APPROXIMATE）。
   整条"按屈光度索引"的路成立。
2. **注入时序对**：`内参注入` 在 `开始吃帧` 之前，期间没有帧被喂进去。
3. **记录能复用**：第二次打开这张图走的是 `地图记录`，镜组被命令回同一个屈光度。

生成的 `phone.yaml`：`IMU.T_b_c1` 与原来手推的 90° 那组逐位一致；`Camera1.*` 保持模板值
（没被改，符合设计）；`IMU.Frequency: 402.0`。

`session.json` 里的记录：

```json
"intrinsics": { "diopter": 0.30330169, "fx": 439.22699, "fy": 439.37701,
  "cx": 318.42700, "cy": 239.85600, "k1": 0.018222,
  "source": "estimated", "model": "PJD110", "camera_id": "0",
  "recorded": "2026-10-28T20:23:29" }
```

### 跑完之后又改了两处（**未重编验证**）

- 没有 profile 时 `autofocusMaxD` 兜底 2.5 D —— 否则对着桌面点「启动」会把
  "锁在 0.1 m"永久记进这张图；
- 超出标定范围时不锁，按默认 1.0 D 取内参、让画面保持 AF 当前的样子。

两条都是防"近处锁死"的，不碰已验证的主链路。

### 顺带查出来的

- **出厂内参是名义值，不可信**：`2705.357 = 6.06/9.17504 × 4096`，就是焦距/传感器尺寸算出来的，
  `cx/cy` 正好是 active array 中心。缩放回 640 是 **422.7**，比实测 439.2 低 **3.8%**。
  所以 `factory` 这档兜底不比模板强，注释和界面上都要说清楚。
- **每次「启动」都会 `Timestamp jump detected` → 地图重置一次**：System 复用、帧时间戳跳了
  十几秒。这是既有行为（和本次改动无关），但会让第二趟多一次重建图；单独立项查。
- **IMU 初始化要求开头持续平移**：这趟 36 秒里有一次
  `Not enough motion for initializing. Reseting...`，第二次才成功。和
  [ORB_SLAM3_TRACKING_FLOW.md](../tutorial/ORB_SLAM3_TRACKING_FLOW.md) §八 记的是同一件事。

请执行：

```bash
cd /home/welkin/SLAMAPP/SlamCapture
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb logcat -s CameraCapture:I MainActivity:I CalibStore:I slam:I
```

| # | 做什么 | 期望 |
|---|---|---|
| ① | 冷启动 → 准备 | 日志有 `传感器方向=90`、出厂内参、对焦距离标定值 |
| ② | adb pull `files/phone.yaml` | `T_b_c1` 与原来逐位一致（本机 90°）；`Camera1.*` 保持模板值 |
| ③ | 点「启动」 | 按钮先变「锁定并开始」；对焦收敛后自动开跑，日志有 `内参（档位插值）…` |
| ④ | 对着 0.2 m 的桌面点「启动」 | 不锁在近处：要么按表取近档，要么提示"离最近的标定档差 X D" |
| ⑤ | 跑一段 → 看 `session.json` | 多出 `intrinsics` 段，值与日志一致 |
| ⑥ | 回入口页重开这张图 → 启动 | 日志 `内参（地图记录）`；镜组回到记录的屈光度 |
| ⑦ | 看 `meta.json` | `calib` 段有 focus_mode / diopter / fx / k1 |
| ⑧ | 手改 profile 里唯一一行的 fx，再新建地图 | 值的改变直接反映到测距读数 |
| ⑨ | 回归 | 录制、存图、历史轨迹、路径点、测距、地图查看器 |

## 遗留

- **标定页（M3）还没做**：现在只有兜底两级（模板值 / 出厂值），
  `CalibStore` 已经支持多行、插值、增删，缺的是 App 内标定本身。
- **`meta.json` 的 `calib` 段没验**：这一趟没开录制，下次带录制跑一次就行。
- **出厂值实测是名义值**（见上），`factory` 这条兜底实际没什么用，M3 做完就可以不管它。
- **IMU 频率没有现测**：用 profile 里的 `imu_frequency_hz`，没有就按 402 Hz。
  换到 200 Hz 的手机上这个值可能是错的，M3 的标定页应该一起量出来。
- **A4 板在 640×480 下检测余量不大**，M3 第一步要单独验证。
- **跨设备载图**：`session.json` 记了 model / camera_id，但界面还不会比对和警告。
- **`LENS_DISTORTION` 系数顺序未核**，见决策 ⑥。
- **测距侧的内参缓存**只在注入时刷新；如果以后有人绕过注入改 yaml，两者会不一致。
- **GPL**：分发给别人（哪怕只是装到朋友手机上）触发 ORB-SLAM3 的 GPLv3 义务，
  需要能提供对应源码。`CLAUDE.md` 里"自用、不对外分发"的前提已经不成立。

## 回滚

```
app/src/main/assets/phone.yaml
app/src/main/java/com/example/slamcapture/{CalibProfile,SettingsTemplate}.kt（新）
app/src/main/java/com/example/slamcapture/{CameraCapture,MainActivity,SessionWriter}.kt
app/src/main/cpp/{slam_engine.h,slam_engine.cpp,measure_center.h,measure_center.cpp,jni_bridge.cpp}
app/src/main/java/com/example/slamcapture/SlamBridge.kt
thirdparty/orb_slam3/{include/System.h,src/System.cc,include/Tracking.h,src/Tracking.cc}
app/src/main/cpp/thirdparty/README.md（补丁 33）
```

没有磁盘格式变更：`.osa`、轨迹、录像都不受影响；`profile.json` 和 `session.json`
里新增的 `intrinsics` 只是附加字段，删掉即回到"用模板值"。
