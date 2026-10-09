# 会话与轨迹的保存：开关、弹窗、一趟一个目录

> 用户定的顺序：**先把「保存会话」和「轨迹」做完**，做完他提交、开分支，
> 再去查初始化为什么反复。**这次不动 git**（用户明确说的）。

## 动机

用户在 review M5 的抽屉时提了四件事：

1. **抽屉里那个「录制」应该是开关**，不是点击按钮 —— 而且**打开了启动 SLAM 就该自动录**
2. **停止 SLAM 时弹窗问「是否保存这一趟」**
3. **轨迹保存到一个文件夹里**
4. 一个设计问题：**2D 地图的轨迹和 SLAM 生成的轨迹是不是一个东西？能不能当一个用？
   轨迹怎么和地图对应？**

查这个设计问题时，顺手发现 M5 那份轨迹文件**是坏的**（见「决策三」）。

## 改动

### C++

| 文件 | 改什么 |
|---|---|
| `thirdparty/orb_slam3/include/System.h` | **加一行** `unsigned long KeyFramesInMap()` —— 上游没有，重置检测需要它 |
| `orb_track.{h,cpp}` | `TrackResult` 加 `trackingState` + `kfCount`；**`valid` 收紧成「只在跟踪 `OK` 时才算数」** |
| `slam_engine.{h,cpp}` | 重置检测（KF 数掉到 0）→ **清空轨迹** + `resetCount++`；`start()` 清轨迹和计数；新增 `resetCount()` / `trajSize()` |
| `jni_bridge.cpp` | 加 `nativeResetCount` / `nativeTrajSize` |

### Kotlin

| 文件 | 改什么 |
|---|---|
| `res/layout/activity_main.xml` | 抽屉里的 `Button` → `SwitchCompat`（「录制落盘」） |
| `MainActivity.kt` | 开关语义；**启动 SLAM 时若开着就自动录**；停止时弹窗；`files/maps/<时间戳>/` 目录；`session-N/` 分段；`run.json` |
| `SlamBridge.kt` | 加两个 `external fun` |

### 目录结构

```
files/
├── sessions/                    # 录制模式录的（没有图）—— 不动
└── maps/
    └── 20260929-143022/         # 一趟 = 一个目录
        ├── trajectory.txt       # 轨迹（只含跟踪 OK 的帧）
        ├── run.json             # 时间、图来源、段数、轨迹点数、重置次数
        ├── session-1/           # 开关第一次开→关
        │   ├── cam0/  imu0/  meta.json
        └── session-2/           # 又打开
            └── …
```

## 决策

### 一、2D 地图的轨迹 **和** SLAM 的轨迹，是同一个东西

2D 地图上那条线就是 SLAM 轨迹的**俯视投影**（丢掉高度、转顶视）。**所以一个文件，不用区分名字。**

### 二、⭐ 「目录 = 坐标系；文件名 = 内容」

`Twc` 是**在地图的坐标系里**的 —— 脱离那张图，轨迹只是一串相对量。所以：

- **轨迹**和**测量标记**（marks，2D 地图上点选出来的锚点）**不是同一样东西**，但**共享同一个坐标系**
- ⇒ **同目录、不同文件名**。这样「把轨迹投到 2D 地图上」「把标记投上去」都只是读同一个目录

### 三、⚠️ 而且 M5 那份轨迹**本来就是坏的**

在 `System.cc:778` 找到了 ORB-SLAM3 自己的做法：

```cpp
void System::SaveTrajectoryEuRoC(const string &filename, Map* pMap)
{
    cout << "Saving trajectory of map " << pMap->GetId() << endl;   // ← per map
    Sophus::SE3f Twb = vpKFs[0]->GetImuPose();   // ← 锚定到【这张图的第一个关键帧】
    // Frames not localized (tracking failure) are not saved.      // ← 没定位的帧不存
```

对照 M5 那份 224 行的轨迹：

| 现象 | 原因 |
|---|---|
| **212 行是 `0 0 0` 单位位姿** | **我们在存「没定位的帧」** —— 那时 `TrackMonocular` 返回的是单位阵，`allFinite()` 照样过。`SaveTrajectoryEuRoC` 明确不存 |
| **12 个非零位姿** | 12 次初始化成功，**夹在 12 段互不相容的坐标系之间** |

**那份文件不是「算得差」，是把 12 段不同坐标系的数据拼成了一个文件。**
两处都修了：`valid` 只认 `OK`；重置即清空。

### 四、重置就清空轨迹 —— **不做「纪元」**（用户定的）

我原本想的是保留多段、每段带 `(mapId, epoch)`。**用户指出**：

> 如果重置了，那之前轨迹也没用啊

**对，而且比这更硬**：`ResetActiveMap` 是**原地清空**（`Atlas::clearMap()` → `mpCurrentMap->clear()`），
**`Map::mnId` 不变**，但坐标系**重新锚定**到新的首关键帧。所以：

- **前一段本来就没用**（拼不到一起）
- **`mapId` 不变也没关系** —— 永远只有一段自洽的轨迹，没有「要区分是哪张图」的场景

**结论：见到重置就 `traj.clear()`。** 比带纪元简单得多，而且更诚实。

> **副产品**：`resetCount` 给 UI 显示 —— **「地图已重置 N 次」就是初始化问题最直接的证据**，
> 比翻 logcat 直观得多。这是下一步查初始化问题的抓手。

### 五、目录名用**时间戳**不用 `mapId`

`mapId` 会随进程重启和新建图变；时间戳稳定。而且「另存为新图」天然就是新目录（M8）。
用户：「目录结构现在怎么方便怎么来，后续应该挺好改的吧」—— 对，它就是一个 `newRunDir()`。

### 六、⚠️ 每段写自己的 `session-N/`（用户选的）

**用户问出来的**：「如果开始录制 SLAM 启动了又点击关闭，程序会怎么处理？」

查下来发现**「关掉再打开」会毁数据**：`SessionWriter.start()` 用 `FileOutputStream`
打开 `cam0/data.csv` —— **截断**。而上一段写的 PNG 还在磁盘上：

| 磁盘上 | 内容 |
|---|---|
| `cam0/data.csv` | **只有第二段的帧** |
| `cam0/data/*.png` | **两段的 PNG 都在** |

**csv 和 PNG 对不上，而且坏得很隐蔽**（PC 那边读 csv 只看到第二段，像是"少了一段"）。

**用户定的语义：停写保留、重开另起一段。** 所以每段一个新目录 `session-1/`、`session-2/`…

### 七、⚠️ 「不保存」必须是**明确动作**

弹窗里点外面 / 按返回 **都算保存**。删数据不能靠手滑，只能靠明确点「不保存」。

### 八、地图的「另存为新图 / 保存为原图」是 **M8**，不是漏了

用户提的是**三选一**（另存为新图 / 保存为原图 / 不保存）。查了 `SaveAtlas`：

```cpp
// System.cc:1410
string pathSaveFileName = "./";
pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
```

**路径硬写成 `"./"`** —— 安卓上 CWD 不可写；而且 `mStrSaveAtlasToFile` 来自 yaml。
**现在根本没有「图」可以存或载入**，所以对话框现在只做「保存 / 不保存」两选一。
**弹窗文案里已经注明这一点**，M8 再补成三选一。

## 验证

### 1. JNI 符号双向对齐 ✅

```bash
$ comm -23 <(Kotlin 的 external fun) <(C++ 的 Java_…_)   # Kotlin 有 C++ 没有
（空）
$ comm -13 …                                             # C++ 有 Kotlin 没有
（空）
新增的两个：
  nativeResetCount   Kotlin 1 / C++ 1
  nativeTrajSize     Kotlin 1 / C++ 1
```

**15 个符号，两边逐个对上。**

### 2. 残留引用检查 ✅

`btnRecord` / `lastSessionDir` / `newSessionDir` 在 `MainActivity.kt` 里已清空
（`grep` 命中的那几处在 `CaptureActivity.kt` —— **录制页自己的同名函数**，不是残留）。
`toggleRecording` / `lastTrajectory` 已完全移除。

### 3. 通读了 `MainActivity.kt` 的结构 ✅

类在最后一行闭合；`openHardware` / `stopHardware` / `render` / 新增的几个 helper 逐个看过。
（在这个文件上因为括号吃过两次亏，所以这次逐段读。）

### 4. ❌ **没有编译**

按约定编译是用户跑的。**C++ 和 Kotlin 都还没过编译器** —— 所以上面那些是**静态检查**，
不是「跑过了」。要验的话：

```bash
cd android/SlamCapture && ./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

**已知最容易在这里出错的地方**：`System.h` 那一行访问器要能编过
（`mpAtlas->KeyFramesInMap()` 是 `Atlas` 的公开方法，`System.h` 已经 `#include "Atlas.h"`）；
以及 `orb_track.cpp` 里 `kTrackingOk` 那个匿名 namespace 常量。

## 补充（同日）：⚠️「一趟一个目录」是错的 —— 改成「一次会话一个目录」

**用户报的**：

> 我明明还在同一个会话里，又没有退出去重新点击新建地图，为什么地图文件夹不一样了

设备上确实是两个，而且都是「新建地图」：

```
20260929-210256   created 21:03:32   sessions:1  resets:7
20260929-213214   created 21:33:36   sessions:0  resets:16
```

**根因**：会话目录（当时叫 `runDir`）建在 `openHardware()` 里 —— **每次按「启动 SLAM」都建一个**。
所以「启动 → 结束 → 再启动」就裂成两个文件夹。

**用户的模型才是对的：一次「新建地图」= 一个文件夹。**

### 新结构

```
files/maps/20260929-210256/     ← 一次「新建地图 / 打开地图」= 一个文件夹
├── session.json                 ← 会话汇总（写一次）
├── run-1/                       ← 坐标系单元 = 一次「启动→结束」
│   ├── run.json                 ← 轨迹点数 / 重置次数 / 段数
│   ├── trajectory.txt
│   └── session-1/  session-2/   ← 录制开关拨动产生的段
└── run-2/
```

### 改了什么

- 会话目录**第一次「启动 SLAM」时建、之后复用**（`sessionDir`，跟着 Activity 生命周期）
- 每次「启动→结束」在它下面开一个 `run-N/`
- ⚠️ **顺带修好一件事**：「不保存」现在**只删 `run-N/`**，不会连带删掉前面已经保存的那几趟
  （原来整个目录一起删 —— 那是个隐患）
- 会话目录丢空了就一并删掉，免得留一地空文件夹

### 再补充：丢轨迹时**单独问一次会话**（用户定的第二版）

**用户先提的是**「sessions 统一放到 map 的 sessions 目录下，不然你点丢弃 sessions 也丢了」，
**随即自己否了**：「不行，但这样 run 和 session 编号对不上」——

> 只是不保存轨迹的时候加一个逻辑，如果录制了 session 就再询问一下要不要保存 sessions，
> 要的话就只不要轨迹相关内容，不要的话就全删了

**理由是对的**：录一段会话得**走一趟**才录得到，成本比轨迹高得多，不该跟着轨迹一起没。

**最终的交互：**

```
按「结束」
  └─ 「保存这一趟？」  [保存] [不保存]
       ├─ 保存 ──────────────► 轨迹 + 会话都留
       └─ 不保存
            └─ 这一趟录过会话吗？
                 ├─ 没录 → 整个删掉
                 └─ 录过 → 「这一趟录了 N 段会话，要保留吗？」
                            ├─ 保留会话 → 只删 trajectory.txt + run.json
                            └─ 全部删除 → 整个删掉
```

**结构因此不动**（`run-N/session-M/` 保持不变）—— 用户选的：**编号对齐优先**。

### ⚠️ 编译期抓到的两个错（我手上没编译器，都是用户/IDE 报的）

| 报错 | 原因 |
|---|---|
| `Function invocation 'nowStamp()' expected` | 字符串模板里写成了 `"$nowStamp()"` —— Kotlin 解析成「变量 `$nowStamp` + 字面量 `()`」。**函数调用必须写 `${nowStamp()}`** |
| `Expecting member declaration` | KDoc 里写了 `` `session-*/` ``，**里面的 `*/` 把注释提前结束了**，后面的 `。 */` 变成了代码 |

**两个都已经扫过全文件确认只有那一处**，并写进了检查脚本：字符串模板里 `$函数名(` 的形式、
以及一行里出现两次 `*/` 的行。
- 两个 `json` 故意不同名（`session.json` / `run.json`），免得会话级和趟级混淆

**为什么 `run-N/` 才是坐标系单元**：`nativeStart()` 会清空 C++ 的轨迹，重置时又清一次 ——
所以 `run-N/trajectory.txt` 内部一定自洽。会话目录只是把它们收在一起。这比原来那句
「目录 = 坐标系」更准。

### ⚠️ 我这轮又犯了一次同类错误

重构时删掉了 `nextSessionDir()` 的**定义**，但**忘了它还有调用**（`startRecording()` 里）——
那是编译错误。写记录时扫出来的。

所以这次补了一道全量检查：**所有被调用的私有方法是否都有定义、定义的是否都被用到**
（脚本见下），把这类错在报告之前挡住。

```python
# 去掉注释和字符串后，比对「fun 定义的名字」和「被调用的名字」
defined = set(re.findall(r'\bfun\s+([A-Za-z_]\w*)\s*\(', body))
called  = set(re.findall(r'(?<![\w.])([a-z][A-Za-z0-9_]*)\s*\(', body))
```

## 遗留

- **轨迹和 2D 地图的「对应」只做了一半** —— 目录结构（同目录 = 同坐标系）定了，
  但 **2D 地图视图本身还没做**（`APP_DESIGN` §5）。做的时候直接读 `trajectory.txt` 投影即可
- **`marks.json`（测量标记）还没有** —— 目录里留了位置，等阶段六
- **地图的另存/覆盖**（M8，见决策八）
- **`session-N/` 会随开关拨动增长** —— 一次运行最多产生几个目录。用户选的语义，先这样
- **切后台也停止、也开新 `run-N/`** —— 来回切会留下一串只有一两个点的 run。没做清理，
  等实际用起来觉得烦再说
- **⚠️ 21:32 那次重录没有数据** —— `sessions: 0`（录制开关没开），`trajectory.txt` 0 字节。
  所以那一趟分析不了，只留下 `map_resets: 16` 这一个信息。**下次要开着开关录。**
- **初始化问题**（下一步）：`resetCount` 上屏之后，它就是最直接的证据

## 回滚

**整个功能一起退**（它是一件事）：

1. `thirdparty/orb_slam3/include/System.h` —— 删掉 `KeyFramesInMap()` 那一行
2. `orb_track.{h,cpp}` —— `TrackResult` 删掉两个新字段；`valid` 改回只判 `allFinite()`
3. `slam_engine.{h,cpp}` —— 删掉重置检测、`resetCount`、`trajSize`；`start()` 恢复成
   只 `running.store(true)`
4. `jni_bridge.cpp` / `SlamBridge.kt` —— 删掉两个符号
5. `activity_main.xml` —— `SwitchCompat` 改回 `Button`
6. `MainActivity.kt` —— `git checkout` 取回 M5 那版

**只回滚「轨迹按图存、只存 OK 帧」这一条**：`orb_track.cpp` 里把
`if (out->trackingState != kTrackingOk) return false;` 删掉即可 ——
但**那份轨迹又会变成一堆原点位姿**，不建议单独退。
