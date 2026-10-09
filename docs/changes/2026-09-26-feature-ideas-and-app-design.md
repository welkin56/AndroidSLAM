# 2026-09-26 功能设想评估与 App 形态定案

> **本文件是重写版，且改过名**（原名 `2026-09-26-map-feature-ideas-assessment.md`）。
> 原版是同一天写的，含三条被推翻的论断，已打过一次补记；再追加就是
> 「正文 + 补记 + 补记」的糊状物，所以按用户指示**删掉重写**。
> 之后讨论又推进了几轮（`SlamEngine`、测距交互、2D 地图），所以在**当天结束前
> 又重写了一次**，把最终状态一次记全。
>
> 「被推翻的判断」那一节完整保留 —— 按 CLAUDE.md，那是改动记录最有价值的部分。

## 动机

用户提了三个功能设想，问可行性：

1. 输出水平面地图（设备高度或指定高度），并标出当前位置
2. 对保存的地图做 AR 导航，虚拟箭头渲染路线
3. SLAM 时实时指示"还没扫描建模到的部分"

并澄清了项目初衷：读完《SLAM 十四讲》想做点东西，**但更希望做出来的东西有价值、
能给生活带来方便**，因此开始质疑"测距"的实用性。

讨论持续一整天，方向**多次收敛**，我**被推翻七次**（见最后一节）。

## 改动

**无代码改动。** 文档的最终形态：

| 文件 | 动作 |
|---|---|
| `tutorial/APP_DESIGN.md` | **新建** —— 已定的 App 形态（502 行）。**不属于任何单一阶段**，所以单独立一份 |
| `tutorial/STAGE6_MEASUREMENT.md` | **新建**，取代 `STAGE6_PHOTO_MEASUREMENT.md`（已删） |
| `tutorial/MAP_FEATURE_IDEAS.md` | 重写。**一度打算删掉**（设想阶段的产物，使命已完成），但结论是**留着**：里面还有"地图持久化"等**框架事实**和几条**别处没有承接的分析** |
| `tutorial/STAGE5_JNI_BRIDGE.md` | 改动大：`SlamWrapper` → **`SlamEngine`**（含图管理）、§5 整节重写、接口加 `nativeStart/Stop/Relocalize`、**新增 M8** |
| `tutorial/STAGE7_UI_STATE_MACHINE.md` | 加相机/IMU 挂载的**修订说明** |
| `tutorial/ANDROID_SLAM_TUTORIAL.md` | §7 改成实时测量、§8 加判据、路线图与执行顺序 |
| `docs/README.md`、`CLAUDE.md` | 相应更新。**`CLAUDE.md` 不走版本控制**，改了退不回来 |

## 决策

分成五组。

### 一、功能定位（三条设想各收敛一次）

**剔除 AR 导航**：难点**不在渲染，在路径规划**——稀疏点云**不构成可通行空间**，
A*/Dijkstra 无从下手，得先建占据栅格。**这是三个设想里唯一需要新算法的部分。**

**「室内定位」降级为「迷宫路线记录」**：

| 需要什么 | 室内定位 | 路线记录 |
|---|---|---|
| 重定位 / 地图复用 | 必需 | **不需要** |
| 绝对精度 | 要 | **不要**，只要相对形状对 |
| 尺度误差 ±2% | 有影响 | **无所谓** |

**代价是放弃"知道自己在建筑里的绝对位置"**，换来「全部落在零框架改动那一批里」。

**「未扫描区域」提升为「SLAM 可观测性」**（⚠️ 我推翻了自己）：原判断是"该并进阶段六"，
**错在**它不只是服务测距——它针对的是这个项目**真正的历史痛点**：
采了 20 秒全废但当时不知道、1/3 概率失败但事后才发现、花好几轮从日志反推 `mTinit`。

**新增「随身尺寸测量」**（触发点是用户一句「不可能总是带着一把尺子」）。

### 二、价值定位的纠正

**⚠️ 纠正**：原版把「家具能不能进门」当核心价值 —— **那是别的会话幻想出来的，
不是用户意图**。实际需求是"**随身量个大概**"。

**连带**：参照物校正从「不是可选项」**降为可选**（那个论断的前提是"差 1cm 就不行"）。

### 三、架构：2×2 塌缩成一条流水线

用户最初提的是「SLAM 模式 / 测距模式」×「已有地图 / 未知地图」。
**两个轴正交，而且都不是"模式"**：做成模式会立刻遇到「我想边看轨迹边量」。

**切图（A）而非真合并（B）**：理由不是"容易"，是 **B 想解决的问题在 A 的语义下
不存在**（B 多出来的价值只是保住"我从门口走进商场那几步"）。

**重定位从「自动」降为「手动按钮」** ⭐ **用户提出，是本设计里最实质的简化**：
一次解决三个问题——不用改启动路径、不用设计失败回退、**场景选择交给站在现场的人**。
失败代价从「会话废掉」降到「再走两步按一次」。

**锚点**：`{ 名字, 位姿, 描述子 }`，把「在整张地图里找我在哪」降级为
「去找我记得的那个点」。**分两层**，第 1 层只是给人看的标记（零上游改动）。

> **📌 2026-09-30 更正（原文保留）：这一条记错了用户的意思。**
>
> 用户原话：「是为了解决混淆场景重定位问题时，我提出**人为地在没有混淆场景进行重定位**，
> **没有把它做进功能的打算**」；而**「锚点」这个词用户本意是「路径点」**（为了导航）。
>
> 也就是说：**上面这段把用户的一个临时思路当成了功能，还占用了他的词。**
>
> **现在的定案见 [APP_DESIGN](../tutorial/APP_DESIGN.md) §4**：
> 概念改叫**路径点**（`{名字, 位姿}`，**不带描述子**），一套 UI 两种用法（导航 / 提示在哪重定位）；
> 「限制候选范围」那层**先不做**，触发条件是「真的撞上定错」。

**`SlamEngine`** ⭐ **用户提出**：一个新的 C++ 封装类，持有 `System*` +
**`referenceMap_`（「旧图」）**，把三种来源（新建=空 / 载入磁盘 / 停止时那张）
收进**同一个字段**，并且**拥有「切图」这个 ORB-SLAM3 没有的能力**。

**「停止」= 结束会话**，那张图成为「旧图」；再启动靠**重定位**接回去，
**不自动续上**（停止期间手机可能移动了）。

### 四、生命周期：修订 09-21 的决定

09-21 把**相机和 IMU 绑在「准备」上**；**但 SLAM 也需要它们且要常跑**，所以改绑到
**「启动 SLAM」**。形态上：

```
入口页（新建 / 继续上次）  ← 只记选择，不加载任何东西
   ↓
启动 SLAM  ← 第一次才构造 System（词袋十几到几十秒）
   ↓
运行中  ⇄  停止（= 结束会话）
```

**判据是「这个资源的代价类型」**：**耗电的跟着用户意图走，耗时的跟着进程走**
（`System` 跨停止保留，否则每次开关都要重等词袋）。

**落盘（录制）是独立的第四个开关**，跟已有的采集端状态机走，**一行都不用改**。

### 五、测距与 2D 地图

**测距**：十字固定在画面中心 + **实时读数（零操作）** + 「添加标记点」。
**先做「到目标的距离」（a）**，两点距离（b）用同一套交互延伸，**不选模式**。
**不拍照**（原设计是拍照后点像素）、**结果不保存**。

> 顺带一个没预料到的好处：**十字固定 ⟹ 它对应的图像坐标恒定**，
> letterbox 和 90° 旋转都不改变中心，**不需要通用的反向映射**。

**2D 地图**三件事：

| | 决定 | 理由 |
|---|---|---|
| **边界** | **不做** | 稀疏点云里**没有"墙"**；膨胀成实线是**在假装**。斑驳才是诚实的 |
| **z** | **先做高度着色**，不做切片 | **"哪个 z 是地面"答不上来**（重力对齐给方向，原点在初始化位置；杆臂没标，拿相机高度当基准不划算） |
| **路径** | **必须存 3D** | **换 z 区间只是重新投影，记录不动**。只存 2D 就真丢了 |

**规则：记录事实（3D 位姿），显示是策略（怎么投影）。**

**文件布局**：`.osa`（地图，我们只读不写）+ `.anchors.json` + `.path.csv`。
**「我们造的概念」不进 `.osa`** —— 写它就得改上游那 13 个文件的序列化。

### 六、可观测性的可行性（用户追问）

**结论：可行性高，不矛盾。**

- **5 个信号 `System` 上现成**：`GetTrackingState` / `isLost` / `isFinished` /
  `GetTimeFromIMUInit` / `MapChanged`
- 缺的只有「整张地图」（`KeyFramesInMap` / `IniertialBA` 在 `Map` 上，`System` 没转发）
- **最贵的覆盖率的成本被分摊掉了**：它要的「地图访问接口」，
  **切图和 2D 地图本来就要**

**一条要守的约束**：**覆盖率跟着「当前图」走**——重定位切图之后覆盖率换了对象，
必须跟着重算，不能攒成全局的。

## 被推翻的判断（7 条，都是查源码后改的）

**这一节是本次最有价值的部分 —— 七次里有六次是"读了几行代码就推断全局"。**

| # | 我原来说的 | 实际 |
|---|---|---|
| 1 | 定位模式在 Mono-Inertial 上生效 | `Tracking.cc` 里作者自己标了 **`TODO Not available in inertial mode`**，那条分支走纯视觉 VO、绕过 IMU |
| 2 | 加载旧图后，跟丢的搜寻范围变成历史所有图 | `DetectRelocalizationCandidates` 有 `pMap` 过滤（`KeyFrameDatabase.cc:102`），**只在当前图里选候选**；而加载后当前图是空的新图 → **一个候选都找不到** |
| 3 | 每次启动都要重做惯性初始化 | `mbImuInitialized` **被 `Map::serialize` 保存**；但加载后 `CreateNewMap()` 建了新图 → 实际行为**待实测** |
| 4 | 主教程 §10 有 `System::GetAllKeyFrames()` | `System` 上**没有**这个方法。**是我 09-25 重写时从旧教程照抄的** |
| 5 | `GetAllKeyFrames()` 不是线程安全的 | `Map::GetAllKeyFrames()` **是有锁的**；真危险是**返回的裸指针出了函数就没保护** |
| 6 | 参照物校正「不是可选项」 | 用户澄清价值是「粗糙好用的随身尺子」→ **降级为可选** |
| 7 | （价值论证里用了）「家具能不能进门」 | **别的会话幻想出来的，不是用户意图** |

**这七条本身是个信号**：ORB-SLAM3 的**多地图 / Atlas 那块是它最不成熟的部分**
（作者留 TODO、`KeyFrameDatabase::serialize()` 是死代码、`SetCurrentMap` 干脆不存在）。
**靠阅读推断风险很高，那条路必须小步实测。**

## 验证

**以下都是实际跑过的命令和输出。**

**1. Boost 是硬依赖**

```
$ grep -n -i boost CMakeLists.txt
124:-lboost_serialization

$ ldd lib/libORB_SLAM3.so | grep boost
libboost_serialization.so.1.83.0 => /lib/x86_64-linux-gnu/libboost_serialization.so.1.83.0

$ grep -rln "boost" src/ include/        → 13 个文件
```

`Atlas.h:31-32` 与 `:51` 的 boost 引用**都没有 `#ifdef` 保护** —— 所以**不存地图也要链接它**。

**2. 存/读路径写死**

```
$ sed -n '1411,1413p' src/System.cc
        string pathSaveFileName = "./";
```

`LoadAtlas` 侧同样写法在 `:1450`。Android 上 CWD 是 `/`，**不可写**。

**3. 加载时 KeyFrameDatabase 会重建**

```
System::LoadAtlas()                        System.cc:1445
  ├─ :1499  mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase)
  └─ :1501  mpAtlas->PostLoad()
              └─ Atlas::PostLoad()                          Atlas.cc:335
                  └─ :348  pMi->PostLoad(mpKeyFrameDB, ...)
                            └─ Map::PostLoad()              Map.cc:427
                                └─ :469  pKFDB->add(pKFi)
```

**4. 候选被当前图过滤**（推翻第 2 条）

```
$ awk '/DetectRelocalizationCandidates/,/^}/' src/KeyFrameDatabase.cc | sed -n '96,105p'
            if (pKFi->GetMap() != pMap)
                continue;
```

**5. `System` 没有地图访问接口**（推翻第 4 条）

```
$ grep -nE "Atlas|GetMap|Map\*" include/System.h
213:    Atlas* mpAtlas;        ← private
```

公有方法只有 `GetTrackedMapPoints` / `GetTrackedKeyPointsUn` / `GetTrackingState` /
`isLost` / `isFinished` / `GetTimeFromIMUInit` / `MapChanged` / `GetImageScale` 等。

**6. `Map::GetAllKeyFrames()` 有锁**（推翻第 5 条）

```
$ grep -n "vector<KeyFrame\*> Map::GetAllKeyFrames" -A 4 src/Map.cc
147:vector<KeyFrame*> Map::GetAllKeyFrames()
149-    unique_lock<mutex> lock(mMutexMap);
```

**7. 定位模式对惯性模式是 TODO**（推翻第 1 条）

```
// Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
```

**8. `Atlas` 没有 `SetCurrentMap()`**

```
$ grep -nE "SetCurrentMap|GetCurrentMap|CreateNewMap|GetAllMaps" include/Atlas.h
79:    void CreateNewMap();
108:   vector<Map*> GetAllMaps();
116:   Map* GetCurrentMap();
```

**9. 重力对齐**（2D 地图的前提）

`LocalMapping.cc:1282` 的 `ApplyScaledRotation(Twg, mScale, true)` —— **行号核对过是对的**。

**10. 未验证**

- **`LoadAtlas` 这条路一次都没跑过。** 已在 `MAP_FEATURE_IDEAS.md`「待实测」写明
  三步实验（①②零风险，先做）。
- **Boost 交叉编译到 Android 没试过。**
- 本次**无代码改动**，没有可执行的东西被验证。

## 遗留

- **文档大量重写与新建，用户尚未逐份审核。**
- **`SaveAtlas` 写死 `./`** 那处必改还没改（要动 `ORB_SLAM3/src/System.cc`）。
- **`MAP_FEATURE_IDEAS.md` 的存废**：一度打算删（设想使命已完成），最终**留着** ——
  它还有「地图持久化」等别处没有承接的框架事实。**但里面确实有冗余**
  （三个设想的描述、重力对齐等已进设计文档）。**将来若删除，先搬这 6 块**：
  共同前提 / 稀疏地图 / **持久化四个坑** / **人工指示 vs 语义分割的论证** /
  **形态分时刻 + 未扫描的定义** / 多传感器。
- **STAGE6 有一处标了未定**：「添加标记点」是一个按钮还是两个。
- **锚点存哪**已定成 sidecar，但**锚点怎么删、怎么改名**没定。
- **多张旧图怎么选**：目前只说"挑关键帧最多的那张"，多张同场景的情况没处理。
- **`CLAUDE.md` 已按新方案改**（原来写的是拍照流程）。它**不走版本控制**，
  所以这次改动没有 git 可退 —— 只能靠本文件记着原样。

## 回滚

纯文档。涉及：

- **删除**：`tutorial/STAGE6_PHOTO_MEASUREMENT.md`（**有 git 历史**，可 `git checkout` 找回）
- **新建**：`tutorial/APP_DESIGN.md`、`tutorial/STAGE6_MEASUREMENT.md`
- **重写**：`tutorial/MAP_FEATURE_IDEAS.md`、本文件（**均已改名/重写多次**）
- **更新**：`tutorial/STAGE5_JNI_BRIDGE.md`、`tutorial/STAGE7_UI_STATE_MACHINE.md`、
  `tutorial/ANDROID_SLAM_TUTORIAL.md`、`docs/README.md`、`CLAUDE.md`

**⚠️ 注意**：`MAP_FEATURE_IDEAS.md` 和本文件（含改名前的 `-map-feature-ideas-assessment`）
在重写前**从未提交过**（`git status` 里是 `??`），所以**没有 git 历史可退**。
用户知情并选择直接覆盖。

**`android/SlamCapture/` 和 `ORB_SLAM3/` 一个字都没动。**
