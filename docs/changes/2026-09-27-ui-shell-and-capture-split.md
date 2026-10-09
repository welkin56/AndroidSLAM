# 2026-09-27 UI 外壳：三个页面，录制独立成模式

## 动机

M4 上设备验过之后（[见 2026-09-27-m4](2026-09-27-m4-construct-system.md)），
按约定的次序做「UI 外壳」。**做之前用户把形态定成了三个页面**：

```
EntryActivity（冷启动落这儿）
├─ 「＋ 新建地图」            → MainActivity
├─ 列表：已保存的图           → MainActivity
└─ 「录制模式」               → CaptureActivity

MainActivity（SLAM 主界面）  状态 → 画面 → [启动 SLAM][重定位] ＋ 右上角 ☰ 抽屉
CaptureActivity（录制页）    画面 → 状态 → [开始准备][开始录制]
```

这条路径上，原来的方案（入口页二选一 + 主界面骨架）被改成了「**选一个已保存的图**」，
**「步 B：『或』语义的硬件类」整个取消**。

## 改动

### Kotlin

| 文件 | 改了什么 |
|---|---|
| `EntryActivity.kt` + `activity_entry.xml` | **新增**。「＋ 新建地图」+ 已保存图的列表（空时显示提示）+ 「录制模式」 |
| `CaptureActivity.kt` + `activity_capture.xml` | **新增**。就是原来 `MainActivity` 那一屏整体搬过来，**逻辑一行没改** |
| `MainActivity.kt` + `activity_main.xml` | **重写**成 SLAM 页：状态 → 画面 → 两按钮，顶层 `DrawerLayout` |
| `AndroidManifest.xml` | `EntryActivity` 成 launcher；`MainActivity` 改 `exported=false`；加 `CaptureActivity` |

**`CaptureActivity` 是从 git 里取的**（`9e53ff2` 那一版 `MainActivity.kt`，
即 M4 之前），只改了类名和 `TAG`。理由：手剥 M4 加的那些东西容易漏，而从 git 取
**逐字节就是那个跑通过的版本**，「采集流程没被弄坏」这条回归最好验。

### C++

`slam_engine.{h,cpp}` + `jni_bridge.cpp` 加 `elapsedMs()` / `nativeElapsedMs()`。

**这是修 M4 的一个真 bug**：`MainActivity` 里那个 `slamStartedAtNs` 是 **Activity 的字段**，
而 `System` 的构造是**进程级的事实**。从入口页再进来一次，计时归零而 `System` 还在造 ——
界面就卡在「加载中」转圈，按「启动」也没用（C++ 那边被 CAS 挡掉）。
**谁干的活谁报时间**：改成 C++ 报「已经加载了多久」。

### 文档

- `APP_DESIGN` §2.1（入口页三块）/ §6（**四个开关 → 三个开关 + 录制是独立模式**）/
  §7.1（**撤回**「或」语义的硬件类）/ §9（保存时机是个洞）/ §10（删掉「自动重定位」那条）
- `STAGE7` 那张「相机 + IMU 绑在 SLAM 运行中」的表 —— 整张作废
- 本记录 + `README.md` 索引

## 决策

### 一、⭐ 录制和 SLAM **互斥**，各自开相机（用户定的）→ **步 B 取消**

原计划里 M4 之后还有一步：「一个『或』语义的硬件类，SLAM / 录制两个申请者」。
**它取消了** —— 申请者只剩一个，「或」没有意义。

用户给了两条理由，都成立，第二条比表面更硬：

| 理由 | 具体 |
|---|---|
| **性能** | ORB-SLAM3 的特征提取是耗时的大头（STAGE5 §8），录制是 30 fps PNG 落盘。叠一起帧率会抖 —— 而**帧率稳不稳正是录制那套的判据之一**（重录做 A/B 对比要看它） |
| **对焦所有权** | `beginSettle()` 是「先收敛、再把 AF 锁死」。而**锁焦的另一个后果是：fx 只在那个对焦距离上等于 435.2**。两个模式的**对焦策略直接冲突** —— 录制每次「开始准备」都要重新收敛，而 SLAM 一旦跑起来**绝不能被重新对焦打断**（位姿会跳）。**这不是"多改几行"，是两个所有者要管同一颗镜组。** |

**代价说清楚：「建图且录制」这个组合不存在了。** §6 原先举的三个组合
（建图不录 / 录不建图 / 不记路径但录）**全都还在** —— 只有这一个没了，
而它原本也没被举过。

### 二、两个页面各是各的界面，不共用骨架

`CaptureActivity` 的布局**就是现在这一屏**，它**根本没有 ☰** —— 不是"藏起来"，是不存在。
同理没有「启动 SLAM」「重定位」。

**否决「共用骨架 + 按模式显示/隐藏」**：那样布局里会留着看不见的控件，以后谁改布局
都得先想清楚「这个在哪个模式下才露」。这和 §6 那条已有的教训同源
（「不做成一个 enum，因为它们两两之间没有必然联系」）。

### 三、两页的排布**故意不同**

| | 排布 | 为什么 |
|---|---|---|
| 录制页 | 画面 → 状态 → 按钮 | 盯着的是**画面**（构图、对上焦没有），状态是补充 |
| SLAM 页 | 状态 → 画面 → 按钮 | 盯着的是**"现在能不能用"**（加载中/就绪/丢没丢），画面是辅助 |

### 四、选择就是 `String?`，不用 enum

「新建」= `loadFromFile` 传 `null`，「选一张旧图」= 传路径。**它们是同一个参数的两个取值，
不是两种模式** —— 这正是 §2.4「一个动作、三种来源」的直接后果。加一层 enum 是自找的。

### 五、「启动 SLAM」保持**单态**，不做成启动/停止

`nativeStop` 是 M5 的，现在点「停止」没有任何东西可停 —— 那是个死按钮。
而「重定位」灰着**性质不同**：那是**有理由的灰**（还没有旧图，§2.2），不是"没实现"。

### 六、抽屉用 `DrawerLayout`，**不加依赖**

`androidx.drawerlayout` 已经是 `appcompat` 的传递依赖，本地 gradle 缓存里也有。
主题是 `NoActionBar`，所以没有系统 ActionBar 的汉堡键，自己放了一个 ☰。

抽屉里先放**一条占位**（灰着的「保存轨迹（M8）」）—— 空抽屉比一个灰按钮更让人困惑。

## 验证

### 1. C++ 交叉编译通过，6 个 JNI 符号逐字对上

```
$ ninja -j4        → 退出码 0

C++ 导出：nativeElapsedMs nativeHello nativeInit nativeIsReady nativeLogSelfTest nativeState
Kotlin：  nativeElapsedMs nativeHello nativeInit nativeIsReady nativeLogSelfTest nativeState
```

### 2. 拆页时抓到的一个错

`CaptureActivity` 是从 git 取出来的，**它的 `setContentView(R.layout.activity_main)`
还指着 SLAM 页那个布局** —— 那样 `findViewById(R.id.btnA)` 会返回 null，
一进来就崩。已改成 `R.layout.activity_capture`。

**这个是靠「逐个核对每个 Activity 指向哪个布局 + 布局里有哪些 id」抓出来的**，
不是靠读代码看出来的。

### 3. ⭐ 用户抓到的一个错：替换代码块时把收尾的 `}` 一起吃掉了

入口页的列表本来扫在 `onCreate` 里。而它**只在 `onCreate` 扫一次**是个真问题：
启主界面时**不 `finish()`**，所以这个 Activity 只会被 stop、不会被销毁 ——
从主界面退回来时**列表是陈旧的**，将来存了一张图再退回来，新图不会出现。**已挪到 `onStart`。**

**但第一版改法写错了**：我把「新加 `onStart` + `refreshMapList`」和「删掉 `onCreate`
里那段」揉成了一个替换，`old_string` 里带了 `onCreate` 的收尾 `}`，而 `new_string`
末尾给的是 `refreshMapList` 的收尾 `}` —— **两个函数于是嵌套进了 `onCreate`**。
Kotlin 允许函数里声明函数，但 `override` 不行，**编译不过**。

**教训**：替换「一个函数的尾部 + 它后面新增的东西」时，**收尾的那个 `}` 到底属于谁
必须数清楚** —— 光数新加那段是不够的。事后用「数括号 + 核对每个顶层声明缩进是否为 4」
扫了一遍三个文件，全平。

### 4. ✅ 已上设备验证通过（2026-09-28 补记）

**用户跑过了**：Gradle 构建通过，下面那 6 条走通了 —— 冷启动落入口页、
「＋ 新建地图」进 SLAM 页、Back 能回入口页、「启动 SLAM」能加载到「就绪」、
☰ 抽屉能开、「录制模式」进去采集流程照旧。

**用户反馈的一条**：「启动 SLAM」之后一直显示「SLAM 已启动」，**没有停止**。
那是**有意留的** —— `nativeStop` 是下一步的活，当时点「停止」没有任何东西可停，
那是死按钮。**已在 M5 补上**（[见 2026-09-28-m5](2026-09-28-m5-slam-on-phone.md)）。

下面那 6 条留作记录。

```bash
cd android/SlamCapture && ./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

1. 冷启动落在入口页：有「＋ 新建地图」、列表显示「还没有保存的图」、「录制模式」
2. 「＋ 新建地图」→ SLAM 页，状态栏「未启动（新建地图）」；**Back 回入口页**
3. 「启动 SLAM」→「词袋加载中…（已 Xs）」→ 约 38 秒后「就绪」，**App 不退出**
4. **点右上角 ☰** → 抽屉从右边出来，里面有灰着的「保存轨迹（M8）」
5. 「重定位」全程灰着
6. **关键回归**：「录制模式」→ 录制页，采集流程和以前一模一样 ——
   开始准备 / 相机起 / 预览出图 / 能录一段 / 停止录制 / 取消准备

## 遗留

- **两个页面的相机装配代码会各有一份**（`camThread` + `cam` + `imu` + `preview`
  的接线约 50 行）。M5 两个页面都要真开相机时再看要不要抽出来共用 ——
  那时候才知道两边的装配到底像不像
- **SLAM 页中间是个黑框** —— 相机接线是 M5
- **「停止 SLAM」还没有** —— 要等 M5 的 `nativeStop`
- **保存什么时候触发** —— 见 `APP_DESIGN` §10，M8 之前必须补
- **`<名字>` 从哪来** / **「一个会话一组文件」vs `SaveAtlas` 写出整个 `Atlas`** /
  **换一张旧图要不要重启进程** / **`loadFromFile` 和 `System` 的结构对不上** —— 都是 M8 的
- **内存**：跑 20 分钟，记 `Atlas::GetAllMaps().size()`、当前图 KF 数、进程 native 内存

## 回滚

**整个这一步都不要**：把 `EntryActivity` / `CaptureActivity` 和它们两个布局删掉，
`MainActivity` + `activity_main.xml` 用 `git checkout 4aaf86c -- <那两个文件>` 取回
M4 那一版，Manifest 把 launcher 改回 `MainActivity`。

**只回滚「录制独立」这一条**：把 `CaptureActivity` 改名回 `MainActivity`，
删掉新的 SLAM 页，`AndroidManifest` 改回去 —— 但那样 `APP_DESIGN` §6.2/§7.1
和 `STAGE7` 那几处文档也要一起回滚，因为它们是同一个决定的。

C++ 那边只有 `elapsedMs` 一个增量，`git diff` 可退。
