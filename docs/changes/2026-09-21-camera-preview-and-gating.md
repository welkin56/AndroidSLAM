# 2026-09-21 实时预览 + 「准备/录制」两段式流程

## 动机

用户提的需求，原话：

> 希望在加一个功能，有一个准备录制的按钮，一开始没有画面，点击准备录制后预览输出画面，
> 开始对焦采集IMU，两个就绪前"开始录制"显示"强制开始"代表 imugaveup 或者 camergiveup，
> 由用户决定要不要强行开始录制，**不要超时自动开始**。

三件事捆在一起：

1. **拍标定板时看不见画面** —— 板子进没进画面、糊没糊、曝光够不够，全靠录完拉到 PC 上
   才知道，一轮白拍就是几分钟。之后 SLAM 测距那一版也要「把相机帧显示到屏幕上 + 在它
   上面叠东西」，现在顺手搭出来。
2. **超时放行 = 用户不知情的时候数据质量被降级了。** 老流程是「按一个按钮 → 等两条线
   就绪 → 超时自动开录」，而录制时看不见画面。用户要的是：先看画面、先等就绪，
   **要不要带着没就绪的状态硬上，由他按那个按钮决定**。
3. **2026-09-18 那次「相机不该拥有 IMU 的状态」只改了一半** —— 字段、`capturing` 闩锁、
   `maybeBegin()`、`markImuReady()`、`imuTimeout` 还全在 `CameraCapture` 里，要读四个地方
   才能回答「什么时候开始落盘」。

## 改动

### 新增 `PreviewView.kt`

从**已经在跑的那路 ImageReader 的 Y 平面**渲染，也就是交给落盘的那个 `ByteArray`，
逐字节相同。**刻意不额外挂一路 camera surface**：多一路输出会让相机重新协商 stream
配置，可能换 sensor mode 或裁切，`fx` 就变了，整个标定作废。展开（灰度 → ARGB）留在
主线程 `onDraw`，相机线程只做「换个引用 + 请求重绘」，负载和没有预览时一样。

方向：缓冲区永远是传感器原生方向（640×480 横着的），顺时针转 90° 交给 `Canvas` 画出去
（像素循环保持线性写入，否则每帧 30 万次跨 2.5 KB 步长的写全是 cache miss）。

### `CameraCapture.kt` —— `started` 布尔量换成 epoch 计数

`started` 挡不住这个序列：`开始准备 →(5ms) 取消准备 →(3ms) 开始准备`。第一次
`openCamera` 的 `onOpened` 迟到，读到的 `started` 是**第二次刚置的 true**，于是把过期设备
写进 `device`；紧接着第二次 open 拿到 `ERROR_CAMERA_IN_USE`，`onError` 里那行无条件的
`device = null` 把唯一活着的设备抹掉。结果：**预览照常出图，但对焦永远收敛不了，相机被
永久占死，只能重启进程。**

改成每个回调捕获自己出生时的 `epoch`，跑起来先比一次，不等就只 `close()` 自己那份资源
然后退出，**绝不写字段**。用奇偶编码运行状态（奇数 = 正在跑）而不是另开一个布尔：
「这一代还算不算数」和「相机在不在跑」本来就是同一个问题，分两个字段迟早互相矛盾。

同一次还修了四处：

- `acquireLatestImage()` 原来在 `try` 外面，reader 关掉后在途回调会让它抛
  `IllegalStateException` 逃出 Handler → **进程直接死**。包进 try，并且 `reader.close()`
  之前先 `setOnImageAvailableListener(null, null)`。
- `beginSettle()` 里 `ready = !supportsAf`（原来是写死 `false`）。定焦摄像头上
  `CONTROL_AF_STATE` 恒为 `INACTIVE`，`focusCallback` 永远不置位 →「开始录制」**永远
  不会变绿**，`forced` 也恒为 true。
- `openCamera` 包 try/catch：没权限时抛的是**运行期** `SecurityException`，而且在相机
  线程的 runnable 里抛，不接就是进程死。
- 新增 `lastError`。**没有它，出错的表现就是一块黑屏加一个永远红的按钮，没有任何解释。**

**删掉 `endSettle()`**：相机每次都是从开的，不存在「自由预览」态，没有调用者。这等于把
「`ready`/`focusLockArmed` 只在 `start()`/`stop()` 里复位」变成一条显式不变式，写进了类注释。

> `TEMPLATE_RECORD`、`pickFpsRange()`、关 EIS/OIS 一个都没动 —— 这是 `fx` 不变的前提。

### `ImuCollector.kt` —— 拆出 `attach`/`detach`

传感器整段时间开着，而只有真正在录的那一段才往文件里写。所以「传感器在不在跑」和
「在不在记录」必须分开：`start`/`stop` 管前者，`attach(w)`/`detach` 管后者。

`ready` 的**形状也改了**：从「曾经收到过样本」的闩锁，改成「**最近 500 ms 内收到过样本**」
（`lastSampleNs` + `STALE_NS`）。理由见「决策」一节。

### `SessionWriter.kt`

- `offerFrame()` / `offerImu()` 先看 `running`，关了就计 `dropped` 再返回。关掉了一个
  TOCTOU：`MainActivity` 读序是「先 writer 后 phase」，拆除写序是「先 phase 后 writer」，
  **读写序相反**。窗口只有毫秒级，但老代码在这条路上是**静默丢弃且不计数**，于是
  `frames_written + frames_dropped` 不等于相机实际出的帧数。
- `start()` 的 IO 异常收掉（新增 `ioError`），不抛给主线程调用方。
- 新增 `threadAlive`。`stop()` 的 `join(5000)` 超时之后**不能再去删目录** —— writer 线程
  还活着会和删除并发，`writeFrame` 一路 `FileNotFoundException`。
- 新增 `forced` / `firstFrameTsNs` / `firstImuTsNs`，写进 `meta.json`。后两个是为事后审查：
  以前想知道某个 session 的 IMU 比相机早多少，只能自己去解两个 CSV。

### `MainActivity.kt` —— 状态机重写

```
IDLE ──[开始准备]──► ARMED ⇄ RECORDING
  ▲                   │
  └───[取消准备]───────┘
```

| 状态 | A 按钮（左） | B 按钮（右） |
|---|---|---|
| IDLE | `开始准备` | `开始录制` 灰、不可点 |
| ARMED（未就绪） | `取消准备` | **`强制开始` 红** |
| ARMED（就绪） | `取消准备` | **`开始录制` 绿** |
| RECORDING | `取消录制` | `停止录制` |

红/绿用 `backgroundTintList`，**不用 `setBackgroundColor`** —— 后者会丢掉 AppCompat 按钮的
圆角和按下态。

### `activity_main.xml` / `AndroidManifest.xml`

布局换成 预览(weight=1, 黑底) + 状态栏 + 一行两个按钮。manifest 的
`screenOrientation` 从 `locked` 改成 `portrait`（**对录到的像素零影响**，缓冲区永远是
传感器原生方向）。

## 决策

### 落盘的 PNG 转不转正

**不转，只转预览。** 90° 是相机坐标系绕光轴的刚性旋转，被 `K` 和 `T_b_c1` 完全吸收，
**不改变任何三维距离**。显示层本来就躲不掉一次屏幕↔图像的映射，加个 90° 只是把「缩放」
变成「缩放+旋转」。反过来，转正会把已有 4 个 session、439.23 那套标定劈成两套方向约定，
收益为零。

**用户纠正了我一次判断：** 我说「一直竖着手机录制的」，用户说「我一直是竖着手机录制的」时
我此前看错了。此前用 ArUco 解板子位姿验证过：不转 −1.4°，`ROTATE_90_CW` +88.6°
（正立应为 +90°）。`phone.yaml:71-74` 早就写明了这件事，`T_b_c1` 的 `[0,-1,0] / [-1,0,0]`
就是这个 90°。**整套约定自洽，什么都不用修，只是渲染出来的画面是躺着的。**

### **否决：把 IMU 的寿命拉到 App 前台**（被用户推翻）

我提议让 IMU 在 App 前台期间一直登记着，这样 `取消准备 → 开始准备` 只要约 1 秒，
`取消准备` 自己就变成了「重新校准相机」，不需要第二个按钮。

**用户否决：**「还是把 imu 采集与否归到是否准备吧，我能接受冷启动，接受不了无用资源一直耗电」

**采用：两条线都由「准备」这个动作开关。** 理由：两条线的开销都由那个按钮正当化 ——
IMU 的 3.3 s 冷启动和相机的独占占用，都是按下它时明确同意付的。代价是
`取消准备 → 开始准备` 重付约 3.3 s，**接受**，因为那是偶发路径，而常见路径上的
每一次「停止录制」不额外付任何东西。

### **否决：加一个「校准相机」按钮**

镜组锁在 ARMED 进入那一刻的距离上。如果**改变了拍摄距离**，预览会糊而 `cam.ready`
还是 true（它是个只增不减的闩锁）——绿按钮在说假话。

否决理由：它 = 引入**第二个「让相机就绪」的操作**。它和 `开始准备` 做的是同一件事
（`beginSettle()` vs `start()+beginSettle()`），只是起点不同；判据和操作都不再唯一，
下一个读代码的人（就是几个月后的我）必须同时理解两个按钮才能回答「现在到底准备好没有」。
**那正是这次要拆掉的那种债。**

而且这个场景比看上去窄得多：**糊的条件是拍摄距离变了，不是「挪了手机」** —— 重新瞄准、
距离不变的话画面照旧清晰。真嫌烦了，补丁是一行：`stopRecording` 里加一句
`cam.beginSettle()`。现在不加。

### ARMED 期间要不要连续对焦

**不要。** camera app 的直觉是「预览时连续对焦、按下快门才锁」，很诱人（预览永远清晰、
`cam.ready` 变成实时指示灯），但它破坏了这套流程存在的理由：切 `AF_MODE_OFF` 是本项目在
这台机器上踩过**最贵**的坑（前 7 帧清晰、第 8 帧起整段糊）。把它挪到「按下录制之后」，
那道切换就发生在**用户没法先验证**的时刻，失败代价从「重来一次准备」变成「废掉一整段
录制」。ARMED 期间就锁死，代价是改了拍摄距离会糊 —— 而**那个糊是看得见的**。

### 预热用时间戳而不是定时器

按下 `开始录制` 的那一刻 `T`：`framesFromTsNs = T + 300ms`，落盘判据是三合一
`phase == RECORDING && w != null && tsNs >= framesFromTsNs`。

**不 `postDelayed(300)` 再切状态**：那要么多一个布尔、要么多一个状态，于是「在录」这一个
事实有了两个真值来源 —— 正是这个项目反复踩的坑。而 `Image.getTimestamp()` 和
`SensorEvent.timestamp` 本来就是同一个 `CLOCK_BOOTTIME` 基准，所以可以直接写成一个字段、
一次比较。附带好处：`image.timestamp` 是**曝光起始时刻**，比的是曝光时刻而不是投递时刻，
09-18 那条「两个流的时间关系」的隐患在这一侧被结构性地关掉了。

### `ready` 为什么改掉「闩锁」这个形状（用户提问逼出来的）

用户问：**「取消录制或者结束录制后仍是准备状态，随时可以开始下一次录制，那你是怎么保证
每次 imu 都在照片前记录的？」**

先把两件事分开：

- **保证不来自 `ready`，来自 `framesFromTsNs`。** 相机帧在 `T+300ms` 之前一律丢弃，
  无论 IMU 什么状态；而 IMU 在 `attach` 之后的下一个陀螺仪事件（≤5 ms）就开始写。
  所以 IMU 领先约 295 ms，**跟第几段无关**。第二段连冷启动都没有，比第一段更稳。
  `ready` 只是让「强制开始」那条路能被**识别**出来。
- **但 `ready` 原来的形状确实是错的。** 做成「曾经出过数」的闩锁时，停止录制后它仍为
  true，**结果对、理由是错的**：它靠的是历史，不是现状。传感器中途哑掉的话它照样 true，
  而编排层据此判断「这一段 IMU 会领先相机」就变成了假话，且没有任何迹象。

**采用：改成「最近 500 ms 内收到过样本」的活判据。** 于是停止录制后仍为 true 是因为
**事实仍然成立**，不是因为谁忘了复位 —— **它不再是闩锁，所以压根不存在「复位」这回事**。
传感器哑了半秒后自动变 false。另外在 `start()` 里加了一道「只能 start 一次」的守卫，
让「复用一个已停掉的对象」在运行时直接响，而不是只靠文档劝。

**顺手改了两处：**

- `sampleSeen` 的置位点从 `writer.offerImu()` **之后**挪到加速度计回调里。老位置的意思
  是「已经写进 CSV 了」，新位置的意思是「传感器在出数」—— 编排层要在**还没 attach** 的
  时刻做判断，那一刻必然一行都没写，只能问后者。
- **强制开始时不再白等那 300 ms**：预热存在的唯一理由就是让 IMU 领先，IMU 都没出数就
  没东西可领先，那 300 ms 是纯浪费，还会白丢这一段开头能用的画面。

录制中「这一段到底写进去没有」用 `SessionWriter.firstImuTsNs` 报 —— 那是**权威事实**，
事前判断永远可能过时，所以两个都留。

## 验证

**编译：未验证。** 按 CLAUDE.md 第三节的约定，编译交给用户执行，命令见
[tutorial/STAGE2_ANDROID_CAPTURE.md](../tutorial/STAGE2_ANDROID_CAPTURE.md)（`./gradlew assembleDebug` +
`adb install -r app/build/outputs/apk/debug/app-debug.apk`）。**注意改之前工作区是编译不过的**
（`MainActivity` 引用了布局里还不存在的 `R.id.preview`），所以这次六个文件必须一起改完。

**设备端：未验证。** 要按顺序走这 12 条，其中 4 条是专门盯老 bug 的：

1. 冷启动：预览全黑；`开始录制` 是灰的、点不动。
2. 点 `开始准备` → 变 `取消准备` / 红色 `强制开始`；约 3.3 s 后变绿色；画面**竖着、正的**。
3. 点 `开始录制` → 状态栏先「IMU 预热」约 0.3 s，然后帧数从 0 开始涨。
4. **停止录制后立刻再录一段**，第二段 `cam0/data.csv` 必须有数据（专验「二次录制永远不落帧」）。
   且 logcat 里**不应**再出现冷启动日志。
5. 点 `取消录制` → 确认弹窗 → 目录消失、**仍在 ARMED**（按钮还是绿的）。
6. 点 `取消准备` → 预览黑掉、`files/sessions/` 无残留（`*.deleting` 也清掉）。
   再点 `开始准备` → **会重新等约 3.3 s**（预期行为）。
7. **狂点**：`开始准备`↔`取消准备` 间隔几十毫秒连点十几轮，结束时预览必须黑、
   logcat **没有** `ERROR_CAMERA_IN_USE`，随后仍能正常出画面（验 epoch 改造）。
8. 切后台：ARMED 中按 Home → 回来是 IDLE；录制中按 Home → 文件**保住**。无 `WindowLeaked`。
9. **关键回归 1**：`head -1 imu0/data.csv` 必须 **小于** `head -1 cam0/data.csv` 且差约 0.3 秒；
   再核对 `meta.json` 的 `first_imu_ts_ns` / `first_frame_ts_ns`。
10. **关键回归 2**：对着标定板录一段跑 `tools/calibration/calibrate_camera.py`，
    `fx` 应仍在 **439** 附近、RMS 0.3 px 量级。**fx 动了就说明采集路径被碰到了，立刻回滚。**
11. 强制开始：`开始准备` 后趁 IMU 没出数立刻点红按钮 → logcat 有 ERROR、帧照落、
    `meta.json` 里 `"forced": true`、`first_imu_ts_ns` 为 0。
12. 帧率 ≈ 30 fps、掉帧 ≈ 0。

**ArUco 那条 90° 的结论**不是本次跑的，是此前验证的结果，写在这里只是备查。

## 遗留

- **PNG 不转正。** 将来若要转：`extractGray` 里顺带转置（零额外分配）、yaml 改四处
  （`width/height`、`fx↔fy`、`cx/cy → (H-1-cy, cx)`、**`T_b_c1` 左乘绕相机 Z 的 90°**），
  外加**已有 4 个 session 怎么办**。
- **`cam.ready` 仍是个只增不减的闩锁**（`ImuCollector.ready` 这次改活了，它不是）。
  在 ARMED 里改了拍摄距离，预览会是糊的而按钮仍然是绿的。预览里能直接看出来，
  出路是 `取消准备`。
- **ARMED 是「占着相机和传感器」的状态**，别的 App 用不了这颗相机。切后台会拆掉。
- `停止录制` 之后不重新对焦（见「决策」）。
- `SessionWriter.stop()` 与在途帧之间仍有一帧的竞态（现在会计入 dropped，不再静默）。
- 切后台只挡 `onStop`；分屏/多窗口下失活只走 `onPause`。
- **`phone.yaml` 仍持有旧内参**（fx=422.7），标定值备份在
  `tools/calibration/phone_calibrated_2026-09-20.yaml`。与本次无关。

## 回滚

`android/SlamCapture/` 这个仓库里 `git diff` 可退；`PreviewView.kt` 是新增文件，用
`git clean -f` 或 `git checkout` 后手动删。
涉及的七个文件：`CameraCapture.kt` / `PreviewView.kt` / `ImuCollector.kt` /
`SessionWriter.kt` / `MainActivity.kt` / `activity_main.xml` / `AndroidManifest.xml`。

**`ORB_SLAM3/phone.yaml` 和 `tools/` 一个字都没动。**
