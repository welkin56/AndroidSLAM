# 2026-10-24 界面重做：Material 3、状态芯片与诊断面板

## 动机

用户要求重新设计并美化界面，并明确了几项偏好：

- 整体方向选 **Material 3 动态色**；
- 强调色「怎么好看怎么来」；
- 状态信息采用 **状态芯片 + 诊断面板**；
- **跟随系统深浅色**。

改造前是 `Theme.AppCompat.NoActionBar` 的默认浅色界面：按钮靠运行时 `backgroundTintList`
临时染色，状态区把位置、轨迹点数、纹理数、地图统计、保存结果和文件路径全塞进一段等宽文本，
入口页是原生 ListView。功能没问题，但视觉上没有体系，信息也没有层级。

## 改动

### 主题与基础资源

- `app/build.gradle.kts` — 引入 `com.google.android.material:material:1.12.0`。
- `app/src/main/AndroidManifest.xml` — 应用主题改为 `Theme.SlamCapture`，注册 `SlamApp`。
- `app/src/main/java/com/example/slamcapture/SlamApp.kt`（新） — `DynamicColors.applyToActivitiesIfAvailable()`，
  Android 12+ 从壁纸取色。
- `res/values/colors.xml` / `res/values-night/colors.xml`（新） — 以青色 `#00696C` 为种子的完整 M3 色板，
  深浅各一套。这是低版本和动态色不可用时的兜底。
- `res/values/themes.xml` / `res/values-night/themes.xml`（新） — `Theme.SlamCapture.Base` 承载全部颜色角色，
  明暗只覆盖 `windowLightStatusBar`；状态栏/导航栏颜色引用 `?attr/colorSurface`，跟着动态色走。
- `SlamApp.kt` 里的 `Activity.applySystemBarAppearance()` — API 26 的导航栏图标没有主题属性，
  在三个 Activity 里按 `uiMode` 运行时设置明暗。
- `res/values/styles.xml`（新） — 芯片文字、地图缩放键、测距按钮的尺寸与形状。
- `res/drawable/`（新） — 芯片/横幅圆角底、查看器顶栏底，以及 menu / add / remove / chevron / close / map / record 矢量图标。

### 入口页

- `res/layout/activity_entry.xml` — 标题 + 新建按钮 + 「已保存的地图」分区 + 列表 + 底部录制模式。
- `res/layout/item_map.xml`（新） — 卡片式列表项：名字、修改时间、文件大小、右侧箭头。
- `EntryActivity.kt` — `ArrayAdapter` 换成 `BaseAdapter`，显示时间与文件大小；不再只给一行字符串。

### SLAM 主界面

- `res/layout/activity_main.xml` —
  - 顶部改为可横向滚动的四颗状态芯片（跟踪 / 轨迹 / 录制 / 地图）+ 菜单键；
  - 芯片下方是默认隐藏的横幅，只在加载、失败、恢复、丢跟踪、错误时出现；
  - 横幅内加 `LinearProgressIndicator`；
  - 小地图与查看器的缩放键改为图标按钮；
  - 抽屉重排为「显示 / 标记 / 地图」三个分区，底部是可展开的诊断面板；
  - 切片厚度 `SeekBar` 换成 M3 `Slider`。
- `MainActivity.kt` —
  - `render()` 拆成三块：芯片、横幅、诊断文本；
  - 新增 `uiMessage`（一次性错误）和 `transientMsg`（保存成功后停留 5 秒）；
  - 运行中主按钮用 `colorError`，其余状态还原主题默认填充色；颜色全部走
    `MaterialColors.getColor(View, attr)`，不再写死 `#C62828` / `#DE000000`；
  - 切片厚度直接以米为单位读写偏好，`snapRange()` 对齐 `stepSize`；
  - 所有 `AlertDialog.Builder` 换成 `MaterialAlertDialogBuilder`，`CheckBox` 换成 `MaterialCheckBox`；
  - 路径点命名弹窗改用 `dialog_text_input.xml`（`TextInputLayout`）。
- `res/layout/dialog_text_input.xml`（新） — 带外框的文字输入弹窗。

### 录制页

- `res/layout/activity_capture.xml` — 状态区改成卡片，按钮改为 outlined + filled 两档。
- `CaptureActivity.kt` — 去掉深色字/白字的手工切换；就绪 = `colorPrimary`，强制开始 = `colorError`，
  停止 = `colorSecondaryContainer`。错误提示同样走 `uiMessage`。

### 画面叠加层

- `PreviewView.kt` — 读数与段长标签加圆角深色底衬；数字启用 `tnum` 等宽字形，读数变化时不左右跳。
- `MapView.kt` — 小地图底板圆角 6dp → 16dp，和 M3 卡片一致。

## 决策

- **否决：暗色仪器终端 / 浅色测绘纸 / 瑞士单色。** 三者都更适合这个灰度画面 + 测量数字的场景，
  但用户明确选了 Material 3，按用户的选择执行；只在执行时用青色种子保留一点「仪器感」。
- **采用：青色种子的固定色板 + 动态色。** Android 12+ 跟随壁纸，低版本不会退化成默认紫色。
  种子沿用已有的青色叠加层，动态色关闭时和画面语言仍然一致。
- **否决：让叠加层也跟着动态色走。** 青色折线、琥珀吸附点、绿色路径点是**功能色**，
  用来区分「测量 / 吸附 / 路标」；跟着壁纸变会失去语义。它们继续写死在画笔里。
- **采用：芯片 + 横幅 + 诊断三段式。**
  - 芯片：4Hz 刷新、一眼要看到的四件事；
  - 横幅：只放「需要你注意」的话，正常跟踪时完全收起，把高度还给画面；
  - 诊断：位置、纹理、地图统计、保存结果、路径，收进抽屉，默认折叠。
- **否决：抽屉改用 `NavigationView`。** 抽屉里还有滑块、图标按钮和可展开的诊断文本，
  `NavigationView` 的菜单模型装不下，会为了形状牺牲功能。保留 `LinearLayout`，只做 M3 样式。
- **否决：edge-to-edge 全屏。** targetSdk 34 下要自己处理系统栏 inset，收益只是画面多几十像素，
  却会让相机预览顶到状态栏图标下面。保持系统默认装饰区。
- **采用：`Slider` 取代 `SeekBar`。** M3 的滑块和主题一致；代价是 `value` 必须对齐 `stepSize`，
  旧偏好里可能存着任意小数，所以加了 `snapRange()`。
- **采用：保存成功提示短暂停留 5 秒。** 保存完成后 ticker 会停，因此用 handler 单独安排一次收起；
  不这么做，要么横幅永久占位，要么用户看不到保存结果。

## 验证

**未构建、未在真机运行。** 按项目约定，编译由用户执行。

做过的静态检查：

```
$ for f in $(find app/src/main/res -name '*.xml'); do xmllint --noout "$f"; done
（无输出，全部通过）

$ 交叉检查 layout 里的 @+id 与 Kotlin 里的 R.id
used but missing: （无）
```

复查过：`MainActivity` 不再引用 `SeekBar` / `EditText` / `CheckBox` / `AlertDialog.Builder`，
`CaptureActivity` 不再引用写死的 `#C62828` / `#2E7D32` / `#DE000000`。

另外把 `material-1.12.0.aar` 拉到 `/tmp` 里核对了资源名：

```
$ grep "style Theme_Material3_DayNight_NoActionBar" R.txt
int style Theme_Material3_DayNight_NoActionBar 0x0
$ grep "style Widget_Material3_Button_IconButton" R.txt
int style Widget_Material3_Button_IconButton 0x0
$ javap ... Slider | grep addOnChangeListener
public void addOnChangeListener(com.google.android.material.slider.BaseOnChangeListener);
```

这一步抓到两个会直接编译失败的问题：

- `colorSurfaceTint` 在 Material 1.12 里不存在，已删；
- `?attr/colorBackground` 不是 Material/AppCompat 属性，框架的写法是
  `?android:attr/colorBackground`，三处布局已改。

主题里用到的其余 29 个属性、6 个 `Widget.Material3.*` 样式，以及
`MaterialSwitch` / `MaterialCheckBox` / `MaterialColors` / `DynamicColors` 的类和方法签名
都对上了。

请执行：

```bash
cd /home/welkin/SLAMAPP/SlamCapture
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

重点看四处：

1. 浅色 / 深色各启动一次，确认文字和图标没有「同色不可见」；
2. 抽屉里开关、滑块、诊断展开是否正常；
3. 词袋加载时横幅和进度条出现，加载完自动收起；
4. 走一段 → 结束 → 保存，确认「地图已保存」出现约 5 秒后消失。

## 追加（同日）：横幅改为画面内浮层

**现象（用户反馈）**：运行 SLAM 时，状态芯片下面那行提示文字时有时无，视频画面跟着上下跳。

**原因**：横幅原本在 `rowStatus` 的竖直布局流里。`trkState` 在 OK 和「不稳」之间来回跳，
横幅就在 VISIBLE / GONE 之间切换，预览的 `weight=1` 高度跟着变。

**改法**：

- `bannerWrap` 从 `rowStatus` 移到 `previewWrap` 内，`layout_gravity="bottom"`，加 `elevation`；
  它不再参与布局，出现和消失都不会改变预览高度。
- 横幅有 `elevation`，绘制顺序不一定被查看器盖住，所以 `render()` 在 `viewerOpen` 时
  显式把它设为 GONE。
- 跟踪提示加 2 秒粘滞（`trkMsg` / `trkMsgUntil`）：每出现一次警告就续 2 秒，
  跟踪恢复后等 2 秒再收起，避免 4Hz 采样下文字闪烁。
- `openHardware()` 里清空粘滞状态，上一趟的警告不会带进下一趟。

**验证**：未构建、未真机；XML 通过 `xmllint`。

## 追加（同日）：地图列表点击波纹溢出卡片

**现象（用户反馈）**：入口页点地图卡片时，圆角卡片外出现一块长方形灰色高亮，卡片下方的行距也被涂上。

**原因**：高亮来自 `ListView` 的默认 `listSelector`，它按 item 的矩形绘制，不认卡片的圆角；
卡片上原来的 `android:foreground` 在 ListView 自己处理点击时也拿不到 pressed 状态。

**改法**：

- `activity_entry.xml` 的 `ListView` 设 `android:listSelector="@android:color/transparent"`，
  去掉矩形高亮；
- `item_map.xml` 的卡片改 `clickable=true`，波纹交给 `MaterialCardView` 自己画（按圆角裁剪）；
- `EntryActivity` 去掉 `setOnItemClickListener`，在 `getView` 里给卡片设点击。

**验证**：未构建、未真机；XML 通过 `xmllint`。真机上波纹应只出现在圆角卡片内部。

## 遗留

- `res/color/measure_button_text.xml` 已无人引用；先保留，等确认测距按钮的文字色完全由
  主题接管后再删。
- Android 12+ 的动态色来自壁纸，可能和画面上的青色/琥珀功能色不协调。这是功能色与主题色分离的
  代价，暂不接受把功能色也主题化。
- 入口页空状态的图标是 24dp 矢量经 `drawableTop` 放大显示，可能偏小；真机看过后再决定是否换成
  独立 `ImageView`。
- 未做窗口 inset / 刘海屏适配，沿用系统默认装饰区。

## 回滚

按顺序：

1. `AndroidManifest.xml`：主题改回 `@style/Theme.AppCompat.NoActionBar`，删掉 `android:name=".SlamApp"`；
2. `app/build.gradle.kts`：删掉 material 依赖；
3. 删除新增的 `SlamApp.kt`、`values/colors.xml`、`values-night/`、`values/themes.xml`、`values/styles.xml`、
   `res/drawable/`、`item_map.xml`、`dialog_text_input.xml`；
4. 用 `git checkout -- app/src/main/res/layout app/src/main/java/com/example/slamcapture` 恢复其余文件。

只回滚布局会编译不过（Kotlin 引用了新的芯片/诊断视图），所以 1–4 要一起做。
