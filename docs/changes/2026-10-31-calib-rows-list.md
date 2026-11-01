# 2026-10-31 标定档位列表页；同档不再按 ±0.15 D 覆盖

## 动机

用户提了两件事，它们其实是一件事：

1. 在「入口页 → 标定」之间插一个**档位列表页**：每行 `d / fx / RMS / 日期`，
   能新建、能删除，能按屈光度或最近时间排序，各支持正序倒序。
2. `saveRow` 里**±0.15 D 内不要覆盖旧行**，「如果是完全相同的屈光度还是直接覆盖吧」。

第 2 条的直接后果是第 1 条的必要性：一旦允许同档附近留多行，就必须有个地方能看到它们、
删掉标歪的那一行。而它顺带修掉了上一轮讨论里的一个判据问题 ——

> 重投影误差 RMS 是**内符合**（同一批角点、同一个模型，视角少时天然更好看），
> 真正的判据是「**同一档重标多次的离散度**」。

旧逻辑把 ±0.15 D 一律当成同一档覆盖掉，恰好把这个样本销毁了：同一位置标三次，
`profile.json` 里只剩最后一次。所以这次不是加功能，是把一个**正在丢证据**的行为改掉。

## 改动

- `app/src/main/java/com/example/slamcapture/CalibProfile.kt` — 加顶层常量
  `SAME_D_EPS = 0.01f`。不能 private：`CalibActivity` 也要用。
- `app/src/main/java/com/example/slamcapture/CalibActivity.kt` — 三处：
  覆盖判据 `0.15f` → `SAME_D_EPS`；`date` 格式 `yyyy-MM-dd` → `yyyy-MM-dd HH:mm`；
  保存弹窗补一行「覆盖了 x.xx D 那一档」或「新增一档，共 N 个档位」。
- `app/src/main/java/com/example/slamcapture/CalibListActivity.kt`（新）— 列表页。
- `app/src/main/res/layout/activity_calib_list.xml`（新）— 版式照入口页：标题 → 副标题 →
  主按钮 → 分区标题 + 排序芯片 → 卡片列表。
- `app/src/main/res/layout/item_calib_row.xml`（新）— 一行卡片，骨架同 `item_map.xml`（含右侧箭头）。
- `app/src/main/AndroidManifest.xml` — 注册 `CalibListActivity`，属性照抄 `CalibActivity`。
- `app/src/main/java/com/example/slamcapture/EntryActivity.kt` — `btnCalib` 改指向列表页。

## 决策

- **采用：`ListView + BaseAdapter`，不引 RecyclerView。** 仓库里唯一的列表就是
  `EntryActivity.MapAdapter`（`item_map.xml` 卡片 + `listSelector=transparent` + 卡片自绘波纹），
  沿用它是零依赖改动；RecyclerView 要显式加依赖（material 只是传递带进来一个 2018 年的 1.0.0），
  且会和现有写法分成两套。同理不开 viewBinding —— 全仓库都是 `findViewById`。

- **采用：排序只在内存里做。** `CalibStore.load` 和 `save` 都会 `sortedBy { it.d }`
  （`CalibProfile.kt:71`、`:90`），所以「用户选的顺序」写回 JSON 会被静默丢掉。
  显示顺序由 `order: List<Int>` 独立维护，存档顺序保持按 `d` 升序 —— 那正是 `interpolate`
  期望的顺序。

- **采用：删除按 `prof.rows` 下标定位，不传 `CalibRow` 对象。**
  `CalibRow` 是 data class，两行数值完全相同时 `equals` 相等。而「同档标了两次」正是
  这次要支持的情形，用对象去 `remove` 会删错行。`order` 里存的本来就是下标，直接用它。

- **采用：`SAME_D_EPS = 0.01f`，而不是浮点 `==`。** 两次锁焦的 `LENS_FOCUS_DISTANCE`
  读数几乎不会逐位相同，写相等等于把「覆盖」这个功能删掉。0.01 D 在 1 m 处约 1 cm，
  作为「同一次标定」的判定足够严；比它远的差异就是真实档位。

- **采用：`date` 字符串加时分，不加独立时间戳字段。** 用户要求「按最近时间排序」，
  而只到天的话同一天两次标定排不出先后。改成 `yyyy-MM-dd HH:mm` 后字符串比较仍正确
  （空格 `0x20` 小于任何数字），老的 `yyyy-MM-dd` 行自动被当作当天 00:00，
  不需要写迁移代码，JSON 结构也不变。

- **采用：整页照入口页的节奏排，主按钮放标题下方而不是页底。**
  第一版把「新建档位」贴在页面底部、上方压一条分隔线，读起来像两个互不相干的区域，
  而且页面中段是一大片空白。改成入口页的顺序后：标题 → 副标题 → 主按钮 → 分区标题 →
  卡片列表，从上到下只有一条主线。代价是列表下方直接到屏幕边缘，所以 ListView 补
  `paddingBottom=20dp`。

- **采用：排序芯片贴在分区标题右侧，不单独占一行。**
  它们是列表的控件，不是页面的一层；单独占一行会让页面多一段「工具栏」，
  而这页最重要的动作是「新建」和「点开看」，排序是次要的。

- **采用：卡片右侧加 `ic_chevron_right`。** 这里点开是弹窗而不是新页面，语义上不严格对，
  但箭头在这个 App 里已经表示「这行可以点」，一致性比语义精确更重要。

- **采用：排序控件用芯片（`bg_chip` + `TextAppearance.SlamCapture.Chip`），不用 TextButton。**
  第一版是两颗 TextButton，实测在空页面上看着像两个蓝色文字链接、不像可点的控件。
  这套芯片样式在 `activity_main` 的状态栏已经在用（四颗状态芯片），换过来零新样式，
  只多一个 `bg_chip_selected.xml`（填主色，几何和 `bg_chip` 一致，切换时不跳）。
  选中态靠「填色 + 反白 + 带箭头」，未选中是描边底 —— 方向只标在选中的那颗上，
  两颗都带箭头会让人以为各自有方向。

- **否决：空状态给个图标。** 第一版用了 `drawableTop="@drawable/ic_add"`，两个问题：
  ① 和底部按钮的「+」重复，看起来像多出来一个加号；② `gravity="center"` + `drawableTop`
  在实测里把图标甩到了视图顶部，孤零零悬在排序条下面。仓库里也没有「标定」语义的图标，
  索性只留文字 —— 和入口页空状态的文字部分同一套排版（`BodyMedium` + `colorOnSurfaceVariant`
  + `\n\n` 分段）。

- **否决：在列表页再做一个「手填内参」对话框。** 能让 `source="manual"` 第一次真正被生产，
  但要做 8 个 float 的校验（而仓库里目前没有任何输入校验的先例），实现量翻倍，
  且当前需求（新建 = 标定一次）用不上。留到有实际需要时再说。

- **否决：把「哪一行在用」标出来。** 不再覆盖之后，同档附近的多行会**同时**作为插值节点，
  没有一个「在用的那一行」可言 —— 标了就是编一个代码并不执行的语义。

## 验证

**已构建、已装机**（2026-10-31）。真机观察到的两处版式问题当场改掉了：空状态的
`drawableTop` 图标与按钮的「+」重复且飘到顶上（已删）；排序控件从 `TextButton`
改成芯片（和 `activity_main` 的状态芯片同一套）。后来那一版的最终形态是
「标题 → 副标题 → 主按钮 → 分区标题 + 排序芯片 → 卡片列表」。

这一页随后成为标定链路的入口，真机跑了 5 次标定，档位写入/排序/删除都正常。

下面这段是首次安装时的检查清单，保留作回归用。

真机按顺序走一遍，覆盖全部分支：

1. 入口页「标定」→ 进**列表页**（已有档位则显示 N 行，否则空状态）
2. 「新建档位」→ 老标定页；保存 → **自动回列表页**且多出一行
3. 不换位置不换对焦再标一次 → 列表**仍只有一行**（覆盖），日期刷新到分钟
4. 换个距离再标 → **两行**
5. 点「屈光度」翻转升降序；点「最近时间」按时间排、再点翻转
6. 点一行 → 详情弹窗的 fy/cx/cy/k1/views 与列表上的 fx/RMS 对得上
7. 长按 → 删除；确认同档附近的另一行**没有被连带删掉**
8. 全删光 → 空状态；进地图，日志应是 `还没有标定表，用的是模板内参`（`MainActivity.kt:878`）

落盘抽查：

```bash
adb pull /sdcard/Android/data/com.example.slamcapture/files/calib/profile.json
```

看 `rows` 数量、`date` 是否形如 `2026-10-31 17:26`、老行是否仍为 `2026-10-30`。

## 遗留

- **详情弹窗没有显示「与同档邻行的 fx 差」。** 不覆盖之后，标歪的那一行会一直在表里，
  而「哪一行歪了」只能靠自己横向比 fx。要做的话就是在 `showDetail` 里找 0.15 D 内的邻行、
  算极差百分比 —— 那正好就是「离散度」判据的界面化。
- **没有手填 / 导入内参的入口**（见「决策」里的否决）。
- **空 `rows` 时 `CalibProfile` 仍在**（机型、相机、IMU 频率都留着），
  `MainActivity` 会走 `fallback()` 用模板内参。这条只读了代码，没真机验证。
- 排序偏好不持久化，退出页面回到默认（屈光度升序）。

## 回滚

1. 删 `CalibListActivity.kt`、`activity_calib_list.xml`、`item_calib_row.xml`；
2. `AndroidManifest.xml` 删掉 `CalibListActivity` 那 5 行（含上方注释）；
3. `EntryActivity.kt` 的 `btnCalib` 改回 `CalibActivity::class.java`；
4. `CalibProfile.kt` 删掉 `SAME_D_EPS` 常量；
5. `CalibActivity.kt` 把 `date` 格式改回 `yyyy-MM-dd`、覆盖判据改回 `0.15f`，
   弹窗那句「覆盖/新增」可留可删（只影响文案）。

`profile.json` 里已经写进去的 `yyyy-MM-dd HH:mm` 不需要回滚 —— 老代码只把它当字符串存和透传，
没有解析。
