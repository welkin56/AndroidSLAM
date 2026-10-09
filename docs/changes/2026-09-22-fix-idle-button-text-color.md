# 2026-09-22 修复主界面按钮白底白字：文字色改为随状态与背景成对推出

## 动机

用户报告：**刚打开 App 时，两个按钮都是白色的，字也是白色的，看不到字。**

根因：`render()` 在中性态（IDLE / RECORDING）把 `backgroundTintList` 置 `null`，
按钮背景露出白色；而按钮文字色从未被代码设置过，一直是主题
（`Theme.AppCompat.NoActionBar`）给的白色 → 白底白字。

## 改动

- `android/SlamCapture/app/src/main/res/layout/activity_main.xml`
  — 两个 Button 显式加 `android:textColor="#DE000000"`，保证首帧渲染
  （`render()` 跑之前）也是可读的；注释里写清为什么必须显式给。

- `android/SlamCapture/app/src/main/java/com/example/slamcapture/MainActivity.kt`
  — companion 增加 `DARK_TEXT` 常量；`render()` 的三个 Phase 分支里，
  **文字色与背景成对设置**：中性态（tint=null 白底）配 `DARK_TEXT`，
  ARMED 的绿/红着色态配 `Color.WHITE`。意图是让「按钮的外观 = f(phase)」
  这条既有原则覆盖到文字色——之前只有背景从 phase 推出，文字被遗漏了。

## 决策

- **否决：** 只在 XML 里改文字色、不动 `render()`。理由：ARMED 阶段
  绿/红底若继承深色字，对比度差且和着色语义不符；文字色必须是运行时
  状态的一部分，XML 只能兜首帧。
- **否决：** 不再把 `backgroundTintList` 置 null（改成恢复主题默认着色）。
  理由：中性态用白底是现有设计（配合状态栏的绿/红形成对比），用户提的
  需求是「改字体颜色」，不动背景。若以后发现某机型默认 tint 也是浅色，
  再回头走这条路。
- **采用：** 背景、文字成对从 phase 推出。理由：改动最小、贴合用户要求、
  修复了「只改背景不改字」这一类问题的根源。

## 验证

```
$ ./gradlew compileDebugKotlin
...
> Task :app:compileDebugKotlin
w: ...CameraCapture.kt:289:11 'createCaptureSession...' is deprecated.  ← 原有警告，与本次无关
BUILD SUCCESSFUL in 9s
```

**设备上未验证**——需要用户安装后确认三个阶段的按钮文字均可读
（IDLE 深字、ARMED 绿/红白字、RECORDING 深字）。

## 遗留

- 未在真机看过效果；若 `Theme.AppCompat.NoActionBar` 在该机型上给按钮的
  禁用态（btnB disabled）加了额外的淡化处理导致深字仍不清晰，需要再调。
- RECORDING 阶段的「取消录制」同样是白底深字，与危险语义（通常用红色）
  不匹配——但这是既有设计（危险靠弹窗表达），本次没动。

## 回滚

1. `activity_main.xml`：删掉两个 Button 的 `android:textColor` 及其上方注释。
2. `MainActivity.kt`：删掉 `DARK_TEXT` 常量；`render()` 三个分支里的
   `setTextColor(...)` 调用全部删掉，恢复为只设 `backgroundTintList`。
