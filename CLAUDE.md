# SLAMAPP — 工作约定

自用 Android SLAM 测距 App。**这份是给我（Claude）看的操作约定**，不是教程。
教程在 [docs/tutorial/ANDROID_SLAM_TUTORIAL.md](docs/tutorial/ANDROID_SLAM_TUTORIAL.md)。

---

## 一、每次改动后必须写改动记录

**在向用户报告「做完了」之前**，先在 `docs/changes/` 下写一份。

- 命名 `YYYY-MM-DD-<english-kebab-slug>.md` —— 英文文件名、中文正文、**绝对日期**。
- 同一天同一件事的后续**追加进已有文件**，不建 `-2` 版本。
- 粒度 = 一次提交。**能用一句话说清**就是一个文件；说不清就是几件事，该拆。
- **失败和回滚也要写。**「试过 X，结果更糟，已改回」是这里面最值钱的内容 ——
  git 只记录活下来的东西，记不下没走通的路。
- 模板 [docs/changes/_TEMPLATE.md](docs/changes/_TEMPLATE.md)，六节：
  动机 / 改动 / **决策** / 验证 / 遗留 / 回滚。
  **「决策」是核心** —— 考虑过但否决的方案，git 记不了这个。
- **验证要贴真实输出。** 没跑过就写「未验证」，不要写「应该没问题」。
- 写完在 `docs/changes/README.md` 加一行（**最新在前**），否则记录会沉底。

---

## 二、项目是什么

Android App：SLAM 建图 + 实时定位 + 在**实时画面上**测距。

**硬约束（不要动摇）**

- 自用、不对外分发 → GPL 义务不触发 → ORB-SLAM3 这类 GPL 框架可以放心用。


**已定的选型**

- ORB-SLAM3 **Mono-Inertial**。纯单目没有米制尺度，测距不可用。
- JNI 桥接，已剥离 Pangolin（`Viewer.cc` / `MapDrawer.cc` / `System.cc`）。
- 测距的深度来自 SLAM 维护的**稀疏地图**，不是照片本身。
- 交互：SLAM 先跑一段建好图 → 再在**实时画面**上测（不是拍完照再点）。
  先做「到目标的距离」，两点距离用同一套交互延伸。
- App 形态见 [docs/tutorial/APP_DESIGN.md](docs/tutorial/APP_DESIGN.md)。

---

## 三、怎么和我协作

用户在多轮来回里明确过的偏好，不是我的默认行为。

- **直接改源文件。** 不要给 `sed` 命令、不要「你自己把那几行替换成」。
- **环境类步骤反过来。** 装依赖 / 编译 / 跑标定 → 写成 `.md` 让用户执行，不代跑。
- **不要把已实现的源码嵌进教程文档。** 教程讲怎么做，代码在源文件里。
- **源码注释要短。** 一两行说清「这是什么 / 为什么这么写」就够；论证放 `docs/changes/`。
- **Python 一律用** `~/software/miniconda3/envs/slam/bin/python3`。**绝不 `pip install`**。

- **git 提交和 PR 一律不加署名行。
- **我不碰 git。** 提交、建分支、重置都留给用户；需要检查点就跟他说。

---

## 四、目录

**工作区根 = 仓库根 = `SlamCapture/`。**

```
SlamCapture/
├── CLAUDE.md
├── app/                       Android 采集端（Kotlin + JNI）
│   └── src/main/cpp/thirdparty/orb_slam3/    ORB-SLAM3 的副本
│                                             （移植改动见 thirdparty/README.md）
├── docs/
│   ├── tutorial/              教程，随阶段改写
│   ├── changes/               改动记录，只增不改，见第一节
│   └── bugs/                  踩过的坑，按名字查
└── tools/
    ├── calibration/           标定与验证脚本 + charuco 板 + 标定结果
    └── *.sh                   构建辅助（boost / 词袋推送）
```

**仓库外**（不在工作区，一般不用管）：`../ORB_SLAM3/`（上游 clone，PC 跑对照用）、
`../opencv/`、`../Pangolin/`、`../dataset/`。

> `thirdparty/` 下的 **OpenCV 官方 SDK** 和 **安卓版 Boost** 体积大（872 MB / 185 MB），
> **被 gitignore 挡着、不在 git 里** —— 换机器时按同样位置放回去即可。

---

## 五、环境

- WSL2 + WSLg（Ubuntu 24.04）。**sudo 每次都要先跟用户确认**，不要不打招呼就跑。
- Android SDK `/home/welkin/Android/Sdk`，`ANDROID_HOME` / `ANDROID_SDK_ROOT` 已设。
- JDK 17 (Temurin)。设备通过**无线 adb** 连接。

**PC 上跑 ORB-SLAM3**（在 `../ORB_SLAM3/`，不在工作区）：

```
mono_inertial_euroc <vocab> <settings> <seq_folder> <times_file> [file_name]
```

第 5 个参数**只影响输出文件名**（`kf_<name>.txt` / `f_<name>.txt`），不影响算法。
不传 → 覆盖上次；传 → 多次运行的结果可以并排留着做对比。
