# 2026-10-24 统一业务代码与移植代码的注释风格

## 动机

用户要求在不改变逻辑、功能、命名和可执行行为的前提下，规范、补充和修正已提交代码中的注释。
范围先定为 `app/src/main` 下的 Kotlin 与 C++ 业务代码；随后补充要求 **thirdparty 中移植时改过的代码也要处理**。
风格要求：简洁，少用 Markdown 符号，不把诊断过程和历史故事写进注释。

## 改动

只改注释，未改代码、字符串、标识符和格式以外的内容。

- `app/src/main/java/com/example/slamcapture/*.kt` — 精简类与方法说明，去掉 `**`、反引号、⚠️、
  `§`/`STAGE`/`APP_DESIGN` 引用和日期化的调试叙述；保留线程约束、容量契约、单位、状态码等必要信息。
- `app/src/main/cpp/*.cpp`、`*.h` — 同上。头文件改为简洁的接口说明；实现里的历史案例和根因分析删除，
  只留下「为什么必须这么写」的一句话警告。
- `thirdparty/orb_slam3`、`thirdparty/DBoW2`、`thirdparty/g2o` — 只处理移植时新增的中文注释；
  上游英文注释未动。保留 `add：` 作为移植标记，统一改成纯文本描述。
- `docs/changes/README.md` — 登记本记录。

## 决策

- **采用：按整块注释替换，而不是逐行微调。** 原注释大量使用 Markdown 强调和编号，局部替换会留下混合风格。
- **采用：删除诊断过程，保留后果警告。** 例如「tracking 非 OK 时 t_cw 是旧值」保留，
  「真机上撞过、SIGSEGV 地址多少」删除；后者已记录在 `docs/changes/` 和 `docs/bugs/`。
- **采用：第三方上游英文注释不动。** 只改本项目新增的中文注释，降低后续与上游对比的成本。
- **否决：按 README 第 1–32 条逐条重写第三方注释。** 改动面过大，而且那些条目主要解释移植背景，
  不是代码本身；本次只做风格统一。
- **否决：顺手补充缺失的函数注释。** 例如简单转发的 JNI 方法保持无注释，避免制造噪音。

## 验证

用脚本分别读取 `HEAD` 与工作区的每个改动文件，剥离 `//`、`/* */` 注释后比较非空白 token 序列，
确认代码与字符串字面量完全一致：

```
$ ~/software/miniconda3/envs/slam/bin/python3 -I /tmp/strip_comments.py
CODE DIFF FILES: none
```

**未验证：** 未执行 Gradle / NDK 构建。本次改动只在注释和已有文档中，未触碰编译单元的逻辑。

## 遗留

- 教程文档和 `docs/changes/` 里的历史记录未改；那些地方的符号和长叙述属于文档风格，不是源码注释。
- `thirdparty/README.md` 中的移植条目说明未改。

## 回滚

`git checkout -- <文件>` 即可逐文件恢复。若整体回滚：

```
git checkout -- app/src/main docs/changes/README.md
rm docs/changes/2026-10-24-comment-cleanup.md
```
