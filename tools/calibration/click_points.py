#!/usr/bin/env python3
"""在两帧图像上点选同一对物理标记，输出像素坐标。

用法：
    ~/software/miniconda3/envs/slam/bin/python3 click_points.py <session 目录> \
        --frames <帧号> <帧号>

为什么要有这个工具：灰度 640x480 的图里**没有任何东西能自动告诉我们
「这是标记」** —— 识别这件事只能由人来做。

**默认直接用鼠标点的原始坐标**，不做任何后处理：鼠标指得准就行。
加 `--refine` 可以让程序把点吸附到附近最暗连通块的质心（实测精度 <0.8 px，
但对「点歪了」的容错是拿「可能吸到别的东西」换来的，所以默认不开）。

窗口里的按键：
    鼠标左键   添加一个点
    u          撤销上一个点
    r          清空本帧所有点
    n / 回车   本帧点完，进入下一帧
    q          放弃退出
英文标签是因为本机 matplotlib 没装中文字体，写中文会变成方框。
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import matplotlib
import numpy as np

REFINE_RADIUS = 18   # 精化时在以点击处为中心的 2R+1 方块里找连通块
MIN_MARGIN = 25      # 连通块要比周围中位数暗这么多才算「这是个真标记」
MIN_AREA = 10        # 连通块最小像素数


def load_frame_list(session: Path):
    """读 cam0/data.csv 拿帧号 → 图片路径。

    帧号就是 csv 里的**行序号（0 开始）**，不是第一列那个 15 位纳秒时间戳 ——
    时间戳没法手输，序号可以。两个脚本用的是同一套编号。

    注意 csv 的 filename 列**已经带了 .png**，别再拼一次。
    """
    csv_path = session / "cam0" / "data.csv"
    img_dir = session / "cam0" / "data"
    if not csv_path.is_file():
        sys.exit(f"找不到 {csv_path}")
    out = []
    with csv_path.open() as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            name = row[1] if row[1].lower().endswith(".png") else f"{row[1]}.png"
            out.append((len(out), img_dir / name))
    return out


def refine(img, x, y, radius=REFINE_RADIUS):
    """把点击位置吸附到附近最暗连通块的质心。返回 (x, y, 是否成功)。

    纯几何操作，不依赖任何「这是什么东西」的语义判断。
    """
    H, W = img.shape[:2]
    x0, x1 = max(0, x - radius), min(W, x + radius + 1)
    y0, y1 = max(0, y - radius), min(H, y + radius + 1)
    patch = img[y0:y1, x0:x1]
    if patch.size == 0:
        return float(x), float(y), False

    # 阈值取「比这块的中位数暗一截」——比固定阈值更抗整体明暗变化
    thr = np.median(patch) - 0.35 * (np.median(patch) - patch.min())
    mask = (patch < thr).astype(np.uint8)
    if mask.sum() == 0:
        return float(x), float(y), False

    # 开运算：噪声割出来的碎块经不起 3x3 的腐蚀，实心圆点能扛住。
    # 没有这一步的话，纯噪声里也总能找到「相对自己够暗」的小块，
    # 于是点在空白处也会假装吸附成功（实测 20/20 全部误吸）。
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    if mask.sum() == 0:
        return float(x), float(y), False

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return float(x), float(y), False

    med = float(np.median(patch))
    # 选离点击处最近的那个块（用质心距离），而不是最大的块 ——
    # 免得旁边一个更大的黑东西把点吸走
    best, bestd = None, 1e18
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < MIN_AREA:
            continue
        cx, cy = centroids[i]
        d = (cx - (x - x0)) ** 2 + (cy - (y - y0)) ** 2
        if d >= bestd:
            continue
        # 光「够暗」不够 —— 纯噪声里也能割出小块。要求整块的平均灰度
        # 比周围中位数低出一整截，才算真的有个标记在那儿。
        blob_mean = float(patch[labels == i].mean())
        if med - blob_mean < MIN_MARGIN:
            continue
        best, bestd = i, d
    if best is None:
        return float(x), float(y), False
    cx, cy = centroids[best]
    return float(cx + x0), float(cy + y0), True


def click_frame(ax, fig, img, frame_no, path, need, refine_on=False):
    """在一帧上循环等用户点，直到点够 need 个。返回点列表。

    默认**直接用鼠标点的原始坐标** —— 人自己看得准，不用算法插手。
    refine_on=True 时才把点吸附到附近最暗连通块的质心（备用，默认关）。
    """
    state = {"pts": [], "done": False}

    ax.clear()
    ax.imshow(img, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
    ax.set_title(f"frame {frame_no}  -  click {need} marker(s)  |  u=undo  r=reset  "
                 f"n/Enter=next  q=quit", fontsize=9)
    ax.set_xticks([]); ax.set_yticks([])

    def redraw():
        for t in list(ax.texts):
            t.remove()
        for ln in list(ax.lines):
            ln.remove()
        for i, (px, py, ok) in enumerate(state["pts"]):
            color = "lime" if ok else "red"
            ax.plot([px], [py], marker="+", color=color, markersize=14, mew=1.6)
            ax.text(px + 6, py - 6, f"{i+1}", color=color, fontsize=11, weight="bold")
        fig.canvas.draw_idle()

    def on_click(ev):
        if ev.inaxes is not ax or ev.xdata is None or state["done"]:
            return
        if len(state["pts"]) >= need:
            return
        if refine_on:
            x, y, ok = refine(img, int(round(ev.xdata)), int(round(ev.ydata)))
            tag = "已吸附" if ok else "未吸附（改用原始点击位置）"
        else:
            x, y, ok = float(ev.xdata), float(ev.ydata), True
            tag = "原始点击"
        state["pts"].append((x, y, ok))
        print(f"    点 {len(state['pts'])}: ({x:.2f}, {y:.2f})  {tag}")
        redraw()
        if len(state["pts"]) >= need:
            state["done"] = True
            plt_progress(fig, ax, state["pts"], frame_no)

    def on_key(ev):
        if ev.key == "u" and state["pts"]:
            state["pts"].pop()
            print("    撤销一个点")
            redraw()
        elif ev.key == "r":
            state["pts"].clear()
            print("    清空本帧")
            redraw()
        elif ev.key in ("n", "enter", None):
            if len(state["pts"]) >= need:
                state["done"] = True
                plt_progress(fig, ax, state["pts"], frame_no)
        elif ev.key == "q":
            state["quit"] = True
            state["done"] = True

    state["quit"] = False
    cid = fig.canvas.mpl_connect("button_press_event", on_click)
    kid = fig.canvas.mpl_connect("key_press_event", on_key)

    print(f"  frame {frame_no}: 请在图上点 {need} 个标记")
    while not state["done"]:
        if not fig.canvas.manager.window.isVisible():
            break
        fig.canvas.flush_events()
        matplotlib.pyplot.pause(0.05)

    fig.canvas.mpl_disconnect(cid)
    fig.canvas.mpl_disconnect(kid)
    return state["pts"], state["quit"]


def plt_progress(fig, ax, pts, frame_no):
    print(f"  frame {frame_no} 点完 {len(pts)} 个，按 n 继续（或直接关窗口）")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--frames", type=int, nargs="+", required=True,
                    help="要点的帧号（cam0/data.csv 第一列），至少两帧")
    ap.add_argument("--need", type=int, default=2, help="每帧点几个（默认 2）")
    ap.add_argument("--refine", action="store_true",
                    help="把点吸附到附近最暗连通块的质心（默认关：直接用鼠标点的坐标）")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    session = args.session.expanduser().resolve()
    frames = load_frame_list(session)
    by_no = dict(frames)

    out_path = args.out or (session.parent / f"{session.name}_calib" / "clicked_points.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    matplotlib.use("QtAgg")
    global plt
    import matplotlib.pyplot as plt_mod
    plt = plt_mod
    plt.rcParams["toolbar"] = "none"

    fig, ax = plt.subplots(figsize=(11, 8.5))
    # 必须显式把窗口显示出来。脚本模式下 subplots 建的窗口是**隐藏**的，而事件
    # 循环用 isVisible() 判断「用户是不是把窗口关了」——不 show 的话那个判断立刻
    # 成立，表现是「不弹窗、两帧都空点集、exit 0」，查起来像环境问题其实是没显示。
    # （2026-09-22 踩过。）
    plt.show(block=False)
    result = {}
    try:
        for fno in args.frames:
            if fno not in by_no:
                print(f"⚠ 帧号 {fno} 不在采集里，跳过")
                continue
            img = cv2.imread(str(by_no[fno]), cv2.IMREAD_GRAYSCALE)
            if img is None:
                print(f"⚠ 读不到 {by_no[fno]}，跳过")
                continue
            pts, quit_ = click_frame(ax, fig, img, fno, by_no[fno], args.need,
                                     refine_on=args.refine)
            if quit_:
                print("已放弃，不写出文件。")
                return
            result[str(fno)] = [[round(p[0], 3), round(p[1], 3), bool(p[2])] for p in pts]
    finally:
        plt.close(fig)

    out_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"\n已写出 {out_path}")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print("\n下一步：用 measure_marks.py 做三角化，和真值比。")


if __name__ == "__main__":
    main()
