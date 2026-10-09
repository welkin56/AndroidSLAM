#!/usr/bin/env python3
"""体检一次标定采集的几何质量：板子够不够斜、画面覆盖够不够。

回答的是一个具体问题：**这次标定的 fx 到底是「数据不够」还是「相机就这样」？**

做法是把检测到的帧按「板子法线与光轴夹角」分成几档，各档**分别标定一次**，
比 fx 差多少：

- 各档 fx 一致  → fx 已经被数据约束住了，再拍也不会变（要改只能改方法）
- 各档差很多    → 倾斜度不够，重拍时把板子拍斜，fx 会动

用法：
    ~/software/miniconda3/envs/slam/bin/python3 check_board_tilt.py <session 目录> \
        [--fx 439.2 --fy 439.4 --cx 318.4 --cy 239.9]

`--fx/--fy/--cx/--cy` 只用于解板子位姿来算倾角，不影响分组标定的结果。
检测 2000 帧要几分钟。
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import calibrate_camera as C


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--max-views", type=int, default=400, help="抽稀到多少帧")
    ap.add_argument("--fx", type=float, default=439.227)
    ap.add_argument("--fy", type=float, default=439.377)
    ap.add_argument("--cx", type=float, default=318.427)
    ap.add_argument("--cy", type=float, default=239.856)
    args = ap.parse_args()

    board = cv2.aruco.CharucoBoard(
        (C.SQUARES_X, C.SQUARES_Y), 1.0, 0.75,
        cv2.aruco.getPredefinedDictionary(C.DICT))
    frames = C.load_frame_list(args.session.expanduser().resolve())
    print(f"{len(frames)} 帧，开始检测…", flush=True)
    found = C.detect_all(board, frames)
    print(f"检出 {len(found)} 帧", flush=True)
    if len(found) < 40:
        sys.exit("检出的帧太少，分不出档 —— 这本身就是个问题（板子太小/太糊/光照不对）")
    views = C.subsample(found, args.max_views)
    print(f"抽稀到 {len(views)} 帧\n", flush=True)

    K = np.array([[args.fx, 0, args.cx], [0, args.fy, args.cy], [0, 0, 1.0]])
    obj_all = board.getChessboardCorners()
    tilt = []
    for v in views:
        ok, rvec, _ = cv2.solvePnP(obj_all[v["ids"]].astype(np.float32),
                                   v["corners"].astype(np.float32), K, None)
        if not ok:
            continue
        n = cv2.Rodrigues(rvec)[0] @ np.array([0.0, 0.0, 1.0])
        tilt.append(np.degrees(np.arccos(min(1.0, abs(n[2])))))
    tilt = np.array(tilt)

    print("板子倾角（板法线与光轴夹角）：")
    for p in (5, 25, 50, 75, 95):
        print(f"  {p:>2}% 分位: {np.percentile(tilt, p):5.1f}°")
    for lo, hi, name in ((0, 15, "近乎正对"), (15, 30, "中等倾斜"), (30, 90, "明显倾斜")):
        pct = ((tilt >= lo) & (tilt < hi)).mean() * 100
        print(f"  {name} ({lo}~{hi}°): {pct:4.0f}%")

    print("\n各档分别标定，比 fx：")
    fxs = {}
    for lo, hi, name in ((0, 15, "只留近乎正对"), (15, 30, "只留中等倾斜"),
                         (30, 90, "只留明显倾斜")):
        sel = [v for v, a in zip(views, tilt) if lo <= a < hi]
        if len(sel) < 20:
            print(f"  {name:14s} {len(sel):>4} 帧  —— 太少，跳过")
            continue
        rms, Kx, _, _, _ = cv2.calibrateCamera(
            [obj_all[v["ids"]].astype(np.float32) for v in sel],
            [v["corners"].astype(np.float32) for v in sel],
            views[0]["img_size"], None, np.zeros(5, np.float64),
            flags=cv2.CALIB_FIX_K3)
        fxs[name] = Kx[0, 0]
        print(f"  {name:14s} {len(sel):>4} 帧   fx={Kx[0,0]:8.2f}  fy={Kx[1,1]:8.2f}  "
              f"RMS={rms:.3f}px")

    if len(fxs) >= 2:
        spread = (max(fxs.values()) - min(fxs.values())) / np.mean(list(fxs.values()))
        print()
        if spread < 0.02:
            print(f"→ 各档 fx 只差 {spread*100:.1f}%。**倾斜度不是瓶颈**，"
                  "再拍一次也不会把 fx 挪走。")
            print("  要改 fx 只能改方法（别的方法/别的靶标），或者它本来就是对的。")
        else:
            print(f"→ 各档 fx 差到 {spread*100:.1f}%。**倾斜度是瓶颈** ——"
                  "重拍时把板子拍得更斜，fx 会更可信。")


if __name__ == "__main__":
    main()
