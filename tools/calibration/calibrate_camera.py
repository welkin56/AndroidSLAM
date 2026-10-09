#!/usr/bin/env python3
"""从一次采集里标定手机相机的内参和畸变。

用法：
    ~/software/miniconda3/envs/slam/bin/python3 calibrate_camera.py <session 目录>

为什么要单独写而不是用现成工具：本机没有 ROS，所以 Kalibr 用不了（见
docs/tutorial/STAGE4_CALIBRATION.md）。好在内参标定用 OpenCV 就够 ——
phone.yaml 里 `Camera.type: "PinHole"` 那个模型要的正是 k1, k2, p1, p2，
和 cv2.calibrateCamera 的输出一一对应。

用 ChArUco 板而不是普通棋盘格：角点靠 ArUco marker 定位，板子只入镜一部分
也能用，手持采集宽容得多。

**尺度说明**：板子的方格边长是相对单位（1.0），不是毫米。内参标定对物体点的
尺度不变 —— fx/fy 的单位是像素，与方格实际多大无关。所以不需要尺子。
"""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

SQUARES_X, SQUARES_Y = 9, 6      # 方格数，与 charuco_9x6.png 一致
DICT = cv2.aruco.DICT_5X5_1000
MIN_CORNERS = 8                  # 一帧至少要看到这么多角点才拿来用
DEFAULT_MAX_VIEWS = 150


def load_frame_list(session: Path):
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
            out.append((int(row[0]), img_dir / name))
    return out


def detect_all(board, frames):
    """在每帧里找 ChArUco 角点，返回检测到的那些。"""
    detector = cv2.aruco.CharucoDetector(board)
    found = []
    unreadable = 0
    for t, path in frames:
        img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            unreadable += 1
            if unreadable == 1:
                print(f"读不到图：{path}（后续同类错误不再重复打印）", file=sys.stderr)
            continue
        corners, ids, _, _ = detector.detectBoard(img)
        if ids is None or len(ids) < MIN_CORNERS:
            continue
        found.append({"t": t, "path": path, "img_size": img.shape[::-1],
                      "corners": corners, "ids": ids.ravel()})
    if unreadable:
        print(f"共 {unreadable}/{len(frames)} 帧读不出来")
        if unreadable == len(frames):
            sys.exit("一张图都读不出来 —— 检查路径，或者 csv 里的文件名格式对不对。")
    return found


def subsample(found, max_views):
    """视频 30fps，相邻帧几乎一样。均匀抽稀，避免"在哪儿停得久哪儿权重就大"。"""
    if len(found) <= max_views:
        return found
    idx = np.linspace(0, len(found) - 1, max_views).round().astype(int)
    return [found[i] for i in dict.fromkeys(idx)]


def run_calibration(board, views, img_size, max_err=None):
    obj_all = board.getChessboardCorners()
    obj_pts, img_pts = [], []
    for v in views:
        obj_pts.append(obj_all[v["ids"]].astype(np.float32))
        img_pts.append(v["corners"].astype(np.float32))

    # 只拟合 k1,k2,p1,p2 —— phone.yaml 的 PinHole 模型就这四个，没有 k3。
    #
    # 注意：**改 distCoeffs 的长度没用**。OpenCV 5 无视传入的长度，照样拟合 5 个
    # 并返回 5 个（实测传 np.zeros(4) 和 np.zeros(5) 结果逐位相同）。要限制模型
    # 只能靠 flag。CALIB_FIX_K3 把 k3 钉在 0 不参与拟合，返回的 dist 仍是 5 个元素。
    #
    # 为什么钉住是对的：k1/k2/k3 在观测到的 r 范围内几乎共线，只有它们的**和**
    # 被数据约束住，各自的值是自由变量。画面边角覆盖不足时（本项目正是如此），
    # 三者和会互相抵消出一个看着还行、但 k1/k2 毫无意义的解。
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, img_size, None, np.zeros(5, np.float64),
        flags=cv2.CALIB_FIX_K3)

    # 每帧的重投影误差
    errs = []
    for i, v in enumerate(views):
        proj, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, dist)
        errs.append(float(np.linalg.norm(proj.reshape(-1, 2) - img_pts[i].reshape(-1, 2), axis=1).mean()))
    errs = np.array(errs)

    if max_err is not None and len(views) > 10:
        keep = errs <= max_err
        if keep.sum() >= 10 and keep.sum() < len(views):
            return run_calibration(board, [v for v, k in zip(views, keep) if k], img_size, None)

    return {"rms": rms, "K": K, "dist": dist.ravel(), "errs": errs,
            "views": views, "img_size": img_size}


def coverage_map(views, img_size, out_path):
    """把检测到的角点都画到一张图上，看采集有没有覆盖到画面各个区域。"""
    W, H = img_size
    canvas = np.full((H, W, 3), 30, np.uint8)
    for v in views:
        for x, y in v["corners"].reshape(-1, 2):
            cv2.circle(canvas, (int(round(x)), int(round(y))), 3, (0, 0, 255), -1)
    # 画 3x3 网格，方便肉眼判断覆盖
    for i in (1, 2):
        cv2.line(canvas, (W * i // 3, 0), (W * i // 3, H), (90, 90, 90), 1)
        cv2.line(canvas, (0, H * i // 3), (W, H * i // 3), (90, 90, 90), 1)
    cv2.imwrite(str(out_path), canvas)
    return canvas


def quadrant_coverage(views, img_size):
    """九个格子各占多少比例的检测点 —— 用来定量判断覆盖是否均匀。"""
    W, H = img_size
    counts = np.zeros((3, 3), int)
    for v in views:
        for x, y in v["corners"].reshape(-1, 2):
            counts[min(int(y * 3 / H), 2), min(int(x * 3 / W), 2)] += 1
    total = counts.sum()
    return counts / total if total else counts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path, help="采集目录（含 cam0/data.csv）")
    ap.add_argument("--max-views", type=int, default=DEFAULT_MAX_VIEWS,
                    help=f"最多用多少帧标定（默认 {DEFAULT_MAX_VIEWS}）")
    ap.add_argument("--max-err", type=float, default=1.0,
                    help="重投影误差超过这个值（像素）的帧丢掉后重标（默认 1.0，0 表示不丢）")
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="输出目录（默认写到 session 旁边）")
    args = ap.parse_args()

    session = args.session.expanduser().resolve()
    out_dir = args.out_dir or (session.parent / f"{session.name}_calib")
    out_dir.mkdir(parents=True, exist_ok=True)

    board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), 1.0, 0.75,
                                   cv2.aruco.getPredefinedDictionary(DICT))

    frames = load_frame_list(session)
    print(f"采集目录   {session}")
    print(f"帧数       {len(frames)}")

    found = detect_all(board, frames)
    print(f"检测到板子 {len(found)} 帧（角点 ≥ {MIN_CORNERS}）")
    if len(found) < 20:
        sys.exit("\n可用帧太少，标不准。重录一次：让板子在画面里多停留、多换角度。")

    views = subsample(found, args.max_views)
    print(f"抽稀后用于标定 {len(views)} 帧")

    img_size = views[0]["img_size"]
    print(f"图像尺寸   {img_size[0]}x{img_size[1]}")
    print()

    result = run_calibration(board, views, img_size,
                             max_err=(args.max_err or None))
    views = result["views"]
    if len(views) != len(subsample(found, args.max_views)):
        print(f"丢掉重投影误差 > {args.max_err} px 的帧后，剩 {len(views)} 帧重标")

    K, dist, errs = result["K"], result["dist"], result["errs"]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    print("=" * 62)
    print(f"RMS 重投影误差   {result['rms']:.4f} px")
    print(f"每帧误差         中位 {np.median(errs):.3f}  最大 {errs.max():.3f} px")
    print()
    print(f"  fx  {fx:11.4f}      fy  {fy:11.4f}")
    print(f"  cx  {cx:11.4f}      cy  {cy:11.4f}")
    print()
    print(f"  k1  {dist[0]:11.6f}      k2  {dist[1]:11.6f}")
    print(f"  p1  {dist[2]:11.6f}      p2  {dist[3]:11.6f}")
    print("=" * 62)

    frac = quadrant_coverage(views, img_size)
    print("\n画面覆盖（3x3 网格，各格占比）：")
    for r in range(3):
        print("   " + "  ".join(f"{frac[r, c]*100:5.1f}%" for c in range(3)))
    print("   → 理想是九格都在 5% 以上且分布均匀。边角全是 0 说明镜头边缘没被约束到，")
    print("     畸变系数会不可信。")

    coverage_map(views, img_size, out_dir / "coverage.png")
    print(f"\n覆盖图已写到 {out_dir / 'coverage.png'}（红点是检测到的角点）")

    with (out_dir / "calibration_result.txt").open("w") as f:
        f.write(f"session: {session}\n")
        f.write(f"帧: 检测 {len(found)} / 使用 {len(views)}\n")
        f.write(f"RMS: {result['rms']:.6f} px\n")
        f.write(f"fx: {fx:.6f}\nfy: {fy:.6f}\ncx: {cx:.6f}\ncy: {cy:.6f}\n")
        for i, n in enumerate(["k1", "k2", "p1", "p2", "k3"]):
            if i < len(dist):
                note = "   # CALIB_FIX_K3 钉住，未参与拟合；phone.yaml 不要填这一项" if n == "k3" else ""
                f.write(f"{n}: {dist[i]:.8f}{note}\n")
        f.write("\n每帧误差:\n")
        for v, e in zip(views, errs):
            f.write(f"  {v['t']}  {e:.4f}\n")

    print(f"完整结果已写到 {out_dir / 'calibration_result.txt'}")
    print("\n下一步：把上面四个内参和畸变系数填进")
    print(f"  {Path.home()}/SLAMAPP/ORB_SLAM3/Examples/Monocular-Inertial/phone.yaml")


if __name__ == "__main__":
    main()
