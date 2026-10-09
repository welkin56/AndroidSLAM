#!/usr/bin/env python3
"""用 SLAM 关键帧位姿三角化两个标记，量出它们之间的真实距离。

这是**验内参对不对**的判据：SLAM 轨迹的米制尺度来自 IMU（加速度计），
和相机的 fx 无关；而三角化的深度正比于 fx。所以 fx 若错了，
算出来的两点距离就会按同样的比例错 —— 拿直尺量的真值一比就知道了。

流程（两步）：

    # 1. 跑完 SLAM、有了 KeyFrameTrajectory.txt 之后，先看哪几帧适合点
    python3 measure_marks.py suggest <session 目录> --kf <结果目录>/KeyFrameTrajectory.txt

    # 2. 用 click_points.py 点完点之后，量距离
    python3 measure_marks.py measure <session 目录> \
        --frames <帧号A> <帧号B> --truth 0.500

第二步会对 `--try-fx` 里的每个焦距各算一遍，列成一张表。
**哪一行最接近 --truth，那个 fx 就是对的。**

⚠️ **这条判据的强弱完全取决于两个标记怎么摆。** 测出来的量是
「两点连线」的长度，而 fx 只影响**沿视线方向**的尺度：

- 两点**横平**（垂直于视线，同一深度）→ 距离几乎与 fx 无关，**测不出来**
- 两点**一近一远**（顺着视线斜放）→ 距离正比于 fx，**测得很准**

实测（基线 0.26 m）：fx 从 422.7 换到 439.2（差 3.9%），横平的两点只动 0.16%，
顺着视线的动 4.81%。所以**尺子要一头离镜头近、一头远地斜放**，不能横在面前。
工具会算出一个「灵敏度」系数直接告诉你这次摆得够不够格。

⚠️ 两帧里点的顺序必须一致 —— 第 1 个点对第 1 个点，第 2 个对第 2 个。
点反了不会报错，只会给出一个错得离谱的距离。脚本会打印每个标记的
重投影误差，点反了那个数会很大，能看出来。
"""

import argparse
import csv
import json
import re
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------- 读输入

_NUM = r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?"


def parse_phone_yaml(path: Path):
    """从 phone.yaml 抠出内参和 T_b_c1。

    这个文件是 `%YAML:1.0` + `!!opencv-matrix`，标准的 yaml 解析器读不了，
    所以用正则。只取我们要的那几个键。
    """
    txt = path.read_text()

    def scalar(key, default=0.0):
        m = re.search(rf"^\s*Camera1\.{key}\s*:\s*({_NUM})", txt, re.M)
        if not m:
            print(f"⚠ phone.yaml 里没找到 Camera1.{key}，按 {default} 算")
            return default
        return float(m.group(1))

    fx = scalar("fx", 422.7)
    intr = dict(
        fx=fx, fy=scalar("fy", fx), cx=scalar("cx"), cy=scalar("cy"),
        k1=scalar("k1"), k2=scalar("k2"), p1=scalar("p1"), p2=scalar("p2"),
    )

    # IMU.T_b_c1 是 T_body_cam（p_body = T * p_cam），由 T_wb = T_wc·T_cb 得
    # T_wc = T_wb·T_bc。**这一项不能因为杆臂为 0 就省掉** —— 一度以为 R_bc 是
    # 两帧共同的右乘、不影响距离，实测是错的：它在每个视图里乘在该帧自己的
    # 世界旋转上，不构成全局变换，省掉会把真值 0.500 m 算成 0.703 m。
    T_bc = np.eye(4)
    m = re.search(r"IMU\.T_b_c1\s*:\s*!!opencv-matrix(.*?)data\s*:\s*\[([^\]]*)\]",
                  txt, re.S)
    if m:
        vals = [float(v) for v in m.group(2).replace("\n", " ").split(",")]
        if len(vals) == 16:
            T_bc = np.array(vals).reshape(4, 4)
    else:
        print("⚠ 没解析到 IMU.T_b_c1 —— 结果不可信，相机朝向会整个错掉")
    return intr, T_bc


def load_cam0(session: Path):
    """cam0/data.csv → [(ts_ns, 帧号, 文件名)]。

    帧号是 **csv 行序号（0 开始）**，和 click_points.py 用同一套编号 ——
    第一列那个 15 位纳秒时间戳没法手输，序号可以。
    """
    p = session / "cam0" / "data.csv"
    if not p.is_file():
        sys.exit(f"找不到 {p}")
    out = []
    with p.open() as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            name = row[1] if row[1].lower().endswith(".png") else f"{row[1]}.png"
            out.append((int(row[0]), len(out), name))
    return out


def load_kf_traj(path: Path):
    """KeyFrameTrajectory.txt → [(ts_ns, t(3,), R(3,3))]。

    列是 `ts_ns tx ty tz qx qy qz qw`。IMU 模式下这是 **T_wb**（body 位姿），
    不是相机位姿 —— 见 System.cc 里 `pKF->GetImuPose()` 那行。
    """
    if not path.is_file():
        sys.exit(f"找不到 {path}")
    rows = []
    for line in path.read_text().splitlines():
        v = line.split()
        if len(v) != 8:
            continue
        ts = int(float(v[0]))
        t = np.array([float(x) for x in v[1:4]])
        qx, qy, qz, qw = (float(x) for x in v[4:8])
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        rows.append((ts, t, R))
    if not rows:
        sys.exit(f"{path} 里一行都没解析出来")
    return rows


def nearest_cam_frame(cam, ts):
    """给一个关键帧时间戳，找最近的相机帧号。"""
    return min(cam, key=lambda c: abs(c[0] - ts))


def nearest_kf(kfs, ts):
    return min(kfs, key=lambda k: abs(k[0] - ts))


# ---------------------------------------------------------------- suggest

def cmd_suggest(args):
    session = args.session.expanduser().resolve()
    cam = load_cam0(session)
    kfs = load_kf_traj(args.kf.expanduser().resolve())

    t0 = cam[0][0]
    print(f"关键帧 {len(kfs)} 个，相机帧 0~{len(cam)-1} 共 {len(cam)} 帧，"
          f"采集时长 {(cam[-1][0]-t0)/1e9:.1f} s")
    print(f"\n{'帧号':>8} {'上道后(s)':>10} {'位置 x/y/z (m)':>28}")
    for ts, t, _ in kfs:
        fno = nearest_cam_frame(cam, ts)[1]
        print(f"{fno:>8} {(ts-t0)/1e9:>10.1f} "
              f"{t[0]:>9.3f}{t[1]:>9.3f}{t[2]:>9.3f}")

    # 挑几对：基线够大（视差才够）但又别太大（标记可能已经出画了）
    cand = []
    for i in range(len(kfs)):
        for j in range(i + 1, len(kfs)):
            base = float(np.linalg.norm(kfs[j][1] - kfs[i][1]))
            dt = (kfs[j][0] - kfs[i][0]) / 1e9
            if not (args.min_base <= base <= args.max_base):
                continue
            if dt > args.max_gap:
                continue
            # 基线越接近 0.6 m 越好，时间间隔越短越好
            score = abs(base - 0.6) + 0.05 * dt
            cand.append((score, base, dt, kfs[i], kfs[j]))
    cand.sort(key=lambda c: c[0])

    print(f"\n推荐的候选帧对（基线 {args.min_base}~{args.max_base} m，"
          f"间隔 <{args.max_gap:.0f} s）：")
    print(f"{'帧号A':>8} {'帧号B':>8} {'基线(m)':>9} {'间隔(s)':>9}")
    picked = []
    for score, base, dt, a, b in cand[:args.n]:
        fa = nearest_cam_frame(cam, a[0])[1]
        fb = nearest_cam_frame(cam, b[0])[1]
        if fa in picked or fb in picked:
            continue
        picked += [fa, fb]
        print(f"{fa:>8} {fb:>8} {base:>9.3f} {dt:>9.1f}")

    # 把这些帧的图拷出来，用看图器翻一遍比挨个进 GUI 才知道标记在不在快得多
    if picked and args.preview:
        out = args.preview if isinstance(args.preview, Path) else \
            session.parent / f"{session.name}_calib" / "candidates"
        out.mkdir(parents=True, exist_ok=True)
        by_idx = {idx: name for _, idx, name in cam}
        for fno in picked:
            src = session / "cam0" / "data" / by_idx[fno]
            dst = out / f"{fno:04d}.png"
            shutil.copyfile(src, dst)
        print(f"\n候选帧的图已拷到 {out}")
        print("先用看图器翻一遍，挑两帧**两个标记都清晰入镜、没被挡住**的，")
        print("再拿它们的帧号喂给 click_points.py 的 --frames。")

    print("\n⚠ 两帧里点的顺序必须一致：第 1 个点对第 1 个点。")


# ---------------------------------------------------------------- measure

def triangulate(K, pts_a, pts_b, pose_a, pose_b, dist):
    """两帧各两个像素点 → 世界坐标下的两个三维点。

    pts_* 是 [(x, y, ok), ...]，pose_* 是 (t_wc(3,), R_wc(3,3))。
    返回 (三维点列表, 每点的视差角(度), 每点的重投影误差(px))。
    """
    # 像素 → 归一化坐标（去畸变）。之后投影矩阵里就不用再带 K 了。
    xy = [[float(p[0]), float(p[1])] for p in pts_a + pts_b]
    und = cv2.undistortPoints(
        np.array(xy, np.float64).reshape(-1, 1, 2), K, dist)
    und = und.reshape(-1, 2)
    n = len(pts_a)
    na, nb = und[:n], und[n:]

    out, angles, reps = [], [], []
    for i in range(n):
        rays, cams = [], []
        for u, (t, R) in ((na[i], pose_a), (nb[i], pose_b)):
            d = R @ np.array([u[0], u[1], 1.0])
            rays.append(d)
            cams.append(t)

        # 视差角：两帧打向同一个点的射线夹角。太小 → 深度不可信。
        cos = float(np.dot(rays[0], rays[1]) /
                    (np.linalg.norm(rays[0]) * np.linalg.norm(rays[1]) + 1e-12))
        angles.append(np.degrees(np.arccos(np.clip(cos, -1, 1))))

        # 两帧的投影矩阵：[R_cw | t_cw]，作用在世界点上。用归一化坐标，省掉 K。
        proj = []
        for (t, R) in (pose_a, pose_b):
            R_cw = R.T
            proj.append(np.hstack([R_cw, (-R_cw @ t).reshape(3, 1)]))
        X = cv2.triangulatePoints(proj[0], proj[1],
                                  na[i].reshape(2, 1), nb[i].reshape(2, 1))
        X = (X[:3] / X[3]).reshape(3)
        out.append(X)

        # 重投影误差：把 X 投回两帧，和用户点的地方比。对应关系点反了，
        # 或者匹配到的关键帧位姿不对，这个数就会爆。
        e = []
        for (t, R), (x, y, _) in ((pose_a, pts_a[i]), (pose_b, pts_b[i])):
            Xc = R.T @ (X - t)                      # 世界 → 相机
            p = Xc[:2] / Xc[2]
            e.append(float(np.hypot(p[0] * K[0, 0] + K[0, 2] - x,
                                    p[1] * K[1, 1] + K[1, 2] - y)))
        reps.append(e)
    return out, angles, reps


def cmd_measure(args):
    session = args.session.expanduser().resolve()
    cam = load_cam0(session)
    kfs = load_kf_traj(args.kf.expanduser().resolve())
    intr, T_bc = parse_phone_yaml(args.yaml.expanduser().resolve())

    json_path = args.json or (session.parent / f"{session.name}_calib" / "clicked_points.json")
    if not json_path.is_file():
        sys.exit(f"找不到 {json_path}，先跑 click_points.py")
    clicked = json.loads(json_path.read_text())

    frames = args.frames or [int(k) for k in clicked]
    if len(frames) != 2:
        sys.exit(f"需要恰好两帧，给了 {frames}")
    if not all(str(f) in clicked for f in frames):
        sys.exit(f"clicked_points.json 里没有帧 {frames} 的点；有 {list(clicked)}")

    ts_of = {idx: ts for ts, idx, _ in cam}

    # 两帧各自匹配最近的关键帧，拿 T_wb，再右乘 T_bc 得到相机位姿 T_wc
    poses, used = [], []
    for f in frames:
        if f not in ts_of:
            sys.exit(f"帧号 {f} 不在 0~{len(cam)-1} 里")
        ts = ts_of[f]
        kts, kt, kR = nearest_kf(kfs, ts)
        w = (kts - ts) / 1e9
        if abs(w) > args.max_kf_gap:
            sys.exit(f"帧 {f} 离最近的关键帧差了 {w:+.2f} s（上限 "
                     f"{args.max_kf_gap} s）—— 位姿对不上，换一帧")
        T_wb = np.eye(4)
        T_wb[:3, :3], T_wb[:3, 3] = kR, kt
        T_wc = T_wb @ T_bc
        poses.append((T_wc[:3, 3], T_wc[:3, :3]))
        used.append((f, kts, w))

    base = float(np.linalg.norm(poses[1][0] - poses[0][0]))
    print(f"帧 {frames[0]} ↔ 关键帧 ts={used[0][1]}（差 {used[0][2]:+.3f} s）")
    print(f"帧 {frames[1]} ↔ 关键帧 ts={used[1][1]}（差 {used[1][2]:+.3f} s）")
    print(f"两帧相机基线：{base:.4f} m")

    pts_a = [tuple(p) for p in clicked[str(frames[0])]]
    pts_b = [tuple(p) for p in clicked[str(frames[1])]]
    if len(pts_a) != len(pts_b):
        sys.exit(f"两帧点数不一样：{len(pts_a)} vs {len(pts_b)}")
    for tag, pts in (("A", pts_a), ("B", pts_b)):
        for i, p in enumerate(pts):
            if len(p) > 2 and not p[2]:
                print(f"⚠ 帧{tag} 第 {i+1} 个点标了「未吸附」——如果用的是 "
                      f"--refine，说明当时没找到标记")

    dist = np.array([intr["k1"], intr["k2"], intr["p1"], intr["p2"], 0.0])

    def at(fx):
        """给定 fx 算一遍，返回 (距离, 视差角, 重投影误差)。"""
        K = np.array([[fx, 0, intr["cx"]],
                      [0, fx if args.square_px else intr["fy"], intr["cy"]],
                      [0, 0, 1]], np.float64)
        X, ang, rep = triangulate(K, pts_a, pts_b, poses[0], poses[1], dist)
        return float(np.linalg.norm(X[0] - X[1])), ang, rep

    print(f"\n{'fx':>9} {'距离(m)':>10} {'误差':>10} {'灵敏度':>9} "
          f"{'视差(°)':>9} {'重投影(px)':>12}")
    results = []
    for fx in args.try_fx or [intr["fx"]]:
        d, ang, rep = at(fx)
        # 灵敏度：fx 变 1% 时距离变百分之几。两点连线越顺着视线越大。
        # 横平的两点（垂直于视线）这个数会趋近 0 —— 那种摆法测不出 fx 对错。
        d5 = at(fx * 1.05)[0]
        sens = (d5 - d) / d / 5 * 100
        err = "" if args.truth is None else f"{(d/args.truth-1)*100:+.2f}%"
        results.append((fx, d, ang, rep, sens))
        print(f"{fx:>9.1f} {d:>10.4f} {err:>10} {sens:>8.2f}x "
              f"{min(ang):>9.2f} {max(max(r) for r in rep):>12.2f}")

    print("\n逐点细节：")
    for fx, d, ang, rep, sens in results:
        print(f"  fx={fx:.1f}: 视差 {ang[0]:.2f}° / {ang[1]:.2f}°，"
              f"重投影 {rep[0][0]:.2f} {rep[0][1]:.2f} px | "
              f"{rep[1][0]:.2f} {rep[1][1]:.2f} px")

    worst = max(abs(r[4]) for r in results)
    print()
    if worst < 0.3:
        print(f"⚠ 灵敏度只有 {worst:.2f}x —— 你点的两个标记**基本垂直于视线**，"
              "\n  这种摆法下距离几乎与 fx 无关，**分不出 fx 对错**。"
              "\n  换一次采集：让尺子**一头离镜头近、一头远**（顺着视线斜放），"
              "\n  两个标记的深度差越大越好，灵敏度能到 1x 以上。")
    else:
        print(f"灵敏度 {worst:.2f}x：fx 变 1%，距离变 {worst:.2f}%。"
              f"fx 从 422.7 换到 439.2（差 3.9%）距离该动 "
              f"{3.9*worst:.1f}% —— 足够和直尺真值分辨。")

    if min(min(r[2]) for r in results) < 1.0:
        print("⚠ 视差不到 1°，深度基本是噪声 —— 换基线更大的两帧。")
    if max(e for r in results for pair in r[3] for e in pair) > 5:
        print("⚠ 重投影误差偏大：多半是两帧点的顺序对错了（标记对错了位），"
              "\n  或者匹配到的关键帧位姿不是那一帧的。")

    if args.truth is not None and len(results) > 1:
        best = min(results, key=lambda r: abs(r[1] / args.truth - 1))
        print(f"\n最接近真值 {args.truth} m 的是 fx = {best[0]:.1f}")


def _preview_path(v):
    """argparse 的 type 转换：空字符串 → None。

    ⚠️ 不能直接用 Path：`--preview ""` 会得到 `Path('.')`，它**是真值**，
    于是图被拷到当前目录（踩过一次，脏了 ORB_SLAM3/）。
    """
    return None if v == "" else Path(v)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    here = Path(__file__).resolve().parent
    kf_default = "KeyFrameTrajectory.txt"

    s = sub.add_parser("suggest", help="列出适合点的关键帧")
    s.add_argument("session", type=Path)
    s.add_argument("--kf", type=Path, default=Path(kf_default))
    s.add_argument("--n", type=int, default=6, help="推荐几对（默认 6）")
    s.add_argument("--min-base", type=float, default=0.20)
    s.add_argument("--max-base", type=float, default=1.50)
    s.add_argument("--max-gap", type=float, default=8.0, help="两帧最长间隔秒数")
    s.add_argument("--preview", type=_preview_path, nargs="?", const=True, default=True,
                   help="把候选帧的图拷到 <session>_calib/candidates/（默认开）；"
                        "给个路径就拷到那里；--preview \"\" 可关掉")
    s.set_defaults(func=cmd_suggest)

    m = sub.add_parser("measure", help="三角化算距离")
    m.add_argument("session", type=Path)
    m.add_argument("--kf", type=Path, default=Path(kf_default))
    m.add_argument("--json", type=Path, default=None)
    m.add_argument("--frames", type=int, nargs=2, default=None)
    m.add_argument("--truth", type=float, default=None, help="直尺量的真值（米）")
    m.add_argument("--yaml", type=Path,
                   default=here.parents[1] / "ORB_SLAM3" / "Examples"
                   / "Monocular-Inertial" / "phone.yaml")
    m.add_argument("--try-fx", type=float, nargs="+", default=None,
                   help="用这几个 fx 各算一遍（默认只用 yaml 里的）")
    m.add_argument("--square-px", action="store_true",
                   help="fy 强制等于 fx（默认用 yaml 里的 fy）")
    m.add_argument("--max-kf-gap", type=float, default=0.5,
                   help="帧与匹配到的关键帧允许的最大时间差（秒）")
    m.set_defaults(func=cmd_measure)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
