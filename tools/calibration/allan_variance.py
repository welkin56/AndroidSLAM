#!/usr/bin/env python3
"""Allan 方差标定：从**静置**录制的 imu0/data.csv 提取 IMU 噪声参数。

用法：
    ~/software/miniconda3/envs/slam/bin/python3 allan_variance.py <session 目录>

输出（打印 + 写到 <session>_calib/）：
    allan_result.txt   —— 四个噪声密度 + 拟合诊断（各轴分别给出，附三轴均值）
    allan_curve.png    —— Allan σ–τ 曲线（log-log，含拟合段标注）

四个值直接对应 phone.yaml 的键，**单位就是 yaml 注释里的单位，无需换算**——
ORB-SLAM3 的 Tracking.cc 里 `Ng*sf, Ngw/sf`（sf=√freq）的约定就是连续密度：

    IMU.NoiseGyro = N_g   rad/s/√Hz    白噪声（角随机游走），σ(τ) = N/√τ
    IMU.NoiseAcc  = N_a   m/s²/√Hz     白噪声（速随机游走），同上
    IMU.GyroWalk  = K_g   rad/s²/√Hz   bias 游走（速率随机游走），σ(τ) = K·√(τ/3)
    IMU.AccWalk   = K_a   m/s³/√Hz     同上

拟合方法：overlapping Allan deviation；在 log-log 上用滑窗找局部斜率最接近
-1/2 的 τ 段定白噪声、最接近 +1/2 的 τ 段定游走项，系数取窗口几何中点处
σ(τ)·τ^(±1/2)（对斜率偏差不敏感）。游走项用 IEEE 952 的 σ(τ)=K√(τ/3) 约定。

注意：
- **录制必须全程静置**（桌面、无震动源）。走动/倾斜的数据拟出来的不是噪声。
- 时长 15~20 分钟白噪声就准；游走项（Walk）要 1 小时+ 才细，短了会糙——
  游走项对 VIO 的影响比白噪声小一个量级，糙点可接受，先保留 yaml 现值也行。
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")          # 只存图不弹窗（click_points 的教训）
import matplotlib.pyplot as plt

# σ(τ) = K·√(τ/3) = (K/√3)·τ^(1/2) → K = √3·(σ·τ^(-1/2))
RRW_SQRT3 = np.sqrt(3.0)


def load_imu(csv_path: Path):
    ts, gyro, acc = [], [], []
    with csv_path.open() as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            ts.append(int(row[0]))
            gyro.append((float(row[1]), float(row[2]), float(row[3])))
            acc.append((float(row[4]), float(row[5]), float(row[6])))
    if len(ts) < 10_000:
        sys.exit(f"样本太少（{len(ts)}），至少录 5 分钟再来")
    t = np.asarray(ts, dtype=np.int64) / 1e9
    dt = (t[-1] - t[0]) / (len(t) - 1)
    return dt, np.asarray(gyro), np.asarray(acc)


def overlapping_adev(y, dt, taus):
    """一维速率序列的 overlapping Allan deviation σ(τ)。

    σ²(τ) = mean[(ȳ_{j+m} - ȳ_j)²] / 2，ȳ 是 m 点重叠滑动均值。
    用累积和把每一档 τ 的计算压到 O(N)。
    """
    y = np.asarray(y, dtype=np.float64)
    n = len(y)
    csum = np.concatenate(([0.0], np.cumsum(y)))
    out = np.full(len(taus), np.nan)
    for i, tau in enumerate(taus):
        m = max(int(round(tau / dt)), 2)
        if n < 2 * m + 1:
            continue
        win = (csum[m:] - csum[:-m]) / m          # 重叠窗均值
        diff = win[m:] - win[:-m]                 # 相邻 τ 对
        out[i] = np.sqrt(np.mean(diff ** 2) / 2.0)
    return out


def fit_power_law(taus, adev, target_slope, win=8):
    """滑窗 log-log 线性拟合，取斜率最接近 target_slope 的窗口。

    返回 (系数 C, 实测斜率, τ*, 窗口范围)。C 的定义：σ(τ*) = C·τ*^target_slope，
    τ* 取窗口几何中点——就算实测斜率有偏差，C 也不跟着漂。
    """
    ok = np.isfinite(adev) & (adev > 0)
    lt, la, tv, av = np.log10(taus[ok]), np.log10(adev[ok]), taus[ok], adev[ok]
    if len(lt) < 4:
        return np.nan, np.nan, np.nan, (np.nan, np.nan)
    win = min(win, len(lt))
    best = None
    for i in range(0, len(lt) - win + 1):
        s, a = np.polyfit(lt[i:i + win], la[i:i + win], 1)
        score = abs(s - target_slope)
        if best is None or score < best[0]:
            best = (score, s, i)
    _, s, i = best
    tau_star = float(np.sqrt(tv[i] * tv[i + win - 1]))
    sigma_star = float(10 ** np.interp(np.log10(tau_star), lt, la))
    C = sigma_star * tau_star ** (-target_slope)
    return C, float(s), tau_star, (float(tv[i]), float(tv[i + win - 1]))


def analyze(name, ys, dt, taus, target, walk=False):
    """三个轴各拟合一档，返回 (每轴参数数组, 每轴斜率数组, 第一轴的 σ 序列)。"""
    Cs, slopes = [], []
    adevs = []
    for k in range(3):
        a = overlapping_adev(ys[:, k], dt, taus)
        adevs.append(a)
        C, s, tau_star, span = fit_power_law(taus, a, target)
        if walk:
            C *= RRW_SQRT3          # σ=K√(τ/3) 的约定
        Cs.append(C)
        slopes.append(s)
        print(f"  {name}{'xyz'[k]}: C={C:.3e}  斜率={s:+.2f}（目标 {target:+.1f}）"
              f"  拟合 τ={span[0]:.3g}~{span[1]:.3g}s")
    return np.array(Cs), np.array(slopes), np.array(adevs)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path, help="采集目录（含 imu0/data.csv）")
    args = ap.parse_args()

    session = args.session.expanduser().resolve()
    csv_path = session / "imu0" / "data.csv"
    if not csv_path.is_file():
        sys.exit(f"找不到 {csv_path}")

    dt, gyro, acc = load_imu(csv_path)
    dur = len(gyro) * dt
    print(f"采集目录   {session}")
    print(f"样本       {len(gyro)} 条 @ {1 / dt:.1f} Hz，时长 {dur / 60:.1f} 分钟")
    if dur < 600:
        print("⚠ 时长 <10 分钟：白噪声可用，游走项（Walk）会很糙")

    # τ 从约 5 个采样点跨到总长的 1/30（再长重叠窗就太少）
    m_lo, m_hi = 5, max(int(len(gyro) / 30), 20)
    taus = np.unique(np.round(np.logspace(np.log10(m_lo * dt),
                                          np.log10(m_hi * dt), 50) / dt) * dt)
    print(f"τ 范围     {taus[0] * 1e3:.1f}ms ~ {taus[-1]:.2f}s\n")

    print("陀螺仪（rad/s 系）：")
    Ng, sg, ad_g = analyze("NoiseGyro", gyro, dt, taus, -0.5, walk=False)
    print("加速度计（m/s² 系）：")
    Na, sa, ad_a = analyze("NoiseAcc ", acc, dt, taus, -0.5, walk=False)
    print("陀螺仪游走项：")
    Kg, kg, _ = analyze("GyroWalk ", gyro, dt, taus, +0.5, walk=True)
    print("加速度游走项：")
    Ka, ka, _ = analyze("AccWalk  ", acc, dt, taus, +0.5, walk=True)

    # ---- 曲线图 ----
    out_dir = session.parent / f"{session.name}_calib"
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for ax, ad, name in ((axes[0], ad_g, "gyro"), (axes[1], ad_a, "accel")):
        for k in range(3):
            ok = np.isfinite(ad[k]) & (ad[k] > 0)
            ax.loglog(taus[ok], ad[k][ok], ".", ms=3, label=f"{name}{'xyz'[k]}")
        ax.set_xlabel("τ (s)")
        ax.set_ylabel("σ(τ)")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(f"Allan deviation — {session.name}（静置 {dur / 60:.1f} min）")
    fig.tight_layout()
    fig_path = out_dir / "allan_curve.png"
    fig.savefig(fig_path, dpi=110)

    # ---- 结果文件 ----
    def row(unit, vals, slopes, target):
        return (f"  xyz = {vals[0]:.3e} / {vals[1]:.3e} / {vals[2]:.3e}\n"
                f"  三轴均值 = {vals.mean():.3e} {unit}    拟合斜率 "
                f"{slopes[0]:+.2f}/{slopes[1]:+.2f}/{slopes[2]:+.2f}（目标 {target:+.1f}）\n")

    txt = (f"session: {session}\n"
           f"样本: {len(gyro)} @ {1 / dt:.1f} Hz，{dur / 60:.1f} min\n\n"
           f"IMU.NoiseGyro（rad/s/√Hz，白噪声）:\n{row('rad/s/√Hz', Ng, sg, -0.5)}"
           f"IMU.NoiseAcc（m/s²/√Hz，白噪声）:\n{row('m/s²/√Hz', Na, sa, -0.5)}"
           f"IMU.GyroWalk（rad/s²/√Hz，游走）:\n{row('rad/s²/√Hz', Kg, kg, +0.5)}"
           f"IMU.AccWalk（m/s³/√Hz，游走）:\n{row('m/s³/√Hz', Ka, ka, +0.5)}\n"
           f"换算约定: Tracking.cc 的 Ng*√f, Ngw/√f —— 上面的值直接填 yaml。\n"
           f"游走项约定: σ(τ)=K√(τ/3)（IEEE 952）。短录制时游走项糙，可保留旧值。\n")
    (out_dir / "allan_result.txt").write_text(txt)

    print("\n" + "=" * 60)
    print(txt)
    print(f"已写 {out_dir / 'allan_result.txt'} 与 {fig_path}")


if __name__ == "__main__":
    main()
