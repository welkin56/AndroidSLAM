#!/usr/bin/env bash
#
# 抓一份 IMU 噪声的运行时状态，用于标定前后对比。
#
#   bash tools/snapshot-imu-noise.sh before
#   （在 App 里跑一次 IMU 噪声标定）
#   bash tools/snapshot-imu-noise.sh after
#   diff -u /tmp/imu-noise-before/summary.txt /tmp/imu-noise-after/summary.txt
#
# 抓三样：ORB-SLAM3 实际读的 files/phone.yaml、profile.json 的 imu_noise 段、
# 当前提交号（确认两次对比的是同一份代码）。
#
# 注意：标定完不会立刻改 phone.yaml。那份文件在 nativeStart 之前由
# resolveSettingsPath() 重新生成，所以「标定后」要看 profile.json；
# 「标定 + 冷启动 App 并进一次地图」之后，phone.yaml 才会跟着变。
set -u

TAG="${1:-}"
if [ -z "$TAG" ]; then
    echo "用法: $0 <标签>   例：$0 before"
    exit 1
fi

PKG=com.example.slamcapture
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="/tmp/imu-noise-$TAG"
mkdir -p "$OUT"

if ! adb get-state >/dev/null 2>&1; then
    echo "没有连上设备，先跑 adb devices"
    exit 1
fi

# 1) 运行时配置。ORB-SLAM3 读的是这一份，不是 assets 里的模板。
adb shell run-as "$PKG" cat files/phone.yaml > "$OUT/phone.yaml" 2>/dev/null
# 2) 标定档位表与噪声记录。external files dir，adb shell 能直接 cat。
adb shell cat "/sdcard/Android/data/$PKG/files/calib/profile.json" > "$OUT/profile.json" 2>/dev/null
# 3) 代码版本
git -C "$ROOT" rev-parse --short HEAD > "$OUT/commit.txt" 2>/dev/null || echo unknown > "$OUT/commit.txt"

{
    echo "# $TAG   $(date '+%F %T')   commit=$(cat "$OUT/commit.txt")"
    echo
    echo "[运行时 phone.yaml]   ORB-SLAM3 实际读的"
    if [ -s "$OUT/phone.yaml" ]; then
        grep -E '^IMU\.(NoiseGyro|NoiseAcc|GyroWalk|AccWalk|Frequency)' "$OUT/phone.yaml" \
            | sed 's/^/  /'
    else
        echo "  读不到。release 包不能用 run-as；debug 包要先启动过一次 SLAM。"
    fi
    echo
    echo "[profile.json 的 imu_noise]"
    if [ -s "$OUT/profile.json" ]; then
        if grep -q '"imu_noise"' "$OUT/profile.json"; then
            sed -n '/"imu_noise"/,/^  }/p' "$OUT/profile.json" | sed 's/^/  /'
            grep -o '"imu_frequency_hz": [0-9.]*' "$OUT/profile.json" | head -1 \
                | sed 's/^/  /'
        else
            echo "  没有 imu_noise 段（这一版 APK 还没有 IMU 标定功能）"
        fi
    else
        echo "  没有 profile.json"
    fi
} | tee "$OUT/summary.txt"

echo
echo "原始文件在 $OUT/"
