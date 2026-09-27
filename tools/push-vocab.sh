#!/usr/bin/env bash
#
# 把词袋 ORBvoc.txt（145 MB）推到手机，供 App 构造 ORB-SLAM3 的 System 时读取。
#
# 为什么不进 APK：它会让 APK 从 60 MB 涨到 200 MB+，而它是一次性的
# （「实际使用时打进 assets 并 gzip」是以后的方案，见 M4 的改动记录）。
#
# 为什么用外部文件目录：和采集 session 同一个根，ADB 读写都不需要 root ——
# STAGE2 当初选 getExternalFilesDir() 就是这个理由。
#
# 用法：
#   cd android/SlamCapture && ./tools/push-vocab.sh
#
# 可用环境变量覆盖：VOCAB_SRC（本地词袋路径）、PKG
set -euo pipefail

PKG=${PKG:-com.example.slamcapture}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
VOCAB_SRC=${VOCAB_SRC:-$HERE/../../../ORB_SLAM3/Vocabulary/ORBvoc.txt}
DEST=/sdcard/Android/data/$PKG/files/

if [ ! -f "$VOCAB_SRC" ]; then
    echo "找不到词袋：$VOCAB_SRC" >&2
    echo "用 VOCAB_SRC=<路径> 指一下。" >&2
    exit 1
fi

# /sdcard/Android/data/<包名>/ 是 App 第一次跑起来时才建的。
if ! adb shell test -d "/sdcard/Android/data/$PKG"; then
    echo "手机上还没有 $PKG 的数据目录。" >&2
    echo "先把这个 App 装上并启动一次，再跑这个脚本。" >&2
    exit 1
fi

adb shell mkdir -p "$DEST"
echo "== 推 $(du -h "$VOCAB_SRC" | cut -f1) → $DEST（要等一会儿）"
adb push "$VOCAB_SRC" "$DEST"

echo
echo "== 推完了，手机上现在是："
adb shell ls -l "$DEST"
echo
echo "接下来：点 App 里的「启动 SLAM」。"
