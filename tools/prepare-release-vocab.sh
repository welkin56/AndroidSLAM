#!/usr/bin/env bash
#
# 把词袋 ORBvoc.txt 复制到 app/src/release/assets/，供 release 打包读进 APK。
#
# 为什么不放 src/main/assets：debug 构建会跟着变慢、变胖，改一行代码也要重打 139 MB。
#
# 用法：
#   cd android/SlamCapture && ./tools/prepare-release-vocab.sh
#
# 可用环境变量覆盖：VOCAB_SRC（本地词袋路径）
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
VOCAB_SRC=${VOCAB_SRC:-$HERE/../../ORB_SLAM3/Vocabulary/ORBvoc.txt}
DEST_DIR=$ROOT/app/src/release/assets
DEST=$DEST_DIR/ORBvoc.txt

if [ ! -f "$VOCAB_SRC" ]; then
    echo "找不到词袋：$VOCAB_SRC" >&2
    echo "用 VOCAB_SRC=<路径> 指一下。" >&2
    exit 1
fi

# 已经复制过就跳过 —— 139 MB 每次 cp 要几秒，没必要。
if [ -f "$DEST" ] && [ "$(stat -c%s "$DEST")" -eq "$(stat -c%s "$VOCAB_SRC")" ]; then
    echo "== 已经在位（$(du -h "$DEST" | cut -f1)），跳过"
    exit 0
fi

mkdir -p "$DEST_DIR"
echo "== 复制 $(du -h "$VOCAB_SRC" | cut -f1) → $DEST"
# 先写 .part 再改名，中途打断不会留下半个文件被下一次构建当成可用。
cp "$VOCAB_SRC" "$DEST.part"
mv "$DEST.part" "$DEST"

ls -l "$DEST"
echo
echo "接下来：./gradlew :app:assembleRelease"
