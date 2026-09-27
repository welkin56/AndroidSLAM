#!/usr/bin/env bash
#
# 交叉编译一份安卓版 Boost.Serialization，装到 app/src/main/cpp/thirdparty/boost-android/。
#
# 为什么非编不可：ORB-SLAM3 无条件链 boost_serialization —— 12 个文件里的 serialize()
# 没有 #ifdef 保护，哪怕一行地图都不存也要链上（见 STAGE5_JNI_BRIDGE.md §3.4）。
# 而 Boost 没有官方安卓发布物，只能自己编。
#
# 为什么 gitignore 掉：它是构建产物，绑死 NDK 版本和编译选项，150 MB，
# 其中真正用到的只有 libboost_serialization 一个。
#
# 用法：
#   cd android/SlamCapture && ./tools/setup-boost-android.sh            # 编完留着源码缓存
#   cd android/SlamCapture && ./tools/setup-boost-android.sh --clean    # 编完把缓存删掉
#
# 可用环境变量覆盖：BOOST_VERSION / ANDROID_NDK_HOME / API / JOBS / PREFIX / WORK
#
# 2026-09-27 实跑通过（Boost 1.83.0 + NDK 26.1.10909125 + API 26），
# 产出 libboost_serialization.a 1.06 MB。b2 认 --user-config。
# 未验证的：换 Boost 版本、换 NDK、换 API level。—— 以实际报错为准。
set -euo pipefail

BOOST_VERSION=${BOOST_VERSION:-1.83.0}   # 和 PC 上那份对齐（libboost_serialization.so.1.83.0）
API=${API:-26}                           # 和 build.gradle.kts 的 minSdk 一致
JOBS=${JOBS:-$(nproc)}

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PREFIX=${PREFIX:-$HERE/../app/src/main/cpp/thirdparty/boost-android}
NDK=${ANDROID_NDK_HOME:-$HOME/Android/Sdk/ndk/26.1.10909125}
WORK=${WORK:-$HOME/.cache/slamapp-boost}

# 编完之后 $WORK 里那份源码要不要留？默认留 —— 以后换 NDK 或改 API level 重编时，
# b2 能接着上次的中间产物编，省十几分钟。不想占磁盘就加 --clean，或者照结尾打印的
# 那条命令事后删。
CLEAN=0
for arg in "$@"; do
    case "$arg" in
        --clean) CLEAN=1 ;;
        -h|--help) awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "不认识的参数：$arg（只有 --clean）" >&2; exit 2 ;;
    esac
done

VERSION_UNDERSCORE=${BOOST_VERSION//./_}
TARBALL=boost_${VERSION_UNDERSCORE}.tar.bz2
URL=https://archives.boost.io/release/${BOOST_VERSION}/source/${TARBALL}
SRC=$WORK/boost_${VERSION_UNDERSCORE}
LLVM=$NDK/toolchains/llvm/prebuilt/linux-x86_64

if [ ! -x "$LLVM/bin/clang++" ]; then
    echo "找不到 NDK 的 clang：$LLVM/bin/clang++" >&2
    echo "用 ANDROID_NDK_HOME 指一下，或改脚本里的默认值。" >&2
    exit 1
fi

mkdir -p "$WORK" "$PREFIX"

# ---- 1. 下载并解包 ----
if [ ! -d "$SRC" ]; then
    if [ ! -f "$WORK/$TARBALL" ]; then
        echo "== 下载 $URL"
        if command -v curl >/dev/null; then
            curl -fL --retry 3 -o "$WORK/$TARBALL" "$URL"
        else
            wget -O "$WORK/$TARBALL" "$URL"
        fi
    fi
    echo "== 解包 $TARBALL"
    tar xf "$WORK/$TARBALL" -C "$WORK"
fi

# ---- 2. 告诉 b2 用 NDK 的 clang ----
# Boost 没有现成的 android toolset，得自己拼一个：clang 加一串交叉编译参数。
# --target 指到 aarch64-linux-android<API>，并显式带上 llvm-ar/llvm-ranlib
# （NDK 里没有叫 ar 的那个）。
cat > "$WORK/user-config.jam" <<EOF
using clang : android
    : $LLVM/bin/clang++ :
      <archiver>$LLVM/bin/llvm-ar
      <ranlib>$LLVM/bin/llvm-ranlib
      <compileflags>--target=aarch64-linux-android$API
      <compileflags>--sysroot=$LLVM/sysroot
      <compileflags>-fPIC
      <compileflags>-std=c++14
      <compileflags>-D__ANDROID_API__=$API
    ;
EOF

# ---- 3. 编，然后装 ----
# --layout=system 是必须的：默认布局给出的是带工具链后缀的长名字
# （libboost_serialization-clang-linux-mt-...a），而 CMakeLists 里 find_library
# 找的是朴素名 libboost_serialization.a。
if [ ! -x "$SRC/b2" ]; then
    echo "== bootstrap"
    (cd "$SRC" && ./bootstrap.sh)
fi

echo "== b2 交叉编译（-j$JOBS，只编 serialization）"
(cd "$SRC" && ./b2 -j"$JOBS" \
    --user-config="$WORK/user-config.jam" \
    toolset=clang-android \
    target-os=android \
    architecture=arm \
    address-model=64 \
    binary-format=elf \
    threading=multi \
    link=static \
    runtime-link=shared \
    variant=release \
    --with-serialization \
    --layout=system \
    --prefix="$PREFIX" \
    install)

# ---- 4. 自证 ----
echo
if [ -f "$PREFIX/lib/libboost_serialization.a" ]; then
    echo "== 装好了：$PREFIX"
    ls -la "$PREFIX/lib/"
    echo
    echo "头文件：$(du -sh "$PREFIX/include" | cut -f1)（Boost 的头是全量装的，只有几个被用到）"
    echo
    # 实测：源码树 921 MB + 源码包 118 MB ≈ 1.1 GB
    if [ "$CLEAN" = 1 ]; then
        echo "== --clean：删掉临时文件 $(du -sh "$WORK" 2>/dev/null | cut -f1)"
        rm -rf "$WORK"
    else
        echo "== 临时文件留着了（下次重编能快十几分钟）："
        echo "     $WORK   $(du -sh "$WORK" 2>/dev/null | cut -f1)"
        echo "   不要了就：  rm -rf $WORK"
    fi
    echo
    echo "接下来：cd android/SlamCapture && ./gradlew assembleDebug"
else
    echo "== 没找到 $PREFIX/lib/libboost_serialization.a，编失败了" >&2
    exit 1
fi
