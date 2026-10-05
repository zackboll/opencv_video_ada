#!/bin/sh
# Linux ASan/UBSan raw-boundary driver; "native" runs the same driver without sanitizers.
set -eu
platform=$(uname -s)
if [ "${1:-sanitizers}" != native ] && [ "$platform" != Linux ]; then
    echo 'error: Linux sanitizer driver only' >&2; exit 1
fi
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
core=${OPENCV_CORE_ALIRE_PREFIX:?run using alr exec after building Core}
package=
for candidate in opencv5 opencv4 opencv; do
    if pkg-config --exists "$candidate"; then package=$candidate; break; fi
done
[ -n "$package" ] || { echo 'error: missing OpenCV metadata' >&2; exit 1; }
mkdir -p obj/sanitizers
flags='-g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'
suffix=
case "${1:-sanitizers}" in
    sanitizers) ;;
    native) flags='-g -O1'; suffix=-native ;;
    *) echo 'error: expected native or sanitizers' >&2; exit 1 ;;
esac
core_shim="$core/lib/libopencv_core_shim.a"
opencv_cflags=$(pkg-config --cflags "$package" | sed 's/-I/-isystem /g')
compiler=${CXX:-g++}
if [ "$platform" = Darwin ]; then
    compiler=$(xcrun --find clang++)
    set -- -isysroot "$(xcrun --sdk macosx --show-sdk-path)"
    core_shim="$core/lib/libopencv_core_shim.dylib"
    export DYLD_LIBRARY_PATH="$core/lib:$root/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
else
    set --
fi
[ -f "$core_shim" ] || { echo 'error: build the resolved Core shim first' >&2; exit 1; }
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -Icpp "-I$core/cpp" $opencv_cflags \
    cpp/opencv_video_shim.cpp tests/cpp/native_boundary.cpp \
    "$core_shim" $(pkg-config --libs "$package") \
    -o "obj/sanitizers/video-boundary$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 "obj/sanitizers/video-boundary$suffix"
