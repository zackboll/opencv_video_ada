#!/bin/sh
# Task 022 OPT-IN research runner for the arithmetic and pointer safety of the OpenCV 5.0.0
# FLOATING-POINT warpAffine kernels used by the existing ECC operations
# (warpAffineLinearInvoker_32FC1 and warpAffineNearestInvoker_8UC1).
#
# This is NOT part of scripts/test.sh, scripts/run_sanitizers.sh or any CI workflow, and it
# is NOT a sanitizer-clean test: against an OpenCV 5.0.0 build whose OWN libraries are
# instrumented with -fsanitize=address,undefined,float-cast-overflow, the deliberately
# extreme Level A cases are EXPECTED to record a pointer-index-overflow report from inside
# OpenCV's warp_kernels.simd.hpp. Those reports are the evidence. The runner classifies
# them, and fails only if the probe itself breaks (build error, crash, ASan/LSan error, a
# UBSan report outside OpenCV's warp sources, or a negative control that reports).
#
# Usage:
#   OPENCV_PREFIX=/path/to/instrumented/opencv-5.0.0-install \
#   OPENCV_CORE_SOURCE=/path/to/opencv_core_ada-checkout \
#   sh scripts/research_ecc5_float_warp.sh [CASE ...]
#
# With no CASE every default case runs, each in its own process (UBSan reports a source
# location once per process). Width sweeps and bounded searches are opt-in only:
#   level-a-sweep-linear:WIDTH  level-a-sweep-nearest:WIDTH          (1..65536)
#   search-translation:FIRST:COUNT  search-euclidean:FIRST:COUNT      (COUNT 1..2000)
#   search-euclidean-seeded:FIRST:COUNT  search-affine:FIRST:COUNT
#   search-affine-seeded:FIRST:COUNT
#
# OPENCV_CORE_SOURCE is the pinned Core crate (alire/cache/pins/opencv_core_<pin>), used only
# for its shim sources and the Core bridge header; nothing from it is vendored.
# Output goes to obj/research/ (ignored by git).
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
[ "$(uname -s)" = Linux ] || { echo 'error: research runner is Linux-only' >&2; exit 1; }
prefix=${OPENCV_PREFIX:?set OPENCV_PREFIX to an instrumented OpenCV 5.0 install}
core=${OPENCV_CORE_SOURCE:?set OPENCV_CORE_SOURCE to the pinned opencv_core checkout}
all_cases='conversion-check negative level-a-far-control level-a-matrix-b
level-a-boundary-below level-a-boundary-above
level-a-finite-translation-linear level-a-finite-translation-nearest
level-a-finite-euclidean level-a-finite-affine level-a-overflow-to-inf
level-a-scalar-finite-linear level-a-scalar-finite-nearest level-a-roi-linear
level-a-nonfinite-translation level-a-inf-translation level-a-nonfinite-euclidean
level-a-nonfinite-affine-coeff
trace-translation trace-euclidean trace-euclidean-seeded trace-affine trace-affine-n435
level-c-translation level-c-euclidean level-c-euclidean-seeded level-c-affine
level-c-affine-n435'
# Cases whose zero-report outcome is a required negative control.
controls='conversion-check negative level-a-far-control level-a-matrix-b level-a-boundary-below'
cases=${*:-$all_cases}
for case_name in $cases; do
    case "$case_name" in level-a-sweep-*:*|search-*:*:*) continue ;; esac
    printf '%s\n' $all_cases | grep -qx -- "$case_name" ||
        { echo "error: unknown case $case_name" >&2; exit 1; }
done
for header in "$core/cpp/opencv_core_shim.h" "$core/cpp/opencv_core_module_bridge.hpp"; do
    [ -f "$header" ] || { echo "error: missing $header" >&2; exit 1; }
done
[ -f "$prefix/lib/pkgconfig/opencv5.pc" ] ||
    { echo "error: Task 022 targets OpenCV 5.0; no opencv5.pc under $prefix/lib/pkgconfig" >&2; exit 1; }
export PKG_CONFIG_PATH="$prefix/lib/pkgconfig"
version=$(pkg-config --modversion opencv5)
cflags=$(pkg-config --cflags opencv5 | sed 's/-I/-isystem /g')
libs=$(pkg-config --libs opencv5)
compiler=${CXX:-g++}
flags='-g -O1 -fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer'
out="obj/research/$version"
mkdir -p "$out"
binary="$out/ecc5-float-warp-probe"

"$compiler" -std=c++17 -Wall -Wextra -Wpedantic -Werror $flags \
    -Icpp -Itests/cpp "-I$core/cpp" $cflags \
    "$core/cpp/opencv_core_shim.cpp" cpp/opencv_video_shim.cpp \
    tests/cpp/ecc5_float_warp_probe.cpp \
    "-L$prefix/lib" "-Wl,-rpath,$prefix/lib" $libs -o "$binary"

echo "== research_ecc5_float_warp OpenCV=$version prefix=$prefix"
echo "== compiler: $("$compiler" --version | head -n 1)"
echo "== cpu: $(uname -m) $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
echo "== probe sha256: $(sha256sum "$binary" | cut -d' ' -f1)"
echo "== loaded OpenCV libraries (ldd):"
ldd "$binary" | grep -i opencv
# The OpenCV libraries THEMSELVES must be instrumented, not just the probe and binding.
imgproc=$(ldd "$binary" | awk '/libopencv_imgproc/ {print $3}')
[ -n "$imgproc" ] || { echo 'error: libopencv_imgproc not loaded' >&2; exit 1; }
echo "== imgproc $(readlink -f "$imgproc") sha256: $(sha256sum "$imgproc" | cut -d' ' -f1)"
nm -D "$imgproc" | grep -q '__ubsan_handle_pointer_overflow' ||
    { echo "error: $imgproc is not UBSan-instrumented" >&2; exit 1; }
echo "== imgproc imports __ubsan_handle_pointer_overflow (instrumented OpenCV confirmed)"

failed=0
for case_name in $cases; do
    log="$out/$(printf '%s' "$case_name" | tr ':' '-').log"
    status=0
    ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
    UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=0:report_error_type=1 \
        "$binary" $(printf '%s' "$case_name" | tr ':' ' ') > "$log" 2>&1 || status=$?
    expected='/modules/imgproc/src/\(warp_kernels\|warp_common\)[^ ]*: runtime error:'
    upstream=$(grep -c "$expected" "$log" || true)
    foreign=$(grep 'runtime error:' "$log" | grep -vc "$expected" || true)
    asan=$(grep -c 'ERROR: AddressSanitizer\|ERROR: LeakSanitizer' "$log" || true)
    is_control=0
    if printf '%s\n' $controls | grep -qx -- "$case_name"; then is_control=1; fi
    if [ "$status" -ne 0 ] || [ "$foreign" -ne 0 ] || [ "$asan" -ne 0 ]; then
        verdict=RESEARCH_PROBE_FAILED; failed=1
    elif [ "$is_control" -eq 1 ] && [ "$upstream" -ne 0 ]; then
        verdict=NEGATIVE_CONTROL_REPORTED; failed=1
    elif [ "$upstream" -gt 0 ]; then
        verdict=EXPECTED_UPSTREAM_UB_OBSERVED
    else
        verdict=NO_UPSTREAM_REPORT
    fi
    echo "== case $case_name: $verdict (exit $status, warp UBSan $upstream, other UBSan $foreign, ASan $asan) log $log"
    grep -E 'runtime error:|DISPATCH|COORD_REACH|FIRST_UNSAFE|model-valid|SOURCE|UNSAFE_INTERMEDIATE|first unsafe|BINDING_CALL_END|published|NEGATIVE_CONTROL|CONVERSION_MODEL|BOUNDARY|SEARCH_|PEAK_RSS' "$log" | sed 's/^/     /'
done
echo "== NO_UPSTREAM_REPORT is not evidence of defined behavior. UBSan's pointer-overflow check"
echo "== fires only when the computed address wraps; an out-of-range pointer that does not wrap"
echo "== is formed silently. COORD_REACH classifies every case independently of UBSan."
exit $failed

