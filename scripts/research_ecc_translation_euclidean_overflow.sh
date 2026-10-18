#!/bin/sh
# Task 021 OPT-IN research runner for the native signed-overflow risk in the existing ECC
# TRANSLATION and EUCLIDEAN APIs.
#
# This is NOT part of scripts/test.sh, scripts/run_sanitizers.sh or any CI workflow, and it
# is NOT a sanitizer-clean test: against an UBSan-instrumented OpenCV 4.1/4.6/4.10 the
# deliberately extreme Level A cases are EXPECTED to record signed-integer-overflow reports
# from inside OpenCV's warpAffine. Those reports are the evidence. The runner classifies
# them instead of failing on them, and fails only if the probe itself breaks (build error,
# crash, failed negative control, or a rotation-only matrix that overflows).
#
# Usage:
#   OPENCV_PREFIX=/path/to/instrumented/opencv-install \
#   OPENCV_CORE_SOURCE=/path/to/opencv_core_ada-checkout \
#   sh scripts/research_ecc_translation_euclidean_overflow.sh [CASE ...]
#
# With no CASE every case below runs, each in its own process (UBSan reports a given source
# location once per process, so cases must not share a process):
#   negative conversion-check level-a-euclidean-bigangle
#   level-a-rowbase-control level-a-rowbase-y level-a-rowbase-x level-a-rowbase-euclidean
#   level-a-conversion-euclidean
#   level-a-translation-extreme level-a-euclidean-extreme
#   level-a-nonfinite-translation level-a-nonfinite-euclidean
#   level-a-translation-far level-a-euclidean-far
#   trace-translation trace-euclidean-identity trace-euclidean-seeded
#   level-c-translation level-c-euclidean-identity level-c-euclidean-seeded
# Bounded searches are opt-in only (never in the default list):
#   search-translation:FIRST:COUNT  search-euclidean-identity:FIRST:COUNT
#   search-euclidean-seeded:FIRST:COUNT
#
# OPENCV_PREFIX must contain an OpenCV 4.1-4.x or 5.0 install whose own libraries were built
# with -fsanitize=address,undefined,float-cast-overflow (see
# docs/tasks/021-translation-euclidean-safety-audit.md for the exact CMake configuration).
# OPENCV_CORE_SOURCE is the pinned Core crate (alire/cache/pins/opencv_core_<pin>), used
# only for its shim sources and the Core bridge header; nothing from it is vendored.
# Output goes to obj/research/ (ignored by git).
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
[ "$(uname -s)" = Linux ] || { echo 'error: research runner is Linux-only' >&2; exit 1; }
prefix=${OPENCV_PREFIX:?set OPENCV_PREFIX to an instrumented OpenCV install}
core=${OPENCV_CORE_SOURCE:?set OPENCV_CORE_SOURCE to the pinned opencv_core checkout}
all_cases='negative conversion-check level-a-euclidean-bigangle
level-a-rowbase-control level-a-rowbase-y level-a-rowbase-x level-a-rowbase-euclidean
level-a-conversion-euclidean
level-a-translation-extreme level-a-euclidean-extreme
level-a-nonfinite-translation level-a-nonfinite-euclidean
level-a-translation-far level-a-euclidean-far
trace-translation trace-euclidean-identity trace-euclidean-seeded
level-c-translation level-c-euclidean-identity level-c-euclidean-seeded'
cases=${*:-$all_cases}
for case_name in $cases; do
    case "$case_name" in search-*:*:*) continue ;; esac
    printf '%s\n' $all_cases | grep -qx -- "$case_name" ||
        { echo "error: unknown case $case_name" >&2; exit 1; }
done
for header in "$core/cpp/opencv_core_shim.h" "$core/cpp/opencv_core_module_bridge.hpp"; do
    [ -f "$header" ] || { echo "error: missing $header" >&2; exit 1; }
done

pc=
for candidate in opencv5 opencv4; do
    if [ -f "$prefix/lib/pkgconfig/$candidate.pc" ]; then pc=$candidate; break; fi
done
[ -n "$pc" ] || { echo "error: no opencv4.pc/opencv5.pc under $prefix/lib/pkgconfig" >&2; exit 1; }
export PKG_CONFIG_PATH="$prefix/lib/pkgconfig"
version=$(pkg-config --modversion "$pc")
cflags=$(pkg-config --cflags "$pc" | sed 's/-I/-isystem /g')
libs=$(pkg-config --libs "$pc")
compiler=${CXX:-g++}
flags='-g -O1 -fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer'
out="obj/research/$version"
mkdir -p "$out"
binary="$out/ecc-translation-euclidean-overflow-probe"

"$compiler" -std=c++17 -Wall -Wextra -Wpedantic -Werror $flags \
    -Icpp -Itests/cpp "-I$core/cpp" $cflags \
    "$core/cpp/opencv_core_shim.cpp" cpp/opencv_video_shim.cpp \
    tests/cpp/ecc_translation_euclidean_overflow_probe.cpp \
    "-L$prefix/lib" "-Wl,-rpath,$prefix/lib" $libs -o "$binary"

echo "== research_ecc_translation_euclidean_overflow OpenCV=$version prefix=$prefix"
echo "== compiler: $("$compiler" --version | head -n 1)"
echo "== cpu: $(uname -m)"
echo "== loaded OpenCV libraries (ldd):"
ldd "$binary" | grep -i opencv

failed=0
for case_name in $cases; do
    log="$out/$(printf '%s' "$case_name" | tr ':' '-').log"
    status=0
    ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
    UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=0:report_error_type=1 \
        "$binary" $(printf '%s' "$case_name" | tr ':' ' ') > "$log" 2>&1 || status=$?
    # Expected evidence: a runtime error reported inside OpenCV's imgproc warp sources
    # (4.x legacy imgwarp fixed-point addition, or the 5.0 float-coordinate warp kernels for
    # the deliberately nonfinite cases). Any report outside those OpenCV sources (in the
    # shims or the probe) is foreign and fails the runner.
    expected='/modules/imgproc/src/\(imgwarp\|warp_kernels\|warp_common\)[^ ]*: runtime error:'
    upstream=$(grep -c "$expected" "$log" || true)
    foreign=$(grep 'runtime error:' "$log" | grep -vc "$expected" || true)
    asan=$(grep -c 'ERROR: AddressSanitizer\|ERROR: LeakSanitizer' "$log" || true)
    if [ "$status" -ne 0 ] || [ "$foreign" -ne 0 ] || [ "$asan" -ne 0 ]; then
        verdict=RESEARCH_PROBE_FAILED; failed=1
    elif [ "$upstream" -gt 0 ]; then
        verdict=EXPECTED_UPSTREAM_UB_OBSERVED
    else
        verdict=NO_UPSTREAM_REPORT
    fi
    echo "== case $case_name: $verdict (exit $status, warpAffine UBSan $upstream, other UBSan $foreign, ASan $asan) log $log"
    grep -E 'runtime error:|UNSAFE_INTERMEDIATE|MODEL_INVARIANT|first unsafe|model-valid|ROTATION_ONLY|BINDING_CALL_END|NEGATIVE_CONTROL|int64 model|SEARCH_|max.translation|CONVERSION|DIAG_|ROUNDING_MODE|MXCSR_RC|TRANSLATION_WINDOW|EUCLIDEAN_WINDOW|ROWBASE_|FIRST_UNSAFE|FIRST_UB|counts:|PEAK_RSS' "$log" | sed 's/^/     /'
done
echo "== NO_UPSTREAM_REPORT is not evidence of defined arithmetic. OpenCV 5.0 routes CV_32FC1/CV_8UC1"
echo "== through float-coordinate kernels and marks the legacy integer blockline CV_DISABLE_UBSAN."
exit $failed
