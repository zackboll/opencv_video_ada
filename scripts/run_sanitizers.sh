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

# Separate test build: fault injection never appears in the production library.
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -DOPENCV_VIDEO_TEST_FAULTS -Icpp "-I$core/cpp" $opencv_cflags \
    cpp/opencv_video_shim.cpp tests/cpp/native_boundary.cpp \
    "$core_shim" $(pkg-config --libs "$package") \
    -o "obj/sanitizers/video-faults$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 "obj/sanitizers/video-faults$suffix"

"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags $opencv_cflags tests/cpp/direct_oracle.cpp \
    $(pkg-config --libs "$package") -o "obj/sanitizers/video-oracle$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 "obj/sanitizers/video-oracle$suffix"

# Independent ECC translation oracle under the same sanitizers (fixtures only).
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -Itests/cpp $opencv_cflags tests/cpp/ecc_oracle.cpp \
    $(pkg-config --libs "$package") -o "obj/sanitizers/ecc-oracle$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "obj/sanitizers/ecc-oracle$suffix" "obj/sanitizers/ecc-translation$suffix.txt"

# Independent ECC Euclidean oracle under the same sanitizers (fixtures only).
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -Itests/cpp $opencv_cflags tests/cpp/ecc_euclidean_oracle.cpp \
    $(pkg-config --libs "$package") -o "obj/sanitizers/ecc-euclidean-oracle$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "obj/sanitizers/ecc-euclidean-oracle$suffix" "obj/sanitizers/ecc-euclidean$suffix.txt"

# Independent seeded ECC Euclidean oracle (Task 013) under the same sanitizers.
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -Itests/cpp $opencv_cflags tests/cpp/ecc_euclidean_seeded_oracle.cpp \
    $(pkg-config --libs "$package") -o "obj/sanitizers/ecc-euclidean-seeded-oracle$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "obj/sanitizers/ecc-euclidean-seeded-oracle$suffix" "obj/sanitizers/ecc-euclidean-seeded$suffix.txt"

# Independent ECC affine oracle (Task 014) under the same sanitizers.
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -Itests/cpp $opencv_cflags tests/cpp/ecc_affine_oracle.cpp \
    $(pkg-config --libs "$package") -o "obj/sanitizers/ecc-affine-oracle$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "obj/sanitizers/ecc-affine-oracle$suffix" "obj/sanitizers/ecc-affine$suffix.txt"

# Independent seeded ECC affine oracle (Task 015) under the same sanitizers.
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -Itests/cpp $opencv_cflags tests/cpp/ecc_affine_seeded_oracle.cpp \
    $(pkg-config --libs "$package") -o "obj/sanitizers/ecc-affine-seeded-oracle$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "obj/sanitizers/ecc-affine-seeded-oracle$suffix" "obj/sanitizers/ecc-affine-seeded$suffix.txt"

# Independent ECC homography oracle (Task 016) under the same sanitizers (fixtures only).
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    $flags -Itests/cpp $opencv_cflags tests/cpp/ecc_homography_oracle.cpp \
    $(pkg-config --libs "$package") -o "obj/sanitizers/ecc-homography-oracle$suffix"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "obj/sanitizers/ecc-homography-oracle$suffix" "obj/sanitizers/ecc-homography$suffix.txt"
