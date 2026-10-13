#!/bin/sh
# Standalone direct OpenCV oracle, never linked to the Ada Video binding/shim.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
pkg_config=${PKG_CONFIG:-pkg-config}
case "$(uname -s)" in
    MINGW*|MSYS*)
        if [ -z "${PKG_CONFIG:-}" ] && command -v x86_64-w64-mingw32-pkg-config >/dev/null 2>&1; then
            pkg_config=x86_64-w64-mingw32-pkg-config
        fi ;;
esac
package=
for candidate in opencv5 opencv4 opencv; do
    if "$pkg_config" --exists "$candidate"; then package=$candidate; break; fi
done
[ -n "$package" ] || { echo 'error: missing OpenCV metadata' >&2; exit 1; }
compiler=$(sed -n 's/^[ ]*Cxx_Driver := "\(.*\)";/\1/p' config/opencv_video_install.gpr)
case "$(uname -s)" in
    Darwin) compiler=$(xcrun --find clang++); set -- -isysroot "$(xcrun --sdk macosx --show-sdk-path)" ;;
    MINGW*|MSYS*) compiler=$(cygpath -u "$compiler"); set -- ;;
    *) set -- ;;
esac
mkdir -p obj/oracle
cflags=$("$pkg_config" --cflags "$package" | sed 's/-I/-isystem /g')
case "$(uname -s)" in
    MINGW*|MSYS*)
        # opencv5.pc includes the MinGW CRT include root as well as opencv5.
        # Reclassifying the compiler's own root breaks GCC include_next in
        # cstdlib. Use the configured OpenCV-only path, as the shim build does.
        include=$(sed -n 's/^[ ]*Include_Switch := "-I\(.*\)";/\1/p' config/opencv_video_install.gpr)
        [ -n "$include" ] || { echo 'error: missing configured OpenCV include' >&2; exit 1; }
        cflags="-isystem $include" ;;
esac
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror $cflags \
    tests/cpp/direct_oracle.cpp $("$pkg_config" --libs "$package") -o obj/oracle/direct-oracle
# Independent ECC translation oracle: same compiler/flags, own output file and format.
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror $cflags \
    tests/cpp/ecc_oracle.cpp $("$pkg_config" --libs "$package") -o obj/oracle/ecc-oracle
rm -f obj/oracle/ecc-translation.txt
sh scripts/run_native.sh obj/oracle/ecc-oracle obj/oracle/ecc-translation.txt
# Independent ECC Euclidean oracle (Task 012): own source, binary and output file.
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror $cflags -Itests/cpp \
    tests/cpp/ecc_euclidean_oracle.cpp $("$pkg_config" --libs "$package") -o obj/oracle/ecc-euclidean-oracle
rm -f obj/oracle/ecc-euclidean.txt
sh scripts/run_native.sh obj/oracle/ecc-euclidean-oracle obj/oracle/ecc-euclidean.txt
# Independent seeded ECC Euclidean oracle (Task 013): own source, binary and output file.
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror $cflags -Itests/cpp \
    tests/cpp/ecc_euclidean_seeded_oracle.cpp $("$pkg_config" --libs "$package") -o obj/oracle/ecc-euclidean-seeded-oracle
rm -f obj/oracle/ecc-euclidean-seeded.txt
sh scripts/run_native.sh obj/oracle/ecc-euclidean-seeded-oracle obj/oracle/ecc-euclidean-seeded.txt
# Independent ECC affine oracle (Task 014): own source, binary and output file.
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror $cflags -Itests/cpp \
    tests/cpp/ecc_affine_oracle.cpp $("$pkg_config" --libs "$package") -o obj/oracle/ecc-affine-oracle
rm -f obj/oracle/ecc-affine.txt
sh scripts/run_native.sh obj/oracle/ecc-affine-oracle obj/oracle/ecc-affine.txt
# Independent seeded ECC affine oracle (Task 015): own source, binary and output file.
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror $cflags -Itests/cpp \
    tests/cpp/ecc_affine_seeded_oracle.cpp $("$pkg_config" --libs "$package") -o obj/oracle/ecc-affine-seeded-oracle
rm -f obj/oracle/ecc-affine-seeded.txt
sh scripts/run_native.sh obj/oracle/ecc-affine-seeded-oracle obj/oracle/ecc-affine-seeded.txt
sh scripts/run_native.sh obj/oracle/direct-oracle obj/oracle/forward-backward.txt obj/oracle/trackability.txt obj/oracle/farneback.txt obj/oracle/farneback-seeded.txt
