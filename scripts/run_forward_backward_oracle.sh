#!/bin/sh
# Standalone direct OpenCV oracle, never linked to the Ada Video binding/shim.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
package=
for candidate in opencv5 opencv4 opencv; do
    if pkg-config --exists "$candidate"; then package=$candidate; break; fi
done
[ -n "$package" ] || { echo 'error: missing OpenCV metadata' >&2; exit 1; }
compiler=$(sed -n 's/^[ ]*Cxx_Driver := "\(.*\)";/\1/p' config/opencv_video_install.gpr)
case "$(uname -s)" in
    Darwin) compiler=$(xcrun --find clang++); set -- -isysroot "$(xcrun --sdk macosx --show-sdk-path)" ;;
    MINGW*|MSYS*) compiler=$(cygpath -u "$compiler"); set -- ;;
    *) set -- ;;
esac
mkdir -p obj/oracle
cflags=$(pkg-config --cflags "$package" | sed 's/-I/-isystem /g')
"$compiler" "$@" -std=c++17 -Wall -Wextra -Wpedantic -Werror $cflags \
    tests/cpp/direct_oracle.cpp $(pkg-config --libs "$package") -o obj/oracle/direct-oracle
sh scripts/run_native.sh obj/oracle/direct-oracle obj/oracle/forward-backward.txt