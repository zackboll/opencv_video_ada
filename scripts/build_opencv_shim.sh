#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
set -eu
case "${1:-}" in
    Static_PIC|Relocatable) exit 0 ;;
    External_Relocatable) ;;
    *) echo 'error: missing/unknown shim build mode' >&2; exit 1 ;;
esac
[ "$#" -eq 7 ] || { echo 'error: expected build mode, compiler, include, bridge, and three import libraries' >&2; exit 1; }
cxx=$2; include=$3; bridge=$4; video_lib=$5; core=$6; core_shim=$7
case "$cxx" in *gnat_native*) echo 'error: GNAT g++ is not a compatible MSYS2 C++ driver' >&2; exit 1 ;; esac
for file in "$cxx" "$include" "$bridge/opencv_core_module_bridge.hpp" "$video_lib" "$core" "$core_shim"; do
    [ -e "$file" ] || { printf 'error: missing %s\n' "$file" >&2; exit 1; }
done
mkdir -p lib obj/shim/external
object=obj/shim/external/opencv_video_shim.o
source=cpp/opencv_video_shim.cpp
dll=lib/libopencv_video_shim.dll
import=lib/libopencv_video_shim.dll.a
rm -f "$object" "$dll" "$import"
if command -v cygpath >/dev/null 2>&1; then
    object=$(cygpath -m "$object"); source=$(cygpath -m "$source")
    dll=$(cygpath -m "$dll"); import=$(cygpath -m "$import")
fi
printf 'Building External_Relocatable OpenCV Video shim\nC++ driver: %s\n' "$cxx"
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$cxx" -c -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    -DOPENCV_VIDEO_BUILDING "-I$include" "-I$bridge" \
    -o "$object" "$source"
env -u CPATH -u C_INCLUDE_PATH -u CPLUS_INCLUDE_PATH -u LIBRARY_PATH \
    -u GCC_EXEC_PREFIX -u COMPILER_PATH \
    "$cxx" -shared -o "$dll" "-Wl,--out-implib,$import" \
    "$object" "$video_lib" "$core" "$core_shim"
