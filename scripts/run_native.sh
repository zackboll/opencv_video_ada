#!/bin/sh
set -eu
[ "$#" -ge 1 ] || { echo "usage: $0 EXECUTABLE [ARGUMENT...]" >&2; exit 1; }
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
binary=$1; shift
case "$(uname -s)" in
    MINGW*|MSYS*)
        driver=$(sed -n 's/^[ ]*Cxx_Driver := "\(.*\)";/\1/p' "$root/config/opencv_video_install.gpr")
        [ -n "$driver" ] || { echo 'error: build before running Windows binaries' >&2; exit 1; }
        mingw_bin=$(cygpath -u "$(dirname "$driver")")
        core_prefix=${OPENCV_CORE_ALIRE_PREFIX:?Run this script through alr exec}
        core_lib=$(cygpath -u "$core_prefix/lib")
        PATH="$root/lib:$core_lib:$mingw_bin:$PATH"; export PATH
        [ -f "$binary" ] || binary="$binary.exe" ;;
esac
[ -f "$binary" ] || { printf 'error: executable not found: %s\n' "$binary" >&2; exit 1; }
exec "$binary" "$@"
