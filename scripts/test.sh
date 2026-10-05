#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
alr -n build
alr -n -C tests build
alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests
case "$(uname -s)" in
    Linux|Darwin) alr -n exec -- sh scripts/run_sanitizers.sh native ;;
esac
