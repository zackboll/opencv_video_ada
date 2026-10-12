#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
alr -n build
alr -n -C tests build
alr -n exec -- sh scripts/run_forward_backward_oracle.sh
VIDEO_FORWARD_BACKWARD_ORACLE="$root/obj/oracle/forward-backward.txt" \
VIDEO_TRACKABILITY_ORACLE="$root/obj/oracle/trackability.txt" \
VIDEO_FARNEBACK_ORACLE="$root/obj/oracle/farneback.txt" \
VIDEO_FARNEBACK_SEEDED_ORACLE="$root/obj/oracle/farneback-seeded.txt" \
VIDEO_ECC_TRANSLATION_ORACLE="$root/obj/oracle/ecc-translation.txt" \
VIDEO_ECC_EUCLIDEAN_ORACLE="$root/obj/oracle/ecc-euclidean.txt" \
VIDEO_ECC_EUCLIDEAN_SEEDED_ORACLE="$root/obj/oracle/ecc-euclidean-seeded.txt" \
VIDEO_ECC_AFFINE_ORACLE="$root/obj/oracle/ecc-affine.txt" \
    alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests
case "$(uname -s)" in
    Linux|Darwin) alr -n exec -- sh scripts/run_sanitizers.sh native ;;
esac
