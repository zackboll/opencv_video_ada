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
# R1 (2026-10-19), also opt-in and never in the default set:
#   r1                                   all R1 ordinary-coordinate cases (r1_cases below)
#   r1-sweep:WIDTH[:neg[:baseline]]      (2,1) or (-2,-1) translation at one width, 32..4096
#   observe:CASE                         run CASE under gdb and classify every scalar-tail
#                                        srcptr the AVX2 kernels actually form (needs gdb and
#                                        the exact instrumented imgproc build; see observe())
# Every default case now also prints a PTR_RANGE verdict: the defined (no invalid pointer
# formed) classification of each scalar-tail src + srcstep*iy + ix against the ASan-reported
# source allocation. SOURCE_LEVEL_UNSAFE_SANITIZER_SILENT marks an out-of-allocation pointer
# that no sanitizer reported.
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
# R1 (2026-10-19) opt-in ordinary-coordinate cases. Run them by naming them (or `r1`); the
# Task 022 default set above is unchanged. r1-sweep:WIDTH[:neg[:baseline]] takes 32..4096.
r1_cases='r1-identity-control r1-ordinary-positive r1-ordinary-negative
r1-ordinary-positive-baseline r1-ordinary-negative-baseline r1-roi-positive r1-roi-negative'
# Cases whose zero-UBSan-report outcome is required. These are SANITIZER-NEGATIVE controls
# only: R1 shows that `negative` itself forms out-of-allocation scalar-tail pointers (see the
# PTR_RANGE lines), so a clean sanitizer result here is NOT evidence of pointer safety.
controls='conversion-check negative level-a-far-control level-a-matrix-b level-a-boundary-below
r1-identity-control'
# The one POINTER-RANGE-negative control: every scalar-tail pointer must stay in the chunk.
pointer_controls='r1-identity-control'
cases=${*:-$all_cases}
# Expand the `r1` keyword wherever it appears.
expanded=
for case_name in $cases; do
    if [ "$case_name" = r1 ]; then expanded="$expanded $r1_cases"
    else expanded="$expanded $case_name"; fi
done
cases=$expanded
for case_name in $cases; do
    case "$case_name" in level-a-sweep-*:*|search-*:*:*|r1-sweep:*|observe:*) continue ;; esac
    printf '%s\n' $r1_cases | grep -qx -- "$case_name" && continue
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
# R1 native observation (opt-in, `observe:CASE`): run CASE under gdb and record, at the
# instruction immediately after each scalar tail's `lea` that materialises
# srcptr = src + srcstep*iy + ix, the actual srcptr register, and at each kernel entry the
# ASan chunk of src_data. The instruction offsets are specific to ONE build, so the sha256
# of libopencv_imgproc is checked first; any other build is refused, not guessed.
r1_imgproc_sha=73c4d0d95b3dd0ad5f52b3d7f094677b3bbbe347ac10e1f9e72e87f8a15e7292
observe() {
    target=$1
    [ "$(sha256sum "$imgproc" | cut -d' ' -f1)" = "$r1_imgproc_sha" ] ||
        { echo "error: observe needs imgproc sha256 $r1_imgproc_sha (offsets are build-specific)" >&2; return 1; }
    command -v gdb > /dev/null || { echo 'error: observe needs gdb' >&2; return 1; }
    lam='::{lambda(cv::Range const&)#1}::operator()(cv::Range const&) const'
    lin="cv::opt_AVX2::warpAffineLinearInvoker_32FC1(float const*, unsigned long, int, int, float*, unsigned long, int, int, double const*, int, double const*)"
    near="cv::opt_AVX2::warpAffineNearestInvoker_8UC1(unsigned char const*, unsigned long, int, int, unsigned char*, unsigned long, int, int, double const*, int, double const*)"
    script="$out/observe-$target.gdb"
    cat > "$script" <<EOF
set pagination off
set height 0
set breakpoint pending on
set print thread-events off
handle SIGSEGV nostop noprint pass
break cv::opt_AVX2::warpAffineLinearInvoker_32FC1
run
delete
break *('$lin$lam' + 55231)
commands
silent
printf "NAT L src=%#lx srcptr=%#lx off=%ld ix=%ld iy=%ld\\n", \$rdi, \$r14, ((long)\$r14-(long)\$rdi)/4, \$rcx, \$r8
continue
end
break *('$near$lam' + 31839)
commands
silent
printf "NAT N src=%#lx srcptr=%#lx off=%ld ix=%ld iy=%ld\\n", \$r11, \$r12, (long)\$r12-(long)\$r11, \$r8, \$rax
continue
end
break cv::opt_AVX2::warpAffineLinearInvoker_32FC1
commands
silent
set \$cb = (unsigned long)__sanitizer_get_allocated_begin((void*)src_data)
printf "ENT L src=%#lx chunk=%#lx size=%lu\\n", src_data, \$cb, (unsigned long)__sanitizer_get_allocated_size((void*)\$cb)
continue
end
break cv::opt_AVX2::warpAffineNearestInvoker_8UC1
commands
silent
set \$cb = (unsigned long)__sanitizer_get_allocated_begin((void*)src_data)
printf "ENT N src=%#lx chunk=%#lx size=%lu\\n", src_data, \$cb, (unsigned long)__sanitizer_get_allocated_size((void*)\$cb)
continue
end
continue
EOF
    olog="$out/observe-$target.log"
    ASAN_OPTIONS=detect_leaks=0 gdb -batch -nx -x "$script" --args "$binary" "$target" > "$olog" 2>&1 || true
    echo "== observe $target: $(grep -oE 'BINDING_CALL_END status=[0-9]+' "$olog" || echo 'no binding call') log $olog"
    # The first linear call precedes its entry record (the bootstrap breakpoint consumes it),
    # so its tail hits are reported as UNMATCHED and never classified.
    python3 - "$olog" <<'PY'
import re, sys
ent, res, shapes = {}, {}, {}
for line in open(sys.argv[1]):
    m = re.match(r'ENT ([LN]) src=(0x[0-9a-f]+) chunk=(0x[0-9a-f]+) size=(\d+)', line)
    if m:
        ent[m.group(1)] = (int(m.group(2), 16), int(m.group(3), 16), int(m.group(4)))
        continue
    m = re.match(r'NAT ([LN]) src=(0x[0-9a-f]+) srcptr=(0x[0-9a-f]+) off=(-?\d+) ix=(-?\d+) iy=(-?\d+)', line)
    if not m:
        continue
    k, src, p = m.group(1), int(m.group(2), 16), int(m.group(3), 16)
    r = res.setdefault(k, {})
    if k not in ent or ent[k][0] != src:
        r['UNMATCHED'] = r.get('UNMATCHED', 0) + 1
        continue
    _, cb, sz = ent[k]
    c = 'PTR_OUTSIDE_ALLOCATION' if p < cb or p > cb + sz else ('PTR_ONE_PAST' if p == cb + sz else 'IN_CHUNK')
    r[c] = r.get(c, 0) + 1
    if c == 'PTR_OUTSIDE_ALLOCATION':
        key = (k, int(m.group(5)), int(m.group(6)), int(m.group(4)))
        shapes[key] = shapes.get(key, 0) + 1
name = {'L': 'opt_AVX2::warpAffineLinearInvoker_32FC1', 'N': 'opt_AVX2::warpAffineNearestInvoker_8UC1'}
for k in sorted(res):
    print('     NATIVE_OBSERVED %s %s' % (name[k], ' '.join('%s=%d' % kv for kv in sorted(res[k].items()))))
for key, n in sorted(shapes.items(), key=lambda kv: -kv[1])[:4]:
    print('     NATIVE_OUTSIDE %s ix=%d iy=%d element_offset=%d hits=%d' % (name[key[0]], key[1], key[2], key[3], n))
print('     NATIVE_OUTSIDE distinct_shapes=%d' % len(shapes))
PY
}
for case_name in $cases; do
    case "$case_name" in observe:*) observe "${case_name#observe:}" || failed=1; continue ;; esac
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
    # R1: source-level pointer-range verdict, independent of UBSan. Counts the scalar-tail
    # pointers (AVX2 or baseline scope) the defined model places outside the ASan chunk.
    oob=$(grep -c 'scope=\(avx2\|baseline\)-scalar-tail .*worst=PTR_OUTSIDE_ALLOCATION' "$log" || true)
    if [ "$oob" -gt 0 ]; then ptr=PTR_OUTSIDE_ALLOCATION
    elif grep -q 'PTR_RANGE' "$log"; then ptr=POINTER_RANGE_NEGATIVE
    else ptr=POINTER_RANGE_NOT_EVALUATED; fi
    is_ptr_control=0
    if printf '%s\n' $pointer_controls | grep -qx -- "$case_name"; then is_ptr_control=1; fi
    if [ "$status" -ne 0 ] || [ "$foreign" -ne 0 ] || [ "$asan" -ne 0 ]; then
        verdict=RESEARCH_PROBE_FAILED; failed=1
    elif [ "$is_control" -eq 1 ] && [ "$upstream" -ne 0 ]; then
        verdict=NEGATIVE_CONTROL_REPORTED; failed=1
    elif [ "$is_ptr_control" -eq 1 ] && [ "$ptr" != POINTER_RANGE_NEGATIVE ]; then
        verdict=POINTER_CONTROL_OUTSIDE_ALLOCATION; failed=1
    elif [ "$upstream" -gt 0 ]; then
        verdict=EXPECTED_UPSTREAM_UB_OBSERVED
    elif [ "$ptr" = PTR_OUTSIDE_ALLOCATION ]; then
        verdict=SOURCE_LEVEL_UNSAFE_SANITIZER_SILENT
    else
        verdict=NO_UPSTREAM_REPORT
    fi
    echo "== case $case_name: $verdict pointer=$ptr (exit $status, warp UBSan $upstream, other UBSan $foreign, ASan $asan) log $log"
    grep -E 'runtime error:|DISPATCH|COORD_REACH|FIRST_UNSAFE|model-valid|SOURCE|UNSAFE_INTERMEDIATE|first unsafe|BINDING_CALL_END|published|NEGATIVE_CONTROL|CONVERSION_MODEL|BOUNDARY|SEARCH_|PEAK_RSS|PIXEL_PTR|scope=(avx2|baseline)-scalar-tail|FIRST_OUT_OF_ALLOCATION|TRACE_SUMMARY' "$log" | sed 's/^/     /'
done
echo "== NO_UPSTREAM_REPORT is not evidence of defined behavior. UBSan's pointer-overflow check"
echo "== fires only when the computed address wraps; an out-of-range pointer that does not wrap"
echo "== is formed silently. COORD_REACH classifies every case independently of UBSan."
echo "== R1: SOURCE_LEVEL_UNSAFE_SANITIZER_SILENT means the defined pointer-range model places a"
echo "== scalar-tail src+srcstep*iy+ix outside the ASan-reported source allocation while no"
echo "== sanitizer fired. The runner exits 0 for it: it is the EXPECTED finding, not a pass."
exit $failed
