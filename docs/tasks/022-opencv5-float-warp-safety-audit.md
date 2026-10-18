# Task 022 — OpenCV 5.0.0 floating-point warp kernel safety audit

Branch `safety/022-opencv5-float-warp-audit`, from verified `origin/main`
`bc4d619c4a33a5fcd55029674ccb7840f1cfbeef` (the PR #21 merge). Crate version `0.1.0-dev`,
Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326`, 25 production exports and 198 AUnit
registrations are all **unchanged**. No production Ada, C++, dependency or CI-topology change
was made: the only edits are the new opt-in research harness, documentation, and dated
consumer notices.

This task asks whether OpenCV 5.0.0's newer **floating-point** `warpAffine` kernels are safe
for the finite intermediate transforms ECC can generate, especially for very large,
nonfinite or outside-the-image coordinates. Tasks 020/021 audited the **legacy 4.x
fixed-point** path; Task 021 found that only a **NaN** translation reached a pointer-index
overflow in the 5.0 float kernel (`warp_kernels.simd.hpp:3764`) and that no finite case was
tested beyond `2^31`. Task 022 extends exactly that gap.

## Headline result

**A finite, model-valid Float32 translation (and Euclidean and affine warp) whose mapped
coordinate reaches or exceeds `2^31` executes the SAME undefined operation in the 5.0 float
kernels that Task 021 observed only for NaN.** The nonfinite NaN input was not special: the
UBSan-suppressed `cvFloor(float)` / `cvRound(float)` emit the `INT_MIN` integer-indefinite
sentinel for every `|coordinate| >= 2^31` just as they do for NaN/Inf, and that sentinel then
feeds the source-pointer `src + srcstep*iy + ix` formed **before** the bounds test. This is
`DIRECT_NATIVE_UB_CONFIRMED` for a finite, model-valid matrix (Level A), on x86_64, in both
the AVX2 and cpu_baseline dispatch paths, for both ECC kernels.

It was **not** reproduced through the native ECC optimizer (Level B) or the real binding
(Level C): no translation, Euclidean or affine ECC run in this audit produced a coordinate
within three orders of magnitude of `2^31`. The dispositions below keep those levels
independent.

## Dispositions (kept independent; one result never qualifies another path)

### Per kernel (x86_64, AVX2 + cpu_baseline, the only paths tested)

| Kernel | Finite-coordinate disposition |
|---|---|
| `warpAffineLinearInvoker_32FC1` (ECC image/gradients, INTER_LINEAR) | **`DIRECT_NATIVE_UB_CONFIRMED`** for any mapped coordinate `>= 2^31` or `< -2^31`, finite and model-valid. **`SAFETY_GATE_NOT_ESTABLISHED`** for the ECC operating envelope. |
| `warpAffineNearestInvoker_8UC1` (ECC mask, INTER_NEAREST) | **`DIRECT_NATIVE_UB_CONFIRMED`** for the same coordinate domain (separate evidence). **`SAFETY_GATE_NOT_ESTABLISHED`** for the ECC envelope. |

### Per existing ECC operation (shared native warp path)

| API / C export | Disposition |
|---|---|
| Translation `opencv_video_estimate_ecc_translation` | **`SAFETY_GATE_NOT_ESTABLISHED`** (Level A finite UB demonstrated on the shared kernel; Level B/C `NOT_REPRODUCED`). |
| Euclidean `opencv_video_estimate_ecc_euclidean` / `_seeded` | **`SAFETY_GATE_NOT_ESTABLISHED`** (same). |
| Affine `opencv_video_estimate_ecc_affine` / `_seeded` | **`SAFETY_GATE_NOT_ESTABLISHED`** on 5.0 (Level B/C `NOT_REPRODUCED` here). The 4.x `KNOWN_NATIVE_UB_UNMITIGATED` disposition from Task 020 is **unchanged**. |

No operation reaches `SAFETY_GATE_ESTABLISHED`: nothing in the ECC source bounds the
accumulated translation below `2^31` across the published envelope, and a model-valid finite
matrix with `|t| >= 2^31` **does** execute UB on the shared kernel. No operation reaches
`KNOWN_NATIVE_UB_UNMITIGATED` on 5.0 for the float kernels, because no Level B or Level C
reproducer was found. `NOT_REPRODUCED` is not proof of safety.

The earlier Task 020 (`KNOWN_NATIVE_UB_UNMITIGATED`, affine, 4.x) and Task 021
(`SAFETY_GATE_NOT_ESTABLISHED`, translation/Euclidean) dispositions and their notices remain
intact and are not weakened.

## 1. Repository verification

* `origin/main` SHA `bc4d619c4a33a5fcd55029674ccb7840f1cfbeef`, matching the expected
  baseline. PR #21 (`safety/021-translation-euclidean-ecc-audit`) is **MERGED**
  (2026-10-18T17:40:20Z, merge commit `bc4d619`).
* PR #21 post-merge CI, read once (not polled): **Cross-platform Video** run 39150664396
  **succeeded** (repository-checks, linux, macos, linux-sanitizers all success). **Windows
  post-merge Video** run 39150664411 was **in_progress** when read (started
  2026-10-18T17:40:25Z); its result is recorded as incomplete, not as a pass or a failure.
* No Task 022 branch, worktree or PR existed (`git ls-remote` and `gh pr list` checked). The
  main working tree was clean. A dedicated worktree `/home/zboll/git/opencv/video-task022` was
  created on the new branch.
* `.clinerules/*.md` read. Baseline inventory confirmed by `scripts/check_repository.py`
  (25 ABI exports, 198 AUnit registrations, Core pin, crate `0.1.0-dev`).

## 2. Exact OpenCV source audited

OpenCV `5.0.0` (`CV_VERSION_MAJOR 5 / MINOR 0 / REVISION 0`), the upstream tag tarball at
`~/.cache/video-task017/opencv-5.0.0`, the same tree that produced the instrumented install.
Line numbers below are from that tree.

| File | Lines | Content |
|---|---|---|
| `modules/video/src/ecc.cpp` | 523-526 | Four `warpAffine(..., WARP_INVERSE_MAP)` calls per iteration: image, gradX, gradY (`CV_32FC1`, `INTER_LINEAR`), mask (`CV_8UC1`, `INTER_NEAREST`) |
| | 551, 574, 581-582, 593-595, 607 | `countNonZero`, `hessian.inv()`, `rho` NaN check, `lambda_d <= 0` check, `update_warping_matrix_ECC` — all **after** the four warps |
| `modules/imgproc/src/imgwarp.cpp` | 67-73 | `getImgWarpFunc` returns a function only for `INTER_CUBIC`, so `genericWarp` (75) declines LINEAR/NEAREST |
| | 2451-2506 | `cv::warpAffine`: OpenCL branches only for `UMat` output; `genericWarp`; then `hal::warpAffine` |
| | 2238-2243 | `hal::warpAffine`: `CALL_HAL(cv_hal_warpAffine)` first (custom HAL hook) |
| | 2251 | `CV_8UC1 + INTER_NEAREST` → `CV_CPU_DISPATCH(warpAffineNearestInvoker_8UC1)` |
| | 2329 | `CV_32FC1 + INTER_LINEAR` → `CV_CPU_DISPATCH(warpAffineLinearInvoker_32FC1)` |
| `modules/imgproc/src/warp_kernels.simd.hpp` | 360-445 | `warpAffineNearestInvoker_8UC1` (vector loop 421-433, scalar tail 436-441) |
| | 3682-3771 | `warpAffineLinearInvoker_32FC1` (vector loop 3744-3757, scalar tail 3760-3767) |
| | 111, 220 | Declaration-only prototypes of the same two kernels (before `CV_CPU_OPTIMIZATION_DECLARATIONS_ONLY` at 328) |
| | 10-16, 40-67 | Vector coordinate macros: row base, `v_fma`, `v_floor`/`v_round`, inner mask, address |
| `modules/imgproc/src/warp_common.scalar.hpp` | 10-12 | `CV_WARP_SCALAR_SHUFFLE_DEF_C1`: **`srcptr = src + srcstep * iy + ix`** |
| | 130-134 | Scalar coordinate conversion: `cvRound` (nearest), `cvFloor` + fraction (linear) |
| | 136-152 | `CV_WARP_SCALAR_SHUFFLE`: COMPUTE_COORD, then DEF (pointer), **then** bounds test (139-145) |
| | 60-73 | `CV_WARP_SCALAR_FETCH_PIXEL_C1`: per-tap bounds and border handling |
| `modules/imgproc/src/warp_common.vector.hpp` | 6-33 | `SHUFFLE_ALLWITHIN`: `src + addr[i]` (only when every lane's inner mask passes) |
| | 176-189, 252-273 | `FETCH_PIXEL_C1`, `SHUFFLE_NOTALLWITHIN`: outer mask, constant-border store and `continue` |
| `modules/core/include/opencv2/core/fast_math.hpp` | 330-355 | `cvRound(float)`: x86 `_mm_cvtss_si32` |
| | 364-385 | `cvFloor(float)`: **`CV_DISABLE_UBSAN`**, `(int)__builtin_floorf` |
| `modules/imgproc/CMakeLists.txt` | 14 | `ocv_add_dispatched_file(warp_kernels SSE2 SSE4_1 AVX2 NEON NEON_FP16 RVV LASX)` — no AVX-512 variant |

The Task 021 line `warp_kernels.simd.hpp:3764` was re-verified: it is the
`CV_WARP_SCALAR_SHUFFLE(LINEAR, C1, 32F);` statement in the scalar tail of
`warpAffineLinearInvoker_32FC1`. UBSan attributes the expanded macro to that line; the first
undefined pointer operation inside it is the `CV_WARP_SCALAR_SHUFFLE_DEF_C1` declaration
`const float *srcptr = src + srcstep * iy + ix;` (`warp_common.scalar.hpp:12`). The nearest
twin is `warp_kernels.simd.hpp:439` (`CV_WARP_SCALAR_SHUFFLE(NEAREST, C1, 8U);`).


## 3. CPU dispatch actually executed

Instrumented install `~/.cache/video-task017/install-5.0.0-san` (CMake `RelWithDebInfo`,
`CPU_BASELINE=SSE3`, `CPU_DISPATCH=SSE4_1;SSE4_2;AVX;FP16;AVX2;AVX512_SKX`, `WITH_IPP=OFF`,
`WITH_OPENCL=OFF`, `BUILD_LIST=core,imgproc,video`, pthreads parallel framework). The generated
`warp_kernels.simd_declarations.hpp` lists `CV_CPU_DISPATCH_MODES_ALL AVX2, SSE4_1, BASELINE`:
**no AVX-512 variant of the warp kernels exists** even though the host supports AVX-512, because
`imgproc/CMakeLists.txt:14` does not request one. `nm` confirms `cv::opt_AVX2::`,
`cv::opt_SSE4_1::` and `cv::cpu_baseline::` symbols for both kernels.

Host: AMD Ryzen 9 7945HX (Zen 4; AVX2, FMA, AVX-512F/BW/DQ/VL), 32 logical CPUs, Debian, g++
14.2.0, gdb 16.3.

| Configuration | Kernel executed (gdb breakpoint hit) |
|---|---|
| Default (`useOptimized=1`) linear | `cv::opt_AVX2::warpAffineLinearInvoker_32FC1` |
| Default nearest | `cv::opt_AVX2::warpAffineNearestInvoker_8UC1` |
| `cv::setUseOptimized(false)` linear | `cv::cpu_baseline::warpAffineLinearInvoker_32FC1` |
| `cv::setUseOptimized(false)` nearest | `cv::cpu_baseline::warpAffineNearestInvoker_8UC1` |

The legacy `cv::WarpAffineInvoker` was never hit, matching Task 021. Disassembly of the AVX2
worker lambdas (`_M_invoke`) shows the conversion instructions actually used: linear
`vroundps` (floor) + `vcvttps2dq` for the vector body; nearest `vcvtps2dq` (round to nearest)
for the vector body and `vcvtsd2si`/scalar conversions in the tail; both use `vfmadd132ps` for
the coordinate and `vpmulld`/`vpaddd` for the vector address `srcstep*iy + ix`.

**Vector lane count (confirmed from the loaded library, not assumed).** The AVX2 linear
worker operates on `%ymm` registers (e.g. `6a746fa: vroundps $0x1,%ymm1,%ymm0` then
`vcvttps2dq %ymm0,%ymm0`), i.e. 256-bit = 8 float lanes; `vroundps` immediate `0x1` is
round-toward-negative-infinity, i.e. `v_floor`. The unroll factor is therefore
`uf = vlanes_32 * 2 = 16` destination pixels per vector iteration
(`warp_kernels.simd.hpp:3713-3715`). The cpu_baseline worker uses 128-bit `cvtps2dq` /
`cvtdq2ps` (SSE2; `CV_SIMD128=1`, baseline `CV_SIMD_WIDTH=16` → 4 lanes, `uf = 8`), so the
baseline path **also** has a vector body plus a scalar tail; "baseline" is not "scalar only".

For width `W` the AVX2 vector body covers `16*floor(W/16)` columns and the scalar tail the
remaining `W mod 16`. Widths 32, 64, 96, 128 and 16384 have **no** AVX2 scalar tail; 33, 63, 97,
129, 16383 and 65535 do. **gdb confirms this inside the running kernel**, reached through the
real `opencv_video_estimate_ecc_translation` export at 97x64: a breakpoint at
`warp_kernels.simd.hpp:3764` stops in `cv::opt_AVX2::warpAffineLinearInvoker_32FC1` with
`vlanes_32 = 8`, `max_vlanes_32 = 8`, `uf = 16`, `srccols = 97`, `srcstep = 97` and **`x = 96`**:
the single (`97 mod 16 = 1`) scalar-tail column.


## 4. Float32 conversion semantics (established independently of Task 021)

Task 021's `cvRound(double)` window (ties-to-even around `2^31 - 0.5`) does **not** carry over:
the 5.0 kernels convert **Float32** coordinates. Float32 spacing at `2^31` is exactly `128`, and
both `+2^31` (`0x4f000000`) and `-2^31` (`0xcf000000`) are representable. No Float32 value lies
in `(2^31 - 128, 2^31)`, so there is no half-ulp tie at the boundary, and floor and
round-to-nearest-even have the **same** in-range set:

> A Float32 coordinate `v` converts to a valid int32 iff `isfinite(v)` and `-2^31 <= v < 2^31`.
> The largest in-range positive value is `2147483520` (`0x4effffff`).

| Operation | Source | x86 instruction (observed) | Out-of-range / NaN / Inf result |
|---|---|---|---|
| Scalar `cvRound(float)` (nearest tail) | `fast_math.hpp:338-340` | `cvtss2si` | `INT_MIN` (integer indefinite) |
| Scalar `cvFloor(float)` (linear tail) | `fast_math.hpp:364-369`, **`CV_DISABLE_UBSAN`** | `(int)__builtin_floorf` → truncating convert | `INT_MIN`; the C++ cast is UB for out-of-range input and UBSan is suppressed on this function |
| Vector `v_floor` (AVX2 linear) | `warp_kernels.simd.hpp:41-42` | `vroundps $1` + `vcvttps2dq` | `INT_MIN` per lane |
| Vector `v_round` (AVX2 nearest) | `warp_kernels.simd.hpp:48-49` | `vcvtps2dq` | `INT_MIN` per lane |

The probe's `conversion-check` case verifies the model through the **defined** intrinsics
`_mm_cvtss_si32` and `_mm_cvttss_si32(floorf(v))` for 17 probes (0, ±100.75, `2^20`, `2^24`,
`2^31-128`, `-2^31`, `2^31`, `2^31+256`, `-(2^31+256)`, `2^32`, `3e9`, `1e12`, `FLT_MAX`, ±Inf,
NaN): **17/17 MATCH**, `ROUNDING_MODE FE_TONEAREST`, `MXCSR_RC 0`. It also records that the
decimal literal `2.1474836e9f` is stored as `2147483648` (= `2^31`, **out of range**), so a
test author writing a "just below `2^31`" literal would silently test the out-of-range case;
every boundary in this audit is built with `nextafterf` instead.

**Classification.** An out-of-range Float32→int32 conversion is itself **undefined** in C++
(`[conv.fpint]`); in `cvFloor(float)` that UB is hidden by `CV_DISABLE_UBSAN`, and in the
intrinsic paths it is a defined instruction that returns the sentinel. Either way the first
**unsafe** operation is the conversion, and it is classified separately from the **next**
undefined operation, the pointer arithmetic in section 5. For NaN/Inf the same instructions
return the same sentinel, which is why the Task 021 NaN replay and a finite `3e9` replay hit
the identical site.

**Float overflow to infinity.** Finite coefficients can still produce an infinite
coordinate (e.g. `3e38*x + 3e38` overflows Float32 at `x >= 1`). This is IEEE-defined (no C++
UB) and then follows the nonfinite path. The `level-a-overflow-to-inf` case
(`[3e38 0 3e38; 0 1 0]`, 16383 wide) produced **0** UBSan reports: only `sx` is out of range, and
the resulting pointer did not wrap the address space (section 5, "Why some cases are silent").


## 5. Pointer arithmetic: formation before the guard

The scalar shuffle (`warp_common.scalar.hpp:136-152`) runs, in order:

1. `CV_WARP_SCALAR_*_COMPUTE_COORD` — `ix, iy = cvFloor/cvRound(sx, sy)` (`:131-134`);
2. `CV_WARP_SCALAR_SHUFFLE_DEF_C1` — **`const T *srcptr = src + srcstep * iy + ix;`** (`:12`);
3. the bounds test `(unsigned)ix < (unsigned)(srccols-1) && (unsigned)iy < (unsigned)(srcrows-1)` (`:139-140`);
4. for out-of-image taps, the outer test and `BORDER_CONSTANT` store + `continue` (`:143-149`).

Step 2 executes **unconditionally, before** step 3. `srcstep` is `size_t`, `iy`/`ix` are
`int`. With the `INT_MIN` sentinel, `srcstep * iy` converts `iy` to `size_t` (`2^64 - 2^31`)
and the product wraps modulo `2^64`; adding the result to `src` is a pointer offset of roughly
`-srcstep * 2^31` elements, far outside the source allocation. **C++ `[expr.add]` makes forming
that pointer undefined even if it is never dereferenced.** The later bounds test (step 3)
rejects the tap correctly, so no out-of-bounds load occurs, but a successful guard does not
retroactively make the earlier pointer expression defined.

### The six distinctions required by the task

| # | Category | Finding for finite `|coord| >= 2^31` (both kernels, scalar tail) |
|---|---|---|
| 1 | Valid pointer, valid dereference | Only for in-range taps. |
| 2 | **Invalid pointer formed, not dereferenced** | **YES — the confirmed UB.** `srcptr` is formed out of range, then the guard sends the pixel to `BORDER_CONSTANT`. |
| 3 | Actual out-of-bounds load or store | **NOT observed.** ASan reported 0 errors in every case; the guard holds. |
| 4 | Arithmetic overflow in address calculation | The `size_t` multiply wraps (defined, unsigned); the **pointer** addition is the UB. No signed `int` overflow in the float path itself. |
| 5 | Correct border handling despite a wrong integer coordinate | **YES.** `INT_MIN` as `unsigned` is `2^31`, which fails `< srccols-1` and the outer `ix+1 >= srccols+1` test, so the pixel is written as the border value. `dst_sum = 0` was observed for every out-of-range replay. |
| 6 | Silent data corruption without a sanitizer diagnostic | **Not observed for the sentinel** (it is always rejected). See section 6 for the in-range wrong-pixel question. |

### Why some cases are silent under UBSan

GCC's `-fsanitize=pointer-overflow` reports only when the computed address **wraps** the address
space; it cannot see an out-of-object pointer that stays inside `[0, 2^64)`. The byte offset is
`(srcstep*iy + ix) * sizeof(T) mod 2^64`. With the probe's measured base `0x7f8ca9058840`:

| Case | Byte offset | Result | UBSan |
|---|---|---|---|
| Linear 16383 wide, `ix = iy = INT_MIN` | `-2^47` B (-131072 GiB) | `0xffffff8c...` (wraps) | **reported** |
| Linear 97 wide, both `INT_MIN` | -784 GiB | `0x00007ec8...` (no wrap) | silent |
| Linear 16383 wide, only `ix = INT_MIN` | -8 GiB | `0x00007f8a...` (no wrap) | silent |
| Nearest 65535 wide, both `INT_MIN` | `-2^47` B | wraps | **reported** |
| Nearest 16383 wide, both `INT_MIN` | -32768 GiB | `0x00005f8c...` (no wrap) | silent |

So the UBSan report needs a large `srcstep * sizeof(T)`: about 16k Float32 columns for linear,
about 64k UInt8 columns for nearest. **Every one of the silent rows forms the same undefined
out-of-object pointer**; they are silent only because of the sanitizer's detection limit. The
probe's `COORD_REACH` classification (`OUT_OF_RANGE_FINITE`) flags them independently of UBSan,
and the documented dispositions are based on that source-level fact, not on report counts.

**A second, independent condition: the scalar tail must run.** Once the source is wide enough
for the offset to wrap, the report depends **only** on whether the row has an AVX2 scalar tail.
One width per process, linear, `t = (3e9, 3e9)`:

| Width | `W mod 16` | UBSan reports |
|---|---|---|
| 16383, 16385, 16401, 32769, 65521, 65535 | 15, 1, 1, 1, 1, 15 | **1 each** |
| 16384, 16400, 32768, 65520, 65536 | 0 | **0 each** |

Every multiple of 16 is clean and every non-multiple reports, at every wide width tested. That
is the source structure directly: a multiple-of-16 row is covered entirely by the vector body,
which never forms `src + srcstep*iy + ix` for a sentinel lane (below). It **rules out** a
base-address or power-of-two artifact. (An earlier draft of this note attributed the clean
16384/65536 runs to the base address; that explanation was **wrong** and is withdrawn — the
measured offsets for those widths do wrap, and the clean result comes from the absent tail.)

### SIMD body

The vector body (`warp_kernels.simd.hpp:3744-3757`, `:421-433`) computes all `uf` coordinates,
converts them, builds `inner_mask`, and stores `addr = v_fma(v_srcstep, src_iy, src_ix)` **as
int32** (`warp_kernels.simd.hpp:26-28`). It then:

* forms `src + addr[i]` (`warp_common.vector.hpp:8, 28`) **only** in `SHUFFLE_ALLWITHIN`, i.e. only
  when every lane passed the inner mask, so every `addr[i]` is an in-image offset;
* otherwise (`SHUFFLE_NOTALLWITHIN`, `:252-273`), with `BORDER_CONSTANT`, stores the border vector
  and `continue`s when no lane passes the outer mask, and for mixed blocks fetches each lane
  through `CV_WARP_VECTOR_FETCH_PIXEL_C1`, which tests `ix`/`iy` **before** indexing `src[addr_i]`
  (`:176-179`).

So the AVX2 vector body does **not** form an out-of-object `src` pointer for the sentinel. Its
int32 `addr = srcstep*iy + ix` does overflow for the sentinel, but that is a **wrapping SIMD
integer** operation (`vpmulld`/`vpaddd`), not C++ signed arithmetic, and its value is used only
after the mask proves the lane in range. The confirmed UB is confined to the **scalar tail** in
this build. Sentinel lanes in the vector body are masked to the border, which is why
`dst_sum = 0` is identical in both bodies.

This asymmetry is why the width matters (table above): multiple-of-16 widths have no AVX2
scalar tail, so the sentinel never reaches the unconditional pointer formation. **A clean
optimized run on a multiple-of-16 width therefore says nothing about the scalar tail**, and
ECC's own geometries are arbitrary (the binding accepts any width 32..16384).

### Mixed and partial out-of-range coordinates

The same reasoning holds when only `sx` **or** only `sy` is out of range (one sentinel), and for
negative coordinates `<= -2^31 - 128` (the sentinel is the same `INT_MIN`). It holds for affine
matrices where a large linear coefficient, not the translation, pushes a far pixel past `2^31`,
because the coordinate expression is identical.

## 6. Level A — direct native warp replay (instrumented OpenCV 5.0.0)

Every case runs in its own process via `scripts/research_ecc5_float_warp.sh`, BORDER_CONSTANT,
WARP_INVERSE_MAP, 64 rows. "Reports" are UBSan `runtime error` lines from inside
`warp_kernels.simd.hpp`. ASan reported **0** errors and LSan **0** leaks in every case.

### Finite, model-valid (the new evidence)

| Case | Matrix (exact Float32) | Model-valid / seed domain | Width | Kernel (gdb/stack) | Reports | Site |
|---|---|---|---|---|---|---|
| `finite-translation-linear` | `[1 0 3e9; 0 1 3e9]` (`0x4f32d05e`) | translation YES / no seeded op | 16383 | `opt_AVX2::…Linear…32FC1` | **1** | `:3764:17` pointer index expression overflowed |
| `finite-translation-nearest` | same | same | 65535 | `opt_AVX2::…Nearest…8UC1` | **1** | `:439:17` same |
| `finite-euclidean` | θ=-0.5, `t=(3e9, -3e9)` | Euclidean YES / OUTSIDE seed | 16383 | `opt_AVX2::…Linear…32FC1` | **1** | `:3764:17` |
| `finite-affine` | `[1.5 .25 3e9; -.25 -1.5 1e12]` (scale, shear, reflection) | affine YES / OUTSIDE seed | 16383 | `opt_AVX2::…Linear…32FC1` | **1** | `:3764:17` |
| `scalar-finite-linear` | translation `3e9` | YES | 16383 | `cpu_baseline::…Linear…32FC1` | **1** | `:3764:17` |
| `scalar-finite-nearest` | translation `3e9` | YES | 65535 | `cpu_baseline::…Nearest…8UC1` | **1** | `:439:17` |
| `roi-linear` | translation `3e9`, **noncontiguous** source (step 65596 vs 65532 B) | YES | 16383 | `opt_AVX2::…Linear…32FC1` | **1** | `:3764:17` |
| `boundary-above` | `tx = 2^31` exactly (`0x4f000000`), `ty = 0` | YES | 16383 | AVX2 linear | 0 | **silent**: only `ix` is the sentinel; offset -8 GiB does not wrap (section 5). Out of range per `COORD_REACH`. |
| `overflow-to-inf` | `[3e38 0 3e38; 0 1 0]` | finite coefficients | 16383 | AVX2 linear | 0 | **silent**: `sx = +Inf` (IEEE overflow), `sy` in range; same non-wrapping offset |

Linear and nearest are **separate findings** (different kernels, line 3764 vs 439, different
UInt8/Float32 strides), each demonstrated independently.

### Nonfinite (Task 021 regression and extension; not model-valid)

| Case | Matrix | Reports | Site |
|---|---|---|---|
| `nonfinite-translation` (Task 021 replay) | `[1 0 NaN; 0 1 NaN]` | **1** | `:3764:17` — reproduces Task 021 exactly |
| `inf-translation` | `[1 0 +Inf; 0 1 -Inf]` | **1** | `:3764:17` |
| `nonfinite-euclidean` | all six NaN | **1** | `:3764:17` |
| `nonfinite-affine-coeff` | `[1 NaN 0; 0 1 0]` | 0 | silent: `sx = x + y*NaN` is NaN on **every** row (IEEE `0*NaN = NaN`), so `ix` is the sentinel, but `sy = y` stays in range; a single-axis sentinel gives a non-wrapping offset (section 5). Out of range per `COORD_REACH` |

**The finite and nonfinite cases hit the identical first UB.** NaN, ±Inf and finite `3e9`
all convert to the same `INT_MIN` sentinel and reach the same pointer formation. Task 021's
NaN observation is therefore one instance of a general **out-of-int32-range coordinate**
hazard, not a NaN-specific one.

### Negative controls (all required to be clean; all clean)

| Case | Matrix | `COORD_REACH` | Reports |
|---|---|---|---|
| `conversion-check` | 17 probes | — | 0; `CONVERSION_MODEL_OK` |
| `negative` | small translation, θ=0.1 rigid, mild affine (97x64) | IN_RANGE | 0; `NEGATIVE_CONTROL_OK` |
| `far-control` | `t = (1e6, -1e6)` (inside 2^20 seed bound, far outside image) | IN_RANGE | 0 |
| `matrix-b` | Task 020 recorded affine intermediate (64x100) | IN_RANGE, max 2.33e7 | 0 |
| `boundary-below` | `tx = 2^31 - 128` (`0x4effffff`), 16383 wide | IN_RANGE | 0 |

`matrix-b` is the matrix that **overflowed the 4.x fixed-point path** (Task 020). It is clean on
the 5.0 float kernel: its largest mapped coordinate over the 64x100 grid (`23326306` ≈ `2.33e7`)
is about 92× below `2^31`. The 5.0 float kernel is therefore not vulnerable to the specific
matrices that broke 4.x; it is vulnerable to a different, much larger coordinate range.


## 7. Level B (native ECC intermediates) and Level C (real binding)

Level B uses `cv::findTransformECC` directly with the binding's arguments (no mask, Gaussian
5, `COUNT|EPS`, `eps = 1e-8`, 100 iterations) and iteration caps `k = 0..99`. Native ECC is
deterministic and its loop body does not depend on `maxCount`, so the warp left after a
`k`-capped call is exactly the matrix iteration `k+1` passes to the four `warpAffine` calls.
Each intermediate's coordinate reach over the full destination grid is classified with the
defined Float32 model. Level C calls the real C exports with validated inputs and real
Core-owned Mat handles (`opencv_core_mat_create_2d` + the Core bridge; no fabricated handles).

| Operation | Accepted caller input | Initial warp | Geometry | Level B: max finite coordinate reached / first out-of-range iterate | Level C: status, sanitizer |
|---|---|---|---|---|---|
| Translation identity | template `texture`, moving `texture(+2,+1)` | identity | 97x64 | **98.1** / none | status 0, `(2.012, 1.013)`; 0 UBSan, 0 ASan |
| Euclidean identity | moving `rigid_texture(6°, 2, 1)` | identity | 97x64 | **97.5** / none | status 0, angle 0.10474, `(1.997, 1.017)`; 0 / 0 |
| Euclidean seeded | same; seed θ=0.9, `t = (2^20, -2^20)` (in-domain extreme) | seed | 97x64 | **1048635.6** / none. Only the seed reached `warpAffine`: the first capped call (`k=1`) raised `StsNoConv` with the warp unchanged, so no optimizer-generated matrix followed | **status 3** "NaN encountered" (`ecc.cpp:582`); 0 / 0 |
| Affine identity | moving `affine_texture(1.02,.01,-.01,1.03,2,1)` | identity | 97x64 | **100.7** / none | status 0; 0 / 0 |
| Affine seeded, **Task 020 trial n435** | verbatim Task 020 seed `[0.45051536 0.286671013 23.7435341 -0.437181264 1.13443863 -37.4001312]`, template `texture(64,100)`, moving `texture(64,100,+2,+1)` | seed | 100x64 | **33644148** / none | **status 3** "NaN encountered"; 0 / 0 |

**n435 trajectory (Level B, exact):** iterate 1 is the seed (reach 86.4); iterate 2
`[-1.2539 -0.2141 161.02; 1.6503 2.7640 -22.761]` (reach 314.7); iterate 3
`[-8013.5786 11663.9355 408400.4375; -14059.3652 -522875.5625 688888.5625]` (reach
`3.36e7`), after which native ECC raised and presented no further matrix. Iterate 3 is the
same kind of large intermediate Task 020 recorded on 5.0.0. Its worst corner,
`(x=99, y=63)`, maps to `sy = -33644149`, **64× below `2^31`**. It executes on 5.0 without a
report because it is far inside the int32 range; on 4.x the analogous matrix overflowed the
fixed-point path.

**Bounded searches (opt-in, 150 trials each, seeds and geometry from a fixed LCG keyed by trial
index, 32..160 x 32..128, ten image kinds including constant, stripes, noise, low-variance and
an asymmetric parent crop):**

| Search | Unsafe intermediates | Max finite coordinate reached | Wall time | Peak RSS |
|---|---|---|---|---|
| `search-translation:0:150` | **0** | 227.9 (trial 41) | 88 s | 1.14 GB |
| `search-euclidean:0:150` | **0** | 1453.5 (trial 54) | 140 s | 2.02 GB |
| `search-euclidean-seeded:0:150` (incl. `|t|` near 2^20) | **0** | 1048729.8 (trial 69) | 52 s | 1.55 GB |
| `search-affine:0:150` | **0** | 141934.6 (trial 42) | 115 s | 1.07 GB |
| `search-affine-seeded:0:150` (incl. `|A| = 4`, `|t|` near 2^20) | **0** | 1048815.3 (trial 19) | 78 s | 1.05 GB |

The largest coordinate any ECC execution produced in this audit is `3.36e7` (n435) — about
**64× below** the `2^31` threshold. Seeded runs start near `2^20` because the seed itself is
that large; the optimizer did not grow them. **No Level B or Level C reproducer of the
finite-coordinate UB exists**, so every operation is `NOT_REPRODUCED` at Levels B and C.

That is an observation about these trajectories, not a bound. Native ECC throwing
(`StsNoConv`) does **not** make an operation safe: the throw is raised after that iteration's
four `warpAffine` calls (`ecc.cpp:523-526` precede `581-582`), so a hypothetical
out-of-range intermediate would already have executed before the exception (section 9).

## 8. Source-backed safety analysis (the ten required questions)

Answers apply to `warpAffineLinearInvoker_32FC1` (L) and `warpAffineNearestInvoker_8UC1` (N) on
the verified x86_64 AVX2 + cpu_baseline paths. Where L and N differ it is stated.

1. **Are mapped finite Float32 coordinates always finite?** **No.** `x*M[0] + y*M[1] + M[2]` can
   overflow Float32 to ±Inf for finite coefficients (`level-a-overflow-to-inf`). IEEE-defined,
   but the result then takes the nonfinite path. L and N identical.
2. **Can mapped coordinates exceed the int32 range?** **Yes**, whenever `|coordinate| >= 2^31`.
   Nothing in `warpAffine` clamps or rejects them. L and N identical.
3. **Can conversion produce a sentinel or otherwise invalid coordinate?** **Yes.**
   `cvFloor(float)` (L, `CV_DISABLE_UBSAN`) and `cvRound(float)` (N) return `INT_MIN` for every
   out-of-range or nonfinite value. The `(int)` cast inside `cvFloor(float)` is itself UB for
   those inputs, hidden from UBSan by the attribute.
4. **Can adding one to a converted coordinate overflow?** **No.** The scalar outer test
   `(unsigned)(ix+1)` (`warp_common.scalar.hpp:144-145`) adds in **signed `int` before** the cast,
   so this needs a bound, and one exists: the only values `ix`/`iy` can hold are a valid
   conversion of a Float32 in `[-2^31, 2^31 - 128]` or the `INT_MIN` sentinel. The maximum is
   `2147483520`, so `ix + 1 <= 2147483521 <= INT_MAX`; the minimum `INT_MIN + 1` is in range. The
   per-tap `ix + dx`/`iy + dy` (`dx, dy` in `{0, 1}`, `:61`) obey the same bound. The vector
   `v_add(src_ix0, one)` (`warp_common.vector.hpp:254-257`) is a wrapping SIMD add, not C++
   arithmetic. L and N identical. **This bound depends on Float32 spacing at `2^31`**: it would
   fail for a coordinate type that could represent `2^31 - 1` exactly.
5. **Can multiplying a coordinate by the source stride overflow the relevant integer type?**
   Scalar: `srcstep * iy` is `size_t * int`, so `iy` converts to `size_t` and the product is
   **unsigned** and wraps modulo `2^64` (defined). For an in-range `iy` and a valid image the
   product is bounded by the allocation; for the sentinel it wraps to a huge value. The
   **pointer** addition is the UB (question 6). Vector: `addr = v_fma(v_srcstep, src_iy, src_ix)`
   is **int32** (`warp_kernels.simd.hpp:26-28`; `v_srcstep = int(srcstep)`, `:3726`). For the
   sentinel it wraps (SIMD, defined), and it is used only behind the inner mask. For **in-range
   coordinates** inside a valid image it cannot overflow: `srcstep*iy + ix < srcrows*srcstep`,
   which the binding bounds at `2^24` pixels (`<= 2^26` bytes for Float32). Note
   `int(srcstep)` itself requires `srcstep <= INT_MAX`; the binding's 16384-column limit keeps it
   at `<= 65536` elements.
6. **Can a pointer outside the source allocation be formed before a guard?** **Yes — confirmed
   (Level A).** Scalar tail, both L and N: `src + srcstep*iy + ix` at `warp_common.scalar.hpp:12`
   precedes the guard at `:139`. The vector body forms `src + addr[i]` only inside
   `SHUFFLE_ALLWITHIN` (all lanes in range) and `src[addr_i]` only after the per-lane test, so it
   does **not** form one for the sentinel in this build.
7. **Does scalar border handling safely handle extreme coordinates?** **The memory access is
   safe; the expression is not.** For `INT_MIN`, `(unsigned)INT_MIN = 2^31` fails both the inner
   and outer tests, and `BORDER_CONSTANT` writes the border value and `continue`s, so no load
   happens (ASan clean). It cannot undo the pointer already formed in step 2.
8. **Are SIMD and scalar semantics equivalent for out-of-range inputs?** **Same output, different
   safety.** Both send the sentinel pixel to the border (`dst_sum` identical). Only the scalar
   tail forms the invalid pointer. Proven by the width table in section 5: multiples of 16
   (vector-only) report 0, others report 1.
9. **Can incorrect coordinate conversion cause an in-bounds but wrong pixel read?** **Not via the
   sentinel**: `INT_MIN` is never inside `[0, srccols)`, so it always routes to the border. A
   finite coordinate below `2^31` converts exactly (floor/round are exact on representable
   integers above `2^24`, where every Float32 is an integer). The residual precision issue is
   different and is **not** UB: above `2^24` Float32 cannot represent fractional or even every
   integer coordinate, so `sx - ix` loses its fraction and the sample position is coarsened.
   Those coordinates are already far outside any binding image (≤ 16384), so they read the
   border, not a wrong in-image pixel. No in-bounds wrong-pixel read was found or is derivable.
10. **Are different architecture/HAL paths subject to the same reasoning?** **Not established.**
    The scalar macro is shared source, so every build that executes the scalar tail **forms
    `src + srcstep*iy + ix` before the guard** — that part generalizes by source. But the value of
    `ix`/`iy` for out-of-range input is **target-specific**, and so is whether the guard then
    rejects it. x86 returns the `INT_MIN` sentinel (verified here). On ARMv8 OpenCV's NEON
    `v_floor` is `vcvtmq_s32_f32` (`intrin_neon.hpp:2273-2276`), i.e. `FCVTMS`, and the MSVC/ARM64
    scalar `cvFloor(float)` is `vcvtms_s32_f32` (`fast_math.hpp:370-371`). By the Arm architecture
    definition those **saturate** (`INT_MIN`/`INT_MAX`) and convert **NaN to 0**. That is
    architecture-reference knowledge, **not executed in this audit**. If it holds, a NaN coordinate
    on ARM would yield `ix = 0`, **inside** the image, so the guard would pass and a real pixel
    would be read instead of the border: a potential **silent wrong-pixel read** that does not
    exist on x86. It is recorded as an unverified hypothesis and a proof obligation (section 11).
    NEON, RVV, LASX, IPP, OpenCL and a custom `cv_hal_warpAffine` were **not executed**.

**Image-dimension bounds used.** Questions 4 and 5 rely on: binding image limits 32..16384 per
side, area `<= 2^24` (`cpp/opencv_video_shim.h`); Float32 spacing 128 at `2^31`. Neither bound
limits the **coordinate**: a 32x32 image with `tx = 3e9` is accepted geometry. The only thing
keeping ECC's coordinates small is optimizer behaviour, for which no source bound exists
(section 9).

## 9. Can ECC reach the unsafe range? (no source bound)

The finite-coordinate UB needs some mapped coordinate with `|v| >= 2^31` over the destination
grid. `update_warping_matrix_ECC` (`ecc.cpp:230`) **adds** each `deltaP` to the translation
column and (for affine) to every coefficient, without clamping. `deltaP` comes from
`hessianInv * (...)` scaled by `lambda` (`ecc.cpp:574-607`); neither is bounded in the source,
and nothing between iterations checks the warp's magnitude. So:

* **No source-level bound keeps ECC's coordinates below `2^31`.** The 100-iteration cap and the
  binding's `|t| <= 2^20` seed limit bound only the first iterate, not later ones.
* **Observed trajectories stay small.** The largest observed (n435, `3.36e7`) is ~64× short,
  and it arose in a step after which ECC immediately raised.
* **The exception comes too late to protect the warp.** Each iteration runs the four
  `warpAffine` calls (`:523-526`) **before** the `rho` NaN check (`:581-582`) and the
  `lambda_d <= 0` check (`:593-595`). An iterate past `2^31` would execute the unsafe scalar
  tail on that very iteration, before any error is raised. A thrown `StsNoConv` therefore
  cannot qualify an operation as safe.

Reaching `2^31` through ECC is **not shown possible and not shown impossible** — exactly
`SAFETY_GATE_NOT_ESTABLISHED`.

## 10. Coverage: what is and is not qualified

| Path | Executed? | Result |
|---|---|---|
| x86_64 `opt_AVX2` linear 32FC1 + nearest 8UC1, vector body and scalar tail | **Yes** (gdb) | Finite UB in scalar tail; vector body masks the sentinel |
| x86_64 `cpu_baseline` (SSE2 `CV_SIMD128`) linear + nearest | **Yes** (gdb, `setUseOptimized(false)`) | Same finite UB in scalar tail |
| `opt_SSE4_1` variants | Symbols present, **not executed** (AVX2 preferred on this host) | Unqualified |
| AVX-512 | **No warp-kernel variant is built** (`CMakeLists.txt:14`) | n/a in this configuration |
| NEON / NEON_FP16, AArch64, RVV, LASX | **No** | Unqualified; ARM NaN→0 hypothesis (Q10) open |
| IPP (`WITH_IPP=OFF`), OpenCL (`WITH_OPENCL=OFF`; only for `UMat` output anyway) | **No** | Unqualified |
| Custom `cv_hal_warpAffine` (`imgwarp.cpp:2243`) | **No** | Unqualified |
| Windows / macOS / MSVC toolchains | **No** | Unqualified |
| Other `BORDER_*` modes (`borderInterpolate_fast` branch) | **No** (ECC uses `BORDER_CONSTANT`) | Unqualified |
| OpenCV 4.1 / 4.6 / 4.10 | They have no float kernels; Tasks 020/021 cover their fixed-point path | Unchanged |

**Resource budget.** Level A replays use at most a 65535x64 Float32 source (~16 MiB) or UInt8
mask (~4 MiB) per process, never a full-size image; extreme coordinates come from the matrix,
not the geometry. The 28-case default runner finished in about one minute; the largest
per-case peak RSS was 0.72 GB (ASan shadow memory included). Each 150-trial search took 52–140 s at 1.05–2.02 GB peak RSS under ASan. Probe guards:
sweep width 1..65536, search count 1..2000.

## 11. Mitigation research (proposals only; nothing applied)

No OpenCV patch was applied to anything the binding uses, nothing was vendored, and no
production workaround was made. Because finite-coordinate UB **is** confirmed, production work
stops here. The candidate fixes, for owner review:

1. **Delay pointer formation until after the guard.** Move the `srcptr` initializer of
   `CV_WARP_SCALAR_SHUFFLE_DEF_C*` into the in-range branch of `CV_WARP_SCALAR_SHUFFLE` and
   form `src + dy*srcstep + dx` there. This mirrors the vector body, which already forms
   `src + addr[i]` only behind the mask. It is the narrowest fix: it removes the pointer UB
   without changing any output, since every sentinel tap already takes the border branch.
2. **Representability check before conversion.** Test `isfinite(v) && -2^31 <= v < 2^31` on the
   Float32 coordinate and route failures straight to the border. This also removes the cast UB
   hidden by `CV_DISABLE_UBSAN` in `cvFloor(float)`, and makes behaviour identical across targets,
   closing the ARM NaN→0 hypothesis.
3. **Range-safe address arithmetic.** Compute the offset in `ptrdiff_t` only for coordinates
   already proven in range.

Option 1 alone fixes the pointer formation; option 2 is also needed to remove the conversion UB
and the cross-architecture divergence. **None was implemented or validated**, not even on a
disposable copy: a credible validation needs the full oracle/AUnit campaign on a patched build
and a non-x86 target, which is beyond this audit. They remain unvalidated proposals. No upstream
issue or PR was filed.

## 12. Unresolved proof obligations

1. A source-level bound on ECC's accumulated translation and coefficients (or a proof that none
   exists) for each of the three models across the published envelope.
2. Execution of the NEON/AArch64 paths, to test the NaN→0 / saturation hypothesis (Q10) and
   whether it produces in-image wrong-pixel reads.
3. Execution of the `opt_SSE4_1`, RVV and LASX variants, IPP and a custom HAL.
4. Audit of the non-constant `borderInterpolate_fast` branch for the sentinel (not used by ECC,
   but compiled into the same kernels).
5. Validation of a mitigation (section 11) against the full regression and oracle campaign.
6. Windows/macOS toolchain behaviour of `cvFloor(float)` / `cvRound(float)` for out-of-range input.

## 13. Implications for the existing ECC APIs

* **Consumer guidance changes.** Before this task the 5.0 float kernel was known unsafe only for
  **nonfinite** warps, which the binding never accepts as seeds. It is now known unsafe for any
  **finite** warp mapping a pixel to `|coordinate| >= 2^31`, including **model-valid**
  translation, Euclidean and affine matrices. On OpenCV 5.0 every ECC model therefore depends on
  the optimizer never producing such an iterate, and the source does not bound that.
* **Seeds are not the exposure.** The binding limits translation seeds to `2^20` and affine
  linear seeds to `|A| <= 4`; over a binding-sized image (coordinates at most 16383) an in-domain
  affine seed maps a pixel to at most `4*16383 + 4*16383 + 2^20 = 1179640` (Euclidean:
  `1081342`), about **1820×** below `2^31`. The first iterate is therefore always safe on the
  float kernel. The exposure is **later iterates**.
* No API is disabled, no gate is added, and no contract or seed domain changes. Dated notices are
  added to the README and the Ada API comments; the Task 020/021 notices are preserved verbatim.

## 14. Reproduction

```sh
# Instrumented OpenCV 5.0.0 (its own libraries built with ASan/UBSan/float-cast-overflow);
# CMake configuration in ~/.cache/video-task017/build-one.sh (as in Tasks 017/020/021).
OPENCV_PREFIX=$HOME/.cache/video-task017/install-5.0.0-san \
OPENCV_CORE_SOURCE=$PWD/alire/cache/pins/opencv_core_4da9d35e \
  sh scripts/research_ecc5_float_warp.sh            # 28 default cases, one process each

# Opt-in: per-width sweep (vector body vs scalar tail) and bounded searches
sh scripts/research_ecc5_float_warp.sh level-a-sweep-linear:16384 level-a-sweep-linear:16385
sh scripts/research_ecc5_float_warp.sh search-affine-seeded:0:150
```

Expected: runner exit 0. The runner refuses to run against a non-instrumented OpenCV (no
`__ubsan_handle_pointer_overflow` import in `libopencv_imgproc`). Per-case verdicts are in the
qualification report.
