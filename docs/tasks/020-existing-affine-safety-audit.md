# Task 020 — Existing ECC affine safety audit and mitigation assessment

Branch `safety/020-existing-affine-ub`, from verified `origin/main`
`b09a65a78aa39f16a6d358f81517d3b8186eb334` (the PR #19 merge). Crate version `0.1.0-dev`, Core
pin `4da9d35ea21e1b2efe96296243ea668b488c6326`, 25 production exports and 198 AUnit
registrations are all **unchanged**. No dependency or CI topology changed. No production
behavior changed: the only edits to `src/` and `README.md` are comments and documentation.

**Disposition: `KNOWN_NATIVE_UB_UNMITIGATED`.** The unsafe native behavior is confirmed and
reachable through the existing seeded export. No qualified mitigation exists in the binding;
the one credible mitigation needs a patched OpenCV. Hosted CI status is reported separately in
[the qualification report](../task020-qualification.md).

The Task 019 record ([ecc-affine-masked-source-contract.md](../ecc-affine-masked-source-contract.md))
is preserved unchanged and is referenced here, not rewritten.

## 1. Original problem

On every iteration, native `cv::findTransformECC(MOTION_AFFINE)` resamples the smoothed moving
image (`CV_32FC1`, `INTER_LINEAR`), both gradients (`CV_32FC1`, `INTER_LINEAR`) and the support
mask (`CV_8UC1`, `INTER_NEAREST`). It uses `warpAffine(..., WARP_INVERSE_MAP)` with the current
`CV_32F` 2x3 warp (4.1.0 `ecc.cpp:506-509`; 4.6.0/4.10.0 `523-526`; 5.0.0 `523-526`). OpenCV
4.x evaluates the inverse map in 10-bit fixed point (`AB_SCALE = 1024`):

```cpp
adelta[x] = saturate_cast<int>(M[0]*x*AB_SCALE);                   // per column
X0 = saturate_cast<int>((M[1]*y + M[2])*AB_SCALE) + round_delta;   // per row
int X = (X0 + adelta[x]) >> ...;                                   // signed int addition
```

`saturate_cast<int>(double)` is `cvRound`, which on x86-64 GCC is `_mm_cvtsd_si32`. Each
conversion is bounded, but the following `int` addition is not. Two large operands overflow it,
and that overflow is undefined behavior in C++. `round_delta` is 16 for linear and 512 for
nearest. Task 019 recorded two such matrices from its ECC sweeps:

| Matrix | Source run | Exact `CV_32F` coefficients |
|---|---|---|
| A | masked, 97x64 | `22298.8203125, -57255.54296875, 225348.015625, 137349.0625, -361430.03125, -3676567.25` |
| B | unmasked **seeded**, 100x64 | `-5597.05908203125, 8140.32177734375, 285284.03125, -9895.33203125, -362406, 484909.9375` |

Matrix A is the masked-sweep matrix. The Task 019 masked API was never published, so A
implicates the existing APIs only at Level A (shared native code).

## 2. Reproduction and provenance

Research-only, opt-in tooling (never in `scripts/test.sh`, `scripts/run_sanitizers.sh` or any
workflow; nothing enters a production archive):

* `tests/cpp/ecc_affine_overflow_probe.cpp`: one case per process, because UBSan reports a
  source location only once per process.
* `scripts/research_ecc_affine_overflow.sh`: builds the probe with
  `-fsanitize=address,undefined,float-cast-overflow` against a caller-named OpenCV install,
  records `ldd`, runs each case, and classifies its log.

```sh
OPENCV_PREFIX=$HOME/.cache/video-task017/install-4.1.0-san \
OPENCV_CORE_SOURCE=$PWD/alire/cache/pins/opencv_core_4da9d35e \
  sh scripts/research_ecc_affine_overflow.sh            # every default case
sh scripts/research_ecc_affine_overflow.sh level-c-seeded-n103   # one case
sh scripts/research_ecc_affine_overflow.sh search-seeded:0:100   # bounded search
```

Per case it prints `EXPECTED_UPSTREAM_UB_OBSERVED` (signed-integer-overflow reported inside
`modules/imgproc/src/imgwarp*`), `NO_UPSTREAM_REPORT`, or `RESEARCH_PROBE_FAILED`. The last
means non-zero exit, any ASan/LSan error, or any other UBSan report. Only
`RESEARCH_PROBE_FAILED` fails the runner. An expected upstream report is evidence, never a
pass, and never appears in a sanitizer-clean claim. `NO_UPSTREAM_REPORT` is **not** evidence
of defined arithmetic.

**Instrumented OpenCV.** These are the existing Task 017 builds in
`~/.cache/video-task017/install-<v>-san` for 4.1.0, 4.6.0, 4.10.0 and 5.0.0.
`build-one.sh` configured them with CMake `RelWithDebInfo`, `BUILD_LIST=core,imgproc,video` and
no IPP, OpenCL or ITT. Compiler flags were
`CMAKE_CXX_FLAGS=-include cstdint -fsanitize=address,undefined,float-cast-overflow
-fno-omit-frame-pointer`, with the sanitizer link flags, `CPU_BASELINE=SSE3` and
`CPU_DISPATCH=SSE4_1;SSE4_2;AVX;FP16;AVX2;AVX512_SKX`. Each library exports
`__ubsan_handle_add_overflow` and `__ubsan_handle_float_cast_overflow`. The compiler was Debian
`g++` 14.2.0, and the host an AMD Ryzen 9 7945HX (x86_64, AVX2/SSE4.1 reported by
`cv::checkHardwareSupport`, `useOptimized=1`).

**Cases and fixtures.**

| Case | Inputs | Purpose |
|---|---|---|
| `negative` | `[1.03 0.02 2; -0.01 0.98 1]`, 97x64 | Negative control: zero int32-range overflows required (fails the probe otherwise) |
| `level-a-matrix-a` / `-b` | Exact Matrix A at 97x64 / Matrix B at 100x64 | Level A direct native replay, ECC call shapes (`CV_32FC1` linear, `CV_8UC1` nearest) |
| `trace-seeded-n103` | Seed `[0.580653667 -0.412717313 -4.85272264 -0.241246387 0.386808515 -10.5655231]`, 97x64 | Level B iteration provenance |
| `trace-seeded-n435` | Seed `[0.45051536 0.286671013 23.7435341 -0.437181264 1.13443863 -37.4001312]`, 100x64 | Level B (regenerates Matrix B) |
| `trace-identity-97` / `-100` | Identity, 97x64 / 100x64 | Level B for the identity API |
| `level-c-seeded-n103` / `-n435` | Same seeds through `opencv_video_estimate_ecc_affine_seeded` | Level C |
| `level-c-identity-97` / `-100` | `opencv_video_estimate_ecc_affine` | Level C for identity |
| `search-identity:F:N` / `search-seeded:F:N` | Seeded LCG trials, geometry 32-160 x 32-128, five image kinds | Bounded search (opt-in only) |

All ECC cases use the binding's call: `MOTION_AFFINE`, `COUNT|EPS`, 100 iterations, eps `1e-8`,
no mask, Gaussian 5. The template is `ecc_fixture::texture(rows, cols)`, copied verbatim into the
probe; the moving image is `texture(rows, cols, 2, 1)`.

**Seed provenance.** Both seeds are the exact stored Task 019 sweep values. That sweep used the
seed-19 LCG in `/tmp/video-task019-source/sweep.cpp`, unmasked trials `n = 103` (logged as
unsafe on 4.1.0 at 97x64) and `n = 435` (logged on 5.0.0 at 100x64). Every coefficient is
inside the published seed domain (|linear| <= 4, |translation| <= 2**20), so both are validated
caller inputs. No input was fabricated: the Matrix B match below was found by replaying stored
values, not by fitting.

**Level B method.** Native ECC is deterministic, and its loop body does not depend on
`maxCount`. A `findTransformECC` call capped at `k` iterations therefore leaves in the caller's
warp exactly the matrix that iteration `k + 1` of a longer run passes to `warpAffine`. Native
ECC updates that warp in place, so it is observable even when a later iteration throws. This
uses OpenCV's own optimizer; ECC is not reimplemented.

## 3. Existing API exposure

Both Ada overloads of `OpenCV.Video.Estimate_ECC_Affine` call the C exports
`opencv_video_estimate_ecc_affine` (identity) and `opencv_video_estimate_ecc_affine_seeded`.
Both exports reach the shared `ecc_affine_entry` in `cpp/opencv_video_shim.cpp`.

| Stage | Where | Before native ECC? | Relevant to the overflow? |
|---|---|---|---|
| 1. Ada preflight (`Validate_ECC_Arguments`, seed checks) | `src/opencv-video.adb` | yes | Images, options, seed domain only |
| 2. Core Mat handle acquisition | `OpenCV.Core.Module_Interop` callbacks | yes | No |
| 3. C boundary validation (`validate_ecc_inputs`, seed finiteness/magnitude) | shim | yes | Constrains the **first** iterate only |
| 4. Private warp initialization (identity or seed, storage order) | shim | yes | No |
| 5. Native ECC iterations | `findTransformECC` | inside | Unbounded Gauss-Newton updates `deltaP = H^-1 * e` |
| 6. Internal warp updates (`update_warping_matrix_ECC`, in place) | native | inside | Produce the unsafe intermediate |
| 7. `warpAffine` x4 per iteration | native imgproc | inside | **Executes the signed overflow** |
| 8. Exception containment | shim `catch` blocks | after | The overflow raises nothing; usually a later `StsNoConv` follows |
| 9. Final schema/finiteness validation, publication | shim | after | Sees only the last warp; cannot see or undo step 7 |

Every safeguard that runs before native code constrains the inputs, and every safeguard that
runs after it inspects the final output. Nothing runs between optimizer iterations.

**Evidence achieved per API:**

| API | Level A | Level B | Level C |
|---|---|---|---|
| Seeded (`Estimate_ECC_Affine` with `Initial_Transform`; `opencv_video_estimate_ecc_affine_seeded`) | **Demonstrated** (shared native path; Matrix B came from a seeded run) | **Demonstrated** (n103: iteration 3 on 4.1.0; n435: iteration 3 on 5.0.0 regenerates Matrix B bit-exactly) | **Demonstrated on 4.1.0** (n103). Not reproduced with these two seeds on 4.6.0, 4.10.0 or 5.0.0; the trajectory is version-sensitive |
| Identity (`Estimate_ECC_Affine`; `opencv_video_estimate_ecc_affine`) | Applies (same native `warpAffine` code) | `NOT_REPRODUCED` | `NOT_REPRODUCED` |

The identity API is **not** claimed affected merely because it shares native code. It is also
**not** claimed safe. Its first iterate is the identity, but the optimizer is the same and its
updates are just as unbounded. No argument excludes an unsafe later iterate, and the bounded
searches found none.

The Level C n103 stack on instrumented 4.1.0 is
`level_c_seeded -> opencv_video_estimate_ecc_affine_seeded (shim:1171) -> ecc_affine_entry
(shim:926) -> cv::findTransformECC (ecc.cpp:506) -> cv::warpAffine -> hal::warpAffine ->
WarpAffineInvoker::operator() (imgwarp.cpp:2285)`. The binding then returned status 3
(`OPENCV_VIDEO_ERROR_OPENCV`, "NaN encountered") and left the result record untouched. To the
caller this is an ordinary nonconvergence error with no visible sign of the undefined operation.

## 4. Native arithmetic findings

Operands below come from the probe's int64 model (`fixed_point_overflows`). It computes
`int64(X0) + int64(adelta)` and so never invokes signed overflow itself. UBSan reports the same
operands from inside OpenCV.

**Matrix A, 97x64, row 4 column 96 (Task 019 reference pixel), linear.** Pre-conversion X is
`(-57255.54296875*4 + 225348.015625)*1024 = -3762336`, so `X0 = -3762320` and
`adelta[96] = saturate(22298.8203125*96*1024) = -2147483648`. Their int64 sum is `-2151245968`.
Pre-conversion Y is `-5245222272`, so `Y0 = -2147483632`, `bdelta[96] = -2147483648`, and the
int64 sum is `-4294967280`. The first overflowing pixel is row 0, column 16. 5,184 of 6,208
pixels overflow in linear mode, and 5,184 in nearest.

**Matrix B, 100x64, row 5 column 96, linear.** Pre-conversion Y is `-1358970944`, which is
**representable**, so `Y0 = -1358970928`. `bdelta[96] = -972750720`, and the int64 sum
`-2331721648` overflows. X does not overflow: `333809312 + -550213296 = -216403984`. The first
overflowing pixel is row 5, column 78. 5,719 pixels overflow in each mode. Here the overflow is
the addition itself, not any float-to-int conversion.

**Level C intermediate (n103, iteration 3), 97x64.** The warp is
`[-45747.7890625 88966.1875 -3195937.25 -69792.625 128145.84375 -4427218.5]`. At row 0,
column 1 the pre-conversion X is `-3272639744` and Y is `-4533471744`, both out of range, so
`X0 = Y0 = -2147483632` after saturation. X: `-2147483632 + -46845736 = -2194329368`.
Y: `-2147483632 + -71467648 = -2218951280`. 4,422 pixels overflow. This matrix is exactly
the Task 019 4.1.0 `UNSAFE_STORED` entry.

**UBSan reports (instrumented libraries, exact):**

| Version | Linear scalar (X/Y) | Nearest scalar | Nearest SSE4.1 |
|---|---|---|---|
| 4.1.0 | `imgwarp.cpp:2285:41` / `2286:41` | `2244:45` / `2245:45` (with `setUseOptimized(false)`) | `imgwarp.sse4_1.cpp:204:21` / `205:21` |
| 4.6.0 | `2269:41` / `2270:41` | `2229:45` / `2230:45` (with `setUseOptimized(false)`) | `204:21` / `205:21` |
| 4.10.0 | `2373:41` / `2374:41` | `2329:45` / `2330:45` (with `setUseOptimized(false)`) | `204:21` / `205:21` |
| 5.0.0 | not executed for ECC types (see section 5) | not executed | not executed |

Matrix A on 4.10.0 reports `-2147483648 + -3762320` (X) and `-2147483648 + -2147483632` (Y)
for linear, and `-2147483648 + -3761824` / `-2147483648 + -2147483136` for nearest SSE4.1.
The 496-unit X difference is the nearest rounding delta (512 versus 16). Matrix B reports
`-972750720 + -1358970928` (linear) and `-972750720 + -1358970432` (nearest).

**Distinctions requested by the task:**

1. *Out-of-range float-to-int conversion.* OpenCV 4.x clamps the coordinate conversion through
   `saturate_cast`, so the conversion is not UB in C++ terms. `cvRound` itself is documented as
   undefined outside int32, and `saturate_cast` relies on the x86 `cvtsd2si`/"integer
   indefinite" result (`INT_MIN` for both signs) to clamp. That behavior is platform-specific:
   saturation is from a hardware property, not portable code.
2. *`cvRound` on extreme values.* On x86 `_mm_cvtsd_si32` returns `INT_MIN` for +overflow as
   well as -overflow. A huge **positive** coordinate is therefore "saturated" to the most
   negative integer, a sign inversion. UBSan's float-cast check does not flag it, because it is
   an intrinsic rather than a C++ cast.
3. *Signed integer overflow in the addition.* This is the reproduced UB: scalar `int` additions
   in the per-pixel tail of `WarpAffineInvoker` and the SSE4.1 nearest blockline tail.
4. *SIMD integer operations.* `_mm_add_epi32`, `_mm256_add_epi32` and universal-intrinsic
   `v_add` on `v_int32` wrap modulo 2^32. That is defined hardware behavior, not C++ UB, but
   the wrapped coordinate is meaningless. (`imgwarp.sse4_1.cpp:204/205` are the **scalar** tail
   of that file; its vector body uses `_mm_add_epi32`.)
5. *Wrong coordinates without UB reports.* The SIMD bodies, the clamped conversions and
   OpenCV 5.0's suppressed code all produce coordinates unrelated to the true map, then
   `saturate_cast<short>` clamps them. Outputs are wrong but finite, which is why the final
   finiteness check sees nothing.

Nearby arithmetic: the post-addition `>> (AB_BITS - INTER_BITS)`, `& (INTER_TAB_SIZE-1)` and
`saturate_cast<short>` operate on the already-overflowed `int`. Right-shifting a negative
`int` is implementation-defined before C++20, not UB. No other UB operation was found in this
path. On 4.1.0 only, UBSan additionally reports **misaligned `ushort` loads** in
`RemapVec_8u` (`imgwarp.cpp:509-525`) for **any** `CV_8UC1` linear `warpAffine`, including the
moderate negative control. That is a separate pre-existing 4.1.0 issue. It is not on the ECC
path, which warps `CV_8UC1` only with `INTER_NEAREST`, and it is not caused by this finding.

## 5. OpenCV version and dispatch differences

| Version | Path that ECC's `warpAffine` executes for `CV_32FC1` linear / `CV_8UC1` nearest | Signed-overflow reachable? | UBSan visible? |
|---|---|---|---|
| 4.1.0, 4.6.0, 4.10.0 | `hal::warpAffine` builds `adelta`/`bdelta`, then `WarpAffineInvoker`. Linear: AVX2 `warpAffineBlockline` (`_mm256_add_epi32`, defined wrap) or `CV_SIMD128` `v_add`, then the **scalar** tail at `imgwarp.cpp` (lines above). Nearest: SSE4.1 `WarpAffineInvoker_Blockline_SSE41` vector body (`_mm_add_epi32`) plus its scalar tail at `imgwarp.sse4_1.cpp:204-205`, or the generic scalar tail when SSE4.1 is unavailable | **Yes**, reproduced | Yes (scalar tails); the SIMD bodies wrap silently |
| 5.0.0 | `hal::warpAffine` dispatches (`CV_CPU_DISPATCH`, AVX2 here: `cv::opt_AVX2::warpAffineLinearInvoker_32FC1` and `cv::opt_AVX2::warpAffineNearestInvoker_8UC1`, confirmed with gdb) to the **new float-coordinate kernels** in `warp_kernels.simd.hpp`, and returns. The legacy `adelta` + `warpAffineBlockline*` code is still present (marked `CV_DISABLE_UBSAN`) but is **not executed** for these types | Not through the legacy addition. The new kernels form float pixel coordinates and convert with `v_floor` / `v_round` / `cvFloor` / `cvRound` | No reports (also suppressed on `cvFloor(float)` by `CV_DISABLE_UBSAN`) |

The Task 019 note says the OpenCV 5.0 overflow is suppressed by `CV_DISABLE_UBSAN`. That is
true of the source but incomplete: for the types ECC uses, 5.0.0 never reaches that block.
The absence of 5.0 reports is still **not** evidence of defined arithmetic, for three reasons:

* The new kernels convert float to int (`_mm256_cvttps_epi32(_mm256_floor_ps)`,
  `_mm256_cvtps_epi32`, scalar `cvFloor`/`cvRound`). For every recorded unsafe ECC matrix the
  float pixel coordinate stays within int32 (largest observed about 9.4e7). Coordinates beyond
  2^31 would need much larger iterates, and nothing bounds the iterates. The scalar
  `(int)__builtin_floorf` in `cvFloor(float)` would then be a genuine out-of-range
  conversion, and it is explicitly UBSan-suppressed.
* The source address is formed as `v_fma(srcstep, iy, ix)` in `v_int32`. With `ix`, `iy` near
  1e7 the product wraps modulo 2^32 (defined SIMD behavior). It is only used under a bounds
  mask, so it does not reach memory, but the code relies on that masking.
* Other HAL backends (`cv_hal_warpAffine*` replacements, NEON, RVV, LASX) were not exercised.

**Execution paths actually exercised:** x86_64 AVX2 and SSE4.1 dispatch (default), plus
scalar/baseline via `cv::setUseOptimized(false)` (4.1.0, 4.6.0, 4.10.0), all single-threaded
(`cv::setNumThreads(1)`). AVX-512, NEON, RVV, LASX, IPP and OpenCL paths were not run.

## 6. Mitigation candidates

### Option A — Upstream OpenCV correction (research patch built and tested)

**Why widening alone is insufficient.** Changing `X0 + adelta[x]` to `int64_t` removes the UB
but not the wrong coordinate, because both operands are already saturated. For Matrix A at
row 0, column 16 the true scaled `bdelta` is `2250327040`, but the saturated `int` is
`-2147483648` (the x86 `cvRound` sign inversion). The widened sum is `-4294967280`, while the
mathematically correct coordinate is `-1514477808`, a difference of 2.78e9. A correct fix must
bound or compute the **sum** before any int32 step, and must keep `cvRound` operands in range.

**Research patch** ([`docs/research/task020_imgwarp_guard.py`](../research/task020_imgwarp_guard.py)).
This anchored source rewriter was run on scratch copies of the OpenCV 4.10.0 and 4.1.0 trees.
Its output was checked byte-identical (`cmp`) to the sources that were built and tested. It
also applies cleanly to 4.6.0, but that build was not made. It is a review artifact: not
vendored, never run by any repository build, and not an OpenCV source file. It makes two
changes to `imgwarp.cpp`:

1. The `adelta`/`bdelta` table conversion clamps the scaled double to +-2^30 (NaN to -2^30)
   before `cvRound`.
2. A per-row-block guard in `WarpAffineInvoker`. In double it checks that the row base, the
   first and last column deltas and both endpoint sums are within +-2^30. Affine coordinates
   are linear in `x`, so the endpoints bound the whole block. The row then takes the
   unchanged (bit-identical) fast path. Otherwise a scalar fallback computes each pixel
   coordinate in double, clamps the **sum** to +-2^30 and applies the upstream shift/alpha
   post-processing. No int32 addition can overflow, and the fallback uses no SIMD.

**Research patch results** (instrumented `-fsanitize=address,undefined,float-cast-overflow`
builds, configured like the originals; `ldd` confirmed the patched libraries were loaded):

| Check | 4.10.0 patched | 4.1.0 patched |
|---|---|---|
| Level A Matrix A / B: signed-overflow reports | 0 / 0 (original: 4 / 2) | 0 / 0 (original: 4 / 2) |
| Level C `level-c-seeded-n103` through the real export | not reproducible on 4.10.0 originally | **0** (original: 4); still `OPENCV_VIDEO_ERROR_OPENCV` "NaN encountered" |
| Negative control | OK | OK |
| 99-line `warpAffine` corpus (4 geometries x 12 transforms, incl. border-crossing, fully outside, 1e6 translation, scale 40; `BORDER_CONSTANT` and `BORDER_REPLICATE`; `CV_32FC1` linear, `CV_8UC1` nearest and linear) plus 3 native ECC fits | **byte-identical** to original | **byte-identical** to original |
| Native affine ECC, 9 fits (identity, n103, n435 x 97x64, 100x64, 128x128; 100 iterations, eps 1e-8) | byte-identical | identical except the one fit whose original run executed the overflow (100x64 n103): same final warp, error changes from `lambda_d <= 0` to "NaN encountered" |
| Scalar dispatch (`setUseOptimized(false)`) vs default | Normal transform identical to original in both modes. For Matrix A the patched output is all border (sum 0) in both modes; the original UB output differs between modes (sum 6220 vs 2186) | not run |
| Timing, 1024x1024 `CV_32FC1` linear x50 (instrumented, 3 runs) | 708-718 ms vs original 710-740 ms (no measurable overhead) | not run |
| Existing native oracles and AUnit against the patched library | **not run** | **not run** |

The patch does what was intended for these paths: no reproduced overflow remains, and in-range
inputs are bit-identical. It does **not** qualify as a mitigation, for these reasons:

* It covers only `WarpAffineInvoker` on 4.x. It was not ported to 4.6.0 or written for the
  5.0.0 float kernels. The SIMD bodies run only inside the guarded range, but NEON, RVV, LASX,
  IPP, OpenCL and the HAL replacement hooks were not examined.
* The fallback's `cvRound` of a clamped double is in range, but this was checked on x86 only.
* Downstream consumers use their own OpenCV builds, so the binding cannot ship this.
* No upstream issue or pull request exists yet, so maintainers have not reviewed whether the
  saturated-coordinate semantics (any out-of-range coordinate maps to the border) are
  acceptable.

### Option B — Binding-side input validation

**Conclusion: binding-side preflight cannot close the defect.** The binding controls image
geometry, iteration count, epsilon and (for the seeded API) the first iterate. The unsafe
matrices appear at iteration **3** from in-domain seeds, so even `maximum_iterations <= 2`
would only push the problem back one step. The magnitude of `deltaP = H^-1 e` depends on the
Hessian conditioning of the warped images. That conditioning is data-dependent and unbounded:
`lambda_d` can be tiny, and the singular-Hessian guard is absent. So no restriction on
dimensions, seed magnitude or iteration count bounds *every* permitted iterate. Smaller seeds
do not help (n103's largest linear coefficient is 0.58), smaller images do not help (the
reproducers are 97x64), and capping iterations at 2 would break the API. The Task 015 claim
that the seed bounds make the overflow unreachable is withdrawn (see the dated note in
`ecc-affine-seeded-source-contract.md`).

### Option C — Native optimizer interception

`findTransformECC` exposes no per-iteration callback. Its loop, `update_warping_matrix_ECC` and
the `warpAffine` calls are all internal to `ecc.cpp`. Validating each intermediate before it
reaches `warpAffine` would therefore need one of three things:

1. **An upstream callback or option.** OpenCV would have to add an API to `findTransformECC`,
   which it does not have in any version.
2. **A locally maintained fork of `ecc.cpp`.** This is the Task 019 diagnostic technique: a
   renamed copy of `findTransformECC` with a check before each `warpAffine`. As production code
   it would be a copied ECC optimizer, which the task forbids ("do not copy and modify the ECC
   algorithm inside the binding merely to bypass the safety gate"), and it would have to track
   four upstream versions.
3. **Interposing `cv::warpAffine`** through symbol interposition or wrapping. This is fragile,
   ABI-specific, unavailable for static or Windows builds, and changes behavior for every
   caller in the process.

None of these is acceptable without an owner decision, and only (1) is maintainable.

### Option D — Disable or restrict the existing affine APIs

* **Compatibility.** Both overloads and both C exports are in released-API territory
  (`0.1.0-dev`, used since Tasks 014/015). Removing them breaks source and C ABI
  compatibility. Turning them into always-error stubs keeps the ABI but changes behavior.
* **Compile-time gate.** A GPR or Ada configuration switch that removes the Ada overloads
  does not stop a consumer calling the C symbols directly, because they are exported from
  `libopencv_video_shim.a`. Full enforcement would mean making the C entry points refuse at
  runtime, or not compiling them at all.
* **Already-built consumers** keep whatever archive they linked. A new release only protects
  rebuilt consumers.
* **Explicit acknowledgement** (for example a new `Accept_Native_Affine_UB` options field)
  changes the options record, which would need a new subtype or overload.
* **Versioning.** Disabling or gating either API is a behavior change requiring a version bump
  and release note.

Nothing was implemented. Default-disabling or gating needs explicit owner approval.

### Option E — Alternative alignment model

Homography ECC (`MOTION_HOMOGRAPHY`, Tasks 016-018) resamples with `warpPerspective`, so it
does not execute the reproduced `adelta`/`bdelta` addition. An affine seed `[A | t; 0 0 1]` is
conceptually expressible. It is **not** a drop-in replacement, for several reasons:

* It optimizes eight parameters, so it fits a different model with different conditioning and
  convergence basins.
* It returns a 3x3 matrix with perspective terms.
* It has its own seed domain (the denominator rule) and its own oracles.
* `warpPerspective` has its own fixed-point path (`X0 + M[0]*x` in double, then
  `saturate_cast`). This task did not audit it at operand level.

It is recorded only as a separately qualified existing contract. Switching to it would be a
silent optimizer change, so it is not proposed as the mitigation.

### Decision matrix

| | A: upstream fix | B: input validation | C: interception | D: disable/gate | E: homography |
|---|---|---|---|---|---|
| Eliminates the reproduced UB? | Yes on tested 4.x `WarpAffineInvoker` paths (research patch) | **No** | Yes in principle (needs per-iteration hook) | Yes, by not calling native affine ECC | Does not execute this addition; `warpPerspective` not audited |
| Covers unmasked identity and seeded? | Yes (fix is below ECC) | No | Yes | Yes | N/A (different API) |
| Requires rebuilding OpenCV? | **Yes** (until released upstream and adopted) | No | Yes (upstream API) or a fork | No | No |
| Works on all supported versions? | Only after upstream backports; 4.1-4.x are not maintained branches | n/a | Only future versions with the new API | Yes | Yes |
| Compatibility impact | None for the binding API; documented minimum-version or patched-build requirement | Would break valid inputs without closing the defect | None if upstream; large if forked | **Breaking** (source/ABI or behavior); needs version bump | Different model, outputs and contract |
| Maintenance cost | Low in-repo; upstream engagement | n/a | High (fork) or upstream-dependent | Low code, high consumer cost | Already maintained |
| Missing evidence | Upstream review; 4.6.0 and 5.0 port; non-x86 dispatch; full oracle/AUnit run on patched builds; 5.0 float-kernel conversion audit | n/a (disproved) | Upstream API design | Owner decision; consumer inventory | Operand-level `warpPerspective` audit |

## 7. Recommended owner decision

**Recommended direction: Option A upstream, with Option D as an explicit fallback, keeping
the current APIs plus the disclosure in the meantime.**

Before Option A can be adopted:

1. File an upstream OpenCV issue with the Level A reproducer (Matrices A and B) and the Level
   B evidence (in-domain seed -> iteration-3 matrix). Propose a fix along the lines of the
   research patch, which guards the sum, not just the addition type.
2. Obtain an upstream-reviewed fix. Establish which releases carry it, and whether 4.x
   backports exist. Audit the 5.0 float-kernel conversions (`cvFloor(float)` is
   `CV_DISABLE_UBSAN`) for coordinates beyond 2^31.
3. Decide the binding's policy. Either qualify affine ECC only on OpenCV versions containing
   the fix (a configure-time version gate that sets an Ada-visible "affine UB-qualified"
   flag), or document it as unqualified on older versions.
4. Re-run this audit's probe and the full regression on the fixed releases.

If no upstream fix is forthcoming, the owner must choose between:

* keeping the APIs unqualified with the disclosure (status quo); or
* Option D, a breaking or opt-in gate with a version bump. Making the C entry points refuse by
  default is the only form that cannot be bypassed.

**Owner decisions required:**

* Whether to engage upstream (Option A).
* Whether any gating or deprecation (Option D) is acceptable, and at what version.
* Whether translation and Euclidean ECC need the same operand-level audit. They share the
  `warpAffine` branch.

## 8. Consumer compatibility impact

No API, ABI, behavior, dependency or version change is made. Consumers see:

* the README safety notice;
* `SAFETY` comments on both `Estimate_ECC_Affine` declarations in `src/opencv-video.ads`;
* dated follow-up notes in both affine source contracts.

Existing builds are unaffected. Comment-only changes cannot introduce warnings, and no pragma
or `Obsolescent` marker was added, so `-gnatwe` builds keep compiling.

## 9. Qualification evidence

See [task020-qualification.md](../task020-qualification.md). Normal regression results are
reported separately from the expected native-UB reproductions. Passing regression does **not**
resolve the native safety problem.

## 10. Remaining unresolved safety risks

* **The seeded API can execute signed integer overflow inside OpenCV 4.x for validated inputs**
  (demonstrated on 4.1.0). Its trajectories are version-sensitive: the same seed does not
  reproduce on 4.6.0 or 4.10.0, but nothing excludes other inputs there, and Level A shows the
  arithmetic is identical.
* The identity API is `NOT_REPRODUCED`. It is not shown safe.
* On OpenCV 5.0 the reproduced addition is not executed for ECC's types. The float-kernel
  conversions and the UBSan-suppressed `cvFloor(float)` remain unaudited for extreme iterates.
* x86 `cvRound` sign inversion at saturation: wrong-but-finite resampling can steer the
  optimizer even where no UB is reported.
* Translation and Euclidean ECC were not audited at operand level.
* Non-x86 dispatch, IPP, OpenCL and custom HAL builds were not exercised.
* The pre-existing 4.1.0 `RemapVec_8u` misaligned-load report (not on the ECC path) is noted
  but out of scope.
