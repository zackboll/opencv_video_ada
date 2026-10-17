# Seeded homography ECC source contract and safety gate (Task 017)

Scope: `cv::findTransformECC(..., MOTION_HOMOGRAPHY, ...)` with a caller-supplied initial 3x3
Float32 warp, `COUNT|EPS`, no mask, Gaussian size 5, CPU, UInt8 C1, the Task 011 image/option
envelope. Sources reviewed at tags **4.1.0, 4.6.0, 4.10.0 and 5.0.0**: `modules/video/src/ecc.cpp`,
`modules/imgproc/src/imgwarp.cpp` (and 4.1.0 `imgwarp.sse4_1.cpp`), `modules/core/src/lapack.cpp`,
`modules/core/src/matrix_decomp.cpp` and `modules/core/src/arithm.simd.hpp`. Line numbers below are
4.10.0 unless stated; 4.6.0 `ecc.cpp` is byte-identical to 4.10.0.

## Disposition

**`SAFETY_GATE_ESTABLISHED`** for the seed domain below, on IEC 60559 targets (every qualified
target: GCC/Clang on x86-64 and arm64 report `__STDC_IEC_559__` / `numeric_limits<float>::is_iec559`).
This is a memory-safety and defined-arithmetic conclusion only. It does **not** claim convergence,
accuracy, or that the native iterates stay inside the seed domain (they do not; see "Escape").

The gate does not rest on bounding native intermediate matrices. Native ECC updates the eight
non-H22 coefficients without any constraint, so no seed rule can bound later iterations. The
argument is instead that every operation that consumes an unconstrained warp is either clamped or
IEEE-defined, and that no warp value is ever used for indexing or allocation:

1. `warpPerspective` clamps projected coordinates in `double` before integer conversion.
2. Floating division by zero and out-of-range double-to-float conversion follow IEC 60559 (Annex F),
   which the qualified compilers implement and document.
3. LU inversion, the Jacobian, the dot products and the update use matrix values only arithmetically.

The seed rules exist so the *first* iteration starts from a matrix the Jacobian model can represent
(H22 exactly 1, positive denominator over the template) and with moderate magnitudes. They are
binding policy, not an OpenCV guarantee.

## Seed consumption (all four versions)

* A nonempty warp is used as given; only an empty one becomes `Mat::eye(3, 3, CV_32F)`
  (4.1.0 `ecc.cpp:361-368`; 4.6.0/4.10.0 `:378-385`; 5.0.0 `:358-365`). 5.0.0 reaches this through
  `findTransformECCWithMask` with an empty template mask.
* Every iteration first calls `warpPerspective(..., INTER_LINEAR + WARP_INVERSE_MAP)` with the current
  matrix on the image, both gradients and the mask (4.10.0 `:530-533`; 4.1.0 `:513-516`; 5.0.0
  `:528-531`), so the seed is consumed by iteration 1.
* `update_warping_matrix_ECC` adds the 8-vector at `mapPtr[0,3,6,1,4,7,2,5]` and never writes
  `mapPtr[8]` (4.10.0 `:285-296`; 5.0.0 `:262-273`). H22 is therefore exactly the seed's H22.
* **The Jacobian hard-codes H22 = 1.** `den_ = Xgrid*H20 + Ygrid*H21 + 1.0` (4.10.0 `:83`; 5.0.0
  `addWeighted(src3, h2_, src4, h5_, 1.0, den_)`), while `warpPerspective` divides by the real
  `H20*x + H21*y + H22`. A seed with H22 /= 1 makes the Jacobian disagree with the warp being
  optimized. The binding therefore requires H22 to be exactly 1; it never normalizes a seed.
* The update is in place, so an exception can leave the private warp modified. The binding never
  publishes it after an exception.

## Paths that consume an unconstrained warp

| Path | Source | Behavior with huge, zero-denominator or NaN warps |
|---|---|---|
| `warpPerspective` scale | 4.1.0 `imgwarp.cpp:2768,2791`; 4.6.0 `:2799,2929`; 4.10.0 `:3161,3237,3263`; 5.0.0 `:2621,2751,2962,2995`; SIMD `v_select(W != 0, 1/W, 0)` | `W = W ? INTER_TAB_SIZE/W : 0`: a zero denominator selects scale 0; no division by zero. |
| `warpPerspective` int conversion | same lines + 1..3 | `std::max((double)INT_MIN, std::min((double)INT_MAX, v))` before `saturate_cast<int>`. A NaN `v` makes `std::min` return `INT_MAX` (libstdc++ `min` returns its first argument unless `b < a`), so the converted value is always in range. Coordinate buffers are fixed-size stack arrays; `remap` applies the border. |
| Jacobian denominator | `ecc.cpp:83-99` (`cv::divide` on `CV_32F`, `op_div_f::r` = `a / b`, `arithm.simd.hpp:1573-1579`) | No guard. Zero gives ±inf/NaN under IEC 60559. Observed (below): the denominator crossed zero inside the template and reached exactly 0. |
| Hessian / projections | `ecc.cpp:228,239,242` (`(float)` of `double` dot products and `pow(norm, 2)`) | Out-of-range double-to-float conversion is undefined in ISO C++ (`[conv.double]`) but rounds to ±inf under IEC 60559 / Annex F, which GCC documents it follows. Measured maximum finite narrowed value: 7.75e20, 18 orders of magnitude below `FLT_MAX`. |
| Illumination term | `error = lambda*templateZM - imageWarped` (`:595`); `addWeighted` narrows its scalars with `(float)` (`arithm.simd.hpp:1764`; 4.1.0 `:1744`) | Same narrowing argument; `lambda` was included in the measured maximum above. |
| `Mat::inv` (LU) | `hal::LU32f` → `LUImpl` (`matrix_decomp.cpp:15-55`) | Pivot index chosen by comparisons over valid indices; NaN compares false. Singular input returns 0 (Task 016: zero inverse). No value-derived indexing. |
| Guards | `cvIsNaN(rho)` `:573`; `lambda_d <= 0.0` `:586` (4.1.0 `:556,569`; 5.0.0 `:581,593`) | Unchanged from Task 016. Infinite `rho`, a NaN `lambda_d` and huge `deltaP` are not rejected natively. |

Allocation sizes and loop bounds depend only on the image geometry (unchanged from Task 016:
Jacobian `rows x 8*cols <= 2^27` Float32 elements).

## Seed-input policy (binding policy)

| Coefficient | Accepted |
|---|---|
| all nine | finite |
| `H22` | exactly `1.0` |
| `H00 H01 H10 H11` | `abs <= 4` |
| `H02 H12` | `abs <= 2**20` pixels |
| `H20 H21` | `abs(H20)*(cols-1) + abs(H21)*(rows-1) <= 0.5`, evaluated in Float64 |

* The denominator rule makes the seed's `D = H20*x + H21*y + 1` lie in `[0.5, 1.5]` at every
  template pixel, so iteration 1 never starts on or near a pole and never flips orientation. It is
  geometry-dependent and therefore checked after the image size is known (Ada and C use the same
  rule; the products are exact enough in Float64 for Float32 inputs and integer sizes <= 16,383).
* Linear and translation limits are the Task 015 affine limits. For homography they are not needed
  for the clamped integer conversion; they keep the seed's projected coordinates near the image
  (`<= (2*4*16383 + 2^20) / 0.5 ~= 2.4e6`). They were **not** loosened because a larger seed
  happened to return.
* No determinant, invertibility, conditioning or orientation rule is applied to the input beyond
  these. A singular finite seed (all linear terms 0) is accepted and produced a native exception.
* Violations are rejected before any native call: `OPENCV_VIDEO_ERROR_INVALID_ARGUMENT` (C) or Ada
  `OpenCV_Error` without the shim prefix.
* Output policy is Task 016's, unchanged: schema/storage identity, nine finite coefficients, H22
  exactly 1, finite correlation. **The seed domain is not applied to the output.**

## Measured native behavior

### Probe (`tests/cpp/ecc_homography_seeded_probe.cpp`)

64x96 analytic texture against the same texture shifted (+2, +1), epsilon 1e-8, one thread. 2000
deterministic seeds (18 at each single-coefficient limit, the rest random inside the domain, H11
down to 1e-8); 1995 pass the domain. Then 64 edge-of-denominator seeds (`|H20|(cols-1) +
|H21|(rows-1) = 0.5` split between the two terms, with and without extreme linear/translation
terms), 24 of which are in the domain, run for 100 iterations. "Escaped" counts native successes
whose output is outside the seed domain. The probe exits 2 if native ECC changes H22: it never did.

Release builds (`~/.cache/video-task017/install-<v>`; 4.10.0 is the system package):

| OpenCV | 1 iteration OK / EXC / escaped | 100 iterations OK / EXC / escaped | max \|coef\| (100 it.) | edge seeds OK |
|---|---|---|---:|---|
| 4.1.0 | 687 / 1308 / 653 | 192 / 1803 / 170 | 2.36e6 | 9 / 24 |
| 4.6.0 | 687 / 1308 / 653 | 191 / 1804 / 169 | 2.38e6 | 9 / 24 |
| 4.10.0 | 687 / 1308 / 653 | 191 / 1804 / 169 | 2.38e6 | 9 / 24 |
| 5.0.0 | 687 / 1308 / 652 | 177 / 1818 / 153 | 5.84e6 | 9 / 24 |

**Instrumented OpenCV.** Each version was also built from source (`RelWithDebInfo`,
`-fsanitize=address,undefined,float-cast-overflow`, core/imgproc/video, no IPP/OpenCL/ITT;
`nm -D` shows `__asan_report*`, `__ubsan_handle*` and `__ubsan_handle_float_cast_overflow` in
`libopencv_imgproc`) and the instrumented probe run with `halt_on_error=1`, leak detection on:

| OpenCV | 1 iteration, 1995 seeds OK / EXC | 100 iterations, 595 seeds OK / EXC | Reports |
|---|---|---|---|
| 4.1.0 | 687 / 1308 | 61 / 534 | 0 |
| 4.6.0 | 687 / 1308 | 59 / 536 | 0 |
| 4.10.0 | 687 / 1308 | 59 / 536 | 0 |
| 5.0.0 | 691 / 1304 | 55 / 540 | 0 |

All runs exited 0 with zero `runtime error`/`AddressSanitizer` lines. The instrumented 5.0.0 build
differs numerically from its Release build (different optimization level); outcomes, not bits, are
compared across builds. This also closes Task 016's open item 1 for the paths exercised here: the
`warpPerspective` NaN/huge-coordinate conversion ran under instrumented OpenCV without a report.

### Diagnostic copy of 4.10.0 `ecc.cpp` (not committed)

A scratch copy of 4.10.0 `ecc.cpp` (renamed entry point, linked against system 4.10.0) added:
a trap if any finite `double` narrowed by `project_onto_jacobian_ECC` or `lambda` exceeds `FLT_MAX`;
the running maximum of those values; and the minimum of `abs(den_)` per iteration. Seeds as above,
100 iterations:

| Geometry | Seeds | Narrowing traps | Max finite narrowed value | Min \|den\| |
|---|---:|---:|---:|---:|
| 64x96 | 3000 | 0 | 3.0e14 | 1.9e-8 |
| 128x128 | 1500 | 0 | 2.5e15 | **0** |
| 256x512 | 300 | 0 | 7.1e18 | 6.8e-8 |
| 1024x1024 | 40 | 0 | 2.3e19 | 1.6e-6 |
| 32x16384 | 40 | 0 | 7.8e20 | 2.7e-6 |
| 16384x32 | 40 | 0 | 2.2e13 | 2.6e-6 |

* The probe's own 2000-seed, 100-iteration sweep was also run through the copy (Hessian and
  projection traps only): 0 traps, and the denominator changed sign inside the template grid in
  4432 iterations (`minMaxLoc` never reported an all-negative or NaN range). At 128x128 an exact
  float zero was reached. Native ECC continued with IEEE infinities/NaNs and, in these runs, the NaN
  or `lambda_d` guard or the iteration limit ended it; no crash or hang.
* The identity-initialized Task 016 path (its seven hard fixtures plus noise) also crossed zero
  once (min -0.19, max 2.49) with minimum `|den|` 2.5e-4. Zero crossings are therefore a property of
  native ECC, not something seeding introduces; seeding makes them more frequent.

### Escape

About a third of one-iteration successes leave the seed domain (up to `|coef| ~ 2.4e6` on
4.x and `5.8e6` / `1.5e7` on 5.0.0 Release / instrumented). Consequently a returned result, including
a Task 016 identity-initialized result, is not guaranteed to be an acceptable seed; reusing it as
`Initial_Transform` may be rejected by seed validation. The binding does not clamp, normalize or
re-project it.

## Unproven

1. Memory safety rests on source review plus the measurements above, not a proof. Instrumentation
   covers C++ code; SIMD intrinsic paths are hardware-defined and only reviewed.
2. Reliance on IEC 60559 semantics for floating division by zero and double-to-float overflow is
   explicit. A target without IEC 60559 floating point is not qualified.
3. Larger geometries had few seeds (40) because each 100-iteration call is expensive; the numbers are
   observations, not bounds.
4. Windows (MSYS2 GCC, also IEC 60559) is covered by source review only until the post-merge job.