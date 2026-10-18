# Seeded ECC Euclidean source contract (Task 013)

Binding target: `cv::findTransformECC(template, moving, warp, MOTION_EUCLIDEAN,
TermCriteria(COUNT|EPS, n, eps), noArray(), 5)` where `warp` is a **private** `CV_32FC1` 2x3
matrix initialised from the caller's rotation and translation instead of `Mat::eye(2, 3, CV_32F)`.
Everything else is Task 012: same motion model, criteria, no mask, Gaussian size 5, image
envelope, option limits, output validation and `atan2` angle decoding.

## Sources reviewed

`modules/video/src/ecc.cpp` was downloaded from the upstream tags `4.1.0`, `4.6.0`, `4.10.0`
and `5.0.0` and read for the seeded path. `ecc.cpp` at 4.6.0 is byte-identical to 4.10.0
(`cmp`). `image_jacobian_euclidean_ECC` and `update_warping_matrix_ECC` differ between 4.1.0 and
5.0.0 only in formatting, assertion ordering, and a `CV_MAKETYPE(CV_32F, channels)` form of the
Jacobian type assertion; the arithmetic is the same. 5.0.0 moves the body into
`findTransformECCWithMask`; the seven-argument `findTransformECC` forwards to it.

## What native ECC does with a supplied warp

1. **It reads the matrix before refinement.** `Mat map = warpMatrix.getMat()` shares the
   caller's storage. The first iteration calls `warpAffine(..., map, ..., INTER_LINEAR +
   WARP_INVERSE_MAP)` on the image and both gradients, and `image_jacobian_euclidean_ECC` reads
   `h0 = map[0]` and `h1 = map[3]` (cosine and sine of the current angle) from the same matrix.
   The supplied rotation and translation are the starting point. The "uninitialised matrix"
   branch only runs when `map.empty()`.
2. **Allocation and schema are preserved.** The only reallocation path is `map.empty()`; a
   non-empty matrix is validated (`CV_32FC1`, `cols == 3`, `rows == 2 || 3`) and updated in
   place. The binding still records `warp.data` and rejects a changed pointer, geometry, type or
   non-contiguous layout after the call.
3. **Rotation and translation are both consumed.** The warp is applied directly (template ->
   moving coordinates, `WARP_INVERSE_MAP`) and `update_warping_matrix_ECC` adds the translation
   update to `mapPtr[2]`, `mapPtr[5]` in place.
4. **The matrix is mutated every iteration** and is the only output channel for the transform.
5. **It can be left partly modified when native code throws.** `StsNoConv` ("correlation is
   going to be minimized") and `NaN encountered` are raised from inside the loop after earlier
   iterations already updated `map`. The binding never reads the warp after an exception and
   `*result` is written only on success.
6. **Correlation semantics are unchanged.** `rho = -1`, `last_rho = -eps`; `rho` is computed at
   the start of an iteration, before that iteration's update, so it scores the warp one step
   before the returned matrix. A nonzero start changes only the path to the optimum.
7. **No other conversion is sensitive to a nonidentity start.** Images are converted to `CV_32F`
   and Gaussian-blurred before the loop; the warp is only passed to `warpAffine`. A large
   translation samples the border; it does not overflow an integer conversion. The one place a
   large angle matters is the `asin` below.

## The `asin` limitation and the seed domain

```cpp
double new_theta = updatePtr[0];
new_theta += asin(mapPtr[3]);
```

`mapPtr[3]` is `sin(theta)` of the **current** matrix; `asin` returns `[-pi/2, pi/2]`. A start
angle outside that interval is reconstructed as `pi - theta` and the first update is applied to
the wrong angle. Inside the interval a large update can still push the next iterate across
`pi/2`.

Binding policy (not an OpenCV guarantee, not a claim that every accepted seed converges):

| Component | Accepted | Why |
| --- | --- | --- |
| translation X, Y | finite, `abs <= 2**20` | `2**20` is exactly representable in Float32 and far larger than any 16,384-pixel image side |
| rotation | finite, `abs <= 1.0` rad (about 57.3 degrees) | strictly inside the principal `asin` branch, margin below pi/2 |

Violations raise `OpenCV_Error` (`OPENCV_VIDEO_ERROR_INVALID_ARGUMENT` from the shim) and are
never clamped. Validation runs in widened `double` arithmetic before any native matrix is built
and before any native call. The probes gave no evidence that demanded a narrower domain: seeds at
exactly +-1 rad and at +-2**20 translation reach native code and either return a finite result or
raise a native `StsNoConv` with `*result` unchanged. The bound is deliberately not widened because
a probe returns something finite. Final-output angles beyond a quarter turn remain valid
**outputs** (Task 012's +120/-150 degree corruption tests); that does not authorise such inputs.

## Private matrix construction

```text
warp(0,0) =  cos(theta)   warp(0,1) = -sin(theta)   warp(0,2) = initial_translation_x
warp(1,0) =  sin(theta)   warp(1,1) =  cos(theta)   warp(1,2) = initial_translation_y
```

`cos`/`sin` are evaluated in `double`; each element is converted to `float` once. The matrix is a
fresh `Mat::eye(2, 3, CV_32F)` overwritten element by element, so it is a continuous 2x3
`CV_32FC1` allocation. The shim checks geometry, type, continuity and six finite elements before
the call, records the storage pointer and re-checks it afterwards. The seed crosses the C ABI by
value; no native warp is ever caller-owned.

## Experiments: native probes (tests/cpp/ecc_euclidean_seeded_probe.cpp)

128x128 analytic texture; `rigid_texture(deg, tx, ty)` is the moving image; 50 iterations,
epsilon 1e-4. "identity" is Task 012's start. Seeds: "near" = (true - 0.5 deg, tx + 0.3,
ty - 0.2), "wrong-rot" = (-deg, tx, ty), "wrong-trans" = (deg, tx + 25, ty - 25). The probe ran
against all four versions (4.1.0 and 5.0.0 pinned local installs, 4.6.0 in the Debian 12
container, 4.10.0 system).

| Fixture | identity start | near seed (all four) | wrong-rot seed | wrong-trans seed |
| --- | --- | --- | --- | --- |
| +8 deg | throws, all four | recovers 7.998 deg, rho 0.9935 | throws | throws |
| -8 deg | wrong finite +6.4 deg, rho 0.05 on 4.1.0/4.6.0/4.10.0; **throws on 5.0.0** | recovers -8.005 deg | throws | throws |
| +10 deg | wrong finite (tx 17, ty -17, rho 0.60), all four | recovers 9.996 deg | throws | wrong finite, rho 0.81 |
| +12 deg | throws, all four | recovers 12.002 deg | throws | wrong finite, rho 0.81 |
| +15 deg | throws, all four | recovers 15.004 deg | wrong finite -65.5 deg, rho 0.39 on 4.1.0/4.6.0/4.10.0; **throws on 5.0.0** | throws |
| +10 deg, (2,1) | wrong finite (tx 18.8, ty -15.9, rho 0.60), all four | recovers 9.998 deg, (1.99, 1.01) | throws on 4.1.0/4.6.0/4.10.0; **wrong finite -8.55 deg on 5.0.0** | wrong finite, rho 0.81 |
| +12 deg, (2,1) | throws, all four | recovers 12.001 deg | throws | wrong finite, rho 0.81 |

Portable distinguishing fixtures (same qualitative outcome on all four versions): **+8 deg,
+12 deg and +12 deg + (2,1)** (identity throws, near seed recovers); **+10 deg and +10 deg +
(2,1)** (identity returns a wrong finite optimum, near seed recovers); and the wrong-sign
rotation seed on +8/+12 (throws). The -8 deg identity outcome, the +15 deg wrong-rot outcome and
the +10 deg + (2,1) wrong-rot outcome **differ by version**, so AUnit does not assert them; they
are recorded only by the same-build oracle (obj/oracle/ecc-euclidean-seeded.txt), which the
installed OpenCV regenerates on every run.

A seed is refined, not returned unchanged: for the seed (+2.5 deg, 2.3, 0.8) against a true
(+3 deg, 2, 1) the result is (+2.988 deg, 1.981, 1.021) and all three components are closer to
the truth than the seed. With 1 iteration the same call returns a visibly different result
(rho 0.967) than with 50 (rho 0.9933).

## Follow-up (Task 021, 2026-10-18): seed bound versus native arithmetic

Finding 7 above says "A large translation samples the border; it does not overflow an
integer conversion." The Task 013 record and its qualification are preserved. Task 021
narrows that statement:

* **The conversion itself.** `saturate_cast<int>` of the row base does not overflow.
  Above `2^21`, x86 `cvRound` returns `INT_MIN` instead (a sign inversion, defined at the
  intrinsic level).
* **The addition.** The next step, `X0 + adelta[x]` / `Y0 + bdelta[x]` in signed `int` in
  OpenCV 4.x `warpAffine`, **does** overflow when a translation lies in a narrow band just
  below `2^21` (about `2.08e6 .. 2.097e6`).
* **Confirmed on 4.x.** UBSan reported it on instrumented 4.1.0, 4.6.0 and 4.10.0 for a
  model-valid rigid matrix (Level A,
  [Task 021](tasks/021-translation-euclidean-safety-audit.md)).

**The accepted seed domain (`|t| <= 2^20`, `|θ| <= 1`) lies below the band**, so the first
iterate cannot reach it. Later optimizer updates are not bounded by the seed. Results so
far:

* **Extreme seed.** `(0.9 rad, 2^20, -2^20)` through
  `opencv_video_estimate_ecc_euclidean_seeded` returned the ordinary "NaN encountered"
  error with no UBSan report on all four versions. The images no longer overlap, so `rho`
  is `0/0`.
* **Searches.** 720 bounded seeded trials (four versions) covered zero, near-correct, wrong,
  ±1 rad, near-`2^20` and opposing seeds. No intermediate entered the window. The largest
  translation recorded, 1048575.6, was a starting seed value, not an optimizer step.

**Disposition: `SAFETY_GATE_NOT_ESTABLISHED`** (`NOT_REPRODUCED`; this is not shown to be
safe).

## Not claimed

* That an accepted seed converges, or that a finite result is correct. Wrong-but-finite results
  are returned as values, as in Task 012.
* Cross-version numerical equality or equal convergence basins.
* That the seed bound prevents all incorrect optimisations.
* Freedom from undefined behaviour inside OpenCV's shared imgproc code. No ECC-specific UB
  exception is added; the separately accepted Farneback pointer-formation issue is unrelated and
  untouched.

