# ECC Euclidean source contract (Task 012)

Binding target: `cv::findTransformECC(template, moving, warp, MOTION_EUCLIDEAN,
TermCriteria(COUNT|EPS, n, eps), noArray(), 5)` with a private `Mat::eye(2, 3, CV_32F)`.

## Sources reviewed

`modules/video/src/ecc.cpp` and `modules/video/include/opencv2/video/tracking.hpp` were
downloaded from the upstream tags `4.1.0`, `4.6.0`, `4.10.0` and `5.0.0` and read for the
Euclidean path. `ecc.cpp` at 4.6.0 is byte-identical to 4.10.0 (checked with `cmp`).
`image_jacobian_euclidean_ECC`, `update_warping_matrix_ECC` and the three-parameter
selection are the same in all four. 5.0.0 only restructures the entry point into
`findTransformECCWithMask`.

## Findings

1. **Parameters.** `MOTION_EUCLIDEAN` optimizes three parameters (angle, tx, ty).
   `update_warping_matrix_ECC` asserts `map.rows == 2 && update.rows == 3`.
2. **Jacobian.** `image_jacobian_euclidean_ECC` asserts `dst.cols == src1.cols * 3` and
   fills three `w`-wide blocks (angle, x, y), against two for translation. It reads
   `h0 = map[0]` (cos) and `h1 = map[3]` (sin) from the current warp.
3. **Warp layout and direction.** The warp is 2x3 `CV_32FC1`, row-major
   `[cos -sin tx; sin cos ty]`; template coordinates map to moving coordinates, and
   OpenCV brings the moving image back with `warpAffine(..., WARP_INVERSE_MAP)`. This was
   confirmed by experiment (below), not only by reading.
4. **Update rule and the `asin` limitation.** In all four versions:

   ```cpp
   double new_theta = updatePtr[0];
   new_theta += asin(mapPtr[3]);
   mapPtr[2] += updatePtr[1];
   mapPtr[5] += updatePtr[2];
   mapPtr[0] = mapPtr[4] = (float)cos(new_theta);
   mapPtr[3] = (float)sin(new_theta);
   mapPtr[1] = -mapPtr[3];
   ```

   The previous angle is recovered from `sin(theta)` alone. `asin` has range
   `[-pi/2, pi/2]`, so for a current angle beyond a quarter turn it returns
   `pi - theta`, not `theta`, and the update is applied to the wrong angle. Nothing in the
   loop can track a rotation past +/-90 degrees. The experiments below show the practical
   basin is far smaller than that and irregular, so the `asin` ambiguity is not the
   dominant limit for the textures tested; it is a further structural limit. The binding
   decodes the final angle with `atan2(w[3], w[0])`, never `asin`. That returned angle is
   the angle of the final warp, not a reconstruction of the iteration history.
5. **Initialization and termination.** Same as translation: `rho = -1`,
   `last_rho = -eps`; loop while `i <= maxCount && |rho - last_rho| >= eps`. Reaching
   `maxCount` is not an error. **The returned correlation is computed at the start of the
   last iteration, before its update**, so it scores the warp one step earlier than the
   returned matrix.
6. **Warp storage.** `findTransformECC` takes the caller's `Mat` by reference and updates
   it in place. The binding records `warp.data` before the call and rejects a changed
   pointer, dimensions, type or non-contiguous layout.
7. **Rigid structure of the result.** Each update writes `mapPtr[0] = mapPtr[4]` from a
   single `float` cast and `mapPtr[1] = -mapPtr[3]` exactly, so `a == d` and `b == -c`
   hold bit for bit. Over the 20 converged fixtures of the Euclidean oracle on each of
   4.1.0, 4.10.0 and 5.0.0 the largest deviation was 0 for `a-d`, `b+c` and `a*b+c*d`,
   and 5.8e-8 (4.1.0) / 5.9e-8 (4.10.0, 5.0.0) for `a*a+c*c-1`, `b*b+d*d-1` and `det-1`.
8. **Memory.** The Euclidean path adds one `hs x 3*ws` Jacobian (translation: `hs x
   2*ws`) and a `3x3` Hessian. Measured below; it is about 33% above translation.
9. **Integer arithmetic.** The Jacobian width is `3*ws`, at most 49,152 for
   `ws <= 16384`. `project_onto_jacobian_ECC` slices `colRange(i*w, (i+1)*w)` for `i < 3`,
   so `3*w` is the only product and is far below `INT_MAX`. Plane sizes stay below
   `INT_MAX` with `hs*ws <= 2^24`.
10. **Division / non-finite.** `rho = correlation / (imgNorm * tmpNorm)`: constant images
    give NaN and upstream throws `StsNoConv "NaN encountered."`. `lambda_d <= 0` throws
    `StsNoConv`. A very small positive `lambda_d` is not guarded. The binding re-checks
    finiteness of every warp entry, the correlation and the extracted angle.

These findings come from reading the source and the measurements below; they are not a
proof of the absence of undefined behaviour. No out-of-bounds or invalid-memory access
was found in the Euclidean path, and none occurred under ASan/UBSan.

## Validation of the returned matrix (binding policy)

After the native call the shim validates, in this order: warp storage identity,
dimensions exactly 2x3, `CV_32FC1`, contiguity, six finite entries, finite correlation,
then with `a=w[0], b=w[1], c=w[3], d=w[4]` in double:

| Check | Bound | Reason |
| --- | --- | --- |
| `abs(a - d)`, `abs(b + c)` | `<= 1e-6` | exact in native output; the slack only tolerates nothing but a corrupted copy |
| `abs(a*a + c*c - 1)`, `abs(b*b + d*d - 1)`, `abs(a*b + c*d)` | `<= 1e-5` | observed <= 5.9e-8, about 170x margin |
| `determinant > 0`, `abs(det - 1)` | `<= 1e-5` | rejects reflection and scale; observed <= 5.9e-8 |
| `atan2(c, a)` finite | | |

No maximum rotation is imposed: a wrong but rigid answer is a convergence limitation,
not an invalid matrix.

## Measured behaviour (128x128 analytic texture, defaults 50 / 1e-4), 4.10.0 and 4.1.0

Probe `tests/cpp/ecc_euclidean_probe.cpp` (OpenCV only; no shim, no Core bridge). The
moving image is `cv::warpAffine(template, W, BORDER_REFLECT_101)` with the forward matrix;
the residual is the mean absolute difference over the interior (24 px margin) before and
after `warpAffine(moving, estimate, INTER_LINEAR + WARP_INVERSE_MAP)`. The fixture was
cross-checked against analytic resampling (interior MAD 0.28). Rows below are 4.10.0.
4.1.0 gives the same values to about 1e-4 (for example tx -0.00676 against -0.00674),
except where stated; cross-version equality is not claimed.

| Case | Estimated angle (rad) / tx / ty | rho | Residual before -> after |
| --- | --- | --- | --- |
| identity | 0 / 0 / 0 | 1.0000 | 0 -> 0 |
| translation (2,1) | 0.00000 / 2.0107 / 1.0083 | 0.99965 | 13.69 -> 0.00 |
| +1 deg | 0.01747 (true 0.01745) / 0.005 / -0.009 | 0.9936 | 8.04 -> 0.46 |
| -1 deg | -0.01769 / -0.018 / 0.008 | 0.9957 | 8.04 -> 0.46 |
| +3 deg | 0.05233 (true 0.05236) / -0.007 / 0.007 | 0.9940 | 22.54 -> 0.42 |
| -3 deg | -0.05239 / 0.001 / -0.005 | 0.9935 | 22.47 -> 0.42 |
| +6 deg | 0.10474 (true 0.10472) | 0.9944 | 35.82 -> 0.41 |
| +3 deg, t=(2,1) | 0.05213 / 1.9816 / 1.0165 | 0.9934 | 21.86 -> 0.42 |
| +3 deg, t=(-2,-1) | 0.05224 / -2.0190 / -0.9861 | 0.9932 | 28.54 -> 0.43 |
| centred +3 deg, centre shift (1.5,-1) | 0.05230 / 4.9136 / -4.2426 (matrix 4.9104 / -4.2363) | 0.9935 | 12.04 -> 0.43 |
| reversed roles (+3 deg, (2,1)) | -0.05219 / -2.0328 / -0.9060 | 0.9935 | 21.86 -> 0.05 |
| 0.8x+20 photometric | 0.05209 / 1.9776 / 1.0207 | 0.9909 | 20.74 -> 7.23 |
| 1 / 3 iterations | 0.05397 / 2.1504 / 1.0889 (rho 0.610); 0.05217 / 1.9834 / 1.0160 (rho 0.9937) | | |

Largest error on good cases: 0.00024 rad and 0.04 px (rotation-only translation, plus
about 0.02 px on combined cases). Test tolerances are 0.005 rad and 0.1 px, about 20x and
2.5x those errors. The residual after alignment is nonzero because of interpolation, the
reflected border and (photometric row) the intensity change.

The returned `Displacement` for a rotation about the image centre is the matrix
translation, not the centre shift: for 3 degrees about (63.5, 63.5) with shift (1.5, -1)
the true matrix translation is (4.9104, -4.2363) and the estimate is (4.9136, -4.2426);
the centre shift (1.5, -1) is not returned. Independent point mapping in the AUnit test
confirms the centre lands at centre + (1.5, -1).

### Convergence basin (angle only, 128x128, defaults), 4.10.0

`tests/cpp/ecc_euclidean_limits.cpp basin`:

| Angle | Outcome |
| --- | --- |
| +/-1, 3, 5, 6 deg | converges, error < 0.03 deg |
| +8 deg | throws `StsNoConv` |
| +9, +10, +11 deg | **success, correct angle but wrong translation** (tx ~ 17, ty ~ -17), rho ~ 0.60 |
| -8 deg | **success, wrong sign: +6.4 deg**, rho 0.05 |
| +12 .. +179 deg | throws |
| -12 deg | converges correctly (rho 0.99) |
| -9, -10, -11, -13 .. -17 deg | throws |
| -20 deg | **success, -3.4 deg**, rho 0.12 |
| -60 deg | **success, -35.9 deg**, rho 0.18 |
| +/-90, 120, 170, 179 deg | throws |

The basin is irregular and non-monotonic and depends on the texture, so the binding
claims nothing beyond a few degrees. Results are also version-dependent: at 4.1.0 the
10-degree probe returns a wrong finite warp (angle 0.1797 rad, tx 54, ty -19, rho 0.55)
while at 4.10.0 the same probe throws, and the 15-degree probe converges at 4.1.0 and
throws at 4.10.0. Low rho (<= 0.6) accompanied every wrong result here, but the binding
applies no correlation threshold.

### Memory (4.10.0, peak resident `VmHWM`, 50 iterations)

| Image | Translation | Euclidean |
| --- | --- | --- |
| 4096x4096 | 1,007,032 kB | 1,334,432 kB |
| 1024x16384 | | 1,333,312 kB |
| 16384x1024 | | 1,334,420 kB |

The Euclidean model needs about 1.3 GB at the 2^24-pixel limit (translation about 1.0 GB).
The pixel limit is unchanged: the policy was a memory/time bound that this increase does
not invalidate. No routine test allocates a limit-sized image pair; limit violations are
rejected before native work with allocations that ECC never touches.

## Follow-up (Task 021, 2026-10-18): native `warpAffine` arithmetic

The Task 012 findings and qualification stand. The statement "No out-of-bounds or
invalid-memory access was found in the Euclidean path, and none occurred under ASan/UBSan"
is accurate for what was run: the binding was instrumented and OpenCV was not.

[Task 021](tasks/021-translation-euclidean-safety-audit.md) then audited the shared
`warpAffine` fixed-point addition at operand level:

* **Bounded rotation does not bound the coordinate.** Bounded rotation coefficients
  (`|cos|, |sin| <= 1`) bound only the per-column delta. They do not bound the row base,
  which carries the accumulated `tx`/`ty`.
* **Model-valid overflow confirmed.** A rigid matrix (θ = 0.5 rad, `ty = 2090000`)
  overflows `Y0 + bdelta[x]` in signed `int`. UBSan reported this on instrumented 4.1.0,
  4.6.0 and 4.10.0 (Level A).
* **Row-base overflow (Task 021-R1).** Building the row base,
  `Y0 = cvRound((c*y + ty)*1024) + round_delta`, is itself a signed `int` addition. It can
  overflow before any column delta is applied. The rotated row term is not a multiple of 128,
  so for Euclidean this can happen even in `INTER_LINEAR` mode (`round_delta = 16`).
  * Example: θ = -0.5, `t = (0, 2097109.875)` at 64x97. At row 48 the row base converts to
    `2147483647`. The first UB is `2147483647 + 16`, reported at
    `imgwarp.cpp:2214:84` / `2200:84` / `2300:84` on 4.1.0 / 4.6.0 / 4.10.0.
* **Conversion followed by overflow (Task 021-R1).** Above `2^21`, x86 `cvRound` returns
  `INT_MIN`. Translation's non-negative deltas leave that in range. A rotation with a
  negative column coefficient on the same axis, however, overflows
  `INT_MIN + round_delta + delta` below `INT32_MIN`.
  * Example: θ = -0.5, `ty = 2097200`. UBSan reported `-47129 + -2147483632` at
    `imgwarp.cpp:2286:41` / `2270:41` / `2374:41`.
  * The model flagged every tested `ty` up to `1e9`. This route therefore has **no upper
    bound in `|t|`**.
* **Rotation alone cannot overflow.** A ~89-degree rotation with small translation
  produced no overflow.
* **Not reproduced through ECC.** No native ECC run (Level B) or call to
  `opencv_video_estimate_ecc_euclidean` (Level C) reached the overflow. The largest
  accumulated `|t|` over 600 bounded identity trials (four versions) was 5522, against a
  window near `2.08e6`.
* **OpenCV 5.0.0.** It does not execute these additions for ECC's types. gdb confirmed this
  for the row-base cases as well.

**Disposition: `SAFETY_GATE_NOT_ESTABLISHED`.** No argument bounds the accumulated
translation, and no execution was shown to reach any window. Task 021-R1 adds Level A
evidence only; it does not change this disposition.

## Not claimed

* Recovery of arbitrary or near-180-degree rotations, or a continuous unwrapped angle.
* That a finite, rotation-consistent result is an accurate alignment.
* Cross-version numerical equality.
* Absence of undefined behaviour inside OpenCV's shared imgproc code.
