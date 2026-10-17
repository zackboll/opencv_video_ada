# Masked homography ECC source contract and safety gate (Task 018)

Scope: the common seven-argument `cv::findTransformECC(template, moving, warp, MOTION_HOMOGRAPHY,
TermCriteria(COUNT|EPS, n, eps), inputMask, 5)` with a non-empty `inputMask`, CPU, UInt8 C1 images,
the Task 011 image/option envelope and either the identity warp (Task 016) or a Task 017 in-domain
seed. `findTransformECCWithMask` (5.0 only) is **not** used. Sources reviewed at tags **4.1.0,
4.6.0, 4.10.0 and 5.0.0**: `modules/video/src/ecc.cpp`, `modules/imgproc/src/thresh.cpp`,
`smooth.dispatch.cpp`, `imgwarp.cpp`, `modules/core/src/mean.dispatch.cpp`, `copy.cpp`,
`count_non_zero.dispatch.cpp`, `arithm.cpp`. `ecc.cpp` at 4.6.0 is byte-identical to 4.10.0; 4.1.0
differs from 4.10.0 only in comments and `computeECC` (unused). Line numbers are 4.10.0 unless stated.

## Disposition

**`SAFETY_GATE_ESTABLISHED`** for a moving-image mask that is a non-empty 2-D `CV_8UC1` Mat with
exactly the moving image's rows and columns and at least one nonzero pixel (Regions accepted),
combined with the unchanged Task 016/017 image, option and seed contracts, on IEC 60559 targets.

This is a memory-safety and defined-arithmetic conclusion only. A mask does **not** make
alignment reliable, does not guarantee convergence, and can make a finite result worse (see
"Insufficient support"). The Task 016/017 gates are not assumed to cover this path; every
mask-specific operation is reviewed and exercised below.

Accepted restrictions (binding policy, enforced in Ada and again in the C shim):

| Property | Accepted | Why |
|---|---|---|
| Presence | non-empty | an empty `Mat` is treated natively as *no mask* (measured bitwise equal to `noArray()` on all four versions); the masked API must not silently unmask |
| Dimensions | 2 | 4.x accepts a 96x128x1 3-D Mat as 2-D, 5.0.0 throws; restricting removes the difference |
| Depth / channels | UInt8, 1 | 4.x rejects every other depth in `meanStdDev`; 5.0.0 accepts 16U/16S/32F/64F (bitwise equal to 8U for 0/1 data); 8S and 32S throw in `threshold` on all versions; multi-channel throws on all versions |
| Geometry | rows and columns of the moving image (and therefore of the template) | mismatched sizes throw on all versions (`mul`/`setTo` size checks) before the first iteration |
| Row step | `<= INT_MAX` | same rule as the images |
| Support | at least one nonzero pixel (checked natively with `cv::countNonZero` in the shim) | an all-zero mask cannot ever contribute; rejected before native execution with a binding diagnostic |
| Storage | contiguous or Region (non-contiguous) | native reads it only through `threshold` into a private Mat |

No minimum-support restriction beyond "one nonzero pixel" is applied; see "Insufficient support".

## Native mask path (all four versions)

1. **Coordinate frame.** `inputMask` is the mask of `inputImage` (the moving image). It is
   binarized into `preMask` of size `hd x wd` (moving geometry) and warped every iteration with the
   same matrix as the moving image: `warpPerspective(preMask, imageMask, map, imageMask.size(),
   INTER_NEAREST + WARP_INVERSE_MAP)` (4.1.0 `ecc.cpp:516`, 4.10.0 `:533`, 5.0.0 `:531`). The result
   `imageMask` is in template coordinates. There is no template mask on this path; template pixels
   are excluded only when their warped location falls on an excluded moving pixel or outside the
   moving image (constant border 0, which is also the unmasked behavior).
2. **Binarization.** `threshold(inputMask, preMask, 0, 1, THRESH_BINARY)` (4.1.0 `:447`, 4.10.0
   `:464`, 5.0.0 `:483`): every nonzero value becomes 1, zero stays 0. For 8U the threshold is
   `cvFloor(0) = 0` and `maxval = cvRound(1) = 1` (`thresh.cpp`, all four versions). Values 1, 7, 255
   and arbitrary nonzero values are therefore equivalent; measured bitwise on all four versions.
   `threshold` writes only its destination (`_dst.create`), never the input.
3. **Erosion by Gaussian smoothing.** `preMask` is converted to `CV_32F`,
   `GaussianBlur(Size(5,5), 0, 0)` (default `BORDER_REFLECT_101`, fixed kernel
   `[1 4 6 4 1]/16` per axis), scaled by `0.5/0.95` and rounded back to 8U (4.10.0 `:470-477`;
   4.1.0 `:453-460`; 5.0.0 `:485-490`). A pixel stays valid only if the Gaussian-weighted fraction of
   valid pixels in its 5x5 neighborhood is about 0.95 or more, i.e. its whole 3x3 neighborhood is
   valid and at most a small weight of the outer ring is excluded. Image borders do not erode
   (reflection). Consequences (measured, identical on all versions): an isolated pixel, any grid of
   isolated pixels (spacing 2..8), strips 1-4 pixels wide and blocks up to 4x4 have **zero**
   effective support; a k x k block keeps (k-4)^2 pixels; a 5-pixel strip keeps one line.
4. **Gradient masking.** 4.x multiplies both moving-image gradients by the eroded 0/1 float mask
   (`gradientX.mul(preMaskFloat)`, 4.10.0 `:495-496`). 5.0.0 sets them to 0 where the eroded mask is
   0 (`setTo(0, preMask == 0)`, `:494-499`) only when a mask is given. For 0/1 masks these are the
   same values.
5. **Statistics.** `meanStdDev(..., imageMask)`, `subtract(..., imageMask)`, `templateZM` is zeroed
   each iteration (4.10.0 `:541`) and `countNonZero(imageMask)` scales both norms (4.x `:544-545`,
   5.0.0 `validPixels`, `:551-553`).
6. **Mask immutability.** The caller's mask is only read by `threshold`. Measured: the mask, a
   Region's parent, the template and the moving image are unchanged afterwards; a mask aliasing the
   moving image's storage gives the same result as its clone.
7. **4.x vs 5.0.0.** Same algorithm for one-channel 8U masks. Numerical results differ in the last
   bits as for the unmasked path. Outcome differences appear only for marginal supports (see table
   below).

### All-included masks

In 4.x the unmasked path runs the same preprocessing with an all-ones `preMask`. In 5.0.0 the
unmasked path skips it, but an all-ones mask stays all ones after erosion (reflect border), so
zeroing touches nothing. **Measured on all four versions: an all-included mask (values 1, 7, 255 or
arbitrary nonzero) is bitwise identical (nine Float32 coefficients and the Float64 correlation) to
`noArray()`**, for identity and seeded starts (`semantics`, 60 checks per version, 0 violations).
The binding therefore asserts exact equivalence on the same build.

## Insufficient support

When `countNonZero(imageMask) == 0` in an iteration (empty effective support, or no valid moving
pixel under the warped template grid), `meanStdDev` returns mean 0 and deviation 0 (`nz0 ? 1./nz0 :
0`, `mean.dispatch.cpp`), both norms are exactly 0, `templateZM` is all zero so `correlation` is
exactly 0, and `rho = 0/0` is NaN. The existing `cvIsNaN(rho)` guard throws `StsNoConv` "NaN
encountered" (4.10.0 `:572-575`) before `lambda`, the update or another warp. `hessian.inv()` runs
first on the zero Hessian and returns zeros (Task 016 measurement); the value is discarded.

Evidence that this is deterministic, not a hazard:

* Diagnostic `ecc.cpp` copies (4.1.0, 4.10.0, 5.0.0) counted 339-663 zero-support iterations per
  sweep; **every one** ended in the NaN exception, `rho` was never infinite, and no iteration had zero
  norms with a nonzero correlation.
* Randomized sweeps (1200 one-iteration and 600 hundred-iteration calls per version, random
  rectangles, densities, strips, blocks, isolated pixels and border bands, half with in-domain seeds):
  527 / 263 masks with empty effective support, **0 native successes** among them on every version.
* Non-overlap: valid columns 0..23 with a seed shifting the template by +60 or +104 raises (NaN) on
  every version, for 1 and 50 iterations.

The binding therefore rejects only the all-zero mask itself (a clear caller error). It does **not**
replicate the native erosion to reject "nonzero but eroded away" masks: the native failure is
deterministic and translated, and a replica would have to track the Gaussian implementation of each
OpenCV version. Masks with very small effective support (8x8 to 12x12 blocks, 6-8-pixel strips) can
succeed natively with **finite but meaningless** matrices (coefficients up to ~1.8e5, correlation
0.62-0.98). The binding publishes them unchanged, as it does every other finite native result.

Small-support outcomes on a translated 128x128 texture (identity start; NaN = "NaN encountered",
LAM = `lambda_d <= 0`):

| Mask | Effective px | 4.1.0 | 4.6.0 | 4.10.0 | 5.0.0 |
|---|---:|---|---|---|---|
| all zero, single px, grids, strips 1-4, blocks <= 5 | 0-1 | NaN | NaN | NaN | NaN |
| row strip 5 | 128 | OK | OK | OK | OK |
| row strip 6 / 7 / 8 | 256-512 | LAM / OK / LAM | OK / NaN / NaN | OK / NaN / NaN | LAM / OK / OK |
| column strip 6 / 7 / 8 | 256-512 | OK / OK / OK | LAM / OK / LAM | LAM / OK / LAM | OK / LAM / NaN |
| blocks 6, 7, 9 | 4-25 | LAM | LAM | LAM | LAM |
| blocks 8, 10, 12 | 16-64 | OK (wild) | OK (wild) | OK (wild) | OK (wild) |
| block 11 | 49 | LAM | OK | OK | LAM |
| random density 0.5 | ~1 | NaN | NaN | NaN | NaN |
| random density 0.8-0.99, cross | 528-14867 | OK | OK | OK | OK |

These are version-specific numerical outcomes of an ill-posed problem; the binding does not
normalize them.

Other degenerate inputs: constant intensity over the valid support (template and moving) raises
"NaN encountered" on all versions for 1 and 50 iterations; constant images with an all-included mask
raise the same; x-only stripes masked to a horizontal strip succeed with the identity and correlation
0.71 (rank-deficient, as unmasked). If a near-constant support ever produced `rho = ±inf`
(`cvIsNaN` does not catch infinity; Task 011 finding 10), the value is IEEE-defined and the binding's
existing finite-correlation check rejects it at publication.

## Allocation and arithmetic bounds

* Mask-dependent allocations: `preMask` (`hd x wd` 8U), a temporary `preMaskFloat` (`hd x wd` 32F),
  in 5.0.0 a temporary `zeroMask` (`hd x wd` 8U), and `imageMask` (`hs x ws` 8U, allocated in the
  unmasked path too). Sizes depend only on geometry, never on mask values; the area limit (2^24) and
  the 16384 side limit are unchanged.
* `countNonZero` returns `int`; at most 2^24. Norms and products are computed in `double`.
* The mask never indexes memory: `threshold`, `GaussianBlur`, `mul`/`setTo` and `meanStdDev` iterate
  over the matrix geometry; the warp of the mask uses the same clamped `warpPerspective` coordinate
  path reviewed in Tasks 016/017 (INTER_NEAREST variant: `W = W ? 1./W : 0`, coordinates clamped in
  `double` to `[INT_MIN, INT_MAX]` before `saturate_cast`, 4.10.0 `WarpPerspectiveInvoker`, 5.0.0
  `warpPerspectiveBlocklineNN`).
* Non-contiguous Regions: `threshold` (`thresh_8u` row loop when not continuous), `countNonZero` and
  `meanStdDev` (`NAryMatIterator`) all handle steps. Measured: a 128x128 window of a 160x160 parent
  is bitwise equal to its contiguous clone.

Peak RSS, 10 iterations, translated texture, `setNumThreads(1)`, mask = 1/3 x 1/3 excluded rectangle
("full" = all-included mask). The probe owns the caller mask, so the column includes it:

| Geometry | 4.1.0 none / full / masked | 4.6.0 | 4.10.0 (system) | 5.0.0 |
|---|---|---|---|---|
| 640x480 | 42 / 42 / 42 MiB | 41 / 42 / 42 | 55 / 55 / 55 | 43 / 42 / 42 |
| 1920x1080 | 238 / 239 / 240 | 238 / 239 / 239 | 249 / 251 / 252 | 231 / 235 / 234 |
| 4096x4096 | 1863 / 1880 / 1880 | 1864 / 1880 / 1880 | 1876 / 1892 / 1893 | 1817 / 1834 / 1834 |
| 16384x1024 | 1864 / 1880 / 1880 | 1864 / 1880 / 1881 | 1877 / 1893 / 1893 | 1818 / 1834 / 1832 |
| 1024x16384 | 1864 / 1879 / 1880 | 1863 / 1880 / 1879 | 1876 / 1893 / 1893 | 1816 / 1833 / 1833 |

At 2^24 pixels the masked call is +16 MiB, exactly the caller-owned 16 MiB mask: native mask
processing adds no measurable peak (its temporaries are freed before the iterations, which dominate).

## Probes and instrumentation

`tests/cpp/ecc_homography_masked_probe.cpp` (direct OpenCV only; no shim, no Core bridge):
`semantics`, `types`, `support`, `occlusion`, `degenerate`, `sweep`, `mem`.

| Campaign | 4.1.0 | 4.6.0 | 4.10.0 | 5.0.0 |
|---|---|---|---|---|
| Release probe: semantics checks | 60 / 0 violations | 60 / 0 | 60 / 0 | 60 / 0 |
| Release sweep, 1 it (1200): OK / EXC | 407 / 793 | 407 / 793 | 407 / 793 | 406 / 794 |
| Release sweep, 100 it (600): OK / EXC | 156 / 444 | 158 / 442 | 158 / 442 | 159 / 441 |
| **Instrumented OpenCV** sweep 1 it | 407 / 793, 0 reports | 407 / 793, 0 | 407 / 793, 0 | 405 / 795, 0 |
| **Instrumented OpenCV** sweep 100 it | 156 / 444, 0 reports | 158 / 442, 0 | 158 / 442, 0 | 153 / 447, 0 |
| **Instrumented OpenCV** semantics, types, support, occlusion, degenerate | rc 0, 0 reports | rc 0, 0 | rc 0, 0 | rc 0, 0 |
| Diagnostic `ecc.cpp` (sweeps + modes) | 0 traps | (= 4.10.0 source) | 0 traps | 0 traps |

In every sweep: no successful call returned a nonfinite coefficient or correlation, H22 never
changed, and the largest finite coefficient was 1.1e7.

* **Instrumented OpenCV** = the Task 017 source builds of core/imgproc/video for each version
  (`RelWithDebInfo`, `-fsanitize=address,undefined,float-cast-overflow`; `nm -D` shows
  `__asan_report*` and `__ubsan_handle_float_cast_overflow` in `libopencv_imgproc`), run with
  `halt_on_error=1` and leak detection; `ldd` confirmed each probe loaded its `install-<v>-san`
  libraries. This instruments upstream OpenCV itself, separately from the binding sanitizer runs.
* **Diagnostic `ecc.cpp`** = scratch copies of the 4.1.0, 4.10.0 and 5.0.0 `ecc.cpp` (renamed
  entry points, linked against the matching library, not committed) that trap any finite
  double-to-float narrowing above `FLT_MAX` in the Hessian, projections and `lambda`, and count
  zero-support iterations, infinite `rho` and zero-norm/nonzero-correlation iterations. Largest
  finite narrowed value: 2.0e14. Counts are quoted above.

## Occlusion evidence (masking can help; it is not a guarantee)

128x128 texture, moving image = known homography with a rectangle (moving pixels) replaced by a
saturated block, noise, a misleading shifted texture or stripes; mask excludes that rectangle grown
by 3 pixels. Error = worst corner/centre distance to the truth.

| Fixture (4.10.0) | unmasked | masked | same rect in template frame | transposed rect |
|---|---|---|---|---|
| translation (2,1), shifted texture | 7.42 px | 0.074 px | 0.074 px | 8.23 px |
| mild perspective, saturated block | EXC | 0.12 px | 0.10 px | EXC |
| LTP seeded, saturated block | EXC | 0.28 px | 371 px | EXC |
| LTP seeded, shifted texture | 6.62 px | 0.28 px | 2.85 px | 7.84 px |
| scale+perspective seeded, shifted | 8.41 px | 0.43 px | 0.19 px | 14.26 px |

The same ordering holds on 4.1.0, 4.6.0 and 5.0.0 (4.6.0 equals 4.10.0). For small translations a
mask placed in the template frame is almost the same mask, so frame errors are observable only with
larger motion (LTP rows).

## Unproven

1. Memory safety rests on source review plus the measurements above, not a proof. SIMD intrinsic
   paths are hardware-defined and reviewed only.
2. IEC 60559 semantics are relied on as in Tasks 016/017.
3. Windows runs only in the post-merge workflow; it is source-reviewed only here.
