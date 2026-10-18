# ECC translation source contract (Task 011)

Binding target: `cv::findTransformECC(template, moving, warp, MOTION_TRANSLATION,
TermCriteria(COUNT|EPS, n, eps), noArray(), 5)`.

## Sources reviewed

`modules/video/src/ecc.cpp` and `modules/video/include/opencv2/video/tracking.hpp`
were downloaded from the exact upstream tags `4.1.0`, `4.6.0`, `4.10.0`, `5.0.0`
and read for the translation path. `ecc.cpp` at 4.6.0 is byte-identical to 4.10.0.
4.1.0 differs from 4.10.0 only in a comment and in where the 6-argument overload is
defined. 5.0.0 restructures the function as `findTransformECCWithMask`.

These findings come from reading the source and from the measurements below. They
are not a proof of absence of undefined behaviour; the sanitizer campaign (later
milestone) is the empirical check. Only 4.1.0, 4.10.0 and 5.0.0 were executed so far;
4.6.0 is source-reviewed only (identical to 4.10.0) and not yet run.

## Findings

1. **Overload.** The seven-argument `findTransformECC(templateImage, inputImage,
   warpMatrix, motionType, criteria, inputMask, gaussFiltSize)` is `CV_EXPORTS_W` in
   all four `tracking.hpp` files. In 5.0.0 it forwards to
   `findTransformECCWithMask(template, input, noArray(), mask, ...)`.
2. **Direction.** Each iteration warps the moving image with
   `warpAffine(..., map, INTER_LINEAR + WARP_INVERSE_MAP)` into the template frame,
   so the warp maps template coordinates to moving coordinates:
   `moving = template + (tx, ty)`. It is not an inverse warp. Measured: content
   shifted by `(+2,+1)` gives about `(+2.01,+1.01)`.
3. **Layout.** `map` is 2x3 `CV_32FC1`, row-major `[1 0 tx; 0 1 ty]`.
   `update_warping_matrix_ECC` for translation only does `ptr[2] += d[0]; ptr[5] += d[1]`;
   entries 0, 1, 3, 4 are never written.
4. **Identity init.** Upstream replaces an empty warp by `Mat::eye(2,3,CV_32F)`. The
   binding passes its own identity warp and does not rely on that.
5. **Termination.** `rho = -1`, `last_rho = -eps`; loop while
   `i <= maxCount && |rho - last_rho| >= eps`. Reaching `maxCount` without meeting
   `eps` is not an error. **The returned correlation is computed at the start of the
   last iteration, before its update**, so it scores the warp one step earlier than
   the returned translation.
6. **Gaussian / mask.** Both images are `convertTo` float then
   `GaussianBlur(Size(5,5), 0, 0)`. Gradients: `filter2D` with `[-0.5 0 0.5]`. With no
   mask `preMask` is all ones.
7. **Type / geometry checks.** 4.x: equal `type()` and `CV_8UC1` or `CV_32FC1`.
   5.0.0: 1 or 3 channels, equal depth in 8U/16U/32F/64F. Template and moving sizes may
   differ upstream (`hs,ws` vs `hd,wd`); the binding requires equal sizes and 2-D
   `CV_8UC1`. There is no minimum-size check in any version.
8. **Allocations.** About a dozen `hs*ws` float planes (`Xgrid`, `Ygrid`, `templateZM`,
   `templateFloat`, `imageFloat`, `imageWarped`, gradients and warped gradients,
   `error`) plus a `hs x 2*ws` Jacobian. Measured peak RSS at 4096x4096 (2^24 pixels),
   and at 16384x1024 and 1024x16384: **922-983 MB** in all three executed versions.
9. **Integer arithmetic.** The Jacobian width is `2*ws`, at most 32,768 for
   `ws <= 16384`. `project_onto_jacobian_ECC` slices `colRange(i*w,(i+1)*w)` with
   `i < 2`, so `2*w` is the only product. With `hs*ws <= 2^24`, `countNonZero` (`int`)
   and plane sizes are far below `INT_MAX`; `Mat::dot` accumulates in double.
10. **Division / non-finite.** `rho = correlation / (imgNorm * tmpNorm)`. Constant
    images give NaN and upstream throws `StsNoConv "NaN encountered."` through
    `cvIsNaN` only, so an infinite `rho` would not be caught upstream; the binding
    re-checks finiteness. `lambda_d <= 0` throws `StsNoConv`. A very small positive
    `lambda_d` is not guarded: on 4.1.0 and 4.10.0 an unseeded `(6,4)` shift returned
    a **successful** wild result `(44.3, 89.5)`, rho 0.72 (5.0.0 throws). The behaviour of
    `hessian.inv()` on a singular matrix was not separately probed.
11. **4.x vs 5.0.** Same algorithm and constants for one channel; numeric results
    differ slightly (below).
12. **Small images.** `1x1`..`2x2` throw on all versions; `3x3` and `4x4` succeed on
    4.x and throw on 5.0.0; `5x5`, `6x6`, `8x8` throw; `16x16` and above succeed on all
    versions with a larger bias (about 1.03 for a true shift of 1).

## Derived envelope (binding policy, not an OpenCV guarantee)

| Quantity | Limit | Reason |
| --- | --- | --- |
| Rows, columns | 32 .. 16,384 | Below 32 results differ between versions (finding 12); 16,384 keeps `2*ws` <= 32,768. |
| Pixel area | <= 16,777,216 (2^24) | Memory/time policy: about 1 GB measured at the limit; 4K frames fit. |
| `Maximum_Iterations` | 1 .. 100 | Binding policy. |
| `Epsilon` | finite, 1e-8 .. 0.1 | Binding policy; zero or negative would disable the stop test. |
| Gaussian filter size | fixed 5 | Not exposed. |

The area limit is a memory/time policy; the arithmetic limit is far higher. No
ECC-specific unbounded memory-safety defect was found in the source. This does not
rule out defects in shared imgproc code (`warpAffine`, `GaussianBlur`, `filter2D`);
the Farneback UB acceptance does not extend to ECC.

## Measured behaviour (128x128 analytic texture, defaults 50 / 1e-4)

Probe: `tests/cpp/ecc_probe.cpp` (OpenCV only; no shim, no Core bridge).

| Case | 4.1.0 = 4.10.0 | 5.0.0 |
| --- | --- | --- |
| identity | (0,0), rho 1.0 | (0,0), rho 1.0 |
| (+2,+1) | (2.0115, 1.0076), rho 0.99967 | (2.0089, 1.0049) |
| (-2,-1) | (-2.0084, -0.9925) | (-2.0074, -1.0050) |
| (3,0) | (2.9923, 0.0133) | (3.0078, 0.0002) |
| (0,3) | (0.0109, 3.0032) | (0.0014, 3.0080) |
| contrast 0.8x+20 | (2.0139, 1.0067) | (2.0095, 1.0048) |
| constant vs constant | `StsNoConv` NaN | same |
| noise vs noise | `StsNoConv` | same |
| (12,7), (40,30) | `StsNoConv` | same |
| (6,4) | success (44.3, 89.5), rho 0.72 | `StsNoConv` |
| independent noise vs texture | success (-4.27,-0.64), rho 0.034 | similar |
| x-only stripes, shift 3 | success (0,0), rho 0.38 | same |
| Region vs contiguous clone | bitwise equal | bitwise equal |
| reversed roles (2,1) | negated | negated |

Geometry check: `warpAffine(moving, estimate, WARP_INVERSE_MAP)` reduces the interior
mean absolute difference for `(2,1)` from 13.68 to 0.000, and a negated translation
leaves 25.9. A finite result is not proof of alignment (stripes, noise, 4.x `(6,4)`).
Photometric variants do not reach zero residual because the images differ in intensity.

Test tolerances chosen from these runs: displacement 0.05 px at 128x128 (worst
measured error 0.0173); `rho` is never thresholded by the binding.

## Follow-up (Task 021, 2026-10-18): native `warpAffine` arithmetic

The findings above remain the Task 011 record, and its qualification results stand. They
established ECC's own integer arithmetic, memory envelope and numerical behavior. They did
**not** audit the shared `warpAffine` fixed-point arithmetic that native ECC calls four
times per iteration. The sanitizer runs instrumented only the binding, not OpenCV.

[Task 021](tasks/021-translation-euclidean-safety-audit.md) audited that path at operand
level:

* **OpenCV 4.x signed-overflow site.** The 4.x `WarpAffineInvoker` adds `X0 + adelta[x]` in
  signed `int`. For a translation warp `[1 0 tx; 0 1 ty]` that addition overflows when `tx`
  lies in a narrow band just below `2^21` (about `2.0808e6 .. 2.097e6` at 16383 columns).
* **Confirmed for a model-valid matrix.** UBSan confirmed the overflow on instrumented 4.1.0,
  4.6.0 and 4.10.0 by direct replay of `[1 0 2090000; 0 1 0]` (Level A).
* **Not reproduced through ECC.** No native ECC run (Level B) or call to
  `opencv_video_estimate_ecc_translation` (Level C) reached it. The largest accumulated
  `|t|` over 660 bounded translation trials (four versions) was 3274.
* **OpenCV 5.0.0.** It routes ECC's types to float-coordinate kernels and does not execute
  this addition.

**Disposition: `SAFETY_GATE_NOT_ESTABLISHED`.** Nothing bounds the optimizer's accumulated
translation, so the band is not excluded. It is also not shown reachable, so the API is not
declared unsafe. The phrase "the arithmetic limit is far higher" above applies to ECC's own
`2*ws` / `countNonZero` arithmetic only. It does not apply to `warpAffine` coordinates.

## Windows MSYS2 provisioning correction

The Task 010 post-merge Windows run failed in "Install MSYS2 native dependencies":
`pacman -Sy` partial upgrade left `libgcc`/`libstdc++` 16.2.0-5 against `cc-libs`
requiring 16.2.0-4. The workflow now runs `pacman -Syu --noconfirm` first and uses
`-Syu` for the install. The setup-alire source I read only toggles
`msys2.do_not_install`; it performs no package upgrade. This is not verified on a
Windows runner; only the post-merge run can show it. A first `-Syu` may update the
MSYS2 runtime and require a restart; if the post-merge run shows that, a further step
is needed.
