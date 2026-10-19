# OpenCV Video for Ada

## Safety notice: affine ECC native signed-overflow risk (Task 020, 2026-10-18)

`Estimate_ECC_Affine` (both overloads: identity-initialized, Task 014, and caller-seeded,
Task 015) is **not qualified as free of native undefined behavior**. Disposition:
`KNOWN_NATIVE_UB_UNMITIGATED`. Both APIs stay available and unchanged.

* Native affine ECC can generate a finite `CV_32F` intermediate warp whose inverse-map
  `warpAffine` evaluation performs a signed 32-bit integer addition that overflows. The
  addition is in the OpenCV 4.x fixed-point path (`X0 + adelta[x]`, `Y0 + bdelta[x]`).
  OpenCV 5.0 keeps the legacy fixed-point `warpAffine` implementations, and their block-line
  functions are annotated `CV_DISABLE_UBSAN`. On the tested x86_64 5.0.0 build (default AVX2
  dispatch), affine ECC's image types (`CV_32FC1` linear, `CV_8UC1` nearest) went to newer
  floating-point-coordinate kernels instead, so the legacy overflowing addition was not
  executed in those tests. This does **not** establish 5.0 native safety: the new kernels'
  float-to-integer conversions for extreme coordinates (including the UBSan-suppressed
  `cvFloor(float)`) and other architecture, HAL and hardware paths remain unqualified.
* **The seeded operation is implicated through the real binding.** A single call to
  `opencv_video_estimate_ecc_affine_seeded` (the C export used by the seeded Ada overload),
  with a seed inside the published seed domain and a 97x64 texture pair, reaches that overflow.
  ECC generates the bad matrix at its third iteration; UBSan reports it on instrumented
  OpenCV 4.1.0. The call then returns an ordinary `OpenCV_Error`, so the caller sees no sign
  that undefined behavior happened.
* The identity-initialized operation was **not reproduced** as reaching the overflow
  (`NOT_REPRODUCED`, not `SAFE`). It calls the same native optimizer and `warpAffine` path,
  and no binding-side argument excludes it.
* The binding validates the final result only. Checking the output cannot repair an unsafe
  intermediate: the undefined operation has already happened inside OpenCV before control
  returns. The binding's seed bounds do not prevent it either, because the optimizer's
  later iterates are not bounded by the seed.
* Sanitizer-clean binding tests prove nothing about upstream native safety. The repository's
  ASan/UBSan runs instrument the shim and tests, not the prebuilt OpenCV libraries. Ordinary
  images usually converge without reaching the overflow; that does not make the API safe.
* Safety-sensitive consumers should not treat affine ECC as qualified for undefined behavior
  until a mitigation is established. The candidate mitigations, the reproducer and the owner
  decision are in [docs/tasks/020-existing-affine-safety-audit.md](docs/tasks/020-existing-affine-safety-audit.md)
  and [docs/task020-qualification.md](docs/task020-qualification.md). The original native
  finding is in [docs/ecc-affine-masked-source-contract.md](docs/ecc-affine-masked-source-contract.md).

Translation and Euclidean ECC go through the same non-homography `warpAffine` branch but were
not audited at operand level by Task 020. Homography ECC uses `warpPerspective` and is not
implicated by this finding.

## Safety notice: translation and Euclidean ECC native arithmetic (Task 021, 2026-10-18)

`Estimate_ECC_Translation` and both `Estimate_ECC_Euclidean` overloads (identity and
caller-seeded) are **not qualified as free of native undefined behavior**. Each has the
independent disposition `SAFETY_GATE_NOT_ESTABLISHED`. Unlike affine, they are **not**
shown to reach undefined behavior. All three APIs stay available and unchanged.

* **The OpenCV 4.x defect also covers these models.** The fixed-point `warpAffine`
  addition that overflows for affine ECC also overflows for a **model-valid** translation
  warp (`[1 0 2090000; 0 1 0]`) and a model-valid rigid Euclidean warp (θ = 0.5 rad,
  translation 2,090,000 px).
* **What was confirmed.** UBSan confirmed these overflows on instrumented OpenCV 4.1.0,
  4.6.0 and 4.10.0 by direct native replay. Bounded rotation coefficients do not prevent
  them: the overflowing operand carries the accumulated translation, and the overflow band
  is a translation just below 2^21 pixels.
* **Row-base rounding (Task 021-R1 correction).** OpenCV builds each row's fixed-point base
  as `cvRound(...) + round_delta` in signed `int`, and that addition can itself overflow,
  before any per-column delta is applied. A zero Y column delta does **not** make the Y
  coordinate safe. It was confirmed by direct replay at an ordinary 64x97 image for the
  model-valid translation `[1 0 0; 0 1 2097151.875]` (nearest-neighbour mask warp; 4.1.0,
  4.6.0, 4.10.0). For Euclidean, a rotation also exposes a route that has no upper bound in
  translation beyond 2^21. The dispositions are unchanged.
* **Not reproduced through ECC.** Neither native ECC (iteration-capped traces) nor the
  real C exports reached that band on any version (`NOT_REPRODUCED`, not `SAFE`). The
  largest accumulated translation over 1,980 bounded, deterministic trials was about
  5,500 px. A seed near the `2^20` limit made native ECC stop with an ordinary "NaN
  encountered" error.
* **Why the gate is not established.** Nothing in native ECC bounds the accumulated
  translation. The Euclidean seed bound (`|t| <= 2^20`) constrains only the first iterate.
* **OpenCV 5.0.0.** The tested 5.0.0 build sends these image types to float-coordinate
  kernels. It did not execute the overflowing addition for any finite warp, but a directly
  replayed **NaN** translation warp produced an undefined out-of-range pointer computation
  in that kernel. No ECC run was shown to produce a NaN warp. Coordinates beyond int32 and
  other architecture, HAL and hardware paths remain unqualified.
* **Details.** See [docs/tasks/021-translation-euclidean-safety-audit.md](docs/tasks/021-translation-euclidean-safety-audit.md)
  and [docs/task021-qualification.md](docs/task021-qualification.md).

## Safety notice: OpenCV 5.0 floating-point warp kernels (Task 022, 2026-10-19)

> **R1 correction (2026-10-19): ordinary ECC use is affected on OpenCV 5.0.** The original
> notice below said the 5.0 hazard needed coordinates near `2^31` and was not reached by ECC.
> **That is superseded.** On the tested OpenCV 5.0.0 x86_64 build, **all five** ECC operations
> (`Estimate_ECC_Translation`, both `Estimate_ECC_Euclidean` overloads, both
> `Estimate_ECC_Affine` overloads) were observed, through the real binding with ordinary
> inputs, forming source pointers **outside the source allocation**. This is undefined behavior
> in C++. Each is now **`KNOWN_NATIVE_UB_UNMITIGATED` on OpenCV 5.0**.
>
> * **Cause.** OpenCV's scalar remainder loop computes `src + srcstep*iy + ix` *before* its
>   bounds check, for every pixel it handles. One pixel just past the image edge is enough. On
>   the 97×64 translation fixture with a `(2,1)` shift, pixel `(96,63)` maps to source
>   `(98,64)`, an element offset of 6306. That is 96 elements past the end of the real
>   allocation, with an ordinary int32 coordinate.
> * **Successful results are affected.** The translation, Euclidean and affine calls returned
>   status OK with correct alignments while forming these pointers. The seeded calls form them
>   on their first warp, from seeds inside the published domain.
> * **No memory is read out of bounds.** The bounds check still routes these pixels to the
>   border, so ASan is clean. UBSan is also silent, because the address does not wrap. The
>   finding comes from source analysis, the measured allocation bounds and gdb captures of the
>   actual pointer register, not from a sanitizer report.
> * **Which widths are hit.** It occurs whenever the image width leaves a scalar remainder:
>   width not a multiple of 16 for AVX2, or of 8 for the baseline path. Widths that are
>   multiples avoid this loop. That is an observation, not a supported mitigation.
> * **Scope.** OpenCV 5.0.0 on x86_64 (AVX2 and baseline dispatch) only. The 4.x dispositions
>   above (Task 020, Task 021) are unchanged. All APIs stay available and unchanged. A fix
>   needs a separate, owner-approved task.
> * **Int32 boundary corrected.** "Magnitude `2^31` or more" below is inexact: `-2^31` is
>   representable. The valid range is `-2^31 <= v < 2^31`.
>
> Details: [task note section R1](docs/tasks/022-opencv5-float-warp-safety-audit.md#r1--ordinary-out-of-allocation-scalar-pointer-formation-2026-10-19)
> and [qualification Part R1](docs/task022-qualification.md#part-r1--ordinary-coordinate-pointer-formation-2026-10-19).

*Original Task 022 notice (history; its exposure conclusions are superseded on 5.0 by R1 above):*

On OpenCV 5.0.0, `Estimate_ECC_Translation`, both `Estimate_ECC_Euclidean` overloads and both
`Estimate_ECC_Affine` overloads are **not qualified as free of native undefined behavior**.
Each has the disposition `SAFETY_GATE_NOT_ESTABLISHED` on 5.0. All APIs stay available and
unchanged. This notice **adds to** the Task 020 and Task 021 notices above; it does not replace
or weaken them, and the Task 020 affine disposition (`KNOWN_NATIVE_UB_UNMITIGATED`) is
unchanged.

* **Finite warps are affected, not only NaN.** Task 021 found the 5.0 float kernel unsafe only
  for a NaN warp. Task 022 shows the **same** undefined operation for any **finite** warp that
  maps a pixel to a coordinate of magnitude `2^31` or more, including **model-valid**
  translation, rigid Euclidean and general affine matrices (for example a translation of
  `3e9` pixels).
* **What happens.** Both kernels ECC uses (`warpAffineLinearInvoker_32FC1` for the image and
  gradients, `warpAffineNearestInvoker_8UC1` for the mask) convert the Float32 coordinate with a
  UBSan-suppressed `cvFloor`/`cvRound`. On x86 this gives an `INT_MIN` sentinel, and the scalar
  tail then forms an out-of-object source pointer **before** its bounds check. The bounds check
  still sends the pixel to the border, so no out-of-bounds read was observed (ASan clean). The
  pointer arithmetic itself is undefined behavior, which a later check cannot undo.
* **Confirmed by direct replay only.** UBSan confirmed it on an instrumented OpenCV 5.0.0
  (x86_64), in both AVX2 and baseline dispatch, for both kernels (Level A).
* **Not reproduced through ECC.** No native ECC trajectory or real binding call came close: the
  largest coordinate any ECC run produced was about `3.4e7`, roughly 64× below `2^31`
  (`NOT_REPRODUCED`, not `SAFE`). Nothing in native ECC bounds its later iterates, and the
  four warps run **before** ECC's own NaN/convergence checks, so an error return does not prove
  the unsafe path was avoided.
* **Seeds are not the exposure.** The published seed domains keep the first iterate about 1800×
  below the threshold; only later optimizer iterates could reach it.
* **Platform scope.** Only x86_64 AVX2 and cpu_baseline paths were executed. NEON/AArch64, RVV,
  LASX, IPP, OpenCL, custom HAL, Windows and macOS are unqualified. On ARM the conversion may
  behave differently (an unverified NaN-to-0 hypothesis could turn this into a silent in-image
  read).
* **Details.** See [docs/tasks/022-opencv5-float-warp-safety-audit.md](docs/tasks/022-opencv5-float-warp-safety-audit.md)
  and [docs/task022-qualification.md](docs/task022-qualification.md).

### Moving-image-masked ECC homography alignment (Task 018)

```ada
Alignment : constant OpenCV.Video.ECC_Homography_Result :=
  OpenCV.Video.Estimate_ECC_Homography
    (Template_Image    => Previous_Image,
     Moving_Image      => Current_Image,
     Moving_Mask       => Valid_Moving_Pixels,
     Initial_Transform => Predicted_Homography);
```

A complete identity-initialized example, excluding a region of the current frame that the caller
already knows is unreliable (for example a moving vehicle or a lens obstruction):

```ada
with OpenCV.Core;
with OpenCV.Video;

procedure Align_With_Mask
  (Previous_Image, Current_Image : OpenCV.Core.Mat;
   Alignment                     : out OpenCV.Video.ECC_Homography_Result)
is
   --  One byte per CURRENT (moving) image pixel; same rows and columns as Current_Image.
   Valid_Moving_Pixels : OpenCV.Core.Mat :=
     OpenCV.Core.Create
       (Current_Image.Rows, Current_Image.Columns, (Depth => OpenCV.Core.UInt8, Channels => 1));
begin
   Valid_Moving_Pixels.Set_To ((others => 255.0));   --  every pixel eligible
   declare
      --  Exclude columns 400 .. 559, rows 120 .. 279 of the CURRENT image (caller policy).
      --  The Region shares the mask's storage.
      Excluded : OpenCV.Core.Mat :=
        Valid_Moving_Pixels.Region ((X => 400, Y => 120, Width => 160, Height => 160));
   begin
      Excluded.Set_To ((others => 0.0));
   end;

   --  Raises OpenCV.OpenCV_Error for an invalid mask, native nonconvergence or no remaining
   --  eligible support; no result is fabricated.
   Alignment := OpenCV.Video.Estimate_ECC_Homography
     (Template_Image => Previous_Image,
      Moving_Image   => Current_Image,
      Moving_Mask    => Valid_Moving_Pixels);
end Align_With_Mask;
```

Both overloads call the common seven-argument `cv::findTransformECC` with `MOTION_HOMOGRAPHY`,
`COUNT | EPS`, Gaussian size 5 and the caller's mask as the native **input (moving-image) mask**.
There is no template mask, and OpenCV 5.0's `findTransformECCWithMask` is not used. Matrix
convention, H22 rule, options, output checks and result type are those of Tasks 016/017; the seeded
overload keeps the Task 017 seed domain unchanged.

* **Coordinate frame.** `Moving_Mask (row, column)` describes `Moving_Image (row, column)`. The mask
  is not a second transform: native ECC warps it with the moving image (nearest neighbour,
  `WARP_INVERSE_MAP`), so a template pixel contributes only if the moving pixel it maps to is
  eligible. A mask drawn in template coordinates is a different mask; for large motion it gives
  wrong results (qualified by tests with asymmetric, transposed and template-frame masks).
* **Values.** Zero excludes; any nonzero value includes. 1, 255 and other nonzero values give
  bitwise identical results. An all-included mask gives exactly the unmasked result, on every
  qualified version.
* **Valid input domain** (checked in Ada and again in C before any native call; violations raise
  `OpenCV_Error`, nothing is converted or clamped): nonempty 2-D UInt8 C1 Mat; exactly the moving
  image's rows and columns; at least one nonzero pixel. Regions (including non-contiguous ones) are
  accepted. The images keep the Task 011 envelope (32 .. 16384 per side, at most `2**24` pixels).
* **Native mask processing.** OpenCV binarizes the mask and erodes it with a 5 x 5 Gaussian
  (`0.5/0.95` threshold): a pixel stays eligible only if essentially its whole 3 x 3 neighbourhood is
  eligible. Isolated pixels, pixel grids, strips up to 4 pixels wide and blocks up to 4 x 4
  contribute nothing. Moving-image gradients are masked as well.
* **Insufficient support.** If no eligible pixel remains under the warped template (eroded away,
  or no overlap after warping), native ECC raises "NaN encountered"; the binding propagates
  `OpenCV_Error` and never fabricates a result. Very small surviving support (for example a 12 x 12
  block) can also return a **finite but meaningless** matrix with a high correlation; it is
  published unchanged. The outcome of such marginal masks differs between OpenCV versions.
* **Ownership.** The mask, both images, all Region parents and `Initial_Transform` are never
  modified. The result is a plain Ada value with no native resource.
* **Why masking helps, and its limits.** Excluding pixels that do not follow the scene motion removes
  them from the correlation, the Hessian and the update. In the oracle's occlusion fixtures a
  misleading occluder moved the unmasked estimate by 1.7-5.6 pixels at interior reference points
  (or made native ECC raise), while the masked estimate stayed within 0.02-0.15 pixels, on all four
  qualified versions. A mask never guarantees convergence or correctness: it
  reduces the available signal, can remove the texture that constrains the motion, and cannot repair
  a wrong seed. Choosing what to exclude is the caller's policy; the binding does not detect moving
  objects, occlusions or unreliable pixels.
* **Correlation and nonconvergence.** The correlation is the native value over the eligible pixels
  only (not comparable with an unmasked correlation) and is not a confidence. Nonconvergence raises.
* **Memory.** Mask processing adds no measurable peak beyond the caller's mask itself: about 1.88 GiB
  at `2**24` pixels, versus 1.86 GiB unmasked (the difference is the 16 MiB caller mask).

**Safety gate.** `SAFETY_GATE_ESTABLISHED` was recorded (commit `f9d3db9`) before any production
code, from a four-version source review, direct-native probes, OpenCV builds instrumented with
ASan/UBSan/`float-cast-overflow` and diagnostic copies of `ecc.cpp`.

Production ABI: 25 exports. The new exports are `opencv_video_estimate_ecc_homography_masked` and
`opencv_video_estimate_ecc_homography_masked_seeded` (one extra `const opencv_core_mat_handle *`;
48-byte result reused, padding bytes 36-39 zero on success, caller object untouched on failure).
AUnit has 198 registrations. See the [task note](docs/tasks/018-masked-ecc-homography.md), the
[source contract](docs/ecc-homography-masked-source-contract.md) and the
[qualification](docs/task018-qualification.md). The new code has not yet run on Windows.

### Caller-seeded ECC homography alignment (Task 017)

```ada
Refined : constant ECC_Homography_Result :=
  OpenCV.Video.Estimate_ECC_Homography
    (Template_Image    => Previous_Image,
     Moving_Image      => Current_Image,
     Initial_Transform => Predicted_Homography);
```

The same native `findTransformECC` (`MOTION_HOMOGRAPHY`, `COUNT | EPS`, no mask, Gaussian size 5)
as Task 016, but the private 3x3 Float32 warp starts from `Initial_Transform.Warp` instead of the
identity. The matrix convention, output checks and result type are Task 016's.

* **Storage order.** The nine seed coefficients are placed in row-major storage order `H00 .. H22`,
  not the native update-vector order. Nothing is transposed, negated, inverted, normalized,
  decomposed or composed.
* **`Initial_Transform.Correlation` is ignored.** It is never read, validated or propagated.
* **Seed domain (binding policy, not an OpenCV guarantee).** Checked after the image and option
  checks, before any native call; a violation raises `OpenCV_Error` and is never clamped:
  * every coefficient finite;
  * `H22` exactly `1.0` (native ECC never updates H22 and its Jacobian assumes 1, so a seed with
    another scale is rejected rather than normalized);
  * `H00 H01 H10 H11` at most 4 in magnitude, `H02 H12` at most `2**20` pixels;
  * `|H20| * (Columns - 1) + |H21| * (Rows - 1) <= 0.5`, so the seed's denominator stays within
    `0.5 .. 1.5` over the template. This depends on the image size: on 128 x 128 it allows a
    projective term of about `3.9e-3`.
* **The domain is not an output policy.** Native ECC updates the matrix without constraint; about a
  third of measured one-iteration successes left the seed domain. A returned result (seeded or not)
  is published as Task 016 publishes it, and passing an out-of-domain result back as a seed raises.
* **Identity seed equivalence.** The identity seed and the default record give a byte-identical
  result to the identity-initialized overload, including the same native failures.
* **Seed consumption.** Identity initialization raises on strong perspective (`H20 = H21 = 0.002`),
  a 40/30-pixel shift, a reflection, a 15% scale with perspective and a 0.15 shear with perspective
  on every qualified version; a nearby seed converges on the same pairs.
* **Nonconvergence and wrong results.** Constant images, noise, wrong-direction or insufficiently
  close seeds and singular seeds raise. A finite result can still be wrong (for example a wrong seed
  with one iteration, or an extreme anisotropic seed); no correlation threshold hides it.
* **Result value.** Plain value, no native ownership; Regions are accepted (the seed domain uses the
  Region's own size); inputs and the initial record are never modified. It is an image-alignment
  matrix, not a camera pose or navigation estimate.

**Safety gate.** `SAFETY_GATE_ESTABLISHED` was recorded before the API commit. Every native path that
consumes an unconstrained warp was reviewed on 4.1.0, 4.6.0, 4.10.0 and 5.0.0 and exercised against
OpenCV builds instrumented with ASan/UBSan and `float-cast-overflow`, with no reports. The gate relies
on IEC 60559 floating point for division by zero and for out-of-range double-to-float conversion.

Production ABI: 23 exports. The new export is `opencv_video_estimate_ecc_homography_seeded` (nine
`float` seed coefficients by value, 48-byte result reused). AUnit has 186 registrations. See the
[task note](docs/tasks/017-seeded-ecc-homography.md), the
[source contract](docs/ecc-homography-seeded-source-contract.md) and the
[qualification](docs/task017-qualification.md). The new code has not yet run on Windows.

### ECC eight-parameter homography alignment (Task 016)

```ada
Alignment : constant ECC_Homography_Result :=
  OpenCV.Video.Estimate_ECC_Homography
    (Template_Image => Previous_Image,
     Moving_Image   => Current_Image);
```

This calls native `cv::findTransformECC` with these settings:
* `MOTION_HOMOGRAPHY`;
* a privately identity-initialized 3x3 Float32 warp;
* `COUNT | EPS` termination;
* no mask;
* Gaussian size 5.

`ECC_Homography_Options` is a subtype of `ECC_Translation_Options`. The returned matrix maps
**template coordinates into moving coordinates**, in pixels, with X right and Y down:

```text
[ H00 H01 H02 ]      D        = H20*x + H21*y + H22
[ H10 H11 H12 ]      moving_x = (H00*x + H01*y + H02) / D
[ H20 H21 H22 ]      moving_y = (H10*x + H11*y + H12) / D
```

* **Eight parameters, nine coefficients.** Native ECC optimizes eight parameters and never writes
  H22, so H22 is exactly 1 for this identity-initialized call. Any other value is reported as
  `OpenCV_Error`. The coefficients are returned in row-major storage order, not the native
  update-vector order.
* **Projective denominator.** The mapping is undefined where `D = 0` and extreme near it. The
  binding does not resample, so it neither detects nor alters such points.
* **Resampling.** To resample the moving image into the template frame, apply the same matrix with
  `WARP_INVERSE_MAP`. The binding never inverts, transposes, normalizes or decomposes it.
* **Correlation.** This is the native value, computed before the final native update. It is not a
  confidence score.
* **Errors and wrong results.** Native nonconvergence raises `OpenCV_Error`, for example on constant
  images, uncorrelated noise, strong perspective, a near-zero or zero-crossing denominator, a large
  translation or a reflection. A finite result can still be wrong: noise against a texture
  (all versions), and low-gradient or ill-conditioned pairs on 4.6.0 and 4.10.0. No correlation threshold
  hides these cases, and a finite result does not guarantee correct registration.
* **Small basin.** From identity, convergence covers about 2% scale or shear, 2 degrees of rotation,
  a few pixels of translation and perspective terms of about `2e-4` at 128 px. Use the Task 017
  seeded overload for larger motion with a prediction.
* **Homography versus affine.** On five measured perspective fixtures, the homography interior
  residual (0.35-0.36 gray levels) was 2.2-4.2 times lower than affine ECC's (0.77-1.49), on 4.10.0.
  Homography does not always outperform affine; on pure affine motion the two are equal.
* **Limits.** These are the Task 011 envelope: 2-D UInt8 C1, 32 .. 16,384 per side, at most `2**24`
  pixels, 1 .. 100 iterations, epsilon 1e-8 .. 0.1. The measured native peak at `2**24` pixels was
  about 1.9 GiB on 4.10.0, including a 512 MiB Jacobian. This is a measurement, not a limit, and
  validation does not guarantee the memory is available.
* **Result value.** The result is a plain value with no native ownership. Regions are accepted, and
  inputs are never modified. It is an image-alignment matrix, not a camera pose, attitude,
  geographic mapping or navigation estimate.

The native safety gate (four-version source review of the unguarded projective-denominator path)
was established before implementation. Upstream OpenCV is not sanitizer-instrumented by these
tests.

Production ABI: 22 exports. The new export is `opencv_video_estimate_ecc_homography`, with a 48-byte,
8-aligned result. AUnit has 176 registrations. See:
* the [task note](docs/tasks/016-ecc-homography.md);
* the [source contract](docs/ecc-homography-source-contract.md);
* the [qualification](docs/task016-qualification.md).

The new code has not yet run on Windows; that happens in the push-to-main Windows workflow.

### Caller-seeded ECC affine alignment (Task 015)

```ada
Refined : constant ECC_Affine_Result :=
  OpenCV.Video.Estimate_ECC_Affine
    (Template_Image    => Previous_Image,
     Moving_Image      => Current_Image,
     Initial_Transform => Predicted_Affine);
```

The same native `findTransformECC` (`MOTION_AFFINE`, `COUNT | EPS`, no mask, Gaussian size 5) as
Task 014, but the private 2x3 Float32 warp starts from `Initial_Transform.Warp` instead of the
identity. A previous `ECC_Affine_Result` may be passed directly.

* **Row-major storage order.** The seed is placed in native matrix storage order `M00 M01 M02 M10
  M11 M12`. This is *not* the native update-vector order (`M00 M10 M01 M11 M02 M12`) used inside the
  optimizer. Nothing is transposed, negated, inverted, decomposed or composed; the seed uses the
  same template-to-moving convention as the result.
* **`Initial_Transform.Correlation` is ignored.** It is never read, validated, propagated or used as
  a threshold, so even a nonfinite value changes nothing. The returned correlation is always the
  new native value (computed before the final update).
* **No implicit composition.** The seed is a prediction for *this* image pair. A previous pair's
  result is not automatically the right transform for the next pair.
* **Seed domain (binding policy, not an OpenCV guarantee).** `M00 M01 M10 M11` finite with absolute
  value at most 4; `M02 M12` finite with absolute value at most `2**20` pixels. Violations raise
  `OpenCV_Error` and are never clamped. Singular, near-singular and reflected seeds pass validation
  because they are not unsafe; they usually make native ECC raise or return a poor finite matrix.
  An accepted seed does not imply convergence or correct alignment.
* **Identity seed equivalence.** The identity seed (and the default `others => <>` record) gives a
  byte-identical result to the identity-initialized overload, including the same native failures.
* **Seed consumption.** Identity initialization raises on a 0.12 shear, a 0.2 shear, a reflection,
  a 12% anisotropic scale and an up-to-20% strong deformation on every qualified version; a
  near-correct affine seed converges on the same pairs, so the seed is genuinely consumed.
* **Nonconvergence** (constant, noise, deliberately wrong seeds) raises; a finite result can still
  be wrong, e.g. a poor seed can end at a low-correlation matrix.
* The result is a plain value; Regions are accepted, inputs and the initial record are never
  modified.

Production ABI: 21 exports (`opencv_video_estimate_ecc_affine_seeded`, result structure reused,
size 32 bytes). AUnit has 163 registrations. See the [task note](docs/tasks/015-seeded-ecc-affine.md),
the [source contract](docs/ecc-affine-seeded-source-contract.md) and the
[qualification](docs/task015-qualification.md). The accepted upstream Farnebäck pointer-arithmetic
finding does not extend to ECC.

### ECC six-parameter affine alignment (Task 014)

```ada
Alignment : constant ECC_Affine_Result :=
  OpenCV.Video.Estimate_ECC_Affine
    (Template_Image => Previous_Image,
     Moving_Image   => Current_Image);
```

Native `cv::findTransformECC` with `MOTION_AFFINE`, an identity-initialized private warp, `COUNT | EPS`
termination (`ECC_Affine_Options` is a subtype of `ECC_Translation_Options`), no mask and Gaussian
size 5. The returned matrix is the native 2x3 warp, in pixels with X right and Y down, mapping
**template coordinates into moving coordinates**:

```text
[ M00 M01 M02 ]      moving_x = M00 * template_x + M01 * template_y + M02
[ M10 M11 M12 ]      moving_y = M10 * template_x + M11 * template_y + M12
```

* **Full matrix, no decomposition.** All six coefficients are returned as `Float32_Value`; a general
  affine matrix (scale, shear, rotation, reflection) has no unique rotation/scale/shear split, so
  none is imposed. Apply the same matrix with `WARP_INVERSE_MAP` to resample the moving image into
  the template frame; do not transpose, negate or invert it.
* **Unlike Euclidean ECC** (Task 012) the linear part is not required to be a rotation: no
  orthogonality or determinant test is applied, so scale, shear and finite singular matrices are
  returned as the native optimizer produced them. Only a malformed warp or a nonfinite coefficient
  or correlation is an error. Affine ECC updates its six coefficients directly (no `asin` rule).
* **Correlation** is the native value, computed before the final update (it scores the previous
  matrix). Native nonconvergence raises `OpenCV_Error`; a returned matrix is not proof of
  registration (noise against a texture returns a finite wrong matrix with low correlation).
* **Convergence basin.** Identity-initialized: small transforms (about 1-3% scale, a degree or
  three of rotation, a few pixels of translation, ~0.03 shear) converge; reflection, 0.2 shear
  and 20% scale change throw on every qualified version.
* **Limits** are the Task 011 envelope (2-D UInt8 C1, 32 .. 16,384 per side, at most 2**24 pixels,
  iterations 1 .. 100, epsilon 1e-8 .. 0.1). Peak memory at 2**24 pixels was measured at about
  1.2-1.3 GB (see the task note); validation does not guarantee the memory is available.
* The result is a plain value; Regions are accepted and inputs are never modified. It is an
  image-alignment matrix, not a geographic mapping, camera pose, yaw or navigation estimate.

Production ABI: 20 exports (`opencv_video_estimate_ecc_affine`, result size 32 bytes). AUnit has 153
registrations. See the [task note](docs/tasks/014-ecc-affine.md), the
[source contract](docs/ecc-affine-source-contract.md) and the
[qualification](docs/task014-qualification.md). Task 013's post-merge Windows run (38197543379)
completed successfully on `e16a00c`; Windows remains push-to-main only. The accepted upstream
Farnebäck pointer-arithmetic finding does not extend to ECC.

### Caller-seeded ECC Euclidean alignment (Task 013)

```ada
Refined : constant ECC_Euclidean_Result :=
  OpenCV.Video.Estimate_ECC_Euclidean
    (Template_Image    => Previous_Image,
     Moving_Image      => Current_Image,
     Initial_Transform => Predicted_Transform);
```

One new overload: the private native 2x3 Float32 warp starts from `Initial_Transform` instead
of the identity, and native `cv::findTransformECC` (`MOTION_EUCLIDEAN`, same criteria, no mask,
Gaussian size 5) refines it. Options still precede the seed and default. The Task 012
identity-initialized function and Task 011's translation API are unchanged.

* **Only `Displacement` and `Rotation_Radians` are used.** `Initial_Transform.Correlation` is
  ignored: it is not an input, not a threshold, never validated and never propagated, so a
  previous `ECC_Euclidean_Result` can be passed back directly. The seed is never modified.
* **Direction and origin.** The seed is a template-to-moving matrix with the Task 012 convention
  (`moving = R(t) * template + Displacement`, X right, Y down), rotating about the image origin
  `(0,0)`, not the centre. A centred rotation needs the compensating matrix translation
  (`tx = cx - (cos t * cx - sin t * cy) + dx`, `ty = cy - (sin t * cx + cos t * cy) + dy`);
  `Displacement` is never reinterpreted as the centre displacement. No transforms are composed,
  and the matrix is applied with `WARP_INVERSE_MAP` without negating anything.
* **Seed domain (binding policy, not an OpenCV limit).** Displacement components finite with
  absolute value `<= 2**20`; rotation finite with absolute value `<= 1.0` radian. OpenCV's update
  decodes the previous angle with `asin`, which only represents `[-Pi/2, Pi/2]`, so the angle bound
  keeps the seed in the principal branch. Violations raise `OpenCV_Error`; nothing is clamped. The
  returned angle is still decoded with `atan2`, in `[-Pi, Pi]`.
* **A seed is a prediction.** `Predicted_Transform` must describe the transformation for the
  current image pair; a previous pair's result may be a useful prediction but is not
  automatically correct for the next pair. An accepted seed can still end in native
  nonconvergence (`OpenCV_Error`, no fabricated identity) or in a wrong finite optimum, and a
  finite rigid result does not prove alignment. `Correlation` remains the native value computed
  before the final update.
* **Evidence that the seed is consumed.** On the same 128x128 texture, identity initialization
  throws for +8 and +12 degrees and returns a wrong finite warp for +10 degrees on every
  supported OpenCV version; a near-correct seed recovers all three, and a seed with the wrong
  rotation sign throws (`docs/ecc-euclidean-seeded-source-contract.md`).
* **Limits unchanged from Task 012.** Same image envelope (2-D UInt8 C1, 32 .. 16,384 per side,
  at most 2**24 pixels), the same 1 .. 100 iterations and `1.0E-8 .. 0.1` epsilon, about 1.3 GB
  at the pixel limit, and Core Regions accepted. Supported OpenCV: 4.1 - 4.x and 5.0.
* The production C ABI is now **19 exports** (one added:
  `opencv_video_estimate_ecc_euclidean_seeded`) and AUnit has 140 registrations. See
  [task note](docs/tasks/013-seeded-ecc-euclidean.md),
  [source contract](docs/ecc-euclidean-seeded-source-contract.md) and
  [qualification](docs/task013-qualification.md). Task 012's pinned and post-merge CI all
  succeeded (recorded there), and the separately accepted Farnebäck upstream pointer-formation
  limitation is untouched. Windows remains push-to-main only.

### ECC Euclidean translation and rotation alignment (Task 012)

```ada
Alignment : constant ECC_Euclidean_Result :=
  OpenCV.Video.Estimate_ECC_Euclidean
    (Template_Image => Previous_Image,
     Moving_Image   => Current_Image);
```

Native `cv::findTransformECC` with `MOTION_EUCLIDEAN`; same images and options as the
translation API (`ECC_Euclidean_Options` is a subtype of `ECC_Translation_Options`).
The returned values describe the native 2x3 warp that maps **template coordinates to
moving coordinates** (X right, Y down):

```text
moving_x = cos(t) * template_x - sin(t) * template_y + Displacement.X
moving_y = sin(t) * template_x + cos(t) * template_y + Displacement.Y     t = Rotation_Radians
```

`Rotation_Radians` is in radians and is extracted from the final warp with `atan2`, in
`[-Pi, Pi]`. Rotation is about the coordinate origin (top-left), so `Displacement` is the
matrix translation: for a rotation about the image centre `(cx,cy)` it contains the
compensating terms `cx - (cos t * cx - sin t * cy)` and `cy - (sin t * cx + cos t * cy)`,
not the displacement of the centre. Resample the moving image into the template frame by
applying the same matrix with `WARP_INVERSE_MAP`; do not negate it. This is an image
alignment angle, not a heading or attitude. OpenCV's update decodes the previous angle
with `asin`, so large rotations are not recoverable; measured reliable range is a few
degrees, beyond which ECC throws or returns a wrong finite warp (see the source contract).
A finite rigid result does not prove alignment. Native nonconvergence raises
`OpenCV_Error`; the result is a plain value and inputs are never modified. ECC with this
model needs about 1.3 GB at the 2**24-pixel limit (translation about 1.0 GB). The
production ABI is 18 exports. Task 011's `Estimate_ECC_Translation` is unchanged.
See the [source contract](docs/ecc-euclidean-source-contract.md) and
[qualification](docs/task012-qualification.md). The Task 011 post-merge Windows result and
the separate Farnebäck upstream UB limitation are recorded there.

### ECC translation alignment (Task 011)

```ada
Alignment : constant ECC_Translation_Result :=
  OpenCV.Video.Estimate_ECC_Translation
    (Template_Image => Previous_Image,
     Moving_Image   => Current_Image);
```

Native `cv::findTransformECC` with `MOTION_TRANSLATION`, a privately initialized
identity warp, `COUNT | EPS` termination (`ECC_Translation_Options`: 1 .. 100
iterations, default 50; epsilon `1.0E-8 .. 0.1`, default 0.0001), no mask and a fixed
Gaussian size of 5. `Displacement` maps template coordinates into the moving image
(`moving = template + Displacement`, X right, Y down); it is not the inverse warp, and
OpenCV applies it with `WARP_INVERSE_MAP` to bring the moving image into the template
frame. `Correlation` is the value native ECC returns, computed before its final update,
so it need not score the returned displacement exactly; it is not a confidence and is
never clamped or thresholded. Images are nonempty 2-D UInt8 C1 of equal size, 32 ..
16,384 per side and at most 2**24 pixels (Regions are accepted; ECC needs on the order
of 1 GB of temporaries at the largest size). Native nonconvergence raises
`OpenCV_Error`; a successful call does not prove correct registration (ECC has a small
convergence basin and can finish at a wrong optimum). Inputs are never modified. This
is not a navigation solution. No masks, seeded transforms or other motion models yet.
The Task 010 Windows MSYS2 installer now uses `pacman -Syu`; its real effect is
confirmed only by the post-merge Windows run. The accepted Farnebäck upstream UB
limitation does not apply to, or extend to, ECC. The production ABI was 17 exports in Task 011 (18 with Task 012).
See the [source contract](docs/ecc-source-contract.md) and
[qualification](docs/task011-qualification.md).

### Caller-seeded dense Farnebäck flow (Task 010)

```ada
Initial : constant OpenCV.Core.Mat := Previous_Motion_Estimate;

Refined : constant OpenCV.Core.Mat :=
  OpenCV.Video.Calculate_Farneback_Flow
    (Previous_Image => Previous,
     Next_Image     => Current,
     Initial_Flow   => Initial);
```

`Previous_Motion_Estimate` is assumed to be a valid `Float32` C2 Core Mat with the
image geometry. `Initial_Flow` is a displacement *prediction* in pixels (channel 0
dx, channel 1 dy, signed); native Farnebäck (`OPTFLOW_USE_INITIAL_FLOW`, flags 4)
refines it. A prediction does not guarantee physically correct motion, and no
per-pixel confidence is returned. The seed must be nonempty, 2-D, `Float32` with two
channels and exactly the image rows/columns (Regions are accepted); every component
must be finite with absolute value at most 2**20 pixels (a binding policy, never
clamped). The caller keeps ownership of the seed: the binding copies it into private
storage before native mutation, never modifies it, and returns a distinct Core-owned
Mat. The complete `Farneback_Options` record applies (`Options` precedes
`Initial_Flow`). The accepted legacy OpenCV limitation (KNOWN_UPSTREAM_UB_ACCEPTED,
below) remains applicable. This is not a navigation filter or state estimator. The
production ABI is now 16 exports and AUnit has 107 registrations. See
[qualification](docs/task010-qualification.md).

### Dense Farnebäck flow (Task 009)

```ada
Flow : constant OpenCV.Core.Mat :=
  Calculate_Farneback_Flow (Previous_Image, Next_Image);  --  assumes UInt8 C1 images
```

Returns a new Core-owned Float32 C2 Mat of the image size; channel 0 is dx and
channel 1 dy with `Previous(y,x) ~ Next(y+dy, x+dx)`. `Farneback_Options` defaults
are scale .5, 3 levels, window 15, 3 iterations, neighborhood 5, sigma 1.2 (flags
are always 0). Images are nonempty 2-D UInt8 C1 of equal size, at least 16x16 and at most 134,217,727 pixels (Integer_32'Last / 16); out-of-range
options or images raise `OpenCV_Error`. Every component is validated finite and
nothing is returned on failure. The production ABI is now 15 exports and AUnit has
99 registrations. **Known limitation (KNOWN_UPSTREAM_UB_ACCEPTED):** OpenCV 4.1.0,
4.6.0 and 4.10.0 form an out-of-range pointer in their CPU Farnebäck code before a
bounds check; this is accepted by the project owner for compatibility but makes
these versions unsuitable as-is for safety-sensitive deployments. See the
[source review](docs/farneback-source-contract.md) and
[qualification](docs/task009-qualification.md).

### Forward/backward consistency with owned pyramids (Task 008)

```ada
Previous_Pyramid : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Previous_Image);
Current_Pyramid  : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Current_Image);
Diagnostics : constant Forward_Backward_Track_Array :=
  Track_PyrLK_Forward_Backward (Previous_Pyramid, Current_Pyramid, Points);
Seeded_Diagnostics : constant Forward_Backward_Track_Array :=
  Track_PyrLK_Forward_Backward
    (Previous_Pyramid, Current_Pyramid, Points,
     Initial_Next_Points => Predicted_Points);
```

This usage fragment assumes application images and arrays. Optional forward
predictions correspond by iteration position; counts must match but lower bounds
may differ. Options precede required predictions, as for the raw-image overloads.
Both legs use the same two owned objects, reversed for backward tracking, without
rebuilding pyramids. Only forward successes enter the backward call: its sources
are forward locations and its destination predictions are the original points.
This is Ada composition, not a single native OpenCV forward/backward API.

Results preserve every input index and the complete forward photometric error.
If either leg fails, recovery is the original point, distance is zero, and
`Backward_Tracked`/`Consistent` are false. Successful tracks can still be
geometrically inconsistent. The finite nonnegative caller threshold (including
zero) compares against the returned Float32 Euclidean round-trip distance, with
equality accepted. Low error is evidence, not proof, of physical correspondence.
The same pyramid compatibility checks apply even with empty arrays. Source,
Region and parent lifetimes do not constrain owned pyramids; sequential reuse
is supported, not general concurrent-use safety. See the
[contract](docs/forward-backward-contract.md) and
[qualification](docs/task008-qualification.md). Task 008 retains all 73 prior
tests and adds 19 registrations (92 total); the production ABI remains 14 exports.

### Minimum-eigenvalue trackability with owned pyramids (Task 007)

```ada
Previous_Pyramid : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Previous_Image);
Current_Pyramid  : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Current_Image);
Quality : constant Trackability_Track_Array := Track_PyrLK_Trackability
  (Previous_Pyramid, Current_Pyramid, Points);
Seeded_Quality : constant Trackability_Track_Array := Track_PyrLK_Trackability
  (Previous_Pyramid, Current_Pyramid, Points,
   Initial_Next_Points => Predicted_Points);
```

This usage fragment assumes application images and arrays. Options precedes the
required predictions; named predictions permit default options. Counts must match,
but array lower bounds may differ. Both overloads preserve `Points'Range`. Stored
image/derivative vectors are reused directly, with flags **8 / 12** respectively;
seeds are privately cloned and refined, never returned as failed coordinates.
Pyramid geometry/window/requested-depth compatibility is checked even for empty
points; available depth can truncate naturally.

`Minimum_Eigenvalue` measures **previous-patch LK conditioning**, normalized by
window area. It is not photometric difference, round-trip error or probability:

- `Point_Track.Error`: photometric patch difference;
- `Trackability_Track.Minimum_Eigenvalue`: previous-patch LK conditioning;
- `Forward_Backward_Track.Round_Trip_Error`: geometric round-trip inconsistency.

A failed track returns the original previous point but retains valid native
quality, including a positive eigenvalue on failed next search or threshold
rejection. Every quality slot must be defined, finite and nonnegative. Native
unwritten/invalid quality raises `OpenCV_Error` for the entire call; no zero is
fabricated. In particular OpenCV 5.0 KleidiCV can leave unavailable-previous
quality unwritten: prebuilt vectors still reach the per-level HAL, so they are
not a workaround for that backend contract. See the [source contract](docs/pyrlk-source-contract.md)
and [qualification](docs/task007-qualification.md) for actual backend evidence.
No universal application/navigation quality threshold or combined score is given.
Owned pyramids survive source/Region/parent finalization and support sequential
reuse/role reversal; concurrent safety is not claimed.

### Seeded tracking with reusable owned pyramids

```ada
declare
   Previous_Pyramid : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Previous_Image);
   Current_Pyramid  : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Current_Image);
   Tracks : constant Point_Track_Array := Track_PyrLK
     (Previous_Pyramid    => Previous_Pyramid,
      Next_Pyramid        => Current_Pyramid,
      Points              => Points,
      Initial_Next_Points => Predicted_Points);
begin
   for Track of Tracks loop
      if Track.Tracked then
         Process_Tracked_Point (Track.Next_Point);
      end if;
   end loop;
end;
```

This usage fragment assumes application images, point arrays and processing
procedure. Predictions assist convergence, not guarantee a physically correct
match. Equal array lengths are required; lower bounds may differ. Predictions
are privately copied and refined with native flag 4. Failed tracks return the
previous point and zero photometric error, never the prediction. Owned pyramids
remain usable after source Mats/Regions/parents are finalized and support
sequential reuse and frame-role reversal, not a general thread-safety guarantee.
Task 006 retains the original 59 tests and adds six focused campaigns (65 AUnit
registrations); the C ABI inventory is twelve semantic exports/imports.

Handwritten thick Ada binding for the OpenCV **video** module used by temporal
tracking in the GPS-denied navigation stack. Repository: `opencv_video_ada`;
Alire crate: `opencv_video`; public package: `OpenCV.Video`.

**Version 0.1.0-dev; bootstrap source only, not a release.**
Task 001 qualifies the bootstrap; see [source contract](docs/pyrlk-source-contract.md)
and [qualification record](docs/task001-qualification.md) for exact evidence and
version/CI results. `docs/tasks/001-validate-bootstrap.md` remains the task brief.

## Native module

```text
Ada crate/public package        OpenCV native module
opencv_video / OpenCV.Video  -> opencv2/video/tracking.hpp -> opencv_video
```

This is intentionally **not** a `videoio` binding.

## Initial API

The bootstrap exposes one vertically integrated operation:

```ada
Tracks := OpenCV.Video.Track_PyrLK
  (Previous_Image => Previous,
   Next_Image     => Current,
   Points         => Points);
```

It wraps `cv::calcOpticalFlowPyrLK` with:

- nonempty 2-D UInt8 C1 images of identical geometry;
- Ada arrays of `OpenCV.Float32_Point`;
- typed window/level/termination/min-eigenvalue options;
- per-point `Tracked`, `Next_Point` and `Error` results;
- preserved Ada input bounds;
- deterministic failed-track values (`Next_Point = Previous_Point`, `Error = 0`);
- exception containment and failure-atomic publication in the C++ shim.

Non-contiguous Core Regions are supported with Region-local point coordinates
and native parent-aware pyramid borders. Points may be outside image bounds;
their finite coordinates must have absolute value at most 2**29 for safe native
integer conversion. Windows are 3..255 in each dimension, levels 0..30,
iterations 1..100, epsilon finite in (0,10], and minimum eigenvalue threshold
finite in [0, Float32_Value'Last]. Unsafe image padding/stride arithmetic is
rejected before the native call. Successful errors are finite nonnegative mean
patch L1 differences, not displacement accuracy or confidence. Invalid native
successful results raise `OpenCV_Error`; they are not silently normalized.

### Seeded initial-flow tracking (Task 002)

The original overload remains unchanged: its initial next estimate is the previous
point and its native flags are zero. The seeded overload uses a caller prediction
as the initial next estimate, with only `cv::OPTFLOW_USE_INITIAL_FLOW`:

```ada
Tracks := OpenCV.Video.Track_PyrLK
  (Previous_Image      => Previous,
   Next_Image          => Current,
   Points              => Points,
   Initial_Next_Points => Predictions);
```

`Points` and `Predictions` must have equal lengths; correspondence is by iteration
position, so ranges `5 .. 8` and `20 .. 23` are valid together. Results preserve
`Points'Range`. Both inputs contain Float32 values (no Float64 narrowing), with
finite coordinates bounded by `2**29`; neither needs to lie inside the image.
There is no extra displacement bound. Empty pairs retain the empty-result
contract after image/options validation; mismatched counts raise `OpenCV_Error`.

The seeded declaration places defaulted `Options` **before** required
`Initial_Next_Points`. This avoids making existing four-argument positional option
aggregates ambiguous in Ada. Omit options with named `Initial_Next_Points` as above,
or pass `(Previous, Current, Points, Options, Predictions)` positionally.

Seeds aid optimization/convergence, **not correctness**: a successful LK status is
not proof of the right correspondence. Predictions from camera-motion/navigation
models may be useful; this binding has no IMU, estimator, or navigation dependency.
Seeds and images remain unchanged. Failed tracks still return the original
previous point and zero error, **not** the prediction. The shim clones seeds into
private storage and publishes all three outputs only after successful validation.

See [Task 002](docs/tasks/002-initial-flow-seeding.md) and its
[qualification record](docs/task002-qualification.md). The existing `lk_synthetic`
example is retained; the focused AUnit and independent native oracle demonstrate
the distinguishing seeded fixture without another example executable.

Prebuilt pyramids and UMat remain excluded. Minimum-eigenvalue output is available
only through the distinct Task 004 trackability API below.

### Forward/backward consistency diagnostics (Task 003)

Ordinary PyrLK asks **“where did this point go?”** The composed forward/backward
diagnostic asks **“if I track it there and then back again, do I return to where
I started?”** This operation lives entirely in Ada; OpenCV does not provide a
single forward/backward API here.

```ada
Diagnostics := OpenCV.Video.Track_PyrLK_Forward_Backward
  (Previous_Image => Previous,
   Next_Image     => Current,
   Points         => Points,
   Options        =>
     (Tracking                 => (others => <>),
      Maximum_Round_Trip_Error => 1.0));
```

The seeded-forward overload also accepts named `Initial_Next_Points`, with the
same equal-count, iteration-position and differing-lower-bound rules as Task 002.
Options precedes required seeds. Images, points and predictions remain unchanged.
One diagnostic is returned for **every** input point, preserving `Points'Range`;
no point is silently filtered out.

Only forward successes enter the compact backward call. It tracks from the next
image back into the previous image, starting at each forward location and
**seeding the backward destination with the original previous point**, not the
forward location. The compact results are mapped back to their original indices.

Each `Forward_Backward_Track` contains:

- `Forward`: the ordinary `Point_Track`, unchanged;
- `Backward_Tracked`: whether backward LK succeeded;
- `Recovered_Previous_Point`: the backward location, or the original if unavailable;
- `Round_Trip_Error`: Euclidean original-to-recovered distance **in pixels**,
  calculated in Float64 then checked/converted to finite nonnegative Float32;
- `Consistent`: forward success **and** backward success **and** a valid returned
  distance `<= Maximum_Round_Trip_Error`.

**`Forward.Error` is native mean patch L1 photometric error; `Round_Trip_Error`
is geometric displacement. They are separate quantities, not confidence scores.**
If either pass fails, recovery is the original point, distance is zero, and both
`Backward_Tracked` and `Consistent` are false. That zero never signifies success.

The threshold must be finite and nonnegative; zero is legal, and no arbitrary
maximum or silent clamping is imposed. The default one pixel is a convenience,
**not estimator policy or a universal acceptance criterion**. Low round-trip
error is useful evidence, **not proof of a correct physical correspondence**:
ambiguous repeated patches can still agree in both directions.

See [Task 003](docs/tasks/003-forward-backward-consistency.md) and its
[qualification record](docs/task003-qualification.md) for arithmetic bounds,
real inconsistent fixtures, mapping evidence and native version results.

### Minimum-eigenvalue trackability (Task 004)

```ada
Quality := OpenCV.Video.Track_PyrLK_Trackability
  (Previous_Image => Previous,
   Next_Image     => Current,
   Points         => Points);
--  Or supply Initial_Next_Points => Predictions, with optional typed Options.
```

The distinct `Trackability_Track_Array` preserves `Points'Range` and contains
`Previous_Point`, `Next_Point`, `Tracked`, and **`Minimum_Eigenvalue`**, not `Error`.
Native flags are 8 (unseeded) or 12 (seeded); no raw flags interface is exposed.
`Point_Track` and all existing ordinary/forward-backward operations are unchanged:
**`Point_Track.Error` always means successful mean L1 photometric patch error.**

The new metric is local LK conditioning of the **previous-image** patch, normalized
by window pixel count. Higher values generally indicate stronger two-dimensional
gradient structure. It is not probability, match confidence, reprojection error,
photometric error, or proof of correct correspondence; no universal application
threshold is prescribed. All returned values are finite nonnegative Float32.

Quality is independent of status: threshold rejection can retain a positive
eigenvalue, and a strong previous patch can retain quality after next-search
failure. An unavailable previous patch returns zero on the reviewed CPU path.
Every failed `Next_Point` is still the original point. The shim sentinel-initializes
private quality storage and rejects invalid/unwritten native values atomically,
never clamps them or silently changes status. Existing images/options/points/seeds
validation and immutability apply, including differing seed bounds and Regions.
See the [source contract](docs/pyrlk-source-contract.md#minimum-eigenvalue-source-review-task-004)
and [qualification record](docs/task004-qualification.md).

**Backend limitation:** OpenCV 5.0's KleidiCV 26.03 HAL (observed in macOS ARM64
Homebrew 5.0.0_5) leaves quality unwritten for unavailable previous patches. The
new API deliberately raises `OpenCV_Error` for the entire call on that path,
instead of fabricating zero. The reviewed fallback CPU implementations write
defined zero. This does not change ordinary tracking or failed photometric errors.

### Reusable owned PyrLK pyramids (Task 005)

```ada
declare
   Previous_Pyramid : constant OpenCV.Video.PyrLK_Pyramid :=
     OpenCV.Video.Build_PyrLK_Pyramid (Previous);
   Current_Pyramid : constant OpenCV.Video.PyrLK_Pyramid :=
     OpenCV.Video.Build_PyrLK_Pyramid (Current);
begin
   Tracks := OpenCV.Video.Track_PyrLK
     (Previous_Pyramid, Current_Pyramid, Points);
end;
```

`PyrLK_Pyramid` is a limited private, finalized **binding-owned** object, not a
native OpenCV class or an alternative application Mat wrapper. It owns every
image and derivative level after construction; source mutation/finalization and
Region/parent scope exit do not invalidate it. Default objects are empty and safe
to finalize. `Is_Empty` is valid for them; all metadata queries and tracking reject
empty objects with `OpenCV_Error`. No level/derivative/native-handle extraction.

`PyrLK_Pyramid_Options` contains only `Window_Size` (default 21x21, bounds 3..255)
and `Max_Level` (default 3, bounds 0..30). Builds always use derivatives=true,
reuse-input=false, `BORDER_REFLECT_101` image borders and `BORDER_CONSTANT`
derivative borders. A strided Region's parent may contribute border pixels during
construction, as in raw-image PyrLK; there is no `BORDER_ISOLATED` policy change.

Read-only queries: `Requested_Max_Level`, `Available_Max_Level`,
`Build_Window_Size`, `Is_Empty`. Native geometry can truncate available levels:
tracking accepts requests <= **both requested build levels**, then uses the minimum
of its request and both available levels. Build/track windows must match exactly
and base geometry must agree, even for empty points. Point bounds, validation,
photometric error and failed-value normalization retain the ordinary contract.

The native tracker receives the stored vectors directly; it does not rebuild
pyramids per call. Sequential repeated reuse and next-now/previous-later use are
supported; general concurrent thread safety is not promised. Precomputing dx/dy
costs memory but permits reuse when a current frame becomes previous. No universal
speedup is claimed without measurements. Seeded and trackability overloads are now available; forward/backward
pyramid composition remains deferred. See [Task 005](docs/tasks/005-reusable-pyrlk-pyramids.md)
and its [qualification record](docs/task005-qualification.md).

## Build

Requirements:

- Alire;
- Ada 2022 toolchain;
- C++17 compiler;
- pkg-config;
- OpenCV development files providing the native `video` module;
- `opencv_core ~0.4.0`.

The bootstrap keeps the same Core source pin used by the Calib3D bootstrap:
`4da9d35ea21e1b2efe96296243ea668b488c6326`.

On Debian/Ubuntu:

```sh
sudo apt-get update
sudo apt-get install -y build-essential pkg-config libopencv-dev
alr -n build
alr test
```

Run the synthetic translation example:

```sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/lk_synthetic
```

## Architecture

```text
Ada application
    OpenCV.Video
        Ada point/options/result values
            private Ada C ABI
                callback-scoped Core Mat handles
                    C++17 stable-C shim
                        cv::calcOpticalFlowPyrLK
                            OpenCV video

Application Mat ownership: OpenCV.Core
Native Mat borrowing: OpenCV.Core.Module_Interop callback scope only
```

No C++ object, STL container, `cv::Mat`, native pointer, or native handle appears
in the public Ada API.

## Supported bootstrap range

The configure script accepts representative OpenCV versions in the existing
binding-family range:

- OpenCV 4.1 through 4.x;
- OpenCV 5.0.x.

The manual compatibility workflow is set up for 4.1.0, 4.10.0 and 5.0.0.
Workflow definitions alone are not evidence of compatibility; the qualification
record identifies the runs and results.

## CI policy

PR CI: repository checks, Linux, macOS and Linux ASan/UBSan. Windows/MSYS2 is a
separate push-to-main-only workflow. The pinned OpenCV source-build matrix is
manual-only.

## Deliberate exclusions

The first slice does not bind:

- forward/backward prebuilt-pyramid composition;
- DIS dense optical flow and Farneback options beyond the Task 009/010 subset (Gaussian, UMat);
- template masks, dual-mask ECC, masks for translation/Euclidean/affine ECC, and multiscale ECC
  registration (translation, Euclidean, seeded Euclidean, identity-initialized and seeded affine,
  and identity-initialized, seeded and moving-image-masked homography ECC are bound);
- KalmanFilter;
- CamShift / meanShift;
- UMat/OpenCL overloads;
- VideoCapture / VideoWriter / codecs (`videoio`);
- feature detection, feature matching, Calib3D, geospatial transforms or
  navigation-filter policy.

Those should be added only as separate, reviewed vertical tasks.

## License

Apache-2.0; see `LICENSE` and `NOTICE`.
