# Task 016 — ECC eight-parameter homography alignment

Adds one Ada function and one C export. Scope is identity-initialized `MOTION_HOMOGRAPHY` only. Not
included: seeded homography, mask, multiscale ECC, homography decomposition, inversion, resampling
or navigation filter.

## Safety gate first

The safety gate in `docs/ecc-homography-source-contract.md` was established before any production
code was written. It landed in commit `e3d72bf`, before the API commit `c39a071`.

The gate reviewed these OpenCV sources at 4.1.0, 4.6.0, 4.10.0 and 5.0.0:
* `ecc.cpp`;
* `tracking.hpp`;
* `imgwarp.cpp`.

It also measured native behavior with `tests/cpp/ecc_homography_probe.cpp`. Its disposition is
`SAFETY_GATE_ESTABLISHED` for the identity-initialized, unmasked, CPU, UInt8 C1 call. That is a
memory-safety and bounded-arithmetic conclusion only. It does not claim convergence or alignment
accuracy.

## Ada

```ada
subtype ECC_Homography_Options is ECC_Translation_Options;

type ECC_Homography_Warp is record   --  defaults: identity
   H00, H01, H02 : OpenCV.Float32_Value;
   H10, H11, H12 : OpenCV.Float32_Value;
   H20, H21, H22 : OpenCV.Float32_Value;
end record;

type ECC_Homography_Result is record
   Warp        : ECC_Homography_Warp;
   Correlation : OpenCV.Float64_Value;
end record;

function Estimate_ECC_Homography
  (Template_Image : OpenCV.Core.Mat;
   Moving_Image   : OpenCV.Core.Mat;
   Options        : ECC_Homography_Options := (others => <>))
   return ECC_Homography_Result;
```

These declarations are abbreviated; the exact ones are in `src/opencv-video.ads`. The result is a
plain Ada value that owns no native resource, and no native object is exposed.

## C

```c
typedef struct opencv_video_ecc_homography_result {
    float h00, h01, h02, h10, h11, h12, h20, h21, h22;   /* offsets 0..32 */
    double correlation;                                  /* offset 40 */
} opencv_video_ecc_homography_result;                    /* size 48, alignment 8 */

opencv_video_status opencv_video_estimate_ecc_homography(
    const opencv_core_mat_handle *template_image, const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations, double epsilon,
    opencv_video_ecc_homography_result *result);
```

* Bytes 36-39 are padding.
* The layout is checked with `static_assert` in C++ and `_Static_assert` in C11.
* The production archive exports 22 `opencv_video_*` functions; this is the 22nd.
* The first 21 exports are unchanged.
* The production archive contains no test hook.

## Native call

```text
cv::findTransformECC(template, moving, warp, MOTION_HOMOGRAPHY,
                     TermCriteria(COUNT | EPS, iterations, epsilon), noArray(), 5)
```

* `warp` is a private 3x3 `CV_32F` identity.
* There is no mask, and the Gaussian filter size is 5.
* On 5.0.0 the call forwards to `findTransformECCWithMask` with an empty mask.

After the native call, the shim checks the result in this order:

1. The private warp is still 3x3, `CV_32F`, continuous and the same storage.
2. All nine coefficients are finite.
3. H22 is exactly 1.0.
4. The correlation is finite.

The 48-byte output is written once, after every check. On any failure it is unchanged. C++
exceptions are caught in the shim and never cross the C ABI.

## Matrix convention

```text
[ H00 H01 H02 ]       D        = H20*x + H21*y + H22
[ H10 H11 H12 ]       moving_x = (H00*x + H01*y + H02) / D
[ H20 H21 H22 ]       moving_y = (H10*x + H11*y + H12) / D
```

* **Direction and units.** Template coordinates map to moving coordinates, in pixels, with X right
  and Y down.
* **Storage order.** The matrix is stored row-major. The native update-vector order (`H00 H10 H20
  H01 H11 H21 H02 H12`) is internal and is not exposed.
* **Eight parameters, nine coefficients.** Native ECC optimizes eight parameters and never writes
  H22. For this identity-initialized call, H22 is therefore exactly 1. Any other value is a
  binding error, not something to normalize.
* **Resampling.** To resample the moving image into the template frame, apply the same matrix with
  `WARP_INVERSE_MAP`. The binding never inverts, transposes, normalizes or decomposes the matrix.
* **Zero denominator.** The mapping is undefined where `D = 0` and extreme near it. The binding does
  not resample, so it neither detects nor alters such points.

## Contract

* **Inputs and options.** The Task 011 envelope, shared by every ECC model:
  * non-empty 2-D UInt8 C1 images of matching size;
  * rows and columns 32-16,384;
  * at most `2**24` pixels;
  * iterations 1-100;
  * finite epsilon `1e-8 .. 0.1`.

  Regions are accepted. Inputs are never modified.
* **Correlation.** This is the native value, computed before the final native update. It is not
  clamped and is not a confidence score.
* **Errors.** Native nonconvergence raises `OpenCV_Error`, and no identity result is fabricated.
  Invalid arguments raise `OpenCV_Error` before any native call.
* **Finite is not correct.** Noise against a texture (all versions), low-gradient imagery and an
  ill-conditioned projective pair (4.6.0, 4.10.0) return finite but incorrect matrices. No correlation
  threshold is imposed to hide them. A finite result does not prove correct registration.
* **Small basin.** From identity, convergence covers about:
  * 2% scale or shear;
  * 2 degrees of rotation;
  * a few pixels of translation;
  * perspective terms of about `2e-4` at 128 px.
* **Not navigation.** The result is an image-alignment matrix. It is not a camera pose, attitude, 3D
  motion, geographic mapping or navigation estimate.

## Tests

* **AUnit.** 13 new registrations bring the total to 176:
  * defaults;
  * identity/translation;
  * rotation/scale/shear;
  * projective coefficients;
  * homography versus affine;
  * inverse warp;
  * options/repeatability;
  * Regions/lifetime;
  * other models unchanged;
  * validation;
  * nonconvergence;
  * direct oracle;
  * oracle integrity.
* **Oracle.** `tests/cpp/ecc_homography_oracle.cpp` has 37 fixtures and calls OpenCV directly.
* **Native boundary.** `run_ecc_homography()` in `tests/cpp/native_boundary.cpp` covers:
  * bitwise equality with a direct native call;
  * 48-byte failure atomicity;
  * validation;
  * eight exception injections;
  * 20 rejected and 5 accepted output corruptions;
  * 8 schema mutations.
* **Smoke and research.**
  * `tests/src/homography_smoke.adb` is the smoke program.
  * `tests/cpp/ecc_homography_probe.cpp` is the research probe.

See `docs/task016-qualification.md` for measured results, version differences and open items.
