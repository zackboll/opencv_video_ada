# Task 014 — ECC six-parameter affine image alignment

Branch `feature/014-ecc-affine`, from main `e16a00cb1a725465bdf1259e1372423f63b9f4f1`
(Task 013 merge of PR #13, reviewed feature head `7575054cbb0cdcd0ac987f93f12e2c0f2f102071`).
Crate version stays `0.1.0-dev`; the Core pin stays `4da9d35ea21e1b2efe96296243ea668b488c6326`
in all three Alire roots; no new dependency; CI topology unchanged (Windows stays push-to-main only).

## Scope

One new Ada function, `Estimate_ECC_Affine`, and one new C export,
`opencv_video_estimate_ecc_affine` (the 20th). It binds `cv::findTransformECC` with
`cv::MOTION_AFFINE`, a private identity-initialized 2x3 Float32 warp, `COUNT | EPS`, no mask and
Gaussian size 5. Out of scope: caller-seeded affine ECC, homography, masks, multiscale ECC,
image-warping APIs, motion-model selection flags, geographic or navigation transformations.

## Public API

```ada
subtype ECC_Affine_Options is ECC_Translation_Options;

type ECC_Affine_Warp is record
   M00 : OpenCV.Float32_Value := 1.0;   M01 : OpenCV.Float32_Value := 0.0;   M02 : OpenCV.Float32_Value := 0.0;
   M10 : OpenCV.Float32_Value := 0.0;   M11 : OpenCV.Float32_Value := 1.0;   M12 : OpenCV.Float32_Value := 0.0;
end record;

type ECC_Affine_Result is record
   Warp        : ECC_Affine_Warp := (others => <>);
   Correlation : OpenCV.Float64_Value := 0.0;
end record;

function Estimate_ECC_Affine
  (Template_Image : OpenCV.Core.Mat;
   Moving_Image   : OpenCV.Core.Mat;
   Options        : ECC_Affine_Options := (others => <>)) return ECC_Affine_Result;
```

The six native coefficients are returned directly, without a rotation/scale/shear decomposition
(a general affine matrix has no unique decomposition, especially with shear or reflection). No
existing result type changed and no `cv::Mat` is exposed.

## C ABI

```c
typedef struct { float m00, m01, m02, m10, m11, m12; double correlation; } opencv_video_ecc_affine_result;
opencv_video_status opencv_video_estimate_ecc_affine(const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image, int32_t maximum_iterations, double epsilon,
    opencv_video_ecc_affine_result *result);
```

Offsets 0, 4, 8, 12, 16, 20, 24; size 32; alignment 8. Checked by a C++ `static_assert`, a C11
`_Static_assert` (the header compiles as C11 with `-Wall -Wextra -Wpedantic -Werror`), the boundary
driver, and mirrored with `Convention => C` in the private Ada package. The production ABI is
**20** exports; none of the 19 earlier signatures changed.

## Behaviour

* Image and option envelope identical to Tasks 011-013 (UInt8 C1, equal 2-D geometry, 32 .. 16,384
  per side, at most 2**24 pixels, iterations 1 .. 100, epsilon finite in 1e-8 .. 0.1). Validation
  reuses `Validate_ECC_Arguments` and `validate_ecc_inputs`.
* Output is validated for storage identity, 2x3 `CV_32FC1` contiguous layout, six finite
  coefficients and a finite correlation. **No** rigidity, orthogonality or determinant policy is
  applied: scale, shear, reflection and singular finite matrices are legitimate affine output
  (see `ecc-affine-source-contract.md`). The Euclidean validation is not reused.
* `*result` is written only after every check succeeds; any failure leaves the caller's 32 bytes
  unchanged. `cv::Exception`, `std::bad_alloc`, other standard exceptions and unknown exceptions
  are contained. Native nonconvergence stays an error; identity is never fabricated.

## Memory

Measured peak resident memory (`VmHWM`, direct OpenCV, `tests/cpp/ecc_affine_limits.cpp`) on
4096 x 4096 (`2**24` pixels): translation 1.01 GB, Euclidean 1.33 GB, **affine 1.27 GB**
(4.10.0); 4.1.0 1.25 GB; 5.0.0 1.21 GB. 1024 x 16384 and 16384 x 1024: 1.27 GB. The six-block
Jacobian is twice the size of the Euclidean one (384 MiB versus 192 MiB), yet the measured
affine peak is not higher than the Euclidean peak; the reason was not investigated (it was not
profiled), so no explanation is claimed. The existing envelope is **retained** on the strength
of these measurements, without a new affine-specific limit. Passing validation does not guarantee that a machine has this memory.
Maximum-size workloads are not run in CI.
