# Task 015 — Caller-seeded ECC affine alignment

Adds one Ada overload and one C export. Scope is caller-seeded `MOTION_AFFINE` only: no
homography, decomposition, mask, multiscale ECC or navigation filter.

## Ada

```ada
function Estimate_ECC_Affine
  (Template_Image    : OpenCV.Core.Mat;
   Moving_Image      : OpenCV.Core.Mat;
   Options           : ECC_Affine_Options := (others => <>);
   Initial_Transform : ECC_Affine_Result)
   return ECC_Affine_Result;
```

`ECC_Affine_Warp` and `ECC_Affine_Result` are reused. Only `Initial_Transform.Warp` is an input;
`Initial_Transform.Correlation` is never read, validated or returned.

## C

```c
opencv_video_status opencv_video_estimate_ecc_affine_seeded(
    const opencv_core_mat_handle *template_image, const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations, double epsilon,
    float initial_m00, float initial_m01, float initial_m02,
    float initial_m10, float initial_m11, float initial_m12,
    opencv_video_ecc_affine_result *result);
```

Six coefficients by value in row-major storage order; result structure reused (32 bytes). The
production archive exports 21 `opencv_video_*` functions and no test hooks.

## Implementation

Identity-initialized and seeded affine ECC share one private helper (`ecc_affine_entry`): identical
image/option validation, native call, schema/finiteness checks and publication. Only the initial
private matrix differs. The 32-byte output is written once, after every check, and is untouched on
any failure. Task 014's identity ABI and numerical behavior are unchanged (identity-seed
equivalence is tested byte-for-byte).

## Seed domain

`M00 M01 M10 M11`: finite, `abs <= 4`. `M02 M12`: finite, `abs <= 2**20`. See
`docs/ecc-affine-seeded-source-contract.md`. No determinant or conditioning rule on the input.

## Tests

* 10 new AUnit registrations (163 total): identity equivalence, consumption, refinement and
  direction, correlation ignored, immutability/Regions, seed validation, image validation,
  nonconvergence, direct oracle, oracle integrity.
* `tests/cpp/ecc_affine_seeded_oracle.cpp` (34 fixtures, direct OpenCV only) and
  `tests/cpp/ecc_affine_seeded_probe.cpp` (research).
* `run_ecc_affine_seeded()` in `tests/cpp/native_boundary.cpp`, including all eight exception
  injections and post-native corruption.

See `docs/task015-qualification.md` for measured results and open items.
