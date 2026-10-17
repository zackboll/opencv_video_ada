# Task 018 — Moving-image-masked ECC homography alignment

Adds two Ada overloads and two C exports. Scope is a **moving-image mask** for the existing
identity-initialized (Task 016) and caller-seeded (Task 017) homography ECC operations. Not
included: template masks, dual-mask ECC, masks for translation/Euclidean/affine ECC, multiscale or
pyramid registration, homography inversion/decomposition, Calib3D, navigation fusion, new
dependencies, or changes to existing result types and Core pins.

## Safety gate first

`docs/ecc-homography-masked-source-contract.md` recorded `SAFETY_GATE_ESTABLISHED` in commit
`f9d3db9`, before the API commit `ef3daac`. The existing unmasked gates were not assumed to cover
the mask path. The investigation reviewed, on 4.1.0, 4.6.0, 4.10.0 and 5.0.0, how the
seven-argument `findTransformECC` thresholds, Gaussian-erodes, warps and applies the moving mask to
gradients and statistics, and what happens with no remaining support (`countNonZero = 0` gives
`rho = 0/0`, caught by the existing `cvIsNaN` guard before any update). Evidence: direct-native
probes; OpenCV builds instrumented with ASan/UBSan/`float-cast-overflow` for all four versions (0
reports); diagnostic `ecc.cpp` copies counting zero-support iterations, infinite `rho` and
narrowing overflow (none besides the NaN path); peak-memory measurements.

## Ada

```ada
function Estimate_ECC_Homography
  (Template_Image : OpenCV.Core.Mat;
   Moving_Image   : OpenCV.Core.Mat;
   Moving_Mask    : OpenCV.Core.Mat;
   Options        : ECC_Homography_Options := (others => <>))
   return ECC_Homography_Result;

function Estimate_ECC_Homography
  (Template_Image    : OpenCV.Core.Mat;
   Moving_Image      : OpenCV.Core.Mat;
   Moving_Mask       : OpenCV.Core.Mat;
   Options           : ECC_Homography_Options := (others => <>);
   Initial_Transform : ECC_Homography_Result)
   return ECC_Homography_Result;
```

Result and option types are reused. The existing overloads are unchanged in source and behavior
(their oracles, boundary and fault campaigns pass unmodified).

## C

```c
opencv_video_status opencv_video_estimate_ecc_homography_masked(
    const opencv_core_mat_handle *template_image, const opencv_core_mat_handle *moving_image,
    const opencv_core_mat_handle *moving_mask, int32_t maximum_iterations, double epsilon,
    opencv_video_ecc_homography_result *result);

opencv_video_status opencv_video_estimate_ecc_homography_masked_seeded(
    const opencv_core_mat_handle *template_image, const opencv_core_mat_handle *moving_image,
    const opencv_core_mat_handle *moving_mask, int32_t maximum_iterations, double epsilon,
    float initial_h00, float initial_h01, float initial_h02,
    float initial_h10, float initial_h11, float initial_h12,
    float initial_h20, float initial_h21, float initial_h22,
    opencv_video_ecc_homography_result *result);
```

25 production exports. The 48-byte result structure is reused with the Task 017-R1 representation
guarantee (all bytes defined, padding 36-39 zero, published once after every check; untouched on
failure). No existing export or structure changed.

## Mask contract

* Moving-image pixel coordinates (row, column of `Moving_Image`); no transform of its own.
* Zero excludes, nonzero includes; 1, 255 and arbitrary nonzero are bitwise equivalent.
* Nonempty, 2-D, UInt8, one channel, exactly the moving image's rows and columns, row step
  `<= INT_MAX`, at least one nonzero pixel; Regions accepted. Checked in Ada (metadata) and in the
  shim (metadata and `countNonZero`) after the image/option envelope and before the seed.
* Never written, converted or normalized; the validated caller Mat is passed as a read-only
  `InputArray`.
* An all-included mask is bitwise identical to the unmasked call on every qualified version, so it
  is asserted exactly.
* No minimum-support rule beyond "one nonzero pixel": masks that native erosion empties raise
  natively and deterministically ("NaN encountered"); a binding replica of the erosion would track
  version-specific Gaussian code for no safety gain. Marginal surviving support can return a finite
  but meaningless matrix, which is published unchanged.

## Implementation

All four homography exports share `ecc_homography_entry`. It gained the mask handle and a flag;
unmasked exports pass `cv::noArray()` exactly as before, masked exports pass the validated mask.
Validation order: null result, images and options, mask, seed. The native call, output-schema,
finiteness and H22 checks, test hooks and publication are shared. The Ada body factors the Task 017
seed checks into `Validate_Homography_Seed` (same messages) and adds `Validate_Moving_Mask`.

## Tests

* 12 new AUnit registrations (198 total, all 186 retained).
* `tests/cpp/ecc_homography_masked_oracle.cpp`: 53 fixtures, direct OpenCV only, bit-pattern output,
  fixture/mask checksums, unmasked-equality flag and native error class.
* `tests/cpp/ecc_homography_masked_probe.cpp`: safety-gate research probe (not part of CI).
* `run_ecc_homography_masked()` in `tests/cpp/native_boundary.cpp`, production and fault builds.

See `docs/task018-qualification.md` for measured results, mutation results and open items.
