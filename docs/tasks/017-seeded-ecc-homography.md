# Task 017 — Caller-seeded ECC homography alignment

Adds one Ada overload and one C export. Scope is caller-seeded `MOTION_HOMOGRAPHY` only. Not
included: mask, multiscale ECC, homography decomposition, inversion, normalization, composition,
resampling or a navigation filter.

## Safety gate first

`docs/ecc-homography-seeded-source-contract.md` recorded `SAFETY_GATE_ESTABLISHED` in commit
`7fb0cb8`, before the API commit `485d11e`. Native ECC updates the eight non-H22 coefficients without
constraint, so the gate does not try to bound later iterations through the seed. It shows instead
that every native operation consuming an unconstrained warp is clamped (`warpPerspective`) or
IEC 60559-defined (division, double-to-float narrowing), and that no warp value indexes memory or
sizes an allocation. Evidence: four-version source review; direct-native probes on all four
versions; instrumented OpenCV (ASan/UBSan/`float-cast-overflow`) for all four versions; a diagnostic
copy of `ecc.cpp` that trapped finite narrowing above `FLT_MAX` (none) and recorded the denominator
reaching exactly 0.

## Ada

```ada
function Estimate_ECC_Homography
  (Template_Image    : OpenCV.Core.Mat;
   Moving_Image      : OpenCV.Core.Mat;
   Options           : ECC_Homography_Options := (others => <>);
   Initial_Transform : ECC_Homography_Result)
   return ECC_Homography_Result;
```

`ECC_Homography_Warp` and `ECC_Homography_Result` are reused. Only `Initial_Transform.Warp` is an
input; `Initial_Transform.Correlation` is never read.

## C

```c
opencv_video_status opencv_video_estimate_ecc_homography_seeded(
    const opencv_core_mat_handle *template_image, const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations, double epsilon,
    float initial_h00, float initial_h01, float initial_h02,
    float initial_h10, float initial_h11, float initial_h12,
    float initial_h20, float initial_h21, float initial_h22,
    opencv_video_ecc_homography_result *result);
```

Nine coefficients by value in row-major storage order; the 48-byte result structure is reused. The
production archive exports 23 `opencv_video_*` functions and no test hooks.

## Implementation

Identity-initialized and seeded homography ECC share one private helper (`ecc_homography_entry`):
identical image/option validation, native call, schema/finiteness/H22 checks and publication. Only
the initial private matrix and the seed check differ. The seed check runs after the image check
because the denominator rule needs the template size. Task 016's export behavior is unchanged
(identity-seed equivalence is tested byte-for-byte, and the Task 016 boundary, fault and oracle
campaigns still pass).

## Seed domain

All nine finite; `H22 = 1.0` exactly; `|H00|,|H01|,|H10|,|H11| <= 4`; `|H02|,|H12| <= 2**20`;
`|H20|*(cols-1) + |H21|*(rows-1) <= 0.5` in Float64. The output is **not** restricted to this domain.

## Tests

* 10 new AUnit registrations (186 total): identity equivalence, consumption, refinement and
  direction, correlation ignored, immutability/Regions, seed validation, image validation,
  nonconvergence, direct oracle, oracle integrity.
* `tests/cpp/ecc_homography_seeded_oracle.cpp` (35 fixtures, direct OpenCV only, every seed checked
  against an independent restatement of the domain) and `tests/cpp/ecc_homography_seeded_probe.cpp`
  (research).
* `run_ecc_homography_seeded()` in `tests/cpp/native_boundary.cpp`: bitwise agreement with a direct
  seeded call, identity equivalence, 62 seed rejections (27 nonfinite, 6 H22, 16 linear, 6 translation, 7 denominator) plus the geometry rule, option boundaries,
  domain-edge seeds, Regions, aliases, and in the fault build 8 exception injections, 20 rejected
  corruptions, 5 accepted finite matrices and 8 schema mutations.

See `docs/task017-qualification.md` for measured results and open items.