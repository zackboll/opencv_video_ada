# Task 011 — ECC translation alignment and Windows MSYS2 repair

Branch `feature/011-ecc-translation`, from main `130f196128c17a41d1eb0f6c4eec41015661bd74`
(Task 010 merge of PR #10). Crate version stays `0.1.0-dev`; Core pin stays
`4da9d35ea21e1b2efe96296243ea668b488c6326`; no new dependency; CI topology unchanged.

## Scope

1. First commit: repair the Windows post-merge dependency installer. The Task 010 run
   failed because `pacman -Sy` refreshed the package database without upgrading
   installed packages (`libgcc`/`libstdc++` 16.2.0-5 versus `cc-libs` requiring
   16.2.0-4). The workflow now runs a separate `pacman -Syu --noconfirm` step and uses
   `-Syu` for the install. No `--nodeps`, `--overwrite`, downgrade, pin or retry.
   `scripts/check_repository.py` and `scripts/test_configuration.py` inspect the real
   workflow and reject `-Sy` and dependency-bypass flags. The trigger stays
   push-to-main only. **This is not proven on Windows until the post-merge run.**
2. Bind `cv::findTransformECC` with `MOTION_TRANSLATION` as
   `OpenCV.Video.Estimate_ECC_Translation`, one new C export
   (`opencv_video_estimate_ecc_translation`) and one fixed-layout C result.

Out of scope: Euclidean, affine and homography models, masks, caller-supplied initial
warps, image warping, ECC multiscale.

## Public API

`ECC_Translation_Options` (`Maximum_Iterations` 1 .. 100, default 50; `Epsilon`
finite in `1.0E-8 .. 0.1`, default 0.0001), `ECC_Translation_Result`
(`Displacement : OpenCV.Float32_Point`, `Correlation : OpenCV.Float64_Value`) and
`Estimate_ECC_Translation (Template_Image, Moving_Image, Options)`.

Images: nonempty 2-D UInt8 C1, identical rows and columns, each 32 .. 16,384, at most
2**24 pixels in total. Core Regions (including non-contiguous ones) are accepted.
Anything else raises `OpenCV_Error`; values are never clamped.

## Contract

* Displacement maps template to moving coordinates: `moving = template + Displacement`
  (X right, Y down). It is not the inverse warp. OpenCV applies the matrix with
  `WARP_INVERSE_MAP` when it warps the moving image into the template frame.
* `Correlation` is the value `findTransformECC` returns. Upstream computes it at the
  start of the final iteration, before the final update, so it can describe the warp
  one step earlier than the returned displacement. The binding neither recomputes nor
  clamps it and sets no acceptance threshold.
* A returned result means the native optimizer returned. It does not mean the
  registration is right: ECC can end at a wrong local optimum with a finite
  correlation, and the same image pair can succeed on one OpenCV version and throw on
  another.
* Native nonconvergence (`StsNoConv`) and every other native failure raise
  `OpenCV_Error`. No identity result is substituted.
* The caller's images and their parents are not modified. The result is a plain value;
  no native object outlives the call.
* The displacement is an image-intensity alignment output only. It is not a navigation
  solution, a geographic position or verified physical motion.

## Native ABI

`opencv_video_estimate_ecc_translation(template, moving, maximum_iterations, epsilon,
result)` with `opencv_video_ecc_translation_result { float displacement_x; float
displacement_y; double correlation; }`: offsets 0, 4, 8, size 16. The production ABI
is 17 exports. `*result` is written only after every native call and validation has
succeeded; it is untouched on any failure. All C++ exceptions are contained.

## Qualification

See `docs/ecc-source-contract.md` (upstream review, envelope, measurements) and
`docs/task011-qualification.md` (campaign results).
