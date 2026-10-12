# Task 013 — Caller-seeded ECC Euclidean alignment

Branch `feature/013-seeded-ecc-euclidean`, from main `d73714047e41beaf0cb1a42b32758c56ab7e767f`
(Task 012 merge of PR #12, reviewed feature head `61ed6279636a07f42f5c83ab892444bacffbed7c`).
Crate version stays `0.1.0-dev`; the Core pin stays `4da9d35ea21e1b2efe96296243ea668b488c6326`
in all three Alire roots; no new dependency; CI topology unchanged (Windows stays
push-to-main only).

## Scope

One new Ada overload of `Estimate_ECC_Euclidean` that lets the caller supply the initial
rigid transformation, and one new C export, `opencv_video_estimate_ecc_euclidean_seeded`.
The Task 012 identity-initialized function and export, and Task 011's translation API, are
unchanged. Out of scope: seeded translation ECC, affine and homography models, masks,
multiscale ECC, composition of transforms, a navigation filter, any public warp type.

## Public API

```ada
function Estimate_ECC_Euclidean
  (Template_Image    : OpenCV.Core.Mat;
   Moving_Image      : OpenCV.Core.Mat;
   Options           : ECC_Euclidean_Options := (others => <>);
   Initial_Transform : ECC_Euclidean_Result)
   return ECC_Euclidean_Result;
```

`Options` is defaulted and precedes the required seed, following the seeded PyrLK and
Farnebäck APIs. `ECC_Euclidean_Result` is unchanged, so a previous result can be passed back:

```ada
Refined : constant ECC_Euclidean_Result :=
  OpenCV.Video.Estimate_ECC_Euclidean
    (Template_Image    => Previous_Image,
     Moving_Image      => Current_Image,
     Initial_Transform => Predicted_Transform);
```

`Predicted_Transform` must describe the transformation for **this** image pair. A previous
pair's result may be a useful prediction but is not automatically the right transform for the
next pair; no transform composition is performed.

## Initial-transform semantics

* Only `Displacement.X`, `Displacement.Y` and `Rotation_Radians` initialize the warp.
* **`Initial_Transform.Correlation` is ignored.** It is not an optimization input, a
  threshold or a confidence, is never validated, never propagated to the result and never
  affects convergence. Changing only the initial correlation changes nothing (AUnit
  `ECC_Seeded_Input_Independence`).
* The seed is a template-to-moving matrix, exactly the Task 012 convention, rotating about the
  image-coordinate origin `(0,0)`. A rotation about the centre needs the compensating
  matrix translation (`tx = cx - (cos t * cx - sin t * cy) + dx`, and likewise `ty`);
  `Displacement` is never reinterpreted as a centre displacement.
* The seed is a prediction, not a guarantee. `Initial_Transform` is an `in` parameter and is
  never modified.

## Seed domain (binding policy)

| Component | Accepted |
| --- | --- |
| `Displacement.X`, `.Y` | finite, absolute value `<= 2**20` pixels |
| `Rotation_Radians` | finite, absolute value `<= 1.0` radian |

Violations raise `OpenCV_Error`; nothing is clamped. The angle bound keeps the seed inside
the principal `asin` branch of OpenCV's update rule with margin below pi/2. These are
conservative policy limits, not OpenCV limits, and not a promise that an accepted seed
converges. The domain is checked twice with the same constants: in the Ada preflight and in the
C++ shim (widened to `double`), so direct C callers get the same rejection.

## Native ABI

```c
opencv_video_status opencv_video_estimate_ecc_euclidean_seeded(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    float initial_translation_x,
    float initial_translation_y,
    double initial_rotation_radians,
    opencv_video_ecc_euclidean_result *result);
```

The existing 24-byte `opencv_video_ecc_euclidean_result` is reused. The seed crosses the ABI by
value, so no caller-owned native warp exists. The production ABI is **19 exports**, up from 18.
The identity-initialized and seeded exports share one private helper
(`run_ecc_euclidean`) that performs native invocation, schema, finiteness and rigid-matrix
validation, `atan2` decoding and atomic publication; only the initial warp differs.
A zero seed enters native ECC with the exact identity matrix of Task 012 and produces byte
identical results and statuses (AUnit `ECC_Seeded_Zero_Equivalence`, native boundary test).

## Qualification

See `docs/ecc-euclidean-seeded-source-contract.md` (upstream review of four versions, matrix
construction, seed domain, native probes) and `docs/task013-qualification.md` (campaign
results, mutation findings, per-version results, CI).
