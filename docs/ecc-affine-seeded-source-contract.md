# Seeded affine ECC source contract (Task 015)

> **Follow-up note, 2026-10-18 (Task 020).** The historical review below said the
> `X0 + adelta[x]` overflow "was reasoned from source, not demonstrated". It also said "the seed
> bounds make it unreachable under the 16,384-pixel geometry limit". **That second conclusion is
> wrong.** The seed bounds constrain only the first iterate. Native ECC applies unbounded
> Gauss-Newton updates afterwards, and later intermediate warps can be arbitrarily large.
>
> Task 020 demonstrated the exposure through the real C export
> `opencv_video_estimate_ecc_affine_seeded`. The seed `[0.580653667 -0.412717313 -4.85272264
> -0.241246387 0.386808515 -10.5655231]` is inside the published domain. With a 97x64 texture
> pair, 100 iterations and eps 1e-8, ECC produced the third-iteration warp
> `[-45747.7891 88966.1875 -3195937.25 -69792.625 128145.844 -4427218.5]`. UBSan on
> instrumented OpenCV 4.1.0 then reported `imgwarp.cpp:2285/2286` and
> `imgwarp.sse4_1.cpp:204/205` signed-integer overflow, with
> `ecc_affine_entry -> findTransformECC -> warpAffine` on the stack. The call returned
> `OPENCV_VIDEO_ERROR_OPENCV` ("NaN encountered").
>
> Seed `[0.45051536 0.286671013 23.7435341 -0.437181264 1.13443863 -37.4001312]` at 100x64
> makes native ECC generate PR #19 Matrix B exactly at iteration 3 on OpenCV 5.0.0. Matrix B is
> outside int32 for the 4.x arithmetic model. OpenCV 5.0 does not execute that integer path for
> these types; see the Task 020 note.
>
> See [tasks/020-existing-affine-safety-audit.md](tasks/020-existing-affine-safety-audit.md),
> [task020-qualification.md](task020-qualification.md) and
> [ecc-affine-masked-source-contract.md](ecc-affine-masked-source-contract.md). Disposition:
> `KNOWN_NATIVE_UB_UNMITIGATED`. The seed policy is unchanged and remains a valid input
> contract, but it is **not** a native-safety guarantee. The Task 015 measurements below remain
> valid records of what they measured.

Scope: `cv::findTransformECC(..., MOTION_AFFINE, ...)` with a caller-supplied initial 2x3 Float32
warp. Sources read: `modules/video/src/ecc.cpp` of the pinned local trees 4.1.0 and 5.0.0 (built
from `opencv-4.1.0` / `opencv-5.0.0` under `~/.cache/video-task004`) and the installed 4.10.0
headers/behavior. **4.6.0 sources were not re-read in this session**; it is covered only by the
behavioral campaign (see the qualification report), and the Task 014 source review of 4.6.0 is the
only source evidence for it.

## What the source shows (4.1.0 and 5.0.0)

1. `findTransformECC` keeps a nonempty `warpMatrix`; only an *empty* matrix is replaced by
   `Mat::eye(rowCount, 3, CV_32F)` (4.1.0 `ecc.cpp:361-368`; 5.0.0 `findTransformECCWithMask`
   `ecc.cpp:357-365`). A caller-initialized 2x3 CV_32F matrix is therefore used as given. In 5.0.0
   the seven-argument overload forwards through `findTransformECCWithMask`.
2. The matrix is consumed in the first iteration: `warpAffine(imageFloat, ..., map, ...)` and the
   same call for both gradients and the mask (4.1.0 `ecc.cpp:506-509`).
3. The Jacobian uses the warped gradients, so the seed enters the six-column Jacobian only through
   those resampled images.
4. `update_warping_matrix_ECC` adds the six updates in place with the **update-vector order**
   `u0..u5 -> map[0], map[3], map[1], map[4], map[2], map[5]` (4.1.0 `ecc.cpp:282-287`). The
   stored matrix keeps row-major storage order `M00 M01 M02 M10 M11 M12`, so the seed is written in
   storage order and never permuted.
5. Because the update is in place, an exception can leave the **private** warp partially modified.
   The binding never publishes it after an exception.
6. Termination (`COUNT | EPS`), correlation (computed before the final update) and the Gaussian/mask
   policy are unchanged from Task 014. The native storage pointer is not reallocated for an
   initialized 2x3 CV_32F matrix; the shim still asserts this after the call.
7. `warpAffine` converts the matrix to fixed point: `saturate_cast<int>((M[1]*y + M[2]) * AB_SCALE)`
   with `AB_SCALE = 1 << 10` (4.1.0 `imgwarp.cpp:2184,2213-2214`; 5.0.0 `imgwarp.cpp:1997-2019`).
   `saturate_cast<int>` clamps instead of overflowing, so a huge coefficient saturates and does not
   trigger signed-integer overflow at that conversion. The following adds (`X0 + adelta[x]`) are
   `int` additions; saturated values of both operands could in principle overflow. This is why
   the seed domain is bounded rather than left open.

## Seed-input policy (binding policy, not an OpenCV guarantee)

| Component | Accepted |
|---|---|
| `M00 M01 M10 M11` | finite, `abs <= 4` |
| `M02 M12` | finite, `abs <= 2**20` pixels |

Evidence for these limits: image coordinates are at most 16,383, so with `|M| <= 4` the sampled
coordinate is at most about `4 * 16,383 * 2 + 2**20 ~= 1.18e6`, and `* AB_SCALE (1024)` stays near
`1.2e9`, inside `int32` (2.147e9). Without the bounds (Float32 up to `3.4e38`) the product
saturates, and the later `int` additions could overflow. The bounds were therefore kept at the
proposed values; they were **not loosened** because some larger seed happened to return. They are
conservative with respect to int32 range only; they do not claim convergence.

Measured behavior on the 128x128 texture pair (all three local OpenCV versions; the probe is
`tests/cpp/ecc_affine_seeded_probe.cpp`, mode `domain`):

* At the accepted limits (linear 4, translation 2**20), singular (all zero, rank 1),
  near-singular, tiny scale `1e-6`, shear 4, translation `1e5` and every seed beyond the limits:
  native ECC raised an `OpenCV` error (`ecc.cpp` "The algorithm stopped before its convergence")
  on all three versions. No crash, hang or sanitizer finding occurred. These outcomes are
  recorded as native nonconvergence, **not** as binding defects.
* A reflection seed `[-1 0 0; 0 1 0]` returned a finite matrix on all three versions but with a
  low correlation (0.821). A strongly anisotropic seed `(4, 0.01)` returned a finite, meaningless
  matrix on 4.1.0 (rho 0.068) and 4.10.0 (rho 0.030) and raised on 5.0.0. These are
  wrong-but-finite or version-dependent outcomes; the oracle records them per version.
* No determinant, invertibility or conditioning restriction is applied to the **input**. A
  singular input produced a native exception rather than an unsafe computation, so no extra rule
  is warranted. This is separate from Task 014's output policy, which is unchanged.
* The bound check runs before any native call; failures return `INVALID_ARGUMENT` (C) or an Ada
  `OpenCV_Error` without the shim prefix.

## Limits of this review

* The `int` addition overflow described in item 7 was reasoned from source, not demonstrated; no
  reproducer was constructed, and no undefined behavior was observed in the ASan/UBSan runs. The
  seed bounds make it unreachable under the 16,384-pixel geometry limit.
* Extreme seeds above the bounds were exercised only through direct native calls without
  sanitizer instrumentation of OpenCV itself (the OpenCV libraries are prebuilt).
* The unrelated Farnebäck pointer-formation issue remains separately accepted.
