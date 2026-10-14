# ECC homography source contract and safety gate (Task 016)

Sources reviewed: `modules/video/src/ecc.cpp`, `modules/video/include/opencv2/video/tracking.hpp` and
`modules/imgproc/src/imgwarp.cpp` at OpenCV tags **4.1.0, 4.6.0, 4.10.0 and 5.0.0**. `ecc.cpp` is
byte-identical between 4.6.0 and 4.10.0. 4.1.0 differs from 4.10.0 only in comments and in
`computeECC` (unused by the binding). 5.0.0 forwards `findTransformECC` through
`findTransformECCWithMask` (empty template mask), uses `addWeighted` instead of `src*h + ...` in the
Jacobian, and relaxes the Jacobian type assertion to `CV_MAKETYPE(CV_32F, src1.channels())`. The
homography update mapping, the `lambda_d <= 0` guard and the `cvIsNaN(rho)` guard are the same in all
four versions. The public seven-argument overload exists in all four.

## Disposition

**`SAFETY_GATE_ESTABLISHED`** for the identity-initialized, unmasked, CPU, UInt8 C1 call, on the
evidence below. This is a memory-safety and bounded-arithmetic conclusion only. It does **not** claim
convergence, alignment accuracy or a well-conditioned projective mapping.

Not established: sanitizer cleanliness of upstream OpenCV. The prebuilt OpenCV libraries are not
instrumented and no instrumented OpenCV was built for this gate (see "Unproven").

## Model and storage

`MOTION_HOMOGRAPHY`, 8 parameters, 3x3 `CV_32FC1` warp (`findTransformECC` asserts `map.rows == 3` for
this model). The warp maps **template coordinates to moving coordinates** and is applied with
`warpPerspective(..., INTER_LINEAR + WARP_INVERSE_MAP)` to the input image:

```text
D = H20*x + H21*y + H22
moving_x = (H00*x + H01*y + H02) / D
moving_y = (H10*x + H11*y + H12) / D
```

`update_warping_matrix_ECC` (identical in all four versions) adds the 8-vector in place at row-major
offsets: `update[0]->H00 [1]->H10 [2]->H20 [3]->H01 [4]->H11 [5]->H21 [6]->H02 [7]->H12`
(`mapPtr[0,3,6,1,4,7,2,5]`). `mapPtr[8]` (H22) is never written, so it stays exactly 1 for an
identity-initialized call. The stored matrix is row-major `H00 H01 H02 H10 H11 H12 H20 H21 H22`; the
binding returns storage order, not update-vector order. `image_jacobian_homo_ECC` reads
`hptr[0,3,6,1,4,7,2,5]` as h0..h7.

## Projective denominator

`image_jacobian_homo_ECC` builds `den = Xgrid*H20 + Ygrid*H21 + 1.0` over the image and divides
`hatX`, `hatY`, `Ix` and `Iy` by it with `cv::divide`.

* No guard in `ecc.cpp` prevents `den` from being zero or tiny.
* `cv::divide` on `CV_32F` follows IEEE (measured on 4.1.0, 4.10.0, 5.0.0): `1/0 = +inf`,
  `-1/0 = -inf`, `0/0 = NaN`, `1e30/1e-30 = +inf`. It does not throw.
* Non-finite values reach the Jacobian, the 8x8 Hessian (`norm`/`dot`) and the projections.
* `Mat::inv()` (default LU) of a singular 8x8 `CV_32F` matrix returned an all-zero matrix without
  throwing (measured, norm 0; `1e-30*I` also gave 0). A zero inverse gives `deltaP = 0`.

## Native resampling (`warpPerspective`)

Reviewed in 4.1.0, 4.6.0, 4.10.0 (`WarpPerspectiveInvoker`, scalar and SIMD lines) and 5.0.0
(`hal::warpPerspectiveBlockline[NN]`, same logic).

* `W = W0 + M[6]*x1; W = W ? INTER_TAB_SIZE / W : 0;` A zero denominator selects scale 0; no division
  by zero (SIMD: `v_select(v_ne(W, 0), its / W, 0)`).
* Projected coordinates are clamped in double to `[INT_MIN, INT_MAX]` before `saturate_cast<int>`
  and `saturate_cast<short>`. The coordinate buffers are fixed-size stack arrays
  (`short XY[2048], A[1024]`), independent of image size and matrix. `remap` applies the border policy.
* Measured on 4.1.0, 4.10.0, 5.0.0: M[6]=M[7]=1e30, 1e300 coefficients, an all-zero last row and a NaN
  coefficient all returned normally (no exception, no signal).
* `remap` asserts coordinate limits: a 65536x256 image raised `imgwarp.cpp:1813` (`cv::Exception`) on
  4.10.0; 16384x1024 succeeded.

## Hessian, lambda and the failure paths

Guards present (all four versions): `cvIsNaN(rho)` raises `StsNoConv`; `lambda_d <= 0.0` sets
`rho = -1` and raises `StsNoConv`.

Guards absent: any finite or magnitude check on `deltaP`, `hessianInv` or `lambda`; a tiny positive
`lambda_d` yields a huge `lambda` and a huge update; infinite `rho` is not rejected; a NaN `lambda_d`
passes the `<= 0` test. The last update is applied after `rho` is computed, so the returned matrix was
never scored.

So a large or non-finite intermediate matrix is possible in the source. The next iteration only passes
it to `warpPerspective` (bounded above) and the Jacobian code (floating point, no matrix-derived
indexing). A NaN warp gives NaN statistics and the `cvIsNaN(rho)` guard. The binding's final-matrix
finite check protects only the returned value; this document does not rely on it for memory safety.

## Integer and allocation arithmetic

* Jacobian `Mat(hs, ws*8, CV_32F)`: width 16384 gives 131072 columns; at the area limit
  (`rows*cols <= 2^24`) it has `2^27` elements (512 MiB), below `INT_MAX`. `colRange` offsets reach
  `8*w <= 131072`.
* Other allocations are `hs*ws` Float32/UInt8 planes (grids, template/image float copies, gradients,
  warped planes, error, `den`, `hatX`, `hatY`, divided gradients, masks). `Mat` step products use
  `size_t`. Allocation failure raises `cv::Exception`/`std::bad_alloc`.

## Measured native behavior (`tests/cpp/ecc_homography_probe.cpp`)

128x128 analytic texture, 50 iterations, eps 1e-4, one process per case, `ulimit -v 6 GB`,
`timeout 90`. Measured on 4.1.0, 4.10.0 and 5.0.0 on this host. **4.6.0 was not measured**: no
container runtime is available on this host, so its native behavior is only source-reviewed here.
Every case ran to normal completion; none crashed or hung.

* **Converged from identity** on all three versions: identity, translation (+2,+1), rotation 2 deg,
  scale 1.02, nonuniform scale, shear 0.02, affine combination, mild perspective, H20 and H21 at
  +/-0.0002, both terms, rotation+perspective, scale/shear/translation+perspective. Interior residual
  with the returned matrix and `WARP_INVERSE_MAP` was 0.25-0.29 gray levels, close to the true matrix
  (0.24-0.28).
* Returned coefficients are **not exact**: translation gave H02 = 2.0475 and H12 = 1.0478; max|W-H|
  was 0.003-0.048 for converged cases. Assertions must use measured tolerances.
* H22 was exactly 1.0 in every success on every version.
* **Homography beats affine** on perspective fixtures (4.10.0 residual, homography vs affine):
  persp-h20-pos 0.284 vs 1.259; persp-both 0.285 vs 1.544; persp-mild 0.288 vs 0.397; rot-persp
  0.286 vs 0.760; sst-persp 0.277 vs 0.763. They are equal for pure affine fixtures.
* **Native nonconvergence (`StsNoConv`)** on all three versions: persp-strong (H20 = H21 = 0.002),
  persp-near-zero-den (H20 = -0.0078), persp-den-crosses-zero (H20 = -0.012), singular-rank2,
  large-update (40 px, 30 px), reflect-x, constant. Both guards were exercised (NaN guard and
  `lambda_d <= 0`).
* **Incorrect-but-finite success**: illcond (H11 = 1e-6) on 4.10.0 returned correlation 0.0968,
  H00 = 3.81, H02 = -193.8, H20 = 0.0067 and a worse residual (96.5) than identity (37.6). On 4.1.0
  and 5.0.0 it raised. noise-vs-noise returned correlation 0.503 on all three versions.
* No case in this list reached `den = 0` inside the image with a finite success; the near-zero and
  crossing cases were stopped by the native guard.

## Memory (direct native, 4.10.0, x86_64, 10 iterations, peak RSS, MiB)

| Size | Translation | Euclidean | Affine | Homography |
|---|---:|---:|---:|---:|
| 128x128 | 21 | 22 | 22 | 22 |
| 1024x1024 | 81 | 102 | 97 | 137 |
| 2048x2048 | 262 | 342 | 326 | 485 |
| 4096x4096 (2^24 px) | 984 | not measured | 1,239 | 1,879 |
| 16384x1024 (2^24 px) | not measured | not measured | 1,239 | 1,879 |

At 4096x4096: affine 1.47 s, homography 2.19 s. The 512 MiB Jacobian is about 27% of the homography
peak. No allocation failure occurred. 65536x256 fails in `remap` (outside the 16,384 limit).

## Input contract

The existing ECC envelope (non-empty 2-D UInt8 C1, matching size, rows and columns 32-16,384, area at
most 2^24, iterations 1-100, finite epsilon 1e-8..0.1, Gaussian size 5, no mask) has no integer
overflow with eight parameters and a measured peak of 1.88 GiB at the area limit. Memory failure is a
`cv::Exception`/`std::bad_alloc`, not undefined behavior. No homography-specific reduction is needed.

## Unproven

1. Upstream OpenCV was not sanitizer-instrumented; the probes show behavior, not absence of UB. The
   NaN-to-int conversion in `warpPerspective` is clamped by `max/min`, but this was not verified under
   an instrumented OpenCV.
2. 4.6.0 was unmeasured by the Checkpoint 1 probe. In Checkpoint 4 the full oracle, boundary,
   fault and sanitizer campaign ran on 4.6.0 in a local Debian 12 container. Its 37 oracle lines are
   byte-identical to 4.10.0; see `docs/task016-qualification.md`. The probe itself was not rerun on
   4.6.0.
3. Convergence basin from identity is small (about 2% scale/shear, 2 deg, a few pixels, perspective
   coefficients about 2e-4 at 128 px). Caller seeding is out of scope.
4. Points with `D = 0` or near 0 are a property of the returned matrix, not of call memory safety. The
   binding does not resample and must not silently convert them.

## Checkpoint 3 boundary and fault qualification (4.10.0)

`run_ecc_homography()` in `tests/cpp/native_boundary.cpp` calls the real
`opencv_video_estimate_ecc_homography` export through Core-owned Mat handles. It compares the result
with its own independent `cv::findTransformECC(MOTION_HOMOGRAPHY)` call: identity 3x3 warp,
`COUNT|EPS`, no mask, Gaussian size 5.

* **Exact match on the same build.** On the deterministic CPU path the nine coefficients and the
  correlation match bitwise (`memcmp`). This holds for all eleven geometric fixtures, the option
  boundaries, the Regions and the three known wrong-but-finite successes on 4.10.0.
  On 4.1.0 and 5.0.0, low-gradient and ill-conditioned raise natively (Checkpoint 4). The boundary
  now follows the same-build direct outcome for those three fixtures:
  * when native succeeds, the result must be published bitwise-equal;
  * when native raises, the 48 bytes must be untouched;
  * at least one wrong-but-finite success is still required (noise-vs-texture).
* **Byte-exact failure atomicity.** Every expected failure leaves the caller's whole 48-byte object
  unchanged, including the 4 padding bytes at offset 36. This was checked with a patterned sentinel.
* **Homography-only fault hooks.** Only the homography export calls them, they are compiled only under
  `OPENCV_VIDEO_TEST_FAULTS`, and they run after fault stage 2 and before validation:
  * `opencv_video_test_corrupt_homography(float *warp, double *correlation)` receives the private
    3x3 Float32 warp, which has nine valid positions.
  * `opencv_video_test_homography_schema(cv::Mat &)` can replace the private header using only
    well-defined `cv::Mat` operations.
* **Generic hook unchanged.** The 2x3 `opencv_video_test_corrupt_ecc` modes are untouched and the
  homography export no longer calls that hook. The boundary test checks that generic modes do not
  affect homography output.
* **Schema mutation 5 (non-continuity).** It uses a column view of a fresh 3x4 allocation. A
  non-continuous header over the original 36-byte storage would read past the allocation, so
  non-continuity cannot be tested apart from storage identity without undefined behavior.

### Mutation campaign

**Setup.**
* Mutants were applied to a scratch copy of `934517b` plus the Checkpoint 3 harness. They were never
  applied to the worktree.
* Every edit pattern was required to match exactly once between
  `opencv_video_estimate_ecc_homography(` and the next export.
* AUnit ran against pristine oracle files that were not regenerated.

**Counting rule.** A mutant counts as KILLED only if it built, a test ran, and a named check failed.

**Baseline.** Mutant 00 is an unmodified build. It passed AUnit 176/176 and both boundary drivers.

| # | Mutation | Result | First named failure |
|---|---|---|---|
| 1 | `MOTION_AFFINE` | killed | `ECC_Homography_Direct_Oracle`: binding raised where native succeeded; boundary `identity` |
| 2 | 2x3 private warp | killed | same as 1 |
| 3 | identity without ECC | killed | oracle H00 differs `translation-2-1`; boundary `identity` |
| 4 | ignore native output | killed | oracle H00 differs `translation-2-1` |
| 5 | swap H02/H20 | killed | oracle H02 differs |
| 6 | swap H12/H21 | killed | oracle H12 differs |
| 7 | swap H01/H10 | killed | oracle H01 differs; `ECC_Homography_Inverse_Warp` |
| 8 | negate H20 | killed | oracle H20 differs |
| 9 | negate H21 | killed | oracle H21 differs |
| 10 | zero H20/H21 | killed | oracle H20 differs |
| 11 | publish H22 = 0 | killed | oracle "H22 is not exactly 1" |
| 12 | force H22 = 1 after the H22 check | **equivalent** | none; the check already guarantees `w[8] == 1.0f`, so the forced value equals the observed one |
| 12b | remove the H22 check and force H22 = 1 | killed | fault boundary: "corrupted native output accepted" (H22 = 0 / 1.01 modes) |
| 13 | fixed correlation | killed | oracle correlation differs |
| 14 | ignore iterations | killed | oracle H00 differs `it1`; boundary "option boundary differs" |
| 15 | ignore epsilon | killed | oracle H01 differs `it100-e8` |
| 16 | swap roles | killed | oracle H00 differs; boundary `translation (2,1)` |
| 17 | publish before validation | killed | fault boundary: "corrupted output was partially published" |
| 18 | no H20 finiteness check | killed | fault boundary: "corrupted native output accepted" |
| 19 | no correlation finiteness check | killed | fault boundary: "corrupted native output accepted" |
| 20 | suppress nonconvergence | killed | oracle "binding succeeded where native raised: constant" |

Mutants 17-19 change final-output paths that real native results cannot reach on 4.10.0. AUnit and
the production boundary still pass with them, and only the injected corruption detects them.

## Can a production binding be defended?

Yes, under the contract above. Every path found ends in a native exception, clamped sampling, or a
finite-check failure at the binding boundary; none was found to read or write outside a buffer.
Re-examine if the sanitizer campaigns or an instrumented OpenCV show otherwise.
