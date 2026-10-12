# ECC affine source contract (Task 014)

Sources reviewed: `modules/video/src/ecc.cpp` at OpenCV tags **4.1.0, 4.6.0, 4.10.0 and 5.0.0**
(4.6.0 and 4.10.0 are byte-identical; `diff` reports no difference) and the public declarations in
`modules/video/include/opencv2/video/tracking.hpp`. Line numbers are 4.1.0 unless stated.

## Entry point and model

`cv::findTransformECC(template, input, warp, MOTION_AFFINE, criteria, mask, gaussFiltSize)`. In
5.0.0 this overload forwards to `findTransformECCWithMask(template, input, noArray(), mask, ...)`,
so the binding's no-mask call has an empty template mask and an empty input mask. The binding
passes an identity 2x3 `CV_32F` warp, `TermCriteria(COUNT | EPS, n, eps)`, `noArray()`, Gaussian
size 5 and UInt8 C1 images (CPU `Mat` path). `numberOfParameters` is 6 for `MOTION_AFFINE`.

## Warp convention

The warp is applied with `warpAffine(..., INTER_LINEAR + WARP_INVERSE_MAP)` to the *input* image,
so `W` maps **template coordinates into moving coordinates**:

```text
moving_x = M00 * template_x + M01 * template_y + M02
moving_y = M10 * template_x + M11 * template_y + M12
```

Pixels, X right, Y down. The same matrix with `WARP_INVERSE_MAP` resamples the moving image into
the template frame; the binding never transposes, negates or inverts it.

## Jacobian (`image_jacobian_affine_ECC`)

Six image-width blocks (`dst.cols == 6 * src1.cols`, `CV_32FC1`): `Ix*X` (M00), `Iy*X` (M10),
`Ix*Y` (M01), `Iy*Y` (M11), `Ix` (M02), `Iy` (M12). The matrix is `hs x (6 * ws)`; at 16,384
columns the width is 98,304 elements. Under the binding area limit (`rows * cols <= 2^24`) it has
at most `6 * 2^24 = 100,663,296` elements, below `INT_MAX`, and 402,653,184 bytes (384 MiB; the
Euclidean Jacobian is 192 MiB).

## Update mapping (`update_warping_matrix_ECC`)

Identical in all four versions (4.1.0 lines 281-288, 5.0.0 lines 258-265):

```cpp
mapPtr[0] += updatePtr[0];   // M00
mapPtr[3] += updatePtr[1];   // M10
mapPtr[1] += updatePtr[2];   // M01
mapPtr[4] += updatePtr[3];   // M11
mapPtr[2] += updatePtr[4];   // M02
mapPtr[5] += updatePtr[5];   // M12
```

The update vector order (M00, M10, M01, M11, M02, M12) matches the Jacobian block order and
differs from row-major. The update is applied **in place at row-major offsets**, so the stored
warp is the plain row-major 2x3 matrix: the binding reads `warp.ptr<float>()[0..5]` as
`M00 M01 M02 M10 M11 M12` with no reordering. Affine ECC adds updates directly to the matrix
coefficients; the Euclidean `asin` angle rule does not apply.

## Hessian, projection and lambda

`project_onto_jacobian_ECC` builds the 6x6 Hessian (`norm(block)^2` on the diagonal, `dot`
off-diagonal, each cast to `float`) and 6x1 projections. `hessianInv = hessian.inv()` (default
LU) is taken *before* the `cvIsNaN(rho)` test. Guards present in the source:

* `cvIsNaN(rho)` raises `StsNoConv` ("NaN encountered").
* `lambda_d <= 0.0` sets `rho = -1` and raises `StsNoConv`.

Guards **absent**: a very small positive `lambda_d` (huge `lambda`) and a singular Hessian.
Measured with OpenCV 4.10.0: `Mat::inv()` of a singular 6x6 `CV_32F` matrix does not throw; it
returned an all-zero matrix (`/tmp/inv.cpp`, norm 0). A zero inverse gives `deltaP = 0`, so that
path stalls instead of producing nonfinite values. Whether other intermediate overflows can
produce nonfinite coefficients was not demonstrated; the binding verifies all six coefficients and
the correlation are finite after the call, so any such outcome is reported as an error rather than
published. Constant images give NaN norms; the `rho` NaN test turns that into `StsNoConv`.

## Return value

`rho` is computed from the warp at the *start* of the final iteration (before the last update), so
the correlation scores the previous matrix, not the returned one. Exhausting the iteration count
is **not** an error: the loop ends and `rho` is returned. `StsNoConv` arises only from the two
guards above.

## Determinant, reflection and conditioning (measured)

From `tests/cpp/ecc_affine_probe.cpp` and the 32-fixture `ecc_affine_oracle.cpp` on 4.1.0, 4.6.0
(via the 4.6.0 oracle file), 4.10.0 and 5.0.0 (27 fixtures succeed and 5 raise on every version;
the raising ones are `reflect-x`, `shear-0.2`, `strong-1.2-0.8`, `constant`, `noise-vs-noise`):

* Genuine small fits return determinants from **0.9598** (0.98 isotropic scale) to about 1.04
  (1.02 isotropic scale); none was singular or negative.
* The one wrong-but-finite success, `noise-vs-texture`, returned correlation 0.084-0.093,
  `M00` about 1.20-1.21, `M02` about -10.1 to -10.4 and determinant 1.30-1.32.
* A reflection (`det = -1`) and the strong/shear deformations fall outside the
  identity-initialized basin: native ECC raises `StsNoConv` (`lambda_d <= 0`). A reflection is
  therefore not obtainable from identity initialization, although the binding would not reject
  one if native ECC returned it.

Policy: the binding checks warp storage identity, a contiguous 2x3 `CV_32FC1` layout, six finite
coefficients and a finite correlation. It does **not** require `det > 0`, `det = 1`,
orthogonality or a conditioning bound: an affine matrix legitimately carries scale, shear and
reflection, and no demonstrated unsafe arithmetic path was found that a determinant test would
prevent. A caller that resamples with the matrix must itself handle a singular result; the
binding does not resample. No ECC undefined behavior was identified by source review, and no
ECC-specific sanitizer waiver exists (the accepted Farneback upstream finding does not extend
to ECC).
