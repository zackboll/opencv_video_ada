# Masked ECC affine source contract

Status: `SAFETY_GATE_NOT_ESTABLISHED`

This document records the mandatory native-safety investigation for moving-image-mask
overloads of identity-initialized and caller-seeded ECC affine alignment. The gate
fails, so this task adds no Ada API, C export, tests, or public documentation of a
usable binding. Existing ECC, PyrLK, and Farneback APIs are unchanged.

## Scope investigated

The operation is `cv::findTransformECC` with `MOTION_AFFINE` and a moving-image mask
on OpenCV 4.1, and the same operation through `findTransformECCWithMask` on OpenCV
4.6, 4.10, and 5.0. Unmasked affine calls were included because the optimizer uses
the same warp path. The relevant native code is `modules/video/src/ecc.cpp`: each
iteration updates the six-parameter warp and immediately calls `warpAffine` on the
image, both gradients, and the mask.

No public raw flags, Farneback, Kalman, CamShift, meanShift, UMat/OpenCL, VideoIO,
feature detection, Calib3D, navigation fusion, or estimator policy was added.

## Fixed-point operation

All four versions implement inverse-map `warpAffine` in fixed point. For each output
row `y` and column `x`, the scalar path computes:

```cpp
int X0 = saturate_cast<int>((M[1] * y + M[2]) * 1024) + 16;
int Y0 = saturate_cast<int>((M[4] * y + M[5]) * 1024) + 16;
int X = (X0 + adelta[x]) >> (AB_BITS - INTER_BITS);
int Y = (Y0 + bdelta[x]) >> (AB_BITS - INTER_BITS);
```

`adelta[x]` is `cvRound(M[0] * x * 1024)`, `bdelta[x]` is `cvRound(M[3] * x * 1024)`,
and `AB_BITS - INTER_BITS` is 5 for linear interpolation. Nearest interpolation uses
the same addition with a shift of 10. The additions are signed `int` additions in:

- OpenCV 4.1: `imgwarp.cpp:2285-2286` and `imgwarp.sse4_1.cpp:204-205`
- OpenCV 4.6: `imgwarp.cpp:2269-2270` and `imgwarp.sse4_1.cpp:204-205`
- OpenCV 4.10: `imgwarp.cpp:2373-2374` and `imgwarp.sse4_1.cpp:204-205`
- OpenCV 5.0: `imgwarp.cpp:2388-2389`, with AVX2 additions in `imgwarp.avx2.cpp:67-70`

`saturate_cast<int>` clamps the row base, but it does not protect the subsequent
addition. A saturated `X0` or `Y0` plus a large delta can therefore overflow. An
absolute-magnitude estimate is not the criterion used here: both terms were evaluated
with their actual signs, and only a sum outside `[INT_MIN, INT_MAX]` was treated as
unsafe.

## Instrumentation

The checked OpenCV builds were the existing RelWithDebInfo builds at
`~/.cache/video-task017/install-<version>-san`. Their compile lines include
`-fsanitize=address,undefined,float-cast-overflow`, IPP, ITT, OpenCL, and
`opencv_world` are off. A direct `warpAffine` probe with an extreme coefficient
produced this report, proving that the installed libraries were actually instrumented:

```text
imgwarp.cpp:2285: signed integer overflow: -2147483648 + -2147483632
```

OpenCV 5 annotates `warpAffineBlockline` and `warpAffineBlocklineNN` with
`CV_DISABLE_UBSAN`, which expands to `no_sanitize("undefined")`. Its absence of a
UBSan diagnostic is therefore not evidence that the addition is safe.

## ECC-generated failures

A deterministic synthetic sweep used the repository's checker texture, 600 trials,
100 iterations, and both identity and finite caller seeds. It covered masked and
unmasked calls at 48x81, 64x97, 64x96, and 64x100. The diagnostic copied each
iteration matrix through `CV_32F`, the precision ECC stores, and evaluated the exact
signed sums at the corners.

The following stored matrices produced sums outside int32. Replaying each exact
matrix through the instrumented `warpAffine` reproduced UBSan reports on 4.1, 4.6,
and 4.10:

| Version and call | Geometry | Stored matrix | Signed result | UBSan |
| --- | --- | --- | --- | --- |
| 4.1 masked identity seed | 64x97 | `[22298.8203125, -57255.54296875, 225348.015625, 137349.0625, -361430.03125, -3676567.25]` | `Y=-3764804848` at `(0,0)` | yes |
| 5.0 unmasked finite seed | 64x100 | `[-5597.05908203125, 8140.32177734375, 285284.03125, -9895.33203125, -362406, 484909.9375]` | `Y=-22882988080` at `(0,63)` | suppressed on 5.0; reported on 4.1/4.6/4.10 |

The first matrix arises during an ordinary masked identity-seeded call. The second
shows that the same hazard also occurs without a mask. Additional stored matrices
with coefficients above about 500,000 saturated before the addition, so UBSan did not
report a new overflow for those particular additions; saturation does not establish a
safe range for the other additions.

No successful result in the sweep was non-finite, but that does not qualify the path.
The unsafe additions occur on intermediate matrices before ECC rejects the iteration
with `NaN` or non-convergence. A caller-side check of only the initial seed or the
final returned matrix cannot prevent them.

## Gate decision

`SAFETY_GATE_NOT_ESTABLISHED`

Unconstrained affine optimizer updates can reach `warpAffine` with finite `CV_32F`
matrices whose signed fixed-point sums overflow int32. This was reproduced from ECC
itself and confirmed with UBSan on every supported OpenCV release that does not
suppress the check. OpenCV 5 suppresses the diagnostic around the same operation.
The required masked identity and seeded affine overloads must not be exposed.
