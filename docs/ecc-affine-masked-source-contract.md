# Masked ECC affine source-level safety contract

## Disposition

`SAFETY_GATE_NOT_ESTABLISHED`

No Ada overload, C export, or test was added. Unconstrained `MOTION_AFFINE`
updates can pass `findTransformECC` a finite `CV_32F` matrix whose inverse-map
`warpAffine` evaluation executes signed `int` addition overflow. This was
reproduced on the instrumented OpenCV 4.1.0, 4.6.0, and 4.10.0 builds. OpenCV
5.0.0 contains the same addition, but `CV_DISABLE_UBSAN` suppresses the
diagnostic; that suppression does not establish defined behavior.

## ECC entry points

The public declarations are in `modules/video/include/opencv2/video/tracking.hpp`.
Their implementations are in `modules/video/src/ecc.cpp`.

| Version | Public ECC entry points |
| --- | --- |
| 4.1.0 | `findTransformECC`, declared at lines 336 and 343. `findTransformECCWithMask` does not exist. |
| 4.6.0 | `findTransformECC`, declared at lines 336 and 343. `findTransformECCWithMask` does not exist. |
| 4.10.0 | `findTransformECC`, declared at lines 336 and 343. `findTransformECCWithMask` does not exist. |
| 5.0.0 | `findTransformECC` at lines 374 and 381, `findTransformECCWithMask` at line 422, and `findTransformECCMultiScale` at line 506. |

On 4.1.0, the six-argument overload at `ecc.cpp:592-598` forwards to the
seven-argument implementation at line 343, supplying `gaussFiltSize = 5`.
On 4.6.0 and 4.10.0, that forwarding overload is at lines 609-615 and the
implementation is at line 360. Thus input masks are available through the
ordinary `findTransformECC` API on all three 4.x versions.

On 5.0.0, `findTransformECC` at `ecc.cpp:614-622` creates an empty template
mask with `noArray()` and forwards to `findTransformECCWithMask` at line 337.
The compatibility overload at lines 626-629 forwards to the seven-argument
`findTransformECC`. `findTransformECCMultiScale` is a separate pyramid API and
was not used by the reproducer.

## Warp evaluation

`findTransformECC` selects the geometric warp directly. It does not occur
inside `image_jacobian_affine_ECC`; that function only builds the affine
parameter Jacobian after the image and gradients have been warped.

When `motionType != MOTION_HOMOGRAPHY`, the implementation calls `warpAffine`
for the image and both gradients with `INTER_LINEAR + WARP_INVERSE_MAP`, then
calls it for the mask with `INTER_NEAREST + WARP_INVERSE_MAP`. This branch
covers translation, Euclidean, and affine motion. The calls are at:

- 4.1.0 `ecc.cpp:506-509`, inside `findTransformECC` beginning at line 343
- 4.6.0 and 4.10.0 `ecc.cpp:523-526`, inside `findTransformECC` beginning at line 360
- 5.0.0 `ecc.cpp:523-526`, inside `findTransformECCWithMask` beginning at line 337

The homography branch instead calls `warpPerspective` at 4.1.0 `ecc.cpp:513-516`,
4.6.0/4.10.0 `ecc.cpp:530-533`, and 5.0.0 `ecc.cpp:528-531`. That perspective
implementation is outside the signed `adelta`/`bdelta` addition reproduced
below, so this `warpAffine` overflow does not implicate homography.

For every tested version, that setup is:

```cpp
const int AB_BITS = MAX(10, (int)INTER_BITS);
const int AB_SCALE = 1 << AB_BITS;
adelta[x] = saturate_cast<int>(M[0] * x * AB_SCALE);
bdelta[x] = saturate_cast<int>(M[3] * x * AB_SCALE);
```

`INTER_BITS` is 5, so `AB_BITS` is 10 and `AB_SCALE` is 1024. For each
destination row, the scalar base values are:

```cpp
int X0 = saturate_cast<int>((M[1] * y + M[2]) * AB_SCALE) + round_delta;
int Y0 = saturate_cast<int>((M[4] * y + M[5]) * AB_SCALE) + round_delta;
```

`round_delta` is `AB_SCALE / 2` (512) for nearest sampling and
`AB_SCALE / INTER_TAB_SIZE / 2` (16) for linear sampling.

`saturate_cast<int>(double)` calls `cvRound`. On this x86-64 GCC build,
`cvRound(double)` uses `_mm_cvtsd_si32`. Its contract says that a value outside
`INT_MIN .. INT_MAX` has an undefined result. The following coordinate-table
additions are separate operations and require their own signed-range check;
saturating one operand does not make `X0 + adelta[x]` or `Y0 + bdelta[x]` safe.

The scalar linear additions are:

| Version | Source lines |
| --- | --- |
| 4.1.0 | `imgwarp.cpp:2285-2286` |
| 4.6.0 | `imgwarp.cpp:2269-2270` |
| 4.10.0 | `imgwarp.cpp:2373-2374` |
| 5.0.0 | `imgwarp.cpp:2437-2438`, inside `CV_DISABLE_UBSAN` at line 2394 |

The scalar nearest additions are 4.1.0 lines 2244-2245, 4.6.0 lines 2229-2230,
4.10.0 lines 2329-2330, and 5.0.0 lines 2387-2388. The 5.0.0 nearest function
starts at `imgwarp.cpp:2364`, and its linear counterpart starts at line 2395.
Both carry `CV_DISABLE_UBSAN` on their preceding lines, 2363 and 2394.

SIMD paths perform the same signed addition. The SSE4.1 nearest implementation
executes `_mm_add_epi32` at `imgwarp.sse4_1.cpp:204-205` in all four versions.
OpenCV 5.0.0 also has scalar additions and SIMD `v_add` calls in
`warpAffineBlockline`, all covered by the function-level suppression above.

## Reproduced operands

The compiler is Debian `g++` 14.2.0. The probe links the previously built
ASan/UBSan OpenCV installations and calls `warpAffine` directly with
`setNumThreads(1)`. `UBSAN_OPTIONS` is `print_stacktrace=1:halt_on_error=0`.

Both matrices below are the exact stored `CV_32F` values from the earlier ECC
sweep. Matrix A came from the masked 64×97 run, and matrix B from the unmasked
seeded 64×100 run. They are therefore actual ECC-generated optimizer matrices,
not arbitrary synthetic counterexamples. Replaying the stored values preserves
the evidence without rerunning the nondeterministic search.

### Matrix A — masked 64×97

```text
[22298.8203125, -57255.54296875, 225348.015625,
 137349.0625,   -361430.03125,  -3676567.25]
```

At destination row 4, column 96, linear mode:

- floating-point X coordinate:
  `(-57255.54296875 * 4 + 225348.015625) * 1024 = -3762336`
- `saturate_cast<int>` gives `-3762336`; adding delta 16 gives `X0 = -3762320`
- `adelta[96] = saturate_cast<int>(22298.8203125 * 96 * 1024) = -2147483648`
- `int64_t` sum: `-3762320 + -2147483648 = -2151245968`

The corresponding Y pre-conversion is `-5245222272`. Its saturated base plus 16
is `Y0 = -2147483632`, and `bdelta[96] = -2147483648`. Their `int64_t` sum is
`-4294967280`.

UBSan reports exactly these additions. The 4.1.0 linear diagnostic is:

```text
imgwarp.cpp:2285:41: signed integer overflow:
-2147483648 + -3762320 cannot be represented in type 'int'
imgwarp.cpp:2286:41: signed integer overflow:
-2147483648 + -2147483632 cannot be represented in type 'int'
```

The same operands are reported at 4.6.0 lines 2269-2270 and 4.10.0 lines
2373-2374. The nearest SSE4.1 path reports
`-2147483648 + -3761824` and `-2147483648 + -2147483632` at lines 204-205. The
512-unit difference is the nearest-mode rounding delta, so those are different
integer additions, not alternate interpretations of the linear operands.

### Matrix B — unmasked seeded 64×100

```text
[-5597.05908203125, 8140.32177734375, 285284.03125,
 -9895.33203125,   -362406,           484909.9375]
```

At destination row 5, column 96, linear mode:

- floating-point Y coordinate:
  `(-362406 * 5 + 484909.9375) * 1024 = -1358970944`
- `saturate_cast<int>` gives `-1358970944`; adding delta 16 gives
  `Y0 = -1358970928`
- `bdelta[96] = saturate_cast<int>(-9895.33203125 * 96 * 1024) = -972750720`
- `int64_t` sum: `-1358970928 + -972750720 = -2331721648`

This Y coordinate is representable as a signed 32-bit integer before addition.
Classifying the floating-point coordinate itself as an overflow would therefore
be incorrect. The overflow is the later signed addition of `Y0` and
`bdelta[96]`.

The X operands do not overflow: the pre-conversion value is `333809295.5`,
`X0 = 333809312`, `adelta[96] = -550213296`, and their `int64_t` sum is
`-216403984`.

UBSan on 4.1.0, 4.6.0, and 4.10.0 reports:

```text
signed integer overflow:
-972750720 + -1358970928 cannot be represented in type 'int'
```

The source locations are the scalar linear Y additions at 4.1.0 line 2286,
4.6.0 line 2270, and 4.10.0 line 2374. The SSE4.1 nearest path reports
`-972750720 + -1358970432` at `imgwarp.sse4_1.cpp:205`, again reflecting its
512-unit rounding delta.

## OpenCV 5.0 behavior

The 5.0.0 probe calculates the same overflowing `int64_t` sums for both
matrices, but its UBSan log contains no signed-overflow report. This is
expected: both `warpAffineBlocklineNN` and `warpAffineBlockline` are annotated
`CV_DISABLE_UBSAN`. The macro expands to a no-sanitize function attribute for
the undefined-behavior sanitizer. It disables detection around the operation;
it does not change the signed `int` addition into defined wraparound or
saturated arithmetic.

## API implication

Matrix B was generated by the existing unmasked seeded affine ECC path. The
failed gate therefore implicates that existing affine API, not only a proposed
masked overload. Translation and Euclidean motion use the same non-homography
`warpAffine` branch, but these reproducers do not exercise their parameter
updates, so those models require separate operand-level audits. This result
does not implicate homography: its branch calls `warpPerspective` rather than
the reproduced `warpAffine` fixed-point addition.

The masked affine API remains blocked. No production binding, public Ada API,
C export, or test was changed, and no sanitizer or test was suppressed to make
this gate pass.
