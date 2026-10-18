# Task 021 — Translation and Euclidean ECC native arithmetic safety audit

Branch `safety/021-translation-euclidean-ecc-audit`, from verified `origin/main`
`6915ddc7a46cad41c37620b72a63f2b1b81dd043` (the PR #20 merge). Crate version `0.1.0-dev`,
Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326`, 25 production exports and 198 AUnit
registrations are all **unchanged**. No dependency, CI topology or production behavior
changed: the only edits to `src/` and `README.md` are comments and documentation.

This task audits whether the native signed-integer-overflow hazard proven for affine ECC in
Task 020 also affects the existing translation and Euclidean operations. It does **not**
re-open Task 020; the affine finding (`KNOWN_NATIVE_UB_UNMITIGATED`) and its notices remain
intact.

## Dispositions (kept separate; one unsafe model does not implicate the others)

| API | C export | Disposition |
|---|---|---|
| Translation identity | `opencv_video_estimate_ecc_translation` | **`SAFETY_GATE_NOT_ESTABLISHED`** |
| Euclidean identity | `opencv_video_estimate_ecc_euclidean` | **`SAFETY_GATE_NOT_ESTABLISHED`** |
| Euclidean seeded | `opencv_video_estimate_ecc_euclidean_seeded` | **`SAFETY_GATE_NOT_ESTABLISHED`** |

None reached `KNOWN_NATIVE_UB_UNMITIGATED`: unlike affine, **no model-valid ECC execution
(Level B) or real binding call (Level C) was shown to reach the overflow** for these two
models, on any of the four versions. None reached `SAFETY_GATE_ESTABLISHED` either: the
optimizer's accumulated translation is not bounded by any source-level argument across the
published envelope, and a model-valid extreme matrix **does** overflow the same native
`warpAffine` fixed-point path (Level A). `NOT_REPRODUCED` is not proof of safety.

**Task 021-R1 (2026-10-18).** The first audit missed an earlier signed addition: the row-base
rounding `X0/Y0 = cvRound(...) + round_delta`. It wrongly concluded that only the X addition
can overflow for translation. The row-base overflow is now reproduced natively on 4.1.0,
4.6.0 and 4.10.0, at an ordinary 64x97 image, for Y and X, using the model-valid translation
`[1 0 0; 0 1 2097151.875]` and its X twin (`INTER_NEAREST`, `512 + 2147483520`). It is also
reproduced for a Euclidean rotation in linear mode. It is identified as the **first**
undefined operation there (section 3). It is Level A evidence only, and it does not change
any disposition.

Hosted CI status is in [the qualification report](../task021-qualification.md).

## 1. Repository verification

* `origin/main` SHA `6915ddc7a46cad41c37620b72a63f2b1b81dd043`, PR #20 (`safety/020-existing-affine-ub`)
  merged 2026-10-18.
* PR #20 post-merge CI read once: **Cross-platform Video succeeded**; **Windows post-merge
  Video failed in the `Upgrade MSYS2 system coherently` step** with
  `failed retrieving file ... Operation too slow ... error: failed to commit transaction
  (download library error)` — a transient MSYS2 mirror/download failure before any build or
  test, on `6915ddc`, not a code regression (the previous head `b09a65a` succeeded). It is
  push-to-main only and unrelated to this task. No baseline correction was needed.
* No pre-existing Task 021 branch, worktree or PR. Working tree clean. A dedicated worktree
  `/home/zboll/git/opencv/video-task021` was created on the new branch.
* Expected baseline confirmed by `scripts/check_repository.py`: 25 ABI declarations/imports,
  198 AUnit registrations, Core pin `4da9d35ea21e`, crate `0.1.0-dev`.

## 2. Source-level audit — all four versions

`modules/video/src/ecc.cpp` was read at the exact upstream tags 4.1.0, 4.6.0, 4.10.0 and
5.0.0. `update_warping_matrix_ECC` and the main loop are identical across 4.1.0/4.6.0/4.10.0
(byte-identical for 4.6.0 and 4.10.0); 5.0.0 only reformats and moves the body into
`findTransformECCWithMask`. On the non-homography path every version resamples four images
per iteration with `warpAffine(..., WARP_INVERSE_MAP)` and the current `CV_32F` 2x3 warp
(4.1.0 `ecc.cpp:506-509`; 4.6.0/4.10.0 `523-526`; 5.0.0 `523-526`):

1. the smoothed moving image (`CV_32FC1`, `INTER_LINEAR`);
2. the X gradient (`CV_32FC1`, `INTER_LINEAR`);
3. the Y gradient (`CV_32FC1`, `INTER_LINEAR`);
4. the effective mask (`CV_8UC1`, `INTER_NEAREST`) — for the unmasked API this is an
   internally generated all-ones `preMask`.

### Translation update model

`update_warping_matrix_ECC` for `MOTION_TRANSLATION` writes only the translation column
(identical in all four versions):

```cpp
mapPtr[2] += updatePtr[0];   // tx
mapPtr[5] += updatePtr[1];   // ty
```

Entries 0, 1, 3 and 4 are never written, so the linear part stays the private identity on
**every** iteration. The shim re-checks this after the call (`opencv_video_shim.cpp:713`).
For the fixed-point math (section 3) this means `adelta[x] = x*1024` and `bdelta[x] = 0` for
every iterate. Only `tx` and `ty` change.

> **Correction (Task 021-R1, 2026-10-18).** The original text continued "and only the X
> addition can overflow". **That was wrong.** `bdelta[x] = 0` makes the per-column Y
> addition `Y0 + bdelta[x]` trivially safe, but it says nothing about the **row base**
> `Y0 = cvRound((y + ty)*1024) + round_delta`, which is itself a signed `int` addition. For
> `ty` just below `2^21` that addition overflows before any column delta is applied. It was
> reproduced natively on 4.1.0, 4.6.0 and 4.10.0 for both X0 and Y0 (section 3, "Row-base
> rounding overflow").

### Euclidean update model

```cpp
double new_theta = updatePtr[0];
new_theta += asin(mapPtr[3]);
mapPtr[2] += updatePtr[1];
mapPtr[5] += updatePtr[2];
mapPtr[0] = mapPtr[4] = (float)cos(new_theta);
mapPtr[3] = (float)sin(new_theta);
mapPtr[1] = -mapPtr[3];
```

* **Bounded rotation coefficients.** For any finite `new_theta`, the Float32 casts give
  `|cos|, |sin| <= 1`. The per-column deltas are then bounded: `|adelta[x]|, |bdelta[x]| <=
  x*1024 <= 16_775_168` for `x <= 16383`. That bounds the **linear part only**. The full
  coordinate also contains the row base `(M[1]*y + M[2])*1024`, which carries the
  accumulated `tx`/`ty` and is not bounded by the rotation.
* **Accumulated translation.** `tx` and `ty` accumulate exactly as in translation and
  affine. Nothing in the loop limits them.
* **Float32 rounding.** `a == d` and `b == -c` hold bit-for-bit. Rounding changes the
  rotation coefficients by at most about 6e-8, which does not affect the overflow analysis.
* **`asin` domain.** The argument to `asin` is always a previously stored
  `(float)sin(...)`. Correctly rounded `sinf` values lie in `[-1, 1]`, so `asin` stays in
  its domain. The principal-branch ambiguity beyond a quarter turn limits convergence
  (Task 012); it is not an arithmetic hazard.
* **Nonfinite updates.** If `deltaP` were NaN, the next `warpAffine` would consume a NaN
  map before `rho` raised `StsNoConv "NaN encountered"`. On x86, `cvRound(NaN*1024)` is
  `cvtsd2si -> INT_MIN`. What happens next depends on the model:
  * **Translation.** Only `tx`/`ty` become NaN. The row base saturates to `INT_MIN`, and
    `adelta = x*1024 >= 0` keeps the sum in range. The `level-a-nonfinite-translation`
    replay on 4.x showed no overflow.
  * **Euclidean.** `cos(NaN)` and `sin(NaN)` make **every** coefficient NaN, so `adelta`
    and `bdelta` are also `INT_MIN`. The addition is then `INT_MIN + INT_MIN`: a signed
    overflow, reported by `level-a-nonfinite-euclidean` on all 4.x versions.

  Whether native ECC can produce a NaN update is **not established**. For 2x2 and 3x3 the
  default `Mat::inv()` (`DECOMP_LU`) returns an all-zero matrix when the determinant is
  exactly 0 (`lapack.cpp` `invert`). A singular Hessian therefore gives `deltaP = 0`, not
  NaN. A NaN step needs NaN or Inf in the Hessian, the projections or `lambda`. No input in
  the envelope was shown to produce that. Section 5 records these replays as
  non-model-valid Level A evidence only.
* **Large or nonfinite updates and exception ordering.** `deltaP = H^-1 * e` has no step
  bound, and `lambda_d` has no positive lower bound (Tasks 011/012 recorded a successful
  4.x translation result of (44.3, 89.5) from a (6, 4) shift). Both exceptions
  (`lambda_d <= 0`, NaN `rho`) are raised after that iteration's four `warpAffine` calls.
  So a step that produced an unsafe matrix would have been consumed before the error
  reached the caller. That is the same ordering Task 020 documented for affine.
* **Seeded domain.** The seed has `|tx|, |ty| <= 2**20 = 1_048_576` and `|theta| <= 1` rad.
  Both translation bounds lie **below** the overflow window in section 3, which starts
  near `2.08e6`. The first iterate is therefore defined. Only later optimizer updates
  could cross the window.

### Native resampling, by version and type

| Version | `CV_32FC1` linear image/gradients; `CV_8UC1` nearest mask | Overflowing int32 addition executed? |
|---|---|---|
| 4.1.0, 4.6.0, 4.10.0 | `hal::warpAffine` builds `adelta`/`bdelta`, then `WarpAffineInvoker`, in 32x128 blocks for 64-row images. Linear: AVX2 `warpAffineBlockline` or `CV_SIMD128` `v_add` body (defined wrap), then a **scalar** per-pixel tail (`imgwarp.cpp`). Nearest: SSE4.1 `WarpAffineInvoker_Blockline_SSE41` body (`_mm_add_epi32`) plus its scalar tail (`imgwarp.sse4_1.cpp:204-205`), or the generic scalar loop when SSE4.1 is off | **Yes, in the scalar tails** (Level A below). The SIMD bodies wrap silently |
| 5.0.0 | `hal::warpAffine` dispatches these types to the float-coordinate kernels `cv::opt_AVX2::warpAffineLinearInvoker_32FC1` and `cv::opt_AVX2::warpAffineNearestInvoker_8UC1` (`warp_kernels.simd.hpp`) and returns. gdb confirmed this for a real `opencv_video_estimate_ecc_translation` call: per iteration, three linear calls (image and both gradients) and one nearest call (mask), with no `WarpAffineInvoker` hit. The legacy integer blocklines are present but marked `CV_DISABLE_UBSAN` and are not executed | **No**, not the legacy addition. Coordinates are `float sx = x*M[0] + y*M[1] + M[2]`, converted with `v_floor`/`v_round` or scalar `cvFloor`/`cvRound` (`warp_common.scalar.hpp:131-133`); scalar `cvFloor(float)` is `CV_DISABLE_UBSAN` (`core/fast_math.hpp:364`). Unaudited beyond `2^31` |

## 3. Operand-level arithmetic analysis

Legacy 4.x `warpAffine`, with `AB_SCALE = 1024`. Each destination pixel `(x, y)` uses the
`WARP_INVERSE_MAP` 2x3 matrix `M`:

```text
adelta[x] = saturate_cast<int>(M[0]*x*1024)                    // (1) per column (hal::warpAffine)
bdelta[x] = saturate_cast<int>(M[3]*x*1024)
X0 = saturate_cast<int>((M[1]*y + M[2])*1024) + round_delta    // (1) conversion, (2) signed int add, per row
Y0 = saturate_cast<int>((M[4]*y + M[5])*1024) + round_delta
X  = X0 + adelta[x];   Y = Y0 + bdelta[x];                     // (3) signed int add, per pixel
```

There are **three distinct conditions** (Task 021-R1). The original Task 021 analysis
treated only (3):

1. **Out-of-range conversion.** `saturate_cast<int>(double)` is `cvRound`. On x86-64 it
   returns `INT_MIN` for NaN or for any value that rounds outside int32. This is a defined
   intrinsic, so it is not UB, but the value is wrong.
2. **Row-base rounding overflow.** `cvRound(...) + round_delta` is a signed `int` addition
   that runs once per row, **before** any per-column delta. Source lines (4.x
   `WarpAffineInvoker::operator()`):

   | Version | `X0` | `Y0` | `round_delta` |
   |---|---|---|---|
   | 4.1.0 | `imgwarp.cpp:2213` | `:2214` | `:2185` |
   | 4.6.0 | `imgwarp.cpp:2199` | `:2200` | `:2174` |
   | 4.10.0 | `imgwarp.cpp:2299` | `:2300` | `:2271` |
   | 5.0.0 | `imgwarp.cpp:2018` | `:2019` | `:1999` (not executed for ECC's types; section 2) |

   `round_delta = interpolation == INTER_NEAREST ? AB_SCALE/2 : AB_SCALE/INTER_TAB_SIZE/2`,
   which is **512 for nearest** and 16 for linear (`AB_SCALE = 1024`, `INTER_TAB_SIZE = 32`).
3. **Per-column coordinate overflow.** `X0 + adelta[x]` / `Y0 + bdelta[x]`, in the
   blockline SIMD body (which wraps silently) or the scalar tails (visible to UBSan).

Execution order is: column tables, then for each row the X0 conversion and add, then the Y0
conversion and add, then the per-pixel adds. (2) therefore precedes (3). A row whose base
overflows must be classified as a row-base overflow, not as a later coordinate addition.

`round_delta` is 16 for linear and 512 for nearest. `saturate_cast<int>(double)` is
`cvRound`. On x86-64 that is `_mm_cvtsd_si32`, and this was **verified on the test host**,
not assumed (`conversion-check`, qualification report). For NaN, or any value that rounds
outside int32, it returns `INT_MIN` for both signs. It does not clamp. The diagnostic uses
an explicit model (`x86_cvround_model`): it range-checks every double first and passes only
representable values to the real conversion. It performs the final addition in `int64`, so
its own arithmetic is defined. It never executes the overflowing addition.

### Translation (`[1 0 tx; 0 1 ty]`; identity linear part)

`adelta[x] = x*1024`, which is at most `16383*1024 = 16_775_168` and always in range.
`bdelta[x] = 0`, so the **per-column** Y addition `Y0 + bdelta[x] = Y0` cannot overflow.
**That does not make Y safe** (Task 021-R1 correction): `Y0` is itself built by the signed
row-base addition (2), and so is `X0`. There are two translation windows.

**(a) Row-base rounding overflow, condition (2), on either axis.** It overflows when
`cvRound(v*1024)` is in range but `cvRound(v*1024) + round_delta > INT32_MAX`, where `v` is
`tx` (for X0) or `y + ty` (for Y0, any row `y`). The Float32 spacing at `2^21` is `0.125`, so
`v*1024` steps by 128:

| `round_delta` | Representable `v` that overflow | Count |
|---|---|---|
| 512 (nearest) | `2097151.5`, `.625`, `.75`, `.875` (`cvRound` = `2147483136` .. `2147483520`) | 4 |
| 16 (linear) | none: the largest in-range value, `2147483520`, plus 16 is `2147483536` | 0 |

So for translation the row-base overflow occurs **only in the nearest-neighbour mask warp**.
It is **independent of image width** and reproduces at an ordinary 64x97 image. For `Y0`,
the row index widens the `ty` window to `rows` disjoint 0.5-pixel bands. For 64 rows:
`ty` from `2097088.5` to `2097151.875`, fractional part `.5` to `.875`
(`ROWBASE_Y_WINDOW`). `tx` has only the four values above, because `X0` uses `M[1] = 0`.

**(b) Per-column coordinate overflow, condition (3), X only.** This is the original Task 021
window:

```text
X0 + adelta[x] > INT32_MAX   ⇔  tx*1024 > 2^31 - 1 - 16_775_168 - round_delta  ⇒  tx > ~2_080_770
tx*1024 rounds into int32                                                     ⇒  tx < 2_097_152
```

Its lower bound is `(2^31 - cols*1024)/1024`, so it narrows as the image narrows.

**Above `2^21`.** `cvRound` returns `INT_MIN` (condition (1), a defined intrinsic). `X0`/`Y0`
then become `INT_MIN + round_delta`, which is in range. A non-negative `adelta`, or the zero
`bdelta`, keeps the sum in range: **no addition overflow for translation, but a wrong
(sign-inverted) coordinate**. For negative `t`, the in-range minimum `≈ INT_MIN + 16` plus
a non-negative delta never overflows.

The full translation hazard is therefore `tx ∈ (~2.0808e6, 2.097152e6)` for X, plus the
nearest-mode row-base bands for `ty` just below `2^21`. The four nearest-mode `tx` values lie
inside the X window, but there the **first** undefined operation is the X0 row base, not the
per-column add (`level-a-rowbase-x`). The public
binding has **no translation seed**, so the identity start is `tx = ty = 0`, and both windows
are reachable only by accumulated optimizer updates.

### Euclidean (`[c -s tx; s c ty]`, `c = cos θ`, `s = sin θ`)

Both `adelta[x] = round(c*x*1024)` and `bdelta[x] = round(s*x*1024)` satisfy
`|·| <= x*1024 <= 16_775_168`. **Neither the rotation nor the Float32 rounding of its
coefficients can overflow either sum by itself.** The `euclidean-bigangle` control (θ ≈ 89°,
`t = (3, -2)`, 16383x64) shows 0 model overflows and 0 UBSan reports. The row bases are
`X0 = round((-s*y + tx)*1024)` and `Y0 = round((c*y + ty)*1024)`. A sum overflows when one of
them is a large in-range value of the same sign as its delta. With `|y| <= 16383` the rotated
row term is at most `16383*1024 ≈ 1.68e7` in magnitude (about 16,383 pixels), so the
condition again amounts to `|tx|` or `|ty|` lying within roughly `±3.3e4` of
`2^21 = 2_097_152`. Either sign can now overflow
(for example negative `ty` with negative `s`). **Bounded rotation coefficients bound the
column delta, not the row base.** The hazard therefore has the same magnitude as for
translation, and it applies to whichever of X or Y carries the large translation.

**Task 021-R1 additions for Euclidean.** Two routes exist that translation does not have:

* **Row-base overflow in linear mode as well.** The rotated row term `c*y*1024` (or
  `-s*y*1024`) is not a multiple of 128. A Euclidean row base can therefore land anywhere in
  `[INT32_MAX - 511, INT32_MAX]`, so even `round_delta = 16` can overflow. Example:
  `level-a-rowbase-euclidean`, θ = -0.5 rad, `t = (0, 2097109.875)`, 64x97, with
  `is_euclidean_valid` YES. At row 48, `cvRound((c*48 + ty)*1024) = 2147483647`. The first UB
  is then `Y0 = 2147483647 + 16` (linear) and `+ 512` (nearest), with no column delta
  involved.
* **Conversion followed by overflow, with no upper bound.** Above `2^21` the row base
  converts to the `INT_MIN` sentinel, condition (1). Translation's deltas are non-negative,
  so that is harmless there. A rotation with `s < 0` gives `bdelta[x] < 0`, and
  `INT_MIN + round_delta + bdelta[x]` overflows below `INT32_MIN`. Example:
  `level-a-conversion-euclidean`, θ = -0.5, `ty = 2097200`. Its first UB is
  `Y0 + bdelta[1] = -2147483632 + -491`. In the model (`EUCLIDEAN_WINDOW`, 64x97) every
  tested `ty` from `2097200` to `1e9` overflows. The Euclidean hazard is therefore **not**
  confined to a band below `2^21`. It extends to every `|ty|` beyond `2^21` while `s < 0`,
  and to every `|tx|` beyond `2^21` while `c < 0`: the column coefficient on that axis must
  be negative, so the delta pushes the sentinel below `INT32_MIN`. The seed bound `2^20`
  still lies below all of it.

**Seeded Euclidean.** The accepted seed `|tx|, |ty| <= 2^20 = 1_048_576` is about
`1.03e6` below the lower bound of the window, so a seed alone produces a defined
first iterate. The seed does not bound later updates.


### Confirmed overflow operands (Level A replay of model-valid matrices)

Each was found by the int64 model **and** reported by UBSan inside OpenCV. Geometry is
16383x64. With 64 rows, `WarpAffineInvoker` uses 32x128 blocks, so the last block is 127
columns wide. Its highest columns fall in the scalar tail, where UBSan can see the add. With
16384 columns, a multiple of 16, the overflow lands entirely in the wrapping SIMD bodies:
the first run used 16384 and gave 590,848 model overflows and **0** UBSan reports.

**Translation `[1 0 2090000; 0 1 0]`** (`is_translation_valid`: YES). Row 0:
`x_pre = 2_140_160_000`, `X0 = 2_140_160_016`. The first raster overflow is at column 7152:
`adelta = 7_323_648`, and the int64 sum `2_147_483_664` exceeds `INT32_MAX` by 17. `Y0 = 16`,
`bdelta = 0`, `Y = 16` (no overflow). 590,784 pixels overflow per mode.

| UBSan site | 4.1.0 | 4.6.0 | 4.10.0 | Reported operands (int64 sum) |
|---|---|---|---|---|
| Linear scalar tail, X | `imgwarp.cpp:2285:41` | `2269:41` | `2373:41` | `16769024 + 2140160016` (`2156929040`) |
| Nearest SSE4.1 tail, X | `imgwarp.sse4_1.cpp:204:21` | `204:21` | `204:21` | `16760832 + 2140160512` (`2156921344`) |
| Nearest generic tail, X (`setUseOptimized(false)`) | `imgwarp.cpp:2244:45` | `2229:45` | `2329:45` | `16769024 + 2140160512` (`2156929536`) |

**Euclidean `[cos 0.5, -sin 0.5, 0; sin 0.5, cos 0.5, 2090000]`** (`is_euclidean_valid`:
YES; θ = 0.5 rad). Row 0: `y_pre = 2_140_160_000`, `Y0 = 2_140_160_016`. The first raster
overflow is at column 14918: `bdelta = 7_323_720`, and the int64 sum is `2_147_483_736`.
`X0 = 16`, `adelta = 13_405_979`, `X = 13_405_995` (no overflow). 97,430 pixels overflow for
linear and 97,495 for nearest.

| UBSan site | 4.1.0 | 4.6.0 | 4.10.0 | Reported operands (int64 sum) |
|---|---|---|---|---|
| Linear scalar tail, Y | `imgwarp.cpp:2286:41` | `2270:41` | `2374:41` | `8039499 + 2140160016` (`2148199515`) |
| Nearest SSE4.1 tail, Y | `imgwarp.sse4_1.cpp:205:21` | `205:21` | `205:21` | `8035571 + 2140160512` (`2148196083`) |
| Nearest generic tail, Y (`setUseOptimized(false)`) | `imgwarp.cpp:2245:45` | `2230:45` | `2330:45` | `8039499 + 2140160512` (`2148200011`) |

UBSan prints the first scalar-tail pixel it reaches. The first raster pixel above comes from
the model, which visits pixels in raster order; both appear in the logs. With
`setUseOptimized(false)` the linear reports are unchanged. On 5.0.0, every finite Level A
replay gave **zero** reports in both dispatch modes, because the float kernels run instead
(the nonfinite replays are in section 5).

These two cases are **per-column addition** overflows, condition (3). Their row-base
doubles (`2.14e9`) are representable, `cvRound` converts them exactly, and the row-base
additions (`2_140_160_000 + 16/512`) are in range. In these two cases no float-to-int
conversion is out of range. The row-base and conversion-then-add cases below are different.

### Row-base rounding overflow (Task 021-R1; Level A, model-valid matrices)

Geometry is **64x97** (cols x rows: 97x64), an ordinary ECC size. The calls are
`cv::warpAffine` with `CV_32FC1` + `INTER_LINEAR | WARP_INVERSE_MAP`, then `CV_8UC1` +
`INTER_NEAREST | WARP_INVERSE_MAP`. These are the same call shapes native ECC uses.

**Translation Y `[1 0 0; 0 1 2097151.875]`** (`level-a-rowbase-y`; `is_translation_valid`
YES). `2097151.875` is exact in Float32 (`0x49FFFFFF`). Row 0:

```text
Y before conversion = (1*0 + 2097151.875)*1024 = 2147483520.0
converted (cvRound) = 2147483520            (in int32: no condition (1))
round_delta         = 512                   (INTER_NEAREST)
int64 sum           = 2147484032  >  INT32_MAX = 2147483647   → condition (2), first UB
```

`bdelta[x] = 0` is irrelevant: the overflow happens while building `Y0`, before any
`Y0 + bdelta[x]`. Linear mode (`round_delta = 16`) gives `2147483536`, which is in range. At
row 1 the conversion goes out of range and yields the `INT_MIN` sentinel, which is harmless
for translation. The model's `FIRST_UB` is `ROWBASE axis=Y row=0`.

**Translation X `[1 0 2097151.875; 0 1 0]`** (`level-a-rowbase-x`). The X0 row base gives the
same `2147483520 + 512` in nearest mode, which is the first UB (`FIRST_UB=ROWBASE axis=X
row=0`). In linear mode the row base is `2147483536`, in range, and the first UB is the
per-column `X0 + adelta[1] = 2147483536 + 1024`.

**Euclidean θ = -0.5, `t = (0, 2097109.875)`** (`level-a-rowbase-euclidean`;
`is_euclidean_valid` YES). At row 48 the row base converts to `2147483647`. The first UB is
`2147483647 + 16` in linear mode and `+ 512` in nearest mode.

| Case | 4.1.0 | 4.6.0 | 4.10.0 | Reported operands |
|---|---|---|---|---|
| rowbase-y, nearest, Y0 | `imgwarp.cpp:2214:84` | `2200:84` | `2300:84` | `512 + 2147483520` |
| rowbase-x, nearest, X0 | `imgwarp.cpp:2213:84` | `2199:84` | `2299:84` | `512 + 2147483520` |
| rowbase-x, linear, per-column X (scalar tail) | `imgwarp.cpp:2285:41` | `2269:41` | `2373:41` | `98304 + 2147483536` |
| rowbase-euclidean, linear, Y0 | `imgwarp.cpp:2214:84` | `2200:84` | `2300:84` | `16 + 2147483647` |

Column `:84` on the `X0`/`Y0` lines is the `+ round_delta` operator. The diagnostic itself
is distinct from the per-column `:41`/`:45`/`sse4_1:204-205` sites. With
`setUseOptimized(false)` the same row-base lines fire, because they sit in the generic
`WarpAffineInvoker` row loop and not in a SIMD kernel. The only difference is that the nearest
per-column report moves from `sse4_1:205:21` to the generic tail (`imgwarp.cpp:2245:45`,
`2230:45`, `2330:45`). UBSan reports each source location once
per process. In `rowbase-euclidean` the nearest call therefore reaches the already-reported
`Y0` line silently, and its remaining reports are later per-column sites. The int64 model
confirms that nearest's first UB is also the row base. The 98304 operand is the
SIMD-to-tail boundary column, `96*1024`; the model's raster-first pixel is column 1.

**Conversion followed by overflow, Euclidean θ = -0.5, `ty = 2097200`**
(`level-a-conversion-euclidean`; model-valid). Every Y row base is out of range, so
condition (1) gives `INT_MIN`. The row-base add gives `INT_MIN + 16`, which is in range.
Then `Y0 + bdelta[x]` with `bdelta[x] = round(-0.479*x*1024) < 0` overflows. The first UB is
`-2147483632 + -491` (linear, column 1). UBSan reports: 4.1.0 `imgwarp.cpp:2286:41`,
4.6.0 `2270:41` and 4.10.0 `2374:41`, all `-47129 + -2147483632`; nearest gives
`imgwarp.sse4_1.cpp:205:21`, `-47129 + -2147483136`.

**Negative control** (`level-a-rowbase-control`, `t = 2097000` on each axis, 64x97). The
worst Y row base is `(63 + 2097000)*1024 + 512 = 2147393024`. The worst X sum is
`2147328000 + 512 + 96*1024 = 2147426816`. Both are in range: the model reports 0, and there
are 0 UBSan reports on every version, in both dispatch modes. The exact row-base boundary
(`2097151.375` safe, `2097151.5` unsafe, nearest, row 0) is in `ROWBASE_WINDOW` and
`ROWBASE_Y_WINDOW`. It is not used as a native control, because at 64x97 the Y rows and X
columns push a boundary value over by other routes.

**5.0.0.** All four cases gave **0** reports. gdb on each case, in default and
`setUseOptimized(false)` modes, hit only `cv::opt_AVX2::` (default) or
`cv::cpu_baseline::` (scalar) `warpAffineLinearInvoker_32FC1` and
`warpAffineNearestInvoker_8UC1`, never `cv::WarpAffineInvoker::operator()`. In 5.0.0
`hal::warpAffine` (`imgwarp.cpp:2238-2361`; dispatch at `:2251` for nearest `CV_8UC1` and
`:2329` for linear `CV_32FC1`), `CV_CPU_DISPATCH` chains end in
`CV_CPU_CALL_BASELINE(fn, args) return (cpu_baseline::fn args)` (`cv_cpu_helper.h:654`),
with `if (CV_CPU_HAS_SUPPORT_AVX2) return (opt_AVX2::fn args)` before it. So every path
returns, and the `break` after each case is unreachable. The legacy `WarpAffineInvoker`
(with the `X0`/`Y0` lines at `imgwarp.cpp:2018-2019`, constructed at `:2357-2360`) is
therefore not reached for these types. The 5.0 row-base lines exist but are not executed
by ECC's resampling.


The window bounds were checked with the model (`conversion-check`, 16383x64,
identity-linear). Overflowing pixels per linear/nearest mode:

| `tx` | Overflowing pixels |
|---|---|
| 1048576 (`2^20`, the seed limit) | 0/0 |
| 2000000 | 0/0 |
| 2080700 | 0/0 |
| 2080800 | 1984/1984 |
| 2090000 | 590784/590784 |
| 2097151.5 | 1048448/1048512 |
| 2097152.5 (`cvRound` → `INT_MIN`) | 0/0 |
| 3000000 | 0/0 |

This matches the derived per-column X window `(~2080770, 2097152)` exactly. The counts are
pixels at which a signed `int` addition overflows. A pixel whose only issue is an
out-of-range conversion is not counted. Task 021-R1 kept this metric. The original model
already detected every row-base overflow numerically, because the column-0 pixel of an
overflowing row was out of range in its int64 sum. Its defect was the label: it reported
every such overflow as `X0 + adelta` / `Y0 + bdelta`, and the documentation drew the wrong
conclusion ("only X"). Each search and trace verdict is per matrix and is unchanged, so the
existing campaign results stand.

Row-base windows (`conversion-check`, model only):

| Probe | Result |
|---|---|
| `ROWBASE_WINDOW` (one row, Float32 `t` in `[2097140, 2097153)`) | nearest: 4 values, `2097151.5`..`2097151.875`; linear: 0 |
| `ROWBASE_Y_WINDOW` (64x97, nearest/linear) | `ty = 2097088.375`: 0/0 · `2097088.5`: 0/97 · `2097100.375`: 0/0 · `2097100.5`: 0/97 · `2097100.875`: 0/97 · `2097101.0`: 0/0 · `2097151.375`: 0/0 · `2097151.5`: 0/97 (linear/nearest) |
| `EUCLIDEAN_WINDOW` (θ = -0.5, 64x97, linear/nearest) | `ty = 2^20`, `2.0e6`, `2097000`: 0/0 · `2097109.875`: 1537/1522 · `2097151.875`: 6048/6082 · `2097200`, `3e6`, `1e9`: 6144/6080 |

## 4. Reproduction harness

The tooling is opt-in research only. It is never run by `scripts/test.sh`,
`scripts/run_sanitizers.sh` or any workflow, and no GPR, build script, checker or workflow
references it:

* `tests/cpp/ecc_translation_euclidean_overflow_probe.cpp` runs one case per process,
  because UBSan reports a given source location only once per process. It reuses
  `tests/cpp/ecc_fixture.hpp` for deterministic textures and uses real Core-owned handles
  for Level C.
* `scripts/research_ecc_translation_euclidean_overflow.sh` builds the probe, the real Video
  shim and the pinned Core shim with `-fsanitize=address,undefined,float-cast-overflow`
  (`-Werror`) against a caller-named instrumented OpenCV. It records `ldd` and runs each
  case. Each log is classified `EXPECTED_UPSTREAM_UB_OBSERVED`, `NO_UPSTREAM_REPORT` or
  `RESEARCH_PROBE_FAILED`. Only the last fails the runner; it means a non-zero exit, any
  ASan/LSan error, or any UBSan report outside `imgwarp*`.

```sh
OPENCV_PREFIX=$HOME/.cache/video-task017/install-4.10.0-san \
OPENCV_CORE_SOURCE=$PWD/alire/cache/pins/opencv_core_4da9d35e \
  sh scripts/research_ecc_translation_euclidean_overflow.sh           # all 20 default cases (15 + 5 from Task 021-R1)
sh scripts/research_ecc_translation_euclidean_overflow.sh search-euclidean-seeded:0:180
```

A direct binary invocation with a trailing `scalar` argument (for example
`.../ecc-translation-euclidean-overflow-probe level-a-translation-extreme scalar`) sets
`cv::setUseOptimized(false)`.

**Instrumented OpenCV.** The Task 017/020 builds `~/.cache/video-task017/install-<v>-san` for
4.1.0, 4.6.0, 4.10.0 and 5.0.0. They were configured by `build-one.sh` as CMake
`RelWithDebInfo` with `BUILD_LIST=core,imgproc,video`; IPP, OpenCL and ITT off;
`-include cstdint -fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer`;
`CPU_BASELINE=SSE3`; and `CPU_DISPATCH=SSE4_1;SSE4_2;AVX;FP16;AVX2;AVX512_SKX`. The UBSan
group `undefined` includes `signed-integer-overflow`. The compiler was Debian g++ 14.2.0, on
an AMD Ryzen 9 7945HX (x86_64, AVX2 and SSE4.1 reported). Every run used
`cv::setNumThreads(1)`, and `ldd` confirmed the instrumented libraries on every run.

**Level B method** (Task 020). Native ECC is deterministic and its loop body does not depend
on `maxCount`. A call capped at `k` iterations therefore leaves in the caller's warp exactly
the matrix that iteration `k+1` hands to `warpAffine`. Each candidate matrix is captured and
modelled **before** any call consumes it. A trace stops at the first unsafe matrix, so trace
logs carry no UBSan report by design. Each trace also checks the model invariants on every
iterate (`MODEL_INVARIANT_VIOLATED`). None was ever printed.

## 5. Evidence per API

| API | Level A | Level B | Level C |
|---|---|---|---|
| **Translation** | **Demonstrated** on 4.1.0, 4.6.0 and 4.10.0: model-valid `[1 0 2090000; 0 1 0]` overflows the per-column X add (16383x64), and (Task 021-R1) `[1 0 0; 0 1 2097151.875]` / `[1 0 2097151.875; 0 1 0]` overflow the **Y0/X0 row base** at 64x97 (operands in section 3). 5.0.0: not executed | `NOT_REPRODUCED`. Trace (97x64, (+2,+1)): 8 distinct iterates on every version, then a repeated warp ends it; max `\|t\|` 2.14. Searches: max `\|t\|` **3273.9** (trial 73) | `NOT_REPRODUCED`. Status 0, displacement (2.020, 1.027); 5.0.0 gives (2.012, 1.013). 0 UBSan reports on every version |
| **Euclidean identity** | **Demonstrated** on 4.x: model-valid rigid θ=0.5, `ty=2090000` overflows the per-column Y add; (Task 021-R1) θ=-0.5, `ty=2097109.875` overflows the Y0 row base in **linear** mode too, and θ=-0.5, `ty=2097200` overflows after an out-of-range conversion (64x97). The θ≈89° control has no overflow | `NOT_REPRODUCED`. Trace (97x64, 6°, (2,1)): 17 (4.1.0), 87 (4.6.0/4.10.0), 10 (5.0.0) distinct iterates; max `\|t\|` 2.65. Searches: max `\|t\|` **5522.0** (4.10.0 trial 208) | `NOT_REPRODUCED`. Status 0, angle 0.10475, displacement (1.996, 1.016). 0 reports |
| **Euclidean seeded** | Same Level A as identity. θ=0.5 lies inside the seeded angle domain, but `ty=2090000` lies **outside** the `2^20` seed domain, so this matrix is reachable only as a later iterate | `NOT_REPRODUCED`. Trace from the in-domain extreme seed `(θ=0.9, tx=2^20, ty=-2^20)`: the first iterate is safe, then the very next call raised `StsNoConv` "NaN encountered" (the image falls entirely outside), so no further matrix reached `warpAffine`. Searches: max `\|t\|` 1048575.6, the seed itself (trial 51) | `NOT_REPRODUCED`. The same seed through `opencv_video_estimate_ecc_euclidean_seeded` returned status 3 (`OPENCV_VIDEO_ERROR_OPENCV`, "NaN encountered") on every version. 0 reports |

**Nonfinite replays (Level A, NOT model-valid: `is_*_valid` = NO).** These record what the
native resampler does if a NaN optimizer update ever occurred. None was shown to occur.

| Case | 4.1.0 / 4.6.0 / 4.10.0 | 5.0.0 |
|---|---|---|
| `level-a-nonfinite-translation` `[1 0 NaN; 0 1 NaN]`, 16383x64 | 0 model overflows, 0 reports | **1 report**: `warp_kernels.simd.hpp:3764:17: runtime error: pointer index expression with base 0x7fa84b7f9840 overflowed to 0xffffffa84b7f9840`, in `cv::opt_AVX2::warpAffineLinearInvoker_32FC1`, scalar tail |
| `level-a-nonfinite-euclidean` (all six NaN), 97x64 | **4 reports**: `imgwarp.cpp` linear X/Y (`2285/2286`, `2269/2270`, `2373/2374`) and `sse4_1.cpp:204/205`, operands `-2147483648 + -2147483632` (linear) and `-2147483648 + -2147483136` (nearest); 6,208 of 6,208 pixels per mode | 0 reports |

The 5.0.0 finding is new. It is the first UBSan-visible report in the 5.0 float-coordinate
kernel. In the scalar tail, `cvFloor(NaN)` (`CV_DISABLE_UBSAN`, x86 result `INT_MIN`) yields
`ix`/`iy` of `INT_MIN`, and the address arithmetic `src + srcstep*iy + ix` forms an
out-of-range pointer. That is undefined pointer arithmetic, even though the bounds mask
prevents the load. The report is not suppressed, so the `CV_DISABLE_UBSAN` annotation on
`cvFloor` does not hide it. It requires a nonfinite coordinate, which the binding never
supplies and native ECC was never shown to generate.

**Negative controls.** These report zero model overflows and zero UBSan reports on every
version, so the harness does not flag every transform:

* `negative`: translation `[1 0 2; 0 1 1]` and Euclidean (θ=0.05, t=(2,1)) at 256x64;
* `level-a-euclidean-bigangle` (θ = 1.5533 rad ≈ 89°) at 16383x64;
* `level-a-translation-far` `[1 0 1e6; 0 1 1e6]` (16383x64) and `level-a-euclidean-far`
  (θ=0.5, t=(-1e6, 1e6), 97x64). Both are finite model-valid warps inside the `2^20` seed
  bound and entirely outside the image, and both are clean on 5.0.0 too;
* the window checks at `tx = 2^20`, `2.0e6` and `2.0807e6`;
* (Task 021-R1) `level-a-rowbase-control`, `t = 2097000` on X and on Y at 64x97, the
  row-base negative control, plus the safe neighbours in `ROWBASE_WINDOW` /
  `ROWBASE_Y_WINDOW` (`2097151.375`, `2097088.375`, `2097101.0`).

**Why the seeded extreme seed cannot walk into the window.** With `|tx|` or `|ty|` near
`2^20`, every warped sample falls outside the source, so `imageMask` is empty. Then
`countNonZero = 0`, `tmpNorm = imgNorm = 0` and `rho = 0/0 = NaN`, and native ECC raises
before any update. The far-translation seed therefore terminates rather than accumulating.
This is an observation about these images, not a bound: the window could still be reached
from a seed that keeps partial overlap while a large, poorly conditioned step is taken.

## 6. Bounded campaign coverage

Every trial is deterministic and indexed. `campaign_trial(n, flavor)` seeds an LCG with
`21*2654435761 + n*40503 + 7*flavor` and draws the following inputs:

* **Geometry.** Rows 32..128 and columns 32..160, asymmetric and non-square, inside the
  published envelope.
* **Image kind.** One of twelve kinds, `n % 12`:
  0. moderate translation;
  1. translation near the convergence limit, up to 9 px;
  2. rotation up to ±15° plus translation up to ±20 px;
  3. ±3° rotation about the origin, which needs compensating translation for centre content;
  4. low-gradient (low-variance) texture;
  5. constant versus constant (nearly constant images);
  6. periodic, anisotropic stripes with an ambiguous shift;
  7. unrelated LCG noise;
  8. weakly conditioned gradients (low-variance template);
  9. high-contrast structure concentrated in narrow bands;
  10. a region cut from a larger parent Mat;
  11. a second rotation-plus-translation draw.
* **Euclidean seed** (flavor 2 only). The class is `(n / 12) % 6`, so every class meets every
  image kind. Every seed is inside the binding's domain:
  * zero seed;
  * near-correct seed;
  * wrong but accepted (negated rotation and translation);
  * angle within `1e-3` of ±1 rad;
  * displacement within 64 px of ±`2^20`;
  * rotation opposing the true rotation, with an unrelated translation.

Every trial ran 100 iterations at eps `1e-8`, the strictest options the envelope allows,
with no mask. For each trial, every iterate's matrix was evaluated with the int64 model
before it was consumed.

**Budget.** Each campaign was set at 300 s wall-clock and 1.5 GB peak RSS per process.

| OpenCV | Translation trials | Euclidean identity trials | Euclidean seeded trials | Unsafe | Max `\|t\|` reached (trial) | Wall time (s) |
|---|---|---|---|---|---|---|
| 4.1.0 | 0-119 | 0-119 | 0-179 | **0** | 3273.9 (T73), 486.7 (E112), 1048575.6 (S51) | 46 / 180 / 156 |
| 4.6.0 | 0-119 | 0-119 | 0-179 | **0** | 3273.9 (T73), 486.7 (E112), 1048575.6 (S51) | 45 / 178 / 173 |
| 4.10.0 | 0-299 | 0-239 | 0-179 | **0** | 3273.9 (T73), **5522.0** (E208), 1048575.6 (S51) | 44+35 / 171+185 / 166 |
| 5.0.0 | 0-119 | 0-119 | 0-179 | **0** | 3273.9 (T73), 143.0 (E76), 1048575.6 (S51) | 89 / 152 / 136 |

Every campaign stayed within budget. Peak RSS was 858 MB for the 120-trial translation search
and 209 MB for a seeded slice; both were measured under ASan with `VmHWM`. Totals: 660
translation, 600 Euclidean identity and 720 Euclidean seeded trials (1,980), with 0 unsafe
intermediates. Each campaign log contains 0 UBSan reports, 0 ASan
reports and 0 LSan reports. The highest translation any search reached that was **not** a
seed was 5522, which is **0.27%** of the lower edge of the window (`~2.08e6`). The seeded
maximum, 1048575.6, is a starting seed value: trial 51 is in the near-`2^20` displacement
class. The optimizer did not produce it.
Result: **`NOT_REPRODUCED`, not `SAFE`.**

Reproduce one trial with
`sh scripts/research_ecc_translation_euclidean_overflow.sh search-euclidean-identity:208:1`.
The same index always regenerates the same image pair, geometry and seed.

The reproduction was checked: running `search-euclidean-identity:208:1` again on 4.10.0
regenerated `5521.99512` exactly.

## 7. Version and dispatch coverage

| Path | Executed? | Result |
|---|---|---|
| 4.1.0/4.6.0/4.10.0, default x86_64 dispatch (AVX2 linear blockline, SSE4.1 nearest blockline, scalar tails) | Yes | Level A overflow in the scalar tails; SIMD bodies wrap silently |
| 4.1.0/4.6.0/4.10.0, `setUseOptimized(false)` (generic scalar loops, `CV_SIMD128` bodies) | Yes (Level A) | Level A overflow: linear `2285/2269/2373`, nearest `2244/2229/2329` and `2245/2230/2330` |
| 5.0.0 default (`cv::opt_AVX2::warpAffineLinearInvoker_32FC1` / `warpAffineNearestInvoker_8UC1`) | Yes, gdb-confirmed through the real translation export | 0 reports for every finite matrix; legacy addition not executed. **1 pointer-index-overflow report** (`warp_kernels.simd.hpp:3764`) for the non-model-valid NaN translation replay |
| 5.0.0 `setUseOptimized(false)` (`cv::cpu_baseline::warpAffine*Invoker_*`) | Yes, gdb-confirmed | 0 reports; legacy addition not executed |
| AVX-512, NEON and other AArch64 kernels, RVV, LASX (4.10.0+ `imgwarp.lasx.cpp`), IPP, OpenCL, custom `cv_hal_warpAffine*` HAL, Windows/macOS toolchains | **No** | Unqualified |

On 5.0.0, the float kernels compute `float sx = x*M[0] + y*M[1] + M[2]` and convert it to an
integer. For every **finite** matrix tested the coordinate is far within int32: the window
replays at `|t| ≈ 2.09e6`, and the far-outside replays at `|t| = 1e6`. None produced a
report. A finite coordinate beyond `2^31` would need `|t| > 2.1e9`, about 1000x the 4.x
window, and nothing in this audit approached it.

The NaN translation replay shows that the 5.0 scalar tail is **not** UB-free for nonfinite
coordinates. The undefined operation is `src + srcstep*iy + ix` with `ix = iy = INT_MIN`,
from the UBSan-suppressed `cvFloor(NaN)`; the subsequent bounds mask blocks only the load.
The same address formation with a finite coordinate beyond int32 is therefore expected to
be unsafe as well. It was not executed. A clean run on one dispatch configuration cannot
establish that every platform path is safe.

## 8. Comparison with the affine finding

| Property | Translation | Euclidean | Affine |
|---|---|---|---|
| Optimized parameters | 2 | 3 | 6 |
| Linear matrix | Identity (never written) | Rigid rotation, `\|c\|,\|s\| <= 1` | General affine, unbounded |
| Translation updates | Accumulated, unbounded | Accumulated, unbounded | Accumulated, unbounded |
| Per-column `adelta`/`bdelta` | `x*1024`, `0` (bounded) | bounded by `x*1024` | **unbounded**; saturates for large coefficients |
| Native geometric warp | `warpAffine` | `warpAffine` | `warpAffine` |
| Overflow trigger on 4.x | Per-column: `tx` in a band up to ~16,400 px wide just below `2^21` (width = image columns). Row base (Task 021-R1): `tx` or `y+ty` in `[2097151.5, 2097151.875]`, nearest mode only, any image width | Per-column and row base (both modes) near `2^21`; **plus** conversion-then-add for any `\|t\|` beyond `2^21` with a matching rotation sign | large linear coefficients **or** translation |
| Seeded public operation | No | Yes (`\|t\| <= 2^20`, `\|θ\| <= 1`) | Yes (`\|A\| <= 4`, `\|t\| <= 2^20`) |
| Level A, model-valid | Demonstrated | Demonstrated | Demonstrated |
| Level B, ECC-generated | `NOT_REPRODUCED` | `NOT_REPRODUCED` | **Demonstrated** (iteration 3) |
| Level C, real binding | `NOT_REPRODUCED` | `NOT_REPRODUCED` (both overloads) | **Demonstrated** (seeded, 4.1.0) |
| Safety disposition | **`SAFETY_GATE_NOT_ESTABLISHED`** | **`SAFETY_GATE_NOT_ESTABLISHED`** (identity and seeded) | `KNOWN_NATIVE_UB_UNMITIGATED` (Task 020) |

**Do bounded linear coefficients prevent unsafe fixed-point coordinates?** No.

* The large operand is the row base `X0`/`Y0`, which carries `tx`/`ty`. Bounded rotation
  coefficients bound only the column delta added to it. As Task 021-R1 adds, **building the
  row base is itself a signed addition that can overflow** (`cvRound(...) + round_delta`).
  Neither bounded nor zero column deltas protect that step.
* Translation and Euclidean therefore overflow under the same leading condition: an
  accumulated translation within about 1% of `2^21`. For Euclidean, a rotation of the
  matching sign also lets any translation beyond `2^21` overflow, through the `INT_MIN`
  conversion sentinel (section 3).
* The constraint does shrink the attack surface. Affine reached overflow through linear
  coefficients of order `1e4` to `1e5` at iteration 3 (Task 020 Matrices A and B and the
  n103 intermediate). Translation and Euclidean cannot use that route: they need a
  translation of about 2.08 million pixels. The optimizer never came within three orders of
  magnitude of that on any tested in-envelope input.
* The difference is one of reachability, not of the native defect.

**Version specificity.** The reproduced addition belongs to OpenCV 4.x's fixed-point
`WarpAffineInvoker` (4.1.0, 4.6.0 and 4.10.0 on x86_64). The verified 5.0.0 path for ECC's
types uses float kernels and does not execute that addition, in either dispatch mode tested.
This is not generalized to untested 5.0 coordinate regimes, to other 4.x releases, or to other
CPU architectures.

## 9. Why each disposition is `SAFETY_GATE_NOT_ESTABLISHED`

`SAFETY_GATE_ESTABLISHED` needs a source-level argument that excludes the unsafe addition
across a precisely defined domain. None exists for these operations:

* **Translation identity.** Excluding the windows would need an upper bound on the
  accumulated `tx` **and** `ty`. Task 021-R1 shows that `ty` alone reaches the Y0 row-base
  overflow, so bounding only `tx` would not suffice. Each step is `deltaP = H^-1 * e`, where
  `H` is the 2x2 gradient Hessian of the warped image. Its conditioning depends on the data
  and is unbounded: stripes give a singular `y` direction, and nearly constant images give
  a near-zero Hessian. `lambda_d` has no positive floor, and there is no step cap. A single ill-conditioned step can
  therefore move `tx` arbitrarily far. Tasks 011/012 already recorded a "successful"
  4.x result of `(44.3, 89.5)` from a `(6, 4)` shift. The searches reached 3274, but
  nothing bounds the next step.
* **Euclidean identity.** The same applies, with a 3x3 Hessian. The bounded rotation does
  not bound `tx` or `ty` (section 3).
* **Euclidean seeded.** The seed bounds only the first iterate, at `2^20`, which is below the
  window. Later steps are as unbounded as in the identity case. A seed near `2^20` also
  starts halfway to the window, although the observed effect was an immediate NaN rather
  than growth.

`KNOWN_NATIVE_UB_UNMITIGATED` needs a model-valid ECC execution or a binding call that
demonstrably reaches the undefined operation. The strongest evidence here is Level A: a
model-valid matrix replayed directly. That shows the native `warpAffine` is vulnerable for
matrices these models can represent. It does **not** show that the ECC optimizer produces
them. Under Task 021's rules, that is not enough to declare the APIs unsafe.

Binding-side preflight could not close the gap either. As in Task 020 §6 Option B, the
binding controls only images, options and the seed. A translation-magnitude check after the
call would see only the final warp, after the overflow had already occurred inside OpenCV.

## 10. Production behavior, mitigation and owner decision

No production behavior changed:

* translation, Euclidean and affine ECC are all still enabled;
* seed validation, the 25 C exports and the ABI layouts are unchanged;
* the OpenCV dependency and the Core pin are unchanged;
* no overload, mask or motion model was added or removed;
* the optimizer and numerical results are unchanged;
* homography is untouched;
* the Task 020 affine decision (keep available during development) stands.

No new *reached* native defect was confirmed for these models, so the stop criterion did not
trigger and no gate was applied. Translation and Euclidean are covered by the same Task 020
Option A research patch, `docs/research/task020_imgwarp_guard.py`, a guard on the
`WarpAffineInvoker` row block. That patch is model-agnostic. **By source inspection only**
(Task 021-R1), it is inserted before the `X0` line and tests the double row bases `|xa|`,
`|ya| <= 2^30`. For a row base near `2^31` it therefore takes the guarded path before the
`+ round_delta` row-base addition executes, so it would cover the row-base overflow as well
as the per-column sums. It was not rebuilt or re-tested for Task 021 or 021-R1, and it remains
an unadopted research artifact that needs a patched OpenCV.

Owner decisions carried forward:

* whether to report the 4.x `warpAffine` addition upstream (one report can now cite affine
  Level C and translation/Euclidean Level A);
* whether any version gating should cover all three non-homography models.

## 11. Remaining risks and recommended follow-on

* **The native defect is reachable in principle for all three operations on 4.x.** For
  translation the windows are narrow: `tx ≈ 2.08e6 .. 2.097e6` per-column, plus the
  nearest-mode row-base bands for `tx`/`ty` just below `2^21`. No argument excludes them.
  For Euclidean, the conversion-then-add route has no upper bound in `|t|` (section 3).
* **Task 021-R1 correction.** The original analysis missed the row-base addition
  `cvRound(...) + round_delta`. It is now modelled as a separate condition, reproduced
  natively at 64x97, and shown to be the first undefined operation where it applies. The
  dispositions are unchanged, because no new Level B/C evidence was found.
* `NOT_REPRODUCED` covers 1,980 bounded trials. It shows rarity, not impossibility.
* The x86 `cvRound` sign inversion above `2^21` gives a wrong but in-range coordinate with no
  UB report. That can still steer the optimizer.
* **Some nonfinite warps execute UB on each tested version family.** In 4.x, an all-NaN
  Euclidean warp gives `INT_MIN + INT_MIN`; the NaN translation replay was clean. In 5.0, a
  NaN translation forms an out-of-range pointer in the float kernel (16383x64); the all-NaN
  Euclidean replay (97x64) was clean. Neither unsafe case was shown reachable. The only
  barrier is the absence of a NaN optimizer step, because native ECC checks `rho` for NaN
  only *after* the four `warpAffine` calls.
* The OpenCV 5.0 float kernels are unqualified for finite coordinates beyond int32, and
  non-x86, IPP, OpenCL and HAL paths are unqualified.
* **Recommended follow-on:**
  1. a targeted adversarial search for ill-conditioned inputs that produce a single huge
     translation step (for example near-singular stripe Hessians with a partially
     overlapping seed) or a nonfinite step (Inf/NaN in `lambda` or the projections);
  2. a 5.0 float-kernel audit at `|coordinate| > 2^31`, extending the NaN finding;
  3. an AArch64/NEON run;
  4. a decision on Option A upstream engagement covering all three models and the 5.0
     nonfinite-coordinate pointer formation.
