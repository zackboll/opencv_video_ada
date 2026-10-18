# Task 021 qualification — translation and Euclidean ECC native arithmetic audit

Starting `origin/main`: `6915ddc7a46cad41c37620b72a63f2b1b81dd043` (PR #20 merge, merged
2026-10-18T03:34:00Z). Branch `safety/021-translation-euclidean-ecc-audit` in a separate
worktree (`/home/zboll/git/opencv/video-task021`). No Task 021 branch, worktree or PR existed
beforehand, and the working tree was clean. Inventory at start and end: **25 production
exports, 198 AUnit registrations**, Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all
three Alire roots, crate `0.1.0-dev`. All unchanged.

**Technical dispositions** (independent):

| API | Disposition |
|---|---|
| `Estimate_ECC_Translation` / `opencv_video_estimate_ecc_translation` | `SAFETY_GATE_NOT_ESTABLISHED` |
| `Estimate_ECC_Euclidean` / `opencv_video_estimate_ecc_euclidean` | `SAFETY_GATE_NOT_ESTABLISHED` |
| `Estimate_ECC_Euclidean` + `Initial_Transform` / `opencv_video_estimate_ecc_euclidean_seeded` | `SAFETY_GATE_NOT_ESTABLISHED` |

Hosted CI is reported in the last section. Passing regression does not resolve native safety,
and no release-readiness or safety-qualification claim is made.

## PR #20 post-merge CI (read once at start, not polled)

| Run | Workflow | Head | Result |
|---|---|---|---|
| 39094076843 | Cross-platform Video | `6915ddc` | **success** |
| 39094076803 | Windows post-merge Video | `6915ddc` | **failure** in `Upgrade MSYS2 system coherently` |

The Windows failure was investigated through its exact log, not rerun. pacman reported
`failed retrieving file 'mingw-w64-x86_64-graphite2-1.3.15-1-any.pkg.tar.zst' from
mirror.msys2.org : Operation too slow` and `error: failed to commit transaction (download
library error)`, followed by `Errors occurred, no packages were upgraded`. That is a transient
MSYS2 mirror download failure. It happened before any build or test step, so it is **not a
relevant baseline failure**. The previous Windows run on `b09a65a` succeeded. No code or CI
change was made; Windows stays push-to-main only.

## Part 1 — Research evidence (expected upstream UB; NOT sanitizer-clean tests)

These runs are expected to produce UBSan reports from inside OpenCV for the Level A cases.
They are kept separate from the regression campaign and never run in CI. Tooling:
`scripts/research_ecc_translation_euclidean_overflow.sh` and
`tests/cpp/ecc_translation_euclidean_overflow_probe.cpp`.

Setup:

* Instrumented OpenCV: `~/.cache/video-task017/install-<v>-san`, built with
  `-fsanitize=address,undefined,float-cast-overflow`. `ldd` confirmed these libraries on
  every run.
* Runtime options: `UBSAN_OPTIONS=halt_on_error=0`, `ASAN_OPTIONS=detect_leaks=1`.
* Host: g++ 14.2.0, x86_64 with AVX2 and SSE4.1, single thread.


Final probe, all 15 default cases per version (`== case` verdicts):

| Case | 4.1.0 | 4.6.0 | 4.10.0 | 5.0.0 |
|---|---|---|---|---|
| `negative` | 0 reports, `NEGATIVE_CONTROL_OK` | same | same | same |
| `conversion-check` | 13/13 conversions MATCH | same | same | same |
| `level-a-euclidean-bigangle` (≈89°) | 0 model overflows, 0 reports | same | same | same |
| `level-a-translation-far`, `level-a-euclidean-far` (`\|t\|` = 1e6) | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| `level-a-translation-extreme` | **2 reports**: `imgwarp.cpp:2285:41`, `sse4_1.cpp:204:21` | **2**: `2269:41`, `204:21` | **2**: `2373:41`, `204:21` | 0 (not executed) |
| `level-a-euclidean-extreme` | **2**: `2286:41`, `205:21` | **2**: `2270:41`, `205:21` | **2**: `2374:41`, `205:21` | 0 (not executed) |
| `level-a-nonfinite-translation` (NaN t; not model-valid) | 0 | 0 | 0 | **1**: `warp_kernels.simd.hpp:3764:17` pointer index overflow |
| `level-a-nonfinite-euclidean` (all NaN; not model-valid) | **4**: `2285/2286:41`, `204/205:21`, `INT_MIN + INT_MIN+16` | **4**: `2269/2270`, `204/205` | **4**: `2373/2374`, `204/205` | 0 |
| `trace-translation` | no unsafe iterate (8), max `\|t\|` 2.14 | same | same | same |
| `trace-euclidean-identity` | no unsafe iterate (17), 2.65 | (87), 2.65 | (87), 2.65 | (10), 2.65 |
| `trace-euclidean-seeded` | iterate 1 safe, then `StsNoConv` | same | same | same |
| `level-c-translation` | status 0, 0 reports | status 0 | status 0 | status 0 |
| `level-c-euclidean-identity` | status 0, 0 reports | status 0 | status 0 | status 0 |
| `level-c-euclidean-seeded` | status 3 "NaN encountered", 0 reports | status 3 | status 3 | status 3 |

Every case ran with exit 0, 0 non-`imgwarp` UBSan reports and 0 ASan/LSan reports, so no case
was `RESEARCH_PROBE_FAILED`. Wall time was 17-19 s per version.

The runner first classified the 5.0 `warp_kernels` report as foreign, because its
expected-evidence pattern originally matched only `imgwarp*`. The pattern was widened to
OpenCV's imgproc warp sources (`imgwarp`, `warp_kernels`, `warp_common`). Reports from the
shims or the probe still fail it. All 15 cases were then rerun on every version with the
committed probe.

**Reported operands** (identical on every 4.x version):

| Case | Linear scalar tail | Nearest SSE4.1 tail |
|---|---|---|
| Translation, X | `16769024 + 2140160016` | `16760832 + 2140160512` |
| Euclidean, Y | `8039499 + 2140160016` | `8035571 + 2140160512` |

The int64 model's first raster overflows are:

* translation: row 0, column 7152, `2140160016 + 7323648 = 2147483664`;
* Euclidean: row 0, column 14918, `2140160016 + 7323720 = 2147483736`.

**Scalar dispatch** (`setUseOptimized(false)`, Level A). Generic nearest tail reports were
`imgwarp.cpp:2244/2245:45` (4.1.0), `2229/2230:45` (4.6.0) and `2329/2330:45` (4.10.0), with
the same operands as above with 512 rounding. On 5.0.0 there were 0 reports. gdb hit
`cv::cpu_baseline::warpAffineLinearInvoker_32FC1` and `warpAffineNearestInvoker_8UC1`, and
never the legacy invoker.

**5.0.0 default dispatch through the real binding.** gdb on `level-c-translation` hit
`cv::opt_AVX2::warpAffineLinearInvoker_32FC1` three times per iteration and
`cv::opt_AVX2::warpAffineNearestInvoker_8UC1` once, called from
`cv::hal::warpAffine` → `cv::warpAffine`. `WarpAffineInvoker` was never hit.

**`conversion-check`** (on the host CPU). `_mm_cvtsd_si32` returned `INT_MIN` for
`2147483647.5`, `2^31`, `-2^31 - 1`, `±3e9`, `1e300`, `inf` and `NaN`. In-range values were
returned exactly. All 13 results matched the diagnostic's explicit model. Overflowing pixels
per `tx` (16383x64, identity linear, linear/nearest):

| `tx` | Overflowing pixels |
|---|---|
| `2^20` | 0/0 |
| `2.0e6` | 0/0 |
| `2.0807e6` | 0/0 |
| `2.0808e6` | 1984/1984 |
| `2.09e6` | 590784/590784 |
| `2097151.5` | 1048448/1048512 |
| `2097152.5` | 0/0 |
| `3e6` | 0/0 |

### Bounded search (opt-in)

| OpenCV | Translation trials / unsafe / max `\|t\|` | Euclidean identity | Euclidean seeded | Wall (s) |
|---|---|---|---|---|
| 4.1.0 | 0-119 / 0 / 3273.9 (T73) | 0-119 / 0 / 486.7 (E112) | 0-179 / 0 / seed 1048575.6 (S51) | 46 / 180 / 156 |
| 4.6.0 | 0-119 / 0 / 3273.9 | 0-119 / 0 / 486.7 | 0-179 / 0 / seed | 45 / 178 / 173 |
| 4.10.0 | 0-299 / 0 / 3273.9 | 0-239 / 0 / **5522.0** (E208) | 0-179 / 0 / seed | 79 / 356 / 166 |
| 5.0.0 | 0-119 / 0 / 3273.9 | 0-119 / 0 / 143.0 (E76) | 0-179 / 0 / seed | 89 / 152 / 136 |

The budget was 300 s and 1.5 GB per campaign process. Measured peak RSS under ASan was
858 MB (translation, 120 trials) and 209 MB (seeded slice). There were 1,980 trials in total
(660 translation, 600 Euclidean identity, 720 seeded) and 0 unsafe intermediates, with 0
sanitizer reports in any search log. Rerunning `search-euclidean-identity:208:1` regenerated
`5521.99512` exactly.

Result: **`NOT_REPRODUCED`, not `SAFE`.**

### Task 021-R1 — Row-base rounding overflow (correction; research evidence)

**Scope.** This corrects Part 1. The original arithmetic model computed
`X0`/`Y0 = cvRound(...) + round_delta` directly in int64. It therefore never classified that
signed `int` addition as a separate hazard, and the audit wrongly concluded that "only the X
addition can overflow" for translation. The source was re-read at all four tags:

| Version | `X0` | `Y0` | `round_delta` (`INTER_NEAREST` → `AB_SCALE/2 = 512`, linear → 16) |
|---|---|---|---|
| 4.1.0 | `imgwarp.cpp:2213` | `:2214` | `:2185` |
| 4.6.0 | `imgwarp.cpp:2199` | `:2200` | `:2174` |
| 4.10.0 | `imgwarp.cpp:2299` | `:2300` | `:2271` |
| 5.0.0 | `imgwarp.cpp:2018` | `:2019` | `:1999` (legacy invoker; not reached for `CV_32FC1`/`CV_8UC1`) |

**Probe changes (research only).**

* `analyze_fixed_point` sorts every flagged pixel into one of three conditions:
  1. out-of-range conversion;
  2. row-base `+ round_delta` overflow;
  3. per-column `+ delta` overflow.
* `find_first_ub` replays the native execution order and names the **first undefined
  addition** (`FIRST_UB`).
* All sums are int64.
* The returned count, the number of pixels where an int addition overflows, keeps the
  Task 021 metric. A bounded search rerun, shown below, confirms this.
* Five default cases were added: `level-a-rowbase-y`, `level-a-rowbase-x`,
  `level-a-rowbase-euclidean`, `level-a-conversion-euclidean` and the negative control
  `level-a-rowbase-control`.
* `conversion-check` gained `ROWBASE_WINDOW`, `ROWBASE_Y_WINDOW` and `EUCLIDEAN_WINDOW`.

**Default runner, 20 cases per version, final probe:**

| Version | Exit | Cases | `EXPECTED_UPSTREAM_UB_OBSERVED` | `RESEARCH_PROBE_FAILED` | Non-imgproc UBSan / ASan |
|---|---|---|---|---|---|
| 4.1.0 | 0 | 20 | 7 | 0 | 0 / 0 |
| 4.6.0 | 0 | 20 | 7 | 0 | 0 / 0 |
| 4.10.0 | 0 | 20 | 7 | 0 | 0 / 0 |
| 5.0.0 | 0 | 20 | 1 (the NaN translation replay, unchanged) | 0 | 0 / 0 |

The 15 original cases gave the same verdicts and report sites as in the table above.

**Candidate.**

* Matrix: the exact Float32 `[1 0 0; 0 1 2097151.875]` (`0x49FFFFFF`).
* Geometry and call: 64x97 `CV_8UC1`, `INTER_NEAREST | WARP_INVERSE_MAP`.
* Row 0 arithmetic:
  * `Y` before conversion = `2147483520.0`;
  * converted = `2147483520`;
  * `round_delta` = 512;
  * int64 sum = `2147484032`, which exceeds `INT32_MAX`.
* X twin: `[1 0 2097151.875; 0 1 0]`.

| Case (64x97, model-valid) | Model `FIRST_UB` (linear / nearest) | 4.1.0 | 4.6.0 | 4.10.0 | 5.0.0 |
|---|---|---|---|---|---|
| `rowbase-y` | none / **ROWBASE Y row 0** `2147483520 + 512` | `imgwarp.cpp:2214:84` `512 + 2147483520` | `2200:84` same | `2300:84` same | 0 |
| `rowbase-x` | PERCOLUMN X `2147483536 + 1024` / **ROWBASE X row 0** `2147483520 + 512` | `2285:41` `98304 + 2147483536`; `2213:84` `512 + 2147483520` | `2269:41`; `2199:84` | `2373:41`; `2299:84` | 0 |
| `rowbase-euclidean` (θ=-0.5, ty=2097109.875) | **ROWBASE Y row 48** `2147483647 + 16` / `+ 512` | `2214:84` `16 + 2147483647`; `2286:41`, `sse4_1:205:21` `-47129 + …` | `2200:84`; `2270:41`; `205:21` | `2300:84`; `2374:41`; `205:21` | 0 |
| `conversion-euclidean` (θ=-0.5, ty=2097200) | PERCOLUMN Y col 1 `-2147483632 + -491` / col 2 `-2147483136 + -982` (after INT_MIN conversion) | `2286:41` `-47129 + -2147483632`; `sse4_1:205:21` | `2270:41`; `205:21` | `2374:41`; `205:21` | 0 |
| `rowbase-control` (t=2097000, X and Y) | none / none | 0 | 0 | 0 | 0 |

**Scalar dispatch.** With `setUseOptimized(false)`, every version reported the same
row-base sites at `:84`. The nearest per-column site moved to the generic tail
(`2245:45` / `2230:45` / `2330:45`). The control was clean, and 5.0.0 had 0 reports.

**Standalone reproducer.** A program independent of the probe was built against each
instrumented install. It calls `cv::warpAffine` on a 64x97 `CV_8UC1` image with the candidate
matrix. It reported exactly `imgwarp.cpp:2214:84` / `2200:84` / `2300:84` for Y and
`2213:84` / `2199:84` / `2299:84` for X, all `512 + 2147483520`. It reported nothing on
5.0.0.

**5.0.0 execution path.** gdb ran all three row-base cases in both dispatch modes, with
`ASAN_OPTIONS=detect_leaks=0` because LSan cannot run under ptrace. It hit only:

* `cv::opt_AVX2::warpAffineLinearInvoker_32FC1` and `warpAffineNearestInvoker_8UC1`
  (default);
* the `cv::cpu_baseline::` versions of the same two functions (scalar).

Both are called from `cv::hal::warpAffine` (`imgwarp.cpp:2251` nearest, `:2329` linear).
`cv::WarpAffineInvoker::operator()` was never hit. `CV_CPU_DISPATCH` always ends in
`return (cpu_baseline::fn args)` (`cv_cpu_helper.h:654`), so the legacy invoker at
`:2357-2360` is unreachable for these types.

**Model windows** (`conversion-check`, identical on all versions):

* `ROWBASE_WINDOW`: Float32 `t` such that `cvRound(t*1024)` is in range but adding
  `round_delta` overflows.
  * Nearest: 4 values, `2097151.5` .. `2097151.875`.
  * Linear: 0 values.
* `ROWBASE_Y_WINDOW` (64x97, linear/nearest pixels):

  | `ty` | Pixels |
  |---|---|
  | `2097088.375` | 0/0 |
  | `2097088.5` | 0/97 |
  | `2097100.375` | 0/0 |
  | `2097100.5` | 0/97 |
  | `2097100.875` | 0/97 |
  | `2097101.0` | 0/0 |
  | `2097151.375` | 0/0 |
  | `2097151.5` | 0/97 |

* `EUCLIDEAN_WINDOW` (θ=-0.5, 64x97, linear/nearest pixels):

  | `ty` | Pixels |
  |---|---|
  | `2^20`, `2e6`, `2097000` | 0/0 |
  | `2097109.875` | 1537/1522 |
  | `2097151.875` | 6048/6082 |
  | `2097200`, `3e6`, `1e9` | 6144/6080 |

* `TRANSLATION_WINDOW` (16383x64) is unchanged from Part 1. The `2097152.5` and `3e6` rows
  are 0/0 because an out-of-range conversion alone is not counted as an overflowing
  addition.

**Corrected conclusions.**

* **Translation, nearest mode.** For `ty` or `tx` in the row-base band just below `2^21`,
  the first undefined operation is the **row-base addition**, on either axis. `bdelta = 0`
  does not protect `Y0`.
* **Translation, linear mode.** There is no row-base overflow. Float32 spacing makes the
  largest in-range base `2147483520`, and adding 16 fits. Only the per-column X window
  applies.
* **Euclidean.** A row-base overflow occurs in linear mode as well. There is also a
  conversion-then-add route that has no upper bound in `|t|` beyond `2^21` when the
  rotation sign matches.
* **Seed bound.** `2^20` lies below every window.
* **Evidence level.** Every case is Level A. There is no Level B or Level C reproduction,
  so **the dispositions remain `SAFETY_GATE_NOT_ESTABLISHED`.**

**Search re-check with the corrected model.** This is a subset of the Part 1 campaign,
rerun only to confirm that the reclassified model leaves the per-matrix verdicts unchanged.
It is not a new campaign.

| Version | `search-translation:0:120` | `search-euclidean-identity:0:24` | `search-euclidean-seeded:48:12` |
|---|---|---|---|
| 4.1.0 | 0 unsafe, max 3273.89 (trial 73) | 0, max 12.85 (trial 1) | 0, max 1048575.62 (trial 51, the seed) |
| 4.6.0 | 0, 3273.89 (73) | 0, 12.85 (1) | 0, 1048575.62 (51) |
| 4.10.0 | 0, 3273.89 (73) | 0, 12.85 (1) | 0, 1048575.62 (51) |
| 5.0.0 | 0, 3273.89 (73) | 0, 13.19 (4) | 0, 1048575.62 (51) |

The maxima and trial indices equal the original runs. The remaining 1,980-trial campaign
was **not** rerun. Its per-matrix verdicts are unaffected by construction (probe comment on
`fixed_point_overflows`).

### Task 021-R2 — x86 conversion classification (research-helper correction)

**Defect.** R1's `analyze_fixed_point` decided whether a conversion was out of range with
`oob_i32(std::llround(v))`. Its `CONVERSION` report also printed
`(int64_t)std::llround(v)` as an "operand". Both were wrong:

* **Wrong rounding rule.** OpenCV's x86 `cvRound(double)` is `_mm_cvtsd_si32`
  (`fast_math.hpp` at all four tags). It rounds under MXCSR: round to nearest, **ties to
  even**. `llround` rounds ties **away from zero**. The two disagree at
  `-2147483648.5`: x86 gives the representable `INT_MIN`, while `llround` gives
  `-2147483649`, which is out of range.
* **Invalid conversion.** For NaN, ±Inf and huge finite values, `llround` is itself an
  invalid conversion. It raises `FE_INVALID` and its result is unspecified; on this host it
  returned `INT64_MIN`. The R1 log lines `operands -9223372036854775808 + 0 int64 sum=…` in
  the NaN replays were artefacts of that call, not native values.

**Correction** (`tests/cpp/ecc_translation_euclidean_overflow_probe.cpp`):

* A single predicate, `classify_x86_cvround`, returns `NONFINITE`, `OUT_OF_RANGE` or
  `IN_RANGE`.
  * `IN_RANGE` means `-2147483648.5 <= v < 2147483647.5`. The lower bound is inclusive
    because that tie rounds to the even, representable `INT_MIN`. The upper bound is
    exclusive because that tie rounds to the even `2^31`, which is not representable.
  * `isfinite` is tested first, so an ordered comparison never sees a NaN; on x86 such a
    comparison would itself raise `FE_INVALID`.
* `x86_cvround_model`, `analyze_fixed_point` and the `ROWBASE_WINDOW` scan now all use it.
  The window scan previously had its own copy of the interval and its own `llround`.
* The probe no longer calls `llround`, `lround` or `lrint`, and casts no double to an
  integer. Only `IN_RANGE` values reach `cv::saturate_cast<int>`.
* A `CONVERSION` finding now reports the operand it feeds (`X0-base`, `Y0-base`, `adelta` or
  `bdelta`), the pre-conversion double (printed with `%.17g`), its class, and
  `x86-result=-2147483648 (integer indefinite)`. No integer operands or sum are fabricated.
* `FIRST_UB` is unchanged. It still reports the first overflowing signed-int addition, with
  two valid int32 operands and an int64 sum.

**Why counts cannot change, by construction.** R1's `x86_cvround_model` already used the
correct interval; only the separate `conv_oor` lambda used `llround`. Every int64 sum, the
`FIRST_UB` replay and the returned metric were therefore computed from correct values.

The metric is `ub_pixels = rowbase + percolumn + conversion_then_add_overflow`. Each pixel
contributes 1 exactly when a row-base or per-column addition overflows, whatever
`conv_oor` says. A wrong conversion flag could only move a pixel between the `conversion`
and `rowbase`/`percolumn` buckets, or change the `FIRST_UNSAFE` label. It could never change
a count that the controls, windows, traces or searches use.

**New `conversion-check` boundary tests.** All of these were checked on all four builds.

* **Rounding mode.** `fegetround() == FE_TONEAREST` and MXCSR RC = 0. Any other mode is
  reported and fails the mode.
* **Boundary probes.** Each of the 23 probes has an independently stated expected class and
  value, and is compared against the classifier, the model and the real `cvtsd2si`:

  | Input | Class | `cvtsd2si` |
  |---|---|---|
  | `-2147483648.5` | IN_RANGE (tie to even) | `-2147483648` |
  | `-2147483648.49`, `-2147483648.0` | IN_RANGE | `-2147483648` |
  | `-2147483648.51`, `-2147483649.0` | OUT_OF_RANGE | `INT_MIN` (indefinite) |
  | `2147483647.0`, `2147483647.49` | IN_RANGE | `2147483647` |
  | `2147483647.5`, `2147483648.0` | OUT_OF_RANGE | `INT_MIN` |
  | `±Inf`, `NaN` | NONFINITE | `INT_MIN` |
  | `±1e300`, `±3e9` | OUT_OF_RANGE | `INT_MIN` |
  | `0.5`, `1.5`, `2.5`, `-2.5`, `2147483646.5`, `-2147483647.5`, `0` | IN_RANGE | `0`, `2`, `2`, `-2`, `2147483646`, `-2147483648`, `0` (ties to even) |

  Result: **23/23 MATCH, 0 MISMATCH on 4.1.0, 4.6.0, 4.10.0 and 5.0.0.**
* **Reporting on invalid values.** `analyze_fixed_point`, `find_first_ub` and
  `print_first_unsafe` were run with row bases of NaN, ±Inf and ±1e300. In each case
  `FE_INVALID` stayed clear, with `first_kind=CONVERSION`, operand `X0-base` and
  `FIRST_UB=NONE`: **5/5 OK** on every version.
* **Sensitivity control.** `cvtsd2si(NaN)` raised `FE_INVALID=1` (OK on every version), so
  the flag check can detect a real invalid conversion.

**Updated research campaign.** The probe was rebuilt from scratch on each version with the
established flags (`-Wall -Wextra -Wpedantic -Werror`,
`-fsanitize=address,undefined,float-cast-overflow`), and all 20 default cases were rerun:

| Version | Exit | Cases | `EXPECTED_UPSTREAM_UB_OBSERVED` | `RESEARCH_PROBE_FAILED` | Unexpected UBSan / ASan |
|---|---|---|---|---|---|
| 4.1.0 | 0 | 20 | 7 | 0 | 0 / 0 |
| 4.6.0 | 0 | 20 | 7 | 0 | 0 / 0 |
| 4.10.0 | 0 | 20 | 7 | 0 | 0 / 0 |
| 5.0.0 | 0 | 20 | 1 | 0 | 0 / 0 |

**Before/after snapshot.** All 20 cases × 4 versions were snapshotted before and after the
change. The snapshot covered:

* verdicts;
* every native report site and its operands;
* every `int64 model`, `counts:`, `FIRST_UB` and window line;
* every control and binding-status line.

The diff contains only two kinds of change:

1. The reformatted `CONVERSION` boundary and `DIAG_*` lines in `conversion-check`.
2. The `FIRST_UNSAFE=CONVERSION` lines in four cases, which changed from fabricated integers
   to the new format:
   * `level-a-rowbase-y` (linear; `Y0-base` `2147484544`);
   * `level-a-conversion-euclidean` (`Y0-base` `2147532800`);
   * `level-a-nonfinite-translation` and `level-a-nonfinite-euclidean` (`X0-base` `nan`,
     `NONFINITE`).

Everything else is **identical**: all counts and breakdowns, windows, `FIRST_UB` lines,
verdicts, and every native UBSan site with its operands. That covers:

* the translation X0/Y0 row-base sites (`2213/2214:84`, `2199/2200:84`, `2299/2300:84`);
* the Euclidean row base in both modes;
* conversion-then-add;
* the original per-column sites;
* the 5.0.0 NaN-translation pointer report (`warp_kernels.simd.hpp:3764`);
* all negative controls, which stay clean.

A `setUseOptimized(false)` rerun of the five row-base, conversion and control cases, on all
four versions, gave the same sites as R1.

**Bounded search re-check (R2).** This is a subset of the campaign, rerun with the corrected
probe. It was compared with the pre-R2 run of the same probe procedure, made at the
starting head:

| Version | `search-euclidean-identity:208:1` | `search-translation:0:120` | `search-euclidean-identity:0:24` | `search-euclidean-seeded:48:12` |
|---|---|---|---|---|
| 4.1.0 | 0 unsafe, max 5466.17 | 0, 3273.89 (73) | 0, 12.85 (1) | 0, 1048575.62 (51) |
| 4.6.0 | 0, 5521.995 | 0, 3273.89 (73) | 0, 12.85 (1) | 0, 1048575.62 (51) |
| 4.10.0 | 0, 5521.995 | 0, 3273.89 (73) | 0, 12.85 (1) | 0, 1048575.62 (51) |
| 5.0.0 | 0, 60.77 | 0, 3273.89 (73) | 0, 13.19 (4) | 0, 1048575.62 (51) |

* Every `SEARCH_SUMMARY` line is identical to the pre-R2 run.
* There were no `SEARCH_UNSAFE` lines and no sanitizer reports.
* Trial 208 reproduces the Part 1 maximum of **5522.0** on 4.10.0 (and on 4.6.0). On 4.1.0
  and 5.0.0 the same trial follows a different trajectory and reaches a smaller maximum.
* The full 1,980-trial campaign was **not** rerun, because by the argument above its
  per-matrix verdicts cannot change.

**Conclusion.** No overflow count or native finding changed. The only classification
changes are the four `FIRST_UNSAFE=CONVERSION` report lines, which are now well defined.
The three dispositions remain **`SAFETY_GATE_NOT_ESTABLISHED`**. The README and the public
Ada comments are unchanged, because none of their factual statements is affected.

## Part 2 — Normal regression campaign (binding qualification, unchanged scope)

Each row is an independent `git clone --no-local` of the committed head, built from source.
The 4.1.0, 4.6.0 and 5.0.0 rows use the uninstrumented Release installs
`~/.cache/video-task017/install-<v>` through `PKG_CONFIG_PATH` and `LD_LIBRARY_PATH`. The
4.10.0 row uses the system package. Each row runs these steps:

* repository checker, the configuration tests, `sh -n` on every script, and
  `git diff --check` against the baseline;
* Development, Validation and Release `alr build`;
* `scripts/test.sh`: every direct-native ECC oracle, AUnit, the uninstrumented native
  boundary, fault injection and corruption;
* `scripts/run_sanitizers.sh`: ASan, UBSan and leak detection over the shim, the boundary
  driver, the fault build and every direct oracle;
* the export inventory (`nm`);
* the `lk_synthetic` example, and the `ecc_smoke` and `homography_smoke` programs;
* a check that no build file, workflow or regression script references the research probe.

All four rows ran on clean clones of `d7b2ab9a1010463403aba876245ad7eaa86877d7`, the
findings and documentation commit. Two commits followed it:

* `4ccdf2b` added the nonfinite and far replays to the research probe and runner;
* `629e040` added this file and corrected documentation.

Neither commit touches `src/`, `cpp/`, a GPR, a workflow or a regression script, and none of
those references the probe. The 4.10.0 row was nevertheless rerun on a clean clone of the
final content head `629e040a4228c6d500c5ec971eb276b77a6d2cf9`, with the identical result shown in the table below.

| OpenCV | Library loaded by `run_tests` (`ldd`) | Checker / config / `sh -n` / diff | Builds D/V/R | `test.sh` | AUnit run/ok/fail/err | Translation / Euclidean / seeded oracle | Affine / seeded oracle | Homography / seeded / masked (native ok/exc) | Sanitizer step | Exports / test symbols | `lk_synthetic`, smokes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 4.1.0 | `install-4.1.0/lib/libopencv_video.so.4.1` | 0/0/0/0 | 0/0/0 | 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 28/9, 20/15, 39/14 | rc 0, 0 findings | 25 / 0 | 4/4 tracked, PASS, PASS |
| 4.6.0 | `install-4.6.0/lib/libopencv_video.so.406` | 0/0/0/0 | 0/0/0 | 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 30/7, 20/15, 39/14 | rc 0, 0 findings | 25 / 0 | 4/4, PASS, PASS |
| 4.10.0 | system `/usr/lib/x86_64-linux-gnu/libopencv_video.so.410` | 0/0/0/0 | 0/0/0 | 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 30/7, 20/15, 39/14 | rc 0, 0 findings | 25 / 0 | 4/4, PASS, PASS |
| 5.0.0 | `install-5.0.0/lib/libopencv_video.so.500` | 0/0/0/0 | 0/0/0 | 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 28/9, 19/16, 39/14 | rc 0, 0 findings | 25 / 0 | 4/4, PASS, PASS |

Every row has these properties:

* Each oracle printed its own installed `CV_VERSION`.
* Each oracle ran three times: the oracle step, the native driver and the sanitizer driver.
* The row logs contained zero `runtime error`, `ERROR: AddressSanitizer`,
  `ERROR: LeakSanitizer` or `SUMMARY:` lines, and zero compiler `warning:` lines.
* The production archive `libopencv_video_shim.a` contains only `opencv_video_shim.o`.
  It defines 25 `opencv_video_*` text symbols and no `opencv_video_test_*` symbol.
* No GPR, workflow, Python checker or regression script references
  `ecc_translation_euclidean_overflow`.
* The oracle and homography counts equal the Task 020 campaign on every version.
* The 12 configuration tests passed (`Ran 12 tests ... OK`).

**Instrumentation scope.** In this campaign `run_sanitizers.sh` instruments the **binding**
and links the **uninstrumented**, prebuilt OpenCV. A clean result therefore says nothing
about native OpenCV safety, as Part 1 shows. These ordinary tests do not independently
demonstrate upstream native safety for any motion model. The research probe is not part of
`test.sh`, `run_sanitizers.sh` or any workflow, so it cannot affect these results.

**No AUnit registrations were added.** Task 021 is research and documentation only:

* the Ada changes are comments;
* the C++ shim is byte-identical to the baseline;
* the probe's expected-UB cases cannot run in a sanitizer-clean suite.

Regression equivalence is the qualification criterion. The registration count (198), the
export count (25) and every oracle fixture count equal the Task 020 campaign on each version.

## Not claimed

* That translation, identity Euclidean or seeded Euclidean ECC is free of native UB on any
  OpenCV version. Each is `SAFETY_GATE_NOT_ESTABLISHED`.
* That any of them is unsafe. No Level B or Level C reproducer exists; the Level A matrices
  are model-valid but were never shown to be produced by ECC.
* That the 5.0 float kernels are UB-free for coordinates beyond `2^31`. A NaN coordinate
  is shown **not** to be UB-free (`warp_kernels.simd.hpp:3764`).
* That native ECC can produce a nonfinite warp. The nonfinite replays are Level A only and
  are not model-valid.
* Any AVX-512, NEON, AArch64, RVV, LASX, IPP, OpenCL, custom HAL, Windows or macOS result for
  the research probe. The probe is Linux/x86_64 only.
* Any change to the Task 020 affine disposition (`KNOWN_NATIVE_UB_UNMITIGATED`). It is
  unchanged.
* Release readiness. No version, tag or release change was made.

## Hosted CI

The Task 021 PR targets `main` and is non-draft, with auto-merge disabled. Its workflow
(repository-checks, linux, macos, linux-sanitizers) covers only the normal regression scope.
It does not, and must not, run the research probe. Final-head results are recorded on the PR
rather than here, so this file need not chase its own commit. A head that has not finished is
reported as `HOSTED_CI_PENDING`, separately from the technical dispositions. Windows remains
push-to-main only and is not triggered by this PR.
