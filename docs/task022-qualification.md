# Task 022 qualification — OpenCV 5.0.0 floating-point warp kernel safety audit

Starting `origin/main`: `bc4d619c4a33a5fcd55029674ccb7840f1cfbeef` (PR #21 merge, merged
2026-10-18T17:40:20Z). Branch `safety/022-opencv5-float-warp-audit` in a separate worktree
(`/home/zboll/git/opencv/video-task022`). No Task 022 branch, worktree or PR existed
beforehand, and the working tree was clean. Inventory at start and end: **25 production
exports, 198 AUnit registrations**, Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all
three Alire roots, crate `0.1.0-dev`. All unchanged.

**No normal production code was changed.** `cpp/` and every Ada body are byte-identical to the
baseline; the Ada spec and README changes are comments and dated notices only.

> **R1 correction (2026-10-19).** The table below is the **original** Task 022 result, kept as
> history. It is superseded on 5.0 by section **Part R1** at the end of this file. On the tested
> OpenCV 5.0.0 x86_64 build, all five ECC operations, called through the real binding with
> ordinary inputs, form scalar-tail source pointers outside the source allocation. Each is now
> `KNOWN_NATIVE_UB_UNMITIGATED` on 5.0. This rests on native register capture and allocation
> analysis; there is **no** sanitizer report. "`|coord| >= 2^31`" is also wrong at exactly
> `-2^31`, which is representable.

**Technical dispositions (original Task 022, superseded on 5.0 by Part R1)** (independent; full
reasoning in [the task note](tasks/022-opencv5-float-warp-safety-audit.md)):

| Subject | Disposition |
|---|---|
| `warpAffineLinearInvoker_32FC1`, finite `|coord| >= 2^31` (x86_64 AVX2 + cpu_baseline) | `DIRECT_NATIVE_UB_CONFIRMED` — finite, model-valid matrix |
| `warpAffineNearestInvoker_8UC1`, same domain | `DIRECT_NATIVE_UB_CONFIRMED` — finite, model-valid matrix (separate evidence) |
| Translation ECC (identity) on 5.0 | `SAFETY_GATE_NOT_ESTABLISHED` (Level B/C `NOT_REPRODUCED`) |
| Euclidean ECC (identity, seeded) on 5.0 | `SAFETY_GATE_NOT_ESTABLISHED` (Level B/C `NOT_REPRODUCED`) |
| Affine ECC (identity, seeded) on 5.0 | `SAFETY_GATE_NOT_ESTABLISHED` (Level B/C `NOT_REPRODUCED`) |
| Affine ECC on 4.x (Task 020) | `KNOWN_NATIVE_UB_UNMITIGATED` — **unchanged** |
| Translation/Euclidean ECC on 4.x (Task 021) | `SAFETY_GATE_NOT_ESTABLISHED` — **unchanged** |

Passing regression does not resolve native safety; no release-readiness or safety-qualification
claim is made.

## PR #21 post-merge CI (read once at start, not polled)

| Run | Workflow | Head | Result |
|---|---|---|---|
| 39150664396 | Cross-platform Video | `bc4d619` | **success** (repository-checks, linux, macos, linux-sanitizers) |
| 39150664411 | Windows post-merge Video | `bc4d619` | **in_progress** when read (job `windows` started 2026-10-18T17:40:25Z) — incomplete, recorded neither as pass nor failure |

## Part 1 — Research evidence (expected upstream UB; NOT sanitizer-clean tests)

These runs are **expected** to produce UBSan reports from inside OpenCV for the Level A cases.
They are kept separate from the regression campaign and never run in CI. Tooling:
`scripts/research_ecc5_float_warp.sh` and `tests/cpp/ecc5_float_warp_probe.cpp`.

Setup:

* Instrumented OpenCV 5.0.0: `~/.cache/video-task017/install-5.0.0-san`, its **own** libraries
  built with `-fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer`
  (RelWithDebInfo, `CPU_BASELINE=SSE3`, dispatch up to `AVX512_SKX`, IPP/OpenCL off). The runner
  verified via `nm -D` that `libopencv_imgproc.so.5.0.0` imports
  `__ubsan_handle_pointer_overflow`; the uninstrumented `install-5.0.0` build does not (0
  imports), so the check discriminates. `ldd` confirmed all OpenCV libraries resolve under the
  instrumented prefix.
* Probe built with `-std=c++17 -Wall -Wextra -Wpedantic -Werror -g -O1
  -fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer`: **0 warnings**.
* Runtime: `UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=0:report_error_type=1`,
  `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1`, one case per process.
* Host: AMD Ryzen 9 7945HX (x86_64, AVX2/FMA/AVX-512), g++ 14.2.0 (Debian 14.2.0-19), gdb 16.3.
* Identities: probe sha256 `b4ce0004f9e1a65402e03b9ae9c14659a3c14092a9eaf13e95efae2e02cf615e`
  (final source, rebuilt by the final runner pass), instrumented imgproc
  `install-5.0.0-san/lib/libopencv_imgproc.so.5.0.0` sha256
  `73c4d0d95b3dd0ad5f52b3d7f094677b3bbbe347ac10e1f9e72e87f8a15e7292`.

**Final default runner pass: `RUNNER_EXIT=0`, 28 cases, 10 `EXPECTED_UPSTREAM_UB_OBSERVED`, 18
`NO_UPSTREAM_REPORT`, 0 `RESEARCH_PROBE_FAILED`, 0 `NEGATIVE_CONTROL_REPORTED`.** Across all 28
cases: 0 UBSan reports outside OpenCV's warp sources, 0 ASan errors, 0 LSan leaks, every exit 0.

| Case | Level | Finite? | Verdict | UBSan site (first unsafe operation) |
|---|---|---|---|---|
| `conversion-check` | control | — | NO_UPSTREAM_REPORT | 17/17 MATCH, `CONVERSION_MODEL_OK` |
| `negative` | control | yes | NO_UPSTREAM_REPORT | `NEGATIVE_CONTROL_OK` |
| `level-a-far-control` (`|t|=1e6`) | control | yes | NO_UPSTREAM_REPORT | IN_RANGE |
| `level-a-matrix-b` (Task 020) | control | yes | NO_UPSTREAM_REPORT | IN_RANGE, max 2.33e7 |
| `level-a-boundary-below` (`2^31-128`) | control | yes | NO_UPSTREAM_REPORT | IN_RANGE |
| `level-a-boundary-above` (`2^31`) | A | yes | NO_UPSTREAM_REPORT (silent: single axis, non-wrapping offset) | `COORD_REACH` OUT_OF_RANGE_FINITE |
| `level-a-finite-translation-linear` | A | **yes, model-valid** | **EXPECTED_UPSTREAM_UB_OBSERVED** | `warp_kernels.simd.hpp:3764:17` pointer index expression overflowed, `opt_AVX2::warpAffineLinearInvoker_32FC1` |
| `level-a-finite-translation-nearest` | A | **yes, model-valid** | **EXPECTED_UPSTREAM_UB_OBSERVED** | `warp_kernels.simd.hpp:439:17`, `opt_AVX2::warpAffineNearestInvoker_8UC1` |
| `level-a-finite-euclidean` | A | **yes, model-valid** | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:3764:17` |
| `level-a-finite-affine` | A | **yes, model-valid** | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:3764:17` |
| `level-a-overflow-to-inf` | A | finite coeffs → Inf coord | NO_UPSTREAM_REPORT (silent) | `COORD_REACH` NONFINITE |
| `level-a-scalar-finite-linear` | A | **yes** | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:3764:17`, `cpu_baseline::…32FC1` (gdb) |
| `level-a-scalar-finite-nearest` | A | **yes** | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:439:17`, `cpu_baseline::…8UC1` (gdb) |
| `level-a-roi-linear` (noncontiguous) | A | **yes** | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:3764:17` |
| `level-a-nonfinite-translation` (Task 021 replay) | A | no (NaN) | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:3764:17` — Task 021 reproduced |
| `level-a-inf-translation` | A | no (±Inf) | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:3764:17` |
| `level-a-nonfinite-euclidean` | A | no (all NaN) | **EXPECTED_UPSTREAM_UB_OBSERVED** | `:3764:17` |
| `level-a-nonfinite-affine-coeff` | A | no (one NaN) | NO_UPSTREAM_REPORT (silent: single axis) | `COORD_REACH` NONFINITE |
| `trace-translation` | B | — | NO_UPSTREAM_REPORT | max coord 98.1, none out of range |
| `trace-euclidean` | B | — | NO_UPSTREAM_REPORT | max 97.5 |
| `trace-euclidean-seeded` | B | — | NO_UPSTREAM_REPORT | max 1048635.6 (seed only; first capped call raised) |
| `trace-affine` | B | — | NO_UPSTREAM_REPORT | max 100.7 |
| `trace-affine-n435` (Task 020 seed) | B | — | NO_UPSTREAM_REPORT | max 33644148, iterate 3, then ECC raised |
| `level-c-translation` | C | — | NO_UPSTREAM_REPORT | status 0, `(2.012, 1.013)` |
| `level-c-euclidean` | C | — | NO_UPSTREAM_REPORT | status 0, angle 0.10474 |
| `level-c-euclidean-seeded` | C | — | NO_UPSTREAM_REPORT | status 3, "NaN encountered" |
| `level-c-affine` | C | — | NO_UPSTREAM_REPORT | status 0 |
| `level-c-affine-n435` | C | — | NO_UPSTREAM_REPORT | status 3, "NaN encountered" |

**Opt-in width sweep** (one width per process, linear, translation `3e9`): widths 16383, 16385,
16401, 32769, 65521, 65535 (`W mod 16 != 0`) report **1** each; 16384, 16400, 32768, 65520, 65536
(`W mod 16 == 0`, no AVX2 scalar tail) report **0** each. Widths 32, 33, 63, 64, 96, 97, 128,
129 report 0 for both kernels (offset too small to wrap; still out of range per `COORD_REACH`).
Nearest reports only at 65535 within that list, matching its 1-byte stride.

**Opt-in bounded searches** (`0:150` each): translation, Euclidean, Euclidean-seeded, affine and
affine-seeded found **0** out-of-range intermediates and **0** UBSan/ASan reports; the largest
coordinates reached were 227.9, 1453.5, 1048729.8, 141934.6 and 1048815.3 (52–140 s, peak RSS
1.05–2.02 GB each).

**gdb dispatch confirmation**: default → `cv::opt_AVX2::warpAffineLinearInvoker_32FC1` and
`cv::opt_AVX2::warpAffineNearestInvoker_8UC1`; `setUseOptimized(false)` →
`cv::cpu_baseline::` twins; `cv::WarpAffineInvoker` never hit. Through the real
`opencv_video_estimate_ecc_translation` export (97x64), a breakpoint at `:3764` showed
`vlanes_32 = 8`, `uf = 16`, `x = 96` (the single scalar-tail column).

No mitigation was applied to any OpenCV used here; section 11 of the task note lists the
unvalidated proposals.

## Part 2 — Normal regression campaign (binding qualification, unchanged scope)

Each row is an independent `git clone --no-local` of head
`920d7394c20b0a4692f60ed8f6922c015653ab8a`, built from source, run **serially**. The 4.1.0,
4.6.0 and 5.0.0 rows use the **uninstrumented** Release installs
`~/.cache/video-task017/install-<v>` through `PKG_CONFIG_PATH`/`LD_LIBRARY_PATH`; 4.10.0 uses
the system package. Each row runs: repository checker, the 12 configuration tests, `sh -n` on
every script, `git diff --check bc4d619 HEAD`, Development/Validation/Release `alr build`,
`scripts/test.sh` (every direct-native oracle, AUnit, native boundary, fault injection and
corruption), `scripts/run_sanitizers.sh` (ASan/UBSan/leak over the shim, boundary driver, fault
build and every oracle), the export inventory, `lk_synthetic`, the `ecc_smoke` and
`homography_smoke` programs, `ldd`, and a check that no GPR, workflow or regression script
references the research probe.

| OpenCV | Library loaded by `run_tests` (`ldd`) | All 15 steps | AUnit run/ok/fail/err | Translation / Euclidean / seeded oracle | Affine / seeded oracle | Homography / seeded / masked (native ok/exc) | Exports / test symbols | `lk_synthetic`, smokes | Findings |
|---|---|---|---|---|---|---|---|---|---|
| 4.1.0 | `install-4.1.0/lib/libopencv_video.so.4.1` | rc 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 28/9, 20/15, 39/14 | 25 / 0 | 4/4 tracked, PASS, PASS | 0 |
| 4.6.0 | `install-4.6.0/lib/libopencv_video.so.406` | rc 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 30/7, 20/15, 39/14 | 25 / 0 | 4/4 tracked, PASS, PASS | 0 |
| 4.10.0 | system `/usr/lib/x86_64-linux-gnu/libopencv_video.so.410` | rc 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 30/7, 20/15, 39/14 | 25 / 0 | 4/4 tracked, PASS, PASS | 0 |
| 5.0.0 | `install-5.0.0/lib/libopencv_video.so.500` | rc 0 | 198/198/0/0 | 27 / 24 / 37 PASS | 32 / 34 PASS | 28/9, 19/16, 39/14 | 25 / 0 | 4/4 tracked, PASS, PASS | 0 |

"Findings" counts `runtime error`, `ERROR: AddressSanitizer`, `ERROR: LeakSanitizer`,
`SUMMARY:` and compiler `warning:` lines in the full row log: **0 of each on every row**. Each
oracle printed its own installed `CV_VERSION`. The production archive `libopencv_video_shim.a`
contains only `opencv_video_shim.o`, defining 25 `opencv_video_*` text symbols and 0 test
symbols. The oracle and homography counts are **identical to the Task 021 campaign** on every
version (including the 5.0.0 homography-seeded 19/16 split).

An earlier campaign start on head `eff6be7` was **stopped**, not counted: its 4.1.0 row's
`git diff --check` step returned 2 (`new blank line at EOF` in three new Task 022 files).
Commit `920d739` removed those trailing blank lines (whitespace only). The rebuilt probe has the
identical sha256 `b4ce0004…615e`, so the Part 1 evidence is unaffected. All four rows above were
then run from scratch on `920d739`.

**Instrumentation scope.** In this campaign `run_sanitizers.sh` instruments the **binding** and
links the **uninstrumented** prebuilt OpenCV. A clean result says nothing about native OpenCV
safety, as Part 1 shows. The research probe is not part of `test.sh`, `run_sanitizers.sh` or any
workflow, so it cannot affect these results.

**No production code changed.** No AUnit registrations were added: Task 022 is research and
documentation only, and the probe's expected-UB cases cannot run in a sanitizer-clean suite.
Regression equivalence is the criterion; registration count (198), export count (25) and every
oracle fixture count equal the Task 021 campaign.

## Inventory confirmation

* 25 production ABI exports (checker + `nm`), 198 AUnit registrations (checker + 198 run).
* Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all three Alire roots (checker).
* Crate version `0.1.0-dev`. No dependency added.
* `git diff --stat bc4d619 HEAD` touches only: `README.md`, `src/opencv-video.ads` (comments),
  `docs/tasks/022-…`, `docs/task022-qualification.md`, `scripts/research_ecc5_float_warp.sh`,
  `tests/cpp/ecc5_float_warp_probe.cpp`. `cpp/`, every `.adb`, every GPR and `.github/` are
  byte-identical to the baseline.
* CI topology unchanged: the PR workflow has repository-checks, linux, macos and
  linux-sanitizers; Windows stays a separate push-to-main-only workflow (checker "CI topology"
  PASS).

## Not claimed

* That any ECC operation is free of native UB on any OpenCV version. On 5.0 each is
  `SAFETY_GATE_NOT_ESTABLISHED`.
* That any ECC operation reaches the 5.0 finite-coordinate UB. No Level B or Level C reproducer
  exists; the Level A matrices are model-valid but were never shown to be produced by ECC.
* Anything about the 5.0 float kernels on targets other than x86_64 — neither that they share
  the x86 behaviour nor that they are safe for coordinates below `2^31`. Only x86_64 AVX2 and
  cpu_baseline were executed.
* That an out-of-bounds **read** occurs. None was observed (ASan clean); the defect is invalid pointer
  formation.
* Any AArch64/NEON result, including the NaN→0 hypothesis, which is unverified.
* That any mitigation works. None was implemented or validated.
* Any change to the Task 020 (`KNOWN_NATIVE_UB_UNMITIGATED`) or Task 021
  (`SAFETY_GATE_NOT_ESTABLISHED`) dispositions. They are unchanged.
* Release readiness. No version, tag or release change was made.

## Hosted CI

PR #22 targets `main`, is non-draft, and has auto-merge disabled. Its workflow covers only the
normal regression scope and does not run the research probe. Windows stays push-to-main only and
is not triggered by this PR. Final-head results are recorded on the PR rather than here, so this
file does not chase its own commit. A head that has not finished is reported as
`HOSTED_CI_PENDING`, separately from the technical dispositions.

## Part R1 — Ordinary-coordinate pointer formation (2026-10-19)

Starting head `0977d07c72dbd7f44aecf54eb3de0860751e1f79`: local branch, `origin` after fetch and
the PR #22 head all agreed. PR #22 open, non-draft, unmerged, auto-merge off; worktree clean. No
new branch or PR. The full analysis is in
[task note section R1](tasks/022-opencv5-float-warp-safety-audit.md#r1--ordinary-out-of-allocation-scalar-pointer-formation-2026-10-19).

**Setup.** Same instrumented OpenCV 5.0.0 as Part 1 (imgproc sha256
`73c4d0d95b3dd0ad5f52b3d7f094677b3bbbe347ac10e1f9e72e87f8a15e7292`, imports
`__ubsan_handle_pointer_overflow`). Rebuilt probe sha256
`fd3bb1efabaaae5799142e3fa359355fe43ea9b4646414f702b4fadb94711d74`, built with `-Werror` and 0
warnings. Host, compiler and runtime options as in Part 1, plus gdb 16.3 for the opt-in
`observe:` cases.

**New probe diagnostic.** `pointer_range()` / `pixel_pointer()` compute each scalar-tail index
in defined int64 arithmetic and compare `srcstep*iy + ix` against the **ASan-reported malloc
chunk** of the exact source (`__sanitizer_get_allocated_begin/size`), not against `rows*cols`.
They form no invalid pointer, do no pointer subtraction across objects, and cast no float to
int. Classes: `PTR_WITHIN_ALLOCATION`, `PTR_ONE_PAST`, `PTR_WITHIN_PARENT_OUTSIDE_ROI`,
`PTR_OUTSIDE_ALLOCATION` (tagged `ANY_ALIGNMENT` or `THIS_LAYOUT_ONLY` against the
placement-independent `fastMalloc` bound), `UNDETERMINED`.

### Default runner (all 28 original cases), with the new pointer column

**`RUNNER_EXIT=0`.** 10 `EXPECTED_UPSTREAM_UB_OBSERVED` (the same 10 cases and UBSan sites as
Part 1), **12 `SOURCE_LEVEL_UNSAFE_SANITIZER_SILENT`**, 6 `NO_UPSTREAM_REPORT`. 0 foreign UBSan,
0 ASan, 0 LSan, every exit 0.

| Cases | UBSan verdict (unchanged) | R1 pointer verdict |
|---|---|---|
| `conversion-check` | NO_UPSTREAM_REPORT | not evaluated (no warp) |
| `negative`, `level-a-far-control`, `level-a-matrix-b`, `level-a-boundary-below` (sanitizer-negative controls) | NO_UPSTREAM_REPORT (0 reports, control satisfied) | **`PTR_OUTSIDE_ALLOCATION`**, so source-level unsafe and sanitizer-silent |
| `level-a-boundary-above`, `level-a-overflow-to-inf`, `level-a-nonfinite-affine-coeff` | NO_UPSTREAM_REPORT (silent, as in Part 1) | **`PTR_OUTSIDE_ALLOCATION`** |
| the 10 UBSan-reporting Level A cases | EXPECTED_UPSTREAM_UB_OBSERVED | `PTR_OUTSIDE_ALLOCATION` |
| `trace-translation/euclidean/euclidean-seeded/affine/affine-n435` | NO_UPSTREAM_REPORT | **`PTR_OUTSIDE_ALLOCATION`**, first at iteration 2/2/1/2/1 |
| `level-c-*` (5) | NO_UPSTREAM_REPORT | not evaluated by the model (native capture below) |

The original Part 1 counts (10 / 18) are preserved above. R1 only splits the 18 silent cases
into 12 source-level unsafe and 6 not evaluated or conversion-only.

### Opt-in R1 cases

`sh scripts/research_ecc5_float_warp.sh r1 r1-sweep:97:neg r1-sweep:111:neg r1-sweep:33
observe:level-c-translation observe:level-c-euclidean observe:level-c-affine
observe:level-c-euclidean-seeded observe:level-c-affine-n435` gave **`EXIT=0`**. Every case had
UBSan 0 and ASan 0.

| Case | Dispatch | Scalar tail runs | Pointer verdict (AVX2 tail) | Key pointer |
|---|---|---|---|---|
| `r1-identity-control` | AVX2 | yes (`x=96`) | **`POINTER_RANGE_NEGATIVE`** (control satisfied) | worst offset 6207 = last element |
| `r1-ordinary-positive` `(2,1)` | AVX2 | yes | **`PTR_OUTSIDE_ALLOCATION (ANY_ALIGNMENT)`** L and N | (96,63) → (98,64), offset 6306; chunk L `[-16,6210)`, N `[-64,6216)` |
| `r1-ordinary-positive-baseline` | `cpu_baseline` (`useOptimized=0`) | yes | `PTR_OUTSIDE_ALLOCATION (ANY_ALIGNMENT)` L and N | same, offset 6306 |
| `r1-ordinary-negative` `(-2,-1)` | AVX2 | yes | `PTR_WITHIN_PARENT_OUTSIDE_ROI` | (96,0) → (94,-1), offset −3, inside the chunk header |
| `r1-ordinary-negative-baseline` | `cpu_baseline` | yes | `PTR_WITHIN_PARENT_OUTSIDE_ROI` | offset −3 |
| `r1-sweep:111:neg` | AVX2 | yes (`x=96..110`) | `PTR_OUTSIDE_ALLOCATION (THIS_LAYOUT_ONLY)` linear | (96,0) → (94,-1), offset −17, chunk begin −16 |
| `r1-sweep:97:neg` | AVX2 | yes | `PTR_WITHIN_PARENT_OUTSIDE_ROI` | offset −3 |
| `r1-sweep:33` `(2,1)` | AVX2 | yes (`x=32`) | L `ANY_ALIGNMENT`, N `THIS_LAYOUT_ONLY` | — |
| `r1-roi-positive` / `r1-roi-negative` | AVX2 | yes | `PTR_WITHIN_PARENT_OUTSIDE_ROI` | offsets 7330 / −19 inside parent chunk `[-360,7794)` |

**Width sweep** (`(2,1)`, native AVX2 tail hits L/N, and the same with `useOptimized=0`):
widths 32, 64, 96 and 128 give 0 / 0, with no scalar tail and no verdict. Widths 33, 97 and
129 give 64 / 64, and 98 gives 128 / 128, all `PTR_OUTSIDE_ALLOCATION`. Baseline (`uf = 8`) is
identical for these widths.

**Native observation (`observe:`; gdb register capture at the post-`lea` instruction of the
unmodified `opt_AVX2` kernels, chunk read at kernel entry).** All five real C exports, with
genuine Core handles:

| C export | status | linear: outside / in-chunk | nearest: outside / in-chunk |
|---|---|---|---|
| `estimate_ecc_translation` | 0 | 18 / 1262 | 6 / 442 |
| `estimate_ecc_euclidean` | 0 | 267 / 1589 | 98 / 542 |
| `estimate_ecc_euclidean_seeded` (θ 0.9, t ±2^20) | 3 | 128 / 0 | 64 / 0 |
| `estimate_ecc_affine` | 0 | 597 / 18539 | 198 / 6202 |
| `estimate_ecc_affine_seeded` (n435) | 3 | 2048 / 0 | 768 / 0 |

The first linear call per process is not classified (`UNMATCHED`; it is consumed by the gdb
bootstrap breakpoint), so these counts are lower bounds. 0 sanitizer reports in every run.

### Corrected dispositions (5.0.0, x86_64 as tested)

| Subject | Disposition | Evidence basis |
|---|---|---|
| `warpAffineLinearInvoker_32FC1`, ordinary int32 coordinates | **`DIRECT_NATIVE_UB_CONFIRMED`** | source + allocation analysis + native register capture; **no** sanitizer report |
| `warpAffineNearestInvoker_8UC1`, ordinary int32 coordinates | **`DIRECT_NATIVE_UB_CONFIRMED`** | same, captured separately |
| Both kernels, extreme/nonfinite (Part 1) | `DIRECT_NATIVE_UB_CONFIRMED` (unchanged) | UBSan `:3764` / `:439` |
| Translation ECC on 5.0 | **`KNOWN_NATIVE_UB_UNMITIGATED`** | Level B iter 2; Level C status 0 with 24 OOB |
| Euclidean ECC on 5.0 | **`KNOWN_NATIVE_UB_UNMITIGATED`** | Level B iter 2; Level C status 0 with 365 OOB |
| Euclidean seeded ECC on 5.0 | **`KNOWN_NATIVE_UB_UNMITIGATED`** | Level B iter 1 (in-domain seed); Level C status 3, 192 OOB |
| Affine ECC on 5.0 | **`KNOWN_NATIVE_UB_UNMITIGATED`** | Level B iter 2; Level C status 0 with 795 OOB |
| Affine seeded ECC on 5.0 | **`KNOWN_NATIVE_UB_UNMITIGATED`** | Level B iter 1 (in-domain seed); Level C status 3, 2816 OOB |
| Affine ECC on 4.x (Task 020) | `KNOWN_NATIVE_UB_UNMITIGATED`, **unchanged** | — |
| Translation/Euclidean ECC on 4.x (Task 021) | `SAFETY_GATE_NOT_ESTABLISHED`, **unchanged** | — |

Each operation was promoted on its own Level C capture; none was promoted by analogy.

### R1 regression and inventory

R1 changes only `tests/cpp/ecc5_float_warp_probe.cpp`, `scripts/research_ecc5_float_warp.sh`,
`README.md`, `src/opencv-video.ads` (comments only) and these two docs. No build input of the
production library, tests or examples changed: `cpp/`, every `.adb`, every GPR, the Alire
manifests and `.github/` are byte-identical to `0977d07`. The Part 2 four-version campaign
(on `920d739`, identical production sources) therefore stays valid as the regression evidence.
In addition, the normal OpenCV 5.0 regression (`scripts/test.sh`) was re-run on the R1 head; see
the R1 regression note below. An unchanged binding regression proves nothing about native
pointer safety.
