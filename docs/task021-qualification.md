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
findings and documentation commit. Later commits add only this qualification file.

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
