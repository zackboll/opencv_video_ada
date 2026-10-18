# Task 022 qualification — OpenCV 5.0.0 floating-point warp kernel safety audit

Starting `origin/main`: `bc4d619c4a33a5fcd55029674ccb7840f1cfbeef` (PR #21 merge, merged
2026-10-18T17:40:20Z). Branch `safety/022-opencv5-float-warp-audit` in a separate worktree
(`/home/zboll/git/opencv/video-task022`). No Task 022 branch, worktree or PR existed
beforehand, and the working tree was clean. Inventory at start and end: **25 production
exports, 198 AUnit registrations**, Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all
three Alire roots, crate `0.1.0-dev`. All unchanged.

**No normal production code was changed.** `cpp/` and every Ada body are byte-identical to the
baseline; the Ada spec and README changes are comments and dated notices only.

**Technical dispositions** (independent; full reasoning in
[the task note](tasks/022-opencv5-float-warp-safety-audit.md)):

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
