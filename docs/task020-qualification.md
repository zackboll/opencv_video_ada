# Task 020 qualification — existing ECC affine safety audit

Starting `origin/main`: `b09a65a78aa39f16a6d358f81517d3b8186eb334` (PR #19 merge, merged
2026-10-18T00:40:48Z). Branch `safety/020-existing-affine-ub` in a separate worktree
(`/home/zboll/git/opencv/video-task020`). No existing Task 020 branch or PR was found, and the
working tree was clean at the start. Inventory at start and end: **25 production exports, 198
AUnit registrations**, Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all three Alire
roots, crate `0.1.0-dev`. All unchanged.

**Technical disposition: `KNOWN_NATIVE_UB_UNMITIGATED`.** Hosted CI is reported in the last
section. Passing regression does not resolve the native safety problem, and no release-readiness
or safety-qualification claim is made.

## PR #19 post-merge CI (read once at start, not polled)

| Run | Workflow | Head | State when read |
|---|---|---|---|
| 39083652008 | Cross-platform Video | `b09a65a` | queued |
| 39083652015 | Windows post-merge Video | `b09a65a` | in progress |

Both were incomplete when Task 020 began and are recorded as such.

## Part 1 — Expected native-UB reproductions (research evidence, NOT sanitizer-clean tests)

These runs are expected to produce UBSan reports from inside OpenCV. They are kept separate
from the regression campaign below and never run in CI. Tooling:
`scripts/research_ecc_affine_overflow.sh` and `tests/cpp/ecc_affine_overflow_probe.cpp`.
The instrumented OpenCV builds are `~/.cache/video-task017/install-<v>-san`
(`-fsanitize=address,undefined,float-cast-overflow`); `ldd` confirmed them for every run.
UBSan ran with `halt_on_error=0`, and ASan with `detect_leaks=1`.

### Level A — direct native replay of the exact PR #19 matrices

| OpenCV | `negative` | Matrix A 97x64 | Matrix B 100x64 |
|---|---|---|---|
| 4.1.0 | OK, 0 reports | 4 signed-overflow: `imgwarp.cpp:2285:41` `-2147483648 + -3762320`, `2286:41` `-2147483648 + -2147483632`, `imgwarp.sse4_1.cpp:204:21` `-2147483648 + -3761824`, `205:21` `-2147483648 + -2147483136` | 2: `imgwarp.cpp:2286:41` `-972750720 + -1358970928`, `imgwarp.sse4_1.cpp:205:21` `-972750720 + -1358970432` |
| 4.6.0 | OK, 0 | 4: lines `2269:41`, `2270:41`, `sse4_1:204:21`, `205:21` (same operands) | 2: `2270:41`, `sse4_1:205:21` (same operands) |
| 4.10.0 | OK, 0 | 4: lines `2373:41`, `2374:41`, `sse4_1:204:21`, `205:21` (same operands) | 2: `2374:41`, `sse4_1:205:21` (same operands) |
| 5.0.0 | OK, 0 | 0 reports (legacy integer path not executed; see the task note, section 5) | 0 reports |

The int64 model printed the same operands on every version: Matrix A has 5,184 overflowing
pixels per mode, Matrix B 5,719. With `cv::setUseOptimized(false)`, the scalar nearest
additions are also reported: 4.1.0 `2244:45/2245:45`, 4.6.0 `2229:45/2230:45`, 4.10.0
`2329:45/2330:45`. The negative control `[1.03 0.02 2; -0.01 0.98 1]` reported zero model
overflows and zero UBSan reports on every version.

### Level B — ECC-generated intermediates (native optimizer, iteration-capped replay)

| Case | 4.1.0 | 4.6.0 | 4.10.0 | 5.0.0 |
|---|---|---|---|---|
| `trace-seeded-n103` (97x64) | **unsafe at iteration 3**: `[-45747.7890625 88966.1875 -3195937.25 -69792.625 128145.84375 -4427218.5]`, 4,422 px | not reproduced | not reproduced | not reproduced |
| `trace-seeded-n435` (100x64) | not reproduced | not reproduced | not reproduced | **unsafe at iteration 3**: `[-5597.05908203125 8140.32177734375 285284.03125 -9895.33203125 -362406 484909.9375]` = **PR #19 Matrix B, bit-exact** |
| `trace-identity-97`, `-100` | not reproduced (100 iterates) | not reproduced | not reproduced | not reproduced |

The instrumented and uninstrumented Release builds gave the same iteration-3 matrices. A trace
stops after the capped call that produced the unsafe matrix. It never runs the iteration that
would consume that matrix, so trace logs contain no UBSan reports by design. Execution of the
overflow is shown by Levels A and C.

### Level C — real exported binding, validated caller inputs

| Case | 4.1.0 | 4.6.0 | 4.10.0 | 5.0.0 |
|---|---|---|---|---|
| `level-c-seeded-n103` | **4 signed-overflow reports** (`imgwarp.cpp:2285/2286`, `sse4_1:204/205`) with stack `ecc_affine_entry` (shim:926) -> `opencv_video_estimate_ecc_affine_seeded` (shim:1171); status 3 "NaN encountered"; reproduced twice | 0 reports; status 3 | 0; status 3 | 0; status 3 |
| `level-c-seeded-n435` | 0; status 3 | 0; status 3 | 0; status 3 | 0; status 3 (5.0 does not execute the integer path) |
| `level-c-identity-97`, `-100` | 0; status 0 | 0; status 0 | 0; status 0 | 0; status 0 |

No case produced an ASan, LSan or unexpected UBSan finding.

### Bounded seeded search (opt-in `search-identity` / `search-seeded`)

The trials are deterministic: an LCG keyed by the trial index sets the geometry (32-160 x
32-128) and one of five image kinds (translation, moderate affine, low contrast, unrelated,
strong anisotropy). Each run used 100 iterations, eps 1e-8 and no mask.

| OpenCV | Identity trials / unsafe | Seeded trials / unsafe |
|---|---|---|
| 4.1.0 | 570 / 0 | 570 / 0 |
| 4.6.0 | 450 / 0 | 450 / 0 |
| 4.10.0 | 570 / 0 | 570 / 0 |
| 5.0.0 | 570 / 0 | 570 / 0 |

Result: `NOT_REPRODUCED`, not `SAFE`. The search domain differs from Task 019's: it uses
varied geometry and image kinds rather than fixed geometry with masks. It shows the
reproducers are rare, not that they are impossible.

## Part 2 — Mitigation experiment (temporary patched OpenCV, research only)

`docs/research/task020_imgwarp_guard.py` was run on scratch copies of the 4.10.0 and 4.1.0
sources, and its output was checked byte-identical with `cmp` against the sources that were
built. The patched trees were built with the same CMake configuration and sanitizer flags into
`/tmp/t020-patch/install-<v>-san`, and `ldd` confirmed the patched libraries were loaded.

| Check | 4.10.0 patched | 4.1.0 patched |
|---|---|---|
| `negative`, Level A Matrix A, Matrix B | 0 / 0 / 0 reports (original 0 / 4 / 2) | 0 / 0 / 0 (original 0 / 4 / 2) |
| `level-c-seeded-n103` through the real export | 0 reports; status 3 "NaN encountered" | 0 reports (original 4); status 3 "NaN encountered" |
| `warpAffine` corpus (99 lines: 4 geometries x 12 transforms x 2 borders x 3 type/mode pairs, plus 3 ECC fits) | byte-identical to original | byte-identical to original |
| Native affine ECC, 9 fits (identity, n103, n435 x 3 geometries) | byte-identical | 8 byte-identical. The original run of 100x64 n103 executed the overflow; the patched run has the same final warp, with error "NaN encountered" instead of "lambda_d <= 0" |
| Default vs `setUseOptimized(false)` | Normal transform identical to original in both modes. Matrix A: patched output all-border (sum 0) in both modes; the original UB output depends on the mode (6220 vs 2186) | not run |
| 1024x1024 `CV_32FC1` linear warp x50, 3 runs | 708-718 ms (original 710-740 ms) | not run |
| Existing oracles / AUnit against patched library | not run | not run |

On the tested paths the patch removes every reproduced overflow and keeps in-range results
bit-identical. It is **not** a qualified mitigation; see Option A in the task note for the
missing evidence.

## Part 3 — Normal regression campaign (binding qualification, unchanged scope)

Each row is an independent `git clone --no-local` of the committed head, built from source.
The pinned 4.1.0, 4.6.0 and 5.0.0 rows use the uninstrumented Release installs
`~/.cache/video-task017/install-<v>` through `PKG_CONFIG_PATH` and `LD_LIBRARY_PATH`. The
4.10.0 row uses the system package. Steps per row:

* repository checker, the 12 configuration tests, `sh -n` on every script, and
  `git diff --check` against the baseline;
* Development, Validation and Release `alr build`;
* `scripts/test.sh` (every oracle, AUnit, uninstrumented native boundary, fault and corruption);
* `scripts/run_sanitizers.sh` (ASan, UBSan and leak detection over the shim, boundary, fault
  build and every direct oracle);
* the export inventory (`nm`);
* the `lk_synthetic` example, and the `ecc_smoke` and `homography_smoke` programs.

All rows ran on clean clones of `cf2bea4` (the evidence/disclosure commit). Later commits
change documentation only.

| OpenCV | Library loaded by `run_tests` (`ldd`) | Checker, config, `sh -n`, diff | Builds D/V/R | AUnit run/ok/fail/err | Affine oracle (32) / seeded (34) | Homography / seeded / masked oracle (native ok/exc) | Sanitizer step | Exports / test symbols | `lk_synthetic`, smokes |
|---|---|---|---|---|---|---|---|---|---|
| 4.1.0 | `install-4.1.0/lib/libopencv_video.so.4.1` | 0/0/0/0 | 0/0/0 | 198/198/0/0 | PASS / PASS | 28/9, 20/15, 39/14 | rc 0, 0 findings | 25 / 0 | 0, 0 |
| 4.6.0 | `install-4.6.0/lib/libopencv_video.so.406` | 0/0/0/0 | 0/0/0 | 198/198/0/0 | PASS / PASS | 30/7, 20/15, 39/14 | rc 0, 0 findings | 25 / 0 | 0, 0 |
| 4.10.0 | system `/usr/lib/x86_64-linux-gnu/libopencv_video.so.410` | 0/0/0/0 | 0/0/0 | 198/198/0/0 | PASS / PASS | 30/7, 20/15, 39/14 | rc 0, 0 findings | 25 / 0 | 0, 0 |
| 5.0.0 | `install-5.0.0/lib/libopencv_video.so.500` | 0/0/0/0 | 0/0/0 | 198/198/0/0 | PASS / PASS | 28/9, 19/16, 39/14 | rc 0, 0 findings | 25 / 0 | 0, 0 |

Every row has these properties:

* `Native_Version` matched the selected version.
* Each oracle printed its own `CV_VERSION`.
* The logs contained zero `runtime error`, `ERROR: AddressSanitizer`, `ERROR: LeakSanitizer`
  or `SUMMARY:` lines, and zero compiler `warning:` lines.
* `lk_synthetic` reported 4/4 tracked points.
* `ecc_smoke` and `homography_smoke` printed PASS.

The oracle counts equal the Task 018 campaign on every version. The production archive
contains only `opencv_video_shim.o`, and no build file, script or workflow references the
research probe. A separate worktree run on 4.10.0 gave the same results.

The repository checker reports 25 ABI declarations and imports and 198 AUnit
registrations. Core pin `4da9d35ea21e` appears in all three roots.

**Instrumentation scope.** In this campaign `run_sanitizers.sh` instruments the **binding**
and links the **uninstrumented** OpenCV. A clean result therefore says nothing about native
OpenCV safety, as the Part 1 evidence shows. The existing affine oracles (32 and 34 fixtures)
and the homography oracles still match their own version's native output. The research probe
is not part of `test.sh` or `run_sanitizers.sh`, so it has no effect on these results.

## Not claimed

* That either affine API is free of native UB on any OpenCV version.
* That the identity API is safe: its result is `NOT_REPRODUCED`.
* That the 5.0 float kernels are UB-free for arbitrarily large iterates.
* That the research patch is a qualified mitigation (see Part 2).
* Any Windows, macOS, AArch64 or non-AVX2 result for the research probe. The probe is
  Linux/x86_64 only.
* An operand-level audit of translation or Euclidean ECC, or of `warpPerspective`.
* Release readiness. No version, tag or release change was made.

## Hosted CI

Recorded in the PR. The PR workflow (repository-checks, linux, macos, linux-sanitizers)
exercises only the normal regression scope. It does not, and must not, run the research probe.
Windows remains push-to-main only.
