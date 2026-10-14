# Task 016 qualification — ECC eight-parameter homography alignment

## Baseline

* **Starting point.** Starting `origin/main` is `19ec682775422793e02ab6fbab097ff401dfcd51`, on
  branch `feature/016-ecc-homography`.
* **Inventory change.** Exports went from 21 to 22, and AUnit registrations from 163 to 176. The
  crate is `0.1.0-dev`.
* **Core pin.** `4da9d35ea21e1b2efe96296243ea668b488c6326` is unchanged in all three Alire roots.
* **Task 015 checks (baseline only).** These succeeded and were not re-examined:
  * pinned run 38326904553;
  * cross-platform run 38328499681;
  * Windows run 38328499614.

| Checkpoint | Commit | Content |
|---|---|---|
| 1 safety gate | `e3d72bf` | four-version source review, direct-native probe, memory |
| 1 API | `c39a071` | Ada API, 22nd export, smoke |
| 2 | `934517b` | 37-fixture oracle, 13 AUnit registrations |
| 3 | `0dc9694` | native boundary, injected faults, corruption, mutation campaign |
| 4 fix | `2696841` | boundary follows same-build native outcome (test-only, see below) |
| 4 docs | this commit | task note, this file, README |

## Safety gate

`SAFETY_GATE_ESTABLISHED` was recorded before any production code; see
`docs/ecc-homography-source-contract.md`.

**Sources reviewed.** `ecc.cpp`, `tracking.hpp` and `imgwarp.cpp` at 4.1.0, 4.6.0, 4.10.0 and 5.0.0.

**Known unguarded path.** `image_jacobian_homo_ECC` divides by `den = H20*x + H21*y + 1` with no
zero or tiny guard.
* `cv::divide` follows IEEE: it produces infinity or NaN and does not throw.
* `Mat::inv` of a singular 8x8 matrix returned zero.
* The native `cvIsNaN(rho)` and `lambda_d <= 0` guards end most such runs with `StsNoConv`.

**`warpPerspective`.** A zero projective denominator selects scale 0. Projected coordinates are
clamped before integer conversion. Buffers are fixed-size, and 1e30/1e300/NaN coefficients returned
normally (measured on 4.1.0, 4.10.0 and 5.0.0).

**What the binding's final checks cannot do.** The finite, H22 and schema checks run after the native
call, so they cannot retroactively protect native intermediate operations. Memory-safety rests on
the source review above.

**Sanitizer evidence.** No new ECC undefined behavior was observed in any campaign. Upstream OpenCV
was not instrumented, so absence of findings is not proof that upstream behavior is safe.

**Allocation failure.** It is handled by the established exception boundary: `cv::Exception` and
`std::bad_alloc` are caught in the shim, and the exception injections cover both stages. Full
upstream allocation-failure injection was not performed.

**Farnebäck.** The accepted upstream Farnebäck pointer-arithmetic finding is separate and does not
extend to ECC.

## Native observations versus binding requirements

The oracle records whatever native OpenCV does on the same build. The binding requirement is
agreement with that outcome:
* bitwise-equal coefficients and correlation on success, H22 exactly 1, and a finite result;
* `OpenCV_Error` with an unchanged result on a native exception.

How many fixtures succeed or raise is a native observation, not a requirement.

### Oracle (37 fixtures, `tests/cpp/ecc_homography_oracle.cpp`)

| OpenCV | Native successes | Native exceptions |
|---|---:|---:|
| 4.1.0 | 28 | 9 |
| 4.6.0 | 30 | 7 |
| 4.10.0 | 30 | 7 |
| 5.0.0 | 28 | 9 |

**Exceptions on every version:**
* constant images;
* uncorrelated noise (noise-vs-noise);
* strong perspective;
* near-zero denominator;
* denominator crossing zero;
* large translation;
* reflection.

**Version differences.** `lowgrad` and `illcond` succeed on 4.6.0 and 4.10.0 but raise on 4.1.0 and
5.0.0. All 37 fixture lines on 4.6.0 are byte-identical to 4.10.0, consistent with their
byte-identical `ecc.cpp`. 4.1.0 and 5.0.0 have the same success/exception pattern but differ
numerically, for example the `noise-vs-texture` correlation: 0.0905 on 4.1.0, 0.0795 on 5.0.0 and
0.0794 on 4.6.0/4.10.0.

**Wrong-but-finite successes.**
* `noise-vs-texture` (all versions): correlation below 0.2.
* `lowgrad` (4.6.0, 4.10.0): coefficients in the hundreds and H02 of about 7.8e4.
* `illcond` (4.6.0, 4.10.0): H00 about 3.81, H02 about -193.8, correlation 0.097. Its residual is worse
  than identity.

No correlation threshold was added to suppress these.

Every native success on every version has H22 exactly 1.0. No zero-denominator or difficult case was
converted into a fabricated identity result. Those cases raise natively and the binding reports
`OpenCV_Error`.

### Corrective commit `2696841` (test-only)

On 4.1.0 and 5.0.0, AUnit passed 176/176 but `scripts/test.sh` exited 1 with
`FAIL: low-gradient`. The native boundary had required that the three 4.10.0 wrong-but-finite
fixtures succeed, while the same-build direct `findTransformECC` raises for `lowgrad` and `illcond`
on those versions. This is an independent native observation, confirmed by both the oracle and the
boundary's own direct call.

The boundary now follows the same-build direct outcome for these three fixtures:
* on success, it requires bitwise publication and a correlation below 0.5;
* on a native exception, it requires an error status, a diagnostic and the 48 bytes untouched.

It still requires at least one wrong-but-finite success: `noise-vs-texture`, which is finite on
every qualified version.

No tolerance was relaxed, and the production shim did not change. The AUnit nonconvergence test
already accepted either outcome for those fixtures.

### Homography versus affine (`ECC_Homography_Versus_Affine`)

Five Checkpoint 2 fixtures (h20-pos, h21-neg, h20-h21-both, rot-persp, sst-persp) were measured on
4.10.0 using the interior residual in gray levels:

| Model | Interior residual |
|---|---|
| Homography | 0.35-0.36 |
| Affine | 0.77-1.49 |

The affine/homography ratio was 2.2-4.2. The test asserts affine > 1.8 x homography, a finite and
correlated affine result, and a nonzero homography projective term. It passes on every qualified
version (AUnit 176/176).

The very mild perspective fixture (ratio about 1.2) is deliberately excluded. Homography is **not**
claimed to always outperform affine ECC; for pure affine fixtures the two models are equal.

## Native boundary and fault campaign

`run_ecc_homography()` in `tests/cpp/native_boundary.cpp` calls the real export through Core-owned
handles. It runs in two builds:
* the production shim;
* the separate `OPENCV_VIDEO_TEST_FAULTS` build.

**Every successful case matches a direct `findTransformECC(MOTION_HOMOGRAPHY)` call bitwise.** That
covers:
* all nine coefficients and the correlation;
* geometric fixtures;
* option boundaries;
* Regions;
* aliasing;
* finite wrong outcomes.

**Every failure leaves the caller's 48 bytes unchanged.** This includes the padding and is checked
against a patterned sentinel.

**The fault build must reach this line:**
`PASS: ECC homography faults: 8 exception injections, 20 rejected corruptions, 5 accepted finite
projective matrices, 8 schema mutations, generic hook isolation`

That line covers:
* **8 exception injections.** `std::bad_alloc`, `cv::Exception`, `std::runtime_error` and a
  non-standard `throw 42`, each injected at stage 1 (before native) and stage 2 (after native,
  before validation). Each maps to the documented status with a diagnostic, and the result stays
  untouched.
* **20 rejected output corruptions:**
  * NaN or infinity in each of the nine coefficients, including H20, H21 and H22 in both signs;
  * finite H22 values other than 1: 0, 1.01, -1, and the two adjacent floats around 1;
  * NaN, +infinity or -infinity correlation.
* **5 accepted finite projective matrices** (H22 = 1). These are published exactly; no affine,
  determinant or perspective-magnitude policy is applied.
* **8 schema mutations:**
  * clone (storage identity);
  * 1x9 and 9x1 reshapes;
  * `CV_32S` over the same data;
  * a non-continuous column view;
  * a 3-D header;
  * 2x3;
  * `CV_64F`.
* **Generic hook isolation.** The 2x3 `opencv_video_test_corrupt_ecc` modes do not reach the
  homography export.

The test hooks exist only in the fault build. Each production archive was checked with `nm`: 22
`T opencv_video_*` symbols and 0 `test_` symbols.

## Mutation campaign (Checkpoint 3, unchanged)

The table is in the source contract. Results:
* Mutants 1-11 and 13-20 were **killed** (19).
* Mutant 12 (force H22 = 1 after the H22 check) is **equivalent**. The check already guarantees
  H22 = 1, so the later assignment is observationally redundant.
* Variant 12b (remove the check and force H22 = 1) was killed.

Mutants 17-19 are killed only by the fault-build corruption hooks.

Checkpoint 4 changed only test code, in the boundary branches for natively raising wrong-but-finite
fixtures. It made no production change, so the campaign was not repeated. The mutants that used
those boundary lines (1-4, 16, 20) were all killed by earlier, unchanged checks: the oracle,
`identity`, `translation (2,1)` and `constant`.

## Version campaigns

Each campaign ran in an independent `git clone --no-local` of the exact commit, rebuilt from source.
Nothing was copied from the worktree.

**Steps per campaign:**
1. development, validation and release builds;
2. repository checker;
3. configuration tests;
4. `sh -n` on every script, and `git diff --check`;
5. `scripts/test.sh`, which runs:
   * the builds;
   * every direct-native oracle, including the 37-fixture homography oracle;
   * AUnit;
   * the uninstrumented native boundary and fault drivers;
6. `alr exec -- sh scripts/run_sanitizers.sh` (instrumented);
7. `homography_smoke`;
8. the examples build and `lk_synthetic`.

**Library selection.** Pinned versions used `~/.cache/video-task004/install-<v>` via
`PKG_CONFIG_PATH` and `LD_LIBRARY_PATH`. 5.0.0 resolves `opencv5.pc`. 4.6.0 ran in the local
`opencv-core-debian12-opencv-4.6` container (Debian 12 `libopencv-dev`) against a read-only bind of
the repository.

Clean-clone campaign of `2696841d01e95c5fed79f096d853069c716ab75e`:

| OpenCV | Library loaded by `run_tests` | Builds D/V/R | Checker, config, `sh -n`, diff | `test.sh` | AUnit run/ok/fail/err | Homography oracle | Instrumented sanitizers | Smoke, `lk_synthetic` |
|---|---|---|---|---|---|---|---|---|
| 4.1.0 | `install-4.1.0/lib/libopencv_video.so.4.1` | 0/0/0 | 0/0/0/0 | 0 | 176/176/0/0 | 28 OK / 9 EXC | rc 0, 0 findings | 0, 0 |
| 4.6.0 | container `/usr/lib/x86_64-linux-gnu/libopencv_video.so.406` | 0/0/0 | 0/0/0/0 | 0 | 176/176/0/0 | 30 OK / 7 EXC | rc 0, 0 findings | 0, 0 |
| 4.10.0 | system `/usr/lib/x86_64-linux-gnu/libopencv_video.so.410` | 0/0/0 | 0/0/0/0 | 0 | 176/176/0/0 | 30 OK / 7 EXC | rc 0, 0 findings | 0, 0 |
| 5.0.0 | `install-5.0.0/lib/libopencv_video.so.500` (`opencv5.pc`) | 0/0/0 | 0/0/0/0 | 0 | 176/176/0/0 | 28 OK / 9 EXC | rc 0, 0 findings | 0, 0 |

**Reported PASS lines.** In every row both `test.sh` and the instrumented run reported:
* the production and fault boundary PASS lines;
* every retained direct-native oracle.

**Before the fix.** On Checkpoint 3 head `0dc9694`, the 4.1.0 and 5.0.0 clean clones gave AUnit
176/176 but `test.sh`/sanitizers rc 1 (`FAIL: low-gradient`). That led to `2696841`.

**5.0.0 path.** On 5.0.0, `findTransformECC` forwards to `findTransformECCWithMask`. The oracle
and boundary exercise exactly that path.

**Final head.** The documentation-only final head is re-qualified the same way; see the PR.

**Checks in every row:**
* `Native_Version` matched the selected version.
* The loaded library came from the selected install, checked with `ldd tests/bin/run_tests`.
* Each oracle printed its own `CV_VERSION`.
* No sanitizer report or runtime error appeared: zero `AddressSanitizer`, `runtime error`,
  `LeakSanitizer` or `SUMMARY:` lines in any instrumented log.

**Sanitizer scope.** The shim, `native_boundary.cpp` (production and fault builds) and every
direct-native oracle are compiled with `-fsanitize=address,undefined`, and leak detection is on.
Instrumentation was confirmed with `nm` (`__asan_report*` / `__ubsan_handle*` present). The prebuilt
OpenCV libraries are **not** instrumented. `scripts/test.sh` runs only the `native`, uninstrumented
mode of the same driver.

**Ada-level results.** The Ada-level AUnit, smoke and example results come from uninstrumented
binaries.

## Memory (Checkpoint 1, OpenCV 4.10.0 x86_64, direct native, peak RSS)

| Native model | 4096x4096 (2^24 px) |
|---|---:|
| Translation ECC | about 984 MiB |
| Affine ECC | about 1,239 MiB |
| Homography ECC | about 1,879 MiB |

Homography also measured about 1,879 MiB at 16384x1024.

**The Jacobian.** The homography Jacobian alone is 512 MiB at the area limit. Its column count of
`8*width`, at most 131,072, stays below `INT_MAX`.

**Caveats.**
* These are measured peaks, not guaranteed limits. They depend on the native version, the
  allocator, the images and the execution path.
* Input validation does not guarantee free memory. An allocation failure can still occur inside the
  accepted geometry; it surfaces as `OpenCV_Error`.
* The geometry envelope was kept because the safety and resource assessment supported it.
* No 4.1.0, 4.6.0 or 5.0.0 memory values were measured.
* No maximum-geometry workload is in CI.

## Remaining limitations

* **Small basin.** The identity-initialized convergence basin is small. Caller seeding is out of
  scope.
* **Finite does not mean correct.** Finite results can be wrong, and the correlation is not a
  confidence score. It is computed before the final native update.
* **Zero denominator.** Points where `D = 0` are a property of the returned matrix. The binding does
  not resample and does not detect them.
* **Upstream sanitizers.** Upstream OpenCV sanitizer cleanliness is not established.
* **Version differences.** Native results differ across versions; see the oracle table. AUnit
  assertions use measured tolerances, and same-build oracle equality is exact.

## Windows and hosted CI

Windows stays push-to-main only and was not dispatched. The new homography code has **not** run on
Windows. It will first run in the post-merge Windows job, and Task 015's Windows success is baseline
evidence only.

Hosted cross-platform and pinned-matrix results for the final head are recorded in the PR
conversation. They are not pre-claimed here.
