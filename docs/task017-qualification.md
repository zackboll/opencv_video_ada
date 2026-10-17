# Task 017 qualification — caller-seeded ECC homography alignment

## Baseline

* **Starting point.** `origin/main` was `ebaf0e75a8e35894f2f3b464e69d03487e3d9ade` (merge of PR #16,
  Task 016, merged 2026-10-14). Branch `feature/017-seeded-ecc-homography`; no other Task 017 branch
  or PR existed.
* **Baseline checks on that commit (4.10.0).** `alr -n build`, `scripts/test.sh` (AUnit 176/176) and
  the instrumented `scripts/run_sanitizers.sh` succeeded; the repository checker reported 22 exports
  and 176 registrations.
* **Inventory change.** Exports 22 → 23; AUnit registrations 176 → 186. Core pin
  `4da9d35ea21e1b2efe96296243ea668b488c6326` unchanged in all three Alire roots.

| Step | Commit | Content |
|---|---|---|
| safety gate | `7fb0cb8` | four-version source review, probes, instrumented OpenCV, diagnostic `ecc.cpp` |
| API | `485d11e` | Ada overload, shared `ecc_homography_entry`, 23rd export |
| oracle/AUnit | `9cdc57a` | 35-fixture oracle, 10 AUnit registrations |
| boundary | `78229b7` | native boundary, seed domain, faults, corruption |
| docs | `d791226` | task note, README |
| mutation fix | `523b957` | diagnostic checks for seed rejections (test-only) |
| report | this commit | this file |

## Safety gate

`SAFETY_GATE_ESTABLISHED` was recorded before any production code; see
`docs/ecc-homography-seeded-source-contract.md`. Summary:

* Native ECC applies **no constraint** to the eight updated coefficients. The seed domain therefore
  only fixes the starting point; it is not claimed to bound later iterations, and it is not an
  output policy.
* Every native operation that consumes the unconstrained warp was reviewed on 4.1.0, 4.6.0, 4.10.0
  and 5.0.0: `warpPerspective` clamps coordinates in `double` before integer conversion (NaN maps to
  `INT_MAX`); the Jacobian denominator divides without a guard (IEC 60559 ±inf/NaN, observed to reach
  exactly 0); Hessian/projection/`lambda` values are narrowed `double → float` (IEC 60559 rounds to
  ±inf; measured finite maximum 7.75e20); LU and the update use values arithmetically only. No warp
  value indexes memory or sizes an allocation.
* **Instrumented OpenCV.** Core/imgproc/video for all four versions were built from source with
  `-fsanitize=address,undefined,float-cast-overflow` (confirmed with `nm -D`). The probe ran 1995
  in-domain seeds for one iteration and 595 for 100 iterations, plus 24 edge seeds, per version:
  0 reports, exit 0 everywhere.
* **Diagnostic `ecc.cpp`.** A scratch copy of 4.10.0 `ecc.cpp` trapped any finite narrowing above
  `FLT_MAX` across 64x96 … 32x16384 geometries: 0 traps.
* **Reliance.** IEC 60559 floating point on every qualified target (x86-64, arm64; GCC and Clang).

## Native observations versus binding requirements

The oracle records what native OpenCV does on the same build. The binding requirement is agreement
with it (bitwise coefficients and correlation on success, H22 exactly 1; `OpenCV_Error` with an
unchanged result on a native exception). Success/exception counts are observations.

### Oracle (35 fixtures, `tests/cpp/ecc_homography_seeded_oracle.cpp`)

| OpenCV | Native successes | Native exceptions |
|---|---:|---:|
| 4.1.0 | 20 | 15 |
| 4.6.0 | 20 | 15 |
| 4.10.0 | 20 | 15 |
| 5.0.0 | 19 | 16 |

* The only outcome difference is `limit-linear` (all linear terms 4): a finite, meaningless matrix
  (correlation 0.003 / 0.0009) on 4.x, an exception on 5.0.0. 4.6.0 output is byte-identical to
  4.10.0 apart from the version line.
* **Seed consumption (every version).** Identity initialization raises and the near seed converges
  for strong perspective (`H20 = H21 = 0.002`), a 40/30-pixel shift, translation+perspective, a
  reflection, 15% scale+perspective and 0.15 shear+perspective. Converged results were within
  0.0037 (linear), 0.093 (translation) and 1.8e-5 (projective) of the truth on every version;
  correlation 0.989-1.000.
* **Native exceptions on every version:** identity seeds outside the basin, wrong-direction and
  insufficiently close perspective seeds, a wrong affine seed, `den-edge-x` (`H20` at the domain
  edge), the translation/all-limits seeds, the all-zero and `1e-6` linear seeds, constant images and
  noise.
* **Wrong-but-finite successes:** `pb-wrong-persp-it1` (correlation 0.28, output outside the seed
  domain), `aniso-seed` (correlation 0.013-0.016, coefficients in the hundreds to ten-thousands) and
  `limit-linear` on 4.x. They are published unchanged; no threshold hides them.
* **The truth itself can be outside the domain.** `H20 = H21 = 0.002` on 128x128 gives
  `0.002*127*2 = 0.508 > 0.5`, so that exact seed is rejected; a 0.0018 seed converges to it.

## Native boundary and fault campaign

`run_ecc_homography_seeded()` calls the real export through Core-owned handles, in the production
build and the separate `OPENCV_VIDEO_TEST_FAULTS` build:

* bitwise equality with an independent direct seeded `findTransformECC` on success, and failure
  agreement with all 48 bytes (including padding) untouched otherwise: near seeds, option
  boundaries, ten domain-edge or wrong seeds on two fixtures, Regions, same-handle and shared
  allocation;
* identity seed byte-identical to the identity export on four pairs (two successes, two failures);
* 62 seed rejections (27 nonfinite, 6 non-unit H22, 16 linear, 6 translation, 7 denominator), each
  with the documented diagnostic, plus the geometry rule on 128x256 and 256x128, and the Region's own
  geometry;
* option validation precedes seed validation;
* an out-of-domain native result passed back as a seed is rejected;
* fault build: `PASS: ECC homography seeded faults: 8 exception injections, 20 rejected corruptions,
  5 accepted finite projective matrices, 8 schema mutations, seed rejection before hooks`. Mode 34
  (coefficients near `3e38`) is published exactly: the seed domain is not an output policy.

Production archives: 23 `T opencv_video_*` symbols, 0 `test_` symbols (`nm`).

## Mutation campaign

Run on a scratch copy of the worktree at `d791226` (mutant 17 rerun after `523b957`); each mutant rebuilt and checked with both boundary
builds (C mutants) and the full AUnit suite.

| # | Mutant | Result | First killer |
|---|---|---|---|
| 1 | C: drop H22 seed check | killed | boundary "non-unit H22 seed accepted" |
| 2 | C: skip H11 linear bound | killed | boundary linear rejection |
| 3 | C: skip H12 translation bound | killed | boundary translation rejection |
| 4 | C: swap rows/cols in the denominator rule | killed | boundary geometry rule |
| 5 | C: denominator bound 1.0 | killed | boundary "out-of-domain native output accepted as a seed" |
| 6 | C: skip H22 finiteness | killed | boundary "seed rejected for the wrong reason" |
| 7 | C: transpose the seed | killed | boundary strong perspective; 6 AUnit tests |
| 8 | C: drop projective seed terms | killed | boundary; 6 AUnit tests |
| 9 | C: swap H20/H21 arguments | killed | boundary bitwise; oracle |
| 10 | C: ignore the seed | killed (compile) | see 10b |
| 10b | C: ignore the seed, compiles | killed | boundary strong perspective; 5 AUnit tests |
| 11 | C: identity export uses a nonidentity seed | killed | boundary identity; Task 016 AUnit |
| 12 | Ada: drop H22 check | killed | `ECC_Homography_Seeded_Validation` |
| 13 | Ada: wrong dimension for H21 | killed | `ECC_Homography_Seeded_Validation` |
| 14 | Ada: swap H20/H21 arguments | killed | oracle |
| 15 | Ada: swap H02/H10 arguments | killed | 5 AUnit tests |
| 16 | Ada: skip H11 bound | killed | `ECC_Homography_Seeded_Validation` |
| 17 | Ada: skip H22 finiteness | **survived**, then killed | `ECC_Homography_Seeded_Validation` after `523b957` |
| 18 | Ada: return the initial correlation | killed | 5+ AUnit tests |
| 19 | Ada: normalize H22 to 1 | killed | `ECC_Homography_Seeded_Validation` |

Mutant 17 survived the first run because a NaN H22 was still rejected (by the C shim, or by the
Ada H22 test) and the test checked only *that* it was rejected in preflight. `523b957` adds
diagnostic checks; the rerun killed it. The production code did not change.

## Memory (direct native, 4.10.0, x86_64, 10 iterations, seeded, peak RSS)

| Size | Peak RSS | Time |
|---|---:|---:|
| 128x128 | 22 MiB | 0.03 s |
| 1024x1024 | 136 MiB | 0.30 s |
| 2048x2048 | 484 MiB | 1.2 s |
| 4096x4096 (2^24 px) | 1,876 MiB | 4.5 s |
| 1024x16384 (2^24 px) | 1,877 MiB | 3.4 s |

Equal to Task 016's identity-initialized homography (1,879 MiB): seeding adds no allocation. The
seed in these runs raised natively after its iterations; the peak is reached during the iterations.

## Version campaigns

Each campaign ran in an independent `git clone --no-local` of `523b957`, rebuilt from source.
Steps: development, validation and release builds; repository checker; configuration tests; `sh -n`
on every script; `git diff --check`; `scripts/test.sh`; instrumented `scripts/run_sanitizers.sh`;
`homography_smoke`; examples build and `lk_synthetic`. Pinned versions used
`~/.cache/video-task017/install-<v>` through `PKG_CONFIG_PATH`/`LD_LIBRARY_PATH` (Release,
core/imgproc/video, no IPP/OpenCL/ITT). 4.6.0 used the same kind of local source build rather than
the Debian 12 container.

Clean-clone campaign of `523b9576a496fe3c596101b802a1ebcf68c8415f`:

| OpenCV | Library loaded by `run_tests` | Builds D/V/R | Checker, config, `sh -n`, diff | `test.sh` | AUnit run/ok/fail/err | Task 016 oracle | Seeded oracle | Instrumented sanitizers | Smoke, `lk_synthetic` |
|---|---|---|---|---|---|---|---|---|---|
| 4.1.0 | `install-4.1.0/lib/libopencv_video.so.4.1` | 0/0/0 | 0/0/0/0 | 0 | 186/186/0/0 | 28 OK / 9 EXC | 20 OK / 15 EXC | rc 0, 0 findings | 0, 0 |
| 4.6.0 | `install-4.6.0/lib/libopencv_video.so.406` | 0/0/0 | 0/0/0/0 | 0 | 186/186/0/0 | 30 OK / 7 EXC | 20 OK / 15 EXC | rc 0, 0 findings | 0, 0 |
| 4.10.0 | system `/usr/lib/x86_64-linux-gnu/libopencv_video.so.410` | 0/0/0 | 0/0/0/0 | 0 | 186/186/0/0 | 30 OK / 7 EXC | 20 OK / 15 EXC | rc 0, 0 findings | 0, 0 |
| 5.0.0 | `install-5.0.0/lib/libopencv_video.so.500` (`opencv5.pc`) | 0/0/0 | 0/0/0/0 | 0 | 186/186/0/0 | 28 OK / 9 EXC | 19 OK / 16 EXC | rc 0, 0 findings | 0, 0 |

The Task 016 oracle counts are unchanged from Task 016 on every version, and the 4.6.0 seeded oracle
is byte-identical to 4.10.0 after the version line. The documentation-only final head is not a new
code state; it is covered by hosted CI (see the PR).

**Checks in every row:** `Native_Version` matched; `ldd tests/bin/run_tests` showed the selected
library; each oracle printed its own `CV_VERSION`; zero `AddressSanitizer`, `runtime error`,
`LeakSanitizer` or `SUMMARY:` lines. The shim, both boundary builds and every direct oracle are
instrumented in `run_sanitizers.sh`; the OpenCV libraries used by the campaign are the
uninstrumented Release builds (instrumented OpenCV was used for the safety-gate probes only).

## Remaining limitations

* **Seeds are not composed.** A previous pair's result is not automatically a good seed for the
  next pair, and it may lie outside the seed domain.
* **Finite does not mean correct.** See the wrong-but-finite fixtures above.
* **Zero denominator.** Points where `D = 0` are a property of the returned matrix; the binding does
  not resample and does not detect them. Native iterations can pass through `D = 0` internally.
* **IEC 60559 reliance** is explicit; non-IEC targets are not qualified.
* **Windows.** The new code has not run on Windows; it first runs in the push-to-main Windows job.
  Hosted CI results for the final head are recorded in the PR conversation, not pre-claimed here.

## Task 017-R1 — Deterministic C ABI padding correction

**Root cause and original behavior.** `opencv_video_ecc_homography_result` is 48 bytes: nine
Float32 coefficients at 0-35, four padding bytes at 36-39, Float64 correlation at 40-47. Up to
`97e0ef2`, the shared `ecc_homography_entry` (used by both the Task 016 identity-initialized
export and the Task 017 seeded export) assigned the ten members of an uninitialized local object
and published it with `*result = computed`. A structure copy need not preserve, initialize or
even copy padding, so bytes 36-39 of a successful result were unspecified. The existing tests
compared only bytes 0-35 and the correlation on success, so they did not observe this.

**Correction (`fa7303c`).** After every input, seed, native and output-schema check has passed,
the shim builds the representation in a local `unsigned char[48]`. It zero-fills that buffer,
writes each field with `std::memcpy` at its `offsetof` offset, clears bytes 36-39 explicitly, and
publishes all 48 bytes with one `std::memcpy`. New compile-time checks require the type to be
trivially copyable and standard-layout, and require padding to be exactly `[offsetof(h22) + 4,
offsetof(correlation)) = [36, 40)`. These are in addition to the existing layout assertions in
`opencv_video_shim.h`. The ABI, the Ada record, offsets, alignment and numerical values are
unchanged. The affine, Euclidean and translation results were not changed.

**Guarantees.**
* **Success:** all 48 bytes are defined, and bytes 36-39 are zero. The bytes depend only on the
  coefficients and correlation, not on the caller's prior buffer contents.
* **Failure:** the caller's object is not written at all. The only write to `*result` is the
  final publication, and nothing zeroes or prefills it before validation.

**Tests (`tests/cpp/native_boundary.cpp`).**
* `hom_matches_direct`, used by every success comparison in both homography test functions,
  now also requires bytes 36-39 to be zero. The repeat-call checks compare all 48 bytes after a
  different prefill, where before they compared 36 bytes plus the correlation.
* Both injected-finite loops (modes 30-34) require zero padding.
* New `run_ecc_homography_padding()`:
  * Five successful cases (identity export on two pairs, identity seed on two pairs, and a near
    seed where identity raises), each run with three distinct prefills whose padding bytes are
    all nonzero. Each run matches the direct native call bitwise, has zero padding, and produces
    the same 48 bytes as the other prefills.
  * The identity seed and the identity export give identical 48 bytes from different prefills.
  * Every prefill is preserved byte for byte on input-validation, seed-validation and native
    failures (27 cases). The fault build adds 8 injected exceptions, 20 rejected corruptions and
    8 schema mutations on both exports, for 243 cases in total.
* **Negative control.** The unchanged boundary test linked against the shim from `97e0ef2`
  fails in both the production and fault builds:
  `identity: padding bytes 36-39 = 145 252 219 6` (GCC 14.2, `-O2`). The bytes are unspecified,
  not a stable value. With the correction, the same tests pass.

**Qualification of `fa7303c`.**

Clean-clone campaign (`git clone --no-local`, same `campaign.sh` steps as above) of
`fa7303c75df65b21ebafef5daf5af1407464ccde`:

| OpenCV | Builds D/V/R | Checker, config, `sh -n`, diff | AUnit run/ok/fail/err | Task 016 oracle | Seeded oracle | Sanitizer step (ASan/UBSan/LSan) | R1 boundary: production / fault build | Smoke, `lk_synthetic` |
|---|---|---|---|---|---|---|---|---|
| 4.1.0 | 0/0/0 | 0/0/0/0 | 186/186/0/0 | 28 OK / 9 EXC | 20 OK / 15 EXC | rc 0, 0 findings | PASS (27 failures) / PASS (243 failures) | 0, 0 |
| 4.6.0 | 0/0/0 | 0/0/0/0 | 186/186/0/0 | 30 OK / 7 EXC | 20 OK / 15 EXC | rc 0, 0 findings | PASS (27) / PASS (243) | 0, 0 |
| 4.10.0 | 0/0/0 | 0/0/0/0 | 186/186/0/0 | 30 OK / 7 EXC | 20 OK / 15 EXC | rc 0, 0 findings | PASS (27) / PASS (243) | 0, 0 |
| 5.0.0 | 0/0/0 | 0/0/0/0 | 186/186/0/0 | 28 OK / 9 EXC | 19 OK / 16 EXC | rc 0, 0 findings | PASS (27) / PASS (243) | 0, 0 |

Each row passed with five success cases times three prefills. `ldd` showed the selected library
on every version. The seeded-oracle output files are byte-identical to the pre-R1 campaign of
`523b957` on every version, and the 4.6.0 output equals 4.10.0 after the version line. The
correction did not change numerical behavior. The production archive exports 23
`T opencv_video_*` symbols and 0 test symbols, and the checker still reports 23 exports and 186
AUnit registrations. The native boundary tests are built and run only by
`scripts/run_sanitizers.sh`, so the R1 checks ran instrumented.

No MemorySanitizer or Valgrind run was made (neither clang++ nor valgrind is installed on the
qualification host). The evidence is the explicit byte-level tests and the negative control.

**Platform limitations.** The tests prove zero padding for the shim build on each tested
toolchain (Linux GCC locally; Linux and macOS in hosted CI). The correction does not depend on
the compiler preserving padding through copies. Windows has not run the new code; it first runs
in the push-to-main Windows workflow.
