# Task 014 qualification — ECC six-parameter affine alignment

Starting `origin/main`: `e16a00cb1a725465bdf1259e1372423f63b9f4f1` (PR #13 merge; reviewed Task 013
head `7575054cbb0cdcd0ac987f93f12e2c0f2f102071` is an ancestor). Branch `feature/014-ecc-affine`.
Inventory at start: 19 production exports, 140 AUnit registrations, crate `0.1.0-dev`, Core pin
`4da9d35ea21e1b2efe96296243ea668b488c6326` in all three roots, no overlapping open PR.

## Task 013 post-merge CI (checked, not assumed)

| Run | Workflow | Head SHA | Result |
| --- | --- | --- | --- |
| 38196079842 | Pinned OpenCV compatibility | `7575054` | success, 4.1.0 / 4.10.0 / 5.0.0 jobs all success |
| 38197543419 | Cross-platform Video | `e16a00c` | success: linux, repository-checks, linux-sanitizers, macos |
| 38197543379 | Windows post-merge Video | `e16a00c` | in progress when Task 014 started; later checked: **success** |

The Windows workflow keeps its push-to-main-only trigger. No baseline correction was needed.

## Source review

`modules/video/src/ecc.cpp` was fetched and read at 4.1.0, 4.6.0, 4.10.0 and 5.0.0 (4.6.0 and
4.10.0 are identical). The affine Jacobian has six image-width blocks, and the affine update
mapping `0->M00, 1->M10, 2->M01, 3->M11, 4->M02, 5->M12` is identical in all four versions. Details,
including the absent small-`lambda_d` and singular-Hessian guards, are in
[`ecc-affine-source-contract.md`](ecc-affine-source-contract.md). `Mat::inv()` of a singular 6x6
Float32 matrix was measured on 4.10.0 only: it returns zeros without throwing. No ECC undefined
behavior or new safety finding was identified.

## Direct-native research and oracle

`tests/cpp/ecc_affine_probe.cpp` (research, committed) and `tests/cpp/ecc_affine_oracle.cpp` link
only OpenCV. The oracle records 32 fixtures; **27 succeed and 5 raise** on every version
(`reflect-x`, `shear-0.2`, `strong-1.2-0.8`, `constant`, `noise-vs-noise`). The analytic
fixture `affine_texture` evaluates `signal(A^-1 (p - t))` with an explicit 2x2 inverse; Ada
reproduces it independently and the checksums of every template and moving image must match.

Selected 4.10.0 results (six coefficients, row-major; truth in parentheses):

| Fixture | M00 | M01 | M02 | M10 | M11 | M12 | rho |
| --- | --- | --- | --- | --- | --- | --- | --- |
| translation (1,0,2,0,1,1) | 0.99978 | 0.00005 | 2.0201 | 0.0000 | 0.99985 | 1.0142 | 0.9997 |
| combined (1.01,.012,2,-.008,.99,1) | 1.00968 | 0.01200 | 2.0268 | -0.00804 | 0.98993 | 1.0067 | 0.9960 |
| x shear 0.03 | 0.99977 | 0.03006 | 0.0174 | -0.00004 | 1.00017 | -0.0105 | 0.9970 |
| aniso 1.03/0.97 | 1.03002 | -0.00002 | -0.0005 | 0.00009 | 0.96973 | 0.0072 | 0.9998 |

Tolerances in AUnit are 0.003 for linear terms and 0.1 pixel for translation; they were chosen
from these measurements and not loosened to hide placement errors. The oracle comparison uses
1e-6 (linear), 1e-5 (translation, determinant) and 1e-6 (correlation) against the same-build
result, with no cross-version equality claimed (coefficients differ in the 4th-5th decimal and
`noise-vs-texture` differs visibly across versions).

Affine versus Euclidean (4.10.0, fixture `[1.03 .03; -.02 .97]`): affine residual about 0.26,
Euclidean residual about 5.0, so the affine operation fits a transform the rigid model cannot.
The Euclidean model still returns a finite result there (correlation 0.946), so the AUnit assertion is
that its residual is larger and its correlation lower, not that it fails.

**Wrong-but-finite outcome:** `noise-vs-texture` succeeds with correlation 0.084-0.093, `M00` about
1.20, `M02` about -10.1..-10.4, determinant 1.30-1.32. Determinants of all 27 successes are
between 0.9598 and 1.3190. See the source contract for the determinant policy (none beyond finiteness).

## Memory (direct native probe)

`tests/cpp/ecc_affine_limits.cpp`, peak `VmHWM`, 50-iteration limit, shifted textured pair:

| Geometry | translation | Euclidean | affine |
| --- | --- | --- | --- |
| 1024 x 1024 | 84 MB | 104 MB | 101 MB |
| 4096 x 4096 (2**24) | 1.01 GB | 1.33 GB | 1.27 GB (4.10.0), 1.25 GB (4.1.0), 1.21 GB (5.0.0) |
| 1024 x 16384 | | 1.33 GB | 1.27 GB |
| 16384 x 1024 | | 1.34 GB | 1.27 GB |

The envelope is retained with no new affine-specific limit. 4.6.0 was not measured. No maximum-size
case is in CI.

## AUnit

153 registered = 140 retained + 13 new (`ECC_Affine_*`: Defaults, Identity_And_Translation,
Rotation_And_Scale, Shear_And_Combined, Versus_Euclidean, Inverse_Warp, Options_And_Repeatability,
Regions_Lifetime, Other_Models_Unchanged, Validation, Degenerate_And_Limits, Direct_Oracle,
Oracle_Integrity). `Oracle_Integrity` checks a missing file and then rewrites 20 tampered copies of the real oracle (empty,
truncated twice, wrong format version, wrong OpenCV version, wrong declared count, removed fixture
record, renamed fixture, two altered checksums, swapped M01/M10, M02/M12 and M00/M11, altered M02,
M12, correlation and determinant, success recorded as exception, exception recorded as success,
unknown outcome) and requires each to be rejected for its specific reason.

## Native boundary and faults

`tests/cpp/native_boundary.cpp` `run_ecc_affine` uses a fixed 32-byte sentinel and `memcmp`. It covers
nine real transforms bitwise against a direct call, repeatability, same-handle, shared-allocation
headers, role reversal, all argument and option failures (null template/moving/result, iterations,
epsilon, depth, channels, size, empty, below/above dimension, above area), four nonconvergent
inputs, a genuine non-contiguous Region pair, the eight exception injections (before and after
native execution), eleven nonfinite post-native corruptions (each coefficient and correlation) and
four injected **finite** non-rigid matrices (scale+shear, reflection, singular, extreme finite) that
must be published unchanged. Production and fault builds both passed. Test hooks are in the test
driver only; the production ABI has no hook.

## Mutation sensitivity

Eighteen mutants, each applied to a scratch clone of the committed code (nothing committed). Every
one compiled, ran the 153 AUnit tests against the pristine oracle files plus the native boundary
driver (production and fault builds), and was **killed** by a named failure.

| # | Mutant | Killed by |
| --- | --- | --- |
| 1 | `MOTION_AFFINE` -> `MOTION_EUCLIDEAN` | `ECC_Affine_Direct_Oracle`, `_Inverse_Warp`, boundary `identity` |
| 2 | return identity unconditionally | `_Identity_And_Translation`, `_Direct_Oracle`, `_Degenerate_And_Limits` |
| 3 | drop native ECC execution | `_Degenerate_And_Limits` (constant images accepted), `_Direct_Oracle` |
| 4 | swap M01/M10 | `_Direct_Oracle` (M01 differs), `_Inverse_Warp` |
| 5 | swap M02/M12 | `_Direct_Oracle`, `_Identity_And_Translation` |
| 6 | swap M00/M11 | `_Direct_Oracle`, `_Rotation_And_Scale` |
| 7 | negate translation M02 | `_Identity_And_Translation`, `_Direct_Oracle` |
| 8 | negate shear M01 | `_Shear_And_Combined`, `_Direct_Oracle` |
| 9 | force uniform scale (M11 = M00) | `_Rotation_And_Scale`, `_Direct_Oracle` |
| 10 | zero both shear coefficients | `_Shear_And_Combined`, `_Direct_Oracle` |
| 11 | negate the correlation | `_Direct_Oracle` (correlation differs) |
| 12 | ignore maximum iterations | `_Options_And_Repeatability`, `_Direct_Oracle` (`it1`) |
| 13 | ignore epsilon | `_Options_And_Repeatability`, `_Direct_Oracle` (`it100-e8`) |
| 14 | reverse template and moving | `_Identity_And_Translation`, `_Direct_Oracle`, boundary `translation` |
| 15 | publish output before validation | boundary "partially published" (fault build only) |
| 16 | validate 5 of 6 coefficients | boundary "nonfinite native output accepted" (fault build only) |
| 17 | drop correlation-finiteness check | boundary "nonfinite native output accepted" (fault build only) |
| 18 | suppress native nonconvergence | `_Degenerate_And_Limits`, `_Direct_Oracle` (`reflect-x`), boundary |

The per-test names for rows 1-14 and 18 are taken from the first three failure lines the driver
printed (`/tmp/t14/mutation-run.log`); the lists above are not exhaustive. Mutants 15-17 are
unreachable through ordinary inputs and are killed only by the test-only post-native corruption
mechanism in the boundary driver's fault build; AUnit returned rc=0 for them. Several killed mutants
also made `Oracle_Integrity` fail ("pristine affine oracle rejected"); that is a consequence of the
same defect, not independent evidence. No mutant survived or failed to compile. No
equivalent-mutant claim is made. Mutation was run on OpenCV 4.10.0 only.

## Per-version campaign

Each row: an independent `git clone --no-local` of a committed head, then `scripts/test.sh` (builds,
generates every oracle, runs AUnit, the native boundary, fault and corruption binary),
`scripts/run_sanitizers.sh` (ASan + UBSan + leak detection over the Video shim, Core shim,
boundary, fault build and all direct oracles including the affine oracle) and the retained
`lk_synthetic` example. The configured `Native_Version` and the `ldd`-resolved library were checked.

| OpenCV | Loaded library | AUnit reg / run / passed / fail / err | test.sh | ASan+UBSan+leak | Findings | Example |
| --- | --- | --- | --- | --- | --- | --- |
| 4.1.0 | pinned prefix `install-4.1.0/lib/libopencv_video.so.4.1` | 153 / 153 / 153 / 0 / 0 | exit 0 | exit 0 | 0 | exit 0 |
| 4.6.0 | Debian 12 container `/usr/lib/x86_64-linux-gnu/libopencv_video.so.406` | 153 / 153 / 153 / 0 / 0 | exit 0 | exit 0 | 0 | exit 0 |
| 4.10.0 | system `libopencv_video.so.410` | 153 / 153 / 153 / 0 / 0 | exit 0 | exit 0 | 0 | exit 0 |
| 5.0.0 | pinned prefix `install-5.0.0/lib/libopencv_video.so.500` (`opencv5.pc`) | 153 / 153 / 153 / 0 / 0 | exit 0 | exit 0 | 0 | exit 0 |

The 4.1.0, 4.6.0 and 5.0.0 rows ran on clones of `cf2c8a5`; the 4.10.0 row ran on a clone of
`e0e9f72`. Later commits are documentation only. The sanitizers instrument the binding, shim and
test code; the OpenCV libraries are prebuilt and uninstrumented, so a clean run does not show
OpenCV is UB-free, and there is no ECC-specific waiver. The separately accepted Farneback upstream
pointer-arithmetic finding is unchanged and does not extend to ECC. Each oracle was compared with
its own version's native output; the affine oracle has 27 successes and 5 raises on all four
versions, and no cross-version numeric equality is claimed.

## Other gates

Repository checker (20 ABI declarations/imports, 153 registrations, Core pin in all three roots),
12 configuration tests, C11 compilation of the header with `-Wall -Wextra -Wpedantic -Werror`
(`_Static_assert` on size 32 and all seven offsets) and a C++17 compile, shell syntax of every
script, and `git diff --check`. Development, Validation and Release `alr build` and the tests
project under Validation exited 0 with zero warning lines (4.10.0 only).

## Not claimed

* Real Windows behaviour of Task 014 code (Windows is push-to-main only).
* That any OpenCV version is ASan/UBSan-clean internally.
* Mutation results on 4.1.0, 4.6.0 or 5.0.0.
* Memory at 2**24 pixels on 4.6.0 (not measured).
* That a returned matrix is a correct registration, or the behaviour of arbitrary large transforms.
* Reflection recovery: identity-initialized ECC raises on it on every qualified version.

