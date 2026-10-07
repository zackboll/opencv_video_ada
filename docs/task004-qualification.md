# Task 004 qualification record

## Baseline and scope

Fetched `origin/main` exactly `19c394f097582d8b2480938cc7b14dbbb9f39843`.
Task 003 reviewed head `4807745eb0a806c3e4f426542c9e81d3c71959de` is an ancestor.
Clean worktree, no open/overlapping PR, and all three exact Core pins
`4da9d35ea21e1b2efe96296243ea668b488c6326` verified before creating
`feature/004-pyrlk-min-eigenvalues` from fetched main. Version stays 0.1.0-dev.

Task 003 post-merge [Windows run 37555765267](https://github.com/zackboll/opencv_video_ada/actions/runs/37555765267)
finished **failure** on that exact main SHA. Exact failed logs retrieved: public
and AUnit builds succeed, then `run_forward_backward_oracle.sh` exits with
`error: missing OpenCV metadata` before AUnit execution. Its plain pkg-config
lookup omitted configure's MSYS2 cross-prefixed selection. Fixed the actual script
to respect PKG_CONFIG and prefer `x86_64-w64-mingw32-pkg-config` on MSYS2; an isolated
actual-script regression covers automatic selection and explicit override with
plain pkg-config deliberately unavailable. Eight configuration tests now pass.
This diagnoses/fixes a deterministic Task 003 harness regression; it is **not**
a successful baseline Windows run or Task 004 Windows qualification. Windows
remains push-to-main-only and cannot be rerun for this branch under that policy.

## Implementation and source evidence

See [Task 004 brief](tasks/004-pyrlk-min-eigenvalues.md) and
[source review](pyrlk-source-contract.md#minimum-eigenvalue-source-review-task-004).
Public separate Trackability_Track has Minimum_Eigenvalue, never Error; both
overloads preserve original bounds. Existing Point_Track, ordinary/seeded APIs,
photometric error semantics and forward/backward composition remain unchanged.
Ada uses private semantic-neutral transport records, not quality hidden inside a
public Point_Track. Two new C exports use 8/12, existing exports retain 0/4.

Private NaN-initialized Nx1 CV_32F quality storage, pointer reuse and schema checks,
all-slot finite/nonnegative validation before publication; failed next points
normalize but meaningful eigenvalues remain. No raw flags or public handle, no
vendored bridge, new dependency or production fault hook. Native exception and
alias/atomicity protections shared across all modes. Arbitrary vendor HALs are
not universally qualified: fallback source writes all slots, HAL prose alone
does not prove it, and sentinels reject undefined output. Tiny negative roundoff
is not clamped: any negative native output deliberately raises OpenCV_Error.

## Local evidence established so far

Debian Linux x86_64, Alire 2.1.1, GNAT/Alire C++ 16.1.0; system g++ 14.2.0;
gprbuild package 26.0.1 (reports 26.0.0). Warnings remain errors.

- OpenCV 4.10.0 full build/test: **51 registered, 51 executed, 51 passed,
  0 failed assertions, 0 unexpected errors**. All original 37 retained, 14 quality
  additions. Includes threshold rejection retaining positive quality, distinct
  unavailable previous/next patches, seeds/refinement, arbitrary/Positive'Last
  bounds, empty results, inherited validation, Regions and all input immutability.
- Independent direct-only OpenCV oracle: retained 8 ordinary + 4 seeded + 20
  forward/backward entries and 27 quality entries, flags 8/12; AUnit compares
  status/normalized next/quality within 1e-5. Raw actual-shim/Core campaign exercises
  both new exports through existing null/schema/options/coordinates/alias/storage/
  atomicity/empty/exception tests and direct failed-quality comparisons.
- Direct native experiments initially measured all three versions: corner
  0.73402756, edge zero, flat zero. Tests use corner >0.5 vs zero and half/twice
  measured E thresholds, with unchanged metrics on rejection. Texture values
  0.1515..0.5019; identity photometric Error zero versus positive quality rejects
  an accidentally missing flag. A 256-direction integer/fractional ramp sweep
  found no negative eigenvalues (not a universal roundoff proof).
- A 5.0 oracle-shape defect was diagnosed from exact AUnit mapping failure:
  vector-to-Mat construction changes dimensionality in 5.0. Explicit Nx1 shape
  and iteration by point count fix the harness, not assertions/tolerances.
- Temporary **external** shim copy disables flag 8: boundary exits **1** at the
  direct native comparison. Repository production flag remains intact.
- Packaged 4.10 actual Video-source ASan+UBSan production/fault/oracle campaign
  passes with detect_leaks=1/halt_on_error=1, no reported findings/leaks/uncaught
  exceptions. Upstream OpenCV and prebuilt Core are not claimed fully instrumented.
- Retained example 4/4 passes. Seven production exports, no test fault symbol;
  C11 header warnings-as-errors, repository check, eight configuration tests,
  all shell syntax checks and git diff --check pass.
- Public Development/Validation/Release builds pass; Validation AUnit reports
  51 executed/passed, zero failed assertions/unexpected errors.

Version campaign logs must be checked for actual configured/runtime versions,
not just script labels. Earlier temporary installations disappeared during the
campaign; mislabeled fallback-to-system runs are **not** compatibility evidence.
Stable independent source installations are being built for repeat qualification.
Remote final-head CI, pinned compatibility, profiles and clean clone are not yet
claimed complete in this record.