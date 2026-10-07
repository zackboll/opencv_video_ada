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

## Local qualification

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

The completed serial repeat campaign uses stable source-built 4.1.0 and 5.0.0
installations (core/imgproc/video, OpenCL/IPP/TBB disabled) and packaged 4.10.0.
Configured Native_Version and native oracle/boundary runtime versions were checked:

| OpenCV | build | AUnit registered/executed/passed | assertions/errors | boundary/faults | ASan/UBSan | oracle | example |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 4.1.0 | pass | 51 / 51 / 51 | 0 / 0 | pass | pass / pass | 27 quality + retained entries | 4/4 |
| 4.10.0 | pass | 51 / 51 / 51 | 0 / 0 | pass | pass / pass | 27 quality + retained entries | 4/4 |
| 5.0.0 | pass | 51 / 51 / 51 | 0 / 0 | pass | pass / pass | 27 quality + retained entries | 4/4 |

Earlier temporary installations disappeared during the first campaign; mislabeled
fallback-to-system runs are **excluded**, not compatibility evidence. Native source
build warnings are upstream; binding warnings remain errors. Clean independent Git
clone of `640cda40c26d429a62c05b2f8da302ba404cfdba`, no generated artifacts copied,
passes 4.10 public/tests, AUnit 51/51 with zero failures/errors, production/fault/
oracle/ASan/UBSan, 4/4 example, repository/configuration/shell/diff checks.

## Remote campaign (in progress)

PR [#4](https://github.com/zackboll/opencv_video_ada/pull/4) is open, non-draft,
unmerged, auto-merge disabled. Implementation head `640cda40c26d429a62c05b2f8da302ba404cfdba`.
[Initial PR run 37557392971](https://github.com/zackboll/opencv_video_ada/actions/runs/37557392971)
passes repository-checks, Linux OpenCV 4.6.0 (51/51, zero failures/errors, full
boundary/oracle/example), Linux sanitizers. macOS Homebrew 5.0.0 fails before AUnit
at direct-native `undefined native eigenvalue`. Exact complete and failed logs
retrieved; no blind rerun or scalar clamp. Diagnostic-only follow-up
`ccd21276f2fb9a6e466ba734fb67ddf1373687f1` reports mode/index/status/value to identify
the native backend violation. Its exact log establishes mode 3, point 1, status
false, **NaN sentinel retained** at previous (-1000,-1000). Verified exact Homebrew
5.0.0_5 bottle and its KleidiCV dispatch; reviewed pinned KleidiCV 26.03 source
which skips err on that path. Production correctly rejects it. Qualification now
records this specific undefined slot explicitly and requires whole-call OpenCV_Error
and failure atomicity, while still requiring native zero on CPU builds. No quality
is fabricated, no status changed, no tolerance weakened, no backend disabled.
This is a deliberate portability restriction: unavailable-previous quality calls
can raise on this backend, unlike the defined-zero fallback CPU result.

[Pinned compatibility run 37557391208](https://github.com/zackboll/opencv_video_ada/actions/runs/37557391208)
completed successfully on implementation head 640cda40: source-built 4.1.0, 4.10.0,
5.0.0 each reports 51 registered/executed/passed, zero failed assertions/errors,
full ordinary/seeded/quality boundary/faults, 27 quality plus retained native oracle
entries, actual Video ASan/UBSan and 4/4 example. Exact complete logs retrieved and
native versions checked. This earlier head is not claimed as a final-head run;
final-head cross-platform and repeat pinned checks remain required.