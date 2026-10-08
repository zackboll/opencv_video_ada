# Task 007 qualification — owned-pyramid trackability

## Baseline and provenance

Fetched main exactly `1a77c8403dc520828988a9a876fa781c0cf2f4fe`, Task 006 PR #6
merge, reviewed parent `358f150d9d73a44851db2a284d492a6c3c2cf596`. Clean tree,
no open/overlapping PR, unchanged three Core pins
`4da9d35ea21e1b2efe96296243ea668b488c6326` and version 0.1.0-dev verified before
branching `feature/007-pyramid-trackability` from fetched main.

Task 006 outstanding CI was initially pending, not called successful. It later
completed successfully; exact logs retrieved:

- [Post-merge Windows 37714770855](https://github.com/zackboll/opencv_video_ada/actions/runs/37714770855):
  MSYS2 OpenCV 5.0.0, 65 executed/passed, zero assertions/errors; retained oracle
  and Video/Core DLL checks pass. No Windows regression found. This is baseline
  evidence, not Task 007 Windows qualification or Windows sanitizers.
- [Post-merge cross-platform 37714770820](https://github.com/zackboll/opencv_video_ada/actions/runs/37714770820):
  repository/linux/macos/sanitizers all pass. Linux 4.6.0 and macOS ARM64 5.0.0,
  65/65, retained raw KleidiCV unavailable-previous NaN rejection confirmed.
- [Final-head pinned 37710881091](https://github.com/zackboll/opencv_video_ada/actions/runs/37710881091):
  4.1.0/4.10.0/5.0.0 all completed successfully, 65/65 and complete native/fault/
  oracle/sanitizer/example campaigns. The long 5.0 dependency-install delay
  cleared without rerun or sleep polling. Earlier 37709995714 passed the same
  implementation before Task 006's documentation-only final commit.

## Implementation and source contract

Two public overloads retain Trackability_Track_Array:

```ada
function Track_PyrLK_Trackability
  (Previous_Pyramid : PyrLK_Pyramid;
   Next_Pyramid     : PyrLK_Pyramid;
   Points           : Tracking_Point_Array;
   Options          : PyrLK_Options := (others => <>))
   return Trackability_Track_Array;

function Track_PyrLK_Trackability
  (Previous_Pyramid    : PyrLK_Pyramid;
   Next_Pyramid        : PyrLK_Pyramid;
   Points              : Tracking_Point_Array;
   Options             : PyrLK_Options := (others => <>);
   Initial_Next_Points : Tracking_Point_Array)
   return Trackability_Track_Array;
```

New semantic C exports opencv_video_track_pyr_lk_pyramids_min_eigenvalues and
opencv_video_track_pyr_lk_pyramids_seeded_min_eigenvalues use flags exactly **8/12**.
Inventory grows from 12 to 14; all prior signatures/types/routes remain. No raw
flags, new result type, public handle, dependency, version or CI topology change.
Ada pyramid-first dispatch selects quality explicitly, never the placeholder raw
Core images. Callback-scoped Core handles, private seed clones and atomic results
remain. Stored derivative-interleaved vectors pass directly, without rebuilding,
regenerating derivatives or mutating stored data.

See source-contract Task 007 section for authoritative tagged implementation,
declaration and test references. Previous derivatives are reused; available depths
clamp effective level. Quality is previous-patch LK normal-matrix minimum
eigenvalue normalized by window area, written before threshold rejection. Seeds
affect destination convergence, not conditioning. Failed next coordinates normalize
to originals while finite nonnegative quality survives status failure.

Existing continuous Nx1 CV_32F allocation initializes every quality slot to quiet
NaN, saves data identity, checks native reuse/schema and validates every slot
independently of status before publication. Missing/negative/nonfinite quality is
never replaced with zero. Independent native oracle also initializes NaNs and
checks storage reuse. CPU unavailable-previous patch writes zero; 5.0 vector
inputs still reach per-level HAL. KleidiCV 26.03 source skips err on unavailable
previous patches; actual macOS vector evidence is recorded at the remote milestone
below. Tests independently distinguish raw/vector definedness, require whole-call
OpenCV_Error for unwritten quality, and verify caller outputs remain unchanged.

## Local qualification

Linux x86_64, Alire 2.1.1, GNAT/Alire C++ 16.1.0, system C++ 14.2.0;
gprbuild package 26.0.1. Warnings remain errors. System OpenCV 4.10.0 and isolated
source-installed 4.1.0/5.0.0 use explicit PKG_CONFIG_PATH/LD_LIBRARY_PATH.
Configured version and loaded libraries are checked, not a system fallback.

The initial three-version campaign passed 72/72; an explicit seeded/unseeded
unavailable-previous backend test brings the final registration inventory to **73**.
All 65 original cases remain. Final rerun results are recorded before push.

New campaigns cover direct vector quality oracle, raw/pyramid identity and seeded
translation equivalence, metric distinction, corner/edge/flat and robust measured
half/twice thresholds, failed next quality, unavailable previous backend definedness,
seed independence, arbitrary/differing/Positive'Last bounds, empty arrays,
compatibility/options/count/unsafe seeds, natural truncation, lifetime, reuse and
reversed roles. Direct 27-entry vector oracle uses 1e-5 same-build scalar/coordinate
tolerance. Native boundary additionally repeats unavailable previous with flag 12.
No cross-version bitwise or concurrent-use guarantee is claimed.

Measured vector corner/edge/flat: **0.73402756 / 0 / 0**; textured identity quality
approximately **0.265748, 0.197183, 0.151508, 0.501894**, while photometric identity
error is zero. Threshold-rejected tracks retain eigenvalue. Next search outside
the frame retains valid positive previous quality and normalized original point.
(12,7), level zero, predictions (12.25,6.75) refine within .05 pixels, differ
from seeds by >.20 X; unseeded misses by >5 X or fails. Seed variation leaves
previous quality unchanged. Source Mats, strided Region views and parents finalize
after mutation before tracking; owned data remains usable. Repeated same-seed
results stable within 1e-5; alternate seeds and reversed frame roles pass.

Actual shim/Core boundary runs both new exports through null/schema/unsafe/finite/
window/depth/options/header-alias/shared-allocation/empty/failed-status checks.
Both new modes have eight before/after-native fault cases: bad_alloc,
cv::Exception, standard and unknown exceptions. Output header/storage snapshots
and shared seed/output allocations remain unchanged on failure. No production
fault export. Existing native/build/destroy/fault campaigns retained.

External temporary mutations (production source verified unchanged):

| mutation | exit | distinguishing failure |
| --- | --- | --- |
| unseeded pyramid quality bit 8 removed | 1 | quality differs from direct flag-8/12 oracle |
| seeded pyramid quality bit 8 removed | 1 | quality differs from direct flag-8/12 oracle |
| seeded pyramid quality bit 4 removed | 1 | next-point normalization/agreement fails |

Actual Video shim, boundary/fault drivers and independent oracle are instrumented
with ASan/UBSan, detect_leaks=1, halt_on_error=1. Initial all-version campaigns
exit zero with zero reported binding-attributable findings/leaks/escaped exceptions.
Prebuilt Core and upstream OpenCV are **not** claimed fully instrumented.
All public Development/Validation/Release builds pass; initial Validation AUnit
72/72. Eight configuration tests, repository checker, C11 header warnings-as-errors,
shell syntax and git diff --check pass. Production nm shows exactly 14 Video
exports, no fault hook. Retained example 4/4 on each version.

## Remote/clean-clone milestone

Final serial local campaign on **each** 4.1.0/4.10.0/5.0.0: public build,
**73 registered / 73 executed / 73 passed / 0 assertions / 0 errors**, direct
27-entry vector quality oracle plus every retained oracle, actual shim/Core raw
boundary and fault campaigns, ASan/UBSan/leak checks and retained 4/4 example all
pass. Configured versions and ldd-loaded version-specific Video libraries agree.
CPU builds write defined zero for unavailable previous quality in both new modes.
Final Development/Validation/Release builds and Validation 73/73 pass. Static,
eight configuration, C11 and shell/diff checks pass; nm inventory is 14.

Independent clean Git clone of implementation commit
`4a01bab638a6803f94cee9fb06601b68d57f34c4`, with no generated products copied,
reproduces 4.10.0 build, 73/73 (zero failures/errors), complete direct/native/fault/
sanitizer campaigns, retained 4/4 example and repository/configuration/shell/diff
checks, exit zero and clean clone worktree. A follow-up adds explicit Ada IEEE
special-seed validation (GNAT may raise Constraint_Error at its validity barrier,
as in prior tests); raw C++ directly rejects both coordinate components without
that barrier. Validation AUnit 73/73 and complete native campaigns were repeated.

Pushed qualification head `857635fd85e28904981cb77ee16a44ddf9d2a853` was then
requalified locally on all three actual versions: 73/73, zero failures/errors,
complete native/fault/oracle/ASan/UBSan/leak/example campaigns, exit zero. All
three external mutations still fail; production source unchanged. Independent
clean clone updated to this exact head also passes the complete 4.10.0 campaign,
all three public profiles, Validation AUnit 73/73 and `alr -n test` (73/73).
All 65 prior registrations, three public result records, four ordinary overloads,
two forward/backward overloads, twelve prior C signatures, manifests and workflows
were compared against main and are unchanged.

PR [#7](https://github.com/zackboll/opencv_video_ada/pull/7) opened non-draft against
main; auto-merge disabled. Initial-head [PR run 37716206535](https://github.com/zackboll/opencv_video_ada/actions/runs/37716206535)
and [manual pinned run 37716205562](https://github.com/zackboll/opencv_video_ada/actions/runs/37716205562)
were triggered on that head. At the local final-requalification milestone,
repository-checks passed, Linux/macOS were testing, Linux sanitizers instrumenting,
and all three pinned jobs were building native modules. These are pending, not
passes. Actual macOS vector evidence and later CI states are recorded in PR
milestone comments as they become available; completed runs are not attributed
to later evidence-only commits.

No pending
workflow is counted successful. Review gate requires open/non-draft/unmerged PR,
auto-merge disabled, clean tree and local/remote/PR-head equality. No merge, tag,
release, amend, published-history rebase or force-push is authorized or performed.

### Completed implementation-head required CI

[Cross-platform 37716206535](https://github.com/zackboll/opencv_video_ada/actions/runs/37716206535)
completed **SUCCESS** on `857635fd85e28904981cb77ee16a44ddf9d2a853`:
repository-checks, Linux, macOS and Linux sanitizers all pass. Exact completed job
logs retrieved. Linux OpenCV 4.6.0 and macOS ARM64 Homebrew 5.0.0 each report
**73 executed / 73 passed / 0 assertions / 0 errors**, complete direct vector
oracle, actual shim/Core boundary and exception campaigns, retained 4/4 example.
Linux ASan/UBSan/leaks has zero reported binding-attributable findings. macOS
linkage verifies Video/Core shims and libc++ without libstdc++.

Actual macOS direct oracle explicitly reports, at previous (-1000,-1000),
status false and eigenvalue **NaN sentinel retained**, with both `pyramids=0`
and **`pyramids=1`**. Precomputed derivatives do **not** bypass KleidiCV per-level
LK or cure its unwritten-quality path. Both new Ada modes require OpenCV_Error
for that complete call; actual boundary flag 8/12 cases reject it and preserve
pre-existing output headers/storage atomically. Valid patches, threshold rejection
and failed next searches still publish defined quality. No backend disabled,
missing value fabricated, status altered or tolerance weakened.

At this completed-PR milestone, pinned 4.1/4.10 are executing their test campaigns
and pinned 5.0 is building native modules; none is called a completed pass yet.
This evidence-only follow-up changes no implementation/tests/workflows. Final-head
required and pinned results are recorded in the PR review-gate comment, rather
than attributing earlier runs to the later documentation commit.
