# Task 006 qualification

## Baseline and Windows gate

Fetched main exactly `d17bcd95e10e1e07ca2f2254f10a781e4da26fe4`, Task 005
PR #5 merge. Reviewed head `9e68a90ff96bf1f56337224201965b25c3d28243` is an
ancestor. Clean worktree, no overlapping open PRs, unchanged three Core pins
`4da9d35ea21e1b2efe96296243ea668b488c6326`, crate 0.1.0-dev verified.
Branch created from fetched main: feature/006-seeded-pyramid-tracking.

Task 005 [Windows run 37708960148](https://github.com/zackboll/opencv_video_ada/actions/runs/37708960148)
completed SUCCESS on that merge SHA. Exact logs retrieved: MSYS2 OpenCV 5.0.0,
59 registered/executed/passed, zero assertion failures/unexpected errors,
independent oracle including owned-pyramid metadata/equivalence and retained
8 ordinary + 4 seeded + 20 composed + 27 quality entries. Video DLL inspection
reports libopencv_video-500.dll and libopencv_core_shim.dll. The Task 005 MinGW
cstdlib/include_next correction is effective: oracle compilation/execution
succeeds. No additional Windows harness defect found. Windows did NOT execute
the Linux actual-shim native/fault/sanitizer campaign; no Windows ASan/UBSan/leak
claim is made. No Windows PR/manual workflow added or triggered.

## Implementation and source review

One public owned-pyramid seeded overload, Options before required predictions;
one new semantic export opencv_video_track_pyr_lk_pyramids_seeded. Twelve total
production exports/imports, no fault hook. Existing APIs and Core ownership/pins
unchanged. Ada checks the pyramid seeded path before raw seeded dispatch and
rejects unsupported private combinations. Native vector invocation uses exactly
OPTFLOW_USE_INITIAL_FLOW = 4, with no quality flag. Private seed clone, private
results, complete validation, failed normalization and atomic publication remain.

See source contract Task 006 section for tagged 4.1/4.10/5.0 declaration,
implementation and test review: interleaved derivative recognition, continuous
Float32 points/seeds, effective depth clamping, coarsest seed scaling/finer
refinement, stored previous derivatives and 5.0 pre-HAL scaling. No new image
bounds/displacement restriction, bitwise equality or general thread safety.
Task 004 KleidiCV quality definedness limitation is untouched.

## Local qualification

Linux x86_64, Alire 2.1.1, GNAT 16.1.0, gprbuild 26.0.1 package,
system g++ 14.2.0. Complete campaign on source-installed 4.1.0 and 5.0.0 using
isolated PKG_CONFIG_PATH/LD_LIBRARY_PATH, and system 4.10.0. Exact configured and
executed native versions checked; every boundary/oracle reports the selected tag.

Each version: 65 registered/executed/passed, zero failed assertions/unexpected
errors. Original 59 retained, six additional multi-case campaigns. Identity,
(2,1), and distinguishing 96x96 (12,7), 21x21 level-zero fixture pass. Predictions
P+(12.25,6.75) refine to within .05 per coordinate, differ from predictions by
>.20 in X, while unseeded misses by >5 in X or fails. Raw seeded/prebuilt seeded
status exact and normalized coordinates/photometric errors within 1e-5 on the
same build; no cross-version bitwise promise. External disabled-flag mutation
fails with `FAIL: seeded pyramid native oracle/reuse`; mutation never committed.

Different lower bounds, Points'Range, Positive'Last, matched empty arrays,
count/unsafe-coordinate/options/pyramid compatibility validation pass. Outside
previous/next predictions fail deterministically with previous point/zero error.
Source pixels are zeroed after build and all source objects finalized before use;
real strided Regions and parents also leave scope. Stable multiple-seed sequential
reuse, repeat first predictions and reverse-role use pass; no retained Core handle.
Initial reverse experiment using default multilevel tracking with an approximate
forward endpoint exceeded the .05 round-trip assertion; the role-reversal test
uses known integer translated points and level zero, isolating storage integrity
from multilevel interpolation/convergence. No tolerance was loosened.

Direct-only native oracle retains prior output protocols/cases, adds privately
owned flag-4 derivative-interleaved pyramid predictions, refinement and repeat
reuse checks. Actual shim boundary compares all success status/coordinates/errors
to independent direct flag-4 invocation within 1e-5 and proves source independence.
Nulls, bad seed depth/channels/shape/count/stride, nonfinite/unsafe coordinates,
input-input/header/output aliases, shared storage, geometry/window/depth/options,
empty arrays and failed normalization pass. Eight new before/after-native faults
(bad allocation, cv::Exception, standard exception, unknown exception) preserve
all three output headers/data; shared seed/output storage remains unchanged too.
All retained boundary/fault/build/create/destroy campaigns pass.

ASan+UBSan with detect_leaks=1/halt_on_error=1 instruments actual Video shim,
boundary and separate fault build; direct oracle instrumented too. Each version
exits zero with no reported binding-attributable leaks/UAF/double frees/invalid
access/UB/escaped exceptions. Upstream OpenCV and prebuilt Core are NOT claimed
fully sanitized. Retained synthetic example passes 4/4 per version.

Development, Validation and Release public builds pass; Validation full AUnit
65/65 with zero failures/errors. Repository checker (12 ABI/65 registrations),
eight configuration tests, C11 header warnings-as-errors, shell syntax and
git diff --check pass. No binding warning policy or tolerance weakened.

## Remote review gate

Independent clean clone of implementation commit
`816cd43117ed96ed3d39d4bf7b98aa01f95ae334`, no generated products copied,
reproduces 4.10.0 builds, 65/65 AUnit with zero failures/errors, full native/fault/
direct-oracle and ASan/UBSan/leak campaigns, retained 4/4 example, repository/
configuration/diff checks, all exit zero.

Final-head PR and pinned run evidence is recorded after completion below, with
exact final SHA/run URLs in the PR review-gate comment. Earlier runs are never
claimed to test later documentation commits.

PR [#6](https://github.com/zackboll/opencv_video_ada/pull/6) is open/non-draft,
unmerged, auto-merge disabled. Qualification head
`09830f7d9c3b61edbebb8ae1e81f661667a87701` passed
[required CI run 37709997315](https://github.com/zackboll/opencv_video_ada/actions/runs/37709997315):
repository-checks, linux, macos, linux-sanitizers. Exact job logs retrieved:
Linux OpenCV 4.6.0 and macOS ARM64 Homebrew 5.0.0 each pass 65/65, zero assertion
failures/unexpected errors, complete direct/native/fault campaigns and 4/4 example.
macOS Video/Core shim linkage verifies libc++ and absence of libstdc++ linkage;
GNAT toolchain emits a non-failing macOS deployment-version linker warning.
Linux actual-shim ASan/UBSan/leak campaign passes with no reported findings.

[Pinned run 37709995714](https://github.com/zackboll/opencv_video_ada/actions/runs/37709995714)
passes on that same head. Retrieved exact logs for independently source-built
4.1.0, 4.10.0 and 5.0.0: each passes public build, 65/65 AUnit, zero assertions/
errors, seeded-vector direct oracle, actual-shim boundary/faults, ASan/UBSan/leaks,
retained Task 001–005 regressions and 4/4 example. Selected native version checks
pass; no fallback to system version accepted. No deterministic CI failure or
blind rerun occurred. No unresolved Task 006 implementation issue found.

This evidence-only follow-up changes no code/tests/workflows. Required PR and
pinned workflows are verified again on its final pushed head; those final-head
run URLs and SHA are recorded in the PR review-gate comment. No merge/tag/release/
amend/published-history rebase/force-push/auto-merge occurred.