# Task 007 — Minimum-eigenvalue trackability with owned PyrLK pyramids

Baseline: fetched main `1a77c8403dc520828988a9a876fa781c0cf2f4fe`, Task 006
PR #6 merge, reviewed head `358f150d9d73a44851db2a284d492a6c3c2cf596`.
Branch `feature/007-pyramid-trackability`. Core remains pinned in all three roots
to `4da9d35ea21e1b2efe96296243ea668b488c6326`; version remains 0.1.0-dev.

Add two Track_PyrLK_Trackability overloads accepting owned Previous_Pyramid and
Next_Pyramid, Points, defaulted Options, and (seeded only) required
Initial_Next_Points after Options. Reuse Trackability_Track_Array. Preserve every
existing public type/API and photometric/forward-backward semantic contract.

Add exactly two semantic C exports:
opencv_video_track_pyr_lk_pyramids_min_eigenvalues and
opencv_video_track_pyr_lk_pyramids_seeded_min_eigenvalues. Native flags exactly
8/12, no public flags. Stored derivative-interleaved vectors go directly to LK,
without rebuilding or mutating either pyramid. Clone predictions privately.

Allocate continuous Nx1 CV_32F quality, initialize all slots to quiet NaN, retain
storage identity, require reuse, validate schema and every finite/nonnegative
slot independently of status before atomic publication. Never fabricate missing
quality. Failed next coordinates normalize to previous, but valid quality stays.
An unwritten quality slot raises OpenCV_Error for the complete call.

Retain compatibility, options, count, coordinate and alias validation, including
empty-array validation, arbitrary lower bounds and natural depth truncation.
Effective level is min(request, both available depths). Keep Core borrowing
callback-scoped and preserve owned source/Region/parent lifetime independence.

Review authoritative 4.1.0/4.10.0/5.0.0 headers, implementations and tests, including
vector dispatch and 5.0 KleidiCV definedness. Direct native oracle must separately
exercise derivative-interleaved vectors with NaN storage. Compare actual shim
status, successful points and defined quality, failed normalization and strict
undefined-output rejection. Qualify texture identity, corner/edge/flat, robust
thresholds, distinguishing (12,7) predictions (12.25,6.75), failed next searches,
unavailable previous patches, sequential reuse and reversed roles.

Retain all 65 prior AUnit registrations. Extend actual Core/shim boundaries and
separate before/after native allocation/cv/standard/unknown exception campaigns
for both modes. Perform external temporary mutations removing flag 8 from each
quality path and flag 4 from seeded quality. No mutations enter production.

Run repository/configuration/C11/shell/diff checks, warnings-as-errors profiles,
full AUnit, direct oracle, boundary/faults, actual Video ASan/UBSan/leaks, example,
independent clean clone and all three selected native versions. Record exact
results, backend differences and pending states in task007-qualification.md.

Use normal commits/pushes. Open one non-draft PR against main; auto-merge disabled.
Required final-head jobs remain repository-checks/linux/macos/linux-sanitizers;
Windows remains main-push-only and pinned compatibility manual-only. Retrieve
exact failure logs, correct deterministic defects, never blindly rerun or use
sleep/status polling loops. Stop open/unmerged at review, or report precise
infrastructure-pending status. No merge/tag/release/amend/published-history rebase/
force-push. No forward/backward pyramid composition, confidence fusion, navigation
threshold policy, new dependencies, or unrelated APIs.
