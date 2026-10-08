# Task 006 — Seeded PyrLK with owned pyramids

Baseline: fetched main `d17bcd95e10e1e07ca2f2254f10a781e4da26fe4`, Task 005
PR #5, reviewed head `9e68a90ff96bf1f56337224201965b25c3d28243`.
Branch: `feature/006-seeded-pyramid-tracking`. Core pin and crate version unchanged.

Add exactly one public seeded Track_PyrLK overload for two owned pyramids and
one semantic C export opencv_video_track_pyr_lk_pyramids_seeded. Options precedes
required predictions. Preserve all previous APIs and photometric result semantics.
Correspondence is by iteration position, equal lengths, arbitrary differing bounds;
result preserves Points'Range. Failed results use previous point and zero error,
never the prediction. Seeds are validated then privately cloned before native LK.

Use native derivative-interleaved vectors with flags exactly 4. Reject null objects,
geometry/window mismatch and requests beyond either requested build depth, even
for empty points. Effective level is min(request, both available depths). Retain
finite coordinates within ±2**29, options/count/schema validation, exception
containment and failure-atomic three-output publication. Never mutate pyramid data.
No pyramid quality or forward/backward composition, public handles/flags/levels,
new dependencies, source retention, thread-safety promise or unrelated features.

Qualify authoritative 4.1.0/4.10.0/5.0.0 sources and all prior 59 tests. Add identity,
small and distinguishing (12,7) translations, refinement/consumption, raw/prebuilt
equivalence, bounds/empty/failure/validation, source/Region lifetime, sequential
reuse and role reversal. Extend direct-only oracle and actual Core/shim boundary,
including null/schema/coordinate/alias/shared-storage failures and eight new
before/after native exception cases. Run actual shim ASan/UBSan/leak checks,
profiles, examples, clean clone, repository/configuration/C11/shell/diff checks.
Flag-4-disabled external mutation must fail the distinguishing campaign.

Determine Task 005 post-merge Windows run 37708960148 outcome from exact logs;
do not add or trigger Windows PR/manual CI. Complete final-head required PR CI
and manual pinned compatibility. Leave one non-draft PR open/unmerged, auto-merge
disabled, clean tree and local/remote/PR-head equality. Normal commits/pushes only;
no merge/tag/release/amend/rebase/force-push/branch deletion.