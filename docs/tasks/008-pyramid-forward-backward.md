# Task 008 — Forward/backward consistency with owned PyrLK pyramids

Start from fetched main `4e24501ff6ec5967ad930a9a9e0d9fa596dfe27d`, Task 007
PR #7 merge, reviewed head `bcbbf2d1bf8d070e2fa2a62a8b8869a5ad7b7671`.
Branch `feature/008-pyramid-forward-backward`. Keep Core pin
`4da9d35ea21e1b2efe96296243ea668b488c6326` in all three roots and crate version
0.1.0-dev unchanged.

Add two Track_PyrLK_Forward_Backward overloads accepting Previous_Pyramid,
Next_Pyramid, Points, defaulted Forward_Backward_Options, and (seeded only)
required Initial_Next_Points after Options. Retain existing types and raw APIs.
Compose the existing qualified tracking paths; no production C ABI change.

Forward uses flags 0/4. Compact only successes with saved original indices.
Backward reverses the same owned objects, uses forward positions as sources,
and original positions as destination predictions with flag 4. Share the Task
003 mapping algorithm; reuse Round_Trip_Distance and its checked Float64
arithmetic/Float32 publication. Preserve Points'Range including Positive'Last.
Failures expose original recovery/zero distance/false backward and consistency;
both-success distance compares inclusively to the finite nonnegative threshold.
Preserve the complete Forward record and photometric error.

Retain pyramid compatibility and coordinate/count checks, including empty input,
different seed bounds and natural available-depth truncation. Skip backward
native work for no survivors. Do not hide backward validation exceptions or
publish partial diagnostics. Preserve ownership and source/Region independence.

Retain all 73 Task 001–007 registrations; exercise good (2,1), seeded (12,7),
real inconsistency, backward failure, mixed mapping 7..11, single survivor,
empty/all-failed/all-success, thresholds/equality/zero, extremes, invalid inputs,
raw/prebuilt equivalence, lifetime/reuse/reversal and quality regressions.
Extend the independent direct-C++ oracle with derivative-interleaved vectors,
exact existing build settings and seeded backward originals. Preserve protocols.
An isolated flag-4-disabled mutation must fail a backward-distinguishing test.

Run repository/configuration/C11/shell/diff checks, all public profiles, Validation
AUnit, independent clean clone, native boundary/faults, ASan/UBSan/leaks, direct
oracle and retained example. Qualify actual configured/loaded OpenCV 4.1.0,
4.10.0 and 5.0.0, without system fallback. Preserve KleidiCV quality-definedness
restriction and CI topology. Record completed and pending evidence accurately.

Push normal commits, open one non-draft PR against main, disable auto-merge,
run final-head required and manual pinned CI. Retrieve deterministic failure
logs, never blindly rerun or sleep-poll. Stop open/unmerged with clean tree and
local/remote/PR-head equality, or precise infrastructure-pending evidence.
No merge/tag/release/amend/history rebase/force-push, confidence fusion,
application policy, new dependencies, raw pointers/flags or concurrency claims.