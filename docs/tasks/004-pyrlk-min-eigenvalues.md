# Task 004 — PyrLK minimum-eigenvalue trackability

Baseline fetched main `19c394f097582d8b2480938cc7b14dbbb9f39843`, Task 003
PR #3 merge, reviewed head `4807745eb0a806c3e4f426542c9e81d3c71959de`.
Branch `feature/004-pyrlk-min-eigenvalues`; Core pin remains
`4da9d35ea21e1b2efe96296243ea668b488c6326` in all three Alire roots.

Add distinct `Trackability_Track`, `Trackability_Track_Array`, and unseeded/seeded
`Track_PyrLK_Trackability` overloads. Keep defaulted Options before required seeds.
Never change `Point_Track` or the photometric meaning of its Error, the existing
0/4 exports, or forward/backward composition. Expose local previous-patch minimum
eigenvalue independently of status; every failed next point is the previous point.
No raw flags word, optional backend, estimator policy or extra dependency.

New semantic C exports use exactly 8/12. Keep Core callback borrowing, private seed
clones/results, alias validation, bounded diagnostics, exception containment and
failure-atomic publication. Preallocate NaN-initialized quality, verify native
storage reuse, reject every unwritten/nonfinite/negative slot independently of
status; never clamp or silently change status. No production fault hook.

Review declarations, CPU source, dispatch/HAL, tests and Mat allocation in upstream
4.1.0 / 4.10.0 / 5.0.0. Document formula, normalization, computation timing,
threshold and distinct previous/next failure paths, output-definition limits,
roundoff and inherited arithmetic restrictions in the source contract.

Retain 37 tests; add focused quality, seeded (12,7), corner/edge/flat, robust
threshold, failure, metric distinction, bounds/empty/validation/Region/immutability
coverage. Independent direct OpenCV flag-8/12 oracle must compare meaningful
quality even on failure. Extend both raw exports through actual Core/shim boundary
and sanitizers; temporarily disable flag 8 outside the repository and prove tests
fail. Run all Alire roots serially, with warnings as errors, exact test counts,
example, configuration/C11/shell/diff checks, profiles and clean-clone campaign.

Determine Task 003 Windows run 37555765267 final result and investigate deterministic
regressions before claiming a healthy baseline. PR jobs remain repository-checks,
linux, macos, linux-sanitizers; Windows stays push-to-main-only, compatibility
manual-only. Create one non-draft PR; verify final-head CI, pinned compatibility,
clean tree and local/remote/PR-head equality. Stop open/unmerged with auto-merge
disabled. No main change, merge, amend, rebase published history, force-push, tag,
release or version bump. Exact results belong in the qualification record.