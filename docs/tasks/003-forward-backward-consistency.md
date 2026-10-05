# Task 003 — Forward/backward PyrLK consistency diagnostics

## Baseline and review gate

Start from fetched `origin/main` `de0a2872462a16f11c2f0eba4ecebce249db865c`,
Task 002 PR #2 merge, reviewed head `ad247113945bfe435a7e4252d252ec07f29407bd`.
Read agent/rule files, README, source contract, Task 001/002 and qualifications;
confirm ancestry, clean tree, no overlapping PR, and all three Core pins exactly
`4da9d35ea21e1b2efe96296243ea668b488c6326`. Inspect Task 002 post-merge Windows
through its final result; investigate deterministic defects. Create
`feature/003-forward-backward-consistency` from fetched main.

Use normal commits and pushes only. Stop with one open, non-draft, unmerged PR,
auto-merge disabled, completed final-head required CI and manual pinned
compatibility, local/remote/PR-head equality and clean worktree. No merge, tag,
release, amend, published-history rebase, force-push or branch deletion. Keep
crate version, Core pin and CI topology unchanged.

## Public API

Add `Forward_Backward_Options` with `Tracking : PyrLK_Options` and
`Maximum_Round_Trip_Error : Float32_Value := 1.0` (pixels), validated finite and
nonnegative, zero allowed, no arbitrary maximum/clamp. Add
`Forward_Backward_Track` containing `Forward : Point_Track`, `Backward_Tracked`,
`Recovered_Previous_Point`, `Round_Trip_Error`, `Consistent`, and its Positive-indexed
array type. Add both overloads:

```ada
function Track_PyrLK_Forward_Backward
  (Previous_Image : OpenCV.Core.Mat;
   Next_Image     : OpenCV.Core.Mat;
   Points         : Tracking_Point_Array;
   Options        : Forward_Backward_Options := (others => <>))
   return Forward_Backward_Track_Array;

function Track_PyrLK_Forward_Backward
  (Previous_Image      : OpenCV.Core.Mat;
   Next_Image          : OpenCV.Core.Mat;
   Points              : Tracking_Point_Array;
   Options             : Forward_Backward_Options := (others => <>);
   Initial_Next_Points : Tracking_Point_Array)
   return Forward_Backward_Track_Array;
```

Keep existing PyrLK declarations unchanged. Options precedes required seeds to
avoid positional aggregate ambiguity. Seeds correspond by iteration position,
with equal lengths but potentially different lower bounds; never mutate them.

## Composition and semantics

1. Call existing unseeded or seeded `Track_PyrLK` previous -> next.
2. Compact only forward successes, saving their original Ada indices.
3. Backward inputs are their forward locations; backward destination predictions
   are their **original previous-frame positions**. Call existing seeded PyrLK
   next -> previous; do not use the forward positions as destination predictions.
4. Map compact results back without filtering any input out. Preserve `Points'Range`.
5. When both passes succeed, compute Euclidean distance in Float64 and check
   before Float32 conversion. Raise `OpenCV_Error` for unrepresentable/invalid
   results, never saturate. `Consistent` iff both succeed and returned valid
   distance <= threshold.
6. On either failure use original recovery, zero distance, false backward status
   and consistency. Preserve the complete forward result.

`Forward.Error` is native successful mean patch L1 photometric error, not the
geometric pixel distance. A low round trip is evidence, not proof of the globally
correct physical correspondence. No estimator, IMU, covariance, feature detection,
Calib3D, navigation state, track IDs, RANSAC or implicit acceptance/filter policy.
Prefer pure Ada composition: no new native algorithm/C ABI/helper/container.

## Qualification

Retain all 22 original tests. Cover identity, small translation, seeded (12,7),
real forward-successful inconsistency, above/below/equal/zero thresholds, both
failure directions, mixed/interleaved outcomes, arbitrary/extreme input bounds,
differing seed bounds, empties/count mismatch, invalid/nonfinite thresholds where
representable, existing validation, strided Core Regions, image/point/seed
immutability and ordinary API equivalence. Use real Core Mats, deterministic
GUI/file/camera-independent textures. Prove compact mapping at original indices
7,9,11 separated by forward failures 8,10.

An independent direct-C++ OpenCV oracle must make both native calls, seeded
backward toward originals, and independently calculate distance. Compare status,
forward/recovered positions and round trip for mapped entries. Experimentally
establish inconsistent fixtures on **4.1.0, 4.10.0, 5.0.0** before fixing assertions;
redesign nonportable fixtures instead of loosening thresholds/per-version magic.
Use justified tolerances, not bitwise cross-version promises.

Run public builds, full AUnit (exact registered/executed/passed/failure/error counts),
retained example, direct oracle, entire existing production/seeded raw boundary,
exception faults, actual Video-shim ASan/UBSan, repository/configuration/C11-header/
shell/diff checks, independent clean clone and Development/Validation/Release
profiles. Keep warnings as errors and Alire roots serial. State if production
native code is unchanged; do not claim new ABI qualification or fully instrumented
upstream/Core. GNATprove optional; no proof-specific helper needed without a
concrete property to prove.

PR/main remains repository-checks, linux, macos, linux-sanitizers. Windows remains
push-to-main-only; pinned compatibility remains manual-only. Diagnose deterministic
CI failures from exact logs, not blind reruns. Record all provenance, commit SHAs,
PR/head/state evidence, native/CI results, arithmetic/seeding/fixture/mapping
contracts and unresolved issues in the qualification and final review handoff.