# Task 002 — Seeded PyrLK initial-flow tracking

## Baseline and review gate

Start from fetched main `20d220c2e9bd46cac3d6fe1918a896b181156312`, Task 001
PR #1 merge, reviewed head `210e284a93ec08f5fdd061370823124540b0d972`.
Verify ancestry, clean worktree, no overlapping PR, unchanged Core pin
`4da9d35ea21e1b2efe96296243ea668b488c6326` in all Alire roots, and final
post-merge Windows result. Branch `feature/002-initial-flow-seeding`.
Open one non-draft PR; complete required final-head CI and the manual pinned
compatibility campaign. Stop open, unmerged, auto-merge disabled, with a clean
tree and local/remote/PR-head equality. Use normal commits/pushes only: no merge,
tag, release, version bump, amend, published-history rebase, or force-push.

## Final public contract

Retain the original unseeded overload and C export unchanged, flags=0. Add:

```ada
function Track_PyrLK
  (Previous_Image      : OpenCV.Core.Mat;
   Next_Image          : OpenCV.Core.Mat;
   Points              : Tracking_Point_Array;
   Options             : PyrLK_Options := (others => <>);
   Initial_Next_Points : Tracking_Point_Array) return Point_Track_Array;
```

The parameter order deliberately avoids ambiguous existing positional option
aggregates. Named `Initial_Next_Points => Predictions` permits default options.
This semantic overload uses **only** `cv::OPTFLOW_USE_INITIAL_FLOW`, not a generic
flags interface. Equal counts, iteration-order correspondence, differing lower
bounds allowed, return `Points'Range`. Coordinates are already Float32, finite
and absolute value <=2**29; count fits signed 32 bits. No image-membership or
displacement restriction. Existing image/options safety contract applies.
Empty pairs preserve the previous array's null range after image/options checks;
one empty/one nonempty or any mismatched count raises `OpenCV_Error`.

`Point_Track` is unchanged. Success contains original previous point, refined
next point, true status and validated finite nonnegative native photometric
error. Failure contains original previous point in both fields, false status,
and zero error. Predictions are convergence aids, not correctness guarantees.
No IMU, navigation, feature, or estimator dependency.

## Native ownership and atomicity

Add `opencv_video_track_pyr_lk_seeded` with an additional opaque Core input handle;
fixed-width integers and C scalar doubles only. Borrow all seven headers within
Core callbacks. Validate inputs/options/counts/aliasing; clone seeds into private
`computed_next`. Native may mutate that private storage, never the input seed or
caller-visible outputs. Validate schema and every success slot, normalize failed
slots without reading native undefined data, then publish all three headers.
Exceptions remain contained; outputs remain unchanged on failure. Output headers
must be distinct from inputs and each other; input/input aliases and distinct
headers sharing allocations are safe. Private seeds require continuous Nx1
CV_32FC2 (typed empty allowed); strided seeds are rejected. Image Regions remain
supported without flattening.

## Required evidence

- Inspect authoritative declarations, CPU implementation, and relevant tests
  for 4.1.0 / 4.10.0 / 5.0.0, not API prose alone. Document seeded validation,
  scaling, mutation, bounds, conversions, undefined failed outputs and differences
  in `docs/pyrlk-source-contract.md`.
- Retain all 16 Task 001 AUnit cases; add seeded identity, useful translation and
  seed-consumption proof, differing bounds, mismatch/empty/nonfinite/unsafe seeds,
  outside-seed/failed normalization, immutability/errors and strided Regions.
- Independent direct-C++ oracle invokes flag 4 without the Ada/shim route;
  actual shim comparison checks status and meaningful successful outputs.
- Actual Core/shim raw campaign: seed schema/count/stride/finite/bounds/null,
  header/allocation aliases, empty success, unchanged failures and exception faults.
- Actual Video shim ASan/UBSan success/failure campaigns with no findings/leaks/
  uncaught exceptions; do not claim upstream/prebuilt Core fully instrumented.
- Pinned version campaign: builds, full AUnit, boundary, oracle, faults,
  configured sanitizers, and retained `lk_synthetic`. No extra example needed:
  README and the distinguishing tests demonstrate the API without duplication.
- Repository/configuration/shell/diff checks, warnings-as-errors, clean clone and
  relevant build profiles. Record exact registered/executed/passed/failure/error
  counts and final-head CI URLs, not merely test presence.

## Scope and CI

No pyramids/buildOpticalFlowPyramid, minimum-eigenvalue error mode, ECC, Farneback,
DIS, Kalman, UMat, VideoIO, feature/Calib3D/navigation types, public native handles,
vendored Core bridge, or changed Core pin. PR/main jobs remain repository-checks,
Linux, macOS and Linux sanitizers. Windows remains push-to-main-only; pinned
compatibility remains manual-only. Diagnose deterministic CI failures from exact
logs and correct via normal follow-up commits, never blind reruns.