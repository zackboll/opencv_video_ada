# Task 008 qualification — owned-pyramid forward/backward consistency

## Baseline

Fetched main exactly `4e24501ff6ec5967ad930a9a9e0d9fa596dfe27d`, Task 007
PR #7 merge. Reviewed head `bcbbf2d1bf8d070e2fa2a62a8b8869a5ad7b7671` is
an ancestor. Clean starting worktree and no open/overlapping PR verified before
branching `feature/008-pyramid-forward-backward`. All three Core pins remain
`4da9d35ea21e1b2efe96296243ea668b488c6326`; crate version 0.1.0-dev,
dependencies, production C++/header/imports and CI workflows are unchanged.

Task 007 post-merge [cross-platform 37717119922](https://github.com/zackboll/opencv_video_ada/actions/runs/37717119922)
completed successfully: repository-checks/Linux/macOS/Linux sanitizers all pass.
[Windows 37717119885](https://github.com/zackboll/opencv_video_ada/actions/runs/37717119885)
completed successfully with actual OpenCV 5.0.0, 73 passed/0 assertions/0 errors.
Exact completed logs were retrieved. [Pinned 37716653612](https://github.com/zackboll/opencv_video_ada/actions/runs/37716653612)
4.1.0 and 4.10.0 passed; at last successful query 5.0.0 was in progress in
“Install build tools and external-package metadata”. This is pending, not passed.
Later REST Actions queries returned HTTP 403 rate-limit exceeded for user 4860920;
this is infrastructure, not an implementation failure. No blind rerun or polling
loop was used.

## API and implementation

```ada
function Track_PyrLK_Forward_Backward
  (Previous_Pyramid : PyrLK_Pyramid;
   Next_Pyramid     : PyrLK_Pyramid;
   Points           : Tracking_Point_Array;
   Options          : Forward_Backward_Options := (others => <>))
   return Forward_Backward_Track_Array;

function Track_PyrLK_Forward_Backward
  (Previous_Pyramid    : PyrLK_Pyramid;
   Next_Pyramid        : PyrLK_Pyramid;
   Points              : Tracking_Point_Array;
   Options             : Forward_Backward_Options := (others => <>);
   Initial_Next_Points : Tracking_Point_Array)
   return Forward_Backward_Track_Array;
```

The existing Complete_Backward becomes a private generic with limited-private
source type and seeded-tracking formal. Image and pyramid instantiations share
one compaction/mapping/distance algorithm, with no allocation framework, unsafe
access values or persistent borrowed handles. Forward calls qualified pyramid
Track_PyrLK; backward reverses the same two objects and passes forward positions
as sources and compact original positions as destination predictions. Existing
native vector path uses flag 0/4 forward and flag 4 backward. It directly consumes
owned stored levels; neither leg rebuilds. Production ABI remains 14 exports.

Round_Trip_Distance is unchanged: Float64 subtraction/squares/sqrt, checked
finite/nonnegative/Float32 representability, then inclusive comparison of the
returned Float32 distance. Zero threshold remains legal. Forward record/error
is preserved completely. Unavailable recovery remains original/zero/false;
Count=0 skips native backward tracking. An exception prevents a returned partial
array. Existing option/geometry/window/requested-depth/coordinate/count validation
is inherited even for empty input, and naturally truncated available depth is
accepted. No Task 007 quality/KleidiCV restriction was changed.

## Local campaigns

Environment: Debian Linux x86_64, Alire 2.1.1, GNAT 16.1.0/GPRbuild 26.0.0.
System OpenCV 4.10.0; separately selected installations under
`/home/zboll/.cache/video-task004/install-4.1.0` and `install-5.0.0`.
Configured Native_Version and ldd-loaded Video/Core libraries are checked for
each version; neither pinned version falls back to system OpenCV.

Initial complete campaigns on all three versions passed **92 registered,
92 executed, 92 passed, 0 failed assertions, 0 unexpected errors**. All 73
baseline registrations remain and original Task 003 tests are byte-for-byte
unchanged. Nineteen focused new registrations cover identity, small/seeded large
translation, genuine inconsistency, backward failure, interleaved 7..11 mapping,
Positive'Last, empty/count checks, invalid threshold/options, Regions, direct
oracle, raw/prebuilt equivalence, captured-source lifetime/reuse, compatibility,
and natural truncation with exactly one survivor. All-forward-failed and
all-forward-successful cases are separate tests. Existing Task 001–007 native
boundary/fault/quality/lifetime campaigns are retained.

The independent direct OpenCV oracle builds derivative-interleaved vectors with
withDerivatives=true, tryReuseInputImage=false, REFLECT_101 image border and
CONSTANT derivative border. Four modes produce 20 mapped vector diagnostic
entries, alongside unchanged raw and quality protocols. AUnit compares forward
and backward statuses, locations/recovery/distance with 1e-5 tolerance. A follow-up
also compares forward photometric errors and each compact recovery against an
independent one-point seeded backward call; final requalification is recorded
below rather than attributing the initial results to these additions.

Measured local 4.10 vector oracle: (2,1) round trips at successful 7/9/11 are
0.00123339/0.00169813/0.0000137541 pixels. Seeded (12,7) round trips are
0.00184255/0.00140094/0.00197022. Unseeded level-zero (12,7), source (25,25),
reports both successes but 2.67040353 pixels, rejected at threshold 1. Blank
destination gives forward success/backward failure at all three interior entries,
with original recovery and zero distance. Outside sources at 8/10 fail.
Good-fixture tolerance 0.05 has a substantial margin; same-build 1e-5 comparisons
are not cross-version bitwise equality promises.

Threshold above/below/equal to measured distance and zero identity/motion tests
pass. Compact mapping preserves 7..11 and compares independent one-point results;
Positive'Last does not compute a source successor. Raw/prebuilt seeded/unseeded
comparisons include forward photometric error and complete diagnostic decisions.
Captured image/strided Region/parent objects are mutated then finalized before
diagnostics. The same two pyramids support repeated calls with differing arrays
and reversed roles; concurrent safety is not claimed.

Initial complete native actual-Core/shim boundaries, allocation/cv/standard/
unknown injected exceptions, direct oracle and ASan/UBSan/leak-detection campaigns
pass all three versions. No reported binding-attributable findings. Video source
is instrumented; prebuilt Core and upstream OpenCV are not claimed fully
instrumented. Retained synthetic example passes all three versions. Development,
Validation and Release builds, Validation AUnit, repository checker, eight
configuration tests, C11 warnings-as-errors header, shell syntax and diff checks
pass. No warnings policy was weakened.

## Final qualification and review gate

Final added oracle/mapping checks, isolated backward flag-4 mutation, independent
clean clone and hosted final-head results are recorded when completed. Pending
results are never counted as passes. No merge, tag, release, amend, published
history rebase, force-push or auto-merge is authorized or performed.