# Task 005 qualification record

## Baseline and scope

Fetched origin/main exactly `5139efb24bfe95b1a5f561f79cd45ddb2dac7493`, Task 004
PR #4 `Add PyrLK minimum-eigenvalue track quality` merge. Reviewed head
`e87c25e53b20a7ab622bbdb3c618981f841d7bdb` is an ancestor. Clean worktree, no open
PR, and identical Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all three
Alire roots confirmed before branching `feature/005-reusable-pyrlk-pyramids`.
No version, dependency or CI-topology change.

Task 004 [Windows run 37560308362](https://github.com/zackboll/opencv_video_ada/actions/runs/37560308362)
finished **failure** on the exact merge SHA. Complete logs retrieved. MSYS2 OpenCV
**5.0.0**; public and AUnit builds succeeded, but the direct-only oracle failed
compilation in MinGW 16.2.0 cstdlib at `#include_next <stdlib.h>` (header not found).
No AUnit execution counts, oracle pass or DLL inspection pass can be claimed;
DLL inspection was skipped. The oracle reclassified all pkg-config include paths,
including the CRT root, as system paths, unlike the successful production build.
It now uses the configured OpenCV-only include directory on MSYS2 and clears
compiler-path contamination like the existing production shim build. The isolated
actual-script MSYS2/explicit-PKG_CONFIG regression enforces the selected include
path. This is a diagnosed harness correction, not successful Windows qualification;
Windows remains push-to-main-only, and no branch Windows run was triggered.

## Ownership/API/ABI

See the [brief](tasks/005-reusable-pyrlk-pyramids.md), README and
[source review](pyrlk-source-contract.md#owned-reusable-pyramids-task-005).
Public limited private PyrLK_Pyramid completes as Limited_Controlled, owns one
opaque native handle, finalizes null-safely and cannot legally be shallow-copied.
Build_PyrLK_Pyramid accepts Core Mat and focused window/level options; queries are
Is_Empty, Requested_Max_Level, Available_Max_Level, Build_Window_Size. Only ordinary
unseeded photometric Track_PyrLK gains a two-pyramid overload. Empty metadata raises
OpenCV_Error; no native handle/level/derivative extraction.

Opaque C forward declaration plus four exports: pyramid_create, pyramid_destroy,
pyramid_metadata, track_pyr_lk_pyramids (all prefixed opencv_video_). C/Ada imports
match all eleven total semantic exports; C11 header compiles warnings-as-errors.
No STL definition, exception or test hook appears in the production header/ABI.
Create clears *out before work, constructs with unique_ptr RAII and publishes only
after invariant validation. All build/track exceptions are contained. Destroy is
null-safe. Track computes into private Mats, validates and normalizes before atomic
publication, reusing the existing result safety path.

Build settings: derivatives=true; reuse input=false; pyramid BORDER_REFLECT_101;
derivative BORDER_CONSTANT. Native vector alternates CV_8UC1/CV_16SC2 image/dx-dy,
exactly 2*(available+1) entries. Pair geometry, rounded-up halving, owned allocations,
exact window padding/ROI and natural truncation are validated. Requested level is
distinct from available. Window equality, matching base geometry and tracking
request <= both requested levels are required; effective depth is min(request,
both available). Stored vectors go directly to native PyrLK, flags=0, no rebuilding.
No seeded/quality/composed pyramid overloads or general concurrency guarantee.
Derivative memory is a tradeoff; no universal speedup claim.

## Local evidence

Debian x86_64, Alire 2.1.1, GNAT/Alire C++ 16.1.0, gprbuild package 26.0.1
(reports 26.0.0). Warnings remain errors. Packaged OpenCV 4.10.0 and separate retained
source-built 4.1.0/5.0.0 installations with core/imgproc/video and OpenCL/IPP/TBB off.
Alire roots/campaigns run serially.

| OpenCV | build/AUnit registered/executed/passed | assertions/errors | native/fault/oracle | ASan/UBSan/leaks | example |
| --- | --- | --- | --- | --- | --- |
| 4.1.0 | pass, 59/59/59 | 0/0 | pass | no reported findings | 4/4 |
| 4.10.0 | pass, 59/59/59 | 0/0 | pass | no reported findings | 4/4 |
| 5.0.0 | pass, 59/59/59 | 0/0 | pass | no reported findings | 4/4 |

All 51 existing Task 001–004 tests unchanged; eight additions group focused checks
for metadata/level0/3/30/truncation, raw-prebuilt comparison and translation, failed
normalization, nondefault/Positive'Last/empty bounds, source mutation, finalized
source and strided Region/parent lifetime, repeated/reversed-role reuse, geometry/
window/requested-depth/default-object rejection and build image/options validation.
Window 21 level fixtures on all three versions: square 32/64/96/256 requested 3 or
30 returns 0/1/2/3, with vector lengths 2/4/6/8. Requested 0 always returns 0.

Independent direct-native oracle builds vectors with exact settings and compares
raw/prebuilt status and meaningful coordinates/errors within 1e-5. Actual shim/Core
boundary separately compares identical native vectors, normalizes failed points,
mutates/finalizes sources, repeats tracking and exercises null/alias/compatibility/
invalid construction and repeated 100 create/destroy scopes. No bitwise or universal
cross-version equality promise. Existing direct ordinary/seeded/forward-backward/
quality oracle scenarios and KleidiCV definedness restriction remain intact.

Separate test-only faults cover before/after build and before/after native track:
bad_alloc, cv::Exception, standard exception and unknown exception at each stage;
16 new injected cases require null create outputs and unchanged three track outputs.
All prior fault cases retained. Production nm shows eleven C exports, no fault hook.
Linux ASan+UBSan instruments actual Video source, boundary, separate faults and direct
oracle with detect_leaks=1/halt_on_error=1. Zero reported binding-attributable leaks,
UAF/double free/access/UB/uncaught exceptions; upstream/OpenCV/prebuilt Core are not
claimed fully instrumented. Normal Ada scope-exit destruction is additionally
qualified by lifetime tests and the limited controlled ownership representation.

Repository checker, eight actual-script configuration tests, shell syntax,
git diff --check and C11 header warnings-as-errors pass. Public Development,
Validation, Release and Validation AUnit qualification are recorded with the remote
and clean-clone results below once complete.

## Remote and independent clean-clone evidence

Pending completion; no PR/CI or clean-clone pass is claimed by this initial record.
The review gate requires all final-head PR jobs and the manual pinned matrix,
open/non-draft/unmerged PR, auto-merge disabled, clean tree and head equality.