# Task 005 — Reusable owned PyrLK pyramids

Start from fetched main `5139efb24bfe95b1a5f561f79cd45ddb2dac7493`, Task 004 PR #4
merge, reviewed ancestor `e87c25e53b20a7ab622bbdb3c618981f841d7bdb`.
Branch `feature/005-reusable-pyrlk-pyramids`; all three Core pins remain
`4da9d35ea21e1b2efe96296243ea668b488c6326`. Preserve warnings-as-errors, versions,
CI topology and every Task 001–004 test, including the KleidiCV restriction.

## Contract

Add limited private finalized `PyrLK_Pyramid`, focused build options (window 21x21,
level 3; validated 3..255 and 0..30), `Build_PyrLK_Pyramid`, `Is_Empty`,
`Requested_Max_Level`, `Available_Max_Level`, `Build_Window_Size`, and only the
ordinary unseeded photometric `Track_PyrLK` overload taking two pyramids.
Metadata queries on empty objects raise OpenCV_Error; empty finalization is safe.
No public native pointers, Mat level extraction or derivative extraction.

Build nonempty 2-D UInt8 C1 images including non-contiguous Core Regions through
callback borrowing. Always derivatives=true, tryReuseInputImage=false,
pyrBorder=BORDER_REFLECT_101, derivBorder=BORDER_CONSTANT. Own all storage afterward;
source mutation/finalization and Region/parent scope exit must be directly tested.
Parent pixels may affect Region borders during construction, not its later lifetime.

Private native handle owns vector and metadata. Validate interleaved UInt8 C1 /
signed-16 C2 pairs, exact count 2*(available+1), pair geometry, rounded-up halving,
padding/ROI and actual level truncation before publication. Create clears *out,
uses RAII and contains all exceptions; null destroy is safe. No STL crosses C ABI.

Tracking uses stored vectors directly, flags=0, no rebuilding. Reject empty objects,
different geometry, unequal build/track windows and tracking request above either
requested build level. Effective depth is min(request, both available depths).
Preserve point range, finite-success photometric values and failed original/zero.
Sequential repeat and reversed-role reuse are required, not general concurrency.
No seeded/quality/forward-backward pyramid APIs, backend fixes or performance claims.

## Qualification and review gate

Retain 51 tests and add ownership, source/Region/parent lifetime, repeat/role reuse,
metadata (0/3/30, large/truncated images), raw equivalence, translation, normalization,
bounds, empty/invalid construction and compatibility tests. Independent oracle must
build vectors directly and invoke native tracking, never the shim. Actual Core/shim
boundary and separate test-only construction/tracking exception injection must
prove atomicity. ASan/UBSan/leak detection instruments actual Video source.

Run repository/configuration/C11/shell/diff checks, all public build profiles,
Validation AUnit, retained example, clean clone and complete 4.1.0/4.10.0/5.0.0
campaigns. Record exact counts/results; investigate Task 004 Windows run
37560308362 to final result and diagnose exact failure logs.

Push normal commits, open one non-draft PR against main with auto-merge disabled,
run manual-only pinned compatibility and final-head repository/linux/macos/Linux
sanitizer CI. Windows stays push-to-main-only. Stop open/unmerged with clean tree
and local/remote/PR-head equality. No merge/tag/release/amend/rebase/force-push.