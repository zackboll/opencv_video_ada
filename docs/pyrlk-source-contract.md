# PyrLK portable source contract (Tasks 001, 002, 004, 005 and 006)

## Seeded vector-pyramid source review (Task 006)

The authoritative tagged tracking.hpp declarations (4.1/4.10 lines 106–164,
5.0 lines 110–172) permit buildOpticalFlowPyramid inputs and define initial flow
as flag 4. Reviewed each tag's modules/video/src/lkpyramid.cpp and
modules/video/test/test_optflowpyrlk.cpp. The latter's legacy image/point test and
Mat-point regression do not independently prove seeded vector behavior; the new
direct and actual-shim experiments provide that evidence.

CPU calc: 4.1 lines 1229–1369, 4.10 1260–1400, 5.0 1051–1191. Both point
arrays require checkVector(2,CV_32F,true) and equal count; flag 4 suppresses native
next-point creation. STD_VECTOR_MAT recognition checks odd last index, derivative
channels twice image channels and signed derivative depth, selecting step 2.
Each available depth clamps maxLevel independently. The binding additionally
rejects requests beyond either requested build depth and mismatched windows.
Previous derivatives come directly from prevPyr[level*2+1]; they are not rebuilt.
Next derivatives are stored for later role reversal, not read in that direction.

Invoker seed scaling: 4.1 lines 197–207, 4.10 201–211, 5.0 133–143.
At effective coarsest level, predictions divide by 2**level; finer levels double
the preceding refined result. These operations depend on flags and effective
maxLevel, not whether inputs originated as images or vector pyramids. OpenCV 5.0
scales predictions before CALL_HAL(LKOpticalFlowLevel), passing the same stored
derivatives and already-scaled next points. HAL does not receive a separate
initial-flow switch. Fallback and HAL rounding prohibit cross-version bitwise
promises; the Task 004 KleidiCV quality definedness limitation remains unchanged.

Status and photometric L1 error follow the existing ordinary/seeded contract.
Failed native next/error are not promised meaningful. Video reads neither failed
value: it substitutes previous point and zero error, not the seed. Seeds need not
be inside image bounds; no maximum displacement is introduced. A private seed
clone and private status/error storage isolate all native mutation, then complete
schema/success-value validation precedes three-header publication. Flags are
exactly 4 for seeded pyramids; unsupported pyramid-quality routing is rejected.

## Evidence

Reviewed upstream tags, not just API prose:

| tag | peeled commit |
| --- | --- |
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` |

For each tag inspect these authoritative files (replace TAG in the URLs):

- https://github.com/opencv/opencv/blob/TAG/modules/video/src/lkpyramid.cpp
- https://github.com/opencv/opencv/blob/TAG/modules/video/include/opencv2/video/tracking.hpp
- https://github.com/opencv/opencv/blob/TAG/modules/video/test/test_optflowpyrlk.cpp
- https://github.com/opencv/opencv/blob/TAG/modules/core/src/matrix.cpp

Useful 4.10 anchors: `LKTrackerInvoker::operator()` lines 190–722;
`buildOpticalFlowPyramid` lines 726–821; `SparsePyrLKOpticalFlowImpl::calc`
lines 1250–1403. Equivalent 5.0 anchors: invoker lines 114–667, pyramid
669–764, calc 1038–1194. Equivalent 4.1 anchors: invoker 186–696,
pyramid 698 onward, calc 1218 onward. Upstream's accuracy test checks only
status-success slots, explicitly looks for NaN errors, uses external fixtures,
and accepts a bounded number of lost/bad points. It is not our synthetic oracle.

## Actual native contract

### Owned reusable pyramids (Task 005)

The same three upstream tags listed above implement `buildOpticalFlowPyramid`
in lkpyramid.cpp (4.1 starts at 698; 4.10 726–822; 5.0 669–764).
`modules/video/src/lkpyramid.hpp` defines `cv::detail::deriv_type` as **short**,
verified in all three source trees: native derivative depth is CV_16S, not Float32.
For our UInt8 C1 / derivatives=true subset the vector alternates CV_8UC1 image
and CV_16SC2 dx/dy, with exactly `2*(returned_max_level+1)` entries. Logical image
and derivative geometry agrees at each level. Next dimensions are `(size+1)/2`;
when either next dimension <= corresponding window dimension, the builder shrinks
the vector and returns the current level instead of the requested maximum.

Each member is an interior ROI of its own allocated padded Mat: offset equals
window width/height and whole geometry equals interior + twice the window.
The shim validates these exact construction invariants, nonempty 2-D/type/owned
allocation and expected truncation before storing the object and before tracking.
The same simple level fixtures with window 21 are source-derived: square 32/64/96/
256 requested 3 or 30 naturally returns 0/1/2/3; requested 0 returns 0 throughout.
Qualification records actual experiments, not just those source expectations.

`SparsePyrLKOpticalFlowImpl::calc` recognizes STD_VECTOR_MAT on both sides (4.10
1302–1356; corresponding branches in 4.1 and 5.0). Odd highest vector index,
twice image channel count and derivative depth select stride 2. It reduces maxLevel
to available depths. The level loop uses previous derivative entries directly
when stride 2 is detected; only the image-only path recomputes Scharr derivatives.
Next-side derivative entries need not be used. OpenCV copies Mat **headers** into
local vectors; the binding passes its stored vectors directly, without rebuilding
or deep-copying levels per call. The tracker checks ROI padding with locateROI.

Task 005 deliberately builds with derivatives=true, tryReuseInputImage=false,
BORDER_REFLECT_101 and BORDER_CONSTANT. Disabling input reuse bypasses the level-zero
borrowed-Region assignment and always allocates/copies padded storage. copyMakeBorder
may still consult the source parent when building level zero without BORDER_ISOLATED.
This preserves Region border semantics; completed levels no longer depend on the
borrowed Region/header/parent. Higher-level image and derivative borders use native
isolated handling internally; no binding border toggle is added.

The opaque owned object is **binding policy**, not a native OpenCV class. Requested
and available levels stay distinct. Exact build/track window equality and request
<= both requested depths are deliberately stricter compatibility policy; tracking
uses min(request, both available depths). Sequential read-only reuse is supported,
not a universal concurrency or speedup promise. Derivative memory is paid once per
frame so it can become a previous frame later. Seeded/quality/composed pyramid APIs
remain deferred. Ordinary raw APIs and the Task 004 HAL limitation are unchanged.

- The Mat CPU pyramid builder asserts `depth()==CV_8U`; the tracker asserts
  corresponding pyramid sizes and types match. It handles multiple channels
  through `cn`; UInt8 C1 is our deliberate, useful portable grayscale subset,
  **not** a claim that native accepts only C1. These paths use 2-D row/column
  geometry. Empty images with nonempty points eventually fail native operations;
  reject them explicitly in Ada and C++ instead of relying on an assertion.
- Pyramids use `copyMakeBorder`, `pyrDown`, `locateROI`, row pointers and strides.
  Non-contiguous Mat Regions are supported, not flattened by the wrapper.
  `tryReuseInputImage` can reuse a Region with sufficient surrounding padding;
  border construction can consult the parent allocation. Coordinates are local
  to the Region. Region tracking need not equal tracking an isolated clone near
  the Region boundary. The AUnit Region fixture checks real strided Core Mats.
- Previous points pass `checkVector(2,CV_32F,true)`. Native accepts suitable
  continuous vector shapes, including Nx1 two-channel and Nx2 single-channel.
  Without initial-flow flags, next points are created with the input size/type.
  The private Video ABI deliberately requires Nx1 CV_32FC2 to match marshaling.
  `OpenCV.Float32_Point` already provides binary32 X/Y Ada values. No redundant
  Video point type or record-layout assumption is needed: Core typed access
  marshals coordinates individually.
- Status is Nx1 CV_8U, initialized to one; zero means tracking failed. Err is
  Nx1 CV_32F. With flags zero, successful err is the sum of absolute interpolated
  patch differences divided by `32*width*channels*height`: mean L1 photometric
  difference, **not** displacement accuracy, confidence or squared error.
- **Failed err is not generally initialized.** The out-of-previous-patch branch
  sets it to zero, but low-eigenvalue/singular, out-of-next-patch and final-error
  bounds branches can skip it. Next-point storage may contain intermediate
  estimates. Never read either failed slot. Both shim and thick Ada deliberately
  normalize failure to previous point and zero error. This is an Ada contract,
  not a native guarantee.
- The successful CPU branch writes err at level zero. For bounded UInt8 C1
  patches and valid interpolation its finite nonnegative sum and positive
  denominator produce finite error. Native source offers no unconditional
  guarantee for arbitrary huge windows/coordinates or every vendor backend;
  the shim validates successful coordinates/error and raises on invalid output,
  rather than coercing it or marking it failed. No extra optional-error state is
  necessary for a successfully returned supported result.
- Points near and outside the nominal image are deliberately tested against
  padded patch bounds, not `0 <= x < width`. A point just outside may succeed;
  a distant point normally returns failure. There is **no image-bounds public
  precondition**. Extremely large finite floats do require a conversion-safety
  restriction because `cvFloor` converts them to signed integers.
- Native asserts both window dimensions >2 and maxLevel >=0; even windows are
  legal. The requested pyramid depth is reduced when the next level would be
  no larger than the window. It allocates `(maxLevel+1)` headers before that
  reduction, and the invoker uses `1 << level`.
- Native COUNT is clamped to [0,100], EPS to [0,10]; missing flags default to
  count=30/epsilon=.01. Epsilon is then squared for comparison with delta norm.
  Video always supplies COUNT|EPS and rejects outside its explicit ranges,
  avoiding silent clamping. `minEigThreshold` is converted double-to-float;
  Video rejects negative, nonfinite or float-overflowing thresholds.
- Native has a zero-count release-and-return branch after point schema and
  window/level validation, before image processing. An arbitrary empty Mat is
  not necessarily accepted by `checkVector`: all three implementations require
  non-null `data`, so ordinary empty Mats return -1 rather than reaching the
  nominal zero-count branch. The shim's explicit typed-empty handling is a
  deliberate thick-binding behavior, not a claim about native empty-Mat success.
  Ada handles empty point arrays
  before marshaling, but still validates images/options. The shim independently
  supports a typed empty CV_32FC2 Mat and clears all output headers on success.
- OpenCV uses assertions/errors that throw `cv::Exception`, plus standard
  allocation failures. Local temporary results are checked before publication.
  Errors are caught as OpenCV/standard/unknown status codes, with a bounded,
  allocation-free thread-local diagnostic buffer even during bad_alloc handling.

## Safety subset and version differences

Windows 3..255 keep patch area, `area()*3`, and `32*area()` comfortably inside
signed int and finite Float32 accumulation. Max_Level 0..30 bounds header counts
and signed shifts. Count 1..100 and epsilon (0,10] expose the useful positive
native criteria subset. Finite coordinates with absolute value <=2**29 leave
margin for window subtraction and pyramid scaling before integer conversion.
Padded rows*columns must fit `INT_MAX/4`; original strides must fit
`INT_MAX/padded_rows`. These conservative checks protect native signed-stride,
row-offset, derivative-buffer and padding paths, not just C argument conversions.
Large but valid images can still exhaust memory; that is an exception, not a
partial result. There is no claim of successful allocation for every allowed
size. Native-internal convergence and custom HAL implementations remain upstream
responsibilities; invalid successful results are not exposed as Ada values.

4.1 uses older SSE code and `calcSharrDeriv`; 4.10 uses universal intrinsics and
`calcScharrDeriv`, with equivalent public CPU schemas, failure and criteria
semantics. 5.0 adds Scharr/LK HAL dispatch and a scaled-points buffer, and removes
some obsolete paths. Those optimized paths may change rounding/iteration
results, so coordinate tolerances are used, not cross-version bitwise equality.
5.0 also changes `checkVector`'s 2-D condition to `dims <= 2`; the private Video
Nx1 2-D point schema remains unchanged and avoids depending on that difference.
The Core pin already supports 5.0's own Mat ABI differences; Video does not
duplicate Core headers. Mat inputs do not request UMat/OpenCL execution. A source
review does not establish version qualification; see the qualification record
and exact CI runs for native results.

## Ownership / failure atomicity

Core owns every opaque handle, Mat header and application allocation. Nested
`Module_Interop` callbacks keep six (unseeded) or seven (seeded) borrowed headers
alive for one call;
Video neither deletes nor retains any borrowed pointer. The C ABI contains only
opaque Core handles and fixed-width/scalar C types, never C++/STL objects.
Non-null invalid/freed pointers cannot be validated safely: callers must supply
real live Core handles. Output headers must be distinct from all inputs and
each other. Existing output contents remain **unchanged on failure**, rather
than being cleared; Ada initializes its three output Mats empty before work.
Clearing existing outputs on failure would contradict failure atomicity.
Only successful empty-point calls clear existing output headers.

Fault hooks are compiled exclusively into a separate native qualification
binary, not declared in the production C header or Ada imports. Tests throw
bad_alloc, cv::Exception and a nonstandard exception before the native call and
after native computation, checking containment and unchanged output headers.

## Initial-flow source review (Task 002)

The same peeled commits above were inspected again, including `tracking.hpp`,
`lkpyramid.cpp`, and `test_optflowpyrlk.cpp` for **each** tag. Authoritative anchors:

| tag | declaration / flag | CPU nextPts validation | initialization | seeded floor / update / final floor |
| --- | --- | --- | --- | --- |
| 4.1.0 | tracking.hpp 56, 138–158, 178–183 | lkpyramid.cpp 1223–1263 | 195–208 | 459–479 / 636–652 / 658–693 |
| 4.10.0 | tracking.hpp 56, 138–158, 178–183 | lkpyramid.cpp 1254–1294 | 199–212 | 489–509 / 664–680 / 686–721 |
| 5.0.0 | tracking.hpp 59, 146–166, 186–191 | lkpyramid.cpp 1045–1085 | 131–157 | 434–454 / 609–625 / 630–666 |

1. Native `OPTFLOW_USE_INITIAL_FLOW` is enum value **4** in all three tags. When
   set, `_nextPts.create` is skipped. Existing storage must pass
   `checkVector(2, CV_32F, true) == npoints`: continuous Float32 two-vectors of the
   same count. CPU source does **not** require identical Mat shape/type layout
   if both satisfy that vector check. Our private ABI deliberately requires the
   same continuous **N x 1 CV_32FC2** schema as previous-point marshaling and
   rejects strided seed Mats. Ada arrays do not require identical lower bounds.
2. At the actual coarsest pyramid level, native reads the full-resolution seed
   and multiplies it by `1/(1 << level)`. At each finer level it doubles the
   preceding refined result, rather than rereading the original seed. 5.0 moves
   this initialization into a loop before the HAL dispatch, using a separate
   scaled previous-point buffer; the CPU fallback has the same semantics.
3. Native writes the scaled next estimate **before** checking the previous patch,
   then writes iterative refinements. A later point, level, allocation, or backend
   failure can occur after earlier writes. Supplying a caller-visible output Mat
   or a shallow copy of the seed is therefore unsafe. Video clones the validated
   seed into private `computed_next`; status/error are also local. Headers publish
   only after schema and every success-slot check. Failed slots are overwritten
   without reading either native next point or native error.
4. The previous patch and seeded next patch are checked against padded bounds
   **after** half-window subtraction and `cvFloor`, not against the nominal image.
   A seed just outside the image may succeed; a distant seed normally produces
   status zero at level zero. It is not a precondition violation. A textureless
   previous patch can fail before the seeded patch is examined at all.
5. Input seeds undergo Float32 power-of-two scaling, finer-level multiplication
   by two, half-window subtraction, and signed-int `cvFloor` in the iteration and
   final photometric-error branches. `cvRound` acts on fractional interpolation
   weights, not on a previous-to-seed displacement. There is **no integer
   subtraction of seed and previous coordinates** in this Mat CPU path. The
   existing finite `abs(coordinate) <= 2**29` restriction leaves ample signed-int
   margin for initial scaling/window subtraction and unrefined scale restoration.
   Distant seeds fail padded bounds before interpolation. No additional caller
   displacement/delta bound is necessary. Iterative `delta` is computed internally
   from patch gradients/residuals just as in unseeded LK; bounding the initial
   displacement would not bound native convergence. Task 001's explicit limitation
   concerning native internal arithmetic/custom HAL remains, not a claim to prove
   arbitrary upstream/vendor implementations safe by post-validating outputs.
6. Failed `nextPts` can contain scaled/intermediate/predicted coordinates; even a
   finite original seed gives no dependable failed-result meaning. Failed `err`
   may still be unwritten on singular/low-eigenvalue/out-of-next-patch branches.
   Native test `accuracy` compares only success coordinates and permits bounded
   losses; the `submat` regression checks a Region invocation. None of these
   upstream tests is a seed-consumption oracle or defines failed output values.
7. COUNT/EPS normalization, window/level arithmetic, image/stride restrictions,
   and successful mean patch L1 error are unchanged by flag 4. The binding supplies
   no eigenvalue-error flag. Neither Float64 coordinate narrowing nor a raw public
   flag word is introduced. Point/seed counts are checked before integer conversion.
8. There is no material initial-flow contract difference across 4.1, 4.10 and 5.0.
   SSE/universal-intrinsics/HAL differences still justify numerical tolerances,
   not bitwise equality. Mat inputs do not select the UMat/OpenCL path.

### Distinguishing native experiment

Use the existing deterministic 96x96 texture
`(row*17 + col*29 + (row*col)%251)%256`, zero-filled integer translation `(12,7)`,
and points `(25,25), (45,32), (60,50), (35,65)`. Window 21x21, **Max_Level=0**,
COUNT|EPS `(30,.01)`, eigenvalue threshold `1e-4`. Supply predictions offset by
`(12.25,6.75)`, deliberately not exact matches. The correct patches are well inside
the images. Full-resolution local LK cannot traverse this large translation from
the previous point on this high-frequency texture; the prediction starts within
the intended convergence basin.

Direct-C++ experiments on 4.1.0, 4.10.0 and 5.0.0 show all four seeded solutions
within **0.002 pixels per coordinate** of `(12,7)`, while all four unseeded solutions
are more than **5 pixels** from the intended match. The test requires 0.05-pixel
Euclidean accuracy, at least 0.20-pixel refinement away from the supplied seed,
and >5-pixel unseeded separation (or native failure). The 0.05 bound is a subpixel
accuracy criterion with substantial measured margin, unchanged across versions;
it does not conceal convergence differences. This rejects both an ignored flag
and an implementation that merely returns the seed. AUnit checks the same fixture
and actual-shim output is compared with direct native status, successful coordinates
and errors within 1e-5 on identical inputs. Failed native errors are never compared.

## Minimum-eigenvalue source review (Task 004)

Reviewed the same three peeled tags above: declarations, the complete CPU invoker
and dispatch, relevant accuracy/submat tests, `matrix.cpp`, `matrix_wrap.cpp`, and
5.0 `modules/video/src/hal_replacement.hpp`. The upstream accuracy test passes
`noArray()` for errors and compares successful coordinates; the submat regression
only requires no exception. Neither proves initialization of flag-8 failed errors.

| tag | flag declaration | previous unavailable | eigenvalue / rejection | output allocation |
| --- | --- | --- | --- | --- |
| 4.1.0 | tracking.hpp 57 (=8) | lkpyramid.cpp 214–226 | 439–455 | 1257–1263 |
| 4.10.0 | tracking.hpp 57 (=8) | lkpyramid.cpp 218–230 | 469–485 | 1288–1294 |
| 5.0.0 | tracking.hpp 60 (=8) | lkpyramid.cpp 167–177 | 414–430 | 1079–1085 |

At each level, the invoker interpolates **previous** Scharr gradients and forms
the symmetric normal matrix with entries `A11`, `A12`, `A22` (accumulated gradient
products scaled by `FLT_SCALE = 1 / 2**20`). The exact native expression is:

```text
(A22 + A11 - sqrt((A11-A22)*(A11-A22) + 4*A12*A12))
  / (2 * window_width * window_height)
```

This is the smaller eigenvalue divided by window pixel count, with native
gradient/interpolation scaling, **not** a unitless probability or photometric
residual. It is computed and written before comparing `minEig < minEigThreshold`
or determinant `D < FLT_EPSILON`, and before any destination-patch iterations.
Flag 4 selects/scales the destination prediction only; it does not change the
previous gradients or this expression. Consequently seeded flag 12 and unseeded
flag 8 have the same previous-patch metric at a given level, even with very
different destination predictions/images. The last (level-zero) computation
determines the returned metric. Normal successful output is full-resolution
previous-patch conditioning, not quality of a matched next patch.

### CPU failure paths and backend limits

Every normal-return fallback-CPU point reaches one of two level-zero writes:

- previous patch outside the padded usable derivative region: status zero,
  error zero, then continue;
- evaluable previous patch: write computed eigenvalue, then possibly reject on
  threshold/determinant, or later fail the next-search bounds check. Those later
  failures do **not** clear the eigenvalue. The final photometric-error branch is
  disabled in flag-8 mode.

Therefore `Tracked=False` does not imply zero quality. A high eigenvalue can
coexist with failed next search; low-but-positive quality survives a high
threshold. Failed nextPts remain unspecified/intermediate and are never exposed.
Existing photometric exports still use 0/4 and discard failed errors exactly as
before. New semantic exports use exactly 8/12; there is no public flags word.

Mat inputs cannot select UMat/OpenCL; the OpenCL implementation also declines
flag 8. 4.1/4.10 OpenVX dispatch is explicitly disabled (`CV_OVX_RUN(false,...)`).
5.0 adds `CALL_HAL(LKOpticalFlowLevel,...)` before the CPU loop, passing the
minimum-eigenvalue Boolean, threshold, error buffer, scaled previous points,
destination predictions, and status only at level zero. The default HAL returns
`NOT_IMPLEMENTED`, falling back to the reviewed CPU loop. A custom HAL returning
OK bypasses it; other HAL error codes throw. HAL prose does not prove every
failed slot is written. There is **no universal all-backend initialization claim**.
The standard/source-built qualification exercises native fallback/optimized CPU
paths, not arbitrary vendor HALs. A vendor must honor flag-8 semantics; finite
but semantically wrong vendor results cannot be detected by value validation.

The 5.0 bundled Carotene HAL was also inspected after the first macOS failure:
`hal/carotene/hal/tegra_hal.hpp` 1937–1962 delegates LK when its configuration is
supported. `hal/carotene/src/opticalflow.cpp` 75–98 writes zero for unavailable
previous patches (at every level); 299–314 uses the same scaled expression, writes
it before threshold/determinant rejection; 324–335 retains it on next-search
failure. Status is optional at coarse levels; the quality Boolean is honored.
This bundled HAL's source does cover both normal quality writes, unlike an
arbitrary external implementation. ARM NEON accumulation and fused arithmetic
can still differ from x86, especially for rank-deficient patches. Source alone
does not establish runtime nonnegativity or select which Homebrew path executed.

**Observed material backend difference:** macOS ARM64 Homebrew OpenCV **5.0.0_5**
uses KleidiCV, not Carotene LK (Carotene's LK adapter is ARMv7-only). Downloaded
the exact arm64_sonoma bottle, verified SHA256
`1d0ac2865a6b50180fbf17b7aa3bdb284cec63baeb4021a059d6abc386cb3d00`, and inspected
its LK invoker: it calls `kleidicv::hal::standalone_lucas_kanade_alg_u8` at 0x22248.
OpenCV 5.0 pins KleidiCV **26.03**, archive MD5
`b85a745bfe0e87e67e30be9533eb6b24` (`hal/kleidicv/kleidicv.cmake`). Reviewed
`adapters/opencv/kleidicv_hal.h` 640–660 and
`kleidicv/src/analysis/standalone_lucas_kanade_alg_common.h` 118–152.
The latter sets status false and continues at 125–130 for an unavailable previous
patch **without writing err**. An evaluable patch writes eigenvalue at 144–145
before threshold rejection and next search. Exact macOS native experiment leaves
NaN at previous (-1000,-1000), status false, level zero. Thus the all-slot CPU
guarantee does not extend to this default ARM HAL; its affected calls raise
OpenCV_Error atomically under the new API. We do not fill missing quality with
zero, disable HAL globally, patch upstream, or mutate status. Direct oracle marks
this slot explicitly unavailable; Ada/raw tests require rejection of the entire
call. CPU implementations must still return defined zero. This is source-backed
backend-dependent **definedness**, not relaxed metric/coordinate tolerances.

### Defensive output allocation and validation

Video privately allocates continuous `N x 1 CV_32F` errors initialized to quiet
NaNs, saves its data pointer, then invokes native LK. All three CPU implementations
call `_err.create(N,1,CV_32F,-1,true)`. MAT `OutputArray::create` delegates to
`Mat::create` (4.1 matrix_wrap.cpp 1287–1313; 4.10 equivalent MAT branch), or
returns immediately on a matching shape/type (5.0 matrix_wrap.cpp 1605–1632).
`Mat::create` retains matching allocated storage: 4.1 matrix.cpp 318–333,
4.10 659–676, 5.0 1085–1103. Thus the preinitialized slots survive allocation.
Native experiments additionally verify pointer reuse. Video rejects replacement
storage rather than assuming its new contents were initialized, validates schema,
and requires **every** slot finite and nonnegative, regardless of status. An
unwritten sentinel, NaN, infinity or negative value raises `OpenCV_Error` before
any output publication. Float32 native storage is marshaled through Core typed
access, with Ada finite/nonnegative checking too; no Float64 narrowing or clamp.
The sentinel detects wholly unwritten slots; it cannot independently prove that
a custom HAL overwrote a valid coarse-level value at the final level.

The exact expression subtracts nearly equal Float32 terms for rank-deficient
patches. Positive-semidefiniteness in real arithmetic does **not** prove native
rounding never yields a tiny negative value; source has no clamp/guarantee. A
cross-version sweep of 256 directional ramp combinations at integer and
fractional points found no negatives, but is not a universal proof. The strict
public contract deliberately rejects any negative, including roundoff, rather
than silently changing the metric. Applications encountering such a native
output receive an exception, not guessed zero quality.

No new coordinate/window/level arithmetic is introduced: this eigenvalue
expression already executes in ordinary mode for threshold rejection. The
existing bounded subset remains sufficient for wrapper-controlled conversions
and allocations, with the prior native-internal/vendor limitations unchanged.
There is no material fallback-CPU semantic difference across these tags; SIMD,
NEON and 5.0 HAL rounding still rule out universal bitwise equality promises.

### Deterministic quality experiments and tolerances

96x96 images, 21x21 window, level zero, point (48,48): a quadrant step corner
(`255` iff row>=48 and col>=48) gives **0.73402756**, a vertical edge (`255` iff
col>=48) gives **0**, constant 127 gives **0**, on all three local versions.
Corner >0.5 versus edge/flat zero has a large measured margin. Tests use half
and twice the measured corner value for rejection/success, never an ULP-sized
margin; the rejected result retains the same eigenvalue.

The retained texture's four previous points give approximately
`0.265748, 0.197183, 0.151508, 0.501894`. Identity photometric L1 is zero while
quality is positive, distinguishing an accidentally disabled flag 8. The seeded
(12,7) fixture retains Task 002 refinement/consumption criteria and eigenvalues
agree with direct flag 12. Distant next predictions fail but retain quality;
distant previous points fail with zero. Seeds P0 versus P0+(32,20) yield the same
previous-patch values. Same-build independent native/Ada and shim comparisons use
1e-5 for scalar/coordinate outputs; failed next coordinates compare only after
deterministic normalization. No numeric relationship between the two metrics is
asserted in general. See Task 004 qualification for exact executed evidence.