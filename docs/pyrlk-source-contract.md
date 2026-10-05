# PyrLK portable source contract (Task 001)

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
`Module_Interop` callbacks keep all six borrowed headers alive for one call;
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