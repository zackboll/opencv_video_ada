# Farnebäck source review — KNOWN_UPSTREAM_UB_ACCEPTED

> **Status update.** The original safety-stop disposition is **superseded** by the
> project owner's explicit decision `ACCEPT_KNOWN_UPSTREAM_UB_FOR_COMPATIBILITY`.
> The finding below is unchanged and remains true; it is now an *accepted upstream
> compatibility exception* (classification **KNOWN_UPSTREAM_UB_ACCEPTED**) for
> OpenCV 4.1.0, 4.6.0 and 4.10.0 (4.6.0 has the identical expression at line 242;
> its file differs from 4.10.0 only in universal-intrinsic spellings). The binding
> calls native `cv::calcOpticalFlowFarneback` and does not copy or patch it.
> **Safety-sensitive deployments should treat this as a documented limitation**:
> an instrumented build of the older CPU algorithm reports the diagnostic below.
> Any *other* sanitizer finding is not covered by this acceptance.

The remainder is the original source review.

## Authoritative sources

Reviewed `modules/video/src/optflowgf.cpp` and declarations in
`modules/video/include/opencv2/video/tracking.hpp` at exact tags
[4.1.0](https://github.com/opencv/opencv/blob/4.1.0/modules/video/src/optflowgf.cpp),
[4.10.0](https://github.com/opencv/opencv/blob/4.10.0/modules/video/src/optflowgf.cpp),
and [5.0.0](https://github.com/opencv/opencv/blob/5.0.0/modules/video/src/optflowgf.cpp).
Related fixtures are `modules/video/test/ocl/test_optflow_farneback.cpp`;
they compare CPU/OpenCL paths, not proof of CPU arithmetic safety.

## Blocking pointer arithmetic

In **4.1.0 and 4.10.0**, `FarnebackUpdateMatrices`, lines 237–253,
contains this sequence (line 242 is the pointer expression):

```cpp
float dx = flow[x*2], dy = flow[x*2+1];
float fx = x + dx, fy = y + dy;
int x1 = cvFloor(fx), y1 = cvFloor(fy);
const float* ptr = R1 + y1*step1 + x1*5;
// ...
if ((unsigned)x1 < (unsigned)(width-1) &&
    (unsigned)y1 < (unsigned)(height-1)) {
    // ptr is dereferenced here
}
```

`step1` is `size_t`. A negative predicted row becomes an unsigned offset
before the bounds check. Pointer formation itself can be undefined even when
the later branch prevents dereference. The expected out-of-bounds prediction
is not an invalid input: tiny residual flow at an identity-frame border suffices.
In **5.0.0**, the pointer expression moves inside the bounds-check branch
(line 249). This is the only difference between the reviewed 4.10.0 and 5.0.0
Farnebäck source files.

Clang 19 UBSan, instrumenting the actual older CPU algorithm translation unit,
reports `addition of unsigned offset ... overflowed ...` at this expression.
Per-case runs on both older tags: identity passes at 16 but fails at 64 and 256; translation (2,1) fails at 16 and 256 but passes at 64; translation (-2,-1) fails at 16, 64 and 256. 5.0.0 passes all nine cases. Failure is data/size dependent, so no size rule excludes it.
The corresponding 5.0.0 instrumented campaign completes without this diagnostic.
This does not claim exhaustive safety of 5.0.0.

No dimensions/stride/option preflight can generally prove that image-derived
intermediate flow will stay within the image at every iteration. Rejecting all
older-version calls would not implement the requested compatible feature.
Full-field finite validation after return is too late. Exceptions do not contain
undefined behavior. Cropping the returned field does not prevent the algorithm
from processing its own borders. Therefore no execution boundary excluding the pointer formation exists for older
versions; the owner accepted this risk (see status update above).

## Output initialization is not the blocker

The CPU entry point calls `_flow0.create`, obtains `flow0`, then at base level
assigns `flow = flow0`. With no usable reductions and no initial-flow flag,
`flow = Mat::zeros(...)` invokes `MatOp_Initializer::assign` in
`modules/core/src/matrix_expressions.cpp`. That implementation calls `create`
with compatible geometry/type and fills via scalar assignment. Compatible
storage is reused, not silently detached. Direct experiments with NaN-filled
caller destinations retain their allocation and return zero nonfinite components
on all three versions for identity and both translation directions at sizes
16, 32, 64, 128 and 256. No unwritten-output defect was observed in these cases.

## Other established observations (not full qualification)

- CPU source checks matching geometry, matching one-channel images and scale
  below one; the intended binding would further restrict to UInt8, nonempty 2-D.
- `Levels` bounds attempted reductions; the loop stops before a reduction makes
  either dimension smaller than 32. It then computes the accepted deepest level
  **and base level zero**. At scale .5 and requested 3, tested sizes 16/32 have
  zero reductions, 64 has one, 128 two, 256 three. This is not PyrLK storage.
- Smoothing uses `sigma=(1/scale-1)*.5`, odd `cvRound(sigma*5)|1`, minimum 3.
  Polynomial expansion has five-channel temporaries and `(width+2*n)*3`
  scratch arithmetic; box refinement has `(width+2*m+2)*5` scratch arithmetic.
  Dimensions, strides and intermediate allocations need widened validation.
- Flags zero select box refinement, not Gaussian refinement. The OpenCL dispatch
  requires a UMat output; direct experiments here use Mat/CPU execution.
- Polynomial preprocessing converts the source to a private Float32 `fimg`
  before blur and resize. Region-versus-clone behavior has **not** been qualified.
- Valid measured motion is signed `(dx,dy)`, rightward/downward positive, in
  pixels. Negative motion is legitimate; dense estimates supply no confidence.

Full allocation-failure, ROI/lifetime, option-extreme and memory-safety review
was stopped at the blocking finding; none is claimed qualified.