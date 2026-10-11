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

# Task 010 addendum — `OPTFLOW_USE_INITIAL_FLOW` (flags 4)

Reviewed `FarnebackOpticalFlowImpl::calc` in `modules/video/src/optflowgf.cpp` at tags
4.1.0, 4.6.0, 4.10.0 and 5.0.0 (all four fetched and diffed). `calc` is identical in
all four; the only differences are universal-intrinsic spellings (4.1.0/4.6.0 vs
4.10.0/5.0.0) and the placement of the `ptr` expression inside
`FarnebackUpdateMatrices` (the accepted upstream finding above). Behavior relevant to
seeded mode:

1. With the flag set, `calc` asserts `_flow0.size() == prev0.size()`, two channels and
   `CV_32F` depth; otherwise it calls `_flow0.create(size, CV_32FC2)`. The seed
   therefore must already be an allocated `CV_32FC2` of image size.
2. `levels` is reduced while `cols*scale` and `rows*scale` stay at least 32; the loop
   then runs from the deepest accepted level down to level zero.
3. At the first (coarsest) level `resize(flow0, flow, Size(width,height), 0, 0,
   INTER_AREA); flow *= scale;` initializes the field in that level's coordinates.
   Later levels use an `INTER_LINEAR` resize of the previous level and `flow *= 1/scale`.
4. At level zero `flow = flow0`; with zero reductions the INTER_AREA resize at scale 1
   rewrites the same storage.
5. `_flow0` is an `InputOutputArray` and is mutated in place, also on paths that later
   throw. The Mat (not UMat/OpenCL) path is selected because the binding passes a Mat.
6. Direct experiments (4.1.0, 4.10.0 and 5.0.0; 4.6.0 only through the full campaign; sizes 16/32/64/128/256 = 0..3 reductions at
   scale .5, 3 levels) show the supplied storage pointer is reused with zero nonfinite
   components. The binding still checks pointer identity and finiteness and rejects
   otherwise.

## Coordinate-conversion domain

`FarnebackUpdateMatrices` computes `fx = x + dx` in `float`, then `cvFloor(fx)`. Images
are limited to `INT_MAX/16` pixels, so a row or column index is below 2^27, well inside
`int`. A component up to 2^20 keeps `x + dx` far inside `int` range, also after the
coarse-level scaling by a factor below one. The 2^20 bound is a **binding policy**, not
an OpenCV guarantee. Native probes with seeds of 2^20, 2^22 and 1e9 produced finite
output on 4.1.0, 4.10.0 and 5.0.0 (4.6.0 was not probed separately), so the bound is conservative rather than forced by
the evidence. It is applied identically in Ada and the C shim, in widened (double)
arithmetic; values outside it are rejected, never clamped.

## Measured behavior (identical on 4.1.0, 4.10.0, 5.0.0; the 4.6.0 AUnit assertions on the same fixtures pass)

Fixture: deterministic texture translated by (12,7), default options, central-half mean:

| size | unseeded | seed (12.25,6.75) | seed (-20,15) | zero seed |
|---|---|---|---|---|
| 128 | (12.00,7.00) | (12.00,7.00) | (-16.9,20.4) | identical to unseeded |
| 192 | (6.25,6.13) | (12.00,7.00) | (-18.3,16.9) | identical to unseeded |

The 128x128 fixture is not distinguishing (unseeded already succeeds). 192x192 is: the
unseeded call fails to recover the translation, the near-correct seed does, and a
different valid seed gives a different wrong answer, so the seed is demonstrably
consumed. Zero-seed output was bitwise identical to unseeded output in the measured
cases (tests allow 1e-3 rather than asserting bitwise identity). Poor seeds are not
corrected; the algorithm refines locally.