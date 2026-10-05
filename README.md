# OpenCV Video for Ada

Handwritten thick Ada binding for the OpenCV **video** module used by temporal
tracking in the GPS-denied navigation stack. Repository: `opencv_video_ada`;
Alire crate: `opencv_video`; public package: `OpenCV.Video`.

**Version 0.1.0-dev; bootstrap source only, not a release.**
Task 001 qualifies the bootstrap; see [source contract](docs/pyrlk-source-contract.md)
and [qualification record](docs/task001-qualification.md) for exact evidence and
version/CI results. `docs/tasks/001-validate-bootstrap.md` remains the task brief.

## Native module

```text
Ada crate/public package        OpenCV native module
opencv_video / OpenCV.Video  -> opencv2/video/tracking.hpp -> opencv_video
```

This is intentionally **not** a `videoio` binding.

## Initial API

The bootstrap exposes one vertically integrated operation:

```ada
Tracks := OpenCV.Video.Track_PyrLK
  (Previous_Image => Previous,
   Next_Image     => Current,
   Points         => Points);
```

It wraps `cv::calcOpticalFlowPyrLK` with:

- nonempty 2-D UInt8 C1 images of identical geometry;
- Ada arrays of `OpenCV.Float32_Point`;
- typed window/level/termination/min-eigenvalue options;
- per-point `Tracked`, `Next_Point` and `Error` results;
- preserved Ada input bounds;
- deterministic failed-track values (`Next_Point = Previous_Point`, `Error = 0`);
- exception containment and failure-atomic publication in the C++ shim.

Non-contiguous Core Regions are supported with Region-local point coordinates
and native parent-aware pyramid borders. Points may be outside image bounds;
their finite coordinates must have absolute value at most 2**29 for safe native
integer conversion. Windows are 3..255 in each dimension, levels 0..30,
iterations 1..100, epsilon finite in (0,10], and minimum eigenvalue threshold
finite in [0, Float32_Value'Last]. Unsafe image padding/stride arithmetic is
rejected before the native call. Successful errors are finite nonnegative mean
patch L1 differences, not displacement accuracy or confidence. Invalid native
successful results raise `OpenCV_Error`; they are not silently normalized.

### Seeded initial-flow tracking (Task 002)

The original overload remains unchanged: its initial next estimate is the previous
point and its native flags are zero. The seeded overload uses a caller prediction
as the initial next estimate, with only `cv::OPTFLOW_USE_INITIAL_FLOW`:

```ada
Tracks := OpenCV.Video.Track_PyrLK
  (Previous_Image      => Previous,
   Next_Image          => Current,
   Points              => Points,
   Initial_Next_Points => Predictions);
```

`Points` and `Predictions` must have equal lengths; correspondence is by iteration
position, so ranges `5 .. 8` and `20 .. 23` are valid together. Results preserve
`Points'Range`. Both inputs contain Float32 values (no Float64 narrowing), with
finite coordinates bounded by `2**29`; neither needs to lie inside the image.
There is no extra displacement bound. Empty pairs retain the empty-result
contract after image/options validation; mismatched counts raise `OpenCV_Error`.

The seeded declaration places defaulted `Options` **before** required
`Initial_Next_Points`. This avoids making existing four-argument positional option
aggregates ambiguous in Ada. Omit options with named `Initial_Next_Points` as above,
or pass `(Previous, Current, Points, Options, Predictions)` positionally.

Seeds aid optimization/convergence, **not correctness**: a successful LK status is
not proof of the right correspondence. Predictions from camera-motion/navigation
models may be useful; this binding has no IMU, estimator, or navigation dependency.
Seeds and images remain unchanged. Failed tracks still return the original
previous point and zero error, **not** the prediction. The shim clones seeds into
private storage and publishes all three outputs only after successful validation.

See [Task 002](docs/tasks/002-initial-flow-seeding.md) and its
[qualification record](docs/task002-qualification.md). The existing `lk_synthetic`
example is retained; the focused AUnit and independent native oracle demonstrate
the distinguishing seeded fixture without another example executable.

LK minimum-eigenvalue output mode, prebuilt pyramids and UMat remain excluded.

## Build

Requirements:

- Alire;
- Ada 2022 toolchain;
- C++17 compiler;
- pkg-config;
- OpenCV development files providing the native `video` module;
- `opencv_core ~0.4.0`.

The bootstrap keeps the same Core source pin used by the Calib3D bootstrap:
`4da9d35ea21e1b2efe96296243ea668b488c6326`.

On Debian/Ubuntu:

```sh
sudo apt-get update
sudo apt-get install -y build-essential pkg-config libopencv-dev
alr -n build
alr test
```

Run the synthetic translation example:

```sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/lk_synthetic
```

## Architecture

```text
Ada application
    OpenCV.Video
        Ada point/options/result values
            private Ada C ABI
                callback-scoped Core Mat handles
                    C++17 stable-C shim
                        cv::calcOpticalFlowPyrLK
                            OpenCV video

Application Mat ownership: OpenCV.Core
Native Mat borrowing: OpenCV.Core.Module_Interop callback scope only
```

No C++ object, STL container, `cv::Mat`, native pointer, or native handle appears
in the public Ada API.

## Supported bootstrap range

The configure script accepts representative OpenCV versions in the existing
binding-family range:

- OpenCV 4.1 through 4.x;
- OpenCV 5.0.x.

The manual compatibility workflow is set up for 4.1.0, 4.10.0 and 5.0.0.
Workflow definitions alone are not evidence of compatibility; the qualification
record identifies the runs and results.

## CI policy

PR CI: repository checks, Linux, macOS and Linux ASan/UBSan. Windows/MSYS2 is a
separate push-to-main-only workflow. The pinned OpenCV source-build matrix is
manual-only.

## Deliberate exclusions

The first slice does not bind:

- `buildOpticalFlowPyramid` as a public API;
- Farneback or DIS dense optical flow;
- ECC registration;
- KalmanFilter;
- CamShift / meanShift;
- UMat/OpenCL overloads;
- VideoCapture / VideoWriter / codecs (`videoio`);
- feature detection, feature matching, Calib3D, geospatial transforms or
  navigation-filter policy.

Those should be added only as separate, reviewed vertical tasks.

## License

Apache-2.0; see `LICENSE` and `NOTICE`.
