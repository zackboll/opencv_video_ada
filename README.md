# OpenCV Video for Ada

### Minimum-eigenvalue trackability with owned pyramids (Task 007)

```ada
Previous_Pyramid : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Previous_Image);
Current_Pyramid  : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Current_Image);
Quality : constant Trackability_Track_Array := Track_PyrLK_Trackability
  (Previous_Pyramid, Current_Pyramid, Points);
Seeded_Quality : constant Trackability_Track_Array := Track_PyrLK_Trackability
  (Previous_Pyramid, Current_Pyramid, Points,
   Initial_Next_Points => Predicted_Points);
```

This usage fragment assumes application images and arrays. Options precedes the
required predictions; named predictions permit default options. Counts must match,
but array lower bounds may differ. Both overloads preserve `Points'Range`. Stored
image/derivative vectors are reused directly, with flags **8 / 12** respectively;
seeds are privately cloned and refined, never returned as failed coordinates.
Pyramid geometry/window/requested-depth compatibility is checked even for empty
points; available depth can truncate naturally.

`Minimum_Eigenvalue` measures **previous-patch LK conditioning**, normalized by
window area. It is not photometric difference, round-trip error or probability:

- `Point_Track.Error`: photometric patch difference;
- `Trackability_Track.Minimum_Eigenvalue`: previous-patch LK conditioning;
- `Forward_Backward_Track.Round_Trip_Error`: geometric round-trip inconsistency.

A failed track returns the original previous point but retains valid native
quality, including a positive eigenvalue on failed next search or threshold
rejection. Every quality slot must be defined, finite and nonnegative. Native
unwritten/invalid quality raises `OpenCV_Error` for the entire call; no zero is
fabricated. In particular OpenCV 5.0 KleidiCV can leave unavailable-previous
quality unwritten: prebuilt vectors still reach the per-level HAL, so they are
not a workaround for that backend contract. See the [source contract](docs/pyrlk-source-contract.md)
and [qualification](docs/task007-qualification.md) for actual backend evidence.
No universal application/navigation quality threshold or combined score is given.
Owned pyramids survive source/Region/parent finalization and support sequential
reuse/role reversal; concurrent safety is not claimed.

### Seeded tracking with reusable owned pyramids

```ada
declare
   Previous_Pyramid : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Previous_Image);
   Current_Pyramid  : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Current_Image);
   Tracks : constant Point_Track_Array := Track_PyrLK
     (Previous_Pyramid    => Previous_Pyramid,
      Next_Pyramid        => Current_Pyramid,
      Points              => Points,
      Initial_Next_Points => Predicted_Points);
begin
   for Track of Tracks loop
      if Track.Tracked then
         Process_Tracked_Point (Track.Next_Point);
      end if;
   end loop;
end;
```

This usage fragment assumes application images, point arrays and processing
procedure. Predictions assist convergence, not guarantee a physically correct
match. Equal array lengths are required; lower bounds may differ. Predictions
are privately copied and refined with native flag 4. Failed tracks return the
previous point and zero photometric error, never the prediction. Owned pyramids
remain usable after source Mats/Regions/parents are finalized and support
sequential reuse and frame-role reversal, not a general thread-safety guarantee.
Task 006 retains the original 59 tests and adds six focused campaigns (65 AUnit
registrations); the C ABI inventory is twelve semantic exports/imports.

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

Prebuilt pyramids and UMat remain excluded. Minimum-eigenvalue output is available
only through the distinct Task 004 trackability API below.

### Forward/backward consistency diagnostics (Task 003)

Ordinary PyrLK asks **“where did this point go?”** The composed forward/backward
diagnostic asks **“if I track it there and then back again, do I return to where
I started?”** This operation lives entirely in Ada; OpenCV does not provide a
single forward/backward API here.

```ada
Diagnostics := OpenCV.Video.Track_PyrLK_Forward_Backward
  (Previous_Image => Previous,
   Next_Image     => Current,
   Points         => Points,
   Options        =>
     (Tracking                 => (others => <>),
      Maximum_Round_Trip_Error => 1.0));
```

The seeded-forward overload also accepts named `Initial_Next_Points`, with the
same equal-count, iteration-position and differing-lower-bound rules as Task 002.
Options precedes required seeds. Images, points and predictions remain unchanged.
One diagnostic is returned for **every** input point, preserving `Points'Range`;
no point is silently filtered out.

Only forward successes enter the compact backward call. It tracks from the next
image back into the previous image, starting at each forward location and
**seeding the backward destination with the original previous point**, not the
forward location. The compact results are mapped back to their original indices.

Each `Forward_Backward_Track` contains:

- `Forward`: the ordinary `Point_Track`, unchanged;
- `Backward_Tracked`: whether backward LK succeeded;
- `Recovered_Previous_Point`: the backward location, or the original if unavailable;
- `Round_Trip_Error`: Euclidean original-to-recovered distance **in pixels**,
  calculated in Float64 then checked/converted to finite nonnegative Float32;
- `Consistent`: forward success **and** backward success **and** a valid returned
  distance `<= Maximum_Round_Trip_Error`.

**`Forward.Error` is native mean patch L1 photometric error; `Round_Trip_Error`
is geometric displacement. They are separate quantities, not confidence scores.**
If either pass fails, recovery is the original point, distance is zero, and both
`Backward_Tracked` and `Consistent` are false. That zero never signifies success.

The threshold must be finite and nonnegative; zero is legal, and no arbitrary
maximum or silent clamping is imposed. The default one pixel is a convenience,
**not estimator policy or a universal acceptance criterion**. Low round-trip
error is useful evidence, **not proof of a correct physical correspondence**:
ambiguous repeated patches can still agree in both directions.

See [Task 003](docs/tasks/003-forward-backward-consistency.md) and its
[qualification record](docs/task003-qualification.md) for arithmetic bounds,
real inconsistent fixtures, mapping evidence and native version results.

### Minimum-eigenvalue trackability (Task 004)

```ada
Quality := OpenCV.Video.Track_PyrLK_Trackability
  (Previous_Image => Previous,
   Next_Image     => Current,
   Points         => Points);
--  Or supply Initial_Next_Points => Predictions, with optional typed Options.
```

The distinct `Trackability_Track_Array` preserves `Points'Range` and contains
`Previous_Point`, `Next_Point`, `Tracked`, and **`Minimum_Eigenvalue`**, not `Error`.
Native flags are 8 (unseeded) or 12 (seeded); no raw flags interface is exposed.
`Point_Track` and all existing ordinary/forward-backward operations are unchanged:
**`Point_Track.Error` always means successful mean L1 photometric patch error.**

The new metric is local LK conditioning of the **previous-image** patch, normalized
by window pixel count. Higher values generally indicate stronger two-dimensional
gradient structure. It is not probability, match confidence, reprojection error,
photometric error, or proof of correct correspondence; no universal application
threshold is prescribed. All returned values are finite nonnegative Float32.

Quality is independent of status: threshold rejection can retain a positive
eigenvalue, and a strong previous patch can retain quality after next-search
failure. An unavailable previous patch returns zero on the reviewed CPU path.
Every failed `Next_Point` is still the original point. The shim sentinel-initializes
private quality storage and rejects invalid/unwritten native values atomically,
never clamps them or silently changes status. Existing images/options/points/seeds
validation and immutability apply, including differing seed bounds and Regions.
See the [source contract](docs/pyrlk-source-contract.md#minimum-eigenvalue-source-review-task-004)
and [qualification record](docs/task004-qualification.md).

**Backend limitation:** OpenCV 5.0's KleidiCV 26.03 HAL (observed in macOS ARM64
Homebrew 5.0.0_5) leaves quality unwritten for unavailable previous patches. The
new API deliberately raises `OpenCV_Error` for the entire call on that path,
instead of fabricating zero. The reviewed fallback CPU implementations write
defined zero. This does not change ordinary tracking or failed photometric errors.

### Reusable owned PyrLK pyramids (Task 005)

```ada
declare
   Previous_Pyramid : constant OpenCV.Video.PyrLK_Pyramid :=
     OpenCV.Video.Build_PyrLK_Pyramid (Previous);
   Current_Pyramid : constant OpenCV.Video.PyrLK_Pyramid :=
     OpenCV.Video.Build_PyrLK_Pyramid (Current);
begin
   Tracks := OpenCV.Video.Track_PyrLK
     (Previous_Pyramid, Current_Pyramid, Points);
end;
```

`PyrLK_Pyramid` is a limited private, finalized **binding-owned** object, not a
native OpenCV class or an alternative application Mat wrapper. It owns every
image and derivative level after construction; source mutation/finalization and
Region/parent scope exit do not invalidate it. Default objects are empty and safe
to finalize. `Is_Empty` is valid for them; all metadata queries and tracking reject
empty objects with `OpenCV_Error`. No level/derivative/native-handle extraction.

`PyrLK_Pyramid_Options` contains only `Window_Size` (default 21x21, bounds 3..255)
and `Max_Level` (default 3, bounds 0..30). Builds always use derivatives=true,
reuse-input=false, `BORDER_REFLECT_101` image borders and `BORDER_CONSTANT`
derivative borders. A strided Region's parent may contribute border pixels during
construction, as in raw-image PyrLK; there is no `BORDER_ISOLATED` policy change.

Read-only queries: `Requested_Max_Level`, `Available_Max_Level`,
`Build_Window_Size`, `Is_Empty`. Native geometry can truncate available levels:
tracking accepts requests <= **both requested build levels**, then uses the minimum
of its request and both available levels. Build/track windows must match exactly
and base geometry must agree, even for empty points. Point bounds, validation,
photometric error and failed-value normalization retain the ordinary contract.

The native tracker receives the stored vectors directly; it does not rebuild
pyramids per call. Sequential repeated reuse and next-now/previous-later use are
supported; general concurrent thread safety is not promised. Precomputing dx/dy
costs memory but permits reuse when a current frame becomes previous. No universal
speedup is claimed without measurements. Seeded and trackability overloads are now available; forward/backward
pyramid composition remains deferred. See [Task 005](docs/tasks/005-reusable-pyrlk-pyramids.md)
and its [qualification record](docs/task005-qualification.md).

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

- forward/backward prebuilt-pyramid composition;
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
