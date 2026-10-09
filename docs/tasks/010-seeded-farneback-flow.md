# Task 010 — caller-seeded dense Farnebäck flow

Adds one public overload and one semantic C export; nothing else changes.

```ada
function Calculate_Farneback_Flow
  (Previous_Image : OpenCV.Core.Mat;
   Next_Image     : OpenCV.Core.Mat;
   Options        : Farneback_Options := (others => <>);
   Initial_Flow   : OpenCV.Core.Mat) return OpenCV.Core.Mat;
```

```c
opencv_video_status opencv_video_calc_farneback_flow_seeded(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *initial_flow,
    opencv_core_mat_handle *result_flow,
    double pyramid_scale, int32_t levels, int32_t window_size,
    int32_t iterations, int32_t poly_neighborhood, double poly_sigma);
```

Native flags are exactly `cv::OPTFLOW_USE_INITIAL_FLOW` (4); the unseeded Task 009
operation keeps flags 0. Production ABI: 15 → 16 exports. AUnit: 99 → 107
registrations. `Farneback_Options`, the Core pin, crate version, dependencies and CI
topology are unchanged; there is no new public type and no public flags argument.

## Seed contract

Nonempty, 2-D, `Float32`, two channels, exactly the image rows/columns; Regions are
accepted. Channel 0 is dx and channel 1 dy (signed pixels). Every component must be
finite with `|v| <= 2**20` (checked in widened arithmetic in Ada and in the C shim;
never clamped). The seed need not be accurate and predicted destinations need not lie
inside the image.

## Ownership and atomicity

The caller's seed is never passed as native mutable output. The shim validates the
schema and every component, clones the seed into private continuous `CV_32FC2`
storage, records its data pointer, calls native with flag 4, verifies the storage was
reused and the schema, validates every output component finite, and only then moves
the private Mat into the separate Core-owned output header. Any failure (including
injected exceptions before or after the native call) leaves the output and the seed
unchanged. Both modes share one private helper; unseeded mode keeps its NaN-sentinel
destination.

## Safety

The Task 009 `KNOWN_UPSTREAM_UB_ACCEPTED` exception (pointer formation in
`FarnebackUpdateMatrices` on 4.1.0/4.6.0/4.10.0) is inherited unchanged and does not
cover any new defect. See [source contract](../farneback-source-contract.md) and
[qualification](../task010-qualification.md).
