# Task 009 — dense Farnebäck flow

Add an unseeded, box-refinement `OpenCV.Video.Calculate_Farneback_Flow` returning
a Core-owned full-resolution Float32 C2 Mat, native flags exactly zero, with no
PyrLK-pyramid reuse or application policy. Core pin, dependencies, version,
ownership and existing APIs are unchanged; the production ABI grows from 14 to 15
exports (`opencv_video_calc_farneback_flow`).

## Disposition history

1. **Safety stop (commit `353a68a`, superseded).** Source review and direct-native
   experiments found that OpenCV 4.1.0, 4.6.0 and 4.10.0 form an out-of-range
   pointer in `FarnebackUpdateMatrices` before the bounds check (undefined
   behavior even though it is never dereferenced). No size/option preflight can
   exclude it. Work stopped for an owner decision. That evidence is preserved in
   [the source contract](../farneback-source-contract.md).
2. **Owner decision: `ACCEPT_KNOWN_UPSTREAM_UB_FOR_COMPATIBILITY`.** The project
   owner reviewed the finding and explicitly authorized implementation against
   native `cv::calcOpticalFlowFarneback` on every supported version, with no copied
   or patched algorithm, no restriction to 5.0.0 and no private implementation.
   This is an accepted upstream compatibility exception, **not** a Task 009
   blocker. It covers only that pointer formation; it is not permission to ignore
   any new defect (invalid read/write, use-after-free, heap corruption,
   binding-introduced UB, unexplained sanitizer failure), which would still stop
   the task.

See [qualification](../task009-qualification.md) for actual results.
