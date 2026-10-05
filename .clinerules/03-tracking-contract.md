# Tracking contract

- `calcOpticalFlowPyrLK` is called with native flags = 0.
- Point coordinates are Float32 because OpenCV requires 2-channel CV_32F points.
- Input point array bounds are preserved in the returned Ada array.
- `Tracked=False` is a normal result, not an exception.
- For failed tracks, return the previous point and zero error; do not expose native
  point/error values whose meaning is not guaranteed.
- The C++ shim computes into local Mats and publishes output headers only after
  the native call and output-schema checks succeed.
