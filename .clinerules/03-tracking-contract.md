# Tracking contract

- Unseeded `calcOpticalFlowPyrLK` uses flags = 0; the seeded overload uses only
  `OPTFLOW_USE_INITIAL_FLOW`. No public raw flag word.
- Separate trackability exports use flags 8 / 12. Every minimum eigenvalue is
  sentinel-initialized, validated finite/nonnegative independently of status;
  failed next points still normalize to previous points. Point_Track.Error
  remains photometric and its failed errors remain zero.
- Seed correspondence is by iteration position with equal array lengths;
  lower bounds may differ. Clone seeds privately before native mutation.
- Point coordinates are Float32 because OpenCV requires 2-channel CV_32F points.
- Input point array bounds are preserved in the returned Ada array.
- `Tracked=False` is a normal result, not an exception.
- For failed tracks, return the previous point and zero error; do not expose native
  point/error values whose meaning is not guaranteed.
- The C++ shim computes into local Mats and publishes output headers only after
  the native call and output-schema checks succeed.
