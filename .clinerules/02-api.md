# API policy

The bootstrap implements only sparse pyramidal Lucas-Kanade tracking:
- UInt8 C1 previous/next images;
- Float32 2-D point arrays;
- typed PyrLK options;
- per-point tracked flag, next point and error;
- failed tracks expose deterministic Ada values, not native undefined output.

Task 002 adds only caller-supplied Float32 next-point predictions through a typed
seeded overload. Do not broaden it into Farneback, ECC, KalmanFilter, CamShift,
meanShift, UMat/OpenCL, prebuilt pyramids, minimum-eigenvalue error mode, VideoIO,
feature detection, Calib3D, navigation fusion, or estimator policy.
