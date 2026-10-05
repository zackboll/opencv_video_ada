# API policy

The bootstrap implements only sparse pyramidal Lucas-Kanade tracking:
- UInt8 C1 previous/next images;
- Float32 2-D point arrays;
- typed PyrLK options;
- per-point tracked flag, next point and error;
- failed tracks expose deterministic Ada values, not native undefined output.

Do not broaden Task 001 into Farneback, ECC, KalmanFilter, CamShift, meanShift,
UMat/OpenCL, prebuilt pyramids, initial-flow guesses, VideoIO, feature detection,
Calib3D, navigation fusion, or estimator policy.
