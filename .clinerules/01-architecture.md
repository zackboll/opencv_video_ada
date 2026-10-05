# Architecture

- Public Ada namespace: `OpenCV.Video`.
- Native backend: `opencv2/video/tracking.hpp` / `opencv_video` on OpenCV 4.1-5.0.
- This is **not** an `opencv_videoio` binding.
- Core owns `Mat`; use callback-scoped `OpenCV.Core.Module_Interop` handles.
- Never vendor `opencv_core_module_bridge.hpp`.
- Keep public outputs as Ada values; do not expose native handles.
- C++ exceptions must not cross the C ABI.
- Build features vertically; do not mechanically generate the OpenCV API.
