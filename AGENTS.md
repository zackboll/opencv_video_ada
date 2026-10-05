# Agent entry point

Read the files in `.clinerules/` before changing this repository.

This is a handwritten thick Ada binding. Production bindings are not generated.
`opencv_core` remains the sole owner of application `Mat` wrappers. Video may
borrow native Mat headers only through `OpenCV.Core.Module_Interop` callbacks.
Never expose C++/STL ABI types or public raw pointers.

The public namespace is `OpenCV.Video`; the native backend is OpenCV's `video`
module (`opencv2/video/tracking.hpp`, `opencv_video`). Do not confuse it with
`videoio`.

Start with `docs/tasks/001-validate-bootstrap.md`. The ZIP has static validation
only; no native Ada/OpenCV build result is claimed. Do not merge, tag, release,
amend, or force-push unless the user explicitly asks.
