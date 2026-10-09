# Task 009 qualification — stopped on demonstrated upstream safety issue

## Baseline and provenance

Fetched `origin/main` = `bd742e351c22ac82a6fb203def3f54c07633e9e8`,
Task 008 PR #8 merge. Reviewed head
`c0dd05d1e3a62abaf4a835e45e89dd85ceaa6dff` is an ancestor.
Starting worktree clean; GitHub open PR inventory empty.
Isolated branch `feature/009-farneback-dense-flow` created from fetched main at
`/home/zboll/git/opencv/video-task009`; existing Task 008 worktree unchanged.
All three Core pins remain `4da9d35ea21e1b2efe96296243ea668b488c6326`.
Crate version remains `0.1.0-dev`; dependencies and CI topology unchanged.

Task 008 post-merge runs inspected once:

- [Windows 37872599345](https://github.com/zackboll/opencv_video_ada/actions/runs/37872599345):
  completed SUCCESS, Windows job SUCCESS, completed 2026-10-09 02:07:58 UTC.
- [Cross-platform 37872599293](https://github.com/zackboll/opencv_video_ada/actions/runs/37872599293):
  completed SUCCESS; repository-checks, Linux, macOS, Linux sanitizers SUCCESS.

These results are Task 008 provenance, not Task 009 Windows or PR qualification.

## Actual experiments

Direct C++ calls link OpenCV, not the Video shim or Core bridge. NaN-initialized
outputs were checked on identity and zero-filled translations `(2,1)` and
`(-2,-1)` at image sizes 16, 32, 64, 128, 256. Every tested output retains
compatible storage, correct Float32 C2 schema and zero nonfinite components.
At size 128, central-half mean vectors are approximately:

| Version | Identity | Positive | Negative |
|---|---|---|---|
| 4.1.0 | (9.31e-8, 1.65e-7) | (1.99998, 1.00000) | (-2.00001, -0.999993) |
| 4.10.0 | (9.65e-8, 1.63e-7) | (1.99998, 1.00000) | (-2.00001, -0.999993) |
| 5.0.0 | (9.65e-8, 1.63e-7) | (1.99998, 1.00000) | (-2.00001, -0.999993) |

These are measured direct-native values, not finalized binding tolerances.
Configured compile-time versions printed; 5.0.0 linkage explicitly verified
with `ldd` to `install-5.0.0/lib/libopencv_*.so.500`. The initial attempted
5.0 build incorrectly selected `opencv4.pc` and printed 4.10.0; it was discarded
and rebuilt using the actual installation's **opencv5.pc**. It is not counted
as 5.0 evidence. 4.1.0 uses its separate install; 4.10.0 uses the system build.

Actual upstream CPU translation-unit UBSan fails **4.1.0 and 4.10.0** (exit 1) in per-case runs: identity at 64/256 (not 16); translation (2,1) at 16/256 (not 64); translation (-2,-1) at 16/64/256. An earlier combined run aborted at its first failure and an earlier statement that identity fails at every size was incorrect. The 5.0.0 instrumented identity/translation
campaign at all five sizes completes with no UBSan diagnostic. Source details
and the reason preflight cannot exclude the path are in the source review.

Evidence is preserved at `/home/zboll/.cache/video-task009/`:
`probe-VERSION.txt`, `ubsan-VERSION-SIZE.txt`, `ubsan-5.0.0.txt`,
`optflowgf-VERSION.cpp`, `instrumented-VERSION.cpp`, and executables.

## Smallest reproducer and instrumentation

Repository reproducer: `tests/native/farneback_safety_probe.cpp`, a 16x16
deterministic image translated by (2,1), default options, flags zero, NaN-filled flow.
Ordinary direct-native compilation (select the desired prefix/pkg name):

```sh
g++ -std=c++17 -Wall -Wextra -Werror \
  tests/native/farneback_safety_probe.cpp \
  $(pkg-config --cflags --libs opencv4) -o /tmp/farneback-probe
/tmp/farneback-probe
```

To expose the issue, **instrument the upstream algorithm**, not just the driver.
Fetch exact tag `modules/video/src/optflowgf.cpp` to external scratch space.
Replace its private build-context `#include "precomp.hpp"` with:

```cpp
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#define CV_INSTRUMENT_REGION()
#define CV_OCL_RUN(condition, expression)
#undef HAVE_OPENCL
```

Remove `#include "opencl_kernels_video.hpp"`. No algorithm statements are
changed. Compile that translation unit alongside the reproducer, with its tag's
headers/libraries, using Clang 19 and:

```text
-std=c++17 -g -O1 -DCV_CPU_HAS_SUPPORT_SSE2=1
-fsanitize=undefined,pointer-overflow -fno-sanitize-recover=all
```

The replacement adds five net source lines: instrumented line 247 corresponds
to authoritative older-tag line 242. Diagnostic:

```text
runtime error: addition of unsigned offset to 0x... overflowed to 0x...
SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior ...:247:35
```

Prebuilt upstream libraries and Core are not claimed fully instrumented.
This scratch compilation instruments the actual Farnebäck CPU source only.

## Not completed / not claimed

No binding implementation or PR was created. Production ABI remains **14**;
the proposed fifteenth export and public Ada API do not exist. All 92 existing
AUnit registrations remain unchanged, but the complete suite was not rerun for
this safety-only investigation. No new binding ownership/Region/fault/oracle
comparison/mutation/ASan/leak/clean-clone qualification or final-head hosted CI
is claimed. Constant-image behavior and a second texture were not qualified.
No final-head pinned workflow was dispatched. This is a safety-stop handoff,
not an open-PR review-gate success.

No merge, tag, release, amend, published-history rebase, force-push, branch
deletion, main change or auto-merge occurred. Production shim, Ada sources,
manifests, workflows and prior tests are unchanged. The standalone reproducer
is intentionally outside normal regression registration and has no fault export.