# Task 009 qualification

> **Disposition update.** The earlier safety stop is superseded. Project owner decision
> **ACCEPT_KNOWN_UPSTREAM_UB_FOR_COMPATIBILITY** authorizes implementation despite the
> documented upstream pointer formation in 4.1.0, 4.6.0 and 4.10.0. That finding is
> reported separately below as **KNOWN_UPSTREAM_UB_ACCEPTED**; it is *not* labelled
> sanitizer-clean and nothing unrelated is suppressed.
>
> The sections from "Baseline and provenance" through "Smallest reproducer" are the
> preserved research evidence from commit `353a68a`. Implementation results follow in
> "Implementation qualification".

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

## Implementation qualification

### Binding

`OpenCV.Video.Calculate_Farneback_Flow (Previous, Next, Options)` and export 15,
`opencv_video_calc_farneback_flow`, call native `cv::calcOpticalFlowFarneback`
with flags **0** (box refinement, no initial flow, no Gaussian flag). Ada validates
images (nonempty, 2-D, UInt8 C1, identical geometry, >= 16x16, <= 2^27 pixels;
Regions accepted) and options (scale .25-.90, levels 1-8, odd window 5-63,
iterations 1-30, neighborhood 5 or 7, sigma .1-10, finite) before any native call;
the C shim repeats every check, rejects output aliasing either input header, and
checks signed-arithmetic bounds. The native result is computed into a private
NaN-sentinel `CV_32FC2` Mat (reuse of that storage is required and verified),
schema- and full-field finite-validated, and only then moved into the Core-owned
output header, so a failure of any kind leaves the output unchanged. All C++
exceptions (cv::Exception, std::exception, unknown) are contained. Ada re-validates
the schema and every component. Direction: channel 0 = dx, channel 1 = dy,
`Previous(y,x) ~ Next(y+dy, x+dx)`; translation (2,1) gives about (+2,+1).

### Results (this machine, serial runs)

| OpenCV | AUnit | direct oracle | ASan/UBSan/leak shim campaigns |
|---|---|---|---|
| 4.1.0 | 99/99, 0 assertions, 0 errors | pass | exit 0 |
| 4.6.0 (Debian 12 container) | 99/99, 0 errors | pass | exit 0 |
| 4.10.0 | 99/99, 0 errors | pass | exit 0 |
| 5.0.0 | 99/99, 0 errors | pass | exit 0 |

AUnit grows from 92 to 99 registrations (7 Farneback tests: identity, translation
directions, schema/ownership, Regions vs compact copy, validation incl. boundary
values, direct-native oracle comparison with <= 1e-5 agreement over 3 flows of
96x96, minimum-size/extreme options). Repository checker: 15 ABI declarations/
imports, 99 registrations. Release, Validation and Development root builds, the
tests Validation build + AUnit, `alr -n test` (99/99), the example, shell syntax,
configuration tests and `git diff --check` all pass.

The native boundary (actual shim + Core bridge, ASan+UBSan+leak) adds: oracle
equality, schema, 30+ validation rejections that leave published output unchanged,
alias rejection, private-storage proof, 8 injected exceptions (allocation, native,
standard, unknown at two stages) and 3 injected post-native NaN/+Inf/-Inf
corruptions (test-only hook, compiled solely in the fault-injection binary), all
contained atomically.

Mutation checks (each applied to the shim, caught by the named failure): nonzero
flags, images swapped, even window accepted, alias check removed, size floor
removed, finite validation weakened, and publish-before-validate. The first run
survived two mutants (finite check, publish-before-validate) because no test could
reach a nonfinite native result; the post-native corruption hook was added and both
are now killed. (Survivors are recorded, not hidden.)

### KNOWN_UPSTREAM_UB_ACCEPTED (reported separately)

Re-run of the committed reproducer in this session, instrumenting the exact upstream
CPU translation unit with Clang 19 `-fsanitize=undefined,pointer-overflow`:

| Version | Result |
|---|---|
| 4.1.0 | exit 1: `addition of unsigned offset ... overflowed` at instrumented line 247 (upstream 242) |
| 4.6.0 | same source expression (line 242); not separately instrumented here |
| 4.10.0 | exit 1, same diagnostic |
| 5.0.0 | exit 0, no diagnostic (not claimed exhaustive) |

The prebuilt OpenCV libraries are not instrumented, so the binding's own
ASan/UBSan campaigns neither exhibit nor mask this diagnostic and are *not* a claim
that the older native code is sanitizer-clean. No finding other than this one was
observed; no invalid memory access, leak, or binding-introduced UB was found.

### Not claimed

No claim that Farneback is exhaustively safe on any version. Clean-clone, hosted
PR CI and final-head pinned compatibility results are recorded in the PR, not here.
No merge, tag, release, amend, published-history rebase, force-push or auto-merge
occurred.
