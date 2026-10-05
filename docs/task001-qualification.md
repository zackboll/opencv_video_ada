# Task 001 qualification record

Starting fetched `origin/main`: `797f501e10b263b9186d235efcfb9edf10cd6da4`.
Clean tree and no existing/overlapping open PR were confirmed before creating
`feature/001-validate-pyrlk-bootstrap`. All three Core pins remain exactly
`4da9d35ea21e1b2efe96296243ea668b488c6326`. No crate version bump.

## Local environment and results

- Debian GNU/Linux 13.7 (trixie), x86_64.
- Alire 2.1.1; selected gnat_native 16.1.0, gprbuild package 26.0.1.
- GNAT reports 16.1.0; GPRBUILD reports **26.0.0 (2026-04-15)**.
- Alire build/native compiler: g++ (GNAT-FSF-builds) 16.1.0.
  System compiler separately reports Debian 14.2.0-19 / GCC 14.2.0.
- pkg-config package `opencv4`, OpenCV 4.10.0;
  `/usr/lib/x86_64-linux-gnu/libopencv_video.so.4.10.0`.
- Root Core prefix:
  `/home/zboll/git/opencv/video/alire/cache/pins/opencv_core_4da9d35e`;
  tests/examples resolve their own same-commit caches under their Alire roots.
- Public crate, tests and example build with warnings as errors. No new warning
  suppressions were added. The pre-existing Apple c11-extension exception is
  unchanged.
- AUnit: **16 registered, 16 executed, 16 passed, 0 failed assertions,
  0 unexpected errors**. Includes identity, horizontal and two-axis translation,
  input ordering, nondefault bounds, empty points/images, rejected schemas and
  options, mixed failed/outside points, padded boundaries, real strided Regions,
  pixel immutability and result independence. IEEE special values can be rejected
  by GNAT validity checking before an Ada record is constructed; raw C++ also
  tests NaN/infinity directly with no Ada validity barrier.
- Example: 4/4 tracks on deterministic (dx=3,dy=2), with next coordinates
  (27.9997,26.9986), (48.0012,34.0013), (62.9964,51.9972),
  (38.0006,67.0010). Maximum coordinate deviation <0.004 pixels; original
  0.20-pixel tolerance preserved and now enforced by the example.
- Independent direct-C++ oracle: 8/8 tracks (64x64 test and 96x96 example
  fixtures), meaningful finite errors. Actual-shim vs direct-native comparison
  matches status exactly and coordinates/errors within 1e-5 on identical inputs.
- Actual production-shim/Core-bridge raw boundary passes, including six null
  handle positions, alias rejection, schemas, geometry, unsafe/nonfinite options
  and points, mixed failed normalization, and successful empty-output clearing.
  Existing output values are preserved on failure, not cleared (failure atomicity).
- Separate fault build: six injected bad_alloc/cv::Exception/unknown exceptions
  before native work and before publication are contained without partial output.
  `nm` confirms production library has only the four expected Video exports and
  no fault hook.
- Linux ASan+UBSan: production boundary, fault boundary and independent oracle
  all pass with `detect_leaks=1`, `halt_on_error=1`; zero reported ASan/UBSan
  findings, leaks, or uncaught exceptions. Video source itself is instrumented;
  packaged OpenCV and prebuilt Core are not claimed to be fully instrumented.
- Repository checker, 7 actual-configure-script Python unit tests, all shell
  syntax checks and `git diff --check` pass. Native driver asserts C ABI scalar
  widths and tests actual output schema; no public Ada/C record layout is assumed.

## Corrections and source decisions

See [authoritative source review](pyrlk-source-contract.md). Fixed missing private
Ada parent package, C++ `finite` ambiguity, allocation-unsafe exception diagnostics,
unsafe native arithmetic/options/conversions, raw header aliasing, and raw failed
slots. Kept UInt8 C1, Core Float32_Point, flags=0 and outside-point semantics.
The deterministic translation fixture was checked independently and was correct;
it was not replaced or its tolerances loosened.

## Remote qualification

Pinned source compatibility and PR CI results will be recorded here after their
exact workflow runs complete. Workflow definitions are not passing evidence.
Windows remains push-to-main-only, deliberately outside the PR review gate.