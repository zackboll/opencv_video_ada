# Task 002 qualification record

## Baseline

Fetched `origin/main`: `20d220c2e9bd46cac3d6fe1918a896b181156312`.
GitHub confirms PR #1 `Qualify the PyrLK Video bootstrap` is merged at that SHA;
reviewed head `210e284a93ec08f5fdd061370823124540b0d972` is its ancestor.
Clean worktree, no open/overlapping PR, and all three unchanged Core pins
`4da9d35ea21e1b2efe96296243ea668b488c6326` were verified before branching.
Branch: `feature/002-initial-flow-seeding`. No crate version bump.

Task 001 post-merge [Windows run 37254794045](https://github.com/zackboll/opencv_video_ada/actions/runs/37254794045)
finished **successfully** on the exact merge SHA, MSYS2 OpenCV **5.0.0**,
16 executed/passed tests, zero assertion failures/errors, retained example and
external DLL linkage checks. Its exact logs were retrieved. Windows topology
remains push-to-main-only; this is baseline evidence, not Task 002 Windows CI.

## API and source decisions

See [Task 002 contract](tasks/002-initial-flow-seeding.md) and
[authoritative source findings](pyrlk-source-contract.md#initial-flow-source-review-task-002).
New overload: `(Previous_Image, Next_Image, Points, Options := default,
Initial_Next_Points)`. Options precedes seeds intentionally: the originally
suggested parameter order made existing positional option aggregates ambiguous
under GNAT. Existing four-argument calls remain source-compatible, including
the unchanged Task 001 aggregate tests. Named `Initial_Next_Points` permits
default options. This is a semantic overload, not a separate abstraction.

New C export `opencv_video_track_pyr_lk_seeded`; original C signature unchanged.
Native flag: **only `cv::OPTFLOW_USE_INITIAL_FLOW` (4)**; unseeded remains **0**.
Equal counts, iteration-position correspondence, result follows previous array
bounds. Finite Float32 coordinates <=2**29 absolute, no Float64 narrowing, no
image-membership or displacement precondition. Private continuous Nx1 CV_32FC2
seeds are cloned; native never writes borrowed inputs/caller outputs. Failed
slots publish previous point/zero without reading undefined native data.

## Local qualification

Debian 13.7 x86_64, Alire 2.1.1; GNAT and Alire g++ 16.1.0; selected gprbuild
package 26.0.1 reports GPRBUILD 26.0.0 (2026-04-15). Warnings remain errors.
Packaged OpenCV 4.10.0, independently source-built OpenCV 4.1.0 and 5.0.0 with
core/imgproc/video, no OpenCL/IPP/TBB. No dependency/Core-pin changes.

Serial local campaign for **each** version:

| OpenCV | public/test build | AUnit registered/executed/passed | assertions/errors | actual boundary/faults | ASan/UBSan | direct oracle | retained example |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 4.1.0 | pass | 22 / 22 / 22 | 0 / 0 | pass | pass / pass | 8 unseeded + 4 seeded | 4/4 |
| 4.10.0 | pass | 22 / 22 / 22 | 0 / 0 | pass | pass / pass | 8 unseeded + 4 seeded | 4/4 |
| 5.0.0 | pass | 22 / 22 / 22 | 0 / 0 | pass | pass / pass | 8 unseeded + 4 seeded | 4/4 |

- All 16 original AUnit cases retained; six new cases exercise seeded identity,
  meaningful successful errors, distinguishing/refined translation, seeds with
  lower bounds 20..23 vs previous points 5..8, preserved result bounds, seed/image
  immutability, mismatched and empty counts, empty validation, unsafe/NaN/infinite
  seeds where representable, padded/outside seeds, deterministic failed values,
  and real strided Core Regions. GNAT can reject IEEE specials at its validity
  barrier; raw C++ tests both coordinate components directly without that barrier.
- Direct-C++ seeded oracle never calls the shim. The `(12,7)` translation,
  level-zero fixture with `(12.25,6.75)` prediction converges within 0.002 pixels
  per coordinate on all three versions; unseeded solutions miss by >5 pixels.
  Oracle requires <=0.05 Euclidean deviation and >=0.20 refinement away from the
  seed, so neither ignored seeds nor merely returned seeds can pass.
- Actual shim/Core comparison matches direct native status exactly and meaningful
  successful coordinates/errors within 1e-5. Raw campaign rejects invalid seed
  count/depth/channels/shape/stride, both nonfinite/unsafe coordinate components,
  seven null handle positions, all output/input header aliases, and output/output
  aliases. Input/input aliases are supported; distinct output headers sharing seed
  storage do not mutate inputs. Empty pairs clear outputs only on success;
  mismatches, invalid arguments, and contained exceptions preserve outputs.
- Six original plus six seeded injected allocation/native/unknown exceptions,
  before native work and before publication, pass. An additional post-compute
  exception with seed/output shared storage is contained without mutation.
  Textureless/outside failures normalize without reading failed native errors.
- ASan+UBSan instruments **actual Video source**, production-boundary, separate
  fault-boundary and direct oracle binaries; `detect_leaks=1`, `halt_on_error=1`.
  All exit zero: zero reported ASan/UBSan findings, leaks, or uncaught exceptions.
  Packaged/source-built OpenCV and prebuilt Core are **not** claimed fully
  instrumented. Production `nm` shows exactly five Video exports, no fault hook.
- Public Development/Validation/Release builds pass; Validation AUnit 22/22,
  zero failures/errors. C header compiles as C11 with warnings-as-errors.
- Repository checker (five imports/exports, 22 registrations), seven actual
  configure-script tests, all shell syntax checks, and `git diff --check` pass.
- Independent clean Git clone of implementation commit
  `68227a9ad6cf178132806ee2963d0ea76b9dd294`, with no generated artifacts copied,
  reproduces public/test builds, AUnit 22/22, native/fault/oracle, ASan/UBSan,
  and retained 4/4 example, all exit zero.
- Mutation experiment compiled a temporary shim copy with flag 4 replaced by
  zero: actual-boundary test exits 1 at the seeded direct-native comparison.
  The repository implementation is unchanged by that experiment.

No material seeded semantic difference was observed across the tested versions;
SIMD/HAL still precludes a universal cross-version bitwise-equality promise.
Native-internal arithmetic/custom HAL remains upstream responsibility as described
in the source contract. No unresolved implementation/portability blocker found
in the qualification campaigns.

## Remote qualification

Implementation/qualification head:
`68227a9ad6cf178132806ee2963d0ea76b9dd294`.
PR [#2 — Add seeded PyrLK initial-flow tracking](https://github.com/zackboll/opencv_video_ada/pull/2).

[PR CI run 37256307856](https://github.com/zackboll/opencv_video_ada/actions/runs/37256307856)
completed successfully: repository-checks, Linux, macOS and Linux sanitizers.
Linux used packaged OpenCV **4.6.0**; macOS used Homebrew **5.0.0**. Both report
22 registered/executed/passed tests, zero assertion failures/errors, actual raw
boundary/fault/oracle and retained 4/4 example. macOS runtime checks verify
opencv_video, Core shim, libc++ and no libstdc++. Linux sanitizer success covers
the actual seeded Video source and zero reported findings/leaks/uncaught exceptions.

[Manual pinned compatibility run 37256306585](https://github.com/zackboll/opencv_video_ada/actions/runs/37256306585)
completed successfully on the implementation head, separate source-built Linux
OpenCV installations:

| OpenCV | build | AUnit registered/executed/passed | assertions/errors | raw boundary + faults | seeded direct oracle | ASan / UBSan | retained example |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 4.1.0 | pass | 22 / 22 / 22 | 0 / 0 | pass | 4/4 (+8 unseeded) | pass / pass | 4/4 |
| 4.10.0 | pass | 22 / 22 / 22 | 0 / 0 | pass | 4/4 (+8 unseeded) | pass / pass | 4/4 |
| 5.0.0 | pass | 22 / 22 / 22 | 0 / 0 | pass | 4/4 (+8 unseeded) | pass / pass | 4/4 |

Exact CI/job logs were retrieved and checked for the native versions, test
counts, boundary/fault/oracle results and sanitizer success. No deterministic
remote failure or blind rerun occurred. Upstream source builds emit their own
compiler/CMake warnings; binding warnings-as-errors were not relaxed. GitHub's
checkout@v4 Node 20 deprecation annotation is non-failing (forced Node 24).

The evidence-only follow-up changes no runtime code, fixtures or workflows.
Required PR jobs are checked again on the final pushed head at handoff; exact
final-head links remain on PR #2 rather than claiming this earlier run tested
a later commit. Windows remains push-to-main-only, not a Task 002 PR job.
No merge, tag, release, amend, rebase, force-push or auto-merge occurred.