# Task 003 qualification record

## Baseline

Fetched `origin/main`: `de0a2872462a16f11c2f0eba4ecebce249db865c`.
Task 002 PR #2 `Add seeded PyrLK initial-flow tracking`, reviewed head
`ad247113945bfe435a7e4252d252ec07f29407bd`, is an ancestor of that merge.
Before creating `feature/003-forward-backward-consistency`, confirmed clean tree,
no open/overlapping PR and exact unchanged Core pin in all three Alire roots:
`4da9d35ea21e1b2efe96296243ea668b488c6326`. Crate version remains 0.1.0-dev.

Task 002 post-merge [Windows run 37257854787](https://github.com/zackboll/opencv_video_ada/actions/runs/37257854787)
completed **successfully** on the exact merge SHA. Retrieved exact logs: MSYS2
OpenCV 5.0.0, 22 executed/passed tests, zero assertion failures/unexpected errors,
external Video/Core DLL linkage checks passed. This is Task 002 baseline evidence,
not a Task 003 Windows run. Windows remains push-to-main-only.

## Implementation contract

See [Task 003 brief](tasks/003-forward-backward-consistency.md) and the separate
[composed contract](forward-backward-contract.md) for exact signatures, arithmetic,
fixtures, mapping and oracle protocol. Added public types:
`Forward_Backward_Options`, `Forward_Backward_Track`, `Forward_Backward_Track_Array`;
added unseeded/seeded `Track_PyrLK_Forward_Backward` overloads, with defaulted
Options before required Initial_Next_Points. Existing PyrLK APIs are unchanged.

Pure Ada composition: forward LK, compact successes with saved source indices,
backward seeded LK from P1 toward original P0, map results back to Points'Range.
No new production C++ code, C ABI export/structure or native helper. Five existing
Video exports remain; no fault hook in production. No new dependency, filtering,
estimator policy or public handle. No proof-specific/SPARK helper introduced.

Distance is Float64 Euclidean original-to-recovered pixel distance, checked finite,
nonnegative and Float32-representable before conversion. Binary32 endpoint bounds
give squared-sum <2**259 (<=2**61 for both endpoints bounded to 2**29), safely below
binary64 overflow. Classification compares the returned Float32 diagnostic to the
finite nonnegative threshold. `Consistent` requires both statuses and valid distance
<= threshold; equality and zero thresholds are legal. Missing recovery is original
point/zero distance with both backward status and consistency false. Forward.Error
remains native photometric mean patch L1, independent of the geometric diagnostic.
Low round trip is useful evidence, not proof of physical correspondence.

## Local qualification

Debian 13.7 x86_64; Alire 2.1.1, GNAT/Alire g++ 16.1.0, selected gprbuild package
26.0.1 (reports GPRBUILD 26.0.0). Packaged OpenCV 4.10.0; independently source-built
4.1.0 and 5.0.0 core/imgproc/video installations retained from Task 002, OpenCL/IPP/
TBB disabled. All commands serial across the three Alire roots; warnings as errors.

| OpenCV | public/tests | AUnit registered/executed/passed | assertions/errors | boundary/faults | ASan/UBSan | direct oracle | example |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 4.1.0 | pass | 37 / 37 / 37 | 0 / 0 | pass | pass / pass | pass | 4/4 |
| 4.10.0 | pass | 37 / 37 / 37 | 0 / 0 | pass | pass / pass | pass | 4/4 |
| 5.0.0 | pass | 37 / 37 / 37 | 0 / 0 | pass | pass / pass | pass | 4/4 |

- All 22 Task 001/002 tests retained; 15 focused additions cover identity, small
  (2,1), seeded (12,7), real inconsistent native tracks, above/below/equal/zero
  thresholds, each failure direction, mixed compact mapping, Positive'Last bounds,
  mismatched/empty arrays, validation, real strided Regions, input immutability,
  ordinary API equivalence and independently generated direct-native comparison.
  GNAT may reject NaN/infinity at its validity barrier before record construction;
  tests permit that rejection, not successful acceptance.
- Fixtures experimentally established on all three versions before assertions:
  good round trips <0.003 pixels (<0.05 acceptance threshold); unseeded level-zero
  (12,7) at (25,25) succeeds both directions but has ~2.6704-pixel round trip,
  rejected at 1 pixel. Texture -> blank gives forward success/backward failure.
  Larger pyramids on the high-frequency small-translation fixture can converge
  poorly; one-level pyramid is explicitly selected, not a loosened tolerance.
- Mapping: 7/9/11 are forward successes, 8/10 distant-source failures; predictions
  make 7/11 consistent but 9 inconsistent (~2.675..2.678 pixels). Each compact
  output equals a separate one-point call at its exact original index. Extreme
  lower bound Positive'Last verifies no source-index successor arithmetic.
- Direct C++ oracle links only OpenCV: retained 8 unseeded + 4 distinguishing
  seeded tracks, plus 20 mapped forward/backward entries over four scenarios.
  AUnit compares exact status and forward/recovered coordinates and distance
  within 1e-5 on identical inputs. Oracle independently validates good/poor/failed
  semantics; no second LK implementation or manually altered returned result.
- Complete existing actual Video-shim/Core-bridge production and seeded raw
  boundary passes unchanged. Six original + six seeded injected exception cases,
  plus retained shared seed/output-storage atomicity campaign, pass. This is
  regression qualification of an unchanged production native ABI.
- ASan+UBSan instruments actual Video source, production boundary, separate fault
  boundary and direct oracle. detect_leaks=1/halt_on_error=1, all exit zero, zero
  reported findings/leaks/uncaught exceptions. Upstream OpenCV/prebuilt Core are
  not claimed fully instrumented.
- Development/Validation/Release public builds pass; Validation AUnit 37/37,
  zero failed assertions/errors. C11 shim header compiles with warnings as errors.
- Repository checker (five imports/exports, 37 registrations), seven actual
  configuration tests, all shell syntax checks and git diff --check pass.

- Independent clean Git clone of implementation commit
  `bbe06f7068b5aba94e15824b93fe01139caa62ef`, with no generated products copied,
  reproduces public/tests builds, AUnit 37/37 with zero failures/errors, direct
  oracle, complete native/fault and ASan/UBSan campaigns, retained 4/4 example,
  repository/configuration/shell/diff checks; all exit zero on OpenCV 4.10.0.

Remote review-gate evidence is recorded after those campaigns complete; these
local results do not claim that later commits already passed CI.