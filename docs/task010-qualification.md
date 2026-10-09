# Task 010 qualification — caller-seeded dense Farnebäck flow

Starting main: `fcfc5e236088d0a184eb4bbcd65cef128be96992` (Task 009 merge of PR #9,
final feature head `635dc656583a811d6326605f64132b39e5f05133`).

## Task 009 CI outcomes (checked at the start of this task)

- [Pinned compatibility 37878704479](https://github.com/zackboll/opencv_video_ada/actions/runs/37878704479): completed SUCCESS (4.1.0, 4.10.0, 5.0.0).
- [Post-merge Windows 37879056153](https://github.com/zackboll/opencv_video_ada/actions/runs/37879056153): completed SUCCESS.
- [Post-merge cross-platform 37879056178](https://github.com/zackboll/opencv_video_ada/actions/runs/37879056178): completed SUCCESS (repository-checks, linux, macos, linux-sanitizers).

## Implementation

- Ada: one new overload `Calculate_Farneback_Flow (Previous_Image, Next_Image, Options, Initial_Flow)`. The Task 009 overload keeps its signature and flags 0. Both share private `Validate_Farneback_Inputs` and `Validate_Farneback_Result`. The seed is checked for nonempty, 2-D, Float32 C2, exact image geometry, and every component finite with `|v| <= 2**20`.
- C: one new export `opencv_video_calc_farneback_flow_seeded` (production ABI 15 -> 16). Both exports call one private helper. Seeded mode validates, clones the seed into a private continuous `CV_32FC2` Mat, calls native with `cv::OPTFLOW_USE_INITIAL_FLOW` (4), checks storage identity and schema, validates every component finite, and only then publishes into the separate Core-owned header. Unseeded mode keeps its NaN-sentinel destination and flags 0.
- Unchanged: `Farneback_Options`, Core pin `4da9d35e...` in all three Alire roots, crate version, dependencies, CI topology.

## Local results (serial runs, Linux x86-64)

| OpenCV | AUnit | direct seeded oracle | ASan/UBSan/leak shim campaign |
|---|---|---|---|
| 4.1.0 | 107/107, 0 assertions failed, 0 errors | pass | exit 0 |
| 4.6.0 (Debian 12 container) | 107/107, 0 failed, 0 errors | pass | exit 0 |
| 4.10.0 | 107/107, 0 failed, 0 errors | pass | exit 0 |
| 5.0.0 | 107/107, 0 failed, 0 errors | pass | exit 0 |

The configured `Native_Version` in `config/opencv_video_install.gpr` matched each row.

AUnit grew 99 -> 107 (all 99 retained unmodified). New tests: default and nondefault
options; consumption and refinement on the 192x192 (12,7) fixture; zero-seed
equivalence (identity, small and larger translation, flat image); nonuniform seed,
immutability and result independence; signed motion, identical images, channel
ordering; Region/parent lifetime for previous, next and seed alone and combined;
validation (structure, non-finite, beyond 2**20, exactly at 2**20); and a full-field
comparison with the direct oracle (maximum difference <= 1e-5, edges included).

The direct oracle (`tests/cpp/direct_oracle.cpp`) writes a separate companion file
`farneback-seeded.txt` with four dense fields; the Task 009 `farneback.txt` is untouched.
It uses neither the shim nor the Core bridge and itself asserts that the original seed
is unchanged and that the seeded result differs from the unseeded one.

Raw boundary (`native_boundary.cpp`, actual shim + Core bridge): valid seeds; null
previous, next, seed and output; Float64, 1-channel, 3-channel and UInt8 seeds; wrong
rows and columns; empty seed; NaN, +-Inf, +-max and +-(2^20+1) in either channel at
the last pixel; invalid options; mismatched, tiny and color images; output aliases with
previous, next and seed; distinct headers sharing seed storage; a non-contiguous Region
seed; replacing and preserving a nonempty destination; repeat determinism; seed
immutability after success and failure; and fault injection (allocation, OpenCV,
standard, unknown, before and after native, plus NaN/+Inf/-Inf corruption after native)
with no partial publication. Fault hooks exist only in the `-DOPENCV_VIDEO_TEST_FAULTS`
test binary.


## Mutation sensitivity (scratch copy, not committed)

| Mutant | Killed by |
|---|---|
| native flags 4 -> 0 (Farneback call) | shim differs from independent seeded call |
| ignore seed (constant field) | shim differs from independent seeded call |
| zero seed | shim differs from independent seeded call |
| swapped components | shim differs from independent seeded call |
| overwrite result with the seed (unchanged clone) | shim differs from independent seeded call |
| native mutates the caller's seed (no clone) | seeded Farneback raw boundary failed |
| publish before validation | nonfinite result was partially published |
| finite check removed | nonfinite component accepted |
| seed-value check removed (all, and bound only) | unsafe seed value accepted |
| output/seed alias allowed | output/seed alias accepted |

Corrections made during the campaign: the first "flags" mutant matched the PyrLK call
rather than the Farneback call, so it was re-targeted and the corrected mutant is
killed. The first seed-check mutant did not compile (unused variable under -Werror) and
was rerun in a compiling form. Mutants ran against the native boundary driver (against
OpenCV 5.0.0); the AUnit suite was not separately run against each mutant.

## KNOWN_UPSTREAM_UB_ACCEPTED

Inherited unchanged from Task 009 (pointer formation in `FarnebackUpdateMatrices` on
4.1.0, 4.6.0, 4.10.0). The prebuilt libraries are not instrumented, so the binding
sanitizer campaigns neither show nor mask it and are not a claim that the older native
code is sanitizer-clean. Seeded flow can drive predictions outside the image routinely;
no additional finding appeared (no invalid read or write, leak, or binding-introduced
UB in any binding ASan/UBSan campaign, and native probes with extreme seeds returned
finite output). The Task 009 instrumented reproducer was not re-run for this task and
no instrumented seeded campaign of the upstream translation unit was performed.

## Not claimed

No claim of exhaustive native safety, no Windows result for this PR (push-to-main
only), and no claim that a seed makes the flow physically correct. No merge, tag,
release, amend, published-history rebase, force-push or auto-merge occurred.

Hosted CI results for the final head are recorded in the PR.
