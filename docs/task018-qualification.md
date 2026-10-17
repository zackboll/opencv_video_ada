# Task 018 qualification — moving-image-masked ECC homography alignment

## Baseline

* **Starting point.** `origin/main` was `114ded7571de01715524a9543aaba263e5ab91b7` (merge of PR #17,
  Task 017 including the R1 padding correction `fa7303c`, merged 2026-10-17). Branch
  `feature/018-masked-ecc-homography` was created in a fresh worktree from that commit; the worktree
  was clean and no other Task 018 branch or PR existed.
* **Post-merge CI on `114ded7`.** Cross-platform Video (repository-checks, linux, macos,
  linux-sanitizers): **success**. Windows post-merge Video: **in progress** when checked (recorded,
  not polled). No baseline failure was confirmed.
* **Inventory change.** Exports 23 → 25; AUnit registrations 186 → 198 (all 186 retained). Core pin
  `4da9d35ea21e1b2efe96296243ea668b488c6326` unchanged in all three Alire roots; no dependency,
  crate version or CI topology change.

| Step | Commit | Content |
|---|---|---|
| safety gate | `f9d3db9` | four-version source review, probe, instrumented OpenCV, diagnostic `ecc.cpp`, memory |
| API | `ef3daac` | two Ada overloads, two exports, shared `ecc_homography_entry` |
| oracle/AUnit | `f203786` | 53-fixture oracle, 12 AUnit registrations |
| boundary | `86bad70` | native boundary, schema, support, faults, padding |
| docs | `4fa2048` | task note, README (compiled example) |
| report | this commit | this file |

## Safety gate

`SAFETY_GATE_ESTABLISHED` was recorded before any production code; see
`docs/ecc-homography-masked-source-contract.md`. Summary:

* The moving mask is binarized (`threshold 0 → 1`), eroded by a 5x5 Gaussian with a `0.5/0.95`
  rescale, multiplied into (4.x) or zeroed into (5.0) the moving gradients, and warped each
  iteration with the moving image (`INTER_NEAREST + WARP_INVERSE_MAP`) into the template frame.
  The mask never indexes memory or sizes an allocation; all mask temporaries depend only on the
  geometry.
* Empty effective support makes both norms and the correlation exactly 0, so `rho = 0/0 = NaN` and
  the existing `cvIsNaN` guard throws before `lambda` or an update. Diagnostic `ecc.cpp` copies
  counted 339-663 such iterations per sweep: all ended in that exception; no infinite `rho`; no
  zero-norm/nonzero-correlation iteration; no finite narrowing above `FLT_MAX`.
* **Instrumented upstream OpenCV** (ASan/UBSan/`float-cast-overflow`, all four versions): semantics,
  types, support, occlusion and degenerate probes plus 1200 one-iteration and 600 hundred-iteration
  randomized mask/seed sweeps per version; 0 reports, rc 0 everywhere.
* Accepted domain: nonempty 2-D UInt8 C1 mask, moving geometry, int-sized step, at least one nonzero
  pixel; Regions allowed. No arbitrary minimum-support count (justified in the contract).

## Native observations versus binding requirements

The oracle records what native OpenCV does on the same build. The binding requirement is bitwise
agreement on success (nine Float32 bit patterns and the Float64 correlation bit pattern, H22 exactly
1, zero padding at the C boundary) and `OpenCV_Error` with an untouched result on native failure.

### Oracle (53 fixtures, `tests/cpp/ecc_homography_masked_oracle.cpp`)

| OpenCV | Native successes | Exceptions (NaN / lambda) | Differ from unmasked | Equal to unmasked |
|---|---:|---:|---:|---:|
| 4.1.0 | 39 | 14 (10 / 4) | 37 | 16 |
| 4.6.0 | 39 | 14 (10 / 4) | 37 | 16 |
| 4.10.0 | 39 | 14 (10 / 4) | 37 | 16 |
| 5.0.0 | 39 | 14 (10 / 4) | 37 | 16 |

* Outcomes and error classes are identical on all four versions for these fixtures; 4.6.0 output is
  byte-identical to 4.10.0 after the version line; 4.1.0 and 5.0.0 differ only in low-order bits.
  Marginal supports outside the oracle (6-11-pixel strips/blocks) do differ by version; they are
  recorded in the source contract rather than normalized.
* **All-included masks** (values 1, 7, 255, arbitrary nonzero; identity, translation, perspective,
  seeded) are bitwise equal to `noArray()` on every version, so the binding asserts exact equality.
* **Mask consumption and coordinate frame.** Every partial mask changes the result. The transposed
  hole and a template-frame mask give different results from the moving-frame mask.

### Occlusion improvement (interior reference-point error, oracle fixtures)

| Fixture | 4.1.0 | 4.6.0 = 4.10.0 | 5.0.0 |
|---|---|---|---|
| translation, misleading texture, identity start | 2.88 → 0.024 px | 2.88 → 0.024 px | 2.70 → 0.021 px |
| translation, saturated block | EXC → 0.024 px | EXC → 0.024 px | EXC → 0.021 px |
| mild perspective, misleading texture | 5.56 → 0.059 px | 5.56 → 0.059 px | 5.56 → 0.060 px |
| translation+perspective, block, **seeded** | EXC → 0.100 px | EXC → 0.100 px | EXC → 0.099 px |
| translation+perspective, texture, **seeded** | 4.47 → 0.100 px | 4.52 → 0.100 px | 4.48 → 0.099 px |
| scale+perspective, texture, **seeded** | 1.75 → 0.149 px | 1.75 → 0.148 px | 1.75 → 0.150 px |
| same block, mask in **template** frame | EXC → 75 px | EXC → 75 px | EXC → 75 px |
| translation texture, **transposed** mask | 2.88 → 3.64 px | 2.88 → 3.10 px | 2.70 → 3.67 px |

The last two rows show that a wrong mask frame does not help and can be catastrophic; the binding
does not detect it. Correlations of masked successes are over the eligible pixels only.

## AUnit (198 registrations; 12 new)

`Full_Equivalence`, `Value_Semantics`, `Occlusion_Improves`, `Seeded_Occlusion`,
`Coordinate_Frame`, `Numerics`, `Support`, `Validation`, `Immutability_And_Regions`,
`Existing_Unchanged`, `Direct_Oracle`, `Oracle_Integrity` (prefix `ECC_Homography_Masked_`).

* Oracle comparison is bitwise through both masked overloads; for identity-seed fixtures the masked
  seeded overload must equal the masked identity overload. The fixture images and masks are rebuilt
  independently in Ada (`ECC_Fixtures.Mask`, `Occlude`) and matched by checksum and nonzero count.
* Integrity rejects missing, empty, truncated (end and mid-record), wrong magic/format/version,
  wrong count, **reordered records**, renamed fixtures, extra tokens, altered options, image and mask
  checksums, mask counts, seed bits, swapped/transposed output coefficients, a one-ulp change of
  H11, altered correlation bits, success/exception swaps, inconsistent error classes, unknown
  outcome tokens and malformed hex.
* Three tolerances were set from measurements during development: the unmasked occlusion error
  threshold (2.0 px; measured 2.7-2.9), the masked-affine tolerance (Task 017 `Hs_Near`; K_SP shifts
  0.1 px when 1200 pixels are excluded), and the narrow left band, which converges to a finite but
  inaccurate result (2.3 px) and is now asserted as such.

## Native boundary and fault campaign

`run_ecc_homography_masked()` (production and `OPENCV_VIDEO_TEST_FAULTS` builds):

* 22 same-build agreements with independent direct masked `findTransformECC` calls (identity and
  seeded, translation, transposed hole, occlusion in moving and template frame, sparse supports,
  non-overlap at 1 and 50 iterations, all-Region inputs, full aliasing, options 1/3/100 and
  `1e-8`/`0.1`, constant images).
* Mask consumption on both exports; full masks 1/7/255 byte-identical to both unmasked exports on
  success and failure; values 1/255/arbitrary byte-identical; an independent direct-level check that
  a full mask equals `noArray()` and a hole does not.
* 24 rejections (12 masks × 2 exports: empty, Float32, Int8, UInt16, 2 and 3 channels, 3-D, rows±1,
  columns+1, 64x64, all-zero), each with its diagnostic and all 48 bytes untouched; null mask, null
  result; ordering images → options → mask → seed; seed domain unchanged.
* Regions (non-contiguous mask bitwise equal to compact, parent unchanged, short Region rejected);
  caller mask, images unchanged.
* Three prefills: success bytes prefill-independent with zero padding 36-39; failures preserve every
  byte.
* Fault build: `PASS: ECC homography masked faults: 32 exception injections, 80 rejected
  corruptions, 32 schema mutations, 20 accepted finite matrices, mask validation before hooks`.

Existing campaigns are unchanged: Task 016/017 boundary, fault (243 padding failures) and padding
checks pass; the Task 016 and Task 017 oracle outputs are **byte-identical** to the Task 017
campaign on every version.

Production archive: 25 `T opencv_video_*` symbols, 0 `test_` symbols (`nm`), every version.

## Mutation campaign

Run on scratch clones of `86bad70` (never on the worktree); every edit pattern was required to match
exactly once. Each mutant was rebuilt and checked with both boundary builds (C mutants) and the full
AUnit suite against pristine oracle files. A mutant is KILLED only if it built, ran and a named
check failed. The table is completed in the follow-up commit once every mutant has run.

MUTATION_ROWS

## Memory

Peak RSS (direct probe, 10 iterations, `setNumThreads(1)`), unmasked / all-included mask / mask with
a 1/3 x 1/3 hole. The probe owns the caller mask, so the masked columns include it:

| Geometry | 4.1.0 | 4.6.0 | 4.10.0 (system) | 5.0.0 |
|---|---|---|---|---|
| 640x480 | 42 / 42 / 42 MiB | 41 / 42 / 42 | 55 / 55 / 55 | 43 / 42 / 42 |
| 1920x1080 | 238 / 239 / 240 | 238 / 239 / 239 | 249 / 251 / 252 | 231 / 235 / 234 |
| 4096x4096 (2^24) | 1863 / 1880 / 1880 | 1864 / 1880 / 1880 | 1876 / 1892 / 1893 | 1817 / 1834 / 1834 |
| 16384x1024 | 1864 / 1880 / 1880 | 1864 / 1880 / 1881 | 1877 / 1893 / 1893 | 1818 / 1834 / 1832 |
| 1024x16384 | 1864 / 1879 / 1880 | 1863 / 1880 / 1879 | 1876 / 1893 / 1893 | 1816 / 1833 / 1833 |

The +16 MiB at `2^24` pixels is the caller-owned mask itself; native mask processing adds no
measurable peak.

## Version campaigns

Each campaign ran in an independent `git clone --no-local` of `86bad70` (the last code commit;
later commits change only documentation), rebuilt from source. Steps: development, validation and
release builds; repository checker; configuration tests; `sh -n` on every script; `git diff --check`;
`scripts/test.sh` (all oracles, AUnit, uninstrumented boundary); instrumented
`scripts/run_sanitizers.sh`; `homography_smoke`; `ecc_smoke`; examples build and `lk_synthetic`.
Pinned versions used `~/.cache/video-task017/install-<v>` (Release, core/imgproc/video, no
IPP/OpenCL/ITT) through `PKG_CONFIG_PATH`/`LD_LIBRARY_PATH`; 4.10.0 is the system package.

| OpenCV | Library loaded by `run_tests` | Builds D/V/R | Checker, config, `sh -n`, diff | AUnit run/ok/fail/err | Task 016 oracle | Task 017 oracle | Masked oracle | Sanitizer step | Exports / test symbols | Smokes, `lk_synthetic` |
|---|---|---|---|---|---|---|---|---|---|---|
| 4.1.0 | `install-4.1.0/lib/libopencv_video.so.4.1` | 0/0/0 | 0/0/0/0 | 198/198/0/0 | 28 OK / 9 EXC | 20 / 15 | 39 / 14 | rc 0, 0 findings | 25 / 0 | 0, 0, 0 |
| 4.6.0 | `install-4.6.0/lib/libopencv_video.so.406` | 0/0/0 | 0/0/0/0 | 198/198/0/0 | 30 / 7 | 20 / 15 | 39 / 14 | rc 0, 0 findings | 25 / 0 | 0, 0, 0 |
| 4.10.0 | system `libopencv_video.so.410` | 0/0/0 | 0/0/0/0 | 198/198/0/0 | 30 / 7 | 20 / 15 | 39 / 14 | rc 0, 0 findings | 25 / 0 | 0, 0, 0 |
| 5.0.0 | `install-5.0.0/lib/libopencv_video.so.500` | 0/0/0 | 0/0/0/0 | 198/198/0/0 | 28 / 9 | 19 / 16 | 39 / 14 | rc 0, 0 findings | 25 / 0 | 0, 0, 0 |

**Checks in every row:** each oracle printed its own `CV_VERSION`; `ldd` showed the selected
library; zero `AddressSanitizer`, `runtime error`, `LeakSanitizer` or `SUMMARY:` lines. The masked
oracle files equal the safety-gate runs byte for byte.

**Instrumentation scope.** `run_sanitizers.sh` instruments the **binding** (Video shim, both boundary
builds and every direct oracle) and links the uninstrumented Release OpenCV. **Instrumented upstream
OpenCV** was used separately for the safety-gate probes (above). These are distinct campaigns.

The README example (`Align_With_Mask` plus the seeded call) was compiled with `-gnatwa -gnatwe` and
run against 4.10.0 on a 960x720 pair: H02 2.016, H12 1.009 (identity) and 2.017 / 1.021 (seeded).

## Remaining limitations

* **Masks are caller policy.** The binding does not detect moving objects or unreliable pixels and
  cannot tell whether a mask is in the right frame.
* **Finite does not mean correct.** Very small surviving support (for example 12x12) and narrow
  bands can return finite, wrong matrices with high correlation.
* **Marginal supports are version-dependent** (success versus NaN/lambda exceptions); see the
  contract's table.
* **IEC 60559 reliance** as in Tasks 016/017.
* **Windows.** The new code has not run on Windows; it first runs in the push-to-main workflow.
  Hosted CI for the PR head is reported in the PR, not pre-claimed here.
