# Task 012 qualification — ECC Euclidean alignment

Starting `origin/main`: `576e2d61cd87f607aa1c47a901b51fb688c656ac` (Task 011 merge of PR
#11; reviewed feature head `2357238dab85dac325cd6a9f965056b17325784e`). Branch
`feature/012-ecc-euclidean`, worktree `/home/zboll/git/opencv/video-task012`.

Inventory: production ABI **18** exports (was 17); AUnit **129** registrations (118
retained unchanged + 11 Euclidean); crate version `0.1.0-dev`; Core pin
`4da9d35ea21e1b2efe96296243ea668b488c6326` in all three Alire roots; no new dependency.

## Task 011 baseline CI (actual results, read with `gh run view`)

| Run | Result |
| --- | --- |
| Pinned OpenCV compatibility 38190113614 (head `2357238`) | completed, **success** |
| Post-merge cross-platform 38190552249 (head `576e2d6`) | completed, **success** |
| Post-merge Windows 38190552280 (head `576e2d6`) | completed, **success** |

No Windows correction was needed, so no Windows change is in this branch. The Windows
result is Task 011's MSYS2 `pacman -Syu` provisioning on the merge commit only; it says
nothing about Task 012 code, which Windows CI (push-to-main only) has not yet run.

## What was added

* `OpenCV.Video.Estimate_ECC_Euclidean`, `ECC_Euclidean_Result`, and the subtype
  `ECC_Euclidean_Options is ECC_Translation_Options`. No public matrix type. The Ada
  preflight is the former Task 011 checks moved into one private
  `Validate_ECC_Arguments`; `Estimate_ECC_Translation` behaves as before.
* `opencv_video_estimate_ecc_euclidean` with
  `opencv_video_ecc_euclidean_result { float, float, double, double }`: offsets 0, 4, 8,
  16, size 24, alignment 8. `static_assert`s in `tests/cpp/native_boundary.cpp`; the Ada
  mirror is a `Convention => C` record. A C11 `_Static_assert` build with
  `-Wall -Wextra -Wpedantic -Werror` confirmed size 24 and offsets 8 and 16.
* The shim shares one `validate_ecc_inputs` (images + options) between the translation and
  Euclidean exports; each keeps its own motion constant and output validation.
* `nm -g --defined-only lib/libopencv_video_shim.a`: exactly 18 `opencv_video_*` text
  symbols, none matching `corrupt` or `test_fault`.

## Native review and measurements

See `docs/ecc-euclidean-source-contract.md`: four upstream tags read, the unchanged
`asin(mapPtr[3])` update and why it limits the reachable angle, the 3-parameter Jacobian,
returned-warp validation bounds (largest observed deviation 5.9e-8 against a 1e-5 bound,
on all of 4.1.0, 4.10.0 and 5.0.0), the angle basin (irregular, version-dependent, wrong
finite results at about 8 to 11 degrees and beyond, thrown otherwise) and memory.

Memory (4.10.0, peak `VmHWM`, 50 iterations): 4096x4096 translation 1,007,032 kB,
**Euclidean 1,334,432 kB**; 1024x16384 1,333,312 kB; 16384x1024 1,334,420 kB. The 2**24
pixel limit is unchanged; the figure is about 33% above translation, not equal to it.
Measured by `tests/cpp/ecc_euclidean_limits.cpp`, outside routine CI.

## Test evidence

* AUnit (11 new): `ECC_Euclidean_Defaults`, `_Identity_And_Translation`,
  `_Signed_Rotations`, `_Rotation_Translation`, `_Centered_Rotation`, `_Inverse_Warp`,
  `_Options_Sensitivity`, `_Regions_Lifetime`, `_Validation`, `_Degenerate_And_Limits`,
  `_Direct_Oracle`. The 118 earlier registrations are textually unchanged (the diff to
  `video_tests.adb` removes no line).
  * Rotations ±1 and ±3 degrees, with and without translation, to 0.005 rad and 0.1 px;
    reversed roles negate the angle; a centred rotation returns the matrix translation
    and two independently mapped points land where the analytic matrix sends them.
  * `ECC_Euclidean_Inverse_Warp`: the returned matrix, applied as `W*p` (what
    `WARP_INVERSE_MAP` does) with bilinear sampling, reduces the interior mean absolute
    difference from more than 10 to under 2 and by more than 8x; the negated
    angle/translation does not align (more than 4x worse).
  * Regions: genuine non-contiguous Regions equal the contiguous clones exactly; parents
    unchanged; the result is used after the Regions and parents are finalized.
* Independent oracle `tests/cpp/ecc_euclidean_oracle.cpp` →
  `obj/oracle/ecc-euclidean.txt` (`VIDEO_ECC_EUCLIDEAN_ORACLE`), 24 fixtures, links
  OpenCV only. It records the six native warp entries, tx, ty, `atan2` angle, rho,
  OK/EXC, the native version and fixture checksums. The Ada test rebuilds every fixture
  independently, compares checksums, then reconstructs the 2x3 matrix from the returned
  value against the six native entries (1e-6 / 1e-5), the signed angle modulo 2 Pi, and
  X/Y/rho. The loader rejects missing, truncated, count-mismatched, wrong-magic or
  checksum-stale data. `obj/oracle/ecc-translation.txt` is unchanged.
* Native boundary (`run_ecc_euclidean`): bit-exact comparison with a direct call for nine
  signed rotation/translation cases; repeatability; reversed roles; same handle; two
  headers sharing storage; null template/moving/output; invalid iterations and epsilons;
  wrong type, channels, geometry and envelope; options forwarded; nonconvergence; centred
  rotation; non-contiguous Regions; byte-wise sentinel preservation after every failure.
* Fault build (`OPENCV_VIDEO_TEST_FAULTS`, separate binary): `bad_alloc`, `cv::Exception`,
  `std::exception` and an unknown exception before and after the native call (8 cases);
  post-native corruption rejected with the result bytes unchanged: NaN X, Inf Y, NaN rho,
  -Inf rho, scaled `a`, asymmetric `b`, a uniformly scaled rotation, a reflection, a NaN
  rotation entry, asymmetric `c`, and a `3e-6` `a != d` that only the strict symmetry
  check rejects. Two valid rigid warps at +120 and -150 degrees are published with the
  `atan2` angle (an `asin` decode would return the wrong value).

## Mutation campaign

Scratch clone `/tmp/t12-mut` only (never the feature tree; nothing committed). Each
mutant was compiled, the oracles regenerated, the full AUnit run and the native boundary
plus fault binary run (`run_sanitizers.sh native`). M01-M15 ran on `56b476a`; M13b, M14c
and M16 were re-run on `0b91342` after corruption mode 13 was added. 4.10.0 only. A mutant
counts as killed only when it compiled, ran and a named check failed.

| Mutant | AUnit | Native boundary / fault binary |
| --- | --- | --- |
| M01 `MOTION_EUCLIDEAN` -> `MOTION_TRANSLATION` | killed (8 tests, incl. `_Direct_Oracle`, `_Signed_Rotations`) | killed: "Euclidean shim differs from direct native call" |
| M02 negate angle | killed (8) | killed |
| M03 swap X/Y translation | killed (7) | killed |
| M04 negate translation | killed (7) | killed |
| M05 angle zero | killed (8) | killed |
| M06 decode angle from `w[2], w[0]` | killed (9) | killed |
| M07 `asin(w[3])` instead of `atan2` | **survived** | killed (bitwise comparison with the direct call) |
| M08 ignore iteration option | killed (2: `_Options_Sensitivity`, `_Direct_Oracle`) | killed: "iteration option not forwarded" |
| M09 ignore epsilon | killed (2) | killed: "epsilon option not forwarded" |
| M10 swap template and moving | killed (9) | killed |
| M11 publish before validating | **survived** | killed: "partially published" |
| M15 suppress native nonconvergence | killed (2) | killed: "constant images did not fail" |
| M17 symmetry check only (no unit/orthogonality/determinant) | **survived** | killed: "corrupted native output accepted" |
| M13b remove strict `a = d`, `b = -c` check | **survived** | survived until mode 13 was added; then killed |
| M16 determinant check only | **survived** | survived until mode 13 was added; then killed |
| M14c remove every rigid-matrix check | **survived** | killed (modes 5-10 and 13) |
| M12 remove finite-angle check | **survived** | **survived** |
| M13a remove orthogonality term | **survived** | **survived** |
| M14a remove unit-length check | **survived** | **survived** |
| M14b remove determinant check | **survived** | **survived** |

M07, M11, M13b, M14c, M16 and M17 cannot be reached with an ordinary image pair: real
angles are small, where `asin` and `atan2` agree far below the oracle tolerance, and no
real native call returns an invalid matrix. They are killed only by the separate fault
binary, not by AUnit. The +120 and -150 degree valid-warp cases there would separate
`asin` from `atan2` independently of the bitwise comparison.

Survivors of the whole suite are M12, M13a, M14a and M14b. I believe each is redundant
rather than untested: with `a = d` and `b = -c` enforced, `a*b + c*d` is `a*(b + c)` and
so is bounded by the symmetry tolerance (M13a); `a*a + c*c - 1` equals `det - 1`, so
unit length and the determinant bound imply each other (M14a, M14b); and the angle is
finite whenever the four entries are (M12). That argument is by reading the algebra, not
by a test, and the survivors are recorded rather than closed. I added no test for them.

Gaps the campaign found and fixed in the tests:
* M13b and M16 survived the first fault build: no corruption mode isolated the strict
  symmetry check. Mode 13 (`a != d` by 3e-6, inside every other bound) was added and both
  are now killed.
* M13b, M14b and M14c did not compile at first (`-Werror=unused-variable`); they were
  rewritten and re-run.


