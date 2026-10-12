# Task 011 qualification

ECC translation alignment (`OpenCV.Video.Estimate_ECC_Translation`) and the Windows
MSYS2 dependency-installation repair. Starting main
`130f196128c17a41d1eb0f6c4eec41015661bd74` (Task 010 merge of PR #10; reviewed
feature head `b1da1ffeb508ab76d475a78da723e8d09a974352`). Crate version `0.1.0-dev`,
Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all three Alire roots,
production ABI 17 exports (was 16), AUnit 118 registrations (was 107, +11).

Pending results are never counted as passes. Items marked **not claimed** were not
established.

## Windows MSYS2 correction

Task 010's post-merge Windows run
(<https://github.com/zackboll/opencv_video_ada/actions/runs/38157448587>) failed in
"Install MSYS2 native dependencies", before any build: `pacman -Sy --noconfirm --needed`
refreshed the database without upgrading installed packages, so installing
`libgcc`/`libstdc++` 16.2.0-5 broke `cc-libs`, which required 16.2.0-4 (the same
applied to `libgfortran`/`libquadmath`). The error text is from the task statement; I
could not retrieve the log text with `gh`, only the failing step name. The
`alire-project/setup-alire` source I read only toggles `msys2.do_not_install`; it
performs no package upgrade, so the correction belongs in this workflow.

Change (commit `fd0f369`): a separate `pacman -Syu --noconfirm` step before the
install, and `-Syu` for the install. No `--nodeps`, `--overwrite`, downgrade,
version pin or retry. `check_repository.py` and `test_configuration.py`
(`WindowsWorkflowTests`) inspect the real workflow file and reject `-Sy` and those
flags; a temporary `-Sy` mutant failed both. The trigger is still push-to-main only.

**Not claimed:** that the workflow works on a Windows runner. Linux and macOS success
says nothing about it. An MSYS2 runtime upgrade can make pacman stop and ask for a
shell restart; if the first post-merge Windows run shows that, a further step is
needed. The real Windows gate is the post-merge run on `main`.

## Native evidence

Host: x86_64 (AMD Ryzen 9 7945HX, 32 threads, 85 GB), Linux 6.12. Upstream sources
for 4.1.0, 4.6.0, 4.10.0 and 5.0.0 were read from the exact tags
(`docs/ecc-source-contract.md`); 4.6.0 `ecc.cpp` is byte-identical to 4.10.0.

### Direction and residual

Content shifted by (+2,+1) gives (+2.01,+1.01) on 4.1.0/4.6.0/4.10.0 and
(+2.009,+1.005) on 5.0.0. Applying the estimate with `warpAffine(...,
WARP_INVERSE_MAP)` reduces the interior mean absolute difference from 13.68 to 0.000;
the negated translation leaves 25.9. Reversing template and moving negates the
displacement. Horizontal (3,0) gives (about 3.0, 0.0) and vertical (0,3) gives (about
0.0, 3.0), so the axes are not swapped.

### Correlation

At defaults the correlation for (2,1) is 0.99967. With one iteration it is 0.8747 at
(2.143,1.121); with two it is 0.9966 at (1.994,0.997). It changes between iterations
and is computed before the final update, so it need not equal the correlation of the
returned warp. Contrast changes (0.8x+20, 1.3x-10) still converge to about (2.01,1.01)
with correlation 0.9997 and 0.9994. It is not clamped or thresholded.

### Incorrect-but-finite and version-dependent outcomes

Fixture: the 128x128 analytic texture shifted by (6,4), defaults 50 / 1e-4.

| Version | Result |
| --- | --- |
| 4.1.0, 4.6.0, 4.10.0 | success: (44.31, 89.47), correlation 0.7204 (far from (6,4)) |
| 5.0.0 | `StsNoConv` ("correlation is going to be minimized") |

The 4.6.0 value comes from the oracle file generated inside the Debian 12 container.
On every version tested, constant images, constant versus texture, and pseudorandom
noise pairs raise; (12,7) and (40,30) raise; noise versus texture succeeds with
(-4.27,-0.64) and correlation 0.034; x-only stripes shifted by 3 succeed with (0,0)
and correlation 0.383. Because the 4.x (6,4) return is a success reported by the
native API with finite, schema-valid output, the binding returns it; it sets no
correlation threshold. Anyone using this result must treat success as "the optimizer
returned", not "the images are aligned".

### Convergence basin and epsilon

Shift (5,3) raises on 4.1.0, 4.10.0 and 5.0.0 at 100 iterations for both epsilon 0.1
and 1e-8 (4.6.0 was not probed for this shift). Shifts up to (4,3) converge. Epsilon
changes the result visibly for (4,2): the correlations differ between epsilon 0.1 and
1e-8, which `ECC_Epsilon_Sensitivity` uses.

### Memory and geometry

`ru_maxrss` of a child process running the default 50-iteration estimate on textured
images shifted by (2,1): 4096x4096 982 MB, 16384x1024 983 MB, 1024x16384 983 MB on
4.10.0 (4.1.0 968 MB, 5.0.0 922 MB at 4096x4096). 2**24 pixels took 2.5-3.0 s. This is
a measured resource requirement on this host, not a safety proof and not a guarantee
that another machine can allocate it; allocation failure is `std::bad_alloc` and
becomes `OpenCV_Error` (fault-injected). CI jobs do not allocate this much.

Ada and C++ enforce 32..16384 per side and 2**24 pixels identically. Over-limit images
(16385 columns or rows; 4097x4096 and 16384x1025 pixels, about 17 MB each) are rejected
by validation before native work (`ECC_Image_Validation`). 32x16384 and 16384x32 run
natively and are accepted (`ECC_Aspect_Ratios`, 4.1.0/4.10.0/5.0.0 probed directly).
Widened arithmetic: the Ada area check uses `Long_Long_Integer`, the shim uses `int64_t`.

## Test inventory

* AUnit: 118 registrations (107 retained unchanged + 11 ECC), enforced by
  `check_repository.py`. The ECC procedures: `ECC_Default_Options`,
  `ECC_Translation_Directions`, `ECC_Options_And_Repeatability`,
  `ECC_Epsilon_Sensitivity`, `ECC_Photometric`, `ECC_Regions_Lifetime`,
  `ECC_Image_Validation`, `ECC_Option_Validation`, `ECC_Direct_Oracle`,
  `ECC_Degenerate_And_Nonconvergence`, `ECC_Aspect_Ratios`.
* Independent oracle `tests/cpp/ecc_oracle.cpp` writes `obj/oracle/ecc-translation.txt`
  (`VIDEO_ECC_TRANSLATION_ORACLE`): 27 fixtures with the OpenCV version, expected
  success or exception, X/Y, correlation and fixture checksums. It links OpenCV only.
  The Ada test rebuilds every fixture independently (`tests/src/ecc_fixtures.adb`),
  compares checksums, then X/Y within 1e-5 and correlation within 1e-6 of the
  same-build native result (observed differences are zero). A missing, empty,
  truncated, count-mismatched or checksum-stale file fails `ECC_Direct_Oracle`; each
  was exercised. Existing oracle files and formats are unchanged.
* `tests/cpp/native_boundary.cpp` `run_ecc`: bit-exact comparison with a direct call;
  null/invalid arguments; option bounds; schema; nonconvergence; non-contiguous
  Regions; two distinct Core headers sharing one allocation; repeatability; byte-wise
  sentinel preservation; eight injected exceptions (before and after native
  execution; `bad_alloc`, `cv::Exception`, `std::exception`, unknown); six post-native
  corruptions (NaN/Inf warp, NaN/-Inf correlation, non-identity linear part). The
  hooks exist only in the `OPENCV_VIDEO_TEST_FAULTS` build.
* `tests/src/ecc_smoke.adb` and `tests/cpp/ecc_raw_oracle.cpp` stay as standalone
  evidence (bit-identical X, Y and correlation for the five shift cases).
* `ecc_probe.cpp` is the research probe behind `docs/ecc-source-contract.md`.

Tolerances: 0.05 px for the integer shifts (4.1.0, 4.10.0 and 5.0.0 worst error 0.0178
in the 128x128 probe; the 32x32 minimum is looser and is only checked to 0.1 px).

## Mutation campaign

Scratch clone only, never the feature tree; a build, oracle regeneration, AUnit and
the native boundary run per mutant. Script: not committed (scratch).

| Mutant | Killed by |
| --- | --- |
| M01 swap template/moving (shim) | 9 ECC AUnit tests incl. `ECC_Direct_Oracle`, `ECC_Translation_Directions`; boundary |
| M02 negate translation | same |
| M03 swap X/Y | same |
| M04 zero translation | same |
| M05 constant correlation | `ECC_Direct_Oracle`, `ECC_Epsilon_Sensitivity`, `ECC_Regions_Lifetime`, `ECC_Translation_Directions`; boundary |
| M06 remove native ECC call | 10 ECC tests incl. `ECC_Degenerate_And_Nonconvergence`; boundary |
| M07 ignore iteration count (shim) | `ECC_Direct_Oracle`, `ECC_Option_Validation`, `ECC_Options_And_Repeatability` |
| M08 ignore epsilon (shim) | `ECC_Direct_Oracle`, `ECC_Epsilon_Sensitivity` |
| M09 publish before validation | boundary only: "ECC corrupted native output was partially published" |
| M10 suppress nonconvergence | `ECC_Degenerate_And_Nonconvergence`, `ECC_Direct_Oracle`; boundary |
| M11 remove native schema check | boundary only: "bad moving accepted" (Ada preflight masks it in AUnit) |
| M12a remove finite-warp check | boundary only (corruption hook) |
| M12b remove finite-correlation check | boundary only (corruption hook) |
| M12c remove identity-linear check | boundary only (corruption hook) |
| M13 remove Ada depth/channel check | `ECC_Image_Validation` (after adding a preflight-specific assertion) |
| M14 ignore iterations (Ada) | `ECC_Direct_Oracle`, `ECC_Option_Validation`, `ECC_Options_And_Repeatability` |
| M15 swap roles (Ada) | 9 ECC tests |
| M16 negate X (Ada) | 9 ECC tests |

Defects found in the campaign itself, not in the binding:
* M13 **survived** the first run: native code rejects wrong depth too, so no test could
  distinguish an Ada preflight removal. Fixed by `ECC_Expect_Preflight_Error`, which
  requires the error not to carry the native-layer prefix; M13 then died.
* M09, M12a-c could not be killed by any real fixture because no real input drives the
  native call to a nonfinite result. A test-only post-native corruption hook (compiled
  only with `OPENCV_VIDEO_TEST_FAULTS`) was added to the boundary test. The first
  M12c text mutant did not compile (`-Werror=parentheses`); it was rewritten and re-run.
* The mutation script's first M06/M10 anchors were wrong (the variable had become
  non-const) and had to be corrected before those mutants were counted.

## Per-version local campaign

Run on independent `git clone --no-local` copies of code head
`650cc69d1c8e22c3f7f43231a3038bd2a098c05d` (no generated products copied; later commits
are documentation only). Each version used `scripts/test.sh` (build, all oracles, full
AUnit, native boundary), the ASan/UBSan/leak driver, the retained example, and
Development/Validation/Release builds with Validation AUnit. Loaded libraries were read
with `ldd` on the boundary binary and match the configured `Native_Version`; no system
OpenCV was substituted for the pinned 4.1.0 and 5.0.0 (private prefixes with
`PKG_CONFIG_PATH`/`LD_LIBRARY_PATH`; 5.0.0 uses `opencv5.pc`).

| OpenCV | Loaded `libopencv_video` | AUnit registered / run / passed / failed assertions / errors | test.sh | ASan+UBSan+leak | Example | Dev / Validation / Release |
| --- | --- | --- | --- | --- | --- | --- |
| 4.1.0 | `install-4.1.0/lib/libopencv_video.so.4.1` | 118 / 118 / 118 / 0 / 0 | exit 0 | exit 0, 0 findings | exit 0 | exit 0 each |
| 4.6.0 (Debian 12 container, distro `libopencv-dev`) | `/lib/x86_64-linux-gnu/libopencv_video.so.406` | 118 / 118 / 118 / 0 / 0 | exit 0 | exit 0, 0 findings | exit 0 | exit 0 each |
| 4.10.0 (system) | `/lib/x86_64-linux-gnu/libopencv_video.so.410` | 118 / 118 / 118 / 0 / 0 | exit 0 | exit 0, 0 findings | exit 0 | exit 0 each |
| 5.0.0 | `install-5.0.0/lib/libopencv_video.so.500` | 118 / 118 / 118 / 0 / 0 | exit 0 | exit 0, 0 findings | exit 0 | exit 0 each |

Every environment also ran the 27-fixture ECC oracle (recording its own version), the
retained PyrLK, forward/backward, trackability, Farneback and seeded Farneback oracles,
and the actual shim/Core boundary with the eight injected exceptions and six corruption
cases, in both the native and the sanitizer drivers. The `ECC_Direct_Oracle` AUnit
compared the binding with the same-build native result, including the version-specific
outcomes in the table above (for example (6,4) succeeding on 4.1.0/4.6.0/4.10.0 and
raising on 5.0.0).

The mutation campaign ran on 4.10.0 only, on an earlier head (`d2ec602`); M13 was
re-run on `09c0e52`. It was not repeated after the later boundary additions.

## Sanitizers

The Video shim, the Core shim and the test drivers are compiled with
`-fsanitize=address,undefined`; the OpenCV libraries are prebuilt and **not
instrumented**, so a clean result says nothing about upstream-internal undefined
behaviour. No sanitizer finding occurred in any version, including in the ECC
boundary, fault and oracle runs, so there is no ECC finding and no ECC waiver.
`KNOWN_UPSTREAM_UB_ACCEPTED` (Farneback, `docs/farneback-source-contract.md`) is a
separate, narrow, previously approved limitation of that function and does not extend
to ECC.

## Other gates (4.10.0 development tree)

Repository checker (17 ABI declarations/imports, 118 registrations, Core pin
`4da9d35ea21e` in all three roots, version `0.1.0-dev`), 12 configuration tests
(including the Windows workflow regression class), C11 header with warnings as errors
and a `sizeof == 16` probe, shell syntax of every script, `git diff --check`, and
`nm`: 17 exported `opencv_video_*` functions and no `corrupt`/`test_fault` symbols in
the production library.

## Not claimed

* Real Windows behaviour of the MSYS2 change (push-to-main only).
* That the 5.0.0, 4.6.0 or 4.1.0 builds are ASan/UBSan-clean inside OpenCV itself.
* Mutation results on 4.1.0, 4.6.0, 5.0.0.
* That the largest geometry allocates on every machine (about 1 GB measured here).
* Hosted CI results, which are recorded in the pull request.
