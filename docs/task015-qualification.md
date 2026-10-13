# Task 015 qualification — caller-seeded ECC affine alignment

Starting `origin/main`: `35c76239820b4b33737bc1588597ff80b12fce8e` (PR #14 merge). Branch
`feature/015-seeded-ecc-affine`. Inventory at start: 20 production exports, 153 AUnit
registrations, crate `0.1.0-dev`, Core pin `4da9d35ea21e1b2efe96296243ea668b488c6326` in all three
Alire roots (unchanged). Inventory now: **21 exports, 163 registrations**.

## Task 014 post-merge CI (read at the start of Task 015)

| Run | Workflow | Head SHA | Outcome |
|---|---|---|---|
| 38200199976 | Pinned OpenCV compatibility | `2250490` (reviewed feature head) | success (4.1.0, 4.10.0, 5.0.0 jobs) |
| 38322384230 | Cross-platform Video | `35c7623` | success (repository-checks, linux, macos, linux-sanitizers) |
| 38322384218 | Windows post-merge Video | `35c7623` | in progress when first read; **success** when re-read later in this session (job `windows`, no failed step) |

No baseline correction was required. Windows stays push-to-main only; it was not dispatched and no
Windows PR CI was created.

## Native source review

See `docs/ecc-affine-seeded-source-contract.md`. In short: a nonempty caller warp is used as given
(only an *empty* matrix becomes the identity), it is consumed by the first `warpAffine`, the six
updates are applied in place in update-vector order while storage stays row-major, and the
`COUNT | EPS` policy and pre-update correlation are unchanged. Sources were read at **4.1.0 and
5.0.0 (local trees)**. The 4.6.0 and 4.10.0 *sources were not re-read for this task*: 4.10.0 is
covered by behavior only, and 4.6.0 by the container behavior run plus Task 014's source review.

## Seed domain

Accepted: `M00 M01 M10 M11` finite with `abs <= 4`; `M02 M12` finite with `abs <= 2**20`. The
proposed candidate was **kept unchanged**. Measured with the research probe
(`ecc_affine_seeded_probe.cpp domain`) on 4.1.0, 4.10.0 and 5.0.0, and by the oracle's fixtures:

* Accepted-limit, singular, rank-1, near-singular, tiny-scale, shear-4 and large-translation
  seeds: native ECC raised (no crash, hang or sanitizer finding).
* Reflection seed `[-1 0 0; 0 1 0]`: finite result, rho 0.821 (wrong but finite) on all versions.
* Anisotropic seed `(4, 0.01)`: finite meaningless result on 4.1.0 (rho 0.068) and 4.10.0
  (rho 0.030), exception on 5.0.0 — a documented version difference.
* No input determinant or conditioning rule was warranted by the measurements.

## Seed consumption (primary requirement)

Identity initialization **raises** on every pair below (4.1.0, 4.10.0, 5.0.0 probe; AUnit and
boundary tests on all four versions). A near-correct seed converges and recovers the known matrix.
Values are from 4.10.0; 4.1.0 and 5.0.0 agree to about the third decimal.

| Pair (truth) | Near seed | Max coefficient error | Residual seed -> refined |
|---|---|---|---|
| shear 0.12 `[1 .12 1; -.05 1 -1]` | `[1 .10 .5; -.04 1 -.7]` | 0.053 | 10.89 -> 0.29 |
| scale `[1.12 .02 2; -.01 .88 1]` | `[1.10 .01 1.5; 0 .90 .8]` | 0.031 | 14.45 -> 0.28 |
| combined `[1.08 .08 2; -.04 .94 -1]` | `[1.06 .07 1.7; -.03 .95 -.8]` | 0.063 | 13.11 -> 0.29 |
| shear 0.2 | `[1 .18 0; 0 1 0]` | 0.031 | 7.48 -> 0.21 |
| reflection `[-1 0 127; 0 1 0]` | `[-.98 .01 126; 0 1 .1]` | 0.007 | 5.36 -> 0.00 |
| strong `[1.2 .1 5; -.1 .8 -3]` | `[1.18 .09 4.5; -.09 .82 -2.7]` | 0.017 | 16.75 -> 2.83 |

The shear-0.2 row shows the worst-coefficient error is not monotone (0.02 -> 0.031, the
translation absorbs residual); the residual still falls, and the test does not claim monotone
coefficient improvement there. Reflection recovery worked on all probed versions but only for a
near-correct seed; it is not a general guarantee. One, three and fifty iterations differ
(correlation 0.876 / 0.994 / 0.994).

Poor seeds are not rescued: `[0.8 -.2 6; .2 1.25 -5]` raises on all versions; the asymmetric seed
`[1 .3 4; -.25 1 -2]` ends at a finite wrong matrix on the scale pair (rho about 0.41-0.43) and
raises on the others. The identity seed equals the identity-initialized operation byte for byte,
including failures.


## Oracle

`tests/cpp/ecc_affine_seeded_oracle.cpp` links OpenCV only and writes 34 fixtures (seed, status, six
outputs, correlation, checksums); the Ada verifier rebuilds each image and seed independently. The
integrity test rejects, for the expected reason: missing, empty, truncated (two forms), wrong
format/native version, wrong count, renamed fixture, altered checksums, altered seed M00/M01/M10/
M02/M12, swapped seed M01-M10 and M02-M12, swapped output M01-M10 and M02-M12, altered output M00
and correlation, and success/exception/unknown outcome misclassification.

## Memory (direct native, VmHWM, 4.10.0, shift pair)

1024x1024: identity 99 MB, seeded 100 MB. 2048x2048: identity 334 MB, seeded 333 MB. The starts are
equivalent within noise. Task 014's 1.2-1.3 GB at 2^24 pixels was not re-measured, and no large
workload was added to CI.

## Native campaigns (head `aa3636a`, serial builds)

| OpenCV | Environment | Dev/Valid/Release | AUnit reg/run/ok/fail/err | ASan+UBSan+leaks | Example |
|---|---|---|---|---|---|
| 4.1.0 | pinned local install | exit 0 each | 163/163/163/0/0 | exit 0 | exit 0 |
| 4.6.0 | Debian 12 container, copy of `aa3636a` | exit 0 each | 163/163/163/0/0 | exit 0 | exit 0 |
| 4.10.0 | local system | exit 0 each | 163/163/163/0/0 | exit 0 | exit 0 |
| 5.0.0 | pinned local install | exit 0 each | 163/163/163/0/0 | exit 0 | exit 0 |

`Native_Version` matched in each configured install, and the oracle PASS lines for the loaded
version were present. The sanitizer script instruments the shim, boundary, fault binary and every
oracle; the prebuilt OpenCV libraries are not instrumented. An earlier 4.1.0 run failed one
assertion: the correlation bound `<= 1.0` was exceeded by Float32 rounding (1.000000085 for an exact
reflection). The bound became `<= 1.000001` with a comment; a test defect, not a binding defect.
Production archive: exactly 21 `T opencv_video_*` symbols and no test hook. Only the 4.6.0 run used
a separate copy of the committed head; the other three rebuilt in the worktree after removing the
ignored `obj` and `config` directories. A full independent clean-clone run on all versions was not
done.

## Mutation results (scratch clones of `aa3636a`; nothing committed)

A mutant counts as KILLED only when it compiled and a named test failed (AUnit and/or the native
boundary, including the fault build). First-pass harness defects: mutants 3-8 did not compile
(`-Werror=misleading-indentation`), mutant 18 hit an unused-variable error, and mutants 12-14 had
patched the earlier Euclidean code instead of the affine path. The harness was corrected and those
were rerun; only the rerun results count.

| # | Mutation | Result | Failing evidence |
|---|---|---|---|
| 1 | ignore initial transform | killed | five seeded AUnit tests; boundary "moderate shear" |
| 2 | seed replaced by identity | killed | seeded AUnit tests; boundary |
| 3 | swap M01/M10 | killed | seeded AUnit + oracle; boundary "moderate shear" |
| 4 | swap M02/M12 | killed | seeded AUnit + oracle; boundary "seeded export differs" |
| 5 | swap M00/M11 | killed | seeded AUnit; boundary "nonuniform scale" |
| 6 | negate shear | killed | seeded AUnit; boundary "moderate shear" |
| 7 | negate translation | killed | seeded AUnit; boundary |
| 8 | ignore initial scale | killed | seeded AUnit; boundary "nonuniform scale" |
| 9 | MOTION_EUCLIDEAN | killed | affine and seeded AUnit; boundary |
| 10 | return seed, no optimization | killed | AUnit; boundary |
| 11 | return initial correlation | killed | `ECC_Affine_Seeded_Correlation_Ignored` and others |
| 12 | validate ignored correlation | killed | `ECC_Affine_Seeded_Correlation_Ignored` |
| 13 | ignore iteration count | killed | affine and seeded oracle, options AUnit tests |
| 14 | ignore epsilon | killed | affine and seeded oracle AUnit; boundary |
| 15 | reverse roles | killed | AUnit; boundary "translation" |
| 16 | publish before validation | killed | boundary only: "constant failure changed result" |
| 17 | remove seed finiteness check | killed | boundary only: "invalid seed coefficient accepted" |
| 18 | remove translation bound | killed | boundary only: "invalid seed coefficient accepted" |
| 19 | remove final finiteness check | killed | fault build only: "nonfinite native output accepted" |
| 20 | suppress nonconvergence | killed | AUnit and boundary "constant images accepted" |

No survivors. In mutant 17 the Ada preflight makes the same rejection unreachable from AUnit, so
only the raw C boundary can see it. Mutants 13 and 14 also fail Task 014 tests because the helper
is shared.
