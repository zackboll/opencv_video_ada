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
