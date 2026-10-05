# Task 001 — Validate the OpenCV Video / PyrLK bootstrap

## Goal

Qualify the generated `opencv_video_ada` bootstrap without broadening the API.
Stop at the review gate with an open, non-draft PR. Do not merge, tag, release,
amend, force-push, or enable auto-merge.

## Starting assumptions

The ZIP was generated to match the current binding-family conventions and Core
pin, but it has **not** been natively built by the generator. Treat every native
claim as unproven until reproduced.

## Required work

1. Read `AGENTS.md`, `CLAUDE.md`, and all `.clinerules/*.md`.
2. Confirm the root/test/example Alire manifests resolve the same Core commit:
   `4da9d35ea21e1b2efe96296243ea668b488c6326`.
3. Inspect authoritative OpenCV 4.1.0, 4.10.0 and 5.0.0 declarations/source for
   `calcOpticalFlowPyrLK`, including input point schema, output schema, status/error
   semantics, image requirements, TermCriteria handling and version differences.
4. Review the C ABI and C++ shim for:
   - null/raw argument safety;
   - integer conversions and option bounds;
   - Core bridge lifetime/ownership;
   - exception containment;
   - failure-atomic output publication;
   - no C++/STL ABI leakage;
   - no undefined failed-track values entering the Ada API.
5. Build the public crate with warnings as errors.
6. Build and run the complete AUnit suite.
7. Build and run `examples/lk_synthetic`.
8. Run the actual-shim native boundary driver uninstrumented.
9. On Linux, run the actual shim under ASan/UBSan.
10. Validate the deterministic synthetic translation fixture against an
    independent direct-C++ OpenCV oracle if needed; do not loosen tolerances
    merely to make a failure green.
11. Verify non-contiguous image Regions if OpenCV's supported semantics allow it;
    add focused AUnit coverage if the public API is meant to support them.
12. Exercise OpenCV 4.1.0, 4.10.0 and 5.0.0 with the manual compatibility workflow
    or equivalent clean source-built environments.
13. Validate Linux and macOS PR jobs. Leave Windows as post-merge-only, matching
    repository policy.
14. Keep fixes as normal follow-up commits. Never amend or force-push.

## Review questions

- Is UInt8 C1 the correct deliberate bootstrap restriction for image input?
- Is `Float32_Point` the correct public value type for native CV_32FC2 points?
- Are failed-track `Next_Point=Previous_Point` and `Error=0` the right thick-Ada
  deterministic semantics?
- Is `Error` well-defined and finite for every successful status under the
  supported versions? If not, represent that state explicitly rather than
  silently normalizing it.
- Should points outside the image remain accepted and be resolved by native
  status, or should a portable public precondition be introduced? Base this on
  source/version evidence rather than intuition.

## Deliverable

Open one PR for Task 001 and stop at review with:

- starting `origin/main` SHA;
- branch name and final pushed SHA;
- local/remote/PR-head equality;
- clean worktree status;
- exact OpenCV/toolchain versions;
- AUnit registered/executed/passed counts;
- example result summary;
- raw-boundary and sanitizer results;
- compatibility-matrix results;
- any deliberate restrictions or unresolved portability findings;
- confirmation that no merge/tag/release/auto-merge/history rewrite occurred.
