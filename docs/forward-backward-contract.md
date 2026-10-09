# Composed forward/backward contract

This is thick Ada composition of the qualified [native PyrLK contracts](pyrlk-source-contract.md),
not a single OpenCV API and not a second Lucas-Kanade implementation. Neither
production C++ nor the five-export C ABI changes in Task 003.

Task 008 extends the identical diagnostic semantics to owned `PyrLK_Pyramid`
sources, with unseeded-forward and seeded-forward overloads. A private generic
shares the existing compaction, saved-index mapping and checked distance logic
between raw images and limited owned pyramids. No additional native export is
introduced; the Task 007 production ABI remains fourteen exports. Both legs
dispatch to existing qualified pyramid tracking operations, which pass stored
derivative-interleaved vectors directly to LK. The same two objects are reversed
for the seeded backward call; no pyramid is rebuilt. The backward destination
predictions are original previous-frame coordinates, never forward locations.
Pyramid geometry, build/track window, requested-depth and tracking-option checks
remain active for empty arrays; naturally truncated available depth is allowed.
Sources/Regions/parents can be mutated and finalized after construction. Repeated
sequential use is supported; concurrent shared use is not promised.

The forward pass uses existing flags 0, or only `OPTFLOW_USE_INITIAL_FLOW` for
caller predictions. Forward status-success entries alone form the backward
source array. A parallel original-point array seeds backward destinations with
P0, while sources are P1 in the next frame. An explicit saved-index array maps
compact backward entries to the input Ada range. Failed forward entries never
run backward LK. No-success inputs skip the backward call entirely. Empty arrays
still validate thresholds and the ordinary image/tracking/seed-count contract.
For owned sources this includes the pyramid compatibility contract too.

The successful-subset count is <= input length. A Natural compact counter starts
at zero and increments once per successful source, never exceeding that count.
Saved source indices are copied directly, never derived from compact offsets.
Loops use array ranges and never compute `Points'Last + 1`; even a one-element
array at `Positive'Last` works. Tests check interleaved indices 7/8/9/10/11 against
separate one-point calls and against independently compacted native results.
These straightforward invariants do not introduce a SPARK-specific helper.

## Arithmetic and classification

Round trip = `sqrt((Float64(recovered.x)-Float64(original.x))**2 +
(Float64(recovered.y)-Float64(original.y))**2)` in pixels, validated before
conversion to the returned Float32 distance. The threshold comparison uses that
**returned Float32 value**, so equality to a measured diagnostic is accepted.
No photometric error participates in classification.

Input coordinates obey |coordinate| <= 2**29. If both endpoints are in that
domain, each difference is <= 2**30 and the sum of squares is <= 2**61.
The native success contract guarantees finite Float32 outputs but does not bound
them to the input domain. Even allowing any finite binary32 endpoint, each
absolute difference is < 2**129 and the sum of squares < 2**259, far below
binary64's approximately 2**1024 overflow boundary. Conversion happens **before**
subtraction; no binary32 squared intermediate exists. Distance is checked finite,
nonnegative and <= Float32_Value'Last before conversion; otherwise `OpenCV_Error`,
not saturation. A forward output used as backward input also undergoes ordinary
PyrLK coordinate validation. There is no relaxed native safety domain.

`Consistent = Forward.Tracked AND Backward_Tracked AND valid distance AND
Round_Trip_Error <= Maximum_Round_Trip_Error`. Both statuses are required: absent
recovery returns original point/zero distance with both backward status and
consistency false. The caller chooses the finite nonnegative pixel threshold;
zero is legal. Low round trip cannot prove correct physical correspondence,
particularly for repetitive/ambiguous texture.

## Deterministic experiments

96x96 UInt8 texture: `(row*17 + col*29 + (row*col)%251)%256`; shifted destinations
are zero-filled. Points: (25,25), (45,32), (60,50), (35,65). Windows 21x21,
COUNT|EPS (30,.01), min eigenvalue 1e-4.

- Good (2,1): Max_Level=1, unseeded forward, seeded backward. Maximum measured
  round trip <0.003 pixels. Larger pyramid levels can reach ambiguous matches
  on this high-frequency texture; specifying level 1 selects the experimentally
  verified accuracy fixture rather than hiding poor convergence in a tolerance.
- Good (12,7): Max_Level=0, forward predictions P0+(12.25,6.75), backward
  predictions P0. Measured round trips 0.0014..0.00291 pixels, <0.05 threshold.
- Poor (12,7): Max_Level=0, unseeded forward. Point (25,25) reports forward and
  backward success but recovers approximately (26.0947,22.5643): **2.6704 pixels**,
  rejected at 1 pixel. This is real native convergence into an incorrect patch.
- Backward unavailable: same texture -> all-zero image, Max_Level=0, forward
  predictions P0+(12.25,6.75). Forward succeeds but backward has no textured
  source gradient and fails. Original recovery/zero diagnostic remain rejected.
- Mixed mapping: indices 7/9/11 contain (25,25)/(45,32)/(60,50), with distant
  sources (-1000,-1000)/(1000,1000) at 8/10. Seeds for 7/11 are correct-offset;
  index 9 deliberately predicts P0+(32,20), giving ~2.675..2.678 pixels error.
  Good, inconsistent and forward-failed entries coexist with nonadjacent successes.

These semantic fixtures were experimentally checked on 4.1.0, 4.10.0 and 5.0.0
before fixing assertions. There are no per-version thresholds. Good cases use
0.05 pixels (substantial margin); the poor case requires >2 pixels. Same-build
native/Ada oracle comparisons use 1e-5 pixels for coordinates/distance; identical
inputs exercise identical native paths, and Float32 distance rounding is much
smaller at these magnitudes. This is not a cross-version bitwise-equality promise.

The direct oracle links only OpenCV, never Video or the Core bridge. It writes
20 mapped entries (four scenarios, five points each); AUnit compares forward
status/location, backward status/recovery and distance. Failed native error/point
slots are never read. The standard test script generates results before AUnit;
for standalone AUnit execution generate them with
`alr -n exec -- sh scripts/run_forward_backward_oracle.sh` from the repository root
and optionally set `VIDEO_FORWARD_BACKWARD_ORACLE` to their absolute path.