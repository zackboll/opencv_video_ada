# Task 012 — ECC Euclidean translation and rotation alignment

Branch `feature/012-ecc-euclidean`, from main `576e2d61cd87f607aa1c47a901b51fb688c656ac`
(Task 011 merge of PR #11, reviewed feature head
`2357238dab85dac325cd6a9f965056b17325784e`). Crate version stays `0.1.0-dev`; Core pin
stays `4da9d35ea21e1b2efe96296243ea668b488c6326`; no new dependency; CI topology
unchanged (Windows stays push-to-main only).

## Scope

Bind `cv::findTransformECC` with `MOTION_EUCLIDEAN` as
`OpenCV.Video.Estimate_ECC_Euclidean`, one new C export
(`opencv_video_estimate_ecc_euclidean`) and one fixed-layout 24-byte C result. The Task 011
`Estimate_ECC_Translation` API, its C export and its tests are unchanged.

Out of scope: affine and homography models, masks, caller-supplied initial warps, image
warping, multiscale ECC, any public warp-matrix type.

## Public API

```ada
subtype ECC_Euclidean_Options is ECC_Translation_Options;

type ECC_Euclidean_Result is record
   Displacement     : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
   Rotation_Radians : OpenCV.Float64_Value := 0.0;
   Correlation      : OpenCV.Float64_Value := 0.0;
end record;

function Estimate_ECC_Euclidean
  (Template_Image : OpenCV.Core.Mat;
   Moving_Image   : OpenCV.Core.Mat;
   Options        : ECC_Euclidean_Options := (others => <>))
   return ECC_Euclidean_Result;
```

The options subtype reuses Task 011's validated record: `Maximum_Iterations` 1 .. 100
(default 50), `Epsilon` finite in `1.0E-8 .. 0.1` (default 0.0001). Images: nonempty 2-D
UInt8 C1, identical geometry, rows and columns 32 .. 16,384, at most 2**24 pixels; Core
Regions (including non-contiguous ones) are accepted. Anything else raises `OpenCV_Error`.

## Transformation convention

```text
moving_x = cos(theta) * template_x - sin(theta) * template_y + Displacement.X
moving_y = sin(theta) * template_x + cos(theta) * template_y + Displacement.Y
```

`theta = Rotation_Radians`; X right, Y down. The matrix maps **template coordinates to
moving coordinates**. To resample the moving image into the template frame, apply this
same matrix with `WARP_INVERSE_MAP`; never negate the angle or translation for that
reason. Rotation is about the coordinate origin `(0,0)`, the top-left pixel, so a rotation
about the image centre `(cx,cy)` plus a centre displacement `(dx,dy)` has

```text
tx = cx - (cos(theta)*cx - sin(theta)*cy) + dx
ty = cy - (sin(theta)*cx + cos(theta)*cy) + dy
```

and `Displacement` is `(tx, ty)`, not `(dx, dy)`. `Rotation_Radians` is `atan2(w10, w00)`
of the returned warp, in `[-pi, pi]`; it is an image-alignment angle, not a heading, yaw
or aircraft attitude.

## Contract

* `Correlation` is the value `findTransformECC` returns. Upstream computes it at the start
  of the final iteration, before the final update, so it can describe the warp one step
  earlier than the returned matrix. No threshold, clamp or confidence is applied.
* A returned result means the native optimizer returned and the returned warp is a
  finite rotation. It does not mean the images are aligned: ECC is a local optimizer and
  can end at a wrong finite optimum. The upstream update decodes the previous angle with
  `asin`, which cannot represent rotations beyond a quarter turn, and the practical basin
  is a few degrees (`docs/ecc-euclidean-source-contract.md`).
* Native nonconvergence and every other native failure raise `OpenCV_Error`. No identity
  result is substituted.
* The caller's images and their parents are not modified. The result is a plain value;
  no native object outlives the call.

## Native ABI

`opencv_video_estimate_ecc_euclidean(template, moving, maximum_iterations, epsilon,
result)` with `opencv_video_ecc_euclidean_result { float displacement_x; float
displacement_y; double rotation_radians; double correlation; }`: offsets 0, 4, 8, 16,
size 24, alignment 8 (static assertions in the header test and the boundary test; the Ada
`Convention => C` mirror is in `opencv-video-internal-c_api.ads`). The production ABI is
18 exports. `*result` is written only after every native call and validation step has
succeeded.

## Qualification

See `docs/ecc-euclidean-source-contract.md` (upstream review, matrix validation bounds,
measurements, convergence basin, memory) and `docs/task012-qualification.md` (campaign
results).
