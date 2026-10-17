with OpenCV.Core;
with Ada.Finalization;
with System;

package OpenCV.Video is
   --  Initial bootstrap slice: sparse pyramidal Lucas-Kanade tracking.
   --  OpenCV.Core remains the sole owner of Mat wrappers.

   type Tracking_Point_Array is array (Positive range <>) of OpenCV.Float32_Point;

   type PyrLK_Options is record
      --  Window dimensions: 3 .. 255; Max_Level: 0 .. 30;
      --  Maximum_Iterations: 1 .. 100; Epsilon: finite, 0 < Epsilon <= 10;
      --  Min_Eigenvalue_Threshold: finite, 0 .. Float32_Value'Last.
      --  Values outside these ranges raise OpenCV_Error, not silent clamping.
      Window_Size              : OpenCV.Size := (Width => 21, Height => 21);
      Max_Level                : Natural := 3;
      Maximum_Iterations       : Positive := 30;
      Epsilon                  : OpenCV.Float64_Value := 0.01;
      Min_Eigenvalue_Threshold : OpenCV.Float64_Value := 1.0E-4;
   end record;

   type Point_Track is record
      Previous_Point : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
      Next_Point     : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
      Tracked        : Boolean := False;
      Error          : OpenCV.Float32_Value := 0.0;
   end record;

   type Point_Track_Array is array (Positive range <>) of Point_Track;

   --  Binding-owned optimization object, not an OpenCV class. Sequential reuse
   --  is supported; no general concurrent-use guarantee is made. No shallow copy.
   type PyrLK_Pyramid is limited private;
   type PyrLK_Pyramid_Options is record
      Window_Size : OpenCV.Size := (Width => 21, Height => 21);
      Max_Level   : Natural := 3;
   end record;
   --  Nonempty 2-D UInt8 C1, including strided Regions. Window 3..255,
   --  requested level 0..30. Always owns image and signed dx/dy derivatives;
   --  no source allocation is retained. Construction may consult Region parent
   --  pixels for borders, but neither Region nor parent is needed afterward.
   function Build_PyrLK_Pyramid
     (Image : OpenCV.Core.Mat;
      Options : PyrLK_Pyramid_Options := (others => <>)) return PyrLK_Pyramid;
   function Is_Empty (Pyramid : PyrLK_Pyramid) return Boolean;
   --  All metadata queries raise OpenCV_Error on an empty/default object.
   function Requested_Max_Level (Pyramid : PyrLK_Pyramid) return Natural;
   function Available_Max_Level (Pyramid : PyrLK_Pyramid) return Natural;
   function Build_Window_Size (Pyramid : PyrLK_Pyramid) return OpenCV.Size;
   --  Ordinary photometric mode only. Same geometry, exact build/track window
   --  equality, and tracking level <= each requested build level are required.
   --  Effective level is min(tracking request, both available levels).
   --  Preserves Points'Range and the ordinary success/failure value contract.
   function Track_PyrLK
     (Previous_Pyramid : PyrLK_Pyramid;
      Next_Pyramid : PyrLK_Pyramid;
      Points : Tracking_Point_Array;
      Options : PyrLK_Options := (others => <>)) return Point_Track_Array;

   --  Initial predictions correspond by iteration position, not array index.
   --  Equal lengths are required; different lower bounds are supported. Seeds
   --  are privately copied, refined with flag 4, and never used for failures.
   function Track_PyrLK
     (Previous_Pyramid    : PyrLK_Pyramid;
      Next_Pyramid        : PyrLK_Pyramid;
      Points              : Tracking_Point_Array;
      Options             : PyrLK_Options := (others => <>);
      Initial_Next_Points : Tracking_Point_Array) return Point_Track_Array;

   --  Track Points from Previous_Image into Next_Image using
   --  cv::calcOpticalFlowPyrLK.
   --
   --  Bootstrap contract:
   --    * images are nonempty, 2-D UInt8 C1 Mats of identical geometry;
    --    * Points are finite Float32 coordinates with absolute value <= 2**29;
    --      they need not be inside the image (native padded-border semantics);
    --    * non-contiguous Core Regions are supported; coordinates are local to
    --      the Region and native pyramid borders may consult its parent pixels;
   --    * input array bounds are preserved in the returned Track array;
   --    * no OPTFLOW_USE_INITIAL_FLOW or LK_GET_MIN_EIGENVALS flag is used;
   --    * when Tracked=False, Next_Point is reset to Previous_Point and
   --      Error is reset to 0.0 instead of exposing native undefined values;
   --    * images and Points are not modified.
    --    * successful Error is finite nonnegative mean patch L1 difference;
    --      an invalid native successful result raises OpenCV_Error, never
    --      silently changes status or error;
    --    * empty Points return the same null range, after image/option checks;
    --    * images exceeding safe native signed stride/padding arithmetic are
    --      rejected with OpenCV_Error before the native call.
   function Track_PyrLK
     (Previous_Image : OpenCV.Core.Mat;
      Next_Image     : OpenCV.Core.Mat;
      Points         : Tracking_Point_Array;
      Options        : PyrLK_Options := (others => <>)) return Point_Track_Array;

   --  Use caller predictions instead of previous points as the initial next
   --  estimates (native OPTFLOW_USE_INITIAL_FLOW only). The two arrays must
   --  have equal lengths; correspondence is by iteration position, not index.
   --  Results preserve Points'Range. Seeds obey the same coordinate bound as
   --  Points, need not be inside the image, and are never modified. All other
   --  validation, successful error and deterministic failure contracts above
   --  apply. Empty pairs validate images/options and preserve the null range.
   --  A prediction aids convergence; it does not guarantee a correct match.
   --  Options precedes the required seed parameter to keep existing positional
   --  option aggregates unambiguous. Use Initial_Next_Points => Predictions
   --  to omit Options, or pass Options then Predictions positionally.
   function Track_PyrLK
     (Previous_Image      : OpenCV.Core.Mat;
      Next_Image          : OpenCV.Core.Mat;
      Points              : Tracking_Point_Array;
      Options             : PyrLK_Options := (others => <>);
      Initial_Next_Points : Tracking_Point_Array) return Point_Track_Array;

   function Successful_Count (Tracks : Point_Track_Array) return Natural;

   type Trackability_Track is record
      Previous_Point     : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
      Next_Point         : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
      Tracked            : Boolean := False;
      Minimum_Eigenvalue : OpenCV.Float32_Value := 0.0;
   end record;

   type Trackability_Track_Array is
     array (Positive range <>) of Trackability_Track;

   --  Local LK conditioning of the previous-image patch, normalized by window
   --  pixel count. This is NOT photometric Error, probability, match confidence,
   --  or a guarantee of correct correspondence. No universal threshold exists.
   --  Minimum_Eigenvalue is finite and nonnegative independently of Tracked:
   --  threshold rejection or unavailable next search can retain useful quality.
   --  An unavailable previous patch returns zero when native defines it; a
   --  backend leaving quality unwritten raises OpenCV_Error. Failed Next_Point is
   --  Previous_Point. Invalid/unwritten native quality raises OpenCV_Error;
   --  values are never clamped. Existing image/coordinate/options validation,
   --  input immutability, Region support and Points'Range preservation apply.
   function Track_PyrLK_Trackability
     (Previous_Image : OpenCV.Core.Mat;
      Next_Image     : OpenCV.Core.Mat;
      Points         : Tracking_Point_Array;
      Options        : PyrLK_Options := (others => <>))
      return Trackability_Track_Array;

   --  Same quality semantics with private cloned initial predictions. Equal
   --  lengths, iteration-position correspondence, differing bounds allowed.
   --  Options precedes required seeds for positional aggregate compatibility.
   function Track_PyrLK_Trackability
     (Previous_Image      : OpenCV.Core.Mat;
      Next_Image          : OpenCV.Core.Mat;
      Points              : Tracking_Point_Array;
      Options             : PyrLK_Options := (others => <>);
      Initial_Next_Points : Tracking_Point_Array)
      return Trackability_Track_Array;

   --  Owned derivative-interleaved pyramids, flags 8/12. Same compatibility
   --  rules as photometric pyramids; defined quality is retained on failure.
   function Track_PyrLK_Trackability
     (Previous_Pyramid : PyrLK_Pyramid;
      Next_Pyramid     : PyrLK_Pyramid;
      Points           : Tracking_Point_Array;
      Options          : PyrLK_Options := (others => <>))
      return Trackability_Track_Array;

   function Track_PyrLK_Trackability
     (Previous_Pyramid    : PyrLK_Pyramid;
      Next_Pyramid        : PyrLK_Pyramid;
      Points              : Tracking_Point_Array;
      Options             : PyrLK_Options := (others => <>);
      Initial_Next_Points : Tracking_Point_Array)
      return Trackability_Track_Array;

   type Forward_Backward_Options is record
      Tracking                 : PyrLK_Options := (others => <>);
      --  Pixels; finite and nonnegative. Zero is legal; no silent clamping.
      Maximum_Round_Trip_Error : OpenCV.Float32_Value := 1.0;
   end record;

   type Forward_Backward_Track is record
      Forward                  : Point_Track;
      Backward_Tracked         : Boolean := False;
      Recovered_Previous_Point : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
      Round_Trip_Error         : OpenCV.Float32_Value := 0.0;
      Consistent               : Boolean := False;
   end record;

   type Forward_Backward_Track_Array is
     array (Positive range <>) of Forward_Backward_Track;

   --  Compose forward LK and seeded backward LK, only for forward successes.
   --  Backward starts at Forward.Next_Point and predicts the original point.
   --  Results preserve Points'Range and never implicitly filter entries.
   --  If either direction fails: recovered = original, distance = zero,
   --  Backward_Tracked = Consistent = False. Otherwise Round_Trip_Error is the
   --  Euclidean pixel distance from original to recovered, computed in Float64
   --  and checked before Float32 conversion. Consistent means both directions
   --  succeeded and the returned distance <= Maximum_Round_Trip_Error.
   --  Forward.Error remains native mean patch L1 photometric error, NOT this
   --  geometric distance. Consistency is useful evidence, not proof of a correct
   --  physical correspondence. Existing image/point/options contracts apply.
   function Track_PyrLK_Forward_Backward
     (Previous_Image : OpenCV.Core.Mat;
      Next_Image     : OpenCV.Core.Mat;
      Points         : Tracking_Point_Array;
      Options        : Forward_Backward_Options := (others => <>))
      return Forward_Backward_Track_Array;

   --  Forward predictions follow the existing seeded count/iteration-position
   --  contract, may have different bounds, and are never modified. Options
   --  precedes required seeds to preserve positional aggregate compatibility.
   function Track_PyrLK_Forward_Backward
     (Previous_Image      : OpenCV.Core.Mat;
      Next_Image          : OpenCV.Core.Mat;
      Points              : Tracking_Point_Array;
      Options             : Forward_Backward_Options := (others => <>);
      Initial_Next_Points : Tracking_Point_Array)
      return Forward_Backward_Track_Array;
   --  Same diagnostics with owned pyramids. Both legs reuse the stored vectors;
   --  the backward leg reverses these objects and predicts original positions.
   --  Existing pyramid compatibility checks apply even to empty point arrays.
   function Track_PyrLK_Forward_Backward
     (Previous_Pyramid : PyrLK_Pyramid;
      Next_Pyramid     : PyrLK_Pyramid;
      Points           : Tracking_Point_Array;
      Options          : Forward_Backward_Options := (others => <>))
      return Forward_Backward_Track_Array;

   function Track_PyrLK_Forward_Backward
     (Previous_Pyramid    : PyrLK_Pyramid;
      Next_Pyramid        : PyrLK_Pyramid;
      Points              : Tracking_Point_Array;
      Options             : Forward_Backward_Options := (others => <>);
      Initial_Next_Points : Tracking_Point_Array)
      return Forward_Backward_Track_Array;
   --  Dense Farneback optical flow (cv::calcOpticalFlowFarneback, flags 0).
   --  Ranges (outside raises OpenCV_Error, never clamps): Pyramid_Scale
   --  0.25 .. 0.90; Levels 1 .. 8; Window_Size odd 5 .. 63; Iterations 1 .. 30;
   --  Poly_Neighborhood 5 or 7; Poly_Sigma finite 0.1 .. 10.
   type Farneback_Options is record
      Pyramid_Scale     : OpenCV.Float64_Value := 0.5;
      Levels            : Positive := 3;
      Window_Size       : Positive := 15;
      Iterations        : Positive := 3;
      Poly_Neighborhood : Positive := 5;
      Poly_Sigma        : OpenCV.Float64_Value := 1.2;
   end record;

   --  Images: nonempty 2-D UInt8 C1 of identical geometry, at least 16 x 16
   --  and at most Integer_32'Last / 16 = 134_217_727 pixels (matching the native shim); Regions are accepted. Returns a new Core-owned
   --  Float32 C2 Mat with the image geometry; channel 0 is dx, channel 1 is
   --  dy, so Previous(y,x) ~ Next(y + dy, x + dx) (a positive-x shift of the
   --  content yields positive dx). Every component is finite or OpenCV_Error is
   --  raised. Inputs are unchanged. KNOWN_UPSTREAM_UB_ACCEPTED: OpenCV 4.x CPU
   --  FarnebackUpdateMatrices forms an out-of-range pointer before its bounds
   --  check (docs/farneback-source-contract.md); not suitable as-is for
   --  safety-sensitive deployments.
   function Calculate_Farneback_Flow
     (Previous_Image : OpenCV.Core.Mat;
      Next_Image     : OpenCV.Core.Mat;
      Options        : Farneback_Options := (others => <>)) return OpenCV.Core.Mat;

   --  Seeded dense Farneback (native OPTFLOW_USE_INITIAL_FLOW, flags 4). Same
   --  image/option contract as above. Initial_Flow is a displacement prediction in
   --  pixels (channel 0 dx, channel 1 dy; signed): nonempty 2-D Float32 C2 with the
   --  exact image rows/columns (Regions accepted), every component finite with
   --  absolute value <= 2**20 (binding policy; never clamped). It is copied into
   --  private storage before native mutation and is never modified. Returns a new,
   --  distinct Core-owned Float32 C2 Mat. Options precedes the required seed so
   --  positional aggregates stay compatible. Not a confidence or navigation filter;
   --  the KNOWN_UPSTREAM_UB_ACCEPTED limitation above applies unchanged.
   function Calculate_Farneback_Flow
     (Previous_Image : OpenCV.Core.Mat;
      Next_Image     : OpenCV.Core.Mat;
      Options        : Farneback_Options := (others => <>);
      Initial_Flow   : OpenCV.Core.Mat) return OpenCV.Core.Mat;

   --  ECC translation alignment (cv::findTransformECC, MOTION_TRANSLATION,
   --  privately initialized identity warp, COUNT|EPS, no mask, Gaussian size 5).
   --  Maximum_Iterations 1 .. 100; Epsilon finite in 1.0E-8 .. 0.1. Values
   --  outside these ranges raise OpenCV_Error and are never clamped.
   type ECC_Translation_Options is record
      Maximum_Iterations : Positive := 50;
      Epsilon            : OpenCV.Float64_Value := 0.0001;
   end record;

   --  Displacement maps template coordinates into the moving image:
   --  moving_x = template_x + Displacement.X, moving_y = template_y + Displacement.Y
   --  (X right, Y down). It is NOT the inverse warp; OpenCV applies it with
   --  WARP_INVERSE_MAP to bring the moving image into the template frame.
   --  Correlation is the finite value returned by native findTransformECC. It
   --  scores the warp from the start of the final native iteration, is an image
   --  intensity measure only, and is neither clamped, normalized nor a confidence.
   type ECC_Translation_Result is record
      Displacement : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
      Correlation  : OpenCV.Float64_Value := 0.0;
   end record;

   --  Both images: nonempty 2-D UInt8 C1, identical geometry, 32 .. 16384 rows and
   --  columns, at most 16_777_216 pixels (Regions accepted; inputs unchanged).
   --  Native nonconvergence (constant or uncorrelated images, translation outside
   --  the unseeded convergence basin) raises OpenCV_Error; no identity result is
   --  fabricated. A returned value does not prove correct physical alignment.
   function Estimate_ECC_Translation
     (Template_Image : OpenCV.Core.Mat;
      Moving_Image   : OpenCV.Core.Mat;
      Options        : ECC_Translation_Options := (others => <>))
      return ECC_Translation_Result;
   --  ECC Euclidean (rigid) alignment: cv::findTransformECC with MOTION_EUCLIDEAN,
   --  privately initialized identity warp, COUNT|EPS, no mask, Gaussian size 5. The
   --  image and option contract is exactly that of Estimate_ECC_Translation.
   subtype ECC_Euclidean_Options is ECC_Translation_Options;

   --  The native 2x3 warp maps TEMPLATE coordinates to MOVING coordinates
   --  (X right, Y down), with theta = Rotation_Radians:
   --     moving_x = cos (theta) * template_x - sin (theta) * template_y + Displacement.X
   --     moving_y = sin (theta) * template_x + cos (theta) * template_y + Displacement.Y
   --  Rotation is about the coordinate origin (0, 0), the top-left pixel, NOT the
   --  image centre; Displacement is the matrix translation column, so a rotation
   --  about the centre carries compensating translation terms. To resample the moving
   --  image into the template frame, apply this same matrix with WARP_INVERSE_MAP;
   --  never negate the angle or translation for that reason.
   --  Rotation_Radians is atan2 (warp(1,0), warp(0,0)) of the returned warp, in
   --  [-Pi, Pi]; precision is limited by the Float32 warp. It is an image-alignment
   --  angle, not a heading, yaw or aircraft attitude. Native updates decode the
   --  previous angle with asin, so recovery is reliable only for modest rotations;
   --  larger ones may fail or converge to a wrong finite result.
   --  Correlation is the native value, computed before the final update.
   type ECC_Euclidean_Result is record
      Displacement     : OpenCV.Float32_Point := (X => 0.0, Y => 0.0);
      Rotation_Radians : OpenCV.Float64_Value := 0.0;
      Correlation      : OpenCV.Float64_Value := 0.0;
   end record;

   --  Native nonconvergence raises OpenCV_Error; no identity is fabricated. A returned
   --  value, even with a valid rotation matrix, does not prove correct alignment.
   function Estimate_ECC_Euclidean
     (Template_Image : OpenCV.Core.Mat;
      Moving_Image   : OpenCV.Core.Mat;
      Options        : ECC_Euclidean_Options := (others => <>))
      return ECC_Euclidean_Result;

   --  Caller-seeded ECC Euclidean alignment (Task 013). The private native warp starts
   --  from Initial_Transform instead of the identity and is refined by the same
   --  findTransformECC call (MOTION_EUCLIDEAN, COUNT|EPS, no mask, Gaussian size 5).
   --  Only Initial_Transform.Displacement and .Rotation_Radians initialize the warp,
   --  using the template-to-moving convention above (rotation about the origin, no
   --  implicit composition). Initial_Transform.Correlation is IGNORED: it is neither an
   --  optimization input, a threshold, nor propagated to the result, so a previous
   --  ECC_Euclidean_Result may be passed directly. The seed is a prediction for THIS
   --  image pair; a previous pair's result is not automatically the right transform.
   --  Seed domain (binding policy, not an OpenCV guarantee): Displacement components
   --  finite with absolute value <= 2**20, Rotation_Radians finite with absolute value
   --  <= 1.0 (keeps the seed inside the principal asin branch used by the native
   --  update). Violations raise OpenCV_Error and are never clamped. An accepted seed
   --  does not guarantee convergence; ECC may still raise on nonconvergence or end at
   --  a wrong finite optimum. Initial_Transform is not modified.
   function Estimate_ECC_Euclidean
     (Template_Image    : OpenCV.Core.Mat;
      Moving_Image      : OpenCV.Core.Mat;
      Options           : ECC_Euclidean_Options := (others => <>);
      Initial_Transform : ECC_Euclidean_Result)
      return ECC_Euclidean_Result;
   --  ECC six-parameter affine alignment (Task 014): cv::findTransformECC with
   --  MOTION_AFFINE, privately initialized identity warp, COUNT|EPS, no mask, Gaussian
   --  size 5. The image and option contract is exactly that of Estimate_ECC_Translation
   --  (ECC_Affine_Options intentionally reuses the same iteration/epsilon policy).
   subtype ECC_Affine_Options is ECC_Translation_Options;

   --  The native 2x3 warp maps TEMPLATE coordinates to MOVING coordinates (X right,
   --  Y down), expressed in pixels:
   --     moving_x = M00 * template_x + M01 * template_y + M02
   --     moving_y = M10 * template_x + M11 * template_y + M12
   --  The six native coefficients are returned directly, row-major, WITHOUT any
   --  rotation/scale/shear decomposition. To resample the moving image into the template
   --  frame apply this same matrix with WARP_INVERSE_MAP; never transpose, negate or
   --  invert it for that reason. The default is the identity. The linear part need not
   --  be a rotation: scale, shear, reflection and near-singular finite matrices are
   --  legitimate native outputs and are not rejected.
   type ECC_Affine_Warp is record
      M00 : OpenCV.Float32_Value := 1.0;
      M01 : OpenCV.Float32_Value := 0.0;
      M02 : OpenCV.Float32_Value := 0.0;
      M10 : OpenCV.Float32_Value := 0.0;
      M11 : OpenCV.Float32_Value := 1.0;
      M12 : OpenCV.Float32_Value := 0.0;
   end record;

   --  Correlation is the native value, computed before the final update. It is an image
   --  intensity measure, neither clamped nor a confidence. The result is a plain Ada
   --  value and owns no native resource. It is an image-alignment matrix, not a
   --  geographic mapping, camera pose, yaw or verified navigation estimate.
   type ECC_Affine_Result is record
      Warp        : ECC_Affine_Warp := (others => <>);
      Correlation : OpenCV.Float64_Value := 0.0;
   end record;

   --  Native nonconvergence (constant or uncorrelated images, transforms outside the
   --  identity-initialized convergence basin) raises OpenCV_Error; no identity result is
   --  fabricated. A returned finite matrix means native ECC returned and the binding
   --  schema held, NOT that the images are reliably registered. Peak memory grows with
   --  image area (six Float32 Jacobian blocks per pixel); validation does not guarantee
   --  sufficient system memory.
   function Estimate_ECC_Affine
     (Template_Image : OpenCV.Core.Mat;
      Moving_Image   : OpenCV.Core.Mat;
      Options        : ECC_Affine_Options := (others => <>))
      return ECC_Affine_Result;

   --  Caller-seeded ECC affine alignment (Task 015). The private native 2x3 warp starts
   --  from Initial_Transform.Warp instead of the identity and is refined by the same
   --  findTransformECC call (MOTION_AFFINE, COUNT|EPS, no mask, Gaussian size 5).
   --  The six coefficients are placed in native matrix STORAGE order (M00 M01 M02 M10
   --  M11 M12), not the native update-vector order (M00 M10 M01 M11 M02 M12), using the
   --  template-to-moving convention above. Nothing is transposed, negated, inverted,
   --  decomposed or composed. Initial_Transform.Correlation is IGNORED (never read,
   --  validated, propagated or used as a threshold), so a previous ECC_Affine_Result may
   --  be passed directly. The seed is a prediction for THIS image pair; a previous pair's
   --  result is not automatically the right transform.
   --  Seed domain (binding policy, not an OpenCV guarantee): M00, M01, M10, M11 finite
   --  with absolute value <= 4.0; M02, M12 finite with absolute value <= 2**20. Violations
   --  raise OpenCV_Error and are never clamped. No determinant, orientation or
   --  conditioning restriction is applied. An accepted seed does not guarantee
   --  convergence; ECC may still raise on nonconvergence or end at a wrong finite
   --  optimum. Initial_Transform is not modified.
   function Estimate_ECC_Affine
     (Template_Image    : OpenCV.Core.Mat;
      Moving_Image      : OpenCV.Core.Mat;
      Options           : ECC_Affine_Options := (others => <>);
      Initial_Transform : ECC_Affine_Result)
      return ECC_Affine_Result;

   --  ECC eight-parameter homography alignment (Task 016): cv::findTransformECC with
   --  MOTION_HOMOGRAPHY, a privately initialized identity 3x3 warp, COUNT|EPS, no mask,
   --  Gaussian size 5. The image and option contract is exactly that of
   --  Estimate_ECC_Translation (the options subtype reuses its validation).
   subtype ECC_Homography_Options is ECC_Translation_Options;

   --  The native 3x3 row-major warp maps TEMPLATE coordinates to MOVING coordinates
   --  (X right, Y down) with homogeneous division:
   --     D        = H20 * x + H21 * y + H22
   --     moving_x = (H00 * x + H01 * y + H02) / D
   --     moving_y = (H10 * x + H11 * y + H12) / D
   --  The nine native coefficients are returned in matrix storage order (not the native
   --  update-vector order). The native update never writes H22, so it is exactly 1.0 for
   --  this identity-initialized call; any other value is reported as OpenCV_Error. To
   --  resample the moving image into the template frame apply this same matrix with
   --  WARP_INVERSE_MAP; never transpose, negate, invert or normalize it for that reason.
   --  The mapping is undefined where D = 0 and extreme near it; the binding does not
   --  resample and does not detect such points. No determinant, scale, perspective or
   --  conditioning limit is applied to a finite native result.
   type ECC_Homography_Warp is record
      H00 : OpenCV.Float32_Value := 1.0;
      H01 : OpenCV.Float32_Value := 0.0;
      H02 : OpenCV.Float32_Value := 0.0;
      H10 : OpenCV.Float32_Value := 0.0;
      H11 : OpenCV.Float32_Value := 1.0;
      H12 : OpenCV.Float32_Value := 0.0;
      H20 : OpenCV.Float32_Value := 0.0;
      H21 : OpenCV.Float32_Value := 0.0;
      H22 : OpenCV.Float32_Value := 1.0;
   end record;

   --  Correlation is the native value, computed before the final update; it is neither
   --  clamped nor a confidence. The result is a plain Ada value owning no native
   --  resource, and an image-alignment matrix only (not a camera pose, 3D motion,
   --  geolocation or verified navigation estimate).
   type ECC_Homography_Result is record
      Warp        : ECC_Homography_Warp := (others => <>);
      Correlation : OpenCV.Float64_Value := 0.0;
   end record;

   --  Native nonconvergence raises OpenCV_Error; no identity result is fabricated. A
   --  returned finite matrix does not mean the images are reliably registered. Identity
   --  initialization has a small convergence basin (docs/ecc-homography-source-contract.md).
   --  Peak memory is about 1.9 GiB at the 2**24-pixel limit; validation does not
   --  guarantee sufficient system memory.
   function Estimate_ECC_Homography
     (Template_Image : OpenCV.Core.Mat;
      Moving_Image   : OpenCV.Core.Mat;
      Options        : ECC_Homography_Options := (others => <>))
      return ECC_Homography_Result;

   --  Caller-seeded ECC homography alignment (Task 017). The private native 3x3 warp starts
   --  from Initial_Transform.Warp instead of the identity and is refined by the same
   --  findTransformECC call (MOTION_HOMOGRAPHY, COUNT|EPS, no mask, Gaussian size 5). The
   --  nine coefficients are placed in native matrix STORAGE order (H00 .. H22) with the
   --  template-to-moving convention above; nothing is transposed, negated, inverted,
   --  normalized, decomposed or composed. Initial_Transform.Correlation is IGNORED (never
   --  read, validated, propagated or used as a threshold).
   --
   --  Seed domain (binding policy, docs/ecc-homography-seeded-source-contract.md), checked
   --  after the image/option contract: every coefficient finite; H22 exactly 1.0 (native ECC
   --  never updates H22 and its Jacobian assumes 1, so a seed is never rescaled); H00, H01,
   --  H10, H11 at most 4.0 in magnitude; H02, H12 at most 2**20; and
   --  |H20| * (Columns - 1) + |H21| * (Rows - 1) <= 0.5 in Long_Float, so the seed's
   --  denominator stays within 0.5 .. 1.5 over the template. A violation raises
   --  OpenCV_Error before any native call.
   --
   --  The output contract is that of the identity-initialized overload; the seed domain is
   --  NOT applied to the result, so a returned result (seeded or not) may itself fall outside
   --  the seed domain and be rejected if passed back as Initial_Transform. A good seed can
   --  converge where the identity does not; a poor one may raise on nonconvergence or end at
   --  a wrong finite optimum. Initial_Transform is not modified.
   function Estimate_ECC_Homography
     (Template_Image    : OpenCV.Core.Mat;
      Moving_Image      : OpenCV.Core.Mat;
      Options           : ECC_Homography_Options := (others => <>);
      Initial_Transform : ECC_Homography_Result)
      return ECC_Homography_Result;
private
   type PyrLK_Pyramid is new Ada.Finalization.Limited_Controlled with record
      Handle : System.Address := System.Null_Address;
      Window : OpenCV.Size := (Width => 0, Height => 0);
      Requested, Available : Natural := 0;
      Rows, Columns : Natural := 0;
   end record;
   overriding procedure Finalize (Pyramid : in out PyrLK_Pyramid);
end OpenCV.Video;
