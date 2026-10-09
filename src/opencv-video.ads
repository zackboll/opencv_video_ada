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
private
   type PyrLK_Pyramid is new Ada.Finalization.Limited_Controlled with record
      Handle : System.Address := System.Null_Address;
      Window : OpenCV.Size := (Width => 0, Height => 0);
      Requested, Available : Natural := 0;
      Rows, Columns : Natural := 0;
   end record;
   overriding procedure Finalize (Pyramid : in out PyrLK_Pyramid);
end OpenCV.Video;
