with OpenCV.Core;

package OpenCV.Video is
   --  Initial bootstrap slice: sparse pyramidal Lucas-Kanade tracking.
   --  OpenCV.Core remains the sole owner of Mat wrappers.

   type Tracking_Point_Array is array (Positive range <>) of OpenCV.Float32_Point;

   type PyrLK_Options is record
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

   --  Track Points from Previous_Image into Next_Image using
   --  cv::calcOpticalFlowPyrLK.
   --
   --  Bootstrap contract:
   --    * images are nonempty, 2-D UInt8 C1 Mats of identical geometry;
   --    * Points are finite Float32 coordinates;
   --    * input array bounds are preserved in the returned Track array;
   --    * no OPTFLOW_USE_INITIAL_FLOW or LK_GET_MIN_EIGENVALS flag is used;
   --    * when Tracked=False, Next_Point is reset to Previous_Point and
   --      Error is reset to 0.0 instead of exposing native undefined values;
   --    * images and Points are not modified.
   function Track_PyrLK
     (Previous_Image : OpenCV.Core.Mat;
      Next_Image     : OpenCV.Core.Mat;
      Points         : Tracking_Point_Array;
      Options        : PyrLK_Options := (others => <>)) return Point_Track_Array;

   function Successful_Count (Tracks : Point_Track_Array) return Natural;
end OpenCV.Video;
