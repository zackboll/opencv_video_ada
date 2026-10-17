with Interfaces;
with Interfaces.C;
with Interfaces.C.Strings;
with OpenCV.Core.Module_Interop;
with System;

package OpenCV.Video.Internal.C_API is
   subtype Status is Interfaces.Integer_32;
   Success : constant Status := 0;

   function Native_Version return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_video_native_version";
   function Native_Backend return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_video_native_backend";
   function Last_Error return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_video_last_error";

   function Track_PyrLK
     (Previous_Image         : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Image             : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Previous_Points        : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Status           : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Error            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Window_Width           : Interfaces.Integer_32;
      Window_Height          : Interfaces.Integer_32;
      Max_Level              : Interfaces.Integer_32;
      Maximum_Iterations     : Interfaces.Integer_32;
      Epsilon                : Interfaces.C.double;
      Min_Eigenvalue_Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk";

   function Track_PyrLK_Seeded
     (Previous_Image         : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Image             : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Previous_Points        : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Initial_Next_Points    : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Status           : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Error            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Window_Width           : Interfaces.Integer_32;
      Window_Height          : Interfaces.Integer_32;
      Max_Level              : Interfaces.Integer_32;
      Maximum_Iterations     : Interfaces.Integer_32;
      Epsilon                : Interfaces.C.double;
      Min_Eigenvalue_Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk_seeded";

   function Track_PyrLK_Quality
     (Previous_Image         : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Image             : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Previous_Points        : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Status           : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Error            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Window_Width           : Interfaces.Integer_32;
      Window_Height          : Interfaces.Integer_32;
      Max_Level              : Interfaces.Integer_32;
      Maximum_Iterations     : Interfaces.Integer_32;
      Epsilon                : Interfaces.C.double;
      Min_Eigenvalue_Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk_min_eigenvalues";

   function Track_PyrLK_Seeded_Quality
     (Previous_Image         : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Image             : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Previous_Points        : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Initial_Next_Points    : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Status           : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Track_Error            : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Window_Width           : Interfaces.Integer_32;
      Window_Height          : Interfaces.Integer_32;
      Max_Level              : Interfaces.Integer_32;
      Maximum_Iterations     : Interfaces.Integer_32;
      Epsilon                : Interfaces.C.double;
      Min_Eigenvalue_Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk_seeded_min_eigenvalues";

   procedure Check (Code : Status; Operation : String);

   function Pyramid_Create
     (Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Width, Height, Requested : Interfaces.Integer_32;
      Handle : access System.Address) return Status
     with Import, Convention => C, External_Name => "opencv_video_pyramid_create";
   procedure Pyramid_Destroy (Handle : System.Address)
     with Import, Convention => C, External_Name => "opencv_video_pyramid_destroy";
   function Pyramid_Metadata
     (Handle : System.Address;
      Width, Height, Requested, Available : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_video_pyramid_metadata";
   function Track_PyrLK_Pyramids
     (Previous, Next : System.Address;
      Points : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points, Track_Status, Track_Error : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Width, Height, Level, Iterations : Interfaces.Integer_32;
      Epsilon, Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk_pyramids";
   function Track_PyrLK_Pyramids_Seeded
     (Previous, Next : System.Address;
      Points, Seeds : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points, Track_Status, Track_Error : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Width, Height, Level, Iterations : Interfaces.Integer_32;
      Epsilon, Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk_pyramids_seeded";
   function Track_PyrLK_Pyramids_Quality
     (Previous, Next : System.Address;
      Points : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points, Track_Status, Track_Error : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Width, Height, Level, Iterations : Interfaces.Integer_32;
      Epsilon, Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk_pyramids_min_eigenvalues";
   function Track_PyrLK_Pyramids_Seeded_Quality
     (Previous, Next : System.Address;
      Points, Seeds : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Next_Points, Track_Status, Track_Error : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Width, Height, Level, Iterations : Interfaces.Integer_32;
      Epsilon, Threshold : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_track_pyr_lk_pyramids_seeded_min_eigenvalues";
   function Calc_Farneback_Flow
     (Previous_Image, Next_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Flow                      : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Pyramid_Scale             : Interfaces.C.double;
      Levels, Window_Size, Iterations, Poly_Neighborhood : Interfaces.Integer_32;
      Poly_Sigma                : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_calc_farneback_flow";
   function Calc_Farneback_Flow_Seeded
     (Previous_Image, Next_Image, Initial_Flow : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Result_Flow                : OpenCV.Core.Module_Interop.Output_Mat_Handle;
      Pyramid_Scale              : Interfaces.C.double;
      Levels, Window_Size, Iterations, Poly_Neighborhood : Interfaces.Integer_32;
      Poly_Sigma                 : Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_video_calc_farneback_flow_seeded";

   --  Mirrors opencv_video_ecc_translation_result exactly (float, float, double).
   type ECC_Translation_Result is record
      Displacement_X : Interfaces.C.C_float := 0.0;
      Displacement_Y : Interfaces.C.C_float := 0.0;
      Correlation    : Interfaces.C.double := 0.0;
   end record
     with Convention => C;

   function Estimate_ECC_Translation
     (Template_Image, Moving_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Maximum_Iterations           : Interfaces.Integer_32;
      Epsilon                      : Interfaces.C.double;
      Result                       : access ECC_Translation_Result) return Status
     with Import, Convention => C, External_Name => "opencv_video_estimate_ecc_translation";
   --  Mirrors opencv_video_ecc_euclidean_result exactly (float, float, double, double):
   --  offsets 0, 4, 8, 16; size 24.
   type ECC_Euclidean_Result is record
      Displacement_X   : Interfaces.C.C_float := 0.0;
      Displacement_Y   : Interfaces.C.C_float := 0.0;
      Rotation_Radians : Interfaces.C.double := 0.0;
      Correlation      : Interfaces.C.double := 0.0;
   end record
     with Convention => C;

   function Estimate_ECC_Euclidean
     (Template_Image, Moving_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Maximum_Iterations           : Interfaces.Integer_32;
      Epsilon                      : Interfaces.C.double;
      Result                       : access ECC_Euclidean_Result) return Status
     with Import, Convention => C, External_Name => "opencv_video_estimate_ecc_euclidean";

   function Estimate_ECC_Euclidean_Seeded
     (Template_Image, Moving_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Maximum_Iterations           : Interfaces.Integer_32;
      Epsilon                      : Interfaces.C.double;
      Initial_Translation_X        : Interfaces.C.C_float;
      Initial_Translation_Y        : Interfaces.C.C_float;
      Initial_Rotation_Radians     : Interfaces.C.double;
      Result                       : access ECC_Euclidean_Result) return Status
     with Import, Convention => C,
          External_Name => "opencv_video_estimate_ecc_euclidean_seeded";
   --  Mirrors opencv_video_ecc_affine_result exactly (six float, double):
   --  offsets 0, 4, 8, 12, 16, 20, 24; size 32.
   type ECC_Affine_Result is record
      M00         : Interfaces.C.C_float := 1.0;
      M01         : Interfaces.C.C_float := 0.0;
      M02         : Interfaces.C.C_float := 0.0;
      M10         : Interfaces.C.C_float := 0.0;
      M11         : Interfaces.C.C_float := 1.0;
      M12         : Interfaces.C.C_float := 0.0;
      Correlation : Interfaces.C.double := 0.0;
   end record
     with Convention => C;

   function Estimate_ECC_Affine
     (Template_Image, Moving_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Maximum_Iterations           : Interfaces.Integer_32;
      Epsilon                      : Interfaces.C.double;
      Result                       : access ECC_Affine_Result) return Status
     with Import, Convention => C, External_Name => "opencv_video_estimate_ecc_affine";
   --  Mirrors opencv_video_ecc_homography_result exactly (nine float, double):
   --  offsets 0, 4, ..., 32; four padding bytes; Correlation at 40; size 48.
   type ECC_Homography_Result is record
      H00         : Interfaces.C.C_float := 1.0;
      H01         : Interfaces.C.C_float := 0.0;
      H02         : Interfaces.C.C_float := 0.0;
      H10         : Interfaces.C.C_float := 0.0;
      H11         : Interfaces.C.C_float := 1.0;
      H12         : Interfaces.C.C_float := 0.0;
      H20         : Interfaces.C.C_float := 0.0;
      H21         : Interfaces.C.C_float := 0.0;
      H22         : Interfaces.C.C_float := 1.0;
      Correlation : Interfaces.C.double := 0.0;
   end record
     with Convention => C;

   function Estimate_ECC_Homography
     (Template_Image, Moving_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Maximum_Iterations           : Interfaces.Integer_32;
      Epsilon                      : Interfaces.C.double;
      Result                       : access ECC_Homography_Result) return Status
     with Import, Convention => C, External_Name => "opencv_video_estimate_ecc_homography";
   function Estimate_ECC_Affine_Seeded
     (Template_Image, Moving_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Maximum_Iterations           : Interfaces.Integer_32;
      Epsilon                      : Interfaces.C.double;
      Initial_M00                  : Interfaces.C.C_float;
      Initial_M01                  : Interfaces.C.C_float;
      Initial_M02                  : Interfaces.C.C_float;
      Initial_M10                  : Interfaces.C.C_float;
      Initial_M11                  : Interfaces.C.C_float;
      Initial_M12                  : Interfaces.C.C_float;
      Result                       : access ECC_Affine_Result) return Status
     with Import, Convention => C,
          External_Name => "opencv_video_estimate_ecc_affine_seeded";

   function Estimate_ECC_Homography_Seeded
     (Template_Image, Moving_Image : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Maximum_Iterations           : Interfaces.Integer_32;
      Epsilon                      : Interfaces.C.double;
      Initial_H00                  : Interfaces.C.C_float;
      Initial_H01                  : Interfaces.C.C_float;
      Initial_H02                  : Interfaces.C.C_float;
      Initial_H10                  : Interfaces.C.C_float;
      Initial_H11                  : Interfaces.C.C_float;
      Initial_H12                  : Interfaces.C.C_float;
      Initial_H20                  : Interfaces.C.C_float;
      Initial_H21                  : Interfaces.C.C_float;
      Initial_H22                  : Interfaces.C.C_float;
      Result                       : access ECC_Homography_Result) return Status
     with Import, Convention => C,
          External_Name => "opencv_video_estimate_ecc_homography_seeded";
end OpenCV.Video.Internal.C_API;
