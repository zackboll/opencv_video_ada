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
end OpenCV.Video.Internal.C_API;
