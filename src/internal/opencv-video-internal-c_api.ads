with Interfaces;
with Interfaces.C;
with Interfaces.C.Strings;
with OpenCV.Core.Module_Interop;

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

   procedure Check (Code : Status; Operation : String);
end OpenCV.Video.Internal.C_API;
