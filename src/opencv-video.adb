with Interfaces;
with Interfaces.C;
with OpenCV.Core.Float32_Access;
with OpenCV.Core.Float32_Vec2;
with OpenCV.Core.Float32_Vec2_Access;
with OpenCV.Core.Module_Interop;
with OpenCV.Core.UInt8_Access;
with OpenCV.Video.Internal.C_API;

package body OpenCV.Video is
   package C renames OpenCV.Video.Internal.C_API;
   package Bridge renames OpenCV.Core.Module_Interop;
   package Vec2 renames OpenCV.Core.Float32_Vec2;
   package Vec2_Access renames OpenCV.Core.Float32_Vec2_Access;
   package Float_Access renames OpenCV.Core.Float32_Access;

   use type OpenCV.Core.Depth_Type;
   use type OpenCV.Core.Channel_Count;
   use type OpenCV.Float32_Value;
   use type OpenCV.Float64_Value;
   use type Interfaces.Unsigned_8;

   function Is_Finite (Value : OpenCV.Float32_Value) return Boolean is
     (Value = Value and then
      Value >= -OpenCV.Float32_Value'Last and then Value <= OpenCV.Float32_Value'Last);

   function Is_Finite (Value : OpenCV.Float64_Value) return Boolean is
     (Value = Value and then
      Value >= -OpenCV.Float64_Value'Last and then Value <= OpenCV.Float64_Value'Last);

   procedure Validate_Images
     (Previous_Image : OpenCV.Core.Mat; Next_Image : OpenCV.Core.Mat) is
   begin
      if Previous_Image.Is_Empty or else Next_Image.Is_Empty then
         raise OpenCV.OpenCV_Error with "PyrLK images must be nonempty";
      end if;
      if Previous_Image.Dimension_Count /= 2 or else Next_Image.Dimension_Count /= 2 then
         raise OpenCV.OpenCV_Error with "PyrLK bootstrap accepts only 2-D images";
      end if;
      if Previous_Image.Depth /= OpenCV.Core.UInt8
        or else Next_Image.Depth /= OpenCV.Core.UInt8
        or else Previous_Image.Channels /= 1
        or else Next_Image.Channels /= 1
      then
         raise OpenCV.OpenCV_Error with "PyrLK images must be UInt8 C1";
      end if;
      if Previous_Image.Rows /= Next_Image.Rows
        or else Previous_Image.Columns /= Next_Image.Columns
      then
         raise OpenCV.OpenCV_Error with "PyrLK image geometry differs";
      end if;
   end Validate_Images;

   procedure Validate (Points : Tracking_Point_Array) is
   begin
      if Points'Length > Long_Long_Integer (Interfaces.Integer_32'Last) then
         raise OpenCV.OpenCV_Error with "PyrLK point count exceeds native integer range";
      end if;
      for Point of Points loop
         if not Is_Finite (Point.X) or else not Is_Finite (Point.Y)
            or else abs Point.X > 536_870_912.0 or else abs Point.Y > 536_870_912.0
         then
            raise OpenCV.OpenCV_Error with "PyrLK points exceed safe native conversion bounds";
         end if;
      end loop;
   end Validate;

   procedure Validate (Options : PyrLK_Options) is
   begin
      if Options.Window_Size.Width not in 3 .. 255
        or else Options.Window_Size.Height not in 3 .. 255
        or else Options.Max_Level > 30 or else Options.Maximum_Iterations > 100
      then
         raise OpenCV.OpenCV_Error with "PyrLK window, level or iteration count out of range";
      end if;
      if not Is_Finite (Options.Epsilon) or else Options.Epsilon <= 0.0
        or else Options.Epsilon > 10.0
      then
         raise OpenCV.OpenCV_Error with "PyrLK epsilon must be finite and positive";
      end if;
      if not Is_Finite (Options.Min_Eigenvalue_Threshold)
        or else Options.Min_Eigenvalue_Threshold < 0.0
        or else Options.Min_Eigenvalue_Threshold >
          OpenCV.Float64_Value (OpenCV.Float32_Value'Last)
      then
         raise OpenCV.OpenCV_Error with
           "PyrLK minimum eigenvalue threshold must be finite and nonnegative";
      end if;
   end Validate;

   function Point_Matrix (Points : Tracking_Point_Array) return OpenCV.Core.Mat is
   begin
      if Points'Length = 0 then
         declare
            Empty : OpenCV.Core.Mat;
         begin
            return Empty;
         end;
      end if;
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (Points'Length, 1, (Depth => OpenCV.Core.Float32, Channels => 2)) do
         for I in Points'Range loop
            Vec2_Access.Set
              (Result, I - Points'First, 0,
               Vec2.Vector'(0 => Points (I).X, 1 => Points (I).Y));
         end loop;
      end return;
   end Point_Matrix;

   function Track_Internal
     (Previous_Image      : OpenCV.Core.Mat;
      Next_Image          : OpenCV.Core.Mat;
      Points              : Tracking_Point_Array;
      Initial_Next_Points : Tracking_Point_Array;
      Seeded              : Boolean;
      Options             : PyrLK_Options) return Point_Track_Array
   is
      Point_Input : OpenCV.Core.Mat;
      Seed_Input  : OpenCV.Core.Mat;
      Next_Points : OpenCV.Core.Mat;
      Status      : OpenCV.Core.Mat;
      Errors      : OpenCV.Core.Mat;
      Code        : C.Status := C.Success;

      procedure Previous_Callback (Previous_Handle : Bridge.Input_Mat_Handle) is
         procedure Next_Callback (Next_Handle : Bridge.Input_Mat_Handle) is
            procedure Point_Callback (Point_Handle : Bridge.Input_Mat_Handle) is
               procedure Next_Point_Callback (Next_Point_Handle : Bridge.Output_Mat_Handle) is
                  procedure Status_Callback (Status_Handle : Bridge.Output_Mat_Handle) is
                     procedure Error_Callback (Error_Handle : Bridge.Output_Mat_Handle) is
                        procedure Seed_Callback (Seed_Handle : Bridge.Input_Mat_Handle) is
                        begin
                           Code := C.Track_PyrLK_Seeded
                             (Previous_Handle, Next_Handle, Point_Handle, Seed_Handle,
                              Next_Point_Handle, Status_Handle, Error_Handle,
                              Interfaces.Integer_32 (Options.Window_Size.Width),
                              Interfaces.Integer_32 (Options.Window_Size.Height),
                              Interfaces.Integer_32 (Options.Max_Level),
                              Interfaces.Integer_32 (Options.Maximum_Iterations),
                              Interfaces.C.double (Options.Epsilon),
                              Interfaces.C.double (Options.Min_Eigenvalue_Threshold));
                        end Seed_Callback;
                     begin
                        if Seeded then
                           Bridge.With_Input_Handle (Seed_Input, Seed_Callback'Access);
                        else
                           Code := C.Track_PyrLK
                             (Previous_Handle, Next_Handle, Point_Handle,
                              Next_Point_Handle, Status_Handle, Error_Handle,
                              Interfaces.Integer_32 (Options.Window_Size.Width),
                              Interfaces.Integer_32 (Options.Window_Size.Height),
                              Interfaces.Integer_32 (Options.Max_Level),
                              Interfaces.Integer_32 (Options.Maximum_Iterations),
                              Interfaces.C.double (Options.Epsilon),
                              Interfaces.C.double (Options.Min_Eigenvalue_Threshold));
                        end if;
                     end Error_Callback;
                  begin
                     Bridge.With_Output_Handle (Errors, Error_Callback'Access);
                  end Status_Callback;
               begin
                  Bridge.With_Output_Handle (Status, Status_Callback'Access);
               end Next_Point_Callback;
            begin
               Bridge.With_Output_Handle (Next_Points, Next_Point_Callback'Access);
            end Point_Callback;
         begin
            Bridge.With_Input_Handle (Point_Input, Point_Callback'Access);
         end Next_Callback;
      begin
         Bridge.With_Input_Handle (Next_Image, Next_Callback'Access);
      end Previous_Callback;
   begin
      Validate_Images (Previous_Image, Next_Image);
      Validate (Points);
      Validate (Options);
      if Seeded then
         if Initial_Next_Points'Length /= Points'Length then
            raise OpenCV.OpenCV_Error with "PyrLK point and seed counts differ";
         end if;
         Validate (Initial_Next_Points);
      end if;

      if Points'Length = 0 then
         return Result : Point_Track_Array (Points'Range) do
            null;
         end return;
      end if;

      Point_Input := Point_Matrix (Points);
      if Seeded then
         Seed_Input := Point_Matrix (Initial_Next_Points);
      end if;
      --  Outputs start empty; the shim publishes complete temporary cv::Mat
      --  results into these Core-owned headers only after native success.
      Bridge.With_Input_Handle (Previous_Image, Previous_Callback'Access);
      C.Check (Code, "Video.Track_PyrLK");

      if Next_Points.Rows /= Points'Length or else Next_Points.Columns /= 1
        or else Next_Points.Depth /= OpenCV.Core.Float32 or else Next_Points.Channels /= 2
        or else Status.Rows /= Points'Length or else Status.Columns /= 1
        or else Status.Depth /= OpenCV.Core.UInt8 or else Status.Channels /= 1
        or else Errors.Rows /= Points'Length or else Errors.Columns /= 1
        or else Errors.Depth /= OpenCV.Core.Float32 or else Errors.Channels /= 1
      then
         raise OpenCV.OpenCV_Error with "Invalid native PyrLK output schema";
      end if;

      return Result : Point_Track_Array (Points'Range) do
         for I in Result'Range loop
            declare
               Row : constant Natural := I - Points'First;
               Flag : constant OpenCV.UInt8_Value := OpenCV.Core.UInt8_Access.Get (Status, Row, 0);
            begin
               if Flag not in 0 | 1 then
                  raise OpenCV.OpenCV_Error with "Invalid native PyrLK status value";
               end if;
               Result (I).Previous_Point := Points (I);
               Result (I).Tracked := Flag = 1;
               if Result (I).Tracked then
                  declare
                     Native_Point : constant Vec2.Vector := Vec2_Access.Get (Next_Points, Row, 0);
                     Native_Error : constant OpenCV.Float32_Value := Float_Access.Get (Errors, Row, 0);
                  begin
                     if not Is_Finite (Native_Point (0)) or else not Is_Finite (Native_Point (1))
                       or else not Is_Finite (Native_Error) or else Native_Error < 0.0
                     then
                        raise OpenCV.OpenCV_Error with "PyrLK produced an invalid successful result";
                     end if;
                     Result (I).Next_Point := (X => Native_Point (0), Y => Native_Point (1));
                     Result (I).Error := Native_Error;
                  end;
               else
                  Result (I).Next_Point := Points (I);
                  Result (I).Error := 0.0;
               end if;
            end;
         end loop;
      end return;
   end Track_Internal;

   function Track_PyrLK
     (Previous_Image : OpenCV.Core.Mat;
      Next_Image     : OpenCV.Core.Mat;
      Points         : Tracking_Point_Array;
      Options        : PyrLK_Options := (others => <>)) return Point_Track_Array is
   begin
      return Track_Internal (Previous_Image, Next_Image, Points, [], False, Options);
   end Track_PyrLK;

   function Track_PyrLK
     (Previous_Image      : OpenCV.Core.Mat;
      Next_Image          : OpenCV.Core.Mat;
      Points              : Tracking_Point_Array;
      Options             : PyrLK_Options := (others => <>);
      Initial_Next_Points : Tracking_Point_Array) return Point_Track_Array is
   begin
      return Track_Internal
        (Previous_Image, Next_Image, Points, Initial_Next_Points, True, Options);
   end Track_PyrLK;

   function Successful_Count (Tracks : Point_Track_Array) return Natural is
      Count : Natural := 0;
   begin
      for Track of Tracks loop
         if Track.Tracked then
            Count := Count + 1;
         end if;
      end loop;
      return Count;
   end Successful_Count;
end OpenCV.Video;
