with AUnit.Assertions;
with AUnit.Test_Caller;
with AUnit.Test_Fixtures;
with OpenCV.Core;
with OpenCV.Core.UInt8_Access;
with OpenCV.Video;

package body Video_Tests is
   use AUnit.Assertions;
   use OpenCV.Video;
   use type OpenCV.Float32_Value;

   type Fixture is new AUnit.Test_Fixtures.Test_Fixture with null record;

   function Texture (Rows, Columns : Positive) return OpenCV.Core.Mat is
   begin
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (Rows, Columns, (Depth => OpenCV.Core.UInt8, Channels => 1)) do
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               OpenCV.Core.UInt8_Access.Set
                 (Result, R, C,
                  OpenCV.UInt8_Value ((R * 17 + C * 29 + (R * C) mod 251) mod 256));
            end loop;
         end loop;
      end return;
   end Texture;

   function Shift (Source : OpenCV.Core.Mat; DX, DY : Integer) return OpenCV.Core.Mat is
      Rows : constant Positive := Source.Rows;
      Columns : constant Positive := Source.Columns;
   begin
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (Rows, Columns, (Depth => OpenCV.Core.UInt8, Channels => 1)) do
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               declare
                  Source_R : constant Integer := R - DY;
                  Source_C : constant Integer := C - DX;
                  Value : OpenCV.UInt8_Value := 0;
               begin
                  if Source_R >= 0 and then Source_R < Rows
                    and then Source_C >= 0 and then Source_C < Columns
                  then
                     Value := OpenCV.Core.UInt8_Access.Get (Source, Source_R, Source_C);
                  end if;
                  OpenCV.Core.UInt8_Access.Set (Result, R, C, Value);
               end;
            end loop;
         end loop;
      end return;
   end Shift;

   function Near (Left, Right : OpenCV.Float32_Value;
                  Tolerance : OpenCV.Float32_Value := 0.20) return Boolean is
     (abs (Left - Right) <= Tolerance);

   Standard_Points : constant Tracking_Point_Array :=
     [1 => (X => 20.0, Y => 20.0),
      2 => (X => 32.0, Y => 24.0),
      3 => (X => 42.0, Y => 35.0),
      4 => (X => 26.0, Y => 45.0)];

   procedure Identity_Tracking (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Standard_Points);
   begin
      Assert (Successful_Count (Tracks) = Standard_Points'Length, "identity tracks were lost");
      for I in Tracks'Range loop
         Assert (Tracks (I).Tracked, "identity point not tracked");
         Assert (Near (Tracks (I).Next_Point.X, Standard_Points (I).X, 0.05)
                 and then Near (Tracks (I).Next_Point.Y, Standard_Points (I).Y, 0.05),
                 "identity point moved");
      end loop;
   end Identity_Tracking;

   procedure Translation_Tracking (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (64, 64);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 2, 1);
      Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, Standard_Points);
   begin
      Assert (Successful_Count (Tracks) = Standard_Points'Length, "translated tracks were lost");
      for I in Tracks'Range loop
         Assert (Tracks (I).Tracked, "translated point not tracked");
         Assert (Near (Tracks (I).Next_Point.X, Standard_Points (I).X + 2.0)
                 and then Near (Tracks (I).Next_Point.Y, Standard_Points (I).Y + 1.0),
                 "translated point differs from oracle");
      end loop;
   end Translation_Tracking;

   procedure Bounds_Are_Preserved (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Points : constant Tracking_Point_Array (5 .. 7) :=
        [5 => (X => 20.0, Y => 20.0), 6 => (X => 30.0, Y => 30.0),
         7 => (X => 40.0, Y => 40.0)];
      Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Points);
   begin
      Assert (Tracks'First = Points'First and then Tracks'Last = Points'Last,
              "Track_PyrLK changed Ada array bounds");
   end Bounds_Are_Preserved;

   procedure Empty_Points (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (32, 32);
      Points : Tracking_Point_Array (1 .. 0);
      Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Points);
   begin
      Assert (Tracks'Length = 0 and then Tracks'First = 1 and then Tracks'Last = 0,
              "empty point sequence did not remain empty");
   end Empty_Points;

   procedure Mismatched_Images (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (64, 64);
      Next : constant OpenCV.Core.Mat := Texture (63, 64);
   begin
      begin
         declare
            Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, Standard_Points);
            pragma Unreferenced (Tracks);
         begin
            Assert (False, "mismatched image geometry accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Mismatched_Images;

   procedure Wrong_Depth (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (64, 64, (Depth => OpenCV.Core.Float32, Channels => 1));
   begin
      begin
         declare
            Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Standard_Points);
            pragma Unreferenced (Tracks);
         begin
            Assert (False, "Float32 image accepted by bootstrap PyrLK");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Wrong_Depth;

   procedure Wrong_Channels (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (64, 64, (Depth => OpenCV.Core.UInt8, Channels => 3));
   begin
      begin
         declare
            Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Standard_Points);
            pragma Unreferenced (Tracks);
         begin
            Assert (False, "multi-channel image accepted by bootstrap PyrLK");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Wrong_Channels;

   procedure Invalid_Options (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      procedure Expect_Error (Options : PyrLK_Options) is
      begin
         declare
            Tracks : constant Point_Track_Array :=
              Track_PyrLK (Image, Image, Standard_Points, Options);
            pragma Unreferenced (Tracks);
         begin
            Assert (False, "invalid PyrLK options accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Expect_Error;
   begin
      Expect_Error ((Window_Size => (Width => 0, Height => 21), others => <>));
      Expect_Error ((Epsilon => 0.0, others => <>));
      Expect_Error ((Min_Eigenvalue_Threshold => -1.0, others => <>));
   end Invalid_Options;

   package Caller is new AUnit.Test_Caller (Fixture);

   function Suite return AUnit.Test_Suites.Access_Test_Suite is
      Result : constant AUnit.Test_Suites.Access_Test_Suite := AUnit.Test_Suites.New_Suite;
   begin
      Result.Add_Test (Caller.Create ("PyrLK identity tracking", Identity_Tracking'Access));
      Result.Add_Test (Caller.Create ("PyrLK integer translation", Translation_Tracking'Access));
      Result.Add_Test (Caller.Create ("Ada point bounds preserved", Bounds_Are_Preserved'Access));
      Result.Add_Test (Caller.Create ("empty points remain empty", Empty_Points'Access));
      Result.Add_Test (Caller.Create ("mismatched image geometry rejected", Mismatched_Images'Access));
      Result.Add_Test (Caller.Create ("non-UInt8 image rejected", Wrong_Depth'Access));
      Result.Add_Test (Caller.Create ("multi-channel image rejected", Wrong_Channels'Access));
      Result.Add_Test (Caller.Create ("invalid PyrLK options rejected", Invalid_Options'Access));
      return Result;
   end Suite;
end Video_Tests;
