with AUnit.Assertions;
with AUnit.Test_Caller;
with AUnit.Test_Fixtures;
with Ada.Unchecked_Conversion;
with Interfaces;
with OpenCV.Core;
with OpenCV.Core.UInt8_Access;
with OpenCV.Video;

package body Video_Tests is
   use AUnit.Assertions;
   use OpenCV.Video;
   use type OpenCV.Float32_Value;
   use type OpenCV.Float64_Value;
   use type OpenCV.Float32_Point;
   use type OpenCV.UInt8_Value;

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
       Expect_Error ((Window_Size => (Width => 1, Height => 21), others => <>));
       Expect_Error ((Window_Size => (Width => 21, Height => 2), others => <>));
       Expect_Error ((Window_Size => (Width => 256, Height => 21), others => <>));
       Expect_Error ((Window_Size => (Width => 21, Height => 256), others => <>));
       Expect_Error ((Max_Level => 31, others => <>));
       Expect_Error ((Max_Level => Natural'Last, others => <>));
       Expect_Error ((Maximum_Iterations => 101, others => <>));
       Expect_Error ((Maximum_Iterations => Positive'Last, others => <>));
      Expect_Error ((Epsilon => 0.0, others => <>));
       Expect_Error ((Epsilon => -1.0, others => <>));
       Expect_Error ((Epsilon => 10.01, others => <>));
      Expect_Error ((Min_Eigenvalue_Threshold => -1.0, others => <>));
       Expect_Error ((Min_Eigenvalue_Threshold => OpenCV.Float64_Value'Last, others => <>));
       declare
          function From_Bits is new Ada.Unchecked_Conversion
            (Interfaces.Unsigned_64, OpenCV.Float64_Value);
          type Bit_Array is array (Positive range <>) of Interfaces.Unsigned_64;
       begin
          for Bits of Bit_Array'(1 => 16#7FF0_0000_0000_0000#,
                                 2 => 16#7FF8_0000_0000_0000#)
          loop
             begin
                Expect_Error ((Epsilon => From_Bits (Bits), others => <>));
             exception
                --  GNAT validity checks can reject IEEE special values before
                --  they can be represented in a public Ada record.
                when Constraint_Error => null;
             end;
             begin
                Expect_Error ((Min_Eigenvalue_Threshold => From_Bits (Bits), others => <>));
             exception
                when Constraint_Error => null;
             end;
          end loop;
       end;
   end Invalid_Options;

   procedure Horizontal_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (64, 64);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 2, 0);
      Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, Standard_Points);
   begin
      for I in Tracks'Range loop
         Assert (Tracks (I).Tracked and then
                   Near (Tracks (I).Next_Point.X, Standard_Points (I).X + 2.0) and then
                   Near (Tracks (I).Next_Point.Y, Standard_Points (I).Y),
                 "horizontal translation/order mismatch");
         Assert (Tracks (I).Error >= 0.0 and then Tracks (I).Error <= 255.0,
                 "successful error is not finite nonnegative L1 evidence");
      end loop;
   end Horizontal_Translation;

   procedure Failed_And_Outside (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Points : constant Tracking_Point_Array :=
        [7 => (20.0, 20.0), 8 => (-100.0, -100.0),
         9 => (32.0, 24.0), 10 => (200.0, 200.0)];
      Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Points);
   begin
      Assert (Tracks'First = 7 and then Tracks'Last = 10, "mixed result bounds");
      for I in Tracks'Range loop
         Assert (Tracks (I).Previous_Point = Points (I), "input ordering changed");
         if I in 8 | 10 then
            Assert (not Tracks (I).Tracked and then Tracks (I).Next_Point = Points (I)
                      and then Tracks (I).Error = 0.0, "failed-track normalization");
         else
            Assert (Tracks (I).Tracked and then
                      Near (Tracks (I).Next_Point.X, Points (I).X, 0.05) and then
                      Near (Tracks (I).Next_Point.Y, Points (I).Y, 0.05), "mixed track ordering");
         end if;
      end loop;
   end Failed_And_Outside;

   procedure Textureless_Failure (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : OpenCV.Core.Mat := OpenCV.Core.Create
        (64, 64, (Depth => OpenCV.Core.UInt8, Channels => 1));
   begin
      Image.Set_To ((Component_0 => 0.0, others => 0.0));
      declare
         Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Standard_Points);
      begin
         Assert (Successful_Count (Tracks) = 0, "flat image should lose all tracks");
         for I in Tracks'Range loop
            Assert (Tracks (I).Next_Point = Standard_Points (I) and then Tracks (I).Error = 0.0,
                    "flat-image failed output is not deterministic");
         end loop;
      end;
   end Textureless_Failure;

   procedure Boundary_Points (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Points : constant Tracking_Point_Array :=
        [1 => (1.0, 1.0), 2 => (62.0, 62.0), 3 => (-1.0, 32.0), 4 => (64.0, 32.0)];
      Tracks : constant Point_Track_Array := Track_PyrLK
        (Image, Image, Points, (Max_Level => 0, others => <>));
   begin
      for I in Tracks'Range loop
         Assert (Tracks (I).Tracked and then
                   Near (Tracks (I).Next_Point.X, Points (I).X, 0.05) and then
                   Near (Tracks (I).Next_Point.Y, Points (I).Y, 0.05),
                 "padded boundary identity differs");
      end loop;
   end Boundary_Points;

   procedure Empty_Image (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : OpenCV.Core.Mat;
   begin
      declare
         Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, Standard_Points);
         pragma Unreferenced (Tracks);
      begin
         Assert (False, "empty image accepted");
      end;
   exception
      when OpenCV.OpenCV_Error => null;
   end Empty_Image;

   procedure Noncontiguous_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous_Parent : constant OpenCV.Core.Mat := Texture (96, 96);
      Next_Parent : constant OpenCV.Core.Mat := Shift (Previous_Parent, 2, 1);
      Previous : constant OpenCV.Core.Mat := Previous_Parent.Region ((8, 8, 64, 64));
      Next : constant OpenCV.Core.Mat := Next_Parent.Region ((8, 8, 64, 64));
      Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, Standard_Points);
   begin
      Assert (not Previous.Is_Continuous and then not Next.Is_Continuous, "fixture is contiguous");
      for I in Tracks'Range loop
         Assert (Tracks (I).Tracked and then
                   Near (Tracks (I).Next_Point.X, Standard_Points (I).X + 2.0) and then
                   Near (Tracks (I).Next_Point.Y, Standard_Points (I).Y + 1.0),
                 "noncontiguous Region translation differs");
      end loop;
   end Noncontiguous_Regions;

   procedure Value_Independence (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : OpenCV.Core.Mat := Texture (64, 64);
      Next : OpenCV.Core.Mat := Shift (Previous, 2, 1);
      Saved_Previous : constant OpenCV.Core.Mat := Previous.Clone;
      Saved_Next : constant OpenCV.Core.Mat := Next.Clone;
      Points : Tracking_Point_Array := Standard_Points;
      Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, Points);
   begin
      for R in 0 .. 63 loop
         for C in 0 .. 63 loop
            Assert (OpenCV.Core.UInt8_Access.Get (Previous, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Previous, R, C) and then
                    OpenCV.Core.UInt8_Access.Get (Next, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Next, R, C), "source pixels mutated");
         end loop;
      end loop;
      Previous.Set_To ((Component_0 => 0.0, others => 0.0));
      Next.Set_To ((Component_0 => 0.0, others => 0.0));
      Points (1) := (0.0, 0.0);
      Assert (Tracks (1).Previous_Point = Standard_Points (1) and then
                Near (Tracks (1).Next_Point.X, 22.0), "result retains mutable source storage");
   end Value_Independence;

   procedure Invalid_Points (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      function From_Bits is new Ada.Unchecked_Conversion
        (Interfaces.Unsigned_32, OpenCV.Float32_Value);
      type Values is array (Positive range <>) of OpenCV.Float32_Value;
   begin
      for Value of Values'[OpenCV.Float32_Value'Last]
      loop
         begin
            declare
               Tracks : constant Point_Track_Array := Track_PyrLK (Image, Image, [1 => (Value, 20.0)]);
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "unsafe point accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
      begin
         declare
            Tracks : constant Point_Track_Array := Track_PyrLK
              (Image, Image, [1 => (From_Bits (16#7FC0_0000#), 20.0)]);
            pragma Unreferenced (Tracks);
         begin
            Assert (False, "NaN point accepted");
         end;
      exception
         when OpenCV.OpenCV_Error | Constraint_Error => null;
      end;
   end Invalid_Points;

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
       Result.Add_Test (Caller.Create ("horizontal translation and finite errors", Horizontal_Translation'Access));
       Result.Add_Test (Caller.Create ("mixed status ordering and outside points", Failed_And_Outside'Access));
       Result.Add_Test (Caller.Create ("textureless deterministic failure", Textureless_Failure'Access));
       Result.Add_Test (Caller.Create ("padded boundary points", Boundary_Points'Access));
       Result.Add_Test (Caller.Create ("empty image rejected", Empty_Image'Access));
       Result.Add_Test (Caller.Create ("noncontiguous Core Regions", Noncontiguous_Regions'Access));
       Result.Add_Test (Caller.Create ("source immutability and value independence", Value_Independence'Access));
       Result.Add_Test (Caller.Create ("nonfinite and unsafe points rejected", Invalid_Points'Access));
      return Result;
   end Suite;
end Video_Tests;
