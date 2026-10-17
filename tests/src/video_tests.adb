with AUnit.Assertions;
with AUnit.Test_Caller;
with AUnit.Test_Fixtures;
with Ada.Environment_Variables;
with Ada.Directories;
with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with ECC_Fixtures;
with Interfaces;
with Interfaces.C.Strings;
with OpenCV.Core;
with OpenCV.Core.Float32_Vec2;
with OpenCV.Core.Float32_Vec2_Access;
with OpenCV.Core.UInt8_Access;
with OpenCV.Video;

package body Video_Tests is
   use AUnit.Assertions;
   use OpenCV.Video;
   use type OpenCV.Float32_Value;
   use type OpenCV.Float64_Value;
   use type OpenCV.Float32_Point;
   use type OpenCV.UInt8_Value;
   use type OpenCV.Size;

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

   procedure Seeded_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Tracks : constant Point_Track_Array := Track_PyrLK
        (Image, Image, Standard_Points, Initial_Next_Points => Standard_Points);
   begin
      for I in Tracks'Range loop
         Assert (Tracks (I).Tracked and then
                   Near (Tracks (I).Next_Point.X, Standard_Points (I).X, 0.05) and then
                   Near (Tracks (I).Next_Point.Y, Standard_Points (I).Y, 0.05) and then
                   Tracks (I).Error >= 0.0 and then Tracks (I).Error <= OpenCV.Float32_Value'Last,
                 "seeded identity/error differs");
      end loop;
   end Seeded_Identity;

   procedure Seeded_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Saved_Previous : constant OpenCV.Core.Mat := Previous.Clone;
      Saved_Next : constant OpenCV.Core.Mat := Next.Clone;
      Points : constant Tracking_Point_Array (5 .. 8) :=
        [(25.0, 25.0), (45.0, 32.0), (60.0, 50.0), (35.0, 65.0)];
      Seeds : Tracking_Point_Array (20 .. 23);
      Options : constant PyrLK_Options := (Max_Level => 0, others => <>);
   begin
      for I in Points'Range loop
         Seeds (I - Points'First + Seeds'First) :=
           (Points (I).X + 12.25, Points (I).Y + 6.75);
      end loop;
      declare
         Saved_Seeds : constant Tracking_Point_Array := Seeds;
         Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, Points, Options, Seeds);
         Plain : constant Point_Track_Array := Track_PyrLK (Previous, Next, Points, Options);
      begin
         Assert (Tracks'First = 5 and then Tracks'Last = 8, "seed bounds replaced point bounds");
         Assert (Seeds = Saved_Seeds, "seed array mutated");
         for I in Tracks'Range loop
            Assert (Tracks (I).Previous_Point = Points (I) and then Tracks (I).Tracked and then
                      Near (Tracks (I).Next_Point.X, Points (I).X + 12.0, 0.05) and then
                      Near (Tracks (I).Next_Point.Y, Points (I).Y + 7.0, 0.05) and then
                      Tracks (I).Error >= 0.0 and then Tracks (I).Error <= OpenCV.Float32_Value'Last,
                    "useful seed failed to refine translation");
            Assert (not Plain (I).Tracked or else
                      abs (Plain (I).Next_Point.X - Points (I).X - 12.0) > 5.0 or else
                      abs (Plain (I).Next_Point.Y - Points (I).Y - 7.0) > 5.0,
                    "fixture no longer distinguishes seed mode");
         end loop;
      end;
      for R in 0 .. 95 loop
         for C in 0 .. 95 loop
            Assert (OpenCV.Core.UInt8_Access.Get (Previous, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Previous, R, C) and then
                      OpenCV.Core.UInt8_Access.Get (Next, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Next, R, C), "seeded images mutated");
         end loop;
      end loop;
   end Seeded_Translation;

   procedure Seeded_Counts (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Empty : constant Tracking_Point_Array (5 .. 4) := [];
      Empty_Seeds : constant Tracking_Point_Array (20 .. 19) := [];
      Tracks : constant Point_Track_Array := Track_PyrLK
        (Image, Image, Empty, Initial_Next_Points => Empty_Seeds);
   begin
      Assert (Tracks'First = 5 and then Tracks'Last = 4, "seeded empty bounds lost");
      for Mode in 1 .. 3 loop
         begin
            declare
               Rejected : constant Point_Track_Array :=
                 (if Mode = 1 then Track_PyrLK (Image, Image, Empty, Initial_Next_Points => Standard_Points)
                  elsif Mode = 2 then Track_PyrLK (Image, Image, Standard_Points, Initial_Next_Points => Empty_Seeds)
                  else Track_PyrLK (Image, Image, Standard_Points, Initial_Next_Points => Standard_Points (1 .. 3)));
               pragma Unreferenced (Rejected);
            begin
               Assert (False, "mismatched seeds accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
      begin
         declare
            Rejected : constant Point_Track_Array := Track_PyrLK
              (Image, Image, Empty, (Max_Level => 31, others => <>), Empty_Seeds);
            pragma Unreferenced (Rejected);
         begin
            Assert (False, "empty pair skipped option validation");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      declare
         No_Image : OpenCV.Core.Mat;
      begin
         declare
            Rejected : constant Point_Track_Array := Track_PyrLK
              (No_Image, No_Image, Empty, Initial_Next_Points => Empty_Seeds);
            pragma Unreferenced (Rejected);
         begin
            Assert (False, "empty seeded pair skipped image validation");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Seeded_Counts;

   procedure Invalid_Seeds (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      function From_Bits is new Ada.Unchecked_Conversion
        (Interfaces.Unsigned_32, OpenCV.Float32_Value);
      type Bit_Array is array (Positive range <>) of Interfaces.Unsigned_32;
   begin
      begin
         declare
            Tracks : constant Point_Track_Array := Track_PyrLK
              (Image, Image, [1 => (20.0, 20.0)], Initial_Next_Points => [1 => (OpenCV.Float32_Value'Last, 20.0)]);
            pragma Unreferenced (Tracks);
         begin
            Assert (False, "unsafe seed accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      for Bits of Bit_Array'[16#7FC0_0000#, 16#7F80_0000#] loop
         begin
            declare
               Tracks : constant Point_Track_Array := Track_PyrLK
                 (Image, Image, [1 => (20.0, 20.0)], Initial_Next_Points => [1 => (20.0, From_Bits (Bits))]);
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "nonfinite seed accepted");
            end;
         exception
            when OpenCV.OpenCV_Error | Constraint_Error => null;
         end;
      end loop;
   end Invalid_Seeds;

   procedure Seeded_Outside (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Points : constant Tracking_Point_Array := [(-1.0, 32.0), (20.0, 20.0), (32.0, 24.0)];
      Seeds : constant Tracking_Point_Array := [(-1.0, 32.0), (-100.0, -100.0), (536_870_912.0, -536_870_912.0)];
      Tracks : constant Point_Track_Array := Track_PyrLK
        (Image, Image, Points, (Max_Level => 0, others => <>), Seeds);
   begin
      Assert (Tracks (1).Tracked and then Near (Tracks (1).Next_Point.X, -1.0, 0.05),
              "padded out-of-image seed rejected");
      for I in 2 .. 3 loop
         Assert (not Tracks (I).Tracked and then Tracks (I).Next_Point = Points (I) and then
                   Tracks (I).Error = 0.0, "failed seed exposed prediction/native garbage");
      end loop;
   end Seeded_Outside;

   procedure Seeded_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous_Parent : constant OpenCV.Core.Mat := Texture (96, 96);
      Next_Parent : constant OpenCV.Core.Mat := Shift (Previous_Parent, 2, 1);
      Previous : constant OpenCV.Core.Mat := Previous_Parent.Region ((8, 8, 64, 64));
      Next : constant OpenCV.Core.Mat := Next_Parent.Region ((8, 8, 64, 64));
      Seeds : Tracking_Point_Array (Standard_Points'Range);
   begin
      for I in Seeds'Range loop
         Seeds (I) := (Standard_Points (I).X + 2.25, Standard_Points (I).Y + 0.75);
      end loop;
      declare
         Tracks : constant Point_Track_Array := Track_PyrLK
           (Previous, Next, Standard_Points, Initial_Next_Points => Seeds);
      begin
         Assert (not Previous.Is_Continuous and then not Next.Is_Continuous, "seeded Region is contiguous");
         for I in Tracks'Range loop
            Assert (Tracks (I).Tracked and then
                      Near (Tracks (I).Next_Point.X, Standard_Points (I).X + 2.0) and then
                      Near (Tracks (I).Next_Point.Y, Standard_Points (I).Y + 1.0), "seeded Region differs");
         end loop;
      end;
   end Seeded_Regions;

   FB_Points : constant Tracking_Point_Array (5 .. 8) :=
     [(25.0, 25.0), (45.0, 32.0), (60.0, 50.0), (35.0, 65.0)];
   FB_Options : constant Forward_Backward_Options :=
     (Tracking => (Max_Level => 0, others => <>), Maximum_Round_Trip_Error => 0.05);

   procedure Assert_Unavailable
     (Track : Forward_Backward_Track; Original : OpenCV.Float32_Point) is
   begin
      Assert (not Track.Backward_Tracked and then not Track.Consistent and then
                Track.Recovered_Previous_Point = Original and then Track.Round_Trip_Error = 0.0,
              "unavailable backward result not deterministic");
   end Assert_Unavailable;

   procedure Assert_Good (Track : Forward_Backward_Track; Original : OpenCV.Float32_Point) is
   begin
      Assert (Track.Forward.Tracked and then Track.Backward_Tracked and then Track.Consistent,
              "good round trip rejected");
      Assert (Track.Round_Trip_Error >= 0.0 and then Track.Round_Trip_Error < 0.05 and then
                Near (Track.Recovered_Previous_Point.X, Original.X, 0.05) and then
                Near (Track.Recovered_Previous_Point.Y, Original.Y, 0.05), "poor recovery");
   end Assert_Good;

   procedure FB_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Image, Image, Standard_Points);
   begin
      for I in Tracks'Range loop
         Assert_Good (Tracks (I), Standard_Points (I));
      end loop;
   end FB_Identity;

   procedure FB_Small_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 2, 1);
      Options : constant Forward_Backward_Options :=
        (Tracking => (Max_Level => 1, others => <>), Maximum_Round_Trip_Error => 0.05);
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Previous, Next, FB_Points, Options);
      Ordinary : constant Point_Track_Array := Track_PyrLK (Previous, Next, FB_Points, Options.Tracking);
   begin
      for I in Tracks'Range loop
         Assert_Good (Tracks (I), FB_Points (I));
         Assert (Tracks (I).Forward = Ordinary (I), "ordinary PyrLK behavior changed");
         Assert (Near (Tracks (I).Forward.Next_Point.X, FB_Points (I).X + 2.0, 0.05) and then
                   Near (Tracks (I).Forward.Next_Point.Y, FB_Points (I).Y + 1.0, 0.05),
                 "forward translation differs");
      end loop;
   end FB_Small_Translation;

   procedure FB_Seeded_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Saved_Previous : constant OpenCV.Core.Mat := Previous.Clone;
      Saved_Next : constant OpenCV.Core.Mat := Next.Clone;
      Points : Tracking_Point_Array := FB_Points;
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      for I in Points'Range loop
         Seeds (I - Points'First + Seeds'First) := (Points (I).X + 12.25, Points (I).Y + 6.75);
      end loop;
      declare
         Saved_Seeds : constant Tracking_Point_Array := Seeds;
         Tracks : constant Forward_Backward_Track_Array :=
           Track_PyrLK_Forward_Backward (Previous, Next, Points, FB_Options, Seeds);
         Ordinary : constant Point_Track_Array :=
           Track_PyrLK (Previous, Next, Points, FB_Options.Tracking, Seeds);
      begin
         Assert (Tracks'First = Points'First and then Tracks'Last = Points'Last, "seed bounds leaked");
         Assert (Points = FB_Points and then Seeds = Saved_Seeds, "caller arrays mutated");
         for I in Tracks'Range loop
            Assert_Good (Tracks (I), Points (I));
            Assert (Tracks (I).Forward = Ordinary (I), "seeded PyrLK behavior changed");
            Assert (Near (Tracks (I).Forward.Next_Point.X, Points (I).X + 12.0, 0.05) and then
                      Near (Tracks (I).Forward.Next_Point.Y, Points (I).Y + 7.0, 0.05),
                    "large seeded translation differs");
         end loop;
         Points (5) := (0.0, 0.0);
         Seeds (20) := (0.0, 0.0);
         Assert (Tracks (5).Forward.Previous_Point = FB_Points (5), "result aliases caller points");
      end;
      for R in 0 .. 95 loop
         for C in 0 .. 95 loop
            Assert (OpenCV.Core.UInt8_Access.Get (Previous, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Previous, R, C) and then
                    OpenCV.Core.UInt8_Access.Get (Next, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Next, R, C), "FB images mutated");
         end loop;
      end loop;
   end FB_Seeded_Translation;

   procedure FB_Inconsistent (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Previous, Next, FB_Points,
                                     (Tracking => FB_Options.Tracking, Maximum_Round_Trip_Error => 1.0));
   begin
      Assert (Tracks (5).Forward.Tracked and then Tracks (5).Backward_Tracked and then
                Tracks (5).Round_Trip_Error > 2.0 and then not Tracks (5).Consistent,
              "native forward-successful inconsistency not diagnosed");
   end FB_Inconsistent;

   procedure FB_Thresholds (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Points : constant Tracking_Point_Array := [7 => FB_Points (5)];
      Measured : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Previous, Next, Points, FB_Options);
      Distance : constant OpenCV.Float32_Value := Measured (7).Round_Trip_Error;
   begin
      Assert (Measured (7).Backward_Tracked and then Distance > 2.0, "threshold fixture not measurable");
      for Mode in 1 .. 3 loop
         declare
            Threshold : constant OpenCV.Float32_Value :=
              (case Mode is when 1 => Distance + 0.001, when 2 => Distance - 0.001, when others => Distance);
            Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
              (Previous, Next, Points,
               (Tracking => FB_Options.Tracking, Maximum_Round_Trip_Error => Threshold));
         begin
            Assert (Tracks (7).Round_Trip_Error = Distance and then
                      Tracks (7).Consistent = (Mode /= 2), "threshold <= comparison differs");
         end;
      end loop;
   end FB_Thresholds;

   procedure FB_Zero (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 2, 1);
      Options : constant Forward_Backward_Options :=
        (Tracking => (Max_Level => 1, others => <>), Maximum_Round_Trip_Error => 0.0);
      Identity : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Previous, Previous, FB_Points, Options);
      Moved : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Previous, Next, FB_Points, Options);
      Positive_Distance : Boolean := False;
   begin
      for I in Identity'Range loop
         Assert (Identity (I).Backward_Tracked and then Identity (I).Round_Trip_Error = 0.0
                   and then Identity (I).Consistent, "exact identity with zero threshold failed");
         Assert (Moved (I).Backward_Tracked and then
                   Moved (I).Consistent = (Moved (I).Round_Trip_Error = 0.0), "zero threshold was clamped");
         Positive_Distance := Positive_Distance or Moved (I).Round_Trip_Error > 0.0;
      end loop;
      Assert (Positive_Distance, "zero-threshold rejection not exercised");
   end FB_Zero;

   procedure FB_Forward_Failure (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : OpenCV.Core.Mat := OpenCV.Core.Create
        (64, 64, (Depth => OpenCV.Core.UInt8, Channels => 1));
      Tracks : Forward_Backward_Track_Array (Standard_Points'Range);
   begin
      Image.Set_To ((others => 0.0));
      Tracks := Track_PyrLK_Forward_Backward (Image, Image, Standard_Points);
      for I in Tracks'Range loop
         Assert (not Tracks (I).Forward.Tracked and then Tracks (I).Forward.Error = 0.0 and then
                   Tracks (I).Forward.Next_Point = Standard_Points (I), "forward failure differs");
         Assert_Unavailable (Tracks (I), Standard_Points (I));
      end loop;
   end FB_Forward_Failure;

   procedure FB_Backward_Failure (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : OpenCV.Core.Mat := OpenCV.Core.Create
        (96, 96, (Depth => OpenCV.Core.UInt8, Channels => 1));
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      Next.Set_To ((others => 0.0));
      for I in FB_Points'Range loop
         Seeds (I - 5 + 20) := (FB_Points (I).X + 12.25, FB_Points (I).Y + 6.75);
      end loop;
      declare
         Tracks : constant Forward_Backward_Track_Array :=
           Track_PyrLK_Forward_Backward (Previous, Next, FB_Points, FB_Options, Seeds);
      begin
         for I in Tracks'Range loop
            Assert (Tracks (I).Forward.Tracked, "blank destination forward fixture no longer succeeds");
            Assert_Unavailable (Tracks (I), FB_Points (I));
         end loop;
      end;
   end FB_Backward_Failure;

   procedure FB_Compact_Mapping (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Points : constant Tracking_Point_Array (7 .. 11) :=
        [(25.0, 25.0), (-1000.0, -1000.0), (45.0, 32.0), (1000.0, 1000.0), (60.0, 50.0)];
      Seeds : constant Tracking_Point_Array (20 .. 24) :=
        [(37.25, 31.75), (-1000.0, -1000.0), (77.0, 52.0), (1000.0, 1000.0), (72.25, 56.75)];
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Previous, Next, Points, FB_Options, Seeds);
   begin
      Assert (Tracks'First = 7 and then Tracks'Last = 11, "compact bounds exposed");
      Assert_Good (Tracks (7), Points (7));
      Assert_Good (Tracks (11), Points (11));
      Assert (Tracks (9).Forward.Tracked and then Tracks (9).Backward_Tracked and then
                Tracks (9).Round_Trip_Error > 2.0 and then not Tracks (9).Consistent,
              "interleaved inconsistent slot mis-mapped");
      for I in Points'Range loop
         Assert (Tracks (I).Forward.Previous_Point = Points (I), "source slot mis-mapped");
         if I = 8 or else I = 10 then
            Assert (not Tracks (I).Forward.Tracked, "outside source unexpectedly succeeded");
            Assert_Unavailable (Tracks (I), Points (I));
         else
            declare
               Single : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                 (Previous, Next, [I => Points (I)], FB_Options,
                  [1 => Seeds (I - Points'First + Seeds'First)]);
            begin
               Assert (Tracks (I) = Single (I), "compact entry differs from independent single call");
            end;
         end if;
      end loop;
   end FB_Compact_Mapping;

   procedure FB_Extreme_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Points : constant Tracking_Point_Array := [Positive'Last => (20.0, 20.0)];
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Image, Image, Points, Initial_Next_Points => [3 => (20.0, 20.0)]);
   begin
      Assert (Tracks'First = Positive'Last and then Tracks'Last = Positive'Last, "extreme bounds lost");
      Assert_Good (Tracks (Positive'Last), Points (Positive'Last));
   end FB_Extreme_Bounds;

   procedure FB_Empty_Counts (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Empty : constant Tracking_Point_Array (7 .. 6) := [];
      Plain : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward (Image, Image, Empty);
      Seeded : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
        (Image, Image, Empty, Initial_Next_Points => Tracking_Point_Array'(20 .. 19 => <>));
   begin
      Assert (Plain'First = 7 and then Plain'Last = 6 and then
                Seeded'First = 7 and then Seeded'Last = 6, "FB empty bounds lost");
      for Mode in 1 .. 3 loop
         begin
            declare
               Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                 (Image, Image, (if Mode = 1 then Empty else Standard_Points),
                  Initial_Next_Points => (if Mode = 2 then Empty else [1 => (20.0, 20.0)]));
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "FB mismatched seed count accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end FB_Empty_Counts;

   procedure FB_Invalid_Threshold (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      function From_Bits is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, OpenCV.Float32_Value);
   begin
      for Mode in 1 .. 4 loop
         begin
            declare
               Threshold : constant OpenCV.Float32_Value := (case Mode is
                 when 1 => -0.01, when 2 => From_Bits (16#7FC0_0000#),
                 when 3 => From_Bits (16#7F80_0000#), when others => From_Bits (16#FF80_0000#));
               Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                 (Image, Image, Standard_Points, (Maximum_Round_Trip_Error => Threshold, others => <>));
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "invalid threshold accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
            when Constraint_Error => Assert (Mode /= 1, "negative threshold hit validity barrier");
         end;
      end loop;
      declare
         Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
           (Image, Image, Standard_Points,
            (Maximum_Round_Trip_Error => OpenCV.Float32_Value'Last, others => <>));
      begin
         Assert_Good (Tracks (1), Standard_Points (1));
      end;
   end FB_Invalid_Threshold;

   procedure FB_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous_Parent : constant OpenCV.Core.Mat := Texture (96, 96);
      Next_Parent : constant OpenCV.Core.Mat := Shift (Previous_Parent, 2, 1);
      Previous : constant OpenCV.Core.Mat := Previous_Parent.Region ((8, 8, 64, 64));
      Next : constant OpenCV.Core.Mat := Next_Parent.Region ((8, 8, 64, 64));
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      Assert (not Previous.Is_Continuous and then not Next.Is_Continuous, "FB Regions contiguous");
      for I in Standard_Points'Range loop
         Seeds (I + 19) := (Standard_Points (I).X + 2.25, Standard_Points (I).Y + 0.75);
      end loop;
      declare
         Plain : constant Forward_Backward_Track_Array :=
           Track_PyrLK_Forward_Backward (Previous, Next, Standard_Points);
         Seeded : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
           (Previous, Next, Standard_Points, Initial_Next_Points => Seeds);
      begin
         for I in Plain'Range loop
            Assert_Good (Plain (I), Standard_Points (I));
            Assert_Good (Seeded (I), Standard_Points (I));
         end loop;
      end;
   end FB_Regions;

   procedure FB_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Empty_Image : OpenCV.Core.Mat;
   begin
      for Mode in 1 .. 4 loop
         begin
            declare
               Options : Forward_Backward_Options;
            begin
               if Mode = 2 then
                  Options.Tracking.Max_Level := 31;
               end if;
               declare
                  Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                    ((if Mode = 1 then Empty_Image else Image), Image,
                     (if Mode <= 2 then Tracking_Point_Array'(7 .. 6 => <>) else Standard_Points),
                     Options, Initial_Next_Points =>
                       (if Mode <= 2 then Tracking_Point_Array'(20 .. 19 => <>)
                        elsif Mode = 3 then Tracking_Point_Array'(1 .. 4 => (OpenCV.Float32_Value'Last, 0.0))
                        else Tracking_Point_Array'(1 .. 4 => (536_871_040.0, 0.0))));
                  pragma Unreferenced (Tracks);
               begin
                  Assert (False, "FB bypassed existing validation");
               end;
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end FB_Validation;

   procedure FB_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Points : constant Tracking_Point_Array (7 .. 11) :=
        [(25.0, 25.0), (-1000.0, -1000.0), (45.0, 32.0), (1000.0, 1000.0), (60.0, 50.0)];
      Seeds : Tracking_Point_Array (20 .. 24);
      Compared : Natural := 0;
      function Close (Actual : OpenCV.Float32_Value; Expected : OpenCV.Float64_Value) return Boolean is
        (abs (OpenCV.Float64_Value (Actual) - Expected) <= 1.0E-5);
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value ("VIDEO_FORWARD_BACKWARD_ORACLE", "../obj/oracle/forward-backward.txt"));
      for I in Points'Range loop
         Seeds (I - 7 + 20) := (Points (I).X + 12.25, Points (I).Y + 6.75);
      end loop;
      for Mode in 0 .. 3 loop
         declare
            Next : OpenCV.Core.Mat := Shift (Previous, (if Mode = 0 then 2 else 12),
                                            (if Mode = 0 then 1 else 7));
            Options : constant Forward_Backward_Options :=
              (Tracking => (Max_Level => (if Mode = 0 then 1 else 0), others => <>), others => <>);
         begin
            if Mode = 3 then
               Next.Set_To ((others => 0.0));
            end if;
            declare
               Tracks : constant Forward_Backward_Track_Array :=
                 (if Mode = 1 or else Mode = 3 then
                    Track_PyrLK_Forward_Backward (Previous, Next, Points, Options, Seeds)
                  else Track_PyrLK_Forward_Backward (Previous, Next, Points, Options));
            begin
               for I in Tracks'Range loop
                  declare
                     Native_Mode, Index, Forward_Status, Backward_Status : Integer;
                     X, Y, Recovered_X, Recovered_Y, Distance : OpenCV.Float64_Value;
                  begin
                     Integers.Get (File, Native_Mode);
                     Integers.Get (File, Index);
                     Integers.Get (File, Forward_Status);
                     Floats.Get (File, X);
                     Floats.Get (File, Y);
                     Integers.Get (File, Backward_Status);
                     Floats.Get (File, Recovered_X);
                     Floats.Get (File, Recovered_Y);
                     Floats.Get (File, Distance);
                     Assert (Native_Mode = Mode and then Index = I, "oracle order differs");
                     Assert (Tracks (I).Forward.Tracked = (Forward_Status = 1) and then
                               Tracks (I).Backward_Tracked = (Backward_Status = 1), "oracle statuses differ");
                     Assert (Close (Tracks (I).Forward.Next_Point.X, X) and then
                               Close (Tracks (I).Forward.Next_Point.Y, Y) and then
                               Close (Tracks (I).Recovered_Previous_Point.X, Recovered_X) and then
                               Close (Tracks (I).Recovered_Previous_Point.Y, Recovered_Y) and then
                               Close (Tracks (I).Round_Trip_Error, Distance), "direct C++/Ada diagnostics differ");
                     Compared := Compared + 1;
                  end;
               end loop;
            end;
         end;
      end loop;
      Assert (Compared = 20, "oracle comparison count differs");
      Ada.Text_IO.Close (File);
   exception
      when others =>
         if Ada.Text_IO.Is_Open (File) then
            Ada.Text_IO.Close (File);
         end if;
         raise;
   end FB_Direct_Oracle;

   procedure Pyramid_FB_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), Standard_Points);
   begin
      for I in Tracks'Range loop
         Assert_Good (Tracks (I), Standard_Points (I));
      end loop;
   end Pyramid_FB_Identity;

   procedure Pyramid_FB_Small_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 2, 1);
      Options : constant Forward_Backward_Options :=
        (Tracking => (Max_Level => 1, others => <>), Maximum_Round_Trip_Error => 0.05);
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), FB_Points, Options);
      Ordinary : constant Point_Track_Array := Track_PyrLK (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), FB_Points, Options.Tracking);
   begin
      for I in Tracks'Range loop
         Assert_Good (Tracks (I), FB_Points (I));
         Assert (Tracks (I).Forward = Ordinary (I), "ordinary PyrLK behavior changed");
         Assert (Near (Tracks (I).Forward.Next_Point.X, FB_Points (I).X + 2.0, 0.05) and then
                   Near (Tracks (I).Forward.Next_Point.Y, FB_Points (I).Y + 1.0, 0.05),
                 "forward translation differs");
      end loop;
   end Pyramid_FB_Small_Translation;

   procedure Pyramid_FB_Seeded_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Saved_Previous : constant OpenCV.Core.Mat := Previous.Clone;
      Saved_Next : constant OpenCV.Core.Mat := Next.Clone;
      Points : Tracking_Point_Array := FB_Points;
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      for I in Points'Range loop
         Seeds (I - Points'First + Seeds'First) := (Points (I).X + 12.25, Points (I).Y + 6.75);
      end loop;
      declare
         Saved_Seeds : constant Tracking_Point_Array := Seeds;
         Tracks : constant Forward_Backward_Track_Array :=
           Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Points, FB_Options, Seeds);
         Ordinary : constant Point_Track_Array :=
           Track_PyrLK (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Points, FB_Options.Tracking, Seeds);
      begin
         Assert (Tracks'First = Points'First and then Tracks'Last = Points'Last, "seed bounds leaked");
         Assert (Points = FB_Points and then Seeds = Saved_Seeds, "caller arrays mutated");
         for I in Tracks'Range loop
            Assert_Good (Tracks (I), Points (I));
            Assert (Tracks (I).Forward = Ordinary (I), "seeded PyrLK behavior changed");
            Assert (Near (Tracks (I).Forward.Next_Point.X, Points (I).X + 12.0, 0.05) and then
                      Near (Tracks (I).Forward.Next_Point.Y, Points (I).Y + 7.0, 0.05),
                    "large seeded translation differs");
         end loop;
         Points (5) := (0.0, 0.0);
         Seeds (20) := (0.0, 0.0);
         Assert (Tracks (5).Forward.Previous_Point = FB_Points (5), "result aliases caller points");
      end;
      for R in 0 .. 95 loop
         for C in 0 .. 95 loop
            Assert (OpenCV.Core.UInt8_Access.Get (Previous, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Previous, R, C) and then
                    OpenCV.Core.UInt8_Access.Get (Next, R, C) =
                      OpenCV.Core.UInt8_Access.Get (Saved_Next, R, C), "FB images mutated");
         end loop;
      end loop;
   end Pyramid_FB_Seeded_Translation;

   procedure Pyramid_FB_Inconsistent (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), FB_Points,
                                     (Tracking => FB_Options.Tracking, Maximum_Round_Trip_Error => 1.0));
   begin
      Assert (Tracks (5).Forward.Tracked and then Tracks (5).Backward_Tracked and then
                Tracks (5).Round_Trip_Error > 2.0 and then not Tracks (5).Consistent,
              "native forward-successful inconsistency not diagnosed");
   end Pyramid_FB_Inconsistent;

   procedure Pyramid_FB_Thresholds (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Points : constant Tracking_Point_Array := [7 => FB_Points (5)];
      Measured : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Points, FB_Options);
      Distance : constant OpenCV.Float32_Value := Measured (7).Round_Trip_Error;
   begin
      Assert (Measured (7).Backward_Tracked and then Distance > 2.0, "threshold fixture not measurable");
      for Mode in 1 .. 3 loop
         declare
            Threshold : constant OpenCV.Float32_Value :=
              (case Mode is when 1 => Distance + 0.001, when 2 => Distance - 0.001, when others => Distance);
            Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
              (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Points,
               (Tracking => FB_Options.Tracking, Maximum_Round_Trip_Error => Threshold));
         begin
            Assert (Tracks (7).Round_Trip_Error = Distance and then
                      Tracks (7).Consistent = (Mode /= 2), "threshold <= comparison differs");
         end;
      end loop;
   end Pyramid_FB_Thresholds;

   procedure Pyramid_FB_Zero (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 2, 1);
      Options : constant Forward_Backward_Options :=
        (Tracking => (Max_Level => 1, others => <>), Maximum_Round_Trip_Error => 0.0);
      Identity : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Previous), FB_Points, Options);
      Moved : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), FB_Points, Options);
      Positive_Distance : Boolean := False;
   begin
      for I in Identity'Range loop
         Assert (Identity (I).Backward_Tracked and then Identity (I).Round_Trip_Error = 0.0
                   and then Identity (I).Consistent, "exact identity with zero threshold failed");
         Assert (Moved (I).Backward_Tracked and then
                   Moved (I).Consistent = (Moved (I).Round_Trip_Error = 0.0), "zero threshold was clamped");
         Positive_Distance := Positive_Distance or Moved (I).Round_Trip_Error > 0.0;
      end loop;
      Assert (Positive_Distance, "zero-threshold rejection not exercised");
   end Pyramid_FB_Zero;

   procedure Pyramid_FB_Forward_Failure (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : OpenCV.Core.Mat := OpenCV.Core.Create
        (64, 64, (Depth => OpenCV.Core.UInt8, Channels => 1));
      Tracks : Forward_Backward_Track_Array (Standard_Points'Range);
   begin
      Image.Set_To ((others => 0.0));
      Tracks := Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), Standard_Points);
      for I in Tracks'Range loop
         Assert (not Tracks (I).Forward.Tracked and then Tracks (I).Forward.Error = 0.0 and then
                   Tracks (I).Forward.Next_Point = Standard_Points (I), "forward failure differs");
         Assert_Unavailable (Tracks (I), Standard_Points (I));
      end loop;
   end Pyramid_FB_Forward_Failure;

   procedure Pyramid_FB_Backward_Failure (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : OpenCV.Core.Mat := OpenCV.Core.Create
        (96, 96, (Depth => OpenCV.Core.UInt8, Channels => 1));
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      Next.Set_To ((others => 0.0));
      for I in FB_Points'Range loop
         Seeds (I - 5 + 20) := (FB_Points (I).X + 12.25, FB_Points (I).Y + 6.75);
      end loop;
      declare
         Tracks : constant Forward_Backward_Track_Array :=
           Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), FB_Points, FB_Options, Seeds);
      begin
         for I in Tracks'Range loop
            Assert (Tracks (I).Forward.Tracked, "blank destination forward fixture no longer succeeds");
            Assert_Unavailable (Tracks (I), FB_Points (I));
         end loop;
      end;
   end Pyramid_FB_Backward_Failure;

   procedure Pyramid_FB_Compact_Mapping (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Points : constant Tracking_Point_Array (7 .. 11) :=
        [(25.0, 25.0), (-1000.0, -1000.0), (45.0, 32.0), (1000.0, 1000.0), (60.0, 50.0)];
      Seeds : constant Tracking_Point_Array (20 .. 24) :=
        [(37.25, 31.75), (-1000.0, -1000.0), (77.0, 52.0), (1000.0, 1000.0), (72.25, 56.75)];
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Points, FB_Options, Seeds);
   begin
      Assert (Tracks'First = 7 and then Tracks'Last = 11, "compact bounds exposed");
      Assert_Good (Tracks (7), Points (7));
      Assert_Good (Tracks (11), Points (11));
      Assert (Tracks (9).Forward.Tracked and then Tracks (9).Backward_Tracked and then
                Tracks (9).Round_Trip_Error > 2.0 and then not Tracks (9).Consistent,
              "interleaved inconsistent slot mis-mapped");
      for I in Points'Range loop
         Assert (Tracks (I).Forward.Previous_Point = Points (I), "source slot mis-mapped");
         if I = 8 or else I = 10 then
            Assert (not Tracks (I).Forward.Tracked, "outside source unexpectedly succeeded");
            Assert_Unavailable (Tracks (I), Points (I));
         else
            declare
               Single : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                 (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), [I => Points (I)], FB_Options,
                  [1 => Seeds (I - Points'First + Seeds'First)]);
               Backward : constant Point_Track_Array := Track_PyrLK
                 (Build_PyrLK_Pyramid (Next), Build_PyrLK_Pyramid (Previous),
                  [I => Tracks (I).Forward.Next_Point], FB_Options.Tracking, [20 => Points (I)]);
            begin
               Assert (Tracks (I) = Single (I), "compact entry differs from independent single call");
               Assert (Backward (I).Tracked = Tracks (I).Backward_Tracked and then
                 Backward (I).Next_Point = Tracks (I).Recovered_Previous_Point,
                 "compact recovery differs from one-point seeded backward");
            end;
         end if;
      end loop;
   end Pyramid_FB_Compact_Mapping;

   procedure Pyramid_FB_Extreme_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Points : constant Tracking_Point_Array := [Positive'Last => (20.0, 20.0)];
      Tracks : constant Forward_Backward_Track_Array :=
        Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), Points, Initial_Next_Points => [3 => (20.0, 20.0)]);
   begin
      Assert (Tracks'First = Positive'Last and then Tracks'Last = Positive'Last, "extreme bounds lost");
      Assert_Good (Tracks (Positive'Last), Points (Positive'Last));
   end Pyramid_FB_Extreme_Bounds;

   procedure Pyramid_FB_Empty_Counts (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Empty : constant Tracking_Point_Array (7 .. 6) := [];
      Plain : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), Empty);
      Seeded : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
        (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), Empty, Initial_Next_Points => Tracking_Point_Array'(20 .. 19 => <>));
   begin
      Assert (Plain'First = 7 and then Plain'Last = 6 and then
                Seeded'First = 7 and then Seeded'Last = 6, "FB empty bounds lost");
      for Mode in 1 .. 3 loop
         begin
            declare
               Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                 (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), (if Mode = 1 then Empty else Standard_Points),
                  Initial_Next_Points => (if Mode = 2 then Empty else [1 => (20.0, 20.0)]));
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "FB mismatched seed count accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end Pyramid_FB_Empty_Counts;

   procedure Pyramid_FB_Invalid_Threshold (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      function From_Bits is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, OpenCV.Float32_Value);
   begin
      for Mode in 1 .. 4 loop
         begin
            declare
               Threshold : constant OpenCV.Float32_Value := (case Mode is
                 when 1 => -0.01, when 2 => From_Bits (16#7FC0_0000#),
                 when 3 => From_Bits (16#7F80_0000#), when others => From_Bits (16#FF80_0000#));
               Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                 (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), Standard_Points, (Maximum_Round_Trip_Error => Threshold, others => <>));
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "invalid threshold accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
            when Constraint_Error => Assert (Mode /= 1, "negative threshold hit validity barrier");
         end;
      end loop;
      declare
         Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
           (Build_PyrLK_Pyramid (Image), Build_PyrLK_Pyramid (Image), Standard_Points,
            (Maximum_Round_Trip_Error => OpenCV.Float32_Value'Last, others => <>));
      begin
         Assert_Good (Tracks (1), Standard_Points (1));
      end;
   end Pyramid_FB_Invalid_Threshold;

   procedure Pyramid_FB_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous_Parent : constant OpenCV.Core.Mat := Texture (96, 96);
      Next_Parent : constant OpenCV.Core.Mat := Shift (Previous_Parent, 2, 1);
      Previous : constant OpenCV.Core.Mat := Previous_Parent.Region ((8, 8, 64, 64));
      Next : constant OpenCV.Core.Mat := Next_Parent.Region ((8, 8, 64, 64));
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      Assert (not Previous.Is_Continuous and then not Next.Is_Continuous, "FB Regions contiguous");
      for I in Standard_Points'Range loop
         Seeds (I + 19) := (Standard_Points (I).X + 2.25, Standard_Points (I).Y + 0.75);
      end loop;
      declare
         Plain : constant Forward_Backward_Track_Array :=
           Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Standard_Points);
         Seeded : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
           (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Standard_Points, Initial_Next_Points => Seeds);
      begin
         for I in Plain'Range loop
            Assert_Good (Plain (I), Standard_Points (I));
            Assert_Good (Seeded (I), Standard_Points (I));
         end loop;
      end;
   end Pyramid_FB_Regions;

   procedure Pyramid_FB_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      Empty_Image : OpenCV.Core.Mat;
   begin
      for Mode in 1 .. 4 loop
         begin
            declare
               Options : Forward_Backward_Options;
            begin
               if Mode = 2 then
                  Options.Tracking.Max_Level := 31;
               end if;
               declare
                  Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                    (Build_PyrLK_Pyramid ((if Mode = 1 then Empty_Image else Image)), Build_PyrLK_Pyramid (Image),
                     (if Mode <= 2 then Tracking_Point_Array'(7 .. 6 => <>) else Standard_Points),
                     Options, Initial_Next_Points =>
                       (if Mode <= 2 then Tracking_Point_Array'(20 .. 19 => <>)
                        elsif Mode = 3 then Tracking_Point_Array'(1 .. 4 => (OpenCV.Float32_Value'Last, 0.0))
                        else Tracking_Point_Array'(1 .. 4 => (536_871_040.0, 0.0))));
                  pragma Unreferenced (Tracks);
               begin
                  Assert (False, "FB bypassed existing validation");
               end;
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end Pyramid_FB_Validation;

   procedure Pyramid_FB_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Points : constant Tracking_Point_Array (7 .. 11) :=
        [(25.0, 25.0), (-1000.0, -1000.0), (45.0, 32.0), (1000.0, 1000.0), (60.0, 50.0)];
      Seeds : Tracking_Point_Array (20 .. 24);
      Compared : Natural := 0;
      function Close (Actual : OpenCV.Float32_Value; Expected : OpenCV.Float64_Value) return Boolean is
        (abs (OpenCV.Float64_Value (Actual) - Expected) <= 1.0E-5);
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value ("VIDEO_FORWARD_BACKWARD_ORACLE", "../obj/oracle/forward-backward.txt") & ".pyramids");
      for I in Points'Range loop
         Seeds (I - 7 + 20) := (Points (I).X + 12.25, Points (I).Y + 6.75);
      end loop;
      for Mode in 0 .. 3 loop
         declare
            Next : OpenCV.Core.Mat := Shift (Previous, (if Mode = 0 then 2 else 12),
                                            (if Mode = 0 then 1 else 7));
            Options : constant Forward_Backward_Options :=
              (Tracking => (Max_Level => (if Mode = 0 then 1 else 0), others => <>), others => <>);
         begin
            if Mode = 3 then
               Next.Set_To ((others => 0.0));
            end if;
            declare
               Tracks : constant Forward_Backward_Track_Array :=
                 (if Mode = 1 or else Mode = 3 then
                    Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Points, Options, Seeds)
                  else Track_PyrLK_Forward_Backward (Build_PyrLK_Pyramid (Previous), Build_PyrLK_Pyramid (Next), Points, Options));
            begin
               for I in Tracks'Range loop
                  declare
                     Native_Mode, Index, Forward_Status, Backward_Status : Integer;
                     X, Y, Recovered_X, Recovered_Y, Distance, Error : OpenCV.Float64_Value;
                  begin
                     Integers.Get (File, Native_Mode);
                     Integers.Get (File, Index);
                     Integers.Get (File, Forward_Status);
                     Floats.Get (File, X);
                     Floats.Get (File, Y);
                     Integers.Get (File, Backward_Status);
                     Floats.Get (File, Recovered_X);
                     Floats.Get (File, Recovered_Y);
                     Floats.Get (File, Distance);
                     Floats.Get (File, Error);
                     Assert (Native_Mode = Mode and then Index = I, "oracle order differs");
                     Assert (Tracks (I).Forward.Tracked = (Forward_Status = 1) and then
                               Tracks (I).Backward_Tracked = (Backward_Status = 1), "oracle statuses differ");
                     Assert (Close (Tracks (I).Forward.Next_Point.X, X) and then
                               Close (Tracks (I).Forward.Next_Point.Y, Y) and then
                               Close (Tracks (I).Recovered_Previous_Point.X, Recovered_X) and then
                               Close (Tracks (I).Recovered_Previous_Point.Y, Recovered_Y) and then
                               Close (Tracks (I).Round_Trip_Error, Distance) and then
                               Close (Tracks (I).Forward.Error, Error), "direct C++/Ada diagnostics differ");
                     Compared := Compared + 1;
                  end;
               end loop;
            end;
         end;
      end loop;
      Assert (Compared = 20, "oracle comparison count differs");
      Ada.Text_IO.Close (File);
   exception
      when others =>
         if Ada.Text_IO.Is_Open (File) then
            Ada.Text_IO.Close (File);
         end if;
         raise;
   end Pyramid_FB_Direct_Oracle;


   procedure Pyramid_FB_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      Points : constant Tracking_Point_Array := FB_Points;
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      for Mode in 0 .. 3 loop
         declare
            B : OpenCV.Core.Mat := Shift (A, (if Mode = 0 then 2 else 12), (if Mode = 0 then 1 else 7));
            Options : constant Forward_Backward_Options :=
              (Tracking => (Max_Level => (if Mode = 0 then 1 else 0), others => <>), others => <>);
         begin
            if Mode = 3 then B.Set_To ((others => 0.0)); end if;
            for I in Points'Range loop Seeds (I + 15) := (Points (I).X + 12.25, Points (I).Y + 6.75); end loop;
            declare
               PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
               PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
               Raw : constant Forward_Backward_Track_Array :=
                 (if Mode = 1 or else Mode = 3 then Track_PyrLK_Forward_Backward (A, B, Points, Options, Seeds)
                  else Track_PyrLK_Forward_Backward (A, B, Points, Options));
               Built : constant Forward_Backward_Track_Array :=
                 (if Mode = 1 or else Mode = 3 then Track_PyrLK_Forward_Backward (PA, PB, Points, Options, Seeds)
                  else Track_PyrLK_Forward_Backward (PA, PB, Points, Options));
            begin
               Assert (Raw'First = Built'First and then Raw'Last = Built'Last, "equivalence bounds");
               for I in Points'Range loop
                  Assert (Raw (I).Forward.Tracked = Built (I).Forward.Tracked and then
                    Raw (I).Backward_Tracked = Built (I).Backward_Tracked and then
                    Raw (I).Consistent = Built (I).Consistent, "equivalence status");
                  Assert (Near (Raw (I).Forward.Next_Point.X, Built (I).Forward.Next_Point.X, 1.0E-5) and then
                    Near (Raw (I).Forward.Next_Point.Y, Built (I).Forward.Next_Point.Y, 1.0E-5) and then
                    Near (Raw (I).Forward.Error, Built (I).Forward.Error, 1.0E-5) and then
                    Near (Raw (I).Recovered_Previous_Point.X, Built (I).Recovered_Previous_Point.X, 1.0E-5) and then
                    Near (Raw (I).Recovered_Previous_Point.Y, Built (I).Recovered_Previous_Point.Y, 1.0E-5) and then
                    Near (Raw (I).Round_Trip_Error, Built (I).Round_Trip_Error, 1.0E-5), "raw/prebuilt numerics");
               end loop;
            end;
         end;
      end loop;
   end Pyramid_FB_Equivalence;

   procedure Pyramid_FB_Lifetime_Reuse (T : in out Fixture) is
      pragma Unreferenced (T);
      function Captured (DX, DY : Integer; Region : Boolean) return PyrLK_Pyramid is
         Parent : OpenCV.Core.Mat := Texture (128, 128);
         Shifted : OpenCV.Core.Mat := Shift (Parent, DX, DY);
         Image : OpenCV.Core.Mat := (if Region then Shifted.Region ((16, 16, 96, 96)) else Shifted.Clone);
      begin
         Assert (not Region or else not Image.Is_Continuous, "lifetime Region contiguous");
         return P : PyrLK_Pyramid := Build_PyrLK_Pyramid (Image) do
            for R in 0 .. 95 loop
               for C in 0 .. 95 loop OpenCV.Core.UInt8_Access.Set (Image, R, C, 0); end loop;
            end loop;
            Parent.Set_To ((others => 0.0));
            Shifted.Set_To ((others => 0.0));
         end return;
      end Captured;
   begin
      for Region in Boolean loop
         declare
            PA : constant PyrLK_Pyramid := Captured (0, 0, Region);
            PB : constant PyrLK_Pyramid := Captured (2, 1, Region);
         begin
            for Repetition in 1 .. 3 loop
               declare
                  Points : constant Tracking_Point_Array :=
                    (if Repetition = 2 then Tracking_Point_Array'(7 => (45.0, 32.0)) else FB_Points);
                  Seeds : Tracking_Point_Array (20 .. 19 + Points'Length);
               begin
                  for I in Points'Range loop
                     Seeds (I - Points'First + 20) :=
                       (Points (I).X + (if Repetition = 2 then 1.75 else 2.25), Points (I).Y + 0.75);
                  end loop;
                  declare
                     Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                       (PA, PB, Points, (Tracking => (Max_Level => 1, others => <>), others => <>), Seeds);
                     Moved : Tracking_Point_Array (Points'Range);
                  begin
                     for I in Points'Range loop
                        Assert_Good (Tracks (I), Points (I));
                        Moved (I) := Tracks (I).Forward.Next_Point;
                     end loop;
                     declare
                        Back : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
                          (PB, PA, Moved, (Tracking => (Max_Level => 1, others => <>), others => <>), Points);
                     begin
                        for I in Back'Range loop Assert_Good (Back (I), Moved (I)); end loop;
                     end;
                  end;
               end;
            end loop;
         end;
      end loop;
   end Pyramid_FB_Lifetime_Reuse;

   procedure Pyramid_FB_Compatibility (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      B : constant OpenCV.Core.Mat := Texture (95, 96);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Shallow : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A, (Max_Level => 0, others => <>));
      Null_Pyramid : PyrLK_Pyramid;
   begin
      for Seeded in Boolean loop
         for Empty in Boolean loop
            for Mode in 0 .. 8 loop
               begin
                  declare
                     Points : constant Tracking_Point_Array :=
                       (if Empty then Tracking_Point_Array'(7 .. 6 => <>)
                        elsif Mode = 7 then [7 => (536_871_040.0, 0.0)] else [7 => (25.0, 25.0)]);
                     Options : Forward_Backward_Options;
                  begin
                     if Mode = 3 then Options.Tracking.Window_Size := (15, 15); end if;
                     if Mode = 4 then Options.Tracking.Epsilon := 0.0; end if;
                     if Mode = 5 then Options.Maximum_Round_Trip_Error := -1.0; end if;
                     if Mode = 6 then Options.Tracking.Max_Level := 31; end if;
                     declare
                        function Run (P, Q : PyrLK_Pyramid) return Forward_Backward_Track_Array is
                        begin
                           if Seeded then
                              return Track_PyrLK_Forward_Backward (P, Q, Points, Options,
                                (if Mode = 8 then Tracking_Point_Array'(20 .. 19 => <>) else Points));
                           else return Track_PyrLK_Forward_Backward (P, Q, Points, Options); end if;
                        end Run;
                        Tracks : constant Forward_Backward_Track_Array :=
                          (if Mode = 0 then Run (Null_Pyramid, PA)
                           elsif Mode = 1 then Run (PA, PB)
                           elsif Mode = 2 then Run (PA, Shallow) else Run (PA, PA));
                        pragma Unreferenced (Tracks);
                     begin
                        Assert ((Empty and then Mode = 7) or else
                          (Mode = 8 and then (Empty or else not Seeded)), "pyramid FB validation bypassed");
                     end;
                  end;
               exception
                  when OpenCV.OpenCV_Error => null;
               end;
            end loop;
         end loop;
      end loop;
   end Pyramid_FB_Compatibility;

   procedure Pyramid_FB_Truncation_Subsets (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A, (Max_Level => 30, others => <>));
      Points : constant Tracking_Point_Array := [7 => (-1000.0, -1000.0), 8 => (25.0, 25.0), 9 => (1000.0, 1000.0)];
      Tracks : constant Forward_Backward_Track_Array := Track_PyrLK_Forward_Backward
        (PA, PA, Points, (Tracking => (Max_Level => 30, others => <>), others => <>));
   begin
      Assert (Available_Max_Level (PA) < 30, "natural depth truncation not exercised");
      Assert_Good (Tracks (8), Points (8));
      Assert (not Tracks (7).Forward.Tracked and then not Tracks (9).Forward.Tracked, "single survivor fixture");
      Assert_Unavailable (Tracks (7), Points (7)); Assert_Unavailable (Tracks (9), Points (9));
   end Pyramid_FB_Truncation_Subsets;

   --  Task 009 dense Farneback tests.
   function Central_Mean (Flow : OpenCV.Core.Mat; Channel : Natural) return OpenCV.Float64_Value is
      Rows : constant Natural := Flow.Rows;
      Columns : constant Natural := Flow.Columns;
      Sum : OpenCV.Float64_Value := 0.0;
      Count : Natural := 0;
   begin
      for R in Rows / 4 .. 3 * Rows / 4 - 1 loop
         for C in Columns / 4 .. 3 * Columns / 4 - 1 loop
            Sum := Sum + OpenCV.Float64_Value
              (OpenCV.Core.Float32_Vec2_Access.Get (Flow, R, C) (Channel));
            Count := Count + 1;
         end loop;
      end loop;
      return Sum / OpenCV.Float64_Value (Count);
   end Central_Mean;

   function Same_Flow (Left, Right : OpenCV.Core.Mat) return Boolean is
      use type OpenCV.Core.Float32_Vec2.Vector;
   begin
      if Left.Rows /= Right.Rows or else Left.Columns /= Right.Columns then
         return False;
      end if;
      for R in 0 .. Left.Rows - 1 loop
         for C in 0 .. Left.Columns - 1 loop
            if OpenCV.Core.Float32_Vec2_Access.Get (Left, R, C)
              /= OpenCV.Core.Float32_Vec2_Access.Get (Right, R, C)
            then
               return False;
            end if;
         end loop;
      end loop;
      return True;
   end Same_Flow;

   function Close_Mean (Actual, Expected : OpenCV.Float64_Value;
                        Tolerance : OpenCV.Float64_Value := 0.15) return Boolean is
     (abs (Actual - Expected) <= Tolerance);

   use type OpenCV.Core.Depth_Type;
   use type OpenCV.Core.Channel_Count;

   procedure Farneback_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Image, Image);
   begin
      Assert (Close_Mean (Central_Mean (Flow, 0), 0.0, 0.01)
              and then Close_Mean (Central_Mean (Flow, 1), 0.0, 0.01),
              "Farneback identity flow is not near zero");
   end Farneback_Identity;

   procedure Farneback_Translation_Directions (T : in out Fixture) is
      pragma Unreferenced (T);
      Base : constant OpenCV.Core.Mat := Texture (128, 128);
      Shifted : constant OpenCV.Core.Mat := Shift (Base, 2, 1);
      Positive_Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Base, Shifted);
      Negative_Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Shifted, Base);
   begin
      Assert (Close_Mean (Central_Mean (Positive_Flow, 0), 2.0)
              and then Close_Mean (Central_Mean (Positive_Flow, 1), 1.0),
              "positive translation direction/magnitude differs");
      Assert (Close_Mean (Central_Mean (Negative_Flow, 0), -2.0)
              and then Close_Mean (Central_Mean (Negative_Flow, 1), -1.0),
              "negative translation direction/magnitude differs");
   end Farneback_Translation_Directions;

   procedure Farneback_Schema_Ownership (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : OpenCV.Core.Mat := Texture (64, 80);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 1, 1);
      Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Previous, Next);
      Before : constant OpenCV.Core.Mat := Flow.Clone;
   begin
      Assert (Flow.Rows = 64 and then Flow.Columns = 80 and then Flow.Dimension_Count = 2
              and then Flow.Depth = OpenCV.Core.Float32 and then Flow.Channels = 2,
              "Farneback output schema differs");
      Previous.Set_To ((others => 9.0));
      Assert (Same_Flow (Flow, Before), "returned flow aliases caller input");
      Assert (Same_Flow (Flow, Calculate_Farneback_Flow (Texture (64, 80), Next)),
              "Farneback is not repeatable");
   end Farneback_Schema_Ownership;

   procedure Farneback_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous_Parent : constant OpenCV.Core.Mat := Texture (96, 96);
      Next_Parent : constant OpenCV.Core.Mat := Shift (Previous_Parent, 2, 1);
      Previous : constant OpenCV.Core.Mat := Previous_Parent.Region ((8, 8, 64, 64));
      Next : constant OpenCV.Core.Mat := Next_Parent.Region ((8, 8, 64, 64));
      Compact : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Previous.Clone, Next.Clone);
      Strided : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Previous, Next);
   begin
      Assert (not Previous.Is_Continuous, "fixture Region should be noncontiguous");
      Assert (Strided.Rows = 64 and then Strided.Columns = 64, "Region flow geometry");
      Assert (Same_Flow (Compact, Strided), "strided Region flow differs from compact copy");
   end Farneback_Regions;

   procedure Farneback_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Good : constant OpenCV.Core.Mat := Texture (32, 32);
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (32, 32, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (32, 32, (Depth => OpenCV.Core.UInt8, Channels => 3));
      Empty : OpenCV.Core.Mat;
      Rejected : Natural := 0;

      procedure Reject (Previous, Next : OpenCV.Core.Mat;
                        Options : Farneback_Options := (others => <>)) is
      begin
         declare
            Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Previous, Next, Options);
            pragma Unreferenced (Flow);
         begin
            Assert (False, "invalid Farneback request accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => Rejected := Rejected + 1;
      end Reject;

      function With_Scale (V : OpenCV.Float64_Value) return Farneback_Options is
        (Pyramid_Scale => V, others => <>);
      function From_Bits is new Ada.Unchecked_Conversion
        (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      type Bit_Array is array (Positive range <>) of Interfaces.Unsigned_64;
   begin
      Reject (Empty, Good);
      Reject (Good, Empty);
      Reject (Float_Image, Float_Image);
      Reject (Color, Color);
      Reject (Good, Texture (32, 33));
      Reject (Texture (15, 40), Texture (15, 40));
      Reject (Texture (40, 15), Texture (40, 15));
      Reject (Good, Good, With_Scale (0.24));
      Reject (Good, Good, With_Scale (0.91));
      Reject (Good, Good, (Levels => 9, others => <>));
      Reject (Good, Good, (Window_Size => 4, others => <>));
      Reject (Good, Good, (Window_Size => 6, others => <>));
      Reject (Good, Good, (Window_Size => 65, others => <>));
      Reject (Good, Good, (Iterations => 31, others => <>));
      Reject (Good, Good, (Poly_Neighborhood => 6, others => <>));
      Reject (Good, Good, (Poly_Sigma => 0.05, others => <>));
      Reject (Good, Good, (Poly_Sigma => 10.5, others => <>));
      --  GNAT validity checks may reject IEEE special values before they can be
      --  stored in a public Ada record (Constraint_Error); either outcome is a
      --  rejection. Native-boundary tests cover non-finite values at the C level.
      for Bits of Bit_Array'(1 => 16#7FF0_0000_0000_0000#, 2 => 16#7FF8_0000_0000_0000#,
                             3 => 16#FFF0_0000_0000_0000#)
      loop
         begin
            Reject (Good, Good, With_Scale (From_Bits (Bits)));
            Reject (Good, Good, (Poly_Sigma => From_Bits (Bits), others => <>));
         exception
            when Constraint_Error => Rejected := Rejected + 2;
         end;
      end loop;
      Assert (Rejected = 17 + 6, "not every invalid Farneback request was rejected");
      declare
         Boundary : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
           (Good, Good, (Pyramid_Scale => 0.25, Levels => 8, Window_Size => 63,
                         Iterations => 30, Poly_Neighborhood => 7, Poly_Sigma => 10.0));
         Minimum : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
           (Good, Good, (Pyramid_Scale => 0.9, Levels => 1, Window_Size => 5,
                         Iterations => 1, Poly_Neighborhood => 5, Poly_Sigma => 0.1));
      begin
         Assert (Boundary.Rows = 32 and then Minimum.Rows = 32, "option boundary values rejected");
      end;
   end Farneback_Validation;

   procedure Farneback_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Largest : OpenCV.Float64_Value := 0.0;
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value ("VIDEO_FARNEBACK_ORACLE", "../obj/oracle/farneback.txt"));
      for Mode in 0 .. 2 loop
         declare
            Native_Mode, Rows, Columns : Integer;
            Next : constant OpenCV.Core.Mat :=
              (if Mode = 0 then Previous.Clone
               elsif Mode = 1 then Shift (Previous, 2, 1) else Shift (Previous, 3, 2));
            Options : constant Farneback_Options :=
              (if Mode = 2 then (Pyramid_Scale => 0.6, Levels => 2, Window_Size => 11,
                                 Iterations => 2, Poly_Neighborhood => 7, Poly_Sigma => 1.5)
               else (others => <>));
            Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Previous, Next, Options);
         begin
            Integers.Get (File, Native_Mode);
            Integers.Get (File, Rows);
            Integers.Get (File, Columns);
            Assert (Native_Mode = Mode and then Rows = Flow.Rows and then Columns = Flow.Columns,
                    "Farneback oracle header differs");
            for R in 0 .. Rows - 1 loop
               for C in 0 .. Columns - 1 loop
                  declare
                     X, Y : OpenCV.Float64_Value;
                     Actual : constant OpenCV.Core.Float32_Vec2.Vector :=
                       OpenCV.Core.Float32_Vec2_Access.Get (Flow, R, C);
                  begin
                     Floats.Get (File, X);
                     Floats.Get (File, Y);
                     Largest := OpenCV.Float64_Value'Max
                       (Largest, OpenCV.Float64_Value'Max
                          (abs (OpenCV.Float64_Value (Actual (0)) - X),
                           abs (OpenCV.Float64_Value (Actual (1)) - Y)));
                  end;
               end loop;
            end loop;
         end;
      end loop;
      Ada.Text_IO.Close (File);
      Assert (Largest <= 1.0E-5, "binding differs from independent native Farneback call");
   end Farneback_Direct_Oracle;

   procedure Farneback_Minimum_Size_Options (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (16, 16);
      Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Image, Shift (Image, 1, 0), (Levels => 8, Window_Size => 63, others => <>));
   begin
      Assert (Flow.Rows = 16 and then Flow.Columns = 16 and then Flow.Channels = 2,
              "minimum-size Farneback geometry");
   end Farneback_Minimum_Size_Options;

   --  Task 010 seeded dense Farneback tests.
   function Seed_Field (Size : Positive; Kind : Natural; DX : OpenCV.Float32_Value := 0.0;
                        DY : OpenCV.Float32_Value := 0.0) return OpenCV.Core.Mat is
   begin
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (Size, Size, (Depth => OpenCV.Core.Float32, Channels => 2)) do
         for R in 0 .. Size - 1 loop
            for C in 0 .. Size - 1 loop
               OpenCV.Core.Float32_Vec2_Access.Set
                 (Result, R, C,
                  (if Kind = 3
                   then [0 => OpenCV.Float32_Value (3.0 + OpenCV.Float64_Value (R - 48) * 0.015625),
                         1 => OpenCV.Float32_Value (2.0 - OpenCV.Float64_Value (C - 48) * 0.0078125)]
                   else [0 => DX, 1 => DY]));
            end loop;
         end loop;
      end return;
   end Seed_Field;

   function Max_Difference (Left, Right : OpenCV.Core.Mat) return OpenCV.Float64_Value is
      Result : OpenCV.Float64_Value := 0.0;
   begin
      for R in 0 .. Left.Rows - 1 loop
         for C in 0 .. Left.Columns - 1 loop
            declare
               A : constant OpenCV.Core.Float32_Vec2.Vector :=
                 OpenCV.Core.Float32_Vec2_Access.Get (Left, R, C);
               B : constant OpenCV.Core.Float32_Vec2.Vector :=
                 OpenCV.Core.Float32_Vec2_Access.Get (Right, R, C);
            begin
               for K in 0 .. 1 loop
                  Result := OpenCV.Float64_Value'Max
                    (Result, abs (OpenCV.Float64_Value (A (K)) - OpenCV.Float64_Value (B (K))));
               end loop;
            end;
         end loop;
      end loop;
      return Result;
   end Max_Difference;

   procedure Seeded_Default_And_Options (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (192, 192);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Prediction : constant OpenCV.Core.Mat := Seed_Field (192, 0, 12.25, 6.75);
      Custom : constant Farneback_Options :=
        (Pyramid_Scale => 0.6, Levels => 2, Window_Size => 11, Iterations => 2,
         Poly_Neighborhood => 7, Poly_Sigma => 1.5);
      Refined : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous_Image => Previous, Next_Image => Next, Initial_Flow => Prediction);
      Custom_Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous_Image => Previous, Next_Image => Next, Options => Custom,
         Initial_Flow => Prediction);
      Positional : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous, Next, (others => <>), Prediction);
   begin
      Assert (Refined.Rows = 192 and then Refined.Columns = 192 and then Refined.Dimension_Count = 2
              and then Refined.Depth = OpenCV.Core.Float32 and then Refined.Channels = 2,
              "seeded Farneback output schema");
      Assert (Close_Mean (Central_Mean (Refined, 0), 12.0, 0.25)
              and then Close_Mean (Central_Mean (Refined, 1), 7.0, 0.25),
              "seeded default-option flow does not recover translation (12,7)");
      Assert (Custom_Flow.Rows = 192 and then Custom_Flow.Channels = 2
              and then Close_Mean (Central_Mean (Custom_Flow, 0), 12.0, 0.5)
              and then Close_Mean (Central_Mean (Custom_Flow, 1), 7.0, 0.5),
              "seeded nondefault-option flow differs from translation");
      Assert (Same_Flow (Refined, Positional), "positional seeded call differs from named call");
      Assert (Max_Difference (Refined, Custom_Flow) > 0.0, "options had no effect on seeded flow");
   end Seeded_Default_And_Options;

   procedure Seeded_Consumption_Refinement (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (192, 192);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Near : constant OpenCV.Core.Mat := Seed_Field (192, 0, 12.25, 6.75);
      Near_Before : constant OpenCV.Core.Mat := Near.Clone;
      Other : constant OpenCV.Core.Mat := Seed_Field (192, 0, -20.0, 15.0);
      Plain : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Previous, Next);
      Seeded : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous, Next, Initial_Flow => Near);
      Different : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous, Next, Initial_Flow => Other);
      Again : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous, Next, Initial_Flow => Near);
   begin
      --  Established experimentally on 4.1/4.6/4.10/5.0: the unseeded call converges to
      --  about (6.2,6.1) on this fixture while the near-correct seed reaches (12,7).
      Assert (abs (Central_Mean (Plain, 0) - 12.0) > 2.0, "fixture no longer defeats unseeded flow");
      Assert (Close_Mean (Central_Mean (Seeded, 0), 12.0, 0.25)
              and then Close_Mean (Central_Mean (Seeded, 1), 7.0, 0.25),
              "seeded flow not close to translation");
      Assert (Max_Difference (Seeded, Plain) > 2.0, "seeded and unseeded flow are not distinguishable");
      Assert (Max_Difference (Seeded, Different) > 2.0
              and then abs (Central_Mean (Different, 0) - Central_Mean (Seeded, 0)) > 2.0,
              "different seed did not change the result");
      Assert (Max_Difference (Seeded, Near) > 0.001, "seeded flow is an unrefined copy of the seed");
      Assert (Max_Difference (Near, Near_Before) = 0.0, "caller seed was mutated");
      Assert (Max_Difference (Other, Seed_Field (192, 0, -20.0, 15.0)) = 0.0, "second seed was mutated");
      Assert (Same_Flow (Seeded, Again), "repeated seeded call is not deterministic");
   end Seeded_Consumption_Refinement;

   procedure Seeded_Zero_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      Texture_Image : constant OpenCV.Core.Mat := Texture (128, 128);
      Flat_Parent : OpenCV.Core.Mat := OpenCV.Core.Create
        (64, 64, (Depth => OpenCV.Core.UInt8, Channels => 1));
      Largest : OpenCV.Float64_Value := 0.0;
   begin
      Flat_Parent.Set_To ((others => 127.0));
      for Case_Index in 0 .. 3 loop
         declare
            Previous : constant OpenCV.Core.Mat :=
              (if Case_Index = 3 then Flat_Parent else Texture_Image);
            Next : constant OpenCV.Core.Mat :=
              (case Case_Index is
                  when 0 => Texture_Image.Clone,
                  when 1 => Shift (Texture_Image, 2, 1),
                  when 2 => Shift (Texture_Image, 12, 7),
                  when others => Flat_Parent.Clone);
            Zero : constant OpenCV.Core.Mat := Seed_Field (Previous.Rows, 0);
            Plain : constant OpenCV.Core.Mat := Calculate_Farneback_Flow (Previous, Next);
            Seeded : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
              (Previous, Next, Initial_Flow => Zero);
         begin
            Largest := OpenCV.Float64_Value'Max (Largest, Max_Difference (Plain, Seeded));
         end;
      end loop;
      --  Measured identical on the supported natives; tolerance leaves room for
      --  initialization-path rounding without asserting bitwise identity.
      Assert (Largest <= 1.0E-3, "zero-seeded flow differs from unseeded flow");
   end Seeded_Zero_Equivalence;

   procedure Seeded_Nonuniform_Independence (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 3, 2);
      Field : OpenCV.Core.Mat := Seed_Field (96, 3);
      Before : constant OpenCV.Core.Mat := Field.Clone;
      Result : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous, Next, Initial_Flow => Field);
      Saved : constant OpenCV.Core.Mat := Result.Clone;
   begin
      Assert (Max_Difference (Field, Before) = 0.0, "nonuniform seed was mutated");
      Assert (Max_Difference (Result, Field) > 0.01, "result is a shallow copy of the seed");
      Assert (Close_Mean (Central_Mean (Result, 0), 3.0, 0.5)
              and then Close_Mean (Central_Mean (Result, 1), 2.0, 0.5),
              "nonuniform seeded flow not near the translation");
      Field.Set_To ((others => 55.0));
      Assert (Same_Flow (Result, Saved), "result aliases the initial-flow allocation");
   end Seeded_Nonuniform_Independence;

   procedure Seeded_Signs_And_Identical (T : in out Fixture) is
      pragma Unreferenced (T);
      Base : constant OpenCV.Core.Mat := Texture (128, 128);
      Shifted : constant OpenCV.Core.Mat := Shift (Base, 2, 1);
      Positive_Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Base, Shifted, Initial_Flow => Seed_Field (128, 0, 2.0, 1.0));
      Negative_Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Shifted, Base, Initial_Flow => Seed_Field (128, 0, -2.0, -1.0));
      Identical : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Base, Base, Initial_Flow => Seed_Field (128, 0, 0.5, -0.5));
      Swapped : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Base, Shifted, Initial_Flow => Seed_Field (128, 0, 1.0, 2.0));
   begin
      Assert (Close_Mean (Central_Mean (Positive_Flow, 0), 2.0)
              and then Close_Mean (Central_Mean (Positive_Flow, 1), 1.0),
              "positive seeded motion differs");
      Assert (Close_Mean (Central_Mean (Negative_Flow, 0), -2.0)
              and then Close_Mean (Central_Mean (Negative_Flow, 1), -1.0),
              "negative seeded motion differs");
      Assert (Close_Mean (Central_Mean (Identical, 0), 0.0, 0.3)
              and then Close_Mean (Central_Mean (Identical, 1), 0.0, 0.3),
              "identical-image seeded flow is not near zero");
      Assert (Swapped.Rows = 128 and then Swapped.Channels = 2
              and then Close_Mean (Central_Mean (Swapped, 0), 2.0, 0.3)
              and then Close_Mean (Central_Mean (Swapped, 1), 1.0, 0.3),
              "channel-ordered refinement of an imperfect seed differs");
   end Seeded_Signs_And_Identical;

   procedure Seeded_Regions_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous_Clone : constant OpenCV.Core.Mat := Texture (64, 64);
      Next_Clone : constant OpenCV.Core.Mat := Shift (Previous_Clone, 2, 1);
      Seed_Clone : constant OpenCV.Core.Mat := Seed_Field (64, 3);
      Expected : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
        (Previous_Clone, Next_Clone, Initial_Flow => Seed_Clone);
   begin
      for Combination in 1 .. 7 loop
         declare
            Use_Previous : constant Boolean := Combination mod 2 = 1;
            Use_Next : constant Boolean := (Combination / 2) mod 2 = 1;
            Use_Seed : constant Boolean := Combination >= 4;
            Result : OpenCV.Core.Mat;
         begin
            declare
               Previous_Parent : OpenCV.Core.Mat := Texture (96, 96);
               Next_Parent : OpenCV.Core.Mat := Shift (Previous_Parent, 2, 1);
               Seed_Parent : OpenCV.Core.Mat := Seed_Field (96, 0, 9.0, 9.0);
            begin
               --  Place identical content inside Regions of larger parents.
               for R in 0 .. 63 loop
                  for C in 0 .. 63 loop
                     OpenCV.Core.UInt8_Access.Set (Previous_Parent, R + 8, C + 8,
                       OpenCV.Core.UInt8_Access.Get (Previous_Clone, R, C));
                     OpenCV.Core.UInt8_Access.Set (Next_Parent, R + 8, C + 8,
                       OpenCV.Core.UInt8_Access.Get (Next_Clone, R, C));
                     OpenCV.Core.Float32_Vec2_Access.Set (Seed_Parent, R + 8, C + 8,
                       OpenCV.Core.Float32_Vec2_Access.Get (Seed_Clone, R, C));
                  end loop;
               end loop;
               declare
                  Previous_Region : constant OpenCV.Core.Mat := Previous_Parent.Region ((8, 8, 64, 64));
                  Next_Region : constant OpenCV.Core.Mat := Next_Parent.Region ((8, 8, 64, 64));
                  Seed_Region : constant OpenCV.Core.Mat := Seed_Parent.Region ((8, 8, 64, 64));
                  Seed_Saved : constant OpenCV.Core.Mat := Seed_Region.Clone;
               begin
                  Assert (not Seed_Region.Is_Continuous, "seed Region should be noncontiguous");
                  Result := Calculate_Farneback_Flow
                    ((if Use_Previous then Previous_Region else Previous_Clone),
                     (if Use_Next then Next_Region else Next_Clone),
                     Initial_Flow => (if Use_Seed then Seed_Region else Seed_Clone));
                  Assert (Max_Difference (Seed_Region, Seed_Saved) = 0.0, "Region seed was mutated");
                  Assert (Max_Difference (Seed_Parent.Region ((0, 0, 8, 8)),
                                          Seed_Field (8, 0, 9.0, 9.0)) = 0.0,
                          "seed parent outside the Region changed");
               end;
               --  Mutate sources before the parents are finalized.
               Previous_Parent.Set_To ((others => 3.0));
               Next_Parent.Set_To ((others => 4.0));
               Seed_Parent.Set_To ((others => 5.0));
            end;
            --  Parents and Regions are finalized; the result must still be valid and identical.
            Assert (Result.Rows = 64 and then Result.Columns = 64 and then Same_Flow (Result, Expected),
                    "Region-sourced seeded flow differs from compact inputs");
         end;
      end loop;
   end Seeded_Regions_Lifetime;

   procedure Seeded_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Good : constant OpenCV.Core.Mat := Texture (32, 32);
      Seed : constant OpenCV.Core.Mat := Seed_Field (32, 0, 1.0, 1.0);
      Seed_Before : constant OpenCV.Core.Mat := Seed.Clone;
      Empty : OpenCV.Core.Mat;
      Rejected : Natural := 0;
      Limit : constant OpenCV.Float32_Value := 1048576.0;
      function From_Bits is new Ada.Unchecked_Conversion
        (Interfaces.Unsigned_32, OpenCV.Float32_Value);

      procedure Reject (Previous, Next, Initial : OpenCV.Core.Mat;
                        Options : Farneback_Options := (others => <>)) is
      begin
         declare
            Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
              (Previous, Next, Options, Initial);
            pragma Unreferenced (Flow);
         begin
            Assert (False, "invalid seeded Farneback request accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => Rejected := Rejected + 1;
      end Reject;

      procedure Reject_Value (Value : OpenCV.Float32_Value; Channel : Natural) is
         Bad : OpenCV.Core.Mat := Seed.Clone;
         Components : OpenCV.Core.Float32_Vec2.Vector := OpenCV.Core.Float32_Vec2_Access.Get (Bad, 5, 7);
      begin
         Components (Channel) := Value;
         OpenCV.Core.Float32_Vec2_Access.Set (Bad, 5, 7, Components);
         Reject (Good, Good, Bad);
      end Reject_Value;
   begin
      Reject (Good, Good, Empty);
      Reject (Empty, Good, Seed);
      Reject (Good, Empty, Seed);
      Reject (Good, Good, Seed_Field (31, 0));
      Reject (Good, Good, OpenCV.Core.Create (32, 33, (Depth => OpenCV.Core.Float32, Channels => 2)));
      Reject (Good, Good, OpenCV.Core.Create (32, 32, (Depth => OpenCV.Core.Float32, Channels => 1)));
      Reject (Good, Good, OpenCV.Core.Create (32, 32, (Depth => OpenCV.Core.Float32, Channels => 3)));
      Reject (Good, Good, OpenCV.Core.Create (32, 32, (Depth => OpenCV.Core.Float64, Channels => 2)));
      Reject (Good, Good, OpenCV.Core.Create (32, 32, (Depth => OpenCV.Core.UInt8, Channels => 2)));
      Reject (Texture (15, 15), Texture (15, 15), Seed_Field (15, 0));
      Reject (Good, Texture (32, 33), Seed);
      Reject (Good, Good, Seed, (Levels => 9, others => <>));
      Reject (Good, Good, Seed, (Window_Size => 6, others => <>));
      Reject (Good, Good, Seed, (Poly_Sigma => 10.5, others => <>));
      Reject (Good, Good, Seed, (Pyramid_Scale => 0.95, others => <>));
      Assert (Rejected = 15, "not every structural seeded request was rejected");
      for Channel in 0 .. 1 loop
         Reject_Value (Limit * 2.0, Channel);
         Reject_Value (-Limit * 2.0, Channel);
         Reject_Value (OpenCV.Float32_Value'Last, Channel);
         Reject_Value (OpenCV.Float32_Value'First, Channel);
         Reject_Value (Limit + 1.0, Channel);
         begin
            Reject_Value (From_Bits (16#7FC0_0000#), Channel);
         exception
            when Constraint_Error => Rejected := Rejected + 1;
         end;
         begin
            Reject_Value (From_Bits (16#7F80_0000#), Channel);
         exception
            when Constraint_Error => Rejected := Rejected + 1;
         end;
         begin
            Reject_Value (From_Bits (16#FF80_0000#), Channel);
         exception
            when Constraint_Error => Rejected := Rejected + 1;
         end;
      end loop;
      Assert (Rejected = 15 + 2 * 8, "not every unsafe seed component was rejected");
      Assert (Max_Difference (Seed, Seed_Before) = 0.0, "rejected call modified the seed");
      declare
         Boundary_Positive : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
           (Good, Good, Initial_Flow => Seed_Field (32, 0, Limit, -Limit));
         Boundary_Negative : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
           (Good, Good, Initial_Flow => Seed_Field (32, 0, -Limit, Limit));
      begin
         Assert (Boundary_Positive.Rows = 32 and then Boundary_Negative.Rows = 32,
                 "seed exactly at the 2**20 bound was rejected");
      end;
   end Seeded_Validation;

   procedure Seeded_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Largest : OpenCV.Float64_Value := 0.0;
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value
          ("VIDEO_FARNEBACK_SEEDED_ORACLE", "../obj/oracle/farneback-seeded.txt"));
      for Mode in 0 .. 3 loop
         declare
            Size : constant Positive := (if Mode < 2 then 192 else 96);
            Native_Mode, Rows, Columns : Integer;
            Previous : constant OpenCV.Core.Mat := Texture (Size, Size);
            Next : constant OpenCV.Core.Mat :=
              (if Mode < 2 then Shift (Previous, 12, 7)
               elsif Mode = 2 then Shift (Previous, 2, 1) else Shift (Previous, 3, 2));
            Seed : constant OpenCV.Core.Mat :=
              (case Mode is
                  when 0 => Seed_Field (Size, 0, 12.25, 6.75),
                  when 1 => Seed_Field (Size, 0, -20.0, 15.0),
                  when 2 => Seed_Field (Size, 0),
                  when others => Seed_Field (Size, 3));
            Options : constant Farneback_Options :=
              (if Mode = 3 then (Pyramid_Scale => 0.6, Levels => 2, Window_Size => 11,
                                 Iterations => 2, Poly_Neighborhood => 7, Poly_Sigma => 1.5)
               else (others => <>));
            Flow : constant OpenCV.Core.Mat := Calculate_Farneback_Flow
              (Previous, Next, Options, Seed);
            Edge_Seen : Boolean := False;
         begin
            Integers.Get (File, Native_Mode);
            Integers.Get (File, Rows);
            Integers.Get (File, Columns);
            Assert (Native_Mode = Mode and then Rows = Flow.Rows and then Columns = Flow.Columns,
                    "seeded Farneback oracle header differs");
            for R in 0 .. Rows - 1 loop
               for C in 0 .. Columns - 1 loop
                  declare
                     X, Y : OpenCV.Float64_Value;
                     Actual : constant OpenCV.Core.Float32_Vec2.Vector :=
                       OpenCV.Core.Float32_Vec2_Access.Get (Flow, R, C);
                  begin
                     Floats.Get (File, X);
                     Floats.Get (File, Y);
                     Edge_Seen := Edge_Seen or else R = 0 or else C = 0
                       or else R = Rows - 1 or else C = Columns - 1;
                     Largest := OpenCV.Float64_Value'Max
                       (Largest, OpenCV.Float64_Value'Max
                          (abs (OpenCV.Float64_Value (Actual (0)) - X),
                           abs (OpenCV.Float64_Value (Actual (1)) - Y)));
                  end;
               end loop;
            end loop;
            Assert (Edge_Seen, "seeded oracle comparison skipped the edges");
         end;
      end loop;
      Ada.Text_IO.Close (File);
      Assert (Largest <= 1.0E-5, "binding differs from independent seeded native Farneback call");
   end Seeded_Direct_Oracle;

   function Quality_Structure (Kind : Natural) return OpenCV.Core.Mat is
   begin
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (96, 96, (Depth => OpenCV.Core.UInt8, Channels => 1)) do
         for R in 0 .. 95 loop
            for C in 0 .. 95 loop
               OpenCV.Core.UInt8_Access.Set (Result, R, C,
                 (if Kind = 0 then (if R >= 48 and then C >= 48 then 255 else 0)
                  elsif Kind = 1 then (if C >= 48 then 255 else 0) else 127));
            end loop;
         end loop;
      end return;
   end Quality_Structure;

   Quality_Points : constant Tracking_Point_Array (5 .. 8) :=
     [(25.0, 25.0), (45.0, 32.0), (60.0, 50.0), (35.0, 65.0)];
   Quality_Options : constant PyrLK_Options := (Max_Level => 0, others => <>);

   procedure Quality_Identity (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Tracks : constant Trackability_Track_Array :=
        Track_PyrLK_Trackability (Image, Image, Quality_Points, Quality_Options);
   begin
      Assert (Tracks'First = 5 and then Tracks'Last = 8, "quality bounds lost");
      for I in Tracks'Range loop
         Assert (Tracks (I).Tracked and then Tracks (I).Minimum_Eigenvalue > 0.1,
                 "identity quality missing (flag 8 ignored)");
         Assert (Tracks (I).Previous_Point = Quality_Points (I) and then
                   Tracks (I).Next_Point = Quality_Points (I), "identity point moved");
      end loop;
   end Quality_Identity;

   procedure Quality_Seeded (T : in out Fixture) is
      pragma Unreferenced (T);
      Previous : constant OpenCV.Core.Mat := Texture (96, 96);
      Next : constant OpenCV.Core.Mat := Shift (Previous, 12, 7);
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      for I in Quality_Points'Range loop
         Seeds (I - 5 + 20) := (Quality_Points (I).X + 12.25, Quality_Points (I).Y + 6.75);
      end loop;
      declare
         Saved : constant Tracking_Point_Array := Seeds;
         Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability
           (Previous, Next, Quality_Points, Quality_Options, Seeds);
         Plain : constant Point_Track_Array := Track_PyrLK
           (Previous, Next, Quality_Points, Quality_Options);
      begin
         Assert (Tracks'First = 5 and then Tracks'Last = 8 and then Seeds = Saved,
                 "seeded quality bounds or seed mutation");
         for I in Tracks'Range loop
            Assert (Tracks (I).Tracked and then Tracks (I).Minimum_Eigenvalue > 0.1,
                    "seeded quality missing");
            Assert (Near (Tracks (I).Next_Point.X, Quality_Points (I).X + 12.0, 0.05)
                      and then Near (Tracks (I).Next_Point.Y, Quality_Points (I).Y + 7.0, 0.05),
                    "quality seeds not consumed");
            Assert (abs (Tracks (I).Next_Point.X - Seeds (I - 5 + 20).X) > 0.20,
                    "quality returned prediction without refinement");
            Assert (not Plain (I).Tracked or else
                      abs (Plain (I).Next_Point.X - Tracks (I).Next_Point.X) > 5.0,
                    "distinguishing quality fixture lost");
         end loop;
      end;
   end Quality_Seeded;

   procedure Quality_Structures (T : in out Fixture) is
      pragma Unreferenced (T);
      Values : array (0 .. 2) of OpenCV.Float32_Value;
   begin
      for Kind in Values'Range loop
         declare
            Image : constant OpenCV.Core.Mat := Quality_Structure (Kind);
            Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability
              (Image, Image, [1 => (48.0, 48.0)], Quality_Options);
         begin
            Values (Kind) := Tracks (1).Minimum_Eigenvalue;
            Assert (Tracks (1).Tracked = (Kind = 0), "structure status differs");
         end;
      end loop;
      Assert (Values (0) > 0.5 and then Values (1) = 0.0 and then Values (2) = 0.0,
              "portable corner/edge/flat conditioning relationship failed");
   end Quality_Structures;

   procedure Check_Quality_Threshold (Below : Boolean) is
      Image : constant OpenCV.Core.Mat := Quality_Structure (0);
      Points : constant Tracking_Point_Array := [7 => (48.0, 48.0)];
      Baseline : constant Trackability_Track_Array :=
        Track_PyrLK_Trackability (Image, Image, Points, Quality_Options);
      Options : PyrLK_Options := Quality_Options;
   begin
      Options.Min_Eigenvalue_Threshold := OpenCV.Float64_Value (Baseline (7).Minimum_Eigenvalue)
        * (if Below then 0.5 else 2.0);
      declare
         Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability (Image, Image, Points, Options);
      begin
         Assert (Tracks (7).Tracked = Below, "threshold not acting on eigenvalue");
         Assert (Tracks (7).Minimum_Eigenvalue = Baseline (7).Minimum_Eigenvalue,
                 "threshold rejection discarded meaningful quality");
         Assert (Tracks (7).Next_Point = Points (7), "threshold point not deterministic");
      end;
   end Check_Quality_Threshold;

   procedure Quality_Threshold_Below (T : in out Fixture) is
      pragma Unreferenced (T);
   begin
      Check_Quality_Threshold (True);
   end Quality_Threshold_Below;

   procedure Quality_Threshold_Above (T : in out Fixture) is
      pragma Unreferenced (T);
   begin
      Check_Quality_Threshold (False);
   end Quality_Threshold_Above;

   function Previous_Quality_Defined return Boolean is
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Mode, Index, Status : Integer;
      Value : OpenCV.Float64_Value;
      Defined : Boolean := True;
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value ("VIDEO_TRACKABILITY_ORACLE", "../obj/oracle/trackability.txt"));
      for Record_Number in 1 .. 27 loop
         Integers.Get (File, Mode); Integers.Get (File, Index); Integers.Get (File, Status);
         for Component in 1 .. 3 loop Floats.Get (File, Value); end loop;
         if Status = -1 then
            Assert (Mode = 3 and then Index = 1, "unexpected undefined native quality path");
            Defined := False;
         end if;
      end loop;
      Ada.Text_IO.Close (File);
      return Defined;
   end Previous_Quality_Defined;

   procedure Quality_Previous_Unavailable (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Points : constant Tracking_Point_Array := [7 => (-1000.0, -1000.0), 8 => (1000.0, 1000.0)];
   begin
      begin
         declare
            Tracks : constant Trackability_Track_Array :=
              Track_PyrLK_Trackability (Image, Image, Points, Quality_Options);
         begin
            Assert (Previous_Quality_Defined, "undefined native quality accepted");
            for I in Tracks'Range loop
               Assert (not Tracks (I).Tracked and then Tracks (I).Minimum_Eigenvalue = 0.0 and then
                         Tracks (I).Next_Point = Points (I), "unavailable previous patch semantics");
            end loop;
         end;
      exception
         when OpenCV.OpenCV_Error =>
            Assert (not Previous_Quality_Defined, "defined previous quality rejected");
      end;
   end Quality_Previous_Unavailable;

   procedure Quality_Next_Unavailable (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Baseline : constant Trackability_Track_Array :=
        Track_PyrLK_Trackability (Image, Image, Quality_Points, Quality_Options);
      Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability
        (Image, Image, Quality_Points, Quality_Options,
         Initial_Next_Points => [20 .. 23 => (-1000.0, -1000.0)]);
   begin
      for I in Tracks'Range loop
         Assert (not Tracks (I).Tracked and then Tracks (I).Next_Point = Quality_Points (I),
                 "unavailable next search point not normalized");
         Assert (Tracks (I).Minimum_Eigenvalue = Baseline (I).Minimum_Eigenvalue and then
                   Tracks (I).Minimum_Eigenvalue > 0.1, "next failure erased previous quality");
      end loop;
   end Quality_Next_Unavailable;

   procedure Quality_Metrics (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Quality : constant Trackability_Track_Array :=
        Track_PyrLK_Trackability (Image, Image, Quality_Points, Quality_Options);
      Ordinary : constant Point_Track_Array := Track_PyrLK (Image, Image, Quality_Points, Quality_Options);
      Next : constant OpenCV.Core.Mat := Shift (Image, 2, 1);
      Options : constant PyrLK_Options := (Max_Level => 1, others => <>);
      Moving : constant Trackability_Track_Array := Track_PyrLK_Trackability (Image, Next, Quality_Points, Options);
      Photo : constant Point_Track_Array := Track_PyrLK (Image, Next, Quality_Points, Options);
   begin
      for I in Quality'Range loop
         Assert (Ordinary (I).Tracked = Quality (I).Tracked and then
                   Ordinary (I).Next_Point = Quality (I).Next_Point, "ordinary identity differs");
         Assert (Ordinary (I).Error = 0.0 and then Quality (I).Minimum_Eigenvalue > 0.1,
                 "photometric L1 error confused with eigenvalue");
         Assert (Moving (I).Tracked = Photo (I).Tracked and then Moving (I).Tracked and then
                   Near (Moving (I).Next_Point.X, Photo (I).Next_Point.X, 1.0E-5) and then
                   Near (Moving (I).Next_Point.Y, Photo (I).Next_Point.Y, 1.0E-5),
                 "quality changed successful ordinary next point");
      end loop;
   end Quality_Metrics;

   procedure Quality_Seed_Independence (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Seeds : Tracking_Point_Array (20 .. 23);
      Baseline : constant Trackability_Track_Array :=
        Track_PyrLK_Trackability (Image, Image, Quality_Points, Quality_Options);
   begin
      for I in Quality_Points'Range loop
         Seeds (I - 5 + 20) := (Quality_Points (I).X + 32.0, Quality_Points (I).Y + 20.0);
      end loop;
      declare
         Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability
           (Image, Image, Quality_Points, Quality_Options, Seeds);
      begin
         for I in Tracks'Range loop
            Assert (Near (Tracks (I).Minimum_Eigenvalue, Baseline (I).Minimum_Eigenvalue, 1.0E-5),
                    "destination seed changed previous-patch quality");
         end loop;
      end;
   end Quality_Seed_Independence;

   procedure Quality_Extreme_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Points : constant Tracking_Point_Array := [Positive'Last => (25.0, 25.0)];
      Plain : constant Trackability_Track_Array := Track_PyrLK_Trackability (Image, Image, Points);
      Seeded : constant Trackability_Track_Array := Track_PyrLK_Trackability
        (Image, Image, Points, Initial_Next_Points => [3 => (25.0, 25.0)]);
   begin
      Assert (Plain'First = Positive'Last and then Plain'Last = Positive'Last and then
                Seeded'First = Positive'Last and then Seeded'Last = Positive'Last, "extreme quality bounds lost");
      Assert (Plain (Positive'Last).Tracked and then Seeded (Positive'Last).Tracked,
              "extreme quality bounds failed");
   end Quality_Extreme_Bounds;

   procedure Quality_Empty (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Empty : constant Tracking_Point_Array (7 .. 6) := [];
      Plain : constant Trackability_Track_Array := Track_PyrLK_Trackability (Image, Image, Empty);
      Seeded : constant Trackability_Track_Array := Track_PyrLK_Trackability
        (Image, Image, Empty, Initial_Next_Points => Tracking_Point_Array'(20 .. 19 => <>));
   begin
      Assert (Plain'First = 7 and then Plain'Last = 6 and then Seeded'First = 7 and then Seeded'Last = 6,
              "empty quality bounds lost");
   end Quality_Empty;

   procedure Quality_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      Empty_Image : OpenCV.Core.Mat;
      Wrong_Depth_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (96, 96, (Depth => OpenCV.Core.Float32, Channels => 1));
      Wrong_Channels_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (96, 96, (Depth => OpenCV.Core.UInt8, Channels => 3));
      Wrong_Geometry : constant OpenCV.Core.Mat := Texture (95, 96);
   begin
      for Seeded in Boolean loop
         for Mode in 0 .. 10 loop
            begin
               declare
                  Previous : constant OpenCV.Core.Mat :=
                    (case Mode is when 0 => Empty_Image, when 1 => Wrong_Depth_Image,
                     when 2 => Wrong_Channels_Image, when 3 => Wrong_Geometry, when others => Image);
                  Points : constant Tracking_Point_Array :=
                    (if Mode = 10 then Tracking_Point_Array'(7 .. 6 => <>)
                     elsif Mode = 8 then [1 => (OpenCV.Float32_Value'Last, 0.0)] else Quality_Points);
                  Options : PyrLK_Options := Quality_Options;
               begin
                  case Mode is
                     when 4 | 10 => Options.Max_Level := 31;
                     when 5 => Options.Window_Size.Width := 2;
                     when 6 => Options.Epsilon := 0.0;
                     when 7 => Options.Min_Eigenvalue_Threshold := -1.0;
                     when others => null;
                  end case;
                  if Seeded or else Mode /= 9 then
                     declare
                        Tracks : constant Trackability_Track_Array :=
                          (if Seeded then Track_PyrLK_Trackability (Previous, Image, Points, Options,
                            Initial_Next_Points => (if Mode = 9 then [20 => (25.0, 25.0)] else Points))
                           else Track_PyrLK_Trackability (Previous, Image, Points, Options));
                        pragma Unreferenced (Tracks);
                     begin
                        Assert (False, "quality bypassed validation");
                     end;
                  end if;
               end;
            exception
               when OpenCV.OpenCV_Error => null;
            end;
         end loop;
      end loop;
      for Bad of Tracking_Point_Array'[(OpenCV.Float32_Value'Last, 0.0), (536_871_040.0, 0.0)] loop
         begin
            declare
               Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability
                 (Image, Image, [1 => (25.0, 25.0)], Initial_Next_Points => [20 => Bad]);
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "unsafe quality seed accepted");
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end Quality_Validation;

   procedure Quality_Regions_Immutability (T : in out Fixture) is
      pragma Unreferenced (T);
      Parent : constant OpenCV.Core.Mat := Texture (128, 128);
      Shifted : constant OpenCV.Core.Mat := Shift (Parent, 2, 1);
      Previous : constant OpenCV.Core.Mat := Parent.Region ((X => 10, Y => 10, Width => 96, Height => 96));
      Next : constant OpenCV.Core.Mat := Shifted.Region ((X => 10, Y => 10, Width => 96, Height => 96));
      Before_Previous : constant OpenCV.Core.Mat := Previous.Clone;
      Before_Next : constant OpenCV.Core.Mat := Next.Clone;
      Points : constant Tracking_Point_Array := Quality_Points;
      Seeds : Tracking_Point_Array (20 .. 23);
   begin
      for I in Points'Range loop
         Seeds (I - 5 + 20) := (Points (I).X + 2.25, Points (I).Y + 0.75);
      end loop;
      declare
         Saved_Points : constant Tracking_Point_Array := Points;
         Saved_Seeds : constant Tracking_Point_Array := Seeds;
         Plain : constant Trackability_Track_Array := Track_PyrLK_Trackability
           (Previous, Next, Points, (Max_Level => 1, others => <>));
         Seeded : constant Trackability_Track_Array := Track_PyrLK_Trackability
           (Previous, Next, Points, Quality_Options, Seeds);
      begin
         Assert (Points = Saved_Points and then Seeds = Saved_Seeds, "quality input arrays mutated");
         for I in Points'Range loop
            Assert (Plain (I).Tracked and then Seeded (I).Tracked and then
                      Plain (I).Minimum_Eigenvalue > 0.0 and then Seeded (I).Minimum_Eigenvalue > 0.0,
                    "strided quality Region failed");
            Assert (Near (Seeded (I).Next_Point.X, Points (I).X + 2.0, 0.05) and then
                      Near (Seeded (I).Next_Point.Y, Points (I).Y + 1.0, 0.05), "Region translation differs");
         end loop;
         for R in 0 .. 95 loop
            for C in 0 .. 95 loop
               Assert (OpenCV.Core.UInt8_Access.Get (Previous, R, C) =
                         OpenCV.Core.UInt8_Access.Get (Before_Previous, R, C) and then
                       OpenCV.Core.UInt8_Access.Get (Next, R, C) =
                         OpenCV.Core.UInt8_Access.Get (Before_Next, R, C), "quality source pixels mutated");
            end loop;
         end loop;
      end;
   end Quality_Regions_Immutability;

   procedure Quality_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Compared : Natural := 0;
      Previous_Defined : constant Boolean := Previous_Quality_Defined;
      function Close (Actual : OpenCV.Float32_Value; Expected : OpenCV.Float64_Value) return Boolean is
        (abs (OpenCV.Float64_Value (Actual) - Expected) <= 1.0E-5);
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value ("VIDEO_TRACKABILITY_ORACLE", "../obj/oracle/trackability.txt"));
      for Mode in 0 .. 8 loop
         declare
            Previous : constant OpenCV.Core.Mat :=
              (if Mode in 5 .. 7 then Quality_Structure (Mode - 5) else Texture (96, 96));
            Next : constant OpenCV.Core.Mat := (if Mode = 1 then Shift (Previous, 12, 7) else Previous);
            Points : Tracking_Point_Array :=
              (if Mode in 5 .. 7 then Tracking_Point_Array'[5 => (48.0, 48.0)] else Quality_Points);
            Seeds : Tracking_Point_Array (20 .. 19 + Points'Length);
            Options : PyrLK_Options := Quality_Options;
         begin
            for I in Points'Range loop
               Seeds (I - 5 + 20) := (Points (I).X + (if Mode = 1 then 12.25 elsif Mode = 8 then 32.0 else 0.0),
                                     Points (I).Y + (if Mode = 1 then 6.75 elsif Mode = 8 then 20.0 else 0.0));
            end loop;
            if Mode = 2 then Options.Min_Eigenvalue_Threshold := 100.0; end if;
            if Mode = 3 then Points (6) := (-1000.0, -1000.0); end if;
            if Mode = 4 then Seeds (21) := (-1000.0, -1000.0); end if;
            if Mode = 3 and then not Previous_Defined then
               begin
                  declare
                     Tracks : constant Trackability_Track_Array :=
                       Track_PyrLK_Trackability (Previous, Next, Points, Options);
                     pragma Unreferenced (Tracks);
                  begin
                     Assert (False, "native unwritten quality accepted by Ada");
                  end;
               exception
                  when OpenCV.OpenCV_Error => null;
               end;
               for I in 0 .. 3 loop
                  declare
                     Native_Mode, Index, Status : Integer;
                     X, Y, Eigenvalue : OpenCV.Float64_Value;
                  begin
                     Integers.Get (File, Native_Mode); Integers.Get (File, Index); Integers.Get (File, Status);
                     Floats.Get (File, X); Floats.Get (File, Y); Floats.Get (File, Eigenvalue);
                     Assert (Native_Mode = 3 and then Index = I and then
                               Status = (if I = 1 then -1 else 1), "undefined-quality oracle mapping");
                     Compared := Compared + 1;
                  end;
               end loop;
            else
            declare
               Tracks : constant Trackability_Track_Array :=
                 (if Mode = 1 or else Mode = 4 or else Mode = 8 then
                    Track_PyrLK_Trackability (Previous, Next, Points, Options, Seeds)
                  else Track_PyrLK_Trackability (Previous, Next, Points, Options));
            begin
               for I in Tracks'Range loop
                  declare
                     Native_Mode, Index, Status : Integer;
                     X, Y, Eigenvalue : OpenCV.Float64_Value;
                  begin
                     Integers.Get (File, Native_Mode); Integers.Get (File, Index); Integers.Get (File, Status);
                     Floats.Get (File, X); Floats.Get (File, Y); Floats.Get (File, Eigenvalue);
                     Assert (Native_Mode = Mode and then Index = I - 5, "quality oracle mapping");
                     Assert (Tracks (I).Tracked = (Status = 1) and then Close (Tracks (I).Next_Point.X, X)
                               and then Close (Tracks (I).Next_Point.Y, Y) and then
                               Close (Tracks (I).Minimum_Eigenvalue, Eigenvalue), "quality differs from direct OpenCV");
                     Compared := Compared + 1;
                  end;
               end loop;
            end;
            end if;
         end;
      end loop;
      Assert (Compared = 27 and then Ada.Text_IO.End_Of_File (File), "quality oracle inventory");
      Ada.Text_IO.Close (File);
   end Quality_Direct_Oracle;

   procedure Pyramid_Metadata (T : in out Fixture) is
      pragma Unreferenced (T);
      Requests : constant array (1 .. 3) of Natural := [0, 3, 30];
      Sizes : constant array (1 .. 4) of Positive := [32, 64, 96, 256];
   begin
      for Requested of Requests loop
         for N of Sizes loop
            declare
               Image : constant OpenCV.Core.Mat := Texture (N, N);
               P : constant PyrLK_Pyramid := Build_PyrLK_Pyramid
                 (Image, (Window_Size => (21, 21), Max_Level => Requested));
               Expected : Natural := 0;
               Size : Natural := N;
            begin
               while Expected < Requested loop
                  Size := (Size + 1) / 2;
                  exit when Size <= 21;
                  Expected := Expected + 1;
               end loop;
               Assert (not Is_Empty (P) and then Requested_Max_Level (P) = Requested
                         and then Available_Max_Level (P) = Expected
                         and then Build_Window_Size (P) = OpenCV.Size'(21, 21),
                         "pyramid metadata/truncation");
            end;
         end loop;
      end loop;
   end Pyramid_Metadata;

   procedure Compare_Tracks (Left, Right : Point_Track_Array) is
   begin
      Assert (Left'First = Right'First and then Left'Last = Right'Last, "pyramid bounds");
      for I in Left'Range loop
         Assert (Left (I).Tracked = Right (I).Tracked and then
                   Left (I).Previous_Point = Right (I).Previous_Point and then
                   Near (Left (I).Next_Point.X, Right (I).Next_Point.X, 1.0E-5) and then
                   Near (Left (I).Next_Point.Y, Right (I).Next_Point.Y, 1.0E-5) and then
                   Near (Left (I).Error, Right (I).Error, 1.0E-5), "raw/prebuilt disagreement");
      end loop;
   end Compare_Tracks;

   procedure Pyramid_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (64, 64);
      B : constant OpenCV.Core.Mat := Shift (A, 2, 1);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Points : constant Tracking_Point_Array (7 .. 11) :=
        [7 => Standard_Points (1), 8 => (-1000.0, -1000.0),
         9 => Standard_Points (2), 10 => (1000.0, 1000.0), 11 => Standard_Points (3)];
   begin
      for Level in 0 .. 3 loop
         declare
            Options : constant PyrLK_Options := (Max_Level => Level, others => <>);
            Raw : constant Point_Track_Array := Track_PyrLK (A, B, Points, Options);
            Built : constant Point_Track_Array := Track_PyrLK (PA, PB, Points, Options);
         begin
            Compare_Tracks (Raw, Built);
            Assert (not Built (8).Tracked and then Built (8).Next_Point = Points (8)
                      and then Built (8).Error = 0.0, "pyramid failed normalization");
            if Level = 1 then
               Assert (Built (7).Tracked and then Near (Built (7).Next_Point.X, 22.0)
                         and then Near (Built (7).Next_Point.Y, 21.0), "pyramid translation");
            end if;
         end;
      end loop;
   end Pyramid_Equivalence;

   procedure Pyramid_Bounds_Empty (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (64, 64);
      P : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Image);
      Points : constant Tracking_Point_Array (Positive'Last .. Positive'Last) :=
        [others => Standard_Points (1)];
      Empty : Tracking_Point_Array (9 .. 8);
      Result : constant Point_Track_Array := Track_PyrLK (P, P, Points);
      None : constant Point_Track_Array := Track_PyrLK (P, P, Empty);
   begin
      Assert (Result'First = Positive'Last and then Result (Positive'Last).Tracked,
              "extreme pyramid point bound");
      Assert (None'First = 9 and then None'Last = 8, "empty pyramid point bound");
   end Pyramid_Bounds_Empty;

   procedure Pyramid_Mutation (T : in out Fixture) is
      pragma Unreferenced (T);
      A : OpenCV.Core.Mat := Texture (64, 64);
      B : OpenCV.Core.Mat := Shift (A, 2, 1);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Before : constant Point_Track_Array := Track_PyrLK (PA, PB, Standard_Points);
   begin
      for R in 0 .. 63 loop
         for Col in 0 .. 63 loop
            OpenCV.Core.UInt8_Access.Set (A, R, Col, 0);
            OpenCV.Core.UInt8_Access.Set (B, R, Col, 255);
         end loop;
      end loop;
      Compare_Tracks (Before, Track_PyrLK (PA, PB, Standard_Points));
   end Pyramid_Mutation;

   function Scoped_Pyramid (Region : Boolean; DX, DY : Integer) return PyrLK_Pyramid is
      Parent : constant OpenCV.Core.Mat := Texture (96, 96);
      Source : constant OpenCV.Core.Mat := Shift (Parent, DX, DY);
   begin
      if Region then
         declare
            View : constant OpenCV.Core.Mat := Source.Region ((X => 10, Y => 10, Width => 64, Height => 64));
         begin
            Assert (not View.Is_Continuous, "pyramid Region must be strided");
            return Build_PyrLK_Pyramid (View);
         end;
      end if;
      return Build_PyrLK_Pyramid (Source);
   end Scoped_Pyramid;

   procedure Pyramid_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
   begin
      for Region in Boolean loop
         declare
            PA : constant PyrLK_Pyramid := Scoped_Pyramid (Region, 0, 0);
            PB : constant PyrLK_Pyramid := Scoped_Pyramid (Region, 2, 1);
            Tracks : constant Point_Track_Array := Track_PyrLK
              (PA, PB, Standard_Points, (Max_Level => 1, others => <>));
         begin
            for I in Tracks'Range loop
               Assert (Tracks (I).Tracked and then
                         Near (Tracks (I).Next_Point.X, Standard_Points (I).X + 2.0) and then
                         Near (Tracks (I).Next_Point.Y, Standard_Points (I).Y + 1.0),
                         "pyramid source/Region/parent lifetime dependence");
            end loop;
         end;
      end loop;
   end Pyramid_Lifetime;

   procedure Pyramid_Reuse (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (64, 64);
      B : constant OpenCV.Core.Mat := Shift (A, 2, 1);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      First : constant Point_Track_Array := Track_PyrLK (PA, PB, Standard_Points);
   begin
      for Repetition in 1 .. 3 loop
         Compare_Tracks (First, Track_PyrLK (PA, PB, Standard_Points));
         Compare_Tracks (Track_PyrLK (B, A, Standard_Points),
                         Track_PyrLK (PB, PA, Standard_Points));
      end loop;
   end Pyramid_Reuse;

   procedure Pyramid_Compatibility (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (64, 64);
      B : constant OpenCV.Core.Mat := Texture (63, 64);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Shallow : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A, (Max_Level => 1, others => <>));
      Empty : PyrLK_Pyramid;
      procedure Reject (Previous, Next : PyrLK_Pyramid; Options : PyrLK_Options) is
      begin
         declare
            Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, [], Options);
            pragma Unreferenced (Tracks);
         begin
            Assert (False, "incompatible pyramid accepted even for empty points");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Reject (PA, PB, (others => <>));
      Reject (PA, PA, (Window_Size => (15, 15), others => <>));
      Reject (PA, PA, (Window_Size => (31, 31), others => <>));
      Reject (Shallow, PA, (others => <>));
      Reject (PA, Shallow, (others => <>));
      Reject (Empty, PA, (others => <>));
      Reject (PA, Empty, (others => <>));
      Assert (Is_Empty (Empty), "default pyramid not empty");
      begin
         declare
            Level : constant Natural := Available_Max_Level (Empty);
            pragma Unreferenced (Level);
         begin
            Assert (False, "empty pyramid metadata accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end Pyramid_Compatibility;

   procedure Pyramid_Build_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (64, 64);
      Empty : OpenCV.Core.Mat;
      Depth : constant OpenCV.Core.Mat := OpenCV.Core.Create (64, 64, (OpenCV.Core.Float32, 1));
      Channels : constant OpenCV.Core.Mat := OpenCV.Core.Create (64, 64, (OpenCV.Core.UInt8, 3));
      Volume : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (OpenCV.Core.Dimension_Array'[4, 4, 4], (OpenCV.Core.UInt8, 1));
      procedure Reject (Image : OpenCV.Core.Mat; Options : PyrLK_Pyramid_Options) is
      begin
         declare
            P : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Image, Options);
            pragma Unreferenced (P);
         begin
            Assert (False, "invalid pyramid construction accepted");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end Reject;
   begin
      Reject (Empty, (others => <>));
      Reject (Depth, (others => <>));
      Reject (Channels, (others => <>));
      Reject (Volume, (others => <>));
      Reject (A, (Window_Size => (2, 21), others => <>));
      Reject (A, (Window_Size => (21, 256), others => <>));
      Reject (A, (Max_Level => 31, others => <>));
   end Pyramid_Build_Validation;

   package Caller is new AUnit.Test_Caller (Fixture);

   procedure Seeded_Pyramid_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      Points : constant Tracking_Point_Array (5 .. 8) := Quality_Points;
   begin
      for Mode in 0 .. 2 loop
         declare
            DX : constant Integer := (if Mode = 2 then 12 elsif Mode = 1 then 2 else 0);
            DY : constant Integer := (if Mode = 2 then 7 elsif Mode = 1 then 1 else 0);
            B : constant OpenCV.Core.Mat := Shift (A, DX, DY);
            PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
            PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
            Seeds : Tracking_Point_Array (20 .. 23);
            Options : constant PyrLK_Options := (Max_Level => 0, others => <>);
         begin
            for I in Points'Range loop
               Seeds (I - Points'First + Seeds'First) :=
                 (Points (I).X + OpenCV.Float32_Value (DX) + 0.25,
                  Points (I).Y + OpenCV.Float32_Value (DY) - 0.25);
            end loop;
            declare
               Raw : constant Point_Track_Array := Track_PyrLK (A, B, Points, Options, Seeds);
               Built : constant Point_Track_Array := Track_PyrLK (PA, PB, Points, Options, Seeds);
               Plain : constant Point_Track_Array := Track_PyrLK (PA, PB, Points, Options);
            begin
               Compare_Tracks (Raw, Built);
               for I in Points'Range loop
                  Assert (Built (I).Tracked and then
                    Near (Built (I).Next_Point.X, Points (I).X + OpenCV.Float32_Value (DX), 0.05) and then
                    Near (Built (I).Next_Point.Y, Points (I).Y + OpenCV.Float32_Value (DY), 0.05),
                    "seeded pyramid known translation");
                  Assert (abs (Built (I).Next_Point.X - Seeds (I - 5 + 20).X) > 0.20,
                          "prediction was returned without refinement");
                  if Mode = 2 then
                     Assert (not Plain (I).Tracked or else
                       abs (Plain (I).Next_Point.X - Built (I).Next_Point.X) > 5.0,
                       "distinguishing fixture no longer distinguishes");
                  end if;
               end loop;
            end;
         end;
      end loop;
   end Seeded_Pyramid_Equivalence;

   procedure Seeded_Pyramid_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      Points : constant Tracking_Point_Array := [Positive'Last => (25.0, 25.0)];
      Tracks : constant Point_Track_Array := Track_PyrLK
        (PA, PA, Points, Initial_Next_Points => [20 => (25.25, 24.75)]);
      Empty : constant Point_Track_Array := Track_PyrLK
        (PA, PA, Tracking_Point_Array'(7 .. 6 => <>),
         Initial_Next_Points => Tracking_Point_Array'(20 .. 19 => <>));
   begin
      Assert (Tracks'First = Positive'Last and then Tracks'Last = Positive'Last and then
                Tracks (Positive'Last).Tracked and then Empty'First = 7 and then Empty'Last = 6,
                "seeded pyramid extreme/empty bounds");
   end Seeded_Pyramid_Bounds;

   procedure Seeded_Pyramid_Failures (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      Points : constant Tracking_Point_Array := [5 => (-1000.0, -1000.0), 6 => (25.0, 25.0)];
      Seeds : constant Tracking_Point_Array := [20 => (25.0, 25.0), 21 => (1000.0, 1000.0)];
      Tracks : constant Point_Track_Array := Track_PyrLK
        (PA, PA, Points, (Max_Level => 0, others => <>), Seeds);
   begin
      for I in Points'Range loop
         Assert (not Tracks (I).Tracked and then Tracks (I).Next_Point = Points (I) and then
                   Tracks (I).Previous_Point = Points (I) and then Tracks (I).Error = 0.0,
                   "failed seeded pyramid exposed prediction/native output");
      end loop;
   end Seeded_Pyramid_Failures;

   procedure Seeded_Pyramid_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      function Captured (DX, DY : Integer; Region : Boolean) return PyrLK_Pyramid is
         Parent : OpenCV.Core.Mat := Texture (128, 128);
         Shifted : constant OpenCV.Core.Mat := Shift (Parent, DX, DY);
         Image : OpenCV.Core.Mat := (if Region then Shifted.Region ((16, 16, 96, 96))
                                    else Shifted.Clone);
      begin
         return P : PyrLK_Pyramid := Build_PyrLK_Pyramid (Image) do
            for Row in 0 .. Image.Rows - 1 loop
               for Column in 0 .. Image.Columns - 1 loop
                  OpenCV.Core.UInt8_Access.Set (Image, Row, Column, 0);
               end loop;
            end loop;
            OpenCV.Core.UInt8_Access.Set (Parent, 41, 41, 0);
         end return;
      end Captured;
      Points : constant Tracking_Point_Array := [5 => (25.0, 25.0), 6 => (45.0, 32.0)];
      Seeds : constant Tracking_Point_Array := [20 => (27.25, 25.75), 21 => (47.25, 32.75)];
   begin
      for Region in Boolean loop
         declare
            PA : constant PyrLK_Pyramid := Captured (0, 0, Region);
            PB : constant PyrLK_Pyramid := Captured (2, 1, Region);
            Tracks : constant Point_Track_Array := Track_PyrLK (PA, PB, Points,
              Initial_Next_Points => Seeds);
         begin
            for I in Points'Range loop
               Assert (Tracks (I).Tracked and then
                 Near (Tracks (I).Next_Point.X, Points (I).X + 2.0, 0.05) and then
                 Near (Tracks (I).Next_Point.Y, Points (I).Y + 1.0, 0.05),
                 "seeded pyramid source/Region lifetime");
            end loop;
         end;
      end loop;
   end Seeded_Pyramid_Lifetime;

   procedure Seeded_Pyramid_Reuse (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      B : constant OpenCV.Core.Mat := Shift (A, 2, 1);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Points : constant Tracking_Point_Array := [5 => (25.0, 25.0)];
      Seeds : constant Tracking_Point_Array := [20 => (27.25, 25.75)];
      First : constant Point_Track_Array := Track_PyrLK (PA, PB, Points, Initial_Next_Points => Seeds);
      Second : constant Point_Track_Array := Track_PyrLK
        (PA, PB, Points, Initial_Next_Points => [30 => (26.75, 26.25)]);
      Again : constant Point_Track_Array := Track_PyrLK (PA, PB, Points, Initial_Next_Points => Seeds);
      Back : constant Point_Track_Array := Track_PyrLK
        (PB, PA, [5 => (27.0, 26.0)], (Max_Level => 0, others => <>), Points);
   begin
      Compare_Tracks (First, Again);
      Assert (Second (5).Tracked and then Back (5).Tracked and then
        Near (Back (5).Next_Point.X, Points (5).X, 0.05) and then
        Near (Back (5).Next_Point.Y, Points (5).Y, 0.05), "seeded pyramid reuse/reversal");
   end Seeded_Pyramid_Reuse;

   procedure Seeded_Pyramid_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      B : constant OpenCV.Core.Mat := Texture (95, 96);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Shallow : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A, (Max_Level => 0, others => <>));
      Null_Pyramid : PyrLK_Pyramid;
   begin
      for Mode in 0 .. 9 loop
         begin
            declare
               Options : PyrLK_Options := (others => <>);
               Points : constant Tracking_Point_Array :=
                 (if Mode = 9 then Tracking_Point_Array'(7 .. 6 => <>) else [5 => (25.0, 25.0)]);
               Seeds : constant Tracking_Point_Array :=
                 (if Mode = 9 then Tracking_Point_Array'(20 .. 19 => <>)
                  elsif Mode = 4 then Tracking_Point_Array'(20 .. 19 => <>)
                  elsif Mode = 5 then [20 => (536_871_040.0, 0.0)]
                  else [20 => (25.0, 25.0)]);
            begin
               if Mode = 3 then Options.Window_Size := (15, 15); end if;
               if Mode = 6 then Options.Epsilon := 0.0; end if;
               if Mode = 7 then Options.Min_Eigenvalue_Threshold := -1.0; end if;
               if Mode in 8 .. 9 then Options.Max_Level := 31; end if;
               declare
                  Tracks : constant Point_Track_Array :=
                    (if Mode = 0 then Track_PyrLK (Null_Pyramid, PA, Points, Options, Seeds)
                     elsif Mode = 1 then Track_PyrLK (PA, PB, Points, Options, Seeds)
                     elsif Mode = 2 then Track_PyrLK (PA, Shallow, Points, Options, Seeds)
                     else Track_PyrLK (PA, PA, Points, Options, Seeds));
                  pragma Unreferenced (Tracks);
               begin
                  Assert (False, "seeded pyramid validation bypassed");
               end;
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end loop;
   end Seeded_Pyramid_Validation;

   function Pyramid_Previous_Quality_Defined return Boolean is
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Mode, Index, Status : Integer;
      Value : OpenCV.Float64_Value;
      Defined : Boolean := True;
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value ("VIDEO_TRACKABILITY_ORACLE", "../obj/oracle/trackability.txt") & ".pyramids");
      for Record_Number in 1 .. 27 loop
         Integers.Get (File, Mode); Integers.Get (File, Index); Integers.Get (File, Status);
         for Component in 1 .. 3 loop Floats.Get (File, Value); end loop;
         if Status = -1 then
            Assert (Mode = 3 and then Index = 1, "unexpected undefined native quality path");
            Defined := False;
         end if;
      end loop;
      Ada.Text_IO.Close (File);
      return Defined;
   end Pyramid_Previous_Quality_Defined;



   procedure Pyramid_Quality_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      package Integers is new Ada.Text_IO.Integer_IO (Integer);
      package Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);
      File : Ada.Text_IO.File_Type;
      Compared : Natural := 0;
      Previous_Defined : constant Boolean := Pyramid_Previous_Quality_Defined;
      function Close (Actual : OpenCV.Float32_Value; Expected : OpenCV.Float64_Value) return Boolean is
        (abs (OpenCV.Float64_Value (Actual) - Expected) <= 1.0E-5);
   begin
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File,
        Ada.Environment_Variables.Value ("VIDEO_TRACKABILITY_ORACLE", "../obj/oracle/trackability.txt") & ".pyramids");
      for Mode in 0 .. 8 loop
         declare
            Previous : constant OpenCV.Core.Mat :=
              (if Mode in 5 .. 7 then Quality_Structure (Mode - 5) else Texture (96, 96));
            Next : constant OpenCV.Core.Mat := (if Mode = 1 then Shift (Previous, 12, 7) else Previous);
            PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Previous);
            PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Next);
            Points : Tracking_Point_Array :=
              (if Mode in 5 .. 7 then Tracking_Point_Array'[5 => (48.0, 48.0)] else Quality_Points);
            Seeds : Tracking_Point_Array (20 .. 19 + Points'Length);
            Options : PyrLK_Options := Quality_Options;
         begin
            for I in Points'Range loop
               Seeds (I - 5 + 20) := (Points (I).X + (if Mode = 1 then 12.25 elsif Mode = 8 then 32.0 else 0.0),
                                     Points (I).Y + (if Mode = 1 then 6.75 elsif Mode = 8 then 20.0 else 0.0));
            end loop;
            if Mode = 2 then Options.Min_Eigenvalue_Threshold := 100.0; end if;
            if Mode = 3 then Points (6) := (-1000.0, -1000.0); end if;
            if Mode = 4 then Seeds (21) := (-1000.0, -1000.0); end if;
            if Mode = 3 and then not Previous_Defined then
               begin
                  declare
                     Tracks : constant Trackability_Track_Array :=
                       Track_PyrLK_Trackability (PA, PB, Points, Options);
                     pragma Unreferenced (Tracks);
                  begin
                     Assert (False, "native unwritten quality accepted by Ada");
                  end;
               exception
                  when OpenCV.OpenCV_Error => null;
               end;
               for I in 0 .. 3 loop
                  declare
                     Native_Mode, Index, Status : Integer;
                     X, Y, Eigenvalue : OpenCV.Float64_Value;
                  begin
                     Integers.Get (File, Native_Mode); Integers.Get (File, Index); Integers.Get (File, Status);
                     Floats.Get (File, X); Floats.Get (File, Y); Floats.Get (File, Eigenvalue);
                     Assert (Native_Mode = 3 and then Index = I and then
                               Status = (if I = 1 then -1 else 1), "undefined-quality oracle mapping");
                     Compared := Compared + 1;
                  end;
               end loop;
            else
            declare
               Tracks : constant Trackability_Track_Array :=
                 (if Mode = 1 or else Mode = 4 or else Mode = 8 then
                    Track_PyrLK_Trackability (PA, PB, Points, Options, Seeds)
                  else Track_PyrLK_Trackability (PA, PB, Points, Options));
            begin
               for I in Tracks'Range loop
                  declare
                     Native_Mode, Index, Status : Integer;
                     X, Y, Eigenvalue : OpenCV.Float64_Value;
                  begin
                     Integers.Get (File, Native_Mode); Integers.Get (File, Index); Integers.Get (File, Status);
                     Floats.Get (File, X); Floats.Get (File, Y); Floats.Get (File, Eigenvalue);
                     Assert (Native_Mode = Mode and then Index = I - 5, "quality oracle mapping");
                     Assert (Tracks (I).Tracked = (Status = 1) and then Close (Tracks (I).Next_Point.X, X)
                               and then Close (Tracks (I).Next_Point.Y, Y) and then
                               Close (Tracks (I).Minimum_Eigenvalue, Eigenvalue), "quality differs from direct OpenCV");
                     Compared := Compared + 1;
                  end;
               end loop;
            end;
            end if;
         end;
      end loop;
      Assert (Compared = 27 and then Ada.Text_IO.End_Of_File (File), "quality oracle inventory");
      Ada.Text_IO.Close (File);
   end Pyramid_Quality_Direct_Oracle;



   procedure Quality_Pyramid_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      Points : constant Tracking_Point_Array (5 .. 8) := Quality_Points;
   begin
      for Mode in 0 .. 2 loop
         declare
            DX : constant Integer := (if Mode = 2 then 12 elsif Mode = 1 then 2 else 0);
            DY : constant Integer := (if Mode = 2 then 7 elsif Mode = 1 then 1 else 0);
            B : constant OpenCV.Core.Mat := Shift (A, DX, DY);
            PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
            PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
            Seeds : Tracking_Point_Array (20 .. 23);
            Options : constant PyrLK_Options := (Max_Level => 0, others => <>);
         begin
            for I in Points'Range loop
               Seeds (I - Points'First + Seeds'First) :=
                 (Points (I).X + OpenCV.Float32_Value (DX) + 0.25,
                  Points (I).Y + OpenCV.Float32_Value (DY) - 0.25);
            end loop;
            declare
               Raw : constant Trackability_Track_Array := Track_PyrLK_Trackability (A, B, Points, Options, Seeds);
               Built : constant Trackability_Track_Array := Track_PyrLK_Trackability (PA, PB, Points, Options, Seeds);
               Plain : constant Trackability_Track_Array := Track_PyrLK_Trackability (PA, PB, Points, Options);
            begin
               for I in Raw'Range loop
                  Assert (Raw (I).Tracked = Built (I).Tracked and then
                    Near (Raw (I).Next_Point.X, Built (I).Next_Point.X, 1.0E-5) and then
                    Near (Raw (I).Next_Point.Y, Built (I).Next_Point.Y, 1.0E-5) and then
                    Near (Raw (I).Minimum_Eigenvalue, Built (I).Minimum_Eigenvalue, 1.0E-5),
                    "raw/pyramid quality or repeat differs");
               end loop;
               for I in Points'Range loop
                  Assert (Built (I).Tracked and then
                    Near (Built (I).Next_Point.X, Points (I).X + OpenCV.Float32_Value (DX), 0.05) and then
                    Near (Built (I).Next_Point.Y, Points (I).Y + OpenCV.Float32_Value (DY), 0.05),
                    "seeded pyramid known translation");
                  Assert (abs (Built (I).Next_Point.X - Seeds (I - 5 + 20).X) > 0.20,
                          "prediction was returned without refinement");
                  if Mode = 2 then
                     Assert (not Plain (I).Tracked or else
                       abs (Plain (I).Next_Point.X - Built (I).Next_Point.X) > 5.0,
                       "distinguishing fixture no longer distinguishes");
                  end if;
               end loop;
            end;
         end;
      end loop;
   end Quality_Pyramid_Equivalence;

   procedure Quality_Pyramid_Bounds (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      Points : constant Tracking_Point_Array := [Positive'Last => (25.0, 25.0)];
      Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability
        (PA, PA, Points, Initial_Next_Points => [20 => (25.25, 24.75)]);
      Empty : constant Trackability_Track_Array := Track_PyrLK_Trackability
        (PA, PA, Tracking_Point_Array'(7 .. 6 => <>),
         Initial_Next_Points => Tracking_Point_Array'(20 .. 19 => <>));
   begin
      Assert (Tracks'First = Positive'Last and then Tracks'Last = Positive'Last and then
                Tracks (Positive'Last).Tracked and then Empty'First = 7 and then Empty'Last = 6,
                "seeded pyramid extreme/empty bounds");
   end Quality_Pyramid_Bounds;

   procedure Quality_Pyramid_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      function Captured (DX, DY : Integer; Region : Boolean) return PyrLK_Pyramid is
         Parent : OpenCV.Core.Mat := Texture (128, 128);
         Shifted : constant OpenCV.Core.Mat := Shift (Parent, DX, DY);
         Image : OpenCV.Core.Mat := (if Region then Shifted.Region ((16, 16, 96, 96))
                                    else Shifted.Clone);
      begin
         return P : PyrLK_Pyramid := Build_PyrLK_Pyramid (Image) do
            for Row in 0 .. Image.Rows - 1 loop
               for Column in 0 .. Image.Columns - 1 loop
                  OpenCV.Core.UInt8_Access.Set (Image, Row, Column, 0);
               end loop;
            end loop;
            OpenCV.Core.UInt8_Access.Set (Parent, 41, 41, 0);
         end return;
      end Captured;
      Points : constant Tracking_Point_Array := [5 => (25.0, 25.0), 6 => (45.0, 32.0)];
      Seeds : constant Tracking_Point_Array := [20 => (27.25, 25.75), 21 => (47.25, 32.75)];
   begin
      for Region in Boolean loop
         declare
            PA : constant PyrLK_Pyramid := Captured (0, 0, Region);
            PB : constant PyrLK_Pyramid := Captured (2, 1, Region);
            Plain : constant Trackability_Track_Array := Track_PyrLK_Trackability (PA, PB, Points);
            Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability (PA, PB, Points,
              Initial_Next_Points => Seeds);
         begin
            for I in Points'Range loop
               Assert (Plain (I).Tracked and then Tracks (I).Tracked and then
                 Near (Tracks (I).Next_Point.X, Points (I).X + 2.0, 0.05) and then
                 Near (Tracks (I).Next_Point.Y, Points (I).Y + 1.0, 0.05),
                 "seeded pyramid source/Region lifetime");
            end loop;
         end;
      end loop;
   end Quality_Pyramid_Lifetime;

   procedure Quality_Pyramid_Reuse (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      B : constant OpenCV.Core.Mat := Shift (A, 2, 1);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Points : constant Tracking_Point_Array := [5 => (25.0, 25.0)];
      Seeds : constant Tracking_Point_Array := [20 => (27.25, 25.75)];
      First : constant Trackability_Track_Array := Track_PyrLK_Trackability (PA, PB, Points, Initial_Next_Points => Seeds);
      Second : constant Trackability_Track_Array := Track_PyrLK_Trackability
        (PA, PB, Points, Initial_Next_Points => [30 => (26.75, 26.25)]);
      Again : constant Trackability_Track_Array := Track_PyrLK_Trackability (PA, PB, Points, Initial_Next_Points => Seeds);
      Back : constant Trackability_Track_Array := Track_PyrLK_Trackability
        (PB, PA, [5 => (27.0, 26.0)], (Max_Level => 0, others => <>), Points);
   begin
      for I in First'Range loop
                  Assert (First (I).Tracked = Again (I).Tracked and then
                    Near (First (I).Next_Point.X, Again (I).Next_Point.X, 1.0E-5) and then
                    Near (First (I).Next_Point.Y, Again (I).Next_Point.Y, 1.0E-5) and then
                    Near (First (I).Minimum_Eigenvalue, Again (I).Minimum_Eigenvalue, 1.0E-5),
                    "raw/pyramid quality or repeat differs");
               end loop;
      Assert (Second (5).Tracked and then Back (5).Tracked and then
        Near (Back (5).Next_Point.X, Points (5).X, 0.05) and then
        Near (Back (5).Next_Point.Y, Points (5).Y, 0.05), "seeded pyramid reuse/reversal");
   end Quality_Pyramid_Reuse;

   procedure Quality_Pyramid_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      A : constant OpenCV.Core.Mat := Texture (96, 96);
      B : constant OpenCV.Core.Mat := Texture (95, 96);
      PA : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A);
      PB : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (B);
      Shallow : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (A, (Max_Level => 0, others => <>));
      Null_Pyramid : PyrLK_Pyramid;
      function From_Bits is new Ada.Unchecked_Conversion
        (Interfaces.Unsigned_32, OpenCV.Float32_Value);
      type Bit_Array is array (Positive range <>) of Interfaces.Unsigned_32;
   begin
      for Seeded in Boolean loop
      for Mode in 0 .. 9 loop
         if Seeded or else Mode not in 4 .. 5 then
         begin
            declare
               Options : PyrLK_Options := (others => <>);
               Points : constant Tracking_Point_Array :=
                 (if Mode = 9 then Tracking_Point_Array'(7 .. 6 => <>) else [5 => (25.0, 25.0)]);
               Seeds : constant Tracking_Point_Array :=
                 (if Mode = 9 then Tracking_Point_Array'(20 .. 19 => <>)
                  elsif Mode = 4 then Tracking_Point_Array'(20 .. 19 => <>)
                  elsif Mode = 5 then [20 => (536_871_040.0, 0.0)]
                  else [20 => (25.0, 25.0)]);
            begin
               if Mode = 3 then Options.Window_Size := (15, 15); end if;
               if Mode = 6 then Options.Epsilon := 0.0; end if;
               if Mode = 7 then Options.Min_Eigenvalue_Threshold := -1.0; end if;
               if Mode in 8 .. 9 then Options.Max_Level := 31; end if;
               declare
                  Tracks : constant Trackability_Track_Array :=
                    (if Seeded then
                       (if Mode = 0 then Track_PyrLK_Trackability (Null_Pyramid, PA, Points, Options, Seeds)
                        elsif Mode = 1 then Track_PyrLK_Trackability (PA, PB, Points, Options, Seeds)
                        elsif Mode = 2 then Track_PyrLK_Trackability (PA, Shallow, Points, Options, Seeds)
                        else Track_PyrLK_Trackability (PA, PA, Points, Options, Seeds))
                     else
                       (if Mode = 0 then Track_PyrLK_Trackability (Null_Pyramid, PA, Points, Options)
                        elsif Mode = 1 then Track_PyrLK_Trackability (PA, PB, Points, Options)
                        elsif Mode = 2 then Track_PyrLK_Trackability (PA, Shallow, Points, Options)
                        else Track_PyrLK_Trackability (PA, PA, Points, Options)));
                  pragma Unreferenced (Tracks);
               begin
                  Assert (False, "seeded pyramid validation bypassed");
               end;
            end;
         exception
            when OpenCV.OpenCV_Error => null;
         end;
      end if;
      end loop;
      end loop;
      for Bits of Bit_Array'[16#7FC0_0000#, 16#7F80_0000#] loop
         begin
            declare
               Tracks : constant Trackability_Track_Array := Track_PyrLK_Trackability
                 (PA, PA, [1 => (25.0, 25.0)], Initial_Next_Points => [20 => (25.0, From_Bits (Bits))]);
               pragma Unreferenced (Tracks);
            begin
               Assert (False, "nonfinite pyramid quality seed accepted");
            end;
         exception
            when OpenCV.OpenCV_Error | Constraint_Error => null;
         end;
      end loop;
   end Quality_Pyramid_Validation;

   procedure Pyramid_Quality_Semantics (T : in out Fixture) is
      pragma Unreferenced (T);
      Values : array (0 .. 2) of OpenCV.Float32_Value;
   begin
      for Kind in Values'Range loop
         declare
            Image : constant OpenCV.Core.Mat := Quality_Structure (Kind);
            P : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Image);
            Points : constant Tracking_Point_Array := [7 => (48.0, 48.0)];
            Baseline : constant Trackability_Track_Array :=
              Track_PyrLK_Trackability (P, P, Points, Quality_Options);
         begin
            Values (Kind) := Baseline (7).Minimum_Eigenvalue;
            Assert (Baseline (7).Tracked = (Kind = 0), "pyramid structure status");
            if Kind = 0 then
               for Seeded in Boolean loop
                  for Below in Boolean loop
                     declare
                        Options : PyrLK_Options := Quality_Options;
                     begin
                        Options.Min_Eigenvalue_Threshold := OpenCV.Float64_Value (Values (Kind)) *
                          (if Below then 0.5 else 2.0);
                        declare
                           Tracks : constant Trackability_Track_Array :=
                             (if Seeded then Track_PyrLK_Trackability (P, P, Points, Options, Points)
                              else Track_PyrLK_Trackability (P, P, Points, Options));
                        begin
                           Assert (Tracks (7).Tracked = Below and then
                             Tracks (7).Minimum_Eigenvalue = Values (Kind) and then
                             Tracks (7).Next_Point = Points (7), "pyramid threshold discarded quality");
                        end;
                     end;
                  end loop;
               end loop;
            end if;
         end;
      end loop;
      Assert (Values (0) > 0.5 and then Values (1) = 0.0 and then Values (2) = 0.0,
              "pyramid corner/edge/flat relationship");
      declare
         Image : constant OpenCV.Core.Mat := Texture (96, 96);
         P : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Image);
         Points : constant Tracking_Point_Array := Quality_Points;
         Plain : constant Point_Track_Array := Track_PyrLK (P, P, Points, Quality_Options);
         Quality : constant Trackability_Track_Array := Track_PyrLK_Trackability (P, P, Points, Quality_Options);
         Seeded : constant Trackability_Track_Array := Track_PyrLK_Trackability (P, P, Points, Quality_Options, Points);
         Failed : constant Trackability_Track_Array := Track_PyrLK_Trackability
           (P, P, [7 => Points (5)], Quality_Options, [20 => (-1000.0, -1000.0)]);
      begin
         for I in Points'Range loop
            Assert (Plain (I).Tracked and then Quality (I).Tracked and then Seeded (I).Tracked and then
              Plain (I).Error = 0.0 and then Quality (I).Minimum_Eigenvalue > 0.0 and then
              Quality (I).Minimum_Eigenvalue = Seeded (I).Minimum_Eigenvalue and then
              Quality (I).Next_Point = Points (I), "pyramid metrics/identity/seed independence");
         end loop;
         Assert (not Failed (7).Tracked and then Failed (7).Next_Point = Points (5) and then
           Failed (7).Minimum_Eigenvalue = Quality (5).Minimum_Eigenvalue,
           "pyramid next failure discarded quality");
      end;
   end Pyramid_Quality_Semantics;

   procedure Pyramid_Quality_Previous_Unavailable (T : in out Fixture) is
      pragma Unreferenced (T);
      Image : constant OpenCV.Core.Mat := Texture (96, 96);
      P : constant PyrLK_Pyramid := Build_PyrLK_Pyramid (Image);
      Points : constant Tracking_Point_Array := [7 => (25.0, 25.0), 8 => (-1000.0, -1000.0)];
      Defined : constant Boolean := Pyramid_Previous_Quality_Defined;
   begin
      for Seeded in Boolean loop
         begin
            declare
               Tracks : constant Trackability_Track_Array :=
                 (if Seeded then Track_PyrLK_Trackability (P, P, Points, Quality_Options, Points)
                  else Track_PyrLK_Trackability (P, P, Points, Quality_Options));
            begin
               Assert (Defined, "native undefined previous quality accepted");
               Assert (Tracks (7).Tracked and then not Tracks (8).Tracked and then
                 Tracks (8).Minimum_Eigenvalue = 0.0 and then Tracks (8).Next_Point = Points (8),
                 "defined CPU previous quality/failed normalization");
            end;
         exception
            when OpenCV.OpenCV_Error => Assert (not Defined, "defined native previous quality rejected");
         end;
      end loop;
   end Pyramid_Quality_Previous_Unavailable;

   ---------------------------------------------------------------------
   --  Task 011: ECC translation
   ---------------------------------------------------------------------
   package EF renames ECC_Fixtures;

   ECC_Tolerance : constant OpenCV.Float32_Value := 0.05;

   function ECC_Close (A, B : ECC_Translation_Result; Tolerance : OpenCV.Float64_Value)
     return Boolean is
     (abs (OpenCV.Float64_Value (A.Displacement.X) - OpenCV.Float64_Value (B.Displacement.X)) <= Tolerance
      and then abs (OpenCV.Float64_Value (A.Displacement.Y) - OpenCV.Float64_Value (B.Displacement.Y))
               <= Tolerance
      and then abs (A.Correlation - B.Correlation) <= Tolerance);

   function ECC_Near (R : ECC_Translation_Result; DX, DY : Integer) return Boolean is
     (abs (R.Displacement.X - OpenCV.Float32_Value (DX)) < ECC_Tolerance
      and then abs (R.Displacement.Y - OpenCV.Float32_Value (DY)) < ECC_Tolerance);

   function ECC_Template return OpenCV.Core.Mat is (EF.Texture (128, 128));

   procedure ECC_Expect_Error
     (Template, Moving : OpenCV.Core.Mat;
      Options : ECC_Translation_Options := (others => <>);
      Message : String)
   is
      Unused : ECC_Translation_Result;
   begin
      Unused := Estimate_ECC_Translation (Template, Moving, Options);
      Assert (False, Message & ": request accepted");
   exception
      when OpenCV.OpenCV_Error => null;
   end ECC_Expect_Error;

   --  The Ada preflight must reject before any native call. A rejection raised by the
   --  shim carries the "Video.Estimate_ECC_Translation:" prefix added by C_API.Check, so
   --  its presence means the Ada validation layer was bypassed.
   procedure ECC_Expect_Preflight_Error
     (Template, Moving : OpenCV.Core.Mat; Message : String)
   is
      Unused : ECC_Translation_Result;
   begin
      Unused := Estimate_ECC_Translation (Template, Moving);
      Assert (False, Message & ": request accepted");
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text   : constant String := Ada.Exceptions.Exception_Message (Error);
            Marker : constant String := "Video.Estimate_ECC_Translation";
         begin
            Assert (Text'Length < Marker'Length
                    or else Text (Text'First .. Text'First + Marker'Length - 1) /= Marker,
                    Message & ": rejected by the native layer, not the Ada preflight");
         end;
   end ECC_Expect_Preflight_Error;

   procedure ECC_Default_Options (T : in out Fixture) is
      pragma Unreferenced (T);
      Defaults : constant ECC_Translation_Options := (others => <>);
      Zero : constant ECC_Translation_Result := (others => <>);
   begin
      Assert (Defaults.Maximum_Iterations = 50 and then Defaults.Epsilon = 0.0001,
              "ECC default options");
      Assert (Zero.Displacement = (X => 0.0, Y => 0.0) and then Zero.Correlation = 0.0,
              "ECC default result");
   end ECC_Default_Options;

   procedure ECC_Translation_Directions (T : in out Fixture) is
      pragma Unreferenced (T);
      Template : constant OpenCV.Core.Mat := ECC_Template;
      Before : constant Long_Long_Integer := EF.Checksum (Template);
      type Shift is record DX, DY : Integer; end record;
      Cases : constant array (1 .. 5) of Shift :=
        [(0, 0), (2, 1), (-2, -1), (3, 0), (0, 3)];
   begin
      for Item of Cases loop
         declare
            Moving : constant OpenCV.Core.Mat := EF.Texture (128, 128, Item.DX, Item.DY);
            Moving_Before : constant Long_Long_Integer := EF.Checksum (Moving);
            Forward : constant ECC_Translation_Result := Estimate_ECC_Translation (Template, Moving);
            Backward : constant ECC_Translation_Result := Estimate_ECC_Translation (Moving, Template);
         begin
            Assert (ECC_Near (Forward, Item.DX, Item.DY), "ECC displacement sign/order/magnitude");
            Assert (ECC_Near (Backward, -Item.DX, -Item.DY), "ECC reversed roles are not negated");
            Assert (Forward.Correlation > 0.99 and then Forward.Correlation < 1.0001,
                    "ECC correlation range");
            Assert (EF.Checksum (Template) = Before and then EF.Checksum (Moving) = Moving_Before,
                    "ECC modified an input");
         end;
      end loop;
      --  X and Y are distinct: a horizontal shift must not produce vertical motion.
      declare
         H : constant ECC_Translation_Result :=
           Estimate_ECC_Translation (Template, EF.Texture (128, 128, 3, 0));
         V : constant ECC_Translation_Result :=
           Estimate_ECC_Translation (Template, EF.Texture (128, 128, 0, 3));
      begin
         Assert (H.Displacement.X > 2.5 and then abs (H.Displacement.Y) < 0.1, "ECC horizontal axis");
         Assert (V.Displacement.Y > 2.5 and then abs (V.Displacement.X) < 0.1, "ECC vertical axis");
      end;
   end ECC_Translation_Directions;

   procedure ECC_Options_And_Repeatability (T : in out Fixture) is
      pragma Unreferenced (T);
      Template : constant OpenCV.Core.Mat := ECC_Template;
      Moving : constant OpenCV.Core.Mat := EF.Texture (128, 128, 2, 1);
      Default_1 : constant ECC_Translation_Result := Estimate_ECC_Translation (Template, Moving);
      Default_2 : constant ECC_Translation_Result := Estimate_ECC_Translation (Template, Moving);
      One : constant ECC_Translation_Result := Estimate_ECC_Translation
        (Template, Moving, (Maximum_Iterations => 1, Epsilon => 0.0001));
      Two : constant ECC_Translation_Result := Estimate_ECC_Translation
        (Template, Moving, (Maximum_Iterations => 2, Epsilon => 0.0001));
      Fine : constant ECC_Translation_Result := Estimate_ECC_Translation
        (Template, Moving, (Maximum_Iterations => 100, Epsilon => 1.0E-8));
      Coarse : constant ECC_Translation_Result := Estimate_ECC_Translation
        (Template, Moving, (Maximum_Iterations => 100, Epsilon => 0.1));
   begin
      Assert (Default_1 = Default_2, "ECC repeated calls differ");
      --  One iteration cannot reach the minimum: a binding that ignores the count fails.
      Assert (abs (One.Displacement.X - 2.0) > 0.1 and then One.Correlation < 0.95,
              "ECC one-iteration option had no distinguishing effect");
      Assert (One.Displacement /= Default_1.Displacement, "ECC iteration option ignored");
      Assert (ECC_Near (Two, 2, 1) and then ECC_Near (Fine, 2, 1) and then ECC_Near (Coarse, 2, 1),
              "ECC valid nondefault options lost accuracy");
   end ECC_Options_And_Repeatability;

   procedure ECC_Epsilon_Sensitivity (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Shift (4,2) is still inside the basin but does not converge within 1e-1 in the
      --  same step count as 1e-8, so the epsilon option measurably changes the result.
      Template : constant OpenCV.Core.Mat := ECC_Template;
      Moving : constant OpenCV.Core.Mat := EF.Texture (128, 128, 4, 2);
      Coarse : constant ECC_Translation_Result := Estimate_ECC_Translation
        (Template, Moving, (Maximum_Iterations => 100, Epsilon => 0.1));
      Fine : constant ECC_Translation_Result := Estimate_ECC_Translation
        (Template, Moving, (Maximum_Iterations => 100, Epsilon => 1.0E-8));
   begin
      Assert (ECC_Near (Coarse, 4, 2) and then ECC_Near (Fine, 4, 2), "ECC (4,2) accuracy");
      Assert (Fine.Correlation - Coarse.Correlation > 1.0E-4, "ECC epsilon option ignored");
   end ECC_Epsilon_Sensitivity;

   procedure ECC_Photometric (T : in out Fixture) is
      pragma Unreferenced (T);
      Template : constant OpenCV.Core.Mat := ECC_Template;
      Shifted : constant OpenCV.Core.Mat := EF.Texture (128, 128, 2, 1);
      Dimmer : constant OpenCV.Core.Mat := EF.Affine (Shifted, 4, 5, 20);
      Brighter : constant OpenCV.Core.Mat := EF.Affine (Shifted, 13, 10, -10);
   begin
      Assert (ECC_Near (Estimate_ECC_Translation (Template, Dimmer), 2, 1), "ECC contrast 0.8x+20");
      Assert (ECC_Near (Estimate_ECC_Translation (Template, Brighter), 2, 1), "ECC contrast 1.3x-10");
   end ECC_Photometric;

   procedure ECC_Regions_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Saved : ECC_Translation_Result;
      Parent_A_Before, Parent_B_Before : Long_Long_Integer;
      Contiguous : ECC_Translation_Result;
   begin
      declare
         Parent_A : constant OpenCV.Core.Mat := EF.Texture (160, 160);
         Parent_B : constant OpenCV.Core.Mat := EF.Texture (160, 160, 2, 1);
         Region_A : constant OpenCV.Core.Mat := Parent_A.Region (Window);
         Region_B : constant OpenCV.Core.Mat := Parent_B.Region (Window);
         Region_A_Before : constant Long_Long_Integer := EF.Checksum (Region_A);
         Region_B_Before : constant Long_Long_Integer := EF.Checksum (Region_B);
      begin
         Assert (not Region_A.Is_Continuous and then not Region_B.Is_Continuous,
                 "ECC fixture Regions are contiguous");
         Parent_A_Before := EF.Checksum (Parent_A);
         Parent_B_Before := EF.Checksum (Parent_B);
         Saved := Estimate_ECC_Translation (Region_A, Region_B);
         Contiguous := Estimate_ECC_Translation (Region_A.Clone, Region_B.Clone);
         declare
            Template_Only : constant ECC_Translation_Result :=
              Estimate_ECC_Translation (Region_A, Region_B.Clone);
            Moving_Only : constant ECC_Translation_Result :=
              Estimate_ECC_Translation (Region_A.Clone, Region_B);
         begin
            Assert (ECC_Near (Saved, 2, 1), "ECC Region displacement");
            Assert (ECC_Close (Template_Only, Saved, 1.0E-6) and then ECC_Close (Moving_Only, Saved, 1.0E-6),
                    "ECC template-only/moving-only Region differs from both-Region result");
         end;
         Assert (ECC_Close (Contiguous, Saved, 1.0E-6),
                 "ECC Region result differs from the contiguous clone");
         Assert (EF.Checksum (Region_A) = Region_A_Before and then EF.Checksum (Region_B) = Region_B_Before
                 and then EF.Checksum (Parent_A) = Parent_A_Before
                 and then EF.Checksum (Parent_B) = Parent_B_Before,
                 "ECC modified a Region or its parent");
      end;
      --  Regions and parents are finalized; the value result is still usable.
      Assert (ECC_Near (Saved, 2, 1) and then Saved.Correlation > 0.99, "ECC result after finalization");
   end ECC_Regions_Lifetime;

   procedure ECC_Image_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Good : constant OpenCV.Core.Mat := ECC_Template;
      Empty : OpenCV.Core.Mat;
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.UInt8, Channels => 3));
      Other_Columns : constant OpenCV.Core.Mat := EF.Texture (128, 129);
      Other_Rows : constant OpenCV.Core.Mat := EF.Texture (129, 128);
      Parent : constant OpenCV.Core.Mat := EF.Texture (160, 160);
   begin
      ECC_Expect_Error (Empty, Good, Message => "empty template");
      ECC_Expect_Error (Good, Empty, Message => "empty moving");
      ECC_Expect_Error (Float_Image, Float_Image, Message => "Float32 images");
      ECC_Expect_Preflight_Error (Float_Image, Float_Image, "Float32 images");
      ECC_Expect_Preflight_Error (Color, Color, "three-channel images");
      ECC_Expect_Preflight_Error (Good, Other_Columns, "different columns");
      ECC_Expect_Error (Good, Float_Image, Message => "Float32 moving");
      ECC_Expect_Error (Color, Color, Message => "three-channel images");
      ECC_Expect_Error (Good, Color, Message => "three-channel moving");
      ECC_Expect_Error (Good, Other_Columns, Message => "different columns");
      ECC_Expect_Error (Other_Rows, Good, Message => "different rows");
      ECC_Expect_Error (EF.Texture (31, 128), EF.Texture (31, 128), Message => "31 rows");
      ECC_Expect_Error (EF.Texture (128, 31), EF.Texture (128, 31), Message => "31 columns");
      ECC_Expect_Error (Parent.Region ((0, 0, 31, 31)), Parent.Region ((0, 0, 31, 31)),
                        Message => "31x31 Regions");
      --  Upper geometry limits are rejected by validation before any native work, so
      --  no ECC temporaries are allocated: columns/rows 16385, and area 2**24 + 4096.
      declare
         Wide_A : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (32, 16_385, (Depth => OpenCV.Core.UInt8, Channels => 1));
         Tall_A : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (16_385, 32, (Depth => OpenCV.Core.UInt8, Channels => 1));
         Area_A : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (4_097, 4_096, (Depth => OpenCV.Core.UInt8, Channels => 1));
         Area_B : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (16_384, 1_025, (Depth => OpenCV.Core.UInt8, Channels => 1));
      begin
         ECC_Expect_Error (Wide_A, Wide_A, Message => "16385 columns");
         ECC_Expect_Error (Tall_A, Tall_A, Message => "16385 rows");
         ECC_Expect_Error (Area_A, Area_A, Message => "4097 x 4096 pixels");
         ECC_Expect_Error (Area_B, Area_B, Message => "16384 x 1025 pixels");
      end;
      --  Minimum accepted geometry (32 x 32) must reach native code and be valid.
      declare
         Smallest : constant ECC_Translation_Result := Estimate_ECC_Translation
           (EF.Texture (32, 32), EF.Texture (32, 32, 1, 0));
      begin
         Assert (Smallest.Displacement.X > 0.9 and then Smallest.Displacement.X < 1.1
                 and then abs (Smallest.Displacement.Y) < 0.1, "ECC 32x32 minimum geometry");
      end;
   end ECC_Image_Validation;

   procedure ECC_Option_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Template : constant OpenCV.Core.Mat := ECC_Template;
      Moving : constant OpenCV.Core.Mat := EF.Texture (128, 128, 2, 1);
      function From_Bits is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      Bad_Epsilons : constant array (Positive range <>) of OpenCV.Float64_Value :=
        [0.0, -1.0E-4, 9.0E-9, 0.1000001, 1.0,
         From_Bits (16#7FF0_0000_0000_0000#),   --  +Inf
         From_Bits (16#FFF0_0000_0000_0000#),   --  -Inf
         From_Bits (16#7FF8_0000_0000_0000#)];  --  quiet NaN
   begin
      ECC_Expect_Error (Template, Moving, (Maximum_Iterations => 101, Epsilon => 0.0001),
                        "101 iterations");
      ECC_Expect_Error (Template, Moving, (Maximum_Iterations => Positive'Last, Epsilon => 0.0001),
                        "Positive'Last iterations");
      for Epsilon of Bad_Epsilons loop
         ECC_Expect_Error (Template, Moving, (Maximum_Iterations => 50, Epsilon => Epsilon),
                           "invalid epsilon");
      end loop;
      --  Inclusive bounds are accepted and never clamped: one iteration is a valid
      --  request and its (deliberately unconverged) result differs from the default.
      declare
         Lower : constant ECC_Translation_Result := Estimate_ECC_Translation
           (Template, Moving, (Maximum_Iterations => 1, Epsilon => 1.0E-8));
         Default : constant ECC_Translation_Result := Estimate_ECC_Translation (Template, Moving);
      begin
         Assert (Lower.Displacement /= Default.Displacement, "ECC lower option bound was clamped");
      end;
      declare
         Upper : constant ECC_Translation_Result := Estimate_ECC_Translation
           (Template, Moving, (Maximum_Iterations => 100, Epsilon => 0.1));
      begin
         Assert (ECC_Near (Upper, 2, 1), "ECC upper option bounds");
      end;
   end ECC_Option_Validation;

   --  One oracle line: the same-build direct OpenCV outcome for a named fixture.
   type ECC_Oracle_Entry is record
      Name       : String (1 .. 24) := [others => ' '];
      Length     : Natural := 0;
      Iterations : Positive := 1;
      Epsilon    : OpenCV.Float64_Value := 0.0;
      Converged  : Boolean := False;
      DX, DY, Rho : OpenCV.Float64_Value := 0.0;
      Template_Sum, Moving_Sum : Long_Long_Integer := 0;
   end record;

   package ECC_Integers is new Ada.Text_IO.Integer_IO (Integer);
   package ECC_Longs is new Ada.Text_IO.Integer_IO (Long_Long_Integer);
   package ECC_Floats is new Ada.Text_IO.Float_IO (OpenCV.Float64_Value);

   Max_ECC_Oracle_Entries : constant := 64;
   type ECC_Oracle_Table is array (1 .. Max_ECC_Oracle_Entries) of ECC_Oracle_Entry;

   --  Strict parser: a missing, truncated, stale-versioned or malformed file raises
   --  (the test then fails); nothing is silently skipped.
   procedure Load_ECC_Oracle
     (Table : out ECC_Oracle_Table; Count : out Natural; Version : out String; Version_Length : out Natural)
   is
      File : Ada.Text_IO.File_Type;
      Magic : constant String :=
        Ada.Environment_Variables.Value
          ("VIDEO_ECC_TRANSLATION_ORACLE", "../obj/oracle/ecc-translation.txt");

      function Word return String is
         Result : String (1 .. 64);
         Last : Natural := 0;
         Ch : Character;
         Eol : Boolean;
      begin
         loop
            Ada.Text_IO.Look_Ahead (File, Ch, Eol);
            exit when Eol or else Ch /= ' ';
            Ada.Text_IO.Get (File, Ch);
         end loop;
         loop
            Ada.Text_IO.Look_Ahead (File, Ch, Eol);
            exit when Eol or else Ch = ' ';
            Ada.Text_IO.Get (File, Ch);
            Last := Last + 1;
            Result (Last) := Ch;
         end loop;
         return Result (1 .. Last);
      end Word;

      procedure Next_Line is
      begin
         Ada.Text_IO.Skip_Line (File);
      end Next_Line;
   begin
      Count := 0;
      Version := [others => ' '];
      Version_Length := 0;
      Table := [others => <>];
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File, Magic);
      Assert (Word = "ecc-oracle", "ECC oracle magic");
      Assert (Word = "1", "ECC oracle format version");
      Next_Line;
      Assert (Word = "version", "ECC oracle version line");
      declare
         V : constant String := Word;
      begin
         Assert (V'Length <= Version'Length, "ECC oracle version too long");
         Version (Version'First .. Version'First + V'Length - 1) := V;
         Version_Length := V'Length;
      end;
      Next_Line;
      loop
         declare
            Tag : constant String := Word;
         begin
            if Tag = "end" then
               declare
                  Declared : Integer;
               begin
                  ECC_Integers.Get (File, Declared);
                  Assert (Declared = Count, "ECC oracle truncated or inconsistent");
               end;
               exit;
            end if;
            Assert (Tag = "fixture" and then Count < Max_ECC_Oracle_Entries, "ECC oracle fixture line");
            Count := Count + 1;
            declare
               E : ECC_Oracle_Entry renames Table (Count);
               Name : constant String := Word;
            begin
               Assert (Name'Length <= E.Name'Length, "ECC oracle name too long");
               E.Name (1 .. Name'Length) := Name;
               E.Length := Name'Length;
               ECC_Integers.Get (File, E.Iterations);
               ECC_Floats.Get (File, E.Epsilon);
               declare
                  Outcome : constant String := Word;
               begin
                  Assert (Outcome = "OK" or else Outcome = "EXC", "ECC oracle outcome");
                  E.Converged := Outcome = "OK";
               end;
               ECC_Floats.Get (File, E.DX);
               ECC_Floats.Get (File, E.DY);
               ECC_Floats.Get (File, E.Rho);
               ECC_Longs.Get (File, E.Template_Sum);
               ECC_Longs.Get (File, E.Moving_Sum);
            end;
            Next_Line;
         end;
      end loop;
      Ada.Text_IO.Close (File);
   exception
      when others =>
         if Ada.Text_IO.Is_Open (File) then
            Ada.Text_IO.Close (File);
         end if;
         raise;
   end Load_ECC_Oracle;

   type ECC_Pair is record
      Template, Moving : OpenCV.Core.Mat;
   end record;

   --  Rebuilds the oracle's fixture by name. Unknown names fail the test.
   function ECC_Build (Name : String) return ECC_Pair is
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      function Tex (DX, DY : Integer) return OpenCV.Core.Mat is (EF.Texture (128, 128, DX, DY));
      function Pair (DX, DY : Integer) return ECC_Pair is (Template => Tex (0, 0), Moving => Tex (DX, DY));
   begin
      if Name = "identity" then return Pair (0, 0);
      elsif Name = "positive" or else Name = "positive-it1" or else Name = "positive-it2"
        or else Name = "positive-it100-e8" or else Name = "positive-e0.1" then return Pair (2, 1);
      elsif Name = "negative" then return Pair (-2, -1);
      elsif Name = "horizontal" then return Pair (3, 0);
      elsif Name = "vertical" then return Pair (0, 3);
      elsif Name = "contrast-0.8+20" then
         return (Tex (0, 0), EF.Affine (Tex (2, 1), 4, 5, 20));
      elsif Name = "contrast-1.3-10" then
         return (Tex (0, 0), EF.Affine (Tex (2, 1), 13, 10, -10));
      elsif Name = "low-variance" then
         return (EF.Affine (Tex (0, 0), 1, 40, 126), EF.Affine (Tex (2, 1), 1, 40, 126));
      elsif Name = "constant" then
         return (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128));
      elsif Name = "constant-vs-texture" then
         return (EF.Constant_Image (128, 128, 128), Tex (0, 0));
      elsif Name = "noise-vs-noise" then return (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9));
      elsif Name = "noise-vs-texture" then return (Tex (0, 0), EF.Noise (128, 128, 7));
      elsif Name = "stripes-shift3" then return (EF.Stripes (128, 128, 0), EF.Stripes (128, 128, 3));
      elsif Name = "large-12-7" then return Pair (12, 7);
      elsif Name = "large-6-4" then return Pair (6, 4);
      elsif Name = "large-40-30" then return Pair (40, 30);
      elsif Name = "region-both" then
         return (EF.Texture (160, 160).Region (Window), EF.Texture (160, 160, 2, 1).Region (Window));
      elsif Name = "region-template" then
         return (EF.Texture (160, 160).Region (Window), EF.Texture (160, 160, 2, 1).Region (Window).Clone);
      elsif Name = "region-moving" then
         return (EF.Texture (160, 160).Region (Window).Clone, EF.Texture (160, 160, 2, 1).Region (Window));
      elsif Name = "shift42-e0.1" or else Name = "shift42-e1e-8" then return Pair (4, 2);
      elsif Name = "shift43-e0.1" or else Name = "shift43-e1e-8" then return Pair (4, 3);
      end if;
      Assert (False, "unknown ECC oracle fixture " & Name);
      return Pair (0, 0);
   end ECC_Build;

   Expected_ECC_Oracle_Fixtures : constant := 27;

   procedure ECC_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Table : ECC_Oracle_Table;
      Count, Version_Length : Natural;
      Version : String (1 .. 32);
      Largest : OpenCV.Float64_Value := 0.0;
      Succeeded, Raised : Natural := 0;
   begin
      Load_ECC_Oracle (Table, Count, Version, Version_Length);
      Assert (Version_Length > 0, "ECC oracle records no native version");
      Assert (Count = Expected_ECC_Oracle_Fixtures, "ECC oracle fixture count is stale");
      for Index in 1 .. Count loop
         declare
            E : ECC_Oracle_Entry renames Table (Index);
            Name : constant String := E.Name (1 .. E.Length);
            Pair : constant ECC_Pair := ECC_Build (Name);
            Options : constant ECC_Translation_Options :=
              (Maximum_Iterations => E.Iterations, Epsilon => E.Epsilon);
         begin
            Assert (EF.Checksum (Pair.Template) = E.Template_Sum
                    and then EF.Checksum (Pair.Moving) = E.Moving_Sum,
                    "ECC oracle fixture " & Name & " differs from the Ada fixture (stale oracle)");
            if E.Converged then
               declare
                  R : constant ECC_Translation_Result :=
                    Estimate_ECC_Translation (Pair.Template, Pair.Moving, Options);
                  EX : constant OpenCV.Float64_Value := abs (OpenCV.Float64_Value (R.Displacement.X) - E.DX);
                  EY : constant OpenCV.Float64_Value := abs (OpenCV.Float64_Value (R.Displacement.Y) - E.DY);
                  ER : constant OpenCV.Float64_Value := abs (R.Correlation - E.Rho);
               begin
                  Succeeded := Succeeded + 1;
                  Largest := OpenCV.Float64_Value'Max (Largest, OpenCV.Float64_Value'Max (EX, OpenCV.Float64_Value'Max (EY, ER)));
                  Assert (EX <= 1.0E-5 and then EY <= 1.0E-5 and then ER <= 1.0E-6,
                          "ECC binding differs from direct native for " & Name);
               end;
            else
               begin
                  declare
                     R : constant ECC_Translation_Result :=
                       Estimate_ECC_Translation (Pair.Template, Pair.Moving, Options);
                     pragma Unreferenced (R);
                  begin
                     Assert (False, "ECC binding succeeded where native raised: " & Name);
                  end;
               exception
                  when OpenCV.OpenCV_Error => Raised := Raised + 1;
               end;
            end if;
         end;
      end loop;
      Assert (Succeeded > 0 and then Raised > 0, "ECC oracle did not cover both outcomes");
      Assert (Succeeded + Raised = Count, "ECC oracle outcome accounting");
   end ECC_Direct_Oracle;

   procedure ECC_Degenerate_And_Nonconvergence (T : in out Fixture) is
      pragma Unreferenced (T);
      Flat : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
      Texture_Image : constant OpenCV.Core.Mat := ECC_Template;
   begin
      --  A constant image has zero variance; native ECC cannot normalize and raises.
      ECC_Expect_Error (Flat, Flat, Message => "constant images");
      ECC_Expect_Error (Flat, Texture_Image, Message => "constant template");
      ECC_Expect_Error (Texture_Image, Flat, Message => "constant moving");
      --  Outcome for noise is version-defined: compare two runs, not a fixed claim.
      declare
         A : constant OpenCV.Core.Mat := EF.Noise (128, 128, 3);
         B : constant OpenCV.Core.Mat := EF.Noise (128, 128, 9);
         First, Second : ECC_Translation_Result;
         First_Raised, Second_Raised : Boolean := False;
      begin
         begin First := Estimate_ECC_Translation (A, B); exception
            when OpenCV.OpenCV_Error => First_Raised := True; end;
         begin Second := Estimate_ECC_Translation (A, B); exception
            when OpenCV.OpenCV_Error => Second_Raised := True; end;
         Assert (First_Raised = Second_Raised and then (First_Raised or else First = Second),
                 "ECC noise outcome is not repeatable");
      end;
      --  A finite result is not alignment proof: stripes carry no vertical information.
      declare
         R : constant ECC_Translation_Result :=
           Estimate_ECC_Translation (EF.Stripes (128, 128, 0), EF.Stripes (128, 128, 0));
      begin
         Assert (abs (R.Displacement.X) < 1.0E-3 and then abs (R.Displacement.Y) < 1.0E-3,
                 "ECC identical stripes");
      end;
   end ECC_Degenerate_And_Nonconvergence;

   procedure ECC_Aspect_Ratios (T : in out Fixture) is
      pragma Unreferenced (T);
      type Geometry is record Rows, Columns : Positive; end record;
      Shapes : constant array (1 .. 6) of Geometry :=
        [(64, 256), (256, 64), (96, 96), (32, 512), (32, 16_384), (16_384, 32)];
   begin
      for Shape of Shapes loop
         declare
            R : constant ECC_Translation_Result := Estimate_ECC_Translation
              (EF.Texture (Shape.Rows, Shape.Columns), EF.Texture (Shape.Rows, Shape.Columns, 2, 1));
         begin
            Assert (abs (R.Displacement.X - 2.0) < 0.1 and then abs (R.Displacement.Y - 1.0) < 0.1,
                    "ECC aspect ratio displacement (incl. 32 x 16384 and 16384 x 32 limits)");
         end;
      end loop;
   end ECC_Aspect_Ratios;

   ---------------------------------------------------------------------
   --  Task 012: ECC Euclidean (translation + rotation)
   ---------------------------------------------------------------------
   package LM renames Ada.Numerics.Long_Elementary_Functions;

   Euc_Angle_Tolerance       : constant Long_Float := 0.005;
   Euc_Translation_Tolerance : constant Long_Float := 0.1;
   Degrees_To_Radians        : constant Long_Float := 3.14159265358979323846 / 180.0;

   function LF (Value : OpenCV.Float32_Value) return Long_Float is (Long_Float (Value));

   function Euc_Pair (Degrees, TX, TY : Long_Float) return ECC_Pair is
     (Template => EF.Texture (128, 128),
      Moving   => EF.Rigid_Texture (128, 128, Degrees, TX, TY));

   function Euc_Near
     (R : ECC_Euclidean_Result; Degrees, TX, TY : Long_Float) return Boolean is
     (abs (Long_Float (R.Rotation_Radians) - Degrees * Degrees_To_Radians) < Euc_Angle_Tolerance
      and then abs (LF (R.Displacement.X) - TX) < Euc_Translation_Tolerance
      and then abs (LF (R.Displacement.Y) - TY) < Euc_Translation_Tolerance);

   --  Applies the returned matrix: template coordinates to moving coordinates.
   procedure Euc_Map
     (R : ECC_Euclidean_Result; X, Y : Long_Float; MX, MY : out Long_Float)
   is
      Theta : constant Long_Float := Long_Float (R.Rotation_Radians);
   begin
      MX := LM.Cos (Theta) * X - LM.Sin (Theta) * Y + LF (R.Displacement.X);
      MY := LM.Sin (Theta) * X + LM.Cos (Theta) * Y + LF (R.Displacement.Y);
   end Euc_Map;

   function Sample (Image : OpenCV.Core.Mat; X, Y : Long_Float) return Long_Float is
      X0 : constant Integer := Integer (Long_Float'Floor (X));
      Y0 : constant Integer := Integer (Long_Float'Floor (Y));
      FX : constant Long_Float := X - Long_Float (X0);
      FY : constant Long_Float := Y - Long_Float (Y0);
      function P (RX, RY : Integer) return Long_Float is
        (Long_Float (OpenCV.Core.UInt8_Access.Get (Image, RY, RX)));
   begin
      return (1.0 - FY) * ((1.0 - FX) * P (X0, Y0) + FX * P (X0 + 1, Y0))
        + FY * ((1.0 - FX) * P (X0, Y0 + 1) + FX * P (X0 + 1, Y0 + 1));
   end Sample;

   --  Mean absolute difference over an interior window. Aligned = True resamples the
   --  moving image at W * p, which is what WARP_INVERSE_MAP does with the returned matrix.
   function Euc_Residual
     (Pair : ECC_Pair; R : ECC_Euclidean_Result; Aligned : Boolean) return Long_Float
   is
      Margin : constant := 24;
      Sum    : Long_Float := 0.0;
      Count  : Natural := 0;
      MX, MY : Long_Float;
   begin
      for Y in Margin .. 127 - Margin loop
         for X in Margin .. 127 - Margin loop
            if Aligned then
               Euc_Map (R, Long_Float (X), Long_Float (Y), MX, MY);
            else
               MX := Long_Float (X);
               MY := Long_Float (Y);
            end if;
            Sum := Sum + abs (Sample (Pair.Moving, MX, MY)
                              - Long_Float (OpenCV.Core.UInt8_Access.Get (Pair.Template, Y, X)));
            Count := Count + 1;
         end loop;
      end loop;
      return Sum / Long_Float (Count);
   end Euc_Residual;

   procedure Euc_Expect_Error
     (Template, Moving : OpenCV.Core.Mat;
      Options : ECC_Euclidean_Options := (others => <>);
      Message : String)
   is
      Unused : ECC_Euclidean_Result;
   begin
      Unused := Estimate_ECC_Euclidean (Template, Moving, Options);
      Assert (False, Message & ": request accepted");
   exception
      when OpenCV.OpenCV_Error => null;
   end Euc_Expect_Error;

   --  A shim-originated rejection carries the "Video.Estimate_ECC_Euclidean" prefix.
   procedure Euc_Expect_Preflight_Error (Template, Moving : OpenCV.Core.Mat; Message : String) is
      Unused : ECC_Euclidean_Result;
   begin
      Unused := Estimate_ECC_Euclidean (Template, Moving);
      Assert (False, Message & ": request accepted");
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text   : constant String := Ada.Exceptions.Exception_Message (Error);
            Marker : constant String := "Video.Estimate_ECC_Euclidean";
         begin
            Assert (Text'Length < Marker'Length
                    or else Text (Text'First .. Text'First + Marker'Length - 1) /= Marker,
                    Message & ": rejected by the native layer, not the Ada preflight");
         end;
   end Euc_Expect_Preflight_Error;

   procedure ECC_Euclidean_Defaults (T : in out Fixture) is
      pragma Unreferenced (T);
      Options     : constant ECC_Euclidean_Options := (others => <>);
      Translation : constant ECC_Translation_Options := Options;
      Zero        : constant ECC_Euclidean_Result := (others => <>);
   begin
      Assert (Options.Maximum_Iterations = 50 and then Options.Epsilon = 0.0001
              and then Translation = Options, "ECC Euclidean options reuse the translation defaults");
      Assert (Zero.Displacement = (X => 0.0, Y => 0.0) and then Zero.Rotation_Radians = 0.0
              and then Zero.Correlation = 0.0, "ECC Euclidean default result");
   end ECC_Euclidean_Defaults;

   procedure ECC_Euclidean_Identity_And_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Base      : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Before    : constant Long_Long_Integer := EF.Checksum (Base);
      Identity  : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Base, Base);
      Pair      : constant ECC_Pair := Euc_Pair (0.0, 2.0, 1.0);
      Shifted   : constant ECC_Euclidean_Result :=
        Estimate_ECC_Euclidean (Pair.Template, Pair.Moving);
      Reference : constant ECC_Translation_Result :=
        Estimate_ECC_Translation (Pair.Template, Pair.Moving);
   begin
      Assert (abs (Identity.Rotation_Radians) < 1.0E-6 and then abs (Identity.Displacement.X) < 1.0E-5
              and then abs (Identity.Displacement.Y) < 1.0E-5
              and then abs (Identity.Correlation - 1.0) < 1.0E-6, "ECC Euclidean identity");
      Assert (Euc_Near (Shifted, 0.0, 2.0, 1.0), "ECC Euclidean translation without rotation");
      Assert (abs (Shifted.Rotation_Radians) < 0.002, "ECC Euclidean translation introduced rotation");
      --  The retained translation-only API is unchanged and agrees on a pure shift.
      Assert (ECC_Near (Reference, 2, 1), "translation-only regression");
      Assert (abs (LF (Reference.Displacement.X) - LF (Shifted.Displacement.X)) < 0.05
              and then abs (LF (Reference.Displacement.Y) - LF (Shifted.Displacement.Y)) < 0.05,
              "Euclidean and translation models disagree on a pure shift");
      Assert (EF.Checksum (Base) = Before, "ECC Euclidean modified an input");
   end ECC_Euclidean_Identity_And_Translation;

   procedure ECC_Euclidean_Signed_Rotations (T : in out Fixture) is
      pragma Unreferenced (T);
      Template : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Angles   : constant array (1 .. 4) of Long_Float := [1.0, -1.0, 3.0, -3.0];
   begin
      for Degrees of Angles loop
         declare
            Moving   : constant OpenCV.Core.Mat := EF.Rigid_Texture (128, 128, Degrees, 0.0, 0.0);
            Forward  : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Template, Moving);
            Backward : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Moving, Template);
         begin
            Assert (Euc_Near (Forward, Degrees, 0.0, 0.0), "ECC signed rotation");
            --  Radians, not degrees: 3 degrees is about 0.0524, never 3.0.
            Assert (abs (Long_Float (Forward.Rotation_Radians)) < 0.06, "ECC angle is not in radians");
            Assert ((Forward.Rotation_Radians > 0.0) = (Degrees > 0.0), "ECC rotation sign");
            Assert (abs (Long_Float (Backward.Rotation_Radians) + Degrees * Degrees_To_Radians)
                    < Euc_Angle_Tolerance, "ECC reversed roles are not negated");
            Assert (Forward.Correlation > 0.98 and then Forward.Correlation <= 1.000001,
                    "ECC rotation correlation range");
         end;
      end loop;
   end ECC_Euclidean_Signed_Rotations;

   procedure ECC_Euclidean_Rotation_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      type Case_Record is record Degrees, TX, TY : Long_Float; end record;
      Cases : constant array (1 .. 4) of Case_Record :=
        [(3.0, 2.0, 1.0), (3.0, -2.0, -1.0), (-3.0, 2.0, -1.0), (-3.0, -2.0, 1.0)];
   begin
      for Item of Cases loop
         declare
            Pair : constant ECC_Pair := Euc_Pair (Item.Degrees, Item.TX, Item.TY);
            R    : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving);
            Swapped_Axes : constant Boolean :=
              abs (LF (R.Displacement.X) - Item.TY) < Euc_Translation_Tolerance
              and then abs (LF (R.Displacement.Y) - Item.TX) < Euc_Translation_Tolerance;
         begin
            Assert (Euc_Near (R, Item.Degrees, Item.TX, Item.TY), "ECC rotation plus translation");
            Assert (not Swapped_Axes, "ECC translation axes swapped");
         end;
      end loop;
      declare
         Pair   : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
         Dimmer : constant OpenCV.Core.Mat := EF.Affine (Pair.Moving, 4, 5, 20);
         Result : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Pair.Template, Dimmer);
      begin
         Assert (Euc_Near (Result, 3.0, 2.0, 1.0), "ECC Euclidean photometric variation");
      end;
   end ECC_Euclidean_Rotation_Translation;

   procedure ECC_Euclidean_Centered_Rotation (T : in out Fixture) is
      pragma Unreferenced (T);
      Theta : constant Long_Float := 3.0 * Degrees_To_Radians;
      CX    : constant Long_Float := 63.5;
      CY    : constant Long_Float := 63.5;
      DX    : constant Long_Float := 1.5;
      DY    : constant Long_Float := -1.0;
      TX    : constant Long_Float := CX - (LM.Cos (Theta) * CX - LM.Sin (Theta) * CY) + DX;
      TY    : constant Long_Float := CY - (LM.Sin (Theta) * CX + LM.Cos (Theta) * CY) + DY;
      Pair  : constant ECC_Pair := Euc_Pair (3.0, TX, TY);
      R     : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving);
      MX, MY : Long_Float;
   begin
      --  The returned translation belongs to the origin-rotation matrix, not to the centre.
      Assert (Euc_Near (R, 3.0, TX, TY), "ECC centred rotation matrix translation");
      Assert (abs (LF (R.Displacement.X) - DX) > 1.0 and then abs (LF (R.Displacement.Y) - DY) > 1.0,
              "ECC Displacement was interpreted as the centre displacement");
      --  Independently transformed points: the centre moves by (DX, DY); an off-centre
      --  point lands where the analytic matrix sends it.
      Euc_Map (R, CX, CY, MX, MY);
      Assert (abs (MX - (CX + DX)) < 0.15 and then abs (MY - (CY + DY)) < 0.15,
              "ECC centre point mapping");
      Euc_Map (R, 10.0, 100.0, MX, MY);
      Assert (abs (MX - (LM.Cos (Theta) * 10.0 - LM.Sin (Theta) * 100.0 + TX)) < 0.15
              and then abs (MY - (LM.Sin (Theta) * 10.0 + LM.Cos (Theta) * 100.0 + TY)) < 0.15,
              "ECC off-centre point mapping");
   end ECC_Euclidean_Centered_Rotation;

   procedure ECC_Euclidean_Inverse_Warp (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair    : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      R       : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving);
      Wrong   : constant ECC_Euclidean_Result :=
        (Displacement     => (X => -R.Displacement.X, Y => -R.Displacement.Y),
         Rotation_Radians => -R.Rotation_Radians,
         Correlation      => R.Correlation);
      Before  : constant Long_Float := Euc_Residual (Pair, R, Aligned => False);
      After   : constant Long_Float := Euc_Residual (Pair, R, Aligned => True);
      Negated : constant Long_Float := Euc_Residual (Pair, Wrong, Aligned => True);
   begin
      Assert (Before > 10.0, "ECC fixture is not misaligned before alignment");
      Assert (After < 2.0 and then After < Before / 8.0, "ECC inverse-map residual did not improve");
      Assert (Negated > 4.0 * After and then Negated > 5.0,
              "ECC negated matrix aligned as well as the returned matrix");
   end ECC_Euclidean_Inverse_Warp;

   procedure ECC_Euclidean_Options_Sensitivity (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      Default_1 : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving);
      Default_2 : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving);
      One : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 1, Epsilon => 0.0001));
      Two : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 2, Epsilon => 0.0001));
      Tight : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 50, Epsilon => 1.0E-8));
      Loose : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 50, Epsilon => 0.1));
   begin
      Assert (Default_1 = Default_2, "ECC Euclidean repeated calls differ");
      Assert (One /= Default_1 and then One.Correlation < Default_1.Correlation - 0.1,
              "ECC Euclidean iteration option ignored");
      Assert (Euc_Near (Two, 3.0, 2.0, 1.0) and then Euc_Near (Tight, 3.0, 2.0, 1.0)
              and then Euc_Near (Loose, 3.0, 2.0, 1.0), "ECC Euclidean nondefault options lost accuracy");
      Assert (Tight /= Loose, "ECC Euclidean epsilon option ignored");
   end ECC_Euclidean_Options_Sensitivity;

   procedure ECC_Euclidean_Regions_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Saved, Contiguous : ECC_Euclidean_Result;
      Before_A, Before_B : Long_Long_Integer;
   begin
      declare
         Parent_A : constant OpenCV.Core.Mat := EF.Texture (160, 160);
         Parent_B : constant OpenCV.Core.Mat := EF.Rigid_Texture (160, 160, 3.0, 2.0, 1.0);
         Region_A : constant OpenCV.Core.Mat := Parent_A.Region (Window);
         Region_B : constant OpenCV.Core.Mat := Parent_B.Region (Window);
      begin
         Before_A := EF.Checksum (Parent_A);
         Before_B := EF.Checksum (Parent_B);
         Assert (not Region_A.Is_Continuous and then not Region_B.Is_Continuous,
                 "ECC Euclidean fixture Regions are contiguous");
         Saved := Estimate_ECC_Euclidean (Region_A, Region_B);
         Contiguous := Estimate_ECC_Euclidean (Region_A.Clone, Region_B.Clone);
         Assert (Saved = Contiguous, "ECC Euclidean Region differs from the contiguous clone");
         Assert (EF.Checksum (Parent_A) = Before_A and then EF.Checksum (Parent_B) = Before_B,
                 "ECC Euclidean modified a Region parent");
      end;
      --  Parents and Regions are finalized; the result is a plain value.
      Assert (abs (Long_Float (Saved.Rotation_Radians) - 3.0 * Degrees_To_Radians) < Euc_Angle_Tolerance
              and then Saved.Correlation > 0.98, "ECC Euclidean result after finalization");
   end ECC_Euclidean_Regions_Lifetime;

   procedure ECC_Euclidean_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Good : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Empty : OpenCV.Core.Mat;
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.UInt8, Channels => 3));
      function From_Bits is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      Bad_Epsilons : constant array (Positive range <>) of OpenCV.Float64_Value :=
        [0.0, -1.0E-4, 9.0E-9, 0.1000001, 1.0,
         From_Bits (16#7FF0_0000_0000_0000#), From_Bits (16#FFF0_0000_0000_0000#),
         From_Bits (16#7FF8_0000_0000_0000#)];
   begin
      Euc_Expect_Error (Empty, Good, Message => "empty template");
      Euc_Expect_Error (Good, Empty, Message => "empty moving");
      Euc_Expect_Preflight_Error (Float_Image, Float_Image, "Float32 images");
      Euc_Expect_Preflight_Error (Color, Color, "three-channel images");
      Euc_Expect_Preflight_Error (Good, EF.Texture (128, 129), "different columns");
      Euc_Expect_Preflight_Error (EF.Texture (129, 128), Good, "different rows");
      Euc_Expect_Error (Good, Float_Image, Message => "Float32 moving");
      Euc_Expect_Error (EF.Texture (31, 128), EF.Texture (31, 128), Message => "31 rows");
      Euc_Expect_Error (EF.Texture (128, 31), EF.Texture (128, 31), Message => "31 columns");
      declare
         Wide : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (32, 16_385, (Depth => OpenCV.Core.UInt8, Channels => 1));
         Area : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (4_097, 4_096, (Depth => OpenCV.Core.UInt8, Channels => 1));
      begin
         --  Rejected before native work: no ECC temporaries are allocated.
         Euc_Expect_Preflight_Error (Wide, Wide, "16385 columns");
         Euc_Expect_Preflight_Error (Area, Area, "4097 x 4096 pixels");
      end;
      Euc_Expect_Error (Good, Good, (Maximum_Iterations => 101, Epsilon => 0.0001),
                        Message => "101 iterations");
      Euc_Expect_Error (Good, Good, (Maximum_Iterations => Positive'Last, Epsilon => 0.0001),
                        Message => "Positive'Last iterations");
      for Epsilon of Bad_Epsilons loop
         Euc_Expect_Error (Good, Good, (Maximum_Iterations => 50, Epsilon => Epsilon),
                           Message => "invalid epsilon");
      end loop;
      declare
         Pair : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
         Lower : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean
           (Pair.Template, Pair.Moving, (Maximum_Iterations => 1, Epsilon => 1.0E-8));
         Upper : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean
           (Pair.Template, Pair.Moving, (Maximum_Iterations => 100, Epsilon => 0.1));
      begin
         Assert (Lower /= Estimate_ECC_Euclidean (Pair.Template, Pair.Moving), "lower option bound clamped");
         Assert (Euc_Near (Upper, 3.0, 2.0, 1.0), "ECC Euclidean upper option bounds");
      end;
   end ECC_Euclidean_Validation;

   procedure ECC_Euclidean_Degenerate_And_Limits (T : in out Fixture) is
      pragma Unreferenced (T);
      Flat    : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
      Texture_Image : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Outside : constant OpenCV.Core.Mat := EF.Rigid_Texture (128, 128, 25.0, 0.0, 0.0);
      Noise   : constant OpenCV.Core.Mat := EF.Noise (128, 128, 7);
   begin
      Euc_Expect_Error (Flat, Flat, Message => "constant images");
      Euc_Expect_Error (Flat, Texture_Image, Message => "constant template");
      Euc_Expect_Error (Texture_Image, Flat, Message => "constant moving");
      Euc_Expect_Error (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9), Message => "uncorrelated noise");
      --  25 degrees lies outside the reliable basin: native ECC throws and no identity is
      --  fabricated. (10 and 15 degrees are version-dependent, so they are not asserted.)
      Euc_Expect_Error (Texture_Image, Outside, Message => "rotation outside the convergence basin");
      --  Finite is not correct: independent noise against a texture may be accepted with a
      --  low correlation and a wrong transform, or rejected; either outcome is repeatable.
      begin
         declare
            R : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Texture_Image, Noise);
         begin
            Assert (R.Correlation < 0.2, "ECC noise vs texture correlation");
            Assert (R = Estimate_ECC_Euclidean (Texture_Image, Noise), "ECC noise result not repeatable");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end ECC_Euclidean_Degenerate_And_Limits;

   --  One Euclidean oracle line: same-build direct OpenCV outcome for a named fixture.
   type Euc_Warp is array (0 .. 5) of Long_Float;
   type Euc_Oracle_Entry is record
      Name         : String (1 .. 24) := [others => ' '];
      Length       : Natural := 0;
      Iterations   : Positive := 1;
      Epsilon      : Long_Float := 0.0;
      Converged    : Boolean := False;
      W            : Euc_Warp := [others => 0.0];
      TX, TY, Theta, Rho : Long_Float := 0.0;
      Template_Sum, Moving_Sum : Long_Long_Integer := 0;
   end record;

   package Euc_Floats is new Ada.Text_IO.Float_IO (Long_Float);

   type Euc_Oracle_Table is array (1 .. 64) of Euc_Oracle_Entry;

   --  Strict parser: a missing, truncated, stale-versioned or malformed file raises.
   procedure Load_Euc_Oracle
     (Table : out Euc_Oracle_Table; Count : out Natural;
      Version : out String; Version_Length : out Natural)
   is
      File : Ada.Text_IO.File_Type;
      Path : constant String :=
        Ada.Environment_Variables.Value
          ("VIDEO_ECC_EUCLIDEAN_ORACLE", "../obj/oracle/ecc-euclidean.txt");

      function Word return String is
         Result : String (1 .. 64);
         Last   : Natural := 0;
         Ch     : Character;
         Eol    : Boolean;
      begin
         loop
            Ada.Text_IO.Look_Ahead (File, Ch, Eol);
            exit when Eol or else Ch /= ' ';
            Ada.Text_IO.Get (File, Ch);
         end loop;
         loop
            Ada.Text_IO.Look_Ahead (File, Ch, Eol);
            exit when Eol or else Ch = ' ';
            Ada.Text_IO.Get (File, Ch);
            Last := Last + 1;
            Result (Last) := Ch;
         end loop;
         return Result (1 .. Last);
      end Word;

      procedure Get_Float (Value : out Long_Float) is
      begin
         Euc_Floats.Get (File, Value);
      end Get_Float;
   begin
      Count := 0;
      Version := [others => ' '];
      Version_Length := 0;
      Table := [others => <>];
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File, Path);
      Assert (Word = "ecc-euclidean-oracle", "Euclidean oracle magic");
      Assert (Word = "1", "Euclidean oracle format version");
      Ada.Text_IO.Skip_Line (File);
      Assert (Word = "version", "Euclidean oracle version line");
      declare
         V : constant String := Word;
      begin
         Assert (V'Length <= Version'Length, "Euclidean oracle version too long");
         Version (Version'First .. Version'First + V'Length - 1) := V;
         Version_Length := V'Length;
      end;
      Ada.Text_IO.Skip_Line (File);
      loop
         declare
            Tag : constant String := Word;
         begin
            if Tag = "end" then
               declare
                  Declared : Integer;
               begin
                  ECC_Integers.Get (File, Declared);
                  Assert (Declared = Count, "Euclidean oracle truncated or inconsistent");
               end;
               exit;
            end if;
            Assert (Tag = "fixture" and then Count < Table'Last, "Euclidean oracle fixture line");
            Count := Count + 1;
            declare
               E    : Euc_Oracle_Entry renames Table (Count);
               Name : constant String := Word;
            begin
               Assert (Name'Length <= E.Name'Length, "Euclidean oracle name too long");
               E.Name (1 .. Name'Length) := Name;
               E.Length := Name'Length;
               ECC_Integers.Get (File, E.Iterations);
               Get_Float (E.Epsilon);
               declare
                  Outcome : constant String := Word;
               begin
                  Assert (Outcome = "OK" or else Outcome = "EXC", "Euclidean oracle outcome");
                  E.Converged := Outcome = "OK";
               end;
               for I in E.W'Range loop
                  Get_Float (E.W (I));
               end loop;
               Get_Float (E.TX);
               Get_Float (E.TY);
               Get_Float (E.Theta);
               Get_Float (E.Rho);
               ECC_Longs.Get (File, E.Template_Sum);
               ECC_Longs.Get (File, E.Moving_Sum);
            end;
            Ada.Text_IO.Skip_Line (File);
         end;
      end loop;
      Ada.Text_IO.Close (File);
   exception
      when others =>
         if Ada.Text_IO.Is_Open (File) then
            Ada.Text_IO.Close (File);
         end if;
         raise;
   end Load_Euc_Oracle;

   --  Rebuilds the Euclidean oracle's fixtures by name (independently of the C++ code).
   function Euc_Build (Name : String) return ECC_Pair is
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Theta3 : constant Long_Float := 3.0 * Degrees_To_Radians;
      Centre_TX : constant Long_Float :=
        63.5 - (LM.Cos (Theta3) * 63.5 - LM.Sin (Theta3) * 63.5) + 1.5;
      Centre_TY : constant Long_Float :=
        63.5 - (LM.Sin (Theta3) * 63.5 + LM.Cos (Theta3) * 63.5) - 1.0;
      function Tex return OpenCV.Core.Mat is (EF.Texture (128, 128));
      function Rigid (D, X, Y : Long_Float) return OpenCV.Core.Mat is
        (EF.Rigid_Texture (128, 128, D, X, Y));
      function P (D, X, Y : Long_Float) return ECC_Pair is (Tex, Rigid (D, X, Y));
   begin
      if Name = "identity" then return (Tex, Tex);
      elsif Name = "translation-2-1" then return P (0.0, 2.0, 1.0);
      elsif Name = "rot+1" then return P (1.0, 0.0, 0.0);
      elsif Name = "rot-1" then return P (-1.0, 0.0, 0.0);
      elsif Name = "rot+3" then return P (3.0, 0.0, 0.0);
      elsif Name = "rot-3" then return P (-3.0, 0.0, 0.0);
      elsif Name = "rot+3-t+2+1" then return P (3.0, 2.0, 1.0);
      elsif Name = "rot+3-t-2-1" then return P (3.0, -2.0, -1.0);
      elsif Name = "rot-3-t+2-1" then return P (-3.0, 2.0, -1.0);
      elsif Name = "centred-rot+3" then return P (3.0, Centre_TX, Centre_TY);
      elsif Name = "reversed-rot+3" then return (Rigid (3.0, 2.0, 1.0), Tex);
      elsif Name = "photometric" then return (Tex, EF.Affine (Rigid (3.0, 2.0, 1.0), 4, 5, 20));
      elsif Name = "constant" then
         return (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128));
      elsif Name = "noise-vs-texture" then return (Tex, EF.Noise (128, 128, 7));
      elsif Name = "noise-vs-noise" then return (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9));
      elsif Name = "rot+10-limit" then return P (10.0, 0.0, 0.0);
      elsif Name = "rot+15-limit" then return P (15.0, 0.0, 0.0);
      elsif Name = "rot+25-outside" then return P (25.0, 0.0, 0.0);
      elsif Name = "it1" or else Name = "it2" or else Name = "it100-e8" or else Name = "e0.1"
        or else Name = "e1e-8" then return P (3.0, 2.0, 1.0);
      elsif Name = "region-both" then
         return (EF.Texture (160, 160).Region (Window),
                 EF.Rigid_Texture (160, 160, 3.0, 2.0, 1.0).Region (Window));
      end if;
      Assert (False, "unknown Euclidean oracle fixture " & Name);
      return (Tex, Tex);
   end Euc_Build;

   Expected_Euc_Oracle_Fixtures : constant := 24;

   --  Absolute angle difference modulo 2 Pi, so +-Pi boundary values compare equal.
   function Angle_Difference (A, B : Long_Float) return Long_Float is
      Pi     : constant Long_Float := 3.14159265358979323846;
      D      : Long_Float := A - B;
   begin
      while D > Pi loop D := D - 2.0 * Pi; end loop;
      while D < -Pi loop D := D + 2.0 * Pi; end loop;
      return abs D;
   end Angle_Difference;

   procedure ECC_Euclidean_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Table : Euc_Oracle_Table;
      Count, Version_Length : Natural;
      Version : String (1 .. 32);
      Succeeded, Raised : Natural := 0;
   begin
      Load_Euc_Oracle (Table, Count, Version, Version_Length);
      Assert (Version_Length > 0, "Euclidean oracle records no native version");
      Assert (Count = Expected_Euc_Oracle_Fixtures, "Euclidean oracle fixture count is stale");
      for Index in 1 .. Count loop
         declare
            E    : Euc_Oracle_Entry renames Table (Index);
            Name : constant String := E.Name (1 .. E.Length);
            Pair : constant ECC_Pair := Euc_Build (Name);
            Options : constant ECC_Euclidean_Options :=
              (Maximum_Iterations => E.Iterations, Epsilon => OpenCV.Float64_Value (E.Epsilon));
         begin
            Assert (EF.Checksum (Pair.Template) = E.Template_Sum
                    and then EF.Checksum (Pair.Moving) = E.Moving_Sum,
                    "Euclidean oracle fixture " & Name & " differs from the Ada fixture (stale oracle)");
            if E.Converged then
               declare
                  R : constant ECC_Euclidean_Result :=
                    Estimate_ECC_Euclidean (Pair.Template, Pair.Moving, Options);
                  Theta : constant Long_Float := Long_Float (R.Rotation_Radians);
                  C : constant Long_Float := LM.Cos (Theta);
                  S : constant Long_Float := LM.Sin (Theta);
               begin
                  Succeeded := Succeeded + 1;
                  --  Reconstruct the full 2 x 3 matrix from the returned value.
                  Assert (abs (C - E.W (0)) < 1.0E-6 and then abs (-S - E.W (1)) < 1.0E-6
                          and then abs (S - E.W (3)) < 1.0E-6 and then abs (C - E.W (4)) < 1.0E-6
                          and then abs (LF (R.Displacement.X) - E.W (2)) < 1.0E-5
                          and then abs (LF (R.Displacement.Y) - E.W (5)) < 1.0E-5,
                          "Euclidean matrix reconstruction differs from the native warp for " & Name);
                  Assert (Angle_Difference (Theta, E.Theta) < 1.0E-6,
                          "Euclidean signed angle differs from native for " & Name);
                  Assert (abs (LF (R.Displacement.X) - E.TX) < 1.0E-5
                          and then abs (LF (R.Displacement.Y) - E.TY) < 1.0E-5
                          and then abs (Long_Float (R.Correlation) - E.Rho) < 1.0E-6,
                          "Euclidean translation/correlation differs from native for " & Name);
               end;
            else
               begin
                  declare
                     R : constant ECC_Euclidean_Result :=
                       Estimate_ECC_Euclidean (Pair.Template, Pair.Moving, Options);
                     pragma Unreferenced (R);
                  begin
                     Assert (False, "Euclidean binding succeeded where native raised: " & Name);
                  end;
               exception
                  when OpenCV.OpenCV_Error => Raised := Raised + 1;
               end;
            end if;
         end;
      end loop;
      Assert (Succeeded > 0 and then Raised > 0, "Euclidean oracle did not cover both outcomes");
      Assert (Succeeded + Raised = Count, "Euclidean oracle outcome accounting");
   end ECC_Euclidean_Direct_Oracle;

   ---------------------------------------------------------------------
   --  Task 013: caller-seeded ECC Euclidean alignment
   ---------------------------------------------------------------------
   function Seed
     (Degrees, TX, TY : Long_Float; Correlation : Long_Float := 0.0) return ECC_Euclidean_Result is
     (Displacement     => (X => OpenCV.Float32_Value (TX), Y => OpenCV.Float32_Value (TY)),
      Rotation_Radians => OpenCV.Float64_Value (Degrees * Degrees_To_Radians),
      Correlation      => OpenCV.Float64_Value (Correlation));

   procedure Run_Seeded
     (Pair : ECC_Pair; Initial : ECC_Euclidean_Result;
      Result : out ECC_Euclidean_Result; Raised : out Boolean;
      Options : ECC_Euclidean_Options := (others => <>)) is
   begin
      Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving, Options, Initial);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Run_Seeded;

   procedure Run_Unseeded
     (Pair : ECC_Pair; Result : out ECC_Euclidean_Result; Raised : out Boolean;
      Options : ECC_Euclidean_Options := (others => <>)) is
   begin
      Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving, Options);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Run_Unseeded;

   type Seed_Outcome is (Accepted, Preflight_Rejected, Native_Failed);

   --  Native-layer failures carry the "Video.Estimate_ECC_Euclidean" prefix (C.Check);
   --  Ada preflight rejections do not.
   function Seed_Outcome_Of
     (Pair : ECC_Pair; Initial : ECC_Euclidean_Result;
      Options : ECC_Euclidean_Options := (others => <>)) return Seed_Outcome
   is
      Marker : constant String := "Video.Estimate_ECC_Euclidean";
      Unused : ECC_Euclidean_Result;
   begin
      Unused := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving, Options, Initial);
      return Accepted;
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text : constant String := Ada.Exceptions.Exception_Message (Error);
         begin
            return (if Text'Length >= Marker'Length
                      and then Text (Text'First .. Text'First + Marker'Length - 1) = Marker
                    then Native_Failed else Preflight_Rejected);
         end;
   end Seed_Outcome_Of;

   procedure ECC_Seeded_Zero_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      type Zero_Case is record Degrees, TX, TY : Long_Float; end record;
      Cases : constant array (1 .. 7) of Zero_Case :=
        [(0.0, 0.0, 0.0), (0.0, 2.0, 1.0), (3.0, 0.0, 0.0), (3.0, 2.0, 1.0),
         (-3.0, 2.0, -1.0), (10.0, 2.0, 1.0), (12.0, 0.0, 0.0)];
      Zero : constant ECC_Euclidean_Result := (others => <>);
      Failures, Successes : Natural := 0;
   begin
      for Item of Cases loop
         declare
            Pair : constant ECC_Pair := Euc_Pair (Item.Degrees, Item.TX, Item.TY);
            Plain, Seeded, Defaulted : ECC_Euclidean_Result;
            Plain_Raised, Seeded_Raised, Defaulted_Raised : Boolean;
         begin
            Run_Unseeded (Pair, Plain, Plain_Raised);
            Run_Seeded (Pair, Zero, Seeded, Seeded_Raised);
            --  Options omitted: the defaulted parameter precedes the required seed.
            begin
               Defaulted := Estimate_ECC_Euclidean
                 (Template_Image => Pair.Template, Moving_Image => Pair.Moving,
                  Initial_Transform => Zero);
               Defaulted_Raised := False;
            exception
               when OpenCV.OpenCV_Error => Defaulted := (others => <>); Defaulted_Raised := True;
            end;
            Assert (Plain_Raised = Seeded_Raised and then Seeded_Raised = Defaulted_Raised,
                    "zero seed changed the success/failure classification");
            Assert (Plain = Seeded and then Seeded = Defaulted,
                    "zero seed differs from the identity-initialized result");
            if Plain_Raised then Failures := Failures + 1; else Successes := Successes + 1; end if;
         end;
      end loop;
      Assert (Failures > 0 and then Successes > 0, "zero-seed cases must cover both outcomes");
   end ECC_Seeded_Zero_Equivalence;

   procedure ECC_Seeded_Signs_And_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      type Signed_Case is record Degrees, TX, TY, Seed_Degrees, Seed_TX, Seed_TY : Long_Float; end record;
      Cases : constant array (1 .. 8) of Signed_Case :=
        [(3.0, 0.0, 0.0, 2.6, 0.2, -0.2), (-3.0, 0.0, 0.0, -2.6, 0.2, -0.2),
         (3.0, 2.0, 1.0, 2.5, 2.3, 0.8), (3.0, -2.0, -1.0, 2.5, -2.3, -0.8),
         (-3.0, 2.0, -1.0, -2.5, 2.3, -0.8), (-3.0, -2.0, 1.0, -2.5, -2.3, 1.2),
         (8.0, 0.0, 0.0, 7.5, 0.3, -0.2), (-8.0, 0.0, 0.0, -8.5, 0.3, -0.2)];
   begin
      for Item of Cases loop
         declare
            Pair : constant ECC_Pair := Euc_Pair (Item.Degrees, Item.TX, Item.TY);
            R : ECC_Euclidean_Result;
            Raised : Boolean;
         begin
            Run_Seeded (Pair, Seed (Item.Seed_Degrees, Item.Seed_TX, Item.Seed_TY), R, Raised);
            Assert (not Raised, "near-correct seed failed in native ECC");
            Assert (Euc_Near (R, Item.Degrees, Item.TX, Item.TY), "seeded ECC did not recover the transform");
            Assert ((R.Rotation_Radians > 0.0) = (Item.Degrees > 0.0), "seeded rotation sign");
            Assert (R.Correlation > 0.98 and then R.Correlation <= 1.000001, "seeded correlation range");
         end;
      end loop;
   end ECC_Seeded_Signs_And_Translation;


   procedure ECC_Seeded_Consumption (T : in out Fixture) is
      pragma Unreferenced (T);
      Rot12      : constant ECC_Pair := Euc_Pair (12.0, 0.0, 0.0);
      Rot10_T    : constant ECC_Pair := Euc_Pair (10.0, 2.0, 1.0);
      Rot10      : constant ECC_Pair := Euc_Pair (10.0, 0.0, 0.0);
      Plain, Near_R, Wrong_R, Other_R : ECC_Euclidean_Result;
      Plain_Raised, Near_Raised, Wrong_Raised, Other_Raised : Boolean;
      Near_Seed  : constant ECC_Euclidean_Result := Seed (9.5, 0.3, -0.2);
   begin
      --  12 degrees: identity initialization raises on every supported OpenCV version, a
      --  near-correct seed recovers it, and a seed with the wrong rotation sign does not.
      Run_Unseeded (Rot12, Plain, Plain_Raised);
      Run_Seeded (Rot12, Seed (11.5, 0.3, -0.2), Near_R, Near_Raised);
      Run_Seeded (Rot12, Seed (-11.5, 0.3, -0.2), Wrong_R, Wrong_Raised);
      Assert (Plain_Raised, "identity initialization unexpectedly solved 12 degrees");
      Assert (not Near_Raised and then Euc_Near (Near_R, 12.0, 0.0, 0.0),
              "seed was not consumed (12 degrees)");
      Assert (Wrong_Raised, "a seed with the wrong rotation sign was not distinguished");
      --  10 degrees + (2,1): identity initialization returns a WRONG finite optimum.
      Run_Unseeded (Rot10_T, Plain, Plain_Raised);
      Run_Seeded (Rot10_T, Seed (9.5, 2.3, 0.8), Near_R, Near_Raised);
      Assert (not Plain_Raised and then not Euc_Near (Plain, 10.0, 2.0, 1.0)
              and then Plain.Correlation < 0.7, "identity result is not a wrong finite optimum");
      Assert (not Near_Raised and then Euc_Near (Near_R, 10.0, 2.0, 1.0)
              and then Near_R.Correlation > 0.98, "seed was not consumed (10 degrees + translation)");
      --  Different valid seeds give meaningfully different results for the same images.
      Run_Seeded (Rot10, Near_Seed, Near_R, Near_Raised);
      Run_Seeded (Rot10, Seed (10.0, 25.0, -25.0), Wrong_R, Wrong_Raised);
      Run_Unseeded (Rot10, Plain, Plain_Raised);
      Assert (not Near_Raised and then Euc_Near (Near_R, 10.0, 0.0, 0.0), "near seed result");
      Assert (not Wrong_Raised and then not Euc_Near (Wrong_R, 10.0, 0.0, 0.0)
              and then abs (LF (Wrong_R.Displacement.X) - LF (Near_R.Displacement.X)) > 10.0
              and then Wrong_R.Correlation < 0.9,
              "inaccurate seed did not produce a different finite result");
      --  The seed is refined, never returned unchanged.
      Assert (Near_R.Displacement /= Near_Seed.Displacement
              and then Near_R.Rotation_Radians /= Near_Seed.Rotation_Radians,
              "near seed was returned without native refinement");
      --  A zero seed is the identity start: it reproduces the unseeded outcome exactly.
      Run_Seeded (Rot10, (others => <>), Other_R, Other_Raised);
      Assert (Other_Raised = Plain_Raised and then (Other_Raised or else Other_R = Plain),
              "zero seed differs from identity initialization");
      Assert (Other_Raised or else Other_R /= Near_R, "zero seed matched the near-seed result");
   end ECC_Seeded_Consumption;

   procedure ECC_Seeded_Refinement_And_Residual (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair    : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      Initial : constant ECC_Euclidean_Result := Seed (2.5, 2.3, 0.8);
      R, One, Two, Full : ECC_Euclidean_Result;
      Raised, Dummy     : Boolean;
   begin
      Run_Seeded (Pair, Initial, R, Raised);
      Assert (not Raised and then Euc_Near (R, 3.0, 2.0, 1.0), "imperfect seed was not refined");
      --  The refined transform is closer to the truth than the seed, in every component.
      Assert (abs (Long_Float (R.Rotation_Radians) - 3.0 * Degrees_To_Radians)
              < abs (Long_Float (Initial.Rotation_Radians) - 3.0 * Degrees_To_Radians),
              "rotation error did not improve over the seed");
      Assert (abs (LF (R.Displacement.X) - 2.0) < abs (LF (Initial.Displacement.X) - 2.0)
              and then abs (LF (R.Displacement.Y) - 1.0) < abs (LF (Initial.Displacement.Y) - 1.0),
              "translation error did not improve over the seed");
      --  Inverse-map residual (moving resampled at W * p): seed < unaligned, refined < seed.
      declare
         Before  : constant Long_Float := Euc_Residual (Pair, (others => <>), Aligned => False);
         At_Seed : constant Long_Float := Euc_Residual (Pair, Initial, Aligned => True);
         After   : constant Long_Float := Euc_Residual (Pair, R, Aligned => True);
      begin
         Assert (Before > 10.0 and then At_Seed < Before, "seed does not reduce the residual");
         Assert (After < At_Seed, "refined residual did not improve over the seed residual");
      end;
      --  Iteration and epsilon options reach the seeded call.
      Run_Seeded (Pair, Initial, One, Dummy, (Maximum_Iterations => 1, Epsilon => 0.0001));
      Run_Seeded (Pair, Initial, Two, Dummy, (Maximum_Iterations => 2, Epsilon => 0.0001));
      Run_Seeded (Pair, Initial, Full, Dummy, (Maximum_Iterations => 50, Epsilon => 1.0E-8));
      Assert (One /= R and then Two /= R and then One /= Two, "seeded iteration option ignored");
      Assert (Full /= R, "seeded epsilon option ignored");
      Assert (One.Correlation < R.Correlation, "one iteration scored like the full run");
   end ECC_Seeded_Refinement_And_Residual;


   procedure ECC_Seeded_Input_Independence (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair     : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      Before_T : constant Long_Long_Integer := EF.Checksum (Pair.Template);
      Before_M : constant Long_Long_Integer := EF.Checksum (Pair.Moving);
      Initial  : constant ECC_Euclidean_Result := Seed (2.5, 2.3, 0.8, Correlation => 0.25);
      Saved    : constant ECC_Euclidean_Result := Initial;
      Correlations : constant array (1 .. 5) of OpenCV.Float64_Value :=
        [0.0, 0.25, 0.999, -1.0, 1.0E300];
      First, Second, Third : ECC_Euclidean_Result;
      Dummy : Boolean;
   begin
      Run_Seeded (Pair, Initial, First, Dummy);
      Assert (Initial = Saved, "the initial transform was modified");
      --  Only the correlation differs: the result is unchanged, so it is not an input.
      for Correlation of Correlations loop
         Run_Seeded (Pair, (Initial with delta Correlation => Correlation), Second, Dummy);
         Assert (Second = First, "initial correlation influenced the result");
      end loop;
      Assert (First.Correlation /= Initial.Correlation, "initial correlation was propagated");
      --  Reuse: the previous result is a valid seed for the next call and is not consumed.
      Run_Seeded (Pair, First, Second, Dummy);
      Assert (Second.Correlation > 0.98 and then Euc_Near (Second, 3.0, 2.0, 1.0),
              "reusing a previous result as a seed");
      Run_Seeded (Pair, First, Third, Dummy);
      Assert (Third = Second, "repeated calls with the same seed differ");
      --  A rejected request leaves the seed intact and a later call still works.
      Assert (Seed_Outcome_Of (Pair, Initial, (Maximum_Iterations => 101, Epsilon => 0.0001))
              = Preflight_Rejected, "invalid options were not rejected by the preflight");
      Assert (Initial = Saved, "the initial transform changed after a rejected request");
      Run_Seeded (Pair, Initial, Third, Dummy);
      Assert (Third = First, "seeded call after a rejection differs");
      Assert (EF.Checksum (Pair.Template) = Before_T and then EF.Checksum (Pair.Moving) = Before_M,
              "seeded ECC modified an image");
   end ECC_Seeded_Input_Independence;

   procedure ECC_Seeded_Centered_Rotation (T : in out Fixture) is
      pragma Unreferenced (T);
      Theta : constant Long_Float := 3.0 * Degrees_To_Radians;
      CX    : constant Long_Float := 63.5;
      CY    : constant Long_Float := 63.5;
      TX    : constant Long_Float := CX - (LM.Cos (Theta) * CX - LM.Sin (Theta) * CY) + 1.5;
      TY    : constant Long_Float := CY - (LM.Sin (Theta) * CX + LM.Cos (Theta) * CY) - 1.0;
      Pair  : constant ECC_Pair := Euc_Pair (3.0, TX, TY);
      Initial : constant ECC_Euclidean_Result :=
        (Displacement     => (X => OpenCV.Float32_Value (TX + 0.4), Y => OpenCV.Float32_Value (TY - 0.3)),
         Rotation_Radians => OpenCV.Float64_Value (2.6 * Degrees_To_Radians), Correlation => 0.0);
      Center_Seed : constant ECC_Euclidean_Result :=
        (Displacement => (X => 1.5, Y => -1.0), Rotation_Radians => OpenCV.Float64_Value (Theta),
         Correlation => 0.0);
      R, Misread : ECC_Euclidean_Result;
      Raised, Misread_Raised : Boolean;
      MX, MY : Long_Float;
   begin
      Run_Seeded (Pair, Initial, R, Raised);
      Assert (not Raised and then Euc_Near (R, 3.0, TX, TY), "centred rotation seed was not refined");
      Euc_Map (R, CX, CY, MX, MY);
      Assert (abs (MX - (CX + 1.5)) < 0.15 and then abs (MY - (CY - 1.0)) < 0.15, "centre point mapping");
      Euc_Map (R, 10.0, 100.0, MX, MY);
      Assert (abs (MX - (LM.Cos (Theta) * 10.0 - LM.Sin (Theta) * 100.0 + TX)) < 0.15
              and then abs (MY - (LM.Sin (Theta) * 10.0 + LM.Cos (Theta) * 100.0 + TY)) < 0.15,
              "off-centre point mapping");
      --  Displacement is the matrix column, not the centre displacement: a seed made of the
      --  centre displacement is wrong by the compensation terms and is not silently corrected.
      Run_Seeded (Pair, Center_Seed, Misread, Misread_Raised);
      Assert (Misread_Raised or else Misread /= R, "centre displacement was reinterpreted as the matrix column");
   end ECC_Seeded_Centered_Rotation;

   procedure ECC_Seeded_Regions_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      Window  : constant OpenCV.Rect := (16, 16, 128, 128);
      Initial : constant ECC_Euclidean_Result := Seed (2.5, 2.3, 0.8);
      Saved, Contiguous : ECC_Euclidean_Result;
      Dummy : Boolean;
      Before_A, Before_B : Long_Long_Integer;
   begin
      declare
         Parent_A : constant OpenCV.Core.Mat := EF.Texture (160, 160);
         Parent_B : constant OpenCV.Core.Mat := EF.Rigid_Texture (160, 160, 3.0, 2.0, 1.0);
         Region_A : constant OpenCV.Core.Mat := Parent_A.Region (Window);
         Region_B : constant OpenCV.Core.Mat := Parent_B.Region (Window);
      begin
         Before_A := EF.Checksum (Parent_A);
         Before_B := EF.Checksum (Parent_B);
         Assert (not Region_A.Is_Continuous and then not Region_B.Is_Continuous, "Regions are contiguous");
         Run_Seeded ((Region_A, Region_B), Initial, Saved, Dummy);
         Run_Seeded ((Region_A.Clone, Region_B.Clone), Initial, Contiguous, Dummy);
         Assert (Saved = Contiguous, "seeded Region differs from the contiguous clone");
         Assert (EF.Checksum (Parent_A) = Before_A and then EF.Checksum (Parent_B) = Before_B,
                 "seeded ECC modified a Region parent");
      end;
      Assert (abs (Long_Float (Saved.Rotation_Radians) - 3.0 * Degrees_To_Radians) < Euc_Angle_Tolerance
              and then Saved.Correlation > 0.98, "seeded result after parent finalization");
   end ECC_Seeded_Regions_Lifetime;


   procedure ECC_Seeded_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      --  NaN/Infinity bit patterns are the inputs under test; do not validity-check them.
      pragma Validity_Checks (Off);
      Pair  : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      function F32 is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, OpenCV.Float32_Value);
      function F64 is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      NaN32 : constant OpenCV.Float32_Value := F32 (16#7FC0_0000#);
      Inf32 : constant OpenCV.Float32_Value := F32 (16#7F80_0000#);
      NaN64 : constant OpenCV.Float64_Value := F64 (16#7FF8_0000_0000_0000#);
      Inf64 : constant OpenCV.Float64_Value := F64 (16#7FF0_0000_0000_0000#);
      Limit : constant OpenCV.Float32_Value := 1_048_576.0;
      Base  : constant ECC_Euclidean_Result := Seed (2.5, 2.3, 0.8);

      procedure Rejected (Initial : ECC_Euclidean_Result; Message : String) is
      begin
         Assert (Seed_Outcome_Of (Pair, Initial) = Preflight_Rejected, Message);
      end Rejected;

      procedure Accepted_By_Preflight (Initial : ECC_Euclidean_Result; Message : String) is
      begin
         Assert (Seed_Outcome_Of (Pair, Initial) /= Preflight_Rejected, Message);
      end Accepted_By_Preflight;
   begin
      --  Non-finite and out-of-range seed components are rejected, never clamped.
      Rejected ((Base with delta Displacement => (X => NaN32, Y => 0.8)), "NaN displacement X");
      Rejected ((Base with delta Displacement => (X => 2.3, Y => NaN32)), "NaN displacement Y");
      Rejected ((Base with delta Displacement => (X => Inf32, Y => 0.8)), "+Inf displacement X");
      Rejected ((Base with delta Displacement => (X => 2.3, Y => -Inf32)), "-Inf displacement Y");
      Rejected ((Base with delta Rotation_Radians => NaN64), "NaN rotation");
      Rejected ((Base with delta Rotation_Radians => Inf64), "+Inf rotation");
      Rejected ((Base with delta Rotation_Radians => -Inf64), "-Inf rotation");
      Rejected ((Base with delta Displacement => (X => Limit * 2.0, Y => 0.0)), "X far beyond 2**20");
      Rejected ((Base with delta Displacement => (X => 0.0, Y => -Limit * 2.0)), "Y far beyond -2**20");
      Rejected ((Base with delta Displacement => (X => OpenCV.Float32_Value'Last, Y => 0.0)),
                "X at Float32'Last");
      Rejected ((Base with delta Displacement => (X => Limit + 0.125, Y => 0.0)), "X just beyond 2**20");
      Rejected ((Base with delta Displacement => (X => 0.0, Y => -(Limit + 0.125))),
                "Y just beyond -2**20");
      Rejected ((Base with delta Rotation_Radians => 1.0000000001), "angle just beyond +1 radian");
      Rejected ((Base with delta Rotation_Radians => -1.0000000001), "angle just beyond -1 radian");
      Rejected ((Base with delta Rotation_Radians => 3.0), "angle beyond the principal asin branch");
      Rejected ((Base with delta Rotation_Radians => OpenCV.Float64_Value'Last), "angle at Float64'Last");
      --  A bad correlation is not a seed component: it is ignored, never validated.
      Assert (Seed_Outcome_Of (Pair, (Base with delta Correlation => NaN64)) = Accepted,
              "NaN initial correlation was treated as an input");
      Assert (Seed_Outcome_Of (Pair, (Base with delta Correlation => Inf64)) = Accepted,
              "infinite initial correlation was treated as an input");
      --  Exactly at the bounds the Ada preflight accepts the seed; native ECC decides the rest.
      Accepted_By_Preflight ((Seed (0.0, 0.0, 0.0) with delta Rotation_Radians => 1.0), "angle +1 rad");
      Accepted_By_Preflight ((Seed (0.0, 0.0, 0.0) with delta Rotation_Radians => -1.0), "angle -1 rad");
      Accepted_By_Preflight
        ((Seed (0.0, 0.0, 0.0) with delta Displacement => (X => Limit, Y => -Limit)), "translation +/-2**20");
      Accepted_By_Preflight
        ((Seed (0.0, 0.0, 0.0) with delta Displacement => (X => -Limit, Y => Limit)), "translation -/+2**20");
   end ECC_Seeded_Validation;


   procedure ECC_Seeded_Image_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Good  : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Empty : OpenCV.Core.Mat;
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.UInt8, Channels => 3));
      Base  : constant ECC_Euclidean_Result := Seed (2.5, 2.3, 0.8);
      Bad_Epsilons : constant array (1 .. 5) of OpenCV.Float64_Value :=
        [0.0, -1.0E-4, 9.0E-9, 0.1000001, 1.0];

      procedure Image_Rejected (Template, Moving : OpenCV.Core.Mat; Message : String) is
         Unused : ECC_Euclidean_Result;
      begin
         Unused := Estimate_ECC_Euclidean (Template, Moving, (others => <>), Base);
         Assert (False, Message & ": request accepted");
      exception
         when OpenCV.OpenCV_Error => null;
      end Image_Rejected;
   begin
      Image_Rejected (Empty, Good, "empty template");
      Image_Rejected (Good, Empty, "empty moving");
      Image_Rejected (Float_Image, Float_Image, "Float32 images");
      Image_Rejected (Color, Color, "three-channel images");
      Image_Rejected (Good, EF.Texture (128, 129), "different columns");
      Image_Rejected (EF.Texture (129, 128), Good, "different rows");
      Image_Rejected (EF.Texture (31, 128), EF.Texture (31, 128), "31 rows");
      Image_Rejected (EF.Texture (128, 31), EF.Texture (128, 31), "31 columns");
      declare
         Pair : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      begin
         Assert (Seed_Outcome_Of (Pair, Base, (Maximum_Iterations => 101, Epsilon => 0.0001))
                 = Preflight_Rejected, "101 iterations");
         Assert (Seed_Outcome_Of (Pair, Base, (Maximum_Iterations => Positive'Last, Epsilon => 0.0001))
                 = Preflight_Rejected, "Positive'Last iterations");
         for Epsilon of Bad_Epsilons loop
            Assert (Seed_Outcome_Of (Pair, Base, (Maximum_Iterations => 50, Epsilon => Epsilon))
                    = Preflight_Rejected, "invalid epsilon");
         end loop;
         Assert (Seed_Outcome_Of (Pair, Base, (Maximum_Iterations => 1, Epsilon => 1.0E-8)) = Accepted
                 and then Seed_Outcome_Of (Pair, Base, (Maximum_Iterations => 100, Epsilon => 0.1)) = Accepted,
                 "option bounds are accepted");
      end;
   end ECC_Seeded_Image_Validation;

   procedure ECC_Seeded_Nonconvergence (T : in out Fixture) is
      pragma Unreferenced (T);
      Flat          : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
      Texture_Image : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Initial       : constant ECC_Euclidean_Result := Seed (1.0, 1.0, 1.0);
      Pair_12       : constant ECC_Pair := Euc_Pair (12.0, 0.0, 0.0);
      Pair_Far      : constant ECC_Pair := Euc_Pair (12.0, 2.0, 1.0);
      Wrong         : ECC_Euclidean_Result;
      Raised        : Boolean;
   begin
      --  Native failures carry the shim prefix; they are not Ada preflight rejections.
      Assert (Seed_Outcome_Of ((Flat, Flat), Initial) = Native_Failed, "constant images");
      Assert (Seed_Outcome_Of ((Flat, Texture_Image), Initial) = Native_Failed, "constant template");
      Assert (Seed_Outcome_Of ((Texture_Image, Flat), Initial) = Native_Failed, "constant moving");
      Assert (Seed_Outcome_Of ((EF.Noise (128, 128, 3), EF.Noise (128, 128, 9)), Initial) = Native_Failed,
              "independent noise");
      --  Valid but badly inaccurate seeds fail natively, and succeed with a near seed.
      Assert (Seed_Outcome_Of (Pair_12, Seed (-12.0, 0.0, 0.0)) = Native_Failed, "wrong-sign rotation seed");
      Assert (Seed_Outcome_Of (Pair_Far, Seed (12.0, 25.0, -25.0)) = Native_Failed, "far translation seed");
      Assert (Seed_Outcome_Of (Pair_12, Seed (11.5, 0.3, -0.2)) = Accepted, "near seed on the same images");
      --  Wrong-but-finite results are returned as values, never turned into errors.
      Run_Seeded (Euc_Pair (10.0, 0.0, 0.0), Seed (10.0, 25.0, -25.0), Wrong, Raised);
      Assert (not Raised and then Wrong.Correlation < 0.9, "wrong finite optimum was suppressed");
   end ECC_Seeded_Nonconvergence;


   --  Seeded Euclidean oracle (Task 013): same-build direct OpenCV outcome for a named
   --  fixture and an explicit initial warp.
   type Seeded_Oracle_Entry is record
      Entry_Data : Euc_Oracle_Entry;
      Seed_Theta, Seed_TX, Seed_TY : Long_Float := 0.0;
   end record;

   type Seeded_Oracle_Table is array (1 .. 64) of Seeded_Oracle_Entry;

   --  Strict parser: a missing, truncated, stale-versioned or malformed file raises.
   procedure Load_Seeded_Oracle
     (Table : out Seeded_Oracle_Table; Count : out Natural;
      Version : out String; Version_Length : out Natural)
   is
      File : Ada.Text_IO.File_Type;
      Path : constant String :=
        Ada.Environment_Variables.Value
          ("VIDEO_ECC_EUCLIDEAN_SEEDED_ORACLE", "../obj/oracle/ecc-euclidean-seeded.txt");

      function Word return String is
         Result : String (1 .. 64);
         Last   : Natural := 0;
         Ch     : Character;
         Eol    : Boolean;
      begin
         loop
            Ada.Text_IO.Look_Ahead (File, Ch, Eol);
            exit when Eol or else Ch /= ' ';
            Ada.Text_IO.Get (File, Ch);
         end loop;
         loop
            Ada.Text_IO.Look_Ahead (File, Ch, Eol);
            exit when Eol or else Ch = ' ';
            Ada.Text_IO.Get (File, Ch);
            Last := Last + 1;
            Result (Last) := Ch;
         end loop;
         return Result (1 .. Last);
      end Word;

      procedure Get_Float (Value : out Long_Float) is
      begin
         Euc_Floats.Get (File, Value);
      end Get_Float;
      procedure Require (Condition : Boolean; Message : String) is
      begin
         if not Condition then
            raise Program_Error with Message;
         end if;
      end Require;
   begin
      Count := 0;
      Version := [others => ' '];
      Version_Length := 0;
      Table := [others => <>];
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File, Path);
      Require (Word = "ecc-euclidean-seeded-oracle", "seeded oracle magic");
      Require (Word = "1", "seeded oracle format version");
      Ada.Text_IO.Skip_Line (File);
      Require (Word = "version", "seeded oracle version line");
      declare
         V : constant String := Word;
      begin
         Require (V'Length <= Version'Length, "seeded oracle version too long");
         Version (Version'First .. Version'First + V'Length - 1) := V;
         Version_Length := V'Length;
      end;
      Ada.Text_IO.Skip_Line (File);
      loop
         declare
            Tag : constant String := Word;
         begin
            if Tag = "end" then
               declare
                  Declared : Integer;
               begin
                  ECC_Integers.Get (File, Declared);
                  Require (Declared = Count, "seeded oracle truncated or inconsistent");
               end;
               exit;
            end if;
            Require (Tag = "fixture" and then Count < Table'Last, "seeded oracle fixture line");
            Count := Count + 1;
            declare
               S    : Seeded_Oracle_Entry renames Table (Count);
               E    : Euc_Oracle_Entry renames S.Entry_Data;
               Name : constant String := Word;
            begin
               Require (Name'Length <= E.Name'Length, "seeded oracle name too long");
               E.Name (1 .. Name'Length) := Name;
               E.Length := Name'Length;
               ECC_Integers.Get (File, E.Iterations);
               Get_Float (E.Epsilon);
               Get_Float (S.Seed_Theta);
               Get_Float (S.Seed_TX);
               Get_Float (S.Seed_TY);
               declare
                  Outcome : constant String := Word;
               begin
                  Require (Outcome = "OK" or else Outcome = "EXC", "seeded oracle outcome");
                  E.Converged := Outcome = "OK";
               end;
               for I in E.W'Range loop
                  Get_Float (E.W (I));
               end loop;
               Get_Float (E.TX);
               Get_Float (E.TY);
               Get_Float (E.Theta);
               Get_Float (E.Rho);
               ECC_Longs.Get (File, E.Template_Sum);
               ECC_Longs.Get (File, E.Moving_Sum);
            end;
            Ada.Text_IO.Skip_Line (File);
         end;
      end loop;
      Ada.Text_IO.Close (File);
   exception
      when others =>
         if Ada.Text_IO.Is_Open (File) then
            Ada.Text_IO.Close (File);
         end if;
         raise;
   end Load_Seeded_Oracle;


   --  Rebuilds each seeded-oracle fixture by name, independently of the C++ code.
   function Seeded_Build (Name : String) return ECC_Pair is
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Theta3 : constant Long_Float := 3.0 * Degrees_To_Radians;
      Centre_TX : constant Long_Float :=
        63.5 - (LM.Cos (Theta3) * 63.5 - LM.Sin (Theta3) * 63.5) + 1.5;
      Centre_TY : constant Long_Float :=
        63.5 - (LM.Sin (Theta3) * 63.5 + LM.Cos (Theta3) * 63.5) - 1.0;
      function Tex return OpenCV.Core.Mat is (EF.Texture (128, 128));
      function P (D, X, Y : Long_Float) return ECC_Pair is (Tex, EF.Rigid_Texture (128, 128, D, X, Y));
      function Starts (Prefix : String) return Boolean is
        (Name'Length >= Prefix'Length and then Name (Name'First .. Name'First + Prefix'Length - 1) = Prefix);
   begin
      if Name = "zero-identity" then return (Tex, Tex);
      elsif Name = "zero-translation-2-1" then return P (0.0, 2.0, 1.0);
      elsif Name = "zero-rot+3" then return P (3.0, 0.0, 0.0);
      elsif Name = "zero-rot+3-t+2+1" or else Name = "imperfect-rot+3-t+2+1" or else Name = "imperfect-it1"
        or else Name = "imperfect-it2" or else Name = "imperfect-it100-e8" or else Name = "imperfect-e0.1"
        or else Name = "bound-angle+1rad" or else Name = "bound-angle-1rad" or else Name = "bound-translation"
      then return P (3.0, 2.0, 1.0);
      elsif Name = "zero-rot+12-nonconv" or else Name = "near-rot+12" or else Name = "wrong-rot-rot+12"
      then return P (12.0, 0.0, 0.0);
      elsif Name = "zero-rot+10-t+2+1" or else Name = "near-rot+10-t+2+1" or else Name = "wrong-rot-rot+10-t+2+1"
      then return P (10.0, 2.0, 1.0);
      elsif Name = "near-rot+8" then return P (8.0, 0.0, 0.0);
      elsif Name = "near-rot-8" then return P (-8.0, 0.0, 0.0);
      elsif Name = "near-rot+10" or else Name = "wrong-trans-rot+10" or else Name = "asym-rot+10"
        or else Name = "asym-rot+10-b" then return P (10.0, 0.0, 0.0);
      elsif Name = "near-rot+12-t+2+1" or else Name = "wrong-trans-rot+12-t+2+1" then return P (12.0, 2.0, 1.0);
      elsif Name = "exact-rot-3-t+2-1" then return P (-3.0, 2.0, -1.0);
      elsif Name = "exact-rot+3-t-2-1" then return P (3.0, -2.0, -1.0);
      elsif Name = "exact-rot-3-t-2+1" then return P (-3.0, -2.0, 1.0);
      elsif Name = "centred-exact" or else Name = "centred-near" then return P (3.0, Centre_TX, Centre_TY);
      elsif Name = "wrong-rot-rot+15" then return P (15.0, 0.0, 0.0);
      elsif Name = "seed-constant" then
         return (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128));
      elsif Name = "seed-noise-vs-noise" then return (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9));
      elsif Name = "seed-noise-vs-texture" then return (Tex, EF.Noise (128, 128, 7));
      elsif Starts ("region-") then
         return (EF.Texture (160, 160).Region (Window),
                 EF.Rigid_Texture (160, 160, 3.0, 2.0, 1.0).Region (Window));
      end if;
      Assert (False, "unknown seeded oracle fixture " & Name);
      return (Tex, Tex);
   end Seeded_Build;

   Expected_Seeded_Oracle_Fixtures : constant := 37;


   procedure ECC_Seeded_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Table : Seeded_Oracle_Table;
      Count, Version_Length : Natural;
      Version : String (1 .. 32);
      Succeeded, Raised, Nonzero_Seed, Distinguishing : Natural := 0;
   begin
      Load_Seeded_Oracle (Table, Count, Version, Version_Length);
      Assert (Version_Length > 0, "seeded oracle records no native version");
      Assert (Count = Expected_Seeded_Oracle_Fixtures, "seeded oracle fixture count is stale");
      for Index in 1 .. Count loop
         declare
            S    : Seeded_Oracle_Entry renames Table (Index);
            E    : Euc_Oracle_Entry renames S.Entry_Data;
            Name : constant String := E.Name (1 .. E.Length);
            Pair : constant ECC_Pair := Seeded_Build (Name);
            Options : constant ECC_Euclidean_Options :=
              (Maximum_Iterations => E.Iterations, Epsilon => OpenCV.Float64_Value (E.Epsilon));
            Initial : constant ECC_Euclidean_Result :=
              (Displacement     =>
                 (X => OpenCV.Float32_Value (S.Seed_TX), Y => OpenCV.Float32_Value (S.Seed_TY)),
               Rotation_Radians => OpenCV.Float64_Value (S.Seed_Theta),
               Correlation      => 0.0);
         begin
            Assert (EF.Checksum (Pair.Template) = E.Template_Sum
                    and then EF.Checksum (Pair.Moving) = E.Moving_Sum,
                    "seeded oracle fixture " & Name & " differs from the Ada fixture (stale oracle)");
            --  The seed identity recorded by the oracle is exactly the value Ada passes.
            Assert (Long_Float (Initial.Displacement.X) = S.Seed_TX
                    and then Long_Float (Initial.Displacement.Y) = S.Seed_TY,
                    "seeded oracle seed identity for " & Name);
            if S.Seed_Theta /= 0.0 or else S.Seed_TX /= 0.0 or else S.Seed_TY /= 0.0 then
               Nonzero_Seed := Nonzero_Seed + 1;
            end if;
            if E.Converged then
               declare
                  R : constant ECC_Euclidean_Result :=
                    Estimate_ECC_Euclidean (Pair.Template, Pair.Moving, Options, Initial);
                  Theta : constant Long_Float := Long_Float (R.Rotation_Radians);
                  C  : constant Long_Float := LM.Cos (Theta);
                  Sn : constant Long_Float := LM.Sin (Theta);
                  Plain : ECC_Euclidean_Result;
                  Plain_Raised : Boolean;
               begin
                  Succeeded := Succeeded + 1;
                  Assert (abs (C - E.W (0)) < 1.0E-6 and then abs (-Sn - E.W (1)) < 1.0E-6
                          and then abs (Sn - E.W (3)) < 1.0E-6 and then abs (C - E.W (4)) < 1.0E-6
                          and then abs (LF (R.Displacement.X) - E.W (2)) < 1.0E-5
                          and then abs (LF (R.Displacement.Y) - E.W (5)) < 1.0E-5,
                          "seeded matrix reconstruction differs from the native warp for " & Name);
                  Assert (Angle_Difference (Theta, E.Theta) < 1.0E-6,
                          "seeded angle differs from native for " & Name);
                  Assert (abs (LF (R.Displacement.X) - E.TX) < 1.0E-5
                          and then abs (LF (R.Displacement.Y) - E.TY) < 1.0E-5
                          and then abs (Long_Float (R.Correlation) - E.Rho) < 1.0E-6,
                          "seeded translation/correlation differs from native for " & Name);
                  --  The seed is distinguishing when identity initialization gives another outcome.
                  Run_Unseeded (Pair, Plain, Plain_Raised, Options);
                  if Plain_Raised or else Plain /= R then
                     Distinguishing := Distinguishing + 1;
                  end if;
               end;
            else
               begin
                  declare
                     R : constant ECC_Euclidean_Result :=
                       Estimate_ECC_Euclidean (Pair.Template, Pair.Moving, Options, Initial);
                     pragma Unreferenced (R);
                  begin
                     Assert (False, "seeded binding succeeded where native raised: " & Name);
                  end;
               exception
                  when OpenCV.OpenCV_Error => Raised := Raised + 1;
               end;
            end if;
         end;
      end loop;
      Assert (Succeeded > 0 and then Raised > 0, "seeded oracle did not cover both outcomes");
      Assert (Succeeded + Raised = Count, "seeded oracle outcome accounting");
      Assert (Nonzero_Seed >= 20, "seeded oracle needs genuinely nonzero native initializations");
      Assert (Distinguishing >= 4, "seeded oracle has too few seed-distinguishing fixtures");
   end ECC_Seeded_Direct_Oracle;


   ---------------------------------------------------------------------
   --  Task 014: ECC six-parameter affine alignment
   ---------------------------------------------------------------------
   Aff_Linear_Tolerance      : constant Long_Float := 0.003;
   Aff_Translation_Tolerance : constant Long_Float := 0.1;

   function Aff_Pair (A, B, C, D, TX, TY : Long_Float) return ECC_Pair is
     (Template => EF.Texture (128, 128),
      Moving   => EF.Affine_Texture (128, 128, A, B, C, D, TX, TY));

   function Aff_Near
     (R : ECC_Affine_Result; A, B, C, D, TX, TY : Long_Float) return Boolean is
     (abs (LF (R.Warp.M00) - A) < Aff_Linear_Tolerance
      and then abs (LF (R.Warp.M01) - B) < Aff_Linear_Tolerance
      and then abs (LF (R.Warp.M10) - C) < Aff_Linear_Tolerance
      and then abs (LF (R.Warp.M11) - D) < Aff_Linear_Tolerance
      and then abs (LF (R.Warp.M02) - TX) < Aff_Translation_Tolerance
      and then abs (LF (R.Warp.M12) - TY) < Aff_Translation_Tolerance);

   --  Applies the returned 2 x 3 matrix: template coordinates to moving coordinates.
   procedure Aff_Map (W : ECC_Affine_Warp; X, Y : Long_Float; MX, MY : out Long_Float) is
   begin
      MX := LF (W.M00) * X + LF (W.M01) * Y + LF (W.M02);
      MY := LF (W.M10) * X + LF (W.M11) * Y + LF (W.M12);
   end Aff_Map;

   --  Mean absolute difference over the interior; Aligned resamples the moving image at
   --  W * p, which is exactly what WARP_INVERSE_MAP does with the returned matrix.
   function Aff_Residual (Pair : ECC_Pair; W : ECC_Affine_Warp; Aligned : Boolean) return Long_Float is
      Margin : constant := 24;
      Sum    : Long_Float := 0.0;
      Count  : Natural := 0;
      MX, MY : Long_Float;
   begin
      for Y in Margin .. 127 - Margin loop
         for X in Margin .. 127 - Margin loop
            if Aligned then
               Aff_Map (W, Long_Float (X), Long_Float (Y), MX, MY);
            else
               MX := Long_Float (X);
               MY := Long_Float (Y);
            end if;
            Sum := Sum + abs (Sample (Pair.Moving, MX, MY)
                              - Long_Float (OpenCV.Core.UInt8_Access.Get (Pair.Template, Y, X)));
            Count := Count + 1;
         end loop;
      end loop;
      return Sum / Long_Float (Count);
   end Aff_Residual;

   function Euc_As_Warp (R : ECC_Euclidean_Result) return ECC_Affine_Warp is
      Theta : constant Long_Float := Long_Float (R.Rotation_Radians);
   begin
      return (M00 => OpenCV.Float32_Value (LM.Cos (Theta)),
              M01 => OpenCV.Float32_Value (-LM.Sin (Theta)),
              M02 => R.Displacement.X,
              M10 => OpenCV.Float32_Value (LM.Sin (Theta)),
              M11 => OpenCV.Float32_Value (LM.Cos (Theta)),
              M12 => R.Displacement.Y);
   end Euc_As_Warp;

   procedure Aff_Expect_Error
     (Template, Moving : OpenCV.Core.Mat;
      Options : ECC_Affine_Options := (others => <>);
      Message : String)
   is
      Unused : ECC_Affine_Result;
   begin
      Unused := Estimate_ECC_Affine (Template, Moving, Options);
      Assert (False, Message & ": request accepted");
   exception
      when OpenCV.OpenCV_Error => null;
   end Aff_Expect_Error;

   --  A shim-originated rejection carries the "Video.Estimate_ECC_Affine" prefix.
   procedure Aff_Expect_Preflight_Error (Template, Moving : OpenCV.Core.Mat; Message : String) is
      Unused : ECC_Affine_Result;
   begin
      Unused := Estimate_ECC_Affine (Template, Moving);
      Assert (False, Message & ": request accepted");
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text   : constant String := Ada.Exceptions.Exception_Message (Error);
            Marker : constant String := "Video.Estimate_ECC_Affine";
         begin
            Assert (Text'Length < Marker'Length
                    or else Text (Text'First .. Text'First + Marker'Length - 1) /= Marker,
                    Message & ": rejected by the native layer, not the Ada preflight");
         end;
   end Aff_Expect_Preflight_Error;

   procedure ECC_Affine_Defaults (T : in out Fixture) is
      pragma Unreferenced (T);
      Options     : constant ECC_Affine_Options := (others => <>);
      Translation : constant ECC_Translation_Options := Options;
      Zero        : constant ECC_Affine_Result := (others => <>);
      Identity    : constant ECC_Affine_Warp := (M00 => 1.0, M01 => 0.0, M02 => 0.0,
                                                  M10 => 0.0, M11 => 1.0, M12 => 0.0);
   begin
      Assert (Options.Maximum_Iterations = 50 and then Options.Epsilon = 0.0001
              and then Translation = Options, "ECC affine options reuse the translation defaults");
      Assert (Zero.Warp = Identity and then Zero.Correlation = 0.0, "ECC affine default result");
      Assert (Zero.Warp.M00 = 1.0 and then Zero.Warp.M11 = 1.0 and then Zero.Warp.M01 = 0.0
              and then Zero.Warp.M10 = 0.0 and then Zero.Warp.M02 = 0.0 and then Zero.Warp.M12 = 0.0,
              "ECC affine default warp is the identity");
   end ECC_Affine_Defaults;

   procedure ECC_Affine_Identity_And_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Base     : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Before   : constant Long_Long_Integer := EF.Checksum (Base);
      Identity : constant ECC_Affine_Result := Estimate_ECC_Affine (Base, Base);
      Forward  : constant ECC_Pair := Aff_Pair (1.0, 0.0, 0.0, 1.0, 2.0, 1.0);
      Backward : constant ECC_Pair := Aff_Pair (1.0, 0.0, 0.0, 1.0, -2.0, -1.0);
      Plus     : constant ECC_Affine_Result := Estimate_ECC_Affine (Forward.Template, Forward.Moving);
      Minus    : constant ECC_Affine_Result := Estimate_ECC_Affine (Backward.Template, Backward.Moving);
      Swapped  : constant ECC_Pair := Aff_Pair (1.0, 0.0, 0.0, 1.0, 1.0, 2.0);
      Distinct : constant ECC_Affine_Result := Estimate_ECC_Affine (Swapped.Template, Swapped.Moving);
   begin
      Assert (Aff_Near (Identity, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
              and then abs (Identity.Correlation - 1.0) < 1.0E-6, "ECC affine identity");
      Assert (Aff_Near (Plus, 1.0, 0.0, 0.0, 1.0, 2.0, 1.0), "ECC affine translation (2,1)");
      Assert (Aff_Near (Minus, 1.0, 0.0, 0.0, 1.0, -2.0, -1.0), "ECC affine translation (-2,-1)");
      --  Placement: M02 is X and M12 is Y; a transposed or swapped translation fails here.
      Assert (abs (LF (Plus.Warp.M02) - 2.0) < 0.1 and then abs (LF (Plus.Warp.M12) - 1.0) < 0.1,
              "ECC affine translation placed in the wrong column entries");
      Assert (abs (LF (Distinct.Warp.M02) - 1.0) < 0.1 and then abs (LF (Distinct.Warp.M12) - 2.0) < 0.1,
              "ECC affine (1,2) translation not distinguished from (2,1)");
      Assert (LF (Plus.Warp.M02) > 0.0 and then LF (Minus.Warp.M02) < 0.0
              and then LF (Plus.Warp.M12) > 0.0 and then LF (Minus.Warp.M12) < 0.0,
              "ECC affine translation signs");
      Assert (EF.Checksum (Base) = Before, "ECC affine modified an input");
   end ECC_Affine_Identity_And_Translation;

   procedure ECC_Affine_Rotation_And_Scale (T : in out Fixture) is
      pragma Unreferenced (T);
      Pi : constant Long_Float := 3.14159265358979323846;
      Angles : constant array (1 .. 4) of Long_Float := [1.0, -1.0, 3.0, -3.0];
   begin
      for Degrees of Angles loop
         declare
            Theta : constant Long_Float := Degrees * Pi / 180.0;
            C     : constant Long_Float := LM.Cos (Theta);
            S     : constant Long_Float := LM.Sin (Theta);
            Pair  : constant ECC_Pair := Aff_Pair (C, -S, S, C, 0.0, 0.0);
            R     : constant ECC_Affine_Result := Estimate_ECC_Affine (Pair.Template, Pair.Moving);
         begin
            --  The sine terms sit at M10 (+sin) and M01 (-sin) for a positive rotation.
            Assert (abs (LF (R.Warp.M10) - S) < Aff_Linear_Tolerance
                    and then abs (LF (R.Warp.M01) + S) < Aff_Linear_Tolerance
                    and then abs (LF (R.Warp.M00) - C) < Aff_Linear_Tolerance
                    and then abs (LF (R.Warp.M11) - C) < Aff_Linear_Tolerance
                    and then abs (LF (R.Warp.M02)) < Aff_Translation_Tolerance
                    and then abs (LF (R.Warp.M12)) < Aff_Translation_Tolerance,
                    "ECC affine signed rotation coefficients");
            Assert ((Degrees > 0.0) = (R.Warp.M10 > 0.0), "ECC affine rotation sign");
         end;
      end loop;
      declare
         Up   : constant ECC_Pair := Aff_Pair (1.02, 0.0, 0.0, 1.02, 0.0, 0.0);
         Down : constant ECC_Pair := Aff_Pair (0.98, 0.0, 0.0, 0.98, 0.0, 0.0);
         Aniso : constant ECC_Pair := Aff_Pair (1.03, 0.0, 0.0, 0.97, 0.0, 0.0);
         Tall  : constant ECC_Pair := Aff_Pair (0.97, 0.0, 0.0, 1.03, 0.0, 0.0);
         RU : constant ECC_Affine_Result := Estimate_ECC_Affine (Up.Template, Up.Moving);
         RD : constant ECC_Affine_Result := Estimate_ECC_Affine (Down.Template, Down.Moving);
         RA : constant ECC_Affine_Result := Estimate_ECC_Affine (Aniso.Template, Aniso.Moving);
         RT : constant ECC_Affine_Result := Estimate_ECC_Affine (Tall.Template, Tall.Moving);
      begin
         Assert (Aff_Near (RU, 1.02, 0.0, 0.0, 1.02, 0.0, 0.0), "ECC affine uniform scale up");
         Assert (Aff_Near (RD, 0.98, 0.0, 0.0, 0.98, 0.0, 0.0), "ECC affine uniform scale down");
         --  Nonuniform: X and Y scales are independent (M00 /= M11), which a similarity
         --  or rigid model cannot express; swapping M00 and M11 fails here.
         Assert (Aff_Near (RA, 1.03, 0.0, 0.0, 0.97, 0.0, 0.0), "ECC affine nonuniform scale");
         Assert (Aff_Near (RT, 0.97, 0.0, 0.0, 1.03, 0.0, 0.0), "ECC affine nonuniform scale swapped axes");
         Assert (LF (RA.Warp.M00) - LF (RA.Warp.M11) > 0.04
                 and then LF (RT.Warp.M11) - LF (RT.Warp.M00) > 0.04,
                 "ECC affine scales were forced to be equal");
      end;
   end ECC_Affine_Rotation_And_Scale;

   procedure ECC_Affine_Shear_And_Combined (T : in out Fixture) is
      pragma Unreferenced (T);
      XP    : constant ECC_Pair := Aff_Pair (1.0, 0.012, 0.0, 1.0, 0.0, 0.0);
      XL    : constant ECC_Pair := Aff_Pair (1.0, 0.03, 0.0, 1.0, 0.0, 0.0);
      YN    : constant ECC_Pair := Aff_Pair (1.0, 0.0, -0.008, 1.0, 0.0, 0.0);
      Both  : constant ECC_Pair := Aff_Pair (1.0, 0.012, -0.008, 1.0, 0.0, 0.0);
      Mixed : constant ECC_Pair := Aff_Pair (1.01, 0.012, -0.008, 0.99, 2.0, 1.0);
      Neg   : constant ECC_Pair := Aff_Pair (0.99, -0.012, 0.008, 1.01, -2.0, -1.0);
      RX    : constant ECC_Affine_Result := Estimate_ECC_Affine (XP.Template, XP.Moving);
      RL    : constant ECC_Affine_Result := Estimate_ECC_Affine (XL.Template, XL.Moving);
      RY    : constant ECC_Affine_Result := Estimate_ECC_Affine (YN.Template, YN.Moving);
      RB    : constant ECC_Affine_Result := Estimate_ECC_Affine (Both.Template, Both.Moving);
      RM    : constant ECC_Affine_Result := Estimate_ECC_Affine (Mixed.Template, Mixed.Moving);
      RN    : constant ECC_Affine_Result := Estimate_ECC_Affine (Neg.Template, Neg.Moving);
      MX, MY : Long_Float;
      type Int_Point is record
         X, Y : Integer;
      end record;
      type Int_Points is array (Positive range <>) of Int_Point;
      Fixture_Points : constant Int_Points := [(20, 30), (60, 64), (100, 90), (45, 110)];
      Corner_Points  : constant Int_Points := [(0, 0), (127, 0), (0, 127), (64, 64)];
   begin
      --  Independent fixture check: moving (A p + t) reproduces template (p) at reference
      --  points, using the forward matrix only (no inverse code shared with the fixture).
      for P of Fixture_Points loop
         declare
            PX : constant Long_Float := Long_Float (P.X);
            PY : constant Long_Float := Long_Float (P.Y);
            QX : constant Long_Float := 1.01 * PX + 0.012 * PY + 2.0;
            QY : constant Long_Float := -0.008 * PX + 0.99 * PY + 1.0;
         begin
            Assert (abs (Sample (Mixed.Moving, QX, QY)
                         - Long_Float (OpenCV.Core.UInt8_Access.Get (Mixed.Template, P.Y, P.X))) < 2.5,
                    "affine fixture does not map template points into the moving image");
         end;
      end loop;

      Assert (Aff_Near (RX, 1.0, 0.012, 0.0, 1.0, 0.0, 0.0), "ECC affine X shear");
      Assert (Aff_Near (RL, 1.0, 0.03, 0.0, 1.0, 0.0, 0.0), "ECC affine larger X shear");
      Assert (Aff_Near (RY, 1.0, 0.0, -0.008, 1.0, 0.0, 0.0), "ECC affine negative Y shear");
      Assert (Aff_Near (RB, 1.0, 0.012, -0.008, 1.0, 0.0, 0.0), "ECC affine two-sided shear");
      --  Coefficient placement: X shear lives in M01 only, Y shear in M10 only, and their
      --  signs are preserved (a swap, transpose or negation fails one of these).
      Assert (RX.Warp.M01 > 0.008 and then abs (RX.Warp.M10) < 0.003, "X shear is not in M01");
      Assert (RL.Warp.M01 > 0.02 and then abs (RL.Warp.M10) < 0.003, "larger X shear is not in M01");
      Assert (RY.Warp.M10 < -0.005 and then abs (RY.Warp.M01) < 0.003, "Y shear is not in M10");
      Assert (RB.Warp.M01 > 0.008 and then RB.Warp.M10 < -0.005, "two-sided shear signs");

      Assert (Aff_Near (RM, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0), "ECC affine combined transform");
      Assert (Aff_Near (RN, 0.99, -0.012, 0.008, 1.01, -2.0, -1.0), "ECC affine combined negative transform");
      Assert (RM.Warp.M00 > RM.Warp.M11 and then RN.Warp.M00 < RN.Warp.M11,
              "ECC affine diagonal entries swapped");
      Assert (RM.Warp.M02 > RM.Warp.M12 and then RM.Warp.M12 > 0.0,
              "ECC affine translation entries swapped or negated");
      Assert (RM.Correlation > 0.98 and then RN.Correlation > 0.98, "ECC affine combined correlation");

      --  Matrix reconstruction from the returned plain values maps reference points.
      for P of Corner_Points loop
         declare
            PX : constant Long_Float := Long_Float (P.X);
            PY : constant Long_Float := Long_Float (P.Y);
         begin
            Aff_Map (RM.Warp, PX, PY, MX, MY);
            Assert (abs (MX - (1.01 * PX + 0.012 * PY + 2.0)) < 0.25
                    and then abs (MY - (-0.008 * PX + 0.99 * PY + 1.0)) < 0.25,
                    "reconstructed affine matrix misplaces a reference point");
         end;
      end loop;
   end ECC_Affine_Shear_And_Combined;

   procedure ECC_Affine_Versus_Euclidean (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair   : constant ECC_Pair := Aff_Pair (1.03, 0.03, -0.02, 0.97, 0.0, 0.0);
      Affine : constant ECC_Affine_Result := Estimate_ECC_Affine (Pair.Template, Pair.Moving);
      Before : constant Long_Float := Aff_Residual (Pair, (others => <>), False);
      After  : constant Long_Float := Aff_Residual (Pair, Affine.Warp, True);
      Rigid_Residual    : Long_Float := Long_Float'Last;
      Rigid_Correlation : Long_Float := -1.0;
   begin
      begin
         declare
            Rigid : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Pair.Template, Pair.Moving);
         begin
            Rigid_Residual := Aff_Residual (Pair, Euc_As_Warp (Rigid), True);
            Rigid_Correlation := Long_Float (Rigid.Correlation);
         end;
      exception
         when OpenCV.OpenCV_Error => null;  --  a rigid model that cannot fit is also a distinction
      end;
      Assert (Aff_Near (Affine, 1.03, 0.03, -0.02, 0.97, 0.0, 0.0), "ECC affine anisotropic shear fit");
      Assert (After < 1.0 and then After * 5.0 < Before, "ECC affine residual did not collapse");
      --  The Euclidean model cannot represent anisotropic scale plus shear: its residual
      --  stays several gray levels and its correlation is lower.
      Assert (Rigid_Residual > 3.0 and then Rigid_Residual > After * 4.0,
              "Euclidean model represents the affine fixture as well as the affine model");
      Assert (Long_Float (Affine.Correlation) > Rigid_Correlation + 0.02,
              "affine correlation is not higher than Euclidean");
      --  A rigid result always has M00 = M11 and M01 = -M10; the affine result does not.
      Assert (LF (Affine.Warp.M00) - LF (Affine.Warp.M11) > 0.04
              and then abs (LF (Affine.Warp.M01) + LF (Affine.Warp.M10)) > 0.005,
              "affine result is constrained like a rigid transform");
   end ECC_Affine_Versus_Euclidean;

   procedure ECC_Affine_Inverse_Warp (T : in out Fixture) is
      pragma Unreferenced (T);
      Cases : constant array (1 .. 4) of ECC_Affine_Warp :=
        [(M00 => 1.01, M01 => 0.012, M02 => 2.0, M10 => -0.008, M11 => 0.99, M12 => 1.0),
         (M00 => 1.0, M01 => 0.03, M02 => 0.0, M10 => 0.0, M11 => 1.0, M12 => 0.0),
         (M00 => 1.03, M01 => 0.0, M02 => 0.0, M10 => 0.0, M11 => 0.97, M12 => 0.0),
         (M00 => 1.0, M01 => 0.0, M02 => -2.0, M10 => 0.0, M11 => 1.0, M12 => -1.0)];
   begin
      for W of Cases loop
         declare
            Pair   : constant ECC_Pair := Aff_Pair
              (LF (W.M00), LF (W.M01), LF (W.M10), LF (W.M11), LF (W.M02), LF (W.M12));
            R      : constant ECC_Affine_Result := Estimate_ECC_Affine (Pair.Template, Pair.Moving);
            Before : constant Long_Float := Aff_Residual (Pair, (others => <>), False);
            After  : constant Long_Float := Aff_Residual (Pair, R.Warp, True);
            --  Negative controls: the same numbers used the wrong way around.
            Negated : constant ECC_Affine_Warp :=
              (M00 => R.Warp.M00, M01 => R.Warp.M01, M02 => -R.Warp.M02,
               M10 => R.Warp.M10, M11 => R.Warp.M11, M12 => -R.Warp.M12);
            Transposed : constant ECC_Affine_Warp :=
              (M00 => R.Warp.M00, M01 => R.Warp.M10, M02 => R.Warp.M02,
               M10 => R.Warp.M01, M11 => R.Warp.M11, M12 => R.Warp.M12);
         begin
            Assert (After < 1.0 and then After * 4.0 < Before,
                    "ECC affine WARP_INVERSE_MAP resampling did not reduce the residual");
            if abs (LF (W.M02)) + abs (LF (W.M12)) > 0.0 then
               Assert (Aff_Residual (Pair, Negated, True) > After * 3.0,
                       "negating the translation does not worsen alignment");
            end if;
            if abs (LF (W.M01)) > 0.0 then
               Assert (Aff_Residual (Pair, Transposed, True) > After * 1.5,
                       "transposing the linear part does not worsen alignment");
            end if;
         end;
      end loop;
   end ECC_Affine_Inverse_Warp;

   procedure ECC_Affine_Options_And_Repeatability (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair    : constant ECC_Pair := Aff_Pair (1.01, 0.012, -0.008, 0.99, 2.0, 1.0);
      Default : constant ECC_Affine_Result := Estimate_ECC_Affine (Pair.Template, Pair.Moving);
      One     : constant ECC_Affine_Result := Estimate_ECC_Affine
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 1, Epsilon => 1.0E-4));
      Three   : constant ECC_Affine_Result := Estimate_ECC_Affine
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 3, Epsilon => 1.0E-4));
      Tight   : constant ECC_Affine_Result := Estimate_ECC_Affine
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 100, Epsilon => 1.0E-8));
      Loose   : constant ECC_Affine_Result := Estimate_ECC_Affine
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 50, Epsilon => 0.1));
   begin
      Assert (Aff_Near (Default, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0), "ECC affine default options");
      Assert (Default = Estimate_ECC_Affine (Pair.Template, Pair.Moving), "ECC affine not repeatable");
      --  One iteration cannot reach the optimum: the maximum-iteration option is consumed.
      Assert (One /= Default and then One.Correlation < Default.Correlation - 0.1,
              "ECC affine iteration option ignored");
      Assert (abs (LF (One.Warp.M02) - 2.0) > 0.3, "one affine iteration reached the optimum");
      Assert (Aff_Near (Three, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0), "ECC affine three iterations");
      Assert (Aff_Near (Tight, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0)
              and then Aff_Near (Loose, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0),
              "ECC affine nondefault options lost accuracy");
      Assert (Tight /= Loose, "ECC affine epsilon option ignored");
      --  Correlation semantics: a finite intensity score that is not clamped by the binding.
      Assert (abs (Default.Correlation) <= 1.0001 and then Default.Correlation > 0.98,
              "ECC affine correlation outside the expected range");
   end ECC_Affine_Options_And_Repeatability;

   procedure ECC_Affine_Regions_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Saved, Contiguous : ECC_Affine_Result;
      Before_A, Before_B : Long_Long_Integer;
   begin
      declare
         Parent_A : constant OpenCV.Core.Mat := EF.Texture (160, 160);
         Parent_B : constant OpenCV.Core.Mat :=
           EF.Affine_Texture (160, 160, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0);
         Region_A : constant OpenCV.Core.Mat := Parent_A.Region (Window);
         Region_B : constant OpenCV.Core.Mat := Parent_B.Region (Window);
      begin
         Before_A := EF.Checksum (Parent_A);
         Before_B := EF.Checksum (Parent_B);
         Assert (not Region_A.Is_Continuous and then not Region_B.Is_Continuous,
                 "ECC affine fixture Regions are contiguous");
         Saved := Estimate_ECC_Affine (Region_A, Region_B);
         Contiguous := Estimate_ECC_Affine (Region_A.Clone, Region_B.Clone);
         Assert (Saved = Contiguous, "ECC affine Region differs from the contiguous clone");
         Assert (EF.Checksum (Parent_A) = Before_A and then EF.Checksum (Parent_B) = Before_B,
                 "ECC affine modified a Region parent");
         --  Template and moving may also share one allocation (same image for both roles).
         Assert (Aff_Near (Estimate_ECC_Affine (Parent_A, Parent_A), 1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
                 "ECC affine with one image in both roles");
      end;
      --  Parents and Regions are finalized; the result is a plain value.
      Assert (abs (LF (Saved.Warp.M00) - 1.01) < Aff_Linear_Tolerance
              and then abs (LF (Saved.Warp.M01) - 0.012) < Aff_Linear_Tolerance
              and then abs (LF (Saved.Warp.M10) + 0.008) < Aff_Linear_Tolerance
              and then abs (LF (Saved.Warp.M11) - 0.99) < Aff_Linear_Tolerance
              and then Saved.Correlation > 0.98, "ECC affine result after finalization");
   end ECC_Affine_Regions_Lifetime;

   procedure ECC_Affine_Other_Models_Unchanged (T : in out Fixture) is
      pragma Unreferenced (T);
      Shift  : constant ECC_Pair := Euc_Pair (0.0, 2.0, 1.0);
      Rigid  : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      Trans  : constant ECC_Translation_Result := Estimate_ECC_Translation (Shift.Template, Shift.Moving);
      Euc    : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Rigid.Template, Rigid.Moving);
      Aff    : constant ECC_Affine_Result := Estimate_ECC_Affine (Shift.Template, Shift.Moving);
   begin
      Assert (ECC_Near (Trans, 2, 1), "translation ECC regression");
      Assert (Euc_Near (Euc, 3.0, 2.0, 1.0), "Euclidean ECC regression");
      --  A pure shift fits the affine model with an identity linear part.
      Assert (Aff_Near (Aff, 1.0, 0.0, 0.0, 1.0, 2.0, 1.0), "affine pure-shift fit");
      Assert (abs (LF (Aff.Warp.M02) - LF (Trans.Displacement.X)) < 0.1
              and then abs (LF (Aff.Warp.M12) - LF (Trans.Displacement.Y)) < 0.1,
              "affine and translation models disagree on a pure shift");
   end ECC_Affine_Other_Models_Unchanged;

   procedure ECC_Affine_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Good : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Empty : OpenCV.Core.Mat;
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.UInt8, Channels => 3));
      function From_Bits is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      Bad_Epsilons : constant array (Positive range <>) of OpenCV.Float64_Value :=
        [0.0, -1.0E-4, 9.0E-9, 0.1000001, 1.0,
         From_Bits (16#7FF0_0000_0000_0000#), From_Bits (16#FFF0_0000_0000_0000#),
         From_Bits (16#7FF8_0000_0000_0000#)];
   begin
      Aff_Expect_Error (Empty, Good, Message => "empty template");
      Aff_Expect_Error (Good, Empty, Message => "empty moving");
      Aff_Expect_Preflight_Error (Float_Image, Float_Image, "Float32 images");
      Aff_Expect_Preflight_Error (Color, Color, "three-channel images");
      Aff_Expect_Preflight_Error (Good, EF.Texture (128, 129), "different columns");
      Aff_Expect_Preflight_Error (EF.Texture (129, 128), Good, "different rows");
      Aff_Expect_Error (Good, Float_Image, Message => "Float32 moving");
      Aff_Expect_Error (EF.Texture (31, 128), EF.Texture (31, 128), Message => "31 rows");
      Aff_Expect_Error (EF.Texture (128, 31), EF.Texture (128, 31), Message => "31 columns");
      declare
         Wide : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (32, 16_385, (Depth => OpenCV.Core.UInt8, Channels => 1));
         Area : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (4_097, 4_096, (Depth => OpenCV.Core.UInt8, Channels => 1));
      begin
         --  Rejected before native work: no ECC temporaries (six Jacobian blocks) are built.
         Aff_Expect_Preflight_Error (Wide, Wide, "16385 columns");
         Aff_Expect_Preflight_Error (Area, Area, "4097 x 4096 pixels");
      end;
      Aff_Expect_Error (Good, Good, (Maximum_Iterations => 101, Epsilon => 0.0001),
                        Message => "101 iterations");
      Aff_Expect_Error (Good, Good, (Maximum_Iterations => Positive'Last, Epsilon => 0.0001),
                        Message => "Positive'Last iterations");
      for Epsilon of Bad_Epsilons loop
         Aff_Expect_Error (Good, Good, (Maximum_Iterations => 50, Epsilon => Epsilon),
                           Message => "invalid epsilon");
      end loop;
      declare
         Pair  : constant ECC_Pair := Aff_Pair (1.01, 0.012, -0.008, 0.99, 2.0, 1.0);
         Lower : constant ECC_Affine_Result := Estimate_ECC_Affine
           (Pair.Template, Pair.Moving, (Maximum_Iterations => 1, Epsilon => 1.0E-8));
         Upper : constant ECC_Affine_Result := Estimate_ECC_Affine
           (Pair.Template, Pair.Moving, (Maximum_Iterations => 100, Epsilon => 0.1));
      begin
         Assert (Lower /= Estimate_ECC_Affine (Pair.Template, Pair.Moving), "lower option bound clamped");
         Assert (Aff_Near (Upper, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0), "ECC affine upper option bounds");
      end;
   end ECC_Affine_Validation;

   procedure ECC_Affine_Degenerate_And_Limits (T : in out Fixture) is
      pragma Unreferenced (T);
      Flat          : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
      Texture_Image : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Noise         : constant OpenCV.Core.Mat := EF.Noise (128, 128, 7);
      Reflected     : constant OpenCV.Core.Mat := EF.Affine_Texture (128, 128, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0);
      Strong        : constant OpenCV.Core.Mat := EF.Affine_Texture (128, 128, 1.2, 0.1, -0.1, 0.8, 5.0, -3.0);
      Sheared       : constant OpenCV.Core.Mat := EF.Affine_Texture (128, 128, 1.0, 0.2, 0.0, 1.0, 0.0, 0.0);
   begin
      Aff_Expect_Error (Flat, Flat, Message => "constant images");
      Aff_Expect_Error (Flat, Texture_Image, Message => "constant template");
      Aff_Expect_Error (Texture_Image, Flat, Message => "constant moving");
      Aff_Expect_Error (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9), Message => "uncorrelated noise");
      --  Reflection and large deformations lie outside the identity-initialized basin on every
      --  qualified version: native ECC raises and no identity is fabricated.
      Aff_Expect_Error (Texture_Image, Reflected, Message => "reflection outside the basin");
      Aff_Expect_Error (Texture_Image, Sheared, Message => "0.2 shear outside the basin");
      Aff_Expect_Error (Texture_Image, Strong, Message => "strong affine deformation");
      --  Finite is not correct: independent noise against a texture is accepted with a low
      --  correlation and a wrong (but finite, non-rigid) matrix, repeatably.
      declare
         R : constant ECC_Affine_Result := Estimate_ECC_Affine (Texture_Image, Noise);
      begin
         Assert (R.Correlation < 0.2, "ECC affine noise-vs-texture correlation");
         Assert (abs (LF (R.Warp.M02)) > 1.0 or else abs (LF (R.Warp.M00) - 1.0) > 0.05,
                 "noise-vs-texture was unexpectedly an identity fit");
         Assert (R = Estimate_ECC_Affine (Texture_Image, Noise), "ECC affine noise result not repeatable");
      exception
         when OpenCV.OpenCV_Error => null;  --  accepted alternative on another native version
      end;
   end ECC_Affine_Degenerate_And_Limits;

   ---------------------------------------------------------------------
   --  Task 014: independent affine oracle (ecc-affine-oracle 1)
   ---------------------------------------------------------------------
   function Aff_Native_Version return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_video_native_version";

   function Aff_Name (Index : Positive) return String is
   begin
      case Index is
         when 1 => return "identity";
         when 2 => return "translation-2-1";
         when 3 => return "translation-m2-m1";
         when 4 => return "rot+1";
         when 5 => return "rot-1";
         when 6 => return "rot+3";
         when 7 => return "rot-3";
         when 8 => return "iso-1.02";
         when 9 => return "iso-0.98";
         when 10 => return "aniso-1.01-0.99";
         when 11 => return "aniso-1.03-0.97";
         when 12 => return "shear-x+0.012";
         when 13 => return "shear-x+0.03";
         when 14 => return "shear-y-0.008";
         when 15 => return "shear-both";
         when 16 => return "combined";
         when 17 => return "combined-neg";
         when 18 => return "rot-scale-shear-t";
         when 19 => return "reflect-x";
         when 20 => return "shear-0.2";
         when 21 => return "strong-1.2-0.8";
         when 22 => return "reversed-combined";
         when 23 => return "photometric";
         when 24 => return "constant";
         when 25 => return "noise-vs-texture";
         when 26 => return "noise-vs-noise";
         when 27 => return "it1";
         when 28 => return "it3";
         when 29 => return "it100-e8";
         when 30 => return "e0.1";
         when 31 => return "e1e-8";
         when 32 => return "region-both";
         when others => raise Constraint_Error with "affine oracle fixture index";
      end case;
   end Aff_Name;

   --  Rebuilds a named oracle fixture independently of the C++ code.
   function Aff_Build (Name : String) return ECC_Pair is
      Pi     : constant Long_Float := 3.14159265358979323846;
      D2R    : constant Long_Float := Pi / 180.0;
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      function Tex return OpenCV.Core.Mat is (EF.Texture (128, 128));
      function A (P, Q, R, S, TX, TY : Long_Float) return OpenCV.Core.Mat is
        (EF.Affine_Texture (128, 128, P, Q, R, S, TX, TY));
      function Rot (Degrees : Long_Float) return OpenCV.Core.Mat is
         T : constant Long_Float := Degrees * Pi / 180.0;
      begin
         return A (LM.Cos (T), -LM.Sin (T), LM.Sin (T), LM.Cos (T), 0.0, 0.0);
      end Rot;
      function Comb return OpenCV.Core.Mat is (A (1.01, 0.012, -0.008, 0.99, 2.0, 1.0));
      function P (M : OpenCV.Core.Mat) return ECC_Pair is (Tex, M);
   begin
      if Name = "identity" then return (Tex, Tex);
      elsif Name = "translation-2-1" then return P (A (1.0, 0.0, 0.0, 1.0, 2.0, 1.0));
      elsif Name = "translation-m2-m1" then return P (A (1.0, 0.0, 0.0, 1.0, -2.0, -1.0));
      elsif Name = "rot+1" then return P (Rot (1.0));
      elsif Name = "rot-1" then return P (Rot (-1.0));
      elsif Name = "rot+3" then return P (Rot (3.0));
      elsif Name = "rot-3" then return P (Rot (-3.0));
      elsif Name = "iso-1.02" then return P (A (1.02, 0.0, 0.0, 1.02, 0.0, 0.0));
      elsif Name = "iso-0.98" then return P (A (0.98, 0.0, 0.0, 0.98, 0.0, 0.0));
      elsif Name = "aniso-1.01-0.99" then return P (A (1.01, 0.0, 0.0, 0.99, 0.0, 0.0));
      elsif Name = "aniso-1.03-0.97" then return P (A (1.03, 0.0, 0.0, 0.97, 0.0, 0.0));
      elsif Name = "shear-x+0.012" then return P (A (1.0, 0.012, 0.0, 1.0, 0.0, 0.0));
      elsif Name = "shear-x+0.03" then return P (A (1.0, 0.03, 0.0, 1.0, 0.0, 0.0));
      elsif Name = "shear-y-0.008" then return P (A (1.0, 0.0, -0.008, 1.0, 0.0, 0.0));
      elsif Name = "shear-both" then return P (A (1.0, 0.012, -0.008, 1.0, 0.0, 0.0));
      elsif Name = "combined" or else Name = "it1" or else Name = "it3" or else Name = "it100-e8"
        or else Name = "e0.1" or else Name = "e1e-8" then return P (Comb);
      elsif Name = "combined-neg" then return P (A (0.99, -0.012, 0.008, 1.01, -2.0, -1.0));
      elsif Name = "rot-scale-shear-t" then
         return P (A (1.02 * LM.Cos (2.0 * D2R), -0.98 * LM.Sin (2.0 * D2R) + 0.01,
                      1.02 * LM.Sin (2.0 * D2R), 0.98 * LM.Cos (2.0 * D2R), 1.5, -1.0));
      elsif Name = "reflect-x" then return P (A (-1.0, 0.0, 0.0, 1.0, 0.0, 0.0));
      elsif Name = "shear-0.2" then return P (A (1.0, 0.2, 0.0, 1.0, 0.0, 0.0));
      elsif Name = "strong-1.2-0.8" then return P (A (1.2, 0.1, -0.1, 0.8, 5.0, -3.0));
      elsif Name = "reversed-combined" then return (Comb, Tex);
      elsif Name = "photometric" then return P (EF.Affine (Comb, 4, 5, 20));
      elsif Name = "constant" then
         return (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128));
      elsif Name = "noise-vs-texture" then return P (EF.Noise (128, 128, 7));
      elsif Name = "noise-vs-noise" then return (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9));
      elsif Name = "region-both" then
         return (EF.Texture (160, 160).Region (Window),
                 EF.Affine_Texture (160, 160, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0).Region (Window));
      end if;
      raise Constraint_Error with "unknown affine oracle fixture " & Name;
   end Aff_Build;


   Expected_Aff_Oracle_Fixtures : constant := 32;

   type Aff_Line_Array is array (1 .. 64) of Ada.Strings.Unbounded.Unbounded_String;
   type Aff_Token_Array is array (1 .. 16) of Ada.Strings.Unbounded.Unbounded_String;

   function Aff_Oracle_Path return String is
     (Ada.Environment_Variables.Value ("VIDEO_ECC_AFFINE_ORACLE", "../obj/oracle/ecc-affine.txt"));

   procedure Aff_Read_Lines (Path : String; Lines : out Aff_Line_Array; Count : out Natural) is
      File : Ada.Text_IO.File_Type;
   begin
      Count := 0;
      Lines := [others => Ada.Strings.Unbounded.Null_Unbounded_String];
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (File) loop
         if Count = Lines'Last then
            Ada.Text_IO.Close (File);
            raise Constraint_Error with "oracle has too many lines";
         end if;
         Count := Count + 1;
         Lines (Count) := Ada.Strings.Unbounded.To_Unbounded_String (Ada.Text_IO.Get_Line (File));
      end loop;
      Ada.Text_IO.Close (File);
   end Aff_Read_Lines;

   procedure Aff_Write_Lines (Path : String; Lines : Aff_Line_Array; Count : Natural) is
      File : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Create (File, Ada.Text_IO.Out_File, Path);
      for I in 1 .. Count loop
         Ada.Text_IO.Put_Line (File, Ada.Strings.Unbounded.To_String (Lines (I)));
      end loop;
      Ada.Text_IO.Close (File);
   end Aff_Write_Lines;

   procedure Aff_Split (Line : String; Tokens : out Aff_Token_Array; Count : out Natural) is
      First : Natural := Line'First;
   begin
      Count := 0;
      Tokens := [others => Ada.Strings.Unbounded.Null_Unbounded_String];
      while First <= Line'Last loop
         if Line (First) = ' ' then
            First := First + 1;
         else
            declare
               Last : Natural := First;
            begin
               while Last < Line'Last and then Line (Last + 1) /= ' ' loop
                  Last := Last + 1;
               end loop;
               if Count = Tokens'Last then
                  Count := Tokens'Last + 1;
                  return;
               end if;
               Count := Count + 1;
               Tokens (Count) := Ada.Strings.Unbounded.To_Unbounded_String (Line (First .. Last));
               First := Last + 1;
            end;
         end if;
      end loop;
   end Aff_Split;

   function Aff_Tok (Tokens : Aff_Token_Array; Index : Positive) return String is
     (Ada.Strings.Unbounded.To_String (Tokens (Index)));

   --  Strict verifier. Returns the empty string when the oracle is complete, current and
   --  agrees with the binding; otherwise a description of the first defect. It never
   --  raises, so tampered copies can be asserted to be rejected.
   function Verify_Aff_Oracle (Path : String; Succeeded, Raised : out Natural) return String is
      Lines : Aff_Line_Array;
      N     : Natural;
      Tok   : Aff_Token_Array;
      TC    : Natural;
      Native : constant String := Interfaces.C.Strings.Value (Aff_Native_Version);
      function Line (I : Positive) return String is (Ada.Strings.Unbounded.To_String (Lines (I)));
   begin
      Succeeded := 0;
      Raised := 0;
      begin
         Aff_Read_Lines (Path, Lines, N);
      exception
         when others => return "oracle missing or unreadable";
      end;
      if N = 0 then return "oracle is empty"; end if;
      if Line (1) /= "ecc-affine-oracle 1" then return "oracle magic or format version"; end if;
      if N < 2 or else Line (2) /= "version " & Native then return "oracle OpenCV version differs"; end if;
      if N < 3 or else Ada.Strings.Fixed.Head (Line (N), 4) /= "end " then
         return "oracle truncated (no end record)";
      end if;
      if Line (N) /= "end" & Integer'Image (Expected_Aff_Oracle_Fixtures) then
         return "oracle record count";
      end if;
      if N - 3 /= Expected_Aff_Oracle_Fixtures then return "oracle fixture record count"; end if;

      for I in 1 .. Expected_Aff_Oracle_Fixtures loop
         Aff_Split (Line (I + 2), Tok, TC);
         if TC /= 15 or else Aff_Tok (Tok, 1) /= "fixture" then return "malformed fixture record"; end if;
         declare
            Name       : constant String := Aff_Name (I);
            Iterations : constant Integer := Integer'Value (Aff_Tok (Tok, 3));
            Epsilon    : constant Long_Float := Long_Float'Value (Aff_Tok (Tok, 4));
            Outcome    : constant String := Aff_Tok (Tok, 5);
            W          : array (0 .. 5) of Long_Float;
            Rho        : Long_Float;
            Det        : Long_Float;
            Pair       : constant ECC_Pair := Aff_Build (Name);
            Options    : constant ECC_Affine_Options :=
              (Maximum_Iterations => Iterations, Epsilon => OpenCV.Float64_Value (Epsilon));
         begin
            if Aff_Tok (Tok, 2) /= Name then return "fixture identity or order: " & Name; end if;
            if Outcome /= "OK" and then Outcome /= "EXC" then return "outcome token: " & Name; end if;
            for K in W'Range loop
               W (K) := Long_Float'Value (Aff_Tok (Tok, 6 + K));
            end loop;
            Rho := Long_Float'Value (Aff_Tok (Tok, 12));
            Det := Long_Float'Value (Aff_Tok (Tok, 13));
            if Long_Long_Integer'Value (Aff_Tok (Tok, 14)) /= EF.Checksum (Pair.Template)
              or else Long_Long_Integer'Value (Aff_Tok (Tok, 15)) /= EF.Checksum (Pair.Moving)
            then
               return "fixture checksum differs: " & Name;
            end if;
            if Outcome = "OK" then
               declare
                  R : ECC_Affine_Result;
               begin
                  begin
                     R := Estimate_ECC_Affine (Pair.Template, Pair.Moving, Options);
                  exception
                     when OpenCV.OpenCV_Error => return "binding raised where native succeeded: " & Name;
                  end;
                  Succeeded := Succeeded + 1;
                  if abs (LF (R.Warp.M00) - W (0)) > 1.0E-6 then return "M00 differs: " & Name; end if;
                  if abs (LF (R.Warp.M01) - W (1)) > 1.0E-6 then return "M01 differs: " & Name; end if;
                  if abs (LF (R.Warp.M02) - W (2)) > 1.0E-5 then return "M02 differs: " & Name; end if;
                  if abs (LF (R.Warp.M10) - W (3)) > 1.0E-6 then return "M10 differs: " & Name; end if;
                  if abs (LF (R.Warp.M11) - W (4)) > 1.0E-6 then return "M11 differs: " & Name; end if;
                  if abs (LF (R.Warp.M12) - W (5)) > 1.0E-5 then return "M12 differs: " & Name; end if;
                  if abs (Long_Float (R.Correlation) - Rho) > 1.0E-6 then
                     return "correlation differs: " & Name;
                  end if;
                  if abs (LF (R.Warp.M00) * LF (R.Warp.M11) - LF (R.Warp.M01) * LF (R.Warp.M10) - Det) > 1.0E-5 then
                     return "determinant differs: " & Name;
                  end if;
               end;
            else
               declare
                  R : ECC_Affine_Result;
               begin
                  R := Estimate_ECC_Affine (Pair.Template, Pair.Moving, Options);
                  return "binding succeeded where native raised: " & Name & Long_Float'Image (LF (R.Warp.M00));
               exception
                  when OpenCV.OpenCV_Error => Raised := Raised + 1;
               end;
            end if;
         end;
      end loop;
      return "";
   exception
      when others => return "oracle parse error";
   end Verify_Aff_Oracle;

   procedure ECC_Affine_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Succeeded, Raised : Natural;
      Defect : constant String := Verify_Aff_Oracle (Aff_Oracle_Path, Succeeded, Raised);
   begin
      Assert (Defect = "", "affine oracle: " & Defect);
      Assert (Succeeded > 0 and then Raised > 0, "affine oracle did not cover both outcomes");
      Assert (Succeeded + Raised = Expected_Aff_Oracle_Fixtures, "affine oracle outcome accounting");
   end ECC_Affine_Direct_Oracle;

   function Aff_With_Token
     (Lines : Aff_Line_Array; Line_Index, Token_Index : Positive; Value : String) return Aff_Line_Array
   is
      Result : Aff_Line_Array := Lines;
      Tok    : Aff_Token_Array;
      TC     : Natural;
      Text   : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Aff_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      Tok (Token_Index) := Ada.Strings.Unbounded.To_Unbounded_String (Value);
      for K in 1 .. TC loop
         if K > 1 then Ada.Strings.Unbounded.Append (Text, " "); end if;
         Ada.Strings.Unbounded.Append (Text, Tok (K));
      end loop;
      Result (Line_Index) := Text;
      return Result;
   end Aff_With_Token;

   function Aff_Swap_Tokens
     (Lines : Aff_Line_Array; Line_Index, A, B : Positive) return Aff_Line_Array
   is
      Tok : Aff_Token_Array;
      TC  : Natural;
   begin
      Aff_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      return Aff_With_Token
        (Aff_With_Token (Lines, Line_Index, A, Aff_Tok (Tok, B)), Line_Index, B, Aff_Tok (Tok, A));
   end Aff_Swap_Tokens;

   --  Fixture line index in the file: 2 header lines, then fixtures in table order.
   function Aff_Line_Of (Name : String) return Positive is
   begin
      for I in 1 .. Expected_Aff_Oracle_Fixtures loop
         if Aff_Name (I) = Name then return I + 2; end if;
      end loop;
      raise Constraint_Error with "no such affine oracle fixture " & Name;
   end Aff_Line_Of;

   procedure ECC_Affine_Oracle_Integrity (T : in out Fixture) is
      pragma Unreferenced (T);
      Original : constant String := Aff_Oracle_Path;
      Scratch  : constant String := Original & ".tampered";
      Lines    : Aff_Line_Array;
      Count    : Natural;
      Succeeded, Raised : Natural;

      procedure Expect_Rejected (Label, Fragment : String; Edited : Aff_Line_Array; Edited_Count : Natural) is
      begin
         Aff_Write_Lines (Scratch, Edited, Edited_Count);
         declare
            Defect : constant String := Verify_Aff_Oracle (Scratch, Succeeded, Raised);
         begin
            Assert (Defect /= "", "tampered affine oracle accepted: " & Label);
            Assert (Ada.Strings.Fixed.Index (Defect, Fragment) > 0,
                    "tampered affine oracle (" & Label & ") rejected for the wrong reason: " & Defect);
         end;
      end Expect_Rejected;

      function Value_Of (Line_Index, Token_Index : Positive) return Long_Float is
         Tok : Aff_Token_Array;
         TC  : Natural;
      begin
         Aff_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
         return Long_Float'Value (Aff_Tok (Tok, Token_Index));
      end Value_Of;
   begin
      Aff_Read_Lines (Original, Lines, Count);
      Assert (Count = Expected_Aff_Oracle_Fixtures + 3, "affine oracle line count");
      Assert (Verify_Aff_Oracle (Original, Succeeded, Raised) = "", "pristine affine oracle rejected");

      --  Missing and empty files.
      Assert (Verify_Aff_Oracle (Original & ".does-not-exist", Succeeded, Raised) = "oracle missing or unreadable",
              "missing affine oracle accepted");
      Expect_Rejected ("empty", "empty", Lines, 0);
      --  Truncated: end record dropped, and a half-written fixture record.
      Expect_Rejected ("truncated", "truncated", Lines, Count - 1);
      Expect_Rejected ("truncated mid-record", "truncated", Lines, Count - 5);
      --  Format and native version.
      Expect_Rejected ("wrong format version", "format version",
                       Aff_With_Token (Lines, 1, 2, "2"), Count);
      Expect_Rejected ("wrong OpenCV version", "OpenCV version",
                       Aff_With_Token (Lines, 2, 2, "0.0.0"), Count);
      --  Record count: end record altered, fixture record removed, extra record appended.
      Expect_Rejected ("wrong declared count", "record count",
                       Aff_With_Token (Lines, Count, 2, "31"), Count);
      declare
         Removed : Aff_Line_Array := Lines;
      begin
         for I in Aff_Line_Of ("it3") .. Count - 1 loop
            Removed (I) := Lines (I + 1);
         end loop;
         Expect_Rejected ("missing fixture record", "record count", Removed, Count - 1);
      end;
      --  Fixture identity and checksum.
      Expect_Rejected ("renamed fixture", "fixture identity",
                       Aff_With_Token (Lines, Aff_Line_Of ("combined"), 2, "combinedX"), Count);
      Expect_Rejected ("altered template checksum", "checksum",
                       Aff_With_Token (Lines, Aff_Line_Of ("combined"), 14, "1"), Count);
      Expect_Rejected ("altered moving checksum", "checksum",
                       Aff_With_Token (Lines, Aff_Line_Of ("shear-both"), 15, "1"), Count);
      --  Swapped coefficients (values differ, so a swap is observable).
      Assert (abs (Value_Of (Aff_Line_Of ("combined"), 7) - Value_Of (Aff_Line_Of ("combined"), 9)) > 0.01,
              "swap fixture M01/M10 must differ");
      Expect_Rejected ("swapped M01 and M10", "differs",
                       Aff_Swap_Tokens (Lines, Aff_Line_Of ("combined"), 7, 9), Count);
      Expect_Rejected ("swapped M02 and M12", "differs",
                       Aff_Swap_Tokens (Lines, Aff_Line_Of ("combined"), 8, 11), Count);
      Expect_Rejected ("swapped M00 and M11", "differs",
                       Aff_Swap_Tokens (Lines, Aff_Line_Of ("aniso-1.03-0.97"), 6, 10), Count);
      --  Altered translation and correlation.
      Expect_Rejected ("altered M02", "M02 differs",
                       Aff_With_Token (Lines, Aff_Line_Of ("translation-2-1"), 8, "3.0"), Count);
      Expect_Rejected ("altered M12", "M12 differs",
                       Aff_With_Token (Lines, Aff_Line_Of ("translation-2-1"), 11, "-1.0"), Count);
      Expect_Rejected ("altered correlation", "correlation differs",
                       Aff_With_Token (Lines, Aff_Line_Of ("combined"), 12, "0.5"), Count);
      Expect_Rejected ("altered determinant", "determinant differs",
                       Aff_With_Token (Lines, Aff_Line_Of ("combined"), 13, "2.0"), Count);
      --  Incorrect outcome classification, in both directions.
      Expect_Rejected ("success recorded as exception", "binding succeeded where native raised",
                       Aff_With_Token (Lines, Aff_Line_Of ("identity"), 5, "EXC"), Count);
      Expect_Rejected ("exception recorded as success", "binding raised where native succeeded",
                       Aff_With_Token (Lines, Aff_Line_Of ("reflect-x"), 5, "OK"), Count);
      Expect_Rejected ("unknown outcome token", "outcome token",
                       Aff_With_Token (Lines, Aff_Line_Of ("identity"), 5, "MAYBE"), Count);
      Ada.Directories.Delete_File (Scratch);
   end ECC_Affine_Oracle_Integrity;

   ---------------------------------------------------------------------
   --  Task 015: caller-seeded ECC affine alignment
   ---------------------------------------------------------------------
   --  Truth is row-major [A B TX; C D TY]: moving = A * template + t.
   type Sd_Truth is array (0 .. 5) of Long_Float;

   Sd_Identity_Truth : constant Sd_Truth := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
   Sd_Small_Truth    : constant Sd_Truth := [1.01, 0.012, 2.0, -0.008, 0.99, 1.0];
   Sd_Shear_Truth    : constant Sd_Truth := [1.0, 0.12, 1.0, -0.05, 1.0, -1.0];
   Sd_Scale_Truth    : constant Sd_Truth := [1.12, 0.02, 2.0, -0.01, 0.88, 1.0];
   Sd_Comb_Truth     : constant Sd_Truth := [1.08, 0.08, 2.0, -0.04, 0.94, -1.0];
   Sd_Shear2_Truth   : constant Sd_Truth := [1.0, 0.2, 0.0, 0.0, 1.0, 0.0];
   Sd_Strong_Truth   : constant Sd_Truth := [1.2, 0.1, 5.0, -0.1, 0.8, -3.0];
   Sd_Reflect_Truth  : constant Sd_Truth := [-1.0, 0.0, 127.0, 0.0, 1.0, 0.0];

   function Sd_Pair (T : Sd_Truth) return ECC_Pair is
     (Aff_Pair (T (0), T (1), T (3), T (4), T (2), T (5)));

   function Mk
     (M00, M01, M02, M10, M11, M12 : OpenCV.Float32_Value;
      Correlation : OpenCV.Float64_Value := 0.0) return ECC_Affine_Result is
     (Warp => (M00 => M00, M01 => M01, M02 => M02, M10 => M10, M11 => M11, M12 => M12),
      Correlation => Correlation);

   function F32_Of (Value : Long_Float) return OpenCV.Float32_Value is
     (OpenCV.Float32_Value (Value));

   function Sd_Seed_Of (T : Sd_Truth) return ECC_Affine_Result is
     (Mk (F32_Of (T (0)), F32_Of (T (1)), F32_Of (T (2)),
          F32_Of (T (3)), F32_Of (T (4)), F32_Of (T (5))));

   Sd_Identity_Seed : constant ECC_Affine_Result := Mk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
   Sd_Shear_Near    : constant ECC_Affine_Result := Mk (1.0, 0.10, 0.5, -0.04, 1.0, -0.7);
   Sd_Scale_Near    : constant ECC_Affine_Result := Mk (1.10, 0.01, 1.5, 0.0, 0.90, 0.8);
   Sd_Comb_Near     : constant ECC_Affine_Result := Mk (1.06, 0.07, 1.7, -0.03, 0.95, -0.8);
   Sd_Shear2_Near   : constant ECC_Affine_Result := Mk (1.0, 0.18, 0.0, 0.0, 1.0, 0.0);
   Sd_Strong_Near   : constant ECC_Affine_Result := Mk (1.18, 0.09, 4.5, -0.09, 0.82, -2.7);
   Sd_Reflect_Near  : constant ECC_Affine_Result := Mk (-0.98, 0.01, 126.0, 0.0, 1.0, 0.1);
   Sd_Wrong_Seed    : constant ECC_Affine_Result := Mk (0.8, -0.2, 6.0, 0.2, 1.25, -5.0);

   --  Largest absolute difference over the six coefficients (explicit metric, not a
   --  confidence measure).
   function Sd_Coef_Error (W : ECC_Affine_Warp; T : Sd_Truth) return Long_Float is
      function D (V : OpenCV.Float32_Value; K : Natural) return Long_Float is
        (abs (LF (V) - T (K)));
   begin
      return Long_Float'Max
        (Long_Float'Max (Long_Float'Max (D (W.M00, 0), D (W.M01, 1)),
                         Long_Float'Max (D (W.M02, 2), D (W.M10, 3))),
         Long_Float'Max (D (W.M11, 4), D (W.M12, 5)));
   end Sd_Coef_Error;

   function Sd_Near (R : ECC_Affine_Result; T : Sd_Truth) return Boolean is
     (Aff_Near (R, T (0), T (1), T (3), T (4), T (2), T (5)));

   --  Interior mean absolute difference with border-clamped bilinear sampling: seeds may map
   --  interior pixels outside the image (native uses BORDER_REFLECT_101; clamping is only a
   --  test metric and never feeds the binding).
   function Sd_Residual (Pair : ECC_Pair; W : ECC_Affine_Warp; Aligned : Boolean) return Long_Float is
      Margin : constant := 24;
      Sum    : Long_Float := 0.0;
      Count  : Natural := 0;
      MX, MY : Long_Float;
      function Clamp (V : Long_Float) return Long_Float is
        (Long_Float'Max (0.0, Long_Float'Min (126.0, V)));
   begin
      for Y in Margin .. 127 - Margin loop
         for X in Margin .. 127 - Margin loop
            if Aligned then
               Aff_Map (W, Long_Float (X), Long_Float (Y), MX, MY);
            else
               MX := Long_Float (X);
               MY := Long_Float (Y);
            end if;
            Sum := Sum + abs (Sample (Pair.Moving, Clamp (MX), Clamp (MY))
                              - Long_Float (OpenCV.Core.UInt8_Access.Get (Pair.Template, Y, X)));
            Count := Count + 1;
         end loop;
      end loop;
      return Sum / Long_Float (Count);
   end Sd_Residual;

   procedure Sd_Run
     (Pair : ECC_Pair; Initial : ECC_Affine_Result;
      Result : out ECC_Affine_Result; Raised : out Boolean;
      Options : ECC_Affine_Options := (others => <>)) is
   begin
      Result := Estimate_ECC_Affine (Pair.Template, Pair.Moving, Options, Initial);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Sd_Run;

   procedure Sd_Run_Plain
     (Pair : ECC_Pair; Result : out ECC_Affine_Result; Raised : out Boolean) is
   begin
      Result := Estimate_ECC_Affine (Pair.Template, Pair.Moving);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Sd_Run_Plain;

   type Sd_Outcome is (Accepted, Preflight_Rejected, Native_Failed);

   --  Native-layer failures carry the "Video.Estimate_ECC_Affine" prefix (C.Check);
   --  Ada preflight rejections do not.
   function Sd_Outcome_Of
     (Pair : ECC_Pair; Initial : ECC_Affine_Result;
      Options : ECC_Affine_Options := (others => <>)) return Sd_Outcome
   is
      Marker : constant String := "Video.Estimate_ECC_Affine";
      Unused : ECC_Affine_Result;
   begin
      Unused := Estimate_ECC_Affine (Pair.Template, Pair.Moving, Options, Initial);
      return Accepted;
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text : constant String := Ada.Exceptions.Exception_Message (Error);
         begin
            return (if Text'Length >= Marker'Length
                      and then Text (Text'First .. Text'First + Marker'Length - 1) = Marker
                    then Native_Failed else Preflight_Rejected);
         end;
   end Sd_Outcome_Of;

   procedure ECC_Affine_Seeded_Identity_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      Pi   : constant Long_Float := 3.14159265358979323846;
      Rot3 : constant Sd_Truth :=
        [LM.Cos (3.0 * Pi / 180.0), -LM.Sin (3.0 * Pi / 180.0), 0.0,
         LM.Sin (3.0 * Pi / 180.0), LM.Cos (3.0 * Pi / 180.0), 0.0];
      Pairs : constant array (1 .. 11) of ECC_Pair :=
        [Sd_Pair (Sd_Identity_Truth), Sd_Pair ([1.0, 0.0, 2.0, 0.0, 1.0, 1.0]),
         Sd_Pair (Sd_Small_Truth), Sd_Pair (Rot3), Sd_Pair ([1.02, 0.0, 0.0, 0.0, 1.02, 0.0]),
         Sd_Pair ([1.0, 0.03, 0.0, 0.0, 1.0, 0.0]), Sd_Pair (Sd_Strong_Truth),
         Sd_Pair (Sd_Shear2_Truth), Sd_Pair (Sd_Reflect_Truth),
         (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128)),
         (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9))];
      With_Correlation : constant ECC_Affine_Result := Mk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.7);
      Successes, Failures : Natural := 0;
   begin
      for Pair of Pairs loop
         declare
            Plain, Seeded, Defaulted, Correlated : ECC_Affine_Result;
            Plain_Raised, Seeded_Raised, Defaulted_Raised, Correlated_Raised : Boolean;
         begin
            Sd_Run_Plain (Pair, Plain, Plain_Raised);
            Sd_Run (Pair, Sd_Identity_Seed, Seeded, Seeded_Raised);
            Sd_Run (Pair, (others => <>), Defaulted, Defaulted_Raised);
            Sd_Run (Pair, With_Correlation, Correlated, Correlated_Raised);
            Assert (Plain_Raised = Seeded_Raised and then Plain_Raised = Defaulted_Raised
                    and then Plain_Raised = Correlated_Raised,
                    "identity seed changed native success or failure");
            if Plain_Raised then
               Failures := Failures + 1;
            else
               Successes := Successes + 1;
               Assert (Plain = Seeded and then Plain = Defaulted and then Plain = Correlated,
                       "identity seed result differs from the identity-initialized operation");
            end if;
         end;
      end loop;
      Assert (Successes >= 5 and then Failures >= 3, "identity-equivalence coverage");
   end ECC_Affine_Seeded_Identity_Equivalence;

   procedure ECC_Affine_Seeded_Consumption (T : in out Fixture) is
      pragma Unreferenced (T);
      type Case_Data is record
         Truth          : Sd_Truth;
         Seed           : ECC_Affine_Result;
         Identity_Fails : Boolean;
         Coef_Improves  : Boolean;
      end record;
      Cases : constant array (1 .. 7) of Case_Data :=
        [(Sd_Shear_Truth, Sd_Shear_Near, True, True),
         (Sd_Shear2_Truth, Sd_Shear2_Near, True, False),
         (Sd_Reflect_Truth, Sd_Reflect_Near, True, True),
         (Sd_Scale_Truth, Sd_Scale_Near, True, True),
         (Sd_Comb_Truth, Sd_Comb_Near, True, True),
         (Sd_Strong_Truth, Sd_Strong_Near, True, True),
         (Sd_Small_Truth, Mk (1.0, 0.01, 1.5, -0.005, 1.0, 0.5), False, False)];
      Distinguishing : Natural := 0;
   begin
      for Item of Cases loop
         declare
            Pair : constant ECC_Pair := Sd_Pair (Item.Truth);
            Plain, Refined, Exact : ECC_Affine_Result;
            Plain_Raised, Refined_Raised, Exact_Raised : Boolean;
            Before : constant Long_Float := Sd_Residual (Pair, Item.Seed.Warp, True);
         begin
            Sd_Run_Plain (Pair, Plain, Plain_Raised);
            Sd_Run (Pair, Item.Seed, Refined, Refined_Raised);
            Assert (not Refined_Raised, "near-correct seed did not converge");
            Assert (Sd_Near (Refined, Item.Truth), "seeded result is not near the known transform");
            Assert (Refined.Warp /= Item.Seed.Warp, "seeded result is the unmodified seed");
            Assert (Sd_Residual (Pair, Refined.Warp, True) < Before * 0.5,
                    "refinement did not reduce the image residual");
            --  Native correlation is not clamped: Float32 rounding can exceed 1.0 slightly
            --  (measured 1.000000085 for the exact reflection).
            Assert (Refined.Correlation > 0.9 and then Refined.Correlation <= 1.000001,
                    "seeded correlation");
            if Item.Coef_Improves then
               Assert (Sd_Coef_Error (Refined.Warp, Item.Truth)
                         < Sd_Coef_Error (Item.Seed.Warp, Item.Truth),
                       "refinement did not reduce the coefficient error");
            end if;
            if Item.Identity_Fails then
               Assert (Plain_Raised, "identity initialization unexpectedly converged");
               Distinguishing := Distinguishing + 1;
            end if;
            Sd_Run (Pair, Sd_Seed_Of (Item.Truth), Exact, Exact_Raised);
            Assert (not Exact_Raised and then Sd_Near (Exact, Item.Truth), "exact seed");
         end;
      end loop;
      Assert (Distinguishing >= 6, "too few identity-fails / seeded-succeeds fixtures");
      Assert (Sd_Outcome_Of (Sd_Pair (Sd_Shear_Truth), Sd_Wrong_Seed) = Native_Failed,
              "a grossly wrong seed must raise, not be fabricated");
   end ECC_Affine_Seeded_Consumption;

   procedure ECC_Affine_Seeded_Refinement_And_Direction (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair : constant ECC_Pair := Sd_Pair (Sd_Comb_Truth);
      One, Many, Three : ECC_Affine_Result;
      One_Raised, Many_Raised, Three_Raised : Boolean;
      Unaligned : constant Long_Float := Sd_Residual (Pair, Sd_Identity_Seed.Warp, False);
   begin
      Sd_Run (Pair, Sd_Comb_Near, One, One_Raised, (Maximum_Iterations => 1, Epsilon => 0.0001));
      Sd_Run (Pair, Sd_Comb_Near, Three, Three_Raised, (Maximum_Iterations => 3, Epsilon => 0.0001));
      Sd_Run (Pair, Sd_Comb_Near, Many, Many_Raised);
      Assert (not One_Raised and then not Three_Raised and then not Many_Raised, "refinement runs");
      Assert (One /= Many and then One /= Three and then Three /= Many,
              "one, three and many iterations are indistinguishable");
      Assert (Sd_Coef_Error (Many.Warp, Sd_Comb_Truth) < Sd_Coef_Error (One.Warp, Sd_Comb_Truth),
              "more iterations did not move the matrix closer to the truth");
      Assert (One.Correlation < Many.Correlation, "one-iteration correlation");
      Assert (Estimate_ECC_Affine (Pair.Template, Pair.Moving, (others => <>), Sd_Comb_Near) = Many,
              "seeded affine not repeatable");

      --  Direction: the returned matrix is applied unchanged with WARP_INVERSE_MAP.
      declare
         W : constant ECC_Affine_Warp := Many.Warp;
         Negated    : constant ECC_Affine_Warp := (W with delta M02 => -W.M02, M12 => -W.M12);
         Transposed : constant ECC_Affine_Warp := (W with delta M01 => W.M10, M10 => W.M01);
         Aligned    : constant Long_Float := Sd_Residual (Pair, W, True);
      begin
         Assert (Aligned < Unaligned * 0.1, "unmodified matrix does not align the moving image");
         Assert (Sd_Residual (Pair, Negated, True) > Aligned * 5.0, "negated translation aligns");
         Assert (Sd_Residual (Pair, Transposed, True) > Aligned * 3.0, "transposed matrix aligns");
         Assert (abs (W.M01 - W.M10) > 0.05 and then abs (W.M02 - W.M12) > 1.0,
                 "direction fixture is symmetric");
      end;

      --  Storage order: seeds with permuted or negated slots must not reproduce the
      --  one-iteration estimate of the correct seed.
      declare
         S : ECC_Affine_Warp renames Sd_Comb_Near.Warp;
         Variants : constant array (1 .. 6) of ECC_Affine_Warp :=
           [(S with delta M01 => S.M10, M10 => S.M01),
            (S with delta M02 => S.M12, M12 => S.M02),
            (S with delta M00 => S.M11, M11 => S.M00),
            (S with delta M01 => -S.M01),
            (S with delta M02 => -S.M02),
            (S with delta M10 => -S.M10)];
      begin
         for V of Variants loop
            declare
               Mutated : ECC_Affine_Result;
               Mutated_Raised : Boolean;
            begin
               Sd_Run (Pair, (Warp => V, Correlation => 0.0), Mutated, Mutated_Raised,
                       (Maximum_Iterations => 1, Epsilon => 0.0001));
               Assert (Mutated_Raised or else Mutated.Warp /= One.Warp,
                       "permuted seed produced the same one-iteration estimate");
               Assert (Mutated_Raised or else Sd_Coef_Error (Mutated.Warp, Sd_Comb_Truth)
                         > Sd_Coef_Error (One.Warp, Sd_Comb_Truth),
                       "permuted seed was not worse than the correct seed");
            end;
         end loop;
      end;
   end ECC_Affine_Seeded_Refinement_And_Direction;

   procedure ECC_Affine_Seeded_Correlation_Ignored (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Validity_Checks (Off);
      function F64 is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      Pair : constant ECC_Pair := Sd_Pair (Sd_Comb_Truth);
      Base : ECC_Affine_Result;
      Base_Raised : Boolean;
      Correlations : constant array (1 .. 7) of OpenCV.Float64_Value :=
        [0.0, 1.0, -1.0, 12345.0, OpenCV.Float64_Value'Last,
         F64 (16#7FF8_0000_0000_0000#), F64 (16#FFF0_0000_0000_0000#)];
   begin
      Sd_Run (Pair, Sd_Comb_Near, Base, Base_Raised);
      Assert (not Base_Raised, "baseline seeded run");
      for C of Correlations loop
         declare
            Initial : constant ECC_Affine_Result := (Sd_Comb_Near with delta Correlation => C);
            Saved   : constant ECC_Affine_Result := Initial;
            Again   : ECC_Affine_Result;
            Raised  : Boolean;
         begin
            Sd_Run (Pair, Initial, Again, Raised);
            Assert (not Raised and then Again = Base, "initial correlation changed the result");
            Assert (Again.Warp = Base.Warp and then Again.Correlation = Base.Correlation,
                    "initial correlation leaked into the result");
            Assert (Initial.Warp = Saved.Warp, "initial warp modified");
         end;
      end loop;
      --  A previous result is a valid seed for the next call, and chaining is repeatable.
      declare
         First  : constant ECC_Affine_Result := Estimate_ECC_Affine
           (Pair.Template, Pair.Moving, (others => <>), Sd_Comb_Near);
         Second : constant ECC_Affine_Result := Estimate_ECC_Affine
           (Pair.Template, Pair.Moving, (others => <>), First);
         Third  : constant ECC_Affine_Result := Estimate_ECC_Affine
           (Pair.Template, Pair.Moving, (others => <>), First);
      begin
         Assert (Sd_Near (Second, Sd_Comb_Truth) and then Second = Third,
                 "previous result reused as a seed");
      end;
   end ECC_Affine_Seeded_Correlation_Ignored;

   procedure ECC_Affine_Seeded_Immutability_And_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Saved, Contiguous : ECC_Affine_Result;
      Before_A, Before_B : Long_Long_Integer;
      Initial : constant ECC_Affine_Result := Mk (1.0, 0.01, 1.5, 0.0, 1.0, 0.5, 0.25);
      Snapshot : constant ECC_Affine_Result := Initial;
      Unused : ECC_Affine_Result;
      Raised : Boolean;
   begin
      declare
         Parent_A : constant OpenCV.Core.Mat := EF.Texture (160, 160);
         Parent_B : constant OpenCV.Core.Mat :=
           EF.Affine_Texture (160, 160, 1.01, 0.012, -0.008, 0.99, 2.0, 1.0);
         Region_A : constant OpenCV.Core.Mat := Parent_A.Region (Window);
         Region_B : constant OpenCV.Core.Mat := Parent_B.Region (Window);
      begin
         Before_A := EF.Checksum (Parent_A);
         Before_B := EF.Checksum (Parent_B);
         Assert (not Region_A.Is_Continuous and then not Region_B.Is_Continuous,
                 "seeded affine fixture Regions are contiguous");
         Saved := Estimate_ECC_Affine (Region_A, Region_B, (others => <>), Initial);
         Contiguous := Estimate_ECC_Affine (Region_A.Clone, Region_B.Clone, (others => <>), Initial);
         Assert (Saved = Contiguous, "seeded Region differs from the contiguous clone");
         Assert (EF.Checksum (Parent_A) = Before_A and then EF.Checksum (Parent_B) = Before_B,
                 "seeded affine modified a Region parent");
         --  One allocation in both roles with a nonidentity seed: source stays unchanged.
         Sd_Run ((Parent_A, Parent_A), Initial, Unused, Raised);
         Assert (EF.Checksum (Parent_A) = Before_A, "shared allocation modified");
      end;
      --  The window shifts the translation by (A - I) * 16; only the linear part is compared.
      Assert (abs (LF (Saved.Warp.M00) - 1.01) < Aff_Linear_Tolerance
              and then abs (LF (Saved.Warp.M01) - 0.012) < Aff_Linear_Tolerance
              and then abs (LF (Saved.Warp.M10) + 0.008) < Aff_Linear_Tolerance
              and then abs (LF (Saved.Warp.M11) - 0.99) < Aff_Linear_Tolerance
              and then Saved.Correlation > 0.98, "seeded result after parent finalization");
      Assert (Initial = Snapshot and then Initial.Correlation = 0.25, "initial record changed");

      --  The initial record is untouched by success, nonconvergence and every rejection.
      declare
         Pair  : constant ECC_Pair := Sd_Pair (Sd_Shear_Truth);
         Flat  : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
         Seeds : constant ECC_Affine_Result := Mk (1.0, 0.10, 0.5, -0.04, 1.0, -0.7, 0.5);
         Copy  : constant ECC_Affine_Result := Seeds;
         Bad   : constant ECC_Affine_Result := Mk (9.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5);
         Bad_Copy : constant ECC_Affine_Result := Bad;
      begin
         Sd_Run (Pair, Seeds, Unused, Raised);
         Assert (not Raised and then Seeds = Copy, "initial record changed by refinement");
         Sd_Run ((Flat, Flat), Seeds, Unused, Raised);
         Assert (Raised and then Seeds = Copy, "initial record changed by nonconvergence");
         Sd_Run (Pair, Seeds, Unused, Raised, (Maximum_Iterations => 101, Epsilon => 0.0001));
         Assert (Raised and then Seeds = Copy, "initial record changed by invalid options");
         Sd_Run (Pair, Bad, Unused, Raised);
         Assert (Raised and then Bad = Bad_Copy, "initial record changed by invalid seed");
         Sd_Run ((Pair.Template, EF.Texture (128, 129)), Seeds, Unused, Raised);
         Assert (Raised and then Seeds = Copy, "initial record changed by invalid images");
      end;
   end ECC_Affine_Seeded_Immutability_And_Regions;

   procedure ECC_Affine_Seeded_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Validity_Checks (Off);
      function F32 is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, OpenCV.Float32_Value);
      NaN : constant OpenCV.Float32_Value := F32 (16#7FC0_0000#);
      Inf : constant OpenCV.Float32_Value := F32 (16#7F80_0000#);
      Lin : constant OpenCV.Float32_Value := 4.0;
      Trn : constant OpenCV.Float32_Value := 1_048_576.0;
      Pair : constant ECC_Pair := Sd_Pair (Sd_Small_Truth);
      Base : constant ECC_Affine_Warp := Sd_Comb_Near.Warp;
      Nonfinite : constant array (1 .. 3) of OpenCV.Float32_Value := [NaN, Inf, -Inf];

      procedure Rejected (W : ECC_Affine_Warp; Message : String) is
      begin
         Assert (Sd_Outcome_Of (Pair, (Warp => W, Correlation => 0.0)) = Preflight_Rejected, Message);
      end Rejected;

      procedure Admitted (W : ECC_Affine_Warp; Message : String) is
      begin
         Assert (Sd_Outcome_Of (Pair, (Warp => W, Correlation => 0.0)) /= Preflight_Rejected, Message);
      end Admitted;
   begin
      for Bad of Nonfinite loop
         Rejected ((Base with delta M00 => Bad), "nonfinite M00");
         Rejected ((Base with delta M01 => Bad), "nonfinite M01");
         Rejected ((Base with delta M02 => Bad), "nonfinite M02");
         Rejected ((Base with delta M10 => Bad), "nonfinite M10");
         Rejected ((Base with delta M11 => Bad), "nonfinite M11");
         Rejected ((Base with delta M12 => Bad), "nonfinite M12");
      end loop;
      Rejected ((Base with delta M00 => OpenCV.Float32_Value'Succ (Lin)), "M00 just beyond 4");
      Rejected ((Base with delta M01 => -OpenCV.Float32_Value'Succ (Lin)), "M01 just beyond -4");
      Rejected ((Base with delta M10 => 5.0), "M10 beyond 4");
      Rejected ((Base with delta M11 => OpenCV.Float32_Value'Last), "M11 at Float32'Last");
      Rejected ((Base with delta M02 => OpenCV.Float32_Value'Succ (Trn)), "M02 just beyond 2**20");
      Rejected ((Base with delta M12 => -OpenCV.Float32_Value'Succ (Trn)), "M12 just beyond -2**20");
      Rejected ((Base with delta M02 => 1.0E9), "M02 at 1e9");
      Rejected ((Base with delta M12 => OpenCV.Float32_Value'First), "M12 at Float32'First");
      --  Exact limits and singular / reflected / near-singular seeds pass preflight; their
      --  native outcome is compared with the direct oracle, not asserted here.
      Admitted ((M00 => Lin, M01 => Lin, M02 => Trn, M10 => -Lin, M11 => -Lin, M12 => -Trn),
                "seed exactly at all limits");
      Admitted ((M00 => -Lin, M01 => 0.0, M02 => 0.0, M10 => 0.0, M11 => Lin, M12 => 0.0),
                "reflected seed at the linear limit");
      Admitted ((M00 => 0.0, M01 => 0.0, M02 => 0.0, M10 => 0.0, M11 => 0.0, M12 => 0.0),
                "all-zero seed passes preflight");
      Admitted ((M00 => 1.0, M01 => 1.0, M02 => 0.0, M10 => 1.0, M11 => 1.0, M12 => 0.0),
                "singular seed passes preflight");
      Admitted ((M00 => 1.0, M01 => 1.0, M02 => 0.0, M10 => 1.0, M11 => 1.0000001, M12 => 0.0),
                "near-singular seed passes preflight");
   end ECC_Affine_Seeded_Validation;

   procedure ECC_Affine_Seeded_Image_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Base  : constant ECC_Affine_Result := (Warp => Sd_Comb_Near.Warp, Correlation => 0.0);
      Pair  : constant ECC_Pair := Sd_Pair (Sd_Small_Truth);
      Good  : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.UInt8, Channels => 3));
      Empty : OpenCV.Core.Mat;
      Small : constant OpenCV.Core.Mat := EF.Texture (31, 128);
      Unused : ECC_Affine_Result;
   begin
      Assert (Sd_Outcome_Of (Pair, Base, (Maximum_Iterations => 101, Epsilon => 0.0001))
              = Preflight_Rejected, "101 iterations");
      Assert (Sd_Outcome_Of (Pair, Base, (Maximum_Iterations => 50, Epsilon => 1.0))
              = Preflight_Rejected, "epsilon 1.0");
      Assert (Sd_Outcome_Of (Pair, Base, (Maximum_Iterations => 50, Epsilon => 0.0))
              = Preflight_Rejected, "epsilon 0.0");
      Assert (Sd_Outcome_Of ((Float_Image, Float_Image), Base) = Preflight_Rejected, "Float32 images");
      Assert (Sd_Outcome_Of ((Color, Color), Base) = Preflight_Rejected, "three-channel images");
      Assert (Sd_Outcome_Of ((Good, EF.Texture (128, 129)), Base) = Preflight_Rejected, "columns");
      Assert (Sd_Outcome_Of ((EF.Texture (129, 128), Good), Base) = Preflight_Rejected, "rows");
      Assert (Sd_Outcome_Of ((Small, Small), Base) = Preflight_Rejected, "31 rows");
      begin
         Unused := Estimate_ECC_Affine (Empty, Good, (others => <>), Base);
         Assert (False, "empty template accepted");
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      Assert (Sd_Outcome_Of (Pair, Base, (Maximum_Iterations => 1, Epsilon => 1.0E-8)) /= Preflight_Rejected
              and then Sd_Outcome_Of (Pair, Base, (Maximum_Iterations => 100, Epsilon => 0.1))
                       /= Preflight_Rejected, "option bounds are accepted");
   end ECC_Affine_Seeded_Image_Validation;

   procedure ECC_Affine_Seeded_Nonconvergence (T : in out Fixture) is
      pragma Unreferenced (T);
      Flat          : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
      Texture_Image : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Seed          : constant ECC_Affine_Result := Mk (1.01, 0.01, 1.0, 0.0, 0.99, 1.0);
   begin
      Assert (Sd_Outcome_Of ((Flat, Flat), Seed) = Native_Failed, "constant images");
      Assert (Sd_Outcome_Of ((Flat, Texture_Image), Seed) = Native_Failed, "constant template");
      --  Measured: a NONIDENTITY seed on a constant moving image does not raise (identity
      --  initialization does). The finite result is meaningless and has near-zero correlation.
      declare
         Flat_Result : ECC_Affine_Result;
         Flat_Raised : Boolean;
      begin
         Sd_Run ((Texture_Image, Flat), Seed, Flat_Result, Flat_Raised);
         Assert (Flat_Raised or else Flat_Result.Correlation < 0.1,
                 "constant moving image produced a confident result");
      end;
      Assert (Sd_Outcome_Of ((EF.Noise (128, 128, 3), EF.Noise (128, 128, 9)), Seed) = Native_Failed,
              "independent noise");
      Assert (Sd_Outcome_Of (Sd_Pair (Sd_Shear_Truth), Sd_Wrong_Seed) = Native_Failed, "wrong seed");
      --  A finite native result need not be correct: this poor seed ends far from the truth
      --  (or raises); nothing is corrected or fabricated by the binding.
      declare
         Pair   : constant ECC_Pair := Sd_Pair (Sd_Scale_Truth);
         R      : ECC_Affine_Result;
         Raised : Boolean;
         Asym   : constant ECC_Affine_Result := Mk (1.0, 0.3, 4.0, -0.25, 1.0, -2.0);
      begin
         Sd_Run (Pair, Asym, R, Raised);
         Assert (Raised or else not Sd_Near (R, Sd_Scale_Truth), "wrong-but-finite outcome");
      end;
   end ECC_Affine_Seeded_Nonconvergence;

   ---------------------------------------------------------------------
   --  Task 015: independent seeded affine oracle (ecc-affine-seeded-oracle 1)
   ---------------------------------------------------------------------
   type Sd_Token_Array is array (1 .. 24) of Ada.Strings.Unbounded.Unbounded_String;

   type Sd_Spec is record
      Name  : String (1 .. 20);
      Kind  : Character;
      Truth : Sd_Truth;
      Seed  : ECC_Affine_Warp;
      Iterations : Positive;
      Epsilon    : Long_Float;
   end record;

   function Sp
     (Name : String; Kind : Character; Truth : Sd_Truth; Seed : ECC_Affine_Result;
      Iterations : Positive := 50; Epsilon : Long_Float := 1.0E-4) return Sd_Spec is
     (Name => Ada.Strings.Fixed.Head (Name, 20), Kind => Kind, Truth => Truth,
      Seed => Seed.Warp, Iterations => Iterations, Epsilon => Epsilon);

   function Z (A, B, C, D, E, F : OpenCV.Float32_Value) return ECC_Affine_Result is
     (Mk (A, B, C, D, E, F));

   Expected_Sd_Oracle_Fixtures : constant := 34;

   Sd_Table : constant array (1 .. Expected_Sd_Oracle_Fixtures) of Sd_Spec :=
     [Sp ("identity-id", 'T', Sd_Identity_Truth, Sd_Identity_Seed),
      Sp ("translation-id", 'T', [1.0, 0.0, 2.0, 0.0, 1.0, 1.0], Sd_Identity_Seed),
      Sp ("small-id", 'T', Sd_Small_Truth, Sd_Identity_Seed),
      Sp ("shear-id", 'T', Sd_Shear_Truth, Sd_Identity_Seed),
      Sp ("shear02-id", 'T', Sd_Shear2_Truth, Sd_Identity_Seed),
      Sp ("reflect-id", 'T', Sd_Reflect_Truth, Sd_Identity_Seed),
      Sp ("shear-exact", 'T', Sd_Shear_Truth, Z (1.0, 0.12, 1.0, -0.05, 1.0, -1.0)),
      Sp ("shear-near", 'T', Sd_Shear_Truth, Sd_Shear_Near),
      Sp ("scale-near", 'T', Sd_Scale_Truth, Sd_Scale_Near),
      Sp ("combined-near", 'T', Sd_Comb_Truth, Sd_Comb_Near),
      Sp ("reflect-near", 'T', Sd_Reflect_Truth, Sd_Reflect_Near),
      Sp ("shear02-near", 'T', Sd_Shear2_Truth, Sd_Shear2_Near),
      Sp ("strong-near", 'T', Sd_Strong_Truth, Sd_Strong_Near),
      Sp ("translation-near", 'T', [1.0, 0.0, 2.0, 0.0, 1.0, 1.0], Z (1.0, 0.0, 1.5, 0.0, 1.0, 0.5)),
      Sp ("small-near", 'T', Sd_Small_Truth, Z (1.0, 0.01, 1.5, -0.005, 1.0, 0.5)),
      Sp ("shear-wrong", 'T', Sd_Shear_Truth, Sd_Wrong_Seed),
      Sp ("scale-asym", 'T', Sd_Scale_Truth, Z (1.0, 0.3, 4.0, -0.25, 1.0, -2.0)),
      Sp ("combined-asym", 'T', Sd_Comb_Truth, Z (1.0, 0.3, 4.0, -0.25, 1.0, -2.0)),
      Sp ("shear02-asym", 'T', Sd_Shear2_Truth, Z (1.0, 0.3, 4.0, -0.25, 1.0, -2.0)),
      Sp ("limit-linear", 'T', Sd_Small_Truth, Z (4.0, 4.0, 0.0, 4.0, 4.0, 0.0)),
      Sp ("limit-translation", 'T', Sd_Small_Truth, Z (1.0, 0.0, 1_048_576.0, 0.0, 1.0, -1_048_576.0)),
      Sp ("all-limits", 'T', Sd_Small_Truth, Z (4.0, -4.0, 1_048_576.0, -4.0, 4.0, -1_048_576.0)),
      Sp ("singular-zero", 'T', Sd_Small_Truth, Z (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
      Sp ("singular-rank1", 'T', Sd_Small_Truth, Z (1.0, 1.0, 0.0, 1.0, 1.0, 0.0)),
      Sp ("near-singular", 'T', Sd_Small_Truth, Z (1.0, 1.0, 0.0, 1.0, 1.0000001, 0.0)),
      Sp ("reflect-seed", 'T', Sd_Small_Truth, Z (-1.0, 0.0, 0.0, 0.0, 1.0, 0.0)),
      Sp ("aniso-seed", 'T', Sd_Small_Truth, Z (4.0, 0.0, 0.0, 0.0, 0.01, 0.0)),
      Sp ("translation-1e5", 'T', Sd_Small_Truth, Z (1.0, 0.0, 100_000.0, 0.0, 1.0, 100_000.0)),
      Sp ("combined-near-it1", 'T', Sd_Comb_Truth, Sd_Comb_Near, 1),
      Sp ("combined-near-it3", 'T', Sd_Comb_Truth, Sd_Comb_Near, 3),
      Sp ("combined-near-e0.1", 'T', Sd_Comb_Truth, Sd_Comb_Near, 50, 0.1),
      Sp ("constant-seeded", 'C', Sd_Identity_Truth, Z (1.01, 0.01, 1.0, 0.0, 0.99, 1.0)),
      Sp ("noise-seeded", 'N', Sd_Identity_Truth, Z (1.01, 0.01, 1.0, 0.0, 0.99, 1.0)),
      Sp ("region-near", 'R', Sd_Small_Truth, Z (1.0, 0.01, 1.5, 0.0, 1.0, 0.5))];

   function Sd_Name (I : Positive) return String is
     (Ada.Strings.Fixed.Trim (Sd_Table (I).Name, Ada.Strings.Both));

   function Sd_Build (Spec : Sd_Spec) return ECC_Pair is
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      T      : Sd_Truth renames Spec.Truth;
   begin
      case Spec.Kind is
         when 'C' => return (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128));
         when 'N' => return (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9));
         when 'R' =>
            return (EF.Texture (160, 160).Region (Window),
                    EF.Affine_Texture (160, 160, T (0), T (1), T (3), T (4), T (2), T (5)).Region (Window));
         when others => return Sd_Pair (T);
      end case;
   end Sd_Build;

   procedure Sd_Split (Line : String; Tokens : out Sd_Token_Array; Count : out Natural) is
      First : Natural := Line'First;
   begin
      Count := 0;
      Tokens := [others => Ada.Strings.Unbounded.Null_Unbounded_String];
      while First <= Line'Last loop
         if Line (First) = ' ' then
            First := First + 1;
         else
            declare
               Last : Natural := First;
            begin
               while Last < Line'Last and then Line (Last + 1) /= ' ' loop
                  Last := Last + 1;
               end loop;
               if Count = Tokens'Last then
                  Count := Tokens'Last + 1;
                  return;
               end if;
               Count := Count + 1;
               Tokens (Count) := Ada.Strings.Unbounded.To_Unbounded_String (Line (First .. Last));
               First := Last + 1;
            end;
         end if;
      end loop;
   end Sd_Split;

   function Sd_Tok (Tokens : Sd_Token_Array; Index : Positive) return String is
     (Ada.Strings.Unbounded.To_String (Tokens (Index)));

   function Sd_Oracle_Path return String is
     (Ada.Environment_Variables.Value
        ("VIDEO_ECC_AFFINE_SEEDED_ORACLE", "../obj/oracle/ecc-affine-seeded.txt"));

   --  Strict verifier (never raises): "" when the oracle is complete, current and agrees with
   --  the binding, otherwise the first defect. Fixture record tokens: 1 fixture, 2 name,
   --  3 iterations, 4 epsilon, 5..10 seed, 11 OK|EXC, 12..17 result, 18 correlation,
   --  19 template checksum, 20 moving checksum.
   function Sd_Compare
     (Tok : Sd_Token_Array; Spec : Sd_Spec; Name : String;
      Succeeded, Raised, Nonidentity, Distinguishing : in out Natural) return String
   is
      Outcome : constant String := Sd_Tok (Tok, 11);
      Pair    : constant ECC_Pair := Sd_Build (Spec);
      Options : constant ECC_Affine_Options :=
        (Maximum_Iterations => Spec.Iterations, Epsilon => OpenCV.Float64_Value (Spec.Epsilon));
      Initial : constant ECC_Affine_Result :=
        (Warp => Spec.Seed, Correlation => 0.0);
      S : array (0 .. 5) of Long_Float;
      W : array (0 .. 5) of Long_Float;
      Seed : constant array (0 .. 5) of Long_Float :=
        [LF (Spec.Seed.M00), LF (Spec.Seed.M01), LF (Spec.Seed.M02),
         LF (Spec.Seed.M10), LF (Spec.Seed.M11), LF (Spec.Seed.M12)];
      Label : constant array (0 .. 5) of String (1 .. 3) :=
        ["M00", "M01", "M02", "M10", "M11", "M12"];
      R, Plain : ECC_Affine_Result;
      Binding_Raised, Plain_Raised : Boolean;
   begin
      if Integer'Value (Sd_Tok (Tok, 3)) /= Spec.Iterations
        or else abs (Long_Float'Value (Sd_Tok (Tok, 4)) - Spec.Epsilon) > 1.0E-15
      then
         return "fixture options differ: " & Name;
      end if;
      if Outcome /= "OK" and then Outcome /= "EXC" then return "outcome token: " & Name; end if;
      for K in S'Range loop
         S (K) := Long_Float'Value (Sd_Tok (Tok, 5 + K));
         W (K) := Long_Float'Value (Sd_Tok (Tok, 12 + K));
         if abs (S (K) - Seed (K)) > 1.0E-9 then return "seed " & Label (K) & " differs: " & Name; end if;
      end loop;
      if Long_Long_Integer'Value (Sd_Tok (Tok, 19)) /= EF.Checksum (Pair.Template)
        or else Long_Long_Integer'Value (Sd_Tok (Tok, 20)) /= EF.Checksum (Pair.Moving)
      then
         return "fixture checksum differs: " & Name;
      end if;
      if Spec.Seed /= Sd_Identity_Seed.Warp then Nonidentity := Nonidentity + 1; end if;
      Sd_Run (Pair, Initial, R, Binding_Raised, Options);
      if Outcome = "OK" then
         if Binding_Raised then return "binding raised where native succeeded: " & Name; end if;
         Succeeded := Succeeded + 1;
         for K in W'Range loop
            declare
               Got : constant Long_Float :=
                 LF (case K is when 0 => R.Warp.M00, when 1 => R.Warp.M01, when 2 => R.Warp.M02,
                               when 3 => R.Warp.M10, when 4 => R.Warp.M11, when others => R.Warp.M12);
               Tolerance : constant Long_Float := (if K = 2 or else K = 5 then 1.0E-5 else 1.0E-6);
            begin
               if abs (Got - W (K)) > Tolerance then return Label (K) & " differs: " & Name; end if;
            end;
         end loop;
         if abs (Long_Float (R.Correlation) - Long_Float'Value (Sd_Tok (Tok, 18))) > 1.0E-6 then
            return "correlation differs: " & Name;
         end if;
         if Spec.Kind = 'T' and then Spec.Seed /= Sd_Identity_Seed.Warp
           and then Sd_Near (R, Spec.Truth)
         then
            Sd_Run_Plain (Pair, Plain, Plain_Raised);
            if Plain_Raised then Distinguishing := Distinguishing + 1; end if;
         end if;
      else
         if not Binding_Raised then return "binding succeeded where native raised: " & Name; end if;
         Raised := Raised + 1;
      end if;
      return "";
   end Sd_Compare;

   function Verify_Sd_Oracle
     (Path : String; Succeeded, Raised, Nonidentity, Distinguishing : out Natural) return String
   is
      Lines : Aff_Line_Array;
      N     : Natural;
      Tok   : Sd_Token_Array;
      TC    : Natural;
      Native : constant String := Interfaces.C.Strings.Value (Aff_Native_Version);
      function Line (I : Positive) return String is (Ada.Strings.Unbounded.To_String (Lines (I)));
   begin
      Succeeded := 0; Raised := 0; Nonidentity := 0; Distinguishing := 0;
      begin
         Aff_Read_Lines (Path, Lines, N);
      exception
         when others => return "oracle missing or unreadable";
      end;
      if N = 0 then return "oracle is empty"; end if;
      if Line (1) /= "ecc-affine-seeded-oracle 1" then return "oracle magic or format version"; end if;
      if N < 2 or else Line (2) /= "version " & Native then return "oracle OpenCV version differs"; end if;
      if N < 3 or else Ada.Strings.Fixed.Head (Line (N), 4) /= "end " then
         return "oracle truncated (no end record)";
      end if;
      if Line (N) /= "end" & Integer'Image (Expected_Sd_Oracle_Fixtures) then
         return "oracle record count";
      end if;
      if N - 3 /= Expected_Sd_Oracle_Fixtures then return "oracle fixture record count"; end if;
      for I in 1 .. Expected_Sd_Oracle_Fixtures loop
         Sd_Split (Line (I + 2), Tok, TC);
         if TC /= 20 or else Sd_Tok (Tok, 1) /= "fixture" then return "malformed fixture record"; end if;
         if Sd_Tok (Tok, 2) /= Sd_Name (I) then return "fixture identity or order: " & Sd_Name (I); end if;
         declare
            Defect : constant String :=
              Sd_Compare (Tok, Sd_Table (I), Sd_Name (I), Succeeded, Raised, Nonidentity, Distinguishing);
         begin
            if Defect /= "" then return Defect; end if;
         end;
      end loop;
      return "";
   exception
      when others => return "oracle parse error";
   end Verify_Sd_Oracle;

   procedure ECC_Affine_Seeded_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Succeeded, Raised, Nonidentity, Distinguishing : Natural;
      Defect : constant String :=
        Verify_Sd_Oracle (Sd_Oracle_Path, Succeeded, Raised, Nonidentity, Distinguishing);
   begin
      Assert (Defect = "", "seeded affine oracle: " & Defect);
      Assert (Succeeded > 0 and then Raised > 0, "seeded oracle did not cover both outcomes");
      Assert (Succeeded + Raised = Expected_Sd_Oracle_Fixtures, "seeded oracle outcome accounting");
      Assert (Nonidentity >= 25, "seeded oracle needs genuinely nonidentity seeds");
      Assert (Distinguishing >= 5, "seeded oracle has too few identity-fails / seeded-succeeds fixtures");
   end ECC_Affine_Seeded_Direct_Oracle;

   function Sd_With_Token
     (Lines : Aff_Line_Array; Line_Index, Token_Index : Positive; Value : String) return Aff_Line_Array
   is
      Result : Aff_Line_Array := Lines;
      Tok    : Sd_Token_Array;
      TC     : Natural;
      Text   : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Sd_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      Tok (Token_Index) := Ada.Strings.Unbounded.To_Unbounded_String (Value);
      for K in 1 .. TC loop
         if K > 1 then Ada.Strings.Unbounded.Append (Text, " "); end if;
         Ada.Strings.Unbounded.Append (Text, Tok (K));
      end loop;
      Result (Line_Index) := Text;
      return Result;
   end Sd_With_Token;

   function Sd_Swap_Tokens
     (Lines : Aff_Line_Array; Line_Index, A, B : Positive) return Aff_Line_Array
   is
      Tok : Sd_Token_Array;
      TC  : Natural;
   begin
      Sd_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      return Sd_With_Token
        (Sd_With_Token (Lines, Line_Index, A, Sd_Tok (Tok, B)), Line_Index, B, Sd_Tok (Tok, A));
   end Sd_Swap_Tokens;

   function Sd_Line_Of (Name : String) return Positive is
   begin
      for I in 1 .. Expected_Sd_Oracle_Fixtures loop
         if Sd_Name (I) = Name then return I + 2; end if;
      end loop;
      raise Constraint_Error with "no such seeded oracle fixture " & Name;
   end Sd_Line_Of;

   procedure ECC_Affine_Seeded_Oracle_Integrity (T : in out Fixture) is
      pragma Unreferenced (T);
      Original : constant String := Sd_Oracle_Path;
      Scratch  : constant String := Original & ".tampered";
      Lines    : Aff_Line_Array;
      Count    : Natural;
      A, B, C, D : Natural;

      procedure Expect_Rejected (Label, Fragment : String; Edited : Aff_Line_Array; Edited_Count : Natural) is
      begin
         Aff_Write_Lines (Scratch, Edited, Edited_Count);
         declare
            Defect : constant String := Verify_Sd_Oracle (Scratch, A, B, C, D);
         begin
            Assert (Defect /= "", "tampered seeded oracle accepted: " & Label);
            Assert (Ada.Strings.Fixed.Index (Defect, Fragment) > 0,
                    "tampered seeded oracle (" & Label & ") rejected for the wrong reason: " & Defect);
         end;
      end Expect_Rejected;

      function Value_Of (Line_Index, Token_Index : Positive) return Long_Float is
         Tok : Sd_Token_Array;
         TC  : Natural;
      begin
         Sd_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
         return Long_Float'Value (Sd_Tok (Tok, Token_Index));
      end Value_Of;
   begin
      Aff_Read_Lines (Original, Lines, Count);
      Assert (Count = Expected_Sd_Oracle_Fixtures + 3, "seeded oracle line count");
      Assert (Verify_Sd_Oracle (Original, A, B, C, D) = "", "pristine seeded oracle rejected");
      declare
         Comb : constant Positive := Sd_Line_Of ("combined-near");
      begin
         Assert (Verify_Sd_Oracle (Original & ".does-not-exist", A, B, C, D) = "oracle missing or unreadable",
                 "missing seeded oracle accepted");
         Expect_Rejected ("empty", "empty", Lines, 0);
         Expect_Rejected ("truncated", "truncated", Lines, Count - 1);
         Expect_Rejected ("truncated mid-record", "truncated", Lines, Count - 5);
         Expect_Rejected ("wrong format version", "format version", Sd_With_Token (Lines, 1, 2, "2"), Count);
         Expect_Rejected ("wrong OpenCV version", "OpenCV version", Sd_With_Token (Lines, 2, 2, "0.0.0"), Count);
         Expect_Rejected ("wrong declared count", "record count", Sd_With_Token (Lines, Count, 2, "33"), Count);
         Expect_Rejected ("renamed fixture", "fixture identity", Sd_With_Token (Lines, Comb, 2, "combined-nearX"), Count);
         Expect_Rejected ("altered template checksum", "checksum", Sd_With_Token (Lines, Comb, 19, "1"), Count);
         Expect_Rejected ("altered moving checksum", "checksum",
                          Sd_With_Token (Lines, Sd_Line_Of ("shear-near"), 20, "1"), Count);
         Expect_Rejected ("altered seed M00", "seed M00", Sd_With_Token (Lines, Comb, 5, "1.0"), Count);
         Expect_Rejected ("altered seed M01", "seed M01", Sd_With_Token (Lines, Comb, 6, "0.5"), Count);
         Expect_Rejected ("altered seed M10", "seed M10", Sd_With_Token (Lines, Comb, 8, "0.5"), Count);
         Expect_Rejected ("swapped seed M01 and M10", "seed M01", Sd_Swap_Tokens (Lines, Comb, 6, 8), Count);
         Expect_Rejected ("altered seed M02", "seed M02", Sd_With_Token (Lines, Comb, 7, "9.0"), Count);
         Expect_Rejected ("altered seed M12", "seed M12", Sd_With_Token (Lines, Comb, 10, "9.0"), Count);
         Expect_Rejected ("swapped seed translations", "seed M02", Sd_Swap_Tokens (Lines, Comb, 7, 10), Count);
         Assert (abs (Value_Of (Comb, 13) - Value_Of (Comb, 15)) > 0.01, "swap fixture M01/M10 must differ");
         Expect_Rejected ("swapped output M01 and M10", "differs", Sd_Swap_Tokens (Lines, Comb, 13, 15), Count);
         Expect_Rejected ("swapped output M02 and M12", "differs", Sd_Swap_Tokens (Lines, Comb, 14, 17), Count);
         Expect_Rejected ("altered output M00", "M00 differs", Sd_With_Token (Lines, Comb, 12, "1.5"), Count);
         Expect_Rejected ("altered correlation", "correlation differs", Sd_With_Token (Lines, Comb, 18, "0.5"), Count);
         Expect_Rejected ("success recorded as exception", "binding succeeded where native raised",
                          Sd_With_Token (Lines, Sd_Line_Of ("identity-id"), 11, "EXC"), Count);
         Expect_Rejected ("exception recorded as success", "binding raised where native succeeded",
                          Sd_With_Token (Lines, Sd_Line_Of ("shear-id"), 11, "OK"), Count);
         Expect_Rejected ("unknown outcome token", "outcome token",
                          Sd_With_Token (Lines, Sd_Line_Of ("identity-id"), 11, "MAYBE"), Count);
      end;
      Ada.Directories.Delete_File (Scratch);
   end ECC_Affine_Seeded_Oracle_Integrity;

   ---------------------------------------------------------------------
   --  Task 016: ECC eight-parameter homography alignment
   ---------------------------------------------------------------------
   subtype HM is EF.Homography_Matrix;

   Hom_Linear_Tolerance      : constant Long_Float := 0.003;
   Hom_Translation_Tolerance : constant Long_Float := 0.1;
   Hom_Projective_Tolerance  : constant Long_Float := 0.00005;

   --  Ground-truth homographies (row-major, template to moving); the values mirror
   --  tests/cpp/ecc_homography_oracle.cpp literally so both sides build equal images.
   K_Identity : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
   K_T21      : constant HM := [1.0, 0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0];
   K_Tm       : constant HM := [1.0, 0.0, -2.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0];
   K_T12      : constant HM := [1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0];
   K_R2       : constant HM := [0.99939083, -0.0348995, 0.0, 0.0348995, 0.99939083, 0.0, 0.0, 0.0, 1.0];
   K_Rm2      : constant HM := [0.99939083, 0.0348995, 0.0, -0.0348995, 0.99939083, 0.0, 0.0, 0.0, 1.0];
   K_S102     : constant HM := [1.02, 0.0, 0.0, 0.0, 1.02, 0.0, 0.0, 0.0, 1.0];
   K_S098     : constant HM := [0.98, 0.0, 0.0, 0.0, 0.98, 0.0, 0.0, 0.0, 1.0];
   K_AN       : constant HM := [1.03, 0.0, 0.0, 0.0, 0.98, 0.0, 0.0, 0.0, 1.0];
   K_SHX      : constant HM := [1.0, 0.02, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
   K_SHY      : constant HM := [1.0, 0.0, 0.0, -0.01, 1.0, 0.0, 0.0, 0.0, 1.0];
   K_AC       : constant HM := [1.01, 0.01, 1.5, -0.01, 0.99, -1.0, 0.0, 0.0, 1.0];
   K_P20p     : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0002, 0.0, 1.0];
   K_P20n     : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0002, 0.0, 1.0];
   K_P21p     : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0002, 1.0];
   K_P21n     : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0002, 1.0];
   K_PB       : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0002, -0.0002, 1.0];
   K_PM       : constant HM := [1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0];
   K_RP       : constant HM := [0.9994, -0.0349, 1.0, 0.0349, 0.9994, 0.5, 0.0001, 0.00005, 1.0];
   K_SP       : constant HM := [1.02, 0.01, 1.0, -0.01, 0.98, -0.5, 0.0001, -0.0001, 1.0];
   K_Strong   : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.002, 0.002, 1.0];
   K_NZ       : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0078, 0.0, 1.0];
   K_CZ       : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.012, 0.0, 1.0];
   K_IC       : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 1.0];
   K_LU       : constant HM := [1.0, 0.0, 40.0, 0.0, 1.0, 30.0, 0.0, 0.0, 1.0];
   K_RX       : constant HM := [-1.0, 0.0, 127.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

   function Hom_Pair (H : HM) return ECC_Pair is
     (Template => EF.Texture (128, 128),
      Moving   => EF.Homography_Texture (128, 128, H));

   function Coeffs (W : ECC_Homography_Warp) return HM is
     [LF (W.H00), LF (W.H01), LF (W.H02), LF (W.H10), LF (W.H11), LF (W.H12),
      LF (W.H20), LF (W.H21), LF (W.H22)];

   function Hom_Near (R : ECC_Homography_Result; H : HM) return Boolean is
     (abs (LF (R.Warp.H00) - H (0)) < Hom_Linear_Tolerance
      and then abs (LF (R.Warp.H01) - H (1)) < Hom_Linear_Tolerance
      and then abs (LF (R.Warp.H02) - H (2)) < Hom_Translation_Tolerance
      and then abs (LF (R.Warp.H10) - H (3)) < Hom_Linear_Tolerance
      and then abs (LF (R.Warp.H11) - H (4)) < Hom_Linear_Tolerance
      and then abs (LF (R.Warp.H12) - H (5)) < Hom_Translation_Tolerance
      and then abs (LF (R.Warp.H20) - H (6)) < Hom_Projective_Tolerance
      and then abs (LF (R.Warp.H21) - H (7)) < Hom_Projective_Tolerance
      and then R.Warp.H22 = 1.0);

   function Swapped (W : HM; A, B : Natural) return HM is
      Result : HM := W;
   begin
      Result (A) := W (B);
      Result (B) := W (A);
      return Result;
   end Swapped;

   function Transposed (W : HM) return HM is
     [W (0), W (3), W (6), W (1), W (4), W (7), W (2), W (5), W (8)];

   function Negated_Translation (W : HM) return HM is
     [W (0), W (1), -W (2), W (3), W (4), -W (5), W (6), W (7), W (8)];

   --  Applies the matrix with homogeneous division: template to moving coordinates.
   procedure Hom_Map (W : HM; X, Y : Long_Float; MX, MY : out Long_Float; Valid : out Boolean) is
      D : constant Long_Float := W (6) * X + W (7) * Y + W (8);
   begin
      MX := 0.0;
      MY := 0.0;
      Valid := abs D > 1.0E-6;
      if Valid then
         MX := (W (0) * X + W (1) * Y + W (2)) / D;
         MY := (W (3) * X + W (4) * Y + W (5)) / D;
      end if;
   end Hom_Map;

   Ref_X : constant array (1 .. 5) of Long_Float := [32.0, 96.0, 32.0, 96.0, 64.0];
   Ref_Y : constant array (1 .. 5) of Long_Float := [32.0, 32.0, 96.0, 96.0, 64.0];

   --  Largest distance between where two matrices send the independent reference points.
   function Hom_Map_Error (A, B : HM) return Long_Float is
      Worst : Long_Float := 0.0;
      AX, AY, BX, BY : Long_Float;
      AV, BV : Boolean;
   begin
      for K in Ref_X'Range loop
         Hom_Map (A, Ref_X (K), Ref_Y (K), AX, AY, AV);
         Hom_Map (B, Ref_X (K), Ref_Y (K), BX, BY, BV);
         if not (AV and then BV) then
            return 1.0E9;
         end if;
         Worst := Long_Float'Max (Worst, Long_Float'Max (abs (AX - BX), abs (AY - BY)));
      end loop;
      return Worst;
   end Hom_Map_Error;

   --  Mean absolute difference over the interior. Aligned resamples the moving image at
   --  W * p (homogeneous division), which is what WARP_INVERSE_MAP does with the returned
   --  matrix. A point mapped outside the image or to a zero denominator counts as 255.
   function Hom_Residual (Pair : ECC_Pair; W : HM; Aligned : Boolean) return Long_Float is
      Margin : constant := 24;
      Sum    : Long_Float := 0.0;
      Count  : Natural := 0;
      MX, MY : Long_Float;
      Valid  : Boolean;
   begin
      for Y in Margin .. 127 - Margin loop
         for X in Margin .. 127 - Margin loop
            if Aligned then
               Hom_Map (W, Long_Float (X), Long_Float (Y), MX, MY, Valid);
            else
               MX := Long_Float (X);
               MY := Long_Float (Y);
               Valid := True;
            end if;
            if Valid and then MX >= 0.0 and then MX < 126.0
              and then MY >= 0.0 and then MY < 126.0
            then
               Sum := Sum + abs (Sample (Pair.Moving, MX, MY)
                                 - Long_Float (OpenCV.Core.UInt8_Access.Get (Pair.Template, Y, X)));
            else
               Sum := Sum + 255.0;
            end if;
            Count := Count + 1;
         end loop;
      end loop;
      return Sum / Long_Float (Count);
   end Hom_Residual;

   procedure Hom_Expect_Error
     (Template, Moving : OpenCV.Core.Mat;
      Options : ECC_Homography_Options := (others => <>);
      Message : String)
   is
      Unused : ECC_Homography_Result;
   begin
      Unused := Estimate_ECC_Homography (Template, Moving, Options);
      Assert (False, Message & ": request accepted");
   exception
      when OpenCV.OpenCV_Error => null;
   end Hom_Expect_Error;

   --  An Ada-preflight rejection does not carry the shim's "Video.Estimate_ECC_Homography"
   --  prefix; a shim or native rejection does.
   procedure Hom_Expect_Preflight_Error (Template, Moving : OpenCV.Core.Mat; Message : String) is
      Unused : ECC_Homography_Result;
   begin
      Unused := Estimate_ECC_Homography (Template, Moving);
      Assert (False, Message & ": request accepted");
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text   : constant String := Ada.Exceptions.Exception_Message (Error);
            Marker : constant String := "Video.Estimate_ECC_Homography";
         begin
            Assert (Text'Length < Marker'Length
                    or else Text (Text'First .. Text'First + Marker'Length - 1) /= Marker,
                    Message & ": rejected by the native layer, not the Ada preflight");
         end;
   end Hom_Expect_Preflight_Error;

   --  Estimates one fixture and checks it against its ground truth.
   function Hom_Check (Label : String; H : HM) return ECC_Homography_Result is
      Pair : constant ECC_Pair := Hom_Pair (H);
      R    : constant ECC_Homography_Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving);
   begin
      Assert (Hom_Near (R, H), "ECC homography " & Label & " coefficients");
      Assert (Hom_Map_Error (Coeffs (R.Warp), H) < 1.5, "ECC homography " & Label & " mapped points");
      Assert (R.Correlation > 0.98, "ECC homography " & Label & " correlation");
      return R;
   end Hom_Check;

   procedure ECC_Homography_Defaults (T : in out Fixture) is
      pragma Unreferenced (T);
      Options  : constant ECC_Homography_Options := (others => <>);
      Zero     : constant ECC_Homography_Result := (others => <>);
      Identity : constant ECC_Homography_Warp :=
        (H00 => 1.0, H01 => 0.0, H02 => 0.0, H10 => 0.0, H11 => 1.0, H12 => 0.0,
         H20 => 0.0, H21 => 0.0, H22 => 1.0);
   begin
      Assert (Options = ECC_Translation_Options'(others => <>),
              "ECC homography options reuse the translation defaults");
      Assert (Zero.Warp = Identity and then Zero.Correlation = 0.0, "ECC homography default result");
      Assert (EF."=" (Coeffs (Zero.Warp), K_Identity), "ECC homography default coefficients are row-major");
   end ECC_Homography_Defaults;

   procedure ECC_Homography_Identity_And_Translation (T : in out Fixture) is
      pragma Unreferenced (T);
      Base      : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Before    : constant Long_Long_Integer := EF.Checksum (Base);
      Identity  : constant ECC_Homography_Result := Estimate_ECC_Homography (Base, Base);
      Plus      : constant ECC_Homography_Result := Hom_Check ("translation (2,1)", K_T21);
      Minus     : constant ECC_Homography_Result := Hom_Check ("translation (-2,-1)", K_Tm);
      Other     : constant ECC_Homography_Result := Hom_Check ("translation (1,2)", K_T12);
   begin
      Assert (Hom_Near (Identity, K_Identity) and then abs (Identity.Correlation - 1.0) < 1.0E-6,
              "ECC homography identity");
      Assert (abs (LF (Identity.Warp.H20)) < 1.0E-6 and then abs (LF (Identity.Warp.H21)) < 1.0E-6,
              "ECC homography identity has no projective part");
      --  Translation lands in H02/H12 with the right signs, never in H20/H21.
      Assert (Plus.Warp.H02 > 1.8 and then Plus.Warp.H12 > 0.8, "ECC homography (2,1) translation signs");
      Assert (Minus.Warp.H02 < -1.8 and then Minus.Warp.H12 < -0.8, "ECC homography (-2,-1) translation signs");
      Assert (abs (LF (Plus.Warp.H20)) < 1.0E-4 and then abs (LF (Plus.Warp.H21)) < 1.0E-4,
              "ECC homography translation leaked into the projective terms");
      Assert (Other.Warp.H02 < Other.Warp.H12 and then Plus.Warp.H02 > Plus.Warp.H12,
              "ECC homography (1,2) translation not distinguished from (2,1)");
      Assert (Plus.Warp.H22 = 1.0 and then Minus.Warp.H22 = 1.0 and then Identity.Warp.H22 = 1.0,
              "ECC homography H22 is exactly 1");
      Assert (EF.Checksum (Base) = Before, "ECC homography modified an input");
   end ECC_Homography_Identity_And_Translation;

   procedure ECC_Homography_Rotation_Scale_Shear (T : in out Fixture) is
      pragma Unreferenced (T);
      Rp  : constant ECC_Homography_Result := Hom_Check ("rotation +2", K_R2);
      Rm  : constant ECC_Homography_Result := Hom_Check ("rotation -2", K_Rm2);
      Up  : constant ECC_Homography_Result := Hom_Check ("scale 1.02", K_S102);
      Dn  : constant ECC_Homography_Result := Hom_Check ("scale 0.98", K_S098);
      An  : constant ECC_Homography_Result := Hom_Check ("anisotropic scale", K_AN);
      ShX : constant ECC_Homography_Result := Hom_Check ("x shear", K_SHX);
      ShY : constant ECC_Homography_Result := Hom_Check ("y shear", K_SHY);
      Aff : constant ECC_Homography_Result := Hom_Check ("affine combination", K_AC);
   begin
      Assert (Rp.Warp.H10 > 0.03 and then Rp.Warp.H01 < -0.03, "ECC homography +rotation signs");
      Assert (Rm.Warp.H10 < -0.03 and then Rm.Warp.H01 > 0.03, "ECC homography -rotation signs");
      Assert (Up.Warp.H00 > 1.01 and then Up.Warp.H11 > 1.01
              and then Dn.Warp.H00 < 0.99 and then Dn.Warp.H11 < 0.99,
              "ECC homography isotropic scale directions");
      Assert (An.Warp.H00 - An.Warp.H11 > 0.04, "ECC homography anisotropic scale diagonal entries");
      Assert (ShX.Warp.H01 > 0.015 and then abs (LF (ShX.Warp.H10)) < 0.003,
              "ECC homography x shear placement");
      Assert (ShY.Warp.H10 < -0.007 and then abs (LF (ShY.Warp.H01)) < 0.003,
              "ECC homography y shear placement");
      Assert (Aff.Warp.H01 > 0.005 and then Aff.Warp.H10 < -0.005 and then Aff.Warp.H02 > 1.3
              and then Aff.Warp.H12 < -0.8, "ECC homography affine-equivalent combination");
      Assert (abs (LF (Aff.Warp.H20)) < 5.0E-5 and then abs (LF (Aff.Warp.H21)) < 5.0E-5,
              "ECC homography affine-equivalent projective terms");
   end ECC_Homography_Rotation_Scale_Shear;

   procedure ECC_Homography_Projective_Coefficients (T : in out Fixture) is
      pragma Unreferenced (T);
      Xp   : constant ECC_Homography_Result := Hom_Check ("positive H20", K_P20p);
      Xn   : constant ECC_Homography_Result := Hom_Check ("negative H20", K_P20n);
      Yp   : constant ECC_Homography_Result := Hom_Check ("positive H21", K_P21p);
      Yn   : constant ECC_Homography_Result := Hom_Check ("negative H21", K_P21n);
      Both : constant ECC_Homography_Result := Hom_Check ("H20 and H21", K_PB);
      Mild : constant ECC_Homography_Result := Hom_Check ("mild perspective", K_PM);
      Rot  : constant ECC_Homography_Result := Hom_Check ("rotation and perspective", K_RP);
      Sst  : constant ECC_Homography_Result := Hom_Check ("scale shear translation perspective", K_SP);
      Reversed : constant ECC_Homography_Result :=
        Estimate_ECC_Homography (EF.Homography_Texture (128, 128, K_PB), EF.Texture (128, 128));
      RW   : constant HM := Coeffs (Rot.Warp);
   begin
      Assert (Xp.Warp.H20 > 0.00015 and then abs (LF (Xp.Warp.H21)) < 0.00005, "positive H20 only");
      Assert (Xn.Warp.H20 < -0.00015 and then abs (LF (Xn.Warp.H21)) < 0.00005, "negative H20 only");
      Assert (Yp.Warp.H21 > 0.00015 and then abs (LF (Yp.Warp.H20)) < 0.00005, "positive H21 only");
      Assert (Yn.Warp.H21 < -0.00015 and then abs (LF (Yn.Warp.H20)) < 0.00005, "negative H21 only");
      Assert (Both.Warp.H20 > 0.00015 and then Both.Warp.H21 < -0.00015, "H20 and H21 together");
      Assert (Mild.Warp.H20 > 0.0 and then Mild.Warp.H21 < 0.0, "mild perspective signs");
      Assert (Rot.Warp.H20 > 0.00005 and then Rot.Warp.H21 > 0.0, "rotation and perspective signs");
      Assert (Sst.Warp.H20 > 0.00005 and then Sst.Warp.H21 < -0.00005,
              "scale/shear/translation and perspective signs");
      Assert (Reversed.Warp.H20 < -0.00015 and then Reversed.Warp.H21 > 0.00015
              and then Reversed.Warp.H22 = 1.0, "image role reversal flips the projective terms");
      --  Misplaced coefficients map the independent reference points somewhere else.
      Assert (Hom_Map_Error (RW, Swapped (RW, 2, 6)) > 5.0, "swapping H02 and H20 preserved the mapping");
      Assert (Hom_Map_Error (RW, Swapped (RW, 5, 7)) > 5.0, "swapping H12 and H21 preserved the mapping");
      Assert (Hom_Map_Error (RW, Swapped (RW, 1, 3)) > 3.0, "swapping H01 and H10 preserved the mapping");
      Assert (Hom_Map_Error (RW, Transposed (RW)) > 3.0, "transposing the matrix preserved the mapping");
      Assert (Hom_Map_Error (RW, Negated_Translation (RW)) > 1.5,
              "negating the translation preserved the mapping");
   end ECC_Homography_Projective_Coefficients;

   function Aff_As_HM (W : ECC_Affine_Warp) return HM is
     [LF (W.M00), LF (W.M01), LF (W.M02), LF (W.M10), LF (W.M11), LF (W.M12), 0.0, 0.0, 1.0];

   procedure ECC_Homography_Versus_Affine (T : in out Fixture) is
      pragma Unreferenced (T);
      --  Fixtures where the measured advantage is material (affine/homography residual ratio
      --  2.2 to 4.2 on 4.10.0). The very mild K_PM perspective is excluded on purpose: there the
      --  affine model is only about 1.2 times worse, which is not a meaningful distinction.
      Cases : constant array (1 .. 5) of HM := [K_P20p, K_P21n, K_PB, K_RP, K_SP];
   begin
      for H of Cases loop
         declare
            Pair : constant ECC_Pair := Hom_Pair (H);
            Hom  : constant ECC_Homography_Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving);
            Aff  : constant ECC_Affine_Result := Estimate_ECC_Affine (Pair.Template, Pair.Moving);
            Before : constant Long_Float := Hom_Residual (Pair, K_Identity, True);
            HR   : constant Long_Float := Hom_Residual (Pair, Coeffs (Hom.Warp), True);
            AR   : constant Long_Float := Hom_Residual (Pair, Aff_As_HM (Aff.Warp), True);
            TR   : constant Long_Float := Hom_Residual (Pair, H, True);
         begin
            --  The affine call returns a finite, plausible matrix: this is not a native failure.
            Assert (Aff.Correlation > 0.9, "ECC affine on a projective fixture is finite and correlated");
            Assert (Before > 2.5, "projective fixture is not already aligned");
            Assert (HR < 0.6 and then HR < Before / 5.0, "ECC homography residual did not collapse");
            Assert (AR > 1.8 * HR, "ECC homography is not materially better than affine");
            Assert (HR < TR + 0.1, "ECC homography residual is worse than the true matrix");
            --  Only the homography result carries a projective part.
            Assert (abs (LF (Hom.Warp.H20)) > 5.0E-5 or else abs (LF (Hom.Warp.H21)) > 5.0E-5,
                    "ECC homography found no projective term");
         end;
      end loop;
   end ECC_Homography_Versus_Affine;

   procedure ECC_Homography_Inverse_Warp (T : in out Fixture) is
      pragma Unreferenced (T);
      Cases : constant array (1 .. 4) of HM := [K_P20p, K_PM, K_RP, K_SP];
   begin
      for H of Cases loop
         declare
            Pair : constant ECC_Pair := Hom_Pair (H);
            R    : constant ECC_Homography_Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving);
            W    : constant HM := Coeffs (R.Warp);
            Good : constant Long_Float := Hom_Residual (Pair, W, True);
         begin
            --  The returned matrix, unchanged, resamples the moving image into the template frame.
            Assert (Hom_Residual (Pair, K_Identity, True) > 5.0 * Good, "inverse warp did not reduce the residual");
            Assert (Hom_Residual (Pair, Transposed (W), True) > 5.0 * Good, "transposed matrix aligned the images");
            --  Only meaningful when the fixture has a real translation (K_P20p has none).
            if abs (W (2)) > 0.5 then
               Assert (Hom_Residual (Pair, Negated_Translation (W), True) > 3.0 * Good,
                       "negated translation aligned the images");
            end if;
            Assert (Hom_Residual (Pair, Swapped (W, 2, 6), True) > 5.0 * Good, "swapped H02/H20 aligned the images");
            Assert (Hom_Residual (Pair, Swapped (W, 5, 7), True) > 5.0 * Good, "swapped H12/H21 aligned the images");
         end;
      end loop;
   end ECC_Homography_Inverse_Warp;

   procedure ECC_Homography_Options_And_Repeatability (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair    : constant ECC_Pair := Hom_Pair (K_PB);
      Default : constant ECC_Homography_Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving);
      One     : constant ECC_Homography_Result := Estimate_ECC_Homography
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 1, Epsilon => 1.0E-4));
      Three   : constant ECC_Homography_Result := Estimate_ECC_Homography
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 3, Epsilon => 1.0E-4));
      Tight   : constant ECC_Homography_Result := Estimate_ECC_Homography
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 50, Epsilon => 1.0E-8));
      Loose   : constant ECC_Homography_Result := Estimate_ECC_Homography
        (Pair.Template, Pair.Moving, (Maximum_Iterations => 50, Epsilon => 0.1));
   begin
      Assert (Hom_Near (Default, K_PB), "ECC homography default options");
      Assert (Default = Estimate_ECC_Homography (Pair.Template, Pair.Moving), "ECC homography not repeatable");
      Assert (One /= Default and then Three /= One, "ECC homography iteration option ignored");
      Assert (One.Warp.H20 < Default.Warp.H20 and then One.Correlation < Default.Correlation,
              "one iteration should be coarser than the converged result");
      Assert (Hom_Near (Three, K_PB), "ECC homography three iterations");
      Assert (Hom_Near (Tight, K_PB) and then Hom_Near (Loose, K_PB), "ECC homography nondefault options lost accuracy");
      Assert (Tight /= Loose, "ECC homography epsilon option ignored");
      Assert (Default.Warp.H22 = 1.0 and then One.Warp.H22 = 1.0 and then Loose.Warp.H22 = 1.0,
              "ECC homography H22 under options");
      Assert (Loose.Correlation > 0.98 and then Loose.Correlation < 1.01,
              "ECC homography correlation outside the observed range");
      --  Option bounds.
      Assert (Estimate_ECC_Homography (Pair.Template, Pair.Moving, (Maximum_Iterations => 100, Epsilon => 0.1))
              /= Estimate_ECC_Homography (Pair.Template, Pair.Moving, (Maximum_Iterations => 1, Epsilon => 1.0E-8)),
              "ECC homography option bounds");
   end ECC_Homography_Options_And_Repeatability;

   procedure ECC_Homography_Regions_Lifetime (T : in out Fixture) is
      pragma Unreferenced (T);
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Saved, Contiguous : ECC_Homography_Result;
      Before_A, Before_B : Long_Long_Integer;
   begin
      declare
         Parent_A : constant OpenCV.Core.Mat := EF.Texture (160, 160);
         Parent_B : constant OpenCV.Core.Mat := EF.Homography_Texture (160, 160, K_PM);
         Region_A : constant OpenCV.Core.Mat := Parent_A.Region (Window);
         Region_B : constant OpenCV.Core.Mat := Parent_B.Region (Window);
      begin
         Before_A := EF.Checksum (Parent_A);
         Before_B := EF.Checksum (Parent_B);
         Assert (not Region_A.Is_Continuous and then not Region_B.Is_Continuous,
                 "ECC homography fixture Regions are contiguous");
         Saved := Estimate_ECC_Homography (Region_A, Region_B);
         Contiguous := Estimate_ECC_Homography (Region_A.Clone, Region_B.Clone);
         Assert (Saved = Contiguous, "ECC homography Region differs from the contiguous clone");
         Assert (EF.Checksum (Parent_A) = Before_A and then EF.Checksum (Parent_B) = Before_B,
                 "ECC homography modified a Region parent");
         Assert (EF.Checksum (Region_A) = EF.Checksum (Region_A.Clone), "ECC homography modified a Region");
         --  One allocation in both roles.
         Assert (Hom_Near (Estimate_ECC_Homography (Parent_A, Parent_A), K_Identity),
                 "ECC homography with one image in both roles");
         Assert (Hom_Near (Estimate_ECC_Homography (Region_A, Region_A), K_Identity),
                 "ECC homography with one Region in both roles");
      end;
      --  Parents and Regions are finalized; the result is a plain value.
      Assert (Saved.Warp.H22 = 1.0 and then Saved.Correlation > 0.98
              and then abs (LF (Saved.Warp.H20) - 0.00005) < 0.0001
              and then abs (LF (Saved.Warp.H21) + 0.00004) < 0.0001, "ECC homography result after finalization");
   end ECC_Homography_Regions_Lifetime;

   procedure ECC_Homography_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Good : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Empty : OpenCV.Core.Mat;
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.UInt8, Channels => 3));
      function From_Bits is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      Bad_Epsilons : constant array (Positive range <>) of OpenCV.Float64_Value :=
        [0.0, -1.0E-4, 9.0E-9, 0.1000001, 1.0,
         From_Bits (16#7FF0_0000_0000_0000#), From_Bits (16#FFF0_0000_0000_0000#),
         From_Bits (16#7FF8_0000_0000_0000#)];
   begin
      Hom_Expect_Error (Empty, Good, Message => "empty template");
      Hom_Expect_Error (Good, Empty, Message => "empty moving");
      Hom_Expect_Preflight_Error (Float_Image, Float_Image, "Float32 images");
      Hom_Expect_Preflight_Error (Color, Color, "three-channel images");
      Hom_Expect_Preflight_Error (Good, EF.Texture (128, 129), "different columns");
      Hom_Expect_Preflight_Error (EF.Texture (129, 128), Good, "different rows");
      Hom_Expect_Error (Good, Float_Image, Message => "Float32 moving");
      Hom_Expect_Error (EF.Texture (31, 128), EF.Texture (31, 128), Message => "31 rows");
      Hom_Expect_Error (EF.Texture (128, 31), EF.Texture (128, 31), Message => "31 columns");
      declare
         Wide : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (32, 16_385, (Depth => OpenCV.Core.UInt8, Channels => 1));
         Area : constant OpenCV.Core.Mat := OpenCV.Core.Create
           (4_097, 4_096, (Depth => OpenCV.Core.UInt8, Channels => 1));
      begin
         --  Rejected before native work: no eight-block Jacobian is allocated.
         Hom_Expect_Preflight_Error (Wide, Wide, "16385 columns");
         Hom_Expect_Preflight_Error (Area, Area, "4097 x 4096 pixels");
      end;
      Hom_Expect_Error (Good, Good, (Maximum_Iterations => 101, Epsilon => 0.0001), Message => "101 iterations");
      Hom_Expect_Error (Good, Good, (Maximum_Iterations => Positive'Last, Epsilon => 0.0001),
                        Message => "Positive'Last iterations");
      for Epsilon of Bad_Epsilons loop
         Hom_Expect_Error (Good, Good, (Maximum_Iterations => 50, Epsilon => Epsilon), Message => "invalid epsilon");
      end loop;
   end ECC_Homography_Validation;

   procedure ECC_Homography_Nonconvergence (T : in out Fixture) is
      pragma Unreferenced (T);
      Flat          : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
      Texture_Image : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Noise         : constant OpenCV.Core.Mat := EF.Noise (128, 128, 7);
   begin
      Hom_Expect_Error (Flat, Flat, Message => "constant images");
      Hom_Expect_Error (Flat, Texture_Image, Message => "constant template");
      Hom_Expect_Error (Texture_Image, Flat, Message => "constant moving");
      Hom_Expect_Error (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9), Message => "uncorrelated noise");
      --  Outside the identity-initialized basin on every locally qualified version.
      Hom_Expect_Error (Texture_Image, EF.Homography_Texture (128, 128, K_Strong), Message => "strong perspective");
      Hom_Expect_Error (Texture_Image, EF.Homography_Texture (128, 128, K_NZ), Message => "near-zero denominator");
      Hom_Expect_Error (Texture_Image, EF.Homography_Texture (128, 128, K_CZ), Message => "denominator crossing zero");
      Hom_Expect_Error (Texture_Image, EF.Homography_Texture (128, 128, K_LU), Message => "large translation");
      Hom_Expect_Error (Texture_Image, EF.Homography_Texture (128, 128, K_RX), Message => "reflection");
      --  Nonconvergence carries the shim prefix and the native diagnostic; no identity is fabricated.
      begin
         declare
            Unused : constant ECC_Homography_Result := Estimate_ECC_Homography (Flat, Flat);
         begin
            null;
         end;
         Assert (False, "constant images accepted");
      exception
         when Error : OpenCV.OpenCV_Error =>
            Assert (Ada.Strings.Fixed.Index (Ada.Exceptions.Exception_Message (Error),
                                             "Video.Estimate_ECC_Homography") = 1,
                    "native nonconvergence lost the operation prefix");
      end;
      --  Finite is not correct: noise against a texture and an ill-conditioned fixture may be
      --  accepted with a low correlation, repeatably; another native version may raise instead.
      begin
         declare
            R : constant ECC_Homography_Result := Estimate_ECC_Homography (Texture_Image, Noise);
         begin
            Assert (R.Correlation < 0.2 and then R.Warp.H22 = 1.0, "ECC homography noise-vs-texture");
            Assert (R = Estimate_ECC_Homography (Texture_Image, Noise), "ECC homography noise not repeatable");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      begin
         declare
            R : constant ECC_Homography_Result :=
              Estimate_ECC_Homography (Texture_Image, EF.Homography_Texture (128, 128, K_IC));
         begin
            Assert (R.Correlation < 0.5 and then Hom_Residual (Hom_Pair (K_IC), Coeffs (R.Warp), True) > 20.0,
                    "ill-conditioned fixture was reported as a good fit");
         end;
      exception
         when OpenCV.OpenCV_Error => null;
      end;
   end ECC_Homography_Nonconvergence;

   ---------------------------------------------------------------------
   --  Task 016: independent homography oracle (ecc-homography-oracle 1)
   ---------------------------------------------------------------------
   Expected_Hom_Oracle_Fixtures : constant := 37;

   type Hom_Line_Array is array (1 .. 64) of Ada.Strings.Unbounded.Unbounded_String;
   type Hom_Token_Array is array (1 .. 20) of Ada.Strings.Unbounded.Unbounded_String;

   function Hom_Oracle_Path return String is
     (Ada.Environment_Variables.Value ("VIDEO_ECC_HOMOGRAPHY_ORACLE", "../obj/oracle/ecc-homography.txt"));

   function Hom_Name (Index : Positive) return String is
   begin
      case Index is
         when 1 => return "identity";
         when 2 => return "translation-2-1";
         when 3 => return "translation-m2-m1";
         when 4 => return "rot+2";
         when 5 => return "rot-2";
         when 6 => return "iso-1.02";
         when 7 => return "iso-0.98";
         when 8 => return "aniso-1.03-0.98";
         when 9 => return "shear-x+0.02";
         when 10 => return "shear-y-0.01";
         when 11 => return "affine-combo";
         when 12 => return "h20-pos";
         when 13 => return "h20-neg";
         when 14 => return "h21-pos";
         when 15 => return "h21-neg";
         when 16 => return "h20-h21-both";
         when 17 => return "persp-mild";
         when 18 => return "rot-persp";
         when 19 => return "sst-persp";
         when 20 => return "reversed-persp-both";
         when 21 => return "photometric-persp-both";
         when 22 => return "it1";
         when 23 => return "it3";
         when 24 => return "it100-e8";
         when 25 => return "e0.1";
         when 26 => return "e1e-8";
         when 27 => return "region-persp-mild";
         when 28 => return "constant";
         when 29 => return "lowgrad";
         when 30 => return "noise-vs-texture";
         when 31 => return "noise-vs-noise";
         when 32 => return "persp-strong";
         when 33 => return "persp-near-zero-den";
         when 34 => return "persp-den-crosses-zero";
         when 35 => return "illcond";
         when 36 => return "large-translation";
         when 37 => return "reflect-x";
         when others => raise Constraint_Error with "homography oracle fixture index";
      end case;
   end Hom_Name;

   --  Rebuilds a named oracle fixture independently of the C++ code.
   function Hom_Build (Name : String) return ECC_Pair is
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      function Tex return OpenCV.Core.Mat is (EF.Texture (128, 128));
      function Mov (H : HM) return OpenCV.Core.Mat is (EF.Homography_Texture (128, 128, H));
      function P (H : HM) return ECC_Pair is (Tex, Mov (H));
   begin
      if Name = "identity" then return (Tex, Tex);
      elsif Name = "translation-2-1" then return P (K_T21);
      elsif Name = "translation-m2-m1" then return P (K_Tm);
      elsif Name = "rot+2" then return P (K_R2);
      elsif Name = "rot-2" then return P (K_Rm2);
      elsif Name = "iso-1.02" then return P (K_S102);
      elsif Name = "iso-0.98" then return P (K_S098);
      elsif Name = "aniso-1.03-0.98" then return P (K_AN);
      elsif Name = "shear-x+0.02" then return P (K_SHX);
      elsif Name = "shear-y-0.01" then return P (K_SHY);
      elsif Name = "affine-combo" then return P (K_AC);
      elsif Name = "h20-pos" then return P (K_P20p);
      elsif Name = "h20-neg" then return P (K_P20n);
      elsif Name = "h21-pos" then return P (K_P21p);
      elsif Name = "h21-neg" then return P (K_P21n);
      elsif Name = "h20-h21-both" or else Name = "it1" or else Name = "it3"
        or else Name = "it100-e8" or else Name = "e0.1" or else Name = "e1e-8" then return P (K_PB);
      elsif Name = "persp-mild" then return P (K_PM);
      elsif Name = "rot-persp" then return P (K_RP);
      elsif Name = "sst-persp" then return P (K_SP);
      elsif Name = "reversed-persp-both" then return (Mov (K_PB), Tex);
      elsif Name = "photometric-persp-both" then return (Tex, EF.Affine (Mov (K_PB), 4, 5, 20));
      elsif Name = "region-persp-mild" then
         declare
            PA : constant OpenCV.Core.Mat := EF.Texture (160, 160);
            PB : constant OpenCV.Core.Mat := EF.Homography_Texture (160, 160, K_PM);
         begin
            --  Region pixels are cloned for the checksum; Region geometry has its own test.
            return (PA.Region (Window).Clone, PB.Region (Window).Clone);
         end;
      elsif Name = "constant" then
         return (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128));
      elsif Name = "lowgrad" then
         return (EF.Affine (Tex, 1, 40, 126), EF.Affine (Mov (K_PB), 1, 40, 126));
      elsif Name = "noise-vs-texture" then return (Tex, EF.Noise (128, 128, 7));
      elsif Name = "noise-vs-noise" then return (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9));
      elsif Name = "persp-strong" then return P (K_Strong);
      elsif Name = "persp-near-zero-den" then return P (K_NZ);
      elsif Name = "persp-den-crosses-zero" then return P (K_CZ);
      elsif Name = "illcond" then return P (K_IC);
      elsif Name = "large-translation" then return P (K_LU);
      elsif Name = "reflect-x" then return P (K_RX);
      end if;
      raise Constraint_Error with "unknown homography oracle fixture " & Name;
   end Hom_Build;

   procedure Hom_Read_Lines (Path : String; Lines : out Hom_Line_Array; Count : out Natural) is
      File : Ada.Text_IO.File_Type;
   begin
      Count := 0;
      Lines := [others => Ada.Strings.Unbounded.Null_Unbounded_String];
      Ada.Text_IO.Open (File, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (File) loop
         if Count = Lines'Last then
            Ada.Text_IO.Close (File);
            raise Constraint_Error with "oracle has too many lines";
         end if;
         Count := Count + 1;
         Lines (Count) := Ada.Strings.Unbounded.To_Unbounded_String (Ada.Text_IO.Get_Line (File));
      end loop;
      Ada.Text_IO.Close (File);
   end Hom_Read_Lines;

   procedure Hom_Write_Lines (Path : String; Lines : Hom_Line_Array; Count : Natural) is
      File : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Create (File, Ada.Text_IO.Out_File, Path);
      for I in 1 .. Count loop
         Ada.Text_IO.Put_Line (File, Ada.Strings.Unbounded.To_String (Lines (I)));
      end loop;
      Ada.Text_IO.Close (File);
   end Hom_Write_Lines;

   procedure Hom_Split (Line : String; Tokens : out Hom_Token_Array; Count : out Natural) is
      First : Natural := Line'First;
   begin
      Count := 0;
      Tokens := [others => Ada.Strings.Unbounded.Null_Unbounded_String];
      while First <= Line'Last loop
         if Line (First) = ' ' then
            First := First + 1;
         else
            declare
               Last : Natural := First;
            begin
               while Last < Line'Last and then Line (Last + 1) /= ' ' loop
                  Last := Last + 1;
               end loop;
               if Count = Tokens'Last then
                  Count := Tokens'Last + 1;
                  return;
               end if;
               Count := Count + 1;
               Tokens (Count) := Ada.Strings.Unbounded.To_Unbounded_String (Line (First .. Last));
               First := Last + 1;
            end;
         end if;
      end loop;
   end Hom_Split;

   function Hom_Tok (Tokens : Hom_Token_Array; Index : Positive) return String is
     (Ada.Strings.Unbounded.To_String (Tokens (Index)));

   --  Same-build agreement: a tight absolute term plus a relative term for large
   --  ill-conditioned values (the illcond fixture returns coefficients in the hundreds).
   function Hom_Close (A, B, Absolute : Long_Float) return Boolean is
     (abs (A - B) <= Absolute + 1.0E-5 * abs (B));

   function Hom_Native_Version return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_video_native_version";

   --  Token layout of a fixture record: 1 "fixture", 2 name, 3 iterations, 4 epsilon,
   --  5 outcome, 6..14 H00..H22 (row-major), 15 correlation, 16 determinant,
   --  17 template checksum, 18 moving checksum.
   function Verify_Hom_Oracle (Path : String; Succeeded, Raised : out Natural) return String is
      Lines  : Hom_Line_Array;
      N      : Natural;
      Tok    : Hom_Token_Array;
      TC     : Natural;
      Native : constant String := Interfaces.C.Strings.Value (Hom_Native_Version);
      function Line (I : Positive) return String is (Ada.Strings.Unbounded.To_String (Lines (I)));
   begin
      Succeeded := 0;
      Raised := 0;
      begin
         Hom_Read_Lines (Path, Lines, N);
      exception
         when others => return "oracle missing or unreadable";
      end;
      if N = 0 then return "oracle is empty"; end if;
      if Line (1) /= "ecc-homography-oracle 1" then return "oracle magic or format version"; end if;
      if N < 2 or else Line (2) /= "version " & Native then return "oracle OpenCV version differs"; end if;
      if N < 3 or else Ada.Strings.Fixed.Head (Line (N), 4) /= "end " then
         return "oracle truncated (no end record)";
      end if;
      if Line (N) /= "end" & Integer'Image (Expected_Hom_Oracle_Fixtures) then
         return "oracle record count";
      end if;
      if N - 3 /= Expected_Hom_Oracle_Fixtures then return "oracle fixture record count"; end if;

      for I in 1 .. Expected_Hom_Oracle_Fixtures loop
         Hom_Split (Line (I + 2), Tok, TC);
         if TC /= 18 or else Hom_Tok (Tok, 1) /= "fixture" then return "malformed fixture record"; end if;
         declare
            Name       : constant String := Hom_Name (I);
            Iterations : constant Integer := Integer'Value (Hom_Tok (Tok, 3));
            Epsilon    : constant Long_Float := Long_Float'Value (Hom_Tok (Tok, 4));
            Outcome    : constant String := Hom_Tok (Tok, 5);
            W          : HM;
            Rho        : Long_Float;
            Pair       : constant ECC_Pair := Hom_Build (Name);
            Options    : constant ECC_Homography_Options :=
              (Maximum_Iterations => Iterations, Epsilon => OpenCV.Float64_Value (Epsilon));
         begin
            if Hom_Tok (Tok, 2) /= Name then return "fixture identity or order: " & Name; end if;
            if Outcome /= "OK" and then Outcome /= "EXC" then return "outcome token: " & Name; end if;
            for K in W'Range loop
               W (K) := Long_Float'Value (Hom_Tok (Tok, 6 + K));
            end loop;
            Rho := Long_Float'Value (Hom_Tok (Tok, 15));
            if Long_Long_Integer'Value (Hom_Tok (Tok, 17)) /= EF.Checksum (Pair.Template)
              or else Long_Long_Integer'Value (Hom_Tok (Tok, 18)) /= EF.Checksum (Pair.Moving)
            then
               return "fixture checksum differs: " & Name;
            end if;
            if Outcome = "OK" then
               declare
                  R : ECC_Homography_Result;
                  B : HM;
               begin
                  begin
                     R := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Options);
                  exception
                     when OpenCV.OpenCV_Error => return "binding raised where native succeeded: " & Name;
                  end;
                  Succeeded := Succeeded + 1;
                  B := Coeffs (R.Warp);
                  if W (8) /= 1.0 or else B (8) /= 1.0 then return "H22 is not exactly 1: " & Name; end if;
                  if not Hom_Close (B (0), W (0), 1.0E-6) then return "H00 differs: " & Name; end if;
                  if not Hom_Close (B (1), W (1), 1.0E-6) then return "H01 differs: " & Name; end if;
                  if not Hom_Close (B (2), W (2), 1.0E-5) then return "H02 differs: " & Name; end if;
                  if not Hom_Close (B (3), W (3), 1.0E-6) then return "H10 differs: " & Name; end if;
                  if not Hom_Close (B (4), W (4), 1.0E-6) then return "H11 differs: " & Name; end if;
                  if not Hom_Close (B (5), W (5), 1.0E-5) then return "H12 differs: " & Name; end if;
                  if not Hom_Close (B (6), W (6), 1.0E-8) then return "H20 differs: " & Name; end if;
                  if not Hom_Close (B (7), W (7), 1.0E-8) then return "H21 differs: " & Name; end if;
                  if abs (Long_Float (R.Correlation) - Rho) > 1.0E-6 then
                     return "correlation differs: " & Name;
                  end if;
               end;
            else
               declare
                  Accepted : Boolean := True;
               begin
                  begin
                     declare
                        Unused : constant ECC_Homography_Result :=
                          Estimate_ECC_Homography (Pair.Template, Pair.Moving, Options);
                     begin
                        null;
                     end;
                  exception
                     when OpenCV.OpenCV_Error => Accepted := False;
                  end;
                  if Accepted then return "binding succeeded where native raised: " & Name; end if;
                  Raised := Raised + 1;
               end;
            end if;
         end;
      end loop;
      return "";
   exception
      when others => return "oracle parse error";
   end Verify_Hom_Oracle;

   procedure ECC_Homography_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Succeeded, Raised : Natural;
      Defect : constant String := Verify_Hom_Oracle (Hom_Oracle_Path, Succeeded, Raised);
   begin
      Assert (Defect = "", "homography oracle: " & Defect);
      Assert (Succeeded > 0 and then Raised > 0, "homography oracle did not cover both outcomes");
      Assert (Succeeded + Raised = Expected_Hom_Oracle_Fixtures, "homography oracle outcome accounting");
   end ECC_Homography_Direct_Oracle;

   function Hom_With_Token
     (Lines : Hom_Line_Array; Line_Index, Token_Index : Positive; Value : String) return Hom_Line_Array
   is
      Result : Hom_Line_Array := Lines;
      Tok    : Hom_Token_Array;
      TC     : Natural;
      Text   : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Hom_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      Tok (Token_Index) := Ada.Strings.Unbounded.To_Unbounded_String (Value);
      for K in 1 .. TC loop
         if K > 1 then Ada.Strings.Unbounded.Append (Text, " "); end if;
         Ada.Strings.Unbounded.Append (Text, Tok (K));
      end loop;
      Result (Line_Index) := Text;
      return Result;
   end Hom_With_Token;

   function Hom_Swap_Tokens
     (Lines : Hom_Line_Array; Line_Index, A, B : Positive) return Hom_Line_Array
   is
      Tok : Hom_Token_Array;
      TC  : Natural;
   begin
      Hom_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      return Hom_With_Token
        (Hom_With_Token (Lines, Line_Index, A, Hom_Tok (Tok, B)), Line_Index, B, Hom_Tok (Tok, A));
   end Hom_Swap_Tokens;

   --  Fixture line index in the file: 2 header lines, then fixtures in table order.
   function Hom_Line_Of (Name : String) return Positive is
   begin
      for I in 1 .. Expected_Hom_Oracle_Fixtures loop
         if Hom_Name (I) = Name then return I + 2; end if;
      end loop;
      raise Constraint_Error with "no such homography oracle fixture " & Name;
   end Hom_Line_Of;

   --  Damaged copies are verified in-process with the real verifier; the oracle generator is
   --  never rerun, so a damaged file cannot be regenerated or skipped.
   procedure ECC_Homography_Oracle_Integrity (T : in out Fixture) is
      pragma Unreferenced (T);
      Original : constant String := Hom_Oracle_Path;
      Scratch  : constant String := Original & ".tampered";
      Lines    : Hom_Line_Array;
      Count    : Natural;
      Succeeded, Raised : Natural;

      procedure Expect_Rejected (Label, Fragment : String; Edited : Hom_Line_Array; Edited_Count : Natural) is
      begin
         Hom_Write_Lines (Scratch, Edited, Edited_Count);
         declare
            Defect : constant String := Verify_Hom_Oracle (Scratch, Succeeded, Raised);
         begin
            Assert (Defect /= "", "tampered homography oracle accepted: " & Label);
            Assert (Ada.Strings.Fixed.Index (Defect, Fragment) > 0,
                    "tampered homography oracle (" & Label & ") rejected for the wrong reason: " & Defect);
         end;
      end Expect_Rejected;

      function Value_Of (Line_Index, Token_Index : Positive) return Long_Float is
         Tok : Hom_Token_Array;
         TC  : Natural;
      begin
         Hom_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
         return Long_Float'Value (Hom_Tok (Tok, Token_Index));
      end Value_Of;
   begin
      Hom_Read_Lines (Original, Lines, Count);
      Assert (Count = Expected_Hom_Oracle_Fixtures + 3, "homography oracle line count");
      Assert (Verify_Hom_Oracle (Original, Succeeded, Raised) = "", "pristine homography oracle rejected");
      Assert (Verify_Hom_Oracle (Original & ".does-not-exist", Succeeded, Raised) = "oracle missing or unreadable",
              "missing homography oracle accepted");
      Expect_Rejected ("empty", "empty", Lines, 0);
      Expect_Rejected ("truncated header", "truncated", Lines, 2);
      Expect_Rejected ("truncated final record", "truncated", Lines, Count - 1);
      Expect_Rejected ("truncated mid-record", "truncated", Lines, Count - 5);
      Expect_Rejected ("wrong magic", "format version", Hom_With_Token (Lines, 1, 1, "ecc-affine-oracle"), Count);
      Expect_Rejected ("wrong format version", "format version", Hom_With_Token (Lines, 1, 2, "2"), Count);
      Expect_Rejected ("wrong OpenCV version", "OpenCV version", Hom_With_Token (Lines, 2, 2, "0.0.0"), Count);
      Expect_Rejected ("wrong declared count", "record count", Hom_With_Token (Lines, Count, 2, "36"), Count);
      declare
         Removed : Hom_Line_Array := Lines;
      begin
         for I in Hom_Line_Of ("it3") .. Count - 1 loop
            Removed (I) := Lines (I + 1);
         end loop;
         Expect_Rejected ("dropped fixture", "record count", Removed, Count - 1);
      end;
      Expect_Rejected ("renamed fixture", "fixture identity",
                       Hom_With_Token (Lines, Hom_Line_Of ("rot-persp"), 2, "rot-perspX"), Count);
      Expect_Rejected ("altered template checksum", "checksum",
                       Hom_With_Token (Lines, Hom_Line_Of ("rot-persp"), 17, "1"), Count);
      Expect_Rejected ("altered moving checksum", "checksum",
                       Hom_With_Token (Lines, Hom_Line_Of ("h20-pos"), 18, "1"), Count);
      Assert (abs (Value_Of (Hom_Line_Of ("persp-mild"), 8) - Value_Of (Hom_Line_Of ("persp-mild"), 12)) > 0.5
              and then abs (Value_Of (Hom_Line_Of ("persp-mild"), 11)
                            - Value_Of (Hom_Line_Of ("persp-mild"), 13)) > 0.1,
              "swap fixtures must have distinguishable H02/H20 and H12/H21");
      Expect_Rejected ("swapped H02 and H20", "differs",
                       Hom_Swap_Tokens (Lines, Hom_Line_Of ("persp-mild"), 8, 12), Count);
      Expect_Rejected ("swapped H12 and H21", "differs",
                       Hom_Swap_Tokens (Lines, Hom_Line_Of ("persp-mild"), 11, 13), Count);
      Expect_Rejected ("modified H20", "H20 differs",
                       Hom_With_Token (Lines, Hom_Line_Of ("h20-pos"), 12, "-0.0002"), Count);
      Expect_Rejected ("modified H21", "H21 differs",
                       Hom_With_Token (Lines, Hom_Line_Of ("h21-pos"), 13, "0.0"), Count);
      Expect_Rejected ("modified H22", "H22", Hom_With_Token (Lines, Hom_Line_Of ("rot-persp"), 14, "2.0"), Count);
      Expect_Rejected ("altered correlation", "correlation differs",
                       Hom_With_Token (Lines, Hom_Line_Of ("rot-persp"), 15, "0.5"), Count);
      Expect_Rejected ("success recorded as exception", "binding succeeded where native raised",
                       Hom_With_Token (Lines, Hom_Line_Of ("identity"), 5, "EXC"), Count);
      Expect_Rejected ("exception recorded as success", "binding raised where native succeeded",
                       Hom_With_Token (Lines, Hom_Line_Of ("reflect-x"), 5, "OK"), Count);
      Expect_Rejected ("unknown outcome token", "outcome token",
                       Hom_With_Token (Lines, Hom_Line_Of ("identity"), 5, "MAYBE"), Count);
      Ada.Directories.Delete_File (Scratch);
   end ECC_Homography_Oracle_Integrity;

   procedure ECC_Homography_Other_Models_Unchanged (T : in out Fixture) is
      pragma Unreferenced (T);
      Shift : constant ECC_Pair := Euc_Pair (0.0, 2.0, 1.0);
      Rigid : constant ECC_Pair := Euc_Pair (3.0, 2.0, 1.0);
      Trans : constant ECC_Translation_Result := Estimate_ECC_Translation (Shift.Template, Shift.Moving);
      Euc   : constant ECC_Euclidean_Result := Estimate_ECC_Euclidean (Rigid.Template, Rigid.Moving);
      Aff   : constant ECC_Affine_Result := Estimate_ECC_Affine (Shift.Template, Shift.Moving);
      Hom   : constant ECC_Homography_Result := Estimate_ECC_Homography (Shift.Template, Shift.Moving);
   begin
      Assert (ECC_Near (Trans, 2, 1), "translation ECC regression");
      Assert (Euc_Near (Euc, 3.0, 2.0, 1.0), "Euclidean ECC regression");
      Assert (Aff_Near (Aff, 1.0, 0.0, 0.0, 1.0, 2.0, 1.0), "affine ECC regression");
      Assert (Hom_Near (Hom, K_T21), "homography agrees with the pure shift");
      Assert (abs (LF (Hom.Warp.H02) - LF (Aff.Warp.M02)) < 0.1
              and then abs (LF (Hom.Warp.H12) - LF (Aff.Warp.M12)) < 0.1,
              "homography and affine models disagree on a pure shift");
   end ECC_Homography_Other_Models_Unchanged;

   ---------------------------------------------------------------------
   --  Task 017: caller-seeded ECC homography alignment
   ---------------------------------------------------------------------
   --  Truths mirror tests/cpp/ecc_homography_seeded_oracle.cpp literally.
   Hs_Strong_Truth : constant HM := [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.002, 0.002, 1.0];
   Hs_Large_Truth  : constant HM := [1.0, 0.0, 40.0, 0.0, 1.0, 30.0, 0.0, 0.0, 1.0];
   Hs_LTP_Truth    : constant HM := [1.0, 0.0, 12.0, 0.0, 1.0, -9.0, 0.0005, -0.0004, 1.0];
   Hs_Scp_Truth    : constant HM := [1.15, 0.02, -4.0, -0.01, 1.12, 3.0, 0.0008, -0.0006, 1.0];
   Hs_Shp_Truth    : constant HM := [1.0, 0.15, -3.0, 0.05, 1.0, 2.0, -0.0007, 0.0009, 1.0];

   function Hk
     (H00, H01, H02, H10, H11, H12, H20, H21 : OpenCV.Float32_Value;
      H22 : OpenCV.Float32_Value := 1.0;
      Correlation : OpenCV.Float64_Value := 0.0) return ECC_Homography_Result is
     (Warp => (H00 => H00, H01 => H01, H02 => H02, H10 => H10, H11 => H11, H12 => H12,
               H20 => H20, H21 => H21, H22 => H22),
      Correlation => Correlation);

   Hs_Identity_Seed : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
   Hs_Strong_Near   : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0018, 0.0018);
   Hs_Large_Near    : constant ECC_Homography_Result := Hk (1.0, 0.0, 38.0, 0.0, 1.0, 28.0, 0.0, 0.0);
   Hs_LTP_Near      : constant ECC_Homography_Result := Hk (1.0, 0.0, 11.0, 0.0, 1.0, -8.0, 0.0004, -0.0003);
   Hs_Reflect_Near  : constant ECC_Homography_Result := Hk (-0.98, 0.01, 126.0, 0.0, 1.0, 0.1, 0.0, 0.0);
   Hs_Scp_Near      : constant ECC_Homography_Result := Hk (1.12, 0.01, -3.0, 0.0, 1.10, 2.5, 0.0006, -0.0005);
   Hs_Shp_Near      : constant ECC_Homography_Result := Hk (1.0, 0.13, -2.5, 0.04, 1.0, 1.6, -0.0006, 0.0008);
   Hs_Mild_Near     : constant ECC_Homography_Result :=
     Hk (1.0, 0.01, 1.0, 0.0, 1.0, 0.5, 0.00004, -0.00003);
   Hs_Wrong_Persp   : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0019, 0.0019);

   --  Seeded results (measured on 4.1.0, 4.6.0, 4.10.0, 5.0.0) are within 0.004 linear,
   --  0.1 translation and 2e-5 projective of the truth; the Task 016 tolerances are kept for
   --  the projective part and widened only where the seeded fixtures are larger motions.
   Hs_Linear_Tolerance      : constant Long_Float := 0.006;
   Hs_Translation_Tolerance : constant Long_Float := 0.15;

   function Hs_Near (R : ECC_Homography_Result; H : HM) return Boolean is
     (abs (LF (R.Warp.H00) - H (0)) < Hs_Linear_Tolerance
      and then abs (LF (R.Warp.H01) - H (1)) < Hs_Linear_Tolerance
      and then abs (LF (R.Warp.H02) - H (2)) < Hs_Translation_Tolerance
      and then abs (LF (R.Warp.H10) - H (3)) < Hs_Linear_Tolerance
      and then abs (LF (R.Warp.H11) - H (4)) < Hs_Linear_Tolerance
      and then abs (LF (R.Warp.H12) - H (5)) < Hs_Translation_Tolerance
      and then abs (LF (R.Warp.H20) - H (6)) < Hom_Projective_Tolerance
      and then abs (LF (R.Warp.H21) - H (7)) < Hom_Projective_Tolerance
      and then R.Warp.H22 = 1.0);

   procedure Hs_Run
     (Pair : ECC_Pair; Initial : ECC_Homography_Result;
      Result : out ECC_Homography_Result; Raised : out Boolean;
      Options : ECC_Homography_Options := (others => <>)) is
   begin
      Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Options, Initial);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Hs_Run;

   procedure Hs_Run_Plain
     (Pair : ECC_Pair; Result : out ECC_Homography_Result; Raised : out Boolean) is
   begin
      Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Hs_Run_Plain;

   --  Native-layer failures carry the "Video.Estimate_ECC_Homography" prefix (C.Check);
   --  Ada preflight rejections do not. Reuses Sd_Outcome.
   function Hs_Outcome_Of
     (Pair : ECC_Pair; Initial : ECC_Homography_Result;
      Options : ECC_Homography_Options := (others => <>)) return Sd_Outcome
   is
      Marker : constant String := "Video.Estimate_ECC_Homography";
      Unused : ECC_Homography_Result;
   begin
      Unused := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Options, Initial);
      return Accepted;
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text : constant String := Ada.Exceptions.Exception_Message (Error);
         begin
            return (if Text'Length >= Marker'Length
                      and then Text (Text'First .. Text'First + Marker'Length - 1) = Marker
                    then Native_Failed else Preflight_Rejected);
         end;
   end Hs_Outcome_Of;

   --  Interior residual with the 24-pixel margin; a seed that maps the interior outside the
   --  image is reported as 255 per pixel by Hom_Residual.
   function Hs_Residual (Pair : ECC_Pair; R : ECC_Homography_Result) return Long_Float is
     (Hom_Residual (Pair, Coeffs (R.Warp), True));

   procedure ECC_Homography_Seeded_Identity_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      Pairs : constant array (1 .. 8) of ECC_Pair :=
        [(EF.Texture (128, 128), EF.Texture (128, 128)), Hom_Pair (K_T21), Hom_Pair (K_PB),
         Hom_Pair (K_RP), Hom_Pair (K_Strong), Hom_Pair (K_LU),
         (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128)),
         (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9))];
      With_Correlation : constant ECC_Homography_Result :=
        (Hs_Identity_Seed with delta Correlation => 0.7);
      Successes, Failures : Natural := 0;
   begin
      for Pair of Pairs loop
         declare
            Plain, Seeded, Defaulted, Correlated : ECC_Homography_Result;
            Plain_Raised, Seeded_Raised, Defaulted_Raised, Correlated_Raised : Boolean;
         begin
            Hs_Run_Plain (Pair, Plain, Plain_Raised);
            Hs_Run (Pair, Hs_Identity_Seed, Seeded, Seeded_Raised);
            Hs_Run (Pair, (others => <>), Defaulted, Defaulted_Raised);
            Hs_Run (Pair, With_Correlation, Correlated, Correlated_Raised);
            Assert (Plain_Raised = Seeded_Raised and then Plain_Raised = Defaulted_Raised
                    and then Plain_Raised = Correlated_Raised,
                    "identity homography seed changed native success or failure");
            if Plain_Raised then
               Failures := Failures + 1;
            else
               Successes := Successes + 1;
               Assert (Plain = Seeded and then Plain = Defaulted and then Plain = Correlated,
                       "identity homography seed differs from the identity-initialized operation");
            end if;
         end;
      end loop;
      Assert (Successes >= 4 and then Failures >= 3, "homography identity-equivalence coverage");
   end ECC_Homography_Seeded_Identity_Equivalence;

   procedure ECC_Homography_Seeded_Consumption (T : in out Fixture) is
      pragma Unreferenced (T);
      type Case_Data is record
         Truth : HM;
         Seed  : ECC_Homography_Result;
      end record;
      Cases : constant array (1 .. 6) of Case_Data :=
        [(Hs_Strong_Truth, Hs_Strong_Near), (Hs_Large_Truth, Hs_Large_Near),
         (Hs_LTP_Truth, Hs_LTP_Near), (K_RX, Hs_Reflect_Near),
         (Hs_Scp_Truth, Hs_Scp_Near), (Hs_Shp_Truth, Hs_Shp_Near)];
      Residual_Checked : Natural := 0;
   begin
      for Item of Cases loop
         declare
            Pair : constant ECC_Pair := Hom_Pair (Item.Truth);
            Plain, Refined : ECC_Homography_Result;
            Plain_Raised, Refined_Raised : Boolean;
            Before : constant Long_Float := Hs_Residual (Pair, Item.Seed);
         begin
            Hs_Run_Plain (Pair, Plain, Plain_Raised);
            Assert (Plain_Raised, "identity homography initialization unexpectedly converged");
            Hs_Run (Pair, Item.Seed, Refined, Refined_Raised);
            Assert (not Refined_Raised, "near-correct homography seed did not converge");
            Assert (Hs_Near (Refined, Item.Truth), "seeded homography is not near the known transform");
            Assert (Hom_Map_Error (Coeffs (Refined.Warp), Item.Truth) < 0.2, "seeded homography mapped points");
            Assert (Refined.Warp /= Item.Seed.Warp, "seeded homography result is the unmodified seed");
            Assert (Refined.Correlation > 0.98 and then Refined.Correlation <= 1.000001,
                    "seeded homography correlation");
            --  Hom_Residual counts unmapped interior pixels as 255. The metric is applicable
            --  only when the TRUTH itself maps the whole interior inside the moving image
            --  (not so for the 40/30 shift); that is a fixture property, not a result one.
            if Hom_Residual (Pair, Item.Truth, True) < 1.0 then
               Assert (Hs_Residual (Pair, Refined) < 0.1 * Before,
                       "seeded homography refinement did not reduce the image residual");
               Residual_Checked := Residual_Checked + 1;
            end if;
         end;
      end loop;
      Assert (Residual_Checked >= 5, "too few residual-checked seeded homography fixtures");
      --  An exact in-domain seed of the large translation converges near itself.
      declare
         Exact : ECC_Homography_Result;
         Raised : Boolean;
      begin
         Hs_Run (Hom_Pair (Hs_Large_Truth), Hk (1.0, 0.0, 40.0, 0.0, 1.0, 30.0, 0.0, 0.0), Exact, Raised);
         Assert (not Raised and then Hs_Near (Exact, Hs_Large_Truth), "exact homography seed");
      end;
   end ECC_Homography_Seeded_Consumption;

   procedure ECC_Homography_Seeded_Refinement_And_Direction (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair : constant ECC_Pair := Hom_Pair (Hs_LTP_Truth);
      One, Three, Many : ECC_Homography_Result;
      One_Raised, Three_Raised, Many_Raised : Boolean;
   begin
      Hs_Run (Pair, Hs_LTP_Near, One, One_Raised, (Maximum_Iterations => 1, Epsilon => 0.0001));
      Hs_Run (Pair, Hs_LTP_Near, Three, Three_Raised, (Maximum_Iterations => 3, Epsilon => 0.0001));
      Hs_Run (Pair, Hs_LTP_Near, Many, Many_Raised);
      Assert (not One_Raised and then not Three_Raised and then not Many_Raised, "seeded homography refinement runs");
      Assert (One /= Many and then One /= Three, "one, three and many iterations are indistinguishable");
      Assert (Hom_Map_Error (Coeffs (Many.Warp), Hs_LTP_Truth) < Hom_Map_Error (Coeffs (One.Warp), Hs_LTP_Truth),
              "more iterations did not move the homography closer to the truth");
      Assert (One.Correlation < Many.Correlation, "one-iteration seeded correlation");
      Assert (Estimate_ECC_Homography (Pair.Template, Pair.Moving, (others => <>), Hs_LTP_Near) = Many,
              "seeded homography not repeatable");
      --  Direction: the returned matrix is applied unchanged with WARP_INVERSE_MAP; its
      --  transpose, negated translation and swapped translation/projective terms do not align.
      declare
         W    : constant HM := Coeffs (Many.Warp);
         Good : constant Long_Float := Hom_Residual (Pair, W, True);
      begin
         Assert (Good < 1.0, "seeded homography residual did not collapse");
         Assert (Hom_Residual (Pair, Transposed (W), True) > 5.0 * Good, "transposed seeded matrix aligned");
         Assert (Hom_Residual (Pair, Negated_Translation (W), True) > 5.0 * Good, "negated translation aligned");
         Assert (Hom_Residual (Pair, Swapped (W, 2, 6), True) > 5.0 * Good, "swapped H02/H20 aligned");
         Assert (Hom_Residual (Pair, Swapped (W, 5, 7), True) > 5.0 * Good, "swapped H12/H21 aligned");
      end;
      --  The seed is consumed in storage order: swapping its translation terms changes the outcome.
      declare
         Swapped_Seed : constant ECC_Homography_Result :=
           (Hs_LTP_Near with delta Warp => (Hs_LTP_Near.Warp with delta H02 => -8.0, H12 => 11.0));
         R : ECC_Homography_Result;
         Raised : Boolean;
      begin
         Hs_Run (Pair, Swapped_Seed, R, Raised);
         Assert (Raised or else not Hs_Near (R, Hs_LTP_Truth), "seed translation order had no effect");
      end;
   end ECC_Homography_Seeded_Refinement_And_Direction;

   procedure ECC_Homography_Seeded_Correlation_Ignored (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Validity_Checks (Off);
      function F64 is new Ada.Unchecked_Conversion (Interfaces.Unsigned_64, OpenCV.Float64_Value);
      Pair : constant ECC_Pair := Hom_Pair (Hs_Strong_Truth);
      Base : ECC_Homography_Result;
      Base_Raised : Boolean;
      Correlations : constant array (1 .. 7) of OpenCV.Float64_Value :=
        [0.0, 1.0, -1.0, 12345.0, OpenCV.Float64_Value'Last,
         F64 (16#7FF8_0000_0000_0000#), F64 (16#FFF0_0000_0000_0000#)];
   begin
      Hs_Run (Pair, Hs_Strong_Near, Base, Base_Raised);
      Assert (not Base_Raised, "baseline seeded homography run");
      for C of Correlations loop
         declare
            Initial : constant ECC_Homography_Result := (Hs_Strong_Near with delta Correlation => C);
            Saved   : constant ECC_Homography_Result := Initial;
            Again   : ECC_Homography_Result;
            Raised  : Boolean;
         begin
            Hs_Run (Pair, Initial, Again, Raised);
            Assert (not Raised and then Again = Base, "initial homography correlation changed the result");
            Assert (Initial.Warp = Saved.Warp, "initial homography warp modified");
         end;
      end loop;
      --  An in-domain previous result is a valid seed for the next call; chaining is repeatable.
      declare
         Mild   : constant ECC_Pair := Hom_Pair (K_PM);
         First  : constant ECC_Homography_Result := Estimate_ECC_Homography (Mild.Template, Mild.Moving);
         Second : constant ECC_Homography_Result :=
           Estimate_ECC_Homography (Mild.Template, Mild.Moving, (others => <>), First);
         Third  : constant ECC_Homography_Result :=
           Estimate_ECC_Homography (Mild.Template, Mild.Moving, (others => <>), First);
      begin
         Assert (Hom_Near (Second, K_PM) and then Second = Third, "previous homography result reused as a seed");
      end;
   end ECC_Homography_Seeded_Correlation_Ignored;

   procedure ECC_Homography_Seeded_Immutability_And_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Saved, Contiguous : ECC_Homography_Result;
      Before_A, Before_B : Long_Long_Integer;
      Initial  : constant ECC_Homography_Result := (Hs_Mild_Near with delta Correlation => 0.25);
      Snapshot : constant ECC_Homography_Result := Initial;
      Unused   : ECC_Homography_Result;
      Raised   : Boolean;
   begin
      declare
         Parent_A : constant OpenCV.Core.Mat := EF.Texture (160, 160);
         Parent_B : constant OpenCV.Core.Mat := EF.Homography_Texture (160, 160, K_PM);
         Region_A : constant OpenCV.Core.Mat := Parent_A.Region (Window);
         Region_B : constant OpenCV.Core.Mat := Parent_B.Region (Window);
      begin
         Before_A := EF.Checksum (Parent_A);
         Before_B := EF.Checksum (Parent_B);
         Assert (not Region_A.Is_Continuous and then not Region_B.Is_Continuous,
                 "seeded homography fixture Regions are contiguous");
         Saved := Estimate_ECC_Homography (Region_A, Region_B, (others => <>), Initial);
         Contiguous := Estimate_ECC_Homography (Region_A.Clone, Region_B.Clone, (others => <>), Initial);
         Assert (Saved = Contiguous, "seeded homography Region differs from the contiguous clone");
         Assert (EF.Checksum (Parent_A) = Before_A and then EF.Checksum (Parent_B) = Before_B,
                 "seeded homography modified a Region parent");
         --  The seed domain uses the Region geometry (128), not the parent's (160): this seed
         --  is at the 128 limit and would exceed it on 160.
         Assert (Hs_Outcome_Of ((Region_A, Region_B), Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5 / 127.0, 0.0))
                 /= Preflight_Rejected, "seed domain used the Region parent geometry");
         Assert (Hs_Outcome_Of ((Parent_A, Parent_B), Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5 / 127.0, 0.0))
                 = Preflight_Rejected, "seed domain ignored the larger parent geometry");
         Hs_Run ((Parent_A, Parent_A), Initial, Unused, Raised);
         Assert (EF.Checksum (Parent_A) = Before_A, "shared allocation modified by seeded homography");
      end;
      Assert (Saved.Warp.H22 = 1.0 and then Saved.Correlation > 0.98, "seeded homography result after finalization");
      Assert (Initial = Snapshot and then Initial.Correlation = 0.25, "initial homography record changed");
      declare
         Pair : constant ECC_Pair := Hom_Pair (Hs_Strong_Truth);
         Flat : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
         Seed : constant ECC_Homography_Result := (Hs_Strong_Near with delta Correlation => 0.5);
         Copy : constant ECC_Homography_Result := Seed;
         Bad  : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.5);
         Bad_Copy : constant ECC_Homography_Result := Bad;
      begin
         Hs_Run (Pair, Seed, Unused, Raised);
         Assert (not Raised and then Seed = Copy, "initial homography record changed by refinement");
         Hs_Run ((Flat, Flat), Seed, Unused, Raised);
         Assert (Raised and then Seed = Copy, "initial homography record changed by nonconvergence");
         Hs_Run (Pair, Seed, Unused, Raised, (Maximum_Iterations => 101, Epsilon => 0.0001));
         Assert (Raised and then Seed = Copy, "initial homography record changed by invalid options");
         Hs_Run (Pair, Bad, Unused, Raised);
         Assert (Raised and then Bad = Bad_Copy, "initial homography record changed by invalid seed");
      end;
   end ECC_Homography_Seeded_Immutability_And_Regions;

   procedure ECC_Homography_Seeded_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      pragma Validity_Checks (Off);
      function F32 is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, OpenCV.Float32_Value);
      NaN : constant OpenCV.Float32_Value := F32 (16#7FC0_0000#);
      Inf : constant OpenCV.Float32_Value := F32 (16#7F80_0000#);
      Lin : constant OpenCV.Float32_Value := 4.0;
      Trn : constant OpenCV.Float32_Value := 1_048_576.0;
      Pair : constant ECC_Pair := Hom_Pair (K_PB);
      Base : constant ECC_Homography_Warp := Hs_Mild_Near.Warp;
      Nonfinite : constant array (1 .. 3) of OpenCV.Float32_Value := [NaN, Inf, -Inf];
      --  Denominator limit on 128 x 128: |H20| * 127 + |H21| * 127 <= 0.5.
      Edge : constant OpenCV.Float32_Value := 0.5 / 127.0;

      procedure Rejected (W : ECC_Homography_Warp; Message : String) is
      begin
         Assert (Hs_Outcome_Of (Pair, (Warp => W, Correlation => 0.0)) = Preflight_Rejected, Message);
      end Rejected;

      procedure Admitted (W : ECC_Homography_Warp; Message : String) is
      begin
         Assert (Hs_Outcome_Of (Pair, (Warp => W, Correlation => 0.0)) /= Preflight_Rejected, Message);
      end Admitted;

      --  Rejected, and for the documented reason (the diagnostic names the violated rule).
      procedure Rejected_For (W : ECC_Homography_Warp; Fragment, Message : String) is
         Unused : ECC_Homography_Result;
      begin
         Unused := Estimate_ECC_Homography (Pair.Template, Pair.Moving, (others => <>), (Warp => W, Correlation => 0.0));
         Assert (False, Message & ": accepted");
      exception
         when Error : OpenCV.OpenCV_Error =>
            Assert (Ada.Strings.Fixed.Index (Ada.Exceptions.Exception_Message (Error), Fragment) > 0,
                    Message & ": rejected for the wrong reason: " & Ada.Exceptions.Exception_Message (Error));
      end Rejected_For;
   begin
      for Bad of Nonfinite loop
         Rejected ((Base with delta H00 => Bad), "nonfinite H00");
         Rejected ((Base with delta H01 => Bad), "nonfinite H01");
         Rejected ((Base with delta H02 => Bad), "nonfinite H02");
         Rejected ((Base with delta H10 => Bad), "nonfinite H10");
         Rejected ((Base with delta H11 => Bad), "nonfinite H11");
         Rejected ((Base with delta H12 => Bad), "nonfinite H12");
         Rejected ((Base with delta H20 => Bad), "nonfinite H20");
         Rejected ((Base with delta H21 => Bad), "nonfinite H21");
         Rejected ((Base with delta H22 => Bad), "nonfinite H22");
         Rejected_For ((Base with delta H22 => Bad), "finite", "nonfinite H22 diagnostic");
         Rejected_For ((Base with delta H20 => Bad), "finite", "nonfinite H20 diagnostic");
      end loop;
      Rejected_For ((Base with delta H22 => 0.0), "H22 must be exactly", "H22 zero");
      Rejected ((Base with delta H22 => -1.0), "H22 negative");
      Rejected_For ((Base with delta H22 => 2.0), "H22 must be exactly", "H22 two (no normalization)");
      Rejected ((Base with delta H22 => OpenCV.Float32_Value'Succ (1.0)), "H22 just above 1");
      Rejected ((Base with delta H22 => OpenCV.Float32_Value'Pred (1.0)), "H22 just below 1");
      Rejected_For ((Base with delta H00 => OpenCV.Float32_Value'Succ (Lin)), "linear", "H00 just beyond 4");
      Rejected ((Base with delta H01 => -OpenCV.Float32_Value'Succ (Lin)), "H01 just beyond -4");
      Rejected ((Base with delta H10 => 5.0), "H10 beyond 4");
      Rejected ((Base with delta H11 => OpenCV.Float32_Value'Last), "H11 at Float32'Last");
      Rejected_For ((Base with delta H02 => OpenCV.Float32_Value'Succ (Trn)), "translations", "H02 just beyond 2**20");
      Rejected ((Base with delta H12 => -OpenCV.Float32_Value'Succ (Trn)), "H12 just beyond -2**20");
      Rejected ((Base with delta H12 => OpenCV.Float32_Value'First), "H12 at Float32'First");
      Rejected_For ((Base with delta H20 => OpenCV.Float32_Value'Succ (Edge), H21 => 0.0), "Columns-1",
                    "H20 just beyond the edge");
      Rejected ((Base with delta H20 => 0.0, H21 => -OpenCV.Float32_Value'Succ (Edge)), "H21 just beyond the edge");
      Rejected ((Base with delta H20 => 0.003, H21 => 0.001), "combined denominator deviation");
      Rejected ((Base with delta H20 => -0.0025, H21 => -0.0025), "negative combined denominator deviation");
      Rejected ((Base with delta H20 => 1.0E30, H21 => 0.0), "huge H20");
      Rejected ((Base with delta H20 => 0.0, H21 => OpenCV.Float32_Value'Last), "H21 at Float32'Last");
      --  Exact limits and singular / reflected seeds pass preflight; native outcome is compared
      --  with the direct oracle, not asserted here.
      Admitted ((H00 => Lin, H01 => -Lin, H02 => Trn, H10 => -Lin, H11 => Lin, H12 => -Trn,
                 H20 => 0.0, H21 => 0.0, H22 => 1.0), "seed exactly at the linear and translation limits");
      Admitted ((Base with delta H20 => Edge, H21 => 0.0), "H20 exactly at the edge");
      Admitted ((Base with delta H20 => 0.0, H21 => -Edge), "H21 exactly at the edge");
      Admitted ((Base with delta H20 => 0.00195, H21 => -0.00195), "mixed edge");
      Admitted ((H00 => 0.0, H01 => 0.0, H02 => 0.0, H10 => 0.0, H11 => 0.0, H12 => 0.0,
                 H20 => 0.0, H21 => 0.0, H22 => 1.0), "all-zero linear seed passes preflight");
      Admitted ((H00 => -1.0, H01 => 0.0, H02 => 127.0, H10 => 0.0, H11 => 1.0, H12 => 0.0,
                 H20 => 0.0, H21 => 0.0, H22 => 1.0), "reflected seed passes preflight");
      --  The rule depends on geometry: valid on 128 columns, rejected on 256.
      declare
         Wide : constant ECC_Pair := (EF.Texture (128, 256), EF.Texture (128, 256, 2, 1));
         Tall : constant ECC_Pair := (EF.Texture (256, 128), EF.Texture (256, 128, 2, 1));
         X_Seed : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0035, 0.0);
         Y_Seed : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0035);
      begin
         Assert (Hs_Outcome_Of (Pair, X_Seed) /= Preflight_Rejected, "H20 seed valid on 128 columns");
         Assert (Hs_Outcome_Of (Wide, X_Seed) = Preflight_Rejected, "H20 seed rule ignored the width");
         Assert (Hs_Outcome_Of (Wide, Y_Seed) /= Preflight_Rejected, "H21 seed rule used the width");
         Assert (Hs_Outcome_Of (Tall, Y_Seed) = Preflight_Rejected, "H21 seed rule ignored the height");
      end;
   end ECC_Homography_Seeded_Validation;

   procedure ECC_Homography_Seeded_Image_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Base  : constant ECC_Homography_Result := Hs_Mild_Near;
      Pair  : constant ECC_Pair := Hom_Pair (K_PB);
      Good  : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      Float_Image : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.Float32, Channels => 1));
      Color : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (128, 128, (Depth => OpenCV.Core.UInt8, Channels => 3));
      Empty : OpenCV.Core.Mat;
      Small : constant OpenCV.Core.Mat := EF.Texture (31, 128);
      Bad_Seed : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0);
      Unused : ECC_Homography_Result;
   begin
      Assert (Hs_Outcome_Of (Pair, Base, (Maximum_Iterations => 101, Epsilon => 0.0001))
              = Preflight_Rejected, "101 iterations");
      Assert (Hs_Outcome_Of (Pair, Base, (Maximum_Iterations => 50, Epsilon => 1.0))
              = Preflight_Rejected, "epsilon 1.0");
      Assert (Hs_Outcome_Of (Pair, Base, (Maximum_Iterations => 50, Epsilon => 0.0))
              = Preflight_Rejected, "epsilon 0.0");
      Assert (Hs_Outcome_Of ((Float_Image, Float_Image), Base) = Preflight_Rejected, "Float32 images");
      Assert (Hs_Outcome_Of ((Color, Color), Base) = Preflight_Rejected, "three-channel images");
      Assert (Hs_Outcome_Of ((Good, EF.Texture (128, 129)), Base) = Preflight_Rejected, "columns");
      Assert (Hs_Outcome_Of ((EF.Texture (129, 128), Good), Base) = Preflight_Rejected, "rows");
      Assert (Hs_Outcome_Of ((Small, Small), Base) = Preflight_Rejected, "31 rows");
      begin
         Unused := Estimate_ECC_Homography (Empty, Good, (others => <>), Base);
         Assert (False, "empty template accepted by the seeded homography");
      exception
         when OpenCV.OpenCV_Error => null;
      end;
      --  Image and option errors are reported before seed errors.
      begin
         Unused := Estimate_ECC_Homography (Small, Small, (others => <>), Bad_Seed);
         Assert (False, "invalid image with invalid seed accepted");
      exception
         when Error : OpenCV.OpenCV_Error =>
            Assert (Ada.Strings.Fixed.Index (Ada.Exceptions.Exception_Message (Error), "ECC image size") > 0,
                    "image validation must precede seed validation");
      end;
      Assert (Hs_Outcome_Of (Pair, Base, (Maximum_Iterations => 1, Epsilon => 1.0E-8)) /= Preflight_Rejected
              and then Hs_Outcome_Of (Pair, Base, (Maximum_Iterations => 100, Epsilon => 0.1))
                       /= Preflight_Rejected, "seeded homography option bounds are accepted");
   end ECC_Homography_Seeded_Image_Validation;

   procedure ECC_Homography_Seeded_Nonconvergence (T : in out Fixture) is
      pragma Unreferenced (T);
      Flat          : constant OpenCV.Core.Mat := EF.Constant_Image (128, 128, 128);
      Texture_Image : constant OpenCV.Core.Mat := EF.Texture (128, 128);
   begin
      Assert (Hs_Outcome_Of ((Flat, Flat), Hs_Mild_Near) = Native_Failed, "seeded constant images");
      Assert (Hs_Outcome_Of ((Flat, Texture_Image), Hs_Mild_Near) = Native_Failed, "seeded constant template");
      Assert (Hs_Outcome_Of ((EF.Noise (128, 128, 3), EF.Noise (128, 128, 9)), Hs_Mild_Near) = Native_Failed,
              "seeded independent noise");
      --  Seeds that are wrong or not close enough raise natively on every qualified version.
      Assert (Hs_Outcome_Of (Hom_Pair (Hs_Strong_Truth), Hs_Wrong_Persp) = Native_Failed,
              "wrong-direction perspective seed");
      Assert (Hs_Outcome_Of (Hom_Pair (Hs_Strong_Truth), Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0015, 0.0015))
              = Native_Failed, "insufficiently close perspective seed");
      Assert (Hs_Outcome_Of (Hom_Pair (K_PB), Hk (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)) = Native_Failed,
              "all-zero linear seed");
      --  Finite is not correct: a wrong seed with one iteration returns a finite, poorly
      --  correlated matrix that leaves the seed domain; it is published, not corrected, and
      --  passing it back as a seed is rejected by the binding.
      declare
         Pair : constant ECC_Pair := Hom_Pair (K_PB);
         R : ECC_Homography_Result;
         Raised : Boolean;
      begin
         Hs_Run (Pair, Hs_Wrong_Persp, R, Raised, (Maximum_Iterations => 1, Epsilon => 0.0001));
         Assert (not Raised and then R.Correlation < 0.5 and then R.Warp.H22 = 1.0,
                 "wrong-but-finite seeded homography outcome");
         Assert (abs LF (R.Warp.H20) * 127.0 + abs LF (R.Warp.H21) * 127.0 > 0.5,
                 "fixture no longer leaves the seed domain");
         Assert (Hs_Outcome_Of (Pair, R) = Preflight_Rejected, "out-of-domain result accepted as a seed");
      end;
      --  A seed with an extreme anisotropic linear part may be accepted natively with a
      --  meaningless matrix (4.x and 5.0 measured); nothing is fabricated.
      declare
         R : ECC_Homography_Result;
         Raised : Boolean;
      begin
         Hs_Run (Hom_Pair (K_PB), Hk (4.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0), R, Raised);
         Assert (Raised or else (R.Correlation < 0.1 and then not Hs_Near (R, K_PB)),
                 "anisotropic seed produced a confident result");
      end;
   end ECC_Homography_Seeded_Nonconvergence;

   ---------------------------------------------------------------------
   --  Task 017: independent seeded homography oracle (ecc-homography-seeded-oracle 1)
   ---------------------------------------------------------------------
   type Hs_Token_Array is array (1 .. 30) of Ada.Strings.Unbounded.Unbounded_String;

   procedure Hs_Split (Line : String; Tokens : out Hs_Token_Array; Count : out Natural) is
      First : Natural := Line'First;
   begin
      Count := 0;
      Tokens := [others => Ada.Strings.Unbounded.Null_Unbounded_String];
      while First <= Line'Last loop
         if Line (First) = ' ' then
            First := First + 1;
         else
            declare
               Last : Natural := First;
            begin
               while Last < Line'Last and then Line (Last + 1) /= ' ' loop
                  Last := Last + 1;
               end loop;
               if Count = Tokens'Last then
                  Count := Tokens'Last + 1;
                  return;
               end if;
               Count := Count + 1;
               Tokens (Count) := Ada.Strings.Unbounded.To_Unbounded_String (Line (First .. Last));
               First := Last + 1;
            end;
         end if;
      end loop;
   end Hs_Split;

   function Hs_Tok (Tokens : Hs_Token_Array; Index : Positive) return String is
     (Ada.Strings.Unbounded.To_String (Tokens (Index)));

   function Hs_With_Token
     (Lines : Hom_Line_Array; Line_Index, Token_Index : Positive; Value : String) return Hom_Line_Array
   is
      Result : Hom_Line_Array := Lines;
      Tok    : Hs_Token_Array;
      TC     : Natural;
      Text   : Ada.Strings.Unbounded.Unbounded_String;
   begin
      Hs_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      Tok (Token_Index) := Ada.Strings.Unbounded.To_Unbounded_String (Value);
      for K in 1 .. TC loop
         if K > 1 then Ada.Strings.Unbounded.Append (Text, " "); end if;
         Ada.Strings.Unbounded.Append (Text, Tok (K));
      end loop;
      Result (Line_Index) := Text;
      return Result;
   end Hs_With_Token;

   function Hs_Swap_Tokens
     (Lines : Hom_Line_Array; Line_Index, A, B : Positive) return Hom_Line_Array
   is
      Tok : Hs_Token_Array;
      TC  : Natural;
   begin
      Hs_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
      return Hs_With_Token
        (Hs_With_Token (Lines, Line_Index, A, Hs_Tok (Tok, B)), Line_Index, B, Hs_Tok (Tok, A));
   end Hs_Swap_Tokens;

   type Hs_Spec is record
      Name       : String (1 .. 20);
      Kind       : Character;
      Truth      : HM;
      Seed       : ECC_Homography_Warp;
      Iterations : Positive;
      Epsilon    : Long_Float;
   end record;

   function Hp
     (Name : String; Kind : Character; Truth : HM; Seed : ECC_Homography_Result;
      Iterations : Positive := 50; Epsilon : Long_Float := 1.0E-4) return Hs_Spec is
     (Name => Ada.Strings.Fixed.Head (Name, 20), Kind => Kind, Truth => Truth,
      Seed => Seed.Warp, Iterations => Iterations, Epsilon => Epsilon);

   Expected_Hs_Oracle_Fixtures : constant := 35;

   Hs_Table : constant array (1 .. Expected_Hs_Oracle_Fixtures) of Hs_Spec :=
     [Hp ("identity-id", 'T', K_Identity, Hs_Identity_Seed),
      Hp ("pb-id", 'T', K_PB, Hs_Identity_Seed),
      Hp ("strong-id", 'T', Hs_Strong_Truth, Hs_Identity_Seed),
      Hp ("large-id", 'T', Hs_Large_Truth, Hs_Identity_Seed),
      Hp ("reflect-id", 'T', K_RX, Hs_Identity_Seed),
      Hp ("scale-persp-id", 'T', Hs_Scp_Truth, Hs_Identity_Seed),
      Hp ("shear-persp-id", 'T', Hs_Shp_Truth, Hs_Identity_Seed),
      Hp ("strong-near", 'T', Hs_Strong_Truth, Hs_Strong_Near),
      Hp ("large-near", 'T', Hs_Large_Truth, Hs_Large_Near),
      Hp ("large-persp-near", 'T', Hs_LTP_Truth, Hs_LTP_Near),
      Hp ("reflect-near", 'T', K_RX, Hs_Reflect_Near),
      Hp ("scale-persp-near", 'T', Hs_Scp_Truth, Hs_Scp_Near),
      Hp ("shear-persp-near", 'T', Hs_Shp_Truth, Hs_Shp_Near),
      Hp ("mild-near", 'T', K_PM, Hs_Mild_Near),
      Hp ("large-exact", 'T', Hs_Large_Truth, Hk (1.0, 0.0, 40.0, 0.0, 1.0, 30.0, 0.0, 0.0)),
      Hp ("pb-wrong-persp", 'T', K_PB, Hs_Wrong_Persp),
      Hp ("strong-wrong-persp", 'T', Hs_Strong_Truth, Hs_Wrong_Persp),
      Hp ("strong-loose", 'T', Hs_Strong_Truth, Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0015, 0.0015)),
      Hp ("pb-wrong-affine", 'T', K_PB, Hk (0.8, -0.2, 6.0, 0.2, 1.25, -5.0, 0.001, -0.001)),
      Hp ("den-edge-x", 'T', K_PB, Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0039, 0.0)),
      Hp ("den-edge-mixed", 'T', K_PB, Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.00195, -0.00195)),
      Hp ("limit-linear", 'T', K_PB, Hk (4.0, 4.0, 0.0, 4.0, 4.0, 0.0, 0.0, 0.0)),
      Hp ("limit-translation", 'T', K_PB, Hk (1.0, 0.0, 1_048_576.0, 0.0, 1.0, -1_048_576.0, 0.0, 0.0)),
      Hp ("all-limits", 'T', K_PB, Hk (4.0, -4.0, 1_048_576.0, -4.0, 4.0, -1_048_576.0, 0.00195, -0.00195)),
      Hp ("singular-zero", 'T', K_PB, Hk (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
      Hp ("tiny-scale", 'T', K_PB, Hk (1.0E-6, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0)),
      Hp ("aniso-seed", 'T', K_PB, Hk (4.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0)),
      Hp ("strong-near-it1", 'T', Hs_Strong_Truth, Hs_Strong_Near, 1),
      Hp ("strong-near-it3", 'T', Hs_Strong_Truth, Hs_Strong_Near, 3),
      Hp ("strong-near-e0.1", 'T', Hs_Strong_Truth, Hs_Strong_Near, 50, 0.1),
      Hp ("strong-near-it100-e8", 'T', Hs_Strong_Truth, Hs_Strong_Near, 100, 1.0E-8),
      Hp ("pb-wrong-persp-it1", 'T', K_PB, Hs_Wrong_Persp, 1),
      Hp ("constant-seeded", 'C', K_Identity, Hs_Mild_Near),
      Hp ("noise-seeded", 'N', K_Identity, Hs_Mild_Near),
      Hp ("region-near", 'R', K_PM, Hs_Mild_Near)];

   function Hs_Name (I : Positive) return String is
     (Ada.Strings.Fixed.Trim (Hs_Table (I).Name, Ada.Strings.Both));

   function Hs_Build (Spec : Hs_Spec) return ECC_Pair is
      use type EF.Homography_Matrix;
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
   begin
      case Spec.Kind is
         when 'C' => return (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128));
         when 'N' => return (EF.Noise (128, 128, 3), EF.Noise (128, 128, 9));
         when 'R' =>
            return (EF.Texture (160, 160).Region (Window),
                    EF.Homography_Texture (160, 160, Spec.Truth).Region (Window));
         when others =>
            if Spec.Truth = K_Identity then
               return (EF.Texture (128, 128), EF.Texture (128, 128));
            end if;
            return Hom_Pair (Spec.Truth);
      end case;
   end Hs_Build;

   function Hs_Oracle_Path return String is
     (Ada.Environment_Variables.Value
        ("VIDEO_ECC_HOMOGRAPHY_SEEDED_ORACLE", "../obj/oracle/ecc-homography-seeded.txt"));

   function Hs_Seed_Values (W : ECC_Homography_Warp) return HM renames Coeffs;

   --  Strict verifier (never raises): "" when the oracle is complete, current and agrees with
   --  the binding, otherwise the first defect. Fixture record tokens: 1 fixture, 2 name,
   --  3 iterations, 4 epsilon, 5..13 seed, 14 OK|EXC, 15..23 result, 24 correlation,
   --  25 template checksum, 26 moving checksum.
   function Hs_Compare
     (Tok : Hs_Token_Array; Spec : Hs_Spec; Name : String;
      Succeeded, Raised, Nonidentity, Distinguishing : in out Natural) return String
   is
      function Tk (I : Positive) return String is (Hs_Tok (Tok, I));
      Result_Outcome : constant String := Tk (14);
      Pair    : constant ECC_Pair := Hs_Build (Spec);
      Options : constant ECC_Homography_Options :=
        (Maximum_Iterations => Spec.Iterations, Epsilon => OpenCV.Float64_Value (Spec.Epsilon));
      Initial : constant ECC_Homography_Result := (Warp => Spec.Seed, Correlation => 0.0);
      Seed    : constant HM := Hs_Seed_Values (Spec.Seed);
      Label   : constant array (0 .. 8) of String (1 .. 3) :=
        ["H00", "H01", "H02", "H10", "H11", "H12", "H20", "H21", "H22"];
      S, W    : HM;
      R, Plain : ECC_Homography_Result;
      Binding_Raised, Plain_Raised : Boolean;
   begin
      if Integer'Value (Tk (3)) /= Spec.Iterations
        or else abs (Long_Float'Value (Tk (4)) - Spec.Epsilon) > 1.0E-15
      then
         return "fixture options differ: " & Name;
      end if;
      if Result_Outcome /= "OK" and then Result_Outcome /= "EXC" then return "outcome token: " & Name; end if;
      for K in S'Range loop
         S (K) := Long_Float'Value (Tk (5 + K));
         W (K) := Long_Float'Value (Tk (15 + K));
         if abs (S (K) - Seed (K)) > 1.0E-12 * (1.0 + abs Seed (K)) then
            return "seed " & Label (K) & " differs: " & Name;
         end if;
      end loop;
      if Long_Long_Integer'Value (Tk (25)) /= EF.Checksum (Pair.Template)
        or else Long_Long_Integer'Value (Tk (26)) /= EF.Checksum (Pair.Moving)
      then
         return "fixture checksum differs: " & Name;
      end if;
      if Spec.Seed /= Hs_Identity_Seed.Warp then Nonidentity := Nonidentity + 1; end if;
      Hs_Run (Pair, Initial, R, Binding_Raised, Options);
      if Result_Outcome = "OK" then
         if Binding_Raised then return "binding raised where native succeeded: " & Name; end if;
         Succeeded := Succeeded + 1;
         declare
            B : constant HM := Coeffs (R.Warp);
         begin
            if W (8) /= 1.0 or else B (8) /= 1.0 then return "H22 is not exactly 1: " & Name; end if;
            for K in 0 .. 7 loop
               declare
                  Absolute : constant Long_Float :=
                    (if K = 2 or else K = 5 then 1.0E-5 elsif K >= 6 then 1.0E-8 else 1.0E-6);
               begin
                  if not Hom_Close (B (K), W (K), Absolute) then return Label (K) & " differs: " & Name; end if;
               end;
            end loop;
         end;
         if abs (Long_Float (R.Correlation) - Long_Float'Value (Tk (24))) > 1.0E-6 then
            return "correlation differs: " & Name;
         end if;
         if Spec.Kind = 'T' and then Spec.Seed /= Hs_Identity_Seed.Warp and then Hs_Near (R, Spec.Truth) then
            Hs_Run_Plain (Pair, Plain, Plain_Raised);
            if Plain_Raised then Distinguishing := Distinguishing + 1; end if;
         end if;
      else
         if not Binding_Raised then return "binding succeeded where native raised: " & Name; end if;
         Raised := Raised + 1;
      end if;
      return "";
   end Hs_Compare;

   function Verify_Hs_Oracle
     (Path : String; Succeeded, Raised, Nonidentity, Distinguishing : out Natural) return String
   is
      Lines : Hom_Line_Array;
      N     : Natural;
      Tok   : Hs_Token_Array;
      TC    : Natural;
      Native : constant String := Interfaces.C.Strings.Value (Hom_Native_Version);
      function Line (I : Positive) return String is (Ada.Strings.Unbounded.To_String (Lines (I)));
   begin
      Succeeded := 0; Raised := 0; Nonidentity := 0; Distinguishing := 0;
      begin
         Hom_Read_Lines (Path, Lines, N);
      exception
         when others => return "oracle missing or unreadable";
      end;
      if N = 0 then return "oracle is empty"; end if;
      if Line (1) /= "ecc-homography-seeded-oracle 1" then return "oracle magic or format version"; end if;
      if N < 2 or else Line (2) /= "version " & Native then return "oracle OpenCV version differs"; end if;
      if N < 3 or else Ada.Strings.Fixed.Head (Line (N), 4) /= "end " then
         return "oracle truncated (no end record)";
      end if;
      if Line (N) /= "end" & Integer'Image (Expected_Hs_Oracle_Fixtures) then
         return "oracle record count";
      end if;
      if N - 3 /= Expected_Hs_Oracle_Fixtures then return "oracle fixture record count"; end if;
      for I in 1 .. Expected_Hs_Oracle_Fixtures loop
         Hs_Split (Line (I + 2), Tok, TC);
         if TC /= 26 or else Hs_Tok (Tok, 1) /= "fixture" then
            return "malformed fixture record";
         end if;
         if Hs_Tok (Tok, 2) /= Hs_Name (I) then
            return "fixture identity or order: " & Hs_Name (I);
         end if;
         declare
            Defect : constant String :=
              Hs_Compare (Tok, Hs_Table (I), Hs_Name (I), Succeeded, Raised, Nonidentity, Distinguishing);
         begin
            if Defect /= "" then return Defect; end if;
         end;
      end loop;
      return "";
   exception
      when others => return "oracle parse error";
   end Verify_Hs_Oracle;

   procedure ECC_Homography_Seeded_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Succeeded, Raised, Nonidentity, Distinguishing : Natural;
      Defect : constant String :=
        Verify_Hs_Oracle (Hs_Oracle_Path, Succeeded, Raised, Nonidentity, Distinguishing);
   begin
      Assert (Defect = "", "seeded homography oracle: " & Defect);
      Assert (Succeeded > 0 and then Raised > 0, "seeded homography oracle did not cover both outcomes");
      Assert (Succeeded + Raised = Expected_Hs_Oracle_Fixtures, "seeded homography oracle outcome accounting");
      Assert (Nonidentity >= 25, "seeded homography oracle needs genuinely nonidentity seeds");
      Assert (Distinguishing >= 6, "seeded homography oracle has too few identity-fails / seeded-succeeds fixtures");
   end ECC_Homography_Seeded_Direct_Oracle;

   procedure ECC_Homography_Seeded_Oracle_Integrity (T : in out Fixture) is
      pragma Unreferenced (T);
      Original : constant String := Hs_Oracle_Path;
      Scratch  : constant String := Original & ".tampered";
      Lines    : Hom_Line_Array;
      Count    : Natural;
      A, B, C, D : Natural;

      function Line_Of (Name : String) return Positive is
      begin
         for I in 1 .. Expected_Hs_Oracle_Fixtures loop
            if Hs_Name (I) = Name then return I + 2; end if;
         end loop;
         raise Constraint_Error with "no such seeded homography oracle fixture " & Name;
      end Line_Of;

      procedure Expect_Rejected (Label, Fragment : String; Edited : Hom_Line_Array; Edited_Count : Natural) is
      begin
         Hom_Write_Lines (Scratch, Edited, Edited_Count);
         declare
            Defect : constant String := Verify_Hs_Oracle (Scratch, A, B, C, D);
         begin
            Assert (Defect /= "", "tampered seeded homography oracle accepted: " & Label);
            Assert (Ada.Strings.Fixed.Index (Defect, Fragment) > 0,
                    "tampered seeded homography oracle (" & Label & ") rejected for the wrong reason: " & Defect);
         end;
      end Expect_Rejected;

      function Value_Of (Line_Index, Token_Index : Positive) return Long_Float is
         Tok : Hs_Token_Array;
         TC  : Natural;
      begin
         Hs_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
         return Long_Float'Value (Hs_Tok (Tok, Token_Index));
      end Value_Of;

      Near : Positive;
   begin
      Hom_Read_Lines (Original, Lines, Count);
      Near := Line_Of ("large-persp-near");
      Assert (Count = Expected_Hs_Oracle_Fixtures + 3, "seeded homography oracle line count");
      Assert (Verify_Hs_Oracle (Original, A, B, C, D) = "", "pristine seeded homography oracle rejected");
      Assert (Verify_Hs_Oracle (Original & ".does-not-exist", A, B, C, D) = "oracle missing or unreadable",
              "missing seeded homography oracle accepted");
      Expect_Rejected ("empty", "empty", Lines, 0);
      Expect_Rejected ("truncated", "truncated", Lines, Count - 1);
      Expect_Rejected ("truncated mid-record", "truncated", Lines, Count - 5);
      Expect_Rejected ("wrong magic", "format version", Hs_With_Token (Lines, 1, 1, "ecc-homography-oracle"), Count);
      Expect_Rejected ("wrong format version", "format version", Hs_With_Token (Lines, 1, 2, "2"), Count);
      Expect_Rejected ("wrong OpenCV version", "OpenCV version", Hs_With_Token (Lines, 2, 2, "0.0.0"), Count);
      Expect_Rejected ("wrong declared count", "record count", Hs_With_Token (Lines, Count, 2, "34"), Count);
      Expect_Rejected ("renamed fixture", "fixture identity", Hs_With_Token (Lines, Near, 2, "large-persp-nearX"), Count);
      Expect_Rejected ("altered options", "options", Hs_With_Token (Lines, Near, 3, "49"), Count);
      Expect_Rejected ("altered template checksum", "checksum", Hs_With_Token (Lines, Near, 25, "1"), Count);
      Expect_Rejected ("altered moving checksum", "checksum", Hs_With_Token (Lines, Near, 26, "1"), Count);
      Expect_Rejected ("altered seed H02", "seed H02", Hs_With_Token (Lines, Near, 7, "9.0"), Count);
      Expect_Rejected ("swapped seed H02 and H12", "seed H02", Hs_Swap_Tokens (Lines, Near, 7, 10), Count);
      Expect_Rejected ("swapped seed H20 and H21", "seed H20", Hs_Swap_Tokens (Lines, Near, 11, 12), Count);
      Expect_Rejected ("altered seed H22", "seed H22", Hs_With_Token (Lines, Near, 13, "2.0"), Count);
      Assert (abs (Value_Of (Near, 17) - Value_Of (Near, 21)) > 0.5
              and then abs (Value_Of (Near, 20) - Value_Of (Near, 22)) > 0.5,
              "swap fixtures must have distinguishable H02/H20 and H12/H21");
      Expect_Rejected ("swapped output H02 and H20", "differs", Hs_Swap_Tokens (Lines, Near, 17, 21), Count);
      Expect_Rejected ("swapped output H12 and H21", "differs", Hs_Swap_Tokens (Lines, Near, 20, 22), Count);
      Expect_Rejected ("transposed output H01/H10", "differs", Hs_Swap_Tokens (Lines, Near, 16, 18), Count);
      Expect_Rejected ("altered output H20", "H20 differs", Hs_With_Token (Lines, Near, 21, "0.0"), Count);
      Expect_Rejected ("altered output H22", "H22", Hs_With_Token (Lines, Near, 23, "2.0"), Count);
      Expect_Rejected ("altered correlation", "correlation differs", Hs_With_Token (Lines, Near, 24, "0.5"), Count);
      Expect_Rejected ("success recorded as exception", "binding succeeded where native raised",
                       Hs_With_Token (Lines, Line_Of ("identity-id"), 14, "EXC"), Count);
      Expect_Rejected ("exception recorded as success", "binding raised where native succeeded",
                       Hs_With_Token (Lines, Line_Of ("strong-id"), 14, "OK"), Count);
      Expect_Rejected ("unknown outcome token", "outcome token",
                       Hs_With_Token (Lines, Line_Of ("identity-id"), 14, "MAYBE"), Count);
      Ada.Directories.Delete_File (Scratch);
   end ECC_Homography_Seeded_Oracle_Integrity;

   ---------------------------------------------------------------------
   --  Task 018: moving-image-masked ECC homography
   ---------------------------------------------------------------------
   subtype Hm_Mask is EF.Mask_Spec;

   function Mask_Full (V : Natural := 255) return Hm_Mask is (Kind => 'F', V => V, others => 0);
   function Mask_Hole (X, Y, W, H : Integer; V : Natural := 255) return Hm_Mask is
     (Kind => 'H', A => X, B => Y, C => W, D => H, V => V);
   --  The asymmetric reference hole: columns 70 .. 109, rows 10 .. 39 of the MOVING image.
   Hm_Hole       : constant Hm_Mask := Mask_Hole (70, 10, 40, 30);
   Hm_Hole_Swap  : constant Hm_Mask := Mask_Hole (10, 70, 30, 40);
   Hm_Multi      : constant Hm_Mask := (Kind => 'M', V => 255, others => 0);
   Hm_Zero       : constant Hm_Mask := (Kind => 'Z', V => 0, others => 0);
   --  Occluder rectangles (moving pixels) and the masks excluding them grown by 3 pixels.
   Hm_Occ_T      : constant OpenCV.Rect := (64, 8, 56, 48);
   Hm_Occ_P      : constant OpenCV.Rect := (60, 4, 64, 52);
   Hm_Mask_T     : constant Hm_Mask := Mask_Hole (61, 5, 62, 54);
   Hm_Mask_P     : constant Hm_Mask := Mask_Hole (57, 1, 70, 58);

   type Hm_Mask_Array is array (Positive range <>) of Hm_Mask;

   function Hm_Mat (Spec : Hm_Mask) return OpenCV.Core.Mat is (EF.Mask (Spec));

   function Hm_Occluded (Truth : HM; Area : OpenCV.Rect; Kind : Natural) return ECC_Pair is
     (Template => EF.Texture (128, 128),
      Moving   => EF.Occlude (EF.Homography_Texture (128, 128, Truth),
                              Natural (Area.X), Natural (Area.Y),
                              Natural (Area.Width), Natural (Area.Height), Kind));

   procedure Hm_Run
     (Pair : ECC_Pair; Mask : OpenCV.Core.Mat;
      Result : out ECC_Homography_Result; Raised : out Boolean;
      Options : ECC_Homography_Options := (others => <>)) is
   begin
      Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Mask, Options);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Hm_Run;

   procedure Hm_Run_Seeded
     (Pair : ECC_Pair; Mask : OpenCV.Core.Mat; Initial : ECC_Homography_Result;
      Result : out ECC_Homography_Result; Raised : out Boolean;
      Options : ECC_Homography_Options := (others => <>)) is
   begin
      Result := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Mask, Options, Initial);
      Raised := False;
   exception
      when OpenCV.OpenCV_Error => Result := (others => <>); Raised := True;
   end Hm_Run_Seeded;

   --  Preflight versus native-layer classification (native messages carry the C.Check prefix).
   function Hm_Outcome_Of
     (Pair : ECC_Pair; Mask : OpenCV.Core.Mat; Seeded : Boolean;
      Initial : ECC_Homography_Result := Hs_Identity_Seed;
      Options : ECC_Homography_Options := (others => <>)) return Sd_Outcome
   is
      Marker : constant String := "Video.Estimate_ECC_Homography";
      Unused : ECC_Homography_Result;
   begin
      if Seeded then
         Unused := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Mask, Options, Initial);
      else
         Unused := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Mask, Options);
      end if;
      return Accepted;
   exception
      when Error : OpenCV.OpenCV_Error =>
         declare
            Text : constant String := Ada.Exceptions.Exception_Message (Error);
         begin
            return (if Text'Length >= Marker'Length
                      and then Text (Text'First .. Text'First + Marker'Length - 1) = Marker
                    then Native_Failed else Preflight_Rejected);
         end;
   end Hm_Outcome_Of;

   function Hm_Message
     (Pair : ECC_Pair; Mask : OpenCV.Core.Mat; Seeded : Boolean;
      Initial : ECC_Homography_Result := Hs_Identity_Seed) return String
   is
      Unused : ECC_Homography_Result;
   begin
      if Seeded then
         Unused := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Mask, (others => <>), Initial);
      else
         Unused := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Mask);
      end if;
      return "";
   exception
      when Error : OpenCV.OpenCV_Error => return Ada.Exceptions.Exception_Message (Error);
   end Hm_Message;

   function Contains (Text, Fragment : String) return Boolean is
     (Ada.Strings.Fixed.Index (Text, Fragment) > 0);

   --  Same outcome; on success bitwise-equal coefficients and correlation (Ada "=" on Float32 and
   --  Float64 is exact for finite values, which are the only values the binding publishes).
   function Hm_Same
     (A : ECC_Homography_Result; A_Raised : Boolean;
      B : ECC_Homography_Result; B_Raised : Boolean) return Boolean is
     (A_Raised = B_Raised and then (A_Raised or else A = B));

   procedure ECC_Homography_Masked_Full_Equivalence (T : in out Fixture) is
      pragma Unreferenced (T);
      Pairs : constant array (1 .. 5) of ECC_Pair :=
        [(EF.Texture (128, 128), EF.Texture (128, 128)), Hom_Pair (K_T21), Hom_Pair (K_PM),
         Hom_Pair (Hs_Scp_Truth), (EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128))];
      Masks : constant array (1 .. 4) of OpenCV.Core.Mat :=
        [Hm_Mat (Mask_Full (255)), Hm_Mat (Mask_Full (1)), Hm_Mat (Mask_Full (7)),
         Hm_Mat ((Kind => 'A', V => 99, others => 0))];
      Plain, Masked, Seeded_Plain, Seeded_Masked : ECC_Homography_Result;
      P_Raised, M_Raised, SP_Raised, SM_Raised : Boolean;
      Successes, Failures : Natural := 0;
   begin
      for Pair of Pairs loop
         Hs_Run_Plain (Pair, Plain, P_Raised);
         Hs_Run (Pair, Hs_Scp_Near, Seeded_Plain, SP_Raised);
         if P_Raised then Failures := Failures + 1; else Successes := Successes + 1; end if;
         for Mask of Masks loop
            Hm_Run (Pair, Mask, Masked, M_Raised);
            Assert (Hm_Same (Plain, P_Raised, Masked, M_Raised),
                    "all-included mask differs from the unmasked identity-initialized call");
            Hm_Run_Seeded (Pair, Mask, Hs_Scp_Near, Seeded_Masked, SM_Raised);
            Assert (Hm_Same (Seeded_Plain, SP_Raised, Seeded_Masked, SM_Raised),
                    "all-included mask differs from the unmasked seeded call");
         end loop;
      end loop;
      Assert (Successes >= 3 and then Failures >= 1, "full-mask equivalence needs both outcomes");
   end ECC_Homography_Masked_Full_Equivalence;

   procedure ECC_Homography_Masked_Value_Semantics (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair : constant ECC_Pair := Hom_Pair (K_T21);
      Plain, R255, R1, RA, RM, RSeed255, RSeed1 : ECC_Homography_Result;
      P_Raised, Raised : Boolean;
   begin
      Hs_Run_Plain (Pair, Plain, P_Raised);
      Hm_Run (Pair, Hm_Mat (Hm_Hole), R255, Raised);
      Assert (not P_Raised and then not Raised, "reference hole fixture must converge");
      Hm_Run (Pair, Hm_Mat (Mask_Hole (70, 10, 40, 30, 1)), R1, Raised);
      Assert (not Raised and then R1 = R255, "mask values 1 and 255 must be equivalent");
      Hm_Run (Pair, Hm_Mat ((Kind => 'A', A => 70, B => 10, C => 40, D => 30, V => 5)), RA, Raised);
      Assert (not Raised and then RA = R255, "arbitrary nonzero mask values must be equivalent to 255");
      Assert (R255 /= Plain, "an excluded rectangle must change the native result");
      Assert (R255.Warp.H22 = 1.0 and then Hom_Near (R255, K_T21), "masked translation is not near (2, 1)");
      Hm_Run (Pair, Hm_Mat (Hm_Multi), RM, Raised);
      Assert (not Raised and then RM /= R255 and then RM /= Plain and then Hom_Near (RM, K_T21),
              "several disjoint excluded regions");
      Hm_Run_Seeded (Hom_Pair (Hs_LTP_Truth), Hm_Mat (Hm_Hole), Hs_LTP_Near, RSeed255, Raised);
      Assert (not Raised, "seeded hole fixture must converge");
      Hm_Run_Seeded (Hom_Pair (Hs_LTP_Truth), Hm_Mat (Mask_Hole (70, 10, 40, 30, 1)), Hs_LTP_Near, RSeed1, Raised);
      Assert (not Raised and then RSeed1 = RSeed255, "seeded: mask values 1 and 255 must be equivalent");
   end ECC_Homography_Masked_Value_Semantics;

   --  Occlusion: excluding the occluded rectangle measurably improves the alignment relative to
   --  the same image pair without a mask (identity start).
   procedure ECC_Homography_Masked_Occlusion_Improves (T : in out Fixture) is
      pragma Unreferenced (T);
      Shifted : constant ECC_Pair := Hm_Occluded (K_T21, Hm_Occ_T, 2);
      Mild    : constant ECC_Pair := Hm_Occluded (K_PM, Hm_Occ_P, 2);
      Block   : constant ECC_Pair := Hm_Occluded (K_T21, Hm_Occ_T, 0);
      Clean, Plain, Masked : ECC_Homography_Result;
      C_Raised, P_Raised, M_Raised : Boolean;
   begin
      Hs_Run_Plain (Hom_Pair (K_T21), Clean, C_Raised);
      Hs_Run_Plain (Shifted, Plain, P_Raised);
      Hm_Run (Shifted, Hm_Mat (Hm_Mask_T), Masked, M_Raised);
      Assert (not C_Raised and then not P_Raised and then not M_Raised, "occlusion fixture outcomes");
      --  Measured (4.1.0 .. 5.0.0): unmasked 2.9 px, masked 0.02-0.07 px at the reference points.
      Assert (Hom_Map_Error (Coeffs (Plain.Warp), K_T21) > 2.0,
              "misleading occluder must corrupt the unmasked estimate");
      Assert (Hom_Map_Error (Coeffs (Masked.Warp), K_T21) < 0.2 and then Hom_Near (Masked, K_T21),
              "masking the occluder must recover the translation");
      Assert (Hom_Map_Error (Coeffs (Masked.Warp), K_T21) * 10.0 < Hom_Map_Error (Coeffs (Plain.Warp), K_T21),
              "masked estimate must be an order of magnitude better");
      Assert (Masked.Correlation > Plain.Correlation, "masked correlation is not higher");
      Hs_Run_Plain (Mild, Plain, P_Raised);
      Hm_Run (Mild, Hm_Mat (Hm_Mask_P), Masked, M_Raised);
      Assert (not P_Raised and then not M_Raised, "perspective occlusion outcomes");
      Assert (Hom_Map_Error (Coeffs (Masked.Warp), K_PM) < 0.3
              and then Hom_Map_Error (Coeffs (Plain.Warp), K_PM) > 3.0,
              "perspective occlusion: masked must be accurate and unmasked wrong");
      --  A saturated block makes unmasked ECC raise natively; the masked call converges.
      Hs_Run_Plain (Block, Plain, P_Raised);
      Hm_Run (Block, Hm_Mat (Hm_Mask_T), Masked, M_Raised);
      Assert (P_Raised and then not M_Raised and then Hom_Near (Masked, K_T21),
              "block occluder: unmasked raises, masked recovers");
   end ECC_Homography_Masked_Occlusion_Improves;

   procedure ECC_Homography_Masked_Seeded_Occlusion (T : in out Fixture) is
      pragma Unreferenced (T);
      Block   : constant ECC_Pair := Hm_Occluded (Hs_LTP_Truth, Hm_Occ_P, 0);
      Shifted : constant ECC_Pair := Hm_Occluded (Hs_LTP_Truth, Hm_Occ_P, 2);
      Scp     : constant ECC_Pair := Hm_Occluded (Hs_Scp_Truth, Hm_Occ_P, 2);
      Plain, Masked, Id_Masked : ECC_Homography_Result;
      P_Raised, M_Raised, I_Raised : Boolean;
   begin
      Hs_Run (Block, Hs_LTP_Near, Plain, P_Raised);
      Hm_Run_Seeded (Block, Hm_Mat (Hm_Mask_P), Hs_LTP_Near, Masked, M_Raised);
      Assert (P_Raised, "seeded block occluder: unmasked must raise natively");
      Assert (not M_Raised and then Hom_Map_Error (Coeffs (Masked.Warp), Hs_LTP_Truth) < 0.5,
              "seeded block occluder: masked must recover the homography");
      --  The initial estimate is required: the same mask from identity does not converge.
      Hm_Run (Block, Hm_Mat (Hm_Mask_P), Id_Masked, I_Raised);
      Assert (I_Raised or else Hom_Map_Error (Coeffs (Id_Masked.Warp), Hs_LTP_Truth) > 1.0,
              "identity start must not reach the seeded masked result");
      Hs_Run (Shifted, Hs_LTP_Near, Plain, P_Raised);
      Hm_Run_Seeded (Shifted, Hm_Mat (Hm_Mask_P), Hs_LTP_Near, Masked, M_Raised);
      Assert (not P_Raised and then not M_Raised, "seeded shifted occluder outcomes");
      Assert (Hom_Map_Error (Coeffs (Masked.Warp), Hs_LTP_Truth) < 0.5
              and then Hom_Map_Error (Coeffs (Plain.Warp), Hs_LTP_Truth) > 3.0,
              "seeded shifted occluder: masked must improve the estimate");
      Hs_Run (Scp, Hs_Scp_Near, Plain, P_Raised);
      Hm_Run_Seeded (Scp, Hm_Mat (Hm_Mask_P), Hs_Scp_Near, Masked, M_Raised);
      Assert (not P_Raised and then not M_Raised
              and then Hom_Map_Error (Coeffs (Masked.Warp), Hs_Scp_Truth) * 5.0
                       < Hom_Map_Error (Coeffs (Plain.Warp), Hs_Scp_Truth),
              "seeded scale+perspective occluder: masked must improve the estimate");
   end ECC_Homography_Masked_Seeded_Occlusion;

   --  The mask is interpreted in MOVING coordinates: the same rectangle placed in the template
   --  frame (moving minus the LTP translation) or transposed gives a measurably different result.
   procedure ECC_Homography_Masked_Coordinate_Frame (T : in out Fixture) is
      pragma Unreferenced (T);
      Block : constant ECC_Pair := Hm_Occluded (Hs_LTP_Truth, Hm_Occ_P, 0);
      Shift : constant ECC_Pair := Hm_Occluded (K_T21, Hm_Occ_T, 2);
      Right, Template_Frame, Transposed : ECC_Homography_Result;
      R_Raised, F_Raised, X_Raised : Boolean;
      Pair : constant ECC_Pair := Hom_Pair (K_T21);
      Hole, Swap : ECC_Homography_Result;
   begin
      Hm_Run_Seeded (Block, Hm_Mat (Hm_Mask_P), Hs_LTP_Near, Right, R_Raised);
      Hm_Run_Seeded (Block, Hm_Mat (Mask_Hole (45, 10, 70, 58)), Hs_LTP_Near, Template_Frame, F_Raised);
      Assert (not R_Raised and then Hom_Map_Error (Coeffs (Right.Warp), Hs_LTP_Truth) < 0.5,
              "moving-frame mask recovers the homography");
      Assert (F_Raised or else Hom_Map_Error (Coeffs (Template_Frame.Warp), Hs_LTP_Truth) > 2.0,
              "template-frame mask must not behave like the moving-frame mask");
      Hm_Run (Shift, Hm_Mat (Hm_Mask_T), Right, R_Raised);
      Hm_Run (Shift, Hm_Mat (Mask_Hole (5, 61, 54, 62)), Transposed, X_Raised);
      Assert (not R_Raised and then Hom_Near (Right, K_T21), "moving-frame occlusion mask");
      Assert (X_Raised or else Hom_Map_Error (Coeffs (Transposed.Warp), K_T21) > 3.0,
              "transposed (rows/columns swapped) mask must not recover the translation");
      Hm_Run (Pair, Hm_Mat (Hm_Hole), Hole, R_Raised);
      Hm_Run (Pair, Hm_Mat (Hm_Hole_Swap), Swap, X_Raised);
      Assert (not R_Raised and then not X_Raised and then Hole /= Swap,
              "an asymmetric hole and its transpose must give different results");
   end ECC_Homography_Masked_Coordinate_Frame;

   procedure ECC_Homography_Masked_Numerics (T : in out Fixture) is
      pragma Unreferenced (T);
      R : ECC_Homography_Result;
      Raised : Boolean;
      Identity_Pair : constant ECC_Pair := (EF.Texture (128, 128), EF.Texture (128, 128));
      One, Many : ECC_Homography_Result;
      O_Raised, M_Raised : Boolean;
      Truths : constant array (1 .. 6) of HM := [K_T21, K_Tm, K_AC, K_SHX, K_PM, K_SP];
   begin
      Hm_Run (Identity_Pair, Hm_Mat (Hm_Hole), R, Raised);
      Assert (not Raised and then Hom_Near (R, K_Identity) and then R.Correlation > 0.999,
              "masked identity alignment");
      for Truth of Truths loop
         Hm_Run (Hom_Pair (Truth), Hm_Mat (Hm_Hole), R, Raised);
         --  Task 017 tolerances: excluding 1200 pixels costs up to 0.1 px of translation (K_SP).
         Assert (not Raised and then Hs_Near (R, Truth), "masked translation/affine/perspective fixture");
         Assert (R.Correlation > 0.9 and then R.Correlation <= 1.0, "masked correlation range");
      end loop;
      --  Seeded perspective that needs the initial estimate.
      Hm_Run (Hom_Pair (Hs_Strong_Truth), Hm_Mat (Hm_Hole), R, Raised);
      Assert (Raised or else not Hs_Near (R, Hs_Strong_Truth), "strong perspective must need a seed");
      Hm_Run_Seeded (Hom_Pair (Hs_Strong_Truth), Hm_Mat (Hm_Hole), Hs_Strong_Near, R, Raised);
      Assert (not Raised and then Hs_Near (R, Hs_Strong_Truth), "seeded masked strong perspective");
      --  Options reach the masked native call.
      Hm_Run (Hom_Pair (K_T21), Hm_Mat (Hm_Hole), One, O_Raised, (Maximum_Iterations => 1, Epsilon => 0.0001));
      Hm_Run (Hom_Pair (K_T21), Hm_Mat (Hm_Hole), Many, M_Raised);
      Assert (not O_Raised and then not M_Raised and then One /= Many
              and then Hom_Map_Error (Coeffs (Many.Warp), K_T21) < Hom_Map_Error (Coeffs (One.Warp), K_T21),
              "masked iteration option has no effect");
      Assert (Estimate_ECC_Homography (Hom_Pair (K_T21).Template, Hom_Pair (K_T21).Moving, Hm_Mat (Hm_Hole)) = Many,
              "masked call is not repeatable");
   end ECC_Homography_Masked_Numerics;

   procedure ECC_Homography_Masked_Support (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair : constant ECC_Pair := Hom_Pair (K_T21);
      Shift_60 : constant ECC_Homography_Result := Hk (1.0, 0.0, 60.0, 0.0, 1.0, 0.0, 0.0, 0.0);
      Left : constant OpenCV.Core.Mat := Hm_Mat ((Kind => 'L', A => 24, V => 255, others => 0));
      Eroded : constant Hm_Mask_Array :=
        [(Kind => 'P', A => 64, B => 64, V => 255, others => 0),
         (Kind => 'G', A => 2, V => 255, others => 0),
         (Kind => 'S', A => 60, B => 4, V => 255, others => 0),
         (Kind => 'B', A => 50, B => 40, C => 5, V => 255, others => 0),
         (Kind => 'D', A => 50, V => 255, others => 0)];
      R : ECC_Homography_Result;
      Raised : Boolean;
   begin
      --  Entirely excluded: rejected by the binding before native execution (both modes).
      Assert (Hm_Outcome_Of (Pair, Hm_Mat (Hm_Zero), False) = Native_Failed
              and then Contains (Hm_Message (Pair, Hm_Mat (Hm_Zero), False), "excludes every pixel"),
              "all-zero mask must be rejected with the binding diagnostic");
      Assert (Contains (Hm_Message (Pair, Hm_Mat (Hm_Zero), True, Hs_LTP_Near), "excludes every pixel"),
              "seeded all-zero mask must be rejected with the binding diagnostic");
      --  Nonzero support that native erosion removes: native nonconvergence (NaN), not a fake result.
      for Spec of Eroded loop
         Assert (EF.Nonzero (Hm_Mat (Spec)) > 0, "sparse fixture must have nonzero pixels");
         Assert (Hm_Outcome_Of (Pair, Hm_Mat (Spec), False) = Native_Failed
                 and then Contains (Hm_Message (Pair, Hm_Mat (Spec), False), "NaN encountered"),
                 "eroded-away sparse mask must raise native nonconvergence");
      end loop;
      --  Very sparse but surviving support: native success (possibly inaccurate); never invented.
      Hm_Run (Pair, Hm_Mat ((Kind => 'D', A => 90, V => 255, others => 0)), R, Raised);
      Assert (not Raised and then R.Warp.H22 = 1.0, "90% density mask converges natively");
      --  Valid moving region that no longer overlaps after warping.
      --  A narrow valid band still overlaps from the identity start: native success, but the
      --  estimate is finite and inaccurate (measured 2.3 px); the binding does not hide that.
      Hm_Run (Pair, Left, R, Raised);
      Assert (not Raised and then R.Warp.H22 = 1.0 and then Hom_Map_Error (Coeffs (R.Warp), K_T21) > 0.5,
              "left band with identity start: finite, inaccurate native result expected");
      Assert (Hm_Outcome_Of (Pair, Left, True, Shift_60) = Native_Failed
              and then Contains (Hm_Message (Pair, Left, True, Shift_60), "NaN encountered"),
              "non-overlapping valid region must raise natively");
      Assert (Hm_Outcome_Of (Pair, Left, True, Shift_60, (Maximum_Iterations => 1, Epsilon => 0.0001))
              = Native_Failed, "non-overlap after the first warp");
      --  Constant valid support and constant images.
      declare
         K : ECC_Pair := Hom_Pair (K_T21);
      begin
         EF.Fill (K.Template, 20, 20, 50, 50, 100);
         EF.Fill (K.Moving, 20, 20, 50, 50, 100);
         Assert (Hm_Outcome_Of (K, Hm_Mat ((Kind => 'Q', V => 255, others => 0)), False) = Native_Failed,
                 "constant intensity over the valid support must raise");
      end;
      Assert (Hm_Outcome_Of ((EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128)),
                             Hm_Mat (Mask_Full), False) = Native_Failed, "constant images with a full mask");
      Assert (Hm_Outcome_Of ((EF.Noise (128, 128, 3), EF.Noise (128, 128, 9)), Hm_Mat (Hm_Hole), False)
              = Native_Failed, "noise with a hole mask");
   end ECC_Homography_Masked_Support;

   procedure ECC_Homography_Masked_Validation (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair  : constant ECC_Pair := Hom_Pair (K_T21);
      Good  : constant OpenCV.Core.Mat := Hm_Mat (Hm_Hole);
      function M (Rows, Columns : Positive; Depth : OpenCV.Core.Depth_Type := OpenCV.Core.UInt8;
                  Channels : OpenCV.Core.Channel_Count := 1) return OpenCV.Core.Mat is
        (OpenCV.Core.Create (Rows, Columns, (Depth => Depth, Channels => Channels)));
      Empty : OpenCV.Core.Mat;
      Wrong : constant array (1 .. 12) of OpenCV.Core.Mat :=
        [Empty, M (128, 128, OpenCV.Core.Int8), M (128, 128, OpenCV.Core.UInt16),
         M (128, 128, OpenCV.Core.Float32), M (128, 128, OpenCV.Core.Float64),
         M (128, 128, OpenCV.Core.UInt8, 2), M (128, 128, OpenCV.Core.UInt8, 3),
         M (127, 128), M (128, 127), M (129, 128), M (64, 64), M (1, 1)];
      Three_D : constant OpenCV.Core.Mat :=
        OpenCV.Core.Create (OpenCV.Core.Dimension_Array'[128, 128, 1], (Depth => OpenCV.Core.UInt8, Channels => 1));
      Bad_Seed : constant ECC_Homography_Result := Hk (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.01, 0.0);
      Non_Unit : constant ECC_Homography_Result :=
        (Hs_Identity_Seed with delta Warp => (Hs_Identity_Seed.Warp with delta H22 => 2.0));
   begin
      for Mask of Wrong loop
         Assert (Hm_Outcome_Of (Pair, Mask, False) = Preflight_Rejected, "invalid mask must be rejected in Ada");
         Assert (Hm_Outcome_Of (Pair, Mask, True) = Preflight_Rejected, "invalid seeded mask must be rejected in Ada");
      end loop;
      Assert (Hm_Outcome_Of (Pair, Three_D, False) = Preflight_Rejected, "3-D mask must be rejected");
      Assert (Contains (Hm_Message (Pair, Empty, False), "nonempty"), "empty mask diagnostic");
      Assert (Contains (Hm_Message (Pair, M (128, 128, OpenCV.Core.Float32), False), "UInt8 C1"), "depth diagnostic");
      Assert (Contains (Hm_Message (Pair, M (128, 128, OpenCV.Core.UInt8, 3), False), "UInt8 C1"), "channel diagnostic");
      Assert (Contains (Hm_Message (Pair, M (127, 128), False), "geometry"), "row geometry diagnostic");
      Assert (Contains (Hm_Message (Pair, M (128, 127), False), "geometry"), "column geometry diagnostic");
      --  Image/option contract first, then mask, then seed (all preflight).
      Assert (Contains (Hm_Message ((EF.Texture (128, 128), EF.Texture (64, 64)), Empty, False), "ECC image"),
              "image validation precedes mask validation");
      begin
         declare
            Unused : constant ECC_Homography_Result :=
              Estimate_ECC_Homography (Pair.Template, Pair.Moving, Empty, (Maximum_Iterations => 101, Epsilon => 0.0001));
         begin
            Assert (False, "invalid options accepted");
         end;
      exception
         when Error : OpenCV.OpenCV_Error =>
            Assert (Contains (Ada.Exceptions.Exception_Message (Error), "Maximum_Iterations"),
                    "option validation precedes mask validation");
      end;
      Assert (Contains (Hm_Message (Pair, Empty, True, Bad_Seed), "moving mask"),
              "mask validation precedes seed validation");
      Assert (Contains (Hm_Message (Pair, Good, True, Bad_Seed), "<= 0.5"), "seed domain still enforced");
      Assert (Contains (Hm_Message (Pair, Good, True, Non_Unit), "H22"), "seed H22 still enforced");
      Assert (Hm_Outcome_Of (Pair, Good, True, Hk (1.0, 0.0, 2.0E6, 0.0, 1.0, 0.0, 0.0, 0.0)) = Preflight_Rejected,
              "seed translation bound still enforced");
      Assert (Hm_Outcome_Of (Pair, Good, False) = Accepted and then Hm_Outcome_Of (Pair, Good, True) = Accepted,
              "valid mask accepted in both modes");
      --  Non-square geometry: 96 rows x 128 columns. The matching mask is accepted; the transposed
      --  128 x 96 shape (same pixel count) is rejected in Ada and in the shim.
      declare
         Wide : constant ECC_Pair := (EF.Texture (96, 128), EF.Texture (96, 128, 2, 1));
         Wide_Mask : OpenCV.Core.Mat := EF.Constant_Image (96, 128, 255);
         Tall_Mask : constant OpenCV.Core.Mat := EF.Constant_Image (128, 96, 255);
         R : ECC_Homography_Result;
         Raised : Boolean;
      begin
         EF.Fill (Wide_Mask, 80, 8, 30, 20, 0);
         Hm_Run (Wide, Wide_Mask, R, Raised);
         Assert (not Raised and then abs (LF (R.Warp.H02) - 2.0) < 0.1 and then abs (LF (R.Warp.H12) - 1.0) < 0.1,
                 "non-square masked translation");
         Assert (Hm_Outcome_Of (Wide, Wide_Mask, True) = Accepted, "non-square seeded mask accepted");
         Assert (Hm_Outcome_Of (Wide, Tall_Mask, False) = Preflight_Rejected
                 and then Hm_Outcome_Of (Wide, Tall_Mask, True) = Preflight_Rejected,
                 "transposed-shape mask must be rejected");
         Assert (Contains (Hm_Message (Wide, Tall_Mask, False), "geometry"), "transposed-shape diagnostic");
      end;
   end ECC_Homography_Masked_Validation;

   procedure ECC_Homography_Masked_Immutability_And_Regions (T : in out Fixture) is
      pragma Unreferenced (T);
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Pair   : constant ECC_Pair := Hom_Pair (K_PM);
      Mask   : constant OpenCV.Core.Mat := Hm_Mat (Hm_Hole);
      Mask_Sum : constant Long_Long_Integer := EF.Checksum (Mask);
      T_Sum  : constant Long_Long_Integer := EF.Checksum (Pair.Template);
      M_Sum  : constant Long_Long_Integer := EF.Checksum (Pair.Moving);
      Contiguous, From_Region, Seeded_Contiguous, Seeded_Region : ECC_Homography_Result;
      Raised : Boolean;
      Initial : constant ECC_Homography_Result := (Hs_Mild_Near with delta Correlation => 0.25);
      Initial_Copy : constant ECC_Homography_Result := Initial;
   begin
      Hm_Run (Pair, Mask, Contiguous, Raised);
      Assert (not Raised, "reference masked call");
      Hm_Run_Seeded (Pair, Mask, Initial, Seeded_Contiguous, Raised);
      Assert (not Raised, "reference seeded masked call");
      Assert (EF.Checksum (Mask) = Mask_Sum and then EF.Checksum (Pair.Template) = T_Sum
              and then EF.Checksum (Pair.Moving) = M_Sum and then Initial = Initial_Copy,
              "mask, images or Initial_Transform were modified");
      --  Non-contiguous Region mask (and images) over larger parents: bitwise equal results; the
      --  parents, including their margins, are unchanged.
      declare
         Mask_Parent : OpenCV.Core.Mat := EF.Constant_Image (160, 160, 0);
         Region_Pair : ECC_Pair;
      begin
         for R in 0 .. 127 loop
            for C in 0 .. 127 loop
               OpenCV.Core.UInt8_Access.Set
                 (Mask_Parent, 16 + R, 16 + C, OpenCV.Core.UInt8_Access.Get (Mask, R, C));
            end loop;
         end loop;
         declare
            Parent_Sum  : constant Long_Long_Integer := EF.Checksum (Mask_Parent);
            Mask_Region : constant OpenCV.Core.Mat := Mask_Parent.Region (Window);
            Moving_Parent : constant OpenCV.Core.Mat := EF.Constant_Image (160, 160, 9);
         begin
            Assert (not Mask_Region.Is_Continuous, "mask Region fixture is contiguous");
            Hm_Run (Pair, Mask_Region, From_Region, Raised);
            Assert (not Raised and then From_Region = Contiguous, "non-contiguous Region mask differs");
            Hm_Run_Seeded (Pair, Mask_Region, Initial, Seeded_Region, Raised);
            Assert (not Raised and then Seeded_Region = Seeded_Contiguous, "seeded Region mask differs");
            Assert (EF.Checksum (Mask_Parent) = Parent_Sum, "mask Region parent modified");
            --  All three inputs as Regions of independent parents.
            Region_Pair := (EF.Texture (160, 160).Region (Window),
                            EF.Homography_Texture (160, 160, K_PM).Region (Window));
            Hm_Run (Region_Pair, Mask_Region, From_Region, Raised);
            Assert (not Raised and then From_Region.Warp.H22 = 1.0, "all-Region masked call");
            --  Wrong Region geometry from the same parent is rejected.
            Assert (Hm_Outcome_Of (Pair, Mask_Parent.Region ((16, 16, 127, 128)), False) = Preflight_Rejected,
                    "short Region mask accepted");
            Assert (EF.Checksum (Moving_Parent) = EF.Checksum (EF.Constant_Image (160, 160, 9)),
                    "unrelated parent changed");
         end;
      end;
      --  Region parent lifetime: the Region alone keeps the storage alive.
      declare
         function Detached_Mask return OpenCV.Core.Mat is
            Parent : OpenCV.Core.Mat := EF.Constant_Image (160, 160, 0);
         begin
            for R in 0 .. 127 loop
               for C in 0 .. 127 loop
                  OpenCV.Core.UInt8_Access.Set
                    (Parent, 16 + R, 16 + C, OpenCV.Core.UInt8_Access.Get (Mask, R, C));
               end loop;
            end loop;
            return Parent.Region (Window);
         end Detached_Mask;
         Survivor : constant OpenCV.Core.Mat := Detached_Mask;
      begin
         Hm_Run (Pair, Survivor, From_Region, Raised);
         Assert (not Raised and then From_Region = Contiguous, "Region outliving its parent handle differs");
      end;
      --  Aliasing: the moving image itself as the mask (texture pixels are all nonzero, so
      --  this is an all-included mask) and the same Mat as template and moving.
      declare
         Plain, Aliased_Result : ECC_Homography_Result;
         P_Raised, A_Raised : Boolean;
         Same : constant OpenCV.Core.Mat := EF.Texture (128, 128);
      begin
         Assert (EF.Nonzero (Pair.Moving) = 128 * 128, "aliasing fixture has zero pixels");
         Hs_Run_Plain (Pair, Plain, P_Raised);
         Hm_Run (Pair, Pair.Moving, Aliased_Result, A_Raised);
         Assert (Hm_Same (Plain, P_Raised, Aliased_Result, A_Raised), "moving image used as its own mask");
         Assert (EF.Checksum (Pair.Moving) = M_Sum, "aliased moving image modified");
         Hm_Run ((Same, Same), Mask, Aliased_Result, A_Raised);
         Assert (not A_Raised and then Hom_Near (Aliased_Result, K_Identity), "same Mat as template and moving");
      end;
   end ECC_Homography_Masked_Immutability_And_Regions;

   --  Existing overloads are source- and behavior-compatible: the unmasked identity and seeded
   --  calls still match their Task 016 / 017 oracles (re-verified by those tests); here they are
   --  checked against masked calls with all-included masks and against each other.
   procedure ECC_Homography_Masked_Existing_Unchanged (T : in out Fixture) is
      pragma Unreferenced (T);
      Pair : constant ECC_Pair := Hom_Pair (K_SP);
      Plain, Seeded_Identity, Masked_Full, Masked_Hole : ECC_Homography_Result;
   begin
      Plain := Estimate_ECC_Homography (Pair.Template, Pair.Moving);
      Seeded_Identity := Estimate_ECC_Homography (Pair.Template, Pair.Moving, (others => <>), Hs_Identity_Seed);
      Masked_Full := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Hm_Mat (Mask_Full));
      Masked_Hole := Estimate_ECC_Homography (Pair.Template, Pair.Moving, Hm_Mat (Hm_Hole));
      Assert (Plain = Seeded_Identity and then Plain = Masked_Full, "unmasked/seeded/full-mask disagree");
      Assert (Masked_Hole /= Plain, "hole mask has no effect");
      Assert (Estimate_ECC_Homography (Pair.Template, Pair.Moving) = Plain, "unmasked call changed after masked calls");
   end ECC_Homography_Masked_Existing_Unchanged;

   ---------------------------------------------------------------------
   --  Task 018: independent masked homography oracle (ecc-homography-masked-oracle 1)
   ---------------------------------------------------------------------
   type Hm_Occlusion is record
      X, Y, W, H, Kind : Natural := 0;
      Present          : Boolean := False;
   end record;

   type Hm_Spec is record
      Name       : String (1 .. 26);
      Image      : Character;
      Truth      : HM;
      Occ        : Hm_Occlusion;
      Seed       : ECC_Homography_Warp;
      Mask       : Hm_Mask;
      Iterations : Positive;
      Epsilon    : Long_Float;
   end record;

   No_Occ : constant Hm_Occlusion := (others => <>);
   function Occ_T (Kind : Natural) return Hm_Occlusion is (64, 8, 56, 48, Kind, True);
   function Occ_P (Kind : Natural) return Hm_Occlusion is (60, 4, 64, 52, Kind, True);

   function Hq
     (Name : String; Image : Character; Truth : HM; Occ : Hm_Occlusion;
      Seed : ECC_Homography_Result; Mask : Hm_Mask;
      Iterations : Positive := 50; Epsilon : Long_Float := 1.0E-4) return Hm_Spec is
     (Name => Ada.Strings.Fixed.Head (Name, 26), Image => Image, Truth => Truth, Occ => Occ,
      Seed => Seed.Warp, Mask => Mask, Iterations => Iterations, Epsilon => Epsilon);

   function Mk (Kind : Character; A, B, C, D : Integer := 0; V : Natural := 255) return Hm_Mask is
     (Kind => Kind, A => A, B => B, C => C, D => D, V => V);

   Hm_Seed60 : constant ECC_Homography_Result := Hk (1.0, 0.0, 60.0, 0.0, 1.0, 0.0, 0.0, 0.0);

   Expected_Hm_Oracle_Fixtures : constant := 53;

   --  Mirrors fixtures() in tests/cpp/ecc_homography_masked_oracle.cpp, in order.
   Hm_Table : constant array (1 .. Expected_Hm_Oracle_Fixtures) of Hm_Spec :=
     [Hq ("identity-full", 'T', K_Identity, No_Occ, Hs_Identity_Seed, Mask_Full),
      Hq ("trans-full255", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mask_Full),
      Hq ("trans-full1", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mask_Full (1)),
      Hq ("trans-full7", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mask_Full (7)),
      Hq ("trans-full-arbitrary", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('A', V => 99)),
      Hq ("trans-hole255", 'T', K_T21, No_Occ, Hs_Identity_Seed, Hm_Hole),
      Hq ("trans-hole1", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mask_Hole (70, 10, 40, 30, 1)),
      Hq ("trans-hole-arbitrary", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('A', 70, 10, 40, 30, 5)),
      Hq ("trans-hole-transposed", 'T', K_T21, No_Occ, Hs_Identity_Seed, Hm_Hole_Swap),
      Hq ("trans-multi-holes", 'T', K_T21, No_Occ, Hs_Identity_Seed, Hm_Multi),
      Hq ("mild-full", 'T', K_PM, No_Occ, Hs_Identity_Seed, Mask_Full),
      Hq ("mild-hole", 'T', K_PM, No_Occ, Hs_Identity_Seed, Hm_Hole),
      Hq ("occ-trans-shifted-full", 'O', K_T21, Occ_T (2), Hs_Identity_Seed, Mask_Full),
      Hq ("occ-trans-shifted-masked", 'O', K_T21, Occ_T (2), Hs_Identity_Seed, Hm_Mask_T),
      Hq ("occ-trans-shifted-transp", 'O', K_T21, Occ_T (2), Hs_Identity_Seed, Mask_Hole (5, 61, 54, 62)),
      Hq ("occ-trans-block-full", 'O', K_T21, Occ_T (0), Hs_Identity_Seed, Mask_Full),
      Hq ("occ-trans-block-masked", 'O', K_T21, Occ_T (0), Hs_Identity_Seed, Hm_Mask_T),
      Hq ("occ-mild-shifted-full", 'O', K_PM, Occ_P (2), Hs_Identity_Seed, Mask_Full),
      Hq ("occ-mild-shifted-masked", 'O', K_PM, Occ_P (2), Hs_Identity_Seed, Hm_Mask_P),
      Hq ("occ-mild-noise-masked", 'O', K_PM, Occ_P (1), Hs_Identity_Seed, Hm_Mask_P),
      Hq ("occ-mild-stripes-masked", 'O', K_PM, Occ_P (3), Hs_Identity_Seed, Hm_Mask_P),
      Hq ("ltp-near-full", 'T', Hs_LTP_Truth, No_Occ, Hs_LTP_Near, Mask_Full),
      Hq ("ltp-near-hole", 'T', Hs_LTP_Truth, No_Occ, Hs_LTP_Near, Hm_Hole),
      Hq ("ltp-id-hole", 'T', Hs_LTP_Truth, No_Occ, Hs_Identity_Seed, Hm_Hole),
      Hq ("occ-ltp-block-full", 'O', Hs_LTP_Truth, Occ_P (0), Hs_LTP_Near, Mask_Full),
      Hq ("occ-ltp-block-masked", 'O', Hs_LTP_Truth, Occ_P (0), Hs_LTP_Near, Hm_Mask_P),
      Hq ("occ-ltp-block-tmplframe", 'O', Hs_LTP_Truth, Occ_P (0), Hs_LTP_Near, Mask_Hole (45, 10, 70, 58)),
      Hq ("occ-ltp-shifted-full", 'O', Hs_LTP_Truth, Occ_P (2), Hs_LTP_Near, Mask_Full),
      Hq ("occ-ltp-shifted-masked", 'O', Hs_LTP_Truth, Occ_P (2), Hs_LTP_Near, Hm_Mask_P),
      Hq ("occ-scp-shifted-full", 'O', Hs_Scp_Truth, Occ_P (2), Hs_Scp_Near, Mask_Full),
      Hq ("occ-scp-shifted-masked", 'O', Hs_Scp_Truth, Occ_P (2), Hs_Scp_Near, Hm_Mask_P),
      Hq ("scp-near-multi-holes", 'T', Hs_Scp_Truth, No_Occ, Hs_Scp_Near, Hm_Multi),
      Hq ("strong-near-hole", 'T', Hs_Strong_Truth, No_Occ, Hs_Strong_Near, Hm_Hole),
      Hq ("all-zero", 'T', K_T21, No_Occ, Hs_Identity_Seed, Hm_Zero),
      Hq ("single-pixel", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('P', 64, 64)),
      Hq ("grid-2", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('G', 2)),
      Hq ("row-strip-4", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('S', 60, 4)),
      Hq ("row-strip-5", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('S', 60, 5)),
      Hq ("block-5", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('B', 50, 40, 5)),
      Hq ("block-12", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('B', 50, 40, 12)),
      Hq ("density-50", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('D', 50)),
      Hq ("density-90", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('D', 90)),
      Hq ("left-band-id", 'T', K_T21, No_Occ, Hs_Identity_Seed, Mk ('L', 24)),
      Hq ("left-band-seed60", 'T', K_T21, No_Occ, Hm_Seed60, Mk ('L', 24)),
      Hq ("left-band-seed60-it1", 'T', K_T21, No_Occ, Hm_Seed60, Mk ('L', 24), 1),
      Hq ("constant-support", 'K', K_T21, No_Occ, Hs_Identity_Seed, Mk ('Q')),
      Hq ("constant-full", 'C', K_Identity, No_Occ, Hs_Identity_Seed, Mask_Full),
      Hq ("noise-hole", 'N', K_Identity, No_Occ, Hs_Identity_Seed, Hm_Hole),
      Hq ("trans-hole-it1", 'T', K_T21, No_Occ, Hs_Identity_Seed, Hm_Hole, 1),
      Hq ("trans-hole-it100-e8", 'T', K_T21, No_Occ, Hs_Identity_Seed, Hm_Hole, 100, 1.0E-8),
      Hq ("trans-hole-e0.1", 'T', K_T21, No_Occ, Hs_Identity_Seed, Hm_Hole, 50, 0.1),
      Hq ("region-mild-hole", 'R', K_PM, No_Occ, Hs_Identity_Seed, Hm_Hole),
      Hq ("region-ltp-near-hole", 'R', Hs_LTP_Truth, No_Occ, Hs_LTP_Near, Hm_Hole)];

   function Hm_Name (I : Positive) return String is
     (Ada.Strings.Fixed.Trim (Hm_Table (I).Name, Ada.Strings.Both));

   type Hm_Inputs is record
      Pair : ECC_Pair;
      Mask : OpenCV.Core.Mat;
   end record;

   function Hm_Build (Spec : Hm_Spec) return Hm_Inputs is
      use type EF.Homography_Matrix;
      Window : constant OpenCV.Rect := (16, 16, 128, 128);
      Mask   : constant OpenCV.Core.Mat := Hm_Mat (Spec.Mask);
   begin
      case Spec.Image is
         when 'C' =>
            return ((EF.Constant_Image (128, 128, 128), EF.Constant_Image (128, 128, 128)), Mask);
         when 'N' => return ((EF.Noise (128, 128, 3), EF.Noise (128, 128, 9)), Mask);
         when 'K' =>
            declare
               P : ECC_Pair := Hom_Pair (Spec.Truth);
            begin
               EF.Fill (P.Template, 20, 20, 50, 50, 100);
               EF.Fill (P.Moving, 20, 20, 50, 50, 100);
               return (P, Mask);
            end;
         when 'R' =>
            declare
               Parent : OpenCV.Core.Mat := EF.Constant_Image (160, 160, 0);
            begin
               for R in 0 .. 127 loop
                  for C in 0 .. 127 loop
                     OpenCV.Core.UInt8_Access.Set
                       (Parent, 16 + R, 16 + C, OpenCV.Core.UInt8_Access.Get (Mask, R, C));
                  end loop;
               end loop;
               return ((EF.Texture (160, 160).Region (Window),
                        EF.Homography_Texture (160, 160, Spec.Truth).Region (Window)),
                       Parent.Region (Window));
            end;
         when others =>
            declare
               P : ECC_Pair :=
                 (if Spec.Truth = K_Identity then (EF.Texture (128, 128), EF.Texture (128, 128))
                  else Hom_Pair (Spec.Truth));
            begin
               if Spec.Image = 'O' then
                  P.Moving := EF.Occlude (P.Moving, Spec.Occ.X, Spec.Occ.Y, Spec.Occ.W, Spec.Occ.H,
                                          Spec.Occ.Kind);
               end if;
               return (P, Mask);
            end;
      end case;
   end Hm_Build;

   function Hm_Oracle_Path return String is
     (Ada.Environment_Variables.Value
        ("VIDEO_ECC_HOMOGRAPHY_MASKED_ORACLE", "../obj/oracle/ecc-homography-masked.txt"));

   function Hex_Value (Text : String) return Interfaces.Unsigned_64 is
      use type Interfaces.Unsigned_64;
      Value : Interfaces.Unsigned_64 := 0;
   begin
      if Text'Length = 0 or else Text'Length > 16 then
         raise Constraint_Error with "hex token";
      end if;
      for Ch of Text loop
         Value := Value * 16 +
           (case Ch is
               when '0' .. '9' => Character'Pos (Ch) - Character'Pos ('0'),
               when 'a' .. 'f' => Character'Pos (Ch) - Character'Pos ('a') + 10,
               when others => raise Constraint_Error with "hex digit");
      end loop;
      return Value;
   end Hex_Value;

   function F32_Bits (V : OpenCV.Float32_Value) return Interfaces.Unsigned_32 is
      function To is new Ada.Unchecked_Conversion (OpenCV.Float32_Value, Interfaces.Unsigned_32);
   begin
      return To (V);
   end F32_Bits;

   function F64_Bits (V : OpenCV.Float64_Value) return Interfaces.Unsigned_64 is
      function To is new Ada.Unchecked_Conversion (OpenCV.Float64_Value, Interfaces.Unsigned_64);
   begin
      return To (V);
   end F64_Bits;

   type Hm_Bits is array (0 .. 8) of Interfaces.Unsigned_32;

   function Warp_Bits (W : ECC_Homography_Warp) return Hm_Bits is
     [F32_Bits (W.H00), F32_Bits (W.H01), F32_Bits (W.H02), F32_Bits (W.H10), F32_Bits (W.H11),
      F32_Bits (W.H12), F32_Bits (W.H20), F32_Bits (W.H21), F32_Bits (W.H22)];

   function Hex8 (V : Interfaces.Unsigned_32) return String is
      use type Interfaces.Unsigned_32;
      Digits_Of : constant String := "0123456789abcdef";
      Result    : String (1 .. 8);
      X         : Interfaces.Unsigned_32 := V;
   begin
      for I in reverse Result'Range loop
         Result (I) := Digits_Of (Integer (X mod 16) + 1);
         X := X / 16;
      end loop;
      return Result;
   end Hex8;

   --  Fixture record tokens: 1 fixture, 2 name, 3 iterations, 4 epsilon, 5..13 seed bits,
   --  14 OK|EXC, 15..23 result bits, 24 correlation bits, 25 template checksum, 26 moving
   --  checksum, 27 mask checksum, 28 mask nonzero count, 29 unmasked-equal, 30 error class.
   function Hm_Compare
     (Tok : Hs_Token_Array; Spec : Hm_Spec; Name : String;
      Succeeded, Raised, Effect, Seeded : in out Natural) return String
   is
      use type Interfaces.Unsigned_32;
      use type Interfaces.Unsigned_64;
      function Tk (I : Positive) return String is (Hs_Tok (Tok, I));
      Outcome : constant String := Tk (14);
      Class   : constant String := Tk (30);
      Inputs  : constant Hm_Inputs := Hm_Build (Spec);
      Options : constant ECC_Homography_Options :=
        (Maximum_Iterations => Spec.Iterations, Epsilon => OpenCV.Float64_Value (Spec.Epsilon));
      Initial : constant ECC_Homography_Result := (Warp => Spec.Seed, Correlation => 0.0);
      Is_Identity_Seed : constant Boolean := Spec.Seed = Hs_Identity_Seed.Warp;
      Seed_Bits : constant Hm_Bits := Warp_Bits (Spec.Seed);
      Label   : constant array (0 .. 8) of String (1 .. 3) :=
        ["H00", "H01", "H02", "H10", "H11", "H12", "H20", "H21", "H22"];
      R, Via_Seed : ECC_Homography_Result;
      Binding_Raised, Seed_Raised : Boolean;
   begin
      if Integer'Value (Tk (3)) /= Spec.Iterations
        or else abs (Long_Float'Value (Tk (4)) - Spec.Epsilon) > 1.0E-15
      then
         return "fixture options differ: " & Name;
      end if;
      if Outcome /= "OK" and then Outcome /= "EXC" then return "outcome token: " & Name; end if;
      if Tk (29) /= "0" and then Tk (29) /= "1" then return "unmasked-equal token: " & Name; end if;
      if Class /= "NONE" and then Class /= "NAN" and then Class /= "LAMBDA" and then Class /= "OTHER" then
         return "error class token: " & Name;
      end if;
      if (Outcome = "OK") /= (Class = "NONE") then return "error class inconsistent: " & Name; end if;
      for K in 0 .. 8 loop
         if Interfaces.Unsigned_32 (Hex_Value (Tk (5 + K))) /= Seed_Bits (K) then
            return "seed " & Label (K) & " differs: " & Name;
         end if;
      end loop;
      if Long_Long_Integer'Value (Tk (25)) /= EF.Checksum (Inputs.Pair.Template)
        or else Long_Long_Integer'Value (Tk (26)) /= EF.Checksum (Inputs.Pair.Moving)
      then
         return "fixture checksum differs: " & Name;
      end if;
      if Long_Long_Integer'Value (Tk (27)) /= EF.Checksum (Inputs.Mask)
        or else Natural'Value (Tk (28)) /= EF.Nonzero (Inputs.Mask)
      then
         return "mask checksum differs: " & Name;
      end if;
      if Spec.Image = 'R' and then Inputs.Mask.Is_Continuous then
         return "Region mask fixture is contiguous: " & Name;
      end if;
      if Tk (29) = "0" then Effect := Effect + 1; end if;
      if not Is_Identity_Seed then
         Seeded := Seeded + 1;
         Hm_Run_Seeded (Inputs.Pair, Inputs.Mask, Initial, R, Binding_Raised, Options);
      else
         Hm_Run (Inputs.Pair, Inputs.Mask, R, Binding_Raised, Options);
         --  The identity seed through the masked seeded overload is the same native call.
         Hm_Run_Seeded (Inputs.Pair, Inputs.Mask, Initial, Via_Seed, Seed_Raised, Options);
         if not Hm_Same (R, Binding_Raised, Via_Seed, Seed_Raised) then
            return "masked identity seed differs from the masked identity overload: " & Name;
         end if;
      end if;
      if Outcome = "OK" then
         if Binding_Raised then return "binding raised where native succeeded: " & Name; end if;
         Succeeded := Succeeded + 1;
         declare
            B : constant Hm_Bits := Warp_Bits (R.Warp);
         begin
            for K in 0 .. 8 loop
               if Interfaces.Unsigned_32 (Hex_Value (Tk (15 + K))) /= B (K) then
                  return Label (K) & " differs: " & Name;
               end if;
            end loop;
            if R.Warp.H22 /= 1.0 then return "H22 is not exactly 1: " & Name; end if;
         end;
         if Hex_Value (Tk (24)) /= F64_Bits (R.Correlation) then
            return "correlation differs: " & Name;
         end if;
      else
         if not Binding_Raised then return "binding succeeded where native raised: " & Name; end if;
         Raised := Raised + 1;
      end if;
      return "";
   end Hm_Compare;

   function Verify_Hm_Oracle
     (Path : String; Succeeded, Raised, Effect, Seeded : out Natural) return String
   is
      Lines  : Hom_Line_Array;
      N      : Natural;
      Tok    : Hs_Token_Array;
      TC     : Natural;
      Native : constant String := Interfaces.C.Strings.Value (Hom_Native_Version);
      function Line (I : Positive) return String is (Ada.Strings.Unbounded.To_String (Lines (I)));
   begin
      Succeeded := 0; Raised := 0; Effect := 0; Seeded := 0;
      begin
         Hom_Read_Lines (Path, Lines, N);
      exception
         when others => return "oracle missing or unreadable";
      end;
      if N = 0 then return "oracle is empty"; end if;
      if Line (1) /= "ecc-homography-masked-oracle 1" then return "oracle magic or format version"; end if;
      if N < 2 or else Line (2) /= "version " & Native then return "oracle OpenCV version differs"; end if;
      if N < 3 or else Ada.Strings.Fixed.Head (Line (N), 4) /= "end " then
         return "oracle truncated (no end record)";
      end if;
      if Line (N) /= "end" & Integer'Image (Expected_Hm_Oracle_Fixtures) then
         return "oracle record count";
      end if;
      if N - 3 /= Expected_Hm_Oracle_Fixtures then return "oracle fixture record count"; end if;
      for I in 1 .. Expected_Hm_Oracle_Fixtures loop
         Hs_Split (Line (I + 2), Tok, TC);
         if TC /= 30 or else Hs_Tok (Tok, 1) /= "fixture" then
            return "malformed fixture record";
         end if;
         if Hs_Tok (Tok, 2) /= Hm_Name (I) then
            return "fixture identity or order: " & Hm_Name (I);
         end if;
         declare
            Defect : constant String :=
              Hm_Compare (Tok, Hm_Table (I), Hm_Name (I), Succeeded, Raised, Effect, Seeded);
         begin
            if Defect /= "" then return Defect; end if;
         end;
      end loop;
      return "";
   exception
      when others => return "oracle parse error";
   end Verify_Hm_Oracle;

   procedure ECC_Homography_Masked_Direct_Oracle (T : in out Fixture) is
      pragma Unreferenced (T);
      Succeeded, Raised, Effect, Seeded : Natural;
      Defect : constant String := Verify_Hm_Oracle (Hm_Oracle_Path, Succeeded, Raised, Effect, Seeded);
   begin
      Assert (Defect = "", "masked homography oracle: " & Defect);
      Assert (Succeeded + Raised = Expected_Hm_Oracle_Fixtures, "masked homography oracle accounting");
      Assert (Succeeded >= 30 and then Raised >= 10, "masked homography oracle must cover both outcomes");
      Assert (Effect >= 30, "masked homography oracle: too few fixtures where the mask changes the result");
      Assert (Seeded >= 12, "masked homography oracle: too few caller-seeded fixtures");
   end ECC_Homography_Masked_Direct_Oracle;

   procedure ECC_Homography_Masked_Oracle_Integrity (T : in out Fixture) is
      pragma Unreferenced (T);
      Original : constant String := Hm_Oracle_Path;
      Scratch  : constant String := Original & ".tampered";
      Lines    : Hom_Line_Array;
      Count    : Natural;
      A, B, C, D : Natural;

      function Line_Of (Name : String) return Positive is
      begin
         for I in 1 .. Expected_Hm_Oracle_Fixtures loop
            if Hm_Name (I) = Name then return I + 2; end if;
         end loop;
         raise Constraint_Error with "no such masked homography oracle fixture " & Name;
      end Line_Of;

      procedure Expect_Rejected (Label, Fragment : String; Edited : Hom_Line_Array; Edited_Count : Natural) is
      begin
         Hom_Write_Lines (Scratch, Edited, Edited_Count);
         declare
            Defect : constant String := Verify_Hm_Oracle (Scratch, A, B, C, D);
         begin
            Assert (Defect /= "", "tampered masked homography oracle accepted: " & Label);
            Assert (Ada.Strings.Fixed.Index (Defect, Fragment) > 0,
                    "tampered masked homography oracle (" & Label & ") rejected for the wrong reason: " & Defect);
         end;
      end Expect_Rejected;

      function Swap_Lines (L : Hom_Line_Array; I, J : Positive) return Hom_Line_Array is
         Result : Hom_Line_Array := L;
      begin
         Result (I) := L (J);
         Result (J) := L (I);
         return Result;
      end Swap_Lines;

      --  The hex token with its least significant bit flipped (a one-ulp change).
      function Flip_Last_Bit (Line_Index, Token_Index : Positive) return String is
         use type Interfaces.Unsigned_64;
         Tok : Hs_Token_Array;
         TC  : Natural;
         V   : Interfaces.Unsigned_64;
      begin
         Hs_Split (Ada.Strings.Unbounded.To_String (Lines (Line_Index)), Tok, TC);
         V := Hex_Value (Hs_Tok (Tok, Token_Index));
         return Hex8 (Interfaces.Unsigned_32 (if V mod 2 = 0 then V + 1 else V - 1));
      end Flip_Last_Bit;

      Seeded_Line, Hole_Line : Positive;
   begin
      Hom_Read_Lines (Original, Lines, Count);
      Seeded_Line := Line_Of ("occ-ltp-block-masked");
      Hole_Line := Line_Of ("trans-hole255");
      Assert (Count = Expected_Hm_Oracle_Fixtures + 3, "masked homography oracle line count");
      Assert (Verify_Hm_Oracle (Original, A, B, C, D) = "", "pristine masked homography oracle rejected");
      Assert (Verify_Hm_Oracle (Original & ".does-not-exist", A, B, C, D) = "oracle missing or unreadable",
              "missing masked homography oracle accepted");
      Expect_Rejected ("empty", "empty", Lines, 0);
      Expect_Rejected ("truncated", "truncated", Lines, Count - 1);
      Expect_Rejected ("truncated mid-record", "truncated", Lines, Count - 7);
      Expect_Rejected ("wrong magic", "format version", Hs_With_Token (Lines, 1, 1, "ecc-homography-seeded-oracle"), Count);
      Expect_Rejected ("wrong format version", "format version", Hs_With_Token (Lines, 1, 2, "2"), Count);
      Expect_Rejected ("wrong OpenCV version", "OpenCV version", Hs_With_Token (Lines, 2, 2, "0.0.0"), Count);
      Expect_Rejected ("wrong declared count", "record count", Hs_With_Token (Lines, Count, 2, "52"), Count);
      Expect_Rejected ("reordered records", "fixture identity or order", Swap_Lines (Lines, Hole_Line, Hole_Line + 1), Count);
      Expect_Rejected ("renamed fixture", "fixture identity", Hs_With_Token (Lines, Seeded_Line, 2, "occ-ltp-block-maskedX"), Count);
      Expect_Rejected ("dropped token", "malformed", Hs_With_Token (Lines, Seeded_Line, 30, "NONE extra"), Count);
      Expect_Rejected ("altered options", "options", Hs_With_Token (Lines, Seeded_Line, 3, "49"), Count);
      Expect_Rejected ("altered template checksum", "fixture checksum", Hs_With_Token (Lines, Seeded_Line, 25, "1"), Count);
      Expect_Rejected ("altered moving checksum", "fixture checksum", Hs_With_Token (Lines, Seeded_Line, 26, "1"), Count);
      Expect_Rejected ("altered mask checksum", "mask checksum", Hs_With_Token (Lines, Seeded_Line, 27, "1"), Count);
      Expect_Rejected ("altered mask count", "mask checksum", Hs_With_Token (Lines, Seeded_Line, 28, "12325"), Count);
      Expect_Rejected ("altered seed H02", "seed H02", Hs_With_Token (Lines, Seeded_Line, 7, "41100000"), Count);
      Expect_Rejected ("swapped seed H02 and H12", "seed H02", Hs_Swap_Tokens (Lines, Seeded_Line, 7, 10), Count);
      Expect_Rejected ("swapped output H02 and H20", "differs", Hs_Swap_Tokens (Lines, Seeded_Line, 17, 21), Count);
      Expect_Rejected ("transposed output H01/H10", "differs", Hs_Swap_Tokens (Lines, Seeded_Line, 16, 18), Count);
      Expect_Rejected ("altered output H11 last bit", "H11 differs",
                       Hs_With_Token (Lines, Hole_Line, 19, Flip_Last_Bit (Hole_Line, 19)), Count);
      Expect_Rejected ("altered correlation", "correlation differs", Hs_With_Token (Lines, Hole_Line, 24, "3fe0000000000000"), Count);
      Expect_Rejected ("success recorded as exception", "binding succeeded where native raised",
                       Hs_With_Token (Hs_With_Token (Lines, Hole_Line, 14, "EXC"), Hole_Line, 30, "NAN"), Count);
      Expect_Rejected ("exception recorded as success", "binding raised where native succeeded",
                       Hs_With_Token (Hs_With_Token (Lines, Line_Of ("single-pixel"), 14, "OK"), Line_Of ("single-pixel"), 30, "NONE"),
                       Count);
      Expect_Rejected ("inconsistent error class", "error class inconsistent",
                       Hs_With_Token (Lines, Hole_Line, 30, "NAN"), Count);
      Expect_Rejected ("unknown outcome token", "outcome token", Hs_With_Token (Lines, Hole_Line, 14, "MAYBE"), Count);
      Expect_Rejected ("malformed hex", "parse error", Hs_With_Token (Lines, Hole_Line, 15, "zz"), Count);
      Ada.Directories.Delete_File (Scratch);
   end ECC_Homography_Masked_Oracle_Integrity;

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
       Result.Add_Test (Caller.Create ("seeded identity and successful errors", Seeded_Identity'Access));
       Result.Add_Test (Caller.Create ("seed consumed, bounds and immutability", Seeded_Translation'Access));
       Result.Add_Test (Caller.Create ("seed counts and empty contract", Seeded_Counts'Access));
       Result.Add_Test (Caller.Create ("unsafe and nonfinite seeds", Invalid_Seeds'Access));
       Result.Add_Test (Caller.Create ("outside seeds and failed normalization", Seeded_Outside'Access));
       Result.Add_Test (Caller.Create ("seeded noncontiguous Regions", Seeded_Regions'Access));
        Result.Add_Test (Caller.Create ("FB identity", FB_Identity'Access));
        Result.Add_Test (Caller.Create ("FB small translation and ordinary equivalence", FB_Small_Translation'Access));
        Result.Add_Test (Caller.Create ("FB seeded large translation and immutability", FB_Seeded_Translation'Access));
        Result.Add_Test (Caller.Create ("FB native forward-successful inconsistency", FB_Inconsistent'Access));
        Result.Add_Test (Caller.Create ("FB above below and equal thresholds", FB_Thresholds'Access));
        Result.Add_Test (Caller.Create ("FB zero threshold", FB_Zero'Access));
        Result.Add_Test (Caller.Create ("FB forward failure", FB_Forward_Failure'Access));
        Result.Add_Test (Caller.Create ("FB backward failure", FB_Backward_Failure'Access));
        Result.Add_Test (Caller.Create ("FB interleaved compact subset mapping", FB_Compact_Mapping'Access));
        Result.Add_Test (Caller.Create ("FB Positive Last bounds", FB_Extreme_Bounds'Access));
        Result.Add_Test (Caller.Create ("FB empty arrays and seed counts", FB_Empty_Counts'Access));
        Result.Add_Test (Caller.Create ("FB finite nonnegative thresholds", FB_Invalid_Threshold'Access));
        Result.Add_Test (Caller.Create ("FB noncontiguous Regions both overloads", FB_Regions'Access));
        Result.Add_Test (Caller.Create ("FB retains image options and seed validation", FB_Validation'Access));
        Result.Add_Test (Caller.Create ("FB independent direct C++ oracle", FB_Direct_Oracle'Access));
       Result.Add_Test (Caller.Create ("quality unseeded identity and bounds", Quality_Identity'Access));
       Result.Add_Test (Caller.Create ("quality seeded 12 7 and differing bounds", Quality_Seeded'Access));
       Result.Add_Test (Caller.Create ("quality corner edge flat", Quality_Structures'Access));
       Result.Add_Test (Caller.Create ("quality threshold below eigenvalue", Quality_Threshold_Below'Access));
       Result.Add_Test (Caller.Create ("quality threshold above retains eigenvalue", Quality_Threshold_Above'Access));
       Result.Add_Test (Caller.Create ("quality previous patch unavailable", Quality_Previous_Unavailable'Access));
       Result.Add_Test (Caller.Create ("quality next search unavailable retains value", Quality_Next_Unavailable'Access));
       Result.Add_Test (Caller.Create ("quality versus photometric metrics and coordinates", Quality_Metrics'Access));
       Result.Add_Test (Caller.Create ("quality independent of destination seeds", Quality_Seed_Independence'Access));
       Result.Add_Test (Caller.Create ("quality Positive Last bounds", Quality_Extreme_Bounds'Access));
       Result.Add_Test (Caller.Create ("quality empty bounds both overloads", Quality_Empty'Access));
       Result.Add_Test (Caller.Create ("quality inherited validation both overloads", Quality_Validation'Access));
       Result.Add_Test (Caller.Create ("quality Regions and all input immutability", Quality_Regions_Immutability'Access));
       Result.Add_Test (Caller.Create ("quality independent direct C++ oracle", Quality_Direct_Oracle'Access));
       Result.Add_Test (Caller.Create ("pyramid metadata level 0 3 30 truncation", Pyramid_Metadata'Access));
       Result.Add_Test (Caller.Create ("pyramid raw equivalence translation and failed normalization", Pyramid_Equivalence'Access));
       Result.Add_Test (Caller.Create ("pyramid extreme and empty bounds", Pyramid_Bounds_Empty'Access));
       Result.Add_Test (Caller.Create ("pyramid source mutation independence", Pyramid_Mutation'Access));
       Result.Add_Test (Caller.Create ("pyramid source Region parent lifetime", Pyramid_Lifetime'Access));
       Result.Add_Test (Caller.Create ("pyramid sequential and reversed-role reuse", Pyramid_Reuse'Access));
       Result.Add_Test (Caller.Create ("pyramid compatibility empty metadata windows depth geometry", Pyramid_Compatibility'Access));
       Result.Add_Test (Caller.Create ("pyramid construction validation", Pyramid_Build_Validation'Access));
        Result.Add_Test (Caller.Create ("seeded pyramid identity translation consumption raw equivalence", Seeded_Pyramid_Equivalence'Access));
        Result.Add_Test (Caller.Create ("seeded pyramid differing extreme empty bounds", Seeded_Pyramid_Bounds'Access));
        Result.Add_Test (Caller.Create ("seeded pyramid failed predictions normalize", Seeded_Pyramid_Failures'Access));
        Result.Add_Test (Caller.Create ("seeded pyramid mutated finalized sources Regions parents", Seeded_Pyramid_Lifetime'Access));
        Result.Add_Test (Caller.Create ("seeded pyramid repeated predictions reversed roles truncation", Seeded_Pyramid_Reuse'Access));
        Result.Add_Test (Caller.Create ("seeded pyramid validation including empty points", Seeded_Pyramid_Validation'Access));
        Result.Add_Test (Caller.Create ("Pyramid_Quality_Direct_Oracle", Pyramid_Quality_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("Quality_Pyramid_Equivalence", Quality_Pyramid_Equivalence'Access));
      Result.Add_Test (Caller.Create ("Quality_Pyramid_Bounds", Quality_Pyramid_Bounds'Access));
      Result.Add_Test (Caller.Create ("Quality_Pyramid_Lifetime", Quality_Pyramid_Lifetime'Access));
      Result.Add_Test (Caller.Create ("Quality_Pyramid_Reuse", Quality_Pyramid_Reuse'Access));
      Result.Add_Test (Caller.Create ("Quality_Pyramid_Validation", Quality_Pyramid_Validation'Access));
      Result.Add_Test (Caller.Create ("pyramid quality metric semantics", Pyramid_Quality_Semantics'Access));
      Result.Add_Test (Caller.Create ("pyramid previous quality backend definedness", Pyramid_Quality_Previous_Unavailable'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Identity", Pyramid_FB_Identity'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Small_Translation", Pyramid_FB_Small_Translation'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Seeded_Translation", Pyramid_FB_Seeded_Translation'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Inconsistent", Pyramid_FB_Inconsistent'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Thresholds", Pyramid_FB_Thresholds'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Zero", Pyramid_FB_Zero'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Forward_Failure", Pyramid_FB_Forward_Failure'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Backward_Failure", Pyramid_FB_Backward_Failure'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Compact_Mapping", Pyramid_FB_Compact_Mapping'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Extreme_Bounds", Pyramid_FB_Extreme_Bounds'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Empty_Counts", Pyramid_FB_Empty_Counts'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Invalid_Threshold", Pyramid_FB_Invalid_Threshold'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Regions", Pyramid_FB_Regions'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Validation", Pyramid_FB_Validation'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Direct_Oracle", Pyramid_FB_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Equivalence", Pyramid_FB_Equivalence'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Lifetime_Reuse", Pyramid_FB_Lifetime_Reuse'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Compatibility", Pyramid_FB_Compatibility'Access));
      Result.Add_Test (Caller.Create ("Pyramid_FB_Truncation_Subsets", Pyramid_FB_Truncation_Subsets'Access));
      Result.Add_Test (Caller.Create ("Farneback_Identity", Farneback_Identity'Access));
      Result.Add_Test (Caller.Create ("Farneback_Translation_Directions", Farneback_Translation_Directions'Access));
      Result.Add_Test (Caller.Create ("Farneback_Schema_Ownership", Farneback_Schema_Ownership'Access));
      Result.Add_Test (Caller.Create ("Farneback_Regions", Farneback_Regions'Access));
      Result.Add_Test (Caller.Create ("Farneback_Validation", Farneback_Validation'Access));
      Result.Add_Test (Caller.Create ("Farneback_Direct_Oracle", Farneback_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("Farneback_Minimum_Size_Options", Farneback_Minimum_Size_Options'Access));
      Result.Add_Test (Caller.Create ("Seeded_Default_And_Options", Seeded_Default_And_Options'Access));
      Result.Add_Test (Caller.Create ("Seeded_Consumption_Refinement", Seeded_Consumption_Refinement'Access));
      Result.Add_Test (Caller.Create ("Seeded_Zero_Equivalence", Seeded_Zero_Equivalence'Access));
      Result.Add_Test (Caller.Create ("Seeded_Nonuniform_Independence", Seeded_Nonuniform_Independence'Access));
      Result.Add_Test (Caller.Create ("Seeded_Signs_And_Identical", Seeded_Signs_And_Identical'Access));
      Result.Add_Test (Caller.Create ("Seeded_Regions_Lifetime", Seeded_Regions_Lifetime'Access));
      Result.Add_Test (Caller.Create ("Seeded_Validation", Seeded_Validation'Access));
      Result.Add_Test (Caller.Create ("Seeded_Direct_Oracle", Seeded_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Default_Options", ECC_Default_Options'Access));
      Result.Add_Test (Caller.Create ("ECC_Translation_Directions", ECC_Translation_Directions'Access));
      Result.Add_Test (Caller.Create ("ECC_Options_And_Repeatability", ECC_Options_And_Repeatability'Access));
      Result.Add_Test (Caller.Create ("ECC_Epsilon_Sensitivity", ECC_Epsilon_Sensitivity'Access));
      Result.Add_Test (Caller.Create ("ECC_Photometric", ECC_Photometric'Access));
      Result.Add_Test (Caller.Create ("ECC_Regions_Lifetime", ECC_Regions_Lifetime'Access));
      Result.Add_Test (Caller.Create ("ECC_Image_Validation", ECC_Image_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Option_Validation", ECC_Option_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Direct_Oracle", ECC_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Degenerate_And_Nonconvergence", ECC_Degenerate_And_Nonconvergence'Access));
      Result.Add_Test (Caller.Create ("ECC_Aspect_Ratios", ECC_Aspect_Ratios'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Defaults", ECC_Euclidean_Defaults'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Identity_And_Translation", ECC_Euclidean_Identity_And_Translation'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Signed_Rotations", ECC_Euclidean_Signed_Rotations'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Rotation_Translation", ECC_Euclidean_Rotation_Translation'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Centered_Rotation", ECC_Euclidean_Centered_Rotation'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Inverse_Warp", ECC_Euclidean_Inverse_Warp'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Options_Sensitivity", ECC_Euclidean_Options_Sensitivity'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Regions_Lifetime", ECC_Euclidean_Regions_Lifetime'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Validation", ECC_Euclidean_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Degenerate_And_Limits", ECC_Euclidean_Degenerate_And_Limits'Access));
      Result.Add_Test (Caller.Create ("ECC_Euclidean_Direct_Oracle", ECC_Euclidean_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Zero_Equivalence", ECC_Seeded_Zero_Equivalence'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Signs_And_Translation", ECC_Seeded_Signs_And_Translation'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Consumption", ECC_Seeded_Consumption'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Refinement_And_Residual", ECC_Seeded_Refinement_And_Residual'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Input_Independence", ECC_Seeded_Input_Independence'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Centered_Rotation", ECC_Seeded_Centered_Rotation'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Regions_Lifetime", ECC_Seeded_Regions_Lifetime'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Validation", ECC_Seeded_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Image_Validation", ECC_Seeded_Image_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Nonconvergence", ECC_Seeded_Nonconvergence'Access));
      Result.Add_Test (Caller.Create ("ECC_Seeded_Direct_Oracle", ECC_Seeded_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Defaults", ECC_Affine_Defaults'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Identity_And_Translation", ECC_Affine_Identity_And_Translation'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Rotation_And_Scale", ECC_Affine_Rotation_And_Scale'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Shear_And_Combined", ECC_Affine_Shear_And_Combined'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Versus_Euclidean", ECC_Affine_Versus_Euclidean'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Inverse_Warp", ECC_Affine_Inverse_Warp'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Options_And_Repeatability", ECC_Affine_Options_And_Repeatability'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Regions_Lifetime", ECC_Affine_Regions_Lifetime'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Other_Models_Unchanged", ECC_Affine_Other_Models_Unchanged'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Validation", ECC_Affine_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Degenerate_And_Limits", ECC_Affine_Degenerate_And_Limits'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Direct_Oracle", ECC_Affine_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Oracle_Integrity", ECC_Affine_Oracle_Integrity'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Identity_Equivalence", ECC_Affine_Seeded_Identity_Equivalence'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Consumption", ECC_Affine_Seeded_Consumption'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Refinement_And_Direction", ECC_Affine_Seeded_Refinement_And_Direction'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Correlation_Ignored", ECC_Affine_Seeded_Correlation_Ignored'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Immutability_And_Regions", ECC_Affine_Seeded_Immutability_And_Regions'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Validation", ECC_Affine_Seeded_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Image_Validation", ECC_Affine_Seeded_Image_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Nonconvergence", ECC_Affine_Seeded_Nonconvergence'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Direct_Oracle", ECC_Affine_Seeded_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Affine_Seeded_Oracle_Integrity", ECC_Affine_Seeded_Oracle_Integrity'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Defaults", ECC_Homography_Defaults'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Identity_And_Translation", ECC_Homography_Identity_And_Translation'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Rotation_Scale_Shear", ECC_Homography_Rotation_Scale_Shear'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Projective_Coefficients", ECC_Homography_Projective_Coefficients'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Versus_Affine", ECC_Homography_Versus_Affine'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Inverse_Warp", ECC_Homography_Inverse_Warp'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Options_And_Repeatability", ECC_Homography_Options_And_Repeatability'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Regions_Lifetime", ECC_Homography_Regions_Lifetime'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Other_Models_Unchanged", ECC_Homography_Other_Models_Unchanged'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Validation", ECC_Homography_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Nonconvergence", ECC_Homography_Nonconvergence'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Direct_Oracle", ECC_Homography_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Oracle_Integrity", ECC_Homography_Oracle_Integrity'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Identity_Equivalence", ECC_Homography_Seeded_Identity_Equivalence'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Consumption", ECC_Homography_Seeded_Consumption'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Refinement_And_Direction", ECC_Homography_Seeded_Refinement_And_Direction'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Correlation_Ignored", ECC_Homography_Seeded_Correlation_Ignored'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Immutability_And_Regions", ECC_Homography_Seeded_Immutability_And_Regions'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Validation", ECC_Homography_Seeded_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Image_Validation", ECC_Homography_Seeded_Image_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Nonconvergence", ECC_Homography_Seeded_Nonconvergence'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Direct_Oracle", ECC_Homography_Seeded_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Seeded_Oracle_Integrity", ECC_Homography_Seeded_Oracle_Integrity'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Full_Equivalence", ECC_Homography_Masked_Full_Equivalence'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Value_Semantics", ECC_Homography_Masked_Value_Semantics'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Occlusion_Improves", ECC_Homography_Masked_Occlusion_Improves'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Seeded_Occlusion", ECC_Homography_Masked_Seeded_Occlusion'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Coordinate_Frame", ECC_Homography_Masked_Coordinate_Frame'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Numerics", ECC_Homography_Masked_Numerics'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Support", ECC_Homography_Masked_Support'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Validation", ECC_Homography_Masked_Validation'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Immutability_And_Regions", ECC_Homography_Masked_Immutability_And_Regions'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Existing_Unchanged", ECC_Homography_Masked_Existing_Unchanged'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Direct_Oracle", ECC_Homography_Masked_Direct_Oracle'Access));
      Result.Add_Test (Caller.Create ("ECC_Homography_Masked_Oracle_Integrity", ECC_Homography_Masked_Oracle_Integrity'Access));
      return Result;
   end Suite;
end Video_Tests;
