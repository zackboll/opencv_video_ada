with AUnit.Assertions;
with AUnit.Test_Caller;
with AUnit.Test_Fixtures;
with Ada.Environment_Variables;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces;
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
      return Result;
   end Suite;
end Video_Tests;
