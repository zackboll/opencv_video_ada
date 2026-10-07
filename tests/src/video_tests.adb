with AUnit.Assertions;
with AUnit.Test_Caller;
with AUnit.Test_Fixtures;
with Ada.Environment_Variables;
with Ada.Text_IO;
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
       return Result;
   end Suite;
end Video_Tests;
