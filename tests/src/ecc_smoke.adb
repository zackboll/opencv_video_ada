--  Focused Task 011 implementation smoke (not an AUnit registration). Writes the
--  exact images it uses to raw files so an independent direct-OpenCV program can
--  run cv::findTransformECC on identical input and be compared.
with Ada.Command_Line;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams.Stream_IO;
with Ada.Text_IO;
with Interfaces;
with OpenCV.Core;
with OpenCV.Core.UInt8_Access;
with OpenCV.Video;

procedure ECC_Smoke is
   use Ada.Text_IO;
   use OpenCV.Video;
   use type OpenCV.Float32_Value;
   use type OpenCV.Float64_Value;
   package Math renames Ada.Numerics.Long_Elementary_Functions;

   N : constant := 128;

   function Signal (X, Y : Long_Float) return Long_Float is
      function Blob (CX, CY, A, S : Long_Float) return Long_Float is
         U : constant Long_Float := X - CX;
         V : constant Long_Float := Y - CY;
      begin
         return A * Math.Exp (-(U * U + V * V) / (2.0 * S * S));
      end Blob;
   begin
      return 127.5 + 30.0 * Math.Sin (0.21 * X + 0.13 * Y)
        + 25.0 * Math.Sin (0.11 * X - 0.27 * Y + 1.0)
        + 20.0 * Math.Cos (0.37 * X + 0.05 * Y + 2.0)
        + 15.0 * Math.Sin (0.06 * X + 0.31 * Y)
        + Blob (40.0, 50.0, 18.0, 6.0) + Blob (90.0, 35.0, -18.0, 5.0)
        + Blob (70.0, 95.0, 18.0, 7.0);
   end Signal;

   function Pixel (V : Long_Float) return OpenCV.UInt8_Value is
     (OpenCV.UInt8_Value (Integer'Max (0, Integer'Min (255, Integer (Long_Float'Rounding (V))))));

   function Texture (DX, DY : Integer; Constant_Value : Integer := -1) return OpenCV.Core.Mat is
   begin
      return Result : OpenCV.Core.Mat := OpenCV.Core.Create
        (N, N, (Depth => OpenCV.Core.UInt8, Channels => 1))
      do
         for R in 0 .. N - 1 loop
            for C in 0 .. N - 1 loop
               OpenCV.Core.UInt8_Access.Set
                 (Result, R, C,
                  Pixel (if Constant_Value >= 0 then Long_Float (Constant_Value)
                         else Signal (Long_Float (C - DX), Long_Float (R - DY))));
            end loop;
         end loop;
      end return;
   end Texture;

   procedure Dump (Image : OpenCV.Core.Mat; Name : String) is
      File : Ada.Streams.Stream_IO.File_Type;
   begin
      Ada.Streams.Stream_IO.Create (File, Ada.Streams.Stream_IO.Out_File, Name);
      for R in 0 .. N - 1 loop
         for C in 0 .. N - 1 loop
            Interfaces.Unsigned_8'Write
              (Ada.Streams.Stream_IO.Stream (File), Interfaces.Unsigned_8 (OpenCV.Core.UInt8_Access.Get (Image, R, C)));
         end loop;
      end loop;
      Ada.Streams.Stream_IO.Close (File);
   end Dump;

   Directory : constant String :=
     (if Ada.Command_Line.Argument_Count >= 1 then Ada.Command_Line.Argument (1) else ".");
   Failures  : Natural := 0;

   procedure Expect (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         Failures := Failures + 1;
         Put_Line ("FAIL: " & Message);
      end if;
   end Expect;

   procedure Run_Case (Name : String; DX, DY : Integer) is
      Template : constant OpenCV.Core.Mat := Texture (0, 0);
      Moving   : constant OpenCV.Core.Mat := Texture (DX, DY);
      Result   : constant ECC_Translation_Result := Estimate_ECC_Translation (Template, Moving);
      Reversed : constant ECC_Translation_Result := Estimate_ECC_Translation (Moving, Template);
   begin
      Dump (Template, Directory & "/" & Name & ".template.raw");
      Dump (Moving, Directory & "/" & Name & ".moving.raw");
      Put_Line ("ECC " & Name & " " & Long_Float'Image (Long_Float (Result.Displacement.X)) & " "
                & Long_Float'Image (Long_Float (Result.Displacement.Y)) & " "
                & Long_Float'Image (Long_Float (Result.Correlation)));
      Expect (abs (Result.Displacement.X - OpenCV.Float32_Value (DX)) < 0.05
              and then abs (Result.Displacement.Y - OpenCV.Float32_Value (DY)) < 0.05,
              Name & ": displacement not within 0.05 of the applied shift");
      Expect (abs (Reversed.Displacement.X + OpenCV.Float32_Value (DX)) < 0.05
              and then abs (Reversed.Displacement.Y + OpenCV.Float32_Value (DY)) < 0.05,
              Name & ": reversed roles not negated");
      Expect (Result.Correlation > 0.99 and then Result.Correlation <= 1.0001,
              Name & ": correlation outside expected range");
   end Run_Case;

   procedure Expect_Error
     (Name : String; Template, Moving : OpenCV.Core.Mat;
      Options : ECC_Translation_Options := (others => <>))
   is
      Unused : ECC_Translation_Result;
   begin
      Unused := Estimate_ECC_Translation (Template, Moving, Options);
      Expect (False, Name & ": expected OpenCV_Error");
   exception
      when OpenCV.OpenCV_Error => Put_Line ("ok error: " & Name);
   end Expect_Error;

   Base   : constant OpenCV.Core.Mat := Texture (0, 0);
   Moving : constant OpenCV.Core.Mat := Texture (2, 1);
begin
   Run_Case ("identity", 0, 0);
   Run_Case ("positive", 2, 1);
   Run_Case ("negative", -2, -1);
   Run_Case ("horizontal", 3, 0);
   Run_Case ("vertical", 0, 3);

   declare
      Defaults : constant ECC_Translation_Options := (others => <>);
   begin
      Expect (Defaults.Maximum_Iterations = 50 and then Defaults.Epsilon = 0.0001, "default options");
   end;
   Expect_Error ("constant images", Texture (0, 0, 128), Texture (0, 0, 128));
   Expect_Error ("translation outside basin", Base, Texture (40, 30));
   Expect_Error ("epsilon zero", Base, Moving, (Maximum_Iterations => 50, Epsilon => 0.0));
   Expect_Error ("epsilon too large", Base, Moving, (Maximum_Iterations => 50, Epsilon => 0.5));
   Expect_Error ("iterations too large", Base, Moving, (Maximum_Iterations => 101, Epsilon => 0.0001));
   Expect_Error ("below minimum size", Base.Region ((0, 0, 31, 31)), Moving.Region ((0, 0, 31, 31)));
   declare
      Wrong_Depth : constant OpenCV.Core.Mat := OpenCV.Core.Create
        (N, N, (Depth => OpenCV.Core.Float32, Channels => 1));
   begin
      Expect_Error ("float32 image", Wrong_Depth, Wrong_Depth);
   end;

   --  Non-contiguous Regions of larger parents.
   declare
      Parent_A : OpenCV.Core.Mat := OpenCV.Core.Create
        (160, 160, (Depth => OpenCV.Core.UInt8, Channels => 1));
      Parent_B : OpenCV.Core.Mat := OpenCV.Core.Create
        (160, 160, (Depth => OpenCV.Core.UInt8, Channels => 1));
   begin
      for R in 0 .. 159 loop
         for C in 0 .. 159 loop
            OpenCV.Core.UInt8_Access.Set (Parent_A, R, C, Pixel (Signal (Long_Float (C), Long_Float (R))));
            OpenCV.Core.UInt8_Access.Set
              (Parent_B, R, C, Pixel (Signal (Long_Float (C - 2), Long_Float (R - 1))));
         end loop;
      end loop;
      declare
         Result : constant ECC_Translation_Result := Estimate_ECC_Translation
           (Parent_A.Region ((16, 16, N, N)), Parent_B.Region ((16, 16, N, N)));
      begin
         Expect (abs (Result.Displacement.X - 2.0) < 0.05 and then abs (Result.Displacement.Y - 1.0) < 0.05,
                 "Region displacement");
         Put_Line ("ECC region " & Long_Float'Image (Long_Float (Result.Displacement.X)) & " "
                   & Long_Float'Image (Long_Float (Result.Displacement.Y)));
      end;
   end;

   if Failures = 0 then
      Put_Line ("PASS: ECC smoke");
   else
      Put_Line ("FAILURES:" & Natural'Image (Failures));
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   end if;
end ECC_Smoke;
