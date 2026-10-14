--  Task 016 implementation-checkpoint smoke (not an AUnit registration): exercises the
--  real Estimate_ECC_Homography through the actual shim and Core bridge on an identity
--  pair, a (+2,+1) translation pair and a constant (nonconverging) pair. The independent
--  oracle and AUnit coverage are separate Task 016 qualification work.
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Text_IO;
with OpenCV.Core;
with OpenCV.Core.UInt8_Access;
with OpenCV.Video;

procedure Homography_Smoke is
   use Ada.Text_IO;
   use OpenCV.Video;
   use type OpenCV.Float32_Value;
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

   procedure Show (Name : String; R : ECC_Homography_Result) is
      W : ECC_Homography_Warp renames R.Warp;
   begin
      Put_Line (Name & " rho=" & R.Correlation'Image);
      Put_Line ("  " & W.H00'Image & W.H01'Image & W.H02'Image);
      Put_Line ("  " & W.H10'Image & W.H11'Image & W.H12'Image);
      Put_Line ("  " & W.H20'Image & W.H21'Image & W.H22'Image);
   end Show;

   Template : constant OpenCV.Core.Mat := Texture (0, 0);
   Shifted  : constant OpenCV.Core.Mat := Texture (2, 1);
   Flat     : constant OpenCV.Core.Mat := Texture (0, 0, 100);
   Same     : constant ECC_Homography_Result := Estimate_ECC_Homography (Template, Template);
   Moved    : constant ECC_Homography_Result := Estimate_ECC_Homography (Template, Shifted);
   Failed   : Boolean := False;
begin
   Show ("identity", Same);
   Show ("translation(+2,+1)", Moved);

   if Same.Warp.H22 /= 1.0 or else Moved.Warp.H22 /= 1.0 then
      raise Program_Error with "H22 must be exactly 1";
   end if;
   if abs (Moved.Warp.H02 - 2.0) > 0.2 or else abs (Moved.Warp.H12 - 1.0) > 0.2
     or else abs (Moved.Warp.H00 - 1.0) > 0.05 or else abs (Moved.Warp.H20) > 0.001
   then
      raise Program_Error with "translation outside measured tolerance";
   end if;

   begin
      declare
         Unused : constant ECC_Homography_Result := Estimate_ECC_Homography (Flat, Flat);
      begin
         null;
      end;
   exception
      when OpenCV.OpenCV_Error => Failed := True;
   end;
   if not Failed then
      raise Program_Error with "constant images must raise OpenCV_Error";
   end if;
   Put_Line ("PASS: Estimate_ECC_Homography identity, translation, H22 and nonconvergence");
end Homography_Smoke;
