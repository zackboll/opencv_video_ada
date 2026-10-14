with Ada.Numerics.Long_Elementary_Functions;
with Interfaces;
with OpenCV.Core.UInt8_Access;

package body ECC_Fixtures is
   package Math renames Ada.Numerics.Long_Elementary_Functions;
   use type Interfaces.Unsigned_32;

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

   function Saturate (V : Long_Float) return OpenCV.UInt8_Value is
     (OpenCV.UInt8_Value (Integer'Max (0, Integer'Min (255, Integer (Long_Float'Rounding (V))))));

   function Blank (Rows, Columns : Positive) return OpenCV.Core.Mat is
     (OpenCV.Core.Create (Rows, Columns, (Depth => OpenCV.Core.UInt8, Channels => 1)));

   function Texture (Rows, Columns : Positive; DX : Integer := 0; DY : Integer := 0)
     return OpenCV.Core.Mat is
   begin
      return Result : OpenCV.Core.Mat := Blank (Rows, Columns) do
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               OpenCV.Core.UInt8_Access.Set
                 (Result, R, C, Saturate (Signal (Long_Float (C - DX), Long_Float (R - DY))));
            end loop;
         end loop;
      end return;
   end Texture;

   function Rigid_Texture
     (Rows, Columns : Positive; Degrees, TX, TY : Long_Float) return OpenCV.Core.Mat
   is
      Theta : constant Long_Float := Degrees * 3.14159265358979323846 / 180.0;
      C     : constant Long_Float := Math.Cos (Theta);
      S     : constant Long_Float := Math.Sin (Theta);
   begin
      return Result : OpenCV.Core.Mat := Blank (Rows, Columns) do
         for R in 0 .. Rows - 1 loop
            for Col in 0 .. Columns - 1 loop
               declare
                  U : constant Long_Float := Long_Float (Col) - TX;
                  V : constant Long_Float := Long_Float (R) - TY;
               begin
                  OpenCV.Core.UInt8_Access.Set
                    (Result, R, Col, Saturate (Signal (C * U + S * V, -S * U + C * V)));
               end;
            end loop;
         end loop;
      end return;
   end Rigid_Texture;

   function Affine_Texture
     (Rows, Columns : Positive; A, B, C, D, TX, TY : Long_Float) return OpenCV.Core.Mat
   is
      Det : constant Long_Float := A * D - B * C;
      IA  : constant Long_Float := D / Det;
      IB  : constant Long_Float := -B / Det;
      IC  : constant Long_Float := -C / Det;
      ID  : constant Long_Float := A / Det;
   begin
      return Result : OpenCV.Core.Mat := Blank (Rows, Columns) do
         for R in 0 .. Rows - 1 loop
            for Col in 0 .. Columns - 1 loop
               declare
                  U : constant Long_Float := Long_Float (Col) - TX;
                  V : constant Long_Float := Long_Float (R) - TY;
               begin
                  OpenCV.Core.UInt8_Access.Set
                    (Result, R, Col, Saturate (Signal (IA * U + IB * V, IC * U + ID * V)));
               end;
            end loop;
         end loop;
      end return;
   end Affine_Texture;

   function Homography_Texture
     (Rows, Columns : Positive; H : Homography_Matrix) return OpenCV.Core.Mat
   is
      C00 : constant Long_Float := H (4) * H (8) - H (5) * H (7);
      C01 : constant Long_Float := H (5) * H (6) - H (3) * H (8);
      C02 : constant Long_Float := H (3) * H (7) - H (4) * H (6);
      Det : constant Long_Float := H (0) * C00 + H (1) * C01 + H (2) * C02;
   begin
      if not (abs Det > 1.0E-12) then
         raise Constraint_Error with "singular homography";
      end if;
      declare
         I0 : constant Long_Float := C00 / Det;
         I1 : constant Long_Float := (H (2) * H (7) - H (1) * H (8)) / Det;
         I2 : constant Long_Float := (H (1) * H (5) - H (2) * H (4)) / Det;
         I3 : constant Long_Float := C01 / Det;
         I4 : constant Long_Float := (H (0) * H (8) - H (2) * H (6)) / Det;
         I5 : constant Long_Float := (H (2) * H (3) - H (0) * H (5)) / Det;
         I6 : constant Long_Float := C02 / Det;
         I7 : constant Long_Float := (H (1) * H (6) - H (0) * H (7)) / Det;
         I8 : constant Long_Float := (H (0) * H (4) - H (1) * H (3)) / Det;
      begin
         return Result : OpenCV.Core.Mat := Blank (Rows, Columns) do
            for R in 0 .. Rows - 1 loop
               for Col in 0 .. Columns - 1 loop
                  declare
                     U : constant Long_Float := Long_Float (Col);
                     V : constant Long_Float := Long_Float (R);
                     D : constant Long_Float := I6 * U + I7 * V + I8;
                  begin
                     if D = 0.0 then
                        raise Constraint_Error with "zero projective denominator";
                     end if;
                     OpenCV.Core.UInt8_Access.Set
                       (Result, R, Col,
                        Saturate (Signal ((I0 * U + I1 * V + I2) / D,
                                          (I3 * U + I4 * V + I5) / D)));
                  end;
               end loop;
            end loop;
         end return;
      end;
   end Homography_Texture;

   function Affine (Source : OpenCV.Core.Mat; Num, Den, Beta : Integer) return OpenCV.Core.Mat is
   begin
      return Result : OpenCV.Core.Mat := Blank (Source.Rows, Source.Columns) do
         for R in 0 .. Source.Rows - 1 loop
            for C in 0 .. Source.Columns - 1 loop
               declare
                  V : constant Integer := Integer (OpenCV.Core.UInt8_Access.Get (Source, R, C));
                  Q : constant Integer := (V * Num + Den / 2) / Den + Beta;
               begin
                  OpenCV.Core.UInt8_Access.Set
                    (Result, R, C, OpenCV.UInt8_Value (Integer'Max (0, Integer'Min (255, Q))));
               end;
            end loop;
         end loop;
      end return;
   end Affine;

   function Constant_Image (Rows, Columns : Positive; Value : Natural) return OpenCV.Core.Mat is
   begin
      return Result : OpenCV.Core.Mat := Blank (Rows, Columns) do
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               OpenCV.Core.UInt8_Access.Set (Result, R, C, OpenCV.UInt8_Value (Value));
            end loop;
         end loop;
      end return;
   end Constant_Image;

   function Noise (Rows, Columns : Positive; Seed : Natural) return OpenCV.Core.Mat is
      State : Interfaces.Unsigned_32 := Interfaces.Unsigned_32 (Seed);
   begin
      return Result : OpenCV.Core.Mat := Blank (Rows, Columns) do
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               State := State * 1664525 + 1013904223;
               OpenCV.Core.UInt8_Access.Set
                 (Result, R, C, OpenCV.UInt8_Value (Interfaces.Shift_Right (State, 24)));
            end loop;
         end loop;
      end return;
   end Noise;

   function Stripes (Rows, Columns : Positive; DX : Integer) return OpenCV.Core.Mat is
      Pi : constant Long_Float := 3.14159265358979323846;
   begin
      return Result : OpenCV.Core.Mat := Blank (Rows, Columns) do
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               OpenCV.Core.UInt8_Access.Set
                 (Result, R, C,
                  Saturate (128.0 + 60.0 * Math.Sin (2.0 * Pi * Long_Float (C - DX) / 16.0)));
            end loop;
         end loop;
      end return;
   end Stripes;

   function Checksum (Image : OpenCV.Core.Mat) return Long_Long_Integer is
      Sum : Long_Long_Integer := 0;
   begin
      for R in 0 .. Image.Rows - 1 loop
         for C in 0 .. Image.Columns - 1 loop
            Sum := Sum + Long_Long_Integer (OpenCV.Core.UInt8_Access.Get (Image, R, C))
              * (1 + (Long_Long_Integer (R) * 977 + Long_Long_Integer (C) * 131) mod 8191);
         end loop;
      end loop;
      return Sum;
   end Checksum;
end ECC_Fixtures;
