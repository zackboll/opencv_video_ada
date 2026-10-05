with Ada.Text_IO;
with OpenCV.Core;
with OpenCV.Core.UInt8_Access;
with OpenCV.Video;

procedure LK_Synthetic is
   use Ada.Text_IO;
   use OpenCV.Video;
   use type OpenCV.Float32_Value;

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
                  SR : constant Integer := R - DY;
                  SC : constant Integer := C - DX;
                  V : OpenCV.UInt8_Value := 0;
               begin
                  if SR >= 0 and then SR < Rows and then SC >= 0 and then SC < Columns then
                     V := OpenCV.Core.UInt8_Access.Get (Source, SR, SC);
                  end if;
                  OpenCV.Core.UInt8_Access.Set (Result, R, C, V);
               end;
            end loop;
         end loop;
      end return;
   end Shift;

   Previous : constant OpenCV.Core.Mat := Texture (96, 96);
   Next : constant OpenCV.Core.Mat := Shift (Previous, 3, 2);
   Points : constant Tracking_Point_Array :=
     [1 => (X => 25.0, Y => 25.0),
      2 => (X => 45.0, Y => 32.0),
      3 => (X => 60.0, Y => 50.0),
      4 => (X => 35.0, Y => 65.0)];
   Tracks : constant Point_Track_Array := Track_PyrLK (Previous, Next, Points);
begin
   Put_Line ("synthetic shift: dx=3 dy=2");
   Put_Line ("successful tracks:" & Natural'Image (Successful_Count (Tracks)) &
             "/" & Natural'Image (Tracks'Length));
   for I in Tracks'Range loop
      if not Tracks (I).Tracked
        or else abs (Tracks (I).Next_Point.X - Points (I).X - 3.0) > 0.20
        or else abs (Tracks (I).Next_Point.Y - Points (I).Y - 2.0) > 0.20
      then
         raise Program_Error with "synthetic translation oracle differs";
      end if;
      Put_Line
        (Positive'Image (I) & ": tracked=" & Boolean'Image (Tracks (I).Tracked) &
         " from=(" & OpenCV.Float32_Value'Image (Tracks (I).Previous_Point.X) &
         "," & OpenCV.Float32_Value'Image (Tracks (I).Previous_Point.Y) & ")" &
         " to=(" & OpenCV.Float32_Value'Image (Tracks (I).Next_Point.X) &
         "," & OpenCV.Float32_Value'Image (Tracks (I).Next_Point.Y) & ")" &
         " error=" & OpenCV.Float32_Value'Image (Tracks (I).Error));
   end loop;
end LK_Synthetic;
