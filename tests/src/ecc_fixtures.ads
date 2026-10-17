with OpenCV.Core;

--  Deterministic ECC fixtures that reproduce tests/cpp/ecc_fixture.hpp pixel for
--  pixel. The oracle test compares checksums to prove the images are identical.
package ECC_Fixtures is
   --  Content moved by (DX, DY): result(x, y) = signal(x - DX, y - DY).
   function Texture (Rows, Columns : Positive; DX : Integer := 0; DY : Integer := 0)
     return OpenCV.Core.Mat;
   --  Rigid template-to-moving warp (Task 012): moving = R(theta) * template + (TX, TY),
   --  theta = Degrees * Pi / 180, evaluated analytically as signal (W**-1 p) with the
   --  same expression order as ecc_fixture::rigid_texture in C++.
   function Rigid_Texture
     (Rows, Columns : Positive; Degrees, TX, TY : Long_Float) return OpenCV.Core.Mat;
   --  Task 014 affine template-to-moving warp: moving = A * template + (TX, TY) with
   --  A = [[A, B], [C, D]], evaluated analytically as signal (A**-1 (p - t)) with the same
   --  explicit 2 x 2 inverse and expression order as ecc_fixture::affine_texture in C++.
   --  A must be invertible.
   function Affine_Texture
     (Rows, Columns : Positive; A, B, C, D, TX, TY : Long_Float) return OpenCV.Core.Mat;
   --  Task 016 homography template-to-moving warp: row-major H (H00 H01 H02 H10 H11 H12
   --  H20 H21 H22), moving(p) = signal (H**-1 p) with homogeneous division, evaluated
   --  analytically with the same adjugate inverse and expression order as
   --  ecc_fixture::homography_texture in C++. Raises Constraint_Error for a singular H or a
   --  zero projective denominator at any pixel (no pixel is invented).
   type Homography_Matrix is array (0 .. 8) of Long_Float;
   function Homography_Texture
     (Rows, Columns : Positive; H : Homography_Matrix) return OpenCV.Core.Mat;
   --  sat((v * Num + Den / 2) / Den + Beta), exact integer arithmetic.
   function Affine (Source : OpenCV.Core.Mat; Num, Den, Beta : Integer) return OpenCV.Core.Mat;
   function Constant_Image (Rows, Columns : Positive; Value : Natural) return OpenCV.Core.Mat;
   function Noise (Rows, Columns : Positive; Seed : Natural) return OpenCV.Core.Mat;
   function Stripes (Rows, Columns : Positive; DX : Integer) return OpenCV.Core.Mat;
   --  Position-weighted sum used to compare Ada and C++ fixtures and to detect mutation
   --  of a source or parent.
   function Checksum (Image : OpenCV.Core.Mat) return Long_Long_Integer;

   --  Task 018 moving-image masks and occlusions, mirroring build_mask / occlude in
   --  tests/cpp/ecc_homography_masked_oracle.cpp (Kind letters and parameters identical).
   --  Masks are 128 x 128 UInt8 C1 in MOVING pixel coordinates (X = column, Y = row).
   type Mask_Spec is record
      Kind          : Character := 'F';
      A, B, C, D    : Integer := 0;
      V             : Natural := 255;
   end record;
   function Mask (Spec : Mask_Spec) return OpenCV.Core.Mat;
   --  Sets Rows Y .. Y + H - 1, columns X .. X + W - 1 of Image to Value (clipped).
   procedure Fill (Image : in out OpenCV.Core.Mat; X, Y, W, H : Integer; Value : Natural);
   --  Replaces moving rectangle (X, Y, W, H) with 0 = constant 250, 1 = Noise (H, W, 1234),
   --  2 = Texture (128, 128, -14, 9) content, 3 = Stripes (H, W, 0).
   function Occlude (Moving : OpenCV.Core.Mat; X, Y, W, H, Kind : Natural) return OpenCV.Core.Mat;
   --  Number of nonzero pixels.
   function Nonzero (Image : OpenCV.Core.Mat) return Natural;
end ECC_Fixtures;
