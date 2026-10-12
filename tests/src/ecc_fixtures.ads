with OpenCV.Core;

--  Deterministic ECC fixtures that reproduce tests/cpp/ecc_fixture.hpp pixel for
--  pixel. The oracle test compares checksums to prove the images are identical.
package ECC_Fixtures is
   --  Content moved by (DX, DY): result(x, y) = signal(x - DX, y - DY).
   function Texture (Rows, Columns : Positive; DX : Integer := 0; DY : Integer := 0)
     return OpenCV.Core.Mat;
   --  sat((v * Num + Den / 2) / Den + Beta), exact integer arithmetic.
   function Affine (Source : OpenCV.Core.Mat; Num, Den, Beta : Integer) return OpenCV.Core.Mat;
   function Constant_Image (Rows, Columns : Positive; Value : Natural) return OpenCV.Core.Mat;
   function Noise (Rows, Columns : Positive; Seed : Natural) return OpenCV.Core.Mat;
   function Stripes (Rows, Columns : Positive; DX : Integer) return OpenCV.Core.Mat;
   --  Position-weighted sum used to compare Ada and C++ fixtures and to detect mutation
   --  of a source or parent.
   function Checksum (Image : OpenCV.Core.Mat) return Long_Long_Integer;
end ECC_Fixtures;
