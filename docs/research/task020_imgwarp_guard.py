#!/usr/bin/env python3
"""Task 020 RESEARCH-ONLY patch for OpenCV 4.x imgwarp.cpp (never vendored or shipped).

Usage (on a scratch copy of an OpenCV 4.1.0 or 4.10.0 source tree, then rebuild OpenCV):
    python3 task020_imgwarp_guard.py <opencv-src>/modules/imgproc/src/imgwarp.cpp
The repository never runs this script; it documents the mitigation experiment in
docs/tasks/020-existing-affine-safety-audit.md (Option A).

Adds a per-row fixed-point range guard to the legacy WarpAffineInvoker:
  * fast path (unchanged, bit-identical): every cvRound operand and every X0+adelta /
    Y0+bdelta sum of the row block is proven within +-2^30 in double before any int math;
  * guarded path: per-pixel double coordinate, NaN-safe clamp of the SUM to +-2^30, then the
    same shift/saturate/alpha post-processing as upstream; no SIMD, no int32 overflow.
Also clamps the adelta/bdelta table conversion so cvRound never receives an out-of-range
value (x86 _mm_cvtsd_si32 otherwise returns INT_MIN for +-overflow). Clamping changes only
entries the guard already routes away from the fast path.
"""
import sys
from pathlib import Path

path = Path(sys.argv[1])
s = path.read_text()

row_old = "                    int X0 = saturate_cast<int>((M[1]*(y + y1) + M[2])*AB_SCALE) + round_delta;\n"
assert s.count(row_old) == 1, "row X0 anchor"
guard = r"""                    // [Task 020 research patch] fixed-point range guard (see apply_patch.py).
                    {
                        const double t020_lim = (double)(1 << 30);
                        const double t020_yy = (double)(y + y1);
                        const double t020_xa = (M[1]*t020_yy + M[2])*AB_SCALE;
                        const double t020_ya = (M[4]*t020_yy + M[5])*AB_SCALE;
                        const double t020_xl = M[0]*x*AB_SCALE, t020_xr = M[0]*(x + bw - 1)*AB_SCALE;
                        const double t020_yl = M[3]*x*AB_SCALE, t020_yr = M[3]*(x + bw - 1)*AB_SCALE;
                        const bool t020_safe =
                            std::fabs(t020_xa) <= t020_lim && std::fabs(t020_ya) <= t020_lim &&
                            std::fabs(t020_xl) <= t020_lim && std::fabs(t020_xr) <= t020_lim &&
                            std::fabs(t020_yl) <= t020_lim && std::fabs(t020_yr) <= t020_lim &&
                            std::fabs(t020_xa + t020_xl) <= t020_lim && std::fabs(t020_xa + t020_xr) <= t020_lim &&
                            std::fabs(t020_ya + t020_yl) <= t020_lim && std::fabs(t020_ya + t020_yr) <= t020_lim;
                        if( !t020_safe )
                        {
                            short* t020_alpha = A + y1*bw;
                            for( int t020_x1 = 0; t020_x1 < bw; t020_x1++ )
                            {
                                double xd = t020_xa + M[0]*(x + t020_x1)*AB_SCALE;
                                double yd = t020_ya + M[3]*(x + t020_x1)*AB_SCALE;
                                xd = (xd != xd) ? -t020_lim : (xd > t020_lim ? t020_lim : (xd < -t020_lim ? -t020_lim : xd));
                                yd = (yd != yd) ? -t020_lim : (yd > t020_lim ? t020_lim : (yd < -t020_lim ? -t020_lim : yd));
                                const int X = cvRound(xd) + round_delta;
                                const int Y = cvRound(yd) + round_delta;
                                if( interpolation == INTER_NEAREST )
                                {
                                    xy[t020_x1*2] = saturate_cast<short>(X >> AB_BITS);
                                    xy[t020_x1*2+1] = saturate_cast<short>(Y >> AB_BITS);
                                }
                                else
                                {
                                    const int XI = X >> (AB_BITS - INTER_BITS);
                                    const int YI = Y >> (AB_BITS - INTER_BITS);
                                    xy[t020_x1*2] = saturate_cast<short>(XI >> INTER_BITS);
                                    xy[t020_x1*2+1] = saturate_cast<short>(YI >> INTER_BITS);
                                    t020_alpha[t020_x1] = (short)((YI & (INTER_TAB_SIZE-1))*INTER_TAB_SIZE +
                                                                  (XI & (INTER_TAB_SIZE-1)));
                                }
                            }
                            continue;
                        }
                    }
"""
s = s.replace(row_old, guard + row_old)

table_old = ("        adelta[x] = saturate_cast<int>(M[0]*x*AB_SCALE);\n"
             "        bdelta[x] = saturate_cast<int>(M[3]*x*AB_SCALE);\n")
assert s.count(table_old) == 1, "adelta table anchor"
table_new = r"""        // [Task 020 research patch] keep cvRound operands in range.
        const double t020_lim = (double)(1 << 30);
        double t020_a = M[0]*x*AB_SCALE, t020_b = M[3]*x*AB_SCALE;
        t020_a = (t020_a != t020_a) ? -t020_lim : (t020_a > t020_lim ? t020_lim : (t020_a < -t020_lim ? -t020_lim : t020_a));
        t020_b = (t020_b != t020_b) ? -t020_lim : (t020_b > t020_lim ? t020_lim : (t020_b < -t020_lim ? -t020_lim : t020_b));
        adelta[x] = cvRound(t020_a);
        bdelta[x] = cvRound(t020_b);
"""
s = s.replace(table_old, table_new)
path.write_text(s)
print("patched", path)
