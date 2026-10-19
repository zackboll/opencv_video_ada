// Task 022 opt-in research reproducer for the arithmetic and pointer safety of the OpenCV
// 5.0.0 FLOATING-POINT warpAffine kernels that the existing ECC operations drive:
//
//     cv::opt_AVX2::warpAffineLinearInvoker_32FC1   (CV_32FC1, INTER_LINEAR)
//     cv::opt_AVX2::warpAffineNearestInvoker_8UC1   (CV_8UC1,  INTER_NEAREST)
//
// and their cv::cpu_baseline twins. hal::warpAffine dispatches ECC's image/gradient
// resamples (CV_32FC1 INTER_LINEAR, imgwarp.cpp:2329) and mask resample (CV_8UC1
// INTER_NEAREST, imgwarp.cpp:2251) to them, with WARP_INVERSE_MAP and BORDER_CONSTANT
// (ecc.cpp:523-526). Tasks 020/021 audited the LEGACY 4.x fixed-point path; their
// fixed-point matrices are NOT re-imported here as evidence for the float kernels.
//
// THIS IS NOT A SANITIZER-CLEAN TEST. Against an OpenCV 5.0.0 build whose OWN libraries are
// instrumented with -fsanitize=address,undefined,float-cast-overflow, the deliberately
// extreme Level A cases are EXPECTED to print a pointer-index-overflow runtime error from
// inside warp_kernels.simd.hpp. That report is the evidence, not a failure. Never run this
// in routine hosted CI; build it only through scripts/research_ecc5_float_warp.sh
// (docs/tasks/022-opencv5-float-warp-safety-audit.md).
//
// One case per process (UBSan reports a given source location only once per process).
// The case list is in main() and in the runner script.
//
// R1 (2026-10-19): the scalar tail forms src + srcstep*iy + ix BEFORE its bounds test for
// EVERY pixel it handles, so an ordinary int32 coordinate one pixel past the image edge
// already forms an out-of-allocation pointer (C++ UB) that no sanitizer reports (no load
// follows; the address does not wrap). pointer_range() classifies each scalar-tail pointer
// against the ASan-reported source allocation in defined arithmetic, and the r1-* cases and
// the runner's observe: mode cover it. See section R1 of the task note.
//
// The probe's own coordinate model uses only defined float arithmetic and isfinite and
// ordered comparisons. It performs NO float-to-int C++ cast, llround or pointer arithmetic
// to predict an unsafe native operation, so any UBSan report originates inside OpenCV. The
// only integer conversions are in conversion-check, through the DEFINED x86 intrinsics
// _mm_cvtss_si32 / _mm_cvttss_si32 (they return the integer-indefinite value 0x80000000
// for out-of-range or nonfinite input; they are not C++ casts).

#include "opencv_video_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"
#include "ecc_fixture.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#if defined(__x86_64__) || defined(_M_X64)
#include <emmintrin.h>
#include <xmmintrin.h>
#endif

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using ecc_fixture::texture;
using ecc_fixture::rigid_texture;
using ecc_fixture::affine_texture;
using ecc_fixture::noise;
using ecc_fixture::constant;
using ecc_fixture::stripes;
using ecc_fixture::low_variance;

using Handle = std::unique_ptr<opencv_core_mat_handle, decltype(&opencv_core_mat_destroy)>;

// Real Core-owned Mat handle (no fake opaque pointers), filled through the Core bridge.
Handle make_handle(const cv::Mat &src) {
    opencv_core_mat_handle *raw = nullptr;
    if (opencv_core_mat_create_2d(src.rows, src.cols, CV_8U, 1, &raw) != OPENCV_CORE_OK)
        throw std::runtime_error("Core matrix factory failed");
    Handle handle(raw, opencv_core_mat_destroy);
    cv::Mat *out = nullptr;
    if (opencv_core_module_output_mat(raw, &out) != OPENCV_CORE_OK || out == nullptr)
        throw std::runtime_error("Core output resolver failed");
    src.copyTo(*out);
    return handle;
}

// R1 (2026-10-19): ground-truth allocation bounds from the AddressSanitizer runtime. These
// report the actual malloc'd chunk (OpenCV's fastMalloc buffer, including its alignment
// header), which is the C++ object for pointer-arithmetic validity -- distinct from the Mat's
// logical rows*cols extent. They READ metadata only; they form and dereference nothing.
extern "C" const void *__sanitizer_get_allocated_begin(const void *p);
extern "C" size_t __sanitizer_get_allocated_size(const void *p);

uint32_t float_bits(float v) {
    uint32_t u;
    std::memcpy(&u, &v, sizeof u);
    return u;
}


// ---------------------------------------------------------------------------------------
// Defined Float32 coordinate model (no unsafe conversion is performed here).
//
// Per destination pixel the 5.0 scalar tail computes
//     float sx = x*M[0] + y*M[1] + M[2];   float sy = x*M[3] + y*M[4] + M[5];
// (warp_kernels.simd.hpp:3761-3762 linear, :437-438 nearest). The vector body computes the
// same affine form with v_fma on a float row base y*M[1]+M[2] (warp_kernels.simd.hpp:15-16,
// 61-64). Conversion is cvFloor(float) (linear) / cvRound(float) (nearest) in the scalar
// tail (warp_common.scalar.hpp:131-134), and v_floor / v_round in the vector body
// (warp_kernels.simd.hpp:40-49). On x86 these lower to cvttss2si / cvtss2si and AVX2
// vcvt(t)ps2dq, which return the integer-indefinite INT_MIN for any v outside
// [-2^31, 2^31) and for NaN/Inf. (R1 correction: the earlier wording "|v| >= 2^31" was wrong
// at exactly -2^31, which converts VALIDLY to INT_MIN -- the same integer as the sentinel,
// but not an invalid conversion.) float32 spacing at 2^31 is 128 and both +-2^31 are
// representable, so the in-range test is exactly -2^31 <= v < 2^31 for floor and
// round-to-nearest-even alike. Conversion validity is NOT pointer validity: see the R1
// pointer_range() model below, which shows in-range coordinates forming invalid pointers.
enum class Conv { IN_RANGE = 0, OUT_OF_RANGE_FINITE = 1, NONFINITE = 2 };

const char *conv_name(Conv c) {
    switch (c) {
        case Conv::IN_RANGE: return "IN_RANGE";
        case Conv::OUT_OF_RANGE_FINITE: return "OUT_OF_RANGE_FINITE";
        default: return "NONFINITE";
    }
}

constexpr float kTwoPow31 = 2147483648.0f;   // exactly representable in float32

// isfinite first: an ordered comparison with NaN would raise FE_INVALID.
Conv classify_coord(float c) {
    if (!std::isfinite(c)) return Conv::NONFINITE;
    return (c >= kTwoPow31 || c < -kTwoPow31) ? Conv::OUT_OF_RANGE_FINITE : Conv::IN_RANGE;
}

// The mapped coordinate is affine in (x, y), so over the destination grid its extremes lie
// at the four corners. Float evaluation at the corners is defined for every input.
struct CoordReach {
    Conv worst = Conv::IN_RANGE;
    char axis = '-';
    int cx = 0, cy = 0;
    float value = 0.0f;
    float max_abs_finite = 0.0f;   // largest finite |coordinate| (NaN/Inf excluded)
};

void consider(CoordReach &r, float v, char axis, int x, int y) {
    const Conv c = classify_coord(v);
    if (std::isfinite(v)) r.max_abs_finite = std::max(r.max_abs_finite, std::fabs(v));
    if (int(c) > int(r.worst)) {
        r.worst = c; r.axis = axis; r.cx = x; r.cy = y; r.value = v;
    }
}

CoordReach coordinate_reach(const float M[6], int rows, int cols) {
    CoordReach r;
    const int xs[2] = {0, cols - 1}, ys[2] = {0, rows - 1};
    for (int y : ys)
        for (int x : xs) {
            const float fx = float(x), fy = float(y);
            consider(r, fx * M[0] + fy * M[1] + M[2], 'X', x, y);
            consider(r, fx * M[3] + fy * M[4] + M[5], 'Y', x, y);
        }
    return r;
}

void print_reach(const char *label, const CoordReach &r) {
    std::printf("  %s COORD_REACH worst=%s axis=%c at (x=%d,y=%d) value=%.9g bits=0x%08x "
                "max|finite_coord|=%.9g\n", label, conv_name(r.worst), r.axis, r.cx, r.cy,
                double(r.value), unsigned(float_bits(r.value)), double(r.max_abs_finite));
    if (r.worst != Conv::IN_RANGE)
        std::printf("    FIRST_UNSAFE=CONVERSION cvFloor/cvRound(float) of a %s coordinate -> "
                    "INT_MIN sentinel; next: pointer src + srcstep*iy + ix formed BEFORE the "
                    "bounds test (warp_common.scalar.hpp:12 vs :139)\n",
                    r.worst == Conv::NONFINITE ? "nonfinite" : "finite int32-out-of-range");
}

void print_matrix(const char *prefix, const float w[6]) {
    std::printf("%s[%.17g %.17g %.17g %.17g %.17g %.17g]\n", prefix, double(w[0]), double(w[1]),
                double(w[2]), double(w[3]), double(w[4]), double(w[5]));
}

void print_matrix_bits(const float w[6]) {
    std::printf("  stored bits=[0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x]\n",
                unsigned(float_bits(w[0])), unsigned(float_bits(w[1])), unsigned(float_bits(w[2])),
                unsigned(float_bits(w[3])), unsigned(float_bits(w[4])), unsigned(float_bits(w[5])));
}

// ---------------------------------------------------------------------------------------
// R1 (2026-10-19): defined source-pointer-range analysis.
//
// The 5.0 scalar tail declares, UNCONDITIONALLY and BEFORE its bounds test,
//     const dtype_ptr *srcptr = src + srcstep*iy + ix;      (warp_common.scalar.hpp:12)
// with ix/iy from cvFloor (linear) or cvRound (nearest) (:131-134); the test
//     (unsigned)ix < (unsigned)(srccols-1) & (unsigned)iy < (unsigned)(srcrows-1)   (:139)
// runs only afterwards. Forming a pointer outside the array object is undefined in C++
// ([expr.add]) whether or not it is dereferenced, and a later test cannot repair it. The
// vector body forms no such pointer: it keeps lane offsets as v_int32 integers and only
// forms src + addr[i] when every lane is inside (warp_common.vector.hpp:26-32, 176-179).
//
// Nothing here casts float to int or forms a native-shaped pointer. The index is predicted
// in int64 (floor / round-half-even for in-range coordinates, the INT_MIN sentinel for the
// rest, as the x86 intrinsics return), and its byte offset is compared with the ACTUAL
// backing chunk reported by the ASan runtime -- not with rows*cols. On glibc/Linux OpenCV's
// fastMalloc is malloc(size + 8 + 64) plus alignment, so the chunk starts before and ends
// after the logical image; for a ROI the chunk is the parent's.
enum class Ptr {
    WITHIN_ALLOCATION = 0,          // inside the chunk and inside the logical image
    ONE_PAST = 1,                   // exactly the chunk's one-past position (valid to form)
    WITHIN_PARENT_OUTSIDE_ROI = 2,  // inside the chunk, outside the logical image
    OUTSIDE_ALLOCATION = 3,         // before the chunk or beyond its one-past: UB to form
    UNDETERMINED = 4                // chunk not resolvable (non-ASan build)
};

const char *ptr_name(Ptr p) {
    switch (p) {
        case Ptr::WITHIN_ALLOCATION: return "PTR_WITHIN_ALLOCATION";
        case Ptr::ONE_PAST: return "PTR_ONE_PAST";
        case Ptr::WITHIN_PARENT_OUTSIDE_ROI: return "PTR_WITHIN_PARENT_OUTSIDE_ROI";
        case Ptr::OUTSIDE_ALLOCATION: return "PTR_OUTSIDE_ALLOCATION";
        default: return "UNDETERMINED";
    }
}

// Kernel index for one axis in defined arithmetic. std::floor / std::nearbyint (default
// FE_TONEAREST, checked by conversion-check) of an IN_RANGE float is exactly representable
// in int64, so the conversion below is defined.
int64_t mapped_index(float coord, bool linear) {
    if (classify_coord(coord) != Conv::IN_RANGE) return INT32_MIN;
    return int64_t(linear ? std::floor(coord) : std::nearbyint(coord));
}

// Worst classification over one set of destination pixels. `robust` counts out-of-allocation
// pointers that are outside for EVERY placement fastMalloc could choose (see pointer_range).
struct PtrScope {
    Ptr worst = Ptr::WITHIN_ALLOCATION;
    int64_t off = 0, ix = 0, iy = 0;
    float sx = 0, sy = 0;
    int x = -1, y = -1;
    long outside = 0, robust = 0, pixels = 0;
    bool worst_robust = false;
    void add(Ptr p, bool rob, int64_t o, int64_t ix_, int64_t iy_, float sx_, float sy_, int x_,
             int y_) {
        ++pixels;
        if (p == Ptr::OUTSIDE_ALLOCATION) ++outside;
        if (rob) ++robust;
        // Prefer a worse class; within OUTSIDE_ALLOCATION prefer an alignment-independent one.
        if (x < 0 || int(p) > int(worst) ||
            (p == worst && p == Ptr::OUTSIDE_ALLOCATION && rob && !worst_robust)) {
            worst = p; worst_robust = rob; off = o; ix = ix_; iy = iy_; sx = sx_; sy = sy_;
            x = x_; y = y_;
        }
    }
};

// First destination column the scalar tail handles for unroll factor uf (the vector loop
// runs `for (; x <= dstcols - uf; x += uf)`). uf = 16 for AVX2 and 8 for the SSE2 baseline.
int scalar_tail_first_x(int dst_cols, int uf) {
    return dst_cols >= uf ? dst_cols - dst_cols % uf : 0;
}

void print_scope(const char *label, const char *scope, const PtrScope &s) {
    if (s.pixels == 0) {
        std::printf("  PTR_RANGE %s scope=%s pixels=0 (no scalar-tail pixel at this width)\n",
                    label, scope);
        return;
    }
    std::printf("  PTR_RANGE %s scope=%s pixels=%ld outside=%ld outside_any_alignment=%ld "
                "worst=%s%s dst=(x=%d,y=%d) mapped=(%.9g,%.9g) mapped_ix=%lld mapped_iy=%lld "
                "mathematical_element_offset=%lld\n",
                label, scope, s.pixels, s.outside, s.robust, ptr_name(s.worst),
                s.worst == Ptr::OUTSIDE_ALLOCATION ? (s.worst_robust ? "(ANY_ALIGNMENT)"
                                                                     : "(THIS_LAYOUT_ONLY)")
                                                   : "",
                s.x, s.y, double(s.sx), double(s.sy), (long long)s.ix, (long long)s.iy,
                (long long)s.off);
}

// Classify the scalar-tail srcptr of every destination pixel of a warp whose source is the
// exact Mat the kernel receives and whose output is dst_rows x dst_cols (ECC: equal sizes).
// The coordinates use the kernel's float expression x*M0 + y*M1 + M2. The probe is built
// without -mfma, so no contraction occurs here; the -O2 -mfma AVX2 kernel may contract to an
// FMA, which can only move a coordinate that lies within one ulp of an integer.
// Returns the worst class within the AVX2 scalar tail.
// verbose=false prints only the AVX2 and baseline scalar-tail verdict lines (Level B traces).
Ptr pointer_range(const char *label, const cv::Mat &src, const float M[6], bool linear,
                  int dst_rows, int dst_cols, bool verbose = true) {
    const int64_t elem = int64_t(src.elemSize1());
    const int64_t step_el = int64_t(src.step[0]) / elem;   // kernel srcstep, in elements
    const unsigned char *base = src.data;
    const void *chunk = __sanitizer_get_allocated_begin(base);
    const int64_t chunk_bytes = chunk ? int64_t(__sanitizer_get_allocated_size(chunk)) : 0;
    // uintptr_t differences: no pointer subtraction across (possibly) distinct objects.
    const int64_t lo = chunk ? int64_t(reinterpret_cast<uintptr_t>(chunk)) -
                                   int64_t(reinterpret_cast<uintptr_t>(base))
                             : 0;
    const int64_t hi = lo + chunk_bytes;   // one-past the chunk, bytes relative to src
    // Placement-independent bound. fastMalloc (alloc.cpp:160-164, glibc/Linux without
    // memalign) returns alignPtr(malloc(n + 8 + 64) + 8, 64): the user data starts 8..71 bytes
    // into the chunk and the chunk ends 1..64 bytes after the Mat's datalimit. A pointer below
    // datastart-71 or above datalimit+64 is therefore outside the chunk for EVERY placement.
    const int64_t ds = int64_t(reinterpret_cast<uintptr_t>(src.datastart)) -
                       int64_t(reinterpret_cast<uintptr_t>(base));
    const int64_t dl = int64_t(reinterpret_cast<uintptr_t>(src.datalimit)) -
                       int64_t(reinterpret_cast<uintptr_t>(base));
    const int64_t robust_lo = ds - 71, robust_hi = dl + 64;
    if (verbose) std::printf("  PTR_SOURCE %s kernel=%s source_rows=%d source_cols=%d "
                "source_step_elements=%lld elem_bytes=%lld submatrix=%d continuous=%d "
                "logical_element_count=%lld\n",
                label, linear ? "warpAffineLinearInvoker_32FC1" : "warpAffineNearestInvoker_8UC1",
                src.rows, src.cols, (long long)step_el, (long long)elem, int(src.isSubmatrix()),
                int(src.isContinuous()), (long long)(int64_t(src.rows) * src.cols));
    if (verbose) std::printf("  PTR_ALLOC %s allocation_begin=%p allocation_bytes=%lld "
                "allocation_element_count=%lld allocation_element_range=[%lld,%lld) "
                "(relative to src.data; source=ASan chunk)\n",
                label, chunk, (long long)chunk_bytes, (long long)(chunk_bytes / elem),
                (long long)(lo / elem), (long long)(hi / elem));
    const int tail16 = scalar_tail_first_x(dst_cols, 16), tail8 = scalar_tail_first_x(dst_cols, 8);
    // `src + srcstep * iy + ix` parses as (src + srcstep*iy) + ix and srcstep is size_t
    // (warp_kernels.simd.hpp:368, :3690). Two pointers are formed, and each must be valid:
    //   P1 = src + srcstep*iy   -- for iy < 0 the operand srcstep*(size_t)iy is the huge
    //        unsigned value 2^64 - srcstep*|iy|, so P1 is outside the array for ANY layout
    //        ([expr.add]/4), even though the generated code wraps to src - srcstep*|iy|;
    //   P2 = P1 + ix            -- signed int operand.
    auto classify = [&](int64_t byte, bool in_image, bool &rob) {
        if (chunk == nullptr) { rob = false; return Ptr::UNDETERMINED; }
        Ptr p;
        if (byte < lo || byte > hi) p = Ptr::OUTSIDE_ALLOCATION;
        else if (byte == hi) p = Ptr::ONE_PAST;
        else p = in_image ? Ptr::WITHIN_ALLOCATION : Ptr::WITHIN_PARENT_OUTSIDE_ROI;
        rob = p == Ptr::OUTSIDE_ALLOCATION && (byte < robust_lo || byte > robust_hi);
        return p;
    };
    // Primary classification: the EFFECTIVE pointer src + srcstep*iy + ix (what the generated
    // code computes). Secondary, reported separately so no conclusion depends on it: under the
    // strict unsigned reading, every pixel with iy < 0 also forms an out-of-array P1.
    PtrScope all, avx2, base8;
    long neg_iy_avx2 = 0, neg_iy_base8 = 0, neg_iy_all = 0;
    for (int y = 0; y < dst_rows; ++y)
        for (int x = 0; x < dst_cols; ++x) {
            const float sx = float(x) * M[0] + float(y) * M[1] + M[2];
            const float sy = float(x) * M[3] + float(y) * M[4] + M[5];
            const int64_t ix = mapped_index(sx, linear), iy = mapped_index(sy, linear);
            const int64_t off = step_el * iy + ix;   // |off| < 2^48: no int64 overflow
            const bool in_image = ix >= 0 && ix < src.cols && iy >= 0 && iy < src.rows;
            bool rob = false;
            const Ptr p = classify(off * elem, in_image, rob);
            all.add(p, rob, off, ix, iy, sx, sy, x, y);
            if (iy < 0) ++neg_iy_all;
            if (x >= tail16) {
                avx2.add(p, rob, off, ix, iy, sx, sy, x, y);
                if (iy < 0) ++neg_iy_avx2;
            }
            if (x >= tail8) {
                base8.add(p, rob, off, ix, iy, sx, sy, x, y);
                if (iy < 0) ++neg_iy_base8;
            }
        }
    if (verbose) std::printf("  PTR_P1_UNSIGNED %s negative_iy_pixels avx2-scalar-tail=%ld "
                "baseline-scalar-tail=%ld all=%ld (strict reading only: src + size_t(srcstep)*iy "
                "with iy<0 adds ~2^64; not used for the primary verdict)\n",
                label, neg_iy_avx2, neg_iy_base8, neg_iy_all);
    if (verbose) std::printf("  PTR_ALLOC_BOUND %s mat_element_range=[%lld,%lld) "
                "placement_independent_element_range=[%lld,%lld]\n",
                label, (long long)(ds / elem), (long long)(dl / elem),
                (long long)(robust_lo / elem), (long long)(robust_hi / elem));
    if (verbose) std::printf("  PTR_TAIL %s dst=%dx%d avx2_uf=16 scalar_tail_x=[%d,%d) baseline_uf=8 "
                "scalar_tail_x=[%d,%d)\n",
                label, dst_cols, dst_rows, tail16, dst_cols, tail8, dst_cols);
    print_scope(label, "avx2-scalar-tail", avx2);
    print_scope(label, "baseline-scalar-tail", base8);
    if (verbose) print_scope(label, "all-pixels(hypothetical-fully-scalar)", all);
    for (const PtrScope *s : {&avx2, &base8}) {
        if (!verbose || s->worst != Ptr::OUTSIDE_ALLOCATION) continue;
        const bool representable = classify_coord(s->sx) == Conv::IN_RANGE &&
                                   classify_coord(s->sy) == Conv::IN_RANGE;
        std::printf("    SOURCE_LEVEL_OUT_OF_ALLOCATION %s scope=%s: the scalar tail forms "
                    "src + srcstep*iy + ix outside the source array BEFORE its bounds test "
                    "(%s index, %s); the border branch then performs no load\n",
                    label, s == &avx2 ? "avx2-scalar-tail" : "baseline-scalar-tail",
                    representable ? "int32-representable" : "INT_MIN-sentinel",
                    s->worst_robust ? "outside for any fastMalloc placement"
                                    : "outside for THIS allocation layout only");
    }
    return avx2.worst;
}


bool all_finite(const float w[6]) {
    for (int i = 0; i < 6; ++i)
        if (!std::isfinite(w[i])) return false;
    return true;
}
bool is_translation_valid(const float w[6]) {
    return all_finite(w) && w[0] == 1.0f && w[1] == 0.0f && w[3] == 0.0f && w[4] == 1.0f;
}
bool is_euclidean_valid(const float w[6]) {
    if (!all_finite(w)) return false;
    const double a = w[0], b = w[1], c = w[3], d = w[4];
    return std::fabs(a - d) <= 1e-6 && std::fabs(b + c) <= 1e-6 &&
           std::fabs(a * a + c * c - 1.0) <= 1e-5;
}
// The affine model has no structural invariant beyond finiteness (Task 014 contract).
bool is_affine_valid(const float w[6]) { return all_finite(w); }

bool model_valid(const std::string &model, const float w[6]) {
    if (model == "translation") return is_translation_valid(w);
    if (model == "euclidean") return is_euclidean_valid(w);
    if (model == "affine") return is_affine_valid(w);
    return false;
}

// Caller-seed domains published by the binding (cpp/opencv_video_shim.h): Euclidean
// |t| <= 2^20 and |theta| <= 1; affine |linear| <= 4 and |t| <= 2^20. Translation has no
// seeded operation.
const char *seed_domain(const std::string &model, const float w[6]) {
    if (!all_finite(w)) return "NONFINITE (not model-valid)";
    const float lim_t = 1048576.0f;
    const bool t_ok = std::fabs(w[2]) <= lim_t && std::fabs(w[5]) <= lim_t;
    if (model == "translation") return "NO_SEEDED_OPERATION";
    if (model == "euclidean") return t_ok ? "INSIDE_SEED_DOMAIN" : "OUTSIDE_SEED_DOMAIN";
    if (model == "affine") {
        const bool a_ok = std::fabs(w[0]) <= 4 && std::fabs(w[1]) <= 4 &&
                          std::fabs(w[3]) <= 4 && std::fabs(w[4]) <= 4;
        return t_ok && a_ok ? "INSIDE_SEED_DOMAIN" : "OUTSIDE_SEED_DOMAIN";
    }
    return "UNKNOWN";
}

// Build a native-shaped Euclidean warp exactly as the shim does: cos/sin in double, each
// element cast to float once, so a == d and b == -c hold bit-for-bit.
void euclidean_matrix(float m[6], double theta, float tx, float ty) {
    const double c = std::cos(theta), s = std::sin(theta);
    m[0] = float(c); m[1] = float(-s); m[2] = tx;
    m[3] = float(s); m[4] = float(c);  m[5] = ty;
}

void print_dispatch() {
#if defined(__x86_64__) || defined(_M_X64)
    std::printf("  DISPATCH OpenCV=%s AVX2=%d AVX512_SKX=%d SSE4_1=%d useOptimized=%d "
                "threads=%d\n", CV_VERSION, int(cv::checkHardwareSupport(CV_CPU_AVX2)),
                int(cv::checkHardwareSupport(CV_CPU_AVX512_SKX)),
                int(cv::checkHardwareSupport(CV_CPU_SSE4_1)), int(cv::useOptimized()),
                cv::getNumThreads());
#else
    std::printf("  DISPATCH OpenCV=%s useOptimized=%d threads=%d (non-x86)\n", CV_VERSION,
                int(cv::useOptimized()), cv::getNumThreads());
#endif
}


// ---------------------------------------------------------------------------------------
// Level A: replay one exact CV_32F matrix through the two ECC call shapes, BORDER_CONSTANT
// and WARP_INVERSE_MAP (ecc.cpp:523-526). Linear uses CV_32FC1, nearest CV_8UC1. With
// roi=true the source is a noncontiguous region of a larger parent (step > cols*elem).
struct LevelAOptions {
    bool linear = true;
    bool nearest = true;
    bool roi = false;
    // R1: report the exact srcptr of one named destination pixel (in addition to the scans).
    int probe_x = -1, probe_y = -1;
};

// R1: the exact source pointer of ONE destination pixel, in defined arithmetic (same model as
// pointer_range), printed with every quantity the task asks for.
void pixel_pointer(const char *label, const cv::Mat &src, const float M[6], bool linear, int x,
                   int y) {
    const int64_t elem = int64_t(src.elemSize1());
    const int64_t step_el = int64_t(src.step[0]) / elem;
    const float sx = float(x) * M[0] + float(y) * M[1] + M[2];
    const float sy = float(x) * M[3] + float(y) * M[4] + M[5];
    const int64_t ix = mapped_index(sx, linear), iy = mapped_index(sy, linear);
    const int64_t off = step_el * iy + ix;
    const void *chunk = __sanitizer_get_allocated_begin(src.data);
    const int64_t bytes = chunk ? int64_t(__sanitizer_get_allocated_size(chunk)) : 0;
    const int64_t lo = chunk ? int64_t(reinterpret_cast<uintptr_t>(chunk)) -
                                   int64_t(reinterpret_cast<uintptr_t>(src.data))
                             : 0;
    const int64_t hi = lo + bytes, b = off * elem;
    const char *verdict = !chunk ? "UNDETERMINED"
                          : (b < lo || b > hi) ? "PTR_OUTSIDE_ALLOCATION"
                          : b == hi ? "PTR_ONE_PAST"
                          : (ix >= 0 && ix < src.cols && iy >= 0 && iy < src.rows)
                              ? "PTR_WITHIN_ALLOCATION"
                              : "PTR_WITHIN_PARENT_OUTSIDE_ROI";
    std::printf("  PIXEL_PTR %s dst=(x=%d,y=%d) mapped=(%.9g,%.9g) mapped_ix=%lld mapped_iy=%lld "
                "source_step_elements=%lld mathematical_element_offset=%lld "
                "logical_last_index=%lld logical_one_past=%lld "
                "allocation_element_range=[%lld,%lld) beyond_one_past_by=%lld "
                "before_begin_by=%lld verdict=%s\n",
                label, x, y, double(sx), double(sy), (long long)ix, (long long)iy,
                (long long)step_el, (long long)off,
                (long long)(int64_t(src.rows - 1) * step_el + src.cols - 1),
                (long long)(int64_t(src.rows - 1) * step_el + src.cols), (long long)(lo / elem),
                (long long)(hi / elem), (long long)(off - hi / elem > 0 ? off - hi / elem : 0),
                (long long)(lo / elem - off > 0 ? lo / elem - off : 0), verdict);
}

void level_a(const char *name, const std::string &model, const float coeff[6], int rows,
             int cols, LevelAOptions opt) {
    std::printf("[level-a] %s model=%s geometry %dx%d (cols x rows)%s\n", name, model.c_str(),
                cols, rows, opt.roi ? " noncontiguous-ROI" : "");
    print_dispatch();
    print_matrix("  stored CV_32F M=", coeff);
    print_matrix_bits(coeff);
    std::printf("  model-valid intermediate=%s seed-domain=%s\n",
                model_valid(model, coeff) ? "YES" : "NO", seed_domain(model, coeff));
    const CoordReach reach = coordinate_reach(coeff, rows, cols);
    print_reach(name, reach);

    cv::Mat map(2, 3, CV_32F);
    for (int i = 0; i < 6; ++i) map.ptr<float>()[i] = coeff[i];
    if (opt.linear) {
        cv::Mat parent, image, dst;
        texture(rows + 8, cols + 16).convertTo(parent, CV_32F);
        image = opt.roi ? parent(cv::Rect(5, 3, cols, rows))
                        : parent(cv::Rect(0, 0, cols, rows)).clone();
        std::printf("  SOURCE linear continuous=%d step=%zu bytes\n", int(image.isContinuous()),
                    image.step[0]);
        pointer_range("linear", image, coeff, true, rows, cols);
        if (opt.probe_x >= 0) pixel_pointer("linear", image, coeff, true, opt.probe_x, opt.probe_y);
        std::printf("  NATIVE_CALL_BEGIN warpAffine CV_32FC1 INTER_LINEAR+WARP_INVERSE_MAP\n");
        std::fflush(stdout);
        cv::warpAffine(image, dst, map, cv::Size(cols, rows),
                       cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
        std::printf("  NATIVE_CALL_END linear dst_sum=%.9g\n", cv::sum(dst)[0]);
    }
    if (opt.nearest) {
        cv::Mat parent(rows + 8, cols + 16, CV_8UC1, cv::Scalar(1)), mask, dst;
        mask = opt.roi ? parent(cv::Rect(5, 3, cols, rows))
                       : parent(cv::Rect(0, 0, cols, rows)).clone();
        std::printf("  SOURCE nearest continuous=%d step=%zu bytes\n", int(mask.isContinuous()),
                    mask.step[0]);
        pointer_range("nearest", mask, coeff, false, rows, cols);
        if (opt.probe_x >= 0) pixel_pointer("nearest", mask, coeff, false, opt.probe_x, opt.probe_y);
        std::printf("  NATIVE_CALL_BEGIN warpAffine CV_8UC1 INTER_NEAREST+WARP_INVERSE_MAP\n");
        std::fflush(stdout);
        cv::warpAffine(mask, dst, map, cv::Size(cols, rows),
                       cv::INTER_NEAREST + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
        std::printf("  NATIVE_CALL_END nearest dst_sum=%.9g\n", cv::sum(dst)[0]);
    }
    std::fflush(stdout);
}

// ---------------------------------------------------------------------------------------
// Level B: every intermediate matrix native findTransformECC passes to warpAffine. Native
// ECC is deterministic and its loop body does not depend on maxCount, so a call capped at k
// iterations leaves in the caller's warp exactly the matrix iteration k+1 consumes. The
// capped call that PRODUCES a matrix never consumes it, so a trace by itself executes no
// unsafe warp; Level A/C show execution. Uses cv::findTransformECC directly with the
// binding's arguments (no mask, Gaussian 5, COUNT|EPS, eps 1e-8); it does not reimplement ECC.
long trace(const char *name, int motionType, const std::string &model, const float seed[6],
           int iterations, const cv::Mat &tmpl, const cv::Mat &moving) {
    const int rows = tmpl.rows, cols = tmpl.cols;
    std::printf("[trace] %s OpenCV %s motion=%d geometry %dx%d up to %d iterations eps=1e-8\n",
                name, CV_VERSION, motionType, cols, rows, iterations);
    print_matrix("  seed=", seed);
    std::printf("  seed model-valid=%s seed-domain=%s\n", model_valid(model, seed) ? "YES" : "NO",
                seed_domain(model, seed));
    // R1: a source shaped exactly like ECC's imageFloat/preMask (Mat(hd, wd, type), contiguous,
    // same size as the template). The mapped coordinates depend only on the warp and geometry,
    // not on pixel values, so these stand in for the real intermediates the kernel receives.
    cv::Mat img32(rows, cols, CV_32FC1, cv::Scalar(0));
    cv::Mat mask8(rows, cols, CV_8UC1, cv::Scalar(1));
    long first_unsafe = -1, first_oob = -1;
    double max_reach = 0.0;
    cv::Mat previous;
    for (int k = 0; k < iterations; ++k) {
        cv::Mat warp(2, 3, CV_32F);
        for (int i = 0; i < 6; ++i) warp.ptr<float>()[i] = seed[i];
        std::string outcome = "returned";
        if (k > 0) {
            try {
                cv::findTransformECC(tmpl, moving, warp, motionType,
                                     cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                                      k, 1e-8),
                                     cv::noArray(), 5);
            } catch (const cv::Exception &) {
                outcome = "raised";
            }
        }
        if (!previous.empty() && cv::norm(previous, warp, cv::NORM_INF) == 0.0) {
            std::printf("  k=%d warp unchanged (%s): native ECC presents no further matrix; "
                        "trace ends\n", k, outcome.c_str());
            break;
        }
        previous = warp.clone();
        const float *w = warp.ptr<float>();
        const CoordReach reach = coordinate_reach(w, rows, cols);
        max_reach = std::max(max_reach, double(reach.max_abs_finite));
        std::printf("  warp entering iteration %d (%s after %d updates) reach=%.9g: ", k + 1,
                    outcome.c_str(), k, double(reach.max_abs_finite));
        print_matrix("", w);
        if (!model_valid(model, w)) std::printf("    MODEL_INVARIANT_VIOLATED\n");
        // R1: ordinary (int32-representable) out-of-allocation pointer formation, per kernel.
        const Ptr pl = pointer_range("    iter-linear", img32, w, true, rows, cols, false);
        const Ptr pn = pointer_range("    iter-nearest", mask8, w, false, rows, cols, false);
        if ((pl == Ptr::OUTSIDE_ALLOCATION || pn == Ptr::OUTSIDE_ALLOCATION) && first_oob < 0) {
            first_oob = k + 1;
            std::printf("    FIRST_OUT_OF_ALLOCATION_ITERATION=%d linear=%s nearest=%s\n", k + 1,
                        ptr_name(pl), ptr_name(pn));
        }
        if (reach.worst != Conv::IN_RANGE) {
            std::printf("    UNSAFE_INTERMEDIATE iteration=%d\n", k + 1);
            print_reach("   ", reach);
            first_unsafe = k + 1;
            break;
        }
    }
    std::printf("  first unsafe iteration=%ld max|finite_coord| reached=%.9g (2^31=%.9g)\n",
                first_unsafe, max_reach, double(kTwoPow31));
    std::printf("  TRACE_SUMMARY %s first_conversion_unsafe_iteration=%ld "
                "first_out_of_allocation_iteration=%ld\n",
                name, first_unsafe, first_oob);
    return first_unsafe;
}

// ---------------------------------------------------------------------------------------
// Level C: the real exported C ABI, validated caller inputs, real Core-owned Mat handles.
void binding_end(opencv_video_status s) {
    std::printf("  BINDING_CALL_END status=%d message=\"%s\"\n", int(s),
                s == OPENCV_VIDEO_OK ? "" : opencv_video_last_error());
}

void level_c_translation(int iterations, const cv::Mat &tmpl, const cv::Mat &moving) {
    std::printf("[level-c] opencv_video_estimate_ecc_translation %dx%d iters=%d eps=1e-8\n",
                tmpl.cols, tmpl.rows, iterations);
    print_dispatch();
    Handle th = make_handle(tmpl), mh = make_handle(moving);
    opencv_video_ecc_translation_result r = {0, 0, 0};
    std::printf("  BINDING_CALL_BEGIN\n");
    std::fflush(stdout);
    const opencv_video_status s =
        opencv_video_estimate_ecc_translation(th.get(), mh.get(), iterations, 1e-8, &r);
    binding_end(s);
    if (s == OPENCV_VIDEO_OK)
        std::printf("  published displacement=(%.9g, %.9g) correlation=%.9g\n",
                    double(r.displacement_x), double(r.displacement_y), r.correlation);
}

void level_c_euclidean(bool seeded, float tx, float ty, double theta, int iterations,
                       const cv::Mat &tmpl, const cv::Mat &moving) {
    std::printf("[level-c] %s %dx%d iters=%d eps=1e-8\n",
                seeded ? "opencv_video_estimate_ecc_euclidean_seeded"
                       : "opencv_video_estimate_ecc_euclidean",
                tmpl.cols, tmpl.rows, iterations);
    print_dispatch();
    if (seeded)
        std::printf("  validated caller seed tx=%.9g ty=%.9g theta=%.9g\n", double(tx),
                    double(ty), theta);
    Handle th = make_handle(tmpl), mh = make_handle(moving);
    opencv_video_ecc_euclidean_result r = {0, 0, 0, 0};
    std::printf("  BINDING_CALL_BEGIN\n");
    std::fflush(stdout);
    const opencv_video_status s =
        seeded ? opencv_video_estimate_ecc_euclidean_seeded(th.get(), mh.get(), iterations, 1e-8,
                                                            tx, ty, theta, &r)
               : opencv_video_estimate_ecc_euclidean(th.get(), mh.get(), iterations, 1e-8, &r);
    binding_end(s);
    if (s == OPENCV_VIDEO_OK)
        std::printf("  published displacement=(%.9g, %.9g) angle=%.9g correlation=%.9g\n",
                    double(r.displacement_x), double(r.displacement_y), r.rotation_radians,
                    r.correlation);
}

void level_c_affine(bool seeded, const float seed[6], int iterations, const cv::Mat &tmpl,
                    const cv::Mat &moving) {
    std::printf("[level-c] %s %dx%d iters=%d eps=1e-8\n",
                seeded ? "opencv_video_estimate_ecc_affine_seeded"
                       : "opencv_video_estimate_ecc_affine",
                tmpl.cols, tmpl.rows, iterations);
    print_dispatch();
    if (seeded) print_matrix("  validated caller seed=", seed);
    Handle th = make_handle(tmpl), mh = make_handle(moving);
    opencv_video_ecc_affine_result r = {0, 0, 0, 0, 0, 0, 0};
    std::printf("  BINDING_CALL_BEGIN\n");
    std::fflush(stdout);
    const opencv_video_status s =
        seeded ? opencv_video_estimate_ecc_affine_seeded(th.get(), mh.get(), iterations, 1e-8,
                                                         seed[0], seed[1], seed[2], seed[3],
                                                         seed[4], seed[5], &r)
               : opencv_video_estimate_ecc_affine(th.get(), mh.get(), iterations, 1e-8, &r);
    binding_end(s);
    if (s == OPENCV_VIDEO_OK) {
        const float w[6] = {r.m00, r.m01, r.m02, r.m10, r.m11, r.m12};
        print_matrix("  published result M=", w);
    }
}

void print_peak_rss() {
    std::FILE *f = std::fopen("/proc/self/status", "r");
    if (f == nullptr) return;
    char line[256];
    while (std::fgets(line, sizeof line, f) != nullptr)
        if (std::string(line).rfind("VmHWM:", 0) == 0) std::printf("  PEAK_RSS %s", line + 6);
    std::fclose(f);
}


// ---------------------------------------------------------------------------------------
// conversion-check: confirm the Float32 conversion model against the hardware through the
// DEFINED x86 intrinsics the kernels use (cvRound(float) == _mm_cvtss_si32; cvFloor(float)
// with GCC builtins == truncation of floorf, observed here through _mm_cvttss_si32 of the
// already-floored value). Both return 0x80000000 for out-of-range/nonfinite input.
int conversion_check() {
    int mismatches = 0;
    const int mode = std::fegetround();
    std::printf("  ROUNDING_MODE %s\n", mode == FE_TONEAREST ? "FE_TONEAREST" : "NON_DEFAULT");
    if (mode != FE_TONEAREST) ++mismatches;
#if defined(__x86_64__) || defined(_M_X64)
    const unsigned rc = (_mm_getcsr() >> 13) & 3u;
    std::printf("  MXCSR_RC %u%s\n", rc, rc == 0 ? " (round-to-nearest-even)" : " (UNEXPECTED)");
    if (rc != 0) ++mismatches;
#endif
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float below = std::nextafterf(kTwoPow31, 0.0f);          // 2^31 - 128
    const float above = std::nextafterf(kTwoPow31, inf);           // 2^31 + 256
    const float neg_below = -kTwoPow31;                            // exactly INT_MIN
    const float neg_beyond = std::nextafterf(-kTwoPow31, -inf);    // -(2^31 + 256)
    // -2^31 is IN_RANGE and converts exactly to INT_MIN, which is indistinguishable from the
    // sentinel by value; it is checked against the expected class only.
    struct P { float v; Conv expect; };
    const P probes[] = {
        {0.0f, Conv::IN_RANGE},          {100.75f, Conv::IN_RANGE},
        {-100.75f, Conv::IN_RANGE},      {1048576.0f, Conv::IN_RANGE},
        {16777216.0f, Conv::IN_RANGE},   {below, Conv::IN_RANGE},
        {neg_below, Conv::IN_RANGE},     {kTwoPow31, Conv::OUT_OF_RANGE_FINITE},
        {above, Conv::OUT_OF_RANGE_FINITE}, {neg_beyond, Conv::OUT_OF_RANGE_FINITE},
        {4294967296.0f, Conv::OUT_OF_RANGE_FINITE}, {1.0e9f * 3.0f, Conv::OUT_OF_RANGE_FINITE},
        {1.0e12f, Conv::OUT_OF_RANGE_FINITE}, {std::numeric_limits<float>::max(), Conv::OUT_OF_RANGE_FINITE},
        {inf, Conv::NONFINITE},          {-inf, Conv::NONFINITE},
        {nan, Conv::NONFINITE},
    };
    for (const P &p : probes) {
        const Conv c = classify_coord(p.v);
        bool ok = c == p.expect;
#if defined(__x86_64__) || defined(_M_X64)
        const int hw_round = _mm_cvtss_si32(_mm_set_ss(p.v));
        const float fl = std::floor(p.v);    // defined for every float (NaN/Inf propagate)
        const int hw_floor = _mm_cvttss_si32(_mm_set_ss(fl));
        // Out-of-range/nonfinite must give the sentinel; in-range (other than -2^31) must not.
        if (p.expect != Conv::IN_RANGE)
            ok = ok && hw_round == INT32_MIN && hw_floor == INT32_MIN;
        else if (p.v != neg_below)
            ok = ok && hw_round != INT32_MIN && hw_floor != INT32_MIN;
        std::printf("  CONVERSION v=%.9g bits=0x%08x class=%s expect=%s cvRound=%d "
                    "cvFloor=%d %s\n", double(p.v), unsigned(float_bits(p.v)), conv_name(c),
                    conv_name(p.expect), hw_round, hw_floor, ok ? "MATCH" : "MISMATCH");
#else
        std::printf("  CONVERSION v=%.9g bits=0x%08x class=%s expect=%s %s (non-x86)\n",
                    double(p.v), unsigned(float_bits(p.v)), conv_name(c), conv_name(p.expect),
                    ok ? "MATCH" : "MISMATCH");
#endif
        if (!ok) ++mismatches;
    }
    std::printf("  BOUNDARY 2^31=%.9g last_in_range=%.9g(0x%08x) first_out=%.9g(0x%08x) "
                "spacing=%.9g\n", double(kTwoPow31), double(below), unsigned(float_bits(below)),
                double(kTwoPow31), unsigned(float_bits(kTwoPow31)), double(kTwoPow31 - below));
    // Decimal literals do not keep their written value in float32.
    const float d1 = 2.1474836e9f, d2 = 1.0e9f;
    std::printf("  LITERAL 2.1474836e9f stored=%.9g (%s); 1e9f stored=%.9g\n", double(d1),
                conv_name(classify_coord(d1)), double(d2));
    std::printf("  %s\n", mismatches == 0 ? "CONVERSION_MODEL_OK" : "CONVERSION_MODEL_FAILED");
    return mismatches;
}

// ---------------------------------------------------------------------------------------
// Bounded, deterministic search (opt-in only). A trial index fully identifies its inputs.
// flavor 0 translation identity, 1 Euclidean identity, 2 Euclidean seeded, 3 affine identity,
// 4 affine seeded. Geometry 32..160 x 32..128; seeds inside the published domains.
struct Trial { int rows = 0, cols = 0, kind = 0; cv::Mat tmpl, moving; float seed[6] = {1, 0, 0, 0, 1, 0}; };

Trial campaign_trial(uint32_t index, int flavor) {
    uint32_t state = 22u * 2654435761u + index * 40503u + uint32_t(flavor) * 7u;
    auto rnd = [&]() { state = state * 1664525u + 1013904223u; return double(state) / 4294967296.0; };
    Trial t;
    t.rows = 32 + int(rnd() * 97);
    t.cols = 32 + int(rnd() * 129);
    t.kind = int(index % 10);
    const double deg = 30.0 * (rnd() - 0.5);
    const double tx = 40.0 * (rnd() - 0.5), ty = 40.0 * (rnd() - 0.5);
    t.tmpl = texture(t.rows, t.cols);
    switch (t.kind) {
        case 0: t.moving = texture(t.rows, t.cols, 2, 1); break;
        case 1: t.moving = texture(t.rows, t.cols, int(10 * rnd()), int(10 * rnd())); break;
        case 2: t.moving = rigid_texture(t.rows, t.cols, deg, tx, ty); break;
        case 3: t.moving = affine_texture(t.rows, t.cols, 1.1, 0.1, -0.05, 0.9, tx, ty); break;
        case 4: t.moving = low_variance(texture(t.rows, t.cols, 2, 1)); break;
        case 5: t.moving = constant(t.rows, t.cols, 128); t.tmpl = constant(t.rows, t.cols, 128); break;
        case 6: t.moving = stripes(t.rows, t.cols, 3); t.tmpl = stripes(t.rows, t.cols, 0); break;
        case 7: t.moving = noise(t.rows, t.cols, index + 1); break;
        case 8: t.moving = texture(t.rows, t.cols, 1, 0); t.tmpl = low_variance(t.tmpl); break;
        default: {
            const cv::Mat parent = texture(t.rows + 16, t.cols + 24, 2, 1);
            t.moving = parent(cv::Rect(12, 8, t.cols, t.rows)).clone();
            break;
        }
    }
    const double u = rnd(), v = rnd(), w = rnd();
    if (flavor == 2) {
        // In-domain Euclidean seed, including displacement near the 2^20 limit.
        const double big = (index % 3 == 0) ? 1048576.0 - 64.0 * u : 60.0 * (u - 0.5);
        euclidean_matrix(t.seed, 1.8 * (v - 0.5), float(big), float(-big * w));
    } else if (flavor == 4) {
        // In-domain affine seed (|linear| <= 4, |t| <= 2^20), including the extremes.
        const double s = (index % 3 == 0) ? 4.0 : 1.0 + 0.5 * (u - 0.5);
        const double big = (index % 3 == 1) ? 1048576.0 - 64.0 * v : 60.0 * (v - 0.5);
        t.seed[0] = float(s);              t.seed[1] = float(3.0 * (w - 0.5));
        t.seed[2] = float(big);            t.seed[3] = float(-3.0 * (u - 0.5));
        t.seed[4] = float(s * (v < 0.5 ? -1 : 1)); t.seed[5] = float(-big * w);
    }
    return t;
}

int first_unsafe_iteration(const Trial &t, int motionType, int iterations, float out[6],
                           double &max_reach) {
    cv::Mat previous;
    max_reach = 0.0;
    for (int k = 0; k < iterations; ++k) {
        cv::Mat warp(2, 3, CV_32F);
        for (int i = 0; i < 6; ++i) warp.ptr<float>()[i] = t.seed[i];
        if (k > 0) {
            try {
                cv::findTransformECC(t.tmpl, t.moving, warp, motionType,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, k, 1e-8),
                    cv::noArray(), 5);
            } catch (const cv::Exception &) {
            }
        }
        if (!previous.empty() && cv::norm(previous, warp, cv::NORM_INF) == 0.0) return -1;
        previous = warp.clone();
        const CoordReach reach = coordinate_reach(warp.ptr<float>(), t.rows, t.cols);
        max_reach = std::max(max_reach, double(reach.max_abs_finite));
        if (reach.worst != Conv::IN_RANGE) {
            for (int i = 0; i < 6; ++i) out[i] = warp.ptr<float>()[i];
            return k + 1;
        }
    }
    return -1;
}

void search(const char *name, int flavor, int motionType, uint32_t first, uint32_t count) {
    std::printf("[search] %s trials %u..%u OpenCV %s (100 iterations, eps 1e-8, no mask)\n",
                name, first, first + count - 1, CV_VERSION);
    long unsafe = 0;
    double global = 0.0;
    uint32_t worst = first;
    for (uint32_t n = first; n < first + count; ++n) {
        const Trial t = campaign_trial(n, flavor);
        float w[6] = {0, 0, 0, 0, 0, 0};
        double mr = 0.0;
        const int it = first_unsafe_iteration(t, motionType, 100, w, mr);
        if (mr > global) { global = mr; worst = n; }
        if (it < 0) continue;
        ++unsafe;
        std::printf("  SEARCH_UNSAFE trial=%u kind=%d geometry=%dx%d iteration=%d ", n, t.kind,
                    t.cols, t.rows, it);
        print_matrix("unsafe warp=", w);
    }
    std::printf("  SEARCH_SUMMARY %s trials=%u unsafe=%ld max|finite_coord| reached=%.9g "
                "(trial %u)\n", name, count, unsafe, global, worst);
}

}  // namespace

int main(int argc, char **argv) {
    const std::string mode = argc > 1 ? argv[1] : "negative";
    const float qnan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    // Deterministic ECC fixtures (64x97, the Task 021 geometry).
    const cv::Mat tmpl = texture(64, 97);
    const cv::Mat moving = texture(64, 97, 2, 1);
    const cv::Mat rigid_moving = rigid_texture(64, 97, 6.0, 2.0, 1.0);
    const cv::Mat affine_moving = affine_texture(64, 97, 1.02, 0.01, -0.01, 1.03, 2.0, 1.0);
    // Task 020 seeded-affine trial n435 fixture, verbatim (texture 64x100, (+2,+1) shift).
    const cv::Mat tmpl_n435 = texture(64, 100), moving_n435 = texture(64, 100, 2, 1);
    const float identity[6] = {1, 0, 0, 0, 1, 0};

    // Finite, model-valid matrices beyond int32 (each value is an exact float32).
    const float trans_oor[6] = {1, 0, 3.0e9f, 0, 1, 3.0e9f};
    float euclid_oor[6];
    euclidean_matrix(euclid_oor, -0.5, 3.0e9f, -3.0e9f);
    const float affine_oor[6] = {1.5f, 0.25f, 3.0e9f, -0.25f, -1.5f, 1.0e12f};
    // 2^31 boundary pair: the last float32 below 2^31 and 2^31 itself, applied in X at x=0.
    const float trans_below[6] = {1, 0, std::nextafterf(kTwoPow31, 0.0f), 0, 1, 0};
    const float trans_at[6] = {1, 0, kTwoPow31, 0, 1, 0};
    // Negative controls.
    const float far_control[6] = {1, 0, 1.0e6f, 0, 1, -1.0e6f};
    // Task 020 recorded affine intermediate (PR #19 Matrix B).
    const float matrix_b[6] = {-5597.05908203125f, 8140.32177734375f, 285284.03125f,
                               -9895.33203125f, -362406.0f, 484909.9375f};
    // Task 019/020 seeded-affine trial n435 seed, verbatim.
    const float seed_n435[6] = {0.45051536f, 0.286671013f, 23.7435341f,
                                -0.437181264f, 1.13443863f, -37.4001312f};
    // Nonfinite diagnostics (not model-valid).
    const float nan_trans[6] = {1, 0, qnan, 0, 1, qnan};
    const float inf_trans[6] = {1, 0, inf, 0, 1, -inf};
    float nan_euclid[6];
    for (float &v : nan_euclid) v = qnan;
    const float nan_affine_coeff[6] = {1, qnan, 0, 0, 1, 0};
    // Finite coefficients whose float product overflows to +Inf at a far pixel.
    const float overflow_to_inf[6] = {3.0e38f, 0, 3.0e38f, 0, 1, 0};

    // UBSan's pointer-overflow check fires only when the byte offset wraps the address space.
    // Linear (4-byte stride) wraps at a ~16k-wide source; nearest (1-byte) needs ~64k.
    const int wide_linear = 16383, wide_nearest = 65535;
    LevelAOptions lin_only;
    lin_only.nearest = false;
    LevelAOptions near_only;
    near_only.linear = false;
    const LevelAOptions both;

    if (mode == "negative") {
        int bad = 0;
        float rigid[6];
        euclidean_matrix(rigid, 0.1, 3.0f, -2.0f);
        const float small_t[6] = {1, 0, 2, 0, 1, 1};
        const float aff[6] = {1.1f, 0.05f, 5.0f, -0.03f, 0.95f, -4.0f};
        const float *const controls[3] = {small_t, rigid, aff};
        for (const float *m : controls)
            if (coordinate_reach(m, 64, 97).worst != Conv::IN_RANGE) ++bad;
        level_a("negative-translation", "translation", small_t, 64, 97, both);
        level_a("negative-euclidean", "euclidean", rigid, 64, 97, both);
        level_a("negative-affine", "affine", aff, 64, 97, both);
        std::printf("%s\n", bad == 0 ? "NEGATIVE_CONTROL_OK" : "NEGATIVE_CONTROL_FAILED");
        return bad == 0 ? 0 : 1;
    }
    if (mode == "conversion-check") return conversion_check() == 0 ? 0 : 1;

    // ---- R1 (2026-10-19): ordinary int32-representable out-of-allocation pointer formation.
    // Exact Float32 translations: 2, 1, -2, -1 are exact. WARP_INVERSE_MAP: src = M * dst.
    const float r1_pos[6] = {1, 0, 2, 0, 1, 1};      // (+2,+1): bottom-right corner overruns
    const float r1_neg[6] = {1, 0, -2, 0, 1, -1};    // (-2,-1): top-left corner underruns
    LevelAOptions r1_both = both;
    if (mode == "r1-ordinary-positive" || mode == "r1-ordinary-positive-baseline" ||
        mode == "r1-ordinary-negative" || mode == "r1-ordinary-negative-baseline") {
        const bool neg = mode.find("negative") != std::string::npos;
        const bool baseline = mode.find("baseline") != std::string::npos;
        if (baseline) cv::setUseOptimized(false);   // forces cv::cpu_baseline
        // The scalar-tail pixel nearest the overrunning/underrunning corner.
        r1_both.probe_x = 96;
        r1_both.probe_y = neg ? 0 : 63;
        level_a(neg ? "r1-ordinary-negative" : "r1-ordinary-positive", "translation",
                neg ? r1_neg : r1_pos, 64, 97, r1_both);
        std::printf("R1_ORDINARY_DONE\n");
    } else if (mode == "r1-roi-positive" || mode == "r1-roi-negative") {
        // Noncontiguous ROI at (5,3) inside a (cols+16)x(rows+8) parent: the out-of-image
        // srcptr may still be inside the parent allocation.
        const bool neg = mode == "r1-roi-negative";
        LevelAOptions roi = both;
        roi.roi = true;
        roi.probe_x = 96;
        roi.probe_y = neg ? 0 : 63;
        level_a(neg ? "r1-roi-negative" : "r1-roi-positive", "translation", neg ? r1_neg : r1_pos,
                64, 97, roi);
        std::printf("R1_ORDINARY_DONE\n");
    } else if (mode == "r1-sweep") {
        // One width per process. AVX2 uf=16: widths divisible by 16 have NO scalar tail.
        const int w = argc > 2 ? std::atoi(argv[2]) : 97;
        if (w < 32 || w > 4096) {
            std::fprintf(stderr, "r1-sweep width outside the 32..4096 budget\n");
            return 2;
        }
        // argv[3] == "neg" uses (-2,-1); the probe pixel is then the FIRST AVX2 scalar-tail
        // column of row 0 (smallest offset), else the last column of the last row.
        const bool neg = argc > 3 && std::string(argv[3]) == "neg";
        if (argc > 4 && std::string(argv[4]) == "baseline") cv::setUseOptimized(false);
        r1_both.probe_x = neg ? std::min(scalar_tail_first_x(w, 16), w - 1) : w - 1;
        r1_both.probe_y = neg ? 0 : 63;
        level_a(neg ? "r1-sweep-negative" : "r1-sweep", "translation", neg ? r1_neg : r1_pos, 64,
                w, r1_both);
        std::printf("R1_ORDINARY_DONE\n");
    } else if (mode == "r1-identity-control") {
        // Genuine pointer-range-negative control: identity over a contiguous source. Every
        // scalar-tail srcptr is inside the image (worst offset = last element).
        r1_both.probe_x = 96;
        r1_both.probe_y = 63;
        level_a("r1-identity-control", "translation", identity, 64, 97, r1_both);
        cv::Mat a(64, 97, CV_32FC1, cv::Scalar(0)), b(64, 97, CV_8UC1, cv::Scalar(1));
        const bool ok = pointer_range("check-linear", a, identity, true, 64, 97, false) !=
                            Ptr::OUTSIDE_ALLOCATION &&
                        pointer_range("check-nearest", b, identity, false, 64, 97, false) !=
                            Ptr::OUTSIDE_ALLOCATION;
        std::printf("%s\n", ok ? "POINTER_RANGE_NEGATIVE_CONTROL_OK"
                               : "POINTER_RANGE_NEGATIVE_CONTROL_FAILED");
        return ok ? 0 : 1;
    }
    if (mode.rfind("r1-", 0) == 0) {
        print_peak_rss();
        return 0;
    }
    if (mode == "level-a-far-control" || mode == "level-a-matrix-b") {
        const bool far = mode == "level-a-far-control";
        const float *m = far ? far_control : matrix_b;
        const int cols = far ? 97 : 100;
        level_a(far ? "far-control" : "matrix-B", far ? "translation" : "affine", m, 64, cols, both);
        const bool ok = coordinate_reach(m, 64, cols).worst == Conv::IN_RANGE;
        std::printf("%s\n", ok ? "NEGATIVE_CONTROL_OK" : "NEGATIVE_CONTROL_FAILED");
        return ok ? 0 : 1;
    }
    if (mode == "level-a-boundary-below") {
        // Last float32 below 2^31: in range. Wide geometry so any GEP wrap would be visible.
        level_a("boundary-below", "translation", trans_below, 64, wide_linear, lin_only);
        const bool ok = coordinate_reach(trans_below, 1, 1).worst == Conv::IN_RANGE;
        std::printf("%s\n", ok ? "NEGATIVE_CONTROL_OK" : "NEGATIVE_CONTROL_FAILED");
        return ok ? 0 : 1;
    }
    if (mode == "level-a-boundary-above")
        level_a("boundary-at-2^31", "translation", trans_at, 64, wide_linear, lin_only);
    else if (mode == "level-a-finite-translation-linear")
        level_a("finite-translation-linear", "translation", trans_oor, 64, wide_linear, lin_only);
    else if (mode == "level-a-finite-translation-nearest")
        level_a("finite-translation-nearest", "translation", trans_oor, 64, wide_nearest, near_only);
    else if (mode == "level-a-finite-euclidean")
        level_a("finite-euclidean", "euclidean", euclid_oor, 64, wide_linear, lin_only);
    else if (mode == "level-a-finite-affine")
        level_a("finite-affine", "affine", affine_oor, 64, wide_linear, lin_only);
    else if (mode == "level-a-overflow-to-inf")
        level_a("overflow-to-inf", "affine", overflow_to_inf, 64, wide_linear, lin_only);
    else if (mode == "level-a-scalar-finite-linear" || mode == "level-a-scalar-finite-nearest") {
        cv::setUseOptimized(false);   // forces cv::cpu_baseline
        const bool lin = mode == "level-a-scalar-finite-linear";
        level_a(lin ? "scalar-finite-linear" : "scalar-finite-nearest", "translation", trans_oor,
                64, lin ? wide_linear : wide_nearest, lin ? lin_only : near_only);
    } else if (mode == "level-a-roi-linear") {
        LevelAOptions roi = lin_only;
        roi.roi = true;
        level_a("roi-linear", "translation", trans_oor, 64, wide_linear, roi);
    } else if (mode == "level-a-nonfinite-translation")
        level_a("nonfinite-translation", "translation", nan_trans, 64, wide_linear, lin_only);
    else if (mode == "level-a-inf-translation")
        level_a("inf-translation", "translation", inf_trans, 64, wide_linear, lin_only);
    else if (mode == "level-a-nonfinite-euclidean")
        level_a("nonfinite-euclidean", "euclidean", nan_euclid, 64, wide_linear, lin_only);
    else if (mode == "level-a-nonfinite-affine-coeff")
        level_a("nonfinite-affine-coeff", "affine", nan_affine_coeff, 64, wide_linear, lin_only);
    else if (mode == "level-a-sweep-linear" || mode == "level-a-sweep-nearest") {
        // One width per process so each width's UBSan report is independent.
        const bool lin = mode == "level-a-sweep-linear";
        const int w = argc > 2 ? std::atoi(argv[2]) : 97;
        if (w < 1 || w > 65536) {
            std::fprintf(stderr, "sweep width out of the 1..65536 budget\n");
            return 2;
        }
        level_a(lin ? "sweep-linear" : "sweep-nearest", "translation", trans_oor, 64, w,
                lin ? lin_only : near_only);
    } else if (mode == "trace-translation")
        trace("translation", cv::MOTION_TRANSLATION, "translation", identity, 100, tmpl, moving);
    else if (mode == "trace-euclidean")
        trace("euclidean", cv::MOTION_EUCLIDEAN, "euclidean", identity, 100, tmpl, rigid_moving);
    else if (mode == "trace-euclidean-seeded") {
        // Task 021 extreme in-domain seed (theta 0.9, tx = 2^20, ty = -2^20).
        float s[6];
        euclidean_matrix(s, 0.9, 1048576.0f, -1048576.0f);
        trace("euclidean-seeded", cv::MOTION_EUCLIDEAN, "euclidean", s, 100, tmpl, rigid_moving);
    } else if (mode == "trace-affine")
        trace("affine", cv::MOTION_AFFINE, "affine", identity, 100, tmpl, affine_moving);
    else if (mode == "trace-affine-n435")
        trace("affine-n435", cv::MOTION_AFFINE, "affine", seed_n435, 100, tmpl_n435, moving_n435);
    else if (mode == "level-c-translation")
        level_c_translation(100, tmpl, moving);
    else if (mode == "level-c-euclidean")
        level_c_euclidean(false, 0, 0, 0, 100, tmpl, rigid_moving);
    else if (mode == "level-c-euclidean-seeded")
        level_c_euclidean(true, 1048576.0f, -1048576.0f, 0.9, 100, tmpl, rigid_moving);
    else if (mode == "level-c-affine")
        level_c_affine(false, identity, 100, tmpl, affine_moving);
    else if (mode == "level-c-affine-n435")
        level_c_affine(true, seed_n435, 100, tmpl_n435, moving_n435);
    else if (mode == "search-translation" || mode == "search-euclidean" ||
             mode == "search-euclidean-seeded" || mode == "search-affine" ||
             mode == "search-affine-seeded") {
        const uint32_t first = argc > 2 ? uint32_t(std::strtoul(argv[2], nullptr, 10)) : 0;
        const uint32_t count = argc > 3 ? uint32_t(std::strtoul(argv[3], nullptr, 10)) : 50;
        if (count == 0 || count > 2000) {
            std::fprintf(stderr, "search count outside the 1..2000 budget\n");
            return 2;
        }
        int flavor = 0, motion = cv::MOTION_TRANSLATION;
        if (mode == "search-euclidean") { flavor = 1; motion = cv::MOTION_EUCLIDEAN; }
        if (mode == "search-euclidean-seeded") { flavor = 2; motion = cv::MOTION_EUCLIDEAN; }
        if (mode == "search-affine") { flavor = 3; motion = cv::MOTION_AFFINE; }
        if (mode == "search-affine-seeded") { flavor = 4; motion = cv::MOTION_AFFINE; }
        search(mode.c_str(), flavor, motion, first, count);
    } else {
        std::fprintf(stderr, "unknown case %s\n", mode.c_str());
        return 2;
    }
    print_peak_rss();
    return 0;
}
