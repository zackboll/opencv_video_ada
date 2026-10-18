// Task 021 opt-in research reproducer for the native signed-overflow risk in the existing
// ECC TRANSLATION and EUCLIDEAN APIs. It reuses the Task 020 methodology but draws
// model-specific conclusions; it does NOT import Task 020's affine matrices as evidence
// for these models.
//
// THIS IS NOT A SANITIZER-CLEAN TEST. Against an UBSan-instrumented OpenCV 4.1/4.6/4.10
// build it is EXPECTED to print signed-integer-overflow reports from inside warpAffine for
// the deliberately extreme Level A cases. Those reports are the evidence, not a failure.
// Never run it in routine hosted CI; build it only through
// scripts/research_ecc_translation_euclidean_overflow.sh against a custom instrumented
// OpenCV install (docs/tasks/021-translation-euclidean-safety-audit.md).
//
// Cases (exactly one per process, because UBSan reports a source location only once):
//   negative
//              Moderate translation and moderate Euclidean warps; must report zero
//              int32-range overflows in the int64 model and zero UBSan reports.
//   conversion-check
//              Observes the target's cvtsd2si result for boundary/NaN doubles through the
//              intrinsic, checks the diagnostic's explicit conversion model against it, maps
//              the per-column overflow window at the widest permitted geometry, and (Task
//              021-R1) maps the row-base rounding window and the 64x97 row-base model.
//   level-a-rowbase-y / level-a-rowbase-x
//              Level A (Task 021-R1): direct native warpAffine of a model-valid translation
//              [1 0 0; 0 1 2097151.875] (or the X transpose) at a NORMAL 64x97 CV_8UC1 image
//              with INTER_NEAREST+WARP_INVERSE_MAP. The FIRST undefined operation is the
//              signed int addition cvRound(...) + round_delta building Y0 (or X0), before any
//              per-column delta. bdelta[x]=0 does not make Y0 safe.
//   level-a-rowbase-euclidean
//              Level A (Task 021-R1): model-valid Euclidean theta=-0.5, ty=2097109.875 on
//              64x97. The rotated row term lets the LINEAR (round_delta 16) Y row base
//              overflow as well; first unsafe op is ROWBASE in both modes.
//   level-a-conversion-euclidean
//              Level A (Task 021-R1): model-valid Euclidean theta=-0.5, ty=2097200 on 64x97.
//              The Y row-base conversion is out of range (INT_MIN sentinel); the row-base add
//              is in range; the later per-column add with a negative bdelta overflows.
//   level-a-rowbase-control
//              Negative control: t = 2097000 on both axes, 64x97 (safe for every row, column
//              and mode); must show no overflow in the model or natively.
//   level-a-nonfinite-translation / level-a-nonfinite-euclidean
//              Level A replay of the NaN warps a nonfinite optimizer update would leave
//              (reported model-valid=NO: they are not finite model matrices). Records what
//              the native resampler would do if such an update ever occurred.
//   level-a-translation-far / level-a-euclidean-far
//              Negative controls: finite model-valid warps with |t| = 1e6 (inside the 2^20
//              seed bound, far outside the image). Must show no overflow on any version.
//   level-a-translation-extreme / level-a-euclidean-extreme
//              Level A: direct native warpAffine replay of a MODEL-VALID extreme-translation
//              matrix (translation: identity linear part; Euclidean: a rigid rotation).
//              Demonstrates a warp vulnerability for a model-valid matrix. It does NOT by
//              itself show that the ECC optimizer reaches such a matrix.
//   level-a-euclidean-bigangle
//              Level A control: a near-quarter-turn rotation with moderate translation.
//              Bounded rotation coefficients alone must NOT overflow (zero model overflows).
//   trace-translation / trace-euclidean-identity / trace-euclidean-seeded
//              Level B: print every intermediate matrix native findTransformECC passes to
//              warpAffine, using OpenCV's own optimizer with increasing iteration caps.
//   level-c-translation / level-c-euclidean-identity / level-c-euclidean-seeded
//              Level C: call the real exported C ABI (opencv_video_estimate_ecc_translation,
//              opencv_video_estimate_ecc_euclidean, opencv_video_estimate_ecc_euclidean_seeded)
//              with validated caller inputs.
//   search-translation / search-euclidean-identity:FIRST:COUNT
//   search-euclidean-seeded:FIRST:COUNT
//              Bounded, deterministic, indexed search over difficult optimizer trajectories.
//
// The int64 diagnostic never performs a signed int32 addition itself, so the probe's own
// arithmetic is defined; any UBSan report originates inside OpenCV.

#include "opencv_video_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"
#include "ecc_fixture.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <vector>

#if defined(__x86_64__) || defined(_M_X64)
#include <emmintrin.h>
#endif

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using ecc_fixture::texture;
using ecc_fixture::rigid_texture;
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

// Task 021-R2: the ONE predicate for "x86 cvRound(double) yields a valid int32 result".
// OpenCV 4.x/5.0 saturate_cast<int>(double) is cvRound, which on x86-64 GCC/MSVC is
// _mm_cvtsd_si32 (cvtsd2si) under the MXCSR rounding mode; the tested mode is
// round-to-nearest, ties-to-even (conversion-check verifies it and fails otherwise). Under
// that mode the accepted interval is
//     -2147483648.5 <= v < 2147483647.5
// The lower bound is INCLUSIVE (-2147483648.5 ties to the even -2147483648, representable);
// the upper bound is EXCLUSIVE (2147483647.5 ties to the even 2147483648, not representable).
// Nonfinite input is rejected by std::isfinite BEFORE any comparison: an ordered comparison
// with a NaN operand (comisd) raises FE_INVALID on x86, and nothing here may perform an
// invalid operation. No integer conversion is executed by this predicate.
// (R1 used std::llround for the classification. llround rounds ties AWAY from zero, so it
// misclassified -2147483648.5 as out of range, and on NaN/huge values it is an invalid
// conversion with an unspecified result. Every classifier now uses this predicate.)
enum class ConvClass { IN_RANGE, OUT_OF_RANGE, NONFINITE };

ConvClass classify_x86_cvround(double v) {
    if (!std::isfinite(v)) return ConvClass::NONFINITE;
    return (v >= -2147483648.5 && v < 2147483647.5) ? ConvClass::IN_RANGE
                                                     : ConvClass::OUT_OF_RANGE;
}

inline bool x86_cvround_in_range(double v) { return classify_x86_cvround(v) == ConvClass::IN_RANGE; }

const char *conv_class_name(ConvClass c) {
    return c == ConvClass::IN_RANGE ? "IN_RANGE" : c == ConvClass::NONFINITE ? "NONFINITE"
                                                                             : "OUT_OF_RANGE";
}

// Explicit model of x86 cvRound: for an out-of-range or nonfinite input the instruction
// returns the "integer indefinite" INT_MIN for BOTH signs (it does not clamp). The model
// never hands such a value to a real conversion; only IN_RANGE values reach
// cv::saturate_cast, i.e. the very cvRound OpenCV executes. conversion-check records the
// observed hardware result separately.
int x86_cvround_model(double v) {
    if (!x86_cvround_in_range(v)) return INT32_MIN;
    return cv::saturate_cast<int>(v);
}

// Does int64 v lie outside the signed int32 range?
inline bool oob_i32(int64_t v) { return v > INT32_MAX || v < INT32_MIN; }

// int64 model of the legacy warpAffine integer path (4.x WarpAffineInvoker; 5.0 legacy
// blockline). It classifies the THREE distinct undefined operations OpenCV can execute, in
// the order the native code performs them, and records which one happens FIRST. All
// potentially overflowing sums are computed in int64, so the diagnostic never overflows
// int32 itself.
//
// Native per row y (imgwarp.cpp, the X0/Y0 lines):
//     int X0 = saturate_cast<int>((M[1]*y + M[2])*AB_SCALE) + round_delta;   // (1) conv, (2) +round_delta
//     int Y0 = saturate_cast<int>((M[4]*y + M[5])*AB_SCALE) + round_delta;
// Native per pixel x (blockline/tail):
//     int X = (X0 + adelta[x]) >> ...; adelta[x]=saturate_cast<int>(M[0]*x*AB_SCALE);  // (1) conv, (3) +delta
//
// Conditions:
//   1. CONVERSION - saturate_cast<int>(double) is cvRound; a double rounding outside int32
//                   yields the x86 "integer indefinite" INT_MIN (defined intrinsic, wrong
//                   value), NOT a clamp. Flagged where the mathematical value is out of range.
//   2. ROWBASE    - the signed int addition `cvRound(...) + round_delta` building X0/Y0, ONCE
//                   PER ROW, BEFORE any per-column delta is applied.
//   3. PERCOLUMN  - the later signed int addition `X0 + adelta[x]` / `Y0 + bdelta[x]`.
// ROWBASE precedes PERCOLUMN in execution, so a row whose base overflows is classified as
// ROWBASE, never as a later coordinate-addition overflow.
struct FixedPointReport {
    long conversion = 0, rowbase = 0, percolumn = 0;
    long conversion_then_add_overflow = 0;  // subset of conversion: INT_MIN sentinel then overflows an add
    const char *first_kind = "NONE";   // "CONVERSION" | "ROWBASE" | "PERCOLUMN"
    char first_axis = '-';
    int first_row = -1, first_col = -1;
    // ROWBASE / PERCOLUMN: the two valid int32 operands and their int64 sum.
    int64_t first_a = 0, first_b = 0, first_sum = 0;
    // CONVERSION (R2): an invalid conversion has no integer operands. Record the original
    // double, its classification and which operand it feeds; never an integer image of it.
    const char *first_conv_operand = "";   // "X0-base" | "Y0-base" | "adelta" | "bdelta"
    double first_conv_value = 0.0;
    ConvClass first_conv_class = ConvClass::IN_RANGE;
    // FIRST_UB: the first SIGNED-INT ADDITION that overflows, in native execution order
    // (column tables; then per row: conv X0, add X0, conv Y0, add Y0; then per pixel: add X,
    // add Y), using the INT_MIN sentinel for an out-of-range conversion exactly as native
    // code would. An out-of-range conversion is NOT itself UB on x86: saturate_cast<int> is
    // the defined cvtsd2si intrinsic (wrong value), so the first UB is a later addition.
    // Ordering caveat: the scan is raster order. Native WarpAffineInvoker walks row blocks
    // of width bw (128 for the 64-row geometries here); for cols <= bw, as in every 64x97
    // case, raster order equals native order. For wider images only the KIND is asserted.
    const char *ub_kind = "NONE";      // "ROWBASE" | "PERCOLUMN"
    char ub_axis = '-';
    int ub_row = -1, ub_col = -1;
    int64_t ub_a = 0, ub_b = 0;
    // Pixels flagged for any reason (conversion out of range and/or an add overflow).
    long total() const { return conversion + rowbase + percolumn; }
    // Pixels where a SIGNED INT ADDITION overflows (row base or per column), including those
    // whose operand came from an out-of-range conversion. This is the Task 021 metric
    // (an out-of-range conversion alone is a wrong value, not UB) and is what every
    // control, window and search compares against.
    long ub_pixels() const { return rowbase + percolumn + conversion_then_add_overflow; }
};

void find_first_ub(FixedPointReport &r, const double M[6], int rows, int cols, int round_delta) {
    constexpr int AB_SCALE = 1024;
    std::vector<int> adelta(cols), bdelta(cols);
    for (int x = 0; x < cols; ++x) {
        adelta[x] = x86_cvround_model(M[0] * x * AB_SCALE);
        bdelta[x] = x86_cvround_model(M[3] * x * AB_SCALE);
    }
    auto hit = [&](const char *kind, char axis, int y, int x, int64_t a, int64_t b) {
        r.ub_kind = kind; r.ub_axis = axis; r.ub_row = y; r.ub_col = x; r.ub_a = a; r.ub_b = b;
    };
    for (int y = 0; y < rows; ++y) {
        const int cx = x86_cvround_model((M[1] * y + M[2]) * AB_SCALE);
        if (oob_i32(int64_t(cx) + round_delta)) return hit("ROWBASE", 'X', y, -1, cx, round_delta);
        const int X0 = cx + round_delta;
        const int cy = x86_cvround_model((M[4] * y + M[5]) * AB_SCALE);
        if (oob_i32(int64_t(cy) + round_delta)) return hit("ROWBASE", 'Y', y, -1, cy, round_delta);
        const int Y0 = cy + round_delta;
        for (int x = 0; x < cols; ++x) {
            if (oob_i32(int64_t(X0) + adelta[x])) return hit("PERCOLUMN", 'X', y, x, X0, adelta[x]);
            if (oob_i32(int64_t(Y0) + bdelta[x])) return hit("PERCOLUMN", 'Y', y, x, Y0, bdelta[x]);
        }
    }
}

FixedPointReport analyze_fixed_point(const double M[6], int rows, int cols, int round_delta) {
    constexpr int AB_SCALE = 1024;  // AB_BITS = max(10, INTER_BITS = 5)
    FixedPointReport r;
    auto record_first = [&](const char *kind, char axis, int y, int x, int64_t a, int64_t b) {
        if (r.first_row >= 0) return;
        r.first_kind = kind; r.first_axis = axis; r.first_row = y; r.first_col = x;
        r.first_a = a; r.first_b = b; r.first_sum = a + b;
    };
    auto record_conv = [&](char axis, int y, int x, const char *operand, double v) {
        if (r.first_row >= 0) return;
        r.first_kind = "CONVERSION"; r.first_axis = axis; r.first_row = y; r.first_col = x;
        r.first_conv_operand = operand; r.first_conv_value = v;
        r.first_conv_class = classify_x86_cvround(v);
    };
    // Same single predicate as x86_cvround_model (R2); no integer conversion is executed.
    auto conv_oor = [](double v) { return !x86_cvround_in_range(v); };
    for (int y = 0; y < rows; ++y) {
        const double x_pre = (M[1] * y + M[2]) * AB_SCALE;
        const double y_pre = (M[4] * y + M[5]) * AB_SCALE;
        const bool convx_oor = conv_oor(x_pre), convy_oor = conv_oor(y_pre);
        const int64_t X0 = int64_t(x86_cvround_model(x_pre)) + round_delta;  // native int wraps; model does not
        const int64_t Y0 = int64_t(x86_cvround_model(y_pre)) + round_delta;
        const bool rowx_ovf = oob_i32(X0), rowy_ovf = oob_i32(Y0);
        for (int x = 0; x < cols; ++x) {
            const double a_pre = M[0] * x * AB_SCALE, b_pre = M[3] * x * AB_SCALE;
            const bool convax_oor = conv_oor(a_pre), convby_oor = conv_oor(b_pre);
            const int adelta = x86_cvround_model(a_pre), bdelta = x86_cvround_model(b_pre);
            const int64_t X = X0 + adelta, Y = Y0 + bdelta;
            const bool colx_ovf = oob_i32(X), coly_ovf = oob_i32(Y);
            const bool any_conv = convx_oor || convy_oor || convax_oor || convby_oor;
            const bool any_row = rowx_ovf || rowy_ovf;
            const bool any_col = colx_ovf || coly_ovf;
            if (!any_conv && !any_row && !any_col) continue;
            if (any_conv) {
                ++r.conversion;
                // The conversion is a defined intrinsic yielding INT_MIN, but the later
                // native int additions then use that sentinel; record when they overflow
                // (e.g. INT_MIN + 16 + negative bdelta in a rotated Euclidean warp).
                if (any_row || any_col) ++r.conversion_then_add_overflow;
                if (convx_oor) record_conv('X', y, x, "X0-base", x_pre);
                else if (convy_oor) record_conv('Y', y, x, "Y0-base", y_pre);
                else if (convax_oor) record_conv('X', y, x, "adelta", a_pre);
                else record_conv('Y', y, x, "bdelta", b_pre);
            } else if (any_row) {
                ++r.rowbase;
                if (rowx_ovf) record_first("ROWBASE", 'X', y, x, int64_t(x86_cvround_model(x_pre)), round_delta);
                else record_first("ROWBASE", 'Y', y, x, int64_t(x86_cvround_model(y_pre)), round_delta);
            } else {
                ++r.percolumn;
                if (colx_ovf) record_first("PERCOLUMN", 'X', y, x, X0, adelta);
                else record_first("PERCOLUMN", 'Y', y, x, Y0, bdelta);
            }
        }
    }
    return r;
}

// Thin wrapper preserving the Task 021 interface: it returns the number of pixels where a
// signed int addition overflows (ub_pixels). The original model already DETECTED every
// row-base overflow, because column 0 has adelta = bdelta = 0, so an overflowing int64
// X0/Y0 left that pixel's final sum out of range; it only MISLABELLED the overflow as a
// per-column addition. ub_pixels is a superset of the original count: it adds only pixels
// whose row base overflowed but whose int64 sum a negative delta brought back into range.
// For any FINITE matrix the per-matrix verdict (count > 0) is identical, because the
// column-0 pixel of an overflowing row was already flagged; trace/search verdicts are per
// matrix. The verbose lines name the first flagged condition (FIRST_UNSAFE) and the first
// actual UB addition (FIRST_UB).
// R2: report the first flagged condition without fabricating integers. A CONVERSION line
// prints the original double (%.17g, which is defined for NaN/Inf/huge values and performs
// no conversion), its classification, the operand it feeds and the x86 integer-indefinite
// result the instruction produces (INT_MIN, an int constant, not derived from the double).
// ROWBASE/PERCOLUMN lines keep the R1 format: two valid int32 operands and their int64 sum.
void print_first_unsafe(const FixedPointReport &r, const char *label, int round_delta) {
    if (std::string(r.first_kind) == "CONVERSION") {
        std::printf("  %s delta=%d FIRST_UNSAFE=CONVERSION axis=%c row=%d col=%d operand=%s "
                    "pre-conversion=%.17g class=%s x86-result=%d (integer indefinite)\n",
                    label, round_delta, r.first_axis, r.first_row, r.first_col,
                    r.first_conv_operand, r.first_conv_value, conv_class_name(r.first_conv_class),
                    INT32_MIN);
        return;
    }
    std::printf("  %s delta=%d FIRST_UNSAFE=%s axis=%c row=%d col=%d operands %lld + %lld "
                "int64 sum=%lld%s\n",
                label, round_delta, r.first_kind, r.first_axis, r.first_row, r.first_col,
                (long long)r.first_a, (long long)r.first_b, (long long)r.first_sum,
                oob_i32(r.first_sum) ? " OVERFLOW" : "");
}

long fixed_point_overflows(const double M[6], int rows, int cols, int round_delta,
                           const char *label, bool verbose) {
    FixedPointReport r = analyze_fixed_point(M, rows, cols, round_delta);
    if (verbose && r.total() > 0) {
        find_first_ub(r, M, rows, cols, round_delta);
        print_first_unsafe(r, label, round_delta);
        std::printf("    counts: conversion=%ld (then add overflow=%ld) rowbase=%ld percolumn=%ld\n",
                    r.conversion, r.conversion_then_add_overflow, r.rowbase, r.percolumn);
        if (r.ub_row >= 0)
            std::printf("    FIRST_UB=%s axis=%c row=%d col=%d int32 operands %lld + %lld "
                        "int64 sum=%lld\n",
                        r.ub_kind, r.ub_axis, r.ub_row, r.ub_col, (long long)r.ub_a,
                        (long long)r.ub_b, (long long)(r.ub_a + r.ub_b));
        else
            std::printf("    FIRST_UB=NONE (out-of-range conversions only; no int add overflows)\n");
    }
    return r.ub_pixels();
}

void print_matrix(const char *prefix, const float w[6]) {
    std::printf("%s[%.17g %.17g %.17g %.17g %.17g %.17g]\n", prefix, double(w[0]), double(w[1]),
                double(w[2]), double(w[3]), double(w[4]), double(w[5]));
}

// Classify a 2x3 warp as a model-valid intermediate for each motion model, so Level A
// cannot be misread as "a general affine matrix overflows". Tolerances are generous
// Float32 bounds; the matrices below are built exactly as native ECC would store them.
bool all_finite(const float w[6]) {
    for (int i = 0; i < 6; ++i)
        if (!std::isfinite(w[i])) return false;
    return true;
}
bool is_translation_valid(const float w[6]) {
    return all_finite(w) && std::fabs(w[0] - 1.0f) <= 1e-5f && std::fabs(w[1]) <= 1e-5f &&
           std::fabs(w[3]) <= 1e-5f && std::fabs(w[4] - 1.0f) <= 1e-5f;
}
bool is_euclidean_valid(const float w[6]) {
    const double a = w[0], b = w[1], c = w[3], d = w[4];
    return all_finite(w) && std::fabs(a - d) <= 1e-5 && std::fabs(b + c) <= 1e-5 &&
           std::fabs(a * a + c * c - 1.0) <= 1e-5;
}

// Build a native-shaped Euclidean warp exactly as the shim does: cos/sin in double, each
// element cast to float once. a=d, b=-c hold bit-for-bit.
void euclidean_matrix(float m[6], double theta, float tx, float ty) {
    const double c = std::cos(theta), s = std::sin(theta);
    m[0] = float(c);
    m[1] = float(-s);
    m[2] = tx;
    m[3] = float(s);
    m[4] = float(c);
    m[5] = ty;
}

// Level A: replay one exact recorded CV_32F matrix through native warpAffine, using the
// exact native ECC call shapes (CV_32FC1 image/gradients with INTER_LINEAR, CV_8UC1 mask
// with INTER_NEAREST, both WARP_INVERSE_MAP).
long level_a(const char *name, const char *model, const float coeff[6], int rows, int cols) {
    std::printf("[level-a] %s model=%s OpenCV %s geometry %dx%d (cols x rows)\n", name, model,
                CV_VERSION, cols, rows);
    print_matrix("  stored CV_32F M=", coeff);
    const bool model_ok = (std::string(model) == "translation") ? is_translation_valid(coeff)
                          : (std::string(model) == "euclidean")  ? is_euclidean_valid(coeff)
                                                                  : true;
    std::printf("  model-valid intermediate=%s\n", model_ok ? "YES" : "NO");
    double M[6];
    for (int i = 0; i < 6; ++i) M[i] = coeff[i];
    const long linear = fixed_point_overflows(M, rows, cols, 16, name, true);
    const long nearest = fixed_point_overflows(M, rows, cols, 512, name, true);
    std::printf("  int64 model: linear overflow pixels=%ld nearest overflow pixels=%ld\n", linear,
                nearest);
    cv::Mat image, mask(rows, cols, CV_8UC1, cv::Scalar(1)), dst, map(2, 3, CV_32F);
    texture(rows, cols).convertTo(image, CV_32F);
    for (int i = 0; i < 6; ++i) map.ptr<float>()[i] = coeff[i];
    std::printf("  NATIVE_CALL_BEGIN warpAffine CV_32FC1 INTER_LINEAR+WARP_INVERSE_MAP\n");
    std::fflush(stdout);
    cv::warpAffine(image, dst, map, image.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    std::printf("  NATIVE_CALL_BEGIN warpAffine CV_8UC1 INTER_NEAREST+WARP_INVERSE_MAP\n");
    std::fflush(stdout);
    cv::warpAffine(mask, dst, map, mask.size(), cv::INTER_NEAREST + cv::WARP_INVERSE_MAP);
    std::printf("  NATIVE_CALL_END\n");
    return linear + nearest;
}

// Level B iteration provenance (identical method to Task 020's trace()): native ECC is
// deterministic and its loop body does not depend on maxCount, so a call capped at k
// iterations leaves in the caller's warp exactly the matrix iteration k+1 would pass to
// warpAffine. This uses cv::findTransformECC directly; it does not reimplement ECC. The
// capped call that PRODUCES the unsafe matrix never consumes it, so trace logs contain no
// UBSan reports by design; execution of the overflow is shown by Levels A and C.
long trace(const char *name, int motionType, const float seed[6], int rows, int cols,
           int iterations, const cv::Mat &tmpl, const cv::Mat &moving) {
    std::printf("[trace] %s OpenCV %s motion=%d geometry %dx%d up to %d iterations\n", name,
                CV_VERSION, motionType, cols, rows, iterations);
    print_matrix("  seed=", seed);
    long first_unsafe = -1;
    cv::Mat previous;
    double max_t = 0.0;
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
        double M[6];
        for (int i = 0; i < 6; ++i) M[i] = w[i];
        max_t = std::max(max_t, std::max(std::fabs(M[2]), std::fabs(M[5])));
        const long linear = fixed_point_overflows(M, rows, cols, 16, "", false);
        const long nearest = fixed_point_overflows(M, rows, cols, 512, "", false);
        std::printf("  warp entering iteration %d (%s after %d updates): ", k + 1, outcome.c_str(), k);
        print_matrix("", w);
        const bool valid = motionType == cv::MOTION_TRANSLATION ? is_translation_valid(w)
                                                                 : is_euclidean_valid(w);
        if (!valid) std::printf("    MODEL_INVARIANT_VIOLATED\n");
        if (linear + nearest > 0) {
            std::printf("    UNSAFE_INTERMEDIATE iteration=%d linear_pixels=%ld nearest_pixels=%ld\n",
                        k + 1, linear, nearest);
            fixed_point_overflows(M, rows, cols, 16, "    linear", true);
            if (first_unsafe < 0) first_unsafe = k + 1;
            break;
        }
    }
    std::printf("  first unsafe iteration=%ld max|translation| reached=%.9g\n", first_unsafe, max_t);
    return first_unsafe;
}

void level_c_translation(int rows, int cols, int iterations, const cv::Mat &tmpl,
                         const cv::Mat &moving) {
    std::printf("[level-c] translation opencv_video_estimate_ecc_translation geometry %dx%d "
                "iters=%d eps=1e-8\n", cols, rows, iterations);
    Handle th = make_handle(tmpl), mh = make_handle(moving);
    opencv_video_ecc_translation_result r = {0, 0, 0};
    std::printf("  BINDING_CALL_BEGIN\n");
    std::fflush(stdout);
    const opencv_video_status status =
        opencv_video_estimate_ecc_translation(th.get(), mh.get(), iterations, 1e-8, &r);
    std::printf("  BINDING_CALL_END status=%d message=\"%s\"\n", int(status),
                status == OPENCV_VIDEO_OK ? "" : opencv_video_last_error());
    if (status == OPENCV_VIDEO_OK)
        std::printf("  published displacement=(%.9g, %.9g) correlation=%.9g\n",
                    double(r.displacement_x), double(r.displacement_y), r.correlation);
}

void level_c_euclidean(const char *name, bool seeded, float tx, float ty, double theta, int rows,
                       int cols, int iterations, const cv::Mat &tmpl, const cv::Mat &moving) {
    std::printf("[level-c] %s %s geometry %dx%d iters=%d eps=1e-8\n", name,
                seeded ? "opencv_video_estimate_ecc_euclidean_seeded"
                       : "opencv_video_estimate_ecc_euclidean",
                cols, rows, iterations);
    if (seeded)
        std::printf("  validated caller seed tx=%.9g ty=%.9g theta=%.9g\n", double(tx), double(ty),
                    theta);
    Handle th = make_handle(tmpl), mh = make_handle(moving);
    opencv_video_ecc_euclidean_result r = {0, 0, 0, 0};
    std::printf("  BINDING_CALL_BEGIN\n");
    std::fflush(stdout);
    const opencv_video_status status =
        seeded ? opencv_video_estimate_ecc_euclidean_seeded(th.get(), mh.get(), iterations, 1e-8,
                                                            tx, ty, theta, &r)
               : opencv_video_estimate_ecc_euclidean(th.get(), mh.get(), iterations, 1e-8, &r);
    std::printf("  BINDING_CALL_END status=%d message=\"%s\"\n", int(status),
                status == OPENCV_VIDEO_OK ? "" : opencv_video_last_error());
    if (status == OPENCV_VIDEO_OK)
        std::printf("  published displacement=(%.9g, %.9g) angle=%.9g correlation=%.9g\n",
                    double(r.displacement_x), double(r.displacement_y), r.rotation_radians,
                    r.correlation);
}

// Deterministic campaign fixture, keyed by trial index (so a trial index fully identifies
// its inputs). Twelve image kinds exercise difficult optimizer trajectories within the
// binding's published envelope (32..16384 per side, <= 2^24 pixels).
struct Trial {
    int rows, cols, kind;
    cv::Mat tmpl, moving;
    float seed[6];
};

Trial campaign_trial(uint32_t index, int flavor) {
    // flavor: 0 translation, 1 euclidean identity, 2 euclidean seeded.
    uint32_t state = 21u * 2654435761u + index * 40503u + uint32_t(flavor) * 7u;
    auto rnd = [&]() {
        state = state * 1664525u + 1013904223u;
        return double(state) / 4294967296.0;
    };
    Trial t;
    t.rows = 32 + int(rnd() * 97);   // 32 .. 128
    t.cols = 32 + int(rnd() * 129);  // 32 .. 160
    t.kind = int(index % 12);
    const double deg = 30.0 * (rnd() - 0.5);  // -15 .. 15 degrees
    const double tx = 40.0 * (rnd() - 0.5), ty = 40.0 * (rnd() - 0.5);
    t.tmpl = texture(t.rows, t.cols);
    switch (t.kind) {
        case 0: t.moving = texture(t.rows, t.cols, 2, 1); break;              // moderate translation
        case 1: t.moving = texture(t.rows, t.cols, int(10 * rnd()), int(10 * rnd())); break; // near-limit
        case 2: t.moving = rigid_texture(t.rows, t.cols, deg, tx, ty); break; // rotation+translation
        case 3: t.moving = rigid_texture(t.rows, t.cols, 3.0 * (deg < 0 ? -1 : 1), 0, 0); break; // rotation about origin
        case 4: t.moving = low_variance(texture(t.rows, t.cols, 2, 1)); break; // low gradient
        case 5: t.moving = constant(t.rows, t.cols, 128); t.tmpl = constant(t.rows, t.cols, 128); break; // nearly constant
        case 6: t.moving = stripes(t.rows, t.cols, 3); t.tmpl = stripes(t.rows, t.cols, 0); break; // anisotropic / periodic
        case 7: t.moving = noise(t.rows, t.cols, index + 1); break;            // unrelated noise
        case 8: t.moving = texture(t.rows, t.cols, 1, 0);                      // weakly conditioned
                 t.tmpl = low_variance(t.tmpl); break;
        case 9: {                                                              // high-contrast bands
            cv::Mat m(t.rows, t.cols, CV_8UC1, cv::Scalar(0));
            for (int c = 4; c < t.cols; c += 12)
                m.colRange(c, std::min(c + 2, t.cols)).setTo(255);
            t.tmpl = m.clone();
            cv::Mat shifted(t.rows, t.cols, CV_8UC1, cv::Scalar(0));
            for (int c = 7; c < t.cols; c += 12)
                shifted.colRange(c, std::min(c + 2, t.cols)).setTo(255);
            t.moving = shifted;
            break;
        }
        case 10: {                                                             // asymmetric from parent
            cv::Mat parent = texture(t.rows + 16, t.cols + 24, 2, 1);
            t.moving = parent(cv::Rect(12, 8, t.cols, t.rows)).clone();
            break;
        }
        default: t.moving = rigid_texture(t.rows, t.cols, deg, tx, ty); break;
    }
    const float identity[6] = {1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 6; ++i) t.seed[i] = identity[i];
    if (flavor == 2) {
        // In-domain Euclidean seed (|t| <= 2^20, |angle| <= 1 rad), category by index so
        // every required seed class is exercised: zero, near-correct, wrong-but-accepted,
        // angle near +-1 rad, displacement near the 2^20 limit, opposing rotation/translation.
        const double true_theta = deg * 3.14159265358979323846 / 180.0;
        const double u = rnd(), v = rnd(), w = rnd();
        double a = 0, sx = 0, sy = 0;
        switch ((index / 12) % 6) {
            case 0: break;                                                  // zero seed
            case 1: a = true_theta + 0.01 * (u - 0.5); sx = tx + (v - 0.5); // near-correct
                    sy = ty + (w - 0.5); break;
            case 2: a = -true_theta; sx = -tx + 30 * (v - 0.5);             // wrong but accepted
                    sy = -ty + 30 * (w - 0.5); break;
            case 3: a = (u < 0.5 ? -1.0 : 1.0) * (1.0 - 1e-3 * v);          // angle near +-1 rad
                    sx = tx; sy = ty; break;
            case 4: a = 0.5 * (u - 0.5);                                    // displacement near 2^20
                    sx = (v < 0.5 ? -1.0 : 1.0) * (1048576.0 - 64.0 * w);
                    sy = (w < 0.5 ? -1.0 : 1.0) * (1048576.0 - 64.0 * v); break;
            default: a = (true_theta >= 0 ? -1.0 : 1.0) * (0.5 + 0.5 * u);  // opposing rot/trans
                     sx = 200.0 * (v - 0.5); sy = 200.0 * (w - 0.5); break;
        }
        euclidean_matrix(t.seed, a, float(sx), float(sy));
    }
    return t;
}

int first_unsafe_iteration(const Trial &t, int motionType, int iterations, float out_warp[6],
                           double &max_translation) {
    cv::Mat previous;
    max_translation = 0.0;
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
        double M[6];
        for (int i = 0; i < 6; ++i) M[i] = warp.ptr<float>()[i];
        max_translation = std::max(max_translation, std::max(std::fabs(M[2]), std::fabs(M[5])));
        if (fixed_point_overflows(M, t.rows, t.cols, 16, "", false) +
                fixed_point_overflows(M, t.rows, t.cols, 512, "", false) > 0) {
            for (int i = 0; i < 6; ++i) out_warp[i] = warp.ptr<float>()[i];
            return k + 1;
        }
    }
    return -1;
}

void search(const char *name, int flavor, int motionType, uint32_t first, uint32_t count) {
    std::printf("[search] %s trials %u..%u OpenCV %s (100 iterations, eps 1e-8, no mask)\n", name,
                first, first + count - 1, CV_VERSION);
    long unsafe = 0;
    double global_max_translation = 0.0;
    uint32_t worst_trial = first;
    for (uint32_t n = first; n < first + count; ++n) {
        const Trial t = campaign_trial(n, flavor);
        float w[6] = {0, 0, 0, 0, 0, 0};
        double max_t = 0.0;
        const int it = first_unsafe_iteration(t, motionType, 100, w, max_t);
        if (max_t > global_max_translation) {
            global_max_translation = max_t;
            worst_trial = n;
        }
        if (it < 0) continue;
        ++unsafe;
        std::printf("  SEARCH_UNSAFE trial=%u kind=%d geometry=%dx%d iteration=%d ", n, t.kind,
                    t.cols, t.rows, it);
        print_matrix("unsafe warp=", w);
    }
    std::printf("  SEARCH_SUMMARY %s trials=%u unsafe=%ld max|translation| reached=%.9g (trial %u)\n",
                name, count, unsafe, global_max_translation, worst_trial);
}

// Peak resident set (Linux VmHWM), so each campaign can be checked against its budget.
void print_peak_rss() {
    std::FILE *f = std::fopen("/proc/self/status", "r");
    if (f == nullptr) return;
    char line[256];
    while (std::fgets(line, sizeof line, f) != nullptr)
        if (std::string(line).rfind("VmHWM:", 0) == 0) std::printf("  PEAK_RSS %s", line + 6);
    std::fclose(f);
}

// conversion-check: (1) observe the target's cvtsd2si result for boundary doubles through
// the _mm_cvtsd_si32 intrinsic (a defined hardware operation, not a C++ float-to-int cast)
// and compare it with x86_cvround_model; (2) evaluate, with the model only (no native call),
// the identity-linear translation overflow window at the widest permitted geometry.
int conversion_check() {
    int mismatches = 0;
    // R2: the boundary comparison is meaningful only under round-to-nearest (ties-to-even).
    // Report the active mode explicitly and fail rather than rely on the wrong boundary.
    const int mode = std::fegetround();
    const char *mode_name = mode == FE_TONEAREST ? "FE_TONEAREST" : mode == FE_UPWARD ? "FE_UPWARD"
                          : mode == FE_DOWNWARD ? "FE_DOWNWARD" : mode == FE_TOWARDZERO ? "FE_TOWARDZERO"
                                                                                         : "UNKNOWN";
    std::printf("  ROUNDING_MODE %s%s\n", mode_name,
                mode == FE_TONEAREST ? "" : " (NOT round-to-nearest: boundary model does not apply)");
    if (mode != FE_TONEAREST) ++mismatches;
#if defined(__x86_64__) || defined(_M_X64)
    const unsigned mxcsr_rc = (_mm_getcsr() >> 13) & 3u;   // 0 = round to nearest (even)
    std::printf("  MXCSR_RC %u%s\n", mxcsr_rc, mxcsr_rc == 0 ? " (round-to-nearest-even)" : " (UNEXPECTED)");
    if (mxcsr_rc != 0) ++mismatches;
#endif
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // Each probe carries the INDEPENDENT expected classification (from the IEEE argument in
    // the predicate's comment), the expected rounded value for in-range probes, and is
    // checked three ways: classifier vs expectation, model vs expectation, and (on x86_64)
    // the real cvtsd2si result vs the model.
    struct Probe { double v; ConvClass expect; int expect_value; };
    const Probe probes[] = {
        {-2147483648.5, ConvClass::IN_RANGE, INT32_MIN},       // tie -> even -2147483648
        {-2147483648.49, ConvClass::IN_RANGE, INT32_MIN},
        {-2147483648.0, ConvClass::IN_RANGE, INT32_MIN},
        {-2147483648.51, ConvClass::OUT_OF_RANGE, INT32_MIN},
        {-2147483649.0, ConvClass::OUT_OF_RANGE, INT32_MIN},
        {2147483647.0, ConvClass::IN_RANGE, INT32_MAX},
        {2147483647.49, ConvClass::IN_RANGE, INT32_MAX},
        {2147483647.5, ConvClass::OUT_OF_RANGE, INT32_MIN},    // tie -> even 2^31, unrepresentable
        {2147483648.0, ConvClass::OUT_OF_RANGE, INT32_MIN},
        {0.5, ConvClass::IN_RANGE, 0},                         // ties-to-even, not away
        {1.5, ConvClass::IN_RANGE, 2},
        {2.5, ConvClass::IN_RANGE, 2},
        {-2.5, ConvClass::IN_RANGE, -2},
        {2147483646.5, ConvClass::IN_RANGE, 2147483646},
        {-2147483647.5, ConvClass::IN_RANGE, -2147483648},
        {0.0, ConvClass::IN_RANGE, 0},
        {3e9, ConvClass::OUT_OF_RANGE, INT32_MIN},
        {-3e9, ConvClass::OUT_OF_RANGE, INT32_MIN},
        {1e300, ConvClass::OUT_OF_RANGE, INT32_MIN},
        {-1e300, ConvClass::OUT_OF_RANGE, INT32_MIN},
        {inf, ConvClass::NONFINITE, INT32_MIN},
        {-inf, ConvClass::NONFINITE, INT32_MIN},
        {nan, ConvClass::NONFINITE, INT32_MIN},
    };
    for (const Probe &p : probes) {
        const ConvClass c = classify_x86_cvround(p.v);
        const int model = x86_cvround_model(p.v);
        const bool class_ok = c == p.expect, model_ok = model == p.expect_value;
#if defined(__x86_64__) || defined(_M_X64)
        const int hw = _mm_cvtsd_si32(_mm_set_sd(p.v));   // defined intrinsic; INT_MIN if invalid
        const bool hw_ok = hw == model;
        std::printf("  CONVERSION v=%.17g class=%s expect=%s cvtsd2si=%d model=%d expect_value=%d %s\n",
                    p.v, conv_class_name(c), conv_class_name(p.expect), hw, model, p.expect_value,
                    class_ok && model_ok && hw_ok ? "MATCH" : "MISMATCH");
#else
        const bool hw_ok = true;
        std::printf("  CONVERSION v=%.17g class=%s expect=%s model=%d expect_value=%d %s "
                    "(non-x86: hardware not compared)\n",
                    p.v, conv_class_name(c), conv_class_name(p.expect), model, p.expect_value,
                    class_ok && model_ok ? "MATCH" : "MISMATCH");
#endif
        if (!class_ok || !model_ok || !hw_ok) ++mismatches;
    }
    // Diagnostic-reporting check on invalid inputs: drive the CONVERSION branch of
    // analyze_fixed_point/print_first_unsafe with NaN, +/-Inf and +/-1e300 row bases and
    // require that no floating-point exception (in particular FE_INVALID, which any invalid
    // float-to-integer conversion such as llround or a cast would raise) is raised while
    // classifying and logging. Nonfinite and huge values reach only isfinite, ordered
    // comparisons with finite operands, and printf("%.17g").
    // Sensitivity control for the FE_INVALID check: a conversion of NaN by the (defined)
    // cvtsd2si intrinsic must raise FE_INVALID, otherwise "no FE_INVALID" below proves
    // nothing (e.g. if exception flags were not observable on this target/compiler).
#if defined(__x86_64__) || defined(_M_X64)
    {
        volatile double vnan = nan;
        std::feclearexcept(FE_ALL_EXCEPT);
        volatile int sink = _mm_cvtsd_si32(_mm_set_sd(vnan));
        (void)sink;
        const bool sensitive = std::fetestexcept(FE_INVALID) != 0;
        std::printf("  DIAG_SENSITIVITY cvtsd2si(NaN) FE_INVALID=%d %s\n", int(sensitive),
                    sensitive ? "OK" : "FAILED (flag check cannot detect invalid conversions)");
        if (!sensitive) ++mismatches;
    }
#endif
    const double invalid[] = {nan, inf, -inf, 1e300, -1e300};
    for (double v : invalid) {
        const double M[6] = {1, 0, v, 0, 1, 0};
        std::feclearexcept(FE_ALL_EXCEPT);
        FixedPointReport r = analyze_fixed_point(M, 1, 2, 512);
        find_first_ub(r, M, 1, 2, 512);
        print_first_unsafe(r, "    DIAG", 512);
        const bool invalid_raised = std::fetestexcept(FE_INVALID) != 0;
        const bool kind_ok = std::string(r.first_kind) == "CONVERSION" &&
                             std::string(r.first_conv_operand) == "X0-base";
        std::printf("  DIAG_REPORT v=%.17g FE_INVALID=%d first_kind=%s FIRST_UB=%s %s\n", v,
                    int(invalid_raised), r.first_kind, r.ub_kind,
                    !invalid_raised && kind_ok ? "OK" : "FAILED");
        if (invalid_raised || !kind_ok) ++mismatches;
    }
    std::feclearexcept(FE_ALL_EXCEPT);
    // Predicted window for 16383 columns: tx in (~2080770, 2097152).
    const double candidates[] = {1048576.0, 2.0e6,       2.0807e6,    2.0808e6,
                                 2.09e6,    2.0971515e6, 2.0971525e6, 3.0e6};
    for (double t : candidates) {
        const double M[6] = {1, 0, double(float(t)), 0, 1, 0};
        std::printf("  TRANSLATION_WINDOW tx=%.9g (float %.9g) cols=16383 rows=64 "
                    "linear=%ld nearest=%ld\n", t, M[2],
                    fixed_point_overflows(M, 64, 16383, 16, "", false),
                    fixed_point_overflows(M, 64, 16383, 512, "", false));
    }
    // Row-base window (Task 021-R1): the smallest representable float32 translation whose
    // cvRound(t*1024) is still in int32 but cvRound(t*1024) + round_delta overflows int32.
    // The float32 step at 2^21 is 0.125 (scaled step 128), so only nearest (round_delta 512)
    // has representable t in the window; linear (16) has none. Reported with the per-row
    // 64x97 int64 model so the FIRST_UNSAFE classification is ROWBASE.
    const int round_deltas[2] = {512, 16};
    for (int rd : round_deltas) {
        int n = 0; double lo = 0, hi = 0;
        for (float t = 2097140.0f; t < 2097153.0f; t = std::nextafter(t, 3.0e6f)) {
            const double pre = double(t) * 1024.0;
            if (!x86_cvround_in_range(pre)) continue;   // R2: shared predicate, no llround
            if (int64_t(x86_cvround_model(pre)) + rd > INT32_MAX) { if (!n) lo = t; hi = t; ++n; }
        }
        std::printf("  ROWBASE_WINDOW round_delta=%d count=%d first=%.6f last=%.6f\n", rd, n, lo, hi);
    }
    // Euclidean window (Task 021-R1), model only, 64x97, theta = -0.5 (s < 0, so bdelta <= 0):
    // unlike translation, the window is NOT bounded above 2^21. Beyond it cvRound returns
    // the INT_MIN sentinel and INT_MIN + round_delta + (negative bdelta) overflows.
    const float euclid_ty[] = {1048576.0f, 2000000.0f, 2097000.0f, 2097109.875f,
                               2097151.875f, 2097200.0f, 3.0e6f, 1.0e9f};
    for (float ty : euclid_ty) {
        float w[6];
        euclidean_matrix(w, -0.5, 0.0f, ty);
        double M[6];
        for (int i = 0; i < 6; ++i) M[i] = w[i];
        std::printf("  EUCLIDEAN_WINDOW theta=-0.5 ty=%.9g cols=97 rows=64 linear=%ld nearest=%ld\n",
                    double(ty), fixed_point_overflows(M, 64, 97, 16, "", false),
                    fixed_point_overflows(M, 64, 97, 512, "", false));
    }
    {
        const double My[6] = {1, 0, 0, 0, 1, double(2097151.875f)};
        const double Mx[6] = {1, 0, double(2097151.875f), 0, 1, 0};
        std::printf("  ROWBASE_MODEL y-translation 64x97 nearest:\n");
        fixed_point_overflows(My, 64, 97, 512, "    Y", true);
        std::printf("  ROWBASE_MODEL x-translation 64x97 nearest:\n");
        fixed_point_overflows(Mx, 64, 97, 512, "    X", true);
    }
    // Translation Y row-base window over rows: Y0 = cvRound((y + ty)*1024) + 512 overflows
    // iff y + ty lies in {2097151.5, .625, .75, .875} for some row y. So ty can sit up to
    // rows-1 pixels below that set. Exact float32 samples, 64x97, nearest, with neighbours.
    const float ywin[] = {2097088.375f, 2097088.5f, 2097100.375f, 2097100.5f, 2097100.875f,
                          2097101.0f, 2097151.375f, 2097151.5f};
    for (float ty : ywin) {
        const double M[6] = {1, 0, 0, 0, 1, double(ty)};
        std::printf("  ROWBASE_Y_WINDOW ty=%.6f rows=64 cols=97 linear=%ld nearest=%ld\n",
                    double(ty), fixed_point_overflows(M, 64, 97, 16, "", false),
                    fixed_point_overflows(M, 64, 97, 512, "", false));
    }
    std::printf("  CONVERSION_SUMMARY mismatches=%d\n", mismatches);
    return mismatches == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char **argv) {
    cv::setNumThreads(1);
    const std::string mode = argc > 1 ? argv[1] : "negative";
    if (argc > 2 && std::string(argv[argc - 1]) == "scalar") cv::setUseOptimized(false);
    std::printf("ecc_translation_euclidean_overflow_probe OpenCV %s compiler %s mode=%s\n",
                CV_VERSION, __VERSION__, mode.c_str());
    std::printf("cv::checkHardwareSupport AVX2=%d SSE4_1=%d; useOptimized=%d\n",
                int(cv::checkHardwareSupport(CV_CPU_AVX2)),
                int(cv::checkHardwareSupport(CV_CPU_SSE4_1)), int(cv::useOptimized()));

    // Extreme but MODEL-VALID intermediates. The translation sits just under the 2^21
    // saturation point (2097152): saturate(t*1024) is a large in-range int, so adding the
    // per-column delta overflows int32. 2090000 is exact in Float32.
    // With 64 rows WarpAffineInvoker uses 32x128 blocks (BLOCK_SZ 64). 16383 columns leaves
    // a final 127-wide block whose last columns (127 mod 16 = 15 nearest, 127 mod 8 = 7
    // linear) are evaluated by the scalar per-pixel tail, where UBSan can see the int32
    // add. A multiple of 16 would hide the overflow in the silently wrapping SIMD body.
    const int rows = 64, wide = 16383;  // widest column reaches the largest adelta/bdelta
    const float translation_extreme[6] = {1, 0, 2090000.0f, 0, 1, 0};
    float euclidean_extreme[6];
    euclidean_matrix(euclidean_extreme, 0.5, 0.0f, 2090000.0f);  // big ty, modest rotation
    float euclidean_bigangle[6];
    euclidean_matrix(euclidean_bigangle, 1.5533, 3.0f, -2.0f);   // ~89 deg, moderate translation

    const int geo_rows = 64, geo_cols = 97;  // ordinary ECC geometry for B/C
    const cv::Mat tmpl = texture(geo_rows, geo_cols);
    const cv::Mat moving = texture(geo_rows, geo_cols, 2, 1);
    const cv::Mat rigid_moving = rigid_texture(geo_rows, geo_cols, 6.0, 2.0, 1.0);
    const float identity[6] = {1, 0, 0, 0, 1, 0};
    float extreme_seed[6];
    euclidean_matrix(extreme_seed, 0.9, 1048576.0f, -1048576.0f);  // in-domain extreme seed

    if (mode == "negative") {
        const float mod_translation[6] = {1, 0, 2.0f, 0, 1, 1.0f};
        float mod_euclidean[6];
        euclidean_matrix(mod_euclidean, 0.05, 2.0f, 1.0f);
        std::printf("[negative] moderate warps; the int64 model must report zero overflows\n");
        const long nt = level_a("moderate-translation", "translation", mod_translation, rows, 256);
        const long ne = level_a("moderate-euclidean", "euclidean", mod_euclidean, rows, 256);
        if (nt != 0 || ne != 0) {
            std::printf("NEGATIVE_CONTROL_FAILED translation=%ld euclidean=%ld\n", nt, ne);
            return 1;
        }
        std::printf("NEGATIVE_CONTROL_OK\n");
    } else if (mode == "conversion-check") {
        return conversion_check();
    } else if (mode == "level-a-translation-far") {
        // Finite, model-valid translation far outside the image but representable in the
        // 5.0 float coordinate and in int32 (|t| = 1e6, inside even the 2^20 seed bound):
        // the 5.0 scalar tail forms src + srcstep*iy + ix before its bounds test.
        const float far[6] = {1, 0, 1.0e6f, 0, 1, 1.0e6f};
        level_a("translation-far", "translation", far, rows, wide);
    } else if (mode == "level-a-euclidean-far") {
        float far[6];
        euclidean_matrix(far, 0.5, -1.0e6f, 1.0e6f);
        level_a("euclidean-far", "euclidean", far, rows, 97);
    } else if (mode == "level-a-nonfinite-translation") {
        // What a NaN translation update would leave: identity linear part, NaN tx/ty.
        // Not a finite model-valid matrix; it records the native behavior if one occurred.
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float w[6] = {1, 0, nan, 0, 1, nan};
        level_a("nonfinite-translation", "translation", w, rows, wide);
    } else if (mode == "level-a-nonfinite-euclidean") {
        // What a NaN angle update would leave: cos/sin(NaN) make every coefficient NaN.
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float w[6] = {nan, nan, nan, nan, nan, nan};
        level_a("nonfinite-euclidean", "euclidean", w, rows, 97);
    } else if (mode == "level-a-rowbase-y" || mode == "level-a-rowbase-x") {
        // Task 021-R1: the row-base construction X0/Y0 = cvRound(...) + round_delta is a
        // signed int addition performed ONCE PER ROW, BEFORE any per-column delta. With a
        // translation of 2097151.875 (exact in float32), cvRound((M*y + t)*1024) at y=0 is
        // 2147483520, and +512 (nearest round_delta) overflows int32 at a NORMAL 64x97 image.
        // bdelta[x]=0 does NOT make Y0 safe; Y0 itself is the unsafe operand. INTER_NEAREST so
        // round_delta is 512; INTER_LINEAR's 16 overflows too but nearest is the recorded case.
        const bool yaxis = mode == "level-a-rowbase-y";
        float w[6] = {1, 0, 0, 0, 1, 0};
        w[yaxis ? 5 : 2] = 2097151.875f;
        level_a(yaxis ? "rowbase-y" : "rowbase-x", "translation", w, 64, 97);
    } else if (mode == "level-a-rowbase-euclidean") {
        // Task 021-R1: a MODEL-VALID Euclidean warp (theta = -0.5 rad, tx = 0,
        // ty = 2097109.875, exact in float32) on 64x97. The rotated row term c*y*1024 is not a
        // multiple of 128, so the Y row base can land in [INT32_MAX-15, INT32_MAX] and the
        // LINEAR round_delta (16) overflows too, which translation can never do. bdelta[x] =
        // round(s*x*1024) <= 0 here, so no per-column delta is needed: the first undefined
        // operation is Y0 = cvRound(...) + round_delta (row 48 linear). UBSan reports a source
        // location once per process, so the shared X0/Y0 line is reported by the first
        // (linear) call; the int64 model shows nearest is also ROWBASE.
        float w[6];
        euclidean_matrix(w, -0.5, 0.0f, 2097109.875f);
        level_a("rowbase-euclidean", "euclidean", w, 64, 97);
    } else if (mode == "level-a-conversion-euclidean") {
        // Task 021-R1: model-valid Euclidean theta=-0.5, ty=2097200 (exact in float32) on
        // 64x97. Every Y row base is beyond int32, so cvRound returns the x86 INT_MIN
        // sentinel (condition 1, a defined intrinsic but a wrong value) and the row-base add
        // INT_MIN + round_delta is in range. bdelta[x] = round(s*x*1024) < 0 for x >= 1, so
        // the per-column add Y0 + bdelta then overflows below INT32_MIN. No row base
        // overflows first, isolating the conversion-then-add route. Translation cannot take
        // this route because its deltas are never negative.
        float w[6];
        euclidean_matrix(w, -0.5, 0.0f, 2097200.0f);
        level_a("conversion-euclidean", "euclidean", w, 64, 97);
    } else if (mode == "level-a-rowbase-control") {
        // Negative control: a large translation (2097000, exact in float32) that stays
        // safe across the WHOLE 64x97 image on both axes. For the Y axis the worst row is
        // y=63: (63 + 2097000)*1024 = 2147392512, + 512 = 2147393024 <= INT32_MAX. For the
        // X axis the worst column is x=96: cvRound(2097000*1024)=2147328000, + 512 +
        // 96*1024 = 2147426816 <= INT32_MAX. So no conversion, row-base or per-column
        // overflow occurs. The int64 model must report zero and native must report nothing.
        // (The exact row-base boundary, 2097151.5, is shown analytically by conversion-check
        // ROWBASE_WINDOW; it cannot be a clean native control on 64x97 because the Y row
        // index and the X per-column deltas push a boundary value over.)
        const float wy[6] = {1, 0, 0, 0, 1, 2097000.0f};
        const float wx[6] = {1, 0, 2097000.0f, 0, 1, 0};
        const long ny = level_a("rowbase-control-y", "translation", wy, 64, 97);
        const long nx = level_a("rowbase-control-x", "translation", wx, 64, 97);
        if (ny != 0 || nx != 0) {
            std::printf("NEGATIVE_CONTROL_FAILED rowbase y=%ld x=%ld\n", ny, nx);
            return 1;
        }
        std::printf("NEGATIVE_CONTROL_OK rowbase\n");
    } else if (mode == "level-a-translation-extreme") {
        level_a("translation-extreme", "translation", translation_extreme, rows, wide);
    } else if (mode == "level-a-euclidean-extreme") {
        level_a("euclidean-extreme", "euclidean", euclidean_extreme, rows, wide);
    } else if (mode == "level-a-euclidean-bigangle") {
        const long n = level_a("euclidean-bigangle", "euclidean", euclidean_bigangle, rows, wide);
        std::printf("  %s\n", n == 0 ? "ROTATION_ONLY_NO_OVERFLOW (bounded coefficients)"
                                     : "UNEXPECTED_ROTATION_OVERFLOW");
        if (n != 0) return 1;
    } else if (mode == "trace-translation") {
        trace("translation", cv::MOTION_TRANSLATION, identity, geo_rows, geo_cols, 100, tmpl, moving);
    } else if (mode == "trace-euclidean-identity") {
        trace("euclidean-identity", cv::MOTION_EUCLIDEAN, identity, geo_rows, geo_cols, 100, tmpl,
              rigid_moving);
    } else if (mode == "trace-euclidean-seeded") {
        trace("euclidean-seeded", cv::MOTION_EUCLIDEAN, extreme_seed, geo_rows, geo_cols, 100, tmpl,
              rigid_moving);
    } else if (mode == "level-c-translation") {
        level_c_translation(geo_rows, geo_cols, 100, tmpl, moving);
    } else if (mode == "level-c-euclidean-identity") {
        level_c_euclidean("euclidean-identity", false, 0, 0, 0, geo_rows, geo_cols, 100, tmpl,
                          rigid_moving);
    } else if (mode == "level-c-euclidean-seeded") {
        level_c_euclidean("euclidean-seeded", true, extreme_seed[2], extreme_seed[5], 0.9,
                          geo_rows, geo_cols, 100, tmpl, rigid_moving);
    } else if (mode == "search-translation" || mode == "search-euclidean-identity" ||
               mode == "search-euclidean-seeded") {
        const uint32_t first = argc > 2 ? uint32_t(std::strtoul(argv[2], nullptr, 10)) : 0;
        const uint32_t count = argc > 3 ? uint32_t(std::strtoul(argv[3], nullptr, 10)) : 50;
        const int flavor = mode == "search-translation" ? 0
                         : mode == "search-euclidean-identity" ? 1 : 2;
        const int motion = flavor == 0 ? cv::MOTION_TRANSLATION : cv::MOTION_EUCLIDEAN;
        search(mode.c_str(), flavor, motion, first, count);
        print_peak_rss();
    } else {
        std::fprintf(stderr, "unknown case %s\n", mode.c_str());
        return 2;
    }
    return 0;
}
