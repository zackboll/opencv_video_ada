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
// vcvt(t)ps2dq, which return the integer-indefinite INT_MIN for any |v| >= 2^31 and for
// NaN/Inf. float32 spacing at 2^31 is 128 and both +-2^31 are representable, so the
// in-range test is exactly -2^31 <= v < 2^31 for floor and round-to-nearest-even alike.
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
};

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
    long first_unsafe = -1;
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
        if (reach.worst != Conv::IN_RANGE) {
            std::printf("    UNSAFE_INTERMEDIATE iteration=%d\n", k + 1);
            print_reach("   ", reach);
            first_unsafe = k + 1;
            break;
        }
    }
    std::printf("  first unsafe iteration=%ld max|finite_coord| reached=%.9g (2^31=%.9g)\n",
                first_unsafe, max_reach, double(kTwoPow31));
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

