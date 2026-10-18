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
//              intrinsic, checks the diagnostic's explicit conversion model against it, and
//              maps the translation overflow window at the widest permitted geometry.
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

#if defined(__x86_64__) || defined(_M_X64)
#include <emmintrin.h>
#endif

#include <algorithm>
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

// Explicit model of OpenCV 4.x saturate_cast<int>(double) == cvRound on x86-64, which is
// _mm_cvtsd_si32 (cvtsd2si, round-to-nearest-even). For NaN or a value that rounds outside
// int32 the instruction returns the "integer indefinite" value INT_MIN for BOTH signs; it
// does not clamp. The model never hands an out-of-range double to the real conversion: it
// range-checks first and calls cv::saturate_cast only for values that are representable.
// The conversion-check mode records the observed target behavior separately.
int x86_cvround_model(double v) {
    if (!(v >= -2147483648.5 && v < 2147483647.5)) return INT32_MIN;
    return cv::saturate_cast<int>(v);
}

// int64 model of the legacy warpAffine integer path (4.x WarpAffineInvoker; 5.0 legacy
// blockline). Returns the number of destination pixels whose X0+adelta or Y0+bdelta lies
// outside int32 for the given rounding delta (16 linear, 512 nearest). The final add is
// performed in int64, so the DIAGNOSTIC never overflows int32.
long fixed_point_overflows(const double M[6], int rows, int cols, int round_delta,
                           const char *label, bool verbose) {
    constexpr int AB_SCALE = 1024;  // AB_BITS = max(10, INTER_BITS = 5)
    long overflows = 0;
    for (int y = 0; y < rows; ++y) {
        const double x_pre = (M[1] * y + M[2]) * AB_SCALE;
        const double y_pre = (M[4] * y + M[5]) * AB_SCALE;
        const int64_t X0 = int64_t(x86_cvround_model(x_pre)) + round_delta;
        const int64_t Y0 = int64_t(x86_cvround_model(y_pre)) + round_delta;
        for (int x = 0; x < cols; ++x) {
            const int adelta = x86_cvround_model(M[0] * x * AB_SCALE);
            const int bdelta = x86_cvround_model(M[3] * x * AB_SCALE);
            const int64_t X = X0 + adelta, Y = Y0 + bdelta;
            const bool x_bad = X > INT32_MAX || X < INT32_MIN;
            const bool y_bad = Y > INT32_MAX || Y < INT32_MIN;
            if (!x_bad && !y_bad) continue;
            if (verbose && overflows == 0)
                std::printf("  %s first overflow (delta %d) row=%d col=%d\n"
                            "    pre-conversion x=%.17g y=%.17g\n"
                            "    X0=%lld adelta=%d int64 sum=%lld%s\n"
                            "    Y0=%lld bdelta=%d int64 sum=%lld%s\n",
                            label, round_delta, y, x, x_pre, y_pre,
                            (long long)X0, adelta, (long long)X, x_bad ? " OVERFLOW" : "",
                            (long long)Y0, bdelta, (long long)Y, y_bad ? " OVERFLOW" : "");
            ++overflows;
        }
    }
    return overflows;
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
#if defined(__x86_64__) || defined(_M_X64)
    const double probes[] = {0.0, 2147483647.0, 2147483647.49, 2147483647.5, 2147483648.0,
                             -2147483648.0, -2147483648.5, -2147483649.0, 3e9, -3e9, 1e300,
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()};
    for (double v : probes) {
        const int hw = _mm_cvtsd_si32(_mm_set_sd(v));
        const int model = x86_cvround_model(v);
        std::printf("  CONVERSION v=%.17g cvtsd2si=%d model=%d %s\n", v, hw, model,
                    hw == model ? "MATCH" : "MISMATCH");
        if (hw != model) ++mismatches;
    }
#else
    std::printf("  CONVERSION non-x86 target: model not validated\n");
#endif
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
