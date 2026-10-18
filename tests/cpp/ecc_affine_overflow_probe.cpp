// Task 020 opt-in research reproducer for the existing ECC affine signed-overflow risk.
//
// THIS IS NOT A SANITIZER-CLEAN TEST. Against an UBSan-instrumented OpenCV 4.1/4.6/4.10
// build it is EXPECTED to print signed-integer-overflow reports from inside warpAffine.
// Those reports are the evidence, not a probe failure. Never run it in routine hosted
// CI; build it only through scripts/research_ecc_affine_overflow.sh against a custom
// instrumented OpenCV install (docs/tasks/020-existing-affine-safety-audit.md).
//
// Cases (exactly one per process, because UBSan reports a source location only once):
//   level-a-matrix-a / level-a-matrix-b
//              Level A: direct native replay of the exact recorded PR #19 CV_32F matrices
//              through cv::warpAffine (recorded geometry, linear and nearest inverse map).
//   trace-seeded-n103 / trace-seeded-n435 / trace-identity-97 / trace-identity-100
//              Level B: print every intermediate matrix native ECC passes to warpAffine,
//              using OpenCV's own findTransformECC with increasing iteration caps.
//   level-c-seeded-n103 / level-c-seeded-n435 / level-c-identity-97 / level-c-identity-100
//              Level C: call the real exported C ABI (opencv_video_estimate_ecc_affine_seeded
//              or opencv_video_estimate_ecc_affine) with validated caller inputs.
//   negative   Moderate affine transform; must report zero int32-range overflows.
//
// The int64 diagnostic never performs a signed int32 addition itself, so the probe's
// own arithmetic is defined; any UBSan report originates inside OpenCV.

#include "opencv_video_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

// Identical to tests/cpp/ecc_fixture.hpp::signal/texture (copied to keep the probe
// self-contained; the Task 019 sweep used ecc_fixture.hpp directly).
double signal(double x, double y) {
    auto blob = [&](double cx, double cy, double a, double s) {
        const double u = x - cx, v = y - cy;
        return a * std::exp(-(u * u + v * v) / (2.0 * s * s));
    };
    return 127.5 + 30.0 * std::sin(0.21 * x + 0.13 * y) + 25.0 * std::sin(0.11 * x - 0.27 * y + 1.0)
        + 20.0 * std::cos(0.37 * x + 0.05 * y + 2.0) + 15.0 * std::sin(0.06 * x + 0.31 * y)
        + blob(40, 50, 18, 6) + blob(90, 35, -18, 5) + blob(70, 95, 18, 7);
}

cv::Mat texture(int rows, int cols, int dx = 0, int dy = 0) {
    cv::Mat m(rows, cols, CV_8UC1);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            m.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(signal(c - dx, r - dy));
    return m;
}

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

// int64 model of the legacy warpAffine integer path (4.x WarpAffineInvoker; 5.0 legacy
// blockline). Returns the number of destination pixels whose X0+adelta or Y0+bdelta
// lies outside int32 for the given rounding delta (16 linear, 512 nearest).
long fixed_point_overflows(const double M[6], int rows, int cols, int round_delta,
                           const char *label, bool verbose) {
    constexpr int AB_SCALE = 1024;  // AB_BITS = max(10, INTER_BITS = 5)
    long overflows = 0;
    for (int y = 0; y < rows; ++y) {
        const double x_pre = (M[1] * y + M[2]) * AB_SCALE;
        const double y_pre = (M[4] * y + M[5]) * AB_SCALE;
        const int64_t X0 = int64_t(cv::saturate_cast<int>(x_pre)) + round_delta;
        const int64_t Y0 = int64_t(cv::saturate_cast<int>(y_pre)) + round_delta;
        for (int x = 0; x < cols; ++x) {
            const int adelta = cv::saturate_cast<int>(M[0] * x * AB_SCALE);
            const int bdelta = cv::saturate_cast<int>(M[3] * x * AB_SCALE);
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

// Level A: replay one exact recorded CV_32F matrix through native warpAffine.
long level_a(const char *name, const float coeff[6], int rows, int cols) {
    std::printf("[level-a] %s OpenCV %s geometry %dx%d (cols x rows)\n", name, CV_VERSION, cols, rows);
    print_matrix("  stored CV_32F M=", coeff);
    double M[6];
    for (int i = 0; i < 6; ++i) M[i] = coeff[i];
    const long linear = fixed_point_overflows(M, rows, cols, 16, name, true);
    const long nearest = fixed_point_overflows(M, rows, cols, 512, name, true);
    std::printf("  int64 model: linear overflow pixels=%ld nearest overflow pixels=%ld\n", linear, nearest);
    // Exactly the native ECC call shapes: the smoothed image and gradients are CV_32FC1
    // warped with INTER_LINEAR, and the CV_8UC1 support mask with INTER_NEAREST, both with
    // WARP_INVERSE_MAP and the CV_32F 2x3 warp.
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

// Level C: one call to the real exported seeded C ABI. Every intermediate warp is
// generated by native ECC inside the shim's ecc_affine_entry.
void level_c_seeded(const char *name, const float seed[6], int rows, int cols, int iterations) {
    std::printf("[level-c] %s opencv_video_estimate_ecc_affine_seeded geometry %dx%d iters=%d eps=1e-8\n",
                name, cols, rows, iterations);
    print_matrix("  validated caller seed=", seed);
    Handle tmpl = make_handle(texture(rows, cols));
    Handle moving = make_handle(texture(rows, cols, 2, 1));
    opencv_video_ecc_affine_result r = {0, 0, 0, 0, 0, 0, 0};
    std::printf("  BINDING_CALL_BEGIN\n");
    std::fflush(stdout);
    const opencv_video_status status = opencv_video_estimate_ecc_affine_seeded(
        tmpl.get(), moving.get(), iterations, 1e-8, seed[0], seed[1], seed[2], seed[3], seed[4],
        seed[5], &r);
    std::printf("  BINDING_CALL_END status=%d message=\"%s\"\n", int(status),
                status == OPENCV_VIDEO_OK ? "" : opencv_video_last_error());
    if (status == OPENCV_VIDEO_OK) {
        const float w[6] = {r.m00, r.m01, r.m02, r.m10, r.m11, r.m12};
        print_matrix("  published result M=", w);
    }
}

void level_c_identity(int rows, int cols, int iterations) {
    std::printf("[level-c] identity opencv_video_estimate_ecc_affine geometry %dx%d iters=%d eps=1e-8\n",
                cols, rows, iterations);
    Handle tmpl = make_handle(texture(rows, cols));
    Handle moving = make_handle(texture(rows, cols, 2, 1));
    opencv_video_ecc_affine_result r = {0, 0, 0, 0, 0, 0, 0};
    std::printf("  BINDING_CALL_BEGIN\n");
    std::fflush(stdout);
    const opencv_video_status status =
        opencv_video_estimate_ecc_affine(tmpl.get(), moving.get(), iterations, 1e-8, &r);
    std::printf("  BINDING_CALL_END status=%d message=\"%s\"\n", int(status),
                status == OPENCV_VIDEO_OK ? "" : opencv_video_last_error());
}

// Level B iteration provenance. Native ECC is deterministic, and with COUNT|EPS and
// eps = 1e-8 a call capped at k iterations executes exactly the first k iterations of the
// longer run (the loop body does not depend on maxCount). The warp after a k-capped call is
// therefore the matrix that iteration k + 1 passes to warpAffine. Native ECC updates the
// caller's CV_32F warp in place, so the state is observable even when a later iteration
// throws. This uses cv::findTransformECC directly (same arguments as ecc_affine_entry:
// MOTION_AFFINE, no mask, Gaussian 5); it does not reimplement ECC.
long trace(const char *name, const float seed[6], int rows, int cols, int iterations) {
    std::printf("[trace] %s OpenCV %s geometry %dx%d up to %d iterations\n", name, CV_VERSION, cols,
                rows, iterations);
    print_matrix("  seed=", seed);
    const cv::Mat tmpl = texture(rows, cols), moving = texture(rows, cols, 2, 1);
    long first_unsafe = -1;
    cv::Mat previous;
    for (int k = 0; k < iterations; ++k) {
        cv::Mat warp(2, 3, CV_32F);
        for (int i = 0; i < 6; ++i) warp.ptr<float>()[i] = seed[i];
        std::string outcome = "returned";
        if (k > 0) {
            try {
                cv::findTransformECC(tmpl, moving, warp, cv::MOTION_AFFINE,
                                     cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                                      k, 1e-8),
                                     cv::noArray(), 5);
            } catch (const cv::Exception &) {
                outcome = "raised";
            }
        }
        // Unchanged warp: native ECC terminated (EPS reached, or it raised before another
        // update) or applied a zero update, after which rho repeats and EPS ends the loop.
        // Either way no new matrix reaches warpAffine, so the trace is complete.
        if (!previous.empty() && cv::norm(previous, warp, cv::NORM_INF) == 0.0) {
            std::printf("  k=%d warp unchanged (%s): native ECC presents no further matrix; trace ends\n",
                        k, outcome.c_str());
            break;
        }
        previous = warp.clone();
        double M[6];
        for (int i = 0; i < 6; ++i) M[i] = warp.ptr<float>()[i];
        const long linear = fixed_point_overflows(M, rows, cols, 16, "", false);
        const long nearest = fixed_point_overflows(M, rows, cols, 512, "", false);
        std::printf("  warp entering iteration %d (%s after %d updates): ", k + 1, outcome.c_str(), k);
        print_matrix("", warp.ptr<float>());
        if (linear + nearest > 0) {
            std::printf("    UNSAFE_INTERMEDIATE iteration=%d linear_pixels=%ld nearest_pixels=%ld\n",
                        k + 1, linear, nearest);
            fixed_point_overflows(M, rows, cols, 16, "    linear", true);
            if (first_unsafe < 0) first_unsafe = k + 1;
            break;
        }
    }
    std::printf("  first unsafe iteration=%ld\n", first_unsafe);
    return first_unsafe;
}

// Bounded, seeded research search (Task 020). Each trial draws an image pair and geometry
// from a fixed LCG stream, so a trial index fully identifies its inputs. For every trial
// the first unsafe intermediate (if any) is located with the same iteration-cap replay as
// trace(). `seeded` selects a validated in-domain seed (as in Task 019) instead of the
// identity. The search is bounded; a clean result is NOT_REPRODUCED, never SAFE.
struct SearchTrial {
    int rows, cols, kind;
    float seed[6];
    cv::Mat tmpl, moving;
};

SearchTrial search_trial(uint32_t index, bool seeded) {
    uint32_t state = 20u * 2654435761u + index * 40503u + (seeded ? 7u : 0u);
    auto rnd = [&]() {
        state = state * 1664525u + 1013904223u;
        return double(state) / 4294967296.0;
    };
    SearchTrial t;
    t.rows = 32 + int(rnd() * 97);   // 32 .. 128
    t.cols = 32 + int(rnd() * 129);  // 32 .. 160
    t.kind = int(index % 5);
    t.tmpl = texture(t.rows, t.cols);
    const double a = 1 + 0.3 * (rnd() - 0.5), b = 0.3 * (rnd() - 0.5);
    const double c = 0.3 * (rnd() - 0.5), d = 1 + 0.3 * (rnd() - 0.5);
    const double tx = 16 * (rnd() - 0.5), ty = 16 * (rnd() - 0.5);
    t.moving.create(t.rows, t.cols, CV_8UC1);
    const double det = a * d - b * c;
    for (int r = 0; r < t.rows; ++r)
        for (int col = 0; col < t.cols; ++col) {
            double value = 0;
            const double u = col - tx, v = r - ty;
            const double sx = (d * u - b * v) / det, sy = (-c * u + a * v) / det;
            switch (t.kind) {
                case 0: value = signal(col - 2, r - 1); break;                 // translation
                case 1: value = signal(sx, sy); break;                         // moderate affine
                case 2: value = 127.5 + 0.1 * (signal(sx, sy) - 127.5); break; // low contrast
                case 3: value = (r * 131 + col * 71 + (r * col) % 97) % 256; break; // unrelated
                default: value = signal(sx * 1.6, sy * 0.6); break;            // strong anisotropy
            }
            t.moving.at<uint8_t>(r, col) = cv::saturate_cast<uint8_t>(value);
        }
    const float identity[6] = {1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 6; ++i) t.seed[i] = identity[i];
    if (seeded) {  // Task 019 in-domain seed distribution
        t.seed[0] = float(0.25 + 1.5 * rnd()); t.seed[1] = float(rnd() - 0.5);
        t.seed[2] = float(80 * rnd() - 40);    t.seed[3] = float(rnd() - 0.5);
        t.seed[4] = float(0.25 + 1.5 * rnd()); t.seed[5] = float(80 * rnd() - 40);
    }
    return t;
}

// Returns the first iteration whose entering warp overflows int32, or -1.
int first_unsafe_iteration(const SearchTrial &t, int iterations, float out_warp[6]) {
    cv::Mat previous;
    for (int k = 0; k < iterations; ++k) {
        cv::Mat warp(2, 3, CV_32F);
        for (int i = 0; i < 6; ++i) warp.ptr<float>()[i] = t.seed[i];
        bool raised = false;
        if (k > 0) {
            try {
                cv::findTransformECC(t.tmpl, t.moving, warp, cv::MOTION_AFFINE,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, k, 1e-8),
                    cv::noArray(), 5);
            } catch (const cv::Exception &) {
                raised = true;
            }
        }
        // An unchanged warp means native ECC stopped (EPS reached or it raised before any
        // further update): no later iteration executes, so the search for this trial ends.
        if (!previous.empty() && cv::norm(previous, warp, cv::NORM_INF) == 0.0) return -1;
        (void)raised;
        previous = warp.clone();
        double M[6];
        for (int i = 0; i < 6; ++i) M[i] = warp.ptr<float>()[i];
        if (fixed_point_overflows(M, t.rows, t.cols, 16, "", false) +
                fixed_point_overflows(M, t.rows, t.cols, 512, "", false) > 0) {
            for (int i = 0; i < 6; ++i) out_warp[i] = warp.ptr<float>()[i];
            return k + 1;
        }
    }
    return -1;
}

void search(bool seeded, uint32_t first, uint32_t count) {
    std::printf("[search] %s trials %u..%u OpenCV %s (100 iterations, eps 1e-8, no mask)\n",
                seeded ? "seeded" : "identity", first, first + count - 1, CV_VERSION);
    long unsafe = 0;
    for (uint32_t n = first; n < first + count; ++n) {
        const SearchTrial t = search_trial(n, seeded);
        float w[6] = {0, 0, 0, 0, 0, 0};
        const int it = first_unsafe_iteration(t, 100, w);
        if (it < 0) continue;
        ++unsafe;
        std::printf("  SEARCH_UNSAFE trial=%u kind=%d geometry=%dx%d iteration=%d ", n, t.kind, t.cols,
                    t.rows, it);
        print_matrix("seed=", t.seed);
        print_matrix("    unsafe warp=", w);
    }
    std::printf("  SEARCH_SUMMARY %s trials=%u unsafe=%ld\n", seeded ? "seeded" : "identity", count, unsafe);
}

}  // namespace

int main(int argc, char **argv) {
    cv::setNumThreads(1);
    const std::string mode = argc > 1 ? argv[1] : "negative";
    std::printf("ecc_affine_overflow_probe OpenCV %s compiler %s mode=%s\n", CV_VERSION, __VERSION__,
                mode.c_str());
    std::printf("cv::checkHardwareSupport AVX2=%d SSE4_1=%d; useOptimized=%d\n",
                int(cv::checkHardwareSupport(CV_CPU_AVX2)), int(cv::checkHardwareSupport(CV_CPU_SSE4_1)),
                int(cv::useOptimized()));

    // Exact recorded optimizer matrices from PR #19 (docs/ecc-affine-masked-source-contract.md).
    const float matrix_a[6] = {22298.8203125f, -57255.54296875f, 225348.015625f,
                               137349.0625f, -361430.03125f, -3676567.25f};
    const float matrix_b[6] = {-5597.05908203125f, 8140.32177734375f, 285284.03125f,
                               -9895.33203125f, -362406.0f, 484909.9375f};
    // Task 019 deterministic sweep (seed 19 LCG), unmasked seeded trial n = 103 at 64x97.
    // Every coefficient lies inside the published affine seed domain (|linear| <= 4,
    // |translation| <= 2**20), so it is a validated caller input to the shipped API.
    const float seed_n103[6] = {0.580653667f, -0.412717313f, -4.85272264f,
                                -0.241246387f, 0.386808515f, -10.5655231f};
    // Task 019 unmasked seeded trial n = 435 at 64x100 (reported on OpenCV 5.0.0 only).
    const float seed_n435[6] = {0.45051536f, 0.286671013f, 23.7435341f,
                                -0.437181264f, 1.13443863f, -37.4001312f};
    const float identity[6] = {1, 0, 0, 0, 1, 0};

    // One case per process: UBSan reports each source location only once per process,
    // so running two cases together would hide the second case's identical report.
    if (mode == "level-a-matrix-a") {
        level_a("matrix-A", matrix_a, 64, 97);
    } else if (mode == "level-a-matrix-b") {
        level_a("matrix-B", matrix_b, 64, 100);
    } else if (mode == "trace-seeded-n103") {
        trace("seeded-n103", seed_n103, 64, 97, 100);
    } else if (mode == "trace-seeded-n435") {
        trace("seeded-n435", seed_n435, 64, 100, 100);
    } else if (mode == "trace-identity-97") {
        trace("identity", identity, 64, 97, 100);
    } else if (mode == "trace-identity-100") {
        trace("identity", identity, 64, 100, 100);
    } else if (mode == "level-c-seeded-n103") {
        level_c_seeded("seeded-n103", seed_n103, 64, 97, 100);
    } else if (mode == "level-c-seeded-n435") {
        level_c_seeded("seeded-n435", seed_n435, 64, 100, 100);
    } else if (mode == "level-c-identity-97") {
        level_c_identity(64, 97, 100);
    } else if (mode == "level-c-identity-100") {
        level_c_identity(64, 100, 100);
    } else if (mode == "search-identity" || mode == "search-seeded") {
        const uint32_t first = argc > 2 ? uint32_t(std::strtoul(argv[2], nullptr, 10)) : 0;
        const uint32_t count = argc > 3 ? uint32_t(std::strtoul(argv[3], nullptr, 10)) : 50;
        search(mode == "search-seeded", first, count);
    } else if (mode == "negative") {
        const float moderate[6] = {1.03f, 0.02f, 2.0f, -0.01f, 0.98f, 1.0f};
        std::printf("[negative] moderate transform; the int64 model must report zero overflows\n");
        const long n = level_a("moderate", moderate, 64, 97);
        if (n != 0) {
            std::printf("NEGATIVE_CONTROL_FAILED overflow pixels=%ld\n", n);
            return 1;
        }
        std::printf("NEGATIVE_CONTROL_OK\n");
    } else {
        std::fprintf(stderr, "unknown case %s\n", mode.c_str());
        return 2;
    }
    return 0;
}
