// Stage B research probe: direct native cv::findTransformECC(MOTION_TRANSLATION).
// Deliberately links OpenCV only: no Video shim, no Core bridge.
#include "ecc_fixture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

using namespace ecc_fixture;

static double interior_residual(const cv::Mat &a, const cv::Mat &b, int margin) {
    const cv::Rect roi(margin, margin, a.cols - 2 * margin, a.rows - 2 * margin);
    cv::Mat fa, fb;
    a(roi).convertTo(fa, CV_32F);
    b(roi).convertTo(fb, CV_32F);
    return cv::mean(cv::abs(fa - fb))[0];
}

struct Outcome {
    bool ok = false;
    float dx = 0, dy = 0;
    double rho = 0;
    std::string error;
    cv::Mat warp;
};

static Outcome estimate(const cv::Mat &tmpl, const cv::Mat &moving, int iterations = 50,
                        double eps = 1e-4) {
    Outcome o;
    o.warp = cv::Mat::eye(2, 3, CV_32F);
    try {
        o.rho = cv::findTransformECC(tmpl, moving, o.warp, cv::MOTION_TRANSLATION,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, eps),
            cv::noArray(), 5);
        o.ok = true;
        o.dx = o.warp.at<float>(0, 2);
        o.dy = o.warp.at<float>(1, 2);
    } catch (const cv::Exception &e) {
        o.error = std::string("cv::Exception code=") + std::to_string(e.code) + " " +
            e.err.substr(0, 80);
    }
    return o;
}

static void report(const char *name, const cv::Mat &tmpl, const cv::Mat &moving,
                   int iterations = 50, double eps = 1e-4) {
    const Outcome o = estimate(tmpl, moving, iterations, eps);
    std::cout << std::fixed << std::setprecision(6) << name << ' ';
    if (!o.ok) {
        std::cout << "EXCEPTION " << o.error << '\n';
        return;
    }
    const bool schema = o.warp.rows == 2 && o.warp.cols == 3 && o.warp.type() == CV_32FC1 &&
        o.warp.at<float>(0, 0) == 1 && o.warp.at<float>(0, 1) == 0 &&
        o.warp.at<float>(1, 0) == 0 && o.warp.at<float>(1, 1) == 1;
    // Direction check: warp moving back into the template frame with WARP_INVERSE_MAP.
    if (!std::isfinite(o.dx) || !std::isfinite(o.dy) || !std::isfinite(o.rho) ||
        std::abs(o.dx) > 32 || std::abs(o.dy) > 32) {
        std::cout << "NONFINITE-OR-WILD dx=" << o.dx << " dy=" << o.dy << " rho=" << o.rho
                  << " schema=" << (schema ? "identity-linear" : "BAD") << '\n';
        return;
    }
    const int margin = std::min(8 + int(std::ceil(std::max(std::abs(o.dx), std::abs(o.dy)))),
                                (std::min(tmpl.rows, tmpl.cols) - 1) / 2);
    if (margin < 0 || tmpl.rows - 2 * margin < 1 || tmpl.cols - 2 * margin < 1) {
        std::cout << "OK dx=" << o.dx << " dy=" << o.dy << " rho=" << o.rho << " (no residual)\n";
        return;
    }
    cv::Mat aligned, negated;
    cv::warpAffine(moving, aligned, o.warp, moving.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);
    cv::Mat inv = o.warp.clone();
    inv.at<float>(0, 2) = -o.dx;
    inv.at<float>(1, 2) = -o.dy;
    cv::warpAffine(moving, negated, inv, moving.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);
    std::cout << "OK dx=" << o.dx << " dy=" << o.dy << " rho=" << o.rho << " schema="
              << (schema ? "identity-linear" : "BAD")
              << " resid_before=" << interior_residual(tmpl, moving, margin)
              << " resid_after=" << interior_residual(tmpl, aligned, margin)
              << " resid_negated=" << interior_residual(tmpl, negated, margin) << '\n';
}

int main(int argc, char **argv) {
    std::cout << std::unitbuf << "opencv=" << CV_VERSION << '\n';
    if (argc == 4 && std::string(argv[1]) == "big") {  // memory/time probe
        const int r = std::atoi(argv[2]), c = std::atoi(argv[3]);
        cv::Mat a = texture(r, c, 0, 0), b = texture(r, c, 2, 1);
        report("big", a, b);
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "small") {  // tiny-geometry behavior
        for (int n : {1, 2, 3, 4, 5, 6, 8, 16, 24, 31, 32, 33, 48}) {
            const std::string name = "small-" + std::to_string(n);
            report(name.c_str(), texture(n, n), texture(n, n, 1, 0));
        }
        return 0;
    }
    const int N = 128;
    const cv::Mat base = texture(N, N);
    report("identity", base, base);
    report("positive(2,1)", base, texture(N, N, 2, 1));
    report("negative(-2,-1)", base, texture(N, N, -2, -1));
    report("horizontal(3,0)", base, texture(N, N, 3, 0));
    report("vertical(0,3)", base, texture(N, N, 0, 3));
    report("contrast0.8+20", base, photometric(texture(N, N, 2, 1), 0.8, 20));
    report("contrast1.3-10", base, photometric(texture(N, N, 2, 1), 1.3, -10));
    report("constant", constant(N, N, 128), constant(N, N, 128));
    report("constant-vs-texture", constant(N, N, 128), base);
    report("low-variance", low_variance(base), low_variance(texture(N, N, 2, 1)));
    report("dissimilar-noise", base, noise(N, N, 7));
    report("noise-vs-noise", noise(N, N, 3), noise(N, N, 9));
    report("large(12,7)", base, texture(N, N, 12, 7));
    report("large(6,4)", base, texture(N, N, 6, 4));
    report("large(40,30)", base, texture(N, N, 40, 30));
    report("stripes-x-shift3", stripes(N, N, 0), stripes(N, N, 3));
    report("stripes-identity", stripes(N, N, 0), stripes(N, N, 0));
    report("iterations=1", base, texture(N, N, 2, 1), 1, 1e-4);
    report("iterations=2", base, texture(N, N, 2, 1), 2, 1e-4);
    report("eps=0.1", base, texture(N, N, 2, 1), 50, 0.1);
    report("eps=1e-8", base, texture(N, N, 2, 1), 50, 1e-8);

    // Non-contiguous Regions versus contiguous clones.
    const cv::Mat parent_a = texture(160, 160, 0, 0), parent_b = texture(160, 160, 2, 1);
    const cv::Rect roi(16, 16, N, N);
    const cv::Mat ra = parent_a(roi), rb = parent_b(roi);
    std::cout << "region-contiguous=" << ra.isContinuous() << rb.isContinuous() << '\n';
    report("region", ra, rb);
    report("region-clone", ra.clone(), rb.clone());
    const Outcome o1 = estimate(ra, rb), o2 = estimate(ra.clone(), rb.clone());
    std::cout << "region-vs-clone-bitwise-equal="
              << (o1.ok && o2.ok && o1.dx == o2.dx && o1.dy == o2.dy && o1.rho == o2.rho) << '\n';

    // Reversed roles must negate the displacement.
    report("reversed(2,1)", texture(N, N, 2, 1), base);

    // Input immutability and aliasing.
    cv::Mat a = base.clone(), a0 = a.clone();
    Outcome same = estimate(a, a);
    std::cout << "alias-identity ok=" << same.ok << " unchanged=" << (cv::norm(a, a0, cv::NORM_INF) == 0)
              << '\n';
    return 0;
}
