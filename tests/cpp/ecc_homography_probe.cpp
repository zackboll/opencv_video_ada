// Task 016 research probe: direct native cv::findTransformECC(MOTION_HOMOGRAPHY).
// Links OpenCV only: no Video shim, no Core bridge. One case per process so the caller can
// apply ulimit/timeout containment.
//
//   ecc_homography_probe case NAME [SIZE]
//   ecc_homography_probe mem MODEL SIZE [F]   (MODEL: trans|euclid|affine|homog; cols=SIZE*F, rows=SIZE/F)
//   ecc_homography_probe prim                 (divide / inv / warpPerspective primitive behavior)
//   ecc_homography_probe list
//
// Convention: H maps TEMPLATE coordinates to MOVING coordinates (homogeneous division);
// moving(p) = signal(H^-1 p) evaluated analytically, with explicit denominator handling.
#include "ecc_fixture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sys/resource.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ecc_fixture;

namespace {

// 3x3 inverse via adjugate; returns false when |det| is tiny.
bool invert3(const double *m, double *o) {
    const double c00 = m[4] * m[8] - m[5] * m[7], c01 = m[5] * m[6] - m[3] * m[8],
                 c02 = m[3] * m[7] - m[4] * m[6];
    const double det = m[0] * c00 + m[1] * c01 + m[2] * c02;
    if (!(std::fabs(det) > 1e-12)) return false;
    o[0] = c00 / det; o[1] = (m[2] * m[7] - m[1] * m[8]) / det; o[2] = (m[1] * m[5] - m[2] * m[4]) / det;
    o[3] = c01 / det; o[4] = (m[0] * m[8] - m[2] * m[6]) / det; o[5] = (m[2] * m[3] - m[0] * m[5]) / det;
    o[6] = c02 / det; o[7] = (m[1] * m[6] - m[0] * m[7]) / det; o[8] = (m[0] * m[4] - m[1] * m[3]) / det;
    return true;
}

// moving(p) = template_signal(H^-1 p). Points whose inverse denominator is zero get mid-gray.
cv::Mat homography_texture(int rows, int cols, const double *H, int *bad = nullptr) {
    double inv[9];
    cv::Mat m(rows, cols, CV_8UC1, cv::Scalar(128));
    if (!invert3(H, inv)) { if (bad) *bad = rows * cols; return m; }
    int nbad = 0;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            const double d = inv[6] * c + inv[7] * r + inv[8];
            if (d == 0.0) { ++nbad; continue; }
            m.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(
                signal((inv[0] * c + inv[1] * r + inv[2]) / d, (inv[3] * c + inv[4] * r + inv[5]) / d));
        }
    if (bad) *bad = nbad;
    return m;
}

struct Outcome { bool ok = false; double rho = 0; float w[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; std::string error; };

int motion_of(const std::string &m) {
    if (m == "trans") return cv::MOTION_TRANSLATION;
    if (m == "euclid") return cv::MOTION_EUCLIDEAN;
    if (m == "affine") return cv::MOTION_AFFINE;
    return cv::MOTION_HOMOGRAPHY;
}

Outcome estimate(const cv::Mat &a, const cv::Mat &b, int motion = cv::MOTION_HOMOGRAPHY,
                 int it = 50, double eps = 1e-4) {
    Outcome o;
    const int rows = motion == cv::MOTION_HOMOGRAPHY ? 3 : 2;
    cv::Mat warp = cv::Mat::eye(rows, 3, CV_32F);
    try {
        o.rho = cv::findTransformECC(a, b, warp, motion,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, it, eps), cv::noArray(), 5);
        o.ok = true;
        for (int i = 0; i < rows * 3; ++i) o.w[i] = warp.ptr<float>()[i];
    } catch (const cv::Exception &e) { o.error = e.what(); }
    return o;
}

double interior_residual(const cv::Mat &a, const cv::Mat &b, int margin) {
    const cv::Rect roi(margin, margin, a.cols - 2 * margin, a.rows - 2 * margin);
    cv::Mat fa, fb;
    a(roi).convertTo(fa, CV_32F);
    b(roi).convertTo(fb, CV_32F);
    return cv::mean(cv::abs(fa - fb))[0];
}

// Resample the moving image into the template frame with WARP_INVERSE_MAP and H as returned.
double residual_with(const cv::Mat &tmpl, const cv::Mat &moving, const double *H, int margin) {
    cv::Mat Hm(3, 3, CV_64F, const_cast<double *>(H)), warped;
    cv::warpPerspective(moving, warped, Hm, tmpl.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    return interior_residual(tmpl, warped, margin);
}

struct Case { const char *name; double h[9]; };
const std::vector<Case> &cases() {
    static const std::vector<Case> c = {
        {"identity", {1, 0, 0, 0, 1, 0, 0, 0, 1}},
        {"translation", {1, 0, 2, 0, 1, 1, 0, 0, 1}},
        {"rotation-2deg", {0.99939083, -0.0348995, 0, 0.0348995, 0.99939083, 0, 0, 0, 1}},
        {"scale-1.02", {1.02, 0, 0, 0, 1.02, 0, 0, 0, 1}},
        {"scale-nonuniform", {1.03, 0, 0, 0, 0.98, 0, 0, 0, 1}},
        {"shear-0.02", {1, 0.02, 0, 0, 1, 0, 0, 0, 1}},
        {"affine-combo", {1.01, 0.01, 1.5, -0.01, 0.99, -1.0, 0, 0, 1}},
        {"persp-mild", {1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0}},
        {"persp-h20-pos", {1, 0, 0, 0, 1, 0, 0.0002, 0, 1}},
        {"persp-h20-neg", {1, 0, 0, 0, 1, 0, -0.0002, 0, 1}},
        {"persp-h21-pos", {1, 0, 0, 0, 1, 0, 0, 0.0002, 1}},
        {"persp-h21-neg", {1, 0, 0, 0, 1, 0, 0, -0.0002, 1}},
        {"persp-both", {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1}},
        {"persp-strong", {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1}},
        {"rot-persp", {0.9994, -0.0349, 1.0, 0.0349, 0.9994, 0.5, 0.0001, 0.00005, 1}},
        {"sst-persp", {1.02, 0.01, 1.0, -0.01, 0.98, -0.5, 0.0001, -0.0001, 1}},
        {"persp-near-zero-den", {1, 0, 0, 0, 1, 0, -0.0078, 0, 1}},
        {"persp-den-crosses-zero", {1, 0, 0, 0, 1, 0, -0.012, 0, 1}},
        {"singular-rank2", {1, 0, 0, 2, 0, 0, 0, 0, 1}},
        {"illcond", {1, 0, 0, 0, 1e-6, 0, 0, 0, 1}},
        {"large-update", {1, 0, 40, 0, 1, 30, 0, 0, 1}},
        {"reflect-x", {-1, 0, 127, 0, 1, 0, 0, 0, 1}},
    };
    return c;
}

void print_w(const Outcome &o) {
    for (int i = 0; i < 9; ++i) std::cout << ' ' << std::setw(11) << o.w[i];
}

int run_special(const std::string &name, int size) {
    cv::Mat tmpl = texture(size, size), moving;
    if (name == "constant") { tmpl = cv::Mat(size, size, CV_8UC1, cv::Scalar(100)); moving = tmpl.clone(); }
    else if (name == "lowgrad") {
        tmpl = cv::Mat(size, size, CV_8UC1);
        for (int r = 0; r < size; ++r) for (int c = 0; c < size; ++c) tmpl.at<uint8_t>(r, c) = uint8_t(100 + (c + r) / 64);
        moving = tmpl.clone();
    } else {
        cv::RNG ra(1), rb(2);
        tmpl = cv::Mat(size, size, CV_8UC1); moving = cv::Mat(size, size, CV_8UC1);
        ra.fill(tmpl, cv::RNG::UNIFORM, 0, 256); rb.fill(moving, cv::RNG::UNIFORM, 0, 256);
        if (name == "noise") moving = tmpl.clone();
    }
    const Outcome o = estimate(tmpl, moving);
    std::cout << name << (o.ok ? " OK  rho=" : " EXC ") << (o.ok ? std::to_string(o.rho) : o.error.substr(0, 90));
    if (o.ok) print_w(o);
    std::cout << '\n';
    return 0;
}

int run_case(const std::string &name, int size) {
    std::cout << std::fixed << std::setprecision(6);
    if (name == "constant" || name == "lowgrad" || name == "noise" || name == "noise-vs-noise")
        return run_special(name, size);
    const cv::Mat tmpl = texture(size, size);
    for (const Case &c : cases()) {
        if (name != c.name) continue;
        int bad = 0;
        const cv::Mat moving = homography_texture(size, size, c.h, &bad);
        // Fixture cross-check: forward warpPerspective of the template with the TRUE H.
        cv::Mat H(3, 3, CV_64F, const_cast<double *>(c.h)), chk;
        cv::warpPerspective(tmpl, chk, H, tmpl.size(), cv::INTER_LINEAR);
        const int margin = size / 6;
        const double fixture_err = interior_residual(chk, moving, margin);
        const Outcome o = estimate(tmpl, moving);
        const Outcome oa = estimate(tmpl, moving, cv::MOTION_AFFINE);
        std::cout << std::left << std::setw(24) << name << std::right << " baddenom=" << bad
                  << " fixture_vs_warpPerspective=" << fixture_err << '\n';
        std::cout << "  homog: " << (o.ok ? "OK  rho=" + std::to_string(o.rho) : "EXC " + o.error.substr(0, 90)) << '\n';
        if (o.ok) {
            double e[9];
            bool fin = true;
            double maxd = 0;
            for (int i = 0; i < 9; ++i) {
                e[i] = o.w[i];
                fin = fin && std::isfinite(o.w[i]);
                maxd = std::max(maxd, std::fabs(e[i] - c.h[i]));
            }
            static const double kId[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            std::cout << "  W:"; print_w(o);
            std::cout << "\n  max|W-H|=" << maxd << " finite=" << fin
                      << " resid(id)=" << residual_with(tmpl, moving, kId, margin)
                      << " resid(W)=" << residual_with(tmpl, moving, e, margin)
                      << " resid(true)=" << residual_with(tmpl, moving, c.h, margin) << '\n';
        }
        std::cout << "  affine: " << (oa.ok ? "OK  rho=" + std::to_string(oa.rho) : "EXC " + oa.error.substr(0, 90));
        if (oa.ok) {
            const double A[9] = {oa.w[0], oa.w[1], oa.w[2], oa.w[3], oa.w[4], oa.w[5], 0, 0, 1};
            std::cout << " resid(A)=" << residual_with(tmpl, moving, A, margin);
        }
        std::cout << '\n';
        return 0;
    }
    std::cerr << "unknown case " << name << '\n';
    return 2;
}

int run_mem(const std::string &model, int rows, int cols) {
    const cv::Mat a = texture(rows, cols), b = texture(rows, cols, 1, 1);
    const auto t0 = std::chrono::steady_clock::now();
    const Outcome o = estimate(a, b, motion_of(model), 10, 1e-4);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    std::cout << "mem model=" << model << " " << cols << "x" << rows << " ok=" << o.ok
              << " peak_rss_mib=" << ru.ru_maxrss / 1024.0 << " secs=" << secs
              << (o.ok ? "" : " err=" + o.error.substr(0, 80)) << '\n';
    return 0;
}

// Primitive behavior the source review depends on.
int run_prim() {
    std::cout << cv::getVersionString() << '\n';
    cv::Mat num = (cv::Mat_<float>(1, 4) << 1.f, -1.f, 0.f, 1e30f);
    cv::Mat den = (cv::Mat_<float>(1, 4) << 0.f, 0.f, 0.f, 1e-30f), out;
    cv::divide(num, den, out);
    std::cout << "divide f32 [1,-1,0,1e30]/[0,0,0,1e-30] =";
    for (int i = 0; i < 4; ++i) std::cout << ' ' << out.at<float>(0, i);
    cv::Mat sing = cv::Mat::ones(8, 8, CV_32F), si = sing.inv();
    std::cout << "\ninv(singular 8x8 f32) norm=" << cv::norm(si) << '\n';
    cv::Mat big = cv::Mat::eye(8, 8, CV_32F) * 1e-30f, bi = big.inv();
    std::cout << "inv(1e-30*I) norm=" << cv::norm(bi) << '\n';
    cv::Mat src = texture(64, 64), dst;
    const int fl = cv::INTER_LINEAR + cv::WARP_INVERSE_MAP;
    double M[9] = {1, 0, 0, 0, 1, 0, 1e30, 1e30, 1.0};
    cv::warpPerspective(src, dst, cv::Mat(3, 3, CV_64F, M), src.size(), fl);
    std::cout << "warpPerspective(M[6]=M[7]=1e30) ok mean=" << cv::mean(dst)[0] << '\n';
    double Z[9] = {1, 0, 0, 0, 1, 0, 0, 0, 0};
    cv::warpPerspective(src, dst, cv::Mat(3, 3, CV_64F, Z), src.size(), fl);
    std::cout << "warpPerspective(W==0 everywhere) ok mean=" << cv::mean(dst)[0] << '\n';
    double N[9] = {1, 0, 0, 0, 1, 0, std::nan(""), 0, 1};
    cv::warpPerspective(src, dst, cv::Mat(3, 3, CV_64F, N), src.size(), fl);
    std::cout << "warpPerspective(NaN coefficient) ok mean=" << cv::mean(dst)[0] << '\n';
    double I[9] = {1e300, 0, 0, 0, 1e300, 0, 1e300, 1e300, 1.0};
    cv::warpPerspective(src, dst, cv::Mat(3, 3, CV_64F, I), src.size(), fl);
    std::cout << "warpPerspective(1e300 coefficients) ok mean=" << cv::mean(dst)[0] << '\n';
    return 0;
}
}  // namespace

int main(int argc, char **argv) {
    if (argc >= 2 && !std::strcmp(argv[1], "list")) {
        for (const Case &c : cases()) std::cout << c.name << '\n';
        std::cout << "constant\nlowgrad\nnoise\nnoise-vs-noise\n";
        return 0;
    }
    if (argc >= 2 && !std::strcmp(argv[1], "prim")) return run_prim();
    if (argc >= 4 && !std::strcmp(argv[1], "mem")) {
        const int size = std::stoi(argv[3]);
        const int factor = argc >= 5 ? std::stoi(argv[4]) : 1;
        return run_mem(argv[2], size / factor, size * factor);
    }
    if (argc >= 3 && !std::strcmp(argv[1], "case")) return run_case(argv[2], argc >= 4 ? std::stoi(argv[3]) : 128);
    std::cerr << "usage: probe case NAME [SIZE] | mem MODEL SIZE [F] | prim | list\n";
    return 2;
}
