// Task 014 research probe: direct native cv::findTransformECC(MOTION_AFFINE).
// Links OpenCV only: no Video shim, no Core bridge.
//
// Convention under test: the warp W maps TEMPLATE coordinates to MOVING coordinates,
//   moving_x = M00 * x + M01 * y + M02
//   moving_y = M10 * x + M11 * y + M12
// The moving image is built analytically (ecc_fixture::affine_texture) and cross-checked
// against cv::warpAffine of the template. Reports all six coefficients (row-major),
// rho, the widened determinant and the interior residual before/after WARP_INVERSE_MAP.
#include "ecc_fixture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

using namespace ecc_fixture;

namespace {
const double kPi = 3.14159265358979323846;

struct Outcome {
    bool ok = false;
    double rho = 0;
    float w[6] = {0, 0, 0, 0, 0, 0};
    std::string error;
};

Outcome estimate(const cv::Mat &a, const cv::Mat &b, int it = 50, double eps = 1e-4) {
    Outcome o;
    cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
    try {
        o.rho = cv::findTransformECC(a, b, warp, cv::MOTION_AFFINE,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, it, eps),
            cv::noArray(), 5);
        o.ok = true;
        for (int i = 0; i < 6; ++i) o.w[i] = warp.ptr<float>()[i];
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

// truth is row-major {M00, M01, M02, M10, M11, M12}.
void report(const std::string &name, const cv::Mat &tmpl, const cv::Mat &moving,
            const double truth[6], int it = 50, double eps = 1e-4) {
    const Outcome o = estimate(tmpl, moving, it, eps);
    std::cout << std::left << std::setw(26) << name << std::right << std::fixed
              << std::setprecision(5);
    if (!o.ok) {
        std::cout << "EXC  " << o.error.substr(0, 70) << '\n';
        return;
    }
    cv::Mat est(2, 3, CV_32F, const_cast<float *>(o.w));
    cv::Mat aligned;
    cv::warpAffine(moving, aligned, est, moving.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP,
                   cv::BORDER_REFLECT_101);
    const double det = double(o.w[0]) * double(o.w[4]) - double(o.w[1]) * double(o.w[3]);
    std::cout << "M=[";
    for (int i = 0; i < 6; ++i) std::cout << (i ? " " : "") << std::setw(8) << o.w[i];
    std::cout << "] true=[";
    for (int i = 0; i < 6; ++i) std::cout << (i ? " " : "") << std::setw(8) << truth[i];
    std::cout << "] rho=" << std::setw(8) << o.rho << " det=" << std::setw(8) << det
              << " resid " << std::setprecision(2) << interior_residual(tmpl, moving, 24)
              << "->" << interior_residual(tmpl, aligned, 24) << '\n';
}

void affine(const std::string &name, int N, double a, double b, double c, double d,
            double tx, double ty, int it = 50, double eps = 1e-4) {
    const double row[6] = {a, b, tx, c, d, ty};
    report(name, texture(N, N), affine_texture(N, N, a, b, c, d, tx, ty), row, it, eps);
}
}  // namespace

int main() {
    std::cout << "OpenCV " << CV_VERSION << '\n';
    const int N = 128;
    const cv::Mat base = texture(N, N);
    const double d2r = kPi / 180.0;

    // Fixture cross-check: analytic resampling versus warpAffine for the same matrix.
    {
        const double a = 1.01, b = 0.012, c = -0.008, d = 0.99, tx = 2.0, ty = 1.0;
        double wv[6] = {a, b, tx, c, d, ty};
        cv::Mat w(2, 3, CV_64F, wv);
        cv::Mat via_warp;
        cv::warpAffine(base, via_warp, w, base.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
        std::cout << "fixture interior MAD analytic-vs-warpAffine = "
                  << interior_residual(affine_texture(N, N, a, b, c, d, tx, ty), via_warp, 24) << '\n';
    }

    affine("identity", N, 1, 0, 0, 1, 0, 0);
    affine("translation(2,1)", N, 1, 0, 0, 1, 2, 1);
    affine("translation(-2,-1)", N, 1, 0, 0, 1, -2, -1);
    for (double deg : {1.0, -1.0, 3.0, -3.0}) {
        const double t = deg * d2r;
        affine("rot " + std::to_string(int(deg)), N, std::cos(t), -std::sin(t), std::sin(t),
               std::cos(t), 0, 0);
    }
    affine("iso-scale 1.02", N, 1.02, 0, 0, 1.02, 0, 0);
    affine("iso-scale 0.98", N, 0.98, 0, 0, 0.98, 0, 0);
    affine("aniso 1.01/0.99", N, 1.01, 0, 0, 0.99, 0, 0);
    affine("aniso 1.03/0.97", N, 1.03, 0, 0, 0.97, 0, 0);
    affine("shear x +0.012", N, 1, 0.012, 0, 1, 0, 0);
    affine("shear x +0.03", N, 1, 0.03, 0, 1, 0, 0);
    affine("shear y -0.008", N, 1, 0, -0.008, 1, 0, 0);
    affine("shear both", N, 1, 0.012, -0.008, 1, 0, 0);
    affine("combined small", N, 1.01, 0.012, -0.008, 0.99, 2, 1);
    affine("combined neg", N, 0.99, -0.012, 0.008, 1.01, -2, -1);
    affine("rot+scale+shear+t", N, 1.02 * std::cos(2 * d2r), -0.98 * std::sin(2 * d2r) + 0.01,
           1.02 * std::sin(2 * d2r), 0.98 * std::cos(2 * d2r), 1.5, -1);
    affine("reflect x (det<0)", N, -1, 0, 0, 1, 0, 0);
    affine("shear 0.2", N, 1, 0.2, 0, 1, 0, 0);
    affine("strong 1.2/0.8", N, 1.2, 0.1, -0.1, 0.8, 5, -3);
    affine("it1 combined", N, 1.01, 0.012, -0.008, 0.99, 2, 1, 1);
    affine("it3 combined", N, 1.01, 0.012, -0.008, 0.99, 2, 1, 3);
    affine("eps=0.1 combined", N, 1.01, 0.012, -0.008, 0.99, 2, 1, 50, 0.1);
    affine("eps=1e-8 combined", N, 1.01, 0.012, -0.008, 0.99, 2, 1, 100, 1e-8);
    {
        const double row[6] = {1.01, 0.012, 2, -0.008, 0.99, 1};
        const cv::Mat mv = affine_texture(N, N, 1.01, 0.012, -0.008, 0.99, 2, 1);
        report("reversed roles", mv, base, row);
        report("photometric 0.8x+20", base, photometric(mv, 0.8, 20), row);
    }
    const double z[6] = {1, 0, 0, 0, 1, 0};
    report("constant", constant(N, N, 128), constant(N, N, 128), z);
    report("noise vs texture", base, noise(N, N, 7), z);
    report("noise vs noise", noise(N, N, 3), noise(N, N, 9), z);
    report("stripes shift", stripes(N, N, 0), stripes(N, N, 3), z);
    return 0;
}
