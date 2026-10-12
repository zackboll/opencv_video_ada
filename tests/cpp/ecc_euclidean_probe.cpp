// Task 012 research probe: direct native cv::findTransformECC(MOTION_EUCLIDEAN).
// Links OpenCV only: no Video shim, no Core bridge.
//
// Convention under test: the warp W maps TEMPLATE coordinates to MOVING coordinates,
//   moving_x = cos(t) * x - sin(t) * y + tx
//   moving_y = sin(t) * x + cos(t) * y + ty
// The moving image is built with cv::warpAffine(template, W) (forward map), and the
// fixture is cross-checked against an analytic resampling of the template signal.
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

cv::Mat make_warp(double theta, double tx, double ty) {
    cv::Mat w(2, 3, CV_64F);
    w.at<double>(0, 0) = std::cos(theta);  w.at<double>(0, 1) = -std::sin(theta);
    w.at<double>(0, 2) = tx;
    w.at<double>(1, 0) = std::sin(theta);  w.at<double>(1, 1) = std::cos(theta);
    w.at<double>(1, 2) = ty;
    return w;
}

// Rotation about the image centre (cx, cy), then an extra centre displacement (dx, dy).
cv::Mat centred_warp(double theta, double cx, double cy, double dx, double dy) {
    const double tx = cx - (std::cos(theta) * cx - std::sin(theta) * cy) + dx;
    const double ty = cy - (std::sin(theta) * cx + std::cos(theta) * cy) + dy;
    return make_warp(theta, tx, ty);
}

cv::Mat moving_by_warpaffine(const cv::Mat &tmpl, const cv::Mat &w) {
    cv::Mat out;
    // Forward map: dst(W * p) = src(p).
    cv::warpAffine(tmpl, out, w, tmpl.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
    return out;
}

struct Outcome {
    bool ok = false;
    double rho = 0, theta = 0;
    float w[6] = {0, 0, 0, 0, 0, 0};
    std::string error;
};

Outcome estimate(const cv::Mat &a, const cv::Mat &b, int it = 50, double eps = 1e-4) {
    Outcome o;
    cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
    try {
        o.rho = cv::findTransformECC(a, b, warp, cv::MOTION_EUCLIDEAN,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, it, eps),
            cv::noArray(), 5);
        o.ok = true;
        for (int i = 0; i < 6; ++i) o.w[i] = warp.ptr<float>()[i];
        o.theta = std::atan2(double(o.w[3]), double(o.w[0]));
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

void report(const std::string &name, const cv::Mat &tmpl, const cv::Mat &moving, double t,
            double tx, double ty, int it = 50, double eps = 1e-4) {
    const Outcome o = estimate(tmpl, moving, it, eps);
    std::cout << std::left << std::setw(26) << name << std::right << std::fixed
              << std::setprecision(5);
    if (!o.ok) {
        std::cout << "EXC  " << o.error.substr(0, 60) << '\n';
        return;
    }
    cv::Mat est(2, 3, CV_32F, const_cast<float *>(o.w));
    cv::Mat aligned;
    cv::warpAffine(moving, aligned, est, moving.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP,
                   cv::BORDER_REFLECT_101);
    std::cout << "theta=" << std::setw(9) << o.theta << " (true " << std::setw(9) << t
              << ") tx=" << std::setw(9) << o.w[2] << " (" << std::setw(9) << tx
              << ") ty=" << std::setw(9) << o.w[5] << " (" << std::setw(9) << ty
              << ") rho=" << std::setw(8) << o.rho << " resid " << std::setprecision(2)
              << interior_residual(tmpl, moving, 24) << "->" << interior_residual(tmpl, aligned, 24)
              << '\n';
}
}  // namespace

int main() {
    std::cout << "OpenCV " << CV_VERSION << '\n';
    const int N = 128;
    const cv::Mat base = texture(N, N);
    const double d2r = kPi / 180.0;

    // Fixture cross-check: analytic resampling versus warpAffine for the same matrix.
    {
        const cv::Mat w = make_warp(3 * d2r, 2.0, 1.0);
        const cv::Mat via_warp = moving_by_warpaffine(base, w);
        cv::Mat inv;
        cv::invertAffineTransform(w, inv);
        cv::Mat analytic(N, N, CV_8UC1);
        for (int r = 0; r < N; ++r)
            for (int c = 0; c < N; ++c) {
                const double x = inv.at<double>(0, 0) * c + inv.at<double>(0, 1) * r + inv.at<double>(0, 2);
                const double y = inv.at<double>(1, 0) * c + inv.at<double>(1, 1) * r + inv.at<double>(1, 2);
                analytic.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(signal(x, y));
            }
        std::cout << "fixture interior MAD analytic-vs-warpAffine = "
                  << interior_residual(analytic, via_warp, 24) << '\n';
    }

    report("identity", base, base, 0, 0, 0);
    report("translation-only(2,1)", base, texture(N, N, 2, 1), 0, 2, 1);
    for (double deg : {1.0, -1.0, 3.0, -3.0, 6.0, 10.0, 15.0, 25.0}) {
        const double t = deg * d2r;
        report("rot " + std::to_string(int(deg)) + "deg", base,
               moving_by_warpaffine(base, make_warp(t, 0, 0)), t, 0, 0);
    }
    for (double sgn : {1.0, -1.0}) {
        const double t = 3 * d2r;
        report(sgn > 0 ? "rot3 + (2,1)" : "rot3 + (-2,-1)", base,
               moving_by_warpaffine(base, make_warp(t, 2 * sgn, sgn)), t, 2 * sgn, sgn);
    }
    {
        const double t = 3 * d2r;
        const cv::Mat w = centred_warp(t, (N - 1) / 2.0, (N - 1) / 2.0, 1.5, -1.0);
        report("centred rot3 + (1.5,-1)", base, moving_by_warpaffine(base, w), t,
               w.at<double>(0, 2), w.at<double>(1, 2));
    }
    {
        const double t = 3 * d2r;
        const cv::Mat mv = moving_by_warpaffine(base, make_warp(t, 2, 1));
        report("reversed roles rot3+(2,1)", mv, base, -t, 0, 0);
        report("photometric 0.8x+20", base, photometric(mv, 0.8, 20), t, 2, 1);
        report("iterations=1", base, mv, t, 2, 1, 1);
        report("iterations=3", base, mv, t, 2, 1, 3);
        report("eps=0.1", base, mv, t, 2, 1, 50, 0.1);
        report("eps=1e-8", base, mv, t, 2, 1, 50, 1e-8);
    }
    report("constant", constant(N, N, 128), constant(N, N, 128), 0, 0, 0);
    report("noise vs texture", base, noise(N, N, 7), 0, 0, 0);
    report("noise vs noise", noise(N, N, 3), noise(N, N, 9), 0, 0, 0);
    report("stripes rot3", stripes(N, N, 0),
           moving_by_warpaffine(stripes(N, N, 0), make_warp(3 * d2r, 0, 0)), 3 * d2r, 0, 0);
    return 0;
}
