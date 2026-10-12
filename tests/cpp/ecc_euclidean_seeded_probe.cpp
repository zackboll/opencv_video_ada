// Task 013 exploratory probe (not part of any gate): which fixtures distinguish an
// identity-initialized Euclidean ECC from a seeded one on the linked OpenCV?
// Links OpenCV only. Usage: ecc-euclidean-seeded-probe
//   prints "<fixture> | <seed label> | OK/EXC w2 w5 theta_deg rho" per combination.
#include "ecc_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {
using namespace ecc_fixture;
const int N = 128;
const double kPi = 3.14159265358979323846;

struct Seed { std::string label; double theta_deg, tx, ty; };

void run(const char *fixture, const cv::Mat &a, const cv::Mat &b, const Seed &s) {
    const double t = s.theta_deg * kPi / 180.0;
    cv::Mat warp(2, 3, CV_32F);
    warp.at<float>(0, 0) = float(std::cos(t));
    warp.at<float>(0, 1) = float(-std::sin(t));
    warp.at<float>(0, 2) = float(s.tx);
    warp.at<float>(1, 0) = float(std::sin(t));
    warp.at<float>(1, 1) = float(std::cos(t));
    warp.at<float>(1, 2) = float(s.ty);
    try {
        const double rho = cv::findTransformECC(a, b, warp, cv::MOTION_EUCLIDEAN,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 50, 1e-4),
            cv::noArray(), 5);
        const double th = std::atan2(double(warp.at<float>(1, 0)), double(warp.at<float>(0, 0)));
        std::printf("%-18s | %-14s | OK  tx=%8.4f ty=%8.4f th=%8.4f deg rho=%.4f\n", fixture,
                    s.label.c_str(), warp.at<float>(0, 2), warp.at<float>(1, 2), th * 180.0 / kPi, rho);
    } catch (const cv::Exception &) {
        std::printf("%-18s | %-14s | EXC\n", fixture, s.label.c_str());
    }
}
}  // namespace

int main() {
    std::printf("OpenCV %s\n", CV_VERSION);
    struct Case { const char *name; double deg, tx, ty; };
    const std::vector<Case> cases = {
        {"rot+8", 8, 0, 0},   {"rot-8", -8, 0, 0},   {"rot+10", 10, 0, 0}, {"rot+12", 12, 0, 0},
        {"rot+15", 15, 0, 0}, {"rot+20", 20, 0, 0},  {"rot+3-t+2+1", 3, 2, 1},
        {"rot+10-t+2+1", 10, 2, 1}, {"rot+12-t+2+1", 12, 2, 1},
    };
    const cv::Mat tmpl = texture(N, N);
    for (const auto &c : cases) {
        const cv::Mat moving = rigid_texture(N, N, c.deg, c.tx, c.ty);
        const std::vector<Seed> seeds = {
            {"identity", 0, 0, 0},
            {"near", c.deg - 0.5, c.tx + 0.3, c.ty - 0.2},
            {"near-exact", c.deg, c.tx, c.ty},
            {"wrong-rot", -c.deg, c.tx, c.ty},
            {"wrong-trans", c.deg, c.tx + 25, c.ty - 25},
            {"half", c.deg / 2, c.tx / 2, c.ty / 2},
        };
        for (const auto &s : seeds) run(c.name, tmpl, moving, s);
    }
    return 0;
}
