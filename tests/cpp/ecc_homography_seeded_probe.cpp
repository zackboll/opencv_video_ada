// Task 017 safety investigation only. Direct OpenCV; no Video/Core binding.
// Compile with the selected OpenCV pkg-config flags and -Itests/cpp.
// Arguments: [trials (default 2000)] [iterations per accepted seed (default 1)].
// Every seed passes the proposed binding seed domain (candidate()); the probe counts native
// successes, native exceptions and successful outputs that ESCAPE that domain, and exits 2 if
// native ECC ever changes H22. It then runs 64 edge-of-denominator seeds for 100 iterations.
// Results are recorded in docs/ecc-homography-seeded-source-contract.md.
#include "ecc_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {
using Warp = std::array<float, 9>;
constexpr Warp identity = {1, 0, 0, 0, 1, 0, 0, 0, 1};

bool candidate(const Warp &h, int rows, int cols) {
    for (float v : h) if (!std::isfinite(v)) return false;
    return h[8] == 1 && std::fabs(double(h[0])) <= 4 &&
        std::fabs(double(h[1])) <= 4 && std::fabs(double(h[3])) <= 4 &&
        std::fabs(double(h[4])) <= 4 && std::fabs(double(h[2])) <= 1048576 &&
        std::fabs(double(h[5])) <= 1048576 &&
        std::fabs(double(h[6])) * (cols - 1) +
        std::fabs(double(h[7])) * (rows - 1) <= 0.5;
}

void print(const Warp &h) {
    for (float v : h) std::cout << ' ' << v;
}

bool run(const cv::Mat &a, const cv::Mat &b, const Warp &seed, int iterations,
         Warp &out, double &rho) {
    cv::Mat w(3, 3, CV_32F);
    std::copy(seed.begin(), seed.end(), w.ptr<float>());
    try {
        rho = cv::findTransformECC(a, b, w, cv::MOTION_HOMOGRAPHY,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                             iterations, 1e-8), cv::noArray(), 5);
        std::copy(w.ptr<float>(), w.ptr<float>() + 9, out.begin());
        return true;
    } catch (const cv::Exception &) {
        std::copy(w.ptr<float>(), w.ptr<float>() + 9, out.begin());
        return false;
    }
}
}

int main(int argc, char **argv) {
    cv::setNumThreads(1);
    const int trials = argc > 1 ? std::atoi(argv[1]) : 2000;
    // Optional second argument: iterations for EVERY accepted seed (default 1). With more
    // than one iteration the later native iterations run on unconstrained updated warps.
    const int sweep_iterations = argc > 2 ? std::atoi(argv[2]) : 1;
    const cv::Mat a = ecc_fixture::texture(64, 96);
    const cv::Mat b = ecc_fixture::texture(64, 96, 2, 1);
    std::cout << std::setprecision(17) << "OpenCV " << CV_VERSION
              << " iterations=" << sweep_iterations << '\n';
    std::uint32_t state = 17;
    auto random = [&]() {
        state = state * 1664525u + 1013904223u;
        return double(state) / 4294967296.0;
    };
    int accepted = 0, success = 0, exceptions = 0, escaped = 0;
    double maximum = 0;
    for (int n = 0; n < trials; ++n) {
        Warp seed = identity;
        if (n < 18) {
            const int i = n / 2;
            seed[i] = (n % 2 ? -1.f : 1.f) *
                (i == 2 || i == 5 ? 1048576.f : i == 6 ? 0.5f / 95 :
                 i == 7 ? 0.5f / 63 : i == 8 ? 1.f : 4.f);
        } else {
            seed[0] = float(4 * random());
            seed[1] = float(2 * random() - 1);
            seed[2] = float(80 * random() - 40);
            seed[3] = float(2 * random() - 1);
            seed[4] = float(std::pow(10., -8 * random()));
            seed[5] = float(80 * random() - 40);
            seed[6] = float((random() - 0.5) * 0.5 / 95);
            seed[7] = float((random() - 0.5) * 0.5 / 63);
        }
        if (!candidate(seed, a.rows, a.cols)) continue;
        ++accepted;
        Warp out;
        double rho = 0;
        const bool ok = run(a, b, seed, sweep_iterations, out, rho);
        if (ok) ++success; else ++exceptions;
        if (ok && !candidate(out, a.rows, a.cols)) {
            ++escaped;
            double magnitude = 0;
            for (float v : out) magnitude = std::max(magnitude, std::fabs(double(v)));
            if (magnitude > maximum) {
                maximum = magnitude;
                std::cout << "escape trial=" << n << " rho=" << rho << " seed";
                print(seed);
                std::cout << " output";
                print(out);
                if (sweep_iterations == 1) {
                    Warp final;
                    double final_rho = 0;
                    const bool final_ok = run(a, b, seed, 100, final, final_rho);
                    std::cout << " full=" << (final_ok ? "OK" : "EXC");
                }
                std::cout << '\n';
            }
        }
        if (out[8] != seed[8]) return 2;
    }
    // Adversarial seeds: start inside the domain at its denominator edge, so a single update
    // can drive D = H20*x + H21*y + 1 through zero inside the image on later iterations.
    int adversarial = 0, adversarial_ok = 0;
    for (int k = 0; k < 64; ++k) {
        Warp seed = identity;
        const double share = (k % 8) / 7.0;
        const double sx = (k & 8) ? -1 : 1, sy = (k & 16) ? -1 : 1;
        seed[6] = float(sx * share * 0.5 / (a.cols - 1));
        seed[7] = float(sy * (1 - share) * 0.5 / (a.rows - 1));
        if (k & 32) { seed[0] = 4; seed[4] = 0.25f; seed[2] = -30; seed[5] = 20; }
        if (!candidate(seed, a.rows, a.cols)) continue;
        ++adversarial;
        Warp out;
        double rho = 0;
        if (run(a, b, seed, 100, out, rho)) ++adversarial_ok;
        if (out[8] != seed[8]) return 2;
    }
    std::cout << "accepted=" << accepted << " success=" << success
              << " exceptions=" << exceptions << " escaped=" << escaped
              << " max_coefficient=" << maximum << " adversarial=" << adversarial
              << " adversarial_success=" << adversarial_ok << '\n';
}