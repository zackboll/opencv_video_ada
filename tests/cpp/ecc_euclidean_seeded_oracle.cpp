// Independent direct-OpenCV SEEDED Euclidean ECC oracle (Task 013): links OpenCV only; no
// Video shim and no Core bridge. Writes obj/oracle/ecc-euclidean-seeded.txt:
//   ecc-euclidean-seeded-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> <seed-theta> <seed-tx> <seed-ty> <OK|EXC>
//           w0 w1 w2 w3 w4 w5 <tx> <ty> <theta> <rho> <template-sum> <moving-sum>
//   end <fixture count>
// The seed fields are the exact values placed in the native warp (theta as double, tx/ty
// as the Float32 value widened to double). w0..w5 are the six native Float32 warp entries
// (row-major 2x3); theta is atan2(w3, w0). Floating values use %.17e; EXC lines carry zeros.
#include "ecc_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace {
using namespace ecc_fixture;
const int N = 128;
const double kPi = 3.14159265358979323846;
const double kDeg = kPi / 180.0;

int64_t checksum(const cv::Mat &m) {
    int64_t sum = 0;
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
            sum += int64_t(m.at<uint8_t>(r, c)) * (1 + (int64_t(r) * 977 + int64_t(c) * 131) % 8191);
    return sum;
}

struct Fixture {
    std::string name;
    int iterations;
    double epsilon;
    double seed_theta;
    float seed_tx, seed_ty;
    std::function<cv::Mat()> make_template, make_moving;
};

std::vector<Fixture> fixtures() {
    auto tex = [] { return texture(N, N); };
    auto rigid = [](double deg, double tx, double ty) {
        return [=] { return rigid_texture(N, N, deg, tx, ty); };
    };
    const double cx = (N - 1) / 2.0, cy = (N - 1) / 2.0, t3 = 3.0 * kDeg;
    const double ctx = cx - (std::cos(t3) * cx - std::sin(t3) * cy) + 1.5;
    const double cty = cy - (std::sin(t3) * cx + std::cos(t3) * cy) - 1.0;
    std::vector<Fixture> f;
    auto add = [&](const char *name, std::function<cv::Mat()> a, std::function<cv::Mat()> b,
                   double seed_deg, double sx, double sy, int iterations = 50,
                   double epsilon = 1e-4) {
        f.push_back({name, iterations, epsilon, seed_deg * kDeg, float(sx), float(sy), a, b});
    };
    // Zero seeds: must behave exactly like the identity-initialized Task 012 path.
    add("zero-identity", tex, tex, 0, 0, 0);
    add("zero-translation-2-1", tex, rigid(0, 2, 1), 0, 0, 0);
    add("zero-rot+3", tex, rigid(3, 0, 0), 0, 0, 0);
    add("zero-rot+3-t+2+1", tex, rigid(3, 2, 1), 0, 0, 0);
    add("zero-rot+12-nonconv", tex, rigid(12, 0, 0), 0, 0, 0);
    add("zero-rot+10-t+2+1", tex, rigid(10, 2, 1), 0, 0, 0);
    // Near-correct seeds on fixtures identity initialization cannot recover.
    add("near-rot+8", tex, rigid(8, 0, 0), 7.5, 0.3, -0.2);
    add("near-rot-8", tex, rigid(-8, 0, 0), -8.5, 0.3, -0.2);
    add("near-rot+10", tex, rigid(10, 0, 0), 9.5, 0.3, -0.2);
    add("near-rot+12", tex, rigid(12, 0, 0), 11.5, 0.3, -0.2);
    add("near-rot+10-t+2+1", tex, rigid(10, 2, 1), 9.5, 2.3, 0.8);
    add("near-rot+12-t+2+1", tex, rigid(12, 2, 1), 11.5, 2.3, 0.8);
    // Imperfect seed on a fixture identity initialization also solves (refinement).
    add("imperfect-rot+3-t+2+1", tex, rigid(3, 2, 1), 2.5, 2.3, 0.8);
    add("imperfect-it1", tex, rigid(3, 2, 1), 2.5, 2.3, 0.8, 1, 1e-4);
    add("imperfect-it2", tex, rigid(3, 2, 1), 2.5, 2.3, 0.8, 2, 1e-4);
    add("imperfect-it100-e8", tex, rigid(3, 2, 1), 2.5, 2.3, 0.8, 100, 1e-8);
    add("imperfect-e0.1", tex, rigid(3, 2, 1), 2.5, 2.3, 0.8, 50, 0.1);
    // Exact seeds, both signs, rotation about the origin plus translation.
    add("exact-rot-3-t+2-1", tex, rigid(-3, 2, -1), -3, 2, -1);
    add("exact-rot+3-t-2-1", tex, rigid(3, -2, -1), 3, -2, -1);
    add("exact-rot-3-t-2+1", tex, rigid(-3, -2, 1), -3, -2, 1);
    // Centred rotation: the seed is the origin-rotation MATRIX translation.
    add("centred-exact", tex, [=] { return rigid_texture(N, N, 3.0, ctx, cty); }, 3, ctx, cty);
    add("centred-near", tex, [=] { return rigid_texture(N, N, 3.0, ctx, cty); }, 2.6, ctx + 0.4, cty - 0.3);
    // Deliberately inaccurate but valid seeds (asymmetric so axis/sign mistakes differ).
    add("wrong-trans-rot+10", tex, rigid(10, 0, 0), 10, 25, -25);
    add("wrong-trans-rot+12-t+2+1", tex, rigid(12, 2, 1), 12, 25, -25);
    add("wrong-rot-rot+12", tex, rigid(12, 0, 0), -12, 0, 0);
    add("wrong-rot-rot+15", tex, rigid(15, 0, 0), -15, 0, 0);
    add("wrong-rot-rot+10-t+2+1", tex, rigid(10, 2, 1), -10, 2, 1);
    add("asym-rot+10", tex, rigid(10, 0, 0), 6, 12, -4);
    add("asym-rot+10-b", tex, rigid(10, 0, 0), 8, 5, 14);
    // Degenerate images with a nonzero seed.
    add("seed-constant", [] { return constant(N, N, 128); }, [] { return constant(N, N, 128); }, 1, 1, 1);
    add("seed-noise-vs-noise", [] { return noise(N, N, 3); }, [] { return noise(N, N, 9); }, 1, 1, 1);
    add("seed-noise-vs-texture", tex, [] { return noise(N, N, 7); }, 1, 1, 1);
    // Seed-domain boundaries (accepted by the binding; native outcome is recorded).
    add("bound-angle+1rad", tex, rigid(3, 2, 1), 1.0 / kDeg, 0, 0);
    add("bound-angle-1rad", tex, rigid(3, 2, 1), -1.0 / kDeg, 0, 0);
    add("bound-translation", tex, rigid(3, 2, 1), 0, 1048576.0, -1048576.0);
    // Region fixture: 160x160 parents, 128x128 window at (16,16) (non-contiguous).
    add("region-near", [] { return texture(160, 160); }, [] { return rigid_texture(160, 160, 3, 2, 1); },
        2.5, 2.3, 0.8);
    add("region-zero", [] { return texture(160, 160); }, [] { return rigid_texture(160, 160, 3, 2, 1); },
        0, 0, 0);
    // Exact radian bounds (not round-tripped through degrees).
    for (auto &item : f) {
        if (item.name == "bound-angle+1rad") item.seed_theta = 1.0;
        if (item.name == "bound-angle-1rad") item.seed_theta = -1.0;
    }
    return f;
}

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-euclidean-seeded-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-euclidean-seeded-oracle 1\nversion " << CV_VERSION << '\n';
    int count = 0;
    char buffer[1280];
    const cv::Rect window(16, 16, N, N);
    try {
        for (const auto &f : fixtures()) {
            cv::Mat a = f.make_template(), b = f.make_moving();
            if (starts_with(f.name, "region-")) {
                a = a(window);
                b = b(window);
                if (a.isContinuous() || b.isContinuous()) {
                    std::fprintf(stderr, "oracle: Region fixture is contiguous\n");
                    return 3;
                }
            }
            // Independent warp construction: [c -s tx; s c ty] with Float32 storage.
            const double c = std::cos(f.seed_theta), s = std::sin(f.seed_theta);
            cv::Mat warp(2, 3, CV_32F);
            warp.at<float>(0, 0) = float(c);
            warp.at<float>(0, 1) = float(-s);
            warp.at<float>(0, 2) = f.seed_tx;
            warp.at<float>(1, 0) = float(s);
            warp.at<float>(1, 1) = float(c);
            warp.at<float>(1, 2) = f.seed_ty;
            bool ok = true;
            double rho = 0;
            try {
                rho = cv::findTransformECC(a, b, warp, cv::MOTION_EUCLIDEAN,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                     f.iterations, f.epsilon), cv::noArray(), 5);
            } catch (const cv::Exception &) { ok = false; }
            double w[6] = {0, 0, 0, 0, 0, 0};
            if (ok) {
                if (warp.rows != 2 || warp.cols != 3 || warp.type() != CV_32FC1 || !std::isfinite(rho)) {
                    std::fprintf(stderr, "oracle: malformed native output for %s\n", f.name.c_str());
                    return 4;
                }
                for (int i = 0; i < 6; ++i) {
                    w[i] = double(warp.ptr<float>()[i]);
                    if (!std::isfinite(w[i])) {
                        std::fprintf(stderr, "oracle: nonfinite warp for %s\n", f.name.c_str());
                        return 4;
                    }
                }
            }
            const double theta = ok ? std::atan2(w[3], w[0]) : 0.0;
            std::snprintf(buffer, sizeof buffer,
                "fixture %s %d %.17e %.17e %.17e %.17e %s %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %lld %lld\n",
                f.name.c_str(), f.iterations, f.epsilon, f.seed_theta, double(f.seed_tx),
                double(f.seed_ty), ok ? "OK" : "EXC", w[0], w[1], w[2], w[3], w[4], w[5], w[2], w[5],
                theta, ok ? rho : 0.0, (long long)checksum(a), (long long)checksum(b));
            out << buffer;
            ++count;
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "oracle failure: %s\n", e.what());
        return 1;
    }
    out << "end " << count << '\n';
    out.close();
    if (!out) return 5;
    std::printf("PASS: ECC Euclidean seeded oracle OpenCV %s, %d fixtures\n", CV_VERSION, count);
    return 0;
}
