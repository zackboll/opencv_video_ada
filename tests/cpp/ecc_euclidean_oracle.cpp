// Independent direct-OpenCV Euclidean ECC oracle: links OpenCV only; no Video shim and
// no Core bridge. Writes obj/oracle/ecc-euclidean.txt:
//   ecc-euclidean-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> <OK|EXC> w0 w1 w2 w3 w4 w5 <tx> <ty> <theta>
//           <rho> <template-sum> <moving-sum>
//   end <fixture count>
// w0..w5 are the six native Float32 warp entries (row-major 2x3); theta is
// atan2(w3, w0). Floating values use %.17e; EXC lines carry zeros.
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
    std::function<cv::Mat()> make_template, make_moving;
};

std::vector<Fixture> fixtures() {
    auto tex = [] { return texture(N, N); };
    auto rigid = [](double deg, double tx, double ty) {
        return [=] { return rigid_texture(N, N, deg, tx, ty); };
    };
    // Rotation about the image centre plus a centre displacement (1.5, -1): the matrix
    // translation is derived from cx, cy and theta.
    const double cx = (N - 1) / 2.0, cy = (N - 1) / 2.0, t3 = 3.0 * kPi / 180.0;
    const double ctx = cx - (std::cos(t3) * cx - std::sin(t3) * cy) + 1.5;
    const double cty = cy - (std::sin(t3) * cx + std::cos(t3) * cy) - 1.0;
    std::vector<Fixture> f;
    auto add = [&](const char *name, std::function<cv::Mat()> a, std::function<cv::Mat()> b,
                   int iterations = 50, double epsilon = 1e-4) {
        f.push_back({name, iterations, epsilon, a, b});
    };
    add("identity", tex, tex);
    add("translation-2-1", tex, rigid(0, 2, 1));
    add("rot+1", tex, rigid(1, 0, 0));
    add("rot-1", tex, rigid(-1, 0, 0));
    add("rot+3", tex, rigid(3, 0, 0));
    add("rot-3", tex, rigid(-3, 0, 0));
    add("rot+3-t+2+1", tex, rigid(3, 2, 1));
    add("rot+3-t-2-1", tex, rigid(3, -2, -1));
    add("rot-3-t+2-1", tex, rigid(-3, 2, -1));
    add("centred-rot+3", tex, [=] { return rigid_texture(N, N, 3.0, ctx, cty); });
    add("reversed-rot+3", rigid(3, 2, 1), tex);
    add("photometric", tex,
        [=] { return affine_int(rigid_texture(N, N, 3, 2, 1), 4, 5, 20); });
    add("constant", [] { return constant(N, N, 128); }, [] { return constant(N, N, 128); });
    add("noise-vs-texture", tex, [] { return noise(N, N, 7); });
    add("noise-vs-noise", [] { return noise(N, N, 3); }, [] { return noise(N, N, 9); });
    add("rot+10-limit", tex, rigid(10, 0, 0));
    add("rot+15-limit", tex, rigid(15, 0, 0));
    add("rot+25-outside", tex, rigid(25, 0, 0));
    add("it1", tex, rigid(3, 2, 1), 1, 1e-4);
    add("it2", tex, rigid(3, 2, 1), 2, 1e-4);
    add("it100-e8", tex, rigid(3, 2, 1), 100, 1e-8);
    add("e0.1", tex, rigid(3, 2, 1), 50, 0.1);
    add("e1e-8", tex, rigid(3, 2, 1), 50, 1e-8);
    // Region fixture: 160x160 parents, 128x128 window at (16,16) (non-contiguous).
    add("region-both", [] { return texture(160, 160); }, [] { return rigid_texture(160, 160, 3, 2, 1); });
    return f;
}

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-euclidean-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-euclidean-oracle 1\nversion " << CV_VERSION << '\n';
    int count = 0;
    char buffer[1024];
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
            cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
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
                "fixture %s %d %.17e %s %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %lld %lld\n",
                f.name.c_str(), f.iterations, f.epsilon, ok ? "OK" : "EXC", w[0], w[1], w[2], w[3],
                w[4], w[5], w[2], w[5], theta, ok ? rho : 0.0, (long long)checksum(a),
                (long long)checksum(b));
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
    std::printf("PASS: ECC Euclidean oracle OpenCV %s, %d fixtures\n", CV_VERSION, count);
    return 0;
}
