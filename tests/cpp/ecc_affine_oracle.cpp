// Independent direct-OpenCV affine ECC oracle: links OpenCV only; no Video shim, no Core
// bridge, no production result conversion. Writes obj/oracle/ecc-affine.txt:
//   ecc-affine-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> <OK|EXC> m00 m01 m02 m10 m11 m12 <rho> <det>
//           <template-sum> <moving-sum>
//   end <fixture count>
// m00..m12 are the six native Float32 warp entries in row-major order (warp.ptr<float>()
// [0..5]); det = m00*m11 - m01*m10 evaluated in double. Floating values use %.17e; EXC
// lines carry zeros.
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
    auto aff = [](double a, double b, double c, double d, double tx, double ty) {
        return [=] { return affine_texture(N, N, a, b, c, d, tx, ty); };
    };
    auto rot = [&](double deg) {
        const double t = deg * kPi / 180.0;
        return aff(std::cos(t), -std::sin(t), std::sin(t), std::cos(t), 0, 0);
    };
    const double d2r = kPi / 180.0;
    std::vector<Fixture> f;
    auto add = [&](const char *name, std::function<cv::Mat()> a, std::function<cv::Mat()> b,
                   int iterations = 50, double epsilon = 1e-4) {
        f.push_back({name, iterations, epsilon, a, b});
    };
    add("identity", tex, tex);
    add("translation-2-1", tex, aff(1, 0, 0, 1, 2, 1));
    add("translation-m2-m1", tex, aff(1, 0, 0, 1, -2, -1));
    add("rot+1", tex, rot(1));
    add("rot-1", tex, rot(-1));
    add("rot+3", tex, rot(3));
    add("rot-3", tex, rot(-3));
    add("iso-1.02", tex, aff(1.02, 0, 0, 1.02, 0, 0));
    add("iso-0.98", tex, aff(0.98, 0, 0, 0.98, 0, 0));
    add("aniso-1.01-0.99", tex, aff(1.01, 0, 0, 0.99, 0, 0));
    add("aniso-1.03-0.97", tex, aff(1.03, 0, 0, 0.97, 0, 0));
    add("shear-x+0.012", tex, aff(1, 0.012, 0, 1, 0, 0));
    add("shear-x+0.03", tex, aff(1, 0.03, 0, 1, 0, 0));
    add("shear-y-0.008", tex, aff(1, 0, -0.008, 1, 0, 0));
    add("shear-both", tex, aff(1, 0.012, -0.008, 1, 0, 0));
    add("combined", tex, aff(1.01, 0.012, -0.008, 0.99, 2, 1));
    add("combined-neg", tex, aff(0.99, -0.012, 0.008, 1.01, -2, -1));
    add("rot-scale-shear-t", tex,
        aff(1.02 * std::cos(2 * d2r), -0.98 * std::sin(2 * d2r) + 0.01, 1.02 * std::sin(2 * d2r),
            0.98 * std::cos(2 * d2r), 1.5, -1));
    add("reflect-x", tex, aff(-1, 0, 0, 1, 0, 0));
    add("shear-0.2", tex, aff(1, 0.2, 0, 1, 0, 0));
    add("strong-1.2-0.8", tex, aff(1.2, 0.1, -0.1, 0.8, 5, -3));
    add("reversed-combined", aff(1.01, 0.012, -0.008, 0.99, 2, 1), tex);
    add("photometric", tex,
        [=] { return affine_int(affine_texture(N, N, 1.01, 0.012, -0.008, 0.99, 2, 1), 4, 5, 20); });
    add("constant", [] { return constant(N, N, 128); }, [] { return constant(N, N, 128); });
    add("noise-vs-texture", tex, [] { return noise(N, N, 7); });
    add("noise-vs-noise", [] { return noise(N, N, 3); }, [] { return noise(N, N, 9); });
    add("it1", tex, aff(1.01, 0.012, -0.008, 0.99, 2, 1), 1, 1e-4);
    add("it3", tex, aff(1.01, 0.012, -0.008, 0.99, 2, 1), 3, 1e-4);
    add("it100-e8", tex, aff(1.01, 0.012, -0.008, 0.99, 2, 1), 100, 1e-8);
    add("e0.1", tex, aff(1.01, 0.012, -0.008, 0.99, 2, 1), 50, 0.1);
    add("e1e-8", tex, aff(1.01, 0.012, -0.008, 0.99, 2, 1), 50, 1e-8);
    // Region fixture: 160x160 parents, 128x128 window at (16,16) (non-contiguous).
    add("region-both", [] { return texture(160, 160); },
        [] { return affine_texture(160, 160, 1.01, 0.012, -0.008, 0.99, 2, 1); });
    return f;
}

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-affine-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-affine-oracle 1\nversion " << CV_VERSION << '\n';
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
                rho = cv::findTransformECC(a, b, warp, cv::MOTION_AFFINE,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                     f.iterations, f.epsilon), cv::noArray(), 5);
            } catch (const cv::Exception &) { ok = false; }
            double w[6] = {0, 0, 0, 0, 0, 0};
            double det = 0;
            if (ok) {
                if (warp.dims != 2 || warp.rows != 2 || warp.cols != 3 || warp.type() != CV_32FC1 ||
                    !warp.isContinuous() || !std::isfinite(rho)) {
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
                det = w[0] * w[4] - w[1] * w[3];
            }
            std::snprintf(buffer, sizeof buffer,
                "fixture %s %d %.17e %s %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %lld %lld\n",
                f.name.c_str(), f.iterations, f.epsilon, ok ? "OK" : "EXC", w[0], w[1], w[2], w[3],
                w[4], w[5], ok ? rho : 0.0, det, (long long)checksum(a), (long long)checksum(b));
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
    std::printf("PASS: ECC affine oracle OpenCV %s, %d fixtures\n", CV_VERSION, count);
    return 0;
}
