// Independent direct-OpenCV homography ECC oracle (Task 016): links OpenCV only; no Video
// shim, no Core bridge, no production result conversion. Writes obj/oracle/ecc-homography.txt:
//   ecc-homography-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> <OK|EXC> h00 h01 h02 h10 h11 h12 h20 h21 h22
//           <rho> <det> <template-sum> <moving-sum>
//   end <fixture count>
// h00..h22 are the nine native Float32 warp entries in row-major STORAGE order
// (warp.ptr<float>()[0..8]); det is the 3x3 determinant evaluated in double (research-only
// conditioning diagnostic). Floating values use %.17e; EXC lines carry zeros.
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

// Ground-truth homographies (row-major), template -> moving.
const double kT21[9] = {1, 0, 2, 0, 1, 1, 0, 0, 1};
const double kTm[9] = {1, 0, -2, 0, 1, -1, 0, 0, 1};
const double kR2[9] = {0.99939083, -0.0348995, 0, 0.0348995, 0.99939083, 0, 0, 0, 1};
const double kRm2[9] = {0.99939083, 0.0348995, 0, -0.0348995, 0.99939083, 0, 0, 0, 1};
const double kS102[9] = {1.02, 0, 0, 0, 1.02, 0, 0, 0, 1};
const double kS098[9] = {0.98, 0, 0, 0, 0.98, 0, 0, 0, 1};
const double kAN[9] = {1.03, 0, 0, 0, 0.98, 0, 0, 0, 1};
const double kSHX[9] = {1, 0.02, 0, 0, 1, 0, 0, 0, 1};
const double kSHY[9] = {1, 0, 0, -0.01, 1, 0, 0, 0, 1};
const double kAC[9] = {1.01, 0.01, 1.5, -0.01, 0.99, -1, 0, 0, 1};
const double kP20p[9] = {1, 0, 0, 0, 1, 0, 0.0002, 0, 1};
const double kP20n[9] = {1, 0, 0, 0, 1, 0, -0.0002, 0, 1};
const double kP21p[9] = {1, 0, 0, 0, 1, 0, 0, 0.0002, 1};
const double kP21n[9] = {1, 0, 0, 0, 1, 0, 0, -0.0002, 1};
const double kPB[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
const double kPM[9] = {1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0};
const double kRP[9] = {0.9994, -0.0349, 1.0, 0.0349, 0.9994, 0.5, 0.0001, 0.00005, 1};
const double kSP[9] = {1.02, 0.01, 1.0, -0.01, 0.98, -0.5, 0.0001, -0.0001, 1};
const double kStrong[9] = {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1};
const double kNZ[9] = {1, 0, 0, 0, 1, 0, -0.0078, 0, 1};
const double kCZ[9] = {1, 0, 0, 0, 1, 0, -0.012, 0, 1};
const double kIC[9] = {1, 0, 0, 0, 1.0e-6, 0, 0, 0, 1};
const double kLU[9] = {1, 0, 40, 0, 1, 30, 0, 0, 1};
const double kRX[9] = {-1, 0, 127, 0, 1, 0, 0, 0, 1};

std::vector<Fixture> fixtures() {
    auto tex = [] { return texture(N, N); };
    auto hom = [](const double *h) { return [=] { return homography_texture(N, N, h); }; };
    std::vector<Fixture> f;
    auto add = [&](const char *name, std::function<cv::Mat()> a, std::function<cv::Mat()> b,
                   int iterations = 50, double epsilon = 1e-4) {
        f.push_back({name, iterations, epsilon, a, b});
    };
    add("identity", tex, tex);
    add("translation-2-1", tex, hom(kT21));
    add("translation-m2-m1", tex, hom(kTm));
    add("rot+2", tex, hom(kR2));
    add("rot-2", tex, hom(kRm2));
    add("iso-1.02", tex, hom(kS102));
    add("iso-0.98", tex, hom(kS098));
    add("aniso-1.03-0.98", tex, hom(kAN));
    add("shear-x+0.02", tex, hom(kSHX));
    add("shear-y-0.01", tex, hom(kSHY));
    add("affine-combo", tex, hom(kAC));
    add("h20-pos", tex, hom(kP20p));
    add("h20-neg", tex, hom(kP20n));
    add("h21-pos", tex, hom(kP21p));
    add("h21-neg", tex, hom(kP21n));
    add("h20-h21-both", tex, hom(kPB));
    add("persp-mild", tex, hom(kPM));
    add("rot-persp", tex, hom(kRP));
    add("sst-persp", tex, hom(kSP));
    add("reversed-persp-both", hom(kPB), tex);
    add("photometric-persp-both", tex,
        [] { return affine_int(homography_texture(N, N, kPB), 4, 5, 20); });
    add("it1", tex, hom(kPB), 1, 1e-4);
    add("it3", tex, hom(kPB), 3, 1e-4);
    add("it100-e8", tex, hom(kPB), 100, 1e-8);
    add("e0.1", tex, hom(kPB), 50, 0.1);
    add("e1e-8", tex, hom(kPB), 50, 1e-8);
    // Region fixture: 160x160 parents, 128x128 window at (16,16) (non-contiguous).
    add("region-persp-mild", [] { return texture(160, 160); },
        [] { return homography_texture(160, 160, kPM); });
    add("constant", [] { return constant(N, N, 128); }, [] { return constant(N, N, 128); });
    add("lowgrad", [] { return affine_int(texture(N, N), 1, 40, 126); },
        [] { return affine_int(homography_texture(N, N, kPB), 1, 40, 126); });
    add("noise-vs-texture", tex, [] { return noise(N, N, 7); });
    add("noise-vs-noise", [] { return noise(N, N, 3); }, [] { return noise(N, N, 9); });
    add("persp-strong", tex, hom(kStrong));
    add("persp-near-zero-den", tex, hom(kNZ));
    add("persp-den-crosses-zero", tex, hom(kCZ));
    add("illcond", tex, hom(kIC));
    add("large-translation", tex, hom(kLU));
    add("reflect-x", tex, hom(kRX));
    return f;
}

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-homography-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-homography-oracle 1\nversion " << CV_VERSION << '\n';
    int count = 0, ok_count = 0;
    char buffer[2048];
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
            cv::Mat warp = cv::Mat::eye(3, 3, CV_32F);
            bool ok = true;
            double rho = 0;
            try {
                rho = cv::findTransformECC(a, b, warp, cv::MOTION_HOMOGRAPHY,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                     f.iterations, f.epsilon), cv::noArray(), 5);
            } catch (const cv::Exception &) { ok = false; }
            double w[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            double det = 0;
            if (ok) {
                if (warp.dims != 2 || warp.rows != 3 || warp.cols != 3 || warp.type() != CV_32FC1 ||
                    !warp.isContinuous() || !std::isfinite(rho)) {
                    std::fprintf(stderr, "oracle: malformed native output for %s\n", f.name.c_str());
                    return 4;
                }
                for (int i = 0; i < 9; ++i) {
                    w[i] = double(warp.ptr<float>()[i]);
                    if (!std::isfinite(w[i])) {
                        std::fprintf(stderr, "oracle: nonfinite warp for %s\n", f.name.c_str());
                        return 4;
                    }
                }
                det = w[0] * (w[4] * w[8] - w[5] * w[7]) - w[1] * (w[3] * w[8] - w[5] * w[6]) +
                      w[2] * (w[3] * w[7] - w[4] * w[6]);
                ++ok_count;
            }
            std::snprintf(buffer, sizeof buffer,
                "fixture %s %d %.17e %s %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %lld %lld\n",
                f.name.c_str(), f.iterations, f.epsilon, ok ? "OK" : "EXC", w[0], w[1], w[2], w[3],
                w[4], w[5], w[6], w[7], w[8], ok ? rho : 0.0, det,
                (long long)checksum(a), (long long)checksum(b));
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
    std::printf("PASS: ECC homography oracle OpenCV %s, %d fixtures (%d native successes, %d exceptions)\n",
                CV_VERSION, count, ok_count, count - ok_count);
    return 0;
}
