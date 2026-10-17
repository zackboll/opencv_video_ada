// Independent direct-OpenCV seeded homography ECC oracle (Task 017): links OpenCV only; no
// Video shim, no Core bridge, no production result conversion, no binding seed validation.
// Writes obj/oracle/ecc-homography-seeded.txt:
//   ecc-homography-seeded-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> s00 s01 s02 s10 s11 s12 s20 s21 s22 <OK|EXC>
//           h00 h01 h02 h10 h11 h12 h20 h21 h22 <rho> <template-sum> <moving-sum>
//   end <fixture count>
// s.. are the nine initial Float32 warp entries and h.. the nine returned entries, both in
// native matrix STORAGE order (row-major, warp.ptr<float>()[0..8]), printed with %.17e. EXC
// EXC lines carry zeros. Every seed is inside the documented binding seed domain
// (docs/ecc-homography-seeded-source-contract.md), checked here independently (exit 6);
// out-of-domain seeds never reach native code through the binding and are therefore not
// oracle fixtures. Note that the STRONG truth itself is outside the domain on 128x128
// (0.002*127*2 = 0.508), so no "strong-exact" fixture exists.
// Image kind: 'T' template = texture, moving = homography_texture(truth); 'C' constant pair;
// 'N' independent noise pair; 'R' 160x160 parents with a 128x128 non-contiguous window.
#include "ecc_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
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
    const char *name;
    char kind;
    double truth[9];  // row-major, template -> moving
    float seed[9];    // h00 h01 h02 h10 h11 h12 h20 h21 h22
    int iterations;
    double epsilon;
};

// Truths. The values are mirrored literally in tests/src/video_tests.adb (Hs_* constants).
#define ID {1, 0, 0, 0, 1, 0, 0, 0, 1}
#define PB {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1}
#define STRONG {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1}
#define LARGE {1, 0, 40, 0, 1, 30, 0, 0, 1}
#define LTP {1, 0, 12, 0, 1, -9, 0.0005, -0.0004, 1}
#define REFL {-1, 0, 127, 0, 1, 0, 0, 0, 1}
#define SCP {1.15, 0.02, -4, -0.01, 1.12, 3, 0.0008, -0.0006, 1}
#define SHP {1, 0.15, -3, 0.05, 1, 2, -0.0007, 0.0009, 1}
#define MILD {1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0}
// Seeds (Float32, storage order).
#define IDF {1, 0, 0, 0, 1, 0, 0, 0, 1}
#define STRONG_NEAR {1, 0, 0, 0, 1, 0, 0.0018f, 0.0018f, 1}
#define LARGE_NEAR {1, 0, 38, 0, 1, 28, 0, 0, 1}
#define LTP_NEAR {1, 0, 11, 0, 1, -8, 0.0004f, -0.0003f, 1}
#define REFL_NEAR {-0.98f, 0.01f, 126, 0, 1, 0.1f, 0, 0, 1}
#define SCP_NEAR {1.12f, 0.01f, -3, 0, 1.10f, 2.5f, 0.0006f, -0.0005f, 1}
#define SHP_NEAR {1, 0.13f, -2.5f, 0.04f, 1, 1.6f, -0.0006f, 0.0008f, 1}
#define MILD_NEAR {1, 0.01f, 1, 0, 1, 0.5f, 0.00004f, -0.00003f, 1}

const std::vector<Fixture> &fixtures() {
    static const std::vector<Fixture> f = {
        // Identity seeds: must equal the identity-initialized Task 016 call.
        {"identity-id", 'T', ID, IDF, 50, 1e-4},
        {"pb-id", 'T', PB, IDF, 50, 1e-4},
        {"strong-id", 'T', STRONG, IDF, 50, 1e-4},
        {"large-id", 'T', LARGE, IDF, 50, 1e-4},
        {"reflect-id", 'T', REFL, IDF, 50, 1e-4},
        {"scale-persp-id", 'T', SCP, IDF, 50, 1e-4},
        {"shear-persp-id", 'T', SHP, IDF, 50, 1e-4},
        // Near seeds: identity raises, seeded converges (measured on all four versions).
        {"strong-near", 'T', STRONG, STRONG_NEAR, 50, 1e-4},
        {"large-near", 'T', LARGE, LARGE_NEAR, 50, 1e-4},
        {"large-persp-near", 'T', LTP, LTP_NEAR, 50, 1e-4},
        {"reflect-near", 'T', REFL, REFL_NEAR, 50, 1e-4},
        {"scale-persp-near", 'T', SCP, SCP_NEAR, 50, 1e-4},
        {"shear-persp-near", 'T', SHP, SHP_NEAR, 50, 1e-4},
        {"mild-near", 'T', MILD, MILD_NEAR, 50, 1e-4},
        {"large-exact", 'T', LARGE, {1, 0, 40, 0, 1, 30, 0, 0, 1}, 50, 1e-4},
        // Seeds with wrong projective direction or insufficient proximity.
        {"pb-wrong-persp", 'T', PB, {1, 0, 0, 0, 1, 0, -0.0019f, 0.0019f, 1}, 50, 1e-4},
        {"strong-wrong-persp", 'T', STRONG, {1, 0, 0, 0, 1, 0, -0.0019f, 0.0019f, 1}, 50, 1e-4},
        {"strong-loose", 'T', STRONG, {1, 0, 0, 0, 1, 0, 0.0015f, 0.0015f, 1}, 50, 1e-4},
        {"pb-wrong-affine", 'T', PB, {0.8f, -0.2f, 6, 0.2f, 1.25f, -5, 0.001f, -0.001f, 1}, 50, 1e-4},
        // Domain edges: every seed passes the binding domain; native outcome is recorded.
        {"den-edge-x", 'T', PB, {1, 0, 0, 0, 1, 0, 0.0039f, 0, 1}, 50, 1e-4},
        {"den-edge-mixed", 'T', PB, {1, 0, 0, 0, 1, 0, 0.00195f, -0.00195f, 1}, 50, 1e-4},
        {"limit-linear", 'T', PB, {4, 4, 0, 4, 4, 0, 0, 0, 1}, 50, 1e-4},
        {"limit-translation", 'T', PB, {1, 0, 1048576, 0, 1, -1048576, 0, 0, 1}, 50, 1e-4},
        {"all-limits", 'T', PB, {4, -4, 1048576, -4, 4, -1048576, 0.00195f, -0.00195f, 1}, 50, 1e-4},
        {"singular-zero", 'T', PB, {0, 0, 0, 0, 0, 0, 0, 0, 1}, 50, 1e-4},
        {"tiny-scale", 'T', PB, {1e-6f, 0, 0, 0, 1e-6f, 0, 0, 0, 1}, 50, 1e-4},
        {"aniso-seed", 'T', PB, {4, 0, 0, 0, 0.01f, 0, 0, 0, 1}, 50, 1e-4},
        // Options.
        {"strong-near-it1", 'T', STRONG, STRONG_NEAR, 1, 1e-4},
        {"strong-near-it3", 'T', STRONG, STRONG_NEAR, 3, 1e-4},
        {"strong-near-e0.1", 'T', STRONG, STRONG_NEAR, 50, 0.1},
        {"strong-near-it100-e8", 'T', STRONG, STRONG_NEAR, 100, 1e-8},
        {"pb-wrong-persp-it1", 'T', PB, {1, 0, 0, 0, 1, 0, -0.0019f, 0.0019f, 1}, 1, 1e-4},
        // Nonconvergence and Regions.
        {"constant-seeded", 'C', ID, MILD_NEAR, 50, 1e-4},
        {"noise-seeded", 'N', ID, MILD_NEAR, 50, 1e-4},
        {"region-near", 'R', MILD, MILD_NEAR, 50, 1e-4},
    };
    return f;
}

// Independent restatement of the binding seed domain (not the shim's code).
bool in_seed_domain(const float *s, int rows, int cols) {
    for (int i = 0; i < 9; ++i) if (!std::isfinite(s[i])) return false;
    if (s[8] != 1.0f) return false;
    for (int i : {0, 1, 3, 4}) if (std::fabs(double(s[i])) > 4.0) return false;
    for (int i : {2, 5}) if (std::fabs(double(s[i])) > 1048576.0) return false;
    return std::fabs(double(s[6])) * double(cols - 1) + std::fabs(double(s[7])) * double(rows - 1) <= 0.5;
}

void make_pair(const Fixture &f, cv::Mat &a, cv::Mat &b) {
    switch (f.kind) {
    case 'C': a = constant(N, N, 128); b = constant(N, N, 128); break;
    case 'N': a = noise(N, N, 3); b = noise(N, N, 9); break;
    case 'R':
        a = texture(160, 160)(cv::Rect(16, 16, N, N));
        b = homography_texture(160, 160, f.truth)(cv::Rect(16, 16, N, N));
        break;
    default:
        a = texture(N, N);
        b = (f.truth[0] == 1 && f.truth[1] == 0 && f.truth[2] == 0 && f.truth[3] == 0 &&
             f.truth[4] == 1 && f.truth[5] == 0 && f.truth[6] == 0 && f.truth[7] == 0)
                ? texture(N, N) : homography_texture(N, N, f.truth);
    }
}
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-homography-seeded-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-homography-seeded-oracle 1\nversion " << CV_VERSION << '\n';
    int count = 0, ok_count = 0;
    char buffer[2048];
    try {
        for (const auto &f : fixtures()) {
            cv::Mat a, b;
            make_pair(f, a, b);
            if (f.kind == 'R' && (a.isContinuous() || b.isContinuous())) {
                std::fprintf(stderr, "oracle: Region fixture is contiguous\n");
                return 3;
            }
            if (!in_seed_domain(f.seed, a.rows, a.cols)) {
                std::fprintf(stderr, "oracle: seed outside the binding seed domain for %s\n", f.name);
                return 6;
            }
            cv::Mat warp(3, 3, CV_32F);
            for (int i = 0; i < 9; ++i) warp.ptr<float>()[i] = f.seed[i];
            bool ok = true;
            double rho = 0;
            try {
                rho = cv::findTransformECC(a, b, warp, cv::MOTION_HOMOGRAPHY,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                     f.iterations, f.epsilon), cv::noArray(), 5);
            } catch (const cv::Exception &) { ok = false; }
            double w[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            if (ok) {
                if (warp.dims != 2 || warp.rows != 3 || warp.cols != 3 ||
                    warp.type() != CV_32FC1 || !warp.isContinuous() || !std::isfinite(rho)) {
                    std::fprintf(stderr, "oracle: malformed native output for %s\n", f.name);
                    return 4;
                }
                for (int i = 0; i < 9; ++i) {
                    w[i] = double(warp.ptr<float>()[i]);
                    if (!std::isfinite(w[i])) {
                        std::fprintf(stderr, "oracle: nonfinite warp for %s\n", f.name);
                        return 4;
                    }
                }
                ++ok_count;
            }
            std::snprintf(buffer, sizeof buffer,
                "fixture %s %d %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %s "
                "%.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %.17e %lld %lld\n",
                f.name, f.iterations, f.epsilon, double(f.seed[0]), double(f.seed[1]),
                double(f.seed[2]), double(f.seed[3]), double(f.seed[4]), double(f.seed[5]),
                double(f.seed[6]), double(f.seed[7]), double(f.seed[8]), ok ? "OK" : "EXC",
                w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8], ok ? rho : 0.0,
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
    std::printf("PASS: ECC homography seeded oracle OpenCV %s, %d fixtures (%d native successes, %d exceptions)\n",
                CV_VERSION, count, ok_count, count - ok_count);
    return 0;
}