// Independent direct-OpenCV seeded affine ECC oracle: links OpenCV only; no Video shim, no
// Core bridge, no production result conversion. Writes obj/oracle/ecc-affine-seeded.txt:
//   ecc-affine-seeded-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> s00 s01 s02 s10 s11 s12 <OK|EXC>
//           m00 m01 m02 m10 m11 m12 <rho> <template-sum> <moving-sum>
//   end <fixture count>
// s.. are the six initial Float32 warp entries and m.. the six returned entries, both in
// native matrix storage order (row-major), printed with %.17e. EXC lines carry zeros.
// Image kind: 'T' template = texture, moving = affine_texture(truth); 'C' constant pair;
// 'N' independent noise pair; 'R' 160x160 parents with a 128x128 non-contiguous window.
#include "ecc_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
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
    const char *name;
    char kind;
    double truth[6];  // a b tx c d ty (row-major warp, template -> moving)
    float seed[6];    // m00 m01 m02 m10 m11 m12
    int iterations;
    double epsilon;
};

#define ID {1, 0, 0, 0, 1, 0}
#define SMALL {1.01, 0.012, 2, -0.008, 0.99, 1}
#define SHEAR {1.00, 0.12, 1, -0.05, 1.00, -1}
#define SCALE {1.12, 0.02, 2, -0.01, 0.88, 1}
#define COMB {1.08, 0.08, 2, -0.04, 0.94, -1}
#define SHEAR2 {1, 0.2, 0, 0, 1, 0}
#define STRONG {1.2, 0.1, 5, -0.1, 0.8, -3}
#define REFL {-1, 0, 127, 0, 1, 0}
#define NEAR_COMB {1.06f, 0.07f, 1.7f, -0.03f, 0.95f, -0.8f}
#define WRONG_A {0.8f, -0.2f, 6.0f, 0.2f, 1.25f, -5.0f}
#define ASYM {1.0f, 0.3f, 4.0f, -0.25f, 1.0f, -2.0f}
#define IDF {1, 0, 0, 0, 1, 0}

const std::vector<Fixture> &fixtures() {
    static const std::vector<Fixture> f = {
        {"identity-id", 'T', ID, IDF, 50, 1e-4},
        {"translation-id", 'T', {1, 0, 2, 0, 1, 1}, IDF, 50, 1e-4},
        {"small-id", 'T', SMALL, IDF, 50, 1e-4},
        {"shear-id", 'T', SHEAR, IDF, 50, 1e-4},
        {"shear02-id", 'T', SHEAR2, IDF, 50, 1e-4},
        {"reflect-id", 'T', REFL, IDF, 50, 1e-4},
        {"shear-exact", 'T', SHEAR, {1.00f, 0.12f, 1.0f, -0.05f, 1.00f, -1.0f}, 50, 1e-4},
        {"shear-near", 'T', SHEAR, {1.00f, 0.10f, 0.5f, -0.04f, 1.00f, -0.7f}, 50, 1e-4},
        {"scale-near", 'T', SCALE, {1.10f, 0.01f, 1.5f, 0.0f, 0.90f, 0.8f}, 50, 1e-4},
        {"combined-near", 'T', COMB, NEAR_COMB, 50, 1e-4},
        {"reflect-near", 'T', REFL, {-0.98f, 0.01f, 126.0f, 0.0f, 1.0f, 0.1f}, 50, 1e-4},
        {"shear02-near", 'T', SHEAR2, {1.0f, 0.18f, 0.0f, 0.0f, 1.0f, 0.0f}, 50, 1e-4},
        {"strong-near", 'T', STRONG, {1.18f, 0.09f, 4.5f, -0.09f, 0.82f, -2.7f}, 50, 1e-4},
        {"translation-near", 'T', {1, 0, 2, 0, 1, 1}, {1.0f, 0.0f, 1.5f, 0.0f, 1.0f, 0.5f}, 50, 1e-4},
        {"small-near", 'T', SMALL, {1.0f, 0.01f, 1.5f, -0.005f, 1.0f, 0.5f}, 50, 1e-4},
        {"shear-wrong", 'T', SHEAR, WRONG_A, 50, 1e-4},
        {"scale-asym", 'T', SCALE, ASYM, 50, 1e-4},
        {"combined-asym", 'T', COMB, ASYM, 50, 1e-4},
        {"shear02-asym", 'T', SHEAR2, ASYM, 50, 1e-4},
        {"limit-linear", 'T', SMALL, {4, 4, 0, 4, 4, 0}, 50, 1e-4},
        {"limit-translation", 'T', SMALL, {1, 0, 1048576.0f, 0, 1, -1048576.0f}, 50, 1e-4},
        {"all-limits", 'T', SMALL, {4, -4, 1048576.0f, -4, 4, -1048576.0f}, 50, 1e-4},
        {"singular-zero", 'T', SMALL, {0, 0, 0, 0, 0, 0}, 50, 1e-4},
        {"singular-rank1", 'T', SMALL, {1, 1, 0, 1, 1, 0}, 50, 1e-4},
        {"near-singular", 'T', SMALL, {1, 1, 0, 1, 1.0000001f, 0}, 50, 1e-4},
        {"reflect-seed", 'T', SMALL, {-1, 0, 0, 0, 1, 0}, 50, 1e-4},
        {"aniso-seed", 'T', SMALL, {4, 0, 0, 0, 0.01f, 0}, 50, 1e-4},
        {"translation-1e5", 'T', SMALL, {1, 0, 100000.0f, 0, 1, 100000.0f}, 50, 1e-4},
        {"combined-near-it1", 'T', COMB, NEAR_COMB, 1, 1e-4},
        {"combined-near-it3", 'T', COMB, NEAR_COMB, 3, 1e-4},
        {"combined-near-e0.1", 'T', COMB, NEAR_COMB, 50, 0.1},
        {"constant-seeded", 'C', ID, {1.01f, 0.01f, 1.0f, 0.0f, 0.99f, 1.0f}, 50, 1e-4},
        {"noise-seeded", 'N', ID, {1.01f, 0.01f, 1.0f, 0.0f, 0.99f, 1.0f}, 50, 1e-4},
        {"region-near", 'R', SMALL, {1.00f, 0.01f, 1.5f, 0.0f, 1.0f, 0.5f}, 50, 1e-4},
    };
    return f;
}

void make_pair(const Fixture &f, cv::Mat &a, cv::Mat &b) {
    switch (f.kind) {
    case 'C': a = constant(N, N, 128); b = constant(N, N, 128); break;
    case 'N': a = noise(N, N, 3); b = noise(N, N, 9); break;
    case 'R':
        a = texture(160, 160)(cv::Rect(16, 16, N, N));
        b = affine_texture(160, 160, f.truth[0], f.truth[1], f.truth[3], f.truth[4], f.truth[2],
                           f.truth[5])(cv::Rect(16, 16, N, N));
        break;
    default:
        a = texture(N, N);
        b = affine_texture(N, N, f.truth[0], f.truth[1], f.truth[3], f.truth[4], f.truth[2],
                           f.truth[5]);
    }
}
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-affine-seeded-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-affine-seeded-oracle 1\nversion " << CV_VERSION << '\n';
    int count = 0;
    char buffer[2048];
    try {
        for (const auto &f : fixtures()) {
            cv::Mat a, b;
            make_pair(f, a, b);
            if (f.kind == 'R' && (a.isContinuous() || b.isContinuous())) {
                std::fprintf(stderr, "oracle: Region fixture is contiguous\n");
                return 3;
            }
            cv::Mat warp(2, 3, CV_32F);
            for (int i = 0; i < 6; ++i) warp.ptr<float>()[i] = f.seed[i];
            bool ok = true;
            double rho = 0;
            try {
                rho = cv::findTransformECC(a, b, warp, cv::MOTION_AFFINE,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                     f.iterations, f.epsilon), cv::noArray(), 5);
            } catch (const cv::Exception &) { ok = false; }
            double w[6] = {0, 0, 0, 0, 0, 0};
            if (ok) {
                if (warp.dims != 2 || warp.rows != 2 || warp.cols != 3 ||
                    warp.type() != CV_32FC1 || !warp.isContinuous() || !std::isfinite(rho)) {
                    std::fprintf(stderr, "oracle: malformed native output for %s\n", f.name);
                    return 4;
                }
                for (int i = 0; i < 6; ++i) {
                    w[i] = double(warp.ptr<float>()[i]);
                    if (!std::isfinite(w[i])) {
                        std::fprintf(stderr, "oracle: nonfinite warp for %s\n", f.name);
                        return 4;
                    }
                }
            }
            std::snprintf(buffer, sizeof buffer,
                "fixture %s %d %.17e %.17e %.17e %.17e %.17e %.17e %.17e %s "
                "%.17e %.17e %.17e %.17e %.17e %.17e %.17e %lld %lld\n",
                f.name, f.iterations, f.epsilon, double(f.seed[0]), double(f.seed[1]),
                double(f.seed[2]), double(f.seed[3]), double(f.seed[4]), double(f.seed[5]),
                ok ? "OK" : "EXC", w[0], w[1], w[2], w[3], w[4], w[5], ok ? rho : 0.0,
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
    std::printf("PASS: ECC affine seeded oracle OpenCV %s, %d fixtures\n", CV_VERSION, count);
    return 0;
}

