// Independent direct-OpenCV masked homography ECC oracle (Task 018): links OpenCV only; no Video
// shim, no Core bridge, no binding mask or seed validation. It builds the images, the MOVING-image
// masks and the initial warps itself and calls the seven-argument cv::findTransformECC directly.
// Writes obj/oracle/ecc-homography-masked.txt:
//   ecc-homography-masked-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> s00..s22 <OK|EXC> h00..h22 <rho> <template-sum>
//           <moving-sum> <mask-sum> <mask-nonzero> <unmasked-equal> <error-class>
//   end <fixture count>
// s.. (initial warp) and h.. (returned warp) are Float32 bit patterns (%08x) in native matrix
// STORAGE order; rho is the Float64 bit pattern (%016llx). EXC lines carry zero bits.
// unmasked-equal is 1 when the same call with cv::noArray() gives the same outcome and, on success,
// bitwise identical coefficients and correlation; otherwise 0. error-class is NONE, NAN ("NaN
// encountered"), LAMBDA (lambda_d <= 0) or OTHER.
// Image kinds: 'T' template = texture, moving = homography_texture(truth) (texture for identity);
// 'O' as 'T' with a moving-image rectangle replaced by occluding content; 'C' constant pair;
// 'N' independent noise; 'K' texture pair with a constant 50x50 patch; 'R' 160x160 parents with
// 128x128 non-contiguous windows (images AND mask).
// Mask kinds (128x128, MOVING pixels, 255 unless stated): 'F' full value v; 'H' full value v
// with rectangle (a,b,c,d) set to 0; 'A' arbitrary nonzero 1..255 (LCG seed v) on 'H' support
// (c = 0: full); 'M' three disjoint holes; 'P' single pixel (a,b); 'G' grid spacing a; 'S' row
// strip rows a..a+b-1; 'B' block (a,b) size c; 'D' LCG density a/100; 'L' columns 0..a-1;
// 'Z' all zero; 'Q' the 50x50 patch interior only. Mirrored in tests/src/video_tests.adb (Hm_*).
#include "ecc_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

struct MaskSpec { char kind; int a, b, c, d, v; };
struct Occlusion { int x, y, w, h, kind; };  // kind 0 block 250, 1 noise, 2 shifted texture, 3 stripes

struct Fixture {
    const char *name;
    char image;
    double truth[9];
    Occlusion occ;
    float seed[9];
    MaskSpec mask;
    int iterations;
    double epsilon;
};

#define ID {1, 0, 0, 0, 1, 0, 0, 0, 1}
#define T21 {1, 0, 2, 0, 1, 1, 0, 0, 1}
#define PM {1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0}
#define LTP {1, 0, 12, 0, 1, -9, 0.0005, -0.0004, 1}
#define SCP {1.15, 0.02, -4, -0.01, 1.12, 3, 0.0008, -0.0006, 1}
#define STRONG {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1}
#define IDF {1, 0, 0, 0, 1, 0, 0, 0, 1}
#define LTP_NEAR {1, 0, 11, 0, 1, -8, 0.0004f, -0.0003f, 1}
#define SCP_NEAR {1.12f, 0.01f, -3, 0, 1.10f, 2.5f, 0.0006f, -0.0005f, 1}
#define STRONG_NEAR {1, 0, 0, 0, 1, 0, 0.0018f, 0.0018f, 1}
#define NOOCC {0, 0, 0, 0, 0}
#define OCC_T(k) {64, 8, 56, 48, k}
#define OCC_P(k) {60, 4, 64, 52, k}
#define FULL(v) {'F', 0, 0, 0, 0, v}
#define HOLE(x, y, w, h) {'H', x, y, w, h, 255}

const std::vector<Fixture> &fixtures() {
    static const std::vector<Fixture> f = {
        // Mask value semantics (identity start); all-included masks must equal noArray().
        {"identity-full", 'T', ID, NOOCC, IDF, FULL(255), 50, 1e-4},
        {"trans-full255", 'T', T21, NOOCC, IDF, FULL(255), 50, 1e-4},
        {"trans-full1", 'T', T21, NOOCC, IDF, FULL(1), 50, 1e-4},
        {"trans-full7", 'T', T21, NOOCC, IDF, FULL(7), 50, 1e-4},
        {"trans-full-arbitrary", 'T', T21, NOOCC, IDF, {'A', 0, 0, 0, 0, 99}, 50, 1e-4},
        {"trans-hole255", 'T', T21, NOOCC, IDF, HOLE(70, 10, 40, 30), 50, 1e-4},
        {"trans-hole1", 'T', T21, NOOCC, IDF, {'H', 70, 10, 40, 30, 1}, 50, 1e-4},
        {"trans-hole-arbitrary", 'T', T21, NOOCC, IDF, {'A', 70, 10, 40, 30, 5}, 50, 1e-4},
        {"trans-hole-transposed", 'T', T21, NOOCC, IDF, HOLE(10, 70, 30, 40), 50, 1e-4},
        {"trans-multi-holes", 'T', T21, NOOCC, IDF, {'M', 0, 0, 0, 0, 255}, 50, 1e-4},
        {"mild-full", 'T', PM, NOOCC, IDF, FULL(255), 50, 1e-4},
        {"mild-hole", 'T', PM, NOOCC, IDF, HOLE(70, 10, 40, 30), 50, 1e-4},
        // Occlusion, identity start: the mask excludes the occluder grown by 3 pixels.
        {"occ-trans-shifted-full", 'O', T21, OCC_T(2), IDF, FULL(255), 50, 1e-4},
        {"occ-trans-shifted-masked", 'O', T21, OCC_T(2), IDF, HOLE(61, 5, 62, 54), 50, 1e-4},
        {"occ-trans-shifted-transp", 'O', T21, OCC_T(2), IDF, HOLE(5, 61, 54, 62), 50, 1e-4},
        {"occ-trans-block-full", 'O', T21, OCC_T(0), IDF, FULL(255), 50, 1e-4},
        {"occ-trans-block-masked", 'O', T21, OCC_T(0), IDF, HOLE(61, 5, 62, 54), 50, 1e-4},
        {"occ-mild-shifted-full", 'O', PM, OCC_P(2), IDF, FULL(255), 50, 1e-4},
        {"occ-mild-shifted-masked", 'O', PM, OCC_P(2), IDF, HOLE(57, 1, 70, 58), 50, 1e-4},
        {"occ-mild-noise-masked", 'O', PM, OCC_P(1), IDF, HOLE(57, 1, 70, 58), 50, 1e-4},
        {"occ-mild-stripes-masked", 'O', PM, OCC_P(3), IDF, HOLE(57, 1, 70, 58), 50, 1e-4},
        // Caller-seeded.
        {"ltp-near-full", 'T', LTP, NOOCC, LTP_NEAR, FULL(255), 50, 1e-4},
        {"ltp-near-hole", 'T', LTP, NOOCC, LTP_NEAR, HOLE(70, 10, 40, 30), 50, 1e-4},
        {"ltp-id-hole", 'T', LTP, NOOCC, IDF, HOLE(70, 10, 40, 30), 50, 1e-4},
        {"occ-ltp-block-full", 'O', LTP, OCC_P(0), LTP_NEAR, FULL(255), 50, 1e-4},
        {"occ-ltp-block-masked", 'O', LTP, OCC_P(0), LTP_NEAR, HOLE(57, 1, 70, 58), 50, 1e-4},
        {"occ-ltp-block-tmplframe", 'O', LTP, OCC_P(0), LTP_NEAR, HOLE(45, 10, 70, 58), 50, 1e-4},
        {"occ-ltp-shifted-full", 'O', LTP, OCC_P(2), LTP_NEAR, FULL(255), 50, 1e-4},
        {"occ-ltp-shifted-masked", 'O', LTP, OCC_P(2), LTP_NEAR, HOLE(57, 1, 70, 58), 50, 1e-4},
        {"occ-scp-shifted-full", 'O', SCP, OCC_P(2), SCP_NEAR, FULL(255), 50, 1e-4},
        {"occ-scp-shifted-masked", 'O', SCP, OCC_P(2), SCP_NEAR, HOLE(57, 1, 70, 58), 50, 1e-4},
        {"scp-near-multi-holes", 'T', SCP, NOOCC, SCP_NEAR, {'M', 0, 0, 0, 0, 255}, 50, 1e-4},
        {"strong-near-hole", 'T', STRONG, NOOCC, STRONG_NEAR, HOLE(70, 10, 40, 30), 50, 1e-4},
        // Support: sparse, narrow, eroded away, non-overlapping, degenerate.
        {"all-zero", 'T', T21, NOOCC, IDF, {'Z', 0, 0, 0, 0, 0}, 50, 1e-4},
        {"single-pixel", 'T', T21, NOOCC, IDF, {'P', 64, 64, 0, 0, 255}, 50, 1e-4},
        {"grid-2", 'T', T21, NOOCC, IDF, {'G', 2, 0, 0, 0, 255}, 50, 1e-4},
        {"row-strip-4", 'T', T21, NOOCC, IDF, {'S', 60, 4, 0, 0, 255}, 50, 1e-4},
        {"row-strip-5", 'T', T21, NOOCC, IDF, {'S', 60, 5, 0, 0, 255}, 50, 1e-4},
        {"block-5", 'T', T21, NOOCC, IDF, {'B', 50, 40, 5, 0, 255}, 50, 1e-4},
        {"block-12", 'T', T21, NOOCC, IDF, {'B', 50, 40, 12, 0, 255}, 50, 1e-4},
        {"density-50", 'T', T21, NOOCC, IDF, {'D', 50, 0, 0, 0, 255}, 50, 1e-4},
        {"density-90", 'T', T21, NOOCC, IDF, {'D', 90, 0, 0, 0, 255}, 50, 1e-4},
        {"left-band-id", 'T', T21, NOOCC, IDF, {'L', 24, 0, 0, 0, 255}, 50, 1e-4},
        {"left-band-seed60", 'T', T21, NOOCC, {1, 0, 60, 0, 1, 0, 0, 0, 1}, {'L', 24, 0, 0, 0, 255}, 50, 1e-4},
        {"left-band-seed60-it1", 'T', T21, NOOCC, {1, 0, 60, 0, 1, 0, 0, 0, 1}, {'L', 24, 0, 0, 0, 255}, 1, 1e-4},
        {"constant-support", 'K', T21, NOOCC, IDF, {'Q', 0, 0, 0, 0, 255}, 50, 1e-4},
        {"constant-full", 'C', ID, NOOCC, IDF, FULL(255), 50, 1e-4},
        {"noise-hole", 'N', ID, NOOCC, IDF, HOLE(70, 10, 40, 30), 50, 1e-4},
        // Options and Regions.
        {"trans-hole-it1", 'T', T21, NOOCC, IDF, HOLE(70, 10, 40, 30), 1, 1e-4},
        {"trans-hole-it100-e8", 'T', T21, NOOCC, IDF, HOLE(70, 10, 40, 30), 100, 1e-8},
        {"trans-hole-e0.1", 'T', T21, NOOCC, IDF, HOLE(70, 10, 40, 30), 50, 0.1},
        {"region-mild-hole", 'R', PM, NOOCC, IDF, HOLE(70, 10, 40, 30), 50, 1e-4},
        {"region-ltp-near-hole", 'R', LTP, NOOCC, LTP_NEAR, HOLE(70, 10, 40, 30), 50, 1e-4},
    };
    return f;
}

bool is_identity(const double *t) {
    return t[0] == 1 && t[1] == 0 && t[2] == 0 && t[3] == 0 && t[4] == 1 && t[5] == 0 &&
           t[6] == 0 && t[7] == 0 && t[8] == 1;
}

cv::Mat build_mask(const MaskSpec &m) {
    cv::Mat out(N, N, CV_8UC1, cv::Scalar(0));
    auto set_rect = [&](int x, int y, int w, int h, int v) {
        for (int r = y; r < y + h; ++r)
            for (int c = x; c < x + w; ++c)
                if (r >= 0 && r < N && c >= 0 && c < N) out.at<uint8_t>(r, c) = uint8_t(v);
    };
    uint32_t s = uint32_t(m.v);
    switch (m.kind) {
    case 'F': set_rect(0, 0, N, N, m.v); break;
    case 'H': set_rect(0, 0, N, N, m.v); set_rect(m.a, m.b, m.c, m.d, 0); break;
    case 'A':
        set_rect(0, 0, N, N, 255);
        if (m.c > 0) set_rect(m.a, m.b, m.c, m.d, 0);
        for (int r = 0; r < N; ++r)
            for (int c = 0; c < N; ++c) {
                s = s * 1664525u + 1013904223u;
                if (out.at<uint8_t>(r, c)) out.at<uint8_t>(r, c) = uint8_t(1 + (s >> 24) % 255);
            }
        break;
    case 'M':
        set_rect(0, 0, N, N, m.v);
        set_rect(8, 90, 30, 20, 0);
        set_rect(70, 10, 40, 30, 0);
        set_rect(96, 80, 24, 40, 0);
        break;
    case 'P': set_rect(m.a, m.b, 1, 1, m.v); break;
    case 'G':
        for (int r = 0; r < N; r += m.a)
            for (int c = 0; c < N; c += m.a) out.at<uint8_t>(r, c) = uint8_t(m.v);
        break;
    case 'S': set_rect(0, m.a, N, m.b, m.v); break;
    case 'B': set_rect(m.a, m.b, m.c, m.c, m.v); break;
    case 'D':
        s = 7;
        for (int r = 0; r < N; ++r)
            for (int c = 0; c < N; ++c) {
                s = s * 1664525u + 1013904223u;
                if (int((s >> 8) % 100u) < m.a) out.at<uint8_t>(r, c) = uint8_t(m.v);
            }
        break;
    case 'L': set_rect(0, 0, m.a, N, m.v); break;
    case 'Z': break;
    case 'Q': set_rect(24, 24, 42, 42, m.v); break;
    default: throw std::invalid_argument("mask kind");
    }
    return out;
}

cv::Mat occlude(cv::Mat moving, const Occlusion &o) {
    const cv::Rect r(o.x, o.y, o.w, o.h);
    cv::Mat source;
    switch (o.kind) {
    case 0: source = cv::Mat(o.h, o.w, CV_8UC1, cv::Scalar(250)); break;
    case 1: source = noise(o.h, o.w, 1234); break;
    case 2: source = texture(N, N, -14, 9)(r).clone(); break;
    default: source = stripes(o.h, o.w, 0); break;
    }
    for (int y = 0; y < o.h; ++y)
        for (int x = 0; x < o.w; ++x) moving.at<uint8_t>(o.y + y, o.x + x) = source.at<uint8_t>(y, x);
    return moving;
}

// Images and mask. Region fixtures embed all three in 160x160 parents (window at 16,16).
void make_inputs(const Fixture &f, cv::Mat &a, cv::Mat &b, cv::Mat &m, cv::Mat parents[3]) {
    const cv::Mat mask = build_mask(f.mask);
    switch (f.image) {
    case 'C': a = constant(N, N, 128); b = constant(N, N, 128); break;
    case 'N': a = noise(N, N, 3); b = noise(N, N, 9); break;
    case 'K':
        a = texture(N, N);
        b = homography_texture(N, N, f.truth);
        for (int r = 20; r < 70; ++r)
            for (int c = 20; c < 70; ++c) { a.at<uint8_t>(r, c) = 100; b.at<uint8_t>(r, c) = 100; }
        break;
    case 'R': {
        const cv::Rect w(16, 16, N, N);
        parents[0] = texture(160, 160);
        parents[1] = homography_texture(160, 160, f.truth);
        parents[2] = cv::Mat(160, 160, CV_8UC1, cv::Scalar(0));
        for (int r = 0; r < N; ++r)
            for (int c = 0; c < N; ++c) parents[2].at<uint8_t>(16 + r, 16 + c) = mask.at<uint8_t>(r, c);
        a = parents[0](w); b = parents[1](w); m = parents[2](w);
        return;
    }
    default:
        a = texture(N, N);
        b = is_identity(f.truth) ? texture(N, N) : homography_texture(N, N, f.truth);
        if (f.image == 'O') b = occlude(b, f.occ);
    }
    m = mask;
}

struct Outcome { bool ok = false; float w[9] = {0}; double rho = 0; std::string error; };

Outcome run(const Fixture &f, const cv::Mat &a, const cv::Mat &b, cv::InputArray mask) {
    Outcome o;
    cv::Mat warp(3, 3, CV_32F);
    for (int i = 0; i < 9; ++i) warp.ptr<float>()[i] = f.seed[i];
    try {
        o.rho = cv::findTransformECC(a, b, warp, cv::MOTION_HOMOGRAPHY,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, f.iterations, f.epsilon),
            mask, 5);
        o.ok = true;
        if (warp.dims != 2 || warp.rows != 3 || warp.cols != 3 || warp.type() != CV_32FC1 ||
            !warp.isContinuous())
            throw std::runtime_error(std::string("malformed native output for ") + f.name);
        for (int i = 0; i < 9; ++i) o.w[i] = warp.ptr<float>()[i];
    } catch (const cv::Exception &e) {
        o.ok = false;
        o.error = e.what();
    }
    return o;
}

bool same(const Outcome &x, const Outcome &y) {
    if (x.ok != y.ok) return false;
    return !x.ok || (std::memcmp(x.w, y.w, sizeof x.w) == 0 && std::memcmp(&x.rho, &y.rho, sizeof x.rho) == 0);
}

uint32_t bits32(float v) { uint32_t u; std::memcpy(&u, &v, sizeof u); return u; }
unsigned long long bits64(double v) { unsigned long long u; std::memcpy(&u, &v, sizeof u); return u; }

const char *error_class(const Outcome &o) {
    if (o.ok) return "NONE";
    if (o.error.find("NaN encountered") != std::string::npos) return "NAN";
    if (o.error.find("stopped before its convergence") != std::string::npos) return "LAMBDA";
    return "OTHER";
}
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-homography-masked-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-homography-masked-oracle 1\nversion " << CV_VERSION << '\n';
    int count = 0, ok_count = 0, effect = 0;
    char buffer[1024];
    try {
        for (const auto &f : fixtures()) {
            cv::Mat a, b, m, parents[3];
            make_inputs(f, a, b, m, parents);
            if (f.image == 'R' && (a.isContinuous() || b.isContinuous() || m.isContinuous())) {
                std::fprintf(stderr, "oracle: Region fixture is contiguous\n");
                return 3;
            }
            const int64_t ma = checksum(a), mb = checksum(b), mm = checksum(m);
            const Outcome masked = run(f, a, b, m);
            const Outcome plain = run(f, a, b, cv::noArray());
            if (checksum(a) != ma || checksum(b) != mb || checksum(m) != mm) {
                std::fprintf(stderr, "oracle: native ECC modified an input for %s\n", f.name);
                return 4;
            }
            if (masked.ok) {
                ++ok_count;
                for (float v : masked.w)
                    if (!std::isfinite(v)) { std::fprintf(stderr, "oracle: nonfinite %s\n", f.name); return 4; }
                if (!std::isfinite(masked.rho)) { std::fprintf(stderr, "oracle: nonfinite rho %s\n", f.name); return 4; }
            }
            const bool equal = same(masked, plain);
            if (!equal) ++effect;
            int n = std::snprintf(buffer, sizeof buffer, "fixture %s %d %.17e", f.name, f.iterations, f.epsilon);
            for (float s : f.seed) n += std::snprintf(buffer + n, sizeof buffer - size_t(n), " %08x", bits32(s));
            n += std::snprintf(buffer + n, sizeof buffer - size_t(n), " %s", masked.ok ? "OK" : "EXC");
            for (float w : masked.w) n += std::snprintf(buffer + n, sizeof buffer - size_t(n), " %08x", masked.ok ? bits32(w) : 0u);
            std::snprintf(buffer + n, sizeof buffer - size_t(n), " %016llx %lld %lld %lld %d %d %s\n",
                          masked.ok ? bits64(masked.rho) : 0ull, (long long)ma, (long long)mb, (long long)mm,
                          cv::countNonZero(m), equal ? 1 : 0, error_class(masked));
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
    std::printf("PASS: ECC homography masked oracle OpenCV %s, %d fixtures (%d native successes, %d exceptions, "
                "%d differ from unmasked)\n", CV_VERSION, count, ok_count, count - ok_count, effect);
    return 0;
}
