// Task 018 safety investigation only. Direct OpenCV; no Video shim, no Core bridge.
// Compile with the selected OpenCV pkg-config flags and -Itests/cpp. Results are recorded in
// docs/ecc-homography-masked-source-contract.md.
//
//   probe semantics            mask value / Region / aliasing / immutability equivalences
//   probe types                native behavior for mask depth, channels and geometry
//   probe support              all-zero, sparse, narrow and non-overlapping masks
//   probe occlusion            occlusion fixtures: unmasked vs masked vs wrong-frame masks
//   probe degenerate           constant valid support, constant images, rank-deficient support
//   probe sweep TRIALS ITERS   randomized masks and in-domain seeds (sanitizer/diagnostic runs)
//   probe mem MODE ROWS COLS   peak RSS for MODE masked|unmasked|full
//
// Built with -DTASK018_DIAG it calls cv::task018_findTransformECC (an instrumented scratch copy
// of the selected version's ecc.cpp) instead of the library entry point.
//
// Convention: H maps TEMPLATE coordinates to MOVING coordinates; the mask is in MOVING pixels.
#include "ecc_fixture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sys/resource.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifdef TASK018_DIAG
namespace cv {
double task018_findTransformECC(InputArray, InputArray, InputOutputArray, int, TermCriteria,
                                InputArray, int);
}
#define ECC_ENTRY cv::task018_findTransformECC
extern long task018_iterations, task018_zero_valid, task018_inf_rho, task018_zero_norm_nonzero_corr;
extern long task018_min_valid;
extern double task018_max_narrowed;
#else
#define ECC_ENTRY cv::findTransformECC
#endif

namespace {
using namespace ecc_fixture;
using Warp = std::array<float, 9>;
constexpr Warp kIdentity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
int violations = 0;

struct Outcome {
    bool ok = false;
    double rho = 0;
    Warp w = kIdentity;
    std::string error;
};

Outcome run(const cv::Mat &a, const cv::Mat &b, cv::InputArray mask, const Warp &seed,
            int iterations = 50, double eps = 1e-4) {
    Outcome o;
    cv::Mat warp(3, 3, CV_32F);
    std::copy(seed.begin(), seed.end(), warp.ptr<float>());
    try {
        o.rho = ECC_ENTRY(a, b, warp, cv::MOTION_HOMOGRAPHY,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, eps),
            mask, 5);
        o.ok = true;
    } catch (const cv::Exception &e) {
        o.error = e.what();
    }
    std::copy(warp.ptr<float>(), warp.ptr<float>() + 9, o.w.begin());
    return o;
}

bool bitwise_same(const Outcome &x, const Outcome &y) {
    if (x.ok != y.ok) return false;
    if (!x.ok) return true;
    return std::memcmp(x.w.data(), y.w.data(), sizeof(float) * 9) == 0 &&
           std::memcmp(&x.rho, &y.rho, sizeof(double)) == 0;
}

std::string brief(const Outcome &o) {
    std::ostringstream s;
    s << std::setprecision(9);
    if (!o.ok) {
        std::string e = o.error;
        const auto p = e.find("error: ");
        if (p != std::string::npos) e = e.substr(p + 7);
        for (auto &c : e) if (c == '\n') c = ' ';
        s << "EXC " << e.substr(0, 160);
    } else {
        s << "OK rho=" << o.rho << " H=[";
        for (int i = 0; i < 9; ++i) s << (i ? " " : "") << o.w[i];
        s << "]";
    }
    return s.str();
}

void expect(bool condition, const std::string &what) {
    std::cout << (condition ? "CHECK ok   " : "VIOLATION  ") << what << '\n';
    if (!condition) ++violations;
}

int64_t checksum(const cv::Mat &m) {
    int64_t sum = 0;
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols * int(m.elemSize()); ++c)
            sum += int64_t(m.ptr<uint8_t>(r)[c]) * (1 + (int64_t(r) * 977 + int64_t(c) * 131) % 8191);
    return sum;
}

// Native moving-mask preprocessing transcribed from ecc.cpp (same for an 8UC1 mask in 4.1.0,
// 4.6.0, 4.10.0 and 5.0.0): binarize, Gaussian 5x5, scale by 0.5/0.95, round to 8U.
cv::Mat effective_support(const cv::Mat &mask) {
    cv::Mat pre, f;
    cv::threshold(mask, pre, 0, 1, cv::THRESH_BINARY);
    pre.convertTo(f, CV_32F);
    cv::GaussianBlur(f, f, cv::Size(5, 5), 0, 0);
    f *= (0.5 / 0.95);
    f.convertTo(pre, CV_8U);
    return pre;
}

cv::Mat full_mask(int rows, int cols, uint8_t v = 255) {
    return cv::Mat(rows, cols, CV_8UC1, cv::Scalar(v));
}

cv::Mat excluded_rect(int rows, int cols, cv::Rect r, uint8_t v = 255) {
    cv::Mat m = full_mask(rows, cols, v);
    m(r & cv::Rect(0, 0, cols, rows)).setTo(0);
    return m;
}

// Deterministic arbitrary nonzero values 1..255 where `base` is nonzero.
cv::Mat arbitrary_nonzero(const cv::Mat &base, uint32_t seed) {
    cv::Mat m = base.clone();
    uint32_t s = seed;
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c) {
            s = s * 1664525u + 1013904223u;
            if (m.at<uint8_t>(r, c)) m.at<uint8_t>(r, c) = uint8_t(1 + (s >> 24) % 255);
        }
    return m;
}

// Worst projected error of H against the truth at the template corners and centre.
double corner_error(const Warp &h, const double *t, int rows, int cols) {
    const double pts[5][2] = {{0, 0}, {double(cols - 1), 0}, {0, double(rows - 1)},
                              {double(cols - 1), double(rows - 1)},
                              {(cols - 1) / 2.0, (rows - 1) / 2.0}};
    double worst = 0;
    for (auto &p : pts) {
        const double de = h[6] * p[0] + h[7] * p[1] + h[8];
        const double dt = t[6] * p[0] + t[7] * p[1] + t[8];
        const double ex = (h[0] * p[0] + h[1] * p[1] + h[2]) / de -
                          (t[0] * p[0] + t[1] * p[1] + t[2]) / dt;
        const double ey = (h[3] * p[0] + h[4] * p[1] + h[5]) / de -
                          (t[3] * p[0] + t[4] * p[1] + t[5]) / dt;
        worst = std::max(worst, std::sqrt(ex * ex + ey * ey));
    }
    return worst;
}

const int N = 128;
const double kTranslation[9] = {1, 0, 2, 0, 1, 1, 0, 0, 1};
const double kPerspective[9] = {1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0};  // Task 017 MILD
const double kScp[9] = {1.15, 0.02, -4, -0.01, 1.12, 3, 0.0008, -0.0006, 1};
const Warp kScpNear = {1.12f, 0.01f, -3, 0, 1.10f, 2.5f, 0.0006f, -0.0005f, 1};
const double kLtp[9] = {1, 0, 12, 0, 1, -9, 0.0005, -0.0004, 1};
const Warp kLtpNear = {1, 0, 11, 0, 1, -8, 0.0004f, -0.0003f, 1};

cv::Mat moving_for(const double *t) { return homography_texture(N, N, t); }

int run_semantics() {
    std::cout << "semantics OpenCV " << CV_VERSION << '\n';
    const cv::Mat a = texture(N, N);
    struct Pair { const char *name; cv::Mat b; Warp seed; };
    const std::vector<Pair> pairs = {
        {"translation", moving_for(kTranslation), kIdentity},
        {"perspective", moving_for(kPerspective), kIdentity},
        {"ltp-seeded", moving_for(kLtp), kLtpNear},
        {"scp-seeded", moving_for(kScp), kScpNear},
    };
    const cv::Rect hole(70, 10, 40, 30);  // asymmetric: x 70..109, y 10..39 (moving pixels)
    for (const auto &p : pairs) {
        const std::string n = p.name;
        const Outcome none = run(a, p.b, cv::noArray(), p.seed);
        std::cout << "  " << n << " noArray: " << brief(none) << '\n';
        for (int v : {1, 255, 7})
            expect(bitwise_same(none, run(a, p.b, full_mask(N, N, uint8_t(v)), p.seed)),
                   n + ": all-included mask value " + std::to_string(v) + " == noArray");
        expect(bitwise_same(none, run(a, p.b, arbitrary_nonzero(full_mask(N, N), 99), p.seed)),
               n + ": all-included arbitrary nonzero == noArray");
        expect(bitwise_same(none, run(a, p.b, cv::Mat(), p.seed)),
               n + ": EMPTY Mat mask == noArray (native treats empty as unmasked)");

        const cv::Mat m255 = excluded_rect(N, N, hole, 255);
        const Outcome r255 = run(a, p.b, m255, p.seed);
        std::cout << "  " << n << " rect hole 255: " << brief(r255) << '\n';
        expect(bitwise_same(r255, run(a, p.b, excluded_rect(N, N, hole, 1), p.seed)),
               n + ": rect hole values 1 == 255");
        expect(bitwise_same(r255, run(a, p.b, arbitrary_nonzero(m255, 5), p.seed)),
               n + ": rect hole arbitrary nonzero == 255");
        expect(!bitwise_same(none, r255), n + ": rect hole changes the native result");

        cv::Mat parent(160, 160, CV_8UC1, cv::Scalar(0));
        m255.copyTo(parent(cv::Rect(16, 16, N, N)));
        const cv::Mat region = parent(cv::Rect(16, 16, N, N));
        const int64_t parent_sum = checksum(parent);
        expect(!region.isContinuous(), n + ": Region mask is non-contiguous");
        expect(bitwise_same(r255, run(a, p.b, region, p.seed)),
               n + ": non-contiguous Region mask == contiguous clone");
        expect(parent_sum == checksum(parent), n + ": Region parent unchanged");

        const cv::Mat mask_copy = m255.clone(), a_copy = a.clone(), b_copy = p.b.clone();
        run(a, p.b, m255, p.seed);
        expect(cv::norm(mask_copy, m255, cv::NORM_INF) == 0 && cv::norm(a_copy, a, cv::NORM_INF) == 0 &&
                   cv::norm(b_copy, p.b, cv::NORM_INF) == 0,
               n + ": mask, template and moving unchanged");
        expect(bitwise_same(run(a, p.b, p.b, p.seed), run(a, p.b, p.b.clone(), p.seed)),
               n + ": mask aliasing the moving image == its clone");
        expect(cv::norm(b_copy, p.b, cv::NORM_INF) == 0, n + ": aliased moving unchanged");
        const cv::Mat transposed = excluded_rect(N, N, cv::Rect(hole.y, hole.x, hole.height, hole.width));
        expect(!bitwise_same(r255, run(a, p.b, transposed, p.seed)),
               n + ": transposed hole gives a different result");
        std::cout << "  " << n << " hole nonzero=" << cv::countNonZero(m255)
                  << " effective=" << cv::countNonZero(effective_support(m255)) << '\n';
    }
    std::cout << "semantics violations=" << violations << '\n';
    return violations ? 1 : 0;
}

int run_types() {
    std::cout << "types OpenCV " << CV_VERSION << '\n';
    const cv::Mat a = texture(96, 128), b = texture(96, 128, 2, 1);
    const cv::Mat good = excluded_rect(96, 128, cv::Rect(80, 8, 30, 20));
    const Outcome ref = run(a, b, good, kIdentity);
    std::cout << "  8UC1 reference: " << brief(ref) << '\n';
    struct T { const char *name; int type; };
    for (const T &t : {T{"8SC1", CV_8SC1}, T{"16UC1", CV_16UC1}, T{"16SC1", CV_16SC1},
                       T{"32SC1", CV_32SC1}, T{"32FC1", CV_32FC1}, T{"64FC1", CV_64FC1}}) {
        cv::Mat m;
        good.convertTo(m, t.type, 1.0 / 255.0);
        const Outcome o = run(a, b, m, kIdentity);
        std::cout << "  " << t.name << " (0/1): " << brief(o)
                  << (o.ok ? (bitwise_same(o, ref) ? " [== 8UC1]" : " [differs from 8UC1]") : "") << '\n';
    }
    for (int cn : {2, 3, 4}) {
        std::vector<cv::Mat> planes(size_t(cn), good);
        cv::Mat m;
        cv::merge(planes, m);
        std::cout << "  8UC" << cn << ": " << brief(run(a, b, m, kIdentity)) << '\n';
    }
    std::cout << "  rows+1: " << brief(run(a, b, excluded_rect(97, 128, cv::Rect(80, 8, 30, 20)), kIdentity)) << '\n';
    std::cout << "  cols+1: " << brief(run(a, b, excluded_rect(96, 129, cv::Rect(80, 8, 30, 20)), kIdentity)) << '\n';
    std::cout << "  rows-1: " << brief(run(a, b, excluded_rect(95, 128, cv::Rect(80, 8, 30, 20)), kIdentity)) << '\n';
    std::cout << "  transposed 128x96: " << brief(run(a, b, excluded_rect(128, 96, cv::Rect(8, 80, 20, 30)), kIdentity)) << '\n';
    std::cout << "  1x1: " << brief(run(a, b, full_mask(1, 1), kIdentity)) << '\n';
    std::cout << "  empty: " << brief(run(a, b, cv::Mat(), kIdentity)) << " [unmasked: "
              << brief(run(a, b, cv::noArray(), kIdentity)) << "]\n";
    const int sizes[3] = {96, 128, 1};
    cv::Mat nd(3, sizes, CV_8UC1, cv::Scalar(255));
    std::cout << "  3-D 96x128x1: " << brief(run(a, b, nd, kIdentity)) << '\n';
    return 0;
}

int run_support() {
    std::cout << "support OpenCV " << CV_VERSION << '\n';
    const cv::Mat a = texture(N, N), b = moving_for(kTranslation);
    auto report = [&](const std::string &name, const cv::Mat &m, const Warp &seed = kIdentity,
                      int it = 50) {
        const Outcome o = run(a, b, m, seed, it);
        std::cout << "  " << std::left << std::setw(26) << name << " nonzero=" << std::setw(6)
                  << cv::countNonZero(m) << " effective=" << std::setw(6)
                  << cv::countNonZero(effective_support(m)) << ' ' << brief(o) << '\n';
        return o;
    };
    const cv::Mat zero(N, N, CV_8UC1, cv::Scalar(0));
    expect(!report("all-zero", zero).ok, "all-zero mask raises natively");
    cv::Mat single = zero.clone();
    single.at<uint8_t>(64, 64) = 255;
    report("single pixel", single);
    for (int s : {2, 3, 4, 5, 8}) {
        cv::Mat g = zero.clone();
        for (int r = 0; r < N; r += s)
            for (int c = 0; c < N; c += s) g.at<uint8_t>(r, c) = 255;
        report("grid spacing " + std::to_string(s), g);
    }
    for (int w = 1; w <= 8; ++w) {
        cv::Mat h = zero.clone(), v = zero.clone();
        h(cv::Rect(0, 60, N, w)).setTo(255);
        v(cv::Rect(60, 0, w, N)).setTo(255);
        report("row strip width " + std::to_string(w), h);
        report("column strip width " + std::to_string(w), v);
    }
    for (int k = 1; k <= 12; ++k) {
        cv::Mat blk = zero.clone();
        blk(cv::Rect(50, 40, k, k)).setTo(255);
        report("block " + std::to_string(k) + "x" + std::to_string(k), blk);
    }
    cv::Mat cross = zero.clone();
    cross(cv::Rect(0, 60, N, 6)).setTo(255);
    cross(cv::Rect(60, 0, 6, N)).setTo(255);
    report("cross width 6", cross);
    uint32_t s = 7;
    for (double p : {0.5, 0.8, 0.9, 0.95, 0.99}) {
        cv::Mat r = zero.clone();
        for (int y = 0; y < N; ++y)
            for (int x = 0; x < N; ++x) {
                s = s * 1664525u + 1013904223u;
                if (double(s >> 8) / 16777216.0 < p) r.at<uint8_t>(y, x) = 255;
            }
        std::ostringstream n;
        n << "random density " << p;
        report(n.str(), r);
    }
    // No overlap after warping: valid moving pixels only in columns 0..23, and the seed maps
    // template x to moving x + 60, so no template pixel samples a valid mask pixel.
    cv::Mat left = zero.clone();
    left(cv::Rect(0, 0, 24, N)).setTo(255);
    report("left band, identity", left);
    report("left band, seed +60", left, Warp{1, 0, 60, 0, 1, 0, 0, 0, 1});
    report("left band, seed +60 it1", left, Warp{1, 0, 60, 0, 1, 0, 0, 0, 1}, 1);
    report("left band, seed +104", left, Warp{1, 0, 104, 0, 1, 0, 0, 0, 1});
    report("full, seed +60", full_mask(N, N), Warp{1, 0, 60, 0, 1, 0, 0, 0, 1});
    std::cout << "support violations=" << violations << '\n';
    return violations ? 1 : 0;
}

// Occluded moving image: the true homography everywhere except a rectangle (moving pixels)
// replaced by `kind` content.
cv::Mat occlude(const cv::Mat &moving, cv::Rect r, int kind) {
    cv::Mat out = moving.clone();
    cv::Mat roi = out(r);
    switch (kind) {
    case 0: roi.setTo(250); break;
    case 1: noise(r.height, r.width, 1234).copyTo(roi); break;
    case 2: texture(N, N, -14, 9)(r).copyTo(roi); break;
    default: stripes(r.height, r.width, 0).copyTo(roi); break;
    }
    return out;
}

int run_occlusion() {
    std::cout << "occlusion OpenCV " << CV_VERSION << '\n';
    const cv::Mat a = texture(N, N);
    struct Case { const char *name; const double *truth; Warp seed; cv::Rect r; int kind; };
    const std::vector<Case> cases = {
        {"trans-block", kTranslation, kIdentity, cv::Rect(64, 8, 56, 48), 0},
        {"trans-noise", kTranslation, kIdentity, cv::Rect(64, 8, 56, 48), 1},
        {"trans-shifted", kTranslation, kIdentity, cv::Rect(64, 8, 56, 48), 2},
        {"trans-stripes", kTranslation, kIdentity, cv::Rect(64, 8, 56, 48), 3},
        {"persp-block", kPerspective, kIdentity, cv::Rect(60, 4, 64, 52), 0},
        {"persp-noise", kPerspective, kIdentity, cv::Rect(60, 4, 64, 52), 1},
        {"persp-shifted", kPerspective, kIdentity, cv::Rect(60, 4, 64, 52), 2},
        {"persp-stripes", kPerspective, kIdentity, cv::Rect(60, 4, 64, 52), 3},
        {"ltp-block-seeded", kLtp, kLtpNear, cv::Rect(60, 4, 64, 52), 0},
        {"ltp-shifted-seeded", kLtp, kLtpNear, cv::Rect(60, 4, 64, 52), 2},
        {"ltp-stripes-seeded", kLtp, kLtpNear, cv::Rect(60, 4, 64, 52), 3},
        {"scp-block-seeded", kScp, kScpNear, cv::Rect(60, 4, 64, 52), 0},
        {"scp-shifted-seeded", kScp, kScpNear, cv::Rect(60, 4, 64, 52), 2},
        {"scp-noise-seeded", kScp, kScpNear, cv::Rect(60, 4, 64, 52), 1},
    };
    for (const auto &c : cases) {
        const cv::Mat b = occlude(moving_for(c.truth), c.r, c.kind);
        const cv::Rect grown(c.r.x - 3, c.r.y - 3, c.r.width + 6, c.r.height + 6);
        const cv::Mat moving_mask = excluded_rect(N, N, grown);
        const cv::Rect template_frame(grown.x - int(std::lround(c.truth[2])),
                                      grown.y - int(std::lround(c.truth[5])), grown.width, grown.height);
        const cv::Mat wrong_frame = excluded_rect(N, N, template_frame);
        const cv::Mat transposed = excluded_rect(N, N, cv::Rect(grown.y, grown.x, grown.height, grown.width));
        const Outcome clean = run(a, moving_for(c.truth), cv::noArray(), c.seed);
        const Outcome unmasked = run(a, b, cv::noArray(), c.seed);
        const Outcome masked = run(a, b, moving_mask, c.seed);
        const Outcome wrong = run(a, b, wrong_frame, c.seed);
        const Outcome trans = run(a, b, transposed, c.seed);
        auto err = [&](const Outcome &o) {
            std::ostringstream s;
            if (o.ok) s << std::setprecision(4) << corner_error(o.w, c.truth, N, N) << "px/rho=" << o.rho;
            else s << "EXC";
            return s.str();
        };
        std::cout << "  " << std::left << std::setw(20) << c.name << " clean=" << err(clean)
                  << " | unmasked=" << err(unmasked) << " | masked=" << err(masked)
                  << " | template-frame=" << err(wrong) << " | transposed=" << err(trans) << '\n';
    }
    return 0;
}


// Masks whose valid support is photometrically constant, or that only just survive the native
// erosion: the normalization denominators imgNorm*tmpNorm can be exactly 0 while the
// correlation is a rounding residue, which cvIsNaN does not catch (inf is not NaN).
int run_degenerate() {
    std::cout << "degenerate OpenCV " << CV_VERSION << '\n';
    cv::Mat a = texture(N, N), b = moving_for(kTranslation);
    a(cv::Rect(20, 20, 50, 50)).setTo(100);
    b(cv::Rect(20, 20, 50, 50)).setTo(100);
    for (int inset : {0, 2, 4, 8}) {
        cv::Mat m(N, N, CV_8UC1, cv::Scalar(0));
        m(cv::Rect(20 + inset, 20 + inset, 50 - 2 * inset, 50 - 2 * inset)).setTo(255);
        for (int it : {1, 50}) {
            const Outcome o = run(a, b, m, kIdentity, it);
            std::cout << "  constant-support inset " << inset << " it" << it << " effective="
                      << cv::countNonZero(effective_support(m)) << ' ' << brief(o)
                      << (o.ok && !std::isfinite(o.rho) ? " [NONFINITE rho returned]" : "") << '\n';
        }
    }
    const cv::Mat flat = constant(N, N, 77);
    const Outcome f = run(flat, flat, full_mask(N, N), kIdentity);
    std::cout << "  constant images full mask: " << brief(f) << '\n';
    // Stripes varying only in x, masked to a horizontal strip: rank-deficient Jacobian.
    const cv::Mat sa = stripes(N, N, 0), sb = stripes(N, N, 2);
    cv::Mat strip(N, N, CV_8UC1, cv::Scalar(0));
    strip(cv::Rect(0, 50, N, 12)).setTo(255);
    std::cout << "  stripes strip: " << brief(run(sa, sb, strip, kIdentity)) << '\n';
    return 0;
}

int run_sweep(int trials, int iterations) {
    std::cout << "sweep OpenCV " << CV_VERSION << " trials=" << trials << " iterations=" << iterations << '\n';
    const cv::Mat a = texture(64, 96), b = texture(64, 96, 2, 1);
    uint32_t state = 18;
    auto rnd = [&]() { state = state * 1664525u + 1013904223u; return double(state) / 4294967296.0; };
    int ok = 0, exc = 0, h22 = 0, nonfinite = 0, empty_eff = 0, empty_eff_ok = 0;
    double maximum = 0;
    for (int n = 0; n < trials; ++n) {
        cv::Mat m(64, 96, CV_8UC1, cv::Scalar(0));
        const int kind = n % 6;
        if (kind == 0) {
            m.setTo(255);
            const int holes = 1 + int(rnd() * 4);
            for (int k = 0; k < holes; ++k) {
                const int x = int(rnd() * 80), y = int(rnd() * 50);
                const int w = 4 + int(rnd() * 40), h = 4 + int(rnd() * 30);
                m(cv::Rect(x, y, w, h) & cv::Rect(0, 0, 96, 64)).setTo(0);
            }
        } else if (kind == 1) {
            const double p = rnd();
            for (int y = 0; y < 64; ++y)
                for (int x = 0; x < 96; ++x)
                    if (rnd() < p) m.at<uint8_t>(y, x) = uint8_t(1 + int(rnd() * 254));
        } else if (kind == 2) {
            const int w = 1 + int(rnd() * 8);
            if (rnd() < 0.5) m(cv::Rect(0, int(rnd() * (64 - w)), 96, w)).setTo(255);
            else m(cv::Rect(int(rnd() * (96 - w)), 0, w, 64)).setTo(255);
        } else if (kind == 3) {
            const int w = 1 + int(rnd() * 14), h = 1 + int(rnd() * 14);
            m(cv::Rect(int(rnd() * (96 - w)), int(rnd() * (64 - h)), w, h)).setTo(255);
        } else if (kind == 4) {
            const int count = 1 + int(rnd() * 20);
            for (int k = 0; k < count; ++k) m.at<uint8_t>(int(rnd() * 64), int(rnd() * 96)) = 255;
        } else {
            const int w = 1 + int(rnd() * 12);
            m(cv::Rect(rnd() < 0.5 ? 0 : 96 - w, 0, w, 64)).setTo(255);
        }
        if (cv::countNonZero(m) == 0) m.at<uint8_t>(int(rnd() * 64), int(rnd() * 96)) = 255;
        const bool no_support = cv::countNonZero(effective_support(m)) == 0;
        if (no_support) ++empty_eff;
        Warp seed = kIdentity;
        if (n % 2) {
            seed[0] = float(0.5 + rnd());
            seed[1] = float(rnd() - 0.5);
            seed[2] = float(80 * rnd() - 40);
            seed[3] = float(rnd() - 0.5);
            seed[4] = float(0.5 + rnd());
            seed[5] = float(80 * rnd() - 40);
            seed[6] = float((rnd() - 0.5) * 0.5 / 95);
            seed[7] = float((rnd() - 0.5) * 0.5 / 63);
        }
        const Outcome o = run(a, b, m, seed, iterations, 1e-8);
        if (o.w[8] != seed[8]) ++h22;
        if (o.ok) {
            ++ok;
            if (no_support) ++empty_eff_ok;
            bool finite = std::isfinite(o.rho);
            for (float v : o.w) {
                finite = finite && std::isfinite(v);
                if (std::isfinite(v)) maximum = std::max(maximum, double(std::fabs(v)));
            }
            if (!finite) ++nonfinite;
        } else {
            ++exc;
        }
    }
    std::cout << "sweep ok=" << ok << " exc=" << exc << " nonfinite_success=" << nonfinite
              << " h22_changed=" << h22 << " empty_effective_support=" << empty_eff
              << " empty_effective_support_ok=" << empty_eff_ok
              << " max_finite_coefficient=" << maximum << '\n';
    return h22 ? 2 : 0;
}

int run_mem(const std::string &mode, int rows, int cols) {
    const cv::Mat a = texture(rows, cols), b = texture(rows, cols, 1, 1);
    cv::Mat mask;
    if (mode == "masked") mask = excluded_rect(rows, cols, cv::Rect(cols / 2, rows / 8, cols / 3, rows / 3));
    else if (mode == "full") mask = full_mask(rows, cols);
    const auto t0 = std::chrono::steady_clock::now();
    const Outcome o = mode == "unmasked" ? run(a, b, cv::noArray(), kIdentity, 10, 1e-4)
                                         : run(a, b, mask, kIdentity, 10, 1e-4);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    std::cout << "mem " << mode << ' ' << cols << 'x' << rows << " ok=" << o.ok
              << " peak_rss_mib=" << ru.ru_maxrss / 1024.0 << " secs=" << secs << '\n';
    return 0;
}
}  // namespace

int dispatch(int argc, char **argv);

int main(int argc, char **argv) {
    cv::setNumThreads(1);
    const int rc = dispatch(argc, argv);
#ifdef TASK018_DIAG
    std::cout << "diag iterations=" << task018_iterations << " zero_valid_iterations=" << task018_zero_valid
              << " min_valid=" << task018_min_valid << " inf_rho=" << task018_inf_rho
              << " zero_norm_nonzero_correlation=" << task018_zero_norm_nonzero_corr
              << " max_finite_narrowed=" << task018_max_narrowed << '\n';
#endif
    return rc;
}

int dispatch(int argc, char **argv) {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "semantics") return run_semantics();
    if (mode == "types") return run_types();
    if (mode == "support") return run_support();
    if (mode == "occlusion") return run_occlusion();
    if (mode == "degenerate") return run_degenerate();
    if (mode == "sweep") return run_sweep(argc > 2 ? std::atoi(argv[2]) : 600, argc > 3 ? std::atoi(argv[3]) : 1);
    if (mode == "mem" && argc == 5) return run_mem(argv[2], std::atoi(argv[3]), std::atoi(argv[4]));
    std::cerr << "usage: probe semantics|types|support|occlusion|sweep [T I]|mem MODE ROWS COLS\n";
    return 2;
}
