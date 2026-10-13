// Task 015 research probe: direct native cv::findTransformECC(MOTION_AFFINE) with a
// caller-supplied initial 2x3 Float32 warp in native matrix storage order
// (M00 M01 M02 M10 M11 M12). Links OpenCV only: no Video shim, no Core bridge.
//
//   ecc-affine-seeded-probe consume   - identity vs exact / near / wrong / asymmetric seeds
//   ecc-affine-seeded-probe domain    - extreme finite seeds (bounds, singular, reflected)
//   ecc-affine-seeded-probe memory ROWS COLS [identity|seeded]
//
// Metrics (explicit, not a confidence measure):
//   coefficient error = max_i |estimate_i - truth_i|, over the six coefficients
//   residual          = mean |template - warpAffine(moving, M, WARP_INVERSE_MAP)| over the
//                       interior (24 px margin)
#include "ecc_fixture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

using namespace ecc_fixture;

namespace {
struct Outcome {
    bool ok = false;
    double rho = 0;
    float w[6] = {0, 0, 0, 0, 0, 0};
    std::string error;
};

Outcome estimate(const cv::Mat &a, const cv::Mat &b, const float seed[6], int it = 50,
                 double eps = 1e-4) {
    Outcome o;
    cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
    for (int i = 0; i < 6; ++i) warp.ptr<float>()[i] = seed[i];
    try {
        o.rho = cv::findTransformECC(a, b, warp, cv::MOTION_AFFINE,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, it, eps),
            cv::noArray(), 5);
        o.ok = true;
        for (int i = 0; i < 6; ++i) o.w[i] = warp.ptr<float>()[i];
    } catch (const cv::Exception &e) {
        o.error = e.what();
        const size_t eol = o.error.find('\n');
        if (eol != std::string::npos) o.error.resize(eol);
    }
    return o;
}

double residual(const cv::Mat &tmpl, const cv::Mat &moving, const float m[6]) {
    float copy[6];
    std::memcpy(copy, m, sizeof copy);
    cv::Mat mat(2, 3, CV_32F, copy), aligned;
    cv::warpAffine(moving, aligned, mat, moving.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP,
                   cv::BORDER_REFLECT_101);
    const cv::Rect roi(24, 24, tmpl.cols - 48, tmpl.rows - 48);
    cv::Mat fa, fb;
    tmpl(roi).convertTo(fa, CV_32F);
    aligned(roi).convertTo(fb, CV_32F);
    return cv::mean(cv::abs(fa - fb))[0];
}

double coefficient_error(const float e[6], const float t[6]) {
    double worst = 0;
    for (int i = 0; i < 6; ++i) worst = std::fmax(worst, std::fabs(double(e[i]) - double(t[i])));
    return worst;
}

void show(const char *label, const float s[6], const Outcome &o, const cv::Mat &tmpl,
          const cv::Mat &moving, const float truth[6]) {
    std::printf("  %-12s seed=[%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f] ", label, double(s[0]),
                double(s[1]), double(s[2]), double(s[3]), double(s[4]), double(s[5]));
    if (!o.ok) { std::printf("EXC %s\n", o.error.substr(0, 60).c_str()); return; }
    std::printf("-> [%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f] rho=%.5f coefErr %.4f->%.4f resid %.2f->%.2f\n",
                double(o.w[0]), double(o.w[1]), double(o.w[2]), double(o.w[3]), double(o.w[4]),
                double(o.w[5]), o.rho, coefficient_error(s, truth), coefficient_error(o.w, truth),
                residual(tmpl, moving, s), residual(tmpl, moving, o.w));
}

void consume(const char *name, const float t[6], const float near_seed[6], int N = 128) {
    const cv::Mat a = texture(N, N);
    const cv::Mat b = affine_texture(N, N, t[0], t[1], t[3], t[4], t[2], t[5]);
    const float identity[6] = {1, 0, 0, 0, 1, 0};
    const float wrong[6] = {0.8f, -0.2f, 6.0f, 0.2f, 1.25f, -5.0f};
    const float asym[6] = {1.0f, 0.3f, 4.0f, -0.25f, 1.0f, -2.0f};
    std::printf("%s truth=[%g %g %g %g %g %g]\n", name, double(t[0]), double(t[1]), double(t[2]),
                double(t[3]), double(t[4]), double(t[5]));
    show("identity", identity, estimate(a, b, identity), a, b, t);
    show("exact", t, estimate(a, b, t), a, b, t);
    show("near", near_seed, estimate(a, b, near_seed), a, b, t);
    show("wrong", wrong, estimate(a, b, wrong), a, b, t);
    show("asymmetric", asym, estimate(a, b, asym), a, b, t);
}


void domain() {
    const int N = 128;
    const cv::Mat a = texture(N, N), b = affine_texture(N, N, 1.01, 0.012, -0.008, 0.99, 2, 1);
    struct Case { const char *name; float s[6]; };
    const Case cases[] = {
        {"limit-linear+4",   {4, 4, 0, 4, 4, 0}},
        {"limit-linear-4",   {-4, -4, 0, -4, -4, 0}},
        {"limit-translation", {1, 0, 1048576.f, 0, 1, -1048576.f}},
        {"all-limits",       {4, -4, 1048576.f, -4, 4, -1048576.f}},
        {"singular-zero",    {0, 0, 0, 0, 0, 0}},
        {"singular-rank1",   {1, 1, 0, 1, 1, 0}},
        {"near-singular",    {1, 1, 0, 1, 1.0000001f, 0}},
        {"tiny-scale",       {1e-6f, 0, 0, 0, 1e-6f, 0}},
        {"reflect-x",        {-1, 0, 0, 0, 1, 0}},
        {"reflect-y",        {1, 0, 0, 0, -1, 0}},
        {"aniso-4-0.01",     {4, 0, 0, 0, 0.01f, 0}},
        {"shear-4",          {1, 4, 0, 0, 1, 0}},
        {"translation-1e5",  {1, 0, 100000.f, 0, 1, 100000.f}},
        {"beyond-linear-5",  {5, 0, 0, 0, 5, 0}},
        {"beyond-linear-1e3", {1000, 0, 0, 0, 1000, 0}},
        {"beyond-trans-1e7", {1, 0, 1e7f, 0, 1, 1e7f}},
        {"beyond-trans-1e9", {1, 0, 1e9f, 0, 1, -1e9f}},
        {"float-max",        {3.4e38f, 0, 3.4e38f, 0, 3.4e38f, 3.4e38f}},
    };
    for (const Case &c : cases) {
        const Outcome o = estimate(a, b, c.s);
        std::printf("  %-18s ", c.name);
        if (o.ok)
            std::printf("OK  [%g %g %g %g %g %g] rho=%.4f\n", double(o.w[0]), double(o.w[1]),
                        double(o.w[2]), double(o.w[3]), double(o.w[4]), double(o.w[5]), o.rho);
        else
            std::printf("EXC %s\n", o.error.substr(0, 70).c_str());
    }
}

void memory(int rows, int cols, bool seeded) {
    cv::Mat a(rows, cols, CV_8UC1), b;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            a.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(signal(c % 256, r % 256));
    double shift_values[6] = {1, 0, 2, 0, 1, 1};
    const cv::Mat shift(2, 3, CV_64F, shift_values);
    cv::warpAffine(a, b, shift, a.size());
    const float seed_close[6] = {1, 0, 1.5f, 0, 1, 0.5f};
    const float identity[6] = {1, 0, 0, 0, 1, 0};
    const Outcome o = estimate(a, b, seeded ? seed_close : identity);
    std::ifstream status("/proc/self/status");
    for (std::string line; std::getline(status, line);)
        if (line.rfind("VmHWM:", 0) == 0) std::printf("%s\n", line.c_str());
    std::printf("%dx%d %s ok=%d rho=%.4f\n", rows, cols, seeded ? "seeded" : "identity", int(o.ok),
                o.rho);
}
}  // namespace

int main(int argc, char **argv) {
    std::printf("OpenCV %s\n", CV_VERSION);
    const std::string mode = argc > 1 ? argv[1] : "consume";
    if (mode == "memory" && argc >= 4) {
        memory(std::atoi(argv[2]), std::atoi(argv[3]),
               argc >= 5 && std::string(argv[4]) == "seeded");
        return 0;
    }
    if (mode == "domain") { domain(); return 0; }
    const float shear[6] = {1.00f, 0.12f, 1.0f, -0.05f, 1.00f, -1.0f};
    const float shear_near[6] = {1.00f, 0.10f, 0.5f, -0.04f, 1.00f, -0.7f};
    const float scale[6] = {1.12f, 0.02f, 2.0f, -0.01f, 0.88f, 1.0f};
    const float scale_near[6] = {1.10f, 0.01f, 1.5f, 0.0f, 0.90f, 0.8f};
    const float comb[6] = {1.08f, 0.08f, 2.0f, -0.04f, 0.94f, -1.0f};
    const float comb_near[6] = {1.06f, 0.07f, 1.7f, -0.03f, 0.95f, -0.8f};
    const float refl[6] = {-1.0f, 0.0f, 127.0f, 0.0f, 1.0f, 0.0f};
    const float refl_near[6] = {-0.98f, 0.01f, 126.0f, 0.0f, 1.0f, 0.1f};
    const float shear2[6] = {1.0f, 0.2f, 0.0f, 0.0f, 1.0f, 0.0f};
    const float shear2_near[6] = {1.0f, 0.18f, 0.0f, 0.0f, 1.0f, 0.0f};
    const float strong[6] = {1.2f, 0.1f, 5.0f, -0.1f, 0.8f, -3.0f};
    const float strong_near[6] = {1.18f, 0.09f, 4.5f, -0.09f, 0.82f, -2.7f};
    consume("moderate-shear", shear, shear_near);
    consume("nonuniform-scale", scale, scale_near);
    consume("combined", comb, comb_near);
    consume("reflect-x", refl, refl_near);
    consume("shear-0.2", shear2, shear2_near);
    consume("strong-1.2-0.8", strong, strong_near);
    return 0;
}
