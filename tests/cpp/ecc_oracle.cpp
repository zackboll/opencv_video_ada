// Independent direct-OpenCV ECC translation oracle: links OpenCV only; no Video
// shim, no Core bridge. Writes obj/oracle/ecc-translation.txt (format below).
//   ecc-oracle 1
//   version <CV_VERSION>
//   fixture <name> <iterations> <epsilon> <OK|EXC> <dx> <dy> <rho> <template-sum> <moving-sum>
//   end <fixture count>
// Floating values use %.17e. EXC lines carry zeros. The checksums let the Ada side
// prove it built the identical fixture before comparing.
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

std::vector<Fixture> fixtures() {
    auto tex = [](int dx, int dy) { return [=] { return texture(N, N, dx, dy); }; };
    auto affine = [](int dx, int dy, int num, int den, int beta) {
        return [=] { return affine_int(texture(N, N, dx, dy), num, den, beta); };
    };
    auto big = [](int dx, int dy) { return [=] { return texture(160, 160, dx, dy); }; };
    std::vector<Fixture> f;
    auto add = [&](const char *name, std::function<cv::Mat()> a, std::function<cv::Mat()> b,
                   int iterations = 50, double epsilon = 1e-4) {
        f.push_back({name, iterations, epsilon, a, b});
    };
    add("identity", tex(0, 0), tex(0, 0));
    add("positive", tex(0, 0), tex(2, 1));
    add("negative", tex(0, 0), tex(-2, -1));
    add("horizontal", tex(0, 0), tex(3, 0));
    add("vertical", tex(0, 0), tex(0, 3));
    add("contrast-0.8+20", tex(0, 0), affine(2, 1, 4, 5, 20));
    add("contrast-1.3-10", tex(0, 0), affine(2, 1, 13, 10, -10));
    add("low-variance", [] { return affine_int(texture(N, N), 1, 40, 126); },
        [] { return affine_int(texture(N, N, 2, 1), 1, 40, 126); });
    add("constant", [] { return constant(N, N, 128); }, [] { return constant(N, N, 128); });
    add("constant-vs-texture", [] { return constant(N, N, 128); }, tex(0, 0));
    add("noise-vs-noise", [] { return noise(N, N, 3); }, [] { return noise(N, N, 9); });
    add("noise-vs-texture", tex(0, 0), [] { return noise(N, N, 7); });
    add("stripes-shift3", [] { return stripes(N, N, 0); }, [] { return stripes(N, N, 3); });
    add("large-12-7", tex(0, 0), tex(12, 7));
    add("large-6-4", tex(0, 0), tex(6, 4));
    add("large-40-30", tex(0, 0), tex(40, 30));
    // Region fixtures: parents are 160x160; the 128x128 window at (16,16) is used.
    add("region-both", big(0, 0), big(2, 1));
    add("region-template", big(0, 0), [] { return texture(160, 160, 2, 1)(cv::Rect(16, 16, N, N)).clone(); });
    add("region-moving", [] { return texture(160, 160)(cv::Rect(16, 16, N, N)).clone(); }, big(2, 1));
    add("positive-it1", tex(0, 0), tex(2, 1), 1, 1e-4);
    add("positive-it2", tex(0, 0), tex(2, 1), 2, 1e-4);
    add("positive-it100-e8", tex(0, 0), tex(2, 1), 100, 1e-8);
    add("positive-e0.1", tex(0, 0), tex(2, 1), 50, 0.1);
    add("shift42-e0.1", tex(0, 0), tex(4, 2), 100, 0.1);
    add("shift42-e1e-8", tex(0, 0), tex(4, 2), 100, 1e-8);
    add("shift43-e0.1", tex(0, 0), tex(4, 3), 100, 0.1);
    add("shift43-e1e-8", tex(0, 0), tex(4, 3), 100, 1e-8);
    return f;
}

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }
}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: ecc-oracle OUTPUT\n"); return 2; }
    std::ofstream out(argv[1], std::ios::trunc);
    out << "ecc-oracle 1\nversion " << CV_VERSION << '\n';
    int count = 0;
    char buffer[512];
    const cv::Rect window(16, 16, N, N);
    try {
        for (const auto &f : fixtures()) {
            cv::Mat a = f.make_template(), b = f.make_moving();
            // 160x160 fixture images are used through a genuine non-contiguous Region.
            if (a.rows == 160) a = a(window);
            if (b.rows == 160) b = b(window);
            if (starts_with(f.name, "region-") && f.name == "region-both" &&
                (a.isContinuous() || b.isContinuous())) {
                std::fprintf(stderr, "oracle: Region fixture is contiguous\n");
                return 3;
            }
            cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
            bool ok = true;
            double rho = 0;
            try {
                rho = cv::findTransformECC(a, b, warp, cv::MOTION_TRANSLATION,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                     f.iterations, f.epsilon), cv::noArray(), 5);
            } catch (const cv::Exception &) { ok = false; }
            const double dx = ok ? double(warp.at<float>(0, 2)) : 0.0;
            const double dy = ok ? double(warp.at<float>(1, 2)) : 0.0;
            if (ok && (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(rho) ||
                       warp.rows != 2 || warp.cols != 3 || warp.type() != CV_32FC1)) {
                std::fprintf(stderr, "oracle: nonfinite or malformed native output for %s\n",
                             f.name.c_str());
                return 4;
            }
            std::snprintf(buffer, sizeof buffer, "fixture %s %d %.17e %s %.17e %.17e %.17e %lld %lld\n",
                          f.name.c_str(), f.iterations, f.epsilon, ok ? "OK" : "EXC", dx, dy,
                          ok ? rho : 0.0, (long long)checksum(a), (long long)checksum(b));
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
    std::printf("PASS: ECC translation oracle OpenCV %s, %d fixtures\n", CV_VERSION, count);
    return 0;
}
