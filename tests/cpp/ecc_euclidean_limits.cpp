// Task 012 research: Euclidean ECC memory at the geometry limit and the angle
// convergence basin. Direct OpenCV only; not part of routine CI.
//   ecc-euclidean-limits memory ROWS COLS     one large Euclidean (or translation) call
//   ecc-euclidean-limits basin                angle sweep, prints one line per angle
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

static int run_basin() {
    const int N = 128;
    const cv::Mat base = texture(N, N);
    const double angles[] = {5, 8, 9, 10, 11, 12, 13, 14, 15, 17, 20, 30, 60, 90, 120, 170, 179};
    for (double sign : {1.0, -1.0})
        for (double deg : angles) {
            const double d = sign * deg;
            const cv::Mat moving = rigid_texture(N, N, d, 0, 0);
            cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
            try {
                const double rho = cv::findTransformECC(base, moving, warp, cv::MOTION_EUCLIDEAN,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 50, 1e-4),
                    cv::noArray(), 5);
                const double theta = std::atan2(double(warp.at<float>(1, 0)), double(warp.at<float>(0, 0)));
                std::printf("%+6.1f deg: OK theta=%+9.4f deg tx=%9.3f ty=%9.3f rho=%.4f %s\n", d,
                            theta * 180.0 / 3.14159265358979323846, double(warp.at<float>(0, 2)),
                            double(warp.at<float>(1, 2)), rho,
                            std::fabs(theta * 180.0 / 3.14159265358979323846 - d) < 0.5 ? "correct" : "WRONG");
            } catch (const cv::Exception &) {
                std::printf("%+6.1f deg: EXC\n", d);
            }
        }
    return 0;
}

int main(int argc, char **argv) {
    std::printf("OpenCV %s\n", CV_VERSION);
    if (argc == 2 && std::strcmp(argv[1], "basin") == 0) return run_basin();
    if (argc >= 4 && std::strcmp(argv[1], "memory") == 0) {
        const int rows = std::atoi(argv[2]), cols = std::atoi(argv[3]);
        const bool translation = argc >= 5 && std::strcmp(argv[4], "translation") == 0;
        cv::Mat a(rows, cols, CV_8UC1), b;
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                a.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(signal(c % 256, r % 256));
        const cv::Mat shift = (cv::Mat_<double>(2, 3) << 1, 0, 2, 0, 1, 1);
        cv::warpAffine(a, b, shift, a.size());
        cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
        const double rho = cv::findTransformECC(a, b, warp,
            translation ? cv::MOTION_TRANSLATION : cv::MOTION_EUCLIDEAN,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 50, 1e-4),
            cv::noArray(), 5);
        std::ifstream status("/proc/self/status");
        for (std::string line; std::getline(status, line);)
            if (line.rfind("VmHWM:", 0) == 0) std::printf("%s\n", line.c_str());
        std::printf("%dx%d %s rho=%.4f tx=%.3f ty=%.3f\n", rows, cols,
                    translation ? "translation" : "euclidean", rho, double(warp.at<float>(0, 2)),
                    double(warp.at<float>(1, 2)));
        return 0;
    }
    std::fprintf(stderr, "usage: basin | memory ROWS COLS [translation]\n");
    return 2;
}
