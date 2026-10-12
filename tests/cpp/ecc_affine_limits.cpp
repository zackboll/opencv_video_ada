// Task 014 research: affine ECC peak resident memory at the geometry limit, compared with
// the Euclidean and translation models. Direct OpenCV only; not part of routine CI.
//   ecc-affine-limits memory ROWS COLS [translation|euclidean|affine]
// Prints VmHWM (peak resident set) after one findTransformECC call on a textured pair that
// differs by a (2,1) shift, so every model converges.
#include "ecc_fixture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

using namespace ecc_fixture;

int main(int argc, char **argv) {
    std::printf("OpenCV %s\n", CV_VERSION);
    if (argc < 4 || std::strcmp(argv[1], "memory") != 0) {
        std::fprintf(stderr, "usage: memory ROWS COLS [translation|euclidean|affine]\n");
        return 2;
    }
    const int rows = std::atoi(argv[2]), cols = std::atoi(argv[3]);
    const std::string model = argc >= 5 ? argv[4] : "affine";
    const int motion = model == "translation" ? cv::MOTION_TRANSLATION
                     : model == "euclidean" ? cv::MOTION_EUCLIDEAN : cv::MOTION_AFFINE;
    cv::Mat a(rows, cols, CV_8UC1), b;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            a.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(signal(c % 256, r % 256));
    double shift_values[6] = {1, 0, 2, 0, 1, 1};
    const cv::Mat shift(2, 3, CV_64F, shift_values);
    cv::warpAffine(a, b, shift, a.size());
    cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
    const double rho = cv::findTransformECC(a, b, warp, motion,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 50, 1e-4),
        cv::noArray(), 5);
    std::ifstream status("/proc/self/status");
    for (std::string line; std::getline(status, line);)
        if (line.rfind("VmHWM:", 0) == 0) std::printf("%s\n", line.c_str());
    std::printf("%dx%d %s rho=%.4f M=[%.4f %.4f %.4f %.4f %.4f %.4f]\n", rows, cols, model.c_str(),
                rho, double(warp.at<float>(0, 0)), double(warp.at<float>(0, 1)),
                double(warp.at<float>(0, 2)), double(warp.at<float>(1, 0)),
                double(warp.at<float>(1, 1)), double(warp.at<float>(1, 2)));
    return 0;
}
