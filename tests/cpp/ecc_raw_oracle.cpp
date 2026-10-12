// Direct-OpenCV oracle for the Ada smoke: reads the raw 128x128 UInt8 images the Ada
// program wrote and runs cv::findTransformECC itself. No Video shim, no Core bridge.
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

static cv::Mat load(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    std::vector<char> bytes(128 * 128);
    in.read(bytes.data(), std::streamsize(bytes.size()));
    if (!in) { std::fprintf(stderr, "cannot read %s\n", path.c_str()); std::exit(2); }
    cv::Mat m(128, 128, CV_8UC1);
    std::memcpy(m.data, bytes.data(), bytes.size());
    return m;
}

int main(int argc, char **argv) {
    if (argc < 2) return 2;
    std::printf("opencv %s\n", CV_VERSION);
    for (const char *name : {"identity", "positive", "negative", "horizontal", "vertical"}) {
        const std::string base = std::string(argv[1]) + "/" + name;
        cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
        const double rho = cv::findTransformECC(load(base + ".template.raw"), load(base + ".moving.raw"),
            warp, cv::MOTION_TRANSLATION,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 50, 1e-4),
            cv::noArray(), 5);
        std::printf("ECC %s %.17g %.17g %.17g\n", name, double(warp.at<float>(0, 2)),
                    double(warp.at<float>(1, 2)), rho);
    }
    return 0;
}
