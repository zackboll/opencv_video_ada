#ifndef OPENCV_VIDEO_ECC_FIXTURE_HPP
#define OPENCV_VIDEO_ECC_FIXTURE_HPP
// Deterministic ECC fixtures. Header-only, depends only on OpenCV core.
// Content moves by (dx, dy) from template to moving image:
//   moving(x, y) == template(x - dx, y - dy)
// so the native ECC translation (moving = template + t) is expected to be +(dx, dy).
#include <opencv2/core.hpp>
#include <cmath>
#include <cstdint>

namespace ecc_fixture {

// Smooth, non-periodic analytic texture, always inside 1 .. 254.
inline double signal(double x, double y) {
    auto blob = [&](double cx, double cy, double a, double s) {
        const double u = x - cx, v = y - cy;
        return a * std::exp(-(u * u + v * v) / (2.0 * s * s));
    };
    return 127.5 + 30.0 * std::sin(0.21 * x + 0.13 * y) + 25.0 * std::sin(0.11 * x - 0.27 * y + 1.0)
        + 20.0 * std::cos(0.37 * x + 0.05 * y + 2.0) + 15.0 * std::sin(0.06 * x + 0.31 * y)
        + blob(40, 50, 18, 6) + blob(90, 35, -18, 5) + blob(70, 95, 18, 7);
}

// Texture sampled at (x - dx, y - dy): content shifted by (+dx, +dy).
inline cv::Mat texture(int rows, int cols, int dx = 0, int dy = 0) {
    cv::Mat m(rows, cols, CV_8UC1);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            m.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(signal(c - dx, r - dy));
    return m;
}

inline cv::Mat photometric(const cv::Mat &m, double alpha, double beta) {
    cv::Mat out;
    m.convertTo(out, CV_8UC1, alpha, beta);
    return out;
}

// Deterministic LCG noise: poorly correlated with any texture.
inline cv::Mat noise(int rows, int cols, uint32_t seed) {
    cv::Mat m(rows, cols, CV_8UC1);
    uint32_t s = seed;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            s = s * 1664525u + 1013904223u;
            m.at<uint8_t>(r, c) = uint8_t(s >> 24);
        }
    return m;
}

inline cv::Mat constant(int rows, int cols, uint8_t value) {
    return cv::Mat(rows, cols, CV_8UC1, cv::Scalar(value));
}

// Vertical stripes: varies in x only, so the y gradient is identically zero.
inline cv::Mat stripes(int rows, int cols, int dx) {
    cv::Mat m(rows, cols, CV_8UC1);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            m.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(
                128.0 + 60.0 * std::sin(2.0 * 3.14159265358979323846 * (c - dx) / 16.0));
    return m;
}

// Very low contrast: values confined to a few gray levels.
inline cv::Mat low_variance(const cv::Mat &textured) {
    cv::Mat out;
    textured.convertTo(out, CV_8UC1, 1.0 / 40.0, 126.0);
    return out;
}

}  // namespace ecc_fixture

#endif
