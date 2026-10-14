#ifndef OPENCV_VIDEO_ECC_FIXTURE_HPP
#define OPENCV_VIDEO_ECC_FIXTURE_HPP
// Deterministic ECC fixtures. Header-only, depends only on OpenCV core.
// Content moves by (dx, dy) from template to moving image:
//   moving(x, y) == template(x - dx, y - dy)
// so the native ECC translation (moving = template + t) is expected to be +(dx, dy).
#include <opencv2/core.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>

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

// Task 012 rigid fixture. The native Euclidean warp W maps template coordinates to
// moving coordinates:  moving = R(theta) * template + (tx, ty),  theta = degrees*pi/180.
// moving(p) = signal(W^-1 p), W^-1 p = R(-theta) * (p - t), evaluated analytically (no
// image resampling), so Ada can reproduce it exactly. theta = 0 reduces to texture().
inline cv::Mat rigid_texture(int rows, int cols, double degrees, double tx, double ty) {
    const double theta = degrees * 3.14159265358979323846 / 180.0;
    const double c = std::cos(theta), s = std::sin(theta);
    cv::Mat m(rows, cols, CV_8UC1);
    for (int r = 0; r < rows; ++r)
        for (int col = 0; col < cols; ++col) {
            const double u = col - tx, v = r - ty;
            m.at<uint8_t>(r, col) = cv::saturate_cast<uint8_t>(signal(c * u + s * v, -s * u + c * v));
        }
    return m;
}

// Task 014 affine fixture. The native affine warp W = [A | t] maps template coordinates to
// moving coordinates:  moving = A * template + t,  A = [[a b][c d]].
// moving(p) = signal(A^-1 (p - t)), evaluated analytically (no image resampling) with an
// explicit 2x2 inverse so Ada can reproduce it. A must be invertible.
inline cv::Mat affine_texture(int rows, int cols, double a, double b, double c, double d,
                              double tx, double ty) {
    const double det = a * d - b * c;
    const double ia = d / det, ib = -b / det, ic = -c / det, id = a / det;
    cv::Mat m(rows, cols, CV_8UC1);
    for (int r = 0; r < rows; ++r)
        for (int col = 0; col < cols; ++col) {
            const double u = col - tx, v = r - ty;
            m.at<uint8_t>(r, col) = cv::saturate_cast<uint8_t>(signal(ia * u + ib * v, ic * u + id * v));
        }
    return m;
}

// Task 016 homography fixture. The native warp H (row-major h[0..8]) maps template
// coordinates to moving coordinates with homogeneous division:
//   moving(H p) == template(p)   i.e.   moving(p) = signal(H^-1 p).
// Evaluated analytically (no image resampling) with an explicit adjugate inverse and
// explicit denominator handling so Ada can reproduce it. H must be invertible and the
// inverse denominator must be nonzero at every pixel; otherwise std::invalid_argument is
// thrown (no pixel is invented). Expression order is mirrored in ECC_Fixtures.Homography_Texture.
inline cv::Mat homography_texture(int rows, int cols, const double *h) {
    const double c00 = h[4] * h[8] - h[5] * h[7];
    const double c01 = h[5] * h[6] - h[3] * h[8];
    const double c02 = h[3] * h[7] - h[4] * h[6];
    const double det = h[0] * c00 + h[1] * c01 + h[2] * c02;
    if (!(std::fabs(det) > 1e-12)) throw std::invalid_argument("singular homography");
    const double i0 = c00 / det, i1 = (h[2] * h[7] - h[1] * h[8]) / det,
                 i2 = (h[1] * h[5] - h[2] * h[4]) / det;
    const double i3 = c01 / det, i4 = (h[0] * h[8] - h[2] * h[6]) / det,
                 i5 = (h[2] * h[3] - h[0] * h[5]) / det;
    const double i6 = c02 / det, i7 = (h[1] * h[6] - h[0] * h[7]) / det,
                 i8 = (h[0] * h[4] - h[1] * h[3]) / det;
    cv::Mat m(rows, cols, CV_8UC1);
    for (int r = 0; r < rows; ++r)
        for (int col = 0; col < cols; ++col) {
            const double u = col, v = r;
            const double d = i6 * u + i7 * v + i8;
            if (d == 0.0) throw std::invalid_argument("zero projective denominator");
            m.at<uint8_t>(r, col) = cv::saturate_cast<uint8_t>(
                signal((i0 * u + i1 * v + i2) / d, (i3 * u + i4 * v + i5) / d));
        }
    return m;
}

inline cv::Mat photometric(const cv::Mat &m, double alpha, double beta) {
    cv::Mat out;
    m.convertTo(out, CV_8UC1, alpha, beta);
    return out;
}

// Exact integer photometric map: sat(floor((v*num + den/2)/den) + beta). No floating
// rounding, so Ada and C++ build identical pixels.
inline cv::Mat affine_int(const cv::Mat &m, int num, int den, int beta) {
    cv::Mat out(m.rows, m.cols, CV_8UC1);
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c) {
            const int v = m.at<uint8_t>(r, c);
            const int q = (v * num + den / 2) / den + beta;
            out.at<uint8_t>(r, c) = uint8_t(q < 0 ? 0 : q > 255 ? 255 : q);
        }
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
