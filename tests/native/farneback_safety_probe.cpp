// Standalone upstream safety reproducer, not a binding qualification test.
// Link directly to OpenCV; do not link the Video shim or Core bridge.
#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    cv::Mat image(16, 16, CV_8UC1);
    for (int y = 0; y < image.rows; ++y)
        for (int x = 0; x < image.cols; ++x)
            image.at<unsigned char>(y, x) =
                static_cast<unsigned char>((x * 17 + y * 31 + (x * y) % 97) % 256);
    cv::Mat next = cv::Mat::zeros(image.size(), CV_8UC1);
    image(cv::Rect(0, 0, 14, 15)).copyTo(next(cv::Rect(2, 1, 14, 15)));
    cv::Mat flow(image.size(), CV_32FC2,
                 cv::Scalar::all(std::numeric_limits<float>::quiet_NaN()));
    cv::calcOpticalFlowFarneback(image, next, flow, .5, 3, 15, 3, 5, 1.2, 0);
    std::size_t nonfinite = 0;
    for (int y = 0; y < flow.rows; ++y)
        for (int x = 0; x < flow.cols; ++x) {
            const auto value = flow.at<cv::Vec2f>(y, x);
            nonfinite += !std::isfinite(value[0]);
            nonfinite += !std::isfinite(value[1]);
        }
    std::cout << CV_VERSION << " nonfinite=" << nonfinite << '\n';
    return nonfinite == 0 ? 0 : 1;
}