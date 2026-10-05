#ifndef VIDEO_SYNTHETIC_FIXTURE_HPP
#define VIDEO_SYNTHETIC_FIXTURE_HPP
#include <opencv2/core.hpp>
#include <vector>

inline cv::Mat texture(int n) {
    cv::Mat image(n, n, CV_8UC1);
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c)
            image.at<unsigned char>(r, c) =
                static_cast<unsigned char>((r * 17 + c * 29 + (r * c) % 251) % 256);
    return image;
}
inline cv::Mat translated(const cv::Mat &source, int dx, int dy) {
    cv::Mat result(source.size(), source.type(), cv::Scalar(0));
    source(cv::Rect(0, 0, source.cols - dx, source.rows - dy)).copyTo(
        result(cv::Rect(dx, dy, source.cols - dx, source.rows - dy)));
    return result;
}
inline std::vector<cv::Point2f> fixture_points(int n) {
    if (n == 64) return {{20,20}, {32,24}, {42,35}, {26,45}};
    return {{25,25}, {45,32}, {60,50}, {35,65}};
}
#endif