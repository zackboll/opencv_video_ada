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
// Previous-patch quality structures: 2-D corner, 1-D edge, flat.
inline cv::Mat quality_structure(int kind) {
    cv::Mat image(96, 96, CV_8UC1);
    for (int r = 0; r < 96; ++r)
        for (int c = 0; c < 96; ++c)
            image.at<unsigned char>(r,c) = kind == 0 ? (r >= 48 && c >= 48 ? 255 : 0) :
                                          kind == 1 ? (c >= 48 ? 255 : 0) : 127;
    return image;
}
struct QualityFixture {
    cv::Mat previous, next;
    std::vector<cv::Point2f> points, seeds;
    bool seeded = false;
    double threshold = 1e-4;
};
inline QualityFixture quality_fixture(int mode) {
    QualityFixture f;
    f.previous = mode >= 5 && mode <= 7 ? quality_structure(mode - 5) : texture(96);
    f.next = mode == 1 ? translated(f.previous, 12, 7) : f.previous.clone();
    f.points = mode >= 5 && mode <= 7 ? std::vector<cv::Point2f>{{48,48}} : fixture_points(96);
    f.seeds = f.points;
    f.seeded = mode == 1 || mode == 4 || mode == 8;
    if (mode == 1) for (auto &p : f.seeds) p += cv::Point2f(12.25f,6.75f);
    if (mode == 2) f.threshold = 100; // far above measured texture quality
    if (mode == 3) f.points[1] = {-1000,-1000};
    if (mode == 4) f.seeds[1] = {-1000,-1000};
    if (mode == 8) for (auto &p : f.seeds) p += cv::Point2f(32,20);
    return f;
}
#endif