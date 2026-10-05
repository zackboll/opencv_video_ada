// Independent native oracle: deliberately no Video or Core bridge imports.
#include "synthetic_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    try {
        for (int n : {64, 96}) {
            const auto previous = texture(n);
            const int dx = n == 64 ? 2 : 3, dy = n == 64 ? 1 : 2;
            const auto next = translated(previous, dx, dy);
            const auto points = fixture_points(n);
            std::vector<cv::Point2f> result;
            std::vector<unsigned char> status;
            std::vector<float> error;
            cv::calcOpticalFlowPyrLK(previous, next, points, result, status, error);
            for (size_t i = 0; i < points.size(); ++i) {
                if (status[i] != 1 || std::abs(result[i].x - points[i].x - dx) > 0.20f ||
                    std::abs(result[i].y - points[i].y - dy) > 0.20f ||
                    !std::isfinite(error[i]) || error[i] < 0)
                    throw std::runtime_error("direct native translation differs");
                std::cout << "oracle " << n << " point " << i << " status=" << int(status[i])
                          << " next=" << result[i] << " error=" << error[i] << '\n';
            }
        }
        std::cout << "PASS: independent OpenCV " << CV_VERSION << " oracle (8 tracks)\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}