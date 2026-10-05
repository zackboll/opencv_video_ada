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
        const auto previous = texture(96), next = translated(previous, 12, 7);
        const auto points = fixture_points(96);
        auto seeds = points;
        for (auto &p : seeds) p += cv::Point2f(12.25f, 6.75f);
        std::vector<cv::Point2f> unseeded, seeded = seeds;
        std::vector<unsigned char> status, plain_status;
        std::vector<float> error, plain_error;
        const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
        cv::calcOpticalFlowPyrLK(previous, next, points, unseeded, plain_status, plain_error,
                                cv::Size(21,21), 0, criteria, 0);
        cv::calcOpticalFlowPyrLK(previous, next, points, seeded, status, error,
                                cv::Size(21,21), 0, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
        for (size_t i = 0; i < points.size(); ++i) {
            const auto expected = points[i] + cv::Point2f(12,7);
            if (status[i] != 1 || cv::norm(seeded[i] - expected) > 0.05 ||
                !std::isfinite(error[i]) || error[i] < 0 ||
                (plain_status[i] && cv::norm(unseeded[i] - expected) < 5.0) ||
                cv::norm(seeded[i] - seeds[i]) < 0.20)
                throw std::runtime_error("seed consumption/refinement oracle failed");
            std::cout << "seed oracle point " << i << " plain=" << unseeded[i]
                      << " prediction=" << seeds[i] << " refined=" << seeded[i]
                      << " error=" << error[i] << '\n';
        }
        std::cout << "PASS: independent OpenCV " << CV_VERSION
                  << " oracle (8 unseeded + 4 distinguishing seeded tracks)\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}