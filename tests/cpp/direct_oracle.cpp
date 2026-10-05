// Independent native oracle: deliberately no Video or Core bridge imports.
#include "synthetic_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>

// Diagnostic oracle: direct OpenCV only, including compact-success mapping.
static void forward_backward(std::ostream &output, int mode) {
    const auto previous = texture(96);
    auto next = translated(previous, mode == 0 ? 2 : 12, mode == 0 ? 1 : 7);
    if (mode == 3) next.setTo(0);
    const std::vector<cv::Point2f> points{{25,25}, {-1000,-1000}, {45,32},
                                        {1000,1000}, {60,50}};
    auto forward = points;
    for (auto &p : forward) p += cv::Point2f(12.25f, 6.75f);
    const cv::TermCriteria criteria(3, 30, 0.01);
    std::vector<unsigned char> status, back_status;
    std::vector<float> errors, back_errors;
    cv::calcOpticalFlowPyrLK(previous, next, points, forward, status, errors,
                            {21,21}, mode == 0 ? 1 : 0, criteria,
                            mode == 1 || mode == 3 ? cv::OPTFLOW_USE_INITIAL_FLOW : 0);
    std::vector<cv::Point2f> starts, recovered;
    std::vector<size_t> indices;
    for (size_t i = 0; i < points.size(); ++i) if (status[i]) {
        starts.push_back(forward[i]); recovered.push_back(points[i]); indices.push_back(i);
    }
    if (!starts.empty())
        cv::calcOpticalFlowPyrLK(next, previous, starts, recovered, back_status, back_errors,
                                {21,21}, mode == 0 ? 1 : 0, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
    size_t j = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        const bool f = status[i] != 0;
        const bool b = f && back_status[j] != 0;
        const auto dest = f ? forward[i] : points[i];
        const auto back = b ? recovered[j] : points[i];
        const double dx = double(back.x) - double(points[i].x);
        const double dy = double(back.y) - double(points[i].y);
        const double distance = b ? std::sqrt(dx * dx + dy * dy) : 0;
        const bool outside = i == 1 || i == 3;
        if ((outside && (f || b)) ||
            (!outside && mode <= 1 && (!f || !b || distance >= .05)) ||
            (i == 0 && mode == 2 && (!f || !b || distance <= 2.0)) ||
            (!outside && mode == 3 && (!f || b)))
            throw std::runtime_error("forward/backward semantic fixture failed");
        output << mode << ' ' << i + 7 << ' ' << int(f) << ' ' << dest.x << ' ' << dest.y
               << ' ' << int(b) << ' ' << back.x << ' ' << back.y << ' ' << distance << '\n';
        if (f) {
            if (indices[j] != i) throw std::runtime_error("oracle compact mapping failed");
            ++j;
        }
    }
}

int main(int argc, char **argv) {
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
        std::ofstream file;
        if (argc == 2) {
            file.open(argv[1]);
            if (!file) throw std::runtime_error("cannot create oracle results");
        }
        auto &output = argc == 2 ? static_cast<std::ostream &>(file) : std::cout;
        output << std::setprecision(17);
        for (int mode = 0; mode < 4; ++mode) forward_backward(output, mode);
        std::cout << "PASS: independent OpenCV " << CV_VERSION
                  << " oracle (8 unseeded + 4 seeded + 20 forward/backward entries)\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}