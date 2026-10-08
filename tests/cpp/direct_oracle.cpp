// Independent native oracle: deliberately no Video or Core bridge imports.
#include "synthetic_fixture.hpp"
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <limits>

static void trackability(std::ostream &output) {
    float structures[3] = {};
    std::vector<float> identity;
    for (int mode = 0; mode < 9; ++mode) {
        auto f = quality_fixture(mode);
        cv::Mat points(int(f.points.size()),1,CV_32FC2,
                       f.points.data());
        cv::Mat dest(int(f.seeds.size()),1,CV_32FC2,
                     f.seeds.data());
        dest = dest.clone();
        cv::Mat status;
        cv::Mat values(int(f.points.size()),1,CV_32F,
                       cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
        const auto *storage = values.data;
        cv::calcOpticalFlowPyrLK(f.previous,f.next,points,dest,status,values,{21,21},0,{3,30,.01},
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS | (f.seeded ? cv::OPTFLOW_USE_INITIAL_FLOW : 0),f.threshold);
        if (storage != values.data) throw std::runtime_error("native quality storage not reused");
        for (int i = 0; i < int(f.points.size()); ++i) {
            const float e = values.at<float>(i);
            const bool tracked = status.at<unsigned char>(i) != 0;
            if (!std::isfinite(e) || e < 0) {
                std::cerr << "native quality violation: version=" << CV_VERSION << " mode=" << mode
                          << " point=" << i << " status=" << int(tracked) << " eigenvalue=" << e
                          << " previous=" << f.points[i] << " seed=" << f.seeds[i] << '\n';
                // KleidiCV 26.03 intentionally skips err for unavailable prev.
                // Record absence explicitly; Ada/shim MUST reject the whole call.
                if (mode == 3 && i == 1 && !tracked && std::isnan(e)) {
                    output << mode << ' ' << i << " -1 " << f.points[i].x << ' '
                           << f.points[i].y << " 0\n";
                    continue;
                }
                throw std::runtime_error("undefined native eigenvalue");
            }
            if (mode == 0) identity.push_back(e);
            if (mode >= 5 && mode <= 7) structures[mode-5] = e;
            if ((mode == 0 || mode == 1 || mode == 5) && (!tracked || e <= .1f))
                throw std::runtime_error("strong quality fixture failed");
            if (mode == 1 && cv::norm(dest.at<cv::Point2f>(i)-f.points[i]-cv::Point2f(12,7)) >= .05)
                throw std::runtime_error("quality seed not consumed");
            if (mode == 2 && (tracked || std::abs(e-identity[i]) > 1e-5))
                throw std::runtime_error("threshold discarded quality");
            if (mode == 3 && i == 1 && (tracked || e != 0))
                throw std::runtime_error("previous patch quality not zero");
            if (mode == 4 && i == 1 && (tracked || std::abs(e-identity[i]) > 1e-5))
                throw std::runtime_error("next search discarded quality");
            if (mode == 8 && std::abs(e-identity[i]) > 1e-5)
                throw std::runtime_error("seed changed previous-patch quality");
            const auto p = tracked ? dest.at<cv::Point2f>(i) : f.points[i];
            output << mode << ' ' << i << ' ' << int(tracked) << ' ' << p.x << ' ' << p.y << ' ' << e << '\n';
        }
    }
    if (structures[0] <= .5f || structures[1] != 0 || structures[2] != 0)
        throw std::runtime_error("corner/edge/flat relationship failed");
    const auto image = quality_structure(0);
    for (double multiplier : {.5,2.0}) {
        std::vector<cv::Point2f> points{{48,48}}, dest;
        std::vector<unsigned char> status;
        std::vector<float> values;
        cv::calcOpticalFlowPyrLK(image,image,points,dest,status,values,{21,21},0,{3,30,.01},
                                cv::OPTFLOW_LK_GET_MIN_EIGENVALS,structures[0]*multiplier);
        if (bool(status[0]) != (multiplier < 1) || std::abs(values[0]-structures[0]) > 1e-5)
            throw std::runtime_error("factor-two eigenvalue threshold failed");
    }
    std::cout << "quality corner=" << structures[0] << " edge=" << structures[1]
              << " flat=" << structures[2] << '\n';
}

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
        for (int n : {32,64,96,256}) for (int requested : {0,3,30}) {
            const auto a=texture(n), b=translated(a,2,1);
            std::vector<cv::Mat> pa,pb;
            const int available=cv::buildOpticalFlowPyramid(a,pa,{21,21},requested,true,
                cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
            cv::buildOpticalFlowPyramid(b,pb,{21,21},requested,true,
                cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
            int expected=0, size=n;
            while (expected<requested && (size=(size+1)/2)>21) ++expected;
            if (available!=expected || pa.size()!=size_t(2*(available+1)))
                throw std::runtime_error("direct pyramid metadata");
            for (int level=0;level<=available;++level)
                if (pa[size_t(2*level)].type()!=CV_8UC1 || pa[size_t(2*level+1)].type()!=CV_16SC2)
                    throw std::runtime_error("direct pyramid types");
            if (n>=64) {
                const auto points=fixture_points(n==64?64:96);
                std::vector<cv::Point2f> raw,built;
                std::vector<unsigned char> rs,bs;
                std::vector<float> re,be;
                cv::calcOpticalFlowPyrLK(a,b,points,raw,rs,re,{21,21},requested,{3,30,.01},0);
                cv::calcOpticalFlowPyrLK(pa,pb,points,built,bs,be,{21,21},requested,{3,30,.01},0);
                if (rs!=bs) throw std::runtime_error("direct raw/prebuilt status");
                for (size_t i=0;i<points.size();++i)
                    if (rs[i] && (cv::norm(raw[i]-built[i])>1e-5 || std::abs(re[i]-be[i])>1e-5))
                        throw std::runtime_error("direct raw/prebuilt values");
            }
            std::cout << "pyramid oracle n=" << n << " requested=" << requested
                      << " available=" << available << " entries=" << pa.size() << '\n';
        }
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
        if (argc >= 2) {
            file.open(argv[1]);
            if (!file) throw std::runtime_error("cannot create oracle results");
        }
        auto &output = argc >= 2 ? static_cast<std::ostream &>(file) : std::cout;
        output << std::setprecision(17);
        for (int mode = 0; mode < 4; ++mode) forward_backward(output, mode);
        std::ofstream quality_file;
        if (argc == 3) {
            quality_file.open(argv[2]);
            if (!quality_file) throw std::runtime_error("cannot create quality oracle results");
        }
        auto &quality_output = argc == 3 ? static_cast<std::ostream &>(quality_file) : std::cout;
        quality_output << std::setprecision(17);
        trackability(quality_output);
        std::cout << "PASS: independent OpenCV " << CV_VERSION
                  << " oracle (8 unseeded + 4 seeded + 20 forward/backward + 27 quality entries)\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}