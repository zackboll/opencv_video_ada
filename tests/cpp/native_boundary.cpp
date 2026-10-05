#include "opencv_video_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include "synthetic_fixture.hpp"
#include <cstring>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

static_assert(sizeof(opencv_video_status) == 4, "status ABI width");
static_assert(sizeof(float) == 4 && sizeof(double) == 8, "floating ABI widths");
#ifdef OPENCV_VIDEO_TEST_FAULTS
int fault_stage = 0;
int fault_kind = 0;
void opencv_video_test_fault(int stage) {
    if (stage != fault_stage) return;
    if (fault_kind == 1) throw std::bad_alloc();
    if (fault_kind == 2) CV_Error(cv::Error::StsError, "qualification native exception");
    throw 42;
}
#endif

namespace {
void check(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

using Mat = std::unique_ptr<opencv_core_mat_handle, decltype(&opencv_core_mat_destroy)>;

Mat matrix(int rows, int columns, int depth, int channels) {
    opencv_core_mat_handle *raw = nullptr;
    check(opencv_core_mat_create_2d(rows, columns, depth, channels, &raw) == OPENCV_CORE_OK,
          "Core matrix factory failed");
    return Mat(raw, opencv_core_mat_destroy);
}

cv::Mat &output(opencv_core_mat_handle *handle) {
    cv::Mat *mat = nullptr;
    check(opencv_core_module_output_mat(handle, &mat) == OPENCV_CORE_OK && mat,
          "Core output resolver failed");
    return *mat;
}

void fill_texture(cv::Mat &m) {
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
            m.at<unsigned char>(r,c) = static_cast<unsigned char>((r*17 + c*29 + (r*c)%251) & 255);
}

void shift(const cv::Mat &source, cv::Mat &destination, int dx, int dy) {
    destination.setTo(cv::Scalar(0));
    for (int r = 0; r < destination.rows; ++r) {
        for (int c = 0; c < destination.cols; ++c) {
            const int sr = r - dy;
            const int sc = c - dx;
            if (sr >= 0 && sr < source.rows && sc >= 0 && sc < source.cols)
                destination.at<unsigned char>(r,c) = source.at<unsigned char>(sr,sc);
        }
    }
}

void run() {
    auto previous = matrix(64,64,OPENCV_CORE_DEPTH_UINT8,1);
    auto next = matrix(64,64,OPENCV_CORE_DEPTH_UINT8,1);
    auto points = matrix(4,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto next_points = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    auto status = matrix(1,1,OPENCV_CORE_DEPTH_UINT8,1);
    auto error = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);

    fill_texture(output(previous.get()));
    shift(output(previous.get()), output(next.get()), 2, 1);
    const cv::Vec2f fixtures[] = {{20,20},{32,24},{42,35},{26,45}};
    for (int i=0; i<4; ++i) output(points.get()).at<cv::Vec2f>(i,0) = fixtures[i];

    check(opencv_video_track_pyr_lk(previous.get(), next.get(), points.get(),
          next_points.get(), status.get(), error.get(), 21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_OK,
          "PyrLK raw boundary failed");
    check(output(next_points.get()).type() == CV_32FC2 && output(next_points.get()).rows == 4,
          "next-point output schema");
    check(output(status.get()).type() == CV_8UC1 && output(status.get()).rows == 4,
          "status output schema");
    check(output(error.get()).type() == CV_32FC1 && output(error.get()).rows == 4,
          "error output schema");
    for (int i=0; i<4; ++i) {
        check(output(status.get()).at<unsigned char>(i,0) == 1, "synthetic point not tracked");
        const cv::Vec2f p = output(next_points.get()).at<cv::Vec2f>(i,0);
        check(std::abs(p[0] - (fixtures[i][0] + 2.0f)) < 0.25f &&
              std::abs(p[1] - (fixtures[i][1] + 1.0f)) < 0.25f,
              "synthetic translation differs");
    }

    // Compare actual shim output to a direct native call on identical storage.
    cv::Mat oracle_next, oracle_status, oracle_error;
    cv::calcOpticalFlowPyrLK(output(previous.get()), output(next.get()), output(points.get()),
                            oracle_next, oracle_status, oracle_error);
    check(cv::norm(oracle_next, output(next_points.get()), cv::NORM_INF) < 1e-5 &&
          cv::norm(oracle_status, output(status.get()), cv::NORM_INF) == 0 &&
          cv::norm(oracle_error, output(error.get()), cv::NORM_INF) < 1e-5,
          "shim differs from independent native call");

    const cv::Mat before_next = output(next_points.get()).clone();
    const cv::Mat before_status = output(status.get()).clone();
    const cv::Mat before_error = output(error.get()).clone();
    check(opencv_video_track_pyr_lk(previous.get(), next.get(), points.get(),
          next_points.get(), status.get(), error.get(), 0,21,3,30,0.01,1e-4) ==
          OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "zero-width window accepted");
    check(cv::norm(before_next, output(next_points.get()), cv::NORM_INF) == 0 &&
          cv::norm(before_status, output(status.get()), cv::NORM_INF) == 0 &&
          cv::norm(before_error, output(error.get()), cv::NORM_INF) == 0,
          "failed call modified published outputs");

    check(opencv_video_track_pyr_lk(nullptr,next.get(),points.get(),next_points.get(),status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "null previous image accepted");
    check(opencv_video_track_pyr_lk(previous.get(),next.get(),points.get(),nullptr,status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "null output handle accepted");

    for (double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        check(opencv_video_track_pyr_lk(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),
              21,21,3,30,bad,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "invalid epsilon accepted");
    }
    for (double bad : {-1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        check(opencv_video_track_pyr_lk(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),
              21,21,3,30,0.01,bad) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "invalid eigenvalue threshold accepted");
    }

    auto wrong_depth = matrix(64,64,OPENCV_CORE_DEPTH_FLOAT32,1);
    check(opencv_video_track_pyr_lk(wrong_depth.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "wrong image depth accepted");
    auto wrong_points = matrix(4,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    check(opencv_video_track_pyr_lk(previous.get(),next.get(),wrong_points.get(),next_points.get(),status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "wrong point depth accepted");

    auto call = [&](const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                    const opencv_core_mat_handle *p, opencv_core_mat_handle *q,
                    opencv_core_mat_handle *s, opencv_core_mat_handle *e,
                    int w=21, int h=21, int level=3, int count=30,
                    double eps=0.01, double eig=1e-4) {
        return opencv_video_track_pyr_lk(a,b,p,q,s,e,w,h,level,count,eps,eig);
    };
    auto invalid = [&](opencv_video_status code) {
        check(code == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "invalid boundary argument accepted");
        check(std::strlen(opencv_video_last_error()) != 0, "missing failure diagnostic");
        check(cv::norm(before_next, output(next_points.get()), cv::NORM_INF) == 0 &&
              cv::norm(before_status, output(status.get()), cv::NORM_INF) == 0 &&
              cv::norm(before_error, output(error.get()), cv::NORM_INF) == 0,
              "failure not atomic");
    };
    for (int slot=0; slot<6; ++slot)
        invalid(call(slot==0?nullptr:previous.get(),slot==1?nullptr:next.get(),
                     slot==2?nullptr:points.get(),slot==3?nullptr:next_points.get(),
                     slot==4?nullptr:status.get(),slot==5?nullptr:error.get()));
    invalid(call(previous.get(),next.get(),points.get(),next_points.get(),next_points.get(),error.get()));
    invalid(call(previous.get(),next.get(),points.get(),previous.get(),status.get(),error.get()));
    auto wrong_channels = matrix(64,64,OPENCV_CORE_DEPTH_UINT8,3);
    auto wrong_geometry = matrix(63,64,OPENCV_CORE_DEPTH_UINT8,1);
    invalid(call(previous.get(),wrong_channels.get(),points.get(),next_points.get(),status.get(),error.get()));
    invalid(call(previous.get(),wrong_geometry.get(),points.get(),next_points.get(),status.get(),error.get()));
    for (int bad : {-1,0,1,2,256,std::numeric_limits<int>::max()}) {
        invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),bad));
        invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),21,bad));
    }
    for (int bad : {-1,31,std::numeric_limits<int>::max()})
        invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),21,21,bad));
    for (int bad : {-1,0,101,std::numeric_limits<int>::max()})
        invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),21,21,3,bad));
    invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),21,21,3,30,11.0));
    invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),21,21,3,30,0.01,
                 std::numeric_limits<double>::max()));
    for (float bad : {std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::max()}) {
        output(points.get()).at<cv::Vec2f>(0,0)[0]=bad;
        invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get()));
    }
    output(points.get()).at<cv::Vec2f>(0,0)=fixtures[0];
    auto empty_image = matrix(0,0,OPENCV_CORE_DEPTH_UINT8,1);
    invalid(call(empty_image.get(),next.get(),points.get(),next_points.get(),status.get(),error.get()));
    auto bad_shape = matrix(2,2,OPENCV_CORE_DEPTH_FLOAT32,2);
    invalid(call(previous.get(),next.get(),bad_shape.get(),next_points.get(),status.get(),error.get()));

#ifdef OPENCV_VIDEO_TEST_FAULTS
    for (int stage : {1,2}) for (int kind : {1,2,3}) {
        fault_stage=stage; fault_kind=kind;
        const auto code=call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get());
        check(code == (kind==1?OPENCV_VIDEO_ERROR_STANDARD:
                       kind==2?OPENCV_VIDEO_ERROR_OPENCV:OPENCV_VIDEO_ERROR_UNKNOWN),
              "exception escaped or wrong status");
        check(cv::norm(before_next,output(next_points.get()),cv::NORM_INF)==0 &&
              cv::norm(before_status,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(before_error,output(error.get()),cv::NORM_INF)==0, "fault published partial output");
    }
    fault_stage=0;
    std::cout << "PASS: 6 injected allocation/native/unknown failures contained\n";
#endif
    output(points.get()).at<cv::Vec2f>(1,0)=cv::Vec2f(-100,-100);
    output(points.get()).at<cv::Vec2f>(3,0)=cv::Vec2f(200,200);
    check(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get())==OPENCV_VIDEO_OK,
          "mixed outside points rejected");
    for (int i : {1,3}) {
        check(output(status.get()).at<unsigned char>(i,0)==0 &&
              output(next_points.get()).at<cv::Vec2f>(i,0)==output(points.get()).at<cv::Vec2f>(i,0) &&
              output(error.get()).at<float>(i,0)==0, "failed native output not normalized");
    }
    auto empty_points=matrix(0,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    check(call(previous.get(),next.get(),empty_points.get(),next_points.get(),status.get(),error.get())==OPENCV_VIDEO_OK &&
          output(next_points.get()).empty() && output(status.get()).empty() && output(error.get()).empty(),
          "empty points did not clear all outputs");
    check(std::strlen(opencv_video_last_error())==0,"stale success diagnostic");

    std::cout << "PASS: Video PyrLK raw boundary on "
              << opencv_video_native_version() << " / " << opencv_video_native_backend() << '\n';
}
}  // namespace

int main() {
    try {
        run();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
