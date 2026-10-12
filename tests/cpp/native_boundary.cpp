#include "opencv_video_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include "synthetic_fixture.hpp"
#include "ecc_fixture.hpp"
#include <cstddef>
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
    if (fault_kind == 4) throw std::runtime_error("qualification standard exception");
    throw 42;
}
// ECC: which post-native value to corrupt before validation (test builds only).
// 1 warp[2]=NaN, 2 warp[5]=+Inf, 3 correlation=NaN, 4 correlation=-Inf, 5 warp[0]=1.5
// (linear part no longer identity), 6 warp[1]=0.25.
int ecc_corrupt_mode = 0;
void opencv_video_test_corrupt_ecc(float *warp, double *correlation) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    switch (ecc_corrupt_mode) {
        case 1: warp[2] = nan; break;
        case 2: warp[5] = std::numeric_limits<float>::infinity(); break;
        case 3: *correlation = std::numeric_limits<double>::quiet_NaN(); break;
        case 4: *correlation = -std::numeric_limits<double>::infinity(); break;
        case 5: warp[0] = 1.5f; break;
        case 6: warp[1] = 0.25f; break;
        default: break;
    }
}
int corrupt_mode = 0;  // 1 NaN, 2 +Inf, 3 -Inf written into one private component
void opencv_video_test_corrupt_flow(float *values, size_t count) {
    if (corrupt_mode == 0 || count == 0) return;
    values[count / 2] = corrupt_mode == 1 ? std::numeric_limits<float>::quiet_NaN()
                      : corrupt_mode == 2 ? std::numeric_limits<float>::infinity()
                                          : -std::numeric_limits<float>::infinity();
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

bool unavailable_previous_quality_is_defined() {
    const auto image=texture(96);
    std::vector<cv::Point2f> points{{-1000,-1000}}, dest;
    std::vector<unsigned char> status;
    cv::Mat values(1,1,CV_32F,cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    cv::calcOpticalFlowPyrLK(image,image,points,dest,status,values,{21,21},0,{3,30,.01},8);
    check(status[0]==0,"outside previous point unexpectedly tracked");
    const float e=values.at<float>(0);
    check(e==0 || std::isnan(e),"unexpected unavailable previous quality");
    return e==0;
}

void run(bool quality = false) {
    const auto track = quality ? opencv_video_track_pyr_lk_min_eigenvalues : opencv_video_track_pyr_lk;
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

    check(track(previous.get(), next.get(), points.get(),
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
                            oracle_next, oracle_status, oracle_error, {21,21},3,{3,30,.01},
                            quality ? cv::OPTFLOW_LK_GET_MIN_EIGENVALS : 0);
    check(cv::norm(oracle_next, output(next_points.get()), cv::NORM_INF) < 1e-5 &&
          cv::norm(oracle_status, output(status.get()), cv::NORM_INF) == 0 &&
          cv::norm(oracle_error, output(error.get()), cv::NORM_INF) < 1e-5,
          "shim differs from independent native call");

    const cv::Mat before_next = output(next_points.get()).clone();
    const cv::Mat before_status = output(status.get()).clone();
    const cv::Mat before_error = output(error.get()).clone();
    check(track(previous.get(), next.get(), points.get(),
          next_points.get(), status.get(), error.get(), 0,21,3,30,0.01,1e-4) ==
          OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "zero-width window accepted");
    check(cv::norm(before_next, output(next_points.get()), cv::NORM_INF) == 0 &&
          cv::norm(before_status, output(status.get()), cv::NORM_INF) == 0 &&
          cv::norm(before_error, output(error.get()), cv::NORM_INF) == 0,
          "failed call modified published outputs");

    check(track(nullptr,next.get(),points.get(),next_points.get(),status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "null previous image accepted");
    check(track(previous.get(),next.get(),points.get(),nullptr,status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "null output handle accepted");

    for (double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        check(track(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),
              21,21,3,30,bad,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "invalid epsilon accepted");
    }
    for (double bad : {-1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        check(track(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),
              21,21,3,30,0.01,bad) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "invalid eigenvalue threshold accepted");
    }

    auto wrong_depth = matrix(64,64,OPENCV_CORE_DEPTH_FLOAT32,1);
    check(track(wrong_depth.get(),next.get(),points.get(),next_points.get(),status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "wrong image depth accepted");
    auto wrong_points = matrix(4,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    check(track(previous.get(),next.get(),wrong_points.get(),next_points.get(),status.get(),error.get(),
          21,21,3,30,0.01,1e-4) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "wrong point depth accepted");

    auto call = [&](const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                    const opencv_core_mat_handle *p, opencv_core_mat_handle *q,
                    opencv_core_mat_handle *s, opencv_core_mat_handle *e,
                    int w=21, int h=21, int level=3, int count=30,
                    double eps=0.01, double eig=1e-4) {
        return track(a,b,p,q,s,e,w,h,level,count,eps,eig);
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
    for (auto *input : {previous.get(),next.get(),points.get()}) {
        invalid(call(previous.get(),next.get(),points.get(),input,status.get(),error.get()));
        invalid(call(previous.get(),next.get(),points.get(),next_points.get(),input,error.get()));
        invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),input));
    }
    invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),next_points.get()));
    invalid(call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),status.get()));
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
    const auto outside_code=call(previous.get(),next.get(),points.get(),next_points.get(),status.get(),error.get());
    const bool missing_quality=quality && !unavailable_previous_quality_is_defined();
    check(outside_code==(missing_quality ? OPENCV_VIDEO_ERROR_OPENCV : OPENCV_VIDEO_OK),
          "mixed outside point result differs from native definedness");
    if (missing_quality) {
        check(cv::norm(before_next,output(next_points.get()),cv::NORM_INF)==0 &&
              cv::norm(before_status,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(before_error,output(error.get()),cv::NORM_INF)==0,"undefined quality not atomic");
    }
    for (int i : {1,3}) if (!missing_quality) {
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
              << (quality ? "quality " : "photometric ") << opencv_video_native_version() << " / " << opencv_video_native_backend() << '\n';
}
void run_seeded(bool quality = false) {
    const auto track = quality ? opencv_video_track_pyr_lk_seeded_min_eigenvalues : opencv_video_track_pyr_lk_seeded;
    auto previous = matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto next = matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto points = matrix(4,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto seeds = matrix(4,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto result = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    auto status = matrix(1,1,OPENCV_CORE_DEPTH_UINT8,1);
    auto error = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    output(previous.get()) = texture(96);
    output(next.get()) = translated(output(previous.get()),12,7);
    const auto fixtures = fixture_points(96);
    for (int i=0; i<4; ++i) {
        output(points.get()).at<cv::Point2f>(i,0) = fixtures[i];
        output(seeds.get()).at<cv::Point2f>(i,0) = fixtures[i] + cv::Point2f(12.25f,6.75f);
    }
    const auto saved_seeds = output(seeds.get()).clone();
    const auto saved_previous = output(previous.get()).clone();
    const auto saved_next = output(next.get()).clone();
    auto call = [&](const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                    const opencv_core_mat_handle *p, const opencv_core_mat_handle *seed,
                    opencv_core_mat_handle *q, opencv_core_mat_handle *s, opencv_core_mat_handle *e,
                    int level=0, int w=21, double eps=.01) {
        return track(a,b,p,seed,q,s,e,w,21,level,30,eps,1e-4);
    };
    auto valid = [&]() {
        check(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())
              == OPENCV_VIDEO_OK, "seeded call failed");
    };
    valid();
    cv::Mat oracle_next = saved_seeds.clone(), oracle_status, oracle_error;
    cv::calcOpticalFlowPyrLK(output(previous.get()),output(next.get()),output(points.get()),
                            oracle_next,oracle_status,oracle_error,cv::Size(21,21),0,
                            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,30,.01),
                            cv::OPTFLOW_USE_INITIAL_FLOW | (quality ? cv::OPTFLOW_LK_GET_MIN_EIGENVALS : 0));
    check(cv::norm(oracle_next,output(result.get()),cv::NORM_INF)<1e-5 &&
          cv::norm(oracle_status,output(status.get()),cv::NORM_INF)==0 &&
          cv::norm(oracle_error,output(error.get()),cv::NORM_INF)<1e-5,
          "seeded shim differs from direct native oracle");
    for (int i=0; i<4; ++i)
        check(output(status.get()).at<unsigned char>(i,0)==1 &&
              cv::norm(output(result.get()).at<cv::Point2f>(i,0)-fixtures[i]-cv::Point2f(12,7))<.05,
              "seeded shim ignored useful estimate");
    const auto before_result = output(result.get()).clone();
    const auto before_status = output(status.get()).clone();
    const auto before_error = output(error.get()).clone();
    auto unchanged = [&]() {
        check(cv::norm(before_result,output(result.get()),cv::NORM_INF)==0 &&
              cv::norm(before_status,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(before_error,output(error.get()),cv::NORM_INF)==0,
              "seeded failure not atomic");
    };
    auto invalid = [&](opencv_video_status code) {
        check(code==OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "invalid seeded boundary accepted");
        check(std::strlen(opencv_video_last_error())!=0,"missing seeded diagnostic");
        unchanged();
    };
    for (int slot=0; slot<7; ++slot)
        invalid(call(slot==0?nullptr:previous.get(),slot==1?nullptr:next.get(),
                     slot==2?nullptr:points.get(),slot==3?nullptr:seeds.get(),
                     slot==4?nullptr:result.get(),slot==5?nullptr:status.get(),slot==6?nullptr:error.get()));
    for (auto *input : {previous.get(),next.get(),points.get(),seeds.get()}) {
        invalid(call(previous.get(),next.get(),points.get(),seeds.get(),input,status.get(),error.get()));
        invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),input,error.get()));
        invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),input));
    }
    invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),result.get(),error.get()));
    invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),status.get()));
    invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),result.get()));
    auto wrong_count = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto wrong_depth = matrix(4,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto wrong_channels = matrix(4,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    auto wrong_shape = matrix(2,2,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto strided = matrix(4,2,OPENCV_CORE_DEPTH_FLOAT32,2);
    output(strided.get()) = output(strided.get()).col(0);
    check(!output(strided.get()).isContinuous(), "seed stride fixture continuous");
    for (auto *seed : {wrong_count.get(),wrong_depth.get(),wrong_channels.get(),wrong_shape.get(),strided.get()})
        invalid(call(previous.get(),next.get(),points.get(),seed,result.get(),status.get(),error.get()));
    auto bad_image_depth = matrix(96,96,OPENCV_CORE_DEPTH_FLOAT32,1);
    auto bad_image_channels = matrix(96,96,OPENCV_CORE_DEPTH_UINT8,3);
    auto bad_image_geometry = matrix(95,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto empty_image = matrix(0,0,OPENCV_CORE_DEPTH_UINT8,1);
    for (auto *image : {bad_image_depth.get(),bad_image_channels.get(),bad_image_geometry.get(),empty_image.get()})
        invalid(call(image,next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get()));
    for (auto *p : {wrong_depth.get(),wrong_channels.get(),wrong_shape.get(),strided.get()})
        invalid(call(previous.get(),next.get(),p,seeds.get(),result.get(),status.get(),error.get()));
    invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),31));
    invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),0,0));
    invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),0,21,
                 std::numeric_limits<double>::quiet_NaN()));
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), 536870976.0f}) {
        for (int component : {0,1}) {
            output(seeds.get()).at<cv::Vec2f>(0,0)[component]=bad;
            invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get()));
            output(seeds.get())=saved_seeds.clone();
            output(points.get()).at<cv::Vec2f>(0,0)[component]=bad;
            invalid(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get()));
            output(points.get()).at<cv::Point2f>(0,0)=fixtures[0];
        }
    }
#ifdef OPENCV_VIDEO_TEST_FAULTS
    for (int stage : {1,2}) for (int kind : {1,2,3}) {
        fault_stage=stage; fault_kind=kind;
        const auto code=call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get());
        check(code==(kind==1?OPENCV_VIDEO_ERROR_STANDARD:
                     kind==2?OPENCV_VIDEO_ERROR_OPENCV:OPENCV_VIDEO_ERROR_UNKNOWN),
              "seeded exception escaped or wrong status");
        unchanged();
        check(cv::norm(saved_seeds,output(seeds.get()),cv::NORM_INF)==0,"fault mutated seeds");
    }
    fault_stage=0;
    std::cout << "PASS: 6 seeded injected exceptions contained atomically\n";
#endif
    // Input/input header alias is safe: only the private clone is mutable.
    check(call(previous.get(),previous.get(),points.get(),points.get(),result.get(),status.get(),error.get())
          == OPENCV_VIDEO_OK, "seed/previous-point input alias rejected");
    for (int i=0; i<4; ++i)
        check(output(status.get()).at<unsigned char>(i,0)==1 &&
              cv::norm(output(result.get()).at<cv::Point2f>(i,0)-fixtures[i])<.05,
              "aliased input identity differs");
    check(cv::norm(saved_seeds,output(seeds.get()),cv::NORM_INF)==0 &&
          cv::norm(saved_previous,output(previous.get()),cv::NORM_INF)==0 &&
          cv::norm(saved_next,output(next.get()),cv::NORM_INF)==0,"seeded input mutated");
    // Distinct headers sharing the seed allocation are also safe, on success
    // and after native mutation followed by an exception before publication.
    output(result.get())=output(seeds.get());
#ifdef OPENCV_VIDEO_TEST_FAULTS
    fault_stage=2; fault_kind=2;
    check(call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())
          == OPENCV_VIDEO_ERROR_OPENCV,"shared-storage fault not contained");
    check(output(result.get()).data==output(seeds.get()).data &&
          cv::norm(saved_seeds,output(seeds.get()),cv::NORM_INF)==0 &&
          cv::norm(before_status,output(status.get()),cv::NORM_INF)==0,
          "shared-storage failure mutated seed or output");
    fault_stage=0;
#endif
    valid();
    check(cv::norm(saved_seeds,output(seeds.get()),cv::NORM_INF)==0,"shared allocation mutated seed");
    output(seeds.get()).at<cv::Point2f>(1,0)={-100,-100};
    output(seeds.get()).at<cv::Point2f>(2,0)={536870912.0f,-536870912.0f};
    output(points.get()).at<cv::Point2f>(3,0)={-100,-100};
    const auto before_outside_result=output(result.get()).clone();
    const auto before_outside_status=output(status.get()).clone();
    const auto before_outside_error=output(error.get()).clone();
    const bool missing_quality=quality && !unavailable_previous_quality_is_defined();
    const auto outside_code=call(previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),error.get());
    check(outside_code==(missing_quality ? OPENCV_VIDEO_ERROR_OPENCV : OPENCV_VIDEO_OK),
          "seeded undefined-quality contract");
    if (missing_quality)
        check(cv::norm(before_outside_result,output(result.get()),cv::NORM_INF)==0 &&
              cv::norm(before_outside_status,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(before_outside_error,output(error.get()),cv::NORM_INF)==0,"seeded undefined quality not atomic");
    for (int i : {1,2,3}) if (!missing_quality)
        check(output(status.get()).at<unsigned char>(i,0)==0 &&
              output(result.get()).at<cv::Point2f>(i,0)==output(points.get()).at<cv::Point2f>(i,0) &&
              (quality ? output(error.get()).at<float>(i,0)>=0 : output(error.get()).at<float>(i,0)==0),
              "failed seeded result not normalized");
    // A low-eigenvalue failure can skip both seeded-patch processing and err
    // assignment; the shim must not read native failed-slot storage.
    output(previous.get()).setTo(cv::Scalar(0));
    output(next.get()).setTo(cv::Scalar(0));
    output(points.get()).at<cv::Point2f>(3,0)=fixtures[3];
    valid();
    for (int i=0; i<4; ++i)
        check(output(status.get()).at<unsigned char>(i,0)==0 &&
              output(result.get()).at<cv::Point2f>(i,0)==output(points.get()).at<cv::Point2f>(i,0) &&
              output(error.get()).at<float>(i,0)==0,"singular seeded failure not normalized");
    auto empty = matrix(0,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    const auto last_result = output(result.get()).clone();
    const auto last_status = output(status.get()).clone();
    const auto last_error = output(error.get()).clone();
    for (bool empty_previous : {false,true}) {
        check(call(previous.get(),next.get(),empty_previous?empty.get():points.get(),
                   empty_previous?seeds.get():empty.get(),result.get(),status.get(),error.get())
              == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "empty/nonempty seed pair accepted");
        check(cv::norm(last_result,output(result.get()),cv::NORM_INF)==0 &&
              cv::norm(last_status,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(last_error,output(error.get()),cv::NORM_INF)==0,"empty mismatch not atomic");
    }
    check(call(previous.get(),next.get(),empty.get(),empty.get(),result.get(),status.get(),error.get())
          == OPENCV_VIDEO_OK && output(result.get()).empty() && output(status.get()).empty() &&
          output(error.get()).empty(), "seeded empty pair failed");
    std::cout << (quality ? "quality " : "photometric ") << "PASS: seeded actual-shim/Core boundary, oracle, validation, aliases and atomicity\n";
}
void run_quality(bool pyramids = false) {
    for (int scenario=0; scenario<10; ++scenario) {
        const int mode=scenario==9 ? 3 : scenario;
        auto f=quality_fixture(mode);
        if (scenario==9) f.seeded=true;
        auto previous=matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
        auto next=matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
        auto points=matrix(int(f.points.size()),1,OPENCV_CORE_DEPTH_FLOAT32,2);
        auto seeds=matrix(int(f.seeds.size()),1,OPENCV_CORE_DEPTH_FLOAT32,2);
        auto result=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,2);
        auto status=matrix(1,1,OPENCV_CORE_DEPTH_UINT8,1);
        auto values=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
        output(previous.get())=f.previous;
        output(next.get())=f.next;
        for (int i=0;i<int(f.points.size());++i) {
            output(points.get()).at<cv::Point2f>(i)=f.points[i];
            output(seeds.get()).at<cv::Point2f>(i)=f.seeds[i];
        }
        using Pyramid = std::unique_ptr<opencv_video_pyramid_handle,
            decltype(&opencv_video_pyramid_destroy)>;
        auto build = [](opencv_core_mat_handle *image) {
            opencv_video_pyramid_handle *p=nullptr;
            check(opencv_video_pyramid_create(image,21,21,3,&p)==0,"quality pyramid build");
            return Pyramid(p,opencv_video_pyramid_destroy);
        };
        auto pa=build(previous.get()),pb=build(next.get());
        std::vector<cv::Mat> da,db;
        cv::buildOpticalFlowPyramid(f.previous,da,{21,21},3,true,
            cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
        cv::buildOpticalFlowPyramid(f.next,db,{21,21},3,true,
            cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
        const auto old_result=output(result.get());
        const auto old_status=output(status.get());
        const auto old_values=output(values.get());
        output(result.get()).setTo(cv::Scalar(17,19));
        output(status.get()).setTo(23); output(values.get()).setTo(29);
        const auto code=pyramids ? (f.seeded ?
            opencv_video_track_pyr_lk_pyramids_seeded_min_eigenvalues(
                pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),values.get(),
                21,21,0,30,.01,f.threshold) : opencv_video_track_pyr_lk_pyramids_min_eigenvalues(
                pa.get(),pb.get(),points.get(),result.get(),status.get(),values.get(),
                21,21,0,30,.01,f.threshold)) : (f.seeded ? opencv_video_track_pyr_lk_seeded_min_eigenvalues(
            previous.get(),next.get(),points.get(),seeds.get(),result.get(),status.get(),values.get(),
            21,21,0,30,.01,f.threshold) : opencv_video_track_pyr_lk_min_eigenvalues(
            previous.get(),next.get(),points.get(),result.get(),status.get(),values.get(),
            21,21,0,30,.01,f.threshold));
        cv::Mat dest=output(seeds.get()).clone(), native_status;
        cv::Mat eigenvalues(int(f.points.size()),1,CV_32F,
                            cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
        const auto *storage=eigenvalues.data;
        const int flags=cv::OPTFLOW_LK_GET_MIN_EIGENVALS |
            (f.seeded ? cv::OPTFLOW_USE_INITIAL_FLOW : 0);
        if (pyramids) cv::calcOpticalFlowPyrLK(da,db,output(points.get()),dest,native_status,eigenvalues,
            {21,21},0,{3,30,.01},flags,f.threshold);
        else cv::calcOpticalFlowPyrLK(f.previous,f.next,output(points.get()),dest,native_status,eigenvalues,
            {21,21},0,{3,30,.01},flags,f.threshold);
        check(storage==eigenvalues.data,"native sentinel storage not reused");
        if (mode==3 && std::isnan(eigenvalues.at<float>(1))) {
            check(code==OPENCV_VIDEO_ERROR_OPENCV,"unwritten HAL quality was exposed");
            check(output(result.get()).data==old_result.data && output(status.get()).data==old_status.data &&
                  output(values.get()).data==old_values.data &&
                  output(result.get()).at<cv::Point2f>(0)==cv::Point2f(17,19) &&
                  output(status.get()).at<unsigned char>(0)==23 && output(values.get()).at<float>(0)==29,
                  "unwritten HAL quality published partial outputs");
            continue;
        }
        check(code==OPENCV_VIDEO_OK,"quality boundary call failed");
        check(cv::norm(native_status,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(eigenvalues,output(values.get()),cv::NORM_INF)<1e-5,
              "quality shim differs from direct flag-8/12 oracle");
        for (int i=0;i<int(f.points.size());++i) {
            const float e=output(values.get()).at<float>(i);
            const bool tracked=output(status.get()).at<unsigned char>(i)!=0;
            check(std::isfinite(e) && e>=0,"invalid quality exposed");
            const auto expected=tracked ? dest.at<cv::Point2f>(i) : f.points[i];
            check(cv::norm(expected-output(result.get()).at<cv::Point2f>(i))<1e-5,
                  "quality next-point normalization/agreement failed");
            if (mode==0) check(tracked && e>.1f,"identity eigenvalue flag ignored");
            if (mode==2) check(!tracked && e>.1f,"threshold quality erased");
            if (mode==3 && i==1) check(!tracked && e==0,"previous unavailable quality");
            if (mode==4 && i==1) check(!tracked && e>.1f,"next unavailable quality erased");
        }
        for (int i=0;i<int(f.seeds.size());++i)
            check(output(seeds.get()).at<cv::Point2f>(i)==f.seeds[i],"quality seeds mutated");
    }
    std::cout << "PASS: 31 minimum-eigenvalue boundary/native-oracle entries (including seeded unavailable previous), failed quality retained\n";
}
void run_pyramids() {
    using Pyramid = std::unique_ptr<opencv_video_pyramid_handle,
        decltype(&opencv_video_pyramid_destroy)>;
    opencv_video_pyramid_destroy(nullptr);
    auto a = matrix(64,64,OPENCV_CORE_DEPTH_UINT8,1);
    auto b = matrix(64,64,OPENCV_CORE_DEPTH_UINT8,1);
    fill_texture(output(a.get()));
    shift(output(a.get()),output(b.get()),2,1);
    auto build = [&](opencv_core_mat_handle *image, int level) {
        opencv_video_pyramid_handle *p = nullptr;
        check(opencv_video_pyramid_create(image,21,21,level,&p)==0 && p,
              "pyramid construction failed");
        return Pyramid(p,opencv_video_pyramid_destroy);
    };
    auto pa = build(a.get(),3), pb = build(b.get(),3);
    int32_t w=0,h=0,r=0,l=0;
    check(opencv_video_pyramid_metadata(pa.get(),&w,&h,&r,&l)==0 &&
          w==21 && h==21 && r==3 && l==1,"pyramid metadata failed");
    check(opencv_video_pyramid_metadata(nullptr,&w,&h,&r,&l)!=0,"null metadata accepted");
    auto points=matrix(5,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    for (int i=0;i<4;++i) output(points.get()).at<cv::Point2f>(i)=fixture_points(64)[size_t(i)];
    output(points.get()).at<cv::Point2f>(4)={-1000,-1000};
    auto result=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto status=matrix(1,1,OPENCV_CORE_DEPTH_UINT8,1);
    auto error=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    std::vector<cv::Mat> da,db;
    const int la=cv::buildOpticalFlowPyramid(output(a.get()),da,{21,21},3,true,
        cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
    const int lb=cv::buildOpticalFlowPyramid(output(b.get()),db,{21,21},3,true,
        cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
    cv::Mat expected,flags,errors;
    cv::calcOpticalFlowPyrLK(da,db,output(points.get()),expected,flags,errors,
        {21,21},std::min(la,lb),{3,30,.01},0,1e-4);
    output(a.get()).setTo(0); output(b.get()).setTo(255);
    a.reset(); b.reset();
    auto call=[&](const opencv_video_pyramid_handle *p, const opencv_video_pyramid_handle *q,
                  int window=21,int level=3) {
        return opencv_video_track_pyr_lk_pyramids(p,q,points.get(),result.get(),
            status.get(),error.get(),window,window,level,30,.01,1e-4);
    };
    for (int repeat=0;repeat<3;++repeat) {
        check(call(pa.get(),pb.get())==0,"pyramid tracking failed");
        check(cv::norm(flags,output(status.get()),cv::NORM_INF)==0,"pyramid status oracle");
        for (int i=0;i<5;++i) {
            const bool ok=flags.at<unsigned char>(i)!=0;
            check(cv::norm(output(result.get()).at<cv::Point2f>(i)-
                (ok?expected.at<cv::Point2f>(i):output(points.get()).at<cv::Point2f>(i)))<1e-5,
                "pyramid point oracle/normalization");
            check(std::abs(output(error.get()).at<float>(i)-(ok?errors.at<float>(i):0))<1e-5,
                "pyramid photometric oracle/normalization");
        }
    }
    const auto before=output(result.get()).clone();
    const auto before_status=output(status.get()).clone();
    const auto before_error=output(error.get()).clone();
    for (int window : {15,31}) check(call(pa.get(),pb.get(),window)!=0,"window mismatch");
    check(call(nullptr,pb.get())!=0 && call(pa.get(),nullptr)!=0,"empty pyramid accepted");
    check(cv::norm(before,output(result.get()),cv::NORM_INF)==0,"pyramid failure atomicity");
    auto source=matrix(64,64,OPENCV_CORE_DEPTH_UINT8,1);
    auto shallow=build(source.get(),1);
    check(call(shallow.get(),pb.get())!=0 && call(pa.get(),shallow.get())!=0,
          "requested depth mismatch accepted");
    auto other=matrix(63,64,OPENCV_CORE_DEPTH_UINT8,1);
    auto mismatch=build(other.get(),3);
    check(call(pa.get(),mismatch.get())!=0,"pyramid base geometry mismatch accepted");
    for (int slot=0;slot<4;++slot)
        check(opencv_video_track_pyr_lk_pyramids(pa.get(),pb.get(),
            slot==0?nullptr:points.get(),slot==1?nullptr:result.get(),
            slot==2?nullptr:status.get(),slot==3?nullptr:error.get(),21,21,3,30,.01,1e-4)!=0,
            "null pyramid tracking Core handle accepted");
    check(opencv_video_track_pyr_lk_pyramids(pa.get(),pb.get(),points.get(),points.get(),
        status.get(),error.get(),21,21,3,30,.01,1e-4)!=0,"pyramid output input alias accepted");
    for (int bad : {-1,0,2,256}) {
        opencv_video_pyramid_handle *raw=pa.get();
        check(opencv_video_pyramid_create(source.get(),bad,21,3,&raw)!=0 && raw==nullptr,
              "invalid build window published handle");
    }
    auto wrong=matrix(64,64,OPENCV_CORE_DEPTH_FLOAT32,1);
    opencv_video_pyramid_handle *raw=pa.get();
    check(opencv_video_pyramid_create(wrong.get(),21,21,3,&raw)!=0 && raw==nullptr,
          "invalid image published handle");
    check(opencv_video_pyramid_create(source.get(),21,21,3,nullptr)!=0,"null create output");
#ifdef OPENCV_VIDEO_TEST_FAULTS
    for (int stage : {3,4}) for (int kind : {1,2,3,4}) {
        fault_stage=stage; fault_kind=kind;
        opencv_video_pyramid_handle *raw=pa.get();
        check(opencv_video_pyramid_create(source.get(),21,21,3,&raw)!=0 && raw==nullptr,
              "pyramid create fault publication");
    }
    for (int stage : {1,2}) for (int kind : {1,2,3,4}) {
        fault_stage=stage; fault_kind=kind;
        check(call(pa.get(),pb.get())!=0 &&
              cv::norm(before,output(result.get()),cv::NORM_INF)==0 &&
              cv::norm(before_status,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(before_error,output(error.get()),cv::NORM_INF)==0,"pyramid track fault atomicity");
    }
    fault_stage=0;
#endif
    for (int repeat=0;repeat<100;++repeat) {
        auto source=matrix(64,64,OPENCV_CORE_DEPTH_UINT8,1);
        auto p=build(source.get(),30);
    }
    std::cout << "PASS: owned pyramids, metadata, native oracle, source lifetime, reuse, faults/destruction\n";
}
void run_seeded_pyramids(bool quality = false, bool seeded = true) {
    using Pyramid = std::unique_ptr<opencv_video_pyramid_handle,
        decltype(&opencv_video_pyramid_destroy)>;
    auto a=matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto b=matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
    output(a.get())=texture(96); output(b.get())=seeded ? translated(output(a.get()),12,7) : output(a.get()).clone();
    std::vector<cv::Mat> da,db;
    cv::buildOpticalFlowPyramid(output(a.get()),da,{21,21},3,true,
        cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
    cv::buildOpticalFlowPyramid(output(b.get()),db,{21,21},3,true,
        cv::BORDER_REFLECT_101,cv::BORDER_CONSTANT,false);
    auto build=[&](opencv_core_mat_handle *image, int level=3) {
        opencv_video_pyramid_handle *p=nullptr;
        check(opencv_video_pyramid_create(image,21,21,level,&p)==0 && p,"seeded pyramid build");
        return Pyramid(p,opencv_video_pyramid_destroy);
    };
    auto pa=build(a.get()),pb=build(b.get());
    output(a.get()).setTo(0); output(b.get()).setTo(0); a.reset(); b.reset();
    auto points=matrix(4,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto seeds=matrix(4,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    for (int i=0;i<4;++i) {
        output(points.get()).at<cv::Point2f>(i)=fixture_points(96)[size_t(i)];
        output(seeds.get()).at<cv::Point2f>(i)=fixture_points(96)[size_t(i)]+cv::Point2f(12.25f,6.75f);
    }
    auto result=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto status=matrix(1,1,OPENCV_CORE_DEPTH_UINT8,1);
    auto error=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    auto call=[&](const opencv_video_pyramid_handle *p, const opencv_video_pyramid_handle *q,
                 const opencv_core_mat_handle *pts, const opencv_core_mat_handle *sds,
                 opencv_core_mat_handle *r, opencv_core_mat_handle *s, opencv_core_mat_handle *e,
                 int window=21,int level=0,double epsilon=.01) {
        if (!seeded) {
            return opencv_video_track_pyr_lk_pyramids_min_eigenvalues(p,q,pts,r,s,e,
                window,21,level,30,epsilon,1e-4);
        }
        const auto track=quality ? opencv_video_track_pyr_lk_pyramids_seeded_min_eigenvalues :
            opencv_video_track_pyr_lk_pyramids_seeded;
        return track(p,q,pts,sds,r,s,e,window,21,level,30,epsilon,1e-4);
    };
    const auto saved_seeds=output(seeds.get()).clone();
    cv::Mat direct=saved_seeds.clone(),ds,de;
    if (quality) de=cv::Mat(4,1,CV_32F,cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    cv::calcOpticalFlowPyrLK(da,db,output(points.get()),direct,ds,de,{21,21},0,{3,30,.01},
        (seeded ? 4 : 0) | (quality ? 8 : 0));
    for (int repetition=0;repetition<3;++repetition) {
        check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())==0,
              "valid seeded pyramid failed");
        check(cv::norm(direct,output(result.get()),cv::NORM_INF)<1e-5 &&
              cv::norm(ds,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(de,output(error.get()),cv::NORM_INF)<1e-5 &&
              cv::norm(saved_seeds,output(seeds.get()),cv::NORM_INF)==0,"seeded pyramid native oracle/reuse");
        for (int i=0;i<4;++i)
            check(output(status.get()).at<unsigned char>(i)==1 &&
                  cv::norm(output(result.get()).at<cv::Point2f>(i)-fixture_points(96)[size_t(i)]-
                           (seeded ? cv::Point2f(12,7) : cv::Point2f(0,0)))<.05,"seeded pyramid predictions ignored");
    }
    auto snapshot=[&]() { return std::vector<cv::Mat>{output(result.get()),output(status.get()),output(error.get())}; };
    const auto headers=snapshot();
    const auto rn=output(result.get()).clone(),rs=output(status.get()).clone(),re=output(error.get()).clone();
    auto unchanged=[&]() {
        const auto now=snapshot();
        for (size_t i=0;i<now.size();++i)
            check(now[i].data==headers[i].data && now[i].size()==headers[i].size() &&
                  now[i].type()==headers[i].type(),"failed seeded pyramid changed output header");
        check(cv::norm(rn,output(result.get()),cv::NORM_INF)==0 &&
              cv::norm(rs,output(status.get()),cv::NORM_INF)==0 &&
              cv::norm(re,output(error.get()),cv::NORM_INF)==0,"failed seeded pyramid changed storage");
    };
    for (int slot=0;slot<7;++slot) {
        if (!seeded && slot==3) continue;
        check(call(slot==0?nullptr:pa.get(),slot==1?nullptr:pb.get(),slot==2?nullptr:points.get(),
            slot==3?nullptr:seeds.get(),slot==4?nullptr:result.get(),slot==5?nullptr:status.get(),
            slot==6?nullptr:error.get())!=0,"null seeded pyramid argument accepted"); unchanged();
    }
    for (int mode=0;mode<9;++mode) {
        auto bad=matrix(4,1,OPENCV_CORE_DEPTH_FLOAT32,2);
        output(bad.get())=saved_seeds.clone();
        switch(mode) {
        case 0: output(bad.get())=cv::Mat(4,1,CV_64FC2); break;
        case 1: output(bad.get())=cv::Mat(4,1,CV_32FC1); break;
        case 2: output(bad.get())=cv::Mat(3,1,CV_32FC2); break;
        case 3: output(bad.get())=cv::Mat(2,2,CV_32FC2); break;
        case 4: output(bad.get())=cv::Mat(4,2,CV_32FC2).col(0); break;
        case 5: output(bad.get()).at<cv::Point2f>(0).x=std::numeric_limits<float>::quiet_NaN(); break;
        case 6: output(bad.get()).at<cv::Point2f>(0).y=std::numeric_limits<float>::infinity(); break;
        case 7: output(bad.get()).at<cv::Point2f>(0).x=536871040.f; break;
        default: output(bad.get()).at<cv::Point2f>(0).y=-536871040.f;
        }
        if (seeded) check(call(pa.get(),pb.get(),points.get(),bad.get(),result.get(),status.get(),error.get())!=0,
              "invalid seeded pyramid seed schema accepted");
        if (mode != 2 || seeded) check(call(pa.get(),pb.get(),bad.get(),seeds.get(),result.get(),status.get(),error.get())!=0,
              "invalid pyramid point schema accepted"); unchanged();
    }
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),15)!=0,
          "seeded pyramid window mismatch"); unchanged();
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),21,4)!=0,
          "seeded pyramid level mismatch"); unchanged();
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),21,0,0)!=0,
          "seeded pyramid invalid options"); unchanged();
    auto wrong_image=matrix(95,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto wrong_geometry=build(wrong_image.get());
    auto shallow=build(wrong_image.get(),0);
    check(call(pa.get(),wrong_geometry.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())!=0,
          "seeded pyramid geometry mismatch"); unchanged();
    check(call(shallow.get(),shallow.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),21,1)!=0,
          "seeded pyramid requested build depth mismatch"); unchanged();
    if (seeded) check(call(pa.get(),pb.get(),points.get(),seeds.get(),seeds.get(),status.get(),error.get())!=0,
          "seeded pyramid seed/output alias"); unchanged();
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),points.get(),status.get(),error.get())!=0,
          "pyramid point/output alias"); unchanged();
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),result.get(),error.get())!=0,
          "seeded pyramid output/output alias"); unchanged();
    for (int out=0;out<3;++out) {
        check(call(pa.get(),pb.get(),points.get(),seeds.get(),
            out==0?points.get():result.get(),out==1?points.get():status.get(),
            out==2?points.get():error.get())!=0,"point/output header alias"); unchanged();
        if (seeded) {
            check(call(pa.get(),pb.get(),points.get(),seeds.get(),
                out==0?seeds.get():result.get(),out==1?seeds.get():status.get(),
                out==2?seeds.get():error.get())!=0,"seed/output header alias"); unchanged();
        }
    }
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),status.get())!=0,
          "status/scalar header alias"); unchanged();
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),result.get())!=0,
          "point/scalar header alias"); unchanged();
#ifdef OPENCV_VIDEO_TEST_FAULTS
    for (int stage : {1,2}) for (int kind : {1,2,3,4}) {
        fault_stage=stage; fault_kind=kind;
        check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())!=0,
              "seeded pyramid exception escaped/accepted"); unchanged();
        // Distinct headers sharing seed storage must also remain unchanged.
        output(result.get())=output(seeds.get());
        const auto seed_data=output(seeds.get()).data;
        check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())!=0 &&
              output(result.get()).data==seed_data &&
              cv::norm(saved_seeds,output(seeds.get()),cv::NORM_INF)==0,"shared seed fault atomicity");
        output(result.get())=headers[0];
    }
    fault_stage=0;
#endif
    check(call(pa.get(),pa.get(),points.get(),points.get(),result.get(),status.get(),error.get())==0,
          "safe input/input alias rejected");
    output(result.get())=output(seeds.get());
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())==0 &&
          cv::norm(saved_seeds,output(seeds.get()),cv::NORM_INF)==0,"shared seed success mutation");
    if (!quality) {
    output(points.get()).at<cv::Point2f>(0)={-1000,-1000};
    output(seeds.get()).at<cv::Point2f>(1)={1000,1000};
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())==0,
          "out of image seeded pyramid call rejected");
    for (int i : {0,1}) check(output(status.get()).at<unsigned char>(i)==0 &&
        output(result.get()).at<cv::Point2f>(i)==output(points.get()).at<cv::Point2f>(i) &&
        output(error.get()).at<float>(i)==0,"failed seeded pyramid not normalized");
    }
    output(points.get())=cv::Mat(0,1,CV_32FC2); output(seeds.get())=cv::Mat(0,1,CV_32FC2);
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get(),15)!=0,
          "empty seeded pyramid bypassed compatibility");
    check(call(pa.get(),pb.get(),points.get(),seeds.get(),result.get(),status.get(),error.get())==0 &&
          output(result.get()).empty() && output(status.get()).empty() && output(error.get()).empty(),
          "empty seeded pyramid outputs");
    std::cout << "PASS: pyramid actual shim flags " << (quality ? (seeded ? 12 : 8) : 4)
              << " oracle, ownership, reuse, schema, aliases, atomicity, faults\n";
}
void run_farneback() {
    auto previous = matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto next = matrix(96,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto flow = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    fill_texture(output(previous.get()));
    shift(output(previous.get()), output(next.get()), 2, 1);
    auto call = [&](const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                    opencv_core_mat_handle *out, double scale = 0.5, int levels = 3,
                    int window = 15, int iterations = 3, int poly = 5, double sigma = 1.2) {
        return opencv_video_calc_farneback_flow(a, b, out, scale, levels, window, iterations, poly, sigma);
    };
    check(call(previous.get(), next.get(), flow.get()) == OPENCV_VIDEO_OK, "Farneback raw boundary failed");
    const cv::Mat &published = output(flow.get());
    check(published.type() == CV_32FC2 && published.rows == 96 && published.cols == 96 &&
          published.isContinuous(), "Farneback output schema");
    cv::Mat oracle;
    cv::calcOpticalFlowFarneback(output(previous.get()), output(next.get()), oracle,
                                 0.5, 3, 15, 3, 5, 1.2, 0);
    check(cv::norm(oracle, published, cv::NORM_INF) < 1e-6, "shim differs from independent Farneback call");

    const cv::Mat saved = published.clone();
    const auto unchanged = [&](const char *message) {
        check(output(flow.get()).size() == saved.size() && output(flow.get()).type() == saved.type() &&
              cv::norm(saved, output(flow.get()), cv::NORM_INF) == 0, message);
    };
    check(call(nullptr, next.get(), flow.get()) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null previous");
    check(call(previous.get(), nullptr, flow.get()) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null next");
    check(call(previous.get(), next.get(), nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null output");
    const auto invalid = [&](opencv_video_status code, const char *message) {
        check(code == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, message);
        unchanged(message);
    };
    invalid(call(nullptr, next.get(), flow.get()), "null previous accepted");
    invalid(call(previous.get(), nullptr, flow.get()), "null next accepted");
    check(call(previous.get(), next.get(), nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null output accepted");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    invalid(call(previous.get(), next.get(), flow.get(), 0.2), "scale low accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.95), "scale high accepted");
    invalid(call(previous.get(), next.get(), flow.get(), nan), "scale NaN accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 0), "levels zero accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 9), "levels high accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 4), "window low accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 16), "even window accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 65), "window high accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 15, 0), "iterations zero accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 15, 31), "iterations high accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 15, 3, 6), "neighborhood accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 15, 3, 5, 0.0), "sigma low accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 15, 3, 5, inf), "sigma infinite accepted");
    invalid(call(previous.get(), next.get(), flow.get(), 0.5, 3, 15, 3, 5, nan), "sigma NaN accepted");

    // Wrong image schemas and geometry.
    auto floating = matrix(96,96,OPENCV_CORE_DEPTH_FLOAT32,1);
    auto color = matrix(96,96,OPENCV_CORE_DEPTH_UINT8,3);
    auto wrong_size = matrix(95,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto tiny = matrix(15,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto tiny_other = matrix(15,96,OPENCV_CORE_DEPTH_UINT8,1);
    auto empty = matrix(1,1,OPENCV_CORE_DEPTH_UINT8,1);
    output(empty.get()).release();
    invalid(call(floating.get(), floating.get(), flow.get()), "Float32 image accepted");
    invalid(call(color.get(), color.get(), flow.get()), "three-channel image accepted");
    invalid(call(previous.get(), wrong_size.get(), flow.get()), "geometry mismatch accepted");
    invalid(call(tiny.get(), tiny_other.get(), flow.get()), "image below 16 rows accepted");
    invalid(call(empty.get(), empty.get(), flow.get()), "empty image accepted");

    // Output aliases input headers: rejected before native work.
    check(call(previous.get(), next.get(), previous.get()) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "output/previous alias accepted");
    check(call(previous.get(), next.get(), next.get()) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "output/next alias accepted");

    // Same image for both inputs is safe and replaces the old (wrong-content) flow.
    check(call(previous.get(), previous.get(), flow.get()) == OPENCV_VIDEO_OK, "identity call failed");
    check(cv::norm(output(flow.get()), cv::NORM_INF) < 0.5, "identity flow not near zero");
    check(call(previous.get(), next.get(), flow.get()) == OPENCV_VIDEO_OK, "restore translation flow");
    const cv::Mat restored = output(flow.get()).clone();
    check(cv::norm(restored, saved, cv::NORM_INF) == 0, "repeatable result differs");

    // Output storage is a fresh private allocation: a previously shared buffer is untouched.
    const cv::Mat shared = output(flow.get());
    const auto *shared_data = shared.data;
    check(call(previous.get(), previous.get(), flow.get()) == OPENCV_VIDEO_OK &&
          output(flow.get()).data != shared_data && cv::norm(saved, shared, cv::NORM_INF) == 0,
          "native result overwrote shared published storage");
    check(call(previous.get(), next.get(), flow.get()) == OPENCV_VIDEO_OK, "restore after sharing");

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        const cv::Mat before = output(flow.get()).clone();
        for (int stage : {1,2}) for (int kind : {1,2,3,4}) {
            fault_stage=stage; fault_kind=kind;
            check(call(previous.get(), next.get(), flow.get()) != OPENCV_VIDEO_OK,
                  "Farneback exception escaped/accepted");
            check(cv::norm(before, output(flow.get()), cv::NORM_INF) == 0 &&
                  output(flow.get()).type() == before.type(), "Farneback fault changed published output");
        }
        fault_stage=0;
        // Corrupt one private component after the native call: validation must reject it
        // without publishing anything (a NaN/Inf must never reach the caller).
        for (int mode : {1,2,3}) {
            corrupt_mode=mode;
            check(call(previous.get(), next.get(), flow.get()) == OPENCV_VIDEO_ERROR_OPENCV,
                  "nonfinite Farneback component accepted");
            check(cv::norm(before, output(flow.get()), cv::NORM_INF) == 0 &&
                  output(flow.get()).type() == before.type(),
                  "nonfinite Farneback result was partially published");
        }
        corrupt_mode=0;
    }
#endif
    check(call(previous.get(), next.get(), flow.get()) == OPENCV_VIDEO_OK, "post-fault call failed");
    std::cout << "PASS: Farneback actual shim flags 0 oracle, schema, validation, aliases, private storage, atomicity, faults\n";
}
void run_farneback_seeded() {
    constexpr int n = 96;
    auto previous = matrix(n,n,OPENCV_CORE_DEPTH_UINT8,1);
    auto next = matrix(n,n,OPENCV_CORE_DEPTH_UINT8,1);
    auto seed = matrix(n,n,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto flow = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
    fill_texture(output(previous.get()));
    shift(output(previous.get()), output(next.get()), 12, 7);
    output(seed.get()).setTo(cv::Scalar(12.25, 6.75));
    const cv::Mat seed_saved = output(seed.get()).clone();
    auto call = [&](const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                    const opencv_core_mat_handle *s, opencv_core_mat_handle *out,
                    double scale = 0.5, int levels = 3, int window = 15, int iterations = 3,
                    int poly = 5, double sigma = 1.2) {
        return opencv_video_calc_farneback_flow_seeded(a, b, s, out, scale, levels, window,
                                                       iterations, poly, sigma);
    };
    const auto seed_unchanged = [&](const char *message) {
        check(output(seed.get()).type() == seed_saved.type() && output(seed.get()).size() == seed_saved.size() &&
              cv::norm(seed_saved, output(seed.get()), cv::NORM_INF) == 0, message);
    };
    check(call(previous.get(), next.get(), seed.get(), flow.get()) == OPENCV_VIDEO_OK,
          "seeded Farneback raw boundary failed");
    const cv::Mat &published = output(flow.get());
    check(published.type() == CV_32FC2 && published.rows == n && published.cols == n &&
          published.isContinuous(), "seeded Farneback output schema");
    check(published.data != output(seed.get()).data, "seeded result shares the seed allocation");
    seed_unchanged("successful seeded call mutated the seed");
    cv::Mat oracle = seed_saved.clone();  // independent flag-4 call on a private clone
    cv::calcOpticalFlowFarneback(output(previous.get()), output(next.get()), oracle,
                                 0.5, 3, 15, 3, 5, 1.2, cv::OPTFLOW_USE_INITIAL_FLOW);
    check(cv::norm(oracle, published, cv::NORM_INF) < 1e-6, "shim differs from independent seeded call");
    cv::Mat plain;
    cv::calcOpticalFlowFarneback(output(previous.get()), output(next.get()), plain, 0.5, 3, 15, 3, 5, 1.2, 0);
    check(cv::norm(plain, published, cv::NORM_INF) > 0.5, "seeded shim result equals the unseeded flow");
    check(cv::norm(published, seed_saved, cv::NORM_INF) > 0.001, "seeded shim result is an unrefined seed copy");
    {
        // Different valid seed gives a different result, matching its own direct oracle.
        auto other = matrix(n,n,OPENCV_CORE_DEPTH_FLOAT32,2);
        output(other.get()).setTo(cv::Scalar(-20.0, 15.0));
        auto out = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
        check(call(previous.get(), next.get(), other.get(), out.get()) == OPENCV_VIDEO_OK, "other seed rejected");
        cv::Mat expected(n, n, CV_32FC2, cv::Scalar(-20.0, 15.0));
        cv::calcOpticalFlowFarneback(output(previous.get()), output(next.get()), expected,
                                     0.5, 3, 15, 3, 5, 1.2, cv::OPTFLOW_USE_INITIAL_FLOW);
        check(cv::norm(expected, output(out.get()), cv::NORM_INF) < 1e-6, "other seed differs from oracle");
        check(cv::norm(output(out.get()), published, cv::NORM_INF) > 0.5, "seed was not consumed");
        // Zero seed is accepted and equals the unseeded result on this fixture.
        output(other.get()).setTo(cv::Scalar(0, 0));
        check(call(previous.get(), next.get(), other.get(), out.get()) == OPENCV_VIDEO_OK, "zero seed rejected");
        check(cv::norm(plain, output(out.get()), cv::NORM_INF) < 1e-3, "zero seed differs from unseeded flow");
        // Component order: swapped seed must not behave like the original.
        output(other.get()).setTo(cv::Scalar(6.75, 12.25));
        check(call(previous.get(), next.get(), other.get(), out.get()) == OPENCV_VIDEO_OK, "swapped seed rejected");
        cv::Mat swapped(n, n, CV_32FC2, cv::Scalar(6.75, 12.25));
        cv::calcOpticalFlowFarneback(output(previous.get()), output(next.get()), swapped,
                                     0.5, 3, 15, 3, 5, 1.2, cv::OPTFLOW_USE_INITIAL_FLOW);
        check(cv::norm(swapped, output(out.get()), cv::NORM_INF) < 1e-6, "swapped seed differs from oracle");
    }

    const cv::Mat saved = published.clone();
    const auto unchanged = [&](const char *message) {
        check(output(flow.get()).size() == saved.size() && output(flow.get()).type() == saved.type() &&
              cv::norm(saved, output(flow.get()), cv::NORM_INF) == 0, message);
        seed_unchanged(message);
    };
    const auto invalid = [&](opencv_video_status code, const char *message) {
        check(code == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, message);
        unchanged(message);
    };
    invalid(call(nullptr, next.get(), seed.get(), flow.get()), "null previous accepted");
    invalid(call(previous.get(), nullptr, seed.get(), flow.get()), "null next accepted");
    invalid(call(previous.get(), next.get(), nullptr, flow.get()), "null seed accepted");
    check(call(previous.get(), next.get(), seed.get(), nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "null output accepted");
    unchanged("null output changed state");

    // Wrong seed schema / geometry.
    auto wrong_type = matrix(n,n,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto wrong_channels = matrix(n,n,OPENCV_CORE_DEPTH_FLOAT32,1);
    auto three_channels = matrix(n,n,OPENCV_CORE_DEPTH_FLOAT32,3);
    auto uint8_seed = matrix(n,n,OPENCV_CORE_DEPTH_UINT8,2);
    auto wrong_rows = matrix(n-1,n,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto wrong_cols = matrix(n,n+1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto empty = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    output(empty.get()).release();
    invalid(call(previous.get(), next.get(), wrong_type.get(), flow.get()), "Float64 seed accepted");
    invalid(call(previous.get(), next.get(), wrong_channels.get(), flow.get()), "one-channel seed accepted");
    invalid(call(previous.get(), next.get(), three_channels.get(), flow.get()), "three-channel seed accepted");
    invalid(call(previous.get(), next.get(), uint8_seed.get(), flow.get()), "UInt8 seed accepted");
    invalid(call(previous.get(), next.get(), wrong_rows.get(), flow.get()), "wrong-rows seed accepted");
    invalid(call(previous.get(), next.get(), wrong_cols.get(), flow.get()), "wrong-columns seed accepted");
    invalid(call(previous.get(), next.get(), empty.get(), flow.get()), "empty seed accepted");

    // Invalid seed values: nonfinite, beyond 2^20 (never clamped), at either channel.
    const float nanf = std::numeric_limits<float>::quiet_NaN();
    const float inff = std::numeric_limits<float>::infinity();
    auto bad = matrix(n,n,OPENCV_CORE_DEPTH_FLOAT32,2);
    for (int channel = 0; channel < 2; ++channel)
        for (float value : {nanf, inff, -inff, 1048577.f, -1048577.f, std::numeric_limits<float>::max()}) {
            output(bad.get()).setTo(cv::Scalar(1, 1));
            output(bad.get()).at<cv::Vec2f>(n - 1, n - 1)[channel] = value;  // last pixel: full scan
            invalid(call(previous.get(), next.get(), bad.get(), flow.get()), "unsafe seed value accepted");
        }
    output(bad.get()).setTo(cv::Scalar(1048576.0, -1048576.0));
    check(call(previous.get(), next.get(), bad.get(), flow.get()) == OPENCV_VIDEO_OK, "2^20 boundary seed rejected");
    check(call(previous.get(), next.get(), seed.get(), flow.get()) == OPENCV_VIDEO_OK, "restore seeded flow");
    check(cv::norm(output(flow.get()), saved, cv::NORM_INF) == 0, "restored seeded flow differs");

    // Invalid options and image geometry.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.2), "scale low accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.95), "scale high accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), nan), "scale NaN accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.5, 0), "levels zero accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.5, 9), "levels high accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.5, 3, 16), "even window accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.5, 3, 15, 31), "iterations high accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.5, 3, 15, 3, 6), "poly 6 accepted");
    invalid(call(previous.get(), next.get(), seed.get(), flow.get(), 0.5, 3, 15, 3, 5, nan), "sigma NaN accepted");
    auto other_size = matrix(n,n+1,OPENCV_CORE_DEPTH_UINT8,1);
    invalid(call(previous.get(), other_size.get(), seed.get(), flow.get()), "image geometry mismatch accepted");
    auto color = matrix(n,n,OPENCV_CORE_DEPTH_UINT8,3);
    invalid(call(color.get(), color.get(), seed.get(), flow.get()), "color image accepted");
    auto tiny = matrix(15,15,OPENCV_CORE_DEPTH_UINT8,1);
    auto tiny_seed = matrix(15,15,OPENCV_CORE_DEPTH_FLOAT32,2);
    invalid(call(tiny.get(), tiny.get(), tiny_seed.get(), flow.get()), "tiny image accepted");
    auto seed_for_other = matrix(n,n+1,OPENCV_CORE_DEPTH_FLOAT32,2);
    invalid(call(other_size.get(), other_size.get(), seed.get(), flow.get()), "seed/image geometry mismatch accepted");
    (void)seed_for_other;

    // Output header aliases any input header: rejected before native work, seed untouched.
    invalid(call(previous.get(), next.get(), seed.get(), previous.get()), "output/previous alias accepted");
    invalid(call(previous.get(), next.get(), seed.get(), next.get()), "output/next alias accepted");
    check(call(previous.get(), next.get(), seed.get(), seed.get()) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "output/seed alias accepted");
    seed_unchanged("alias rejection changed the seed");
    check(output(previous.get()).type() == CV_8UC1 && output(next.get()).type() == CV_8UC1,
          "alias rejection changed an image header");

    // Distinct headers sharing the seed's storage: allowed; seed storage is never the output.
    {
        auto sharing = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
        output(sharing.get()) = output(seed.get());  // shallow header copy, same data
        check(output(sharing.get()).data == output(seed.get()).data, "sharing fixture is not shared");
        const auto *shared_data = output(seed.get()).data;
        check(call(previous.get(), next.get(), seed.get(), sharing.get()) == OPENCV_VIDEO_OK,
              "distinct-header shared-storage call failed");
        check(output(sharing.get()).data != shared_data && output(seed.get()).data == shared_data,
              "result replaced or reused the seed allocation");
        seed_unchanged("shared-storage call mutated the seed");
        check(cv::norm(output(sharing.get()), saved, cv::NORM_INF) < 1e-6, "shared-storage result differs");
    }

    // Seed given as a genuine non-contiguous Region; native clone must not leak stride.
    {
        auto parent = matrix(n + 20, n + 20, OPENCV_CORE_DEPTH_FLOAT32, 2);
        output(parent.get()).setTo(cv::Scalar(-7, -7));
        cv::Mat region = output(parent.get())(cv::Rect(10, 10, n, n));
        output(seed.get()).copyTo(region);
        auto roi = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
        output(roi.get()) = region;
        check(!output(roi.get()).isContinuous(), "seed Region fixture is contiguous");
        const cv::Mat parent_saved = output(parent.get()).clone();
        auto out = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT32,1);
        check(call(previous.get(), next.get(), roi.get(), out.get()) == OPENCV_VIDEO_OK, "Region seed failed");
        check(output(out.get()).isContinuous() && cv::norm(output(out.get()), saved, cv::NORM_INF) < 1e-6,
              "Region seed result differs from compact seed");
        check(cv::norm(parent_saved, output(parent.get()), cv::NORM_INF) == 0, "Region seed parent mutated");
        check(output(out.get()).data < output(parent.get()).data ||
              output(out.get()).data >= output(parent.get()).data + output(parent.get()).total() * 8,
              "result lies inside the seed parent");
    }

    // Existing nonempty destination: success replaces, failure preserves.
    {
        auto destination = matrix(7,5,OPENCV_CORE_DEPTH_UINT8,3);
        output(destination.get()).setTo(cv::Scalar(9,9,9));
        const cv::Mat before = output(destination.get()).clone();
        check(call(previous.get(), next.get(), wrong_type.get(), destination.get()) ==
              OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad seed with prior destination accepted");
        check(output(destination.get()).size() == before.size() && output(destination.get()).type() == before.type() &&
              cv::norm(before, output(destination.get()), cv::NORM_INF) == 0, "failed call changed prior destination");
        check(call(previous.get(), next.get(), seed.get(), destination.get()) == OPENCV_VIDEO_OK,
              "successful replacement failed");
        check(output(destination.get()).type() == CV_32FC2 && output(destination.get()).rows == n &&
              cv::norm(output(destination.get()), saved, cv::NORM_INF) < 1e-6, "destination not replaced");
    }

    // Same image for both inputs with a seed; repeated calls are deterministic.
    check(call(previous.get(), previous.get(), seed.get(), flow.get()) == OPENCV_VIDEO_OK, "identical-image seeded call failed");
    for (int repetition = 0; repetition < 3; ++repetition) {
        check(call(previous.get(), next.get(), seed.get(), flow.get()) == OPENCV_VIDEO_OK, "repeat failed");
        check(cv::norm(output(flow.get()), saved, cv::NORM_INF) == 0, "repeat differs");
    }
    seed_unchanged("repeated calls mutated the seed");

    // Private storage proof: a previously shared result buffer stays untouched.
    {
        const cv::Mat shared = output(flow.get());
        const auto *shared_data = shared.data;
        check(call(previous.get(), previous.get(), seed.get(), flow.get()) == OPENCV_VIDEO_OK &&
              output(flow.get()).data != shared_data && cv::norm(saved, shared, cv::NORM_INF) == 0,
              "native result overwrote shared published storage");
        check(call(previous.get(), next.get(), seed.get(), flow.get()) == OPENCV_VIDEO_OK, "restore after sharing");
    }

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        const cv::Mat before = output(flow.get()).clone();
        for (int stage : {1,2}) for (int kind : {1,2,3,4}) {
            fault_stage=stage; fault_kind=kind;
            check(call(previous.get(), next.get(), seed.get(), flow.get()) != OPENCV_VIDEO_OK,
                  "seeded Farneback exception escaped/accepted");
            check(cv::norm(before, output(flow.get()), cv::NORM_INF) == 0 &&
                  output(flow.get()).type() == before.type(), "seeded Farneback fault changed published output");
            seed_unchanged("seeded Farneback fault mutated the seed");
        }
        fault_stage=0;
        for (int mode : {1,2,3}) {
            corrupt_mode=mode;
            check(call(previous.get(), next.get(), seed.get(), flow.get()) == OPENCV_VIDEO_ERROR_OPENCV,
                  "nonfinite seeded Farneback component accepted");
            check(cv::norm(before, output(flow.get()), cv::NORM_INF) == 0 &&
                  output(flow.get()).type() == before.type(),
                  "nonfinite seeded Farneback result was partially published");
            seed_unchanged("post-native corruption reached the caller's seed");
        }
        corrupt_mode=0;
    }
#endif
    check(call(previous.get(), next.get(), seed.get(), flow.get()) == OPENCV_VIDEO_OK, "post-fault seeded call failed");
    std::cout << "PASS: seeded Farneback actual shim flags 4 oracle, seed validation, aliases, Region seed, private clone, atomicity, faults\n";
}

// ECC translation (Task 011): actual shim versus a direct cv::findTransformECC call.
namespace {
const opencv_video_ecc_translation_result kEccSentinel = {123.5f, -456.25f, 789.125};

Mat ecc_handle(const cv::Mat &source) {
    auto h = matrix(source.rows, source.cols, OPENCV_CORE_DEPTH_UINT8, 1);
    source.copyTo(output(h.get()));
    return h;
}

opencv_video_status ecc_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                             opencv_video_ecc_translation_result *r, int iterations = 50,
                             double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_translation(a, b, iterations, epsilon, r);
}

cv::Point2f ecc_direct(const cv::Mat &a, const cv::Mat &b, double &rho) {
    cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
    rho = cv::findTransformECC(a, b, warp, cv::MOTION_TRANSLATION,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 50, 1e-4),
        cv::noArray(), 5);
    return cv::Point2f(warp.at<float>(0, 2), warp.at<float>(1, 2));
}

void ecc_unchanged(const opencv_video_ecc_translation_result &r, const char *message) {
    check(std::memcmp(&r, &kEccSentinel, sizeof r) == 0, message);
}

void run_ecc_values(const cv::Mat &base_image, const Mat &base) {
    const int n = base_image.rows;
    struct Case { int dx, dy; } cases[] = {{0, 0}, {2, 1}, {-2, -1}, {3, 0}, {0, 3}};
    for (const auto &c : cases) {
        const cv::Mat moving_image = ecc_fixture::texture(n, n, c.dx, c.dy);
        auto moving = ecc_handle(moving_image);
        opencv_video_ecc_translation_result r = kEccSentinel;
        check(ecc_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, "ECC valid call failed");
        double rho = 0;
        const cv::Point2f expected = ecc_direct(base_image, moving_image, rho);
        check(r.displacement_x == expected.x && r.displacement_y == expected.y &&
              r.correlation == rho, "ECC shim differs from direct native call");
        check(std::fabs(r.displacement_x - c.dx) < 0.05 && std::fabs(r.displacement_y - c.dy) < 0.05,
              "ECC displacement sign/order/magnitude");
        check(std::isfinite(r.correlation) && r.correlation > 0.99, "ECC correlation");
        opencv_video_ecc_translation_result reversed = kEccSentinel;
        check(ecc_call(moving.get(), base.get(), &reversed) == OPENCV_VIDEO_OK, "ECC reversed failed");
        check(std::fabs(reversed.displacement_x + c.dx) < 0.05 &&
              std::fabs(reversed.displacement_y + c.dy) < 0.05, "ECC reversed roles not negated");
    }
    opencv_video_ecc_translation_result r = kEccSentinel;
    check(ecc_call(base.get(), base.get(), &r) == OPENCV_VIDEO_OK, "ECC same-handle identity call");
    check(std::fabs(r.displacement_x) < 1e-4 && std::fabs(r.displacement_y) < 1e-4 &&
          std::fabs(r.correlation - 1.0) < 1e-6, "ECC same-handle identity near zero");
    {
        double rho = 0;
        const cv::Point2f expected = ecc_direct(base_image, base_image, rho);
        check(r.displacement_x == expected.x && r.displacement_y == expected.y && r.correlation == rho,
              "ECC same-handle identity differs from direct native call");
    }
    check(cv::norm(output(base.get()), base_image, cv::NORM_INF) == 0, "ECC changed an input");
}
}  // namespace

}  // namespace
namespace {
void run_ecc_arguments(const Mat &base, int n) {
    auto moving = ecc_handle(ecc_fixture::texture(n, n, 2, 1));
    opencv_video_ecc_translation_result r = kEccSentinel;
    check(ecc_call(nullptr, moving.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null template");
    ecc_unchanged(r, "null template changed result");
    check(ecc_call(base.get(), nullptr, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null moving");
    ecc_unchanged(r, "null moving changed result");
    check(ecc_call(base.get(), moving.get(), nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "null result accepted");
    for (int iterations : {0, -1, 101})
        check(ecc_call(base.get(), moving.get(), &r, iterations) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "bad iterations accepted");
    for (double epsilon : {0.0, -1e-4, 9e-9, 0.1000001, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()})
        check(ecc_call(base.get(), moving.get(), &r, 50, epsilon) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "bad epsilon accepted");
    ecc_unchanged(r, "invalid options changed result");
    check(ecc_call(base.get(), moving.get(), &r, 1, 1e-8) == OPENCV_VIDEO_OK, "options lower bound");
    r = kEccSentinel;
    check(ecc_call(base.get(), moving.get(), &r, 100, 0.1) == OPENCV_VIDEO_OK, "options upper bound");

    r = kEccSentinel;
    auto wrong_depth = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
    auto wrong_channels = matrix(n, n, OPENCV_CORE_DEPTH_UINT8, 3);
    auto other_size = matrix(n, n + 1, OPENCV_CORE_DEPTH_UINT8, 1);
    for (auto *bad : {wrong_depth.get(), wrong_channels.get(), other_size.get()}) {
        check(ecc_call(base.get(), bad, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad moving accepted");
        check(ecc_call(bad, base.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad template accepted");
    }
    auto small = matrix(31, 128, OPENCV_CORE_DEPTH_UINT8, 1);
    auto small_peer = matrix(31, 128, OPENCV_CORE_DEPTH_UINT8, 1);
    check(ecc_call(small.get(), small_peer.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "below minimum dimension accepted");
    auto empty = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
    output(empty.get()) = cv::Mat();
    check(ecc_call(empty.get(), base.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "empty accepted");
    auto wide = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
    auto wide_peer = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
    output(wide.get()) = cv::Mat(32, 16385, CV_8UC1, cv::Scalar(0));
    output(wide_peer.get()) = cv::Mat(32, 16385, CV_8UC1, cv::Scalar(0));
    check(ecc_call(wide.get(), wide_peer.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
          "above maximum dimension accepted");
    ecc_unchanged(r, "invalid input changed result");
}

void run_ecc_nonconvergence(const Mat &base, int n) {
    auto flat = ecc_handle(ecc_fixture::constant(n, n, 128));
    auto noise_a = ecc_handle(ecc_fixture::noise(n, n, 3));
    auto noise_b = ecc_handle(ecc_fixture::noise(n, n, 9));
    auto far = ecc_handle(ecc_fixture::texture(n, n, 40, 30));
    opencv_video_ecc_translation_result r = kEccSentinel;
    check(ecc_call(flat.get(), flat.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, "constant images accepted");
    ecc_unchanged(r, "constant failure changed result");
    check(ecc_call(noise_a.get(), noise_b.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, "noise accepted");
    ecc_unchanged(r, "noise failure changed result");
    check(ecc_call(base.get(), far.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, "far translation accepted");
    ecc_unchanged(r, "far translation failure changed result");
    check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after nonconvergence");
}
}  // namespace


namespace {
void run_ecc() {
    static_assert(sizeof(opencv_video_ecc_translation_result) == 16 &&
                  offsetof(opencv_video_ecc_translation_result, displacement_x) == 0 &&
                  offsetof(opencv_video_ecc_translation_result, displacement_y) == 4 &&
                  offsetof(opencv_video_ecc_translation_result, correlation) == 8,
                  "ECC result layout");
    const int n = 128;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);
    run_ecc_values(base_image, base);
    run_ecc_arguments(base, n);
    run_ecc_nonconvergence(base, n);

    // Real non-contiguous Regions equal the same direct call on the same Regions.
    {
        const cv::Mat parent_a = ecc_fixture::texture(160, 160);
        const cv::Mat parent_b = ecc_fixture::texture(160, 160, 2, 1);
        const cv::Rect roi(16, 16, n, n);
        auto h_a = ecc_handle(parent_a);
        auto h_b = ecc_handle(parent_b);
        auto region_a = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto region_b = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(region_a.get()) = output(h_a.get())(roi);
        output(region_b.get()) = output(h_b.get())(roi);
        check(!output(region_a.get()).isContinuous(), "fixture Region is contiguous");
        opencv_video_ecc_translation_result r = kEccSentinel;
        check(ecc_call(region_a.get(), region_b.get(), &r) == OPENCV_VIDEO_OK, "ECC Region call failed");
        double rho = 0;
        const cv::Point2f expected = ecc_direct(parent_a(roi), parent_b(roi), rho);
        check(r.displacement_x == expected.x && r.displacement_y == expected.y && r.correlation == rho,
              "ECC Region differs from direct native Region call");
        check(std::fabs(r.displacement_x - 2) < 0.05 && std::fabs(r.displacement_y - 1) < 0.05,
              "ECC Region displacement");
        check(cv::norm(output(h_a.get()), parent_a, cv::NORM_INF) == 0 &&
              cv::norm(output(h_b.get()), parent_b, cv::NORM_INF) == 0, "ECC changed Region parents");
    }

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        auto moving = ecc_handle(ecc_fixture::texture(n, n, 2, 1));
        for (int stage : {1, 2}) for (int kind : {1, 2, 3, 4}) {
            fault_stage = stage; fault_kind = kind;
            opencv_video_ecc_translation_result r = kEccSentinel;
            const auto code = ecc_call(base.get(), moving.get(), &r);
            fault_stage = 0;
            check(code == (kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                           : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD),
                  "ECC injected exception mapped to the wrong status");
            ecc_unchanged(r, "ECC injected failure changed result");
        }
        // Post-native corruption must be rejected with the caller's result untouched.
        for (int mode = 1; mode <= 6; ++mode) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_translation_result r = kEccSentinel;
            const auto code = ecc_call(base.get(), moving.get(), &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "ECC corrupted native output accepted");
            ecc_unchanged(r, "ECC corrupted native output was partially published");
        }
    }
#endif
    auto moving = ecc_handle(ecc_fixture::texture(n, n, 2, 1));
    opencv_video_ecc_translation_result r = kEccSentinel;
    check(ecc_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, "post-fault ECC call failed");
    std::cout << "PASS: ECC translation shim vs direct findTransformECC, signs, roles, options, schema, nonconvergence, Regions, atomicity, faults\n";
}
}  // namespace



int main() {
    try {
        run();
        run_seeded();
        run(true);
        run_seeded(true);
        run_quality();
        run_pyramids();
        run_seeded_pyramids();
        run_quality(true);
        run_seeded_pyramids(true);
        run_seeded_pyramids(true,false);
        run_farneback();
        run_farneback_seeded();
        run_ecc();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
