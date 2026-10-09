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
    if (fault_kind == 4) throw std::runtime_error("qualification standard exception");
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
    }
#endif
    check(call(previous.get(), next.get(), flow.get()) == OPENCV_VIDEO_OK, "post-fault call failed");
    std::cout << "PASS: Farneback actual shim flags 0 oracle, schema, validation, aliases, private storage, atomicity, faults\n";
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
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
