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
void run_seeded() {
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
        return opencv_video_track_pyr_lk_seeded(a,b,p,seed,q,s,e,w,21,level,30,eps,1e-4);
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
                            cv::OPTFLOW_USE_INITIAL_FLOW);
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
    valid();
    for (int i : {1,2,3})
        check(output(status.get()).at<unsigned char>(i,0)==0 &&
              output(result.get()).at<cv::Point2f>(i,0)==output(points.get()).at<cv::Point2f>(i,0) &&
              output(error.get()).at<float>(i,0)==0,"failed seeded result not normalized");
    // A low-eigenvalue failure can skip both seeded-patch processing and err
    // assignment; the shim must not read native failed-slot storage.
    output(previous.get()).setTo(cv::Scalar(0));
    output(next.get()).setTo(cv::Scalar(0));
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
    std::cout << "PASS: seeded actual-shim/Core boundary, oracle, validation, aliases and atomicity\n";
}
}  // namespace

int main() {
    try {
        run();
        run_seeded();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
