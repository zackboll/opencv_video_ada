#include "opencv_video_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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
#include <type_traits>
#include <utility>

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
        // Euclidean-only modes. 7 scales the whole linear part (a=d, b=-c still hold),
        // 8 reflects (negative determinant), 9 NaN rotation entry, 10 asymmetric b/c.
        case 7: warp[0] *= 1.01f; warp[1] *= 1.01f; warp[3] *= 1.01f; warp[4] *= 1.01f; break;
        case 8: warp[4] = -warp[4]; warp[3] = -warp[3]; break;
        case 9: warp[0] = nan; break;
        case 10: warp[3] += 1e-3f; break;
        // 13: a = d violated by 3e-6 only. Unit length, orthogonality and determinant all
        // stay within their 1e-5 bounds, so ONLY the strict symmetry check can reject it.
        case 13: warp[4] += 3e-6f; break;
        // 11/12 replace the warp with a VALID rigid transform beyond the quarter turn
        // (+120 and -150 degrees, translation (3,-4)); real native fits never reach it.
        case 11: case 12: {
            const double t = (ecc_corrupt_mode == 11 ? 120.0 : -150.0) * 3.14159265358979323846 / 180.0;
            warp[0] = warp[4] = float(std::cos(t));
            warp[3] = float(std::sin(t));
            warp[1] = -warp[3];
            warp[2] = 3.0f; warp[5] = -4.0f;
            break;
        }
        // Affine-only modes (Task 014). 21..26 write one nonfinite coefficient each
        // (warp[1]=+Inf, warp[3]=-Inf, warp[4]=NaN, warp[2]=-Inf, warp[5]=NaN,
        // warp[0]=+Inf); the affine entry point must reject all of them.
        case 21: warp[1] = std::numeric_limits<float>::infinity(); break;
        case 22: warp[3] = -std::numeric_limits<float>::infinity(); break;
        case 23: warp[4] = nan; break;
        case 24: warp[2] = -std::numeric_limits<float>::infinity(); break;
        case 25: warp[5] = nan; break;
        case 26: warp[0] = std::numeric_limits<float>::infinity(); break;
        // 30..33 replace the warp with FINITE matrices that are not rigid; the affine entry
        // point must publish them unchanged (no Euclidean-style policy): 30 anisotropic
        // scale + shear, 31 reflection, 32 singular (det 0), 33 large finite values.
        case 30: warp[0] = 1.3f; warp[1] = 0.4f; warp[2] = 5.0f;
                 warp[3] = -0.2f; warp[4] = 0.7f; warp[5] = -3.0f; break;
        case 31: warp[0] = -1.0f; warp[1] = 0.0f; warp[2] = 0.0f;
                 warp[3] = 0.0f; warp[4] = 1.0f; warp[5] = 0.0f; break;
        case 32: warp[0] = 1.0f; warp[1] = 2.0f; warp[2] = 0.5f;
                 warp[3] = 0.5f; warp[4] = 1.0f; warp[5] = -0.5f; break;
        case 33: warp[0] = 3.0e38f; warp[1] = -3.0e38f; warp[2] = 1.0e30f;
                 warp[3] = 1.0e-30f; warp[4] = -2.0e38f; warp[5] = 3.0e38f; break;
        default: break;
    }
}
// Task 016 homography-only post-native hooks (test builds only). The homography export calls
// these instead of opencv_video_test_corrupt_ecc, so the generic 2x3 modes above are untouched
// and no generic mode can write warp[6..8]. `warp` is the private 3x3 Float32 matrix.
//  1 H00=NaN  2 H01=+Inf  3 H02=-Inf  4 H10=NaN  5 H11=+Inf  6 H12=-Inf  7 H20=NaN  8 H21=+Inf
//  9 H22=NaN 10 H22=0    11 H22=1.01 12 corr=NaN 13 corr=+Inf 14 corr=-Inf
// 15 H20=-Inf 16 H21=NaN 17 H22=+Inf 18 H22=nextafter(1,2) 19 H22=nextafter(1,0) 20 H22=-1
// 30..34 replace the matrix with FINITE projective matrices whose H22 is exactly 1.
int hom_corrupt_mode = 0;
const float kHomInjected[5][9] = {
    {1.0f, 0.0f, 2.0f, 0.0f, 1.0f, 1.0f, 3.0e-4f, 0.0f, 1.0f},          // 30 H20 only
    {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -4.0e-4f, 1.0f},         // 31 H21 only
    {0.98f, 0.01f, -1.5f, -0.02f, 1.03f, 0.75f, 2.5e-4f, -1.5e-4f, 1.0f}, // 32 both
    {-1.7f, 0.9f, 12.0f, 2.3f, 0.05f, -8.0f, 0.01f, 0.02f, 1.0f},       // 33 unusual linear part
    {3.0e38f, -3.0e38f, 1.0e30f, 1.0e-30f, -2.0e38f, 3.0e38f, 5.0f, -7.0f, 1.0f},  // 34 extreme
};
void opencv_video_test_corrupt_homography(float *warp, double *correlation) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    switch (hom_corrupt_mode) {
        case 1: warp[0] = nan; break;
        case 2: warp[1] = inf; break;
        case 3: warp[2] = -inf; break;
        case 4: warp[3] = nan; break;
        case 5: warp[4] = inf; break;
        case 6: warp[5] = -inf; break;
        case 7: warp[6] = nan; break;
        case 8: warp[7] = inf; break;
        case 9: warp[8] = nan; break;
        case 10: warp[8] = 0.0f; break;
        case 11: warp[8] = 1.01f; break;
        case 12: *correlation = std::numeric_limits<double>::quiet_NaN(); break;
        case 13: *correlation = std::numeric_limits<double>::infinity(); break;
        case 14: *correlation = -std::numeric_limits<double>::infinity(); break;
        case 15: warp[6] = -inf; break;
        case 16: warp[7] = nan; break;
        case 17: warp[8] = inf; break;
        case 18: warp[8] = std::nextafter(1.0f, 2.0f); break;
        case 19: warp[8] = std::nextafter(1.0f, 0.0f); break;
        case 20: warp[8] = -1.0f; break;
        case 30: case 31: case 32: case 33: case 34:
            std::memcpy(warp, kHomInjected[hom_corrupt_mode - 30], 9 * sizeof(float)); break;
        default: break;
    }
}
// Schema mutations through ordinary, well-defined cv::Mat operations only. hom_schema_keep
// retains the original private allocation so headers that alias it stay valid; it is released
// by the test after the call. 1 fresh 3x3 allocation (storage identity), 2 reshape to 1x9 (same
// storage, rows/cols), 3 reshape to 9x1, 4 CV_32SC1 header over the same storage (type),
// 5 non-continuous 3x3 Float32 view of a fresh 3x4 allocation (a non-continuous header over the
// original 36 bytes would need a >= 16-byte row step and read past the allocation, so continuity
// cannot be isolated from storage identity safely), 6 3-D 1x3x3 header over the same storage
// (dims), 7 fresh 2x3 identity (shape and storage), 8 fresh 3x3 CV_64F identity (type, storage).
// hom_schema_applied counts hooks that completed, so a rejection cannot come from the hook throwing.
int hom_schema_mode = 0;
int hom_schema_applied = 0;
cv::Mat hom_schema_keep;
cv::Mat hom_schema_wide;
void opencv_video_test_homography_schema(cv::Mat &warp) {
    if (hom_schema_mode == 0) return;
    hom_schema_keep = warp;
    unsigned char *data = hom_schema_keep.data;
    switch (hom_schema_mode) {
        case 1: warp = hom_schema_keep.clone(); break;
        case 2: warp = hom_schema_keep.reshape(1, 1); break;
        case 3: warp = hom_schema_keep.reshape(1, 9); break;
        case 4: warp = cv::Mat(3, 3, CV_32SC1, data); break;
        case 5: hom_schema_wide = cv::Mat::zeros(3, 4, CV_32F);
                warp = hom_schema_wide.colRange(0, 3); break;
        case 6: { const int sizes[3] = {1, 3, 3}; warp = cv::Mat(3, sizes, CV_32FC1, data); break; }
        case 7: warp = cv::Mat::eye(2, 3, CV_32F); break;
        case 8: warp = cv::Mat::eye(3, 3, CV_64F); break;
        default: return;
    }
    ++hom_schema_applied;
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

    // Two distinct Core headers sharing one allocation: a valid identity request, and
    // the shared pixels are unchanged afterwards.
    {
        auto first = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto second = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(first.get()) = output(base.get());
        output(second.get()) = output(base.get());
        check(&output(first.get()) != &output(second.get()) &&
              output(first.get()).data == output(second.get()).data &&
              output(first.get()).data == output(base.get()).data, "fixture headers do not share storage");
        opencv_video_ecc_translation_result r = kEccSentinel;
        check(ecc_call(first.get(), second.get(), &r) == OPENCV_VIDEO_OK, "shared-storage identity failed");
        check(std::fabs(r.displacement_x) < 1e-4 && std::fabs(r.displacement_y) < 1e-4 &&
              std::fabs(r.correlation - 1.0) < 1e-6, "shared-storage identity value");
        check(cv::norm(output(first.get()), base_image, cv::NORM_INF) == 0, "shared storage modified");
        // Repeatable.
        opencv_video_ecc_translation_result again = kEccSentinel;
        check(ecc_call(first.get(), second.get(), &again) == OPENCV_VIDEO_OK &&
              std::memcmp(&r, &again, sizeof r) == 0, "ECC call is not repeatable");
    }
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


// ECC Euclidean (Task 012): actual shim versus a direct cv::findTransformECC call.
namespace {
const opencv_video_ecc_euclidean_result kEucSentinel = {123.5f, -456.25f, 789.125, -321.5};
const double kPi = 3.14159265358979323846;

static_assert(sizeof(opencv_video_ecc_euclidean_result) == 24 &&
              alignof(opencv_video_ecc_euclidean_result) == 8 &&
              offsetof(opencv_video_ecc_euclidean_result, displacement_x) == 0 &&
              offsetof(opencv_video_ecc_euclidean_result, displacement_y) == 4 &&
              offsetof(opencv_video_ecc_euclidean_result, rotation_radians) == 8 &&
              offsetof(opencv_video_ecc_euclidean_result, correlation) == 16,
              "ECC Euclidean result layout");

cv::Mat euc_warp(double theta, double tx, double ty) {
    cv::Mat w(2, 3, CV_64F);
    w.at<double>(0, 0) = std::cos(theta); w.at<double>(0, 1) = -std::sin(theta);
    w.at<double>(0, 2) = tx;
    w.at<double>(1, 0) = std::sin(theta); w.at<double>(1, 1) = std::cos(theta);
    w.at<double>(1, 2) = ty;
    return w;
}

cv::Mat euc_moving(const cv::Mat &tmpl, double theta, double tx, double ty) {
    cv::Mat out;
    cv::warpAffine(tmpl, out, euc_warp(theta, tx, ty), tmpl.size(), cv::INTER_LINEAR,
                   cv::BORDER_REFLECT_101);
    return out;
}

opencv_video_status euc_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                             opencv_video_ecc_euclidean_result *r, int iterations = 50,
                             double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_euclidean(a, b, iterations, epsilon, r);
}

void euc_unchanged(const opencv_video_ecc_euclidean_result &r, const char *message) {
    check(std::memcmp(&r, &kEucSentinel, sizeof r) == 0, message);
}

struct EucDirect { double rho; float w[6]; double theta; };

EucDirect euc_direct(const cv::Mat &a, const cv::Mat &b, int iterations = 50, double eps = 1e-4) {
    cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
    EucDirect d;
    d.rho = cv::findTransformECC(a, b, warp, cv::MOTION_EUCLIDEAN,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, eps),
        cv::noArray(), 5);
    for (int i = 0; i < 6; ++i) d.w[i] = warp.ptr<float>()[i];
    d.theta = std::atan2(double(d.w[3]), double(d.w[0]));
    return d;
}

void euc_matches_direct(const opencv_video_ecc_euclidean_result &r, const EucDirect &d,
                        const char *message) {
    check(r.displacement_x == d.w[2] && r.displacement_y == d.w[5] &&
          r.rotation_radians == d.theta && r.correlation == d.rho, message);
}

void run_ecc_euclidean() {
    const int n = 128;
    const double d2r = kPi / 180.0;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);

    // Identity and signed rotations with translation, versus the direct native call.
    struct Case { double deg, tx, ty; } cases[] = {
        {0, 0, 0}, {0, 2, 1}, {1, 0, 0}, {-1, 0, 0}, {3, 0, 0}, {-3, 0, 0},
        {3, 2, 1}, {3, -2, -1}, {-3, 2, -1}};
    for (const auto &c : cases) {
        const double t = c.deg * d2r;
        const cv::Mat moving_image = euc_moving(base_image, t, c.tx, c.ty);
        auto moving = ecc_handle(moving_image);
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(euc_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, "Euclidean valid call failed");
        euc_matches_direct(r, euc_direct(base_image, moving_image),
                           "Euclidean shim differs from direct native call");
        check(std::fabs(r.rotation_radians - t) < 0.005, "Euclidean angle sign/magnitude");
        check(std::fabs(r.displacement_x - c.tx) < 0.1 && std::fabs(r.displacement_y - c.ty) < 0.1,
              "Euclidean translation sign/axis");
        check(r.correlation > 0.98 && r.correlation <= 1.0 + 1e-6, "Euclidean correlation");
        // Repeatable.
        opencv_video_ecc_euclidean_result again = kEucSentinel;
        check(euc_call(base.get(), moving.get(), &again) == OPENCV_VIDEO_OK &&
              std::memcmp(&r, &again, sizeof r) == 0, "Euclidean call is not repeatable");
        // Reversed roles negate the angle.
        opencv_video_ecc_euclidean_result rev = kEucSentinel;
        check(euc_call(moving.get(), base.get(), &rev) == OPENCV_VIDEO_OK, "Euclidean reversed failed");
        check(std::fabs(rev.rotation_radians + t) < 0.005, "Euclidean reversed angle not negated");
    }
    check(cv::norm(output(base.get()), base_image, cv::NORM_INF) == 0, "Euclidean changed an input");

    // Same handle as both inputs.
    {
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(euc_call(base.get(), base.get(), &r) == OPENCV_VIDEO_OK, "Euclidean same-handle failed");
        check(std::fabs(r.rotation_radians) < 1e-6 && std::fabs(r.displacement_x) < 1e-6 &&
              std::fabs(r.displacement_y) < 1e-6 && std::fabs(r.correlation - 1.0) < 1e-6,
              "Euclidean same-handle identity");
    }
    // Distinct headers sharing storage.
    {
        auto first = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto second = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(first.get()) = output(base.get());
        output(second.get()) = output(base.get());
        check(output(first.get()).data == output(second.get()).data, "headers do not share storage");
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(euc_call(first.get(), second.get(), &r) == OPENCV_VIDEO_OK && std::fabs(r.rotation_radians) < 1e-6,
              "Euclidean shared-storage identity");
        check(cv::norm(output(base.get()), base_image, cv::NORM_INF) == 0, "shared storage modified");
    }

    // Argument and option failures leave every result byte unchanged.
    {
        const cv::Mat mv = euc_moving(base_image, 3 * d2r, 2, 1);
        auto moving = ecc_handle(mv);
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(euc_call(nullptr, moving.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null template");
        euc_unchanged(r, "null template changed result");
        check(euc_call(base.get(), nullptr, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null moving");
        euc_unchanged(r, "null moving changed result");
        check(euc_call(base.get(), moving.get(), nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null output");
        for (int iterations : {0, -1, 101, std::numeric_limits<int32_t>::max()}) {
            check(euc_call(base.get(), moving.get(), &r, iterations) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "invalid iterations accepted");
            euc_unchanged(r, "invalid iterations changed result");
        }
        for (double eps : {0.0, -1e-4, 1e-9, 0.11, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
            check(euc_call(base.get(), moving.get(), &r, 50, eps) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "invalid epsilon accepted");
            euc_unchanged(r, "invalid epsilon changed result");
        }
        check(euc_call(base.get(), moving.get(), &r, 1, 1e-8) == OPENCV_VIDEO_OK, "options lower bound");
        r = kEucSentinel;
        check(euc_call(base.get(), moving.get(), &r, 100, 0.1) == OPENCV_VIDEO_OK, "options upper bound");
        // Iteration and epsilon options reach the native call.
        r = kEucSentinel;
        check(euc_call(base.get(), moving.get(), &r, 1) == OPENCV_VIDEO_OK, "one iteration");
        euc_matches_direct(r, euc_direct(base_image, mv, 1), "iteration option not forwarded");
        opencv_video_ecc_euclidean_result full = kEucSentinel;
        check(euc_call(base.get(), moving.get(), &full) == OPENCV_VIDEO_OK, "default call");
        check(std::memcmp(&r, &full, sizeof r) != 0, "iteration count had no effect");
        r = kEucSentinel;
        check(euc_call(base.get(), moving.get(), &r, 50, 1e-8) == OPENCV_VIDEO_OK, "tight epsilon");
        euc_matches_direct(r, euc_direct(base_image, mv, 50, 1e-8), "epsilon option not forwarded");
        check(std::memcmp(&r, &full, sizeof r) != 0, "epsilon had no effect");

        // Wrong type, channels, geometry, envelope.
        r = kEucSentinel;
        auto flt = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
        auto color = matrix(n, n, OPENCV_CORE_DEPTH_UINT8, 3);
        auto other = matrix(n, n + 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto small = matrix(31, 31, OPENCV_CORE_DEPTH_UINT8, 1);
        auto wide = matrix(32, 16385, OPENCV_CORE_DEPTH_UINT8, 1);
        auto area_a = matrix(4097, 4096, OPENCV_CORE_DEPTH_UINT8, 1);
        for (const opencv_core_mat_handle *bad : {flt.get(), color.get(), other.get(), small.get(),
                                                 wide.get(), area_a.get()}) {
            check(euc_call(base.get(), bad, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad moving accepted");
            check(euc_call(bad, base.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad template accepted");
            euc_unchanged(r, "bad image changed result");
        }
    }

    // Nonconvergence is an error with the result untouched, never a fabricated identity.
    {
        auto constant = ecc_handle(ecc_fixture::constant(n, n, 128));
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(euc_call(constant.get(), constant.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV,
              "constant images did not fail");
        euc_unchanged(r, "nonconvergence changed result");
        auto noise = ecc_handle(ecc_fixture::noise(n, n, 3));
        auto noise2 = ecc_handle(ecc_fixture::noise(n, n, 9));
        check(euc_call(noise.get(), noise2.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV,
              "uncorrelated noise did not fail");
        euc_unchanged(r, "noise failure changed result");
    }

    // Centred rotation: the matrix translation, not the centre displacement, is returned.
    {
        const double t = 3 * d2r, cx = (n - 1) / 2.0, cy = (n - 1) / 2.0;
        const double tx = cx - (std::cos(t) * cx - std::sin(t) * cy) + 1.5;
        const double ty = cy - (std::sin(t) * cx + std::cos(t) * cy) - 1.0;
        auto moving = ecc_handle(euc_moving(base_image, t, tx, ty));
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(euc_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, "centred call failed");
        check(std::fabs(r.displacement_x - tx) < 0.1 && std::fabs(r.displacement_y - ty) < 0.1 &&
              std::fabs(r.rotation_radians - t) < 0.005, "centred rotation matrix translation");
        check(std::fabs(r.displacement_x - 1.5) > 1.0, "centred translation equals centre displacement");
    }

    // Genuine non-contiguous Regions equal the direct call on the same Regions.
    {
        const cv::Mat parent_a = ecc_fixture::texture(160, 160);
        const cv::Mat parent_b = euc_moving(parent_a, 3 * d2r, 2, 1);
        const cv::Rect roi(16, 16, n, n);
        auto h_a = ecc_handle(parent_a);
        auto h_b = ecc_handle(parent_b);
        auto region_a = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto region_b = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(region_a.get()) = output(h_a.get())(roi);
        output(region_b.get()) = output(h_b.get())(roi);
        check(!output(region_a.get()).isContinuous(), "fixture Region is contiguous");
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(euc_call(region_a.get(), region_b.get(), &r) == OPENCV_VIDEO_OK, "Euclidean Region call failed");
        euc_matches_direct(r, euc_direct(parent_a(roi), parent_b(roi)),
                           "Euclidean Region differs from direct native Region call");
        check(cv::norm(output(h_a.get()), parent_a, cv::NORM_INF) == 0 &&
              cv::norm(output(h_b.get()), parent_b, cv::NORM_INF) == 0, "Euclidean changed Region parents");
    }

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        auto moving = ecc_handle(euc_moving(base_image, 3 * d2r, 2, 1));
        for (int stage : {1, 2}) for (int kind : {1, 2, 3, 4}) {
            fault_stage = stage; fault_kind = kind;
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = euc_call(base.get(), moving.get(), &r);
            fault_stage = 0;
            check(code == (kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                           : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD),
                  "Euclidean injected exception mapped to the wrong status");
            euc_unchanged(r, "Euclidean injected failure changed result");
        }
        // 1 NaN X, 2 Inf Y, 3 NaN rho, 4 -Inf rho, 5 scaled a, 6 asymmetric b, 7 scaled
        // rotation, 8 reflection, 9 NaN rotation entry, 10 asymmetric c.
        for (int mode : {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = euc_call(base.get(), moving.get(), &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "Euclidean corrupted native output accepted");
            euc_unchanged(r, "Euclidean corrupted native output was partially published");
        }
        // Valid rigid warps beyond a quarter turn are published with the atan2 angle and sign
        // (an asin decode would return pi - theta here).
        for (int mode : {11, 12}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = euc_call(base.get(), moving.get(), &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "Euclidean valid large-angle warp rejected");
            const double expected = (mode == 11 ? 120.0 : -150.0) * kPi / 180.0;
            check(std::fabs(r.rotation_radians - expected) < 1e-6, "Euclidean large angle decoded wrongly");
            check(r.displacement_x == 3.0f && r.displacement_y == -4.0f, "Euclidean large-angle translation");
        }
    }
#endif
    auto moving = ecc_handle(euc_moving(base_image, 3 * d2r, 2, 1));
    opencv_video_ecc_euclidean_result r = kEucSentinel;
    check(euc_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, "post-fault Euclidean call failed");
    std::cout << "PASS: ECC Euclidean shim vs direct findTransformECC, ABI layout, signs, roles, options, nonconvergence, Regions, atomicity, faults, corruption\n";
}
}  // namespace



// Seeded ECC Euclidean (Task 013): actual shim versus a direct cv::findTransformECC call
// started from an independently constructed nonidentity warp.
namespace {
opencv_video_status seeded_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                                double theta, float tx, float ty,
                                opencv_video_ecc_euclidean_result *r, int iterations = 50,
                                double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_euclidean_seeded(a, b, iterations, epsilon, tx, ty, theta, r);
}

struct SeededDirect { bool ok; double rho; float w[6]; double theta; };

SeededDirect seeded_direct(const cv::Mat &a, const cv::Mat &b, double theta, float tx, float ty,
                           int iterations = 50, double eps = 1e-4) {
    cv::Mat warp(2, 3, CV_32F);
    const double c = std::cos(theta), s = std::sin(theta);
    warp.at<float>(0, 0) = float(c);  warp.at<float>(0, 1) = float(-s);  warp.at<float>(0, 2) = tx;
    warp.at<float>(1, 0) = float(s);  warp.at<float>(1, 1) = float(c);   warp.at<float>(1, 2) = ty;
    SeededDirect d{};
    try {
        d.rho = cv::findTransformECC(a, b, warp, cv::MOTION_EUCLIDEAN,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, eps),
            cv::noArray(), 5);
        d.ok = true;
    } catch (const cv::Exception &) { d.ok = false; return d; }
    for (int i = 0; i < 6; ++i) d.w[i] = warp.ptr<float>()[i];
    d.theta = std::atan2(double(d.w[3]), double(d.w[0]));
    return d;
}

void seeded_matches(const opencv_video_ecc_euclidean_result &r, const SeededDirect &d, const char *m) {
    check(d.ok && r.displacement_x == d.w[2] && r.displacement_y == d.w[5] &&
          r.rotation_radians == d.theta && r.correlation == d.rho, m);
}

void run_ecc_euclidean_seeded() {
    const int n = 128;
    const double d2r = kPi / 180.0;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);

    // Native initialization is the supplied nonzero warp: shim == direct seeded call, and the
    // distinguishing fixtures differ from identity initialization.
    struct Case { double deg, tx, ty, sdeg, sx, sy; bool distinguishing; } cases[] = {
        {3, 2, 1, 2.5, 2.3, 0.8, false}, {-3, 2, -1, -2.5, 2.3, -0.8, false},
        {8, 0, 0, 7.5, 0.3, -0.2, true}, {-8, 0, 0, -8.5, 0.3, -0.2, true},
        {10, 2, 1, 9.5, 2.3, 0.8, true}, {12, 0, 0, 11.5, 0.3, -0.2, true}};
    for (const auto &c : cases) {
        const cv::Mat mv = ecc_fixture::rigid_texture(n, n, c.deg, c.tx, c.ty);
        auto moving = ecc_handle(mv);
        const double st = c.sdeg * d2r;
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(seeded_call(base.get(), moving.get(), st, float(c.sx), float(c.sy), &r) == OPENCV_VIDEO_OK,
              "seeded valid call failed");
        seeded_matches(r, seeded_direct(base_image, mv, st, float(c.sx), float(c.sy)),
                       "seeded shim differs from direct seeded native call");
        check(std::fabs(r.rotation_radians - c.deg * d2r) < 0.005 &&
              std::fabs(r.displacement_x - c.tx) < 0.1 && std::fabs(r.displacement_y - c.ty) < 0.1,
              "seeded result did not recover the transform");
        opencv_video_ecc_euclidean_result again = kEucSentinel;
        check(seeded_call(base.get(), moving.get(), st, float(c.sx), float(c.sy), &again) == OPENCV_VIDEO_OK &&
              std::memcmp(&r, &again, sizeof r) == 0, "seeded call is not repeatable");
        if (c.distinguishing) {
            opencv_video_ecc_euclidean_result plain = kEucSentinel;
            const auto code = euc_call(base.get(), moving.get(), &plain);
            check(code != OPENCV_VIDEO_OK || std::memcmp(&plain, &r, sizeof r) != 0,
                  "identity initialization matched the seeded result on a distinguishing fixture");
        }
    }

    // Zero seed == identity-initialized export, byte for byte, including failures.
    {
        struct Z { double deg, tx, ty; } zeros[] = {{0, 0, 0}, {0, 2, 1}, {3, 0, 0}, {3, 2, 1}, {10, 2, 1}, {12, 0, 0}};
        for (const auto &z : zeros) {
            auto moving = ecc_handle(ecc_fixture::rigid_texture(n, n, z.deg, z.tx, z.ty));
            opencv_video_ecc_euclidean_result plain = kEucSentinel, seeded = kEucSentinel;
            const auto a = euc_call(base.get(), moving.get(), &plain);
            const auto b = seeded_call(base.get(), moving.get(), 0.0, 0.0f, 0.0f, &seeded);
            check(a == b, "zero seed changed the status");
            check(std::memcmp(&plain, &seeded, sizeof plain) == 0, "zero seed changed the result bytes");
        }
    }

    // Two different valid seeds on the same images give different results.
    {
        const cv::Mat mv = ecc_fixture::rigid_texture(n, n, 10, 0, 0);
        auto moving = ecc_handle(mv);
        opencv_video_ecc_euclidean_result good = kEucSentinel, far = kEucSentinel;
        check(seeded_call(base.get(), moving.get(), 9.5 * d2r, 0.3f, -0.2f, &good) == OPENCV_VIDEO_OK, "good seed");
        check(seeded_call(base.get(), moving.get(), 10 * d2r, 25.0f, -25.0f, &far) == OPENCV_VIDEO_OK, "far seed");
        check(std::memcmp(&good, &far, sizeof good) != 0, "two different seeds produced the same result");
        seeded_matches(far, seeded_direct(base_image, mv, 10 * d2r, 25.0f, -25.0f), "far seed differs from direct");
        check(far.correlation < 0.9 && good.correlation > 0.98, "wrong-but-finite seed result");
    }

    // Exact accepted bounds reach native code; values just beyond them do not.
    {
        auto moving = ecc_handle(ecc_fixture::rigid_texture(n, n, 3, 2, 1));
        const float limit = 1048576.0f;
        const double bound_angles[] = {1.0, -1.0};
        for (double angle : bound_angles) {
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = seeded_call(base.get(), moving.get(), angle, 0.0f, 0.0f, &r);
            check(code == OPENCV_VIDEO_OK || code == OPENCV_VIDEO_ERROR_OPENCV, "angle bound rejected by preflight");
            if (code != OPENCV_VIDEO_OK) euc_unchanged(r, "native failure at the angle bound changed the result");
        }
        for (float sign : {1.0f, -1.0f}) {
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = seeded_call(base.get(), moving.get(), 0.0, sign * limit, -sign * limit, &r);
            check(code == OPENCV_VIDEO_OK || code == OPENCV_VIDEO_ERROR_OPENCV, "translation bound rejected by preflight");
            if (code != OPENCV_VIDEO_OK) euc_unchanged(r, "native failure at the translation bound changed the result");
        }
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        const float fnan = std::numeric_limits<float>::quiet_NaN();
        const float finf = std::numeric_limits<float>::infinity();
        struct Bad { double theta; float tx, ty; const char *what; } bad[] = {
            {nan, 0, 0, "NaN angle"}, {inf, 0, 0, "+Inf angle"}, {-inf, 0, 0, "-Inf angle"},
            {1.0000000000000002, 0, 0, "angle just above +1"}, {-1.0000000000000002, 0, 0, "angle just below -1"},
            {3.0, 0, 0, "angle outside the asin branch"}, {1e300, 0, 0, "huge angle"},
            {0, fnan, 0, "NaN tx"}, {0, 0, fnan, "NaN ty"}, {0, finf, 0, "+Inf tx"}, {0, 0, -finf, "-Inf ty"},
            {0, 1048576.125f, 0, "tx just above 2^20"}, {0, 0, -1048576.125f, "ty just below -2^20"},
            {0, 3.0e38f, 0, "huge tx"}};
        for (const auto &b : bad) {
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            check(seeded_call(base.get(), moving.get(), b.theta, b.tx, b.ty, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, b.what);
            euc_unchanged(r, b.what);
        }
    }

    // Argument, option and image failures leave the result bytes unchanged.
    {
        const cv::Mat mv = ecc_fixture::rigid_texture(n, n, 3, 2, 1);
        auto moving = ecc_handle(mv);
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(seeded_call(nullptr, moving.get(), 0.04, 2.0f, 1.0f, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null template");
        check(seeded_call(base.get(), nullptr, 0.04, 2.0f, 1.0f, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null moving");
        check(seeded_call(base.get(), moving.get(), 0.04, 2.0f, 1.0f, nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null output");
        euc_unchanged(r, "null argument changed the result");
        for (int it : {0, -1, 101}) {
            check(seeded_call(base.get(), moving.get(), 0.04, 2.0f, 1.0f, &r, it) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "iterations");
            euc_unchanged(r, "invalid iterations changed the result");
        }
        for (double eps : {0.0, 1e-9, 0.11, std::numeric_limits<double>::quiet_NaN()}) {
            check(seeded_call(base.get(), moving.get(), 0.04, 2.0f, 1.0f, &r, 50, eps) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "epsilon");
            euc_unchanged(r, "invalid epsilon changed the result");
        }
        auto flt = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
        auto other = matrix(n, n + 1, OPENCV_CORE_DEPTH_UINT8, 1);
        for (const opencv_core_mat_handle *bad : {flt.get(), other.get()}) {
            check(seeded_call(base.get(), bad, 0.04, 2.0f, 1.0f, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad moving");
            check(seeded_call(bad, base.get(), 0.04, 2.0f, 1.0f, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad template");
            euc_unchanged(r, "bad image changed the result");
        }
        // Iteration and epsilon options reach the seeded native call.
        r = kEucSentinel;
        check(seeded_call(base.get(), moving.get(), 2.5 * d2r, 2.3f, 0.8f, &r, 1) == OPENCV_VIDEO_OK, "one iteration");
        seeded_matches(r, seeded_direct(base_image, mv, 2.5 * d2r, 2.3f, 0.8f, 1), "iteration option not forwarded");
        opencv_video_ecc_euclidean_result tight = kEucSentinel;
        check(seeded_call(base.get(), moving.get(), 2.5 * d2r, 2.3f, 0.8f, &tight, 50, 1e-8) == OPENCV_VIDEO_OK, "tight");
        seeded_matches(tight, seeded_direct(base_image, mv, 2.5 * d2r, 2.3f, 0.8f, 50, 1e-8), "epsilon option not forwarded");
        check(std::memcmp(&r, &tight, sizeof r) != 0, "iteration count had no effect");
    }

    // Nonconvergence with a nonzero seed: error, result untouched.
    {
        auto constant = ecc_handle(ecc_fixture::constant(n, n, 128));
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(seeded_call(constant.get(), constant.get(), 0.02, 1.0f, 1.0f, &r) == OPENCV_VIDEO_ERROR_OPENCV, "constant");
        euc_unchanged(r, "nonconvergence changed the result");
        auto noise_a = ecc_handle(ecc_fixture::noise(n, n, 3));
        auto noise_b = ecc_handle(ecc_fixture::noise(n, n, 9));
        check(seeded_call(noise_a.get(), noise_b.get(), 0.02, 1.0f, 1.0f, &r) == OPENCV_VIDEO_ERROR_OPENCV, "noise");
        euc_unchanged(r, "noise failure changed the result");
        auto rot12 = ecc_handle(ecc_fixture::rigid_texture(n, n, 12, 0, 0));
        check(seeded_call(base.get(), rot12.get(), -12 * d2r, 0.0f, 0.0f, &r) == OPENCV_VIDEO_ERROR_OPENCV, "wrong-sign seed");
        euc_unchanged(r, "wrong-sign seed failure changed the result");
    }

    // Same handle, shared storage, centred rotation, Regions.
    {
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(seeded_call(base.get(), base.get(), 0.02, 0.5f, -0.5f, &r) == OPENCV_VIDEO_OK &&
              std::fabs(r.rotation_radians) < 1e-4 && std::fabs(r.displacement_x) < 0.02 &&
              std::fabs(r.displacement_y) < 0.02, "same-handle seeded identity (seed moved by 0.5 px; residual is epsilon-limited)");
        auto first = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto second = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(first.get()) = output(base.get());
        output(second.get()) = output(base.get());
        r = kEucSentinel;
        check(seeded_call(first.get(), second.get(), 0.02, 0.5f, -0.5f, &r) == OPENCV_VIDEO_OK &&
              std::fabs(r.rotation_radians) < 1e-4, "shared-storage seeded identity");
        check(cv::norm(output(base.get()), base_image, cv::NORM_INF) == 0, "seeded call modified a shared input");

        const double t = 3 * d2r, cx = (n - 1) / 2.0, cy = (n - 1) / 2.0;
        const double tx = cx - (std::cos(t) * cx - std::sin(t) * cy) + 1.5;
        const double ty = cy - (std::sin(t) * cx + std::cos(t) * cy) - 1.0;
        auto centred = ecc_handle(ecc_fixture::rigid_texture(n, n, 3, tx, ty));
        r = kEucSentinel;
        check(seeded_call(base.get(), centred.get(), 2.6 * d2r, float(tx + 0.4), float(ty - 0.3), &r) == OPENCV_VIDEO_OK,
              "centred seeded call");
        check(std::fabs(r.displacement_x - tx) < 0.1 && std::fabs(r.displacement_y - ty) < 0.1 &&
              std::fabs(r.rotation_radians - t) < 0.005 && std::fabs(r.displacement_x - 1.5) > 1.0,
              "centred rotation matrix translation");

        const cv::Mat parent_a = ecc_fixture::texture(160, 160);
        const cv::Mat parent_b = ecc_fixture::rigid_texture(160, 160, 3, 2, 1);
        const cv::Rect roi(16, 16, n, n);
        auto h_a = ecc_handle(parent_a);
        auto h_b = ecc_handle(parent_b);
        auto region_a = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto region_b = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(region_a.get()) = output(h_a.get())(roi);
        output(region_b.get()) = output(h_b.get())(roi);
        check(!output(region_a.get()).isContinuous(), "fixture Region is contiguous");
        r = kEucSentinel;
        check(seeded_call(region_a.get(), region_b.get(), 2.5 * d2r, 2.3f, 0.8f, &r) == OPENCV_VIDEO_OK, "Region seeded call");
        seeded_matches(r, seeded_direct(parent_a(roi), parent_b(roi), 2.5 * d2r, 2.3f, 0.8f), "Region differs from direct");
        check(cv::norm(output(h_a.get()), parent_a, cv::NORM_INF) == 0 &&
              cv::norm(output(h_b.get()), parent_b, cv::NORM_INF) == 0, "seeded call changed Region parents");
    }

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        auto moving = ecc_handle(ecc_fixture::rigid_texture(n, n, 3, 2, 1));
        for (int stage : {1, 2}) for (int kind : {1, 2, 3, 4}) {
            fault_stage = stage; fault_kind = kind;
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = seeded_call(base.get(), moving.get(), 2.5 * d2r, 2.3f, 0.8f, &r);
            fault_stage = 0;
            check(code == (kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                           : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD),
                  "seeded injected exception mapped to the wrong status");
            euc_unchanged(r, "seeded injected failure changed the result");
        }
        for (int mode : {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = seeded_call(base.get(), moving.get(), 2.5 * d2r, 2.3f, 0.8f, &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "seeded corrupted native output accepted");
            euc_unchanged(r, "seeded corrupted native output was partially published");
        }
        // Valid large-angle OUTPUT warps are decoded with atan2; this does not authorize such seeds.
        for (int mode : {11, 12}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_euclidean_result r = kEucSentinel;
            const auto code = seeded_call(base.get(), moving.get(), 2.5 * d2r, 2.3f, 0.8f, &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "seeded valid large-angle output warp rejected");
            const double expected = (mode == 11 ? 120.0 : -150.0) * kPi / 180.0;
            check(std::fabs(r.rotation_radians - expected) < 1e-6 && r.displacement_x == 3.0f &&
                  r.displacement_y == -4.0f, "seeded large-angle output decoded wrongly");
        }
        // Seed validation precedes native execution: a rejected seed never reaches a fault hook.
        fault_stage = 1; fault_kind = 1;
        opencv_video_ecc_euclidean_result r = kEucSentinel;
        check(seeded_call(base.get(), moving.get(), 3.0, 0.0f, 0.0f, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "invalid seed must be rejected before native execution");
        fault_stage = 0;
        euc_unchanged(r, "rejected seed changed the result");
    }
#endif
    auto moving = ecc_handle(ecc_fixture::rigid_texture(n, n, 3, 2, 1));
    opencv_video_ecc_euclidean_result r = kEucSentinel;
    check(seeded_call(base.get(), moving.get(), 2.5 * d2r, 2.3f, 0.8f, &r) == OPENCV_VIDEO_OK, "post-fault seeded call failed");
    std::cout << "PASS: seeded ECC Euclidean shim vs direct nonidentity findTransformECC, zero seed, bounds, signs, atomicity, aliases, Regions, faults, corruption\n";
}
}  // namespace

// ECC affine (Task 014): actual shim versus a direct cv::findTransformECC(MOTION_AFFINE)
// call, with byte-exact failure atomicity on a fixed 32-byte sentinel.
namespace {
static_assert(sizeof(opencv_video_ecc_affine_result) == 32 &&
              alignof(opencv_video_ecc_affine_result) == 8 &&
              offsetof(opencv_video_ecc_affine_result, m00) == 0 &&
              offsetof(opencv_video_ecc_affine_result, m01) == 4 &&
              offsetof(opencv_video_ecc_affine_result, m02) == 8 &&
              offsetof(opencv_video_ecc_affine_result, m10) == 12 &&
              offsetof(opencv_video_ecc_affine_result, m11) == 16 &&
              offsetof(opencv_video_ecc_affine_result, m12) == 20 &&
              offsetof(opencv_video_ecc_affine_result, correlation) == 24,
              "ECC affine result layout");

const opencv_video_ecc_affine_result kAffSentinel = {11.5f, -22.25f, 33.125f, -44.0625f, 55.03125f, -66.5f, 77.75};

opencv_video_status aff_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                             opencv_video_ecc_affine_result *r, int iterations = 50,
                             double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_affine(a, b, iterations, epsilon, r);
}

void aff_unchanged(const opencv_video_ecc_affine_result &r, const char *message) {
    check(std::memcmp(&r, &kAffSentinel, sizeof r) == 0, message);
}

struct AffDirect {
    float w[6];
    double rho;
};

AffDirect aff_direct(const cv::Mat &a, const cv::Mat &b, int iterations = 50, double epsilon = 1e-4) {
    AffDirect d{};
    cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
    d.rho = cv::findTransformECC(a, b, warp, cv::MOTION_AFFINE,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, epsilon),
        cv::noArray(), 5);
    for (int i = 0; i < 6; ++i) d.w[i] = warp.ptr<float>()[i];
    return d;
}

void aff_matches_direct(const opencv_video_ecc_affine_result &r, const AffDirect &d, const char *message) {
    // Bitwise: same library, same inputs, deterministic CPU path.
    check(std::memcmp(&r.m00, &d.w[0], 4) == 0 && std::memcmp(&r.m01, &d.w[1], 4) == 0 &&
          std::memcmp(&r.m02, &d.w[2], 4) == 0 && std::memcmp(&r.m10, &d.w[3], 4) == 0 &&
          std::memcmp(&r.m11, &d.w[4], 4) == 0 && std::memcmp(&r.m12, &d.w[5], 4) == 0 &&
          r.correlation == d.rho, message);
}

cv::Mat aff_moving(int rows, int cols, double a, double b, double c, double d, double tx, double ty) {
    return ecc_fixture::affine_texture(rows, cols, a, b, c, d, tx, ty);
}

void run_ecc_affine() {
    const int n = 128;
    const double d2r = 3.14159265358979323846 / 180.0;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);
    const cv::Mat before_base = base_image.clone();

    struct Case {
        const char *name;
        double a, b, c, d, tx, ty;
    };
    const Case cases[] = {
        {"identity", 1, 0, 0, 1, 0, 0},
        {"translation", 1, 0, 0, 1, 2, 1},
        {"negative translation", 1, 0, 0, 1, -2, -1},
        {"rotation +3", std::cos(3 * d2r), -std::sin(3 * d2r), std::sin(3 * d2r), std::cos(3 * d2r), 0, 0},
        {"rotation -3", std::cos(-3 * d2r), -std::sin(-3 * d2r), std::sin(-3 * d2r), std::cos(-3 * d2r), 0, 0},
        {"anisotropic scale", 1.03, 0, 0, 0.97, 0, 0},
        {"x shear", 1, 0.03, 0, 1, 0, 0},
        {"y shear", 1, 0, -0.008, 1, 0, 0},
        {"combined", 1.01, 0.012, -0.008, 0.99, 2, 1},
    };
    for (const auto &c : cases) {
        const cv::Mat moving_image = aff_moving(n, n, c.a, c.b, c.c, c.d, c.tx, c.ty);
        auto moving = ecc_handle(moving_image);
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(aff_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, c.name);
        aff_matches_direct(r, aff_direct(base_image, moving_image), c.name);
        const double truth[6] = {c.a, c.b, c.tx, c.c, c.d, c.ty};
        const float got[6] = {r.m00, r.m01, r.m02, r.m10, r.m11, r.m12};
        for (int i = 0; i < 6; ++i)
            check(std::fabs(got[i] - truth[i]) < (i == 2 || i == 5 ? 0.1 : 0.003),
                  "affine coefficient position, sign or magnitude");
        check(std::isfinite(r.correlation) && r.correlation > 0.98, "affine correlation");
        opencv_video_ecc_affine_result again = kAffSentinel;
        check(aff_call(base.get(), moving.get(), &again) == OPENCV_VIDEO_OK &&
              std::memcmp(&again, &r, sizeof r) == 0, "affine repeatability");
    }
    // The same handle in both roles is the identity; reversing roles approximately inverts.
    {
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(aff_call(base.get(), base.get(), &r) == OPENCV_VIDEO_OK, "same-handle affine call");
        check(std::fabs(r.m00 - 1) < 1e-5 && std::fabs(r.m11 - 1) < 1e-5 && std::fabs(r.m01) < 1e-5 &&
              std::fabs(r.m10) < 1e-5 && std::fabs(r.m02) < 1e-4 && std::fabs(r.m12) < 1e-4 &&
              std::fabs(r.correlation - 1.0) < 1e-6, "same-handle affine identity");
        const cv::Mat mv = aff_moving(n, n, 1.01, 0.012, -0.008, 0.99, 2, 1);
        auto moving = ecc_handle(mv);
        opencv_video_ecc_affine_result fwd = kAffSentinel, rev = kAffSentinel;
        check(aff_call(base.get(), moving.get(), &fwd) == OPENCV_VIDEO_OK &&
              aff_call(moving.get(), base.get(), &rev) == OPENCV_VIDEO_OK, "role reversal");
        check(rev.m00 < 1.0f && rev.m02 < 0.0f && fwd.m02 > 0.0f, "reversed roles did not invert the transform");
        // Two different headers sharing one allocation behave like one image.
        auto alias = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(alias.get()) = output(base.get());
        opencv_video_ecc_affine_result a = kAffSentinel;
        check(aff_call(base.get(), alias.get(), &a) == OPENCV_VIDEO_OK &&
              std::fabs(a.correlation - 1.0) < 1e-6, "shared-allocation headers");
    }

    // Argument validation and byte-for-byte result preservation.
    {
        auto moving = ecc_handle(aff_moving(n, n, 1.01, 0.012, -0.008, 0.99, 2, 1));
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(aff_call(nullptr, moving.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null template");
        aff_unchanged(r, "null template changed result");
        check(aff_call(base.get(), nullptr, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null moving");
        aff_unchanged(r, "null moving changed result");
        check(aff_call(base.get(), moving.get(), nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "null result accepted");
        for (int iterations : {0, -1, 101})
            check(aff_call(base.get(), moving.get(), &r, iterations) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "bad iterations accepted");
        for (double epsilon : {0.0, -1e-4, 9e-9, 0.1000001, std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()})
            check(aff_call(base.get(), moving.get(), &r, 50, epsilon) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "bad epsilon accepted");
        aff_unchanged(r, "invalid options changed result");
        check(aff_call(base.get(), moving.get(), &r, 1, 1e-8) == OPENCV_VIDEO_OK, "options lower bound");
        r = kAffSentinel;
        check(aff_call(base.get(), moving.get(), &r, 100, 0.1) == OPENCV_VIDEO_OK, "options upper bound");
        r = kAffSentinel;
        auto wrong_depth = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
        auto wrong_channels = matrix(n, n, OPENCV_CORE_DEPTH_UINT8, 3);
        auto other_size = matrix(n, n + 1, OPENCV_CORE_DEPTH_UINT8, 1);
        for (auto *bad : {wrong_depth.get(), wrong_channels.get(), other_size.get()}) {
            check(aff_call(base.get(), bad, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad moving accepted");
            check(aff_call(bad, base.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "bad template accepted");
        }
        auto small = matrix(31, 128, OPENCV_CORE_DEPTH_UINT8, 1);
        auto small_peer = matrix(31, 128, OPENCV_CORE_DEPTH_UINT8, 1);
        check(aff_call(small.get(), small_peer.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "below minimum dimension accepted");
        auto empty = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(empty.get()) = cv::Mat();
        check(aff_call(empty.get(), base.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "empty accepted");
        auto wide = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto wide_peer = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(wide.get()) = cv::Mat(32, 16385, CV_8UC1, cv::Scalar(0));
        output(wide_peer.get()) = cv::Mat(32, 16385, CV_8UC1, cv::Scalar(0));
        check(aff_call(wide.get(), wide_peer.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "above maximum dimension accepted");
        // Area bound: 4097 x 4096 exceeds 2^24 pixels. Header-only Mats with shared 1-row
        // storage would be invalid, so use real zero-filled storage (about 16 MiB each).
        auto area = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto area_peer = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(area.get()) = cv::Mat(4097, 4096, CV_8UC1, cv::Scalar(0));
        output(area_peer.get()) = cv::Mat(4097, 4096, CV_8UC1, cv::Scalar(0));
        check(aff_call(area.get(), area_peer.get(), &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "above maximum area accepted");
        aff_unchanged(r, "invalid input changed result");
    }

    // Native nonconvergence is a failure that leaves the caller's bytes unchanged.
    {
        auto flat = ecc_handle(ecc_fixture::constant(n, n, 128));
        auto noise_a = ecc_handle(ecc_fixture::noise(n, n, 3));
        auto noise_b = ecc_handle(ecc_fixture::noise(n, n, 9));
        auto reflected = ecc_handle(aff_moving(n, n, -1, 0, 0, 1, 0, 0));
        auto strong = ecc_handle(aff_moving(n, n, 1.2, 0.1, -0.1, 0.8, 5, -3));
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(aff_call(flat.get(), flat.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, "constant images accepted");
        aff_unchanged(r, "constant failure changed result");
        check(aff_call(noise_a.get(), noise_b.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, "noise accepted");
        aff_unchanged(r, "noise failure changed result");
        check(aff_call(base.get(), reflected.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, "reflection accepted");
        aff_unchanged(r, "reflection failure changed result");
        check(aff_call(base.get(), strong.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, "strong deformation accepted");
        aff_unchanged(r, "strong deformation failure changed result");
        check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after nonconvergence");
    }

    // Genuine non-contiguous Regions equal the direct call on the same Regions.
    {
        const cv::Mat parent_a = ecc_fixture::texture(160, 160);
        const cv::Mat parent_b = aff_moving(160, 160, 1.01, 0.012, -0.008, 0.99, 2, 1);
        const cv::Rect roi(16, 16, n, n);
        auto h_a = ecc_handle(parent_a);
        auto h_b = ecc_handle(parent_b);
        auto region_a = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto region_b = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(region_a.get()) = output(h_a.get())(roi);
        output(region_b.get()) = output(h_b.get())(roi);
        check(!output(region_a.get()).isContinuous() && !output(region_b.get()).isContinuous(),
              "fixture Region is contiguous");
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(aff_call(region_a.get(), region_b.get(), &r) == OPENCV_VIDEO_OK, "affine Region call failed");
        aff_matches_direct(r, aff_direct(parent_a(roi), parent_b(roi)),
                           "affine Region differs from direct native Region call");
        check(cv::norm(output(h_a.get()), parent_a, cv::NORM_INF) == 0 &&
              cv::norm(output(h_b.get()), parent_b, cv::NORM_INF) == 0, "affine changed Region parents");
    }
    check(cv::norm(output(base.get()), before_base, cv::NORM_INF) == 0, "affine changed an input");

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        const cv::Mat mv = aff_moving(n, n, 1.01, 0.012, -0.008, 0.99, 2, 1);
        auto moving = ecc_handle(mv);
        // All eight exception injections (before and after native execution).
        for (int stage : {1, 2}) for (int kind : {1, 2, 3, 4}) {
            fault_stage = stage; fault_kind = kind;
            opencv_video_ecc_affine_result r = kAffSentinel;
            const auto code = aff_call(base.get(), moving.get(), &r);
            fault_stage = 0;
            check(code == (kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                           : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD),
                  "ECC affine injected exception mapped to the wrong status");
            aff_unchanged(r, "ECC affine injected failure changed result");
        }
        // Post-native nonfinite corruption: NaN/+Inf/-Inf in each of the six coefficients and
        // in the correlation must be rejected with the caller's 32 bytes untouched.
        for (int mode : {1, 2, 3, 4, 9, 21, 22, 23, 24, 25, 26}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_affine_result r = kAffSentinel;
            const auto code = aff_call(base.get(), moving.get(), &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "ECC affine nonfinite native output accepted");
            aff_unchanged(r, "ECC affine nonfinite native output was partially published");
            check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after rejected affine output");
        }
        // Finite non-rigid output is legitimate affine output and must be published: the
        // Euclidean rotation/determinant policy is NOT applied to the affine entry point.
        // Modes 5/6/7/8/10/13 perturb the linear part of a real fit (not identity, scaled,
        // reflected, asymmetric); 11/12 are rigid beyond the Euclidean angle range.
        for (int mode : {5, 6, 7, 8, 10, 11, 12, 13}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_affine_result r = kAffSentinel;
            const auto code = aff_call(base.get(), moving.get(), &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "ECC affine rejected a finite non-rigid matrix");
            check(std::isfinite(r.m00) && std::isfinite(r.m01) && std::isfinite(r.m02) &&
                  std::isfinite(r.m10) && std::isfinite(r.m11) && std::isfinite(r.m12) &&
                  std::memcmp(&r, &kAffSentinel, sizeof r) != 0, "ECC affine finite output not published");
        }
        struct Exact { int mode; float w[6]; };
        const Exact exact[] = {
            {30, {1.3f, 0.4f, 5.0f, -0.2f, 0.7f, -3.0f}},       // anisotropic scale + shear
            {31, {-1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}},       // reflection, det = -1
            {32, {1.0f, 2.0f, 0.5f, 0.5f, 1.0f, -0.5f}},       // singular, det = 0
            {33, {3.0e38f, -3.0e38f, 1.0e30f, 1.0e-30f, -2.0e38f, 3.0e38f}},  // extreme finite
        };
        for (const auto &e : exact) {
            ecc_corrupt_mode = e.mode;
            opencv_video_ecc_affine_result r = kAffSentinel;
            const auto code = aff_call(base.get(), moving.get(), &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "ECC affine rejected an injected finite matrix");
            const float got[6] = {r.m00, r.m01, r.m02, r.m10, r.m11, r.m12};
            check(std::memcmp(got, e.w, sizeof got) == 0, "ECC affine altered an injected finite matrix");
            check(std::isfinite(r.correlation), "ECC affine injected-matrix correlation");
        }
    }
#endif
    auto moving = ecc_handle(aff_moving(n, n, 1.01, 0.012, -0.008, 0.99, 2, 1));
    opencv_video_ecc_affine_result r = kAffSentinel;
    check(aff_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, "post-fault affine call failed");
    std::cout << "PASS: ECC affine shim vs direct MOTION_AFFINE findTransformECC, six coefficients, roles, options, schema, nonconvergence, Regions, atomicity, faults, corruption\n";
}
}  // namespace


// ECC affine seeded (Task 015): the new export versus a direct seeded findTransformECC.
namespace {
opencv_video_status seeded_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                                const float s[6], opencv_video_ecc_affine_result *r,
                                int iterations = 50, double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_affine_seeded(a, b, iterations, epsilon, s[0], s[1], s[2],
                                                   s[3], s[4], s[5], r);
}

// Direct native call with the six seed values in native storage order; ok=false on cv::Exception.
bool seeded_direct(const cv::Mat &a, const cv::Mat &b, const float s[6], AffDirect &d,
                   int iterations = 50, double epsilon = 1e-4) {
    cv::Mat warp(2, 3, CV_32F);
    for (int i = 0; i < 6; ++i) warp.ptr<float>()[i] = s[i];
    try {
        d.rho = cv::findTransformECC(a, b, warp, cv::MOTION_AFFINE,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, epsilon),
            cv::noArray(), 5);
    } catch (const cv::Exception &) { return false; }
    for (int i = 0; i < 6; ++i) d.w[i] = warp.ptr<float>()[i];
    return true;
}

void run_ecc_affine_seeded() {
    const int n = 128;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);
    const cv::Mat before_base = base_image.clone();
    const float identity[6] = {1, 0, 0, 0, 1, 0};

    struct Case { const char *name; double t[6]; float seed[6]; bool identity_fails; };
    // t = a b tx c d ty; seed in storage order m00 m01 m02 m10 m11 m12.
    const Case cases[] = {
        {"moderate shear", {1.00, 0.12, 1, -0.05, 1.00, -1}, {1.00f, 0.10f, 0.5f, -0.04f, 1.00f, -0.7f}, true},
        {"nonuniform scale", {1.12, 0.02, 2, -0.01, 0.88, 1}, {1.10f, 0.01f, 1.5f, 0.0f, 0.90f, 0.8f}, true},
        {"combined", {1.08, 0.08, 2, -0.04, 0.94, -1}, {1.06f, 0.07f, 1.7f, -0.03f, 0.95f, -0.8f}, true},
        {"strong", {1.2, 0.1, 5, -0.1, 0.8, -3}, {1.18f, 0.09f, 4.5f, -0.09f, 0.82f, -2.7f}, true},
        {"shear 0.2", {1, 0.2, 0, 0, 1, 0}, {1.0f, 0.18f, 0.0f, 0.0f, 1.0f, 0.0f}, true},
        {"small", {1.01, 0.012, 2, -0.008, 0.99, 1}, {1.0f, 0.01f, 1.5f, -0.005f, 1.0f, 0.5f}, false},
    };
    for (const auto &c : cases) {
        const cv::Mat moving_image =
            aff_moving(n, n, c.t[0], c.t[1], c.t[3], c.t[4], c.t[2], c.t[5]);
        auto moving = ecc_handle(moving_image);
        AffDirect direct{};
        check(seeded_direct(base_image, moving_image, c.seed, direct), c.name);
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(seeded_call(base.get(), moving.get(), c.seed, &r) == OPENCV_VIDEO_OK, c.name);
        aff_matches_direct(r, direct, "seeded export differs from the direct seeded native call");
        const double truth[6] = {c.t[0], c.t[1], c.t[2], c.t[3], c.t[4], c.t[5]};
        const float got[6] = {r.m00, r.m01, r.m02, r.m10, r.m11, r.m12};
        for (int i = 0; i < 6; ++i)
            check(std::fabs(got[i] - truth[i]) < (i == 2 || i == 5 ? 0.1 : 0.003),
                  "seeded coefficient position, sign or magnitude");
        check(std::memcmp(got, c.seed, sizeof got) != 0, "seeded result equals the seed");
        check(std::isfinite(r.correlation) && r.correlation > 0.9, "seeded correlation");
        opencv_video_ecc_affine_result again = kAffSentinel;
        check(seeded_call(base.get(), moving.get(), c.seed, &again) == OPENCV_VIDEO_OK &&
              std::memcmp(&again, &r, sizeof r) == 0, "seeded repeatability");
        opencv_video_ecc_affine_result plain = kAffSentinel;
        const auto plain_code = aff_call(base.get(), moving.get(), &plain);
        if (c.identity_fails) {
            check(plain_code == OPENCV_VIDEO_ERROR_OPENCV, "identity initialization unexpectedly converged");
            aff_unchanged(plain, "identity failure changed result");
        }
        // Permuting storage order must give a different outcome (six-coefficient ordering).
        float swapped[6] = {c.seed[0], c.seed[3], c.seed[2], c.seed[1], c.seed[4], c.seed[5]};
        opencv_video_ecc_affine_result s = kAffSentinel;
        const auto sc = seeded_call(base.get(), moving.get(), swapped, &s, 1);
        opencv_video_ecc_affine_result o = kAffSentinel;
        const auto oc = seeded_call(base.get(), moving.get(), c.seed, &o, 1);
        check(oc == OPENCV_VIDEO_OK, "one-iteration seeded call");
        check(sc != OPENCV_VIDEO_OK || std::memcmp(&s, &o, sizeof s) != 0, "M01/M10 order not observable");
    }

    // Identity seed equals the identity-initialized export byte for byte, success and failure.
    {
        const Case id_cases[] = {
            {"identity", {1, 0, 0, 0, 1, 0}, {}, false}, {"translation", {1, 0, 2, 0, 1, 1}, {}, false},
            {"small", {1.01, 0.012, 2, -0.008, 0.99, 1}, {}, false}, {"shear02", {1, 0.2, 0, 0, 1, 0}, {}, true},
            {"reflect", {-1, 0, 127, 0, 1, 0}, {}, true}, {"strong", {1.2, 0.1, 5, -0.1, 0.8, -3}, {}, true},
        };
        for (const auto &c : id_cases) {
            auto moving = ecc_handle(aff_moving(n, n, c.t[0], c.t[1], c.t[3], c.t[4], c.t[2], c.t[5]));
            opencv_video_ecc_affine_result a = kAffSentinel, b = kAffSentinel;
            const auto ca = aff_call(base.get(), moving.get(), &a);
            const auto cb = seeded_call(base.get(), moving.get(), identity, &b);
            check(ca == cb, "identity seed status differs");
            check(std::memcmp(&a, &b, sizeof a) == 0, "identity seed result bytes differ");
        }
    }

    // Argument validation and byte-for-byte result preservation.
    {
        const float near_seed[6] = {1.06f, 0.07f, 1.7f, -0.03f, 0.95f, -0.8f};
        auto moving = ecc_handle(aff_moving(n, n, 1.08, 0.08, -0.04, 0.94, 2, -1));
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(seeded_call(nullptr, moving.get(), near_seed, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null template");
        aff_unchanged(r, "null template changed result");
        check(seeded_call(base.get(), nullptr, near_seed, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null moving");
        aff_unchanged(r, "null moving changed result");
        check(seeded_call(base.get(), moving.get(), near_seed, nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "null result accepted");
        for (int iterations : {0, -1, 101})
            check(seeded_call(base.get(), moving.get(), near_seed, &r, iterations) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "bad iterations accepted");
        for (double epsilon : {0.0, -1e-4, 9e-9, 0.1000001, std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()})
            check(seeded_call(base.get(), moving.get(), near_seed, &r, 50, epsilon) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "bad epsilon accepted");
        aff_unchanged(r, "invalid options changed result");
        auto float_image = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
        auto color = matrix(n, n, OPENCV_CORE_DEPTH_UINT8, 3);
        auto narrow = ecc_handle(ecc_fixture::texture(n, n + 1));
        auto tiny = ecc_handle(ecc_fixture::texture(31, 128));
        check(seeded_call(float_image.get(), float_image.get(), near_seed, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "float");
        check(seeded_call(color.get(), color.get(), near_seed, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "color");
        check(seeded_call(base.get(), narrow.get(), near_seed, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "geometry");
        check(seeded_call(tiny.get(), tiny.get(), near_seed, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "tiny");
        aff_unchanged(r, "invalid images changed result");

        // Seed domain: nonfinite and out-of-range coefficients are rejected, bytes untouched.
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float inf = std::numeric_limits<float>::infinity();
        for (int slot = 0; slot < 6; ++slot) {
            const float just_beyond = (slot == 2 || slot == 5) ? 1048577.0f : 4.0000005f;
            for (float bad : {nan, inf, -inf, just_beyond, -just_beyond, std::numeric_limits<float>::max()}) {
                float s[6];
                std::memcpy(s, near_seed, sizeof s);
                s[slot] = bad;
                check(seeded_call(base.get(), moving.get(), s, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                      "invalid seed coefficient accepted");
                aff_unchanged(r, "invalid seed changed result");
            }
        }
        const float at_limits[6] = {4, -4, 1048576.0f, -4, 4, -1048576.0f};
        const auto limit_code = seeded_call(base.get(), moving.get(), at_limits, &r);
        check(limit_code == OPENCV_VIDEO_OK || limit_code == OPENCV_VIDEO_ERROR_OPENCV, "limit seed status");
        if (limit_code != OPENCV_VIDEO_OK) aff_unchanged(r, "limit seed failure changed result");
        // Singular and reflected seeds pass validation; native outcome equals the direct call.
        const float odd[][6] = {{0, 0, 0, 0, 0, 0}, {1, 1, 0, 1, 1, 0}, {-1, 0, 0, 0, 1, 0},
                                {4, 0, 0, 0, 0.01f, 0}, {1, 0, 100000.0f, 0, 1, 100000.0f}};
        for (const auto &s : odd) {
            AffDirect direct{};
            const bool ok = seeded_direct(base_image, output(moving.get()), s, direct);
            opencv_video_ecc_affine_result q = kAffSentinel;
            const auto code = seeded_call(base.get(), moving.get(), s, &q);
            check(code == (ok ? OPENCV_VIDEO_OK : OPENCV_VIDEO_ERROR_OPENCV), "odd seed status differs from direct");
            if (ok) aff_matches_direct(q, direct, "odd seed differs from direct");
            else aff_unchanged(q, "odd seed failure changed result");
        }
        auto flat = ecc_handle(ecc_fixture::constant(n, n, 128));
        opencv_video_ecc_affine_result f = kAffSentinel;
        check(seeded_call(flat.get(), flat.get(), near_seed, &f) == OPENCV_VIDEO_ERROR_OPENCV, "constant seeded");
        aff_unchanged(f, "constant failure changed result");
        const float wrong[6] = {0.8f, -0.2f, 6.0f, 0.2f, 1.25f, -5.0f};
        check(seeded_call(base.get(), moving.get(), wrong, &f) == OPENCV_VIDEO_ERROR_OPENCV, "wrong seed accepted");
        aff_unchanged(f, "wrong seed failure changed result");
        check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after seeded nonconvergence");
    }

    // Same handle twice, shared allocation, and genuine non-contiguous Regions.
    {
        const float s[6] = {1.0f, 0.01f, 1.5f, 0.0f, 1.0f, 0.5f};
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(seeded_call(base.get(), base.get(), s, &r) == OPENCV_VIDEO_OK, "same-handle seeded call");
        AffDirect same{};
        check(seeded_direct(base_image, base_image, s, same), "direct same-image");
        aff_matches_direct(r, same, "same-handle seeded call differs from direct");
        auto alias = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(alias.get()) = output(base.get());
        opencv_video_ecc_affine_result a = kAffSentinel;
        check(seeded_call(base.get(), alias.get(), s, &a) == OPENCV_VIDEO_OK && std::memcmp(&a, &r, sizeof a) == 0,
              "shared-allocation seeded headers");
        const cv::Mat parent_a = ecc_fixture::texture(160, 160);
        const cv::Mat parent_b = aff_moving(160, 160, 1.01, 0.012, -0.008, 0.99, 2, 1);
        const cv::Rect roi(16, 16, n, n);
        auto h_a = ecc_handle(parent_a);
        auto h_b = ecc_handle(parent_b);
        auto region_a = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto region_b = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(region_a.get()) = output(h_a.get())(roi);
        output(region_b.get()) = output(h_b.get())(roi);
        check(!output(region_a.get()).isContinuous() && !output(region_b.get()).isContinuous(),
              "fixture Region is contiguous");
        opencv_video_ecc_affine_result g = kAffSentinel;
        check(seeded_call(region_a.get(), region_b.get(), s, &g) == OPENCV_VIDEO_OK, "seeded Region call failed");
        AffDirect dr{};
        check(seeded_direct(parent_a(roi), parent_b(roi), s, dr), "direct Region");
        aff_matches_direct(g, dr, "seeded Region differs from direct native Region call");
        check(cv::norm(output(h_a.get()), parent_a, cv::NORM_INF) == 0 &&
              cv::norm(output(h_b.get()), parent_b, cv::NORM_INF) == 0, "seeded call changed Region parents");
    }
    check(cv::norm(output(base.get()), before_base, cv::NORM_INF) == 0, "seeded call changed an input");

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        const float s[6] = {1.0f, 0.01f, 1.5f, 0.0f, 1.0f, 0.5f};
        auto moving = ecc_handle(aff_moving(n, n, 1.01, 0.012, -0.008, 0.99, 2, 1));
        // All eight exception injections (before and after native execution).
        for (int stage : {1, 2}) for (int kind : {1, 2, 3, 4}) {
            fault_stage = stage; fault_kind = kind;
            opencv_video_ecc_affine_result r = kAffSentinel;
            const auto code = seeded_call(base.get(), moving.get(), s, &r);
            fault_stage = 0;
            check(code == (kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                           : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD),
                  "seeded ECC affine injected exception mapped to the wrong status");
            aff_unchanged(r, "seeded ECC affine injected failure changed result");
        }
        // Post-native corruption: nonfinite coefficients and correlation are rejected.
        for (int mode : {1, 2, 3, 4, 9, 21, 22, 23, 24, 25, 26}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_affine_result r = kAffSentinel;
            const auto code = seeded_call(base.get(), moving.get(), s, &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "seeded ECC affine nonfinite native output accepted");
            aff_unchanged(r, "seeded ECC affine nonfinite output was partially published");
        }
        // Finite non-rigid outputs remain legitimate (no Euclidean policy on the seeded path).
        for (int mode : {5, 6, 7, 8, 10, 11, 12, 13, 30, 31, 32, 33}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_affine_result r = kAffSentinel;
            const auto code = seeded_call(base.get(), moving.get(), s, &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "seeded ECC affine rejected a finite non-rigid matrix");
            check(std::memcmp(&r, &kAffSentinel, sizeof r) != 0, "seeded ECC affine finite output not published");
        }
    }
#endif
    {
        const float s[6] = {1.0f, 0.01f, 1.5f, 0.0f, 1.0f, 0.5f};
        auto moving = ecc_handle(aff_moving(n, n, 1.01, 0.012, -0.008, 0.99, 2, 1));
        opencv_video_ecc_affine_result r = kAffSentinel;
        check(seeded_call(base.get(), moving.get(), s, &r) == OPENCV_VIDEO_OK, "post-fault seeded affine call failed");
    }
    std::cout << "PASS: ECC affine seeded shim vs direct seeded findTransformECC, six coefficients, identity equivalence, "
                 "seed domain, nonconvergence, Regions, atomicity, faults, corruption\n";
}
}  // namespace

// ECC homography (Task 016): actual shim versus an independent direct
// cv::findTransformECC(MOTION_HOMOGRAPHY) call, with byte-exact 48-byte failure atomicity.
namespace {
static_assert(sizeof(opencv_video_ecc_homography_result) == 48 &&
              alignof(opencv_video_ecc_homography_result) == 8 &&
              offsetof(opencv_video_ecc_homography_result, h00) == 0 &&
              offsetof(opencv_video_ecc_homography_result, h01) == 4 &&
              offsetof(opencv_video_ecc_homography_result, h02) == 8 &&
              offsetof(opencv_video_ecc_homography_result, h10) == 12 &&
              offsetof(opencv_video_ecc_homography_result, h11) == 16 &&
              offsetof(opencv_video_ecc_homography_result, h12) == 20 &&
              offsetof(opencv_video_ecc_homography_result, h20) == 24 &&
              offsetof(opencv_video_ecc_homography_result, h21) == 28 &&
              offsetof(opencv_video_ecc_homography_result, h22) == 32 &&
              offsetof(opencv_video_ecc_homography_result, correlation) == 40,
              "ECC homography result layout");
static_assert(std::is_trivially_copyable<opencv_video_ecc_homography_result>::value,
              "ECC homography result must be trivially copyable");

// The whole 48-byte caller object, including the 4 padding bytes at offset 36, is filled
// with a distinguishable pattern; failure atomicity compares every byte.
opencv_video_ecc_homography_result hom_sentinel() {
    opencv_video_ecc_homography_result r;
    unsigned char *bytes = reinterpret_cast<unsigned char *>(&r);
    for (size_t i = 0; i < sizeof r; ++i) bytes[i] = static_cast<unsigned char>(0xA5 ^ (i * 37));
    return r;
}

void hom_unchanged(const opencv_video_ecc_homography_result &r, const char *message) {
    const opencv_video_ecc_homography_result s = hom_sentinel();
    check(std::memcmp(&r, &s, sizeof r) == 0, message);
}

// Task 017-R1: an alternative caller prefill. Every byte, including padding 36-39, is nonzero
// and differs from hom_sentinel(), so a publication that leaves padding untouched, or copies
// unspecified padding, is observable.
opencv_video_ecc_homography_result hom_pattern(unsigned char seed) {
    opencv_video_ecc_homography_result r;
    unsigned char bytes[sizeof r];
    for (size_t i = 0; i < sizeof r; ++i)
        bytes[i] = static_cast<unsigned char>(((seed + i * 73) % 255) + 1);
    std::memcpy(&r, bytes, sizeof r);
    return r;
}

constexpr size_t kHomPaddingBegin = offsetof(opencv_video_ecc_homography_result, h22) + sizeof(float);
constexpr size_t kHomPaddingEnd = offsetof(opencv_video_ecc_homography_result, correlation);
static_assert(kHomPaddingBegin == 36 && kHomPaddingEnd == 40, "ECC homography padding offsets");

// Reads the object representation through unsigned char (well-defined for any object).
void hom_bytes(const opencv_video_ecc_homography_result &r, unsigned char out[48]) {
    std::memcpy(out, &r, sizeof r);
}

bool hom_padding_is(const opencv_video_ecc_homography_result &r, bool zero) {
    unsigned char b[sizeof r];
    hom_bytes(r, b);
    for (size_t i = kHomPaddingBegin; i < kHomPaddingEnd; ++i)
        if ((b[i] == 0) != zero) return false;
    return true;
}

void hom_padding_zero(const opencv_video_ecc_homography_result &r, const char *message) {
    if (!hom_padding_is(r, true)) {
        unsigned char b[sizeof r];
        hom_bytes(r, b);
        std::cerr << message << ": padding bytes 36-39 =";
        for (size_t i = kHomPaddingBegin; i < kHomPaddingEnd; ++i) std::cerr << ' ' << int(b[i]);
        std::cerr << '\n';
        check(false, "ECC homography success published nonzero padding bytes 36-39");
    }
}

opencv_video_status hom_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                             opencv_video_ecc_homography_result *r, int iterations = 50,
                             double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_homography(a, b, iterations, epsilon, r);
}

struct HomDirect {
    bool ok;
    float w[9];
    double rho;
};

// Independent direct native call: own identity warp, own criteria, no Video shim code.
HomDirect hom_direct(const cv::Mat &a, const cv::Mat &b, int iterations = 50, double epsilon = 1e-4) {
    HomDirect d{};
    cv::Mat warp = cv::Mat::eye(3, 3, CV_32F);
    try {
        d.rho = cv::findTransformECC(a, b, warp, cv::MOTION_HOMOGRAPHY,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, epsilon),
            cv::noArray(), 5);
        d.ok = true;
    } catch (const cv::Exception &) {
        d.ok = false;
    }
    check(warp.rows == 3 && warp.cols == 3 && warp.type() == CV_32FC1 && warp.isContinuous(),
          "direct homography warp schema");
    for (int i = 0; i < 9; ++i) d.w[i] = warp.ptr<float>()[i];
    return d;
}

const float *hom_values(const opencv_video_ecc_homography_result &r, float out[9]) {
    out[0] = r.h00; out[1] = r.h01; out[2] = r.h02;
    out[3] = r.h10; out[4] = r.h11; out[5] = r.h12;
    out[6] = r.h20; out[7] = r.h21; out[8] = r.h22;
    return out;
}

// Bitwise on the same build: same library, same inputs, deterministic CPU path (measured).
void hom_matches_direct(const opencv_video_ecc_homography_result &r, const HomDirect &d,
                        const char *message) {
    check(d.ok, message);
    float got[9];
    hom_values(r, got);
    check(std::memcmp(got, d.w, sizeof got) == 0 &&
          std::memcmp(&r.correlation, &d.rho, sizeof d.rho) == 0, message);
    check(r.h22 == 1.0f && d.w[8] == 1.0f, "ECC homography H22 is not exactly 1");
    hom_padding_zero(r, message);
}

int64_t hom_checksum(const cv::Mat &m) {
    int64_t sum = 0;
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
            sum += int64_t(m.at<uint8_t>(r, c)) * (1 + (int64_t(r) * 977 + int64_t(c) * 131) % 8191);
    return sum;
}

// Mean absolute residual over the interior when the moving image is sampled at H p
// (WARP_INVERSE_MAP semantics: dst(p) = moving(H p)), bilinear, homogeneous division.
double hom_residual(const cv::Mat &tmpl, const cv::Mat &moving, const float *h) {
    double sum = 0;
    int count = 0;
    for (int y = 24; y < tmpl.rows - 24; ++y)
        for (int x = 24; x < tmpl.cols - 24; ++x) {
            const double d = h[6] * x + h[7] * y + h[8];
            const double mx = (h[0] * x + h[1] * y + h[2]) / d;
            const double my = (h[3] * x + h[4] * y + h[5]) / d;
            const int x0 = int(std::floor(mx)), y0 = int(std::floor(my));
            if (x0 < 0 || y0 < 0 || x0 + 1 >= moving.cols || y0 + 1 >= moving.rows) return 1e9;
            const double fx = mx - x0, fy = my - y0;
            auto p = [&](int px, int py) { return double(moving.at<uint8_t>(py, px)); };
            const double v = (1 - fy) * ((1 - fx) * p(x0, y0) + fx * p(x0 + 1, y0)) +
                             fy * ((1 - fx) * p(x0, y0 + 1) + fx * p(x0 + 1, y0 + 1));
            sum += std::fabs(v - tmpl.at<uint8_t>(y, x));
            ++count;
        }
    return sum / count;
}

void run_ecc_homography() {
    const int n = 128;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);
    const cv::Mat before_base = base_image.clone();
    const int64_t base_sum = hom_checksum(base_image);
    const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // Ground truth (template -> moving) and coefficient tolerances: 0.003 linear,
    // 0.1 translation, 5e-5 projective (Task 016 Checkpoint 2, same fixtures).
    struct Case {
        const char *name;
        double h[9];
        bool projective;
    };
    const Case cases[] = {
        {"identity", {1, 0, 0, 0, 1, 0, 0, 0, 1}, false},
        {"translation (2,1)", {1, 0, 2, 0, 1, 1, 0, 0, 1}, false},
        {"rotation +2", {0.99939083, -0.0348995, 0, 0.0348995, 0.99939083, 0, 0, 0, 1}, false},
        {"scale and shear", {1.01, 0.01, 1.5, -0.01, 0.99, -1, 0, 0, 1}, false},
        {"H20 positive", {1, 0, 0, 0, 1, 0, 0.0002, 0, 1}, true},
        {"H20 negative", {1, 0, 0, 0, 1, 0, -0.0002, 0, 1}, true},
        {"H21 positive", {1, 0, 0, 0, 1, 0, 0, 0.0002, 1}, true},
        {"H21 negative", {1, 0, 0, 0, 1, 0, 0, -0.0002, 1}, true},
        {"H20 and H21", {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1}, true},
        {"rotation and perspective", {0.9994, -0.0349, 1.0, 0.0349, 0.9994, 0.5, 0.0001, 0.00005, 1}, true},
        {"scale shear perspective", {1.02, 0.01, 1.0, -0.01, 0.98, -0.5, 0.0001, -0.0001, 1}, true},
    };
    for (const auto &c : cases) {
        const cv::Mat moving_image = (c.name[0] == 'i') ? base_image.clone()
                                                        : ecc_fixture::homography_texture(n, n, c.h);
        auto moving = ecc_handle(moving_image);
        const int64_t moving_sum = hom_checksum(moving_image);
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hom_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, c.name);
        hom_matches_direct(r, hom_direct(base_image, moving_image), c.name);
        float got[9];
        hom_values(r, got);
        for (int i = 0; i < 8; ++i) {
            const double tol = (i == 2 || i == 5) ? 0.1 : (i >= 6 ? 5e-5 : 0.003);
            if (!(std::fabs(got[i] - c.h[i]) < tol)) {
                std::cerr << c.name << " coefficient " << i << " got " << got[i] << " want " << c.h[i] << '\n';
                check(false, "homography coefficient position, sign or magnitude");
            }
        }
        check(got[8] == 1.0f, "homography H22 not exactly 1");
        check(std::isfinite(r.correlation) && r.correlation > 0.98, "homography correlation");
        // Template -> moving under WARP_INVERSE_MAP: residual collapses; the transposed or
        // projective/translation-swapped matrix does not (for nonidentity cases).
        const double good = hom_residual(base_image, moving_image, got);
        check(good < 0.6, "homography WARP_INVERSE_MAP residual did not collapse");
        if (c.projective) {
            float swapped[9];
            std::memcpy(swapped, got, sizeof swapped);
            std::swap(swapped[2], swapped[6]);
            std::swap(swapped[5], swapped[7]);
            check(hom_residual(base_image, moving_image, swapped) > 2.0 * good,
                  "H02/H12 and H20/H21 are indistinguishable");
            const double ident = hom_residual(base_image, moving_image, identity);
            check(ident > 2.0 * good, "projective fixture is already aligned at identity");
        }
        check(hom_checksum(moving_image) == moving_sum &&
              cv::norm(output(moving.get()), moving_image, cv::NORM_INF) == 0, "homography changed moving image");
        opencv_video_ecc_homography_result again = hom_pattern(0x3C);
        check(hom_call(base.get(), moving.get(), &again) == OPENCV_VIDEO_OK &&
              std::memcmp(&again, &r, sizeof r) == 0, "repeated homography call differs (48 bytes)");
    }
    // Affine-equivalent projective matrix (H20 = H21 = 0) agrees with the affine export to
    // within the optimizer's model difference; both see the same affine motion.
    {
        const double h[9] = {1.01, 0.01, 1.5, -0.01, 0.99, -1, 0, 0, 1};
        auto moving = ecc_handle(ecc_fixture::homography_texture(n, n, h));
        opencv_video_ecc_homography_result r = hom_sentinel();
        opencv_video_ecc_affine_result a = kAffSentinel;
        check(hom_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK &&
              aff_call(base.get(), moving.get(), &a) == OPENCV_VIDEO_OK, "affine-equivalent call");
        check(std::fabs(r.h20) < 5e-5 && std::fabs(r.h21) < 5e-5 && r.h22 == 1.0f &&
              std::fabs(r.h00 - a.m00) < 0.003 && std::fabs(r.h01 - a.m01) < 0.003 &&
              std::fabs(r.h02 - a.m02) < 0.1 && std::fabs(r.h10 - a.m10) < 0.003 &&
              std::fabs(r.h11 - a.m11) < 0.003 && std::fabs(r.h12 - a.m12) < 0.1,
              "affine-equivalent homography disagrees with the affine export");
    }

    // Accepted option boundaries equal the direct call with the same options.
    {
        const double h[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        const cv::Mat mv = ecc_fixture::homography_texture(n, n, h);
        auto moving = ecc_handle(mv);
        struct Opt { int it; double eps; } opts[] = {{1, 1e-8}, {1, 0.1}, {100, 1e-8}, {100, 0.1}, {3, 1e-4}};
        for (const auto &o : opts) {
            opencv_video_ecc_homography_result r = hom_sentinel();
            check(hom_call(base.get(), moving.get(), &r, o.it, o.eps) == OPENCV_VIDEO_OK, "option boundary rejected");
            hom_matches_direct(r, hom_direct(base_image, mv, o.it, o.eps), "option boundary differs from direct call");
        }
        opencv_video_ecc_homography_result one = hom_sentinel(), many = hom_sentinel();
        check(hom_call(base.get(), moving.get(), &one, 1, 1e-8) == OPENCV_VIDEO_OK &&
              hom_call(base.get(), moving.get(), &many, 100, 1e-8) == OPENCV_VIDEO_OK, "iteration options");
        check(std::memcmp(&one, &many, 36) != 0, "maximum iterations had no effect");
    }

    // Aliasing, shared allocations, role reversal.
    {
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hom_call(base.get(), base.get(), &r) == OPENCV_VIDEO_OK, "same-handle homography call");
        hom_matches_direct(r, hom_direct(base_image, base_image), "same-handle differs from direct");
        float got[9];
        hom_values(r, got);
        check(std::memcmp(got, identity, sizeof got) == 0 ||
              (std::fabs(got[0] - 1) < 1e-5 && std::fabs(got[4] - 1) < 1e-5 && std::fabs(got[6]) < 1e-7 &&
               std::fabs(got[7]) < 1e-7 && got[8] == 1.0f), "same-handle homography identity");
        check(std::fabs(r.correlation - 1.0) < 1e-6, "same-handle correlation");
        auto alias = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(alias.get()) = output(base.get());
        check(output(alias.get()).data == output(base.get()).data, "alias fixture does not share storage");
        opencv_video_ecc_homography_result a = hom_sentinel();
        check(hom_call(base.get(), alias.get(), &a) == OPENCV_VIDEO_OK &&
              std::memcmp(&a, &r, 36) == 0 && a.correlation == r.correlation, "shared-allocation headers");
        const double h[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        auto moving = ecc_handle(ecc_fixture::homography_texture(n, n, h));
        opencv_video_ecc_homography_result fwd = hom_sentinel(), rev = hom_sentinel();
        check(hom_call(base.get(), moving.get(), &fwd) == OPENCV_VIDEO_OK &&
              hom_call(moving.get(), base.get(), &rev) == OPENCV_VIDEO_OK, "role reversal");
        check(fwd.h20 > 0 && fwd.h21 < 0 && rev.h20 < 0 && rev.h21 > 0, "reversed roles did not invert perspective");
    }

    // Genuine non-contiguous Regions equal the direct call and the contiguous clones.
    {
        const double h[9] = {1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0};
        const cv::Mat parent_a = ecc_fixture::texture(160, 160);
        const cv::Mat parent_b = ecc_fixture::homography_texture(160, 160, h);
        const cv::Mat saved_a = parent_a.clone(), saved_b = parent_b.clone();
        const cv::Rect roi(16, 16, n, n);
        auto h_a = ecc_handle(parent_a);
        auto h_b = ecc_handle(parent_b);
        auto region_a = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto region_b = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(region_a.get()) = output(h_a.get())(roi);
        output(region_b.get()) = output(h_b.get())(roi);
        check(!output(region_a.get()).isContinuous() && !output(region_b.get()).isContinuous(),
              "fixture Region is contiguous");
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hom_call(region_a.get(), region_b.get(), &r) == OPENCV_VIDEO_OK, "homography Region call failed");
        hom_matches_direct(r, hom_direct(parent_a(roi), parent_b(roi)), "Region differs from direct native Region call");
        auto clone_a = ecc_handle(parent_a(roi).clone());
        auto clone_b = ecc_handle(parent_b(roi).clone());
        opencv_video_ecc_homography_result c = hom_sentinel();
        check(hom_call(clone_a.get(), clone_b.get(), &c) == OPENCV_VIDEO_OK &&
              std::memcmp(&c, &r, 36) == 0 && c.correlation == r.correlation,
              "Region and contiguous clone differ");
        check(cv::norm(output(h_a.get()), saved_a, cv::NORM_INF) == 0 &&
              cv::norm(output(h_b.get()), saved_b, cv::NORM_INF) == 0, "homography changed Region parents");
    }

    // Argument validation: every rejected call leaves all 48 bytes unchanged.
    {
        const double h[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        auto moving = ecc_handle(ecc_fixture::homography_texture(n, n, h));
        opencv_video_ecc_homography_result r = hom_sentinel();
        auto rejected = [&](opencv_video_status code, const char *message) {
            check(code == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, message);
            check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after rejected homography call");
            hom_unchanged(r, message);
        };
        rejected(hom_call(nullptr, moving.get(), &r), "null template");
        rejected(hom_call(base.get(), nullptr, &r), "null moving");
        rejected(hom_call(base.get(), moving.get(), nullptr), "null result accepted");
        for (int iterations : {0, -1, 101, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()})
            rejected(hom_call(base.get(), moving.get(), &r, iterations), "bad iterations accepted");
        for (double epsilon : {0.0, -0.0, -1e-4, 9e-9, 9.999999e-9, 0.1000001,
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()})
            rejected(hom_call(base.get(), moving.get(), &r, 50, epsilon), "bad epsilon accepted");
        auto wrong_depth = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
        auto wrong_channels = matrix(n, n, OPENCV_CORE_DEPTH_UINT8, 3);
        auto other_rows = matrix(n + 1, n, OPENCV_CORE_DEPTH_UINT8, 1);
        auto other_cols = matrix(n, n + 1, OPENCV_CORE_DEPTH_UINT8, 1);
        for (auto *bad : {wrong_depth.get(), wrong_channels.get(), other_rows.get(), other_cols.get()}) {
            rejected(hom_call(base.get(), bad, &r), "bad moving accepted");
            rejected(hom_call(bad, base.get(), &r), "bad template accepted");
        }
        auto small = matrix(31, 128, OPENCV_CORE_DEPTH_UINT8, 1);
        auto small_peer = matrix(31, 128, OPENCV_CORE_DEPTH_UINT8, 1);
        rejected(hom_call(small.get(), small_peer.get(), &r), "rows below minimum accepted");
        auto narrow = matrix(128, 31, OPENCV_CORE_DEPTH_UINT8, 1);
        auto narrow_peer = matrix(128, 31, OPENCV_CORE_DEPTH_UINT8, 1);
        rejected(hom_call(narrow.get(), narrow_peer.get(), &r), "columns below minimum accepted");
        auto empty = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(empty.get()) = cv::Mat();
        rejected(hom_call(empty.get(), base.get(), &r), "empty template accepted");
        rejected(hom_call(base.get(), empty.get(), &r), "empty moving accepted");
        auto wide = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto wide_peer = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(wide.get()) = cv::Mat(32, 16385, CV_8UC1, cv::Scalar(0));
        output(wide_peer.get()) = cv::Mat(32, 16385, CV_8UC1, cv::Scalar(0));
        rejected(hom_call(wide.get(), wide_peer.get(), &r), "columns above maximum accepted");
        auto tall = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto tall_peer = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(tall.get()) = cv::Mat(16385, 32, CV_8UC1, cv::Scalar(0));
        output(tall_peer.get()) = cv::Mat(16385, 32, CV_8UC1, cv::Scalar(0));
        rejected(hom_call(tall.get(), tall_peer.get(), &r), "rows above maximum accepted");
        {
            // Area bound: 4097 x 4096 > 2^24 with both sides in range. Real zero-filled
            // storage (about 16 MiB each); released before continuing.
            auto area = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
            auto area_peer = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
            output(area.get()) = cv::Mat(4097, 4096, CV_8UC1, cv::Scalar(0));
            output(area_peer.get()) = cv::Mat(4097, 4096, CV_8UC1, cv::Scalar(0));
            rejected(hom_call(area.get(), area_peer.get(), &r), "above maximum area accepted");
        }
        check(hom_call(base.get(), moving.get(), &r) == OPENCV_VIDEO_OK, "valid call after rejected arguments");
        check(std::strlen(opencv_video_last_error()) == 0, "stale diagnostic after success");
    }

    // Native nonconvergence and native exceptions (the 7 cases raise on 4.1.0, 4.10.0 and 5.0.0):
    // error status, diagnostic, untouched 48 bytes. Wrong-but-finite successes are still published.
    {
        auto expect_failure = [&](const cv::Mat &a, const cv::Mat &b, const char *message) {
            auto ha = ecc_handle(a);
            auto hb = ecc_handle(b);
            check(!hom_direct(a, b).ok, message);
            opencv_video_ecc_homography_result r = hom_sentinel();
            check(hom_call(ha.get(), hb.get(), &r) == OPENCV_VIDEO_ERROR_OPENCV, message);
            check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after native failure");
            hom_unchanged(r, message);
        };
        const double strong[9] = {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1};
        const double near_zero[9] = {1, 0, 0, 0, 1, 0, -0.0078, 0, 1};
        const double crosses[9] = {1, 0, 0, 0, 1, 0, -0.012, 0, 1};
        const double far[9] = {1, 0, 40, 0, 1, 30, 0, 0, 1};
        const double reflect[9] = {-1, 0, 127, 0, 1, 0, 0, 0, 1};
        expect_failure(ecc_fixture::constant(n, n, 128), ecc_fixture::constant(n, n, 128), "constant images accepted");
        expect_failure(ecc_fixture::noise(n, n, 3), ecc_fixture::noise(n, n, 9), "uncorrelated noise accepted");
        expect_failure(base_image, ecc_fixture::homography_texture(n, n, strong), "strong perspective accepted");
        expect_failure(base_image, ecc_fixture::homography_texture(n, n, near_zero), "near-zero denominator accepted");
        expect_failure(base_image, ecc_fixture::homography_texture(n, n, crosses), "denominator crossing zero accepted");
        expect_failure(base_image, ecc_fixture::homography_texture(n, n, far), "large translation accepted");
        expect_failure(base_image, ecc_fixture::homography_texture(n, n, reflect), "reflection accepted");

        const double pb[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        const double ill[9] = {1, 0, 0, 0, 1.0e-6, 0, 0, 0, 1};
        struct Finite { const char *name; cv::Mat a, b; } finite[] = {
            {"low-gradient", ecc_fixture::affine_int(base_image, 1, 40, 126),
             ecc_fixture::affine_int(ecc_fixture::homography_texture(n, n, pb), 1, 40, 126)},
            {"noise-vs-texture", base_image, ecc_fixture::noise(n, n, 7)},
            {"ill-conditioned", base_image, ecc_fixture::homography_texture(n, n, ill)},
        };
        // The outcome is version dependent and follows the same-build direct call: on 4.10.0 all
        // three succeed with a wrong finite matrix; on 4.1.0 and 5.0.0 low-gradient and
        // ill-conditioned raise natively (oracle and probe). Either way the binding must agree with
        // native, and at least one wrong-but-finite success must be published (noise-vs-texture
        // succeeds on every qualified version).
        int finite_successes = 0;
        for (auto &f : finite) {
            auto ha = ecc_handle(f.a);
            auto hb = ecc_handle(f.b);
            const HomDirect d = hom_direct(f.a, f.b);
            opencv_video_ecc_homography_result r = hom_sentinel();
            const opencv_video_status s = hom_call(ha.get(), hb.get(), &r);
            if (d.ok) {
                check(s == OPENCV_VIDEO_OK, f.name);
                hom_matches_direct(r, d, f.name);
                check(r.correlation < 0.5, "wrong-but-finite fixture unexpectedly well correlated");
                ++finite_successes;
            } else {
                check(s == OPENCV_VIDEO_ERROR_OPENCV, f.name);
                check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after native failure");
                hom_unchanged(r, f.name);
            }
        }
        check(finite_successes >= 1, "no wrong-but-finite homography success was observed");
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hom_call(base.get(), base.get(), &r) == OPENCV_VIDEO_OK, "valid call after native failures");
    }
    check(cv::norm(output(base.get()), before_base, cv::NORM_INF) == 0 &&
          hom_checksum(output(base.get())) == base_sum, "homography changed an input");

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        const double h[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        const cv::Mat mv = ecc_fixture::homography_texture(n, n, h);
        auto moving = ecc_handle(mv);
        const int64_t moving_sum = hom_checksum(mv);
        const HomDirect reference = hom_direct(base_image, mv);
        auto inputs_unchanged = [&](const char *message) {
            check(hom_checksum(output(base.get())) == base_sum && hom_checksum(output(moving.get())) == moving_sum,
                  message);
        };
        auto valid_again = [&](const char *message) {
            opencv_video_ecc_homography_result ok = hom_sentinel();
            check(hom_call(base.get(), moving.get(), &ok) == OPENCV_VIDEO_OK, message);
            hom_matches_direct(ok, reference, message);
        };
        int executed = 0;
        // All eight exception injections (before and after native execution).
        for (int stage : {1, 2}) for (int kind : {1, 2, 4, 3}) {
            fault_stage = stage; fault_kind = kind;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hom_call(base.get(), moving.get(), &r);
            fault_stage = 0; fault_kind = 0;
            const auto expected = kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                                : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD;
            check(code == expected, "ECC homography injected exception mapped to the wrong status");
            check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after injected homography exception");
            hom_unchanged(r, "ECC homography injected failure changed result");
            inputs_unchanged("ECC homography injected failure changed an input");
            valid_again("valid homography call after injected exception");
            ++executed;
        }
        check(executed == 8, "not all homography exception injections executed");
        // Invalid arguments are rejected before the native fault hook is reached.
        for (int stage : {1, 2}) {
            fault_stage = stage; fault_kind = 2;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto a = hom_call(base.get(), moving.get(), &r, 0);
            const auto b = hom_call(nullptr, moving.get(), &r);
            const auto c = hom_call(base.get(), moving.get(), nullptr);
            fault_stage = 0; fault_kind = 0;
            check(a == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT && b == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT &&
                  c == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "invalid homography arguments reached the fault hook");
            hom_unchanged(r, "rejected homography arguments changed result");
        }
        // Generic 2x3 ECC corruption modes are not consulted by the homography export.
        for (int mode : {1, 2, 3, 4, 9, 21, 22, 23, 24, 25, 26}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hom_call(base.get(), moving.get(), &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "generic ECC corruption hook reached the homography export");
            hom_matches_direct(r, reference, "generic ECC corruption altered the homography result");
        }
        // Nonfinite or non-unit-H22 post-native output is rejected; all 48 bytes untouched.
        int rejected = 0;
        for (int mode = 1; mode <= 20; ++mode) {
            hom_corrupt_mode = mode;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hom_call(base.get(), moving.get(), &r);
            hom_corrupt_mode = 0;
            if (code != OPENCV_VIDEO_ERROR_OPENCV) std::cerr << "homography corruption mode " << mode << '\n';
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "ECC homography corrupted native output accepted");
            // H22 = NaN/Inf is caught by the finiteness loop; finite non-unit H22 by the H22 check.
            const char *expected_text = (mode >= 12 && mode <= 14) ? "nonfinite correlation"
                : (mode == 10 || mode == 11 || mode >= 18) ? "H22 is not exactly 1"
                : "nonfinite warp entry";
            if (std::strstr(opencv_video_last_error(), expected_text) == nullptr)
                std::cerr << "mode " << mode << " diagnostic: " << opencv_video_last_error() << '\n';
            check(std::strstr(opencv_video_last_error(), expected_text) != nullptr,
                  "homography corruption rejected for the wrong reason");
            hom_unchanged(r, "ECC homography corrupted output was partially published");
            inputs_unchanged("ECC homography corruption changed an input");
            ++rejected;
        }
        check(rejected == 20, "not all homography corruption modes executed");
        // Finite projective matrices with H22 == 1 are legitimate output and published exactly;
        // no affine, determinant or perspective-magnitude policy is applied.
        for (int mode = 30; mode <= 34; ++mode) {
            hom_corrupt_mode = mode;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hom_call(base.get(), moving.get(), &r);
            hom_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "ECC homography rejected an injected finite projective matrix");
            float got[9];
            hom_values(r, got);
            check(std::memcmp(got, kHomInjected[mode - 30], sizeof got) == 0,
                  "ECC homography altered an injected finite matrix");
            check(r.correlation == reference.rho, "injected finite matrix changed the correlation");
            hom_padding_zero(r, "injected finite homography matrix");
            check(std::strlen(opencv_video_last_error()) == 0, "stale diagnostic after accepted matrix");
        }
        // Schema / storage-identity mutations through well-defined cv::Mat operations.
        for (int mode = 1; mode <= 8; ++mode) {
            hom_schema_mode = mode;
            const int applied = hom_schema_applied;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hom_call(base.get(), moving.get(), &r);
            hom_schema_mode = 0;
            hom_schema_keep.release();
            hom_schema_wide.release();
            check(hom_schema_applied == applied + 1, "homography schema hook did not complete");
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "ECC homography accepted a changed native warp schema");
            check(std::strstr(opencv_video_last_error(), "unexpected ECC warp schema") != nullptr,
                  "schema rejection has the wrong diagnostic");
            hom_unchanged(r, "ECC homography schema failure changed result");
        }
        valid_again("valid homography call after corruption campaign");
        std::cout << "PASS: ECC homography faults: 8 exception injections, 20 rejected corruptions, "
                     "5 accepted finite projective matrices, 8 schema mutations, generic hook isolation\n";
    }
#endif
    std::cout << "PASS: ECC homography shim vs direct MOTION_HOMOGRAPHY findTransformECC, nine coefficients, "
                 "H22, layout, sentinel, validation, nonconvergence, aliases, Regions\n";
}
}  // namespace

// ECC homography seeded (Task 017): the new export versus an independent direct seeded
// findTransformECC(MOTION_HOMOGRAPHY) call, seed domain, atomicity, faults and corruption.
namespace {
opencv_video_status hs_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                            const float s[9], opencv_video_ecc_homography_result *r,
                            int iterations = 50, double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_homography_seeded(a, b, iterations, epsilon, s[0], s[1], s[2],
                                                       s[3], s[4], s[5], s[6], s[7], s[8], r);
}

// Direct native call with the nine seed values in native storage order (no shim code).
HomDirect hs_direct(const cv::Mat &a, const cv::Mat &b, const float s[9], int iterations = 50,
                    double epsilon = 1e-4) {
    HomDirect d{};
    cv::Mat warp(3, 3, CV_32F);
    for (int i = 0; i < 9; ++i) warp.ptr<float>()[i] = s[i];
    try {
        d.rho = cv::findTransformECC(a, b, warp, cv::MOTION_HOMOGRAPHY,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, epsilon),
            cv::noArray(), 5);
        d.ok = true;
    } catch (const cv::Exception &) {
        d.ok = false;
    }
    for (int i = 0; i < 9; ++i) d.w[i] = warp.ptr<float>()[i];
    return d;
}

// The binding must agree with the same-build direct call: bitwise on success, an OpenCV
// status with a diagnostic and all 48 bytes untouched on native failure.
void hs_agrees(const opencv_core_mat_handle *ha, const opencv_core_mat_handle *hb, const cv::Mat &a,
               const cv::Mat &b, const float s[9], const char *message, int iterations = 50,
               double epsilon = 1e-4) {
    const HomDirect d = hs_direct(a, b, s, iterations, epsilon);
    opencv_video_ecc_homography_result r = hom_sentinel();
    const auto code = hs_call(ha, hb, s, &r, iterations, epsilon);
    if (d.ok) {
        check(code == OPENCV_VIDEO_OK, message);
        hom_matches_direct(r, d, message);
        check(std::strlen(opencv_video_last_error()) == 0, "stale diagnostic after seeded success");
    } else {
        check(code == OPENCV_VIDEO_ERROR_OPENCV, message);
        check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after seeded native failure");
        hom_unchanged(r, message);
    }
}

void run_ecc_homography_seeded() {
    const int n = 128;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);
    const cv::Mat before_base = base_image.clone();
    const int64_t base_sum = hom_checksum(base_image);
    const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();

    // Identity seed is byte-identical to the identity-initialized export (success and failure).
    {
        const double pb[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        const double strong[9] = {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1};
        const cv::Mat movings[] = {base_image.clone(), ecc_fixture::homography_texture(n, n, pb),
                                   ecc_fixture::homography_texture(n, n, strong),
                                   ecc_fixture::constant(n, n, 128)};
        int successes = 0, failures = 0;
        for (const auto &mv : movings) {
            auto moving = ecc_handle(mv);
            opencv_video_ecc_homography_result plain = hom_sentinel(), seeded = hom_sentinel();
            const auto a = hom_call(base.get(), moving.get(), &plain);
            const auto b = hs_call(base.get(), moving.get(), identity, &seeded);
            check(a == b, "identity seed changed the native outcome");
            check(std::memcmp(&plain, &seeded, sizeof plain) == 0, "identity seed differs from the identity export");
            (a == OPENCV_VIDEO_OK ? successes : failures) += 1;
        }
        check(successes >= 2 && failures >= 2, "identity-equivalence coverage");
    }

    // Near seeds: identity raises, the seeded export converges and matches the direct call.
    struct Case { const char *name; double t[9]; float seed[9]; };
    const Case cases[] = {
        {"strong perspective", {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1}, {1, 0, 0, 0, 1, 0, 0.0018f, 0.0018f, 1}},
        {"large translation", {1, 0, 40, 0, 1, 30, 0, 0, 1}, {1, 0, 38, 0, 1, 28, 0, 0, 1}},
        {"translation and perspective", {1, 0, 12, 0, 1, -9, 0.0005, -0.0004, 1},
         {1, 0, 11, 0, 1, -8, 0.0004f, -0.0003f, 1}},
        {"reflection", {-1, 0, 127, 0, 1, 0, 0, 0, 1}, {-0.98f, 0.01f, 126, 0, 1, 0.1f, 0, 0, 1}},
        {"scale and perspective", {1.15, 0.02, -4, -0.01, 1.12, 3, 0.0008, -0.0006, 1},
         {1.12f, 0.01f, -3, 0, 1.10f, 2.5f, 0.0006f, -0.0005f, 1}},
        {"shear and perspective", {1, 0.15, -3, 0.05, 1, 2, -0.0007, 0.0009, 1},
         {1, 0.13f, -2.5f, 0.04f, 1, 1.6f, -0.0006f, 0.0008f, 1}},
    };
    int residual_checked = 0;
    for (const auto &c : cases) {
        const cv::Mat moving_image = ecc_fixture::homography_texture(n, n, c.t);
        auto moving = ecc_handle(moving_image);
        const int64_t moving_sum = hom_checksum(moving_image);
        opencv_video_ecc_homography_result plain = hom_sentinel();
        check(hom_call(base.get(), moving.get(), &plain) == OPENCV_VIDEO_ERROR_OPENCV,
              "identity initialization unexpectedly converged");
        const HomDirect d = hs_direct(base_image, moving_image, c.seed);
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hs_call(base.get(), moving.get(), c.seed, &r) == OPENCV_VIDEO_OK, c.name);
        hom_matches_direct(r, d, "seeded export differs from the direct seeded native call");
        float got[9];
        hom_values(r, got);
        for (int i = 0; i < 8; ++i) {
            const double tol = (i == 2 || i == 5) ? 0.15 : (i >= 6 ? 5e-5 : 0.006);
            if (!(std::fabs(got[i] - c.t[i]) < tol)) {
                std::cerr << c.name << " coefficient " << i << " got " << got[i] << " want " << c.t[i] << '\n';
                check(false, "seeded homography coefficient position, sign or magnitude");
            }
        }
        check(std::memcmp(got, c.seed, sizeof got) != 0, "seeded result equals the seed");
        check(r.correlation > 0.98, "seeded homography correlation");
        // WARP_INVERSE_MAP direction: the residual collapses relative to the seed. The metric
        // needs the 24-pixel interior to map inside the moving image; the 40/30-pixel shift
        // does not, so it is covered by the coefficients only.
        const double seed_residual = hom_residual(base_image, moving_image, c.seed);
        if (seed_residual < 1e8) {
            check(hom_residual(base_image, moving_image, got) < 0.1 * seed_residual,
                  "seeded refinement did not reduce the image residual");
            ++residual_checked;
        }
        opencv_video_ecc_homography_result again = hom_pattern(0x3C);
        check(hs_call(base.get(), moving.get(), c.seed, &again) == OPENCV_VIDEO_OK &&
              std::memcmp(&again, &r, sizeof r) == 0, "repeated seeded homography call differs (48 bytes)");
        check(hom_checksum(output(moving.get())) == moving_sum, "seeded homography changed moving image");
    }
    check(residual_checked >= 5, "too few residual-checked seeded fixtures");

    // Seed position: transposing or swapping a seed must change the native input (no permutation).
    {
        const double t[9] = {1, 0, 12, 0, 1, -9, 0.0005, -0.0004, 1};
        const cv::Mat mv = ecc_fixture::homography_texture(n, n, t);
        auto moving = ecc_handle(mv);
        const float seed[9] = {1, 0, 11, 0, 1, -8, 0.0004f, -0.0003f, 1};
        const float swapped[9] = {1, 0, -8, 0, 1, 11, 0.0004f, -0.0003f, 1};
        const float permuted[9] = {1, 0, 11, 0, 1, -8, -0.0003f, 0.0004f, 1};
        hs_agrees(base.get(), moving.get(), base_image, mv, seed, "storage-order seed");
        hs_agrees(base.get(), moving.get(), base_image, mv, swapped, "swapped translation seed");
        hs_agrees(base.get(), moving.get(), base_image, mv, permuted, "swapped projective seed");
        opencv_video_ecc_homography_result a = hom_sentinel(), b = hom_sentinel();
        const auto ca = hs_call(base.get(), moving.get(), seed, &a);
        const auto cb = hs_call(base.get(), moving.get(), swapped, &b);
        check(ca == OPENCV_VIDEO_OK && (cb != OPENCV_VIDEO_OK || std::memcmp(&a, &b, 36) != 0),
              "translation seed order has no effect");
    }

    // Option boundaries, domain-edge seeds and wrong seeds follow the same-build direct call.
    {
        const double pb[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        const double strong[9] = {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1};
        const cv::Mat mpb = ecc_fixture::homography_texture(n, n, pb);
        const cv::Mat mst = ecc_fixture::homography_texture(n, n, strong);
        auto hpb = ecc_handle(mpb);
        auto hst = ecc_handle(mst);
        const float near[9] = {1, 0, 0, 0, 1, 0, 0.0018f, 0.0018f, 1};
        struct Opt { int it; double eps; } opts[] = {{1, 1e-8}, {1, 0.1}, {100, 1e-8}, {100, 0.1}, {3, 1e-4}};
        for (const auto &o : opts)
            hs_agrees(base.get(), hst.get(), base_image, mst, near, "seeded option boundary", o.it, o.eps);
        // Exact limits of the domain pass validation; their native outcome is whatever native does.
        const float edges[][9] = {
            {1, 0, 0, 0, 1, 0, 0.5f / 127, 0, 1},
            {1, 0, 0, 0, 1, 0, 0, -0.5f / 127, 1},
            {1, 0, 0, 0, 1, 0, 0.25f / 127, -0.25f / 127, 1},
            {4, -4, 1048576, -4, 4, -1048576, 0, 0, 1},
            {4, 4, 0, 4, 4, 0, 0, 0, 1},
            {0, 0, 0, 0, 0, 0, 0, 0, 1},
            {1e-6f, 0, 0, 0, 1e-6f, 0, 0, 0, 1},
            {4, 0, 0, 0, 0.01f, 0, 0, 0, 1},
            {1, 0, 0, 0, 1, 0, -0.0019f, 0.0019f, 1},
            {0.8f, -0.2f, 6, 0.2f, 1.25f, -5, 0.001f, -0.001f, 1},
        };
        for (const auto &s : edges) {
            hs_agrees(base.get(), hpb.get(), base_image, mpb, s, "domain-edge seed");
            hs_agrees(base.get(), hst.get(), base_image, mst, s, "domain-edge seed (strong)");
        }
        // A wrong-but-finite seeded success is published bitwise (no correlation threshold).
        const float wrong[9] = {1, 0, 0, 0, 1, 0, -0.0019f, 0.0019f, 1};
        const HomDirect d = hs_direct(base_image, mpb, wrong, 1);
        check(d.ok && d.rho < 0.5, "one-iteration wrong seed fixture changed");
        hs_agrees(base.get(), hpb.get(), base_image, mpb, wrong, "wrong-but-finite seeded success", 1);
        // The returned matrix may leave the seed domain; passing it back may be rejected.
        float escaped = std::fabs(d.w[6]) * 127.0f + std::fabs(d.w[7]) * 127.0f;
        check(escaped > 0.5f, "fixture no longer demonstrates an out-of-domain native output");
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hs_call(base.get(), hpb.get(), d.w, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "out-of-domain native output accepted as a seed");
        hom_unchanged(r, "rejected reseed changed result");
    }

    // Seed-domain rejection before any native call (INVALID_ARGUMENT, 48 bytes untouched).
    {
        const double pb[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        auto moving = ecc_handle(ecc_fixture::homography_texture(n, n, pb));
        const float good[9] = {1.0f, 0.01f, 0.5f, -0.01f, 1.0f, -0.5f, 0.0001f, -0.0001f, 1.0f};
        int rejected_count = 0;
        auto rejected = [&](const float s[9], const char *fragment, const char *message) {
            opencv_video_ecc_homography_result r = hom_sentinel();
            check(hs_call(base.get(), moving.get(), s, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, message);
            if (std::strstr(opencv_video_last_error(), fragment) == nullptr)
                std::cerr << message << " diagnostic: " << opencv_video_last_error() << '\n';
            check(std::strstr(opencv_video_last_error(), fragment) != nullptr, "seed rejected for the wrong reason");
            hom_unchanged(r, message);
            ++rejected_count;
        };
        for (int i = 0; i < 9; ++i)
            for (float bad : {nan, inf, -inf}) {
                float s[9];
                std::memcpy(s, good, sizeof s);
                s[i] = bad;
                rejected(s, "nonfinite", "nonfinite seed coefficient accepted");
            }
        for (float h22 : {0.0f, -1.0f, 2.0f, std::nextafter(1.0f, 2.0f), std::nextafter(1.0f, 0.0f), 1e-30f}) {
            float s[9];
            std::memcpy(s, good, sizeof s);
            s[8] = h22;
            rejected(s, "H22", "non-unit H22 seed accepted");
        }
        for (int i : {0, 1, 3, 4})
            for (float bad : {std::nextafter(4.0f, 5.0f), -std::nextafter(4.0f, 5.0f), 1e30f,
                              std::numeric_limits<float>::max()}) {
                float s[9];
                std::memcpy(s, good, sizeof s);
                s[i] = bad;
                rejected(s, "linear", "linear seed coefficient beyond 4 accepted");
            }
        for (int i : {2, 5})
            for (float bad : {std::nextafter(1048576.0f, 2e6f), -std::nextafter(1048576.0f, 2e6f),
                              -std::numeric_limits<float>::max()}) {
                float s[9];
                std::memcpy(s, good, sizeof s);
                s[i] = bad;
                rejected(s, "translation", "translation seed beyond 2^20 accepted");
            }
        // Denominator rule on 128x128: |H20|*127 + |H21|*127 <= 0.5, evaluated in double.
        const float just_over = std::nextafter(0.5f / 127.0f, 1.0f);
        const float den_bad[][2] = {{just_over, 0}, {0, -just_over}, {0.003f, 0.001f}, {-0.0025f, -0.0025f},
                                    {0.01f, 0}, {1e30f, 0}, {0, std::numeric_limits<float>::max()}};
        for (const auto &p : den_bad) {
            float s[9];
            std::memcpy(s, good, sizeof s);
            s[6] = p[0];
            s[7] = p[1];
            rejected(s, "denominator", "denominator deviation above 0.5 accepted");
        }
        check(rejected_count == 27 + 6 + 16 + 6 + 7, "not every seed rejection executed");
        // The rule is geometry dependent: a seed valid at 128x128 is rejected on a 256-wide image.
        {
            const float s[9] = {1, 0, 0, 0, 1, 0, 0.0035f, 0, 1};
            check(std::fabs(double(s[6])) * 127.0 <= 0.5, "geometry fixture is not valid at 128");
            const cv::Mat a = ecc_fixture::texture(n, 256), b = ecc_fixture::texture(n, 256, 2, 1);
            auto ha = ecc_handle(a);
            auto hb = ecc_handle(b);
            opencv_video_ecc_homography_result r = hom_sentinel();
            check(hs_call(ha.get(), hb.get(), s, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "denominator rule ignored the template width");
            hom_unchanged(r, "geometry rejection changed result");
            const float tall[9] = {1, 0, 0, 0, 1, 0, 0, 0.0035f, 1};
            const cv::Mat c = ecc_fixture::texture(256, n), e = ecc_fixture::texture(256, n, 2, 1);
            auto hc = ecc_handle(c);
            auto he = ecc_handle(e);
            check(hs_call(hc.get(), he.get(), tall, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "denominator rule ignored the template height");
            check(hs_call(ha.get(), hb.get(), tall, &r) != OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                  "denominator rule mixed up rows and columns");
        }
        // Image/option validation still precedes the seed (and a bad seed never masks it).
        float bad_seed[9];
        std::memcpy(bad_seed, good, sizeof bad_seed);
        bad_seed[0] = nan;
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hs_call(nullptr, moving.get(), good, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT &&
              hs_call(base.get(), moving.get(), good, nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT &&
              hs_call(base.get(), moving.get(), good, &r, 0) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT &&
              hs_call(base.get(), moving.get(), good, &r, 50, 0.0) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "seeded argument validation");
        check(hs_call(base.get(), moving.get(), bad_seed, &r, 101) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT &&
              std::strstr(opencv_video_last_error(), "Invalid ECC options") != nullptr,
              "option validation must precede seed validation");
        hom_unchanged(r, "rejected seeded arguments changed result");
        // A valid call after rejections.
        check(hs_call(base.get(), moving.get(), good, &r) == OPENCV_VIDEO_OK, "valid call after seed rejections");
    }

    // Nonconvergence, aliases and Regions.
    {
        const float mild[9] = {1, 0.01f, 1, 0, 1, 0.5f, 0.00004f, -0.00003f, 1};
        auto flat = ecc_handle(ecc_fixture::constant(n, n, 128));
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hs_call(flat.get(), flat.get(), mild, &r) == OPENCV_VIDEO_ERROR_OPENCV, "constant images accepted");
        hom_unchanged(r, "constant-image failure changed result");
        auto n1 = ecc_handle(ecc_fixture::noise(n, n, 3));
        auto n2 = ecc_handle(ecc_fixture::noise(n, n, 9));
        check(hs_call(n1.get(), n2.get(), mild, &r) == OPENCV_VIDEO_ERROR_OPENCV, "uncorrelated noise accepted");
        hom_unchanged(r, "noise failure changed result");

        check(hs_call(base.get(), base.get(), mild, &r) == OPENCV_VIDEO_OK, "same-handle seeded call");
        hom_matches_direct(r, hs_direct(base_image, base_image, mild), "same-handle seeded differs from direct");
        auto alias = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(alias.get()) = output(base.get());
        opencv_video_ecc_homography_result a = hom_sentinel();
        check(hs_call(base.get(), alias.get(), mild, &a) == OPENCV_VIDEO_OK &&
              std::memcmp(&a, &r, 36) == 0 && a.correlation == r.correlation, "seeded shared-allocation headers");

        const double h[9] = {1.0, 0.01, 1.0, -0.005, 1.0, 0.5, 0.00005, -0.00004, 1.0};
        const cv::Mat parent_a = ecc_fixture::texture(160, 160);
        const cv::Mat parent_b = ecc_fixture::homography_texture(160, 160, h);
        const cv::Mat saved_a = parent_a.clone(), saved_b = parent_b.clone();
        const cv::Rect roi(16, 16, n, n);
        auto h_a = ecc_handle(parent_a);
        auto h_b = ecc_handle(parent_b);
        auto region_a = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        auto region_b = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(region_a.get()) = output(h_a.get())(roi);
        output(region_b.get()) = output(h_b.get())(roi);
        check(!output(region_a.get()).isContinuous() && !output(region_b.get()).isContinuous(),
              "seeded fixture Region is contiguous");
        opencv_video_ecc_homography_result g = hom_sentinel();
        check(hs_call(region_a.get(), region_b.get(), mild, &g) == OPENCV_VIDEO_OK, "seeded Region call failed");
        hom_matches_direct(g, hs_direct(parent_a(roi), parent_b(roi), mild), "seeded Region differs from direct");
        auto clone_a = ecc_handle(parent_a(roi).clone());
        auto clone_b = ecc_handle(parent_b(roi).clone());
        opencv_video_ecc_homography_result c = hom_sentinel();
        check(hs_call(clone_a.get(), clone_b.get(), mild, &c) == OPENCV_VIDEO_OK &&
              std::memcmp(&c, &g, 36) == 0 && c.correlation == g.correlation, "seeded Region and clone differ");
        check(cv::norm(output(h_a.get()), saved_a, cv::NORM_INF) == 0 &&
              cv::norm(output(h_b.get()), saved_b, cv::NORM_INF) == 0, "seeded homography changed Region parents");
        // The domain uses the Region's geometry (128), not the parent's (160).
        const float region_edge[9] = {1, 0, 0, 0, 1, 0, 0.5f / 127, 0, 1};
        opencv_video_ecc_homography_result e = hom_sentinel();
        check(hs_call(region_a.get(), region_b.get(), region_edge, &e) != OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "seed domain used the Region parent geometry");
    }

#ifdef OPENCV_VIDEO_TEST_FAULTS
    {
        const double t[9] = {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1};
        const float s[9] = {1, 0, 0, 0, 1, 0, 0.0018f, 0.0018f, 1};
        const cv::Mat mv = ecc_fixture::homography_texture(n, n, t);
        auto moving = ecc_handle(mv);
        const int64_t moving_sum = hom_checksum(mv);
        const HomDirect reference = hs_direct(base_image, mv, s);
        check(reference.ok, "seeded fault fixture no longer converges natively");
        auto inputs_unchanged = [&](const char *message) {
            check(hom_checksum(output(base.get())) == base_sum && hom_checksum(output(moving.get())) == moving_sum,
                  message);
        };
        auto valid_again = [&](const char *message) {
            opencv_video_ecc_homography_result ok = hom_sentinel();
            check(hs_call(base.get(), moving.get(), s, &ok) == OPENCV_VIDEO_OK, message);
            hom_matches_direct(ok, reference, message);
        };
        int executed = 0;
        for (int stage : {1, 2}) for (int kind : {1, 2, 4, 3}) {
            fault_stage = stage; fault_kind = kind;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hs_call(base.get(), moving.get(), s, &r);
            fault_stage = 0; fault_kind = 0;
            const auto expected = kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                                : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD;
            check(code == expected, "seeded ECC homography injected exception mapped to the wrong status");
            check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after injected seeded exception");
            hom_unchanged(r, "seeded ECC homography injected failure changed result");
            inputs_unchanged("seeded ECC homography injected failure changed an input");
            valid_again("valid seeded call after injected exception");
            ++executed;
        }
        check(executed == 8, "not all seeded homography exception injections executed");
        // Seed rejection precedes the native fault hook.
        for (int stage : {1, 2}) {
            fault_stage = stage; fault_kind = 2;
            float bad[9];
            std::memcpy(bad, s, sizeof bad);
            bad[8] = 2.0f;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hs_call(base.get(), moving.get(), bad, &r);
            fault_stage = 0; fault_kind = 0;
            check(code == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "invalid seed reached the fault hook");
            hom_unchanged(r, "rejected seed changed result");
        }
        // Generic 2x3 corruption modes are not consulted by the seeded homography export.
        for (int mode : {1, 2, 3, 4, 9, 21, 22, 23, 24, 25, 26}) {
            ecc_corrupt_mode = mode;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hs_call(base.get(), moving.get(), s, &r);
            ecc_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "generic ECC corruption hook reached the seeded homography export");
            hom_matches_direct(r, reference, "generic ECC corruption altered the seeded homography result");
        }
        int rejected = 0;
        for (int mode = 1; mode <= 20; ++mode) {
            hom_corrupt_mode = mode;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hs_call(base.get(), moving.get(), s, &r);
            hom_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "seeded ECC homography corrupted native output accepted");
            const char *expected_text = (mode >= 12 && mode <= 14) ? "nonfinite correlation"
                : (mode == 10 || mode == 11 || mode >= 18) ? "H22 is not exactly 1"
                : "nonfinite warp entry";
            check(std::strstr(opencv_video_last_error(), expected_text) != nullptr,
                  "seeded homography corruption rejected for the wrong reason");
            hom_unchanged(r, "seeded ECC homography corrupted output was partially published");
            inputs_unchanged("seeded ECC homography corruption changed an input");
            ++rejected;
        }
        check(rejected == 20, "not all seeded homography corruption modes executed");
        // Finite projective outputs are published exactly, including ones far outside the seed
        // domain (mode 34): the seed domain is not an output policy.
        for (int mode = 30; mode <= 34; ++mode) {
            hom_corrupt_mode = mode;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hs_call(base.get(), moving.get(), s, &r);
            hom_corrupt_mode = 0;
            check(code == OPENCV_VIDEO_OK, "seeded ECC homography rejected an injected finite projective matrix");
            float got[9];
            hom_values(r, got);
            check(std::memcmp(got, kHomInjected[mode - 30], sizeof got) == 0,
                  "seeded ECC homography altered an injected finite matrix");
            check(r.correlation == reference.rho, "injected finite matrix changed the seeded correlation");
            hom_padding_zero(r, "seeded injected finite homography matrix");
        }
        for (int mode = 1; mode <= 8; ++mode) {
            hom_schema_mode = mode;
            const int applied = hom_schema_applied;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = hs_call(base.get(), moving.get(), s, &r);
            hom_schema_mode = 0;
            hom_schema_keep.release();
            hom_schema_wide.release();
            check(hom_schema_applied == applied + 1, "seeded homography schema hook did not complete");
            check(code == OPENCV_VIDEO_ERROR_OPENCV, "seeded ECC homography accepted a changed native warp schema");
            check(std::strstr(opencv_video_last_error(), "unexpected ECC warp schema") != nullptr,
                  "seeded schema rejection has the wrong diagnostic");
            hom_unchanged(r, "seeded ECC homography schema failure changed result");
        }
        valid_again("valid seeded call after corruption campaign");
        std::cout << "PASS: ECC homography seeded faults: 8 exception injections, 20 rejected corruptions, "
                     "5 accepted finite projective matrices, 8 schema mutations, seed rejection before hooks\n";
    }
#endif
    check(cv::norm(output(base.get()), before_base, cv::NORM_INF) == 0 &&
          hom_checksum(output(base.get())) == base_sum, "seeded homography changed an input");
    std::cout << "PASS: ECC homography seeded shim vs direct seeded findTransformECC, nine coefficients, "
                 "identity equivalence, seed domain, geometry rule, nonconvergence, aliases, Regions\n";
}

// Task 017-R1: deterministic 48-byte object representation for both homography exports.
// Success: every byte is defined (padding 36-39 zero) and independent of the caller's prefill.
// Failure: every caller byte, including padding, is preserved whatever the prefill was.
void run_ecc_homography_padding() {
    const int n = 128;
    const cv::Mat base_image = ecc_fixture::texture(n, n);
    auto base = ecc_handle(base_image);
    const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const opencv_video_ecc_homography_result prefills[] = {hom_sentinel(), hom_pattern(0x11),
                                                          hom_pattern(0xC7)};
    for (const auto &p : prefills)
        check(hom_padding_is(p, false), "prefill padding must be nonzero to observe publication");
    check(std::memcmp(&prefills[0], &prefills[1], 48) != 0 && std::memcmp(&prefills[1], &prefills[2], 48) != 0,
          "prefill patterns must differ");

    // Successful calls: identity export, identity seed, near seed (identity raises there).
    int successes = 0;
    {
        const double pb[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        const double strong[9] = {1, 0, 0, 0, 1, 0, 0.002, 0.002, 1};
        const float near_strong[9] = {1, 0, 0, 0, 1, 0, 0.0018f, 0.0018f, 1};
        const cv::Mat mv_pb = ecc_fixture::homography_texture(n, n, pb);
        const cv::Mat mv_strong = ecc_fixture::homography_texture(n, n, strong);
        auto h_pb = ecc_handle(mv_pb);
        auto h_strong = ecc_handle(mv_strong);
        struct Success {
            const char *name;
            const opencv_core_mat_handle *moving;
            const cv::Mat *moving_image;
            const float *seed;  // nullptr: identity-initialized export
        };
        const Success cases[] = {
            {"identity export, same image", base.get(), &base_image, nullptr},
            {"identity export, perspective", h_pb.get(), &mv_pb, nullptr},
            {"identity seed, same image", base.get(), &base_image, identity},
            {"identity seed, perspective", h_pb.get(), &mv_pb, identity},
            {"near seed, strong perspective", h_strong.get(), &mv_strong, near_strong},
        };
        for (const auto &c : cases) {
            const HomDirect d = c.seed == nullptr ? hom_direct(base_image, *c.moving_image)
                                                  : hs_direct(base_image, *c.moving_image, c.seed);
            unsigned char first[48];
            for (size_t k = 0; k < sizeof prefills / sizeof prefills[0]; ++k) {
                opencv_video_ecc_homography_result r = prefills[k];
                const auto code = c.seed == nullptr ? hom_call(base.get(), c.moving, &r)
                                                    : hs_call(base.get(), c.moving, c.seed, &r);
                check(code == OPENCV_VIDEO_OK, c.name);
                hom_matches_direct(r, d, c.name);  // nine coefficients, correlation, H22, padding
                unsigned char bytes[48];
                hom_bytes(r, bytes);
                for (size_t i = kHomPaddingBegin; i < kHomPaddingEnd; ++i)
                    check(bytes[i] == 0, "ECC homography padding byte not zero");
                if (k == 0)
                    std::memcpy(first, bytes, sizeof first);
                else
                    check(std::memcmp(first, bytes, sizeof first) == 0,
                          "ECC homography 48-byte result depends on the caller's prefill");
            }
            ++successes;
        }
        // Identity seed and the identity-initialized export: identical complete representations,
        // even when the two caller objects start from different prefills.
        for (const auto *mv : {base.get(), h_pb.get()}) {
            opencv_video_ecc_homography_result plain = prefills[1], seeded = prefills[2];
            check(hom_call(base.get(), mv, &plain) == OPENCV_VIDEO_OK &&
                  hs_call(base.get(), mv, identity, &seeded) == OPENCV_VIDEO_OK,
                  "identity equivalence calls");
            check(std::memcmp(&plain, &seeded, sizeof plain) == 0,
                  "identity seed and identity export differ in their 48-byte representation");
        }
    }
    check(successes == 5, "padding success coverage");

    // Failures: every prefill is preserved byte for byte, for both exports.
    int failures = 0;
    {
        const cv::Mat flat_image = ecc_fixture::constant(n, n, 128);
        auto flat = ecc_handle(flat_image);
        auto wrong = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
        float bad_h22[9];
        std::memcpy(bad_h22, identity, sizeof bad_h22);
        bad_h22[8] = 2.0f;
        float nan_seed[9];
        std::memcpy(nan_seed, identity, sizeof nan_seed);
        nan_seed[4] = std::numeric_limits<float>::quiet_NaN();
        float den_seed[9];
        std::memcpy(den_seed, identity, sizeof den_seed);
        den_seed[6] = 0.01f;
        for (const auto &p : prefills) {
            auto expect_kept = [&](opencv_video_status code, opencv_video_status expected,
                                   const opencv_video_ecc_homography_result &r, const char *message) {
                check(code == expected, message);
                check(std::memcmp(&r, &p, sizeof r) == 0, message);
                ++failures;
            };
            opencv_video_ecc_homography_result r = p;
            // Input validation.
            expect_kept(hom_call(nullptr, base.get(), &r), OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, r,
                        "null image changed the prefill");
            expect_kept(hom_call(base.get(), wrong.get(), &r), OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, r,
                        "wrong depth changed the prefill");
            expect_kept(hom_call(base.get(), base.get(), &r, 0), OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, r,
                        "invalid iterations changed the prefill");
            expect_kept(hs_call(base.get(), base.get(), identity, &r, 50, 0.0),
                        OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, r, "invalid epsilon changed the prefill");
            // Seed validation.
            expect_kept(hs_call(base.get(), base.get(), bad_h22, &r), OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, r,
                        "non-unit H22 seed changed the prefill");
            expect_kept(hs_call(base.get(), base.get(), nan_seed, &r), OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, r,
                        "nonfinite seed changed the prefill");
            expect_kept(hs_call(base.get(), base.get(), den_seed, &r), OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, r,
                        "denominator seed changed the prefill");
            // Native OpenCV raises.
            expect_kept(hom_call(flat.get(), flat.get(), &r), OPENCV_VIDEO_ERROR_OPENCV, r,
                        "native failure changed the identity-export prefill");
            expect_kept(hs_call(flat.get(), flat.get(), identity, &r), OPENCV_VIDEO_ERROR_OPENCV, r,
                        "native failure changed the seeded prefill");
#ifdef OPENCV_VIDEO_TEST_FAULTS
            // Injected exceptions, rejected coefficients/correlation, schema mutations: both exports.
            for (int which = 0; which < 2; ++which) {
                auto call = [&](opencv_video_ecc_homography_result *out) {
                    return which == 0 ? hom_call(base.get(), base.get(), out)
                                      : hs_call(base.get(), base.get(), identity, out);
                };
                for (int stage : {1, 2}) for (int kind : {1, 2, 4, 3}) {
                    fault_stage = stage; fault_kind = kind;
                    const auto code = call(&r);
                    fault_stage = 0; fault_kind = 0;
                    const auto expected = kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                                        : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD;
                    expect_kept(code, expected, r, "injected exception changed the prefill");
                }
                for (int mode = 1; mode <= 20; ++mode) {
                    hom_corrupt_mode = mode;
                    const auto code = call(&r);
                    hom_corrupt_mode = 0;
                    expect_kept(code, OPENCV_VIDEO_ERROR_OPENCV, r, "rejected corruption changed the prefill");
                }
                for (int mode = 1; mode <= 8; ++mode) {
                    hom_schema_mode = mode;
                    const auto code = call(&r);
                    hom_schema_mode = 0;
                    hom_schema_keep.release();
                    hom_schema_wide.release();
                    expect_kept(code, OPENCV_VIDEO_ERROR_OPENCV, r, "schema failure changed the prefill");
                }
                // Accepted finite injected matrices are published with zero padding.
                for (int mode = 30; mode <= 34; ++mode) {
                    opencv_video_ecc_homography_result ok = p;
                    hom_corrupt_mode = mode;
                    const auto code = call(&ok);
                    hom_corrupt_mode = 0;
                    check(code == OPENCV_VIDEO_OK, "injected finite matrix rejected");
                    hom_padding_zero(ok, "injected finite matrix padding");
                }
            }
#endif
        }
    }
#ifdef OPENCV_VIDEO_TEST_FAULTS
    const int expected_failures = 3 * (9 + 2 * (8 + 20 + 8));
#else
    const int expected_failures = 3 * 9;
#endif
    check(failures == expected_failures, "padding failure coverage");
    std::cout << "PASS: ECC homography 48-byte representation: " << successes
              << " success cases x 3 prefills with zero padding 36-39 and prefill-independent bytes, "
                 "identity seed == identity export (48 bytes), "
              << failures << " failures preserving every prefill byte\n";
}
}  // namespace


// ECC homography masked (Task 018): both masked exports versus independent direct
// findTransformECC calls that pass the moving-image mask themselves; mask schema, support,
// coordinate frame, Regions, aliasing, immutability, atomicity, padding, faults and corruption.
namespace {
opencv_video_status hm_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                            const opencv_core_mat_handle *m, opencv_video_ecc_homography_result *r,
                            int iterations = 50, double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_homography_masked(a, b, m, iterations, epsilon, r);
}

opencv_video_status hms_call(const opencv_core_mat_handle *a, const opencv_core_mat_handle *b,
                             const opencv_core_mat_handle *m, const float s[9],
                             opencv_video_ecc_homography_result *r, int iterations = 50,
                             double epsilon = 1e-4) {
    return opencv_video_estimate_ecc_homography_masked_seeded(a, b, m, iterations, epsilon, s[0], s[1],
                                                              s[2], s[3], s[4], s[5], s[6], s[7], s[8], r);
}

// Direct native call with its own warp and the caller's mask (no shim code). `mask` empty means
// cv::noArray() so the unmasked direct call is the same helper.
HomDirect hm_direct(const cv::Mat &a, const cv::Mat &b, const cv::Mat &mask, const float s[9],
                    int iterations = 50, double epsilon = 1e-4) {
    HomDirect d{};
    cv::Mat warp(3, 3, CV_32F);
    for (int i = 0; i < 9; ++i) warp.ptr<float>()[i] = s[i];
    try {
        const cv::TermCriteria tc(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, iterations, epsilon);
        d.rho = mask.empty() ? cv::findTransformECC(a, b, warp, cv::MOTION_HOMOGRAPHY, tc, cv::noArray(), 5)
                             : cv::findTransformECC(a, b, warp, cv::MOTION_HOMOGRAPHY, tc, mask, 5);
        d.ok = true;
    } catch (const cv::Exception &) {
        d.ok = false;
    }
    for (int i = 0; i < 9; ++i) d.w[i] = warp.ptr<float>()[i];
    return d;
}

bool hm_same_direct(const HomDirect &x, const HomDirect &y) {
    return x.ok == y.ok && (!x.ok || (std::memcmp(x.w, y.w, sizeof x.w) == 0 &&
                                      std::memcmp(&x.rho, &y.rho, sizeof x.rho) == 0));
}

cv::Mat hm_mask_hole(int n, cv::Rect hole, unsigned char value = 255) {
    cv::Mat m(n, n, CV_8UC1, cv::Scalar(value));
    m(hole & cv::Rect(0, 0, n, n)).setTo(0);
    return m;
}

// Agreement of one export (seed == nullptr: identity-initialized masked export) with the
// direct call using the same mask: bitwise on success with zero padding, OpenCV status and an
// untouched caller object on native failure. Returns whether native succeeded.
bool hm_agrees(const opencv_core_mat_handle *ha, const opencv_core_mat_handle *hb,
               const opencv_core_mat_handle *hm, const cv::Mat &a, const cv::Mat &b, const cv::Mat &m,
               const float *seed, const char *message, int iterations = 50, double epsilon = 1e-4) {
    static const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const HomDirect d = hm_direct(a, b, m, seed ? seed : identity, iterations, epsilon);
    opencv_video_ecc_homography_result r = hom_sentinel();
    const auto code = seed ? hms_call(ha, hb, hm, seed, &r, iterations, epsilon)
                           : hm_call(ha, hb, hm, &r, iterations, epsilon);
    if (d.ok) {
        check(code == OPENCV_VIDEO_OK, message);
        hom_matches_direct(r, d, message);
        check(std::strlen(opencv_video_last_error()) == 0, "stale diagnostic after masked success");
    } else {
        check(code == OPENCV_VIDEO_ERROR_OPENCV, message);
        check(std::strlen(opencv_video_last_error()) > 0, "no diagnostic after masked native failure");
        hom_unchanged(r, message);
    }
    return d.ok;
}

void hm_rejected(opencv_video_status code, const opencv_video_ecc_homography_result &r,
                 const char *fragment, const char *message) {
    check(code == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, message);
    check(std::strstr(opencv_video_last_error(), fragment) != nullptr, message);
    hom_unchanged(r, message);
}

void run_ecc_homography_masked() {
    const int n = 128;
    const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const double t21[9] = {1, 0, 2, 0, 1, 1, 0, 0, 1};
    const double ltp[9] = {1, 0, 12, 0, 1, -9, 0.0005, -0.0004, 1};
    const float ltp_near[9] = {1, 0, 11, 0, 1, -8, 0.0004f, -0.0003f, 1};
    const cv::Mat a_image = ecc_fixture::texture(n, n);
    const cv::Mat t21_image = ecc_fixture::homography_texture(n, n, t21);
    const cv::Mat ltp_image = ecc_fixture::homography_texture(n, n, ltp);
    auto a = ecc_handle(a_image);
    auto b21 = ecc_handle(t21_image);
    auto bltp = ecc_handle(ltp_image);
    const cv::Rect hole_rect(70, 10, 40, 30);  // asymmetric, moving pixels
    const cv::Mat hole_image = hm_mask_hole(n, hole_rect);
    auto hole = ecc_handle(hole_image);
    const int64_t hole_sum = hom_checksum(hole_image);
    int agreements = 0;

    // 1. Accepted masks reach the optimizer: agreement with the direct masked call, and the
    //    result differs from the unmasked export (mask consumed), identity and seeded.
    {
        check(hm_agrees(a.get(), b21.get(), hole.get(), a_image, t21_image, hole_image, nullptr,
                        "masked translation"), "masked translation must converge");
        check(hm_agrees(a.get(), bltp.get(), hole.get(), a_image, ltp_image, hole_image, ltp_near,
                        "masked seeded translation+perspective"), "masked seeded must converge");
        agreements += 2;
        opencv_video_ecc_homography_result plain = hom_sentinel(), masked = hom_sentinel();
        check(hom_call(a.get(), b21.get(), &plain) == OPENCV_VIDEO_OK &&
              hm_call(a.get(), b21.get(), hole.get(), &masked) == OPENCV_VIDEO_OK, "mask consumption calls");
        check(std::memcmp(&plain, &masked, sizeof plain) != 0, "accepted mask did not reach the optimizer");
        opencv_video_ecc_homography_result splain = hom_sentinel(), smasked = hom_sentinel();
        check(hs_call(a.get(), bltp.get(), ltp_near, &splain) == OPENCV_VIDEO_OK &&
              hms_call(a.get(), bltp.get(), hole.get(), ltp_near, &smasked) == OPENCV_VIDEO_OK,
              "seeded mask consumption calls");
        check(std::memcmp(&splain, &smasked, sizeof splain) != 0, "seeded mask did not reach the optimizer");
        // The identity seed through the masked seeded export equals the masked identity export.
        opencv_video_ecc_homography_result via_seed = hom_sentinel();
        check(hms_call(a.get(), b21.get(), hole.get(), identity, &via_seed) == OPENCV_VIDEO_OK &&
              std::memcmp(&via_seed, &masked, sizeof masked) == 0, "masked identity seed differs");
    }

    // 2. Full masks (1, 7, 255) are byte-identical to the unmasked exports on both outcomes.
    {
        const double pb[9] = {1, 0, 0, 0, 1, 0, 0.0002, -0.0002, 1};
        const cv::Mat movings[] = {a_image.clone(), t21_image, ecc_fixture::homography_texture(n, n, pb),
                                   ecc_fixture::constant(n, n, 128)};
        for (int v : {1, 7, 255}) {
            auto full = ecc_handle(cv::Mat(n, n, CV_8UC1, cv::Scalar(v)));
            int ok = 0, failed = 0;
            for (const auto &mv : movings) {
                auto moving = ecc_handle(mv);
                opencv_video_ecc_homography_result p = hom_sentinel(), m = hom_pattern(0x31);
                const auto pc = hom_call(a.get(), moving.get(), &p);
                const auto mc = hm_call(a.get(), moving.get(), full.get(), &m);
                check(pc == mc, "full mask changed the outcome");
                if (pc == OPENCV_VIDEO_OK) {
                    check(std::memcmp(&p, &m, sizeof p) == 0, "full mask differs from the unmasked export");
                    ++ok;
                } else {
                    hom_unchanged(p, "unmasked failure");
                    check(std::memcmp(&m, &(const opencv_video_ecc_homography_result &)hom_pattern(0x31), 48) == 0,
                          "full-mask failure changed the prefill");
                    ++failed;
                }
                opencv_video_ecc_homography_result sp = hom_sentinel(), sm = hom_sentinel();
                check(hs_call(a.get(), moving.get(), ltp_near, &sp) ==
                          hms_call(a.get(), moving.get(), full.get(), ltp_near, &sm) &&
                      std::memcmp(&sp, &sm, sizeof sp) == 0, "seeded full mask differs from the seeded export");
            }
            check(ok >= 2 && failed >= 1, "full-mask equivalence coverage");
        }
    }

    // 3. Value semantics through the shim: 1, 255 and arbitrary nonzero are byte-identical.
    {
        // Independent direct-level facts the shim relies on (no shim code involved).
        const cv::Mat full(n, n, CV_8UC1, cv::Scalar(255));
        check(hm_same_direct(hm_direct(a_image, t21_image, full, identity),
                             hm_direct(a_image, t21_image, cv::Mat(), identity)),
              "direct: full mask differs from noArray");
        check(!hm_same_direct(hm_direct(a_image, t21_image, hole_image, identity),
                              hm_direct(a_image, t21_image, cv::Mat(), identity)),
              "direct: hole mask has no effect");
        cv::Mat arbitrary = hole_image.clone();
        uint32_t s = 5;
        for (int r = 0; r < n; ++r)
            for (int c = 0; c < n; ++c) {
                s = s * 1664525u + 1013904223u;
                if (arbitrary.at<uint8_t>(r, c)) arbitrary.at<uint8_t>(r, c) = uint8_t(1 + (s >> 24) % 255);
            }
        auto ones = ecc_handle(hm_mask_hole(n, hole_rect, 1));
        auto arb = ecc_handle(arbitrary);
        opencv_video_ecc_homography_result r255 = hom_sentinel(), r1 = hom_pattern(3), ra = hom_pattern(9);
        check(hm_call(a.get(), b21.get(), hole.get(), &r255) == OPENCV_VIDEO_OK &&
              hm_call(a.get(), b21.get(), ones.get(), &r1) == OPENCV_VIDEO_OK &&
              hm_call(a.get(), b21.get(), arb.get(), &ra) == OPENCV_VIDEO_OK, "value semantics calls");
        check(std::memcmp(&r255, &r1, 48) == 0 && std::memcmp(&r255, &ra, 48) == 0,
              "mask values 1, 255 and arbitrary nonzero must be byte-identical");
    }

    // 4. Coordinate frame: the transposed hole and a hole in template coordinates are different
    //    masks (asymmetric fixtures); each still agrees with its own direct call.
    {
        const cv::Mat swapped = hm_mask_hole(n, cv::Rect(hole_rect.y, hole_rect.x, hole_rect.height, hole_rect.width));
        auto swapped_h = ecc_handle(swapped);
        check(hm_agrees(a.get(), b21.get(), swapped_h.get(), a_image, t21_image, swapped, nullptr,
                        "transposed hole"), "transposed hole converges");
        ++agreements;
        opencv_video_ecc_homography_result x = hom_sentinel(), y = hom_sentinel();
        hm_call(a.get(), b21.get(), hole.get(), &x);
        hm_call(a.get(), b21.get(), swapped_h.get(), &y);
        check(std::memcmp(&x, &y, 48) != 0, "transposed mask indistinguishable (rows/columns swapped?)");
        // Occluder in the moving image: masking it in MOVING coordinates recovers the motion.
        const double t_ltp[9] = {1, 0, 12, 0, 1, -9, 0.0005, -0.0004, 1};
        cv::Mat occluded = ecc_fixture::homography_texture(n, n, t_ltp);
        occluded(cv::Rect(60, 4, 64, 52)).setTo(250);
        auto occ = ecc_handle(occluded);
        const cv::Mat moving_frame = hm_mask_hole(n, cv::Rect(57, 1, 70, 58));
        const cv::Mat template_frame = hm_mask_hole(n, cv::Rect(45, 10, 70, 58));
        auto mf = ecc_handle(moving_frame);
        auto tf = ecc_handle(template_frame);
        opencv_video_ecc_homography_result unmasked = hom_sentinel(), right = hom_sentinel(), wrong = hom_sentinel();
        check(hs_call(a.get(), occ.get(), ltp_near, &unmasked) == OPENCV_VIDEO_ERROR_OPENCV,
              "occluded seeded unmasked call must raise natively");
        hom_unchanged(unmasked, "occluded native failure changed the result");
        check(hms_call(a.get(), occ.get(), mf.get(), ltp_near, &right) == OPENCV_VIDEO_OK,
              "moving-frame occlusion mask must converge");
        float h[9];
        hom_values(right, h);
        check(hom_residual(a_image, ecc_fixture::homography_texture(n, n, t_ltp), h) < 1.0,
              "moving-frame occlusion mask must recover the homography");
        check(hm_agrees(a.get(), occ.get(), mf.get(), a_image, occluded, moving_frame, ltp_near,
                        "moving-frame occlusion"), "moving-frame occlusion agreement");
        const bool wrong_ok = hm_agrees(a.get(), occ.get(), tf.get(), a_image, occluded, template_frame,
                                        ltp_near, "template-frame occlusion");
        agreements += 2;
        if (wrong_ok) {
            hms_call(a.get(), occ.get(), tf.get(), ltp_near, &wrong);
            float g[9];
            hom_values(wrong, g);
            check(hom_residual(a_image, ecc_fixture::homography_texture(n, n, t_ltp), g) > 2.0,
                  "template-frame mask behaved like the moving-frame mask");
        }
    }

    // 5. Mask schema, geometry and support: INVALID_ARGUMENT before native, diagnostics, untouched.
    {
        auto f32 = matrix(n, n, OPENCV_CORE_DEPTH_FLOAT32, 1);
        output(f32.get()).setTo(1.0f);
        auto s8 = matrix(n, n, OPENCV_CORE_DEPTH_INT8, 1);
        output(s8.get()).setTo(1);
        auto u16 = matrix(n, n, OPENCV_CORE_DEPTH_UINT16, 1);
        output(u16.get()).setTo(1);
        auto c3 = matrix(n, n, OPENCV_CORE_DEPTH_UINT8, 3);
        output(c3.get()).setTo(cv::Scalar(255, 255, 255));
        auto c2 = matrix(n, n, OPENCV_CORE_DEPTH_UINT8, 2);
        output(c2.get()).setTo(cv::Scalar(255, 255));
        auto tall = matrix(n + 1, n, OPENCV_CORE_DEPTH_UINT8, 1);
        output(tall.get()).setTo(255);
        auto wide = matrix(n, n + 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(wide.get()).setTo(255);
        auto shorter = matrix(n - 1, n, OPENCV_CORE_DEPTH_UINT8, 1);
        output(shorter.get()).setTo(255);
        auto small = matrix(64, 64, OPENCV_CORE_DEPTH_UINT8, 1);
        output(small.get()).setTo(255);
        auto empty = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(empty.get()).release();
        auto zero = ecc_handle(cv::Mat(n, n, CV_8UC1, cv::Scalar(0)));
        auto three_d = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        { const int sizes[3] = {n, n, 1}; output(three_d.get()) = cv::Mat(3, sizes, CV_8UC1, cv::Scalar(255)); }
        struct Bad { const opencv_core_mat_handle *mask; const char *fragment; const char *name; };
        const Bad bad[] = {
            {empty.get(), "nonempty 2-D UInt8 C1", "empty mask"},
            {f32.get(), "nonempty 2-D UInt8 C1", "Float32 mask"},
            {s8.get(), "nonempty 2-D UInt8 C1", "Int8 mask"},
            {u16.get(), "nonempty 2-D UInt8 C1", "UInt16 mask"},
            {c2.get(), "nonempty 2-D UInt8 C1", "two-channel mask"},
            {c3.get(), "nonempty 2-D UInt8 C1", "three-channel mask"},
            {three_d.get(), "nonempty 2-D UInt8 C1", "3-D mask"},
            {tall.get(), "geometry differs", "rows+1 mask"},
            {wide.get(), "geometry differs", "columns+1 mask"},
            {shorter.get(), "geometry differs", "rows-1 mask"},
            {small.get(), "geometry differs", "64x64 mask"},
            {zero.get(), "excludes every pixel", "all-zero mask"},
        };
        int rejected = 0;
        for (const auto &x : bad) {
            opencv_video_ecc_homography_result r = hom_sentinel();
            hm_rejected(hm_call(a.get(), b21.get(), x.mask, &r), r, x.fragment, x.name);
            r = hom_sentinel();
            hm_rejected(hms_call(a.get(), b21.get(), x.mask, ltp_near, &r), r, x.fragment, x.name);
            rejected += 2;
        }
        // Null mask handle: invalid argument (resolve), untouched.
        opencv_video_ecc_homography_result r = hom_sentinel();
        check(hm_call(a.get(), b21.get(), nullptr, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "null mask");
        hom_unchanged(r, "null mask changed the result");
        check(hms_call(a.get(), b21.get(), nullptr, ltp_near, &r) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "null seeded mask");
        hom_unchanged(r, "null seeded mask changed the result");
        // Null result.
        check(hm_call(a.get(), b21.get(), hole.get(), nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT &&
              hms_call(a.get(), b21.get(), hole.get(), ltp_near, nullptr) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
              "null result accepted");
        // Ordering: images/options before mask; mask before seed. A mask equal to the moving image's
        // geometry but differing from a mismatched template is reported as an image error.
        auto t64 = ecc_handle(ecc_fixture::texture(64, 64));
        r = hom_sentinel();
        hm_rejected(hm_call(a.get(), t64.get(), hole.get(), &r), r, "ECC images", "image validation first");
        r = hom_sentinel();
        hm_rejected(hm_call(a.get(), b21.get(), empty.get(), &r, 0), r, "Invalid ECC options", "options before mask");
        float bad_seed[9];
        std::memcpy(bad_seed, identity, sizeof bad_seed);
        bad_seed[8] = 2.0f;
        r = hom_sentinel();
        hm_rejected(hms_call(a.get(), b21.get(), empty.get(), bad_seed, &r), r, "moving mask", "mask before seed");
        r = hom_sentinel();
        hm_rejected(hms_call(a.get(), b21.get(), hole.get(), bad_seed, &r), r, "H22", "seed after mask");
        float den_seed[9];
        std::memcpy(den_seed, identity, sizeof den_seed);
        den_seed[6] = 0.01f;
        r = hom_sentinel();
        hm_rejected(hms_call(a.get(), b21.get(), hole.get(), den_seed, &r), r, "denominator", "seed geometry rule");
        check(rejected == 24, "mask rejection coverage");
    }

    // 6. Support that native erosion removes, and non-overlap after warping: the binding forwards
    //    native nonconvergence (no fabricated result); sparse surviving support agrees bitwise.
    {
        cv::Mat single(n, n, CV_8UC1, cv::Scalar(0));
        single.at<uint8_t>(64, 64) = 255;
        cv::Mat strip4(n, n, CV_8UC1, cv::Scalar(0));
        strip4(cv::Rect(0, 60, n, 4)).setTo(255);
        cv::Mat strip5(n, n, CV_8UC1, cv::Scalar(0));
        strip5(cv::Rect(0, 60, n, 5)).setTo(255);
        cv::Mat block12(n, n, CV_8UC1, cv::Scalar(0));
        block12(cv::Rect(50, 40, 12, 12)).setTo(255);
        cv::Mat left(n, n, CV_8UC1, cv::Scalar(0));
        left(cv::Rect(0, 0, 24, n)).setTo(255);
        const float shift60[9] = {1, 0, 60, 0, 1, 0, 0, 0, 1};
        int native_failures = 0, native_successes = 0;
        for (const cv::Mat *m : {&single, &strip4, &strip5, &block12, &left}) {
            auto h = ecc_handle(*m);
            (hm_agrees(a.get(), b21.get(), h.get(), a_image, t21_image, *m, nullptr, "sparse mask")
                 ? native_successes : native_failures) += 1;
            ++agreements;
        }
        auto lh = ecc_handle(left);
        for (int it : {1, 50}) {
            check(!hm_agrees(a.get(), b21.get(), lh.get(), a_image, t21_image, left, shift60,
                             "non-overlapping valid region", it), "non-overlap must raise natively");
            ++native_failures;
            ++agreements;
        }
        check(std::strstr(opencv_video_last_error(), "NaN") != nullptr, "non-overlap diagnostic");
        check(native_failures >= 4 && native_successes >= 2, "support coverage");
    }

    // 7. Regions, aliasing, immutability, options.
    {
        auto parent = matrix(n + 32, n + 32, OPENCV_CORE_DEPTH_UINT8, 1);
        output(parent.get()).setTo(0);
        hole_image.copyTo(output(parent.get())(cv::Rect(16, 16, n, n)));
        auto roi = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(roi.get()) = output(parent.get())(cv::Rect(16, 16, n, n));
        check(!output(roi.get()).isContinuous(), "mask Region fixture is contiguous");
        const cv::Mat parent_saved = output(parent.get()).clone();
        opencv_video_ecc_homography_result compact = hom_sentinel(), region = hom_pattern(0x44);
        check(hm_call(a.get(), b21.get(), hole.get(), &compact) == OPENCV_VIDEO_OK &&
              hm_call(a.get(), b21.get(), roi.get(), &region) == OPENCV_VIDEO_OK, "Region mask calls");
        check(std::memcmp(&compact, &region, 48) == 0, "non-contiguous Region mask differs from compact");
        opencv_video_ecc_homography_result sc = hom_sentinel(), sr = hom_sentinel();
        check(hms_call(a.get(), bltp.get(), hole.get(), ltp_near, &sc) == OPENCV_VIDEO_OK &&
              hms_call(a.get(), bltp.get(), roi.get(), ltp_near, &sr) == OPENCV_VIDEO_OK &&
              std::memcmp(&sc, &sr, 48) == 0, "seeded Region mask differs from compact");
        check(cv::norm(parent_saved, output(parent.get()), cv::NORM_INF) == 0, "mask Region parent mutated");
        // Region of the wrong size from the same parent is rejected.
        auto short_roi = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(short_roi.get()) = output(parent.get())(cv::Rect(16, 16, n, n - 1));
        opencv_video_ecc_homography_result r = hom_sentinel();
        hm_rejected(hm_call(a.get(), b21.get(), short_roi.get(), &r), r, "geometry differs", "short Region mask");
        // All three inputs as Regions agree with the direct call on the same Regions.
        auto ap = ecc_handle(ecc_fixture::texture(n + 32, n + 32));
        auto bp = ecc_handle(ecc_fixture::homography_texture(n + 32, n + 32, t21));
        auto ar = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1), br = matrix(1, 1, OPENCV_CORE_DEPTH_UINT8, 1);
        output(ar.get()) = output(ap.get())(cv::Rect(16, 16, n, n));
        output(br.get()) = output(bp.get())(cv::Rect(16, 16, n, n));
        check(hm_agrees(ar.get(), br.get(), roi.get(), output(ar.get()), output(br.get()), output(roi.get()),
                        nullptr, "all-Region masked call"), "all-Region masked call converges");
        ++agreements;
        // Aliasing: one handle as template, moving AND mask (texture has no zero pixel).
        check(cv::countNonZero(a_image) == n * n, "aliasing fixture has zero pixels");
        check(hm_agrees(a.get(), a.get(), a.get(), a_image, a_image, a_image, nullptr, "full alias"),
              "full alias converges");
        // The moving image as its own mask (every pixel nonzero) behaves as the unmasked export.
        check(cv::countNonZero(t21_image) == n * n, "moving aliasing fixture has zero pixels");
        hm_agrees(a.get(), b21.get(), b21.get(), a_image, t21_image, t21_image, nullptr,
                  "mask aliasing the moving image");
        {
            opencv_video_ecc_homography_result p = hom_sentinel(), q = hom_pattern(0x21);
            check(hom_call(a.get(), b21.get(), &p) == OPENCV_VIDEO_OK &&
                  hm_call(a.get(), b21.get(), b21.get(), &q) == OPENCV_VIDEO_OK &&
                  std::memcmp(&p, &q, 48) == 0, "moving image as mask differs from unmasked");
        }
        agreements += 2;
        // Inputs unchanged.
        check(hom_checksum(output(hole.get())) == hole_sum, "caller mask modified");
        check(cv::norm(output(a.get()), a_image, cv::NORM_INF) == 0 &&
              cv::norm(output(b21.get()), t21_image, cv::NORM_INF) == 0, "masked call modified an image");
        // Options reach the masked native call.
        for (int it : {1, 3, 100}) {
            hm_agrees(a.get(), b21.get(), hole.get(), a_image, t21_image, hole_image, nullptr, "masked options", it, 1e-8);
            hm_agrees(a.get(), bltp.get(), hole.get(), a_image, ltp_image, hole_image, ltp_near, "seeded masked options", it, 0.1);
            agreements += 2;
        }
        opencv_video_ecc_homography_result one = hom_sentinel(), many = hom_sentinel();
        hm_call(a.get(), b21.get(), hole.get(), &one, 1);
        hm_call(a.get(), b21.get(), hole.get(), &many, 50);
        check(std::memcmp(&one, &many, 48) != 0, "masked iteration count has no effect");
        // Constant and noise images with a valid mask: native failure, untouched.
        auto flat = ecc_handle(ecc_fixture::constant(n, n, 128));
        check(!hm_agrees(flat.get(), flat.get(), hole.get(), ecc_fixture::constant(n, n, 128),
                         ecc_fixture::constant(n, n, 128), hole_image, nullptr, "constant masked"),
              "constant images must raise");
        ++agreements;
    }

    // 8. Three prefills on success (zero padding, prefill-independent) and failure (preserved).
    {
        const opencv_video_ecc_homography_result prefills[] = {hom_sentinel(), hom_pattern(0x11), hom_pattern(0xC7)};
        unsigned char first_id[48], first_seed[48];
        auto zero = ecc_handle(cv::Mat(n, n, CV_8UC1, cv::Scalar(0)));
        auto flat = ecc_handle(ecc_fixture::constant(n, n, 128));
        for (size_t k = 0; k < 3; ++k) {
            opencv_video_ecc_homography_result r = prefills[k], s = prefills[k];
            check(hm_call(a.get(), b21.get(), hole.get(), &r) == OPENCV_VIDEO_OK &&
                  hms_call(a.get(), bltp.get(), hole.get(), ltp_near, &s) == OPENCV_VIDEO_OK, "prefill successes");
            hom_padding_zero(r, "masked padding");
            hom_padding_zero(s, "masked seeded padding");
            unsigned char br[48], bs[48];
            hom_bytes(r, br);
            hom_bytes(s, bs);
            if (k == 0) { std::memcpy(first_id, br, 48); std::memcpy(first_seed, bs, 48); }
            check(std::memcmp(first_id, br, 48) == 0 && std::memcmp(first_seed, bs, 48) == 0,
                  "masked success bytes depend on the prefill");
            opencv_video_ecc_homography_result f = prefills[k];
            check(hm_call(a.get(), b21.get(), zero.get(), &f) == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT &&
                  std::memcmp(&f, &prefills[k], 48) == 0, "zero-mask failure changed the prefill");
            check(hms_call(flat.get(), flat.get(), hole.get(), ltp_near, &f) == OPENCV_VIDEO_ERROR_OPENCV &&
                  std::memcmp(&f, &prefills[k], 48) == 0, "native masked failure changed the prefill");
        }
    }

#ifdef OPENCV_VIDEO_TEST_FAULTS
    // 9. Fault build: injected exceptions at both stages, rejected corruption and schema
    //    mutations on both masked exports; accepted finite injected matrices have zero padding.
    {
        int injections = 0, corruptions = 0, schemas = 0, accepted = 0;
        for (int which = 0; which < 2; ++which) {
            auto call = [&](opencv_video_ecc_homography_result *out) {
                return which == 0 ? hm_call(a.get(), b21.get(), hole.get(), out)
                                  : hms_call(a.get(), bltp.get(), hole.get(), ltp_near, out);
            };
            for (const auto &p : {hom_sentinel(), hom_pattern(0x5A)}) {
                for (int stage : {1, 2}) for (int kind : {1, 2, 4, 3}) {
                    opencv_video_ecc_homography_result r = p;
                    fault_stage = stage; fault_kind = kind;
                    const auto code = call(&r);
                    fault_stage = 0; fault_kind = 0;
                    const auto expected = kind == 2 ? OPENCV_VIDEO_ERROR_OPENCV
                                        : kind == 3 ? OPENCV_VIDEO_ERROR_UNKNOWN : OPENCV_VIDEO_ERROR_STANDARD;
                    check(code == expected && std::memcmp(&r, &p, 48) == 0, "masked injected exception");
                    ++injections;
                }
                for (int mode = 1; mode <= 20; ++mode) {
                    opencv_video_ecc_homography_result r = p;
                    hom_corrupt_mode = mode;
                    const auto code = call(&r);
                    hom_corrupt_mode = 0;
                    check(code == OPENCV_VIDEO_ERROR_OPENCV && std::memcmp(&r, &p, 48) == 0,
                          "masked corrupted native output accepted or partially published");
                    ++corruptions;
                }
                for (int mode = 1; mode <= 8; ++mode) {
                    opencv_video_ecc_homography_result r = p;
                    hom_schema_mode = mode;
                    hom_schema_applied = 0;
                    const auto code = call(&r);
                    hom_schema_mode = 0;
                    hom_schema_keep.release();
                    hom_schema_wide.release();
                    check(hom_schema_applied == 1, "masked schema hook not reached");
                    check(code == OPENCV_VIDEO_ERROR_OPENCV && std::memcmp(&r, &p, 48) == 0,
                          "masked altered schema accepted or partially published");
                    ++schemas;
                }
                for (int mode = 30; mode <= 34; ++mode) {
                    opencv_video_ecc_homography_result r = p;
                    hom_corrupt_mode = mode;
                    const auto code = call(&r);
                    hom_corrupt_mode = 0;
                    check(code == OPENCV_VIDEO_OK, "masked injected finite matrix rejected");
                    float got[9];
                    hom_values(r, got);
                    check(std::memcmp(got, kHomInjected[mode - 30], sizeof got) == 0,
                          "masked injected finite matrix not published exactly");
                    hom_padding_zero(r, "masked injected finite padding");
                    ++accepted;
                }
            }
            // Mask validation precedes every hook: a rejected mask never reaches stage 1.
            auto zero = ecc_handle(cv::Mat(n, n, CV_8UC1, cv::Scalar(0)));
            fault_stage = 1; fault_kind = 3;
            opencv_video_ecc_homography_result r = hom_sentinel();
            const auto code = which == 0 ? hm_call(a.get(), b21.get(), zero.get(), &r)
                                         : hms_call(a.get(), bltp.get(), zero.get(), ltp_near, &r);
            fault_stage = 0; fault_kind = 0;
            check(code == OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "mask validation does not precede native stage");
            hom_unchanged(r, "pre-native rejection changed the result");
        }
        check(injections == 32 && corruptions == 80 && schemas == 32 && accepted == 20, "masked fault coverage");
        std::cout << "PASS: ECC homography masked faults: " << injections << " exception injections, "
                  << corruptions << " rejected corruptions, " << schemas << " schema mutations, "
                  << accepted << " accepted finite matrices, mask validation before hooks\n";
    }
#endif
    std::cout << "PASS: ECC homography masked shim vs direct masked findTransformECC: " << agreements
              << " agreements, mask consumption, full-mask equivalence, value semantics, coordinate frame, "
                 "24 schema rejections, support, Regions, aliases, immutability, 3 prefills\n";
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
        run_ecc_euclidean();
        run_ecc_euclidean_seeded();
        run_ecc_affine();
        run_ecc_affine_seeded();
        run_ecc_homography();
        run_ecc_homography_seeded();
        run_ecc_homography_padding();
        run_ecc_homography_masked();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
