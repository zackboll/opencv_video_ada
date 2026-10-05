#include "opencv_video_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <exception>
#include <sstream>
#include <string>
#include <utility>

namespace {
thread_local std::string g_last_error;

void clear_error() { g_last_error.clear(); }

opencv_video_status fail(opencv_video_status code, const std::string &message) {
    g_last_error = message;
    return code;
}

bool finite(double value) { return std::isfinite(value); }

opencv_video_status resolve_input(const opencv_core_mat_handle *handle,
                                  const cv::Mat **out,
                                  const char *name) {
    if (handle == nullptr || out == nullptr) {
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                    std::string(name) + " handle is null");
    }
    const opencv_core_status status = opencv_core_module_input_mat(handle, out);
    if (status != OPENCV_CORE_OK || *out == nullptr) {
        std::ostringstream message;
        message << "Core failed to resolve " << name << " (status " << status << ")";
        return fail(OPENCV_VIDEO_ERROR_CORE_BRIDGE, message.str());
    }
    return OPENCV_VIDEO_OK;
}

opencv_video_status resolve_output(opencv_core_mat_handle *handle,
                                   cv::Mat **out,
                                   const char *name) {
    if (handle == nullptr || out == nullptr) {
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                    std::string(name) + " handle is null");
    }
    const opencv_core_status status = opencv_core_module_output_mat(handle, out);
    if (status != OPENCV_CORE_OK || *out == nullptr) {
        std::ostringstream message;
        message << "Core failed to resolve " << name << " (status " << status << ")";
        return fail(OPENCV_VIDEO_ERROR_CORE_BRIDGE, message.str());
    }
    return OPENCV_VIDEO_OK;
}
}  // namespace

extern "C" const char *opencv_video_native_version(void) { return CV_VERSION; }

extern "C" const char *opencv_video_native_backend(void) { return "video"; }

extern "C" const char *opencv_video_last_error(void) { return g_last_error.c_str(); }

extern "C" opencv_video_status opencv_video_track_pyr_lk(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *track_error,
    int32_t window_width,
    int32_t window_height,
    int32_t max_level,
    int32_t maximum_iterations,
    double epsilon,
    double min_eigenvalue_threshold) {
    try {
        clear_error();
        const cv::Mat *previous = nullptr;
        const cv::Mat *next = nullptr;
        const cv::Mat *points = nullptr;
        cv::Mat *published_next = nullptr;
        cv::Mat *published_status = nullptr;
        cv::Mat *published_error = nullptr;

        opencv_video_status status = resolve_input(previous_image, &previous, "previous image");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_input(next_image, &next, "next image");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_input(previous_points, &points, "previous points");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_output(next_points, &published_next, "next points");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_output(track_status, &published_status, "track status");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_output(track_error, &published_error, "track error");
        if (status != OPENCV_VIDEO_OK) return status;

        if (previous->empty() || next->empty() || previous->dims != 2 || next->dims != 2 ||
            previous->type() != CV_8UC1 || next->type() != CV_8UC1 ||
            previous->size() != next->size()) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                        "PyrLK images must be nonempty matching 2-D UInt8 C1 Mats");
        }

        const int point_count = points->checkVector(2, CV_32F, true);
        if (point_count <= 0) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                        "PyrLK previous points must be a nonempty continuous Float32 2-vector");
        }
        if (window_width <= 0 || window_height <= 0 || max_level < 0 ||
            maximum_iterations <= 0 || !finite(epsilon) || epsilon <= 0.0 ||
            !finite(min_eigenvalue_threshold) || min_eigenvalue_threshold < 0.0) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Invalid PyrLK options");
        }

        cv::Mat computed_next;
        cv::Mat computed_status;
        cv::Mat computed_error;
        const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                        maximum_iterations, epsilon);

        cv::calcOpticalFlowPyrLK(*previous, *next, *points,
                                 computed_next, computed_status, computed_error,
                                 cv::Size(window_width, window_height), max_level,
                                 criteria, 0, min_eigenvalue_threshold);

        if (computed_next.checkVector(2, CV_32F, true) != point_count ||
            computed_status.checkVector(1, CV_8U, true) != point_count ||
            computed_error.checkVector(1, CV_32F, true) != point_count) {
            return fail(OPENCV_VIDEO_ERROR_OPENCV,
                        "OpenCV returned an unexpected PyrLK output schema");
        }

        // Failure atomicity: publish only after every native result is complete
        // and its schema has been validated.
        *published_next = std::move(computed_next);
        *published_status = std::move(computed_status);
        *published_error = std::move(computed_error);
        clear_error();
        return OPENCV_VIDEO_OK;
    } catch (const cv::Exception &error) {
        return fail(OPENCV_VIDEO_ERROR_OPENCV, error.what());
    } catch (const std::exception &error) {
        return fail(OPENCV_VIDEO_ERROR_STANDARD, error.what());
    } catch (...) {
        return fail(OPENCV_VIDEO_ERROR_UNKNOWN, "Unknown C++ exception in PyrLK shim");
    }
}
