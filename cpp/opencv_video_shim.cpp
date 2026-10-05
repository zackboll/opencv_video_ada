#include "opencv_video_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <exception>
#include <cstdio>
#include <limits>
#include <utility>

#ifdef OPENCV_VIDEO_TEST_FAULTS
void opencv_video_test_fault(int);
#endif

namespace {
// Diagnostics must remain safe even while handling allocation failure.
thread_local char g_last_error[512] = {};

void clear_error() noexcept { g_last_error[0] = '\0'; }

opencv_video_status fail(opencv_video_status code, const char *message) noexcept {
    std::snprintf(g_last_error, sizeof(g_last_error), "%s", message);
    return code;
}

opencv_video_status resolve_input(const opencv_core_mat_handle *handle,
                                  const cv::Mat **out,
                                  const char *name) {
    if (handle == nullptr || out == nullptr) {
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                    name);
    }
    const opencv_core_status status = opencv_core_module_input_mat(handle, out);
    if (status != OPENCV_CORE_OK || *out == nullptr) {
        return fail(OPENCV_VIDEO_ERROR_CORE_BRIDGE, name);
    }
    return OPENCV_VIDEO_OK;
}

opencv_video_status resolve_output(opencv_core_mat_handle *handle,
                                   cv::Mat **out,
                                   const char *name) {
    if (handle == nullptr || out == nullptr) {
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                    name);
    }
    const opencv_core_status status = opencv_core_module_output_mat(handle, out);
    if (status != OPENCV_CORE_OK || *out == nullptr) {
        return fail(OPENCV_VIDEO_ERROR_CORE_BRIDGE, name);
    }
    return OPENCV_VIDEO_OK;
}
}  // namespace

extern "C" const char *opencv_video_native_version(void) { return CV_VERSION; }

extern "C" const char *opencv_video_native_backend(void) { return "video"; }

extern "C" const char *opencv_video_last_error(void) { return g_last_error; }

static opencv_video_status track_pyr_lk(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    bool seeded,
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
        const cv::Mat *seeds = nullptr;
        cv::Mat *published_next = nullptr;
        cv::Mat *published_status = nullptr;
        cv::Mat *published_error = nullptr;

        opencv_video_status status = resolve_input(previous_image, &previous, "previous image");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_input(next_image, &next, "next image");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_input(previous_points, &points, "previous points");
        if (status != OPENCV_VIDEO_OK) return status;
        if (seeded) {
            status = resolve_input(initial_next_points, &seeds, "initial next points");
            if (status != OPENCV_VIDEO_OK) return status;
        }
        status = resolve_output(next_points, &published_next, "next points");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_output(track_status, &published_status, "track status");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_output(track_error, &published_error, "track error");
        if (status != OPENCV_VIDEO_OK) return status;

        if (published_next == published_status || published_next == published_error ||
            published_status == published_error ||
            published_next == previous || published_next == next || published_next == points ||
            published_status == previous || published_status == next || published_status == points ||
            published_error == previous || published_error == next || published_error == points ||
            (seeded && (published_next == seeds || published_status == seeds || published_error == seeds))) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Aliased input/output headers");
        }

        if (previous->empty() || next->empty() || previous->dims != 2 || next->dims != 2 ||
            previous->type() != CV_8UC1 || next->type() != CV_8UC1 ||
            previous->size() != next->size()) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                        "PyrLK images must be nonempty matching 2-D UInt8 C1 Mats");
        }

        if (window_width < 3 || window_width > 255 || window_height < 3 || window_height > 255 ||
            max_level < 0 || max_level > 30 || maximum_iterations < 1 || maximum_iterations > 100 ||
            !std::isfinite(epsilon) || epsilon <= 0.0 || epsilon > 10.0 ||
            !std::isfinite(min_eigenvalue_threshold) || min_eigenvalue_threshold < 0.0 ||
            min_eigenvalue_threshold > std::numeric_limits<float>::max()) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Invalid PyrLK options");
        }
        // Native code uses signed int row strides and row*stride expressions.
        const int64_t padded_rows = int64_t(previous->rows) + 2 * window_height;
        const int64_t padded_cols = int64_t(previous->cols) + 2 * window_width;
        if (padded_rows * padded_cols > std::numeric_limits<int>::max() / 4 ||
            previous->step[0] > size_t(std::numeric_limits<int>::max() / padded_rows) ||
            next->step[0] > size_t(std::numeric_limits<int>::max() / padded_rows)) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Image exceeds safe native arithmetic bounds");
        }

        if (seeded && (seeds->dims != 2 || seeds->cols != 1 || seeds->type() != CV_32FC2 ||
                       seeds->total() != points->total() || !seeds->isContinuous())) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                        "Seeds must be matching continuous N x 1 Float32 C2");
        }
        if (points->empty()) {
            if (points->dims != 2 || points->type() != CV_32FC2 || points->total() != 0) {
                return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Invalid empty point schema");
            }
            published_next->release();
            published_status->release();
            published_error->release();
            return OPENCV_VIDEO_OK;
        }
        if (points->dims != 2 || points->cols != 1 || points->type() != CV_32FC2) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Points must be N x 1 Float32 C2");
        }
        const int point_count = points->checkVector(2, CV_32F, true);
        if (point_count <= 0) {
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                        "PyrLK previous points must be a nonempty continuous Float32 2-vector");
        }
        const cv::Point2f *input = points->ptr<cv::Point2f>();
        for (int i = 0; i < point_count; ++i) {
            if (!std::isfinite(input[i].x) || !std::isfinite(input[i].y) ||
                std::abs(input[i].x) > 536870912.0f || std::abs(input[i].y) > 536870912.0f) {
                return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Point exceeds safe native conversion bounds");
            }
        }
        if (seeded) {
            if (seeds->checkVector(2, CV_32F, true) != point_count) {
                return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Seed count mismatch");
            }
            const auto *estimates = seeds->ptr<cv::Point2f>();
            for (int i = 0; i < point_count; ++i) {
                if (!std::isfinite(estimates[i].x) || !std::isfinite(estimates[i].y) ||
                    std::abs(estimates[i].x) > 536870912.0f || std::abs(estimates[i].y) > 536870912.0f) {
                    return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Seed exceeds safe native conversion bounds");
                }
            }
        }

        cv::Mat computed_next;
        // OpenCV mutates InputOutputArray even on paths that later fail. Never
        // pass a borrowed seed or caller-visible output allocation to it.
        if (seeded) computed_next = seeds->clone();
        cv::Mat computed_status;
        cv::Mat computed_error;
        const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                        maximum_iterations, epsilon);

#ifdef OPENCV_VIDEO_TEST_FAULTS
        // Only the separately compiled qualification binary supplies this hook.
        opencv_video_test_fault(1);
#endif

        cv::calcOpticalFlowPyrLK(*previous, *next, *points,
                                 computed_next, computed_status, computed_error,
                                 cv::Size(window_width, window_height), max_level,
                                 criteria, seeded ? cv::OPTFLOW_USE_INITIAL_FLOW : 0,
                                 min_eigenvalue_threshold);

        if (computed_next.checkVector(2, CV_32F, true) != point_count ||
            computed_status.checkVector(1, CV_8U, true) != point_count ||
            computed_error.checkVector(1, CV_32F, true) != point_count) {
            return fail(OPENCV_VIDEO_ERROR_OPENCV,
                        "OpenCV returned an unexpected PyrLK output schema");
        }

#ifdef OPENCV_VIDEO_TEST_FAULTS
        opencv_video_test_fault(2);
#endif
        auto *result = computed_next.ptr<cv::Point2f>();
        auto *flags = computed_status.ptr<unsigned char>();
        auto *errors = computed_error.ptr<float>();
        for (int i = 0; i < point_count; ++i) {
            if (flags[i] == 0) {
                // Never read native failed-track point/error storage.
                result[i] = input[i];
                errors[i] = 0.0f;
            } else if (flags[i] != 1 || !std::isfinite(result[i].x) ||
                       !std::isfinite(result[i].y) || !std::isfinite(errors[i]) || errors[i] < 0) {
                return fail(OPENCV_VIDEO_ERROR_OPENCV, "Invalid successful PyrLK result");
            }
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

extern "C" opencv_video_status opencv_video_track_pyr_lk(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *track_error,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold) {
    return track_pyr_lk(previous_image, next_image, previous_points, nullptr, false,
                        next_points, track_status, track_error, window_width, window_height,
                        max_level, maximum_iterations, epsilon, min_eigenvalue_threshold);
}

extern "C" opencv_video_status opencv_video_track_pyr_lk_seeded(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *track_error,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold) {
    return track_pyr_lk(previous_image, next_image, previous_points, initial_next_points, true,
                        next_points, track_status, track_error, window_width, window_height,
                        max_level, maximum_iterations, epsilon, min_eigenvalue_threshold);
}
