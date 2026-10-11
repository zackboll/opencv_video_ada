#include "opencv_video_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <exception>
#include <cstdio>
#include <limits>
#include <utility>
#include <memory>
#include <vector>
#include <algorithm>

struct opencv_video_pyramid_handle {
    std::vector<cv::Mat> levels;
    cv::Size window;
    int requested = 0;
    int available = 0;
    cv::Size geometry;
};

#ifdef OPENCV_VIDEO_TEST_FAULTS
void opencv_video_test_fault(int);
void opencv_video_test_corrupt_flow(float *values, size_t count);
#endif

namespace {
enum class ErrorMode { Photometric, MinimumEigenvalue };
// Diagnostics must remain safe even while handling allocation failure.
thread_local char g_last_error[512] = {};

void clear_error() noexcept { g_last_error[0] = '\0'; }

bool valid_pyramid(const opencv_video_pyramid_handle &p) {
    if (p.window.width < 3 || p.window.width > 255 ||
        p.window.height < 3 || p.window.height > 255 ||
        p.requested < 0 || p.requested > 30 || p.available < 0 ||
        p.available > p.requested || p.geometry.width <= 0 || p.geometry.height <= 0 ||
        p.levels.size() != size_t(2 * (p.available + 1))) return false;
    auto size = p.geometry;
    for (int level = 0; level <= p.available; ++level) {
        for (int member = 0; member < 2; ++member) {
            const auto &m = p.levels[size_t(2 * level + member)];
            if (m.empty() || m.dims != 2 || m.size() != size ||
                m.type() != (member == 0 ? CV_8UC1 : CV_16SC2) || m.u == nullptr)
                return false;
            cv::Size whole;
            cv::Point offset;
            m.locateROI(whole, offset);
            if (offset.x != p.window.width || offset.y != p.window.height ||
                whole.width != size.width + 2 * p.window.width ||
                whole.height != size.height + 2 * p.window.height) return false;
        }
        size = {(size.width + 1) / 2, (size.height + 1) / 2};
        if (level < p.available &&
            (size.width <= p.window.width || size.height <= p.window.height)) return false;
    }
    return p.available == p.requested ||
        size.width <= p.window.width || size.height <= p.window.height;
}

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

extern "C" opencv_video_status opencv_video_pyramid_create(
    const opencv_core_mat_handle *image, int32_t width, int32_t height,
    int32_t requested, opencv_video_pyramid_handle **out) {
    if (out == nullptr) return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Null pyramid output");
    *out = nullptr;
    try {
        clear_error();
        const cv::Mat *source = nullptr;
        auto code = resolve_input(image, &source, "pyramid image");
        if (code != OPENCV_VIDEO_OK) return code;
        if (source->empty() || source->dims != 2 || source->type() != CV_8UC1 ||
            width < 3 || width > 255 || height < 3 || height > 255 ||
            requested < 0 || requested > 30)
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Invalid pyramid image/options");
        const int64_t rows = int64_t(source->rows) + 2 * height;
        const int64_t cols = int64_t(source->cols) + 2 * width;
        if (rows * cols > std::numeric_limits<int>::max() / 4 ||
            source->step[0] > size_t(std::numeric_limits<int>::max() / rows))
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Unsafe pyramid arithmetic");
        auto p = std::make_unique<opencv_video_pyramid_handle>();
        p->window = {width, height};
        p->requested = requested;
        p->geometry = source->size();
#ifdef OPENCV_VIDEO_TEST_FAULTS
        opencv_video_test_fault(3);
#endif
        p->available = cv::buildOpticalFlowPyramid(*source, p->levels, p->window,
            requested, true, cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, false);
        if (!valid_pyramid(*p))
            return fail(OPENCV_VIDEO_ERROR_OPENCV, "Invalid native pyramid structure");
#ifdef OPENCV_VIDEO_TEST_FAULTS
        opencv_video_test_fault(4);
#endif
        *out = p.release();
        return OPENCV_VIDEO_OK;
    } catch (const cv::Exception &e) {
        return fail(OPENCV_VIDEO_ERROR_OPENCV, e.what());
    } catch (const std::exception &e) {
        return fail(OPENCV_VIDEO_ERROR_STANDARD, e.what());
    } catch (...) {
        return fail(OPENCV_VIDEO_ERROR_UNKNOWN, "Unknown pyramid construction exception");
    }
}

extern "C" void opencv_video_pyramid_destroy(opencv_video_pyramid_handle *p) { delete p; }

extern "C" opencv_video_status opencv_video_pyramid_metadata(
    const opencv_video_pyramid_handle *p, int32_t *width, int32_t *height,
    int32_t *requested, int32_t *available) {
    clear_error();
    if (!p || !width || !height || !requested || !available)
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Null pyramid metadata argument");
    *width = p->window.width;
    *height = p->window.height;
    *requested = p->requested;
    *available = p->available;
    return OPENCV_VIDEO_OK;
}

static opencv_video_status track_pyr_lk(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    bool seeded,
    ErrorMode error_mode,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *track_error,
    int32_t window_width,
    int32_t window_height,
    int32_t max_level,
    int32_t maximum_iterations,
    double epsilon,
    double min_eigenvalue_threshold,
    const opencv_video_pyramid_handle *previous_pyramid = nullptr,
    const opencv_video_pyramid_handle *next_pyramid = nullptr) {
    try {
        clear_error();
        const cv::Mat *previous = nullptr;
        const cv::Mat *next = nullptr;
        const cv::Mat *points = nullptr;
        const cv::Mat *seeds = nullptr;
        cv::Mat *published_next = nullptr;
        cv::Mat *published_status = nullptr;
        cv::Mat *published_error = nullptr;

        opencv_video_status status = OPENCV_VIDEO_OK;
        if (previous_pyramid || next_pyramid) {
            if (!previous_pyramid || !next_pyramid ||
                !valid_pyramid(*previous_pyramid) || !valid_pyramid(*next_pyramid) ||
                previous_pyramid->window != cv::Size(window_width, window_height) ||
                next_pyramid->window != cv::Size(window_width, window_height) ||
                max_level > previous_pyramid->requested || max_level > next_pyramid->requested)
                return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Incompatible pyramids");
            previous = &previous_pyramid->levels[0];
            next = &next_pyramid->levels[0];
        } else {
            status = resolve_input(previous_image, &previous, "previous image");
            if (status != OPENCV_VIDEO_OK) return status;
            status = resolve_input(next_image, &next, "next image");
            if (status != OPENCV_VIDEO_OK) return status;
        }
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
        const bool quality = error_mode == ErrorMode::MinimumEigenvalue;
        if (quality) {
            computed_error = cv::Mat(point_count, 1, CV_32F,
                                    cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
        }
        const auto *error_storage = computed_error.data;
        const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                        maximum_iterations, epsilon);

#ifdef OPENCV_VIDEO_TEST_FAULTS
        // Only the separately compiled qualification binary supplies this hook.
        opencv_video_test_fault(1);
#endif

        if (previous_pyramid) {
            cv::calcOpticalFlowPyrLK(previous_pyramid->levels, next_pyramid->levels, *points,
                computed_next, computed_status, computed_error,
                cv::Size(window_width, window_height),
                std::min({max_level, previous_pyramid->available, next_pyramid->available}),
                criteria, (seeded ? cv::OPTFLOW_USE_INITIAL_FLOW : 0) |
                    (quality ? cv::OPTFLOW_LK_GET_MIN_EIGENVALS : 0), min_eigenvalue_threshold);
        } else cv::calcOpticalFlowPyrLK(*previous, *next, *points,
                                 computed_next, computed_status, computed_error,
                                 cv::Size(window_width, window_height), max_level,
                                 criteria, (seeded ? cv::OPTFLOW_USE_INITIAL_FLOW : 0) |
                                     (quality ? cv::OPTFLOW_LK_GET_MIN_EIGENVALS : 0),
                                 min_eigenvalue_threshold);

        // Mat OutputArray::create reuses matching storage on all reviewed tags.
        // Reject replacement: otherwise the sentinel no longer proves writes.
        if (quality && computed_error.data != error_storage) {
            return fail(OPENCV_VIDEO_ERROR_OPENCV, "PyrLK replaced minimum-eigenvalue storage");
        }
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
            if (quality && (!std::isfinite(errors[i]) || errors[i] < 0)) {
                return fail(OPENCV_VIDEO_ERROR_OPENCV, "Invalid or unwritten PyrLK minimum eigenvalue");
            }
            if (flags[i] == 0) {
                // Never read failed nextPts. Quality is independent of status;
                // only photometric failed errors are undefined and discarded.
                result[i] = input[i];
                if (!quality) errors[i] = 0.0f;
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
                        ErrorMode::Photometric,
                        next_points, track_status, track_error, window_width, window_height,
                        max_level, maximum_iterations, epsilon, min_eigenvalue_threshold);
}

extern "C" opencv_video_status opencv_video_track_pyr_lk_pyramids(
    const opencv_video_pyramid_handle *previous, const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *points, opencv_core_mat_handle *result,
    opencv_core_mat_handle *status, opencv_core_mat_handle *error,
    int32_t width, int32_t height, int32_t level, int32_t iterations,
    double epsilon, double threshold) {
    if (!previous || !next)
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Empty pyramid");
    return track_pyr_lk(nullptr, nullptr, points, nullptr, false, ErrorMode::Photometric,
        result, status, error, width, height, level, iterations, epsilon, threshold, previous, next);
}

extern "C" opencv_video_status opencv_video_track_pyr_lk_pyramids_seeded(
    const opencv_video_pyramid_handle *previous, const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *points, const opencv_core_mat_handle *seeds,
    opencv_core_mat_handle *result, opencv_core_mat_handle *status, opencv_core_mat_handle *error,
    int32_t width, int32_t height, int32_t level, int32_t iterations,
    double epsilon, double threshold) {
    if (!previous || !next)
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Empty pyramid");
    return track_pyr_lk(nullptr, nullptr, points, seeds, true, ErrorMode::Photometric,
        result, status, error, width, height, level, iterations, epsilon, threshold, previous, next);
}

extern "C" opencv_video_status opencv_video_track_pyr_lk_pyramids_min_eigenvalues(
    const opencv_video_pyramid_handle *previous, const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *points, opencv_core_mat_handle *result,
    opencv_core_mat_handle *status, opencv_core_mat_handle *error,
    int32_t width, int32_t height, int32_t level, int32_t iterations,
    double epsilon, double threshold) {
    if (!previous || !next)
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Empty pyramid");
    return track_pyr_lk(nullptr, nullptr, points, nullptr, false, ErrorMode::MinimumEigenvalue,
        result, status, error, width, height, level, iterations, epsilon, threshold, previous, next);
}

extern "C" opencv_video_status opencv_video_track_pyr_lk_pyramids_seeded_min_eigenvalues(
    const opencv_video_pyramid_handle *previous, const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *points, const opencv_core_mat_handle *seeds,
    opencv_core_mat_handle *result, opencv_core_mat_handle *status, opencv_core_mat_handle *error,
    int32_t width, int32_t height, int32_t level, int32_t iterations,
    double epsilon, double threshold) {
    if (!previous || !next)
        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Empty pyramid");
    return track_pyr_lk(nullptr, nullptr, points, seeds, true, ErrorMode::MinimumEigenvalue,
        result, status, error, width, height, level, iterations, epsilon, threshold, previous, next);
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
                        ErrorMode::Photometric,
                        next_points, track_status, track_error, window_width, window_height,
                        max_level, maximum_iterations, epsilon, min_eigenvalue_threshold);
}

extern "C" opencv_video_status opencv_video_track_pyr_lk_min_eigenvalues(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *minimum_eigenvalues,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold) {
    return track_pyr_lk(previous_image, next_image, previous_points, nullptr, false,
                        ErrorMode::MinimumEigenvalue,
                        next_points, track_status, minimum_eigenvalues, window_width, window_height,
                        max_level, maximum_iterations, epsilon, min_eigenvalue_threshold);
}

extern "C" opencv_video_status opencv_video_track_pyr_lk_seeded_min_eigenvalues(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *minimum_eigenvalues,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold) {
    return track_pyr_lk(previous_image, next_image, previous_points, initial_next_points, true,
                        ErrorMode::MinimumEigenvalue,
                        next_points, track_status, minimum_eigenvalues, window_width, window_height,
                        max_level, maximum_iterations, epsilon, min_eigenvalue_threshold);
}

namespace {
// Shared by the unseeded (flags 0) and seeded (flags 4) Farneback exports.
// Binding-policy bound on each initial displacement component (not an OpenCV
// guarantee): 2^20 px. Images have at most INT_MAX/16 pixels, so a column index
// is below 2^23 and x + dx stays far below 2^31 for cvFloor in the CPU kernel.
constexpr double kMaxInitialDisplacement = 1048576.0;

opencv_video_status calc_farneback(const opencv_core_mat_handle *previous_image,
                                   const opencv_core_mat_handle *next_image,
                                   const opencv_core_mat_handle *initial_flow, bool seeded,
                                   opencv_core_mat_handle *flow,
                                   double pyramid_scale, int32_t levels, int32_t window_size,
                                   int32_t iterations, int32_t poly_neighborhood,
                                   double poly_sigma) {
    try {
        clear_error();
        const cv::Mat *previous = nullptr;
        const cv::Mat *next = nullptr;
        const cv::Mat *seed = nullptr;
        cv::Mat *published = nullptr;
        auto status = resolve_input(previous_image, &previous, "previous image");
        if (status != OPENCV_VIDEO_OK) return status;
        status = resolve_input(next_image, &next, "next image");
        if (status != OPENCV_VIDEO_OK) return status;
        if (seeded) {
            status = resolve_input(initial_flow, &seed, "initial flow");
            if (status != OPENCV_VIDEO_OK) return status;
        }
        status = resolve_output(flow, &published, "flow");
        if (status != OPENCV_VIDEO_OK) return status;
        if (published == previous || published == next || (seeded && published == seed))
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Aliased input/output headers");
        if (previous->empty() || next->empty() || previous->dims != 2 || next->dims != 2 ||
            previous->type() != CV_8UC1 || next->type() != CV_8UC1 ||
            previous->size() != next->size() ||
            previous->rows < 16 || previous->cols < 16)
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                        "Farneback images must be matching 2-D UInt8 C1 Mats, at least 16x16");
        if (!std::isfinite(pyramid_scale) || pyramid_scale < 0.25 || pyramid_scale > 0.90 ||
            levels < 1 || levels > 8 || window_size < 5 || window_size > 63 ||
            window_size % 2 == 0 || iterations < 1 || iterations > 30 ||
            (poly_neighborhood != 5 && poly_neighborhood != 7) ||
            !std::isfinite(poly_sigma) || poly_sigma < 0.1 || poly_sigma > 10.0)
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT, "Invalid Farneback options");
        const int64_t pixels = int64_t(previous->rows) * int64_t(previous->cols);
        if (pixels > int64_t(std::numeric_limits<int>::max() / 16) ||
            previous->step[0] > size_t(std::numeric_limits<int>::max() / 16) ||
            next->step[0] > size_t(std::numeric_limits<int>::max() / 16))
            return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                        "Image exceeds safe native arithmetic bounds");

        cv::Mat computed;
        if (seeded) {
            if (seed->empty() || seed->dims != 2 || seed->type() != CV_32FC2 ||
                seed->size() != previous->size())
                return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                            "Initial flow must be a nonempty 2-D Float32 C2 Mat matching the images");
            if (seed->step[0] > size_t(std::numeric_limits<int>::max() / 16))
                return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                            "Initial flow exceeds safe native arithmetic bounds");
            // Validate every component (widened to double) before any native work.
            for (int r = 0; r < seed->rows; ++r) {
                const float *row = seed->ptr<float>(r);
                for (int i = 0; i < seed->cols * 2; ++i) {
                    const double v = double(row[i]);
                    if (!std::isfinite(v) || std::fabs(v) > kMaxInitialDisplacement)
                        return fail(OPENCV_VIDEO_ERROR_INVALID_ARGUMENT,
                                    "Initial flow component is nonfinite or exceeds 2^20 pixels");
                }
            }
            // Private continuous clone: native may mutate it, never the caller's seed.
            computed = seed->clone();
            if (computed.data == seed->data || !computed.isContinuous() ||
                computed.type() != CV_32FC2 || computed.size() != previous->size())
                return fail(OPENCV_VIDEO_ERROR_OPENCV, "Initial flow clone is not private/continuous");
        } else {
            // Private sentinel-filled output: unwritten components cannot look valid.
            const double nan = std::numeric_limits<double>::quiet_NaN();
            computed = cv::Mat(previous->rows, previous->cols, CV_32FC2, cv::Scalar(nan, nan));
        }
        const auto *storage = computed.data;

#ifdef OPENCV_VIDEO_TEST_FAULTS
        opencv_video_test_fault(1);
#endif
        cv::calcOpticalFlowFarneback(*previous, *next, computed, pyramid_scale, levels,
                                     window_size, iterations, poly_neighborhood,
                                     poly_sigma, seeded ? cv::OPTFLOW_USE_INITIAL_FLOW : 0);
        if (computed.data != storage)
            return fail(OPENCV_VIDEO_ERROR_OPENCV, "Farneback replaced output storage");
        if (computed.dims != 2 || computed.type() != CV_32FC2 ||
            computed.rows != previous->rows || computed.cols != previous->cols ||
            !computed.isContinuous())
            return fail(OPENCV_VIDEO_ERROR_OPENCV, "OpenCV returned an unexpected Farneback schema");
#ifdef OPENCV_VIDEO_TEST_FAULTS
        opencv_video_test_fault(2);
        opencv_video_test_corrupt_flow(computed.ptr<float>(), size_t(pixels) * 2);
#endif
        const float *values = computed.ptr<float>();
        const size_t count = size_t(pixels) * 2;
        for (size_t i = 0; i < count; ++i)
            if (!std::isfinite(values[i]))
                return fail(OPENCV_VIDEO_ERROR_OPENCV, "Farneback produced a nonfinite component");

        *published = std::move(computed);
        clear_error();
        return OPENCV_VIDEO_OK;
    } catch (const cv::Exception &e) {
        return fail(OPENCV_VIDEO_ERROR_OPENCV, e.what());
    } catch (const std::exception &e) {
        return fail(OPENCV_VIDEO_ERROR_STANDARD, e.what());
    } catch (...) {
        return fail(OPENCV_VIDEO_ERROR_UNKNOWN, "Unknown Farneback exception");
    }
}
}  // namespace

extern "C" opencv_video_status opencv_video_calc_farneback_flow(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    opencv_core_mat_handle *flow,
    double pyramid_scale, int32_t levels, int32_t window_size,
    int32_t iterations, int32_t poly_neighborhood, double poly_sigma) {
    return calc_farneback(previous_image, next_image, nullptr, false, flow, pyramid_scale,
                          levels, window_size, iterations, poly_neighborhood, poly_sigma);
}

extern "C" opencv_video_status opencv_video_calc_farneback_flow_seeded(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *initial_flow,
    opencv_core_mat_handle *result_flow,
    double pyramid_scale, int32_t levels, int32_t window_size,
    int32_t iterations, int32_t poly_neighborhood, double poly_sigma) {
    return calc_farneback(previous_image, next_image, initial_flow, true, result_flow,
                          pyramid_scale, levels, window_size, iterations, poly_neighborhood,
                          poly_sigma);
}
