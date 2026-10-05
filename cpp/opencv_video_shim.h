#ifndef OPENCV_VIDEO_ADA_SHIM_H
#define OPENCV_VIDEO_ADA_SHIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t opencv_video_status;
typedef struct opencv_core_mat_handle opencv_core_mat_handle;

#define OPENCV_VIDEO_OK ((opencv_video_status)0)
#define OPENCV_VIDEO_ERROR_INVALID_ARGUMENT ((opencv_video_status)1)
#define OPENCV_VIDEO_ERROR_CORE_BRIDGE ((opencv_video_status)2)
#define OPENCV_VIDEO_ERROR_OPENCV ((opencv_video_status)3)
#define OPENCV_VIDEO_ERROR_STANDARD ((opencv_video_status)4)
#define OPENCV_VIDEO_ERROR_UNKNOWN ((opencv_video_status)5)

const char *opencv_video_native_version(void);
const char *opencv_video_native_backend(void);
const char *opencv_video_last_error(void);

/* Handles must be real live Core handles. Output headers must be distinct from
 * each other and all inputs. On failure, existing outputs are unchanged (not
 * cleared); Ada supplies initially empty outputs. On successful empty points,
 * all outputs are cleared. Points are continuous N x 1 CV_32FC2, including a
 * typed empty Mat. Failed slots publish the input point and zero error without
 * reading undefined native slots. Borrowed Core pointers are never retained.
 */
opencv_video_status opencv_video_track_pyr_lk(
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
    double min_eigenvalue_threshold);

#ifdef __cplusplus
}
#endif

#endif
