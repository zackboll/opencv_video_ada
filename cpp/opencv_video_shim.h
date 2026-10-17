#ifndef OPENCV_VIDEO_ADA_SHIM_H
#define OPENCV_VIDEO_ADA_SHIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t opencv_video_status;
typedef struct opencv_core_mat_handle opencv_core_mat_handle;
typedef struct opencv_video_pyramid_handle opencv_video_pyramid_handle;

#define OPENCV_VIDEO_OK ((opencv_video_status)0)
#define OPENCV_VIDEO_ERROR_INVALID_ARGUMENT ((opencv_video_status)1)
#define OPENCV_VIDEO_ERROR_CORE_BRIDGE ((opencv_video_status)2)
#define OPENCV_VIDEO_ERROR_OPENCV ((opencv_video_status)3)
#define OPENCV_VIDEO_ERROR_STANDARD ((opencv_video_status)4)
#define OPENCV_VIDEO_ERROR_UNKNOWN ((opencv_video_status)5)

const char *opencv_video_native_version(void);
const char *opencv_video_native_backend(void);
const char *opencv_video_last_error(void);

/* Owned image/derivative pairs. Create always clears *out before work; destroy
 * accepts null. No borrowed Core storage survives creation. Metadata rejects
 * null objects and null outputs. Live handles must not be destroyed during use. */
opencv_video_status opencv_video_pyramid_create(
    const opencv_core_mat_handle *image, int32_t window_width,
    int32_t window_height, int32_t requested_max_level,
    opencv_video_pyramid_handle **out);
void opencv_video_pyramid_destroy(opencv_video_pyramid_handle *pyramid);
opencv_video_status opencv_video_pyramid_metadata(
    const opencv_video_pyramid_handle *pyramid, int32_t *window_width,
    int32_t *window_height, int32_t *requested_max_level,
    int32_t *available_max_level);
opencv_video_status opencv_video_track_pyr_lk_pyramids(
    const opencv_video_pyramid_handle *previous,
    const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *previous_points,
    opencv_core_mat_handle *next_points, opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *track_error,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold);

/* Owned pyramid photometric tracking, flags exactly 4. Same seed/private-clone
 * and failure-atomic publication contract as the raw-image seeded export. */
opencv_video_status opencv_video_track_pyr_lk_pyramids_seeded(
    const opencv_video_pyramid_handle *previous,
    const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    opencv_core_mat_handle *next_points, opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *track_error,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold);

/* Quality modes: flags 8/12, strict initialized-storage contract. */
opencv_video_status opencv_video_track_pyr_lk_pyramids_min_eigenvalues(
    const opencv_video_pyramid_handle *previous,
    const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *previous_points,
    opencv_core_mat_handle *next_points, opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *minimum_eigenvalues,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold);

/* Owned pyramid quality tracking, flags exactly 12. Same seed/private-clone
 * and failure-atomic publication contract as the raw-image seeded export. */
opencv_video_status opencv_video_track_pyr_lk_pyramids_seeded_min_eigenvalues(
    const opencv_video_pyramid_handle *previous,
    const opencv_video_pyramid_handle *next,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    opencv_core_mat_handle *next_points, opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *minimum_eigenvalues,
    int32_t window_width, int32_t window_height, int32_t max_level,
    int32_t maximum_iterations, double epsilon, double min_eigenvalue_threshold);

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

/* Same contract, using only OPTFLOW_USE_INITIAL_FLOW. Seeds are read-only,
 * continuous N x 1 CV_32FC2 with matching count and finite |coordinate| <= 2^29.
 * Input headers may alias each other; no output header may alias any input.
 * Native nextPts is a private clone, never an application allocation. */
opencv_video_status opencv_video_track_pyr_lk_seeded(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *track_error,
    int32_t window_width,
    int32_t window_height,
    int32_t max_level,
    int32_t maximum_iterations,
    double epsilon,
    double min_eigenvalue_threshold);

/* Quality exports use flags 8 and 12 respectively. Failed points normalize to
 * previous points, but quality is preserved independently of status. Every slot
 * must be written, finite and nonnegative; otherwise outputs remain unchanged. */
opencv_video_status opencv_video_track_pyr_lk_min_eigenvalues(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *minimum_eigenvalues,
    int32_t window_width,
    int32_t window_height,
    int32_t max_level,
    int32_t maximum_iterations,
    double epsilon,
    double min_eigenvalue_threshold);

opencv_video_status opencv_video_track_pyr_lk_seeded_min_eigenvalues(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *previous_points,
    const opencv_core_mat_handle *initial_next_points,
    opencv_core_mat_handle *next_points,
    opencv_core_mat_handle *track_status,
    opencv_core_mat_handle *minimum_eigenvalues,
    int32_t window_width,
    int32_t window_height,
    int32_t max_level,
    int32_t maximum_iterations,
    double epsilon,
    double min_eigenvalue_threshold);

/* Dense Farneback flow (Task 009): flags exactly zero, Float32 C2 full-resolution
 * output. The flow is computed into a private NaN-sentinel Mat, validated finite,
 * and published to *flow only on success (failure leaves *flow unchanged). */
opencv_video_status opencv_video_calc_farneback_flow(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    opencv_core_mat_handle *flow,
    double pyramid_scale, int32_t levels, int32_t window_size,
    int32_t iterations, int32_t poly_neighborhood, double poly_sigma);

/* Seeded dense Farneback (Task 010): flags exactly OPTFLOW_USE_INITIAL_FLOW (4).
 * initial_flow is a read-only 2-D Float32 C2 Mat with the image geometry; every
 * component must be finite with |value| <= 2^20 pixels (binding policy). It is
 * cloned into private continuous storage before native mutation, the caller's seed
 * is never modified, and result_flow may alias no input header. Publication is
 * failure-atomic as for opencv_video_calc_farneback_flow. */
opencv_video_status opencv_video_calc_farneback_flow_seeded(
    const opencv_core_mat_handle *previous_image,
    const opencv_core_mat_handle *next_image,
    const opencv_core_mat_handle *initial_flow,
    opencv_core_mat_handle *result_flow,
    double pyramid_scale, int32_t levels, int32_t window_size,
    int32_t iterations, int32_t poly_neighborhood, double poly_sigma);

/* ECC translation alignment (Task 011): cv::findTransformECC with MOTION_TRANSLATION,
 * a private identity 2x3 Float32 warp, COUNT|EPS criteria, no mask, Gaussian size 5.
 * The native warp maps template coordinates to moving coordinates:
 * moving = template + (displacement_x, displacement_y). Not an inverse warp.
 * Images: nonempty 2-D UInt8 C1, equal geometry, 32..16384 rows/columns, area
 * <= 2^24. maximum_iterations 1..100, epsilon finite in [1e-8, 0.1].
 * *result is written only after every native and validation step succeeds; on any
 * failure it is left unchanged. Native nonconvergence is OPENCV_VIDEO_ERROR_OPENCV. */
typedef struct {
    float displacement_x;
    float displacement_y;
    double correlation;
} opencv_video_ecc_translation_result;

opencv_video_status opencv_video_estimate_ecc_translation(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    opencv_video_ecc_translation_result *result);

/* ECC Euclidean alignment (Task 012): cv::findTransformECC with MOTION_EUCLIDEAN, a
 * private identity 2x3 Float32 warp, COUNT|EPS criteria, no mask, Gaussian size 5.
 * The native warp maps template coordinates to moving coordinates:
 *   moving_x = cos(t) * x - sin(t) * y + displacement_x
 *   moving_y = sin(t) * x + cos(t) * y + displacement_y
 * (X right, Y down; rotation about the coordinate origin, not the image centre).
 * rotation_radians = atan2(warp[1][0], warp[0][0]) of the returned warp, in [-pi, pi].
 * Input and option envelope is identical to opencv_video_estimate_ecc_translation.
 * The returned warp must be a finite rotation (a = d, b = -c, unit columns,
 * determinant +1 within Float32 tolerance) or the call fails.
 * *result is written only after every native and validation step succeeds; on any
 * failure it is left unchanged. Native nonconvergence is OPENCV_VIDEO_ERROR_OPENCV. */
typedef struct {
    float displacement_x;
    float displacement_y;
    double rotation_radians;
    double correlation;
} opencv_video_ecc_euclidean_result;

opencv_video_status opencv_video_estimate_ecc_euclidean(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    opencv_video_ecc_euclidean_result *result);

/* Caller-seeded ECC Euclidean alignment (Task 013). Identical to
 * opencv_video_estimate_ecc_euclidean except the private 2x3 Float32 warp starts as
 *   [cos(r) -sin(r) tx; sin(r) cos(r) ty]
 * with r = initial_rotation_radians, tx/ty = initial_translation_x/y (template -> moving,
 * rotation about the coordinate origin). The seed is passed by value; nothing is retained.
 * Seed domain (binding policy, not an OpenCV guarantee): translations finite with
 * |value| <= 2^20, rotation finite with |value| <= 1 radian (inside the principal asin
 * branch of the native update). Violations return OPENCV_VIDEO_ERROR_INVALID_ARGUMENT and
 * are never clamped. An accepted seed does not imply convergence or a correct optimum.
 * *result is written only after every native and validation step succeeds. */
opencv_video_status opencv_video_estimate_ecc_euclidean_seeded(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    float initial_translation_x,
    float initial_translation_y,
    double initial_rotation_radians,
    opencv_video_ecc_euclidean_result *result);

/* ECC six-parameter affine alignment (Task 014): cv::findTransformECC with MOTION_AFFINE,
 * a private identity 2x3 Float32 warp, COUNT|EPS criteria, no mask, Gaussian size 5.
 * The native warp maps template coordinates to moving coordinates (X right, Y down):
 *   moving_x = m00 * x + m01 * y + m02
 *   moving_y = m10 * x + m11 * y + m12
 * The six row-major native coefficients are returned without decomposition; apply the
 * same matrix with WARP_INVERSE_MAP to resample the moving image into the template frame.
 * Input and option envelope is identical to opencv_video_estimate_ecc_translation.
 * Every coefficient and the correlation must be finite. No rigidity, orthogonality or
 * determinant policy is applied: scale, shear, reflection and near-singular finite
 * matrices that native ECC returns are accepted (docs/ecc-affine-source-contract.md).
 * *result is written only after every native and validation step succeeds; on any
 * failure it is left unchanged. Native nonconvergence is OPENCV_VIDEO_ERROR_OPENCV.
 * Layout on qualified targets: offsets 0,4,8,12,16,20,24; size 32; alignment 8. */
typedef struct {
    float m00;
    float m01;
    float m02;
    float m10;
    float m11;
    float m12;
    double correlation;
} opencv_video_ecc_affine_result;

#if defined(__cplusplus)
static_assert(sizeof(opencv_video_ecc_affine_result) == 32 &&
              alignof(opencv_video_ecc_affine_result) == 8 &&
              offsetof(opencv_video_ecc_affine_result, m00) == 0 &&
              offsetof(opencv_video_ecc_affine_result, m01) == 4 &&
              offsetof(opencv_video_ecc_affine_result, m02) == 8 &&
              offsetof(opencv_video_ecc_affine_result, m10) == 12 &&
              offsetof(opencv_video_ecc_affine_result, m11) == 16 &&
              offsetof(opencv_video_ecc_affine_result, m12) == 20 &&
              offsetof(opencv_video_ecc_affine_result, correlation) == 24,
              "opencv_video_ecc_affine_result layout");
#else
_Static_assert(sizeof(opencv_video_ecc_affine_result) == 32 &&
               _Alignof(opencv_video_ecc_affine_result) == 8 &&
               offsetof(opencv_video_ecc_affine_result, m00) == 0 &&
               offsetof(opencv_video_ecc_affine_result, m01) == 4 &&
               offsetof(opencv_video_ecc_affine_result, m02) == 8 &&
               offsetof(opencv_video_ecc_affine_result, m10) == 12 &&
               offsetof(opencv_video_ecc_affine_result, m11) == 16 &&
               offsetof(opencv_video_ecc_affine_result, m12) == 20 &&
               offsetof(opencv_video_ecc_affine_result, correlation) == 24,
               "opencv_video_ecc_affine_result layout");
#endif

opencv_video_status opencv_video_estimate_ecc_affine(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    opencv_video_ecc_affine_result *result);

/* ECC eight-parameter homography alignment (Task 016): cv::findTransformECC with
 * MOTION_HOMOGRAPHY, a private identity 3x3 Float32 warp, COUNT|EPS criteria, no mask,
 * Gaussian size 5. The native warp maps template coordinates to moving coordinates
 * (X right, Y down) with homogeneous division:
 *   d = h20 * x + h21 * y + h22
 *   moving_x = (h00 * x + h01 * y + h02) / d
 *   moving_y = (h10 * x + h11 * y + h12) / d
 * The nine row-major native storage-order coefficients are returned (not the native
 * update-vector order); apply the same matrix with WARP_INVERSE_MAP to resample. The
 * native update never writes h22, so it is exactly 1 for this identity-initialized call;
 * a different h22 is reported as OPENCV_VIDEO_ERROR_OPENCV. Input and option envelope is
 * identical to opencv_video_estimate_ecc_translation. All nine coefficients and the
 * correlation must be finite. No determinant, scale, perspective or conditioning policy is
 * applied, and mapped points with d == 0 are not detected (docs/ecc-homography-source-contract.md).
 * *result is written only after every native and validation step succeeds; on any
 * failure it is left unchanged. Native nonconvergence is OPENCV_VIDEO_ERROR_OPENCV.
 * Layout on qualified targets: offsets 0,4,...,32 for h00..h22, correlation 40; size 48;
 * alignment 8 (four padding bytes before correlation). */
typedef struct {
    float h00;
    float h01;
    float h02;
    float h10;
    float h11;
    float h12;
    float h20;
    float h21;
    float h22;
    double correlation;
} opencv_video_ecc_homography_result;

#if defined(__cplusplus)
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
              "opencv_video_ecc_homography_result layout");
#else
_Static_assert(sizeof(opencv_video_ecc_homography_result) == 48 &&
               _Alignof(opencv_video_ecc_homography_result) == 8 &&
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
               "opencv_video_ecc_homography_result layout");
#endif

opencv_video_status opencv_video_estimate_ecc_homography(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    opencv_video_ecc_homography_result *result);

/* Caller-seeded affine ECC (Task 015). Identical to opencv_video_estimate_ecc_affine
 * except the private 2x3 Float32 warp starts from the six row-major coefficients
 * (native matrix storage order, NOT the native update-vector order). Each coefficient
 * must be finite; m00,m01,m10,m11 have magnitude <= 4 and m02,m12 magnitude <= 2^20
 * (binding policy, docs/ecc-affine-seeded-source-contract.md). Violations return
 * OPENCV_VIDEO_ERROR_INVALID_ARGUMENT before any native call. The seed is never
 * transposed, negated, inverted or composed. *result is unchanged on every failure. */
opencv_video_status opencv_video_estimate_ecc_affine_seeded(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    float initial_m00,
    float initial_m01,
    float initial_m02,
    float initial_m10,
    float initial_m11,
    float initial_m12,
    opencv_video_ecc_affine_result *result);

/* Caller-seeded homography ECC (Task 017). Identical to opencv_video_estimate_ecc_homography
 * except the private 3x3 Float32 warp starts from the nine row-major coefficients (native
 * matrix storage order, NOT the native update-vector order). Seed domain (binding policy,
 * docs/ecc-homography-seeded-source-contract.md), checked after the image envelope:
 *   every coefficient finite; h22 exactly 1; h00,h01,h10,h11 magnitude <= 4;
 *   h02,h12 magnitude <= 2^20;
 *   fabs(h20)*(columns-1) + fabs(h21)*(rows-1) <= 0.5, evaluated in double.
 * Violations return OPENCV_VIDEO_ERROR_INVALID_ARGUMENT before any native call. The seed is
 * never transposed, negated, inverted, normalized or composed. The output policy is that of
 * opencv_video_estimate_ecc_homography; the seed domain is not applied to the output.
 * *result is unchanged on every failure. */
opencv_video_status opencv_video_estimate_ecc_homography_seeded(
    const opencv_core_mat_handle *template_image,
    const opencv_core_mat_handle *moving_image,
    int32_t maximum_iterations,
    double epsilon,
    float initial_h00,
    float initial_h01,
    float initial_h02,
    float initial_h10,
    float initial_h11,
    float initial_h12,
    float initial_h20,
    float initial_h21,
    float initial_h22,
    opencv_video_ecc_homography_result *result);

#ifdef __cplusplus
}
#endif

#endif
