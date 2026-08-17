/* ccga renderer — minimal mlx-c ray tracer foundation.
 *
 * Camera convention (matches cga/engine): X right, Y down, Z forward, camera
 * at origin. Pinhole: col = fx·X/Z + cx, row = fy·Y/Z + cy, pixel centers are
 * integer indices (no +0.5).
 */
#ifndef CCGA_RENDER_H
#define CCGA_RENDER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float fx, fy, cx, cy;
} ccga_camera;

/* Build a camera from vertical FOV (radians), aspect (W/H), and image size. */
ccga_camera ccga_camera_from_fov(float fov_rad, float aspect, int w, int h);

/* A sphere in camera space (center, radius, linear RGB color 0..1). */
typedef struct {
  float c[3];
  float r;
  float color[3];
} ccga_sphere;

/* Ray-trace a list of camera-space spheres into an H*W*3 linear-RGB buffer.
 * out_rgb is row-major (H*W*3). Returns 0 on success, nonzero on error. */
int ccga_render_spheres(
    const ccga_camera *cam,
    const ccga_sphere *spheres,
    size_t num_spheres,
    const float bg[3],
    int w,
    int h,
    float *out_rgb);

#ifdef __cplusplus
}
#endif

#endif /* CCGA_RENDER_H */
