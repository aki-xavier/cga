/* ccga renderer — minimal sphere ray tracer on mlx-c. */
#include "ccga/render.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "mlx/c/mlx.h"

ccga_camera ccga_camera_from_fov(float fov_rad, float aspect, int w, int h) {
  ccga_camera cam;
  cam.fy = (float)h / (2.0f * tanf(fov_rad / 2.0f));
  cam.fx = cam.fy * aspect;
  cam.cx = (float)(w - 1) / 2.0f;
  cam.cy = (float)(h - 1) / 2.0f;
  return cam;
}

/* Build a float32 array from host data. */
static mlx_array mk_f32(const float *data, const int *shape, int dim) {
  return mlx_array_new_data(data, shape, dim, MLX_FLOAT32);
}

/* Binary elementwise op wrapper (add/subtract/multiply/divide/maximum/minimum). */
typedef int (*binop_t)(mlx_array *, const mlx_array, const mlx_array, const mlx_stream);

static mlx_array binop(binop_t fn, mlx_array a, mlx_array b, mlx_stream s) {
  mlx_array r = mlx_array_new();
  fn(&r, a, b, s);
  return r;
}

/* Unary op wrapper. */
typedef int (*unop_t)(mlx_array *, const mlx_array, const mlx_stream);

static mlx_array unop(unop_t fn, mlx_array a, mlx_stream s) {
  mlx_array r = mlx_array_new();
  fn(&r, a, s);
  return r;
}

/* Sum over the last axis with keepdims, returning shape (...,1). */
static mlx_array sum_last(mlx_array a, mlx_stream s, int keepdims) {
  mlx_array r = mlx_array_new();
  mlx_sum_axis(&r, a, -1, keepdims, s);
  return r;
}

int ccga_render_spheres(
    const ccga_camera *cam,
    const ccga_sphere *spheres,
    size_t num_spheres,
    const float bg[3],
    int w,
    int h,
    float *out_rgb) {
  if (num_spheres == 0) {
    /* fill background */
    for (int i = 0; i < w * h; i++) {
      out_rgb[i * 3 + 0] = bg[0];
      out_rgb[i * 3 + 1] = bg[1];
      out_rgb[i * 3 + 2] = bg[2];
    }
    return 0;
  }

  const int n = w * h;
  mlx_stream s = mlx_default_gpu_stream_new();

  /* Build ray directions on host. */
  float *dirs = (float *)malloc((size_t)n * 3 * sizeof(float));
  if (dirs == NULL) return 1;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      float dx = ((float)x - cam->cx) / cam->fx;
      float dy = ((float)y - cam->cy) / cam->fy;
      float dz = 1.0f;
      float inv = 1.0f / sqrtf(dx * dx + dy * dy + dz * dz);
      dirs[(y * w + x) * 3 + 0] = dx * inv;
      dirs[(y * w + x) * 3 + 1] = dy * inv;
      dirs[(y * w + x) * 3 + 2] = dz * inv;
    }
  }

  int shp_n3[2] = {n, 3};
  int shp_n[1] = {n};
  int shp_3[1] = {3};

  mlx_array d = mk_f32(dirs, shp_n3, 2);
  mlx_array o = mlx_array_new();
  mlx_zeros(&o, shp_n3, 2, MLX_FLOAT32, s);

  /* best_t (N,) = +inf; best_idx (N,) int32 = 0 */
  mlx_array inf_scalar = mlx_array_new_float(INFINITY);
  mlx_array best_t = mlx_array_new();
  mlx_full(&best_t, shp_n, 1, inf_scalar, MLX_FLOAT32, s);
  mlx_array_free(inf_scalar);
  mlx_array best_idx = mlx_array_new();
  mlx_zeros(&best_idx, shp_n, 1, MLX_INT32, s);

  /* sphere centers/radii as (S,3)/(S,) for take */
  float *centers = (float *)malloc(num_spheres * 3 * sizeof(float));
  float *radii = (float *)malloc(num_spheres * sizeof(float));
  float *colors = (float *)malloc(num_spheres * 3 * sizeof(float));
  for (size_t i = 0; i < num_spheres; i++) {
    centers[i * 3 + 0] = spheres[i].c[0];
    centers[i * 3 + 1] = spheres[i].c[1];
    centers[i * 3 + 2] = spheres[i].c[2];
    radii[i] = spheres[i].r;
    colors[i * 3 + 0] = spheres[i].color[0];
    colors[i * 3 + 1] = spheres[i].color[1];
    colors[i * 3 + 2] = spheres[i].color[2];
  }

  for (size_t i = 0; i < num_spheres; i++) {
    /* c broadcast (3,) */
    float c3[3] = {spheres[i].c[0], spheres[i].c[1], spheres[i].c[2]};
    mlx_array c = mk_f32(c3, shp_3, 1);

    mlx_array oc = binop(mlx_subtract, o, c, s);
    /* b = 2 * (oc·d) : elementwise mul, sum last axis (keepdims -> (N,1)) */
    mlx_array ocd = binop(mlx_multiply, oc, d, s);
    mlx_array dot = sum_last(ocd, s, 1); /* (N,1) */
    /* squeeze to (N,) */
    mlx_array dot_n = mlx_array_new();
    mlx_squeeze(&dot_n, dot, s);
    mlx_array two = mlx_array_new_float(2.0f);
    mlx_array b = binop(mlx_multiply, two, dot_n, s);

    /* cq = |oc|^2 - r^2 */
    mlx_array oc2 = binop(mlx_multiply, oc, oc, s);
    mlx_array len2 = sum_last(oc2, s, 0); /* (N,) */
    mlx_array r2 = mlx_array_new_float(spheres[i].r * spheres[i].r);
    mlx_array cq = binop(mlx_subtract, len2, r2, s);

    /* disc = b^2 - 4 cq */
    mlx_array b2 = binop(mlx_multiply, b, b, s);
    mlx_array four = mlx_array_new_float(4.0f);
    mlx_array fourcq = binop(mlx_multiply, four, cq, s);
    mlx_array disc = binop(mlx_subtract, b2, fourcq, s);

    mlx_array sq = mlx_array_new();
    mlx_sqrt(&sq, disc, s);
    mlx_array nb = unop(mlx_negative, b, s);
    mlx_array num = binop(mlx_subtract, nb, sq, s);
    mlx_array two2 = mlx_array_new_float(2.0f);
    mlx_array t = binop(mlx_divide, num, two2, s);

    /* mask = disc > 0 */
    mlx_array zero = mlx_array_new_float(0.0f);
    mlx_array mask = binop(mlx_greater, disc, zero, s);

    /* nearer = mask && t < best_t */
    mlx_array lt = binop(mlx_less, t, best_t, s);
    mlx_array nearer = binop(mlx_logical_and, mask, lt, s);

    /* best_t = where(nearer, t, best_t) */
    mlx_array new_best_t = mlx_array_new();
    mlx_where(&new_best_t, nearer, t, best_t, s);
    /* best_idx = where(nearer, i, best_idx) */
    mlx_array idx = mlx_array_new_int((int)i);
    mlx_array new_best_idx = mlx_array_new();
    mlx_where(&new_best_idx, nearer, idx, best_idx, s);

    mlx_array_free(best_t);
    mlx_array_free(best_idx);
    best_t = new_best_t;
    best_idx = new_best_idx;

    mlx_array_free(c);
    mlx_array_free(oc);
    mlx_array_free(ocd);
    mlx_array_free(dot);
    mlx_array_free(dot_n);
    mlx_array_free(two);
    mlx_array_free(b);
    mlx_array_free(oc2);
    mlx_array_free(len2);
    mlx_array_free(r2);
    mlx_array_free(cq);
    mlx_array_free(b2);
    mlx_array_free(four);
    mlx_array_free(fourcq);
    mlx_array_free(disc);
    mlx_array_free(sq);
    mlx_array_free(nb);
    mlx_array_free(num);
    mlx_array_free(two2);
    mlx_array_free(t);
    mlx_array_free(zero);
    mlx_array_free(mask);
    mlx_array_free(lt);
    mlx_array_free(nearer);
    mlx_array_free(idx);
  }

  /* color = take(colors(S,3), best_idx) -> (N,3) */
  int shp_s3[2] = {(int)num_spheres, 3};
  mlx_array col_arr = mk_f32(colors, shp_s3, 2);
  mlx_array color = mlx_array_new();
  mlx_take_axis(&color, col_arr, best_idx, 0, s);

  /* hit = isfinite(best_t); out = where(hit[:,None], color, bg) */
  mlx_array hit = mlx_array_new();
  mlx_isfinite(&hit, best_t, s);
  mlx_array hit2 = mlx_array_new();
  mlx_expand_dims(&hit2, hit, -1, s);
  mlx_array bg_arr = mk_f32(bg, shp_3, 1);
  mlx_array out = mlx_array_new();
  mlx_where(&out, hit2, color, bg_arr, s);

  mlx_array_eval(out);
  const float *res = mlx_array_data_float32(out);
  if (res != NULL) memcpy(out_rgb, res, (size_t)n * 3 * sizeof(float));

  /* cleanup */
  mlx_array_free(d);
  mlx_array_free(o);
  mlx_array_free(best_t);
  mlx_array_free(best_idx);
  mlx_array_free(col_arr);
  mlx_array_free(color);
  mlx_array_free(hit);
  mlx_array_free(hit2);
  mlx_array_free(bg_arr);
  mlx_array_free(out);
  mlx_stream_free(s);
  free(dirs);
  free(centers);
  free(radii);
  free(colors);
  return res == NULL ? 1 : 0;
}
