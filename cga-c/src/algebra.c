/* ccga — primitives and motors implementation. */
#include "ccga/algebra.h"

#include <math.h>
#include <string.h>

/* ── small 3x3 helpers (row-major) ─────────────────────────────── */

static void mat3_mul_vec3(const float m[9], const float v[3], float out[3]) {
  out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
  out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
  out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

static void mat3_mul_mat3(const float a[9], const float b[9], float out[9]) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      out[r * 3 + c] = a[r * 3 + 0] * b[0 * 3 + c] +
                       a[r * 3 + 1] * b[1 * 3 + c] +
                       a[r * 3 + 2] * b[2 * 3 + c];
    }
  }
}

static void mat3_identity(float out[9]) {
  memset(out, 0, 9 * sizeof(float));
  out[0] = out[4] = out[8] = 1.0f;
}

/* ── Primitives ────────────────────────────────────────────────── */

ccga_mv ccga_point(float x, float y, float z) {
  float r2 = x * x + y * y + z * z;
  return ccga_mv_vector(x, y, z, 1.0f, 0.5f * r2);
}

void ccga_point_coords(const ccga_mv *p, float out[3]) {
  float w = p->c[4];
  out[0] = p->c[1] / w;
  out[1] = p->c[2] / w;
  out[2] = p->c[3] / w;
}

ccga_mv ccga_point_pair(ccga_mv p1, ccga_mv p2) { return ccga_mv_op(p1, p2); }

ccga_mv ccga_line(ccga_mv p1, ccga_mv p2) {
  ccga_mv einf = ccga_mv_vector(0, 0, 0, 0, 1);
  return ccga_mv_op(ccga_mv_op(p1, p2), einf);
}

ccga_mv ccga_plane(float nx, float ny, float nz, float d) {
  float nl = sqrtf(nx * nx + ny * ny + nz * nz);
  float ux = nx / nl, uy = ny / nl, uz = nz / nl;
  return ccga_mv_vector(ux, uy, uz, 0.0f, d);
}

float ccga_plane_dist(const ccga_mv *plane, float x, float y, float z) {
  float nx = plane->c[1], ny = plane->c[2], nz = plane->c[3];
  float d = plane->c[5];
  float nl = sqrtf(nx * nx + ny * ny + nz * nz);
  if (nl < 1e-12f) return INFINITY;
  return (nx * x + ny * y + nz * z - d) / nl;
}

ccga_mv ccga_sphere(float cx, float cy, float cz, float radius) {
  ccga_mv p = ccga_point(cx, cy, cz);
  ccga_mv tail = ccga_mv_vector(0, 0, 0, 0, 0.5f * radius * radius);
  return ccga_mv_sub(p, tail);
}

void ccga_sphere_from_dual(const ccga_mv *s, float out_center[3], float *out_radius) {
  float w = s->c[4];
  float v1 = s->c[1], v2 = s->c[2], v3 = s->c[3];
  float f = s->c[5];
  float cx = v1 / w, cy = v2 / w, cz = v3 / w;
  float rho_sq = (v1 * v1 + v2 * v2 + v3 * v3) / (w * w) - 2.0f * f / w;
  out_center[0] = cx;
  out_center[1] = cy;
  out_center[2] = cz;
  *out_radius = sqrtf(rho_sq > 0 ? rho_sq : 0.0f);
}

ccga_mv ccga_circle(
    float cx, float cy, float cz, float radius, float nx, float ny, float nz) {
  ccga_mv s = ccga_sphere(cx, cy, cz, radius);
  float nl = sqrtf(nx * nx + ny * ny + nz * nz);
  float d_raw = cx * nx + cy * ny + cz * nz;
  float d = nl > 1e-12f ? d_raw / nl : 0.0f;
  ccga_mv p = ccga_plane(nx, ny, nz, d);
  return ccga_mv_op(s, p);
}

ccga_cylinder ccga_cylinder_make(
    float ax, float ay, float az, float dx, float dy, float dz, float radius) {
  ccga_cylinder cyl;
  float al = sqrtf(dx * dx + dy * dy + dz * dz);
  float ux = dx / al, uy = dy / al, uz = dz / al;
  ccga_mv q = ccga_point(ax, ay, az);
  ccga_mv q2 = ccga_point(ax + ux, ay + uy, az + uz);
  cyl.blade = ccga_line(q, q2);
  cyl.radius = radius;
  cyl.axis_dir[0] = ux;
  cyl.axis_dir[1] = uy;
  cyl.axis_dir[2] = uz;
  cyl.axis_point[0] = ax;
  cyl.axis_point[1] = ay;
  cyl.axis_point[2] = az;
  return cyl;
}

/* ── Dupin cyclide ─────────────────────────────────────────────── */

ccga_cyclide ccga_cyclide_make(float a, float b, float d, float sx, float sy, float sz) {
  ccga_cyclide cy;
  cy.a = a;
  cy.b = b;
  cy.d = d;
  cy.shift[0] = sx;
  cy.shift[1] = sy;
  cy.shift[2] = sz;
  return cy;
}

float ccga_cyclide_c(const ccga_cyclide *cy) {
  return sqrtf(cy->a * cy->a - cy->b * cy->b);
}

float ccga_cyclide_implicit(const ccga_cyclide *cy, float x, float y, float z) {
  float a = cy->a, b = cy->b, d = cy->d;
  float c = sqrtf(a * a - b * b);
  x -= cy->shift[0];
  y -= cy->shift[1];
  z -= cy->shift[2];
  float B = b * b - d * d;
  float rho = x * x + y * y + z * z;
  float t = rho + B;
  return t * t - 4.0f * (a * x - c * d) * (a * x - c * d) - 4.0f * b * b * y * y;
}

void ccga_cyclide_gradient(const ccga_cyclide *cy, float x, float y, float z, float out[3]) {
  float a = cy->a, b = cy->b, d = cy->d;
  float c = sqrtf(a * a - b * b);
  x -= cy->shift[0];
  y -= cy->shift[1];
  z -= cy->shift[2];
  float B = b * b - d * d;
  float rho = x * x + y * y + z * z;
  float g = rho + B;
  out[0] = 4.0f * x * g - 8.0f * a * (a * x - c * d);
  out[1] = 4.0f * y * g - 8.0f * b * b * y;
  out[2] = 4.0f * z * g;
}

void ccga_cyclide_normal(const ccga_cyclide *cy, float x, float y, float z, float out[3]) {
  float g[3];
  ccga_cyclide_gradient(cy, x, y, z, g);
  float n = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  if (n < 1e-12f) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 1;
    return;
  }
  out[0] = g[0] / n;
  out[1] = g[1] / n;
  out[2] = g[2] / n;
}

int ccga_cyclide_contains(const ccga_cyclide *cy, float x, float y, float z) {
  return ccga_cyclide_implicit(cy, x, y, z) < 0.0f;
}

/* ── Motors ────────────────────────────────────────────────────── */

ccga_mv ccga_motor_identity(void) { return ccga_mv_scalar(1.0f); }

ccga_mv ccga_motor_rotor(float ax, float ay, float az, float angle) {
  float nl = sqrtf(ax * ax + ay * ay + az * az);
  if (nl < 1e-12f) return ccga_motor_identity();
  float ux = ax / nl, uy = ay / nl, uz = az / nl;
  float half = angle / 2.0f;
  float s = cosf(half);
  float sf = sinf(half);
  ccga_mv v = ccga_mv_zero();
  v.c[0] = s;
  v.c[CCGA_E12] = -sf * uz;
  v.c[CCGA_E13] = sf * uy;
  v.c[CCGA_E23] = -sf * ux;
  return v;
}

ccga_mv ccga_motor_translator(float tx, float ty, float tz) {
  ccga_mv tv = ccga_mv_vector(tx, ty, tz, 0, 0);
  ccga_mv einf = ccga_mv_vector(0, 0, 0, 0, 1);
  ccga_mv wedge = ccga_mv_op(tv, einf);
  return ccga_mv_sub(ccga_mv_scalar(1.0f), ccga_mv_scale(wedge, 0.5f));
}

void ccga_matrix_to_quaternion(const float m[9], float out[4]) {
  float trace = m[0] + m[4] + m[8];
  if (trace > 0.0f) {
    float s = sqrtf(trace + 1.0f) * 2.0f;
    out[0] = 0.25f * s;
    out[1] = (m[7] - m[5]) / s;
    out[2] = (m[2] - m[6]) / s;
    out[3] = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    float s = sqrtf(1.0f + m[0] - m[4] - m[8]) * 2.0f;
    out[0] = (m[7] - m[5]) / s;
    out[1] = 0.25f * s;
    out[2] = (m[1] + m[3]) / s;
    out[3] = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    float s = sqrtf(1.0f + m[4] - m[0] - m[8]) * 2.0f;
    out[0] = (m[2] - m[6]) / s;
    out[1] = (m[1] + m[3]) / s;
    out[2] = 0.25f * s;
    out[3] = (m[5] + m[7]) / s;
  } else {
    float s = sqrtf(1.0f + m[8] - m[0] - m[4]) * 2.0f;
    out[0] = (m[3] - m[1]) / s;
    out[1] = (m[2] + m[6]) / s;
    out[2] = (m[5] + m[7]) / s;
    out[3] = 0.25f * s;
  }
}

ccga_mv ccga_motor_from_quaternion(float w, float x, float y, float z) {
  float n = sqrtf(w * w + x * x + y * y + z * z);
  if (n < 1e-12f) return ccga_motor_identity();
  w /= n;
  x /= n;
  y /= n;
  z /= n;
  float angle = 2.0f * atan2f(sqrtf(x * x + y * y + z * z), w);
  return ccga_motor_rotor(x, y, z, angle);
}

ccga_mv ccga_motor_from_matrix(const float R[9], const float t[3]) {
  float q[4];
  ccga_matrix_to_quaternion(R, q);
  ccga_mv rot = ccga_motor_from_quaternion(q[0], q[1], q[2], q[3]);
  ccga_mv tr = ccga_motor_translator(t[0], t[1], t[2]);
  return ccga_mv_gp(tr, rot);
}

void ccga_motor_to_matrix(const ccga_mv *m, float out[16]) {
  ccga_mv e0 = ccga_mv_vector(0, 0, 0, 1, 0);
  ccga_mv origin = ccga_motor_apply(*m, e0);
  float tx = origin.c[1], ty = origin.c[2], tz = origin.c[3];

  ccga_mv px = ccga_motor_apply(*m, ccga_point(1, 0, 0));
  ccga_mv py = ccga_motor_apply(*m, ccga_point(0, 1, 0));
  ccga_mv pz = ccga_motor_apply(*m, ccga_point(0, 0, 1));

  out[0] = px.c[1] - tx;
  out[4] = px.c[2] - ty;
  out[8] = px.c[3] - tz;
  out[1] = py.c[1] - tx;
  out[5] = py.c[2] - ty;
  out[9] = py.c[3] - tz;
  out[2] = pz.c[1] - tx;
  out[6] = pz.c[2] - ty;
  out[10] = pz.c[3] - tz;
  out[3] = tx;
  out[7] = ty;
  out[11] = tz;
  out[12] = out[13] = out[14] = 0.0f;
  out[15] = 1.0f;
}

ccga_mv ccga_motor_apply(ccga_mv m, ccga_mv obj) {
  return ccga_mv_gp(ccga_mv_gp(m, obj), ccga_mv_reverse(m));
}

ccga_mv ccga_motor_exp(ccga_mv B, float scale) {
  ccga_mv Bv = ccga_mv_scale(B, scale);
  float wx = Bv.c[CCGA_E23], wy = -Bv.c[CCGA_E13], wz = Bv.c[CCGA_E12];
  float vx = Bv.c[CCGA_E1EINF], vy = Bv.c[CCGA_E2EINF], vz = Bv.c[CCGA_E3EINF];

  float w_bar[3] = {2.0f * wx, 2.0f * wy, 2.0f * wz};
  float v_bar[3] = {2.0f * vx, 2.0f * vy, 2.0f * vz};

  float theta = sqrtf(w_bar[0] * w_bar[0] + w_bar[1] * w_bar[1] + w_bar[2] * w_bar[2]);
  float v_norm = sqrtf(v_bar[0] * v_bar[0] + v_bar[1] * v_bar[1] + v_bar[2] * v_bar[2]);

  if (theta < 1e-12f) {
    if (v_norm < 1e-12f) return ccga_motor_identity();
    return ccga_mv_sub(ccga_mv_scalar(1.0f), Bv);
  }
  if (v_norm < 1e-12f) {
    float ax = w_bar[0] / theta, ay = w_bar[1] / theta, az = w_bar[2] / theta;
    return ccga_motor_rotor(ax, ay, az, theta);
  }

  float bx = w_bar[0], by = w_bar[1], bz = w_bar[2];
  float W[9] = {0, -bz, by, bz, 0, -bx, -by, bx, 0};
  float WW[9];
  mat3_mul_mat3(W, W, WW);
  float theta2 = theta * theta;
  float sin_t = sinf(theta), cos_t = cosf(theta);
  float a_r = sin_t / theta, b_r = (1.0f - cos_t) / theta2;
  float a_v = (1.0f - cos_t) / theta2, b_v = (theta - sin_t) / (theta2 * theta);

  float I[9], R[9], V[9], t[3];
  mat3_identity(I);
  for (int i = 0; i < 9; i++) {
    R[i] = I[i] + a_r * W[i] + b_r * WW[i];
    V[i] = I[i] + a_v * W[i] + b_v * WW[i];
  }
  mat3_mul_vec3(V, v_bar, t);
  return ccga_motor_from_matrix(R, t);
}

ccga_mv ccga_motor_log(const ccga_mv *m) {
  float T[16];
  ccga_motor_to_matrix(m, T);
  float R[9] = {T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]};
  float t[3] = {T[3], T[7], T[11]};

  float trace = R[0] + R[4] + R[8];
  float cos_theta = (trace - 1.0f) / 2.0f;
  if (cos_theta > 1.0f) cos_theta = 1.0f;
  if (cos_theta < -1.0f) cos_theta = -1.0f;

  float antisym[3] = {R[7] - R[5], R[2] - R[6], R[3] - R[1]};
  float sin_theta_abs =
      0.5f * sqrtf(antisym[0] * antisym[0] + antisym[1] * antisym[1] + antisym[2] * antisym[2]);
  float theta = atan2f(sin_theta_abs, cos_theta);

  float w_bar[3], v_bar[3];
  if (theta < 1e-9f) {
    w_bar[0] = w_bar[1] = w_bar[2] = 0.0f;
    v_bar[0] = t[0];
    v_bar[1] = t[1];
    v_bar[2] = t[2];
  } else {
    float sin_theta = sinf(theta);
    if (theta < (float)M_PI - 1e-3f) {
      float c = theta / (2.0f * sin_theta);
      w_bar[0] = c * antisym[0];
      w_bar[1] = c * antisym[1];
      w_bar[2] = c * antisym[2];
    } else {
      float axis[3] = {
          sqrtf(fmaxf((R[0] + 1.0f) / 2.0f, 0.0f)),
          sqrtf(fmaxf((R[4] + 1.0f) / 2.0f, 0.0f)),
          sqrtf(fmaxf((R[8] + 1.0f) / 2.0f, 0.0f))};
      int ref = 0;
      if (fabsf(axis[1]) > fabsf(axis[ref])) ref = 1;
      if (fabsf(axis[2]) > fabsf(axis[ref])) ref = 2;
      if (ref == 0) {
        axis[1] = copysignf(axis[1], R[1]);
        axis[2] = copysignf(axis[2], R[2]);
      } else if (ref == 1) {
        axis[0] = copysignf(axis[0], R[1]);
        axis[2] = copysignf(axis[2], R[5]);
      } else {
        axis[0] = copysignf(axis[0], R[2]);
        axis[1] = copysignf(axis[1], R[5]);
      }
      w_bar[0] = axis[0] * theta;
      w_bar[1] = axis[1] * theta;
      w_bar[2] = axis[2] * theta;
    }

    float bx = w_bar[0], by = w_bar[1], bz = w_bar[2];
    float wxm[9] = {0, -bz, by, bz, 0, -bx, -by, bx, 0};
    float wx2[9];
    mat3_mul_mat3(wxm, wxm, wx2);
    float theta2 = theta * theta;
    float coeff = 1.0f / theta2 - (1.0f + cos_theta) / (2.0f * theta * sin_theta);
    float I[9], V_inv[9];
    mat3_identity(I);
    for (int i = 0; i < 9; i++) V_inv[i] = I[i] - 0.5f * wxm[i] + coeff * wx2[i];
    mat3_mul_vec3(V_inv, t, v_bar);
  }

  return ccga_motor_velocity_bivector(
      w_bar[0] / 2.0f, w_bar[1] / 2.0f, w_bar[2] / 2.0f,
      v_bar[0] / 2.0f, v_bar[1] / 2.0f, v_bar[2] / 2.0f);
}

ccga_mv ccga_motor_inverse(ccga_mv m) { return ccga_mv_reverse(m); }

ccga_mv ccga_motor_compose(ccga_mv self, ccga_mv other) {
  return ccga_mv_gp(self, other);
}

ccga_mv ccga_motor_interpolate(ccga_mv self, ccga_mv other, float t) {
  ccga_mv delta = ccga_mv_gp(ccga_mv_reverse(self), other);
  ccga_mv lg = ccga_motor_log(&delta);
  ccga_mv e = ccga_motor_exp(lg, t);
  return ccga_mv_gp(self, e);
}

ccga_mv ccga_motor_velocity_bivector(
    float wx, float wy, float wz, float vx, float vy, float vz) {
  ccga_mv v = ccga_mv_zero();
  v.c[CCGA_E12] = wz;
  v.c[CCGA_E13] = -wy;
  v.c[CCGA_E23] = wx;
  ccga_mv rot = v;
  ccga_mv tv = ccga_mv_vector(vx, vy, vz, 0, 0);
  ccga_mv einf = ccga_mv_vector(0, 0, 0, 0, 1);
  return ccga_mv_add(rot, ccga_mv_op(tv, einf));
}
