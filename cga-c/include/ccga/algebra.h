/* ccga — primitives (direct/dual forms) and motors (rigid versors).
 *
 * Representation convention (identical to cga.algebra):
 *   - Point / PointPair / Line are DIRECT (join) forms.
 *   - Plane / Sphere / Circle are DUAL forms.
 */
#ifndef CCGA_ALGEBRA_H
#define CCGA_ALGEBRA_H

#include "mv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Grade-2 slot constants (bivector components) ─────────────── */
#define CCGA_E12 6
#define CCGA_E13 7
#define CCGA_E1E0 8
#define CCGA_E1EINF 9
#define CCGA_E23 10
#define CCGA_E2E0 11
#define CCGA_E2EINF 12
#define CCGA_E3E0 13
#define CCGA_E3EINF 14
#define CCGA_E0EINF 15

/* ── Primitives ────────────────────────────────────────────────── */

/* Point: p = e0 + x·e1 + y·e2 + z·e3 + ½(x²+y²+z²)·e∞ (null) */
ccga_mv ccga_point(float x, float y, float z);
/* weight-normalized Euclidean coords of a conformal point */
void ccga_point_coords(const ccga_mv *p, float out[3]);

/* PointPair: p1 ∧ p2 (direct, grade 2) */
ccga_mv ccga_point_pair(ccga_mv p1, ccga_mv p2);

/* Line: p1 ∧ p2 ∧ e∞ (direct, grade 3) */
ccga_mv ccga_line(ccga_mv p1, ccga_mv p2);

/* Plane: π = n̂ + d·e∞ (dual, grade 1); normal auto-normalized */
ccga_mv ccga_plane(float nx, float ny, float nz, float d);
/* signed distance from point (x,y,z) to plane */
float ccga_plane_dist(const ccga_mv *plane, float x, float y, float z);

/* Sphere: s = up(c) − ½ρ²·e∞ (dual, grade 1) */
ccga_mv ccga_sphere(float cx, float cy, float cz, float radius);
/* dual sphere blade -> (center, radius) */
void ccga_sphere_from_dual(const ccga_mv *s, float out_center[3], float *out_radius);

/* Circle: dual-sphere ∧ dual-plane (dual, grade 2) */
ccga_mv ccga_circle(
    float cx, float cy, float cz, float radius, float nx, float ny, float nz);

/* Cylinder: axis Line blade + radius (reconstructed primitive) */
typedef struct {
  ccga_mv blade;      /* axis Line (direct, grade 3) */
  float radius;
  float axis_dir[3];  /* unit direction */
  float axis_point[3];
} ccga_cylinder;
ccga_cylinder ccga_cylinder_make(
    float ax, float ay, float az, float dx, float dy, float dz, float radius);

/* ── Dupin cyclide (quartic, non-blade) ────────────────────────── */

typedef struct {
  float a, b, d;   /* design params (a > b > 0, d > 0); c = sqrt(a²−b²) */
  float shift[3];
} ccga_cyclide;
ccga_cyclide ccga_cyclide_make(float a, float b, float d, float sx, float sy, float sz);
float ccga_cyclide_c(const ccga_cyclide *cy);
float ccga_cyclide_implicit(const ccga_cyclide *cy, float x, float y, float z);
void ccga_cyclide_gradient(const ccga_cyclide *cy, float x, float y, float z, float out[3]);
void ccga_cyclide_normal(const ccga_cyclide *cy, float x, float y, float z, float out[3]);
int ccga_cyclide_contains(const ccga_cyclide *cy, float x, float y, float z);

/* ── Motors (even-grade versors; O' = M·O·M̃) ──────────────────── */

ccga_mv ccga_motor_identity(void);
/* rotor: R = cos(θ/2) − sin(θ/2)(nx·e23 + ny·e31 + nz·e12) */
ccga_mv ccga_motor_rotor(float ax, float ay, float az, float angle);
/* translator: T = 1 − (t ∧ e∞)/2 */
ccga_mv ccga_motor_translator(float tx, float ty, float tz);
/* rotor from quaternion (w,x,y,z), MJCF convention */
ccga_mv ccga_motor_from_quaternion(float w, float x, float y, float z);
/* M = T(t)·R, so that from_matrix(R,t).to_matrix() == [R|t] */
ccga_mv ccga_motor_from_matrix(const float R[9], const float t[3]);
/* 4x4 homogeneous [R|t], row-major (16 floats) */
void ccga_motor_to_matrix(const ccga_mv *m, float out[16]);
/* apply versor: O' = M·O·M̃ */
ccga_mv ccga_motor_apply(ccga_mv m, ccga_mv obj);
/* bivector exponential: exp(−scale·B), a motor (SE(3) exp map) */
ccga_mv ccga_motor_exp(ccga_mv B, float scale);
/* log: returns the half-twist bivector B with exp(−B) = M */
ccga_mv ccga_motor_log(const ccga_mv *m);
/* inverse: M⁻¹ = M̃ (unit versor) */
ccga_mv ccga_motor_inverse(ccga_mv m);
/* compose: self ∘ other = self·other */
ccga_mv ccga_motor_compose(ccga_mv self, ccga_mv other);
/* interpolate self -> other: M(t) = self · exp(t·log(self⁻¹·other)) */
ccga_mv ccga_motor_interpolate(ccga_mv self, ccga_mv other, float t);
/* twist bivector V = ω + v∧e∞ */
ccga_mv ccga_motor_velocity_bivector(
    float wx, float wy, float wz, float vx, float vy, float vz);

/* matrix -> quaternion (w,x,y,z) */
void ccga_matrix_to_quaternion(const float m[9], float out[4]);

#ifdef __cplusplus
}
#endif

#endif /* CCGA_ALGEBRA_H */
