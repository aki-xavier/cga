#include <math.h>
#include <stdio.h>

#include "ccga/algebra.h"
#include "ccga/mv.h"

static void print32(const char *label, ccga_mv v) {
  printf("%s:", label);
  for (int i = 0; i < 32; i++) printf(" %.5f", v.c[i]);
  printf("\n");
}

int main(void) {
  ccga_mv e1 = ccga_mv_vector(1, 0, 0, 0, 0);
  float bv[10] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0}; /* e12 */
  ccga_mv e12 = ccga_mv_bivector(bv);
  print32("gp_e1_e12", ccga_mv_gp(e1, e12));
  print32("gp_e12_e1", ccga_mv_gp(e12, e1));

  ccga_mv p = ccga_point(1, 2, 3);
  ccga_mv p_gp = ccga_mv_gp(p, p);
  printf("point_null: %.5f\n", ccga_mv_scalar_part(&p_gp));

  ccga_mv r = ccga_motor_rotor(0, 0, 1, 3.14159265f / 2.0f);
  ccga_mv p1 = ccga_motor_apply(r, ccga_point(1, 0, 0));
  float c1[3];
  ccga_point_coords(&p1, c1);
  printf("rotor_z90_point: %.5f %.5f %.5f\n", c1[0], c1[1], c1[2]);

  ccga_mv tr = ccga_motor_translator(1, 2, 3);
  ccga_mv p0 = ccga_motor_apply(tr, ccga_point(0, 0, 0));
  float c0[3];
  ccga_point_coords(&p0, c0);
  printf("translator_origin: %.5f %.5f %.5f\n", c0[0], c0[1], c0[2]);

  ccga_mv m = ccga_mv_gp(tr, r);
  float mtx[16];
  ccga_motor_to_matrix(&m, mtx);
  printf("motor_matrix:");
  for (int i = 0; i < 16; i++) printf(" %.5f", mtx[i]);
  printf("\n");

  ccga_mv s = ccga_sphere(1, 2, 3, 2);
  float sc[3], sr;
  ccga_sphere_from_dual(&s, sc, &sr);
  printf("sphere_roundtrip: %.5f %.5f %.5f %.5f\n", sc[0], sc[1], sc[2], sr);

  ccga_mv pl = ccga_plane(0, 1, 0, 0);
  printf("plane_dist: %.5f\n", ccga_plane_dist(&pl, 0, 5, 0));

  ccga_mv cir = ccga_circle(0, 0, 0, 2, 0, 0, 1);
  print32("circle", cir);

  ccga_mv B = ccga_motor_velocity_bivector(0, 0, 1, 0.1f, 0.2f, 0.3f);
  ccga_mv M = ccga_motor_exp(B, 1.0f);
  ccga_mv L = ccga_motor_log(&M);
  print32("exp_log", L);

  print32("exp_log_orig", B);

  printf("reverse_e12_c6: %.5f\n", ccga_mv_reverse(e12).c[6]);
  printf("dual_scalar_c31: %.5f\n", ccga_mv_dual(ccga_mv_scalar(1.0f)).c[31]);

  return 0;
}
