/* ccga multivector implementation. */
#include "ccga/mv.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "gp_tables.h"

ccga_mv ccga_mv_zero(void) {
  ccga_mv r;
  memset(r.c, 0, sizeof(r.c));
  return r;
}

ccga_mv ccga_mv_scalar(float s) {
  ccga_mv r = ccga_mv_zero();
  r.c[0] = s;
  return r;
}

ccga_mv ccga_mv_vector(float v1, float v2, float v3, float v0, float ve) {
  ccga_mv r = ccga_mv_zero();
  r.c[1] = v1;
  r.c[2] = v2;
  r.c[3] = v3;
  r.c[4] = v0;
  r.c[5] = ve;
  return r;
}

ccga_mv ccga_mv_bivector(const float comps[10]) {
  ccga_mv r = ccga_mv_zero();
  for (int i = 0; i < 10; i++) r.c[6 + i] = comps[i];
  return r;
}

float ccga_mv_scalar_part(const ccga_mv *a) { return a->c[0]; }

void ccga_mv_euclidean(const ccga_mv *a, float out[3]) {
  out[0] = a->c[1];
  out[1] = a->c[2];
  out[2] = a->c[3];
}

float ccga_mv_e0(const ccga_mv *a) { return a->c[4]; }
float ccga_mv_einf(const ccga_mv *a) { return a->c[5]; }

int ccga_mv_is_zero(const ccga_mv *a) {
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) {
    if (fabsf(a->c[i]) > 1e-10f) return 0;
  }
  return 1;
}

ccga_mv ccga_mv_grade(const ccga_mv *a, int g) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) {
    r.c[i] = a->c[i] * CGA_GRADE_MASK[g][i];
  }
  return r;
}

ccga_mv ccga_mv_add(ccga_mv a, ccga_mv b) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) r.c[i] = a.c[i] + b.c[i];
  return r;
}

ccga_mv ccga_mv_sub(ccga_mv a, ccga_mv b) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) r.c[i] = a.c[i] - b.c[i];
  return r;
}

ccga_mv ccga_mv_scale(ccga_mv a, float s) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) r.c[i] = a.c[i] * s;
  return r;
}

ccga_mv ccga_mv_div(ccga_mv a, float s) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) r.c[i] = a.c[i] / s;
  return r;
}

ccga_mv ccga_mv_neg(ccga_mv a) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) r.c[i] = -a.c[i];
  return r;
}

ccga_mv ccga_mv_gp(ccga_mv a, ccga_mv b) {
  float r[CCGA_NUM_COMPONENTS] = {0};
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) {
    const float ai = a.c[i];
    if (ai == 0.0f) continue;
    for (int j = 0; j < CCGA_NUM_COMPONENTS; j++) {
      const float bj = b.c[j];
      if (bj == 0.0f) continue;
      const float ab = ai * bj;
      const int n = CGA_GP_COUNT[i][j];
      for (int k = 0; k < n; k++) {
        r[CGA_GP_DST[i][j][k]] += (float)CGA_GP_SIGN[i][j][k] * ab;
      }
    }
  }
  ccga_mv out;
  memcpy(out.c, r, sizeof(r));
  return out;
}

ccga_mv ccga_mv_ip(ccga_mv a, ccga_mv b) {
  ccga_mv result = ccga_mv_zero();
  for (int ga = 1; ga < CCGA_NUM_GRADES; ga++) {
    ccga_mv ag = ccga_mv_grade(&a, ga);
    if (ccga_mv_is_zero(&ag)) continue;
    for (int gb = 1; gb < CCGA_NUM_GRADES; gb++) {
      ccga_mv bg = ccga_mv_grade(&b, gb);
      if (ccga_mv_is_zero(&bg)) continue;
      ccga_mv prod = ccga_mv_gp(ag, bg);
      ccga_mv pg = ccga_mv_grade(&prod, abs(gb - ga));
      result = ccga_mv_add(result, pg);
    }
  }
  return result;
}

ccga_mv ccga_mv_op(ccga_mv a, ccga_mv b) {
  ccga_mv result = ccga_mv_zero();
  for (int ga = 0; ga < CCGA_NUM_GRADES; ga++) {
    ccga_mv ag = ccga_mv_grade(&a, ga);
    if (ccga_mv_is_zero(&ag)) continue;
    for (int gb = 0; ga + gb < CCGA_NUM_GRADES; gb++) {
      ccga_mv bg = ccga_mv_grade(&b, gb);
      if (ccga_mv_is_zero(&bg)) continue;
      ccga_mv prod = ccga_mv_gp(ag, bg);
      ccga_mv pg = ccga_mv_grade(&prod, ga + gb);
      result = ccga_mv_add(result, pg);
    }
  }
  return result;
}

ccga_mv ccga_mv_reverse(ccga_mv a) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) r.c[i] = a.c[i] * (float)CGA_REVERSE_MASK[i];
  return r;
}

ccga_mv ccga_mv_grade_involution(ccga_mv a) {
  ccga_mv r;
  for (int i = 0; i < CCGA_NUM_COMPONENTS; i++) r.c[i] = a.c[i] * (float)CGA_INVOLUTION_MASK[i];
  return r;
}

ccga_mv ccga_mv_conjugate(ccga_mv a) {
  return ccga_mv_grade_involution(ccga_mv_reverse(a));
}

ccga_mv ccga_mv_dual(ccga_mv a) {
  ccga_mv i_inv = ccga_mv_zero();
  i_inv.c[31] = 1.0f;
  return ccga_mv_gp(a, i_inv);
}

ccga_mv ccga_mv_undual(ccga_mv a) { return ccga_mv_neg(ccga_mv_dual(a)); }

ccga_mv ccga_mv_meet(ccga_mv a, ccga_mv b) {
  return ccga_mv_dual(ccga_mv_op(ccga_mv_dual(a), ccga_mv_dual(b)));
}

float ccga_mv_norm(const ccga_mv *a) {
  ccga_mv r = ccga_mv_gp(*a, ccga_mv_reverse(*a));
  return sqrtf(fabsf(r.c[0]));
}

ccga_mv ccga_mv_normalized(ccga_mv a) {
  float n = ccga_mv_norm(&a);
  if (n < 1e-12f) return ccga_mv_zero();
  return ccga_mv_div(a, n);
}

ccga_mv ccga_mv_bulk(const ccga_mv *a) {
  ccga_mv r = ccga_mv_zero();
  for (int k = 0; k < 8; k++) r.c[CGA_BULK_INDICES[k]] = a->c[CGA_BULK_INDICES[k]];
  return r;
}

ccga_mv ccga_mv_weight(const ccga_mv *a) {
  return ccga_mv_sub(*a, ccga_mv_bulk(a));
}
