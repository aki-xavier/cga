/* ccga — C port of the cga 5D conformal geometric algebra core.
 *
 * Multivector type (32 components in null basis {e1,e2,e3,e0,e∞}) and the
 * algebraic operations. Component layout and the geometric-product table are
 * generated from the authoritative Python (cga.multivector) and embedded in
 * gp_tables.h, so the C port cannot drift from the reference semantics.
 *
 * Basis ordering (index -> blade):
 *   0: 1 (scalar)
 *   1..5: e1, e2, e3, e0, e∞
 *   6..15: grade-2 blades (e12, e13, e1e0, e1e∞, e23, e2e0, e2e∞, e3e0, e3e∞, e0e∞)
 *   16..25: grade-3
 *   26..30: grade-4
 *   31: e123e0e∞ (pseudoscalar)
 *
 * Metric: e1²=e2²=e3²=1, e0²=e∞²=0, e0·e∞ = e∞·e0 = −1.
 */
#ifndef CCGA_MV_H
#define CCGA_MV_H

#ifdef __cplusplus
extern "C" {
#endif

#define CCGA_NUM_COMPONENTS 32
#define CCGA_NUM_GRADES 6

typedef struct {
  float c[CCGA_NUM_COMPONENTS];
} ccga_mv;

/* Constructors */
ccga_mv ccga_mv_zero(void);
ccga_mv ccga_mv_scalar(float s);
ccga_mv ccga_mv_vector(float v1, float v2, float v3, float v0, float ve);
ccga_mv ccga_mv_bivector(const float comps[10]);

/* Component access */
float ccga_mv_scalar_part(const ccga_mv *a);
void ccga_mv_euclidean(const ccga_mv *a, float out[3]);
float ccga_mv_e0(const ccga_mv *a);
float ccga_mv_einf(const ccga_mv *a);
int ccga_mv_is_zero(const ccga_mv *a);
ccga_mv ccga_mv_grade(const ccga_mv *a, int g);

/* Arithmetic */
ccga_mv ccga_mv_add(ccga_mv a, ccga_mv b);
ccga_mv ccga_mv_sub(ccga_mv a, ccga_mv b);
ccga_mv ccga_mv_scale(ccga_mv a, float s);
ccga_mv ccga_mv_div(ccga_mv a, float s);
ccga_mv ccga_mv_neg(ccga_mv a);

/* Algebra */
ccga_mv ccga_mv_gp(ccga_mv a, ccga_mv b);
ccga_mv ccga_mv_ip(ccga_mv a, ccga_mv b);
ccga_mv ccga_mv_op(ccga_mv a, ccga_mv b);
ccga_mv ccga_mv_reverse(ccga_mv a);
ccga_mv ccga_mv_grade_involution(ccga_mv a);
ccga_mv ccga_mv_conjugate(ccga_mv a);
ccga_mv ccga_mv_dual(ccga_mv a);
ccga_mv ccga_mv_undual(ccga_mv a);
ccga_mv ccga_mv_meet(ccga_mv a, ccga_mv b);
float ccga_mv_norm(const ccga_mv *a);
ccga_mv ccga_mv_normalized(ccga_mv a);
ccga_mv ccga_mv_bulk(const ccga_mv *a);
ccga_mv ccga_mv_weight(const ccga_mv *a);

#ifdef __cplusplus
}
#endif

#endif /* CCGA_MV_H */
