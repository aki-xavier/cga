// Intersection kernels for the non-blade primitives (cone / torus / ellipsoid /
// cyclide), which use the affine ray-inverse transform (see affine_geom.rs).

use cga_core::{mat3_transpose, ConeParams, CyclideParams, EllipsoidParams, TorusParams};
use mlx_rs::complex64;
use mlx_rs::ops;
use mlx_rs::Array;

use crate::mlxops::*;
use crate::{affine_normal, affine_to_local, col, vecmat};

// dk_roots solves a monic quartic t^4 + c3 t^3 + c2 t^2 + c1 t + c0 by
// Durand-Kerner iteration (50 rounds, complex64).  Returns (N,4) sorted real
// roots (invalid = +inf).
fn dk_roots(c3: &Array, c2: &Array, c1: &Array, c0: &Array) -> Array {
    let rad = s_add(
        &ck(ck(ops::stack(
            &[&ck(c3.abs()), &ck(c2.abs()), &ck(c1.abs()), &ck(c0.abs())],
            -1,
        ))
        .max_axes(&[-1], true)),
        1.0,
    );
    let seed = Array::from_slice(
        &[
            complex64::new(0.4, 0.9),
            complex64::new(-0.65, 0.72),
            complex64::new(-0.74, -0.67),
            complex64::new(0.73, -0.68),
        ],
        &[4],
    );
    let mut z = ck(ck(rad.as_type::<complex64>()).multiply(ck(seed.expand_dims(0))));
    let c3c = ck(ck(c3.as_type::<complex64>()).expand_dims(1));
    let c2c = ck(ck(c2.as_type::<complex64>()).expand_dims(1));
    let c1c = ck(ck(c1.as_type::<complex64>()).expand_dims(1));
    let c0c = ck(ck(c0.as_type::<complex64>()).expand_dims(1));
    let idx4 = ck(ops::arange::<f32, f32>(None, 4.0, None));
    let eye = ck(
        ck(ck(ck(idx4.expand_dims(1)).eq(ck(idx4.expand_dims(0)))).as_type::<f32>()).expand_dims(0),
    );
    for _ in 0..50 {
        // pz := c0c.add(z.multiply(c1c.add(z.multiply(c2c.add(z.multiply(c3c.add(z)))))))
        let t4 = ck(c3c.add(&z));
        let t3 = ck(c2c.add(ck(z.multiply(&t4))));
        let t2 = ck(c1c.add(ck(z.multiply(&t3))));
        let pz = ck(c0c.add(ck(z.multiply(&t2))));
        let diff = ck(ck(ck(z.expand_dims(2)).subtract(ck(z.expand_dims(1)))).add(&eye));
        let denom = ck(diff.prod_axes(&[2], false));
        z = ck(z.subtract(ck(pz.divide(&denom))));
    }
    let re = ck(z.real());
    let im = ck(z.imag());
    let real_ok = ck(ck(im.abs()).le(s_mul(&s_max(&ck(re.abs()), 1.0), 1e-3)));
    let roots = ck(ops::select(&real_ok, &re, inf_like(&re)));
    ck(ops::sort_axis(&roots, -1))
}

pub(crate) fn inf_like(a: &Array) -> Array {
    ck(ops::full_like(a, fs(f64::INFINITY), None))
}

// affine_bounds transforms a local AABB's 8 corners by a_fwd (row-major 4x4).
pub(crate) fn affine_bounds(lo: [f64; 3], hi: [f64; 3], a_fwd: &[f64; 16]) -> [[f64; 3]; 2] {
    let mut minp = [0.0; 3];
    let mut maxp = [0.0; 3];
    let mut first = true;
    for i in [lo[0], hi[0]] {
        for j in [lo[1], hi[1]] {
            for k in [lo[2], hi[2]] {
                let x = a_fwd[0] * i + a_fwd[1] * j + a_fwd[2] * k + a_fwd[3];
                let y = a_fwd[4] * i + a_fwd[5] * j + a_fwd[6] * k + a_fwd[7];
                let z = a_fwd[8] * i + a_fwd[9] * j + a_fwd[10] * k + a_fwd[11];
                if first {
                    minp = [x, y, z];
                    maxp = [x, y, z];
                    first = false;
                } else {
                    if x < minp[0] {
                        minp[0] = x;
                    }
                    if y < minp[1] {
                        minp[1] = y;
                    }
                    if z < minp[2] {
                        minp[2] = z;
                    }
                    if x > maxp[0] {
                        maxp[0] = x;
                    }
                    if y > maxp[1] {
                        maxp[1] = y;
                    }
                    if z > maxp[2] {
                        maxp[2] = z;
                    }
                }
            }
        }
    }
    [minp, maxp]
}

// --- cone -------------------------------------------------------------------

pub(crate) fn cone_local_interval(
    r: f64,
    h: f64,
    o: &Array,
    d: &Array,
) -> (Array, Array, Array, Array, Array) {
    let k = r / h;
    let k2 = 1.0 + k * k;
    let wz = s_sub(&col(o, 2), h / 2.0);
    let dz = col(d, 2);
    let wd = ck(ck(ck(o.multiply(d)).sum_axes(&[-1], false)).subtract(s_mul(&dz, h / 2.0)));
    let ww = s_add(
        &ck(ck(ck(o.multiply(o)).sum_axes(&[-1], false)).subtract(s_mul(&col(o, 2), h))),
        h * h / 4.0,
    );
    let a = ck(fs(1.0).subtract(s_mul(&ck(dz.multiply(&dz)), k2)));
    let b = s_mul(&ck(wd.subtract(s_mul(&ck(wz.multiply(&dz)), k2))), 2.0);
    let c = ck(ww.subtract(s_mul(&ck(wz.multiply(&wz)), k2)));
    let mut side_ok = s_gt(&ck(a.abs()), 1e-12);
    let a_s = ck(ops::select(
        &side_ok,
        &a,
        ck(ops::full_like(&a, fs(1e-12), None)),
    ));
    let disc = ck(ck(b.multiply(&b)).subtract(s_mul(&ck(a_s.multiply(&c)), 4.0)));
    side_ok = ck(side_ok.logical_and(s_gt(&disc, 1e-12)));
    let sq = ck(s_max(&disc, 0.0).sqrt());
    let r_lo = ck(ck(ck(b.negative()).subtract(&sq)).divide(s_mul(&a_s, 2.0)));
    let r_hi = ck(ck(ck(b.negative()).add(&sq)).divide(s_mul(&a_s, 2.0)));
    let st0 = ck(ops::minimum(&r_lo, &r_hi));
    let st1 = ck(ops::maximum(&r_lo, &r_hi));
    let safe_dz = ck(ops::select(
        s_gt(&ck(dz.abs()), 1e-9),
        &dz,
        ck(ops::full_like(&dz, fs(1e-9), None)),
    ));
    let t_top = ck(ck(wz.negative()).divide(&safe_dz));
    let t_bot = ck(ck(s_add(&wz, h).negative()).divide(&safe_dz));
    let at0 = ck(ops::minimum(&t_top, &t_bot));
    let at1 = ck(ops::maximum(&t_top, &t_bot));
    let pos_a = s_gt(&a, 1e-12);
    let c1e = ck(ops::select(&pos_a, ck(ops::maximum(&st0, &at0)), &at0));
    let c1x = ck(ops::select(
        &pos_a,
        ck(ops::minimum(&st1, &at1)),
        ck(ops::minimum(&st0, &at1)),
    ));
    let v1 = ck(side_ok.logical_and(ck(c1e.lt(&c1x))));
    let c2e = ck(ops::maximum(&st1, &at0));
    let c2x = at1.clone();
    let v2 = ck(ck(side_ok.logical_and(ck(pos_a.logical_not()))).logical_and(ck(c2e.lt(&c2x))));
    let enter = ck(ops::select(&v1, &c1e, &c2e));
    let exit_ = ck(ops::select(&v1, &c1x, &c2x));
    let valid = ck(v1.logical_or(&v2));

    let n_s0 = cone_side_n(h, k2, o, d, &enter);
    let n_s1 = cone_side_n(h, k2, o, d, &exit_);
    let top_first = ck(ck(t_top.lt(&t_bot)).expand_dims(1));
    let ez = ck(arr3(0.0, 0.0, 1.0).expand_dims(0));
    let n_cap0 = ck(ops::select(&top_first, &ez, ck(ez.negative())));
    let n_cap1 = ck(ops::select(&top_first, ck(ez.negative()), &ez));
    let enter_cap = ck(ck(enter.eq(&at0)).expand_dims(1));
    let exit_cap = ck(ck(exit_.eq(&at1)).expand_dims(1));
    let n0 = ck(ops::select(&enter_cap, &n_cap0, &n_s0));
    let n1 = ck(ops::select(&exit_cap, &n_cap1, &n_s1));
    (enter, exit_, valid, n0, n1)
}

fn cone_side_n(h: f64, k2: f64, o: &Array, d: &Array, t: &Array) -> Array {
    let p = ck(o.add(ck(ck(t.expand_dims(1)).multiply(d))));
    let s = s_sub(&col(&p, 2), h / 2.0);
    let g = ck(ops::stack(
        &[&col(&p, 0), &col(&p, 1), &s_mul(&s, 1.0 - k2)],
        -1,
    ));
    let norm = ck(ck(ck(g.multiply(&g)).sum_axes(&[-1], true)).sqrt());
    ck(g.divide(ck(ops::select(
        s_gt(&norm, 1e-12),
        &norm,
        ck(ops::ones_like(&norm)),
    ))))
}

fn cone_local_intersect(r: f64, h: f64, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (enter, exit_, valid, n0, n1) = cone_local_interval(r, h, o, d);
    let hit_enter = ck(valid.logical_and(s_gt(&enter, 1e-6)));
    let hit_exit =
        ck(ck(valid.logical_and(ck(hit_enter.logical_not()))).logical_and(s_gt(&exit_, 1e-6)));
    let t = ck(ops::select(&hit_enter, &enter, &exit_));
    let mut n = ck(ops::select(
        ck(hit_enter.expand_dims(1)),
        &n0,
        ck(n1.negative()),
    ));
    let mask = ck(hit_enter.logical_or(&hit_exit));
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn cone_intersect(p: ConeParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (t_l, n_l, mask) = cone_local_intersect(p.r, p.h, &o_l, &d_u);
    let t = ck(t_l.divide(col(&lam, 0)));
    let mut n = affine_normal(&n_l, p.a_inv3);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn cone_shadow(p: ConeParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (enter, exit_, valid, _, _) = cone_local_interval(p.r, p.h, &o_l, &d_u);
    let hit_enter = ck(valid.logical_and(s_gt(&enter, 1e-6)));
    let t = ck(ops::select(&hit_enter, &enter, &exit_));
    let mask = ck(hit_enter.logical_or(ck(valid.logical_and(s_gt(&exit_, 1e-6)))));
    (ck(t.divide(col(&lam, 0))), mask)
}

pub fn cone_uv(p: ConeParams, pos: &Array, _n: &Array) -> Array {
    let p_l = ck(vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv)));
    let u = s_add(
        &s_div(
            &ck(ops::atan2(col(&p_l, 1), col(&p_l, 0))),
            2.0 * std::f64::consts::PI,
        ),
        0.5,
    );
    let v = s_div(&s_rsub(&col(&p_l, 2), p.h / 2.0), p.h);
    ck(ops::stack(&[&u, &v], -1))
}

// --- ellipsoid --------------------------------------------------------------

pub fn ellipsoid_intersect(p: EllipsoidParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let b = s_mul(&ck(ck(o_l.multiply(&d_u)).sum_axes(&[-1], false)), 2.0);
    let cq = s_sub(&ck(ck(o_l.multiply(&o_l)).sum_axes(&[-1], false)), 1.0);
    let disc = ck(ck(b.multiply(&b)).subtract(s_mul(&cq, 4.0)));
    let valid = s_gt(&disc, 1e-12);
    let sq = ck(s_max(&disc, 0.0).sqrt());
    let t1 = s_div(&ck(ck(b.negative()).subtract(&sq)), 2.0);
    let t2 = s_div(&ck(ck(b.negative()).add(&sq)), 2.0);
    let t_l = ck(ops::select(
        ck(valid.logical_and(s_gt(&t1, 1e-6))),
        &t1,
        &t2,
    ));
    let mask = ck(valid.logical_and(s_gt(&t_l, 1e-6)));
    let mut n_l = ck(o_l.add(ck(ck(t_l.expand_dims(1)).multiply(&d_u))));
    let inside = ck(mask.logical_and(s_le(&t1, 1e-6)));
    n_l = ck(ops::select(
        ck(inside.expand_dims(1)),
        ck(n_l.negative()),
        &n_l,
    ));
    let t = ck(t_l.divide(col(&lam, 0)));
    let mut n = affine_normal(&n_l, p.a_inv3);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn ellipsoid_shadow(p: EllipsoidParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let b = s_mul(&ck(ck(o_l.multiply(&d_u)).sum_axes(&[-1], false)), 2.0);
    let cq = s_sub(&ck(ck(o_l.multiply(&o_l)).sum_axes(&[-1], false)), 1.0);
    let disc = ck(ck(b.multiply(&b)).subtract(s_mul(&cq, 4.0)));
    let valid = s_gt(&disc, 1e-12);
    let sq = ck(s_max(&disc, 0.0).sqrt());
    let t1 = s_div(&ck(ck(b.negative()).subtract(&sq)), 2.0);
    let t2 = s_div(&ck(ck(b.negative()).add(&sq)), 2.0);
    let t_l = ck(ops::select(
        ck(valid.logical_and(s_gt(&t1, 1e-6))),
        &t1,
        &t2,
    ));
    (
        ck(t_l.divide(col(&lam, 0))),
        ck(valid.logical_and(s_gt(&t_l, 1e-6))),
    )
}

pub fn ellipsoid_uv(p: EllipsoidParams, pos: &Array, _n: &Array) -> Array {
    let p_l = ck(vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv)));
    let u = s_add(
        &s_div(
            &ck(ops::atan2(col(&p_l, 1), col(&p_l, 0))),
            2.0 * std::f64::consts::PI,
        ),
        0.5,
    );
    let v = s_div(
        &ck(ops::acos(s_clip(&col(&p_l, 2), -1.0, 1.0))),
        std::f64::consts::PI,
    );
    ck(ops::stack(&[&u, &v], -1))
}

// --- torus ------------------------------------------------------------------

pub(crate) fn torus_local_crossings(
    major: f64,
    minor: f64,
    o: &Array,
    d: &Array,
) -> (Array, Array, Array) {
    let r2 = major * major;
    let oo = ck(ck(o.multiply(o)).sum_axes(&[-1], false));
    let od = ck(ck(o.multiply(d)).sum_axes(&[-1], false));
    let g = s_add(&oo, r2 - minor * minor);
    let c3 = s_mul(&od, 4.0);
    let c2 = ck(
        ck(s_mul(&g, 2.0).add(s_mul(&ck(od.multiply(&od)), 4.0))).subtract(s_mul(
            &ck(ck(col(d, 0).multiply(col(d, 0))).add(ck(col(d, 1).multiply(col(d, 1))))),
            4.0 * r2,
        )),
    );
    let c1 = ck(s_mul(&ck(od.multiply(&g)), 4.0).subtract(s_mul(
        &ck(ck(col(o, 0).multiply(col(d, 0))).add(ck(col(o, 1).multiply(col(d, 1))))),
        8.0 * r2,
    )));
    let c0 = ck(ck(g.multiply(&g)).subtract(s_mul(
        &ck(ck(col(o, 0).multiply(col(o, 0))).add(ck(col(o, 1).multiply(col(o, 1))))),
        4.0 * r2,
    )));
    let ts = dk_roots(&c3, &c2, &c1, &c0);
    let valid = ck(ts.is_finite());
    let safe_t = ck(ops::select(&valid, &ts, ck(ops::zeros_like(&ts))));
    let p =
        ck(ck(o.expand_dims(1)).add(ck(ck(safe_t.expand_dims(2)).multiply(ck(d.expand_dims(1))))));
    let s = s_add(
        &ck(ck(p.multiply(&p)).sum_axes(&[-1], true)),
        r2 - minor * minor,
    );
    let s1 = col(&s, 0);
    let p0 = ck(p.take_axis(Array::from_slice(&[0], &[]), 2));
    let p1 = ck(p.take_axis(Array::from_slice(&[1], &[]), 2));
    let p2 = ck(p.take_axis(Array::from_slice(&[2], &[]), 2));
    let fac = s_sub(&s1, 2.0 * r2);
    let grad = ck(ops::stack(
        &[
            &ck(fac.multiply(&p0)),
            &ck(fac.multiply(&p1)),
            &ck(s1.multiply(&p2)),
        ],
        -1,
    ));
    let norm = ck(ck(ck(grad.multiply(&grad)).sum_axes(&[-1], true)).sqrt());
    let mut ns = ck(grad.divide(ck(ops::select(
        s_gt(&norm, 1e-12),
        &norm,
        ck(ops::ones_like(&norm)),
    ))));
    ns = ck(ops::select(
        ck(valid.expand_dims(2)),
        &ns,
        ck(ops::zeros_like(&ns)),
    ));
    (ts, ns, valid)
}

pub(crate) fn torus_local_contains(major: f64, minor: f64, p: &Array) -> Array {
    let r2 = major * major;
    let f = ck(ck(s_add(
        &ck(ck(p.multiply(p)).sum_axes(&[-1], false)),
        r2 - minor * minor,
    )
    .square())
    .subtract(s_mul(
        &ck(ck(col(p, 0).multiply(col(p, 0))).add(ck(col(p, 1).multiply(col(p, 1))))),
        4.0 * r2,
    )));
    s_lt(&f, 0.0)
}

pub fn torus_intersect(p: TorusParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, ns, valid) = torus_local_crossings(p.major, p.minor, &o_l, &d_u);
    let pos = ck(valid.logical_and(s_gt(&ts, 1e-6)));
    let cand = ck(ops::select(&pos, &ts, inf_like(&ts)));
    let t_l = ck(cand.min_axes(&[-1], false));
    let mask = ck(t_l.is_finite());
    let idx = ck(ops::indexing::argmin_axis(&cand, -1, false));
    let mut n_l = ck(ck(ns.take_along_axis(
        ck(ops::broadcast_to(
            ck(ck(idx.expand_dims(1)).expand_dims(2)),
            &[ns.shape()[0], 1, 3],
        )),
        1,
    ))
    .take_axis(Array::from_slice(&[0], &[]), 1));
    let inside = ck(mask.logical_and(torus_local_contains(p.major, p.minor, &o_l)));
    n_l = ck(ops::select(
        ck(inside.expand_dims(1)),
        ck(n_l.negative()),
        &n_l,
    ));
    let t = ck(t_l.divide(col(&lam, 0)));
    let mut n = affine_normal(&n_l, p.a_inv3);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn torus_shadow(p: TorusParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, _, valid) = torus_local_crossings(p.major, p.minor, &o_l, &d_u);
    let pos = ck(valid.logical_and(s_gt(&ts, 1e-6)));
    let cand = ck(ops::select(&pos, &ts, inf_like(&ts)));
    let t_l = ck(cand.min_axes(&[-1], false));
    (ck(t_l.divide(col(&lam, 0))), ck(t_l.is_finite()))
}

pub fn torus_uv(p: TorusParams, pos: &Array, _n: &Array) -> Array {
    let p_l = ck(vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv)));
    let rho = ck(ck(ck(col(&p_l, 0).square()).add(ck(col(&p_l, 1).square()))).sqrt());
    let u = s_add(
        &s_div(
            &ck(ops::atan2(col(&p_l, 1), col(&p_l, 0))),
            2.0 * std::f64::consts::PI,
        ),
        0.5,
    );
    let v = s_add(
        &s_div(
            &ck(ops::atan2(col(&p_l, 2), s_sub(&rho, p.major))),
            2.0 * std::f64::consts::PI,
        ),
        0.5,
    );
    ck(ops::stack(&[&u, &v], -1))
}

// --- cyclide ----------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
pub(crate) fn cyclide_local_crossings(
    a: f64,
    b: f64,
    dd: f64,
    c: f64,
    shift: [f64; 3],
    o: &Array,
    d: &Array,
) -> (Array, Array, Array) {
    let ox = s_sub(&col(o, 0), shift[0]);
    let oy = s_sub(&col(o, 1), shift[1]);
    let oz = s_sub(&col(o, 2), shift[2]);
    let dx = col(d, 0);
    let dy = col(d, 1);
    let dz = col(d, 2);
    let big_a = ck(ck(ck(ox.multiply(&ox)).add(ck(oy.multiply(&oy)))).add(ck(oz.multiply(&oz))));
    let b1 = ck(ck(ck(ox.multiply(&dx)).add(ck(oy.multiply(&dy)))).add(ck(oz.multiply(&dz))));
    let bb = b * b - dd * dd;
    let g = s_add(&big_a, bb);
    let p0 = s_sub(&s_mul(&ox, a), c * dd);
    let p1 = s_mul(&dx, a);
    let c3 = s_mul(&b1, 4.0);
    let c2 = ck(ck(ck(s_mul(&g, 2.0).add(s_mul(&ck(b1.multiply(&b1)), 4.0)))
        .subtract(s_mul(&ck(p1.multiply(&p1)), 4.0)))
    .subtract(s_mul(&ck(dy.multiply(&dy)), 4.0 * b * b)));
    let c1 = ck(
        ck(s_mul(&ck(b1.multiply(&g)), 4.0).subtract(s_mul(&ck(p0.multiply(&p1)), 8.0)))
            .subtract(s_mul(&ck(oy.multiply(&dy)), 8.0 * b * b)),
    );
    let c0 = ck(
        ck(ck(g.multiply(&g)).subtract(s_mul(&ck(p0.multiply(&p0)), 4.0)))
            .subtract(s_mul(&ck(oy.multiply(&oy)), 4.0 * b * b)),
    );
    let ts = dk_roots(&c3, &c2, &c1, &c0);
    let valid = ck(ts.is_finite());
    let safe_t = ck(ops::select(&valid, &ts, ck(ops::zeros_like(&ts))));
    let p = ck(ck(
        ck(o.expand_dims(1)).subtract(ck(ck(arr3v(shift).expand_dims(0)).expand_dims(0)))
    )
    .add(ck(ck(safe_t.expand_dims(2)).multiply(ck(d.expand_dims(1))))));
    let mut ns = cyclide_normal(a, b, dd, c, &p);
    ns = ck(ops::select(
        ck(valid.expand_dims(2)),
        &ns,
        ck(ops::zeros_like(&ns)),
    ));
    (ts, ns, valid)
}

fn cyclide_normal(a: f64, b: f64, dd: f64, c: f64, p: &Array) -> Array {
    let bb = b * b - dd * dd;
    let rho = ck(ck(p.multiply(p)).sum_axes(&[-1], true));
    let g = s_add(&rho, bb);
    let g0 = col(&g, 0);
    let p0 = ck(p.take_axis(Array::from_slice(&[0], &[]), 2));
    let p1 = ck(p.take_axis(Array::from_slice(&[1], &[]), 2));
    let p2 = ck(p.take_axis(Array::from_slice(&[2], &[]), 2));
    let grad = ck(ops::stack(
        &[
            &ck(s_mul(&ck(p0.multiply(&g0)), 4.0)
                .subtract(s_mul(&s_sub(&s_mul(&p0, a), c * dd), 8.0 * a))),
            &ck(s_mul(&ck(p1.multiply(&g0)), 4.0).subtract(s_mul(&p1, 8.0 * b * b))),
            &s_mul(&ck(p2.multiply(&g0)), 4.0),
        ],
        -1,
    ));
    let norm = ck(ck(ck(grad.multiply(&grad)).sum_axes(&[-1], true)).sqrt());
    ck(grad.divide(ck(ops::select(
        s_gt(&norm, 1e-12),
        &norm,
        ck(ops::ones_like(&norm)),
    ))))
}

pub(crate) fn cyclide_local_contains(
    a: f64,
    b: f64,
    dd: f64,
    c: f64,
    shift: [f64; 3],
    p: &Array,
) -> Array {
    let x = s_sub(&col(p, 0), shift[0]);
    let y = s_sub(&col(p, 1), shift[1]);
    let z = s_sub(&col(p, 2), shift[2]);
    let bb = b * b - dd * dd;
    let rho = ck(ck(ck(x.multiply(&x)).add(ck(y.multiply(&y)))).add(ck(z.multiply(&z))));
    let f = ck(ck(ck(s_add(&rho, bb).square())
        .subtract(s_mul(&ck(s_sub(&s_mul(&x, a), c * dd).square()), 4.0)))
    .subtract(s_mul(&ck(y.multiply(&y)), 4.0 * b * b)));
    s_lt(&f, 0.0)
}

pub fn cyclide_intersect(p: CyclideParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, ns, valid) = cyclide_local_crossings(p.a, p.b, p.d, p.c, p.shift, &o_l, &d_u);
    let pos = ck(valid.logical_and(s_gt(&ts, 1e-6)));
    let cand = ck(ops::select(&pos, &ts, inf_like(&ts)));
    let t_l = ck(cand.min_axes(&[-1], false));
    let mask = ck(t_l.is_finite());
    let idx = ck(ops::indexing::argmin_axis(&cand, -1, false));
    let mut n_l = ck(ck(ns.take_along_axis(
        ck(ops::broadcast_to(
            ck(ck(idx.expand_dims(1)).expand_dims(2)),
            &[ns.shape()[0], 1, 3],
        )),
        1,
    ))
    .take_axis(Array::from_slice(&[0], &[]), 1));
    let inside = ck(mask.logical_and(cyclide_local_contains(p.a, p.b, p.d, p.c, p.shift, &o_l)));
    n_l = ck(ops::select(
        ck(inside.expand_dims(1)),
        ck(n_l.negative()),
        &n_l,
    ));
    let t = ck(t_l.divide(col(&lam, 0)));
    let mut n = affine_normal(&n_l, p.a_inv3);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn cyclide_shadow(p: CyclideParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, _, valid) = cyclide_local_crossings(p.a, p.b, p.d, p.c, p.shift, &o_l, &d_u);
    let pos = ck(valid.logical_and(s_gt(&ts, 1e-6)));
    let cand = ck(ops::select(&pos, &ts, inf_like(&ts)));
    let t_l = ck(cand.min_axes(&[-1], false));
    (ck(t_l.divide(col(&lam, 0))), ck(t_l.is_finite()))
}

pub fn cyclide_uv(p: CyclideParams, pos: &Array, _n: &Array) -> Array {
    let p_l = ck(vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv)));
    let x = s_sub(&col(&p_l, 0), p.shift[0]);
    let y = s_sub(&col(&p_l, 1), p.shift[1]);
    let z = s_sub(&col(&p_l, 2), p.shift[2]);
    let rho = ck(ck(ck(x.multiply(&x)).add(ck(y.multiply(&y)))).add(ck(z.multiply(&z))));
    let u = s_add(
        &s_div(
            &ck(ops::atan2(
                s_mul(&y, 2.0 * p.b),
                s_mul(&s_sub(&s_mul(&x, p.a), p.c * p.d), 2.0),
            )),
            2.0 * std::f64::consts::PI,
        ),
        0.5,
    );
    let v = s_add(
        &s_div(
            &ck(ops::atan2(
                s_mul(&z, 2.0 * p.b),
                s_rsub(&rho, p.d * p.d + p.b * p.b),
            )),
            2.0 * std::f64::consts::PI,
        ),
        0.5,
    );
    ck(ops::stack(&[&u, &v], -1))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{
        csg_geometry, cyclide_geometry, dupin_cyclide, from_torus_inversion, motor_identity, point,
        sphere_from_dual, CsgOp, DupinCyclide, Geometry,
    };

    // Dupin cyclide tests: blade construction (sphere-family envelope / versor
    // inversion) + engine analytic intersection.
    // Gate: ring cyclide (a=1, b=0.98, d=0.3, c=sqrt(a^2-b^2) ~= 0.199) with c<d<a.

    const CGA_A: f64 = 1.0;
    const CGA_B: f64 = 0.98;
    const CGA_D: f64 = 0.3;

    fn cga_c() -> f64 {
        (CGA_A * CGA_A - CGA_B * CGA_B).sqrt()
    }

    // canonical ring cyclide x-axis intersections (descending)
    fn cga_x1() -> f64 {
        CGA_A + CGA_D - cga_c()
    }
    fn cga_x2() -> f64 {
        CGA_A - CGA_D + cga_c()
    }
    fn cga_x3() -> f64 {
        CGA_D + cga_c() - CGA_A
    }
    fn cga_x4() -> f64 {
        -CGA_A - CGA_D - cga_c()
    }

    fn cga_cy() -> DupinCyclide {
        dupin_cyclide(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0])
    }

    fn mod2pi(x: f64) -> f64 {
        let tau = 2.0 * std::f64::consts::PI;
        let r = x - tau * (x / tau).floor();
        r.min(tau - r)
    }

    fn ray3(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn cyc_hit(g: &Geometry, o: [f64; 3], d: [f64; 3]) -> (f32, bool) {
        let p = crate::geom_to_camera(g, &motor_identity());
        let oa = ray3(o[0], o[1], o[2]);
        let da = ray3(d[0], d[1], d[2]);
        let (t, _, mask) = crate::geom_intersect(&p, &oa, &da);
        t.eval().unwrap();
        mask.eval().unwrap();
        (t.item_cast::<f32>(), mask.as_slice::<bool>()[0])
    }

    // --- algebra model -----------------------------------------------------------

    #[test]
    fn test_cyclide_param_implicit_consistency() {
        let cy = cga_cy();
        for uv in [
            [0.0, 0.0],
            [1.2, 2.3],
            [3.1, 5.4],
            [std::f64::consts::PI, 0.5],
        ] {
            let s = cy.surface(uv[0], uv[1]);
            assert!(cy.implicit(s[0], s[1], s[2]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_cyclide_kind() {
        assert_eq!(cga_cy().kind(), "ring"); // c < d < a
        assert_eq!(
            dupin_cyclide(1.0, 0.6, 1.5, [0.0, 0.0, 0.0]).kind(),
            "spindle"
        ); // d > a
        assert_eq!(dupin_cyclide(1.0, 0.6, 0.3, [0.0, 0.0, 0.0]).kind(), "horn");
        // d < c
    }

    #[test]
    fn test_cyclide_focal_sphere_tangency() {
        let cy = cga_cy();
        for u in [0.0, 1.0, 2.0, 4.0] {
            let r = cy.tangency_residual(u);
            assert!(r[0].abs() < 1e-10);
            assert!(r[1].abs() < 1e-10);
        }
    }

    #[test]
    fn test_cyclide_generator_sphere_is_blade() {
        let cy = cga_cy();
        let s = cy.generator_sphere(0.7);
        let (ctr, r) = sphere_from_dual(&s);
        let sp = cy.spine(0.7);
        let rad = cy.radius(0.7);
        assert!((ctr[0] - sp[0]).abs() < 1e-5);
        assert!((ctr[1] - sp[1]).abs() < 1e-5);
        assert!((ctr[2] - sp[2]).abs() < 1e-5);
        assert!((r - rad).abs() < 1e-5);
    }

    #[test]
    fn test_cyclide_focal_spheres_are_blades() {
        let cy = cga_cy();
        let (s1, s2) = cy.focal_spheres();
        let (c1, r1) = sphere_from_dual(&s1);
        let (c2, r2) = sphere_from_dual(&s2);
        assert!((c1[0] - cga_c()).abs() < 1e-5);
        assert!(c1[1].abs() < 1e-5);
        assert!(c1[2].abs() < 1e-5);
        assert!((r1 - (CGA_A - CGA_D)).abs() < 1e-5);
        assert!((c2[0] + cga_c()).abs() < 1e-5);
        assert!(c2[1].abs() < 1e-5);
        assert!(c2[2].abs() < 1e-5);
        assert!((r2 - (CGA_A + CGA_D)).abs() < 1e-5);
    }

    #[test]
    fn test_cyclide_characteristic_circle_on_surface() {
        let cy = cga_cy();
        let u = 0.9;
        let e = cy.spine(u);
        let ep = [-cy.a * u.sin(), cy.b * u.cos(), 0.0];
        let r = cy.radius(u);
        let rp = cy.c() * u.sin();
        let ep2 = ep[0] * ep[0] + ep[1] * ep[1] + ep[2] * ep[2];
        let lam = r * rp / ep2;
        let center = [e[0] - lam * ep[0], e[1] - lam * ep[1], e[2] - lam * ep[2]];
        let radius = (r * r - lam * lam * ep2).sqrt();
        let nn = ep2.sqrt();
        let n = [ep[0] / nn, ep[1] / nn, ep[2] / nn];
        let e1v = [n[1], -n[0], 0.0];
        let e2v = [
            n[1] * e1v[2] - n[2] * e1v[1],
            n[2] * e1v[0] - n[0] * e1v[2],
            n[0] * e1v[1] - n[1] * e1v[0],
        ];
        let cc = cy.characteristic_circle(u);
        for t in [0.0f64, 1.3, 2.5] {
            let p = [
                center[0] + radius * (e1v[0] * t.cos() + e2v[0] * t.sin()),
                center[1] + radius * (e1v[1] * t.cos() + e2v[1] * t.sin()),
                center[2] + radius * (e1v[2] * t.cos() + e2v[2] * t.sin()),
            ];
            assert!(cy.implicit(p[0], p[1], p[2]).abs() < 1e-8);
            assert!(point(p[0], p[1], p[2]).ip(&cc).scalar_part().abs() < 1e-6);
        }
    }

    #[test]
    fn test_cyclide_uv_roundtrip() {
        let cy = cga_cy();
        for uv in [[0.5, 1.0], [2.0, 4.0], [5.0, 0.2]] {
            let s = cy.surface(uv[0], uv[1]);
            let r = cy.uv(s[0], s[1], s[2]);
            assert!(mod2pi(r[0] - uv[0]) < 1e-6);
            assert!(mod2pi(r[1] + uv[1]) < 1e-6);
        }
    }

    #[test]
    fn test_cyclide_from_torus_inversion() {
        let cy = from_torus_inversion(2.0, 0.5, 1.0);
        assert_eq!(cy.kind(), "ring");
        let r_maj = 2.0;
        let r_min = 0.5;
        let s = 1.0;
        for uv in [[0.7f64, 1.9], [2.1, 0.3]] {
            let u = uv[0];
            let v = uv[1];
            let tx = s + (r_maj + r_min * v.cos()) * u.cos();
            let ty = (r_maj + r_min * v.cos()) * u.sin();
            let tz = r_min * v.sin();
            let n2 = tx * tx + ty * ty + tz * tz;
            assert!(cy.implicit(tx / n2, ty / n2, tz / n2).abs() < 1e-9);
        }
    }

    #[test]
    fn test_cyclide_inversion_versor() {
        let cy = cga_cy();
        for p in [[2.0, 0.0, 0.0], [1.0, 2.0, 3.0], [-0.5, 0.25, 1.5]] {
            let q = cy.invert_point(&point(p[0], p[1], p[2]));
            let r2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
            let c = q.coords();
            assert!((c[0] - p[0] / r2).abs() < 1e-6);
            assert!((c[1] - p[1] / r2).abs() < 1e-6);
            assert!((c[2] - p[2] / r2).abs() < 1e-6);
        }
    }

    // --- engine ------------------------------------------------------------------

    #[test]
    fn test_cyclide_ray_hits_implicit() {
        let g = Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0]));
        let (t, m) = cyc_hit(&g, [3.0, 0.0, 0.0], [-1.0, 0.0, 0.0]);
        assert!(m);
        let cy = cga_cy();
        assert!(cy.implicit(3.0 - f64::from(t), 0.0, 0.0).abs() < 1e-3);
    }

    #[test]
    fn test_cyclide_axis_ray_four_crossings() {
        let g = Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0]));
        let p = crate::geom_to_camera(&g, &motor_identity());
        let (ts, _, valid) = crate::geom_crossings(&p, &ray3(3.0, 0.0, 0.0), &ray3(-1.0, 0.0, 0.0));
        valid.eval().unwrap();
        let v = valid.as_slice::<bool>().to_vec();
        assert_eq!(v.len(), 4);
        assert!(v[0] && v[1] && v[2] && v[3]);
        ts.eval().unwrap();
        let td = ts.as_slice::<f32>().to_vec();
        let mut want = [
            3.0 - cga_x1(),
            3.0 - cga_x2(),
            3.0 - cga_x3(),
            3.0 - cga_x4(),
        ];
        want.sort_by(f64::total_cmp);
        for i in 0..4 {
            assert!((f64::from(td[i]) - want[i]).abs() < 1e-2);
        }
    }

    #[test]
    fn test_cyclide_center_hole_ray_misses() {
        let g = Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0]));
        let (_, m) = cyc_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m);
    }

    #[test]
    fn test_cyclide_contains() {
        let g = Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0]));
        let p = crate::geom_to_camera(&g, &motor_identity());
        let pos = Array::from_slice(&[1.0f32, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0], &[3, 3]);
        let got = crate::geom_contains(&p, &pos);
        got.eval().unwrap();
        assert_eq!(got.as_slice::<bool>(), &[true, false, false]);
    }

    #[test]
    fn test_cyclide_shift() {
        let g = Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [1.0, 0.0, 0.0]));
        let p = crate::geom_to_camera(&g, &motor_identity());
        let pos = Array::from_slice(&[2.0f32, 0.0, 0.0, 1.0, 0.0, 0.0], &[2, 3]);
        let got = crate::geom_contains(&p, &pos);
        got.eval().unwrap();
        assert_eq!(got.as_slice::<bool>(), &[true, false]);
    }

    #[test]
    fn test_cyclide_bounds_contain_surface() {
        let g = Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0]));
        let p = crate::geom_to_camera(&g, &motor_identity());
        let b = crate::geom_bounds(&p).expect("no bounds");
        let bmin = b[0];
        let bmax = b[1];
        let cy = cga_cy();
        for i in 0..7 {
            for j in 0..7 {
                let s = cy.surface(
                    f64::from(i) * 2.0 * std::f64::consts::PI / 6.0,
                    f64::from(j) * 2.0 * std::f64::consts::PI / 6.0,
                );
                assert!(s[0] >= bmin[0] - 1e-6 && s[0] <= bmax[0] + 1e-6);
                assert!(s[1] >= bmin[1] - 1e-6 && s[1] <= bmax[1] + 1e-6);
                assert!(s[2] >= bmin[2] - 1e-6 && s[2] <= bmax[2] + 1e-6);
            }
        }
    }

    #[test]
    fn test_cyclide_csg_combines() {
        let csg = csg_geometry(
            CsgOp::Union,
            vec![
                Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0])),
                Geometry::CyclideGeometry(cyclide_geometry(CGA_A, CGA_B, CGA_D, [0.0, 0.0, 0.0])),
            ],
        );
        let (t, m) = cyc_hit(
            &Geometry::CsgGeometry(csg),
            [3.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
        );
        assert!(m);
        assert!(t > 0.0);
    }
}
