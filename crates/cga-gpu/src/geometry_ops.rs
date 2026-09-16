// Per-pixel ray-intersection kernels (MLX batch, float32).  Each geometry
// provides intersect / intersect_shadow / uv_at / bounds_camera over the
// camera-space parameters computed in geometry.v.

use cga_core::{
    AffineParams, BoxParams, CircleParams, CylinderParams, GeometryParams, PlaneParams,
    SphereParams,
};
use mlx_rs::ops;
use mlx_rs::Array;

use crate::mlxops::*;

// col extracts column i of a (...,N) array as a (...,) array (last-axis index).
#[inline]
pub(crate) fn col(a: &Array, i: i32) -> Array {
    ck(a.take_axis(Array::from_slice(&[i], &[]), -1))
}

// --- sphere -----------------------------------------------------------------

pub fn sphere_intersect(p: SphereParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let c = arr3v(p.c);
    let oc = ck(o.subtract(&c));
    let b = s_mul(&ck(ck(oc.multiply(d)).sum_axes(&[-1], false)), 2.0);
    let cq = s_sub(&ck(ck(oc.multiply(&oc)).sum_axes(&[-1], false)), p.r * p.r);
    let disc = ck(ck(b.multiply(&b)).subtract(s_mul(&cq, 4.0)));
    let valid = s_gt(&disc, 1e-12);
    let sq = ck(s_max(&disc, 0.0).sqrt());
    let t1 = s_div(&ck(ck(b.negative()).subtract(&sq)), 2.0);
    let t2 = s_div(&ck(ck(b.negative()).add(&sq)), 2.0);
    let t = ck(ops::select(
        ck(valid.logical_and(s_gt(&t1, 1e-6))),
        &t1,
        &t2,
    ));
    let mask = ck(valid.logical_and(s_gt(&t, 1e-6)));
    let hit = ck(o.add(ck(ck(t.expand_dims(1)).multiply(d))));
    let mut n = s_div(&ck(hit.subtract(&c)), p.r);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    let inside = ck(mask.logical_and(s_le(&t1, 1e-6)));
    n = ck(ops::select(ck(inside.expand_dims(1)), ck(n.negative()), &n));
    (t, n, mask)
}

pub fn sphere_shadow(p: SphereParams, o: &Array, d: &Array) -> (Array, Array) {
    let c = arr3v(p.c);
    let oc = ck(o.subtract(&c));
    let b = s_mul(&ck(ck(oc.multiply(d)).sum_axes(&[-1], false)), 2.0);
    let cq = s_sub(&ck(ck(oc.multiply(&oc)).sum_axes(&[-1], false)), p.r * p.r);
    let disc = ck(ck(b.multiply(&b)).subtract(s_mul(&cq, 4.0)));
    let valid = s_gt(&disc, 1e-12);
    let sq = ck(s_max(&disc, 0.0).sqrt());
    let t1 = s_div(&ck(ck(b.negative()).subtract(&sq)), 2.0);
    let t2 = s_div(&ck(ck(b.negative()).add(&sq)), 2.0);
    let t = ck(ops::select(
        ck(valid.logical_and(s_gt(&t1, 1e-6))),
        &t1,
        &t2,
    ));
    (t.clone(), ck(valid.logical_and(s_gt(&t, 1e-6))))
}

pub fn sphere_uv(p: SphereParams, pos: &Array, _n: &Array) -> Array {
    let c = arr3v(p.c);
    let q = ck(pos.subtract(&c));
    let x = s_div(
        &ck(ck(q.multiply(arr3v(p.axes[0]))).sum_axes(&[-1], false)),
        p.r,
    );
    let y = s_div(
        &ck(ck(q.multiply(arr3v(p.axes[1]))).sum_axes(&[-1], false)),
        p.r,
    );
    let z = s_clip(
        &s_div(
            &ck(ck(q.multiply(arr3v(p.axes[2]))).sum_axes(&[-1], false)),
            p.r,
        ),
        -1.0,
        1.0,
    );
    let u = s_add(
        &s_div(&ck(ops::atan2(&y, &x)), 2.0 * std::f64::consts::PI),
        0.5,
    );
    let v = s_div(&ck(ops::acos(&z)), std::f64::consts::PI);
    ck(ops::stack(&[&u, &v], -1))
}

// --- plane ------------------------------------------------------------------

pub fn plane_intersect(p: PlaneParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let n = arr3v(p.n);
    let denom = ck(ck(n.multiply(d)).sum_axes(&[-1], false));
    let t = ck(s_rsub(&ck(ck(n.multiply(o)).sum_axes(&[-1], false)), p.d).divide(&denom));
    let mask = ck(s_gt(&ck(denom.abs()), 1e-9).logical_and(s_gt(&t, 1e-6)));
    let n_rep = ck(ops::broadcast_to(&n, o.shape()));
    let n_out = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n_rep,
        ck(ops::zeros_like(&n_rep)),
    ));
    (t, n_out, mask)
}

pub fn plane_shadow(p: PlaneParams, o: &Array, d: &Array) -> (Array, Array) {
    let n = arr3v(p.n);
    let denom = ck(ck(n.multiply(d)).sum_axes(&[-1], false));
    let t = ck(s_rsub(&ck(ck(n.multiply(o)).sum_axes(&[-1], false)), p.d).divide(&denom));
    let mask = ck(s_gt(&ck(denom.abs()), 1e-9).logical_and(s_gt(&t, 1e-6)));
    (t, mask)
}

pub fn plane_uv(_p: PlaneParams, pos: &Array, _n: &Array) -> Array {
    ck(ops::stack(&[&col(pos, 0), &col(pos, 2)], -1))
}

// --- cylinder ---------------------------------------------------------------

fn cylinder_side(p: CylinderParams, o: &Array, d: &Array) -> (Array, Array, Array, Array) {
    let q = arr3v(p.q);
    let u = arr3v(p.u);
    let oc = ck(o.subtract(&q));
    let d_par = ck(ck(d.multiply(&u)).sum_axes(&[-1], true));
    let o_par = ck(ck(oc.multiply(&u)).sum_axes(&[-1], true));
    let d_p = ck(d.subtract(ck(d_par.multiply(&u))));
    let o_p = ck(oc.subtract(ck(o_par.multiply(&u))));
    let a = ck(ck(d_p.multiply(&d_p)).sum_axes(&[-1], false));
    let b = s_mul(&ck(ck(o_p.multiply(&d_p)).sum_axes(&[-1], false)), 2.0);
    let cq = s_sub(
        &ck(ck(o_p.multiply(&o_p)).sum_axes(&[-1], false)),
        p.r * p.r,
    );
    let disc = ck(ck(b.multiply(&b)).subtract(s_mul(&ck(a.multiply(&cq)), 4.0)));
    let valid = ck(s_gt(&a, 1e-12).logical_and(s_gt(&disc, 1e-12)));
    let sq = ck(s_max(&disc, 0.0).sqrt());
    let t1 = ck(ck(ck(b.negative()).subtract(&sq)).divide(s_mul(&a, 2.0)));
    let t2 = ck(ck(ck(b.negative()).add(&sq)).divide(s_mul(&a, 2.0)));
    let t = ck(ops::select(
        ck(valid.logical_and(s_gt(&t1, 1e-6))),
        &t1,
        &t2,
    ));
    let mask = ck(valid.logical_and(s_gt(&t, 1e-6)));
    let hit = ck(o_p.add(ck(ck(t.expand_dims(1)).multiply(&d_p))));
    let mut n = s_div(&hit, p.r);
    let inside = ck(mask.logical_and(s_le(&t1, 1e-6)));
    n = ck(ops::select(ck(inside.expand_dims(1)), ck(n.negative()), &n));
    (t, n, mask, o_par)
}

pub fn cylinder_intersect(p: CylinderParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (t, mut n, mask, o_par) = cylinder_side(p, o, d);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    if p.h < 0.0 {
        return (t, n, mask);
    }
    let h = p.h;
    let u = arr3v(p.u);
    let q = arr3v(p.q);
    let d_par = ck(ck(d.multiply(&u)).sum_axes(&[-1], true));
    let s = ck(o_par.add(ck(ck(t.expand_dims(1)).multiply(&d_par))));
    let side_ok = ck(mask.logical_and(s_le(&col(&ck(s.abs()), 0), h)));
    let denom = col(&d_par, 0);
    let cap_t = ck(ops::stack(
        &[
            &ck(s_rsub(&col(&o_par, 0), h).divide(&denom)),
            &ck(s_rsub(&col(&o_par, 0), -h).divide(&denom)),
        ],
        -1,
    ));
    let mut cap_ok = ck(ops::broadcast_to(
        ck(s_gt(&ck(denom.abs()), 1e-9).expand_dims(1)),
        &[o.shape()[0], 2],
    ));
    cap_ok = ck(cap_ok.logical_and(s_gt(&cap_t, 1e-6)));
    let p_cap =
        ck(ck(o.expand_dims(1)).add(ck(ck(cap_t.expand_dims(2)).multiply(ck(d.expand_dims(1))))));
    let rel = ck(p_cap.subtract(ck(ck(q.expand_dims(0)).expand_dims(0))));
    let u00 = ck(ck(u.expand_dims(0)).expand_dims(0));
    let lat = ck(rel.subtract(ck(
        ck(ck(rel.multiply(&u00)).sum_axes(&[-1], true)).multiply(&u00)
    )));
    cap_ok = ck(cap_ok.logical_and(s_le(
        &ck(ck(lat.multiply(&lat)).sum_axes(&[-1], false)),
        p.r * p.r,
    )));
    let n_cap1 =
        ck(ck(ck(ck(ops::sign(&denom)).negative()).expand_dims(1)).multiply(ck(u.expand_dims(0))));
    let n_cap = ck(ops::stack(&[&n_cap1, &n_cap1], 1));
    let t_all = ck(ops::stack(&[&t, &col(&cap_t, 0), &col(&cap_t, 1)], -1));
    let ok_all = ck(ops::stack(
        &[&side_ok, &col(&cap_ok, 0), &col(&cap_ok, 1)],
        -1,
    ));
    let t_eff = ck(ops::select(&ok_all, &t_all, crate::inf_like(&t_all)));
    let t_min = ck(t_eff.min_axes(&[-1], false));
    let idx = ck(ops::indexing::argmin_axis(&t_eff, -1, false));
    let n_all = ck(ops::stack(
        &[
            &n,
            &ck(n_cap.take_axis(Array::from_slice(&[0], &[]), 1)),
            &ck(n_cap.take_axis(Array::from_slice(&[1], &[]), 1)),
        ],
        1,
    ));
    let n_fin = ck(ck(n_all.take_along_axis(
        ck(ops::broadcast_to(
            ck(ck(idx.expand_dims(1)).expand_dims(2)),
            &[n.shape()[0], 1, 3],
        )),
        1,
    ))
    .take_axis(Array::from_slice(&[0], &[]), 1));
    let fin = ck(ck(t_min.is_finite()).logical_and(s_gt(&t_min, 1e-6)));
    (
        ck(ops::select(&fin, &t_min, &t)),
        ck(ops::select(
            ck(fin.expand_dims(1)),
            &n_fin,
            ck(ops::zeros_like(&n_fin)),
        )),
        fin,
    )
}

pub fn cylinder_shadow(p: CylinderParams, o: &Array, d: &Array) -> (Array, Array) {
    let (t, _, mask, o_par) = cylinder_side(p, o, d);
    if p.h < 0.0 {
        return (t, mask);
    }
    let h = p.h;
    let u = arr3v(p.u);
    let q = arr3v(p.q);
    let d_par = ck(ck(d.multiply(&u)).sum_axes(&[-1], true));
    let s = ck(o_par.add(ck(ck(t.expand_dims(1)).multiply(&d_par))));
    let side_ok = ck(mask.logical_and(s_le(&col(&ck(s.abs()), 0), h)));
    let denom = col(&d_par, 0);
    let cap_t = ck(ops::stack(
        &[
            &ck(s_rsub(&col(&o_par, 0), h).divide(&denom)),
            &ck(s_rsub(&col(&o_par, 0), -h).divide(&denom)),
        ],
        -1,
    ));
    let mut cap_ok = ck(ops::broadcast_to(
        ck(s_gt(&ck(denom.abs()), 1e-9).expand_dims(1)),
        &[o.shape()[0], 2],
    ));
    cap_ok = ck(cap_ok.logical_and(s_gt(&cap_t, 1e-6)));
    let p_cap =
        ck(ck(o.expand_dims(1)).add(ck(ck(cap_t.expand_dims(2)).multiply(ck(d.expand_dims(1))))));
    let rel = ck(p_cap.subtract(ck(ck(q.expand_dims(0)).expand_dims(0))));
    let u00 = ck(ck(u.expand_dims(0)).expand_dims(0));
    let lat = ck(rel.subtract(ck(
        ck(ck(rel.multiply(&u00)).sum_axes(&[-1], true)).multiply(&u00)
    )));
    cap_ok = ck(cap_ok.logical_and(s_le(
        &ck(ck(lat.multiply(&lat)).sum_axes(&[-1], false)),
        p.r * p.r,
    )));
    let t_all = ck(ops::stack(&[&t, &col(&cap_t, 0), &col(&cap_t, 1)], -1));
    let ok_all = ck(ops::stack(
        &[&side_ok, &col(&cap_ok, 0), &col(&cap_ok, 1)],
        -1,
    ));
    let t_eff = ck(ops::select(&ok_all, &t_all, crate::inf_like(&t_all)));
    let t_min = ck(t_eff.min_axes(&[-1], false));
    let fin = ck(ck(t_min.is_finite()).logical_and(s_gt(&t_min, 1e-6)));
    (ck(ops::select(&fin, &t_min, &t)), fin)
}

pub fn cylinder_uv(p: CylinderParams, pos: &Array, _n: &Array) -> Array {
    let q = arr3v(p.q);
    let axis = arr3v(p.u);
    let rel = ck(pos.subtract(&q));
    let axial = ck(ck(ck(rel.multiply(&axis)).sum_axes(&[-1], true)).multiply(&axis));
    let radial = ck(rel.subtract(&axial));
    let seed = arr3(1.0, 0.0, 0.0);
    let alt = arr3(0.0, 1.0, 0.0);
    let mut b1 = ck(ops::select(
        s_lt(
            &ck(ck(axis.take_axis(Array::from_slice(&[0], &[]), 0)).abs()),
            0.9,
        ),
        &seed,
        &alt,
    ));
    b1 = ck(b1.subtract(ck(ck(ck(b1.multiply(&axis)).sum(None)).multiply(&axis))));
    b1 = ck(b1.divide(ck(ck(ck(b1.multiply(&b1)).sum(None)).sqrt())));
    let ax = |i: i32| ck(axis.take_axis(Array::from_slice(&[i], &[]), 0));
    let b1c = |i: i32| ck(b1.take_axis(Array::from_slice(&[i], &[]), 0));
    let b2 = ck(ops::stack(
        &[
            &ck(ck(ax(1).multiply(b1c(2))).subtract(ck(ax(2).multiply(b1c(1))))),
            &ck(ck(ax(2).multiply(b1c(0))).subtract(ck(ax(0).multiply(b1c(2))))),
            &ck(ck(ax(0).multiply(b1c(1))).subtract(ck(ax(1).multiply(b1c(0))))),
        ],
        0,
    ));
    let u = s_add(
        &s_div(
            &ck(ops::atan2(
                ck(ck(radial.multiply(&b2)).sum_axes(&[-1], false)),
                ck(ck(radial.multiply(&b1)).sum_axes(&[-1], false)),
            )),
            2.0 * std::f64::consts::PI,
        ),
        0.5,
    );
    let v = s_div(
        &ck(ck(rel.multiply(&axis)).sum_axes(&[-1], false)),
        2.0 * p.r,
    );
    ck(ops::stack(&[&u, &v], -1))
}

// --- box --------------------------------------------------------------------

pub fn box_intersect(p: BoxParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let c = arr3v(p.c);
    let oc = ck(o.subtract(&c));
    let ax0 = arr3v(p.axes[0]);
    let ax1 = arr3v(p.axes[1]);
    let ax2 = arr3v(p.axes[2]);
    let op = ck(ops::stack(
        &[
            &ck(ck(oc.multiply(&ax0)).sum_axes(&[-1], false)),
            &ck(ck(oc.multiply(&ax1)).sum_axes(&[-1], false)),
            &ck(ck(oc.multiply(&ax2)).sum_axes(&[-1], false)),
        ],
        -1,
    ));
    let dp = ck(ops::stack(
        &[
            &ck(ck(d.multiply(&ax0)).sum_axes(&[-1], false)),
            &ck(ck(d.multiply(&ax1)).sum_axes(&[-1], false)),
            &ck(ck(d.multiply(&ax2)).sum_axes(&[-1], false)),
        ],
        -1,
    ));
    let inv = s_rdiv(&dp, 1.0);
    let half_a = arr3v(p.half);
    let t0 = ck(ck(inv.negative()).multiply(ck(op.add(&half_a))));
    let t1 = ck(ck(inv.negative()).multiply(ck(op.subtract(&half_a))));
    let tmin = ck(ops::minimum(&t0, &t1));
    let tmax = ck(ops::maximum(&t0, &t1));
    let t_entry = ck(tmin.max_axes(&[-1], false));
    let t_exit = ck(tmax.min_axes(&[-1], false));
    let valid = ck(ck(t_entry.lt(&t_exit)).logical_and(s_gt(&t_exit, 1e-6)));
    let i_entry = ck(ops::indexing::argmax_axis(&tmin, -1, false));
    let i_exit = ck(ops::indexing::argmin_axis(&tmax, -1, false));
    let inside_hit = ck(valid.logical_and(s_le(&t_entry, 1e-6)));
    let t = ck(ops::select(
        ck(valid.logical_and(ck(inside_hit.logical_not()))),
        &t_entry,
        &t_exit,
    ));
    let eye = ck(ops::eye::<f32>(3, Some(3), Some(0)));
    // entry-face normal opposes the ray; exit-face normal points along it
    // (same convention as box_crossings in csg.v)
    let n_e = ck(
        ck(eye.take_axis(&i_entry, 0)).multiply(ck(ck(ck(ops::sign(ck(ck(
            dp.take_along_axis(ck(i_entry.expand_dims(1)), -1)
        )
        .squeeze_axes(&[-1]))))
        .negative())
        .expand_dims(1))),
    );
    let n_x = ck(
        ck(eye.take_axis(&i_exit, 0)).multiply(ck(ck(ops::sign(ck(ck(
            dp.take_along_axis(ck(i_exit.expand_dims(1)), -1)
        )
        .squeeze_axes(&[-1]))))
        .expand_dims(1))),
    );
    let mut n = ck(ops::select(ck(inside_hit.expand_dims(1)), &n_x, &n_e));
    // rotate from the box's local axis frame to camera space
    n = crate::vecmat(&n, p.axes);
    n = ck(ops::select(
        ck(valid.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, valid)
}

pub fn box_shadow(p: BoxParams, o: &Array, d: &Array) -> (Array, Array) {
    let c = arr3v(p.c);
    let oc = ck(o.subtract(&c));
    let ax0 = arr3v(p.axes[0]);
    let ax1 = arr3v(p.axes[1]);
    let ax2 = arr3v(p.axes[2]);
    let op = ck(ops::stack(
        &[
            &ck(ck(oc.multiply(&ax0)).sum_axes(&[-1], false)),
            &ck(ck(oc.multiply(&ax1)).sum_axes(&[-1], false)),
            &ck(ck(oc.multiply(&ax2)).sum_axes(&[-1], false)),
        ],
        -1,
    ));
    let dp = ck(ops::stack(
        &[
            &ck(ck(d.multiply(&ax0)).sum_axes(&[-1], false)),
            &ck(ck(d.multiply(&ax1)).sum_axes(&[-1], false)),
            &ck(ck(d.multiply(&ax2)).sum_axes(&[-1], false)),
        ],
        -1,
    ));
    let inv = s_rdiv(&dp, 1.0);
    let half_a = arr3v(p.half);
    let t0 = ck(ck(inv.negative()).multiply(ck(op.add(&half_a))));
    let t1 = ck(ck(inv.negative()).multiply(ck(op.subtract(&half_a))));
    let tmin = ck(ops::minimum(&t0, &t1));
    let tmax = ck(ops::maximum(&t0, &t1));
    let t_entry = ck(tmin.max_axes(&[-1], false));
    let t_exit = ck(tmax.min_axes(&[-1], false));
    let valid = ck(ck(t_entry.lt(&t_exit)).logical_and(s_gt(&t_exit, 1e-6)));
    let inside_hit = ck(valid.logical_and(s_le(&t_entry, 1e-6)));
    let t = ck(ops::select(
        ck(valid.logical_and(ck(inside_hit.logical_not()))),
        &t_entry,
        &t_exit,
    ));
    (t, valid)
}

pub fn box_uv(p: BoxParams, pos: &Array, _n: &Array) -> Array {
    let c = arr3v(p.c);
    let q = ck(pos.subtract(&c));
    let ax0 = arr3v(p.axes[0]);
    let ax1 = arr3v(p.axes[1]);
    let ax2 = arr3v(p.axes[2]);
    let local = ck(ops::stack(
        &[
            &ck(ck(q.multiply(&ax0)).sum_axes(&[-1], false)),
            &ck(ck(q.multiply(&ax1)).sum_axes(&[-1], false)),
            &ck(ck(q.multiply(&ax2)).sum_axes(&[-1], false)),
        ],
        -1,
    ));
    let half_a = arr3v(p.half);
    let face = ck(ops::indexing::argmax_axis(
        ck(ck(local.divide(&half_a)).abs()),
        -1,
        false,
    ));
    let l0 = col(&local, 0);
    let l1 = col(&local, 1);
    let l2 = col(&local, 2);
    let x = ck(ops::select(s_eq(&face, 0.0), &l2, &l0));
    let y = ck(ops::select(s_eq(&face, 2.0), &l1, &l2));
    let sx = ck(ops::select(
        s_eq(&face, 0.0),
        ck(half_a.take_axis(Array::from_slice(&[2], &[]), 0)),
        ck(half_a.take_axis(Array::from_slice(&[0], &[]), 0)),
    ));
    let sy = ck(ops::select(
        s_eq(&face, 2.0),
        ck(half_a.take_axis(Array::from_slice(&[1], &[]), 0)),
        ck(half_a.take_axis(Array::from_slice(&[2], &[]), 0)),
    ));
    ck(ops::stack(
        &[
            &s_add(&ck(x.divide(s_mul(&sx, 2.0))), 0.5),
            &s_add(&ck(y.divide(s_mul(&sy, 2.0))), 0.5),
        ],
        -1,
    ))
}

// --- circle -----------------------------------------------------------------

pub fn circle_intersect(p: CircleParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let c = arr3v(p.c);
    let n = arr3v(p.n);
    let denom = ck(ck(n.multiply(d)).sum_axes(&[-1], false));
    let t = ck(ck(ck(n.multiply(ck(c.subtract(o)))).sum_axes(&[-1], false)).divide(&denom));
    let front = s_lt(&denom, 0.0);
    let hit = ck(o.add(ck(ck(t.expand_dims(1)).multiply(d))));
    let diff = ck(hit.subtract(&c));
    let in_disc = s_le(
        &ck(ck(diff.multiply(&diff)).sum_axes(&[-1], false)),
        p.r * p.r,
    );
    let mask =
        ck(ck(s_gt(&ck(denom.abs()), 1e-9).logical_and(s_gt(&t, 1e-6))).logical_and(&in_disc));
    let n2 = ck(ops::select(ck(front.expand_dims(1)), &n, ck(n.negative())));
    let n_out = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n2,
        ck(ops::zeros_like(&n2)),
    ));
    (t, n_out, mask)
}

pub fn circle_shadow(p: CircleParams, o: &Array, d: &Array) -> (Array, Array) {
    let c = arr3v(p.c);
    let n = arr3v(p.n);
    let denom = ck(ck(n.multiply(d)).sum_axes(&[-1], false));
    let t = ck(ck(ck(n.multiply(ck(c.subtract(o)))).sum_axes(&[-1], false)).divide(&denom));
    let hit = ck(o.add(ck(ck(t.expand_dims(1)).multiply(d))));
    let diff = ck(hit.subtract(&c));
    let in_disc = s_le(
        &ck(ck(diff.multiply(&diff)).sum_axes(&[-1], false)),
        p.r * p.r,
    );
    let mask =
        ck(ck(s_gt(&ck(denom.abs()), 1e-9).logical_and(s_gt(&t, 1e-6))).logical_and(&in_disc));
    (t, mask)
}

pub fn circle_uv(p: CircleParams, pos: &Array, _n: &Array) -> Array {
    let c = arr3v(p.c);
    let q = s_div(&ck(pos.subtract(&c)), 2.0 * p.r);
    ck(ops::stack(
        &[&s_add(&col(&q, 0), 0.5), &s_add(&col(&q, 1), 0.5)],
        -1,
    ))
}

// --- dispatch ---------------------------------------------------------------

pub fn geom_intersect(p: &GeometryParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    match p {
        GeometryParams::SphereParams(p) => sphere_intersect(*p, o, d),
        GeometryParams::PlaneParams(p) => plane_intersect(*p, o, d),
        GeometryParams::CylinderParams(p) => cylinder_intersect(*p, o, d),
        GeometryParams::BoxParams(p) => box_intersect(*p, o, d),
        GeometryParams::CircleParams(p) => circle_intersect(*p, o, d),
        GeometryParams::ConeParams(p) => crate::cone_intersect(*p, o, d),
        GeometryParams::TorusParams(p) => crate::torus_intersect(*p, o, d),
        GeometryParams::EllipsoidParams(p) => crate::ellipsoid_intersect(*p, o, d),
        GeometryParams::CyclideParams(p) => crate::cyclide_intersect(*p, o, d),
        GeometryParams::TrimeshParams(p) => crate::trimesh_intersect(p, o, d),
        GeometryParams::CsgParams(p) => crate::csg_intersect(p, o, d),
        GeometryParams::AffineParams(p) => crate::affine_intersect(p, o, d),
    }
}

pub fn geom_shadow(p: &GeometryParams, o: &Array, d: &Array) -> (Array, Array) {
    match p {
        GeometryParams::SphereParams(p) => sphere_shadow(*p, o, d),
        GeometryParams::PlaneParams(p) => plane_shadow(*p, o, d),
        GeometryParams::CylinderParams(p) => cylinder_shadow(*p, o, d),
        GeometryParams::BoxParams(p) => box_shadow(*p, o, d),
        GeometryParams::CircleParams(p) => circle_shadow(*p, o, d),
        GeometryParams::ConeParams(p) => crate::cone_shadow(*p, o, d),
        GeometryParams::TorusParams(p) => crate::torus_shadow(*p, o, d),
        GeometryParams::EllipsoidParams(p) => crate::ellipsoid_shadow(*p, o, d),
        GeometryParams::CyclideParams(p) => crate::cyclide_shadow(*p, o, d),
        GeometryParams::TrimeshParams(p) => crate::trimesh_shadow(p, o, d),
        GeometryParams::CsgParams(p) => crate::csg_shadow(p, o, d),
        GeometryParams::AffineParams(p) => crate::affine_shadow(p, o, d),
    }
}

pub fn geom_uv(p: &GeometryParams, pos: &Array, n: &Array) -> Array {
    match p {
        GeometryParams::SphereParams(p) => sphere_uv(*p, pos, n),
        GeometryParams::PlaneParams(p) => plane_uv(*p, pos, n),
        GeometryParams::CylinderParams(p) => cylinder_uv(*p, pos, n),
        GeometryParams::BoxParams(p) => box_uv(*p, pos, n),
        GeometryParams::CircleParams(p) => circle_uv(*p, pos, n),
        GeometryParams::ConeParams(p) => crate::cone_uv(*p, pos, n),
        GeometryParams::TorusParams(p) => crate::torus_uv(*p, pos, n),
        GeometryParams::EllipsoidParams(p) => crate::ellipsoid_uv(*p, pos, n),
        GeometryParams::CyclideParams(p) => crate::cyclide_uv(*p, pos, n),
        GeometryParams::TrimeshParams(p) => crate::trimesh_uv(p, pos, n),
        GeometryParams::CsgParams(p) => crate::csg_uv(p, pos, n),
        GeometryParams::AffineParams(p) => crate::affine_uv(p, pos, n),
    }
}

// geom_bounds returns the camera-space AABB (lo, hi), or none if unbounded.
pub fn geom_bounds(p: &GeometryParams) -> Option<[[f64; 3]; 2]> {
    match p {
        GeometryParams::SphereParams(p) => Some([lo(&p.c, p.r), hi(&p.c, p.r)]),
        GeometryParams::PlaneParams(_) => None,
        GeometryParams::CylinderParams(p) => {
            if p.h < 0.0 {
                return None;
            }
            Some(capsule_bounds(&p.q, &p.u, p.r, p.h))
        }
        GeometryParams::BoxParams(p) => Some(box_bounds(&p.c, &p.axes, &p.half)),
        GeometryParams::CircleParams(p) => Some([lo(&p.c, p.r), hi(&p.c, p.r)]),
        GeometryParams::ConeParams(p) => Some(crate::affine_bounds(
            [-p.r, -p.r, -p.h / 2.0],
            [p.r, p.r, p.h / 2.0],
            &p.a_fwd,
        )),
        GeometryParams::TorusParams(p) => {
            let e = p.major + p.minor;
            Some(crate::affine_bounds(
                [-e, -e, -p.minor],
                [e, e, p.minor],
                &p.a_fwd,
            ))
        }
        GeometryParams::EllipsoidParams(p) => Some(crate::affine_bounds(
            [-1.0, -1.0, -1.0],
            [1.0, 1.0, 1.0],
            &p.a_fwd,
        )),
        GeometryParams::CyclideParams(p) => {
            let r = p.d + p.c;
            Some(crate::affine_bounds(
                [p.shift[0] - p.a - r, p.shift[1] - p.b - r, p.shift[2] - r],
                [p.shift[0] + p.a + r, p.shift[1] + p.b + r, p.shift[2] + r],
                &p.a_fwd,
            ))
        }
        GeometryParams::TrimeshParams(p) => Some(crate::affine_bounds(p.lo, p.hi, &p.a_fwd)),
        GeometryParams::CsgParams(p) => crate::csg_bounds(p),
        GeometryParams::AffineParams(p) => affine_bounds_from_inner(p),
    }
}

fn affine_bounds_from_inner(p: &AffineParams) -> Option<[[f64; 3]; 2]> {
    let b = geom_bounds(&p.inner)?;
    Some(crate::affine_bounds(b[0], b[1], &p.a_fwd))
}

fn lo(c: &[f64; 3], r: f64) -> [f64; 3] {
    [c[0] - r, c[1] - r, c[2] - r]
}

fn hi(c: &[f64; 3], r: f64) -> [f64; 3] {
    [c[0] + r, c[1] + r, c[2] + r]
}

fn capsule_bounds(q: &[f64; 3], u: &[f64; 3], r: f64, h: f64) -> [[f64; 3]; 2] {
    let mut lo3 = [0.0; 3];
    let mut hi3 = [0.0; 3];
    for i in 0..3 {
        let e = u[i].abs() * h + r;
        lo3[i] = q[i] - e;
        hi3[i] = q[i] + e;
    }
    [lo3, hi3]
}

fn box_bounds(c: &[f64; 3], axes: &[[f64; 3]; 3], half: &[f64; 3]) -> [[f64; 3]; 2] {
    let mut lo3 = [0.0; 3];
    let mut hi3 = [0.0; 3];
    for i in 0..3 {
        let mut e = 0.0;
        for j in 0..3 {
            e += axes[j][i].abs() * half[j];
        }
        lo3[i] = c[i] - e;
        hi3[i] = c[i] + e;
    }
    [lo3, hi3]
}
