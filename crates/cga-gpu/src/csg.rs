use cga_core::{
    mat3_transpose, BoxParams, ConeParams, CyclideParams, CylinderParams, EllipsoidParams,
    GeometryParams, Mat3, PlaneParams, SphereParams, TorusParams,
};
use mlx_rs::ops;
use mlx_rs::Array;

use crate::mlxops::*;
use crate::{
    affine_normal, affine_to_local, col, cone_local_interval, cyclide_local_contains,
    cyclide_local_crossings, inf_like, torus_local_contains, torus_local_crossings, vecmat,
};

#[inline]
pub(crate) fn affine_point_to_local(a_inv3: Mat3, t_inv: [f64; 3], pos: &Array) -> Array {
    ck(vecmat(pos, mat3_transpose(a_inv3)).add(arr3v(t_inv)))
}

fn sphere_crossings(p: SphereParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (c, b, cq, disc, valid, t1, t2) = crate::sphere_ray_quad(p, o, d);
    let p1 = ck(o.add(ck(ck(t1.expand_dims(1)).multiply(d))));
    let p2 = ck(o.add(ck(ck(t2.expand_dims(1)).multiply(d))));
    let n1 = s_div(&ck(p1.subtract(&c)), p.r);
    let n2 = s_div(&ck(p2.subtract(&c)), p.r);
    let ts = ck(ops::stack(
        &[
            &ck(ops::select(&valid, &t1, inf_like(&t1))),
            &ck(ops::select(&valid, &t2, inf_like(&t2))),
        ],
        -1,
    ));
    let ns = ck(ops::stack(&[&n1, &n2], 1));
    let vs = ck(ops::stack(&[&valid, &valid], -1));
    let scale = ck(ops::maximum(ck(b.multiply(&b)), s_mul(&ck(cq.abs()), 4.0)));
    let amb = crate::fallback::quad_ambig(&disc, &scale);
    crate::fallback::sphere_fallback(&p, o, d, &amb, ts, ns, vs)
}

fn sphere_contains(p: SphereParams, pos: &Array) -> Array {
    let c = arr3v(p.c);
    let q = ck(pos.subtract(&c));
    s_lt(&ck(ck(q.multiply(&q)).sum_axes(&[-1], false)), p.r * p.r)
}

fn plane_crossings(p: PlaneParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let n = arr3v(p.n);
    let denom = ck(ck(n.multiply(d)).sum_axes(&[-1], false));
    let safe = ck(ops::select(
        s_gt(&ck(denom.abs()), 1e-9),
        &denom,
        ck(ops::full_like(&denom, fs(1e-9), None)),
    ));
    let t = ck(s_rsub(&ck(ck(n.multiply(o)).sum_axes(&[-1], false)), p.d).divide(&safe));
    let valid = ck(s_gt(&ck(denom.abs()), 1e-9).expand_dims(1));
    let ts = ck(ops::select(
        &valid,
        ck(t.expand_dims(1)),
        ck(ops::full::<f32>(&[o.shape()[0], 1], &fs(f64::INFINITY))),
    ));
    let ns = ck(ops::broadcast_to(
        ck(ck(n.expand_dims(0)).expand_dims(0)),
        &[o.shape()[0], 1, 3],
    ));
    let amb = s_le(&ck(denom.abs()), 1e-9);
    crate::fallback::plane_fallback(&p, o, d, &amb, ts, ns, valid)
}

fn plane_contains(p: PlaneParams, pos: &Array) -> Array {
    let n = arr3v(p.n);
    s_lt(&ck(ck(pos.multiply(&n)).sum_axes(&[-1], false)), p.d)
}

fn cylinder_crossings(p: CylinderParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (_q, u, d_par, o_par, d_p, o_p) = crate::cylinder_local(p, o, d);
    let (a, b, cq, disc, side_valid, st0, st1) = crate::cylinder_quad(&d_p, &o_p, p.r);
    let p0 = ck(o_p.add(ck(ck(st0.expand_dims(1)).multiply(&d_p))));
    let p1 = ck(o_p.add(ck(ck(st1.expand_dims(1)).multiply(&d_p))));
    let n_s0 = s_div(&p0, p.r);
    let n_s1 = s_div(&p1, p.r);
    let inf = inf_like(&st0);
    let amb = ck(s_le(&a, 1e-9).logical_or(crate::fallback::quad_ambig(
        &disc,
        &ck(ops::maximum(
            ck(b.multiply(&b)),
            s_mul(&ck(ck(a.multiply(&cq)).abs()), 4.0),
        )),
    )));
    let (ts, ns, vs) = if p.h < 0.0 {
        let ts = ck(ops::stack(
            &[
                &ck(ops::select(&side_valid, &st0, &inf)),
                &ck(ops::select(&side_valid, &st1, &inf)),
            ],
            -1,
        ));
        let ns = ck(ops::stack(&[&n_s0, &n_s1], 1));
        let vs = ck(ops::stack(&[&side_valid, &side_valid], -1));
        (ts, ns, vs)
    } else {
        let h = p.h;
        let denom = col(&d_par, 0);
        let safe = ck(ops::select(
            s_gt(&ck(denom.abs()), 1e-9),
            &denom,
            ck(ops::full_like(&denom, fs(1e-9), None)),
        ));
        let t_plus = ck(s_rsub(&col(&o_par, 0), h).divide(&safe));
        let t_minus = ck(s_rsub(&col(&o_par, 0), -h).divide(&safe));
        let at0 = ck(ops::minimum(&t_plus, &t_minus));
        let at1 = ck(ops::maximum(&t_plus, &t_minus));
        let enter = ck(ops::maximum(&st0, &at0));
        let exit_ = ck(ops::minimum(&st1, &at1));
        let valid = ck(side_valid.logical_and(ck(enter.lt(&exit_))));
        let enter_cap = ck(at0.gt(&st0));
        let exit_cap = ck(at1.lt(&st1));
        let n_cap0 = ck(ops::select(
            ck(ck(t_plus.lt(&t_minus)).expand_dims(1)),
            ck(u.expand_dims(0)),
            ck(ck(u.negative()).expand_dims(0)),
        ));
        let n_cap1 = ck(ops::select(
            ck(ck(t_plus.gt(&t_minus)).expand_dims(1)),
            ck(u.expand_dims(0)),
            ck(ck(u.negative()).expand_dims(0)),
        ));
        let n0 = ck(ops::select(ck(enter_cap.expand_dims(1)), &n_cap0, &n_s0));
        let n1 = ck(ops::select(ck(exit_cap.expand_dims(1)), &n_cap1, &n_s1));
        let ts = ck(ops::stack(
            &[
                &ck(ops::select(&valid, &enter, &inf)),
                &ck(ops::select(&valid, &exit_, &inf)),
            ],
            -1,
        ));
        let ns = ck(ops::stack(&[&n0, &n1], 1));
        let vs = ck(ops::stack(&[&valid, &valid], -1));
        (ts, ns, vs)
    };
    crate::fallback::cylinder_fallback(&p, o, d, &amb, ts, ns, vs)
}

fn cylinder_contains(p: CylinderParams, pos: &Array) -> Array {
    let q = arr3v(p.q);
    let u = arr3v(p.u);
    let rel = ck(pos.subtract(&q));
    let s = ck(ck(rel.multiply(&u)).sum_axes(&[-1], true));
    let radial = ck(rel.subtract(ck(s.multiply(&u))));
    let mut inside = s_lt(
        &ck(ck(radial.multiply(&radial)).sum_axes(&[-1], false)),
        p.r * p.r,
    );
    if p.h >= 0.0 {
        inside = ck(inside.logical_and(s_le(&ck(col(&s, 0).abs()), p.h)));
    }
    inside
}

fn box_crossings(p: BoxParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (tmin, tmax, t_entry, t_exit, dp) = crate::box_slab(p, o, d);
    let valid = ck(t_entry.lt(&t_exit));
    let i_e = ck(ops::indexing::argmax_axis(&tmin, -1, false));
    let i_x = ck(ops::indexing::argmin_axis(&tmax, -1, false));
    let eye = ck(ops::eye::<f32>(3, Some(3), Some(0)));
    let dp_e = ck(dp.take_along_axis(ck(i_e.expand_dims(1)), -1));
    let dp_x = ck(dp.take_along_axis(ck(i_x.expand_dims(1)), -1));
    let mut n_e = ck(ck(eye.take_axis(&i_e, 0)).multiply(ck(ck(ops::sign(&dp_e)).negative())));
    let mut n_x = ck(ck(eye.take_axis(&i_x, 0)).multiply(ck(ops::sign(&dp_x))));
    n_e = vecmat(&n_e, p.axes);
    n_x = vecmat(&n_x, p.axes);
    let inf = inf_like(&t_entry);
    let ts = ck(ops::stack(
        &[
            &ck(ops::select(&valid, &t_entry, &inf)),
            &ck(ops::select(&valid, &t_exit, &inf)),
        ],
        -1,
    ));
    let ns = ck(ops::stack(&[&n_e, &n_x], 1));
    let vs = ck(ops::stack(&[&valid, &valid], -1));
    (ts, ns, vs)
}

fn box_contains(p: BoxParams, pos: &Array) -> Array {
    let c = arr3v(p.c);
    let q = ck(pos.subtract(&c));
    let ax0 = arr3v(p.axes[0]);
    let ax1 = arr3v(p.axes[1]);
    let ax2 = arr3v(p.axes[2]);
    let shape = q.shape();
    let mut inside = ck(ops::ones_dtype(
        &shape[..shape.len() - 1],
        mlx_rs::Dtype::Bool,
    ));
    let q0 = ck(ck(q.multiply(&ax0)).sum_axes(&[-1], false));
    let q1 = ck(ck(q.multiply(&ax1)).sum_axes(&[-1], false));
    let q2 = ck(ck(q.multiply(&ax2)).sum_axes(&[-1], false));
    inside = ck(ck(inside.logical_and(s_le(&ck(q0.abs()), p.half[0])))
        .logical_and(s_le(&ck(q1.abs()), p.half[1])));
    inside = ck(inside.logical_and(s_le(&ck(q2.abs()), p.half[2])));
    inside
}

pub fn cone_crossings(p: ConeParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (enter, exit_, valid, n0, n1, amb) = cone_local_interval(p.r, p.h, &o_l, &d_u);
    let inf = inf_like(&enter);
    let ts = ck(ops::stack(
        &[
            &ck(ops::select(&valid, &enter, &inf)),
            &ck(ops::select(&valid, &exit_, &inf)),
        ],
        -1,
    ));
    let mut ns = ck(ops::stack(&[&n0, &n1], 1));
    ns = affine_normal(&ns, p.a_inv3);
    let ts = ck(ts.divide(ck(col(&lam, 0).expand_dims(1))));
    let vs = ck(ops::stack(&[&valid, &valid], -1));
    crate::fallback::cone_fallback(&p, o, d, &amb, ts, ns, vs)
}

fn cone_contains(p: ConeParams, pos: &Array) -> Array {
    let p_l = affine_point_to_local(p.a_inv3, p.t_inv, pos);
    let k2 = 1.0 + (p.r / p.h) * (p.r / p.h);
    let s = s_sub(&col(&p_l, 2), p.h / 2.0);
    let f = ck(ck(
        ck(ck(col(&p_l, 0).square()).add(ck(col(&p_l, 1).square()))).add(ck(s.multiply(&s)))
    )
    .subtract(s_mul(&ck(s.multiply(&s)), k2)));
    ck(ck(s_le(&f, 0.0).logical_and(s_ge(&s, -p.h))).logical_and(s_le(&s, 0.0)))
}

fn ellipsoid_crossings(p: EllipsoidParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (b, cq, disc, valid, t1, t2) = crate::ellipsoid_ray_quad(&o_l, &d_u);
    let p1 = ck(o_l.add(ck(ck(t1.expand_dims(1)).multiply(&d_u))));
    let p2 = ck(o_l.add(ck(ck(t2.expand_dims(1)).multiply(&d_u))));
    let inf = inf_like(&t1);
    let ts = ck(ck(ops::stack(
        &[
            &ck(ops::select(&valid, &t1, &inf)),
            &ck(ops::select(&valid, &t2, &inf)),
        ],
        -1,
    ))
    .divide(ck(col(&lam, 0).expand_dims(1))));
    let mut ns = ck(ops::stack(&[&p1, &p2], 1));
    ns = affine_normal(&ns, p.a_inv3);
    let vs = ck(ops::stack(&[&valid, &valid], -1));
    let scale = ck(ops::maximum(ck(b.multiply(&b)), s_mul(&ck(cq.abs()), 4.0)));
    let amb = crate::fallback::quad_ambig(&disc, &scale);
    crate::fallback::ellipsoid_fallback(&p, o, d, &amb, ts, ns, vs)
}

fn ellipsoid_contains(p: EllipsoidParams, pos: &Array) -> Array {
    let p_l = affine_point_to_local(p.a_inv3, p.t_inv, pos);
    s_lt(&ck(ck(p_l.multiply(&p_l)).sum_axes(&[-1], false)), 1.0)
}

pub fn torus_crossings(p: TorusParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, mut ns, valid, amb) = torus_local_crossings(p.major, p.minor, p.arc, &o_l, &d_u);
    ns = affine_normal(&ns, p.a_inv3);
    let ts = ck(ts.divide(ck(col(&lam, 0).expand_dims(1))));
    crate::fallback::torus_fallback(&p, o, d, &amb, ts, ns, valid)
}

fn torus_contains(p: TorusParams, pos: &Array) -> Array {
    let p_l = affine_point_to_local(p.a_inv3, p.t_inv, pos);
    torus_local_contains(p.major, p.minor, p.arc, &p_l)
}

pub fn cyclide_crossings(p: CyclideParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, mut ns, valid, amb) = cyclide_local_crossings(p.a, p.b, p.d, p.c, p.shift, &o_l, &d_u);
    ns = affine_normal(&ns, p.a_inv3);
    let ts = ck(ts.divide(ck(col(&lam, 0).expand_dims(1))));
    crate::fallback::cyclide_fallback(&p, o, d, &amb, ts, ns, valid)
}

fn cyclide_contains(p: CyclideParams, pos: &Array) -> Array {
    let p_l = affine_point_to_local(p.a_inv3, p.t_inv, pos);
    cyclide_local_contains(p.a, p.b, p.d, p.c, p.shift, &p_l)
}

pub fn geom_crossings(p: &GeometryParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    match p {
        GeometryParams::SphereParams(p) => sphere_crossings(*p, o, d),
        GeometryParams::PlaneParams(p) => plane_crossings(*p, o, d),
        GeometryParams::CylinderParams(p) => cylinder_crossings(*p, o, d),
        GeometryParams::BoxParams(p) => box_crossings(*p, o, d),
        GeometryParams::ConeParams(p) => cone_crossings(*p, o, d),
        GeometryParams::EllipsoidParams(p) => ellipsoid_crossings(*p, o, d),
        GeometryParams::TorusParams(p) => torus_crossings(*p, o, d),
        GeometryParams::CyclideParams(p) => cyclide_crossings(*p, o, d),
        GeometryParams::CircleParams(_) => panic!("circle is not a solid (no crossings)"),
        GeometryParams::CsgParams(p) => crate::csg_crossings(p, o, d),
        GeometryParams::AffineParams(p) => crate::affine_crossings(p, o, d),
    }
}

pub fn geom_contains(p: &GeometryParams, pos: &Array) -> Array {
    match p {
        GeometryParams::SphereParams(p) => sphere_contains(*p, pos),
        GeometryParams::PlaneParams(p) => plane_contains(*p, pos),
        GeometryParams::CylinderParams(p) => cylinder_contains(*p, pos),
        GeometryParams::BoxParams(p) => box_contains(*p, pos),
        GeometryParams::ConeParams(p) => cone_contains(*p, pos),
        GeometryParams::EllipsoidParams(p) => ellipsoid_contains(*p, pos),
        GeometryParams::TorusParams(p) => torus_contains(*p, pos),
        GeometryParams::CyclideParams(p) => cyclide_contains(*p, pos),
        GeometryParams::CircleParams(_) => panic!("circle is not a solid (no contains)"),
        GeometryParams::CsgParams(p) => crate::csg_contains(p, pos),
        GeometryParams::AffineParams(p) => crate::affine_contains(p, pos),
    }
}

#[cfg(test)]
mod tests {
    use crate::{test_contains, test_hit};
    use cga_core::{
        BoxGeometry, CsgGeometry, CsgOp, EllipsoidGeometry, Geometry, Multivector, PlaneGeometry,
        SphereGeometry,
    };

    #[test]
    fn test_csg_difference() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
            ],
        ));
        let (t, m) = test_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.0).abs() < 1e-4);
    }

    #[test]
    fn test_csg_intersection() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Intersection,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
            ],
        ));
        let (t, m) = test_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.5).abs() < 1e-4);
    }

    #[test]
    fn test_csg_halfspace() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Intersection,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            ],
        ));
        let (t, m) = test_hit(&g, [0.0, 5.0, 0.0], [0.0, -1.0, 0.0]);
        assert!(m);
        assert!((f64::from(t) - 5.0).abs() < 1e-4);
    }

    #[test]
    fn test_csg_nested() {
        let inner = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
            ],
        ));
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Intersection,
            vec![
                inner,
                Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            ],
        ));
        let (t, m) = test_hit(&g, [0.0, -0.2, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - (5.0 - 0.96f64.sqrt())).abs() < 1e-3);
    }

    #[test]
    fn test_csg_union_scaled() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Union,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::EllipsoidGeometry(EllipsoidGeometry::new(1.0, 1.0, 3.0)),
            ],
        ));
        let (t, m) = test_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 2.0).abs() < 1e-3);
    }

    #[test]
    fn test_csg_union_contains() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Union,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::BoxGeometry(BoxGeometry::new(4.0, 4.0, 4.0)),
            ],
        ));

        assert_eq!(
            test_contains(
                &g,
                &Multivector::identity(),
                &[[0.9, 0.0, 0.0], [1.5, 0.0, 0.0], [3.0, 0.0, 0.0]]
            ),
            [true, true, false]
        );
    }

    #[test]
    fn test_csg_difference_contains() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::BoxGeometry(BoxGeometry::new(4.0, 4.0, 4.0)),
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
            ],
        ));

        assert_eq!(
            test_contains(
                &g,
                &Multivector::identity(),
                &[[1.5, 0.0, 0.0], [0.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
            ),
            [true, false, false]
        );
    }

    #[test]
    fn test_csg_intersection_contains() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Intersection,
            vec![
                Geometry::BoxGeometry(BoxGeometry::new(4.0, 4.0, 4.0)),
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
            ],
        ));

        assert_eq!(
            test_contains(
                &g,
                &Multivector::identity(),
                &[[0.5, 0.0, 0.0], [1.5, 0.0, 0.0], [3.0, 0.0, 0.0]]
            ),
            [true, false, false]
        );
    }

    #[test]
    fn test_csg_nested_contains() {
        let inner = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::BoxGeometry(BoxGeometry::new(4.0, 4.0, 4.0)),
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
            ],
        ));

        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Intersection,
            vec![
                inner,
                Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            ],
        ));
        assert_eq!(
            test_contains(
                &g,
                &Multivector::identity(),
                &[[1.5, -0.5, 0.0], [1.5, 0.5, 0.0]]
            ),
            [true, false]
        );
    }
}
