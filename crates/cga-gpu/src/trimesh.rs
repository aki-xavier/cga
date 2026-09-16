// Triangle-mesh primitive: brute-force Möller–Trumbore over all faces (no BVH),
// with the affine ray-inverse transform (see affine.v).

use cga_core::TrimeshParams;
use mlx_rs::ops;
use mlx_rs::Array;

use crate::mlxops::*;
use crate::{affine_normal, affine_to_local, col, inf_like, tri_mlx};

// cross3 is the (...,3) row-wise cross product (mlx has no cross).
fn cross3(a: &Array, b: &Array) -> Array {
    let a0 = ck(a.take_axis(Array::from_slice(&[0], &[]), -1));
    let a1 = ck(a.take_axis(Array::from_slice(&[1], &[]), -1));
    let a2 = ck(a.take_axis(Array::from_slice(&[2], &[]), -1));
    let b0 = ck(b.take_axis(Array::from_slice(&[0], &[]), -1));
    let b1 = ck(b.take_axis(Array::from_slice(&[1], &[]), -1));
    let b2 = ck(b.take_axis(Array::from_slice(&[2], &[]), -1));
    ck(ops::stack(
        &[
            &ck(ck(a1.multiply(&b2)).subtract(ck(a2.multiply(&b1)))),
            &ck(ck(a2.multiply(&b0)).subtract(ck(a0.multiply(&b2)))),
            &ck(ck(a0.multiply(&b1)).subtract(ck(a1.multiply(&b0)))),
        ],
        -1,
    ))
}

pub(crate) fn trimesh_mt_all(
    v0: &Array,
    e1: &Array,
    e2: &Array,
    nrm: &Array,
    o: &Array,
    d: &Array,
) -> (Array, Array, Array) {
    let n = o.shape()[0];
    let f = v0.shape()[0];
    let v0c = ck(v0.expand_dims(0));
    let e1c = ck(e1.expand_dims(0));
    let e2c = ck(e2.expand_dims(0));
    let dc = ck(d.expand_dims(1));
    let p = cross3(&dc, &e2c);
    let det = ck(ck(e1c.multiply(&p)).sum_axes(&[-1], false));
    let ok = s_gt(&ck(det.abs()), 1e-10);
    let inv = s_rdiv(&ck(ops::select(&ok, &det, ck(ops::ones_like(&det)))), 1.0);
    let sv = ck(ck(o.expand_dims(1)).subtract(&v0c));
    let u = ck(ck(ck(sv.multiply(&p)).sum_axes(&[-1], false)).multiply(&inv));
    let q = cross3(&sv, &e1c);
    let v = ck(ck(ck(dc.multiply(&q)).sum_axes(&[-1], false)).multiply(&inv));
    let t = ck(ck(ck(e2c.multiply(&q)).sum_axes(&[-1], false)).multiply(&inv));
    let mut hit = ck(ck(ok.logical_and(s_ge(&u, -1e-9))).logical_and(s_ge(&v, -1e-9)));
    hit = ck(ck(hit.logical_and(s_le(&ck(u.add(&v)), 1.0 + 1e-9))).logical_and(s_gt(&t, 1e-6)));
    let tall = ck(ops::select(&hit, &t, inf_like(&t)));
    let valid = ck(tall.is_finite());
    let nall = ck(ops::broadcast_to(ck(nrm.expand_dims(0)), &[n, f, 3]));
    (tall, nall, valid)
}

pub fn trimesh_intersect(p: &TrimeshParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (tv0, te1, te2, tnrm) = tri_mlx(p);
    let (tall, nall, _) = trimesh_mt_all(&tv0, &te1, &te2, &tnrm, &o_l, &d_u);
    let t_l = ck(tall.min_axes(&[-1], false));
    let mask = ck(t_l.is_finite());
    let idx = ck(ops::indexing::argmin_axis(&tall, -1, false));
    let mut n_l = ck(ck(nall.take_along_axis(
        ck(ops::broadcast_to(
            ck(ck(idx.expand_dims(1)).expand_dims(2)),
            &[nall.shape()[0], 1, 3],
        )),
        1,
    ))
    .take_axis(Array::from_slice(&[0], &[]), 1));
    let cos_i = ck(ck(ck(d_u.multiply(&n_l)).sum_axes(&[-1], true)).negative());
    n_l = ck(ops::select(s_lt(&cos_i, 0.0), ck(n_l.negative()), &n_l));
    let t = ck(t_l.divide(col(&lam, 0)));
    let mut n = affine_normal(&n_l, p.a_inv3);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn trimesh_shadow(p: &TrimeshParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (tv0, te1, te2, tnrm) = tri_mlx(p);
    let (tall, _, _) = trimesh_mt_all(&tv0, &te1, &te2, &tnrm, &o_l, &d_u);
    let t_l = ck(tall.min_axes(&[-1], false));
    (ck(t_l.divide(col(&lam, 0))), ck(t_l.is_finite()))
}

pub fn trimesh_uv(_p: &TrimeshParams, pos: &Array, _n: &Array) -> Array {
    // v1: no mesh texture coordinates
    ck(ops::zeros::<f32>(&[pos.shape()[0], 2]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{motor_identity, translator, trimesh_geometry, Geometry};

    // Möller–Trumbore triangle-mesh intersection correctness (no BVH; brute force).

    fn tm_ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn tm_hit(g: &Geometry, o: [f64; 3], d: [f64; 3]) -> (f32, [f32; 3], bool) {
        let p = crate::geom_to_camera(g, &motor_identity());
        let (t, n, mask) =
            crate::geom_intersect(&p, &tm_ray(o[0], o[1], o[2]), &tm_ray(d[0], d[1], d[2]));
        t.eval().unwrap();
        n.eval().unwrap();
        mask.eval().unwrap();
        let nd = n.as_slice::<f32>();
        (
            t.item_cast::<f32>(),
            [nd[0], nd[1], nd[2]],
            mask.as_slice::<bool>()[0],
        )
    }

    fn tm_triangle() -> Geometry {
        // triangle in the z=1 plane, centroid (2/3, 2/3, 1), normal +z
        Geometry::TrimeshGeometry(trimesh_geometry(
            &[[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [0.0, 2.0, 1.0]],
            &[[0, 1, 2]],
        ))
    }

    #[test]
    fn test_trimesh_center_hit() {
        let g = tm_triangle();
        let (t, n, m) = tm_hit(&g, [2.0 / 3.0, 2.0 / 3.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.0).abs() < 1e-3);
        assert!(f64::from(n[0]).abs() < 1e-2);
        assert!(f64::from(n[1]).abs() < 1e-2);
        assert!((f64::from(n[2]) - 1.0).abs() < 1e-2);
    }

    #[test]
    fn test_trimesh_miss_outside() {
        let g = tm_triangle();
        let (_, _, m) = tm_hit(&g, [3.0, 3.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m);
    }

    #[test]
    fn test_trimesh_parallel_ray_miss() {
        let g = tm_triangle();
        let (_, _, m) = tm_hit(&g, [0.5, 0.5, 5.0], [1.0, 0.0, 0.0]);
        assert!(!m);
    }

    #[test]
    fn test_trimesh_backface_normal_flips() {
        let g = tm_triangle();
        let (t, n, m) = tm_hit(&g, [2.0 / 3.0, 2.0 / 3.0, -5.0], [0.0, 0.0, 1.0]);
        assert!(m);
        assert!((f64::from(t) - 6.0).abs() < 1e-3);
        assert!(f64::from(n[0]).abs() < 1e-2);
        assert!(f64::from(n[1]).abs() < 1e-2);
        assert!((f64::from(n[2]) + 1.0).abs() < 1e-2);
    }

    #[test]
    fn test_trimesh_nearest_of_two() {
        let g = Geometry::TrimeshGeometry(trimesh_geometry(
            &[
                [0.0, 0.0, 1.0],
                [2.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 0.0, 2.0],
                [2.0, 0.0, 2.0],
                [0.0, 2.0, 2.0],
            ],
            &[[0, 1, 2], [3, 4, 5]],
        ));
        let (t, _, m) = tm_hit(&g, [2.0 / 3.0, 2.0 / 3.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 3.0).abs() < 1e-3); // nearest face is at z=2
    }

    #[test]
    fn test_trimesh_translated_by_motor() {
        // local triangle at z=1, motor translates by +z=2 -> world z=3
        let g = tm_triangle();
        let p = crate::geom_to_camera(&g, &translator([0.0, 0.0, 2.0]));
        let (t, _, mask) = crate::geom_intersect(
            &p,
            &tm_ray(2.0 / 3.0, 2.0 / 3.0, 5.0),
            &tm_ray(0.0, 0.0, -1.0),
        );
        t.eval().unwrap();
        mask.eval().unwrap();
        assert!(mask.as_slice::<bool>()[0]);
        assert!((f64::from(t.item_cast::<f32>()) - 2.0).abs() < 1e-3);
    }
}
