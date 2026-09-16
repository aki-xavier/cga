// geom_kernels — mlx batch kernels extracted from the cga root package.

use cga_core::{
    affine_from_motor, e1, e2, e3, mat3_new, mat3_transpose, motor_identity, point,
    sphere_from_dual, AffineGeometry, AffineParams, CsgOp, CsgParams, Geometry, GeometryParams,
    Mat3, Multivector, TrimeshParams,
};
use mlx_rs::ops;
use mlx_rs::Array;

use crate::mlxops::*;
use crate::{col, identity3, inf_like, vec3_unit};

pub fn mat3_to_mlx(m: Mat3) -> Array {
    Array::from_slice(
        &[
            m[0][0] as f32,
            m[0][1] as f32,
            m[0][2] as f32,
            m[1][0] as f32,
            m[1][1] as f32,
            m[1][2] as f32,
            m[2][0] as f32,
            m[2][1] as f32,
            m[2][2] as f32,
        ],
        &[3, 3],
    )
}

pub fn vecmat(v: &Array, m: Mat3) -> Array {
    let mm = mat3_to_mlx(m);
    ck(ck(ck(v.expand_dims(-1)).multiply(&mm)).sum_axes(&[-2], false))
}

pub fn affine_to_local(
    a_inv3: Mat3,
    t_inv: [f64; 3],
    o: &Array,
    d: &Array,
) -> (Array, Array, Array) {
    let a3t = mat3_transpose(a_inv3);
    let t3 = arr3v(t_inv);
    let o_l = ck(vecmat(o, a3t).add(&t3));
    let d_l = vecmat(d, a3t);
    let mut lam = ck(ck(ck(d_l.multiply(&d_l)).sum_axes(&[-1], true)).sqrt());
    lam = ck(ops::select(
        s_gt(&lam, 1e-12),
        &lam,
        ck(ops::ones_like(&lam)),
    ));
    let d_u = ck(d_l.divide(&lam));
    (o_l, d_u, lam)
}

pub fn affine_normal(n_l: &Array, a_inv3: Mat3) -> Array {
    let n = vecmat(n_l, a_inv3);
    let norm = ck(ck(ck(n.multiply(&n)).sum_axes(&[-1], true)).sqrt());
    ck(n.divide(ck(ops::select(
        s_gt(&norm, 1e-12),
        &norm,
        ck(ops::ones_like(&norm)),
    ))))
}

pub fn affine_intersect(p: &AffineParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (t_l, n_l, mask) = crate::geom_intersect(&p.inner, &o_l, &d_u);
    let t = ck(t_l.divide(col(&lam, 0)));
    let mut n = affine_normal(&n_l, p.a_inv3);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn affine_shadow(p: &AffineParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (t_l, mask) = crate::geom_shadow(&p.inner, &o_l, &d_u);
    (ck(t_l.divide(col(&lam, 0))), mask)
}

pub fn affine_uv(p: &AffineParams, pos: &Array, n: &Array) -> Array {
    let p_l = crate::affine_point_to_local(p.a_inv3, p.t_inv, pos);
    crate::geom_uv(&p.inner, &p_l, n)
}

pub fn affine_crossings(p: &AffineParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, mut ns, valid) = crate::geom_crossings(&p.inner, &o_l, &d_u);
    ns = affine_normal(&ns, p.a_inv3);
    (ck(ts.divide(ck(col(&lam, 0).expand_dims(1)))), ns, valid)
}

pub fn affine_contains(p: &AffineParams, pos: &Array) -> Array {
    let p_l = crate::affine_point_to_local(p.a_inv3, p.t_inv, pos);
    crate::geom_contains(&p.inner, &p_l)
}

pub(crate) fn csg_crossings(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let mut ts_l: Vec<Array> = Vec::new();
    let mut ns_l: Vec<Array> = Vec::new();
    let mut vs_l: Vec<Array> = Vec::new();
    for cp in &p.children {
        let (t, n, v) = crate::geom_crossings(cp, o, d);
        ts_l.push(t);
        ns_l.push(n);
        vs_l.push(v);
    }
    let refs_t: Vec<&Array> = ts_l.iter().collect();
    let refs_n: Vec<&Array> = ns_l.iter().collect();
    let refs_v: Vec<&Array> = vs_l.iter().collect();
    (
        ck(ops::concatenate(&refs_t, 1)),
        ck(ops::concatenate(&refs_n, 1)),
        ck(ops::concatenate(&refs_v, 1)),
    )
}

pub(crate) fn csg_contains(p: &CsgParams, pos: &Array) -> Array {
    if p.op == CsgOp::Difference {
        let first = crate::geom_contains(&p.children[0], pos);
        let mut rest = ck(ops::zeros_like(&first));
        for cp in &p.children[1..] {
            rest = ck(rest.logical_or(crate::geom_contains(cp, pos)));
        }
        return ck(first.logical_and(ck(rest.logical_not())));
    }
    let mut acc = crate::geom_contains(&p.children[0], pos);
    for cp in &p.children[1..] {
        let cc = crate::geom_contains(cp, pos);
        acc = if p.op == CsgOp::Union {
            ck(acc.logical_or(&cc))
        } else {
            ck(acc.logical_and(&cc))
        };
    }
    acc
}

fn csg_nearest_surface(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (ts, ns, _) = csg_crossings(p, o, d);
    let order = ck(ops::argsort_axis(&ts, 1));
    let ts_s = ck(ts.take_along_axis(&order, 1));
    let ns_s = ck(ns.take_along_axis(ck(order.expand_dims(2)), 1));
    let ts_f = ck(ops::select(
        ck(ts_s.is_finite()),
        &ts_s,
        ck(ops::zeros_like(&ts_s)),
    ));
    let delta = 1e-4;
    let p_plus = ck(ck(o.expand_dims(1)).add(ck(
        ck(s_add(&ts_f, delta).expand_dims(2)).multiply(ck(d.expand_dims(1)))
    )));
    let p_minus = ck(ck(o.expand_dims(1)).add(ck(
        ck(s_sub(&ts_f, delta).expand_dims(2)).multiply(ck(d.expand_dims(1)))
    )));
    let in_plus = csg_contains(p, &p_plus);
    let in_minus = csg_contains(p, &p_minus);
    let flip = ck(ck(ck(in_plus.ne(&in_minus)).logical_and(s_gt(&ts_s, 1e-6)))
        .logical_and(ck(ts_s.is_finite())));
    let cand = ck(ops::select(&flip, &ts_s, inf_like(&ts_s)));
    let t = ck(cand.min_axes(&[1], false));
    let mask = ck(t.is_finite());
    let idx = ck(ops::indexing::argmin_axis(&cand, 1, false));
    let mut n = ck(ck(ns_s.take_along_axis(
        ck(ops::broadcast_to(
            ck(ck(idx.expand_dims(1)).expand_dims(2)),
            &[ns_s.shape()[0], 1, 3],
        )),
        1,
    ))
    .take_axis(Array::from_slice(&[0], &[]), 1));
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn csg_intersect(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    csg_nearest_surface(p, o, d)
}

pub fn csg_shadow(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array) {
    let (t, _, mask) = csg_nearest_surface(p, o, d);
    (t, mask)
}

pub fn csg_uv(p: &CsgParams, pos: &Array, n: &Array) -> Array {
    let mut uv = ck(ops::zeros::<f32>(&[pos.shape()[0], 2]));
    let mut found = ck(ops::zeros_dtype(&[pos.shape()[0]], mlx_rs::Dtype::Bool));
    let delta = 1e-4;
    for cp in &p.children {
        let bp = ck(pos.add(s_mul(n, delta)));
        let bm = ck(pos.subtract(s_mul(n, delta)));
        let boundary = ck(crate::geom_contains(cp, &bp).ne(crate::geom_contains(cp, &bm)));
        let pick = ck(boundary.logical_and(ck(found.logical_not())));
        let uv_c = crate::geom_uv(cp, pos, n);
        uv = ck(ops::select(ck(pick.expand_dims(1)), &uv_c, &uv));
        found = ck(found.logical_or(&pick));
    }
    uv
}

pub fn geom_to_camera(g: &Geometry, m: &Multivector) -> GeometryParams {
    match g {
        Geometry::SphereGeometry(g) => {
            let s = m.apply(&g.blade);
            let (c, r) = sphere_from_dual(&s);
            GeometryParams::SphereParams(cga_core::SphereParams {
                c,
                r,
                axes: [
                    vec3_unit(m.apply(&e1()).euclidean_vector()),
                    vec3_unit(m.apply(&e2()).euclidean_vector()),
                    vec3_unit(m.apply(&e3()).euclidean_vector()),
                ],
            })
        }
        Geometry::PlaneGeometry(g) => {
            let pi = m.apply(&g.blade);
            GeometryParams::PlaneParams(cga_core::PlaneParams {
                n: vec3_unit(pi.euclidean_vector()),
                d: pi.einf_coeff(),
            })
        }
        Geometry::CylinderGeometry(g) => GeometryParams::CylinderParams(cga_core::CylinderParams {
            q: m.apply(&point(0.0, 0.0, 0.0)).coords(),
            u: vec3_unit(m.apply(&e3()).euclidean_vector()),
            r: g.radius,
            h: g.half,
        }),
        Geometry::BoxGeometry(g) => {
            let hf = g.half;
            GeometryParams::BoxParams(cga_core::BoxParams {
                c: m.apply(&point(0.0, 0.0, 0.0)).coords(),
                axes: [
                    vec3_unit(m.apply(&e1()).euclidean_vector()),
                    vec3_unit(m.apply(&e2()).euclidean_vector()),
                    vec3_unit(m.apply(&e3()).euclidean_vector()),
                ],
                half: hf,
            })
        }
        Geometry::CircleGeometry(g) => GeometryParams::CircleParams(cga_core::CircleParams {
            c: m.apply(&point(0.0, 0.0, 0.0)).coords(),
            n: vec3_unit(m.apply(&e3()).euclidean_vector()),
            r: g.radius,
        }),
        Geometry::ConeGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(*m, identity3());
            GeometryParams::ConeParams(cga_core::ConeParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                r: g.radius,
                h: g.height,
            })
        }
        Geometry::TorusGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(*m, identity3());
            GeometryParams::TorusParams(cga_core::TorusParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                major: g.major,
                minor: g.minor,
            })
        }
        Geometry::EllipsoidGeometry(g) => {
            let rr = g.radii;
            let diag = mat3_new([rr[0], 0.0, 0.0], [0.0, rr[1], 0.0], [0.0, 0.0, rr[2]]);
            let (ai, ti, af) = affine_from_motor(*m, diag);
            GeometryParams::EllipsoidParams(cga_core::EllipsoidParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
            })
        }
        Geometry::CyclideGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(*m, identity3());
            let sh = g.shift;
            GeometryParams::CyclideParams(cga_core::CyclideParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                a: g.a,
                b: g.b,
                d: g.d,
                c: (g.a * g.a - g.b * g.b).sqrt(),
                shift: sh,
            })
        }
        Geometry::TrimeshGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(*m, identity3());
            let glo = g.lo;
            let ghi = g.hi;
            GeometryParams::TrimeshParams(TrimeshParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                v0: g.v0.clone(),
                e1: g.e1.clone(),
                e2: g.e2.clone(),
                nrm: g.nrm.clone(),
                lo: glo,
                hi: ghi,
            })
        }
        Geometry::CsgGeometry(g) => {
            let mut ch: Vec<GeometryParams> = Vec::new();
            for c in &g.children {
                ch.push(geom_to_camera(c, m));
            }
            GeometryParams::CsgParams(CsgParams {
                op: g.op,
                children: ch,
            })
        }
        Geometry::AffineGeometry(g) => GeometryParams::AffineParams(affine_to_camera(g, m)),
    }
}

pub fn affine_to_camera(g: &AffineGeometry, m: &Multivector) -> AffineParams {
    let ip = geom_to_camera(&g.inner[0], &motor_identity());
    let full = m.gp(&g.motor);
    let (ai, ti, af) = affine_from_motor(full, g.linear);
    AffineParams {
        inner: Box::new(ip),
        a_inv3: ai,
        t_inv: ti,
        a_fwd: af,
    }
}

pub fn csg_bounds(p: &CsgParams) -> Option<[[f64; 3]; 2]> {
    let mut bnds: Vec<Option<[[f64; 3]; 2]>> = Vec::new();
    for cp in &p.children {
        bnds.push(crate::geom_bounds(cp));
    }
    if p.op == CsgOp::Difference {
        return bnds[0];
    }
    let mut bounded: Vec<[[f64; 3]; 2]> = Vec::new();
    for bb in bnds.iter().flatten() {
        bounded.push(*bb);
    }
    if bounded.is_empty() {
        return None;
    }
    if p.op == CsgOp::Union {
        let mut bmin = [0.0; 3];
        let mut bmax = [0.0; 3];
        let mut first = true;
        for bb in &bounded {
            if first {
                bmin = bb[0];
                bmax = bb[1];
                first = false;
            } else {
                for i in 0..3 {
                    if bb[0][i] < bmin[i] {
                        bmin[i] = bb[0][i];
                    }
                    if bb[1][i] > bmax[i] {
                        bmax[i] = bb[1][i];
                    }
                }
            }
        }
        return Some([bmin, bmax]);
    }
    // intersection
    let mut bmin = [0.0; 3];
    let mut bmax = [0.0; 3];
    let mut first = true;
    for bb in &bounded {
        if first {
            bmin = bb[0];
            bmax = bb[1];
            first = false;
        } else {
            for i in 0..3 {
                if bb[0][i] > bmin[i] {
                    bmin[i] = bb[0][i];
                }
                if bb[1][i] < bmax[i] {
                    bmax[i] = bb[1][i];
                }
            }
        }
    }
    Some([bmin, bmax])
}

pub(crate) fn to_f32_3(v: &[[f64; 3]]) -> Vec<f32> {
    let mut out = vec![0.0f32; v.len() * 3];
    for i in 0..v.len() {
        out[i * 3] = v[i][0] as f32;
        out[i * 3 + 1] = v[i][1] as f32;
        out[i * 3 + 2] = v[i][2] as f32;
    }
    out
}

// tri_mlx converts the CPU TrimeshParams fields to mlx arrays for the kernels.
pub(crate) fn tri_mlx(p: &TrimeshParams) -> (Array, Array, Array, Array) {
    (
        Array::from_slice(&to_f32_3(&p.v0), &[p.v0.len() as i32, 3]),
        Array::from_slice(&to_f32_3(&p.e1), &[p.e1.len() as i32, 3]),
        Array::from_slice(&to_f32_3(&p.e2), &[p.e2.len() as i32, 3]),
        Array::from_slice(&to_f32_3(&p.nrm), &[p.nrm.len() as i32, 3]),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{
        affine_geometry, box_geometry, cone_geometry, csg_geometry, decompose_rigid, extrude,
        motor_rotor, translator, trimesh_geometry,
    };

    // Regression tests for geometry under non-identity transforms:
    //   - cone_contains / trimesh_contains now map query points back to the
    //     local frame (they used to ignore a_inv3/t_inv);
    //   - box_intersect now rotates normals into camera space and flips the
    //     exit-face normal for inside starts (matching box_crossings).

    fn tf_ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn tf_contains(g: &Geometry, m: &Multivector, pts: &[[f64; 3]]) -> Vec<bool> {
        let p = geom_to_camera(g, m);
        let mut flat: Vec<f32> = Vec::with_capacity(pts.len() * 3);
        for q in pts {
            flat.push(q[0] as f32);
            flat.push(q[1] as f32);
            flat.push(q[2] as f32);
        }
        let pos = Array::from_slice(&flat, &[pts.len() as i32, 3]);
        let got = crate::geom_contains(&p, &pos);
        got.eval().unwrap();
        got.as_slice::<bool>().to_vec()
    }

    fn tf_hit(g: &Geometry, m: &Multivector, o: [f64; 3], d: [f64; 3]) -> (f32, [f32; 3], bool) {
        let p = geom_to_camera(g, m);
        let (t, n, mask) =
            crate::geom_intersect(&p, &tf_ray(o[0], o[1], o[2]), &tf_ray(d[0], d[1], d[2]));
        n.eval().unwrap();
        t.eval().unwrap();
        mask.eval().unwrap();
        let nd = n.as_slice::<f32>();
        (
            t.item_cast::<f32>(),
            [nd[0], nd[1], nd[2]],
            mask.as_slice::<bool>()[0],
        )
    }

    #[test]
    fn test_moved_cone_contains() {
        let g = Geometry::ConeGeometry(cone_geometry(1.0, 2.0));
        // cone translated +1 in x: (1.2,0,0) -> local (0.2,0,0), inside;
        // (1.6,0,0) -> local (0.6,0,0), outside (radius at s=-1 is 0.5)
        assert_eq!(
            tf_contains(
                &g,
                &translator([1.0, 0.0, 0.0]),
                &[[1.2, 0.0, 0.0], [1.6, 0.0, 0.0]]
            ),
            [true, false]
        );
        // the identity case the old code already got right
        assert_eq!(
            tf_contains(&g, &motor_identity(), &[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]]),
            [true, false]
        );
    }

    #[test]
    fn test_moved_mesh_contains() {
        let (verts, faces) = extrude(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], 1.0);
        let g = Geometry::TrimeshGeometry(trimesh_geometry(&verts, &faces));
        // mesh translated +5 in x: (5.5,0.4,0.5) inside; (5.5,1.5,0.5) outside.
        // (y=0.4 avoids the side-quad diagonal, where the parity ray would
        // double-count the two triangles sharing the edge.)
        assert_eq!(
            tf_contains(
                &g,
                &translator([5.0, 0.0, 0.0]),
                &[[5.5, 0.4, 0.5], [5.5, 1.5, 0.5]]
            ),
            [true, false]
        );
        assert_eq!(
            tf_contains(&g, &motor_identity(), &[[0.5, 0.4, 0.5], [1.5, 0.5, 0.5]]),
            [true, false]
        );
    }

    #[test]
    fn test_csg_moved_cone_contains() {
        // realistic path: cone as a CSG child, the whole node moved
        let g = Geometry::CsgGeometry(csg_geometry(
            CsgOp::Union,
            vec![
                Geometry::ConeGeometry(cone_geometry(1.0, 2.0)),
                Geometry::ConeGeometry(cone_geometry(1.0, 2.0)),
            ],
        ));
        assert_eq!(
            tf_contains(
                &g,
                &translator([1.0, 0.0, 0.0]),
                &[[1.2, 0.0, 0.0], [1.6, 0.0, 0.0]]
            ),
            [true, false]
        );
    }

    #[test]
    fn test_csg_moved_mesh_contains() {
        let (verts, faces) = extrude(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], 1.0);
        let g = Geometry::CsgGeometry(csg_geometry(
            CsgOp::Union,
            vec![
                Geometry::TrimeshGeometry(trimesh_geometry(&verts, &faces)),
                Geometry::TrimeshGeometry(trimesh_geometry(&verts, &faces)),
            ],
        ));
        assert_eq!(
            tf_contains(
                &g,
                &translator([5.0, 0.0, 0.0]),
                &[[5.5, 0.4, 0.5], [5.5, 1.5, 0.5]]
            ),
            [true, false]
        );
    }

    #[test]
    fn test_rotated_box_normal() {
        // box rotated 30 deg about x; a ray straight down -z hits the tilted local
        // +z face (not an edge at this angle): plane n.x = 0.5 with
        // n = R_x(30) . [0,0,1] = [0, -sin(30), cos(30)], so
        // t = (0.866 * 5 - 0.5) / 0.866 = 4.4226
        let g = Geometry::BoxGeometry(box_geometry(1.0, 1.0, 1.0));
        let m = motor_rotor([1.0, 0.0, 0.0], std::f64::consts::PI / 6.0);
        let (t, n, hit) = tf_hit(&g, &m, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(hit);
        assert!((f64::from(t) - 4.4226).abs() < 1e-3);
        assert!(f64::from(n[0]).abs() < 1e-2);
        assert!((f64::from(n[1]) + 0.5).abs() < 1e-2);
        assert!((f64::from(n[2]) - 3.0f64.sqrt() / 2.0).abs() < 1e-2);
    }

    #[test]
    fn test_box_inside_exit_normal() {
        // ray starts inside an axis-aligned box and exits through the +x face:
        // the outward normal is +x (used to be flipped to -x)
        let g = Geometry::BoxGeometry(box_geometry(1.0, 1.0, 1.0));
        let (t, n, hit) = tf_hit(&g, &motor_identity(), [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        assert!(hit);
        assert!((f64::from(t) - 0.5).abs() < 1e-3);
        assert!((f64::from(n[0]) - 1.0).abs() < 1e-2);
        assert!(f64::from(n[1]).abs() < 1e-2);
        assert!(f64::from(n[2]).abs() < 1e-2);
    }

    #[test]
    fn test_box_entry_normal_unchanged() {
        // axis-aligned box from outside: identical behaviour before/after the fix
        let g = Geometry::BoxGeometry(box_geometry(1.0, 1.0, 1.0));
        let (t, n, hit) = tf_hit(&g, &motor_identity(), [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(hit);
        assert!((f64::from(t) - 4.5).abs() < 1e-3);
        assert!(f64::from(n[0]).abs() < 1e-2);
        assert!(f64::from(n[1]).abs() < 1e-2);
        assert!((f64::from(n[2]) - 1.0).abs() < 1e-2);
    }

    // --- extended modeling (gpu/extended_modeling_test.v) -------------------------
    // (glb / cgs tests live with mesh_io_gltf.rs / scene_lang.rs)

    fn em_ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn em_hit(g: &Geometry, o: [f64; 3], d: [f64; 3]) -> (f32, [f32; 3], bool) {
        let p = geom_to_camera(g, &motor_identity());
        let (t, n, mask) =
            crate::geom_intersect(&p, &em_ray(o[0], o[1], o[2]), &em_ray(d[0], d[1], d[2]));
        n.eval().unwrap();
        t.eval().unwrap();
        mask.eval().unwrap();
        let nd = n.as_slice::<f32>();
        (
            t.item_cast::<f32>(),
            [nd[0], nd[1], nd[2]],
            mask.as_slice::<bool>()[0],
        )
    }

    fn em_contains(g: &Geometry, pts: &[[f64; 3]]) -> Vec<bool> {
        let p = geom_to_camera(g, &motor_identity());
        let mut flat: Vec<f32> = Vec::with_capacity(pts.len() * 3);
        for q in pts {
            flat.push(q[0] as f32);
            flat.push(q[1] as f32);
            flat.push(q[2] as f32);
        }
        let pos = Array::from_slice(&flat, &[pts.len() as i32, 3]);
        let got = crate::geom_contains(&p, &pos);
        got.eval().unwrap();
        got.as_slice::<bool>().to_vec()
    }

    // --- affine ------------------------------------------------------------------

    #[test]
    fn test_affine_scaled_sphere() {
        let g = Geometry::AffineGeometry(affine_geometry(
            Geometry::SphereGeometry(cga_core::sphere_geometry(1.0)),
            mat3_new([2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        ));
        let (t, n, m) = em_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.0).abs() < 1e-5);
        assert!(f64::from(n[0]).abs() < 1e-4);
        assert!(f64::from(n[1]).abs() < 1e-4);
        assert!((f64::from(n[2]) - 1.0).abs() < 1e-4);
        let (_, _, m2) = em_hit(&g, [3.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m2);
    }

    #[test]
    fn test_affine_decompose_rigid_roundtrip() {
        let m = translator([1.0, 2.0, 3.0]).gp(&motor_rotor([0.0, 1.0, 0.0], 0.7));
        let (motor2, lin) = decompose_rigid(m.to_matrix());
        let mut err = 0.0f64;
        for (i, row) in lin.iter().enumerate() {
            for (j, &v) in row.iter().enumerate() {
                let want = if i == j { 1.0 } else { 0.0 };
                let e = (v - want).abs();
                if e > err {
                    err = e;
                }
            }
        }
        assert!(err < 1e-4);
        let tm = motor2.to_matrix();
        assert!((tm[3] - 1.0).abs() < 1e-4);
        assert!((tm[7] - 2.0).abs() < 1e-4);
        assert!((tm[11] - 3.0).abs() < 1e-4);
    }

    // --- new primitives ----------------------------------------------------------

    #[test]
    fn test_cone_side_ray() {
        // r=1 h=2 (k=0.5): at z=-0.5 (s=-1.5) the radius is 0.75 -> t = 4.25
        let g = Geometry::ConeGeometry(cone_geometry(1.0, 2.0));
        let (t, _, m) = em_hit(&g, [5.0, 0.0, -0.5], [-1.0, 0.0, 0.0]);
        assert!(m);
        assert!((f64::from(t) - 4.25).abs() < 1e-4);
    }

    #[test]
    fn test_cone_contains() {
        let g = Geometry::ConeGeometry(cone_geometry(1.0, 2.0));
        assert_eq!(
            em_contains(&g, &[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]]),
            [true, false]
        );
    }

    #[test]
    fn test_torus_rays() {
        let g = Geometry::TorusGeometry(cga_core::torus_geometry(1.0, 0.3));
        let (_, _, m) = em_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m);
        let (t2, n2, m2) = em_hit(&g, [1.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m2);
        assert!((f64::from(t2) - 4.7).abs() < 1e-3);
        assert!(f64::from(n2[0]).abs() < 1e-2);
        assert!(f64::from(n2[1]).abs() < 1e-2);
        assert!((f64::from(n2[2]) - 1.0).abs() < 1e-2);
    }

    #[test]
    fn test_torus_contains() {
        let g = Geometry::TorusGeometry(cga_core::torus_geometry(1.0, 0.3));
        assert_eq!(
            em_contains(&g, &[[1.0, 0.0, 0.1], [0.0, 0.0, 0.0]]),
            [true, false]
        );
    }

    #[test]
    fn test_ellipsoid_is_scaled_sphere() {
        let g = Geometry::EllipsoidGeometry(cga_core::ellipsoid_geometry(2.0, 1.0, 1.0));
        let (t, _, m) = em_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.0).abs() < 1e-4); // z semi-axis = 1
    }

    // --- modeling builders -------------------------------------------------------

    #[test]
    fn test_earclip_l_shape() {
        let l = [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 2.0],
            [2.0, 2.0],
            [2.0, 4.0],
            [0.0, 4.0],
        ];
        assert_eq!(cga_core::triangulate(&l).len(), 4); // 6-vertex L -> 4 triangles
    }

    #[test]
    fn test_extrude_hit_and_contains() {
        let (verts, faces) = extrude(
            &[
                [0.0, 0.0],
                [4.0, 0.0],
                [4.0, 2.0],
                [2.0, 2.0],
                [2.0, 4.0],
                [0.0, 4.0],
            ],
            1.5,
        );
        let g = Geometry::TrimeshGeometry(trimesh_geometry(&verts, &faces));
        let (t, _, m) = em_hit(&g, [1.0, 1.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 3.5).abs() < 1e-5); // top cap z = 1.5
        let (_, _, m2) = em_hit(&g, [3.0, 3.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m2);
        assert_eq!(
            em_contains(&g, &[[1.0, 0.9, 0.6], [3.0, 3.0, 0.75]]),
            [true, false]
        );
    }

    #[test]
    fn test_loft_between_squares() {
        let (verts, faces) = cga_core::loft(
            &[
                vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
                vec![[0.4, 0.4], [1.6, 0.4], [1.6, 1.6], [0.4, 1.6]],
            ],
            &[0.0, 1.0],
        );
        assert_eq!(verts.len(), 8);
        let g = Geometry::TrimeshGeometry(trimesh_geometry(&verts, &faces));
        let (t, _, m) = em_hit(&g, [1.0, 1.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.0).abs() < 1e-4); // top cap z = 1
    }

    // --- mesh IO -----------------------------------------------------------------

    #[test]
    fn test_obj_roundtrip() {
        let (verts, faces) = extrude(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]], 1.0);
        cga_core::save_obj(
            "/tmp/cga_em_obj.obj",
            &[cga_core::ObjMesh {
                vertices: verts.clone(),
                faces: faces.clone(),
                ..Default::default()
            }],
        );
        let (v2, f2) = cga_core::load_obj("/tmp/cga_em_obj.obj").unwrap();
        assert_eq!(v2.len(), verts.len());
        assert_eq!(f2.len(), faces.len());
        for i in 0..verts.len() {
            assert!((v2[i][0] - verts[i][0]).abs() < 1e-9);
            assert!((v2[i][1] - verts[i][1]).abs() < 1e-9);
            assert!((v2[i][2] - verts[i][2]).abs() < 1e-9);
        }
        for i in 0..faces.len() {
            assert_eq!(f2[i], faces[i]);
        }
        let _ = std::fs::remove_file("/tmp/cga_em_obj.obj");
    }

    // --- CGS v3 ------------------------------------------------------------------
    // (test_cgs_glb_roundtrip lives with mesh_io_gltf.rs; the cgs_load-dependent
    // tests below were enabled once scene_lang.rs landed)

    fn geom_kind(g: &Geometry) -> &'static str {
        match g {
            Geometry::CsgGeometry(_) => "csg",
            Geometry::ConeGeometry(_) => "cone",
            Geometry::TorusGeometry(_) => "torus",
            Geometry::EllipsoidGeometry(_) => "ellipsoid",
            Geometry::TrimeshGeometry(_) => "mesh",
            Geometry::SphereGeometry(_) => "sphere",
            Geometry::BoxGeometry(_) => "box",
            Geometry::CylinderGeometry(_) => "cylinder",
            Geometry::PlaneGeometry(_) => "plane",
            _ => "other",
        }
    }

    #[test]
    fn test_cgs_modifier_ordering() {
        let (sc, _) = crate::cgs_load("translate([10,0,0]) scale(2) sphere(r=1);", "");
        assert_eq!(sc.objects.len(), 1);
        let p = sc.objects[0].position;
        assert!((p[0] - 10.0).abs() < 1e-6);
        assert!(p[1].abs() < 1e-6);
        assert!(p[2].abs() < 1e-6);
        let (sc2, _) =
            crate::cgs_load("mirror(axis=[1,0,0]) translate([2.5,0,0]) sphere(r=1);", "");
        let p2 = sc2.objects[0].position;
        assert!((p2[0] + 2.5).abs() < 1e-6);
        assert!(p2[1].abs() < 1e-6);
        assert!(p2[2].abs() < 1e-6);
    }

    #[test]
    fn test_cgs_csg_block_and_new_primitives() {
        let text = "difference() { box(s=[2,2,2]); cylinder(r=0.5, h=4); }\n\
                    cone(r=1, h=2);\n\
                    torus(R=1, r=0.3);\n\
                    ellipsoid(radii=[1,2,3]);\n\
                    extrude(profile=[[0,0],[1,0],[1,1],[0,1]], h=0.5);\n\
                    p1 = [[0,0],[1,0],[1,1],[0,1]];\n\
                    p2 = [[0.2,0.2],[0.8,0.2],[0.8,0.8],[0.2,0.8]];\n\
                    loft(profiles=[p1, p2], zs=[0, 0.5]);";
        let (sc, _) = crate::cgs_load(text, "");
        assert_eq!(sc.objects.len(), 6);
        assert_eq!(geom_kind(&sc.objects[0].geometry), "csg");
        assert_eq!(geom_kind(&sc.objects[1].geometry), "cone");
        assert_eq!(geom_kind(&sc.objects[2].geometry), "torus");
        assert_eq!(geom_kind(&sc.objects[3].geometry), "ellipsoid");
        assert_eq!(geom_kind(&sc.objects[4].geometry), "mesh");
        assert_eq!(geom_kind(&sc.objects[5].geometry), "mesh");
    }

    #[test]
    fn test_cgs_gltf_mesh() {
        // save a GLB then load it back through the CGS mesh() primitive
        let (verts, faces) = extrude(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]], 1.0);
        crate::save_glb(
            "/tmp/cga_cgs.glb",
            &[crate::GltfMeshIn {
                vertices: verts,
                faces,
                transform: None,
                color: None,
            }],
        );
        let (sc, _) = crate::cgs_load("mesh(file=\"cga_cgs.glb\");", "/tmp");
        assert_eq!(sc.objects.len(), 1);
        assert_eq!(geom_kind(&sc.objects[0].geometry), "mesh");
        let _ = std::fs::remove_file("/tmp/cga_cgs.glb");
    }
}
