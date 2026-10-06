use cga_core::TrimeshParams;
use mlx_rs::ops;
use mlx_rs::Array;

use crate::mlxops::*;
use crate::{affine_normal, affine_to_local, col, inf_like, tri_mlx};

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

/// Peak-bytes budget for the (n_rays, f, 3) f32 temporaries of one ray chunk.
/// `trimesh_mt_all` keeps ~6 such temporaries live; the ray axis is chunked
/// so that rays×faces never materializes past this budget (the 2026-10 bench:
/// unchunked 307k rays × 1.9k faces already swap-thrashed a 24 GB machine).
pub(crate) const RAY_CHUNK_BYTES: usize = 2 * 1024 * 1024 * 1024;

fn ray_chunk_len(n_faces: usize, budget: usize) -> usize {
    // ~6 live (n,f,3) f32 temporaries inside trimesh_mt_all + reduce outputs.
    let per_ray = n_faces.max(1) * 3 * 4 * 6;
    (budget / per_ray.max(1)).max(1024)
}

fn trimesh_intersect_all(p: &TrimeshParams, o: &Array, d: &Array) -> (Array, Array, Array) {
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

pub(crate) fn trimesh_shadow_all(p: &TrimeshParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (tv0, te1, te2, tnrm) = tri_mlx(p);
    let (tall, _, _) = trimesh_mt_all(&tv0, &te1, &te2, &tnrm, &o_l, &d_u);
    let t_l = ck(tall.min_axes(&[-1], false));
    (ck(t_l.divide(col(&lam, 0))), ck(t_l.is_finite()))
}

/// Ray-axis chunked dispatch: split the (N,3) ray bundle into slices, run one
/// shot per slice eagerly, concatenate. Bitwise-equal to a single shot (every
/// ray flows through the identical op sequence exactly once).
fn chunk_rays<T>(
    n: usize,
    chunk: usize,
    o: &Array,
    d: &Array,
    f: impl Fn(&Array, &Array) -> T,
    cat: impl Fn(&[T]) -> T,
) -> T {
    if n <= chunk {
        return f(o, d);
    }
    let o = ck(o.contiguous());
    let d = ck(d.contiguous());
    o.eval().unwrap();
    d.eval().unwrap();
    let of: &[f32] = o.as_slice();
    let df: &[f32] = d.as_slice();
    let mut parts: Vec<T> = Vec::new();
    for i in 0..n.div_ceil(chunk) {
        let lo = i * chunk;
        let hi = (lo + chunk).min(n);
        let oc = Array::from_slice(&of[lo * 3..hi * 3], &[(hi - lo) as i32, 3]);
        let dc = Array::from_slice(&df[lo * 3..hi * 3], &[(hi - lo) as i32, 3]);
        parts.push(f(&oc, &dc));
    }
    cat(&parts)
}

pub fn trimesh_intersect(p: &TrimeshParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    self::trimesh_intersect_chunked(p, o, d, RAY_CHUNK_BYTES)
}

pub(crate) fn trimesh_intersect_chunked(
    p: &TrimeshParams,
    o: &Array,
    d: &Array,
    budget: usize,
) -> (Array, Array, Array) {
    let n = o.shape()[0] as usize;
    let chunk = ray_chunk_len(p.v0.len(), budget);
    chunk_rays::<(Array, Array, Array)>(
        n,
        chunk,
        o,
        d,
        |oc, dc| {
            let r = trimesh_intersect_all(p, oc, dc);
            r.0.eval().unwrap();
            r.1.eval().unwrap();
            r.2.eval().unwrap();
            r
        },
        |parts| {
            if parts.len() == 1 {
                return parts.first().unwrap().clone();
            }
            let ts: Vec<&Array> = parts.iter().map(|p| &p.0).collect();
            let ns: Vec<&Array> = parts.iter().map(|p| &p.1).collect();
            let ms: Vec<&Array> = parts.iter().map(|p| &p.2).collect();
            (
                ck(ops::concatenate(&ts, 0)),
                ck(ops::concatenate(&ns, 0)),
                ck(ops::concatenate(&ms, 0)),
            )
        },
    )
}

pub fn trimesh_shadow(p: &TrimeshParams, o: &Array, d: &Array) -> (Array, Array) {
    let n = o.shape()[0] as usize;
    let chunk = ray_chunk_len(p.v0.len(), RAY_CHUNK_BYTES);
    chunk_rays::<(Array, Array)>(
        n,
        chunk,
        o,
        d,
        |oc, dc| {
            let r = trimesh_shadow_all(p, oc, dc);
            r.0.eval().unwrap();
            r.1.eval().unwrap();
            r
        },
        |parts| {
            if parts.len() == 1 {
                return parts.first().unwrap().clone();
            }
            let ts: Vec<&Array> = parts.iter().map(|p| &p.0).collect();
            let ms: Vec<&Array> = parts.iter().map(|p| &p.1).collect();
            (ck(ops::concatenate(&ts, 0)), ck(ops::concatenate(&ms, 0)))
        },
    )
}

pub fn trimesh_uv(_p: &TrimeshParams, pos: &Array, _n: &Array) -> Array {
    ck(ops::zeros::<f32>(&[pos.shape()[0], 2]))
}

/// Summed signed solid angles (van Oosterom–Strackee) of every triangle seen
/// from each query point — 4π × the generalized winding number
/// (Jacobson 2013). `pts` are mesh-local, shape (n, 3); returns (n,).
/// Unlike +x parity counting this classifies non-watertight / open meshes
/// sanely: a small missing patch only shifts the sum by its solid-angle
/// fraction instead of flipping whole regions.
fn winding_sum(tv0: &Array, te1: &Array, te2: &Array, pts: &Array) -> Array {
    let a = ck(ck(tv0.expand_dims(0)).subtract(ck(pts.expand_dims(1)))); // (n,f,3)
    let b = ck(a.add(ck(te1.expand_dims(0))));
    let c = ck(a.add(ck(te2.expand_dims(0))));
    let la = ck(ck(ck(a.multiply(&a)).sum_axes(&[-1], false)).sqrt());
    let lb = ck(ck(ck(b.multiply(&b)).sum_axes(&[-1], false)).sqrt());
    let lc = ck(ck(ck(c.multiply(&c)).sum_axes(&[-1], false)).sqrt());
    let det = ck(ck(a.multiply(&cross3(&b, &c))).sum_axes(&[-1], false));
    let ab = ck(ck(a.multiply(&b)).sum_axes(&[-1], false));
    let bc = ck(ck(b.multiply(&c)).sum_axes(&[-1], false));
    let ca = ck(ck(c.multiply(&a)).sum_axes(&[-1], false));
    let den = ck(
        ck(ck(ck(la.multiply(&lb)).multiply(&lc)).add(&ck(ab.multiply(&lc))))
            .add(&ck(ck(bc.multiply(&la)).add(&ck(ca.multiply(&lb))))),
    );
    let omega = ck(ops::atan2(&det, &den));
    s_mul(&ck(omega.sum_axes(&[-1], false)), 2.0)
}

pub(crate) fn trimesh_winding(p: &TrimeshParams, pts: &Array) -> Array {
    let (tv0, te1, te2, _nrm) = tri_mlx(p);
    winding_sum(&tv0, &te1, &te2, pts)
}

/// Peak-bytes budget for the (n, f, 3) f32 temporaries of one winding chunk.
/// The CSG interval sampler calls `contains` with rays×(k+1) points in one
/// broadcast; beyond this budget the point axis is chunked and each chunk
/// evaluated eagerly (measured OOM limit of the rays×crossings×tris growth).
pub(crate) const WINDING_CHUNK_BYTES: usize = 256 * 1024 * 1024;

pub(crate) fn trimesh_winding_chunked(p: &TrimeshParams, pts: &Array, budget: usize) -> Array {
    let n = pts.shape()[0] as usize;
    // ~8 live (n,f,3) f32 temporaries inside winding_sum.
    let per_pt = p.v0.len().max(1) * 3 * 4 * 8;
    let chunk = (budget / per_pt.max(1)).max(1024);
    if n <= chunk {
        return trimesh_winding(p, pts);
    }
    pts.eval().unwrap();
    let flat: &[f32] = pts.as_slice();
    let (tv0, te1, te2, _nrm) = tri_mlx(p);
    let mut out: Vec<f32> = Vec::with_capacity(n);
    for c in flat.chunks(chunk * 3) {
        let m = (c.len() / 3) as i32;
        let w = winding_sum(&tv0, &te1, &te2, &Array::from_slice(c, &[m, 3]));
        w.eval().unwrap();
        out.extend_from_slice(w.as_slice::<f32>());
    }
    Array::from_slice(&out, &[n as i32])
}

/// Unchunked ray-axis crossings: one MT broadcast over all rays × faces.
fn trimesh_crossings_all(p: &TrimeshParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (tv0, te1, te2, tnrm) = tri_mlx(p);
    let (tall, nall, _) = trimesh_mt_all(&tv0, &te1, &te2, &tnrm, &o_l, &d_u);
    let n = tall.shape()[0];
    let f = tall.shape()[1];
    // Never index past the face axis: take k = min(f, 16), then pad to 16.
    let k = f.min(16);
    let firstk: Vec<i32> = (0..k).collect();
    let order = ck(ck(ops::argsort_axis(&tall, 1)).take_axis(Array::from_slice(&firstk, &[k]), 1));
    let mut ts = ck(ck(tall.take_along_axis(&order, 1)).divide(ck(col(&lam, 0).expand_dims(1))));
    let mut ns = ck(nall.take_along_axis(ck(order.expand_dims(2)), 1));
    if k < 16 {
        let pad = 16 - k;
        ts = ck(ops::concatenate(
            &[&ts, &ck(ops::full::<f32>(&[n, pad], &fs(f64::INFINITY)))],
            1,
        ));
        ns = ck(ops::concatenate(
            &[&ns, &ck(ops::zeros::<f32>(&[n, pad, 3]))],
            1,
        ));
    }
    ns = affine_normal(&ns, p.a_inv3);
    let valid = ck(ts.is_finite());
    (ts, ns, valid)
}

/// Crossings chunked along the ray axis under the same peak-bytes budget as
/// `trimesh_winding_chunked`.
pub(crate) fn trimesh_crossings_chunked(
    p: &TrimeshParams,
    o: &Array,
    d: &Array,
    budget: usize,
) -> (Array, Array, Array) {
    let n = o.shape()[0] as usize;
    // Per-ray live temporaries: ~10 (n,f) f32 arrays inside trimesh_mt_all,
    // plus tall/argsort order (f,i32) and nall (f,3).
    let per_ray = p.v0.len().max(1) * 4 * 16;
    let chunk = (budget / per_ray.max(1)).max(64);
    if n <= chunk {
        return trimesh_crossings_all(p, o, d);
    }
    o.eval().unwrap();
    d.eval().unwrap();
    let of: &[f32] = o.as_slice();
    let df: &[f32] = d.as_slice();
    let mut tsv: Vec<f32> = Vec::with_capacity(n * 16);
    let mut nsv: Vec<f32> = Vec::with_capacity(n * 48);
    let mut vsv: Vec<bool> = Vec::with_capacity(n * 16);
    for (oc, dc) in of.chunks(chunk * 3).zip(df.chunks(chunk * 3)) {
        let m = (oc.len() / 3) as i32;
        let (ts, ns, vs) = trimesh_crossings_all(
            p,
            &Array::from_slice(oc, &[m, 3]),
            &Array::from_slice(dc, &[m, 3]),
        );
        ts.eval().unwrap();
        ns.eval().unwrap();
        vs.eval().unwrap();
        tsv.extend_from_slice(ts.as_slice::<f32>());
        nsv.extend_from_slice(ns.as_slice::<f32>());
        vsv.extend_from_slice(vs.as_slice::<bool>());
    }
    (
        Array::from_slice(&tsv, &[n as i32, 16]),
        Array::from_slice(&nsv, &[n as i32, 16, 3]),
        Array::from_slice(&vsv, &[n as i32, 16]),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{Geometry, Multivector, TrimeshGeometry};

    fn tm_ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn tm_hit(g: &Geometry, o: [f64; 3], d: [f64; 3]) -> (f32, [f32; 3], bool) {
        let p = crate::geom_to_camera(g, &Multivector::identity());
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
        Geometry::TrimeshGeometry(TrimeshGeometry::new(
            &[[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [0.0, 2.0, 1.0]],
            &[[0, 1, 2]],
        ))
    }

    #[test]
    fn test_chunked_intersect_bitwise_equal() {
        // 小网格 + 多光线 + 极小预算 → 强制多块；结果必须与单发逐位一致。
        let g = tm_triangle();
        let p = match crate::geom_to_camera(&g, &Multivector::identity()) {
            cga_core::GeometryParams::TrimeshParams(tp) => tp,
            _ => panic!("trimesh expected"),
        };
        let n = 300usize;
        let od = vec![0f32; n * 3];
        let mut dd = vec![0f32; n * 3];
        for i in 0..n {
            let x = (i % 20) as f32 * 0.1 - 0.5;
            let y = (i / 20) as f32 * 0.1 - 0.5;
            dd[i * 3] = x * 0.1;
            dd[i * 3 + 1] = y * 0.1;
            dd[i * 3 + 2] = 1.0;
        }
        let o = Array::from_slice(&od, &[n as i32, 3]);
        let d = Array::from_slice(&dd, &[n as i32, 3]);
        let (t1, n1, m1) = trimesh_intersect_all(&p, &o, &d);
        // 预算 20_000B：chunk = 20000/(1面*72B) = 277 < 300 → 2 块
        let (t2, n2, m2) = trimesh_intersect_chunked(&p, &o, &d, 20_000);
        for arr in [&t1, &n1, &m1, &t2, &n2, &m2] {
            arr.eval().unwrap();
        }
        assert_eq!(t1.as_slice::<f32>(), t2.as_slice::<f32>());
        assert_eq!(n1.as_slice::<f32>(), n2.as_slice::<f32>());
        assert_eq!(m1.as_slice::<bool>(), m2.as_slice::<bool>());
    }

    #[test]
    fn test_chunked_shadow_bitwise_equal() {
        let g = tm_triangle();
        let p = match crate::geom_to_camera(&g, &Multivector::identity()) {
            cga_core::GeometryParams::TrimeshParams(tp) => tp,
            _ => panic!("trimesh expected"),
        };
        let n = 300usize;
        let mut od = vec![0f32; n * 3];
        let mut dd = vec![0f32; n * 3];
        for i in 0..n {
            od[i * 3 + 1] = -1.0;
            dd[i * 3] = (i % 20) as f32 * 0.01;
            dd[i * 3 + 1] = 1.0;
            dd[i * 3 + 2] = (i / 20) as f32 * 0.01;
        }
        let o = Array::from_slice(&od, &[n as i32, 3]);
        let d = Array::from_slice(&dd, &[n as i32, 3]);
        let (t1, m1) = trimesh_shadow_all(&p, &o, &d);
        let o2 = Array::from_slice(&od, &[n as i32, 3]);
        let d2 = Array::from_slice(&dd, &[n as i32, 3]);
        // 直接以相同预算走 chunk_rays（trimesh_shadow 用全局预算，此处验证分块机制）
        let (t2, m2) = chunk_rays(
            n,
            69,
            &o2,
            &d2,
            |oc, dc| trimesh_shadow_all(&p, oc, dc),
            |parts| {
                let ts: Vec<&Array> = parts.iter().map(|p| &p.0).collect();
                let ms: Vec<&Array> = parts.iter().map(|p| &p.1).collect();
                (ck(ops::concatenate(&ts, 0)), ck(ops::concatenate(&ms, 0)))
            },
        );
        for arr in [&t1, &m1, &t2, &m2] {
            arr.eval().unwrap();
        }
        assert_eq!(t1.as_slice::<f32>(), t2.as_slice::<f32>());
        assert_eq!(m1.as_slice::<bool>(), m2.as_slice::<bool>());
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
        let g = Geometry::TrimeshGeometry(TrimeshGeometry::new(
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
        assert!((f64::from(t) - 3.0).abs() < 1e-3);
    }

    #[test]
    fn test_trimesh_translated_by_motor() {
        let g = tm_triangle();
        let p = crate::geom_to_camera(&g, &Multivector::translator([0.0, 0.0, 2.0]));
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

    fn tm_cube() -> Geometry {
        // Closed unit cube [0,1]^3, globally inward orientation — the winding
        // backend classifies by |w|, so a consistent global flip must not
        // change inside/outside.
        let v = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        let f = [
            [0, 1, 2],
            [0, 2, 3],
            [4, 6, 5],
            [4, 7, 6],
            [0, 4, 5],
            [0, 5, 1],
            [1, 5, 6],
            [1, 6, 2],
            [2, 6, 7],
            [2, 7, 3],
            [3, 7, 4],
            [3, 4, 0],
        ];
        Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f))
    }

    #[test]
    fn test_trimesh_winding_closed_cube() {
        let g = tm_cube();
        let p = crate::geom_to_camera(&g, &Multivector::identity());
        let cga_core::GeometryParams::TrimeshParams(tp) = p else {
            panic!()
        };
        let pts = Array::from_slice(&[0.5f32, 0.5, 0.5, 5.0, 0.5, 0.5], &[2, 3]);
        let w = trimesh_winding(&tp, &pts);
        w.eval().unwrap();
        let ws = w.as_slice::<f32>();
        assert!(
            (ws[0].abs() - 4.0 * std::f32::consts::PI).abs() < 0.1,
            "center |ΣΩ| ≈ 4π, got {}",
            ws[0]
        );
        assert!(ws[1].abs() < 0.1, "far outside |ΣΩ| ≈ 0, got {}", ws[1]);
    }

    #[test]
    fn test_trimesh_winding_chunked_matches_unchunked() {
        let g = tm_cube();
        let p = crate::geom_to_camera(&g, &Multivector::identity());
        let cga_core::GeometryParams::TrimeshParams(tp) = p else {
            panic!()
        };
        let n = 3000usize; // > the 1024-point chunk floor with a tiny budget
        let mut flat = vec![0.0f32; n * 3];
        for i in 0..n {
            flat[i * 3] = (i % 13) as f32 * 0.11 - 0.6;
            flat[i * 3 + 1] = ((i / 13) % 7) as f32 * 0.19 - 0.5;
            flat[i * 3 + 2] = ((i / 91) % 5) as f32 * 0.23 - 0.4;
        }
        let pts = Array::from_slice(&flat, &[n as i32, 3]);
        let full = trimesh_winding_chunked(&tp, &pts, usize::MAX);
        let chunked = trimesh_winding_chunked(&tp, &pts, 1);
        full.eval().unwrap();
        chunked.eval().unwrap();
        assert_eq!(full.as_slice::<f32>(), chunked.as_slice::<f32>());
    }

    #[test]
    fn test_trimesh_crossings_chunked_matches_unchunked() {
        let g = tm_cube();
        let p = crate::geom_to_camera(&g, &Multivector::identity());
        let cga_core::GeometryParams::TrimeshParams(tp) = p else {
            panic!()
        };
        let n = 200usize; // > the 64-ray chunk floor with a tiny budget
        let mut of = vec![0.0f32; n * 3];
        let mut df = vec![0.0f32; n * 3];
        for i in 0..n {
            of[i * 3] = (i % 11) as f32 * 0.13 - 0.6;
            of[i * 3 + 1] = ((i / 11) % 7) as f32 * 0.17 - 0.4;
            of[i * 3 + 2] = 5.0 - (i % 3) as f32;
            let dz = -1.0f32;
            let dx = ((i % 5) as f32 - 2.0) * 0.01;
            let dy = ((i % 7) as f32 - 3.0) * 0.01;
            let inv = 1.0 / (dx * dx + dy * dy + dz * dz).sqrt();
            df[i * 3] = dx * inv;
            df[i * 3 + 1] = dy * inv;
            df[i * 3 + 2] = dz * inv;
        }
        let o = Array::from_slice(&of, &[n as i32, 3]);
        let d = Array::from_slice(&df, &[n as i32, 3]);
        let (tf, nf, vf) = trimesh_crossings_chunked(&tp, &o, &d, usize::MAX);
        let (tc, nc, vc) = trimesh_crossings_chunked(&tp, &o, &d, 1);
        tf.eval().unwrap();
        tc.eval().unwrap();
        nf.eval().unwrap();
        nc.eval().unwrap();
        vf.eval().unwrap();
        vc.eval().unwrap();
        assert_eq!(
            tf.as_slice::<f32>(),
            tc.as_slice::<f32>(),
            "ts 分块应逐位一致"
        );
        assert_eq!(
            nf.as_slice::<f32>(),
            nc.as_slice::<f32>(),
            "ns 分块应逐位一致"
        );
        assert_eq!(
            vf.as_slice::<bool>(),
            vc.as_slice::<bool>(),
            "valid 分块应逐位一致"
        );
    }
}
