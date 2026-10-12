//! CPU f64 certified fallback for the f32 crossing guards.
//!
//! GPU crossings keep the f32 fast path; rays whose discriminant lies inside
//! the cancellation band (or whose leading coefficient degenerates) are
//! re-decided on CPU f64 via `certify::QuadRoots` / `certify::Quartic`.
//! `Double` roots are tangencies and classify as no crossing;
//! `Unknown`/`Indeterminate` abstain (no crossing — the render path leaves
//! the pixel to anti-aliasing).

use cga_core::{
    vec3_add, vec3_dot, vec3_scale, vec3_sub, ConeParams, CyclideParams, CylinderParams,
    EllipsoidParams, Mat3, PlaneParams, SphereParams, TorusParams,
};
use mlx_rs::ops;
use mlx_rs::Array;

use crate::certify::{QuadRoots, Quartic};
use crate::mlxops::*;
use crate::tol;

#[derive(Debug, Clone, Copy)]
pub(crate) struct RowHit {
    pub t: f64,
    pub n: [f64; 3],
}

// 局部系搬移（`local_ray`/`world_normal`/`vunit`/mat3 向量乘）的唯一实现在
// `cga_core::ray`；本模块只做数组管道（挑出歧义光线 → 逐光线调 core 的系数/法线 →
// 交给 `certify` 的精确求根 → 回填）。

/// `|disc| ≤ DEGENERATE_ULPS·ε_f32·scale` — inside this band the f32 sign of
/// the discriminant is not decidable (catastrophic cancellation between b²
/// and 4ac). `scale` must be max(b², 4|a·c|), computed elementwise.
pub(crate) fn quad_ambig(disc: &Array, scale: &Array) -> Array {
    let band = s_mul(
        &ck(ops::maximum(scale, fs(1e-30))),
        (tol::DEGENERATE_ULPS as f64) * (f32::EPSILON as f64),
    );
    ck(ck(disc.abs()).le(&band))
}

/// Indices of the rays flagged by an ambiguity mask.
pub(crate) fn ambiguous_indices(mask: &Array) -> Vec<usize> {
    mask.eval().unwrap();
    mask.as_slice::<bool>()
        .iter()
        .enumerate()
        .filter_map(|(i, &b)| if b { Some(i) } else { None })
        .collect()
}

fn read_rays(o: &Array, d: &Array) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    o.eval().unwrap();
    d.eval().unwrap();
    let os: &[f32] = o.as_slice();
    let ds: &[f32] = d.as_slice();
    let to_f64 = |s: &[f32]| -> Vec<[f64; 3]> {
        s.chunks(3)
            .map(|c| [f64::from(c[0]), f64::from(c[1]), f64::from(c[2])])
            .collect()
    };
    (to_f64(os), to_f64(ds))
}

/// Replace the crossing rows of the flagged rays with the certified f64
/// results. Slots beyond `hits.len()` become invalid (+∞ t, zero normal).
fn splice(
    ts: &Array,
    ns: &Array,
    vs: &Array,
    rows: &[(usize, Vec<RowHit>)],
) -> (Array, Array, Array) {
    let n = ts.shape()[0] as usize;
    let k = ts.shape()[1] as usize;
    ts.eval().unwrap();
    ns.eval().unwrap();
    vs.eval().unwrap();
    let mut tv: Vec<f32> = ts.as_slice::<f32>().to_vec();
    let mut nv: Vec<f32> = ns.as_slice::<f32>().to_vec();
    let mut vv: Vec<bool> = vs.as_slice::<bool>().to_vec();
    for (i, hits) in rows {
        for j in 0..k {
            let slot = i * k + j;
            match hits.get(j) {
                Some(h) if h.t.is_finite() => {
                    tv[slot] = h.t as f32;
                    nv[slot * 3] = h.n[0] as f32;
                    nv[slot * 3 + 1] = h.n[1] as f32;
                    nv[slot * 3 + 2] = h.n[2] as f32;
                    vv[slot] = true;
                }
                _ => {
                    tv[slot] = f32::INFINITY;
                    nv[slot * 3] = 0.0;
                    nv[slot * 3 + 1] = 0.0;
                    nv[slot * 3 + 2] = 0.0;
                    vv[slot] = false;
                }
            }
        }
    }
    (
        Array::from_slice(&tv, &[n as i32, k as i32]),
        Array::from_slice(&nv, &[n as i32, k as i32, 3]),
        Array::from_slice(&vv, &[n as i32, k as i32]),
    )
}

// --- per-primitive f64 row solvers (camera-space rays, certified) ---

fn sphere_rows(p: &SphereParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let (a, b, c) = cga_core::sphere_quad(o, d, p.c, p.r);
    match QuadRoots::solve(a, b, c) {
        QuadRoots::Two(t1, t2) => [t1, t2]
            .into_iter()
            .map(|t| {
                let hp = vec3_add(o, vec3_scale(d, t));
                RowHit {
                    t,
                    n: cga_core::sphere_normal(hp, p.c, p.r),
                }
            })
            .collect(),
        _ => Vec::new(),
    }
}

fn plane_rows(p: &PlaneParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    match cga_core::plane_hit(o, d, p.n, p.d) {
        Some(t) => vec![RowHit { t, n: p.n }],
        None => Vec::new(),
    }
}

fn cylinder_rows(p: &CylinderParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let u = p.u;
    let (a, b, c) = cga_core::cylinder_quad(o, d, p.q, u, p.r);
    // Side interval: None = no side section; (lo, hi) may be unbounded when
    // the ray is parallel to the axis and inside the tube.
    let side: Option<(f64, f64)> = if a == 0.0 && b == 0.0 {
        if c < 0.0 {
            Some((f64::NEG_INFINITY, f64::INFINITY))
        } else {
            None // c > 0: outside; c == 0: on the surface, abstain
        }
    } else {
        match QuadRoots::solve(a, b, c) {
            QuadRoots::Two(t0, t1) => Some((t0, t1)),
            _ => None, // Double = tangent = no crossing; Indeterminate = abstain
        }
    };
    let side_n =
        |t: f64| cga_core::cylinder_side_normal(vec3_add(o, vec3_scale(d, t)), p.q, u, p.r);
    let Some((slo, shi)) = side else {
        return Vec::new();
    };
    if p.h < 0.0 {
        if slo.is_finite() && shi.is_finite() {
            return vec![
                RowHit {
                    t: slo,
                    n: side_n(slo),
                },
                RowHit {
                    t: shi,
                    n: side_n(shi),
                },
            ];
        }
        return Vec::new();
    }
    let (at0, at1) = cga_core::cylinder_cap_ts(o, d, p.q, u, p.h);
    let enter = slo.max(at0);
    let exit_ = shi.min(at1);
    if !enter.is_finite() || !exit_.is_finite() || enter >= exit_ {
        return Vec::new();
    }
    let o_par = vec3_dot(vec3_sub(o, p.q), u);
    let d_par = vec3_dot(d, u);
    let t_plus = (p.h - o_par) / d_par;
    let t_minus = (-p.h - o_par) / d_par;
    let n_cap_enter = if t_plus < t_minus {
        u
    } else {
        vec3_scale(u, -1.0)
    };
    let n_cap_exit = if t_plus > t_minus {
        u
    } else {
        vec3_scale(u, -1.0)
    };
    let n_enter = if enter == at0 {
        n_cap_enter
    } else {
        side_n(enter)
    };
    let n_exit = if exit_ == at1 {
        n_cap_exit
    } else {
        side_n(exit_)
    };
    vec![
        RowHit {
            t: enter,
            n: n_enter,
        },
        RowHit {
            t: exit_,
            n: n_exit,
        },
    ]
}

fn cone_local_rows(r: f64, h: f64, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    // 与数组内核 `cone_local_interval` 同一区间构造（`a > 0` 取 [st0, st1]，
    // `a < 0` 取 [at0, st0] ∪ [st1, at1] 两支），法线规则也一致：端点落在端面上就用
    // 端面法线（+z 在顶点面），否则用侧面法线。
    let (a, b, c) = cga_core::cone_quad(r, h, o, d);
    let disc = b * b - 4.0 * a * c;
    let side_ok = a.abs() > 1e-12 && disc > 1e-12;
    let (t_top, t_bot) = cga_core::cone_cap_ts(h, o, d);
    let (at0, at1) = (t_top.min(t_bot), t_top.max(t_bot));
    let pt = |t: f64| vec3_add(o, vec3_scale(d, t));
    let n_enter_cap = if t_top < t_bot {
        [0.0, 0.0, 1.0]
    } else {
        [0.0, 0.0, -1.0]
    };
    let n_exit_cap = vec3_scale(n_enter_cap, -1.0);
    let mut rows: Vec<RowHit> = Vec::new();
    let push = |enter: f64, exit_: f64, rows: &mut Vec<RowHit>| {
        if enter.is_nan() || exit_.is_nan() || enter >= exit_ {
            return;
        }
        let n0 = if enter == at0 {
            n_enter_cap
        } else {
            cga_core::cone_side_normal(r, h, pt(enter))
        };
        let n1 = if exit_ == at1 {
            n_exit_cap
        } else {
            cga_core::cone_side_normal(r, h, pt(exit_))
        };
        rows.push(RowHit { t: enter, n: n0 });
        rows.push(RowHit { t: exit_, n: n1 });
    };
    if side_ok {
        let sq = disc.max(0.0).sqrt();
        let r_lo = (-b - sq) / (2.0 * a);
        let r_hi = (-b + sq) / (2.0 * a);
        let (st0, st1) = (r_lo.min(r_hi), r_lo.max(r_hi));
        let pos_a = a > 1e-12;
        let (c1e, c1x) = if pos_a {
            (st0.max(at0), st1.min(at1))
        } else {
            (at0, st0.min(at1))
        };
        push(c1e, c1x, &mut rows);
        if !pos_a {
            push(st1.max(at0), at1, &mut rows);
        }
    }
    rows
}

fn ellipsoid_local_rows(o_l: [f64; 3], d_u: [f64; 3]) -> Vec<RowHit> {
    let (a, b, c) = cga_core::ellipsoid_quad(o_l, d_u);
    match QuadRoots::solve(a, b, c) {
        QuadRoots::Two(t1, t2) => [t1, t2]
            .into_iter()
            .map(|t| RowHit {
                t,
                n: cga_core::ellipsoid_normal(o_l, d_u, t), // unit sphere: gradient = point
            })
            .collect(),
        _ => Vec::new(),
    }
}

fn torus_local_rows(major: f64, minor: f64, arc: f64, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let q = Quartic::from_coeffs(cga_core::torus_quartic(major, minor, o, d));
    let mut roots = q.roots().roots().to_vec();
    roots.sort_by(f64::total_cmp);
    roots
        .into_iter()
        .filter_map(|t| {
            let p = vec3_add(o, vec3_scale(d, t));
            let theta = p[1].atan2(p[0]);
            let turned = if theta < 0.0 {
                theta + std::f64::consts::TAU
            } else {
                theta
            };
            if turned > arc {
                return None;
            }
            Some(RowHit {
                t,
                n: cga_core::torus_normal(major, minor, p),
            })
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn cyclide_local_rows(
    a: f64,
    b: f64,
    dd: f64,
    c: f64,
    shift: [f64; 3],
    o: [f64; 3],
    d: [f64; 3],
) -> Vec<RowHit> {
    let q = Quartic::from_coeffs(cga_core::cyclide_quartic(a, b, dd, c, shift, o, d));
    let mut roots = q.roots().roots().to_vec();
    roots.sort_by(f64::total_cmp);
    roots
        .into_iter()
        .map(|t| {
            let p = vec3_add(o, vec3_scale(d, t));
            RowHit {
                t,
                n: cga_core::cyclide_normal(a, b, dd, c, shift, p),
            }
        })
        .collect()
}

// --- 阶段 3 漂移测试入口（只给 cga-gpu 的测试用：f64 行解算器的逐光线入口）---

#[cfg(test)]
pub(crate) mod test_rows {
    use super::*;

    pub(crate) fn sphere(p: &SphereParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
        sphere_rows(p, o, d)
    }
    pub(crate) fn plane(p: &PlaneParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
        plane_rows(p, o, d)
    }
    pub(crate) fn cylinder(p: &CylinderParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
        cylinder_rows(p, o, d)
    }
    pub(crate) fn cone(p: &ConeParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
        local_rows(p.a_inv3, p.t_inv, |o, d| cone_local_rows(p.r, p.h, o, d))(o, d)
    }
    pub(crate) fn ellipsoid(p: &EllipsoidParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
        local_rows(p.a_inv3, p.t_inv, ellipsoid_local_rows)(o, d)
    }
    pub(crate) fn torus(p: &TorusParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
        local_rows(p.a_inv3, p.t_inv, |o, d| {
            torus_local_rows(p.major, p.minor, p.arc, o, d)
        })(o, d)
    }
    pub(crate) fn cyclide(p: &CyclideParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
        local_rows(p.a_inv3, p.t_inv, |o, d| {
            cyclide_local_rows(p.a, p.b, p.d, p.c, p.shift, o, d)
        })(o, d)
    }
}

// --- wiring entry points (take the f32 results, return them patched) ---

/// 用 `row` 在 CPU f64 上重算 `amb` 标记的行，其余行原样保留。
fn certify_rows(
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
    row: impl Fn([f64; 3], [f64; 3]) -> Vec<RowHit>,
) -> (Array, Array, Array) {
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> =
        idx.into_iter().map(|i| (i, row(o64[i], d64[i]))).collect();
    splice(&ts, &ns, &vs, &rows)
}

/// 把局部空间的行解算器 `row` 提升到仿射包装：光线先归一到局部，解出的
/// `t` 除以 `|A⁻¹d|`、法线经 `A⁻ᵀ` 回到世界空间（退化光线无命中）。
fn local_rows(
    a_inv3: Mat3,
    t_inv: [f64; 3],
    row: impl Fn([f64; 3], [f64; 3]) -> Vec<RowHit>,
) -> impl Fn([f64; 3], [f64; 3]) -> Vec<RowHit> {
    move |o, d| {
        let Some((o_l, d_u, lam)) = cga_core::local_ray(a_inv3, t_inv, o, d) else {
            return Vec::new();
        };
        row(o_l, d_u)
            .into_iter()
            .map(|h| RowHit {
                t: h.t / lam,
                n: cga_core::world_normal(a_inv3, h.n),
            })
            .collect()
    }
}

pub(crate) fn sphere_fallback(
    p: &SphereParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    certify_rows(o, d, amb, ts, ns, vs, |o, d| sphere_rows(p, o, d))
}

pub(crate) fn plane_fallback(
    p: &PlaneParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    certify_rows(o, d, amb, ts, ns, vs, |o, d| plane_rows(p, o, d))
}

pub(crate) fn cylinder_fallback(
    p: &CylinderParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    certify_rows(o, d, amb, ts, ns, vs, |o, d| cylinder_rows(p, o, d))
}

pub(crate) fn cone_fallback(
    p: &ConeParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    let row = local_rows(p.a_inv3, p.t_inv, |o, d| cone_local_rows(p.r, p.h, o, d));
    certify_rows(o, d, amb, ts, ns, vs, row)
}

pub(crate) fn ellipsoid_fallback(
    p: &EllipsoidParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    let row = local_rows(p.a_inv3, p.t_inv, ellipsoid_local_rows);
    certify_rows(o, d, amb, ts, ns, vs, row)
}

pub(crate) fn torus_fallback(
    p: &TorusParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    let row = local_rows(p.a_inv3, p.t_inv, |o, d| {
        torus_local_rows(p.major, p.minor, p.arc, o, d)
    });
    certify_rows(o, d, amb, ts, ns, vs, row)
}

pub(crate) fn cyclide_fallback(
    p: &CyclideParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    let row = local_rows(p.a_inv3, p.t_inv, |o, d| {
        cyclide_local_rows(p.a, p.b, p.d, p.c, p.shift, o, d)
    });
    certify_rows(o, d, amb, ts, ns, vs, row)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_ray;
    use cga_core::{CylinderParams, PlaneParams, SphereParams, TorusParams};

    const IDENT: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    const IDENT16: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];

    fn crossings(p: &cga_core::GeometryParams, o: [f64; 3], d: [f64; 3]) -> (Vec<f32>, Vec<bool>) {
        let (ts, _ns, vs) =
            crate::geom_crossings(p, &test_ray(o[0], o[1], o[2]), &test_ray(d[0], d[1], d[2]));
        ts.eval().unwrap();
        vs.eval().unwrap();
        (
            ts.as_slice::<f32>().to_vec(),
            vs.as_slice::<bool>().to_vec(),
        )
    }

    #[test]
    fn sphere_grazing_ray_certified_two_crossings() {
        // o.y one f32 ulp below the tangent: the f32 discriminant (≈1.9e-6 at
        // terms of magnitude 16) sits inside the cancellation band, so the
        // f32 sign is undecidable and the certified path must decide it.
        let y0 = 1.0f32.next_down();
        let p = cga_core::GeometryParams::SphereParams(SphereParams {
            c: [0.0, 0.0, 0.0],
            r: 1.0,
            axes: IDENT,
        });
        let (ts, vs) = crossings(&p, [0.0, f64::from(y0), -2.0], [0.0, 0.0, 1.0]);
        let half = (1.0 - f64::from(y0) * f64::from(y0)).sqrt();
        assert!(vs.iter().all(|&v| v), "两根都应有效: {vs:?}");
        assert!(
            (f64::from(ts[0]) - (2.0 - half)).abs() < 1e-5,
            "t0 应 ≈ {}，实际 {}",
            2.0 - half,
            ts[0]
        );
        assert!(
            (f64::from(ts[1]) - (2.0 + half)).abs() < 1e-5,
            "t1 应 ≈ {}，实际 {}",
            2.0 + half,
            ts[1]
        );
    }

    #[test]
    fn sphere_tangent_ray_certified_no_crossing() {
        let p = cga_core::GeometryParams::SphereParams(SphereParams {
            c: [0.0, 0.0, 0.0],
            r: 1.0,
            axes: IDENT,
        });
        let (_ts, vs) = crossings(&p, [0.0, 1.0, -2.0], [0.0, 0.0, 1.0]);
        assert!(vs.iter().all(|&v| !v), "相切射线应无穿越: {vs:?}");
    }

    #[test]
    fn cylinder_axial_ray_certified_cap_crossings() {
        // Ray parallel to the axis: the f32 guard a > 1e-12 kills the side
        // quadratic and the capped interval collapses to "no crossings".
        // The certified fallback must emit the two cap crossings.
        let p = cga_core::GeometryParams::CylinderParams(CylinderParams {
            q: [0.0, 0.0, 0.0],
            u: [0.0, 0.0, 1.0],
            r: 0.5,
            h: 1.0,
        });
        let (ts, vs) = crossings(&p, [0.1, 0.0, -5.0], [0.0, 0.0, 1.0]);
        assert!(vs.iter().all(|&v| v), "轴向射线应穿两盖: {vs:?}");
        assert!((f64::from(ts[0]) - 4.0).abs() < 1e-4, "进盖 t=4: {}", ts[0]);
        assert!((f64::from(ts[1]) - 6.0).abs() < 1e-4, "出盖 t=6: {}", ts[1]);
    }

    #[test]
    fn plane_parallel_ray_certified_still_invalid() {
        let p = cga_core::GeometryParams::PlaneParams(PlaneParams {
            n: [0.0, 0.0, 1.0],
            d: 1.0,
        });
        let (_ts, vs) = crossings(&p, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        assert!(vs.iter().all(|&v| !v), "平行射线应无穿越: {vs:?}");
    }

    #[test]
    fn plane_near_parallel_ray_certified_crossing() {
        // |n·d| ≈ 5e-10 < 1e-9 guard: f32 path drops the crossing; the f64
        // fallback must recover it at t = 2e9.
        let p = cga_core::GeometryParams::PlaneParams(PlaneParams {
            n: [0.0, 0.0, 1.0],
            d: 1.0,
        });
        let d = [1.0f64, 0.0, 5e-10];
        let nrm = (1.0 + 25e-20f64).sqrt();
        let (_ts, vs) = crossings(&p, [0.0, 0.0, 0.0], [d[0] / nrm, d[1] / nrm, d[2] / nrm]);
        assert!(vs.iter().all(|&v| v), "近平行射线应判定穿越: {vs:?}");
    }

    #[test]
    fn torus_tangent_ray_certified_no_crossing() {
        // Same ray as certify::tests::quartic_tangent_torus_ray_reports_unknown.
        let p = cga_core::GeometryParams::TorusParams(TorusParams {
            a_inv3: IDENT,
            t_inv: [0.0, 0.0, 0.0],
            a_fwd: IDENT16,
            major: 2.0,
            minor: 1.0,
            arc: std::f64::consts::TAU,
        });
        let (_ts, vs) = crossings(&p, [3.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(vs.iter().all(|&v| !v), "相切环面射线应无穿越: {vs:?}");
    }
}
