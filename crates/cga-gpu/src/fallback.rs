//! CPU f64 certified fallback for the f32 crossing guards.
//!
//! GPU crossings keep the f32 fast path; rays whose discriminant lies inside
//! the cancellation band (or whose leading coefficient degenerates) are
//! re-decided on CPU f64 via `certify::QuadRoots` / `certify::Quartic`.
//! `Double` roots are tangencies and classify as no crossing;
//! `Unknown`/`Indeterminate` abstain (no crossing — the render path leaves
//! the pixel to anti-aliasing).

use cga_core::{
    vec3_add, vec3_dot, vec3_len, vec3_scale, vec3_sub, ConeParams, CyclideParams, CylinderParams,
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

/// 单位化：近零退化时返回零向量（与 `cga_scene::vec3_unit` 的 `[0,0,1]` 兜底
/// 方向语义不同，故保留为本模块自有助手）。
#[inline]
fn vunit(a: [f64; 3]) -> [f64; 3] {
    let n = vec3_len(a);
    if n > 1e-300 {
        vec3_scale(a, 1.0 / n)
    } else {
        [0.0, 0.0, 0.0]
    }
}

#[inline]
fn mat3_mul(m: Mat3, v: [f64; 3]) -> [f64; 3] {
    [vec3_dot(m[0], v), vec3_dot(m[1], v), vec3_dot(m[2], v)]
}

#[inline]
fn mat3_mul_transpose(m: Mat3, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

/// o_l = A⁻¹·o + t_inv; d_u = A⁻¹·d / |A⁻¹·d|; lam = |A⁻¹·d|.
fn local_ray(
    a_inv3: Mat3,
    t_inv: [f64; 3],
    o: [f64; 3],
    d: [f64; 3],
) -> Option<([f64; 3], [f64; 3], f64)> {
    let ol = vec3_add(mat3_mul(a_inv3, o), t_inv);
    let dl = mat3_mul(a_inv3, d);
    let lam = vec3_len(dl);
    if !lam.is_finite() || lam <= 1e-300 {
        return None;
    }
    Some((ol, vec3_scale(dl, 1.0 / lam), lam))
}

/// n_world = normalize(A⁻ᵀ · n_l) — the inverse-transpose normal transform,
/// matching `affine_normal` in geom_kernels.rs.
fn world_normal(a_inv3: Mat3, n_l: [f64; 3]) -> [f64; 3] {
    vunit(mat3_mul_transpose(a_inv3, n_l))
}

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
    let oc = vec3_sub(o, p.c);
    let b = 2.0 * vec3_dot(oc, d);
    let c = vec3_dot(oc, oc) - p.r * p.r;
    match QuadRoots::solve(1.0, b, c) {
        QuadRoots::Two(t1, t2) => [t1, t2]
            .into_iter()
            .map(|t| {
                let hp = vec3_add(o, vec3_scale(d, t));
                RowHit {
                    t,
                    n: vec3_scale(vec3_sub(hp, p.c), 1.0 / p.r),
                }
            })
            .collect(),
        _ => Vec::new(),
    }
}

fn plane_rows(p: &PlaneParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let denom = vec3_dot(p.n, d);
    if !denom.is_finite() || denom.abs() <= 1e-30 {
        return Vec::new();
    }
    let t = (p.d - vec3_dot(p.n, o)) / denom;
    if !t.is_finite() {
        return Vec::new();
    }
    vec![RowHit { t, n: p.n }]
}

fn cylinder_rows(p: &CylinderParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let u = p.u;
    let oc = vec3_sub(o, p.q);
    let d_par = vec3_dot(d, u);
    let o_par = vec3_dot(oc, u);
    let d_p = vec3_sub(d, vec3_scale(u, d_par));
    let o_p = vec3_sub(oc, vec3_scale(u, o_par));
    let a = vec3_dot(d_p, d_p);
    let b = 2.0 * vec3_dot(o_p, d_p);
    let c = vec3_dot(o_p, o_p) - p.r * p.r;
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
    let side_n = |t: f64| vec3_scale(vec3_add(o_p, vec3_scale(d_p, t)), 1.0 / p.r);
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
    let (at0, at1) = if d_par.abs() > 1e-30 {
        let t_plus = (p.h - o_par) / d_par;
        let t_minus = (-p.h - o_par) / d_par;
        (t_plus.min(t_minus), t_plus.max(t_minus))
    } else {
        (f64::NEG_INFINITY, f64::INFINITY)
    };
    let enter = slo.max(at0);
    let exit_ = shi.min(at1);
    if !enter.is_finite() || !exit_.is_finite() || enter >= exit_ {
        return Vec::new();
    }
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
    let k = r / h;
    let k2 = 1.0 + k * k;
    let wz = o[2] - h / 2.0;
    let dz = d[2];
    let wd = vec3_dot(o, d) - dz * h / 2.0;
    let ww = vec3_dot(o, o) - o[2] * h + h * h / 4.0;
    let a = 1.0 - k2 * dz * dz;
    let b = 2.0 * (wd - k2 * wz * dz);
    let c = ww - k2 * wz * wz;
    let side_n = |t: f64| {
        let p = vec3_add(o, vec3_scale(d, t));
        let s = p[2] - h / 2.0;
        vunit([p[0], p[1], s * (1.0 - k2)])
    };
    let (at0, at1, t_top, t_bot) = if dz.abs() > 1e-30 {
        let tt = -wz / dz;
        let tb = -(wz + h) / dz;
        (tt.min(tb), tt.max(tb), tt, tb)
    } else {
        (f64::NEG_INFINITY, f64::INFINITY, f64::NAN, f64::NAN)
    };
    // Side interval, mirroring cone_local_interval: a > 0 keeps [st0, st1],
    // a < 0 keeps [st1, ∞); a == 0 (ray parallel to the slope) is linear.
    enum Iv {
        Between(f64, f64),
        Above(f64),
        Below(f64),
        Empty,
    }
    let iv = match QuadRoots::solve(a, b, c) {
        QuadRoots::Two(st0, st1) => {
            if a > 0.0 {
                Iv::Between(st0, st1)
            } else {
                Iv::Above(st1)
            }
        }
        QuadRoots::One(tl) => {
            if b > 0.0 {
                Iv::Below(tl)
            } else {
                Iv::Above(tl)
            }
        }
        _ => Iv::Empty, // Double = tangent; None; Indeterminate = abstain
    };
    let (enter, exit_) = match iv {
        Iv::Between(s0, s1) => (s0.max(at0), s1.min(at1)),
        Iv::Above(s1) => (s1.max(at0), at1),
        Iv::Below(s1) => (at0, s1.min(at1)),
        Iv::Empty => return Vec::new(),
    };
    if !enter.is_finite() || !exit_.is_finite() || enter >= exit_ {
        return Vec::new();
    }
    let n_cap_enter = if t_top < t_bot {
        [0.0, 0.0, 1.0]
    } else {
        [0.0, 0.0, -1.0]
    };
    let n_cap_exit = if t_top > t_bot {
        [0.0, 0.0, 1.0]
    } else {
        [0.0, 0.0, -1.0]
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

fn ellipsoid_local_rows(o_l: [f64; 3], d_u: [f64; 3]) -> Vec<RowHit> {
    let b = 2.0 * vec3_dot(o_l, d_u);
    let c = vec3_dot(o_l, o_l) - 1.0;
    match QuadRoots::solve(1.0, b, c) {
        QuadRoots::Two(t1, t2) => [t1, t2]
            .into_iter()
            .map(|t| RowHit {
                t,
                n: vec3_add(o_l, vec3_scale(d_u, t)), // unit sphere: gradient = point
            })
            .collect(),
        _ => Vec::new(),
    }
}

fn torus_local_rows(major: f64, minor: f64, arc: f64, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let r2 = major * major;
    let q = Quartic::torus(major, minor, o, d);
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
            let s1 = vec3_dot(p, p) + r2 - minor * minor;
            let fac = s1 - 2.0 * r2;
            Some(RowHit {
                t,
                n: vunit([fac * p[0], fac * p[1], s1 * p[2]]),
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
    let bb = b * b - dd * dd;
    let q = Quartic::cyclide(a, b, dd, c, shift, o, d);
    let mut roots = q.roots().roots().to_vec();
    roots.sort_by(f64::total_cmp);
    roots
        .into_iter()
        .map(|t| {
            let p = vec3_sub(vec3_add(o, vec3_scale(d, t)), shift);
            let g = vec3_dot(p, p) + bb;
            let n = [
                4.0 * p[0] * g - 8.0 * a * (a * p[0] - c * dd),
                4.0 * p[1] * g - 8.0 * b * b * p[1],
                4.0 * p[2] * g,
            ];
            RowHit { t, n: vunit(n) }
        })
        .collect()
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
        let Some((o_l, d_u, lam)) = local_ray(a_inv3, t_inv, o, d) else {
            return Vec::new();
        };
        row(o_l, d_u)
            .into_iter()
            .map(|h| RowHit {
                t: h.t / lam,
                n: world_normal(a_inv3, h.n),
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
