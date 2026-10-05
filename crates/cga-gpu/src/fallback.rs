//! CPU f64 certified fallback for the f32 crossing guards.
//!
//! GPU crossings keep the f32 fast path; rays whose discriminant lies inside
//! the cancellation band (or whose leading coefficient degenerates) are
//! re-decided on CPU f64 via `certify::QuadRoots` / `certify::Quartic`.
//! `Double` roots are tangencies and classify as no crossing;
//! `Unknown`/`Indeterminate` abstain (no crossing — the render path leaves
//! the pixel to anti-aliasing).

use cga_core::{
    ConeParams, CyclideParams, CylinderParams, EllipsoidParams, Mat3, PlaneParams, SphereParams,
    TorusParams,
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

#[inline]
fn vsub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn vadd(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
fn vscale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[inline]
fn vdot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn vnorm(a: [f64; 3]) -> f64 {
    vdot(a, a).sqrt()
}

#[inline]
fn vunit(a: [f64; 3]) -> [f64; 3] {
    let n = vnorm(a);
    if n > 1e-300 {
        vscale(a, 1.0 / n)
    } else {
        [0.0, 0.0, 0.0]
    }
}

#[inline]
fn mat3_mul(m: Mat3, v: [f64; 3]) -> [f64; 3] {
    [vdot(m[0], v), vdot(m[1], v), vdot(m[2], v)]
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
    let ol = vadd(mat3_mul(a_inv3, o), t_inv);
    let dl = mat3_mul(a_inv3, d);
    let lam = vnorm(dl);
    if !lam.is_finite() || lam <= 1e-300 {
        return None;
    }
    Some((ol, vscale(dl, 1.0 / lam), lam))
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
    let oc = vsub(o, p.c);
    let b = 2.0 * vdot(oc, d);
    let c = vdot(oc, oc) - p.r * p.r;
    match QuadRoots::solve(1.0, b, c) {
        QuadRoots::Two(t1, t2) => [t1, t2]
            .into_iter()
            .map(|t| {
                let hp = vadd(o, vscale(d, t));
                RowHit {
                    t,
                    n: vscale(vsub(hp, p.c), 1.0 / p.r),
                }
            })
            .collect(),
        _ => Vec::new(),
    }
}

fn plane_rows(p: &PlaneParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let denom = vdot(p.n, d);
    if !denom.is_finite() || denom.abs() <= 1e-30 {
        return Vec::new();
    }
    let t = (p.d - vdot(p.n, o)) / denom;
    if !t.is_finite() {
        return Vec::new();
    }
    vec![RowHit { t, n: p.n }]
}

fn cylinder_rows(p: &CylinderParams, o: [f64; 3], d: [f64; 3]) -> Vec<RowHit> {
    let u = p.u;
    let oc = vsub(o, p.q);
    let d_par = vdot(d, u);
    let o_par = vdot(oc, u);
    let d_p = vsub(d, vscale(u, d_par));
    let o_p = vsub(oc, vscale(u, o_par));
    let a = vdot(d_p, d_p);
    let b = 2.0 * vdot(o_p, d_p);
    let c = vdot(o_p, o_p) - p.r * p.r;
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
    let side_n = |t: f64| vscale(vadd(o_p, vscale(d_p, t)), 1.0 / p.r);
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
    if !(enter < exit_) || !enter.is_finite() || !exit_.is_finite() {
        return Vec::new();
    }
    let t_plus = (p.h - o_par) / d_par;
    let t_minus = (-p.h - o_par) / d_par;
    let n_cap_enter = if t_plus < t_minus { u } else { vscale(u, -1.0) };
    let n_cap_exit = if t_plus > t_minus { u } else { vscale(u, -1.0) };
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
    let wd = vdot(o, d) - dz * h / 2.0;
    let ww = vdot(o, o) - o[2] * h + h * h / 4.0;
    let a = 1.0 - k2 * dz * dz;
    let b = 2.0 * (wd - k2 * wz * dz);
    let c = ww - k2 * wz * wz;
    let side_n = |t: f64| {
        let p = vadd(o, vscale(d, t));
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
    if !(enter < exit_) || !enter.is_finite() || !exit_.is_finite() {
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
    let b = 2.0 * vdot(o_l, d_u);
    let c = vdot(o_l, o_l) - 1.0;
    match QuadRoots::solve(1.0, b, c) {
        QuadRoots::Two(t1, t2) => [t1, t2]
            .into_iter()
            .map(|t| RowHit {
                t,
                n: vadd(o_l, vscale(d_u, t)), // unit sphere: gradient = point
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
            let p = vadd(o, vscale(d, t));
            let theta = p[1].atan2(p[0]);
            let turned = if theta < 0.0 {
                theta + std::f64::consts::TAU
            } else {
                theta
            };
            if turned > arc {
                return None;
            }
            let s1 = vdot(p, p) + r2 - minor * minor;
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
            let p = vsub(vadd(o, vscale(d, t)), shift);
            let g = vdot(p, p) + bb;
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

pub(crate) fn sphere_fallback(
    p: &SphereParams,
    o: &Array,
    d: &Array,
    amb: &Array,
    ts: Array,
    ns: Array,
    vs: Array,
) -> (Array, Array, Array) {
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> = idx
        .into_iter()
        .map(|i| (i, sphere_rows(p, o64[i], d64[i])))
        .collect();
    splice(&ts, &ns, &vs, &rows)
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
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> = idx
        .into_iter()
        .map(|i| (i, plane_rows(p, o64[i], d64[i])))
        .collect();
    splice(&ts, &ns, &vs, &rows)
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
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> = idx
        .into_iter()
        .map(|i| (i, cylinder_rows(p, o64[i], d64[i])))
        .collect();
    splice(&ts, &ns, &vs, &rows)
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
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> = idx
        .into_iter()
        .map(|i| {
            let Some((o_l, d_u, lam)) = local_ray(p.a_inv3, p.t_inv, o64[i], d64[i]) else {
                return (i, Vec::new());
            };
            let hits = cone_local_rows(p.r, p.h, o_l, d_u)
                .into_iter()
                .map(|h| RowHit {
                    t: h.t / lam,
                    n: world_normal(p.a_inv3, h.n),
                })
                .collect();
            (i, hits)
        })
        .collect();
    splice(&ts, &ns, &vs, &rows)
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
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> = idx
        .into_iter()
        .map(|i| {
            let Some((o_l, d_u, lam)) = local_ray(p.a_inv3, p.t_inv, o64[i], d64[i]) else {
                return (i, Vec::new());
            };
            let hits = ellipsoid_local_rows(o_l, d_u)
                .into_iter()
                .map(|h| RowHit {
                    t: h.t / lam,
                    n: world_normal(p.a_inv3, h.n),
                })
                .collect();
            (i, hits)
        })
        .collect();
    splice(&ts, &ns, &vs, &rows)
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
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> = idx
        .into_iter()
        .map(|i| {
            let Some((o_l, d_u, lam)) = local_ray(p.a_inv3, p.t_inv, o64[i], d64[i]) else {
                return (i, Vec::new());
            };
            let hits = torus_local_rows(p.major, p.minor, p.arc, o_l, d_u)
                .into_iter()
                .map(|h| RowHit {
                    t: h.t / lam,
                    n: world_normal(p.a_inv3, h.n),
                })
                .collect();
            (i, hits)
        })
        .collect();
    splice(&ts, &ns, &vs, &rows)
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
    let idx = ambiguous_indices(amb);
    if idx.is_empty() {
        return (ts, ns, vs);
    }
    let (o64, d64) = read_rays(o, d);
    let rows: Vec<(usize, Vec<RowHit>)> = idx
        .into_iter()
        .map(|i| {
            let Some((o_l, d_u, lam)) = local_ray(p.a_inv3, p.t_inv, o64[i], d64[i]) else {
                return (i, Vec::new());
            };
            let hits = cyclide_local_rows(p.a, p.b, p.d, p.c, p.shift, o_l, d_u)
                .into_iter()
                .map(|h| RowHit {
                    t: h.t / lam,
                    n: world_normal(p.a_inv3, h.n),
                })
                .collect();
            (i, hits)
        })
        .collect();
    splice(&ts, &ns, &vs, &rows)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{CylinderParams, PlaneParams, SphereParams, TorusParams};

    const IDENT: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    const IDENT16: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];

    fn ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn crossings(p: &cga_core::GeometryParams, o: [f64; 3], d: [f64; 3]) -> (Vec<f32>, Vec<bool>) {
        let (ts, _ns, vs) =
            crate::geom_crossings(p, &ray(o[0], o[1], o[2]), &ray(d[0], d[1], d[2]));
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
