//! Mesh bake: `GeometryParams` implicit field -> triangle soup.
//!
//! Pure CPU `f64`, no GPU/MLX: portable to Linux/CI and to ForgeCAD's
//! `cad-kernel`. Field sign convention mirrors the GPU `*_contains`
//! kernels (`field < 0` == inside). Polygonization is marching tetrahedra
//! (Kuhn-Freudenthal split: tiny fixed tables, no crack-prone
//! disambiguation); per-triangle winding is fixed by numerical gradient so
//! normals always point outward.
//!
//! Limits (same as the renderer, stated honestly): tangent/degenerate CSG
//! configurations inherit the crossings/contains sampling semantics;
//! thin features below `step` are missed; trimesh fields are O(F) per eval.

use crate::{
    affine_from_motor, mat3_new, motor_identity, sphere_from_dual, AffineParams, BoxParams,
    ConeParams, CsgOp, CyclideParams, CylinderParams, EllipsoidParams, Geometry, GeometryParams,
    PlaneParams, SphereParams, TorusParams, TrimeshParams,
};

/// Resolve `Geometry` to world-frame params under the identity motor.
/// Test/CLI helper; the renderer path uses `geom_to_camera` with the real motor.
pub fn identity_params(g: &Geometry) -> GeometryParams {
    let id = motor_identity();
    let eye: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let eye3 = mat3_new(eye[0], eye[1], eye[2]);
    match g {
        Geometry::SphereGeometry(g) => {
            let (c, r) = sphere_from_dual(&g.blade);
            GeometryParams::SphereParams(SphereParams { c, r, axes: eye })
        }
        Geometry::PlaneGeometry(g) => GeometryParams::PlaneParams(PlaneParams {
            n: g.blade.euclidean_vector(),
            d: g.blade.einf_coeff(),
        }),
        Geometry::CylinderGeometry(g) => GeometryParams::CylinderParams(CylinderParams {
            q: [0.0; 3],
            u: [0.0, 0.0, 1.0],
            r: g.radius,
            h: g.half,
        }),
        Geometry::BoxGeometry(g) => GeometryParams::BoxParams(BoxParams {
            c: [0.0; 3],
            axes: eye,
            half: g.half,
        }),
        Geometry::CircleGeometry(g) => GeometryParams::CircleParams(crate::CircleParams {
            c: [0.0; 3],
            n: [0.0, 0.0, 1.0],
            r: g.radius,
        }),
        Geometry::ConeGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(id, eye3);
            GeometryParams::ConeParams(ConeParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                r: g.radius,
                h: g.height,
            })
        }
        Geometry::TorusGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(id, eye3);
            GeometryParams::TorusParams(TorusParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                major: g.major,
                minor: g.minor,
                arc: g.arc,
            })
        }
        Geometry::EllipsoidGeometry(g) => {
            let rr = g.radii;
            let diag = mat3_new([rr[0], 0.0, 0.0], [0.0, rr[1], 0.0], [0.0, 0.0, rr[2]]);
            let (ai, ti, af) = affine_from_motor(id, diag);
            GeometryParams::EllipsoidParams(EllipsoidParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
            })
        }
        Geometry::CyclideGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(id, eye3);
            GeometryParams::CyclideParams(CyclideParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                a: g.a,
                b: g.b,
                d: g.d,
                c: (g.a * g.a - g.b * g.b).sqrt(),
                shift: g.shift,
            })
        }
        Geometry::TrimeshGeometry(g) => {
            let (ai, ti, af) = affine_from_motor(id, eye3);
            GeometryParams::TrimeshParams(TrimeshParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                v0: g.v0.clone(),
                e1: g.e1.clone(),
                e2: g.e2.clone(),
                nrm: g.nrm.clone(),
                lo: g.lo,
                hi: g.hi,
            })
        }
        Geometry::CsgGeometry(g) => GeometryParams::CsgParams(crate::CsgParams {
            op: g.op,
            children: g.children.iter().map(identity_params).collect(),
        }),
        Geometry::AffineGeometry(g) => {
            let inner = identity_params(&g.inner[0]);
            let (ai, ti, af) = affine_from_motor(g.motor, g.linear);
            GeometryParams::AffineParams(AffineParams {
                inner: Box::new(inner),
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
            })
        }
    }
}

// ---------------------------------------------------------------------------
// scalar field: negative inside
// ---------------------------------------------------------------------------

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// World -> local through `(a_inv3, t_inv)`, mirroring
/// `affine_point_to_local`: `loc[j] = a_inv3[j] . p + t_inv[j]`.
fn to_local(a_inv3: [[f64; 3]; 3], t_inv: [f64; 3], p: [f64; 3]) -> [f64; 3] {
    [
        a_inv3[0][0] * p[0] + a_inv3[0][1] * p[1] + a_inv3[0][2] * p[2] + t_inv[0],
        a_inv3[1][0] * p[0] + a_inv3[1][1] * p[1] + a_inv3[1][2] * p[2] + t_inv[1],
        a_inv3[2][0] * p[0] + a_inv3[2][1] * p[1] + a_inv3[2][2] * p[2] + t_inv[2],
    ]
}

fn sphere_field(p: SphereParams, x: [f64; 3]) -> f64 {
    dot(sub(x, p.c), sub(x, p.c)) - p.r * p.r
}

fn plane_field(p: PlaneParams, x: [f64; 3]) -> f64 {
    dot(p.n, x) - p.d
}

fn cylinder_field(p: CylinderParams, x: [f64; 3]) -> f64 {
    let rel = sub(x, p.q);
    let s = dot(rel, p.u);
    let rad = sub(rel, [p.u[0] * s, p.u[1] * s, p.u[2] * s]);
    let f = dot(rad, rad) - p.r * p.r;
    if p.h < 0.0 {
        return f; // infinite cylinder
    }
    f.max(s.abs() - p.h)
}

fn box_field(p: BoxParams, x: [f64; 3]) -> f64 {
    let q = sub(x, p.c);
    let mut f = (dot(q, p.axes[0]).abs() - p.half[0]).max(dot(q, p.axes[1]).abs() - p.half[1]);
    f = f.max(dot(q, p.axes[2]).abs() - p.half[2]);
    f
}

/// Canonical cone: apex at z=+h/2, base disc at z=-h/2 with radius r.
fn cone_field(p: ConeParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    let s = l[2] - p.h / 2.0;
    let k2 = 1.0 + (p.r / p.h) * (p.r / p.h);
    let f = l[0] * l[0] + l[1] * l[1] + s * s - k2 * s * s;
    f.max(s).max(-(s + p.h))
}

fn ellipsoid_field(p: EllipsoidParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    dot(l, l) - 1.0
}

fn torus_field(p: TorusParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    let r2 = p.major * p.major;
    let f = (dot(l, l) + r2 - p.minor * p.minor).powi(2) - 4.0 * r2 * (l[0] * l[0] + l[1] * l[1]);
    if p.arc >= std::f64::consts::TAU {
        return f;
    }
    let mut theta = l[1].atan2(l[0]);
    if theta < 0.0 {
        theta += std::f64::consts::TAU;
    }
    f.max(theta - p.arc)
}

fn cyclide_field(p: CyclideParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    let x0 = l[0] - p.shift[0];
    let y0 = l[1] - p.shift[1];
    let z0 = l[2] - p.shift[2];
    let bb = p.b * p.b - p.d * p.d;
    let rho = x0 * x0 + y0 * y0 + z0 * z0;
    (rho + bb).powi(2) - 4.0 * (x0 * p.a - p.c * p.d).powi(2) - 4.0 * p.b * p.b * y0 * y0
}

// --- scalar Moller-Trumbore + point-triangle distance (local frame) ---

fn mt_hit(a: [f64; 3], e1: [f64; 3], e2: [f64; 3], o: [f64; 3], d: [f64; 3]) -> Option<f64> {
    let pvec = [
        d[1] * e2[2] - d[2] * e2[1],
        d[2] * e2[0] - d[0] * e2[2],
        d[0] * e2[1] - d[1] * e2[0],
    ];
    let det = dot(e1, pvec);
    if det.abs() < 1e-15 {
        return None;
    }
    let inv = 1.0 / det;
    let tvec = sub(o, a);
    let u = dot(tvec, pvec) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qvec = [
        tvec[1] * e1[2] - tvec[2] * e1[1],
        tvec[2] * e1[0] - tvec[0] * e1[2],
        tvec[0] * e1[1] - tvec[1] * e1[0],
    ];
    let v = dot(d, qvec) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    Some(dot(e2, qvec) * inv)
}

fn pt_tri_dist2(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    // Ericson 5.1.5 closest point, squared distance.
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return dot(ap, ap);
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return dot(bp, bp);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let q = [a[0] + ab[0] * v, a[1] + ab[1] * v, a[2] + ab[2] * v];
        return dot(sub(p, q), sub(p, q));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return dot(cp, cp);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let q = [a[0] + ac[0] * w, a[1] + ac[1] * w, a[2] + ac[2] * w];
        return dot(sub(p, q), sub(p, q));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let q = [
            b[0] + (c[0] - b[0]) * w,
            b[1] + (c[1] - b[1]) * w,
            b[2] + (c[2] - b[2]) * w,
        ];
        return dot(sub(p, q), sub(p, q));
    }
    let n = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let n2 = dot(n, n);
    if n2 < 1e-30 {
        return dot(ap, ap);
    }
    let dist = dot(ap, n) / n2.sqrt();
    dist * dist
}

fn trimesh_field(p: &TrimeshParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    // Fixed near-irrational direction: parity rays along an axis can graze a
    // shared triangle edge (count twice) and misclassify; any generic
    // direction makes that measure-zero for axis-aligned test geometry.
    let d = [0.7241, 0.4413, 0.5306];
    let mut count = 0u32;
    let mut best = f64::INFINITY;
    for i in 0..p.v0.len() {
        let a = p.v0[i];
        let b = [a[0] + p.e1[i][0], a[1] + p.e1[i][1], a[2] + p.e1[i][2]];
        let c = [a[0] + p.e2[i][0], a[1] + p.e2[i][1], a[2] + p.e2[i][2]];
        if let Some(t) = mt_hit(a, p.e1[i], p.e2[i], l, d) {
            if t > 1e-9 {
                count += 1;
            }
        }
        let dd = pt_tri_dist2(l, a, b, c);
        if dd < best {
            best = dd;
        }
    }
    let dist = best.sqrt();
    if count % 2 == 1 {
        -dist
    } else {
        dist
    }
}

fn affine_field(p: &AffineParams, x: [f64; 3]) -> f64 {
    field(&p.inner, to_local(p.a_inv3, p.t_inv, x))
}

fn csg_field(op: CsgOp, children: &[GeometryParams], x: [f64; 3]) -> f64 {
    match op {
        CsgOp::Union => {
            let mut f = f64::INFINITY;
            for c in children {
                let v = field(c, x);
                if v < f {
                    f = v;
                }
            }
            f
        }
        CsgOp::Intersection => {
            let mut f = f64::NEG_INFINITY;
            for c in children {
                let v = field(c, x);
                if v > f {
                    f = v;
                }
            }
            f
        }
        CsgOp::Difference => {
            // first minus union-of-rest (mirrors GPU csg_contains)
            let mut f = field(&children[0], x);
            for c in &children[1..] {
                let v = -field(c, x);
                if v > f {
                    f = v;
                }
            }
            f
        }
    }
}

/// Signed implicit field: `< 0` inside. Panics on non-solids (circle),
/// mirroring the GPU kernels.
pub fn field(params: &GeometryParams, x: [f64; 3]) -> f64 {
    match params {
        GeometryParams::SphereParams(p) => sphere_field(*p, x),
        GeometryParams::PlaneParams(p) => plane_field(*p, x),
        GeometryParams::CylinderParams(p) => cylinder_field(*p, x),
        GeometryParams::BoxParams(p) => box_field(*p, x),
        GeometryParams::ConeParams(p) => cone_field(*p, x),
        GeometryParams::EllipsoidParams(p) => ellipsoid_field(*p, x),
        GeometryParams::TorusParams(p) => torus_field(*p, x),
        GeometryParams::CyclideParams(p) => cyclide_field(*p, x),
        GeometryParams::TrimeshParams(p) => trimesh_field(p, x),
        GeometryParams::AffineParams(p) => affine_field(p, x),
        GeometryParams::CsgParams(p) => csg_field(p.op, &p.children, x),
        GeometryParams::CircleParams(_) => panic!("circle is not a solid (no field)"),
    }
}

// ---------------------------------------------------------------------------
// bounds (world frame), mirroring GPU geom_bounds
// ---------------------------------------------------------------------------

fn apply_fwd(a_fwd: &[f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        a_fwd[0] * p[0] + a_fwd[1] * p[1] + a_fwd[2] * p[2] + a_fwd[3],
        a_fwd[4] * p[0] + a_fwd[5] * p[1] + a_fwd[6] * p[2] + a_fwd[7],
        a_fwd[8] * p[0] + a_fwd[9] * p[1] + a_fwd[10] * p[2] + a_fwd[11],
    ]
}

fn corners_bounds(lo: [f64; 3], hi: [f64; 3], a_fwd: &[f64; 16]) -> [[f64; 3]; 2] {
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for &i in &[lo[0], hi[0]] {
        for &j in &[lo[1], hi[1]] {
            for &k in &[lo[2], hi[2]] {
                let q = apply_fwd(a_fwd, [i, j, k]);
                for a in 0..3 {
                    if q[a] < mn[a] {
                        mn[a] = q[a];
                    }
                    if q[a] > mx[a] {
                        mx[a] = q[a];
                    }
                }
            }
        }
    }
    [mn, mx]
}

fn union_bounds(list: &[Option<[[f64; 3]; 2]>]) -> Option<[[f64; 3]; 2]> {
    let mut out: Option<[[f64; 3]; 2]> = None;
    for b in list.iter().flatten() {
        out = Some(match out {
            None => *b,
            Some([mn, mx]) => {
                let mut lo = mn;
                let mut hi = mx;
                for a in 0..3 {
                    if b[0][a] < lo[a] {
                        lo[a] = b[0][a];
                    }
                    if b[1][a] > hi[a] {
                        hi[a] = b[1][a];
                    }
                }
                [lo, hi]
            }
        });
    }
    out
}

fn intersect_bounds(list: &[Option<[[f64; 3]; 2]>]) -> Option<[[f64; 3]; 2]> {
    let mut out: Option<[[f64; 3]; 2]> = None;
    for b in list.iter().flatten() {
        out = Some(match out {
            None => *b,
            Some([mn, mx]) => {
                let mut lo = mn;
                let mut hi = mx;
                for a in 0..3 {
                    if b[0][a] > lo[a] {
                        lo[a] = b[0][a];
                    }
                    if b[1][a] < hi[a] {
                        hi[a] = b[1][a];
                    }
                }
                [lo, hi]
            }
        });
    }
    out
}

/// World-space AABB, or `None` when unbounded (plane / infinite cylinder).
pub fn bounds_of(params: &GeometryParams) -> Option<[[f64; 3]; 2]> {
    match params {
        GeometryParams::SphereParams(p) => Some([
            [p.c[0] - p.r, p.c[1] - p.r, p.c[2] - p.r],
            [p.c[0] + p.r, p.c[1] + p.r, p.c[2] + p.r],
        ]),
        GeometryParams::PlaneParams(_) => None,
        GeometryParams::CylinderParams(p) => {
            if p.h < 0.0 {
                return None;
            }
            let mut lo = [0.0; 3];
            let mut hi = [0.0; 3];
            for i in 0..3 {
                let e = p.u[i].abs() * p.h + p.r;
                lo[i] = p.q[i] - e;
                hi[i] = p.q[i] + e;
            }
            Some([lo, hi])
        }
        GeometryParams::BoxParams(p) => {
            let mut lo = [0.0; 3];
            let mut hi = [0.0; 3];
            for i in 0..3 {
                let mut e = 0.0;
                for j in 0..3 {
                    e += p.axes[j][i].abs() * p.half[j];
                }
                lo[i] = p.c[i] - e;
                hi[i] = p.c[i] + e;
            }
            Some([lo, hi])
        }
        GeometryParams::ConeParams(p) => Some(corners_bounds(
            [-p.r, -p.r, -p.h / 2.0],
            [p.r, p.r, p.h / 2.0],
            &p.a_fwd,
        )),
        GeometryParams::TorusParams(p) => {
            let e = p.major + p.minor;
            Some(corners_bounds(
                [-e, -e, -p.minor],
                [e, e, p.minor],
                &p.a_fwd,
            ))
        }
        GeometryParams::EllipsoidParams(p) => Some(corners_bounds([-1.0; 3], [1.0; 3], &p.a_fwd)),
        GeometryParams::CyclideParams(p) => {
            let r = p.d + p.c;
            Some(corners_bounds(
                [p.shift[0] - p.a - r, p.shift[1] - p.b - r, p.shift[2] - r],
                [p.shift[0] + p.a + r, p.shift[1] + p.b + r, p.shift[2] + r],
                &p.a_fwd,
            ))
        }
        GeometryParams::TrimeshParams(p) => Some(corners_bounds(p.lo, p.hi, &p.a_fwd)),
        GeometryParams::CsgParams(p) => {
            let bs: Vec<_> = p.children.iter().map(bounds_of).collect();
            if p.op == CsgOp::Difference {
                return bs.into_iter().next().unwrap_or(None);
            }
            if p.op == CsgOp::Union {
                union_bounds(&bs)
            } else {
                intersect_bounds(&bs)
            }
        }
        GeometryParams::AffineParams(p) => {
            let b = bounds_of(&p.inner)?;
            Some(corners_bounds(b[0], b[1], &p.a_fwd))
        }
        GeometryParams::CircleParams(_) => None,
    }
}

// ---------------------------------------------------------------------------
// marching tetrahedra
// ---------------------------------------------------------------------------

const CORN: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];

/// Kuhn-Freudenthal split along the 0-6 diagonal.
const TETS: [[usize; 4]; 6] = [
    [0, 1, 2, 6],
    [0, 1, 5, 6],
    [0, 3, 2, 6],
    [0, 3, 7, 6],
    [0, 4, 5, 6],
    [0, 4, 7, 6],
];

const TET_EDGES: [[usize; 2]; 6] = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];

fn lerp_pt(a: [f64; 3], va: f64, b: [f64; 3], vb: f64) -> [f64; 3] {
    let t = va / (va - vb);
    [
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
    ]
}

fn grad(params: &GeometryParams, x: [f64; 3]) -> [f64; 3] {
    let e = 1e-6;
    [
        (field(params, [x[0] + e, x[1], x[2]]) - field(params, [x[0] - e, x[1], x[2]])) / (2.0 * e),
        (field(params, [x[0], x[1] + e, x[2]]) - field(params, [x[0], x[1] - e, x[2]])) / (2.0 * e),
        (field(params, [x[0], x[1], x[2] + e]) - field(params, [x[0], x[1], x[2] - e])) / (2.0 * e),
    ]
}

fn emit_tri(
    verts: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[i32; 3]>,
    params: &GeometryParams,
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
) {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let n = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let cen = [
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    ];
    let g = grad(params, cen);
    let base = verts.len() as i32;
    if dot(n, g) < 0.0 {
        verts.extend_from_slice(&[a, c, b]);
    } else {
        verts.extend_from_slice(&[a, b, c]);
    }
    faces.push([base, base + 1, base + 2]);
}

fn polygonize_tet(
    verts: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[i32; 3]>,
    params: &GeometryParams,
    p: [[f64; 3]; 4],
    v: [f64; 4],
) {
    let inside = [v[0] < 0.0, v[1] < 0.0, v[2] < 0.0, v[3] < 0.0];
    let n_in: usize = inside.iter().map(|&b| b as usize).sum();
    if n_in == 0 || n_in == 4 {
        return;
    }
    // crossing point per tet edge, None when no sign change
    let mut cross: [Option<[f64; 3]>; 6] = [None; 6];
    for (e, &[i, j]) in TET_EDGES.iter().enumerate() {
        if inside[i] != inside[j] {
            cross[e] = Some(lerp_pt(p[i], v[i], p[j], v[j]));
        }
    }
    let x = |e: usize| cross[e].unwrap();
    if n_in == 1 {
        let k = inside.iter().position(|&b| b).unwrap();
        // the 3 edges incident to k
        let mut es = Vec::new();
        for (e, &[i, j]) in TET_EDGES.iter().enumerate() {
            if i == k || j == k {
                es.push(e);
            }
        }
        emit_tri(verts, faces, params, x(es[0]), x(es[1]), x(es[2]));
    } else if n_in == 3 {
        let k = inside.iter().position(|&b| !b).unwrap();
        let mut es = Vec::new();
        for (e, &[i, j]) in TET_EDGES.iter().enumerate() {
            if i == k || j == k {
                es.push(e);
            }
        }
        emit_tri(verts, faces, params, x(es[0]), x(es[1]), x(es[2]));
    } else {
        // 2 inside (a,b), 2 outside (o1,o2): quad -> 2 tris
        let mut ins = Vec::new();
        let mut outs = Vec::new();
        for (k, &b) in inside.iter().enumerate() {
            if b {
                ins.push(k);
            } else {
                outs.push(k);
            }
        }
        let edge_of = |a: usize, b: usize| {
            TET_EDGES
                .iter()
                .position(|&[i, j]| (i == a && j == b) || (i == b && j == a))
                .unwrap()
        };
        let q0 = x(edge_of(ins[0], outs[0]));
        let q1 = x(edge_of(ins[0], outs[1]));
        let q2 = x(edge_of(ins[1], outs[1]));
        let q3 = x(edge_of(ins[1], outs[0]));
        emit_tri(verts, faces, params, q0, q1, q2);
        emit_tri(verts, faces, params, q0, q2, q3);
    }
}

// ---------------------------------------------------------------------------
// public bake API
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default)]
pub struct BakedMesh {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[i32; 3]>,
}

impl BakedMesh {
    pub fn is_empty(&self) -> bool {
        self.faces.is_empty()
    }

    pub fn triangle_count(&self) -> usize {
        self.faces.len()
    }
}

/// Signed volume (outward winding assumed). Useful for bake sanity checks.
pub fn mesh_volume(verts: &[[f64; 3]], faces: &[[i32; 3]]) -> f64 {
    let mut v = 0.0;
    for f in faces {
        let a = verts[f[0] as usize];
        let b = verts[f[1] as usize];
        let c = verts[f[2] as usize];
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    v / 6.0
}

pub const MAX_BAKE_NODES: usize = 6_000_000;

/// Bake world-frame `params` into a triangle soup at `step` resolution.
/// Unbounded params (plane / infinite cylinder) and non-solids (circle) error.
pub fn bake(params: &GeometryParams, step: f64) -> Result<BakedMesh, String> {
    if !(step > 0.0) || !step.is_finite() {
        return Err(format!("bake: bad step {step}"));
    }
    match params {
        GeometryParams::CircleParams(_) => return Err("bake: circle is not a solid".into()),
        _ => {}
    }
    let [lo, hi] = bounds_of(params).ok_or("bake: unbounded geometry (plane/infinite)")?;
    let nx = ((hi[0] - lo[0]) / step).ceil().max(1.0) as usize;
    let ny = ((hi[1] - lo[1]) / step).ceil().max(1.0) as usize;
    let nz = ((hi[2] - lo[2]) / step).ceil().max(1.0) as usize;
    let nodes = (nx + 1).saturating_mul(ny + 1).saturating_mul(nz + 1);
    if nodes > MAX_BAKE_NODES {
        return Err(format!(
            "bake: grid {nx}x{ny}x{nz} ({nodes} nodes) exceeds limit {MAX_BAKE_NODES}; raise step"
        ));
    }
    let dx = (hi[0] - lo[0]) / nx as f64;
    let dy = (hi[1] - lo[1]) / ny as f64;
    let dz = (hi[2] - lo[2]) / nz as f64;
    let at = |i: usize, j: usize, k: usize| {
        [
            lo[0] + dx * i as f64,
            lo[1] + dy * j as f64,
            lo[2] + dz * k as f64,
        ]
    };
    let mut vals = vec![0.0f64; nodes];
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                vals[k * (ny + 1) * (nx + 1) + j * (nx + 1) + i] = field(params, at(i, j, k));
            }
        }
    }
    let val = |i: usize, j: usize, k: usize| vals[k * (ny + 1) * (nx + 1) + j * (nx + 1) + i];
    let mut mesh = BakedMesh::default();
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let mut cp = [[0.0; 3]; 8];
                let mut cv = [0.0; 8];
                for (c, &[ox, oy, oz]) in CORN.iter().enumerate() {
                    let (ii, jj, kk) = (i + ox, j + oy, k + oz);
                    cp[c] = at(ii, jj, kk);
                    cv[c] = val(ii, jj, kk);
                }
                for tet in &TETS {
                    let p = [cp[tet[0]], cp[tet[1]], cp[tet[2]], cp[tet[3]]];
                    let v = [cv[tet[0]], cv[tet[1]], cv[tet[2]], cv[tet[3]]];
                    polygonize_tet(&mut mesh.vertices, &mut mesh.faces, params, p, v);
                }
            }
        }
    }
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        box_geometry, cone_geometry, csg_geometry, cylinder_geometry, ellipsoid_geometry,
        sphere_geometry, torus_geometry, trimesh_geometry, CsgOp, Geometry,
    };

    fn world(g: &Geometry) -> GeometryParams {
        // identity motor: local == world; mirrors geom_to_camera for rigid parts
        crate::identity_params(g)
    }

    #[test]
    fn sphere_volume() {
        let p = world(&Geometry::SphereGeometry(sphere_geometry(1.0)));
        let m = bake(&p, 0.08).unwrap();
        assert!(m.triangle_count() > 1000);
        let v = mesh_volume(&m.vertices, &m.faces).abs();
        assert!((v - 4.18879).abs() < 4.18879 * 0.08, "V={v}");
    }

    #[test]
    fn box_volume() {
        let p = world(&Geometry::BoxGeometry(box_geometry(2.0, 2.0, 2.0)));
        let m = bake(&p, 0.1).unwrap();
        let v = mesh_volume(&m.vertices, &m.faces).abs();
        assert!((v - 8.0).abs() < 8.0 * 0.05, "V={v}");
    }

    #[test]
    fn difference_removes_volume() {
        let g = Geometry::CsgGeometry(csg_geometry(
            CsgOp::Difference,
            vec![
                Geometry::SphereGeometry(sphere_geometry(1.0)),
                Geometry::BoxGeometry(box_geometry(1.0, 1.0, 1.0)),
            ],
        ));
        let p = world(&g);
        let m = bake(&p, 0.08).unwrap();
        let v = mesh_volume(&m.vertices, &m.faces).abs();
        // sphere minus centered 1^3 box: 4.19 - 1 = 3.19, tolerance for MC facets
        assert!((v - 3.18879).abs() < 0.35, "V={v}");
        assert!(v < 4.18879);
    }

    #[test]
    fn torus_and_cone_bake() {
        let t = world(&Geometry::TorusGeometry(torus_geometry(1.0, 0.3)));
        let mt = bake(&t, 0.08).unwrap();
        assert!(mt.triangle_count() > 500);
        let c = world(&Geometry::ConeGeometry(cone_geometry(0.5, 1.0)));
        let mc = bake(&c, 0.06).unwrap();
        let v = mesh_volume(&mc.vertices, &mc.faces).abs();
        // (1/3)πr²h
        assert!((v - 0.261799).abs() < 0.05, "V={v}");
    }

    #[test]
    fn cylinder_and_ellipsoid_bake() {
        let cy = world(&Geometry::CylinderGeometry(cylinder_geometry(0.5, 2.0)));
        let m = bake(&cy, 0.06).unwrap();
        let v = mesh_volume(&m.vertices, &m.faces).abs();
        assert!(
            (v - std::f64::consts::PI * 0.25 * 2.0).abs() < 0.15,
            "V={v}"
        );
        let el = world(&Geometry::EllipsoidGeometry(ellipsoid_geometry(
            1.0, 0.5, 0.5,
        )));
        let me = bake(&el, 0.06).unwrap();
        let ve = mesh_volume(&me.vertices, &me.faces).abs();
        assert!((ve - 4.18879 * 0.25).abs() < 0.2, "V={ve}");
    }

    #[test]
    fn trimesh_parity_bake() {
        // unit cube as 12 triangles
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
        let g = Geometry::TrimeshGeometry(trimesh_geometry(&v, &f));
        let p = world(&g);
        assert!(field(&p, [0.5, 0.5, 0.5]) < 0.0);
        assert!(field(&p, [2.0, 0.5, 0.5]) > 0.0);
        let m = bake(&p, 0.1).unwrap();
        let vv = mesh_volume(&m.vertices, &m.faces).abs();
        assert!((vv - 1.0).abs() < 0.12, "V={vv}");
    }

    #[test]
    fn unbounded_and_nonsolid_error() {
        let pl = world(&Geometry::PlaneGeometry(crate::plane_geometry(
            [0.0, 1.0, 0.0],
            0.0,
        )));
        assert!(bounds_of(&pl).is_none());
        assert!(bake(&pl, 0.1).is_err());
        let ci = world(&Geometry::CircleGeometry(crate::circle_geometry(1.0)));
        assert!(bake(&ci, 0.1).is_err());
    }
}
