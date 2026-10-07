//! 接触信息（C2）：接触点 + 法向 + 分离度。
//!
//! 约定：
//! - `normal` 是 **b 的分离方向**：把 b 沿它平移 |separation|（穿入时）即可分开；
//!   相离时从 a 的最近点指向 b 的最近点。
//! - 接触点取两侧最近表面点的中点（穿入时两点各在对方内部，中点仍在交区内）。
//! - 语义：`Some(vec![])` = 确切无接触（分离）；`Some(非空)` = 接触/穿入；
//!   `None` = Unknown（未实现的对、CSG 差/交、非刚体仿射等，与 `separation` 同域）。

use super::*;
use pairs::{
    box_box, box_edges, box_mtv_exact, box_vertices, dist_seg, plane_separation, sdf, support_range,
};
use shape::Shape;

/// 两个实体的接触列表。域与 [`separation`](crate::separation) 一致。
pub fn contacts(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> Option<Vec<Contact>> {
    let sa = shape::to_shape(a, wa)?;
    let sb = shape::to_shape(b, wb)?;
    pair_contacts(&sa, &sb)
}

fn pair_contacts(a: &Shape, b: &Shape) -> Option<Vec<Contact>> {
    use Shape::*;
    // 球–X：接触点 = b 上离球心最近的表面点与球面点的中点。
    if let Sphere { c, r } = *a {
        return sphere_contacts(c, r, b, false);
    }
    if let Sphere { c, r } = *b {
        return sphere_contacts(c, r, a, true);
    }
    if let Plane { n, d } = *a {
        return plane_contacts(n, d, b, false);
    }
    if let Plane { n, d } = *b {
        return plane_contacts(n, d, a, true);
    }
    if let (Box { .. }, Box { .. }) = (a, b) {
        return box_box_contacts(a, b);
    }
    None
}

/// 球（c, r）与形状 s 的接触。`flip` = 球是 b（法向取反）。
fn sphere_contacts(c: [f64; 3], r: f64, s: &Shape, flip: bool) -> Option<Vec<Contact>> {
    let d = sdf(s, c)?; // 球心到 s 的带符号距离
    let sep = d - r;
    if sep > 0.0 {
        return Some(Vec::new());
    }
    let q = closest_point(s, c)?; // s 上离 c 最近的点
    let dir = sub(c, q);
    let n = if norm(dir) < 1e-12 {
        // 球心正在 s 表面上：任取一个稳定方向（平面用法向，其余用 +z）
        match s {
            Shape::Plane { n, .. } => *n,
            _ => [0.0, 0.0, 1.0],
        }
    } else {
        unit(dir)
    };
    let on_sphere = sub(c, scale(n, r));
    let mid = scale(add(on_sphere, q), 0.5);
    Some(vec![Contact {
        point: mid,
        normal: if flip { n } else { scale(n, -1.0) },
        separation: sep,
    }])
}

/// 平面（半空间 n·x ≤ d）与形状的接触。`flip` = 平面是 b（法向取反）。
fn plane_contacts(n: [f64; 3], d: f64, s: &Shape, flip: bool) -> Option<Vec<Contact>> {
    if let Shape::Plane { .. } = s {
        return None; // 平面–平面：接触是一条线，v1 不给
    }
    let (lo, hi) = support_range(s, n)?;
    let sep = plane_separation(n, d, s)?;
    if lo - d > 0.0 {
        return Some(Vec::new()); // 确切分离
    }
    let _ = hi;
    // 接触点 = b 在 −n 方向的最深支撑点（穿入时是最深点，接触时恰在界面上）。
    let p = support_point(s, scale(n, -1.0))?;
    Some(vec![Contact {
        point: p,
        normal: if flip { scale(n, -1.0) } else { n },
        separation: sep,
    }])
}

// ---- 最近点 / 支撑点 ------------------------------------------------------

/// 形状表面上离 p 最近的点。`None` = 椭球内部点（v1 保守）。
fn closest_point(s: &Shape, p: [f64; 3]) -> Option<[f64; 3]> {
    match *s {
        Shape::Sphere { c, r } => {
            let d = sub(p, c);
            let n = norm(d);
            Some(if n < 1e-12 {
                add(c, [r, 0.0, 0.0])
            } else {
                add(c, scale(d, r / n))
            })
        }
        Shape::Plane { n, d } => Some(sub(p, scale(n, dot(n, p) - d))),
        Shape::Box { c, axes, half } => {
            let q = sub(p, c);
            let mut out = c;
            for i in 0..3 {
                let x = dot(q, axes[i]).clamp(-half[i], half[i]);
                out = add(out, scale(axes[i], x));
            }
            Some(out)
        }
        Shape::Cylinder { q, u, r, half } => {
            let w = sub(p, q);
            let x = dot(w, u);
            let rad = sub(w, scale(u, x));
            let rho = norm(rad);
            let xc = if half <= 0.0 { x } else { x.clamp(-half, half) };
            let rc = if rho > r { r } else { rho };
            let dir = if rho < 1e-12 {
                [1.0, 0.0, 0.0]
            } else {
                scale(rad, 1.0 / rho)
            };
            Some(add(q, add(scale(u, xc), scale(dir, rc))))
        }
        Shape::Cone { c, axis, r, h } => {
            let q = sub(p, c);
            let z = dot(q, axis);
            let rad = sub(q, scale(axis, z));
            let rho = norm(rad);
            // 截面 (ρ, z) 上求截面三角形边的最近点，再映回 3D。
            let apex = (0.0, h / 2.0);
            let base = (r, -h / 2.0);
            let axbot = (0.0, -h / 2.0);
            let (q1, d1) = closest_on_seg((rho, z), apex, base);
            let (q2, d2) = closest_on_seg((rho, z), base, axbot);
            let (qr, qz) = if d1 <= d2 { q1 } else { q2 };
            let dir = if rho < 1e-12 {
                [1.0, 0.0, 0.0]
            } else {
                scale(rad, 1.0 / rho)
            };
            Some(add(c, add(scale(axis, qz), scale(dir, qr))))
        }
        Shape::Torus {
            c,
            axis,
            major,
            minor,
        } => {
            let q = sub(p, c);
            let z = dot(q, axis);
            let rad = sub(q, scale(axis, z));
            let rho = norm(rad);
            // 截面 (ρ, z)：环心 (major, 0)、半径 minor 的圆。
            let d = [rho - major, z];
            let n = (d[0] * d[0] + d[1] * d[1]).sqrt();
            let (qr, qz) = if n < 1e-12 {
                (major + minor, 0.0)
            } else {
                (major + minor * d[0] / n, minor * d[1] / n)
            };
            let dir = if rho < 1e-12 {
                [1.0, 0.0, 0.0]
            } else {
                scale(rad, 1.0 / rho)
            };
            Some(add(c, add(scale(axis, qz), scale(dir, qr))))
        }
        Shape::Ellipsoid { c, axes, radii } => ellipsoid_closest(c, axes, radii, p).map(|x| x.1),
    }
}

/// 2D 线段上的最近点（返回点与距离）。
fn closest_on_seg(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> ((f64, f64), f64) {
    let ab = (b.0 - a.0, b.1 - a.1);
    let ap = (p.0 - a.0, p.1 - a.1);
    let len2 = ab.0 * ab.0 + ab.1 * ab.1;
    let t = if len2 > 0.0 {
        ((ap.0 * ab.0 + ap.1 * ab.1) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let q = (a.0 + t * ab.0, a.1 + t * ab.1);
    ((q), dist_seg(p, a, b))
}

/// 方向 v（单位）上的支撑点（v 方向的最远点）。
fn support_point(s: &Shape, v: [f64; 3]) -> Option<[f64; 3]> {
    match *s {
        Shape::Sphere { c, r } => Some(add(c, scale(v, r))),
        Shape::Box { c, axes, half } => {
            let mut p = c;
            for i in 0..3 {
                let s = if dot(v, axes[i]) >= 0.0 { 1.0 } else { -1.0 };
                p = add(p, scale(axes[i], s * half[i]));
            }
            Some(p)
        }
        Shape::Cylinder { q, u, r, half } => {
            if half <= 0.0 {
                return None;
            }
            let vu = dot(v, u);
            let cap = if vu >= 0.0 { half } else { -half };
            let perp = sub(v, scale(u, vu));
            let pl = norm(perp);
            let rim = if pl < 1e-12 {
                [0.0; 3]
            } else {
                scale(perp, r / pl)
            };
            Some(add(q, add(scale(u, cap), rim)))
        }
        Shape::Cone { c, axis, r, h } => {
            let apex = add(c, scale(axis, h / 2.0));
            let bc = add(c, scale(axis, -h / 2.0));
            let perp = sub(v, scale(axis, dot(v, axis)));
            let pl = norm(perp);
            let rim = if pl < 1e-12 {
                bc
            } else {
                add(bc, scale(perp, r / pl))
            };
            Some(if dot(v, apex) >= dot(v, rim) {
                apex
            } else {
                rim
            })
        }
        Shape::Torus {
            c,
            axis,
            major,
            minor,
        } => {
            let va = dot(v, axis);
            let perp = sub(v, scale(axis, va));
            let pl = norm(perp);
            let m = if pl < 1e-12 {
                c
            } else {
                add(c, scale(perp, major / pl))
            };
            Some(add(m, scale(v, minor)))
        }
        Shape::Ellipsoid { c, axes, radii } => {
            let vi = [dot(v, axes[0]), dot(v, axes[1]), dot(v, axes[2])];
            let s: f64 = (0..3)
                .map(|i| (vi[i] * radii[i]) * (vi[i] * radii[i]))
                .sum::<f64>()
                .sqrt();
            if s < 1e-12 {
                return None;
            }
            let mut p = c;
            for i in 0..3 {
                p = add(p, scale(axes[i], radii[i] * radii[i] * vi[i] / s));
            }
            Some(p)
        }
        Shape::Plane { .. } => None,
    }
}

/// 椭球最近点：外部精确（单调对分），内部 → None。返回 (距离, 点)。
pub(crate) fn ellipsoid_closest(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    radii: [f64; 3],
    p: [f64; 3],
) -> Option<(f64, [f64; 3])> {
    let q = sub(p, c);
    let a = [dot(q, axes[0]), dot(q, axes[1]), dot(q, axes[2])];
    let s: f64 = (0..3).map(|i| (a[i] / radii[i]) * (a[i] / radii[i])).sum();
    if s <= 1.0 {
        return None;
    }
    let f = |t: f64| -> f64 {
        (0..3)
            .map(|i| {
                let v = radii[i] * a[i] / (t + radii[i] * radii[i]);
                v * v
            })
            .sum::<f64>()
            - 1.0
    };
    let mut lo = 0.0;
    let mut hi = 1.0;
    while f(hi) > 0.0 {
        hi *= 2.0;
        if hi > 1e12 {
            return None;
        }
    }
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let t = 0.5 * (lo + hi);
    let mut out = c;
    let mut d2 = 0.0;
    for i in 0..3 {
        let x = radii[i] * radii[i] * a[i] / (t + radii[i] * radii[i]);
        d2 += (x - a[i]) * (x - a[i]);
        out = add(out, scale(axes[i], x));
    }
    Some((d2.sqrt(), out))
}

// ---- 盒–盒接触流形 --------------------------------------------------------

fn box_box_contacts(a: &Shape, b: &Shape) -> Option<Vec<Contact>> {
    let (
        Shape::Box {
            c: ca,
            axes: aa,
            half: ha,
        },
        Shape::Box {
            c: cb,
            axes: ab,
            half: hb,
        },
    ) = (*a, *b)
    else {
        return None;
    };
    let sep = box_box(a, b)?;
    if sep > 0.0 {
        return Some(Vec::new());
    }
    // 分离方向 = 精确 MTV 方向（box_mtv_exact）。相切（|mtv|≈0）时退化为
    // SAT 最小穿透轴。
    let mtv = box_mtv_exact(ca, aa, ha, cb, ab, hb);
    let l = norm(mtv);
    let normal = if l > 1e-12 {
        scale(mtv, 1.0 / l)
    } else {
        let t = sub(cb, ca);
        let extent = |l: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]| -> f64 {
            (0..3).map(|i| dot(l, axes[i]).abs() * half[i]).sum()
        };
        let mut best_axis = [1.0, 0.0, 0.0];
        let mut min_overlap = f64::INFINITY;
        let mut consider = |l: [f64; 3]| {
            let ll = norm(l);
            if ll < 1e-9 {
                return;
            }
            let l = scale(l, 1.0 / ll);
            let ov = extent(l, aa, ha) + extent(l, ab, hb) - dot(t, l).abs();
            if ov < min_overlap {
                min_overlap = ov;
                best_axis = l;
            }
        };
        for ax in aa.iter().chain(ab.iter()) {
            consider(*ax);
        }
        for axa in aa {
            for axb in ab {
                consider(cross(axa, axb));
            }
        }
        if dot(best_axis, t) >= 0.0 {
            best_axis
        } else {
            scale(best_axis, -1.0)
        }
    };

    // 接触点：A 在 B 内的顶点 + B 在 A 内的顶点 + A 的边 × B 的面 + B 的边 × A 的面。
    let mut pts: Vec<[f64; 3]> = Vec::new();
    for v in box_vertices(ca, aa, ha) {
        if point_in_box(v, cb, ab, hb) {
            pts.push(v);
        }
    }
    for v in box_vertices(cb, ab, hb) {
        if point_in_box(v, ca, aa, ha) {
            pts.push(v);
        }
    }
    for e in box_edges(ca, aa, ha) {
        for f in 0..6 {
            if let Some(p) = edge_face_cross(e, cb, ab, hb, f) {
                pts.push(p);
            }
        }
    }
    for e in box_edges(cb, ab, hb) {
        for f in 0..6 {
            if let Some(p) = edge_face_cross(e, ca, aa, ha, f) {
                pts.push(p);
            }
        }
    }
    // 去重（1e-9 容差）
    let mut uniq: Vec<[f64; 3]> = Vec::new();
    'outer: for p in pts {
        for q in &uniq {
            if norm(sub(p, *q)) < 1e-9 {
                continue 'outer;
            }
        }
        uniq.push(p);
    }
    if uniq.is_empty() {
        // 深包容等无面交点的情形：退化为两心间的中点。
        uniq.push(scale(add(ca, cb), 0.5));
    }
    Some(
        uniq.into_iter()
            .map(|point| Contact {
                point,
                normal,
                separation: sep,
            })
            .collect(),
    )
}

fn point_in_box(p: [f64; 3], c: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]) -> bool {
    let q = sub(p, c);
    (0..3).all(|i| dot(q, axes[i]).abs() <= half[i] + 1e-12)
}

/// 边（两端点）与盒面（轴 f/2、符号 f%2）的交点：两端点在面两侧且交点落在面内。
fn edge_face_cross(
    e: [[f64; 3]; 2],
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    half: [f64; 3],
    f: usize,
) -> Option<[f64; 3]> {
    let j = f / 2;
    let s = if f.is_multiple_of(2) { 1.0 } else { -1.0 };
    let d0 = dot(sub(e[0], c), axes[j]) - s * half[j];
    let d1 = dot(sub(e[1], c), axes[j]) - s * half[j];
    if d0 * d1 > 0.0 || (d0 - d1).abs() < 1e-15 {
        return None;
    }
    let t = d0 / (d0 - d1);
    let p = add(e[0], scale(sub(e[1], e[0]), t));
    let q = sub(p, c);
    let (u, k) = ((j + 1) % 3, (j + 2) % 3);
    if dot(q, axes[u]).abs() <= half[u] + 1e-12 && dot(q, axes[k]).abs() <= half[k] + 1e-12 {
        Some(p)
    } else {
        None
    }
}
