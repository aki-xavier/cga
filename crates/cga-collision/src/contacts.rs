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
    box_box, box_edges, box_mtv_exact, box_vertices, plane_separation, sdf, support_range,
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
    let on_sphere = sub(c, vec3_scale(n, r));
    let mid = vec3_scale(add(on_sphere, q), 0.5);
    Some(vec![Contact {
        point: mid,
        normal: if flip { n } else { vec3_scale(n, -1.0) },
        separation: sep,
    }])
}

/// 平面（半空间 n·x ≤ d）与形状的接触。`flip` = 平面是 b（法向取反）。
fn plane_contacts(n: [f64; 3], d: f64, s: &Shape, flip: bool) -> Option<Vec<Contact>> {
    if let Shape::Plane { .. } = s {
        return None; // 平面–平面：接触是一条线，v1 不给
    }
    let (lo, _) = support_range(s, n)?;
    let sep = plane_separation(n, d, s)?;
    if lo - d > 0.0 {
        return Some(Vec::new()); // 确切分离
    }
    // 接触点 = b 在 −n 方向的最深支撑点（穿入时是最深点，接触时恰在界面上）。
    let p = support_point(s, vec3_scale(n, -1.0))?;
    Some(vec![Contact {
        point: p,
        normal: if flip { vec3_scale(n, -1.0) } else { n },
        separation: sep,
    }])
}

// ---- 最近点 / 支撑点 ------------------------------------------------------

/// 形状表面上离 p 最近的点。`None` = 椭球内部点（v1 保守）。
/// 唯一的实现在 `cga_core::query`。
fn closest_point(s: &Shape, p: [f64; 3]) -> Option<[f64; 3]> {
    match *s {
        Shape::Sphere { c, r } => Some(cga_core::closest_point_sphere(c, r, p)),
        Shape::Plane { n, d } => Some(cga_core::closest_point_plane(n, d, p)),
        Shape::Box { c, axes, half } => Some(cga_core::closest_point_box(c, axes, half, p)),
        Shape::Cylinder { q, u, r, half } => {
            Some(cga_core::closest_point_cylinder(q, u, r, half, p))
        }
        Shape::Cone { c, axis, r, h } => Some(cga_core::closest_point_cone(c, axis, r, h, p)),
        Shape::Torus {
            c,
            axis,
            major,
            minor,
        } => Some(cga_core::closest_point_torus(c, axis, major, minor, p)),
        Shape::Ellipsoid { c, axes, radii } => cga_core::closest_point_ellipsoid(c, axes, radii, p),
    }
}

/// 方向 v（单位）上的支撑点（v 方向的最远点）。唯一的实现在 `cga_core::query`。
fn support_point(s: &Shape, v: [f64; 3]) -> Option<[f64; 3]> {
    match *s {
        Shape::Sphere { c, r } => Some(cga_core::support_point_sphere(c, r, v)),
        Shape::Box { c, axes, half } => Some(cga_core::support_point_box(c, axes, half, v)),
        Shape::Cylinder { q, u, r, half } => cga_core::support_point_cylinder(q, u, r, half, v),
        Shape::Cone { c, axis, r, h } => Some(cga_core::support_point_cone(c, axis, r, h, v)),
        Shape::Torus {
            c,
            axis,
            major,
            minor,
        } => Some(cga_core::support_point_torus(c, axis, major, minor, v)),
        Shape::Ellipsoid { c, axes, radii } => cga_core::support_point_ellipsoid(c, axes, radii, v),
        Shape::Plane { .. } => None,
    }
}

// 椭球最近点的唯一实现在 `cga_core::ellipsoid_closest`（被 `pairs::sdf` 与
// `contacts::closest_point` 直接调用）。

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
        vec3_scale(mtv, 1.0 / l)
    } else {
        let t = sub(cb, ca);
        let extent = |l: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]| -> f64 {
            (0..3).map(|i| vec3_dot(l, axes[i]).abs() * half[i]).sum()
        };
        let mut best_axis = [1.0, 0.0, 0.0];
        let mut min_overlap = f64::INFINITY;
        let mut consider = |l: [f64; 3]| {
            let ll = norm(l);
            if ll < 1e-9 {
                return;
            }
            let l = vec3_scale(l, 1.0 / ll);
            let ov = extent(l, aa, ha) + extent(l, ab, hb) - vec3_dot(t, l).abs();
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
                consider(vec3_cross(axa, axb));
            }
        }
        if vec3_dot(best_axis, t) >= 0.0 {
            best_axis
        } else {
            vec3_scale(best_axis, -1.0)
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
        uniq.push(vec3_scale(add(ca, cb), 0.5));
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
    (0..3).all(|i| vec3_dot(q, axes[i]).abs() <= half[i] + 1e-12)
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
    let d0 = vec3_dot(sub(e[0], c), axes[j]) - s * half[j];
    let d1 = vec3_dot(sub(e[1], c), axes[j]) - s * half[j];
    if d0 * d1 > 0.0 || (d0 - d1).abs() < 1e-15 {
        return None;
    }
    let t = d0 / (d0 - d1);
    let p = add(e[0], vec3_scale(sub(e[1], e[0]), t));
    let q = sub(p, c);
    let (u, k) = ((j + 1) % 3, (j + 2) % 3);
    if vec3_dot(q, axes[u]).abs() <= half[u] + 1e-12
        && vec3_dot(q, axes[k]).abs() <= half[k] + 1e-12
    {
        Some(p)
    } else {
        None
    }
}
