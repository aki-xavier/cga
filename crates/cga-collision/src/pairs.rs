//! 两两分离距离（凸对精确表）。全部在**世界空间形状**上做 CPU f64 标量计算。
//!
//! 已实现的对（其余 → `None` = Unknown）：
//! - 球–{球, 平面, 盒, 柱（含无限长）, 锥, 环面, 椭球（球心在椭球外）}
//! - 平面–{平面, 盒, 柱（有限长）, 锥, 环面, 椭球}（支撑函数法）
//! - 盒–盒（SAT 15 轴；相离时最近特征对精确距离，穿入时 SAT 最小穿透轴深度）

use super::*;
use shape::Shape;

/// 两两分离距离。对称（内部归一化顺序）。
pub(crate) fn pair_separation(a: &Shape, b: &Shape) -> Option<f64> {
    use Shape::*;
    // 球优先：球–X = X 的带符号距离在球心处的值 − r。
    if let Sphere { c, r } = *a {
        return sdf(b, c).map(|d| d - r);
    }
    if let Sphere { c, r } = *b {
        return sdf(a, c).map(|d| d - r);
    }
    // 平面其次：支撑函数法。
    if let Plane { n, d } = *a {
        return plane_separation(n, d, b);
    }
    if let Plane { n, d } = *b {
        return plane_separation(n, d, a);
    }
    if let (Box { .. }, Box { .. }) = (a, b) {
        return box_box(a, b);
    }
    None
}

// ---- 点–形状带符号距离（实体内部为负） -----------------------------------

pub(crate) fn sdf(s: &Shape, p: [f64; 3]) -> Option<f64> {
    match *s {
        Shape::Sphere { c, r } => Some(norm(sub(p, c)) - r),
        Shape::Plane { n, d } => Some(dot(n, p) - d), // 半空间：内部为负
        Shape::Box { c, axes, half } => Some(sdf_box(c, axes, half, p)),
        Shape::Cylinder { q, u, r, half } => Some(sdf_cylinder(q, u, r, half, p)),
        Shape::Cone { c, axis, r, h } => Some(sdf_cone(c, axis, r, h, p)),
        Shape::Torus {
            c,
            axis,
            major,
            minor,
        } => {
            let q = sub(p, c);
            let z = dot(q, axis);
            let rho = norm(sub(q, scale(axis, z)));
            Some(norm([rho - major, z, 0.0]) - minor)
        }
        Shape::Ellipsoid { c, axes, radii } => {
            super::contacts::ellipsoid_closest(c, axes, radii, p).map(|x| x.0)
        }
    }
}

/// 盒 SDF（精确）：到面的欧氏距离，内部为 −min 面距。
fn sdf_box(c: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3], p: [f64; 3]) -> f64 {
    let q = sub(p, c);
    let mut dmax = f64::NEG_INFINITY;
    let mut out2 = 0.0;
    for i in 0..3 {
        let d = dot(q, axes[i]).abs() - half[i];
        dmax = dmax.max(d);
        out2 += d.max(0.0) * d.max(0.0);
    }
    out2.sqrt() + dmax.min(0.0)
}

/// 柱 SDF（精确）：half ≤ 0 为无限长。
fn sdf_cylinder(q0: [f64; 3], u: [f64; 3], r: f64, half: f64, p: [f64; 3]) -> f64 {
    let w = sub(p, q0);
    let x = dot(w, u);
    let rho = norm(sub(w, scale(u, x)));
    let dx = rho - r;
    if half <= 0.0 {
        return dx;
    }
    let dz = x.abs() - half;
    let outside = (dx.max(0.0) * dx.max(0.0) + dz.max(0.0) * dz.max(0.0)).sqrt();
    outside + dx.max(dz).min(0.0)
}

/// 锥 SDF（精确）：截面三角形（顶点 (0, h/2)，底角 (r, −h/2)，轴底 (0, −h/2)）
/// 的 2D 点–边距离 + 内外判定。
fn sdf_cone(c: [f64; 3], axis: [f64; 3], r: f64, h: f64, p: [f64; 3]) -> f64 {
    let q = sub(p, c);
    let z = dot(q, axis);
    let rho = norm(sub(q, scale(axis, z)));
    let apex = (0.0, h / 2.0);
    let base = (r, -h / 2.0);
    let axbot = (0.0, -h / 2.0);
    let d = dist_seg((rho, z), apex, base).min(dist_seg((rho, z), base, axbot));
    let in_z = (-h / 2.0..=h / 2.0).contains(&z);
    let rmax = r * (h / 2.0 - z) / h;
    let inside = in_z && rho <= rmax;
    if inside {
        -d
    } else {
        d
    }
}

/// 2D 点–线段距离。
pub(crate) fn dist_seg(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let ab = (b.0 - a.0, b.1 - a.1);
    let ap = (p.0 - a.0, p.1 - a.1);
    let len2 = ab.0 * ab.0 + ab.1 * ab.1;
    let t = if len2 > 0.0 {
        ((ap.0 * ab.0 + ap.1 * ab.1) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let dx = ap.0 - t * ab.0;
    let dy = ap.1 - t * ab.1;
    (dx * dx + dy * dy).sqrt()
}

// ---- 平面–X：支撑函数 ----------------------------------------------------

/// 实体在方向 n（单位）上的支撑区间 [lo, hi]（n·x 的最小/最大）。
pub(crate) fn support_range(s: &Shape, n: [f64; 3]) -> Option<(f64, f64)> {
    match *s {
        Shape::Sphere { c, r } => Some((dot(n, c) - r, dot(n, c) + r)),
        Shape::Box { c, axes, half } => {
            let e: f64 = (0..3).map(|i| dot(n, axes[i]).abs() * half[i]).sum();
            Some((dot(n, c) - e, dot(n, c) + e))
        }
        Shape::Cylinder { q, u, r, half } => {
            if half <= 0.0 {
                return None; // 无限长柱与平面没有有限支撑区间
            }
            let nu = dot(n, u);
            let e = half * nu.abs() + r * (1.0 - nu * nu).max(0.0).sqrt();
            Some((dot(n, q) - e, dot(n, q) + e))
        }
        Shape::Cone { c, axis, r, h } => {
            let apex = add(c, scale(axis, h / 2.0));
            let bc = add(c, scale(axis, -h / 2.0));
            let s = (1.0 - dot(n, axis) * dot(n, axis)).max(0.0).sqrt();
            let na = dot(n, apex);
            let nb = dot(n, bc);
            Some((na.min(nb - r * s), na.max(nb + r * s)))
        }
        Shape::Torus {
            c,
            axis,
            major,
            minor,
        } => {
            let na = dot(n, axis);
            let e = major * (1.0 - na * na).max(0.0).sqrt() + minor;
            Some((dot(n, c) - e, dot(n, c) + e))
        }
        Shape::Ellipsoid { c, axes, radii } => {
            let e: f64 = (0..3)
                .map(|i| {
                    let v = dot(n, axes[i]) * radii[i];
                    v * v
                })
                .sum::<f64>()
                .sqrt();
            Some((dot(n, c) - e, dot(n, c) + e))
        }
        Shape::Plane { .. } => None, // 平面–平面在 plane_separation 单列
    }
}

/// 平面（半空间 n·x ≤ d）与形状的分离距离。
pub(crate) fn plane_separation(n: [f64; 3], d: f64, s: &Shape) -> Option<f64> {
    if let Shape::Plane { n: n2, d: d2 } = *s {
        // 平行：|Δd|（法向同向取差，反向取和）；相交：0。注意：两个相交半空间
        // 有无限重叠体积，MTV 无定义——这里的 0 是**表面距离**语义（表面相交
        // 即接触），不是穿入深度。
        return if norm(cross(n, n2)) < 1e-9 {
            Some(if dot(n, n2) > 0.0 {
                (d - d2).abs()
            } else {
                (d + d2).abs()
            })
        } else {
            Some(0.0)
        };
    }
    let (lo, hi) = support_range(s, n)?;
    let (lo, hi) = (lo - d, hi - d);
    if lo > 0.0 {
        Some(lo) // 整体在半空间外：间隙
    } else if hi < 0.0 {
        Some(hi) // 整体在半空间内：hi 是离界面最近的点（负值 = 深度）
    } else {
        Some(-f64::min(-lo, hi)) // 穿过界面：穿透深度取浅侧
    }
}

// ---- 盒–盒：SAT 15 轴 + 最近特征对 ----------------------------------------

pub(crate) fn box_box(a: &Shape, b: &Shape) -> Option<f64> {
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
    // SAT：6 个面法向 + 9 个叉积轴（分离轴定理：布尔相离判定是精确的）。
    let t = sub(cb, ca);
    let extent = |l: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]| -> f64 {
        (0..3).map(|i| dot(l, axes[i]).abs() * half[i]).sum()
    };
    let mut separated = false;
    let mut sat = |l: [f64; 3]| {
        let ll = norm(l);
        if ll < 1e-9 {
            return; // 两轴平行，叉积退化：跳过
        }
        let l = scale(l, 1.0 / ll);
        if extent(l, aa, ha) + extent(l, ab, hb) - dot(t, l).abs() < 0.0 {
            separated = true;
        }
    };
    for ax in aa.iter().chain(ab.iter()) {
        sat(*ax);
    }
    for axa in aa {
        for axb in ab {
            sat(cross(axa, axb));
        }
    }
    if !separated {
        // 穿入：精确 MTV = Minkowski 差 zonotope 上离原点最近的边界点
        // （审计 §7：SAT 15 轴最小穿透只是上界，深穿透棱–棱情形会高估）。
        let mtv = box_mtv_exact(ca, aa, ha, cb, ab, hb);
        return Some(-norm(mtv));
    }
    // 相离：最近特征对（顶点–面 + 边–边）精确距离。
    let va = box_vertices(ca, aa, ha);
    let vb = box_vertices(cb, ab, hb);
    let mut best = f64::INFINITY;
    for v in va {
        for f in 0..6 {
            if let Some(d) = point_face_dist(v, cb, ab, hb, f) {
                best = best.min(d);
            }
        }
    }
    for v in vb {
        for f in 0..6 {
            if let Some(d) = point_face_dist(v, ca, aa, ha, f) {
                best = best.min(d);
            }
        }
    }
    for e1 in box_edges(ca, aa, ha) {
        for e2 in box_edges(cb, ab, hb) {
            best = best.min(seg_seg_dist(e1[0], e1[1], e2[0], e2[1]));
        }
    }
    Some(best)
}

/// 两盒穿入的精确 MTV（作用在 b 上可分离）。
///
/// 第一性原理：Z = A ⊖ B 是 6 生成元 zonotope（中心 c = cA − cB）；原点在内
/// ⇔ 相交；MTV = 原点到 ∂Z 的最近点。面枚举：每个非平行生成元对 (i,j) 给出
/// 面法向 n = unit(e_i × e_j)；落在该面平面上的生成元集合（zone）≥ 2 时面是
/// 2D zonotope（中心对称多边形），顶点按角度排序走两条链。对面取各面到原点的
/// 最近点（投影在面内 → 垂足，否则面边界边）。
pub(crate) fn box_mtv_exact(
    ca: [f64; 3],
    aa: [[f64; 3]; 3],
    ha: [f64; 3],
    cb: [f64; 3],
    ab: [[f64; 3]; 3],
    hb: [f64; 3],
) -> [f64; 3] {
    let e = [
        scale(aa[0], ha[0]),
        scale(aa[1], ha[1]),
        scale(aa[2], ha[2]),
        scale(ab[0], hb[0]),
        scale(ab[1], hb[1]),
        scale(ab[2], hb[2]),
    ];
    let c = sub(ca, cb);
    let mut best_d2 = f64::INFINITY;
    let mut best = [0.0; 3];
    for i in 0..6 {
        for j in i + 1..6 {
            let cr = cross(e[i], e[j]);
            if norm(cr) < 1e-12 {
                continue;
            }
            let n0 = unit(cr);
            for s in [1.0, -1.0] {
                let n = scale(n0, s);
                // zone：与面平行的生成元；面心 = c + 非 zone 生成元的支撑。
                let zone: Vec<usize> = (0..6).filter(|&k| dot(e[k], n).abs() < 1e-12).collect();
                let mut fc = c;
                for (k, &ek) in e.iter().enumerate() {
                    if !zone.contains(&k) {
                        fc = add(fc, scale(ek, if dot(ek, n) >= 0.0 { 1.0 } else { -1.0 }));
                    }
                }
                // 面法向朝外（离 c 远的一侧）。
                let (n, fc) = if dot(n, sub(fc, c)) < 0.0 {
                    let nf = scale(n, -1.0);
                    let mut f2 = c;
                    for (k, &ek) in e.iter().enumerate() {
                        if !zone.contains(&k) {
                            f2 = add(f2, scale(ek, if dot(ek, nf) >= 0.0 { 1.0 } else { -1.0 }));
                        }
                    }
                    (nf, f2)
                } else {
                    (n, fc)
                };
                // 2D zonotope 多边形（zone 生成元排序走链）。
                let poly = zone_polygon(&e, &zone, n, fc);
                // 原点到该面的最近点。
                let t = dot(n, fc);
                let proj = scale(n, t);
                let cand = if point_in_convex_2d(&poly, fc, n, proj) {
                    proj
                } else {
                    // 面边界边的最近点
                    let mut q = poly[0];
                    let mut d2min = f64::INFINITY;
                    for k in 0..poly.len() {
                        let p0 = poly[k];
                        let p1 = poly[(k + 1) % poly.len()];
                        let qk = closest_point_on_seg3(proj, p0, p1);
                        let d2 = dot(sub(proj, qk), sub(proj, qk));
                        if d2 < d2min {
                            d2min = d2;
                            q = qk;
                        }
                    }
                    q
                };
                let d2 = dot(cand, cand);
                if d2 < best_d2 {
                    best_d2 = d2;
                    best = cand;
                }
            }
        }
    }
    best
}

/// zone 生成元张成的 2D zonotope 的边界多边形（3D 点，中心对称，质心 = fc）。
fn zone_polygon(e: &[[f64; 3]; 6], zone: &[usize], n: [f64; 3], fc: [f64; 3]) -> Vec<[f64; 3]> {
    let u = unit(e[zone[0]]);
    let v = cross(n, u);
    let mut g2: Vec<(f64, [f64; 3])> = zone
        .iter()
        .map(|&k| {
            let g = e[k];
            let mut ang = dot(g, v).atan2(dot(g, u));
            let mut g = g;
            if ang < 0.0 {
                ang += std::f64::consts::PI;
                g = scale(g, -1.0); // −g 与 g 是同一条边方向
            }
            (ang, g)
        })
        .collect();
    g2.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    // 两条链：V0 = −Σg，依次 +2g 到 V_m = +Σg；另一半边界中心对称 W_k = −V_k。
    let mut sum = [0.0; 3];
    for (_, g) in &g2 {
        sum = add(sum, *g);
    }
    let mut chain = vec![scale(sum, -1.0)];
    for (_, g) in &g2 {
        chain.push(add(*chain.last().unwrap(), scale(*g, 2.0)));
    }
    let m = chain.len() - 1;
    let mut pts: Vec<[f64; 3]> = chain.clone();
    for c in chain.iter().take(m).skip(1) {
        pts.push(scale(*c, -1.0));
    }
    pts.iter().map(|p| add(*p, fc)).collect()
}

/// 点在平面凸多边形内（质心侧判定，与绕向无关）。多边形在法向 n 的平面上。
fn point_in_convex_2d(poly: &[[f64; 3]], fc: [f64; 3], n: [f64; 3], p: [f64; 3]) -> bool {
    for k in 0..poly.len() {
        let a = poly[k];
        let b = poly[(k + 1) % poly.len()];
        let edge = sub(b, a);
        let side = dot(cross(edge, sub(p, a)), n);
        let centroid_side = dot(cross(edge, sub(fc, a)), n);
        if side * centroid_side < -1e-12 {
            return false;
        }
    }
    true
}

fn closest_point_on_seg3(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    let ab = sub(b, a);
    let len2 = dot(ab, ab);
    if len2 < 1e-24 {
        return a;
    }
    let t = (dot(sub(p, a), ab) / len2).clamp(0.0, 1.0);
    add(a, scale(ab, t))
}

pub(crate) fn box_vertices(c: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]) -> Vec<[f64; 3]> {
    let mut out = Vec::with_capacity(8);
    for i in 0..8 {
        let mut p = c;
        for (j, ax) in axes.iter().enumerate() {
            let s = if i & (1 << j) == 0 { -1.0 } else { 1.0 };
            p = add(p, scale(*ax, s * half[j]));
        }
        out.push(p);
    }
    out
}

/// 12 条边（每条两个端点）。
pub(crate) fn box_edges(c: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]) -> Vec<[[f64; 3]; 2]> {
    let mut out = Vec::with_capacity(12);
    for axis in 0..3 {
        let (j, k) = ((axis + 1) % 3, (axis + 2) % 3);
        for s1 in [-1.0, 1.0] {
            for s2 in [-1.0, 1.0] {
                let base = add(
                    add(c, scale(axes[j], s1 * half[j])),
                    scale(axes[k], s2 * half[k]),
                );
                out.push([
                    add(base, scale(axes[axis], -half[axis])),
                    add(base, scale(axes[axis], half[axis])),
                ]);
            }
        }
    }
    out
}

/// 点–面（面 f ∈ 0..6：轴 f/2，符号 f%2）距离。点在该面外侧且投影落在面内
/// 才是分离候选，否则 None。
fn point_face_dist(
    v: [f64; 3],
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    half: [f64; 3],
    f: usize,
) -> Option<f64> {
    let j = f / 2;
    let s = if f.is_multiple_of(2) { 1.0 } else { -1.0 };
    let n = scale(axes[j], s);
    let w = sub(v, c);
    let d = dot(w, n) - half[j];
    if d <= 0.0 {
        return None;
    }
    let (u, k) = ((j + 1) % 3, (j + 2) % 3);
    if dot(w, axes[u]).abs() <= half[u] && dot(w, axes[k]).abs() <= half[k] {
        Some(d)
    } else {
        None
    }
}

/// 线段–线段距离（Ericson 5.1.9）。
fn seg_seg_dist(p1: [f64; 3], q1: [f64; 3], p2: [f64; 3], q2: [f64; 3]) -> f64 {
    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);
    let r = sub(p1, p2);
    let a = dot(d1, d1);
    let e = dot(d2, d2);
    let f = dot(d2, r);
    let (s, t);
    if a <= 1e-12 && e <= 1e-12 {
        return norm(r);
    }
    if a <= 1e-12 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = dot(d1, r);
        if e <= 1e-12 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = dot(d1, d2);
            let denom = a * e - b * b;
            let mut s0 = if denom > 1e-12 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t0 = 0.0;
                s0 = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t0 = 1.0;
                s0 = ((b - c) / a).clamp(0.0, 1.0);
            }
            s = s0;
            t = t0;
        }
    }
    norm(sub(add(p1, scale(d1, s)), add(p2, scale(d2, t))))
}
