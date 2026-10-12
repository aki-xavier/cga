//! 图元几何的**唯一标量定义**（本模块之外不再重复任何图元公式）。
//!
//! 三种查询语义不同，不可互相替代：
//!
//! - `field`：**符号隐式**（< 0 内部 / > 0 外部 / = 0 表面）。量纲不承诺（各图元各自的
//!   二次/四次形式），只保证符号与连续性——marching 网格化、CSG 折叠、符号判定都只用符号。
//! - `contains`：`field` 的符号化，**三值**（无法给出可靠判定的图元返回 `Unknown`，绝不假装）。
//! - `distance` / `closest_point` / `support_*`：**精确欧氏**量；只有存在闭式的图元才给出，
//!   否则 `None`（诚实边界，不做近似冒充）。
//!
//! CGA 锚点（blade 图元）：`sphere`/`plane`/`circle`/`line` 的 `field` 就是它的 GA 关联谓词
//! ——`sphere` 的 `X·ip(S)` 等于 `−(|p−c|²−r²)/2`（即 `field = −2·ip`），`plane` 的
//! `X·ip(P)` 等于有符号距离（即 `field = ip`）。两者的恒等关系由
//! `tests::blade_predicates_agree_with_field` 钉死（这样下层用算术形式加速时，
//! 定义仍然锚在 GA 谓词上）。
//!
//! 非 blade 图元：`box` = 6 个半空间 blade 的交（`box_field` 即逐轴 max 折叠）；
//! `cylinder`/`cone`/`torus`/`ellipsoid`/`cyclide` 用规范参数形式，其中
//! `cone`/`torus`/`ellipsoid`/`cyclide` 通过 `(a_inv3, t_inv)` 进入自己的规范系
//! （与渲染器/碰撞同一个 `a_inv3,t_inv` 约定，见 `affine::affine_from_motor`）。

use crate::csg_node::CsgOp;
use crate::geometry::*;
use crate::motors::Mat3;
use crate::{vec3_dot, vec3_sub};

/// 点归属：三值（`Unknown` = 该图元/该输入无法给出可靠判定）。
///
/// 组合子是三值逻辑（Kleene）：`and` 见假即假、`or` 见真即真、`not` 只翻确定。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Membership {
    Inside,
    Outside,
    Unknown,
}

impl Membership {
    pub fn and(self, o: Membership) -> Membership {
        match (self, o) {
            (Membership::Outside, _) | (_, Membership::Outside) => Membership::Outside,
            (Membership::Inside, Membership::Inside) => Membership::Inside,
            _ => Membership::Unknown,
        }
    }
    pub fn or(self, o: Membership) -> Membership {
        match (self, o) {
            (Membership::Inside, _) | (_, Membership::Inside) => Membership::Inside,
            (Membership::Outside, Membership::Outside) => Membership::Outside,
            _ => Membership::Unknown,
        }
    }
}

impl std::ops::Not for Membership {
    type Output = Membership;
    fn not(self) -> Membership {
        match self {
            Membership::Inside => Membership::Outside,
            Membership::Outside => Membership::Inside,
            Membership::Unknown => Membership::Unknown,
        }
    }
}

/// 父系 → 图元规范系（`affine::affine_from_motor` 的逆映射约定）。
pub fn to_local(a_inv3: Mat3, t_inv: [f64; 3], p: [f64; 3]) -> [f64; 3] {
    [
        a_inv3[0][0] * p[0] + a_inv3[0][1] * p[1] + a_inv3[0][2] * p[2] + t_inv[0],
        a_inv3[1][0] * p[0] + a_inv3[1][1] * p[1] + a_inv3[1][2] * p[2] + t_inv[1],
        a_inv3[2][0] * p[0] + a_inv3[2][1] * p[1] + a_inv3[2][2] * p[2] + t_inv[2],
    ]
}

/// 规范系正向映射（图元规范系 → 父系），与 `to_local` 互为逆。
pub fn a_fwd_point(a_fwd: &[f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        a_fwd[0] * p[0] + a_fwd[1] * p[1] + a_fwd[2] * p[2] + a_fwd[3],
        a_fwd[4] * p[0] + a_fwd[5] * p[1] + a_fwd[6] * p[2] + a_fwd[7],
        a_fwd[8] * p[0] + a_fwd[9] * p[1] + a_fwd[10] * p[2] + a_fwd[11],
    ]
}

/// **规范系（轴 = +z）**下的径向距离 = `√(x² + y²)`。只对已进入规范系的坐标使用；
/// 轴为任意方向的参数（如 `CylinderParams.u`）必须用 `vec3_len`。
fn radial_xy(l: [f64; 3]) -> f64 {
    (l[0] * l[0] + l[1] * l[1]).sqrt()
}

// ---- 符号隐式（截面形式：只依赖 (ρ, s)，与坐标系无关）--------------------

/// 柱面/柱体：轴 = +z，`half ≤ 0` 为无限长。
pub fn cylinder_field_section(r: f64, half: f64, rho: f64, s: f64) -> f64 {
    let f = rho * rho - r * r;
    if half <= 0.0 {
        return f;
    }
    f.max(s.abs() - half)
}

/// 锥（顶点 z=+h/2，底 z=−h/2）。
pub fn cone_field_section(r: f64, h: f64, rho: f64, z: f64) -> f64 {
    let s = z - h / 2.0;
    let k2 = 1.0 + (r / h) * (r / h);
    let f = rho * rho + s * s - k2 * s * s;
    f.max(s).max(-(s + h))
}

/// 环面（轴 = +z）。`arc < 2π` 时为弧管：`θ`（+x 起算的方位角，∈[0, 2π)）超过
/// `arc` 即算外部 → `max(field, θ − arc)`。
pub fn torus_field_section(major: f64, minor: f64, arc: f64, rho: f64, z: f64, theta: f64) -> f64 {
    let r2 = major * major;
    let f = (rho * rho + z * z + r2 - minor * minor).powi(2) - 4.0 * r2 * rho * rho;
    if arc >= std::f64::consts::TAU {
        return f;
    }
    f.max(theta - arc)
}

fn ellipsoid_field_local(l: [f64; 3]) -> f64 {
    vec3_dot(l, l) - 1.0
}

fn sphere_field_local(r: f64, l: [f64; 3]) -> f64 {
    vec3_dot(l, l) - r * r
}

fn cyclide_field_local(p: &CyclideParams, l: [f64; 3]) -> f64 {
    // 环纹面的隐式场唯一实现在 `DupinCyclide::implicit`（cga-core 早已完整，
    // 但此前生产零调用——本次接线，不再是第二份公式）。
    crate::cyclide::DupinCyclide::new(p.a, p.b, p.d, p.shift).implicit(l[0], l[1], l[2])
}

fn box_field_local(half: [f64; 3], l: [f64; 3]) -> f64 {
    let mut f = (l[0].abs() - half[0]).max(l[1].abs() - half[1]);
    f = f.max(l[2].abs() - half[2]);
    f
}

impl GeometryParams {
    /// 符号隐式场。`None` = 该图元没有隐式（圆片）或由子件继承的不确定性。
    pub fn field(&self, x: [f64; 3]) -> Option<f64> {
        match self {
            GeometryParams::SphereParams(p) => Some(sphere_field_local(p.r, vec3_sub(x, p.c))),
            GeometryParams::PlaneParams(p) => Some(vec3_dot(p.n, x) - p.d),
            GeometryParams::CylinderParams(p) => {
                let w = vec3_sub(x, p.q);
                let s = vec3_dot(w, p.u);
                let rad = vec3_sub(w, [p.u[0] * s, p.u[1] * s, p.u[2] * s]);
                // 轴是任意方向 `u`（不是规范系的 +z）⇒ 径向距离取**三维**范数。
                Some(cylinder_field_section(p.r, p.h, crate::vec3_len(rad), s))
            }
            GeometryParams::BoxParams(p) => {
                let q = vec3_sub(x, p.c);
                Some(box_field_local(
                    p.half,
                    [
                        vec3_dot(q, p.axes[0]),
                        vec3_dot(q, p.axes[1]),
                        vec3_dot(q, p.axes[2]),
                    ],
                ))
            }
            GeometryParams::CircleParams(_) => None,
            GeometryParams::ConeParams(p) => {
                let l = to_local(p.a_inv3, p.t_inv, x);
                Some(cone_field_section(p.r, p.h, radial_xy(l), l[2]))
            }
            GeometryParams::TorusParams(p) => {
                let l = to_local(p.a_inv3, p.t_inv, x);
                let mut theta = l[1].atan2(l[0]);
                if theta < 0.0 {
                    theta += std::f64::consts::TAU;
                }
                Some(torus_field_section(
                    p.major,
                    p.minor,
                    p.arc,
                    radial_xy(l),
                    l[2],
                    theta,
                ))
            }
            GeometryParams::EllipsoidParams(p) => {
                Some(ellipsoid_field_local(to_local(p.a_inv3, p.t_inv, x)))
            }
            GeometryParams::CyclideParams(p) => {
                Some(cyclide_field_local(p, to_local(p.a_inv3, p.t_inv, x)))
            }
            GeometryParams::CsgParams(p) => {
                let mut vals = Vec::with_capacity(p.children.len());
                for c in &p.children {
                    vals.push(c.field(x)?);
                }
                let mut it = vals.into_iter();
                let first = it.next()?;
                Some(match p.op {
                    CsgOp::Union => it.fold(first, f64::min),
                    CsgOp::Intersection => it.fold(first, f64::max),
                    CsgOp::Difference => it.fold(first, |f, g| f.max(-g)),
                })
            }
            GeometryParams::AffineParams(p) => p.inner.field(to_local(p.a_inv3, p.t_inv, x)),
        }
    }

    /// 点归属（三值）。`field ≤ 0` 为内部（表面算内部：接触语义）。
    pub fn contains(&self, x: [f64; 3]) -> Membership {
        match self {
            // 圆片没有实体内部；环纹面自交导致内外语义退化（读作 Unknown，不假装）。
            GeometryParams::CircleParams(_) | GeometryParams::CyclideParams(_) => {
                Membership::Unknown
            }
            // 部分环面（弧管）的实体语义在 v1 未定义。
            GeometryParams::TorusParams(p) if p.arc < std::f64::consts::TAU - 1e-9 => {
                Membership::Unknown
            }
            GeometryParams::CsgParams(p) => {
                let mut it = p.children.iter().map(|c| c.contains(x));
                let Some(first) = it.next() else {
                    return Membership::Unknown;
                };
                match p.op {
                    CsgOp::Union => it.fold(first, Membership::or),
                    CsgOp::Intersection => it.fold(first, Membership::and),
                    CsgOp::Difference => it.fold(first, |acc, m| acc.and(!m)),
                }
            }
            GeometryParams::AffineParams(p) => p.inner.contains(to_local(p.a_inv3, p.t_inv, x)),
            _ => match self.field(x) {
                Some(f) if f <= 0.0 => Membership::Inside,
                Some(_) => Membership::Outside,
                None => Membership::Unknown,
            },
        }
    }

    /// 精确欧氏有符号距离（内部为负）。`None` = 无闭式/不支持（圆片、环纹面、CSG、部分环面、椭球内部）。
    pub fn distance(&self, x: [f64; 3]) -> Option<f64> {
        match self {
            GeometryParams::SphereParams(p) => Some(sdf_sphere(p.c, p.r, x)),
            GeometryParams::PlaneParams(p) => sdf_plane(p.n, p.d, x),
            GeometryParams::BoxParams(p) => Some(sdf_box(p.c, p.axes, p.half, x)),
            GeometryParams::CylinderParams(p) => Some(sdf_cylinder(p.q, p.u, p.r, p.h, x)),
            GeometryParams::ConeParams(p) => {
                let l = to_local(p.a_inv3, p.t_inv, x);
                Some(sdf_cone_section(p.r, p.h, radial_xy(l), l[2]))
            }
            GeometryParams::TorusParams(p) => {
                if p.arc < std::f64::consts::TAU - 1e-9 {
                    return None;
                }
                let l = to_local(p.a_inv3, p.t_inv, x);
                Some(sdf_torus_section(p.major, p.minor, radial_xy(l), l[2]))
            }
            GeometryParams::EllipsoidParams(p) => {
                let l = to_local(p.a_inv3, p.t_inv, x);
                // 半径由 a_fwd 的三列长度还原（a_fwd = 电机 · diag(radii)，列长即 radii）。
                let radii = [
                    (p.a_fwd[0] * p.a_fwd[0] + p.a_fwd[4] * p.a_fwd[4] + p.a_fwd[8] * p.a_fwd[8])
                        .sqrt(),
                    (p.a_fwd[1] * p.a_fwd[1] + p.a_fwd[5] * p.a_fwd[5] + p.a_fwd[9] * p.a_fwd[9])
                        .sqrt(),
                    (p.a_fwd[2] * p.a_fwd[2] + p.a_fwd[6] * p.a_fwd[6] + p.a_fwd[10] * p.a_fwd[10])
                        .sqrt(),
                ];
                let a = [l[0] * radii[0], l[1] * radii[1], l[2] * radii[2]];
                ellipsoid_closest_local(radii, a).map(|(d, _)| d)
            }
            _ => None,
        }
    }

    /// 轴向包围盒（保守、闭式；`None` = 无界或不可界定）。
    pub fn bounds(&self) -> Option<[[f64; 3]; 2]> {
        match self {
            GeometryParams::SphereParams(p) => Some([
                [p.c[0] - p.r, p.c[1] - p.r, p.c[2] - p.r],
                [p.c[0] + p.r, p.c[1] + p.r, p.c[2] + p.r],
            ]),
            GeometryParams::PlaneParams(_) => None,
            GeometryParams::CylinderParams(p) => {
                if p.h <= 0.0 {
                    return None; // 无限长柱无界
                }
                // 紧包围盒：沿轴分量 |u_i|·h，垂直轴的径向分量 r·√(1−u_i²)。
                let mut lo = [0.0; 3];
                let mut hi = [0.0; 3];
                for i in 0..3 {
                    let ui = p.u[i];
                    let e = ui.abs() * p.h + p.r * (1.0 - ui * ui).max(0.0).sqrt();
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
            GeometryParams::EllipsoidParams(p) => {
                Some(corners_bounds([-1.0; 3], [1.0; 3], &p.a_fwd))
            }
            GeometryParams::CyclideParams(p) => {
                let r = p.d + p.c;
                Some(corners_bounds(
                    [p.shift[0] - p.a - r, p.shift[1] - p.b - r, p.shift[2] - r],
                    [p.shift[0] + p.a + r, p.shift[1] + p.b + r, p.shift[2] + r],
                    &p.a_fwd,
                ))
            }
            GeometryParams::CsgParams(p) => {
                let bs: Vec<_> = p.children.iter().map(|c| c.bounds()).collect();
                match p.op {
                    CsgOp::Difference => bs.into_iter().next().unwrap_or(None),
                    CsgOp::Union => combine_bounds(&bs, Fold::Union),
                    CsgOp::Intersection => combine_bounds(&bs, Fold::Intersect),
                }
            }
            GeometryParams::AffineParams(p) => {
                let b = p.inner.bounds()?;
                Some(corners_bounds(b[0], b[1], &p.a_fwd))
            }
            GeometryParams::CircleParams(_) => None,
        }
    }
}

fn corners_bounds(lo: [f64; 3], hi: [f64; 3], a_fwd: &[f64; 16]) -> [[f64; 3]; 2] {
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for &i in &[lo[0], hi[0]] {
        for &j in &[lo[1], hi[1]] {
            for &k in &[lo[2], hi[2]] {
                let q = a_fwd_point(a_fwd, [i, j, k]);
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

/// 子件包围盒折叠方向：并集外扩，交集结内收。
#[derive(Clone, Copy)]
enum Fold {
    Union,
    Intersect,
}

fn combine_bounds(list: &[Option<[[f64; 3]; 2]>], fold: Fold) -> Option<[[f64; 3]; 2]> {
    let mut out: Option<[[f64; 3]; 2]> = None;
    for b in list.iter().flatten() {
        out = Some(match out {
            None => *b,
            Some([mn, mx]) => {
                let mut lo = mn;
                let mut hi = mx;
                for a in 0..3 {
                    match fold {
                        Fold::Union => {
                            if b[0][a] < lo[a] {
                                lo[a] = b[0][a];
                            }
                            if b[1][a] > hi[a] {
                                hi[a] = b[1][a];
                            }
                        }
                        Fold::Intersect => {
                            if b[0][a] > lo[a] {
                                lo[a] = b[0][a];
                            }
                            if b[1][a] < hi[a] {
                                hi[a] = b[1][a];
                            }
                        }
                    }
                }
                [lo, hi]
            }
        });
    }
    out
}

// ---- 精确欧氏查询（任意坐标系一致；只依赖图元自身参数）--------------------

/// 点到球的精确距离（内部为负）。
pub fn sdf_sphere(c: [f64; 3], r: f64, p: [f64; 3]) -> f64 {
    let d = vec3_sub(p, c);
    vec3_dot(d, d).sqrt() - r
}

/// 点到平面（半空间 `n·x ≤ d`，内部为负）的精确距离；`n` 任意非零。
pub fn sdf_plane(n: [f64; 3], d: f64, p: [f64; 3]) -> Option<f64> {
    let nl = vec3_dot(n, n).sqrt();
    if nl == 0.0 {
        return None;
    }
    Some((vec3_dot(n, p) - d) / nl)
}

/// 点到盒的精确距离（内部为 −最小面距）。
pub fn sdf_box(c: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3], p: [f64; 3]) -> f64 {
    let q = vec3_sub(p, c);
    let mut dmax = f64::NEG_INFINITY;
    let mut out2 = 0.0;
    for i in 0..3 {
        let d = vec3_dot(q, axes[i]).abs() - half[i];
        dmax = dmax.max(d);
        out2 += d.max(0.0) * d.max(0.0);
    }
    out2.sqrt() + dmax.min(0.0)
}

/// 点到柱的精确距离（截面形式）。`half ≤ 0` 为无限长。
pub fn sdf_cylinder_section(r: f64, half: f64, rho: f64, s: f64) -> f64 {
    let dx = rho - r;
    if half <= 0.0 {
        return dx;
    }
    let dz = s.abs() - half;
    (dx.max(0.0) * dx.max(0.0) + dz.max(0.0) * dz.max(0.0)).sqrt() + dx.max(dz).min(0.0)
}

/// 点到柱的精确距离（世界参数，轴点 `q`、轴向 `u`）。
pub fn sdf_cylinder(q: [f64; 3], u: [f64; 3], r: f64, half: f64, p: [f64; 3]) -> f64 {
    let w = vec3_sub(p, q);
    let s = vec3_dot(w, u);
    let rad = vec3_sub(w, [u[0] * s, u[1] * s, u[2] * s]);
    sdf_cylinder_section(r, half, vec3_dot(rad, rad).sqrt(), s)
}

/// 点到锥的精确距离（截面形式）：截面三角形（顶点 (0, h/2)、底角 (r, −h/2)、
/// 轴底 (0, −h/2)）的 2D 点–边距离 + 内外判定。
pub fn sdf_cone_section(r: f64, h: f64, rho: f64, z: f64) -> f64 {
    let apex = (0.0, h / 2.0);
    let base = (r, -h / 2.0);
    let axbot = (0.0, -h / 2.0);
    let d = dist_seg((rho, z), apex, base).min(dist_seg((rho, z), base, axbot));
    let in_z = (-h / 2.0..=h / 2.0).contains(&z);
    let rmax = r * (h / 2.0 - z) / h;
    if in_z && rho <= rmax {
        -d
    } else {
        d
    }
}

/// 点到环面的精确距离（截面形式）：截面 (ρ, z) 上到圆心 (major, 0)、半径 minor 的圆。
pub fn sdf_torus_section(major: f64, minor: f64, rho: f64, z: f64) -> f64 {
    let dr = rho - major;
    (dr * dr + z * z).sqrt() - minor
}

/// 点到锥的精确距离（世界参数：中心 `c`、轴向 `axis`（单位）、半径 `r`、高 `h`）。
pub fn sdf_cone(c: [f64; 3], axis: [f64; 3], r: f64, h: f64, p: [f64; 3]) -> f64 {
    let q = vec3_sub(p, c);
    let z = vec3_dot(q, axis);
    let rad = vec3_sub(q, [axis[0] * z, axis[1] * z, axis[2] * z]);
    sdf_cone_section(r, h, vec3_dot(rad, rad).sqrt(), z)
}

/// 点到环面的精确距离（世界参数；整环面）。
pub fn sdf_torus(c: [f64; 3], axis: [f64; 3], major: f64, minor: f64, p: [f64; 3]) -> f64 {
    let q = vec3_sub(p, c);
    let z = vec3_dot(q, axis);
    let rad = vec3_sub(q, [axis[0] * z, axis[1] * z, axis[2] * z]);
    sdf_torus_section(major, minor, vec3_dot(rad, rad).sqrt(), z)
}

/// 2D 点–线段距离。
pub fn dist_seg(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
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

fn closest_on_seg(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let ab = (b.0 - a.0, b.1 - a.1);
    let ap = (p.0 - a.0, p.1 - a.1);
    let len2 = ab.0 * ab.0 + ab.1 * ab.1;
    let t = if len2 > 0.0 {
        ((ap.0 * ab.0 + ap.1 * ab.1) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (a.0 + t * ab.0, a.1 + t * ab.1)
}

/// 椭球最近点（外部精确：单调对分；内部 → `None`）。`a` 是已减去中心、
/// 投到椭球主轴上的坐标（未除半径）。返回 (距离, 同坐标系下的最近点)。
pub fn ellipsoid_closest_local(radii: [f64; 3], a: [f64; 3]) -> Option<(f64, [f64; 3])> {
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
    let mut out = [0.0; 3];
    let mut d2 = 0.0;
    for i in 0..3 {
        let x = radii[i] * radii[i] * a[i] / (t + radii[i] * radii[i]);
        d2 += (x - a[i]) * (x - a[i]);
        out[i] = x;
    }
    Some((d2.sqrt(), out))
}

/// 椭球最近点（世界参数：中心 `c`、主轴 `axes`、半径 `radii`）。
pub fn ellipsoid_closest(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    radii: [f64; 3],
    p: [f64; 3],
) -> Option<(f64, [f64; 3])> {
    let q = vec3_sub(p, c);
    let a = [
        vec3_dot(q, axes[0]),
        vec3_dot(q, axes[1]),
        vec3_dot(q, axes[2]),
    ];
    let (d, x) = ellipsoid_closest_local(radii, a)?;
    let mut out = c;
    for i in 0..3 {
        out = crate::vec3_add(out, crate::vec3_scale(axes[i], x[i]));
    }
    Some((d, out))
}

/// 球面上离 `p` 最近的点。
pub fn closest_point_sphere(c: [f64; 3], r: f64, p: [f64; 3]) -> [f64; 3] {
    let d = vec3_sub(p, c);
    let n = vec3_dot(d, d).sqrt();
    if n < 1e-12 {
        return crate::vec3_add(c, [r, 0.0, 0.0]);
    }
    crate::vec3_add(c, crate::vec3_scale(d, r / n))
}

/// 平面（`n` 单位、`d`）上离 `p` 最近的点。
pub fn closest_point_plane(n: [f64; 3], d: f64, p: [f64; 3]) -> [f64; 3] {
    vec3_sub(p, crate::vec3_scale(n, vec3_dot(n, p) - d))
}

/// 盒表面上离 `p` 最近的点（钳到盒面）。
pub fn closest_point_box(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    half: [f64; 3],
    p: [f64; 3],
) -> [f64; 3] {
    let q = vec3_sub(p, c);
    let mut out = c;
    for i in 0..3 {
        let x = vec3_dot(q, axes[i]).clamp(-half[i], half[i]);
        out = crate::vec3_add(out, crate::vec3_scale(axes[i], x));
    }
    out
}

/// 方向 `v`（单位）上的支撑点（该方向最远点）。
pub fn support_point_sphere(c: [f64; 3], r: f64, v: [f64; 3]) -> [f64; 3] {
    crate::vec3_add(c, crate::vec3_scale(v, r))
}

pub fn support_point_box(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    half: [f64; 3],
    v: [f64; 3],
) -> [f64; 3] {
    let mut p = c;
    for i in 0..3 {
        let s = if vec3_dot(v, axes[i]) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        p = crate::vec3_add(p, crate::vec3_scale(axes[i], s * half[i]));
    }
    p
}

/// 支撑区间 `[lo, hi]`（`n` 单位方向上 `n·x` 的最小/最大）。
pub fn support_range_sphere(c: [f64; 3], r: f64, n: [f64; 3]) -> (f64, f64) {
    (vec3_dot(n, c) - r, vec3_dot(n, c) + r)
}

pub fn support_range_box(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    half: [f64; 3],
    n: [f64; 3],
) -> (f64, f64) {
    let e: f64 = (0..3).map(|i| vec3_dot(n, axes[i]).abs() * half[i]).sum();
    (vec3_dot(n, c) - e, vec3_dot(n, c) + e)
}

pub fn support_range_cylinder(
    q: [f64; 3],
    u: [f64; 3],
    r: f64,
    half: f64,
    n: [f64; 3],
) -> Option<(f64, f64)> {
    if half <= 0.0 {
        return None; // 无限长柱没有有限支撑区间
    }
    let nu = vec3_dot(n, u);
    let e = half * nu.abs() + r * (1.0 - nu * nu).max(0.0).sqrt();
    Some((vec3_dot(n, q) - e, vec3_dot(n, q) + e))
}

pub fn support_range_cone(c: [f64; 3], axis: [f64; 3], r: f64, h: f64, n: [f64; 3]) -> (f64, f64) {
    let apex = crate::vec3_add(c, crate::vec3_scale(axis, h / 2.0));
    let bc = crate::vec3_add(c, crate::vec3_scale(axis, -h / 2.0));
    let s = (1.0 - vec3_dot(n, axis) * vec3_dot(n, axis))
        .max(0.0)
        .sqrt();
    let na = vec3_dot(n, apex);
    let nb = vec3_dot(n, bc);
    (na.min(nb - r * s), na.max(nb + r * s))
}

pub fn support_range_torus(
    c: [f64; 3],
    axis: [f64; 3],
    major: f64,
    minor: f64,
    n: [f64; 3],
) -> (f64, f64) {
    let na = vec3_dot(n, axis);
    let e = major * (1.0 - na * na).max(0.0).sqrt() + minor;
    (vec3_dot(n, c) - e, vec3_dot(n, c) + e)
}

pub fn support_range_ellipsoid(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    radii: [f64; 3],
    n: [f64; 3],
) -> (f64, f64) {
    let e: f64 = (0..3)
        .map(|i| {
            let v = vec3_dot(n, axes[i]) * radii[i];
            v * v
        })
        .sum::<f64>()
        .sqrt();
    (vec3_dot(n, c) - e, vec3_dot(n, c) + e)
}

pub fn closest_point_cylinder(
    q: [f64; 3],
    u: [f64; 3],
    r: f64,
    half: f64,
    p: [f64; 3],
) -> [f64; 3] {
    let w = vec3_sub(p, q);
    let s = vec3_dot(w, u);
    let rad = vec3_sub(w, [u[0] * s, u[1] * s, u[2] * s]);
    let rho = vec3_dot(rad, rad).sqrt();
    let sc = if half <= 0.0 { s } else { s.clamp(-half, half) };
    let rc = if rho > r { r } else { rho };
    let dir = if rho < 1e-12 {
        [1.0, 0.0, 0.0]
    } else {
        crate::vec3_scale(rad, 1.0 / rho)
    };
    crate::vec3_add(
        q,
        crate::vec3_add(crate::vec3_scale(u, sc), crate::vec3_scale(dir, rc)),
    )
}

pub fn closest_point_cone(c: [f64; 3], axis: [f64; 3], r: f64, h: f64, p: [f64; 3]) -> [f64; 3] {
    let q = vec3_sub(p, c);
    let z = vec3_dot(q, axis);
    let rad = vec3_sub(q, [axis[0] * z, axis[1] * z, axis[2] * z]);
    let rho = vec3_dot(rad, rad).sqrt();
    let apex = (0.0, h / 2.0);
    let base = (r, -h / 2.0);
    let axbot = (0.0, -h / 2.0);
    let q1 = closest_on_seg((rho, z), apex, base);
    let q2 = closest_on_seg((rho, z), base, axbot);
    let d1 = dist_seg((rho, z), apex, base);
    let d2 = dist_seg((rho, z), base, axbot);
    let (qr, qz) = if d1 <= d2 { q1 } else { q2 };
    let dir = if rho < 1e-12 {
        [1.0, 0.0, 0.0]
    } else {
        crate::vec3_scale(rad, 1.0 / rho)
    };
    crate::vec3_add(
        c,
        crate::vec3_add(crate::vec3_scale(axis, qz), crate::vec3_scale(dir, qr)),
    )
}

pub fn closest_point_torus(
    c: [f64; 3],
    axis: [f64; 3],
    major: f64,
    minor: f64,
    p: [f64; 3],
) -> [f64; 3] {
    let q = vec3_sub(p, c);
    let z = vec3_dot(q, axis);
    let rad = vec3_sub(q, [axis[0] * z, axis[1] * z, axis[2] * z]);
    let rho = vec3_dot(rad, rad).sqrt();
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
        crate::vec3_scale(rad, 1.0 / rho)
    };
    crate::vec3_add(
        c,
        crate::vec3_add(crate::vec3_scale(axis, qz), crate::vec3_scale(dir, qr)),
    )
}

pub fn support_point_cylinder(
    q: [f64; 3],
    u: [f64; 3],
    r: f64,
    half: f64,
    v: [f64; 3],
) -> Option<[f64; 3]> {
    if half <= 0.0 {
        return None;
    }
    let vu = vec3_dot(v, u);
    let cap = if vu >= 0.0 { half } else { -half };
    let perp = vec3_sub(v, [u[0] * vu, u[1] * vu, u[2] * vu]);
    let pl = vec3_dot(perp, perp).sqrt();
    let rim = if pl < 1e-12 {
        [0.0; 3]
    } else {
        crate::vec3_scale(perp, r / pl)
    };
    Some(crate::vec3_add(
        q,
        crate::vec3_add(crate::vec3_scale(u, cap), rim),
    ))
}

pub fn support_point_cone(c: [f64; 3], axis: [f64; 3], r: f64, h: f64, v: [f64; 3]) -> [f64; 3] {
    let apex = crate::vec3_add(c, crate::vec3_scale(axis, h / 2.0));
    let bc = crate::vec3_add(c, crate::vec3_scale(axis, -h / 2.0));
    let perp = vec3_sub(
        v,
        [
            axis[0] * vec3_dot(v, axis),
            axis[1] * vec3_dot(v, axis),
            axis[2] * vec3_dot(v, axis),
        ],
    );
    let pl = vec3_dot(perp, perp).sqrt();
    let rim = if pl < 1e-12 {
        bc
    } else {
        crate::vec3_add(bc, crate::vec3_scale(perp, r / pl))
    };
    if vec3_dot(v, apex) >= vec3_dot(v, rim) {
        apex
    } else {
        rim
    }
}

pub fn support_point_torus(
    c: [f64; 3],
    axis: [f64; 3],
    major: f64,
    minor: f64,
    v: [f64; 3],
) -> [f64; 3] {
    let va = vec3_dot(v, axis);
    let perp = vec3_sub(v, [axis[0] * va, axis[1] * va, axis[2] * va]);
    let pl = vec3_dot(perp, perp).sqrt();
    let m = if pl < 1e-12 {
        c
    } else {
        crate::vec3_add(c, crate::vec3_scale(perp, major / pl))
    };
    crate::vec3_add(m, crate::vec3_scale(v, minor))
}

pub fn support_point_ellipsoid(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    radii: [f64; 3],
    v: [f64; 3],
) -> Option<[f64; 3]> {
    let vi = [
        vec3_dot(v, axes[0]),
        vec3_dot(v, axes[1]),
        vec3_dot(v, axes[2]),
    ];
    let s: f64 = (0..3)
        .map(|i| (vi[i] * radii[i]) * (vi[i] * radii[i]))
        .sum::<f64>()
        .sqrt();
    if s < 1e-12 {
        return None;
    }
    let mut p = c;
    for i in 0..3 {
        p = crate::vec3_add(
            p,
            crate::vec3_scale(axes[i], radii[i] * radii[i] * vi[i] / s),
        );
    }
    Some(p)
}

/// 椭球表面上离 `p` 最近的点（内部 → `None`）。
pub fn closest_point_ellipsoid(
    c: [f64; 3],
    axes: [[f64; 3]; 3],
    radii: [f64; 3],
    p: [f64; 3],
) -> Option<[f64; 3]> {
    ellipsoid_closest(c, axes, radii, p).map(|x| x.1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AffineGeometry, CsgGeometry, CsgParams, Multivector};
    use std::f64::consts::PI;

    fn sphere_params() -> GeometryParams {
        Geometry::SphereGeometry(SphereGeometry::new(2.0)).identity_params()
    }

    fn plane_params() -> GeometryParams {
        Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 0.0, 1.0], 2.0)).identity_params()
    }

    /// CGA 锚点：blade 图元的 `field` 与它的 GA 关联谓词恒等（差固定因子/符号）。
    /// 定义锚在谓词上，实现才能用算术形式而不失身份。
    #[test]
    fn blade_predicates_agree_with_field() {
        // 球：X·ip(S) = −(|p−c|²−r²)/2 ⇒ field = −2·ip
        let gp = sphere_params();
        let blade = Multivector::sphere([0.0, 0.0, 0.0], 2.0);
        for p in [
            [0.0, 0.0, 0.0],
            [1.0, 0.5, -0.25],
            [2.0, 0.0, 0.0],
            [-3.0, 1.0, 2.0],
        ] {
            let ip = Multivector::point(p[0], p[1], p[2])
                .ip(&blade)
                .scalar_part();
            let f = gp.field(p).unwrap();
            assert!(
                (f + 2.0 * ip).abs() < 1e-9,
                "sphere field {f} vs −2·ip {}",
                -2.0 * ip
            );
        }
        // 平面：X·ip(P) = 有符号距离（半空间内部为负，n 单位）⇒ field = ip
        let gp = plane_params();
        let blade = Multivector::plane([0.0, 0.0, 1.0], 2.0);
        for p in [[0.0, 0.0, 0.0], [1.0, 1.0, 5.0], [0.0, -4.0, 2.0]] {
            let ip = Multivector::point(p[0], p[1], p[2])
                .ip(&blade)
                .scalar_part();
            let f = gp.field(p).unwrap();
            assert!((f - ip).abs() < 1e-9, "plane field {f} vs ip {ip}");
        }
    }

    #[test]
    fn sphere_field_and_membership_and_distance() {
        let gp = sphere_params();
        assert_eq!(gp.field([0.0, 0.0, 0.0]), Some(-4.0)); // |p−c|²−r²
        assert_eq!(gp.field([2.0, 0.0, 0.0]), Some(0.0)); // 面上
        assert_eq!(gp.field([3.0, 0.0, 0.0]), Some(5.0)); // 9−4
        assert_eq!(gp.contains([3.0, 0.0, 0.0]), Membership::Outside);
        assert_eq!(gp.contains([2.0, 0.0, 0.0]), Membership::Inside); // 表面算内部
        assert_eq!(gp.distance([3.0, 0.0, 0.0]), Some(1.0));
        assert_eq!(gp.distance([0.0, 0.0, 0.0]), Some(-2.0));
    }

    #[test]
    fn plane_field_normalization_independent() {
        // 未归一化法向：field 的符号不变，distance 归一化。
        let gp = GeometryParams::PlaneParams(PlaneParams {
            n: [0.0, 0.0, 4.0],
            d: 2.0,
        });
        assert_eq!(gp.field([0.0, 0.0, 1.0]), Some(2.0)); // 4·1 − 2
        assert_eq!(gp.distance([0.0, 0.0, 1.5]), Some(1.0)); // (6−2)/4
        assert_eq!(gp.contains([0.0, 0.0, 0.0]), Membership::Inside);
    }

    #[test]
    fn box_cone_torus_cylinder_closed_forms() {
        // 盒 2×2×2：field = max|p_i| − 1（Chebyshev），distance = 精确欧氏。
        let gp = Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0)).identity_params();
        assert_eq!(gp.field([0.5, 0.0, 0.0]), Some(-0.5));
        assert_eq!(gp.field([1.5, 0.0, 0.0]), Some(0.5));
        assert_eq!(gp.distance([1.5, 0.0, 0.0]), Some(0.5));
        assert_eq!(gp.distance([1.5, 2.5, 0.5]), Some((0.25 + 2.25f64).sqrt())); // 角区
        assert_eq!(gp.contains([1.0, 1.0, 1.0]), Membership::Inside);

        // 柱 r=1 h=2（半径 1、高 2 ⇒ half=1）
        let gp = Geometry::CylinderGeometry(CylinderGeometry::new(1.0, 2.0)).identity_params();
        assert_eq!(gp.field([0.0, 0.0, 0.0]), Some(-1.0));
        assert_eq!(gp.distance([2.0, 0.0, 0.0]), Some(1.0));
        assert_eq!(gp.distance([0.0, 0.0, 3.0]), Some(2.0));
        assert_eq!(gp.distance([2.0, 0.0, 3.0]), Some((1.0f64 + 4.0).sqrt()));
        // half ≤ 0 = 无限长：轴向无界
        let inf = Geometry::CylinderGeometry(CylinderGeometry::new(1.0, -1.0)).identity_params();
        assert_eq!(inf.distance([0.0, 0.0, 100.0]), Some(-1.0));
        assert_eq!(inf.bounds(), None);

        // 锥 r=1 h=2：顶点 z=+1、底 z=−1、底半径 1
        let gp = Geometry::ConeGeometry(ConeGeometry::new(1.0, 2.0)).identity_params();
        assert_eq!(gp.field([0.0, 0.0, 1.0]), Some(0.0)); // 顶点在面上
        assert_eq!(gp.field([0.0, 0.0, 0.0]), Some(-0.25)); // s=−0.5 → s²−2s² = −0.25
        assert_eq!(gp.contains([0.0, 0.0, -0.5]), Membership::Inside);
        assert_eq!(gp.distance([0.5, 0.0, 0.0]), Some(0.0)); // 侧面上
        assert_eq!(gp.distance([0.0, 0.0, -3.0]), Some(2.0)); // 底面下方

        // 环面 R=2 r=0.5：面上/管心/外部
        let gp = Geometry::TorusGeometry(TorusGeometry::new(2.0, 0.5)).identity_params();
        assert_eq!(gp.field([2.5, 0.0, 0.0]), Some(0.0));
        assert!((gp.distance([2.7, 0.0, 0.0]).unwrap() - 0.2).abs() < 1e-12);
        assert_eq!(gp.distance([2.0, 0.0, 0.0]), Some(-0.5)); // 管内最深处
        assert_eq!(gp.contains([2.0, 0.0, 0.0]), Membership::Inside);
        assert_eq!(gp.contains([0.0, 0.0, 0.0]), Membership::Outside);
    }

    #[test]
    fn ellipsoid_field_and_exact_distance() {
        // 椭球 radii (1,2,4)：field 是规范系里的 |l|²−1。
        let gp =
            Geometry::EllipsoidGeometry(EllipsoidGeometry::new(1.0, 2.0, 4.0)).identity_params();
        assert_eq!(gp.field([1.0, 0.0, 0.0]), Some(0.0));
        assert_eq!(gp.field([2.0, 0.0, 0.0]), Some(3.0)); // 4−1
        assert_eq!(gp.contains([0.5, 0.0, 0.0]), Membership::Inside);
        // 沿长轴外部：最近点在 (1,0,0)，距离 = x−1
        let d = gp.distance([3.0, 0.0, 0.0]).unwrap();
        assert!((d - 2.0).abs() < 1e-9, "{d}");
        // 内部 → None（不假装）
        assert_eq!(gp.distance([0.0, 0.0, 0.0]), None);
    }

    #[test]
    fn cyclide_and_circle_are_unknown_but_cyclide_has_field() {
        let cy = Geometry::CyclideGeometry(CyclideGeometry::new(1.0, 0.5, 0.25, [0.0; 3]))
            .identity_params();
        // 符号场存在（网格化要用），但内外归属读作 Unknown（自交语义退化）。
        assert!(cy.field([0.0, 0.0, 0.0]).is_some());
        assert_eq!(cy.contains([0.0, 0.0, 0.0]), Membership::Unknown);
        let ci = Geometry::CircleGeometry(CircleGeometry::new(1.0)).identity_params();
        assert_eq!(ci.field([0.0, 0.0, 0.0]), None);
        assert_eq!(ci.contains([0.0, 0.0, 0.0]), Membership::Unknown);
        // 部分环面（弧管）：场有，归属 Unknown。
        let tube =
            Geometry::TorusGeometry(TorusGeometry::tube(2.0, 0.5, PI / 2.0)).identity_params();
        assert!(tube.field([2.5, 0.0, 0.0]).is_some());
        assert_eq!(tube.contains([2.5, 0.0, 0.0]), Membership::Unknown);
    }

    #[test]
    fn csg_membership_is_three_valued() {
        let bx = || Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0));
        let hole = || Geometry::SphereGeometry(SphereGeometry::new(1.0));
        let diff = Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Difference, vec![bx(), hole()]))
            .identity_params();
        assert_eq!(diff.contains([0.9, 0.0, 0.0]), Membership::Outside); // 挖掉的孔
        assert_eq!(diff.contains([0.95, 0.0, 0.0]), Membership::Outside);
        assert_eq!(diff.contains([0.0, 1.9, 0.0]), Membership::Outside); // 盒外
                                                                         // 盒内且孔外 → 内部；盒与孔的表面都算「内部」（接触语义 ⇒ 差集里成为外部）
        assert_eq!(diff.contains([0.9, 0.9, 0.0]), Membership::Inside);
        assert_eq!(diff.contains([0.0, 1.0, 0.0]), Membership::Outside);
        // 交集/并集
        let inter =
            Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Intersection, vec![bx(), hole()]))
                .identity_params();
        assert_eq!(inter.contains([0.0, 0.0, 0.0]), Membership::Inside);
        assert_eq!(inter.contains([0.0, 1.5, 0.0]), Membership::Outside);
        let uni = Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Union, vec![bx(), hole()]))
            .identity_params();
        assert_eq!(uni.contains([0.0, 1.5, 0.0]), Membership::Outside);
    }

    #[test]
    fn csg_field_uses_signed_folds() {
        let child = |v: f64| {
            GeometryParams::SphereParams(SphereParams {
                c: [0.0, 0.0, 0.0],
                r: v,
                axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            })
        };
        let uni = GeometryParams::CsgParams(CsgParams {
            op: CsgOp::Union,
            children: vec![child(1.0), child(2.0)],
        });
        assert_eq!(uni.field([0.0, 0.0, 0.0]), Some(-4.0)); // min(−1, −4)
        let inter = GeometryParams::CsgParams(CsgParams {
            op: CsgOp::Intersection,
            children: vec![child(1.0), child(2.0)],
        });
        assert_eq!(inter.field([0.0, 0.0, 0.0]), Some(-1.0)); // max(−1, −4)
        let diff = GeometryParams::CsgParams(CsgParams {
            op: CsgOp::Difference,
            children: vec![child(2.0), child(1.0)],
        });
        assert_eq!(diff.field([0.0, 0.0, 0.0]), Some(1.0)); // max(−4, +1)
    }

    #[test]
    fn affine_params_fold_the_transform() {
        // 仿射包装（缩放 2 倍）的球：场上等价于半径 2 的球。
        let inner = Geometry::SphereGeometry(SphereGeometry::new(1.0));
        let ag = Geometry::AffineGeometry(AffineGeometry::new(
            inner,
            [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]],
        ))
        .identity_params();
        assert_eq!(ag.field([2.0, 0.0, 0.0]), Some(0.0));
        assert_eq!(ag.contains([1.0, 0.0, 0.0]), Membership::Inside);
        assert_eq!(ag.contains([3.0, 0.0, 0.0]), Membership::Outside);
        assert_eq!(ag.bounds(), Some([[-2.0; 3], [2.0; 3]]), "缩放后的球包围盒");
    }

    #[test]
    fn bounds_match_closed_forms() {
        let b = |g: Geometry| g.identity_params().bounds();
        assert_eq!(
            b(Geometry::SphereGeometry(SphereGeometry::new(1.5))),
            Some([[-1.5; 3], [1.5; 3]])
        );
        assert_eq!(
            b(Geometry::PlaneGeometry(PlaneGeometry::new(
                [0.0, 1.0, 0.0],
                0.0
            ))),
            None
        );
        assert_eq!(
            b(Geometry::BoxGeometry(BoxGeometry::new(2.0, 4.0, 6.0))),
            Some([[-1.0, -2.0, -3.0], [1.0, 2.0, 3.0]])
        );
        assert_eq!(
            b(Geometry::ConeGeometry(ConeGeometry::new(1.0, 4.0))),
            Some([[-1.0, -1.0, -2.0], [1.0, 1.0, 2.0]])
        );
        assert_eq!(
            b(Geometry::CylinderGeometry(CylinderGeometry::new(1.0, 4.0))),
            Some([[-1.0, -1.0, -2.0], [1.0, 1.0, 2.0]]),
            "柱的紧包围盒：径向 ±r、轴向 ±h"
        );
        assert_eq!(
            b(Geometry::CylinderGeometry(CylinderGeometry::new(2.0, -1.0))),
            None,
            "无限长柱无界"
        );
        assert_eq!(
            b(Geometry::TorusGeometry(TorusGeometry::new(2.0, 0.5))),
            Some([[-2.5, -2.5, -0.5], [2.5, 2.5, 0.5]])
        );
        assert_eq!(
            b(Geometry::EllipsoidGeometry(EllipsoidGeometry::new(
                1.0, 2.0, 4.0
            ))),
            Some([[-1.0, -2.0, -4.0], [1.0, 2.0, 4.0]])
        );
        assert_eq!(
            b(Geometry::CircleGeometry(CircleGeometry::new(1.0))),
            None,
            "圆片没有体积包围盒"
        );
    }

    #[test]
    fn support_and_closest_agreement() {
        // 球/盒/柱的支撑点与最近点闭式
        let s = support_point_sphere([1.0, 0.0, 0.0], 2.0, [1.0, 0.0, 0.0]);
        assert_eq!(s, [3.0, 0.0, 0.0]);
        let bx = support_point_box(
            [0.0; 3],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [1.0, 2.0, 3.0],
            [1.0, 1.0, 1.0],
        );
        assert_eq!(bx, [1.0, 2.0, 3.0]);
        // 盒支撑区间：n=x 时 [−1, 1]
        let (lo, hi) = support_range_box(
            [0.0; 3],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [1.0, 2.0, 3.0],
            [1.0, 0.0, 0.0],
        );
        assert_eq!((lo, hi), (-1.0, 1.0));
        // 无限长柱没有有限支撑区间
        assert_eq!(
            support_range_cylinder([0.0; 3], [0.0, 0.0, 1.0], 1.0, -1.0, [1.0, 0.0, 0.0]),
            None
        );
        // 柱最近点：轴上点 → 半径方向
        let c = closest_point_cylinder([0.0; 3], [0.0, 0.0, 1.0], 1.0, 2.0, [3.0, 0.0, 0.5]);
        assert_eq!(c, [1.0, 0.0, 0.5]);
        // 环面最近点：管内点 → 表面
        let t = closest_point_torus([0.0; 3], [0.0, 0.0, 1.0], 2.0, 0.5, [2.0, 0.0, 0.0]);
        assert_eq!(t, [2.5, 0.0, 0.0]);
    }
}
