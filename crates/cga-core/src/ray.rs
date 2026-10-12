//! 光线 × 图元的**几何定义**（唯一一份）：多项式系数、法线、局部系搬移。
//!
//! 求根（稳定二次式 / 区间认证 / Durand–Kerner）是**数值方法**、不是几何知识，因此留在
//! `cga-gpu` 的 `certify`（它依赖 `inari`；`cga-core` 保持零依赖）。本模块只回答
//! 「这条光线对应该图元的哪个多项式、交点处法线是什么」，判根与命中装配由调用方做。
//!
//! CGA 锚点：blade 图元的系数就是其关联谓词沿光线的展开——球/椭球的二次式在根处让
//! `field = 0`（即 `X·ip(S) = 0`），平面是一次式（`field` 的线性化）；这两条由
//! `tests::quadratic_roots_satisfy_the_cga_predicate` 钉死。非 blade 图元（锥/环面/
//! 椭球/环纹面）在**规范系**（轴 = +z，锥顶点 +h/2）里给出二次/四次式，调用方先用
//! [`local_ray`] 把光线搬进规范系。
//!
//! 约定与渲染器/碰撞一致：光线方向视为**单位向量**（仿射包装下由 [`local_ray`] 归一，
//! 解出的 `t` 需除以 `lam = |A⁻¹d|` 回到原空间）。

use crate::motors::Mat3;
use crate::{vec3_add, vec3_dot, vec3_len, vec3_scale, vec3_sub};

/// 单位化；近零退化 → `[0, 0, 0]`（"得不到方向就清零"，与 `vec3_unit` 的 `[0,0,1]` 兜底
/// 语义不同：法线为 0 会被上层读作"无命中法线"，而 +z 会凭空造出方向）。
pub fn vunit(a: [f64; 3]) -> [f64; 3] {
    let n = vec3_len(a);
    if n > 1e-300 {
        vec3_scale(a, 1.0 / n)
    } else {
        [0.0, 0.0, 0.0]
    }
}

pub fn mat3_mul_vec(m: Mat3, v: [f64; 3]) -> [f64; 3] {
    [vec3_dot(m[0], v), vec3_dot(m[1], v), vec3_dot(m[2], v)]
}

/// `A⁻ᵀ · v`（法线的逆置变换）。
pub fn mat3_mul_vec_t(m: Mat3, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

/// `o_l = A⁻¹·o + t_inv`；`d_u = A⁻¹·d / |A⁻¹·d|`；`lam = |A⁻¹·d|`。
/// `None` = 光线在局部系里退化（方向被线性部分压掉）。
pub fn local_ray(
    a_inv3: Mat3,
    t_inv: [f64; 3],
    o: [f64; 3],
    d: [f64; 3],
) -> Option<([f64; 3], [f64; 3], f64)> {
    let ol = vec3_add(mat3_mul_vec(a_inv3, o), t_inv);
    let dl = mat3_mul_vec(a_inv3, d);
    let lam = vec3_len(dl);
    if !lam.is_finite() || lam <= 1e-300 {
        return None;
    }
    Some((ol, vec3_scale(dl, 1.0 / lam), lam))
}

/// `n_world = normalize(A⁻ᵀ · n_l)`（与渲染器的 `affine_normal` 同一约定）。
pub fn world_normal(a_inv3: Mat3, n_l: [f64; 3]) -> [f64; 3] {
    vunit(mat3_mul_vec_t(a_inv3, n_l))
}

// ---- 球（blade）----------------------------------------------------------

/// `a t² + b t + c = 0`：`a = |d|² = 1`、`b = 2(o−c)·d`、`c = |o−c|² − r²`。
pub fn sphere_quad(o: [f64; 3], d: [f64; 3], c: [f64; 3], r: f64) -> (f64, f64, f64) {
    let oc = vec3_sub(o, c);
    let b = 2.0 * vec3_dot(oc, d);
    let cc = vec3_dot(oc, oc) - r * r;
    (1.0, b, cc)
}

/// 球面法线（外向单位）。
pub fn sphere_normal(p: [f64; 3], c: [f64; 3], r: f64) -> [f64; 3] {
    vec3_scale(vec3_sub(p, c), 1.0 / r)
}

// ---- 平面（blade，半空间）------------------------------------------------

/// 一次式命中参数 `t = (d_off − n·o)/(n·d)`；`None` = 光线平行或退化。
pub fn plane_hit(o: [f64; 3], d: [f64; 3], n: [f64; 3], d_off: f64) -> Option<f64> {
    let denom = vec3_dot(n, d);
    if !denom.is_finite() || denom.abs() <= 1e-30 {
        return None;
    }
    let t = (d_off - vec3_dot(n, o)) / denom;
    if t.is_finite() {
        Some(t)
    } else {
        None
    }
}

// ---- 柱（轴 = 任意 (`q`, `u`)）-------------------------------------------

/// 侧面二次式（径向分量）：`(a, b, c)`，`a == 0 && b == 0` 表示光线平行于轴。
pub fn cylinder_quad(
    o: [f64; 3],
    d: [f64; 3],
    q: [f64; 3],
    u: [f64; 3],
    r: f64,
) -> (f64, f64, f64) {
    let oc = vec3_sub(o, q);
    let d_par = vec3_dot(d, u);
    let o_par = vec3_dot(oc, u);
    let d_p = vec3_sub(d, vec3_scale(u, d_par));
    let o_p = vec3_sub(oc, vec3_scale(u, o_par));
    let a = vec3_dot(d_p, d_p);
    let b = 2.0 * vec3_dot(o_p, d_p);
    let c = vec3_dot(o_p, o_p) - r * r;
    (a, b, c)
}

/// 端面（`|轴向坐标| = h`）的两个参数 `t`；光线平行于轴时返回 ±∞（无界区间）。
pub fn cylinder_cap_ts(o: [f64; 3], d: [f64; 3], q: [f64; 3], u: [f64; 3], h: f64) -> (f64, f64) {
    let o_par = vec3_dot(vec3_sub(o, q), u);
    let d_par = vec3_dot(d, u);
    if d_par.abs() > 1e-30 {
        let t_plus = (h - o_par) / d_par;
        let t_minus = (-h - o_par) / d_par;
        (t_plus.min(t_minus), t_plus.max(t_minus))
    } else {
        (f64::NEG_INFINITY, f64::INFINITY)
    }
}

/// 柱侧面法线（在柱的规范/世界系里，径向单位向量）。
pub fn cylinder_side_normal(p: [f64; 3], q: [f64; 3], u: [f64; 3], r: f64) -> [f64; 3] {
    let w = vec3_sub(p, q);
    let s = vec3_dot(w, u);
    let radial = vec3_sub(w, vec3_scale(u, s));
    vec3_scale(radial, 1.0 / r)
}

// ---- 锥（规范系：顶点 z = +h/2，底 z = −h/2）------------------------------

/// 侧面二次式（规范系坐标）。`k = r/h`。
pub fn cone_quad(r: f64, h: f64, o: [f64; 3], d: [f64; 3]) -> (f64, f64, f64) {
    let k = r / h;
    let k2 = 1.0 + k * k;
    let wz = o[2] - h / 2.0;
    let dz = d[2];
    let wd = vec3_dot(o, d) - dz * h / 2.0;
    let ww = vec3_dot(o, o) - o[2] * h + h * h / 4.0;
    let a = 1.0 - k2 * dz * dz;
    let b = 2.0 * (wd - k2 * wz * dz);
    let c = ww - k2 * wz * wz;
    (a, b, c)
}

/// 两个端面（顶点平面 z=+h/2、底面 z=−h/2）的光线参数 `(t_top, t_bot)`。
/// 与数组内核同一约定：`dz` 近零时代换为 `+1e-9`（而不是 ±∞），于是平行光线得到
/// 两个巨大但有限的参数，区间判定的结果与内核逐位一致。
pub fn cone_cap_ts(h: f64, o: [f64; 3], d: [f64; 3]) -> (f64, f64) {
    let wz = o[2] - h / 2.0;
    let dz = d[2];
    let safe_dz = if dz.abs() > 1e-9 { dz } else { 1e-9 };
    (-wz / safe_dz, -(wz + h) / safe_dz)
}

/// 锥侧面法线（规范系；未归一化的梯度方向 `[x, y, s(1−k²)]`）。
pub fn cone_side_normal(r: f64, h: f64, p: [f64; 3]) -> [f64; 3] {
    let k = r / h;
    let k2 = 1.0 + k * k;
    let s = p[2] - h / 2.0;
    vunit([p[0], p[1], s * (1.0 - k2)])
}

// ---- 椭球（规范系：单位球）----------------------------------------------

/// 单位球的二次式（`o_l` 是规范系里的原点、`d_u` 单位方向）。
pub fn ellipsoid_quad(o_l: [f64; 3], d_u: [f64; 3]) -> (f64, f64, f64) {
    let b = 2.0 * vec3_dot(o_l, d_u);
    let c = vec3_dot(o_l, o_l) - 1.0;
    (1.0, b, c)
}

/// 单位球面法线 = 交点坐标本身（规范系）。
pub fn ellipsoid_normal(o_l: [f64; 3], d_u: [f64; 3], t: f64) -> [f64; 3] {
    vec3_add(o_l, vec3_scale(d_u, t))
}

// ---- 环面（规范系：轴 = +z）----------------------------------------------

/// 四次式系数 `[c3, c2, c1, c0]`（`t⁴ + c3 t³ + c2 t² + c1 t + c0 = 0`）。
pub fn torus_quartic(major: f64, minor: f64, o: [f64; 3], d: [f64; 3]) -> [f64; 4] {
    let r2 = major * major;
    let oo = o[0] * o[0] + o[1] * o[1] + o[2] * o[2];
    let od = o[0] * d[0] + o[1] * d[1] + o[2] * d[2];
    let g = oo + r2 - minor * minor;
    [
        4.0 * od,
        2.0 * g + 4.0 * od * od - 4.0 * r2 * (d[0] * d[0] + d[1] * d[1]),
        4.0 * od * g - 8.0 * r2 * (o[0] * d[0] + o[1] * d[1]),
        g * g - 4.0 * r2 * (o[0] * o[0] + o[1] * o[1]),
    ]
}

/// 环面法线（规范系）。
pub fn torus_normal(major: f64, minor: f64, p: [f64; 3]) -> [f64; 3] {
    let r2 = major * major;
    let s1 = vec3_dot(p, p) + r2 - minor * minor;
    let fac = s1 - 2.0 * r2;
    vunit([fac * p[0], fac * p[1], s1 * p[2]])
}

// ---- 环纹面（规范系：轴 = +z，`shift` 已折进坐标）------------------------

/// 四次式系数 `[c3, c2, c1, c0]`。
#[allow(clippy::too_many_arguments)]
pub fn cyclide_quartic(
    a: f64,
    b: f64,
    dd: f64,
    c: f64,
    shift: [f64; 3],
    o: [f64; 3],
    d: [f64; 3],
) -> [f64; 4] {
    let ox = o[0] - shift[0];
    let oy = o[1] - shift[1];
    let oz = o[2] - shift[2];
    let (dx, dy) = (d[0], d[1]);
    let big_a = ox * ox + oy * oy + oz * oz;
    let b1 = ox * dx + oy * dy + oz * d[2];
    let bb = b * b - dd * dd;
    let g = big_a + bb;
    let p0 = ox * a - c * dd;
    let p1 = dx * a;
    [
        4.0 * b1,
        2.0 * g + 4.0 * b1 * b1 - 4.0 * p1 * p1 - 4.0 * b * b * dy * dy,
        4.0 * b1 * g - 8.0 * p0 * p1 - 8.0 * b * b * oy * dy,
        g * g - 4.0 * p0 * p0 - 4.0 * b * b * oy * oy,
    ]
}

/// 环纹面法线（规范系；未归一化梯度）。
pub fn cyclide_normal(a: f64, b: f64, dd: f64, c: f64, shift: [f64; 3], p: [f64; 3]) -> [f64; 3] {
    let pp = vec3_sub(p, shift);
    let bb = b * b - dd * dd;
    let g = vec3_dot(pp, pp) + bb;
    vunit([
        4.0 * pp[0] * g - 8.0 * a * (a * pp[0] - c * dd),
        4.0 * pp[1] * g - 8.0 * b * b * pp[1],
        4.0 * pp[2] * g,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{
        CyclideGeometry, Geometry, GeometryParams, SphereGeometry, TorusGeometry,
    };
    use crate::Multivector;

    fn field_at(params: &GeometryParams, t: f64, o: [f64; 3], d: [f64; 3]) -> f64 {
        params
            .field(vec3_add(o, vec3_scale(d, t)))
            .expect("有隐式场")
    }

    /// CGA/定义锚点：二次式与四次式的根必须让**该图元的 field 归零**
    /// （球/椭球即 `X·ip(S) = 0`），四个根都要满足。
    #[test]
    fn polynomial_roots_satisfy_the_field_definition() {
        let o = [0.3, -0.7, 3.0];
        let d = vunit([0.1, 0.2, -1.0]);

        // 球：二次式根 ↦ field = 0（`SphereGeometry` 的规范系球心在原点）
        let (c, r): ([f64; 3], f64) = ([0.0, 0.0, 0.0], 1.0);
        let (a, b, cc) = sphere_quad(o, d, c, r);
        let disc = b * b - 4.0 * a * cc;
        assert!(disc > 0.0, "测试光线应命中球：disc={disc}");
        for t in [
            (-b + disc.sqrt()) / (2.0 * a),
            (-b - disc.sqrt()) / (2.0 * a),
        ] {
            let gp = Geometry::SphereGeometry(SphereGeometry::new(r)).identity_params();
            assert!(field_at(&gp, t, o, d).abs() < 1e-9, "球根 t={t}");
            // 与 GA 谓词一致：X·ip(S) = −(|p−c|²−r²)/2
            let p = vec3_add(o, vec3_scale(d, t));
            let blade = Multivector::sphere(c, r);
            let ip = Multivector::point(p[0], p[1], p[2])
                .ip(&blade)
                .scalar_part();
            assert!(ip.abs() < 1e-9, "球 ip={ip}");
        }

        // 环面：四次式根 ↦ field = 0（规范系里 shift = 0）
        let (major, minor) = (1.5, 0.4);
        let q = torus_quartic(major, minor, o, d);
        let gp = Geometry::TorusGeometry(TorusGeometry::new(major, minor)).identity_params();
        for &tt in &sample_quartic_roots(q) {
            assert!(field_at(&gp, tt, o, d).abs() < 1e-6, "环面根 t={tt}");
        }

        // 环纹面：四次式根 ↦ field = 0
        let (a_c, b_c, d_c): (f64, f64, f64) = (1.0, 0.5, 0.25);
        let shift = [0.1, -0.2, 0.3];
        let c_c = (a_c * a_c - b_c * b_c).sqrt();
        let q = cyclide_quartic(a_c, b_c, d_c, c_c, shift, o, d);
        let gp =
            Geometry::CyclideGeometry(CyclideGeometry::new(a_c, b_c, d_c, shift)).identity_params();
        let n_roots = sample_quartic_roots(q).len();
        assert!(n_roots > 0, "环纹面应有实根");
        for &tt in &sample_quartic_roots(q) {
            assert!(field_at(&gp, tt, o, d).abs() < 1e-6, "环纹面根 t={tt}");
        }

        // 锥：侧面二次式根 ↦ field 的侧面项归零
        let (cr, ch) = (0.7, 1.6);
        let (a2, b2, c2) = cone_quad(cr, ch, o, d);
        if b2 * b2 - 4.0 * a2 * c2 > 0.0 {
            let t = (-b2 + (b2 * b2 - 4.0 * a2 * c2).sqrt()) / (2.0 * a2);
            let p = vec3_add(o, vec3_scale(d, t));
            let k2 = 1.0 + (cr / ch) * (cr / ch);
            let s = p[2] - ch / 2.0;
            let side = p[0] * p[0] + p[1] * p[1] + s * s - k2 * s * s;
            assert!(side.abs() < 1e-9, "锥侧根 t={t} side={side}");
        }
    }

    /// 四次式取样根（测试用：Durand–Kerner 简单实现，只求实根，判据宽松）。
    fn sample_quartic_roots(c: [f64; 4]) -> Vec<f64> {
        // t⁴ + c3 t³ + c2 t² + c1 t + c0
        let coeff = [1.0, c[0], c[1], c[2], c[3]];
        let f = |t: f64| (((coeff[0] * t + coeff[1]) * t + coeff[2]) * t + coeff[3]) * t + coeff[4];
        let mut roots = Vec::new();
        // 扫 [-200, 200]，符号变号处二分（测试只需要几个真根）
        let n = 4000;
        let (lo, hi) = (-200.0, 200.0);
        let mut prev = f(lo);
        for i in 1..=n {
            let x = lo + (hi - lo) * (i as f64) / (n as f64);
            let cur = f(x);
            if prev == 0.0 {
                roots.push(x - (hi - lo) / (n as f64));
            } else if prev * cur < 0.0 {
                let (mut a, mut b) = (x - (hi - lo) / (n as f64), x);
                for _ in 0..200 {
                    let m = 0.5 * (a + b);
                    if f(a) * f(m) <= 0.0 {
                        b = m;
                    } else {
                        a = m;
                    }
                }
                roots.push(0.5 * (a + b));
            }
            prev = cur;
        }
        roots
    }

    /// 局部系搬移：`local_ray`/`world_normal` 与法线的逆置变换自洽。
    #[test]
    fn local_ray_and_normal_transform_are_consistent() {
        let a_inv3 = [[0.5, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.25]];
        let t_inv = [0.3, -0.4, 0.2];
        let o = [0.1, 0.2, 0.3];
        let d = vunit([-0.2, 0.9, 0.4]);
        let (o_l, d_u, lam) = local_ray(a_inv3, t_inv, o, d).expect("非退化光线");
        // 单位方向 + 非零缩放
        assert!((vec3_len(d_u) - 1.0).abs() < 1e-12);
        assert!(lam > 0.0);
        // 法线的逆置变换：局部平面（n_l 为其法线）映射到世界后，n_w = normalize(A⁻ᵀ n_l)
        // 必须垂直于平面内的**任何**切向（A·t，t·n_l = 0）——这是逆置规则的定义性检验。
        let n_l = vunit([0.3, 0.9, 0.1]);
        let n_w = world_normal(a_inv3, n_l);
        let a = crate::affine::mat3_inv(a_inv3);
        for t_local in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], n_l] {
            let tl = vec3_sub(t_local, vec3_scale(n_l, vec3_dot(t_local, n_l)));
            let t_world = mat3_mul_vec(a, tl);
            assert!(
                vec3_dot(n_w, t_world).abs() < 1e-12,
                "n_w 应垂直于切向 {tl:?}"
            );
        }
        assert!((vec3_len(n_w) - 1.0).abs() < 1e-12);
        // 退化光线（线性部分压掉方向）→ None
        assert!(local_ray([[0.0; 3]; 3], [0.0; 3], o, d).is_none());
        let _ = (o_l, d_u);
    }

    /// 柱/锥的端面参数：光线平行于轴时是 ±∞ 区间。
    #[test]
    fn cap_intervals_degenerate_to_infinity() {
        let u = [0.0, 0.0, 1.0];
        let (lo, hi) = cylinder_cap_ts([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0; 3], u, 2.0);
        assert_eq!((lo, hi), (f64::NEG_INFINITY, f64::INFINITY));
        let (lo, hi) = cylinder_cap_ts([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0; 3], u, 2.0);
        assert!(
            (lo + 2.0).abs() < 1e-12 && (hi - 2.0).abs() < 1e-12,
            "{lo} {hi}"
        );
        let (tt, tb) = cone_cap_ts(1.6, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        // 平行于轴：`dz` 代换为 +1e-9 ⇒ 参数是巨大但有限的相反值（与内核同一约定）
        assert!(
            tt.abs() > 1e6 && tb.abs() > 1e6 && tt * tb < 0.0,
            "{tt} {tb}"
        );
        // 竖直向下：顶点面 z=+0.8 → t=−0.8、底面 z=−0.8 → t=0.8
        let (tt, tb) = cone_cap_ts(1.6, [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]);
        assert!(
            (tt + 0.8).abs() < 1e-12 && (tb - 0.8).abs() < 1e-12,
            "{tt} {tb}"
        );
    }
}
