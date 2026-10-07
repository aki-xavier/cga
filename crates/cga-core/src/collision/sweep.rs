//! 螺旋扫掠连续碰撞检测（C3，计划 `docs/collision-plan.md`）。
//!
//! 运动学：a 以螺旋速度 `xi = [ω; v]`（角速度 + 线速度，见
//! [`Multivector::velocity`]）从 `wa` 出发：`M(t) = exp(t·ξ)·wa`——螺旋轨迹是
//! **精确**的（不是线性插值近似）。
//!
//! 认证：分离函数 `f(t)` 是 1-Lipschitz 的，`|f'| ≤ vmax`（vmax = 螺旋运动下
//! 物体上点的最大速度，由世界包围盒角点到螺旋轴的距离精确上界）。区间
//! `[t0, t1]` 上 `min(f0,f1) − vmax·Δt > 0` ⇒ 认证无接触；无法认证就 8 分
//! 细分；细分到深度上限仍无法认证（擦边等退化）⇒ `Unknown`。不报假阴性，
//! 也不报假阳性。

use super::*;
use crate::Multivector;

/// 螺旋扫掠 TOI（`xi = [ωx, ωy, ωz, vx, vy, vz]`）。
///
/// 返回 `(hit, toi)`：
/// - `(Yes, Some(0.0))`：初始已接触/穿入；
/// - `(Yes, Some(t))`：首次接触时刻（对分收敛）；
/// - `(No, None)`：认证无接触；
/// - `(Unknown, _)`：几何对不支持 / 无法认证（擦边退化）。
pub fn sweep_toi(
    xi: [f64; 6],
    a: &Geometry,
    wa: [f64; 16],
    b: &Geometry,
    wb: [f64; 16],
    t_max: f64,
) -> (Hit, Option<f64>) {
    if t_max <= 0.0 {
        return (Hit::Unknown, None);
    }
    let Some(vmax) = max_speed(xi, a, wa) else {
        return (Hit::Unknown, None); // 无界几何：无法给速度界
    };
    let w = [xi[0], xi[1], xi[2]];
    let v = [xi[3], xi[4], xi[5]];
    // Multivector::velocity 存的是二重矢量系数（物理角速度 = 2ω），喂一半得到
    // 物理螺旋速度。
    let (wh, vh) = (scale(w, 0.5), scale(v, 0.5));
    let f = |t: f64| -> Option<f64> {
        let m = mat4_mul(Multivector::velocity(wh, vh).exp(t).to_matrix(), wa);
        separation(a, m, b, wb)
    };
    let Some(f0) = f(0.0) else {
        return (Hit::Unknown, None);
    };
    if f0 <= 0.0 {
        return (Hit::Yes, Some(0.0));
    }
    let Some(f1) = f(t_max) else {
        return (Hit::Unknown, None);
    };
    match scan(&f, vmax, 0.0, t_max, f0, f1, 12) {
        Scan::Free => (Hit::No, None),
        Scan::Contact(t) => (Hit::Yes, Some(t)),
        Scan::Uncertain => (Hit::Unknown, None),
    }
}

enum Scan {
    Free,
    Contact(f64),
    Uncertain,
}

/// 自适应区间扫描：Lipschitz 界认证无接触；符号翻转对分求根；深度耗尽 → Uncertain。
fn scan(
    f: &dyn Fn(f64) -> Option<f64>,
    vmax: f64,
    t0: f64,
    t1: f64,
    f0: f64,
    f1: f64,
    depth: u32,
) -> Scan {
    // 认证：区间内 f ≥ min(f0,f1) − vmax·Δt > 0 ⇒ 无接触
    if f0.min(f1) - vmax * (t1 - t0) > 0.0 {
        return Scan::Free;
    }
    if f0 > 0.0 && f1 <= 0.0 {
        return Scan::Contact(bisect(f, t0, t1));
    }
    if f0 <= 0.0 {
        return Scan::Contact(t0);
    }
    if depth == 0 {
        return Scan::Uncertain; // 擦边/退化：认证不了就 Unknown
    }
    // 8 分细分，按时间序找第一个接触
    let n = 8;
    let dt = (t1 - t0) / n as f64;
    let mut ti = t0;
    let mut fi = f0;
    for i in 1..=n {
        let tj = if i == n { t1 } else { t0 + dt * i as f64 };
        let Some(fj) = (if i == n { Some(f1) } else { f(tj) }) else {
            return Scan::Uncertain; // 几何对在某些姿态不支持
        };
        match scan(f, vmax, ti, tj, fi, fj, depth - 1) {
            Scan::Free => {}
            s => return s,
        }
        ti = tj;
        fi = fj;
    }
    Scan::Free
}

/// 符号翻转区间 [t0, t1]（f(t0) > 0, f(t1) ≤ 0）对分到 1e-12 相对精度。
fn bisect(f: &dyn Fn(f64) -> Option<f64>, t0: f64, t1: f64) -> f64 {
    let mut lo = t0;
    let mut hi = t1;
    let tol = 1e-12 * (t1 - t0).max(1.0);
    while hi - lo > tol {
        let mid = 0.5 * (lo + hi);
        match f(mid) {
            Some(v) if v > 0.0 => lo = mid,
            _ => hi = mid, // None（姿态上几何不支持）按接触侧收
        }
    }
    0.5 * (lo + hi)
}

/// 螺旋运动下物体上点的最大速度上界：`|ṗ| ≤ |h·ω| + |ω|·dist(p, 轴)`，
/// dist 沿轨迹不变，取世界包围盒 8 角到螺旋轴的最大距离（保守：包围盒 ⊇ 物体）。
/// 无界几何（包围盒 None）→ None（Unknown）。
fn max_speed(xi: [f64; 6], a: &Geometry, wa: [f64; 16]) -> Option<f64> {
    let w = [xi[0], xi[1], xi[2]];
    let v = [xi[3], xi[4], xi[5]];
    let bb = world_aabb(a, wa)?;
    let wn = norm(w);
    if wn < 1e-12 {
        return Some(norm(v)); // 纯平移
    }
    // 螺旋轴：过 o = ω×v/|ω|²，方向 ω̂；轴向速度 = (ω·v/|ω|²)·ω
    let o = scale(cross(w, v), 1.0 / (wn * wn));
    let h = dot(w, v) / (wn * wn);
    let v_par = h.abs() * wn;
    let wu = scale(w, 1.0 / wn);
    let mut r_max = 0.0f64;
    for c in 0..8 {
        let p = [
            if c & 1 == 0 { bb[0][0] } else { bb[1][0] },
            if c & 2 == 0 { bb[0][1] } else { bb[1][1] },
            if c & 4 == 0 { bb[0][2] } else { bb[1][2] },
        ];
        let q = sub(p, o);
        let axial = scale(wu, dot(q, wu));
        r_max = r_max.max(norm(sub(q, axial)));
    }
    Some(v_par + wn * r_max)
}
