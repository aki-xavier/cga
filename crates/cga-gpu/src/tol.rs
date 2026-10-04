//! P1 容差表 —— 所有"拍脑袋常数"集中于此，替代散落的 `1e-4` / `1e-6`。
//!
//! 理论、诊断与验收见 `docs/freeform-robust-boolean.md`。

/// 交叉点有效域：射线语义下只认相机前方的交点（`t > T_MIN`）。
/// 其后的第一个区间恒以射线原点为采样点。
pub const T_MIN: f64 = 0.0;

/// 相邻交叉点的**退化判据**：`gap ≤ ULPS · ε · max(|t|, 1)` 视为同一交点。
///
/// 相切 / 重根会给出一对重合交点，其间区间宽度为 0 —— 该区间判为不可信，
/// 其两侧的翻转一并作废（重合的一对交点互相抵消，切点不会被当成表面）。
/// 取 8 ulp 是因为 f32 下两条不同求根路径的同根可差几个 ulp；
/// 该值必须**远小于**要保留的最薄特征（薄壁 5e-5 ≫ 8·ε·max(|t|,1) ≈ 1e-6）。
pub const DEGENERATE_ULPS: f32 = 8.0;

/// `csg_uv` 的边界位置探针：P1 未改其算法，仅集中管理。
/// 遗留项见 `docs/freeform-robust-boolean.md` §6（P1 验收不含它）。
pub const UV_PROBE: f64 = 1e-4;

/// 某个交叉点 `t` 处的退化阈值（f32，与渲染内核同精度）。
#[inline]
pub fn degenerate_tol(t: f32) -> f32 {
    DEGENERATE_ULPS * f32::EPSILON * t.abs().max(1.0)
}

/// 同上，向量化：对交叉点数组逐元素给出退化阈值。
#[inline]
pub fn tol_arr(t: &mlx_rs::Array) -> mlx_rs::Array {
    use crate::mlxops::{ck, fs, s_mul};
    s_mul(
        &ck(mlx_rs::ops::maximum(&ck(t.abs()), fs(1.0))),
        (DEGENERATE_ULPS as f64) * (f32::EPSILON as f64),
    )
}
