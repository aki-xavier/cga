pub const T_MIN: f64 = 0.0;

pub const DEGENERATE_ULPS: f32 = 8.0;

pub const UV_PROBE: f64 = 1e-4;

#[inline]
pub fn degenerate_tol(t: f32) -> f32 {
    DEGENERATE_ULPS * f32::EPSILON * t.abs().max(1.0)
}

#[inline]
pub fn tol_arr(t: &mlx_rs::Array) -> mlx_rs::Array {
    use crate::mlxops::{ck, fs, s_mul};
    s_mul(
        &ck(mlx_rs::ops::maximum(&ck(t.abs()), fs(1.0))),
        (DEGENERATE_ULPS as f64) * (f32::EPSILON as f64),
    )
}
