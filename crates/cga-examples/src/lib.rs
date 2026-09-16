//! cga-examples — 共享小助手（V demos 里的 `mlx.gc_collect(); mlx.clear_cache()` 等）。

use mlx_rs::Array;

/// data_f32: eval 后取出 (H, W, 4) float32 RGBA 像素（对应 V 的 `img.data_f32()`）。
pub fn data_f32(img: &Array) -> Vec<f32> {
    img.eval().unwrap();
    img.as_slice::<f32>().to_vec()
}

/// 每帧收尾：V 里是 `mlx.gc_collect(); mlx.clear_cache()`（见 editor/server.v）。
/// Rust 里 drop(img) 即释放死 Metal buffer（mlx-rs 没有 gc_collect），
/// 只需把 MLX 不复用的 cache 还给 OS。
pub fn mlx_frame_gc() {
    let _ = mlx_rs::memory::clear_cache();
}
