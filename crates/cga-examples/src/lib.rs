//! cga-examples — 共享小助手（`data_f32` / `mlx_frame_gc` 等）。

use mlx_rs::Array;

/// data_f32: eval 后取出 (H, W, 4) float32 RGBA 像素。
pub fn data_f32(img: &Array) -> Vec<f32> {
    img.eval().unwrap();
    img.as_slice::<f32>().to_vec()
}

/// 每帧收尾：drop(img) 即释放死 Metal buffer（mlx-rs 没有 gc_collect），
/// 这里只需把 MLX 不复用的 cache 还给 OS。
pub fn mlx_frame_gc() {
    let _ = mlx_rs::memory::clear_cache();
}
