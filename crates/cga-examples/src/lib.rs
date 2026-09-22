use mlx_rs::Array;

pub fn data_f32(img: &Array) -> Vec<f32> {
    img.eval().unwrap();
    img.as_slice::<f32>().to_vec()
}

// 为什么: mlx-rs 没有 gc_collect；每帧调用回收不复用的 Metal cache buffer，防止长时间渲染时 GPU 内存持续上涨。
pub fn mlx_frame_gc() {
    let _ = mlx_rs::memory::clear_cache();
}
