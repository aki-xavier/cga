use mlx_rs::Array;

pub fn data_f32(img: &Array) -> Vec<f32> {
    img.eval().unwrap();
    img.as_slice::<f32>().to_vec()
}

pub fn mlx_frame_gc() {
    let _ = mlx_rs::memory::clear_cache();
}
pub mod gif;
pub use gif::*;
