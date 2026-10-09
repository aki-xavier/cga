//! GPU 帧输出（MLX Array → PNG）。PNG 编解码本身已抽至 `cga_scene::png`
//! （docs/module-review.md）；此处路径兼容重导出 + Array 侧的帧写出。

use mlx_rs::Array;

// 路径兼容层（`cga_gpu::{decode_png_rgba, load_png_rgba, encode_png_rgba,
// save_png_rgba}` 旧路径不变）。
pub use cga_scene::{decode_png_rgba, encode_png_rgba, load_png_rgba, save_png_rgba};

pub fn save_frame_png(path: &str, img: &Array) {
    let sh = img.shape();
    let h = sh[0];
    let w = sh[1];
    img.eval().unwrap();
    let data = img.as_slice::<f32>().to_vec();
    save_png_rgba(path, w, h, &f32_rgba_to_u8(&data));
}

pub fn frame_to_png_bytes(img: &Array) -> Vec<u8> {
    let sh = img.shape();
    let h = sh[0];
    let w = sh[1];
    img.eval().unwrap();
    let data = img.as_slice::<f32>().to_vec();
    encode_png_rgba(w, h, &f32_rgba_to_u8(&data))
}

pub fn f32_rgba_to_u8(data: &[f32]) -> Vec<u8> {
    let mut out = vec![0u8; data.len()];
    for (i, v) in data.iter().enumerate() {
        let v = *v;
        if v < 0.0 {
            out[i] = 0;
        } else if v > 255.0 {
            out[i] = 255;
        } else {
            out[i] = (v + 0.5) as u8;
        }
    }
    out
}
