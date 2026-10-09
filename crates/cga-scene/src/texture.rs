//! CPU 纹理：`pixels` 是 `Vec<f32>`（行主序 RGBA，0..1）。GPU 采样所需的
//! MLX Array 转换在 cga-gpu 的 `texture::GpuTexture`（每帧/每次求交转换，
//! 见该处文档）。构造与文件解码全部在 CPU 侧。

use super::*;

#[derive(Clone, Debug)]
pub struct Texture {
    pub pixels: Vec<f32>,
    pub height: i32,
    pub width: i32,
    pub is_linear: bool,
}
impl Texture {
    pub fn from_rgba(rgba: &[Vec<Vec<f64>>]) -> Texture {
        let h = rgba.len();
        if h < 1 {
            panic!("texture must have >= 1 row");
        }
        let w = rgba[0].len();
        let mut flat = Vec::with_capacity(h * w * 4);
        for row in rgba {
            if row.len() != w || row[0].len() != 4 {
                panic!("texture rgba must be (height, width, 4)");
            }
            for px in row {
                for &c in px {
                    flat.push(c as f32);
                }
            }
        }
        Texture {
            pixels: flat,
            height: h as i32,
            width: w as i32,
            is_linear: false,
        }
    }
}
impl Texture {
    pub fn from_u8_rgba(rgba: &[u8], w: i32, h: i32) -> Texture {
        Self::from_raw(rgba, w, h, false)
    }

    fn from_raw(rgba: &[u8], w: i32, h: i32, is_linear: bool) -> Texture {
        let mut flat = vec![0f32; rgba.len()];
        for (i, b) in rgba.iter().enumerate() {
            flat[i] = *b as f32 / 255.0;
        }
        Texture {
            pixels: flat,
            height: h,
            width: w,
            is_linear,
        }
    }
}

pub fn texture_load(path: &str) -> Result<Texture, String> {
    let (rgba, w, h) = load_png_rgba(path)?;
    Ok(Texture::from_u8_rgba(&rgba, w, h))
}

pub fn texture_from_png_bytes(data: &[u8]) -> Result<Texture, String> {
    let (rgba, w, h) = decode_png_rgba(data)?;
    Ok(Texture::from_u8_rgba(&rgba, w, h))
}

pub fn texture_from_bytes(data: &[u8]) -> Result<Texture, String> {
    let img = image::load_from_memory(data).map_err(|e| format!("image decode failed: {e}"))?;
    let rgba = img.to_rgba8();
    let (w, h) = rgba.dimensions();
    Ok(Texture::from_u8_rgba(&rgba.into_raw(), w as i32, h as i32))
}
