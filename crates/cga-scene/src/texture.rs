//! CPU 纹理：`pixels` 是 `Vec<f32>`（行主序 RGBA，0..1）。GPU 采样所需的
//! MLX Array 转换在 cga-gpu 的 `texture::{GpuTexture, GpuTextureCache}`
//! （按内容指纹跨帧缓存，见该处文档）。构造与文件解码全部在 CPU 侧。

use super::*;

#[derive(Clone, Debug)]
pub struct Texture {
    pub pixels: Vec<f32>,
    pub height: i32,
    pub width: i32,
    pub is_linear: bool,
}
impl Texture {
    /// 内容指纹（FNV-1a 64）：尺寸 + 色彩空间标记 + 全部像素位模式。
    ///
    /// 用途是 GPU 侧纹理缓存的键——**同样的内容必得同样的指纹**，内容变了
    /// 指纹就变，缓存不会读到过期像素。与渲染器的 `material_fp` 不同：那个
    /// 只取 (w, h, is_linear)，因为它假定"同一会话内资产文件不变"；这里
    /// 连像素一起哈希，所以纹理就地改内容也能被看见（宁可多上传一次）。
    ///
    /// 碰撞概率：任意两个不同内容撞上同一指纹的概率约 2⁻⁶⁴。这是概率性保证
    /// 而非数学保证——真要严格，得在缓存命中时逐位比对像素（代价与上传同量级，
    /// 缓存就白做了）。此处明确记录该边界，不假装是精确判定。
    pub fn content_fp(&self) -> u64 {
        const OFF: u64 = 0xcbf2_9ce4_8422_2325;
        const PRIME: u64 = 0x1000_0000_01b3;
        let mut h = OFF;
        let mut eat = |bytes: &[u8]| {
            for &b in bytes {
                h ^= b as u64;
                h = h.wrapping_mul(PRIME);
            }
        };
        eat(&self.width.to_le_bytes());
        eat(&self.height.to_le_bytes());
        eat(&[self.is_linear as u8]);
        eat(&(self.pixels.len() as u64).to_le_bytes());
        for px in &self.pixels {
            eat(&px.to_le_bytes());
        }
        h
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn ramp(w: i32, h: i32) -> Texture {
        let mut px = Vec::with_capacity((w * h * 4) as usize);
        for i in 0..(w * h * 4) {
            px.push((i as f32) / 97.0);
        }
        Texture {
            pixels: px,
            height: h,
            width: w,
            is_linear: false,
        }
    }

    #[test]
    fn content_fp_is_deterministic_and_content_sensitive() {
        let a = ramp(4, 3);
        let b = ramp(4, 3);
        // 确定性：同样内容两次构造 → 同指纹（闭式：函数无内部状态）。
        assert_eq!(a.content_fp(), b.content_fp());

        // 敏感：像素动一位 ULP → 指纹变（按位翻转，不是相加——最小次正规数
        // 加到 0.05 上是空操作）。
        let mut c = ramp(4, 3);
        c.pixels[5] = f32::from_bits(c.pixels[5].to_bits() + 1);
        assert_ne!(a.content_fp(), c.content_fp());

        // 敏感：尺寸不同 → 指纹变（即便像素数凑成一样）。
        let d = ramp(3, 4);
        assert_eq!(d.pixels.len(), a.pixels.len());
        assert_ne!(a.content_fp(), d.content_fp());

        // 敏感：色彩空间标记不同 → 指纹变。
        let mut e = ramp(4, 3);
        e.is_linear = true;
        assert_ne!(a.content_fp(), e.content_fp());
    }
}
