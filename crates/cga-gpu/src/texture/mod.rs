//! GPU 侧纹理采样（MLX）。CPU 纹理（`cga_scene::Texture`，Vec<f32> 像素）
//! 与 PNG 编解码已抽至 cga-scene；本模块保留 Array 侧的活：`GpuTexture`
//! （CPU → GPU 的转换 + 双线性采样）与 `GpuTextureCache`（按内容指纹的
//! 跨帧缓存）。
//!
//! 为什么要缓存：`Array::from_slice` 惰性求值，真正的上传发生在 eval。
//! 若每次采样都现转，同一张纹理每帧要重新上传一次（1K RGBA ≈ 16 MB/帧）。
//! [`GpuTextureCache`] 以 [`Texture::content_fp`] 为键，跨帧复用已上传的
//! Array；`IncrementalRenderer` 持有 `Renderer`，缓存因此跨帧存活，一次性
//! `Renderer::new(..).render(..)` 仍是一次一传（无复用可省）。
//!
//! 诚实边界：键是 64 位内容哈希，碰撞概率约 2⁻⁶⁴（见
//! [`Texture::content_fp`] 的说明）。命中即信任——这是概率性判定，不是逐位
//! 比对。真要严格可以在命中时比像素，代价与上传同量级，缓存便无意义。

use std::cell::RefCell;
use std::collections::HashMap;

use crate::geometry_ops::col;
use crate::mlxops::*;
use mlx_rs::{ops, Array};

// 路径兼容层（`cga_gpu::texture::{Texture, WrapMode, texture_load, ...}` 不变）。
pub use cga_scene::{texture_from_bytes, texture_from_png_bytes, texture_load, Texture, WrapMode};

fn srgb_to_linear_arr(rgb: &Array) -> Array {
    ck(ops::select(
        s_le(rgb, 0.04045),
        s_div(rgb, 12.92),
        s_pow(&s_div(&s_add(rgb, 0.055), 1.055), 2.4),
    ))
}

/// GPU 侧纹理：`pixels` 是 MLX Array（[h, w, 4] f32 0..1）。
#[derive(Clone)]
pub struct GpuTexture {
    // 字段逐个列出以便 derive；pixels 是 MLX Array（无 Debug），故手写 Debug。
    pub pixels: Array,
    pub height: i32,
    pub width: i32,
    pub is_linear: bool,
}
impl std::fmt::Debug for GpuTexture {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuTexture")
            .field("height", &self.height)
            .field("width", &self.width)
            .field("is_linear", &self.is_linear)
            .finish_non_exhaustive()
    }
}
impl GpuTexture {
    pub fn from_cpu(t: &Texture) -> GpuTexture {
        GpuTexture {
            pixels: Array::from_slice(&t.pixels, &[t.height, t.width, 4]),
            height: t.height,
            width: t.width,
            is_linear: t.is_linear,
        }
    }
}

/// 按内容指纹的纹理缓存（CPU 纹理 → 已上传的 MLX Array）。
///
/// 语义：
/// - 命中：返回克隆的 `GpuTexture`（MLX Array 克隆是引用计数，不复制数据）；
/// - 未命中：现场转换并登记；
/// - 纹理内容改变 → 指纹变 → 自然失效，**不需要**调用方记得清缓存
///   （这比渲染器 `material_fp` 的 (w,h,is_linear) 签名更严：后者假定会话内
///   资产文件不变，纹理就地改像素得手动 `invalidate`）。
#[derive(Debug, Default)]
pub struct GpuTextureCache {
    map: RefCell<HashMap<u64, GpuTexture>>,
}
impl GpuTextureCache {
    pub fn new() -> GpuTextureCache {
        GpuTextureCache::default()
    }

    /// 取纹理对应的 GPU 侧数据（命中则复用）。
    pub fn get_or_upload(&self, t: &Texture) -> GpuTexture {
        let key = t.content_fp();
        if let Some(g) = self.map.borrow().get(&key) {
            return g.clone();
        }
        let g = GpuTexture::from_cpu(t);
        self.map.borrow_mut().insert(key, g.clone());
        g
    }

    /// 已登记的纹理数（测试用：钉住"重复命中不增长、不同内容各自登记"）。
    pub fn len(&self) -> usize {
        self.map.borrow().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 清空（纹理全部失效）。正常路径用不上——内容变了指纹就变了。
    pub fn clear(&self) {
        self.map.borrow_mut().clear();
    }
}

fn wrap_value(value: &Array, mode: WrapMode) -> Array {
    match mode {
        WrapMode::Repeat => ck(value.subtract(ck(value.floor()))),
        WrapMode::Clamp => s_clip(value, 0.0, 1.0),
    }
}

impl GpuTexture {
    pub fn sample(&self, uv: &Array, wrap_s: WrapMode, wrap_t: WrapMode) -> Array {
        let t = self;
        let sh = uv.shape();
        if sh.len() != 2 || sh[1] != 2 {
            panic!("uv must have shape (count, 2)");
        }
        let u = wrap_value(&col(uv, 0), wrap_s);
        let v = wrap_value(&col(uv, 1), wrap_t);
        let x = s_sub(&s_mul(&u, t.width as f64), 0.5);
        let y = s_sub(&s_mul(&s_rsub(&v, 1.0), t.height as f64), 0.5);
        let x0 = ck(ck(x.floor()).as_type::<i32>());
        let y0 = ck(ck(y.floor()).as_type::<i32>());
        let fx = ck(ck(x.subtract(ck(x0.as_type::<f32>()))).expand_dims(1));
        let fy = ck(ck(y.subtract(ck(y0.as_type::<f32>()))).expand_dims(1));
        let one = Array::from_int(1);
        let x1 = ck(x0.add(&one));
        let y1 = ck(y0.add(&one));
        let (x0r, x1) = if wrap_s == WrapMode::Repeat {
            let w = Array::from_int(t.width);
            (ck(x0.remainder(&w)), ck(x1.remainder(&w)))
        } else {
            let lo = Array::from_int(0);
            let hi = Array::from_int(t.width - 1);
            (
                ck(ops::clip(&x0, (&lo, &hi))),
                ck(ops::clip(&x1, (&lo, &hi))),
            )
        };
        let (y0r, y1) = if wrap_t == WrapMode::Repeat {
            let h = Array::from_int(t.height);
            (ck(y0.remainder(&h)), ck(y1.remainder(&h)))
        } else {
            let lo = Array::from_int(0);
            let hi = Array::from_int(t.height - 1);
            (
                ck(ops::clip(&y0, (&lo, &hi))),
                ck(ops::clip(&y1, (&lo, &hi))),
            )
        };
        let flat = ck(t.pixels.reshape(&[t.height * t.width, 4]));
        let warr = Array::from_int(t.width);
        let c00 = ck(flat.take_axis(ck(ck(y0r.multiply(&warr)).add(&x0r)), 0));
        let c10 = ck(flat.take_axis(ck(ck(y0r.multiply(&warr)).add(&x1)), 0));
        let c01 = ck(flat.take_axis(ck(ck(y1.multiply(&warr)).add(&x0r)), 0));
        let c11 = ck(flat.take_axis(ck(ck(y1.multiply(&warr)).add(&x1)), 0));
        let omfx = ck(fs(1.0).subtract(&fx));
        let omfy = ck(fs(1.0).subtract(&fy));
        let top = ck(ck(c00.multiply(&omfx)).add(ck(c10.multiply(&fx))));
        let bot = ck(ck(c01.multiply(&omfx)).add(ck(c11.multiply(&fx))));
        let res = ck(ck(top.multiply(&omfy)).add(ck(bot.multiply(&fy))));
        let rgb = ck(res.take_axis(Array::from_slice(&[0i32, 1, 2], &[3]), 1));
        let lin = if t.is_linear {
            rgb
        } else {
            srgb_to_linear_arr(&rgb)
        };
        let a = ck(res.take_axis(Array::from_slice(&[3i32], &[1]), 1));
        ck(ops::concatenate(&[&lin, &a], 1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image_io::save_frame_png;
    use crate::scene_graph::Color;
    use crate::{
        Light, Material, MaterialParams, Object, ObjectParams, PerspectiveCamera, Renderer, Scene,
    };
    use cga_core::{BoxGeometry, Geometry};
    use cga_scene::load_png_rgba;

    #[test]
    fn test_png_decode_and_sample() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/gallery/assets/brick.png"
        );
        let (rgba, w, h) = load_png_rgba(path).unwrap();
        assert!(w == 256 && h == 256);
        assert_eq!(rgba.len(), 256 * 256 * 4);
        let tex = texture_load(path).unwrap();
        assert!(tex.width == 256 && tex.height == 256);
        let uv = Array::from_slice(&[0.5f32, 0.5], &[1, 2]);
        let s = GpuTexture::from_cpu(&tex).sample(&uv, WrapMode::Repeat, WrapMode::Repeat);
        s.eval().unwrap();
        let data = s.as_slice::<f32>();
        assert_eq!(data.len(), 4);
        assert!(data[3] == 1.0);
    }

    #[test]
    fn test_textured_render() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/gallery/assets/brick.png"
        );
        let tex = texture_load(path).unwrap();
        let mut sc = Scene::new(None);
        let mut mat = Material::standard(MaterialParams {
            color: Color::from_hex(0xFFFFFF),
            roughness: 0.5,
            metalness: 0.0,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        });
        mat.map = Some(tex);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
            material: mat,
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.5, 1.0, 0.5],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.2));
        let mut cam = PerspectiveCamera::new(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = Renderer::render_frame(sc, cam, 80, 80, 1);
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../artifacts/tests");
        std::fs::create_dir_all(dir).ok();
        save_frame_png(&format!("{dir}/textured_box.png"), &img);
        img.eval().unwrap();
        let data = img.as_slice::<f32>();

        let idx = 40 * 80 * 4 + 40 * 4;
        assert!(data[idx] > data[idx + 2]);
    }

    /// 缓存语义（闭式，无渲染参与）：
    /// - 重复命中同一内容：登记数不增长，返回的像素逐位相同；
    /// - 内容变一位 ULP：视为新纹理（各自登记），不会读到旧像素。
    #[test]
    fn texture_cache_reuses_and_invalidates_by_content() {
        let cache = GpuTextureCache::new();
        assert!(cache.is_empty());

        let mut ramp = Vec::new();
        for i in 0..(2 * 2 * 4) {
            ramp.push((i as f32) / 13.0);
        }
        let t = Texture {
            pixels: ramp.clone(),
            height: 2,
            width: 2,
            is_linear: false,
        };
        let t2 = Texture {
            pixels: ramp.clone(),
            height: 2,
            width: 2,
            is_linear: false,
        };

        // 未命中 → 登记 1；重复命中 → 仍是 1。
        let a = cache.get_or_upload(&t);
        assert_eq!(cache.len(), 1);
        let b = cache.get_or_upload(&t2);
        assert_eq!(cache.len(), 1);
        a.pixels.eval().unwrap();
        b.pixels.eval().unwrap();
        // 同样内容 → 逐位相同的像素（命中复用了同一份上传数据）。
        assert_eq!(a.pixels.as_slice::<f32>(), b.pixels.as_slice::<f32>());

        // 内容改一位 ULP → 指纹变 → 新登记（读到的是新像素，不是旧的）。
        let mut edited = ramp.clone();
        edited[0] = f32::from_bits(edited[0].to_bits() + 1);
        let mut t3 = t.clone();
        t3.pixels = edited;
        let c = cache.get_or_upload(&t3);
        assert_eq!(cache.len(), 2);
        c.pixels.eval().unwrap();
        assert_ne!(c.pixels.as_slice::<f32>()[0], a.pixels.as_slice::<f32>()[0]);

        cache.clear();
        assert!(cache.is_empty());
    }

    /// 端到端：带缓存的多次渲染与「每次现转」的渲染逐位一致。
    /// 这是缓存不许改变语义的那条钉子——缓存只能省上传，不能改像素。
    #[test]
    fn texture_cache_does_not_change_rendered_pixels() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/gallery/assets/brick.png"
        );
        let tex = texture_load(path).unwrap();
        let scene = || {
            let mut sc = Scene::new(None);
            sc.add_object(Object::new(ObjectParams {
                geometry: Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0)),
                material: Material {
                    map: Some(tex.clone()),
                    ..Material::standard(MaterialParams {
                        color: Color::from_hex(0xFFFFFF),
                        roughness: 0.3,
                        metalness: 0.1,
                        emissive: Color::from_hex(0x000000),
                        opacity: 1.0,
                        ior: 1.5,
                        absorption: 0.0,
                    })
                },
                position: [0.0; 3],
                rotation_axis: [0.0, 0.0, 1.0],
                rotation_angle: 0.0,
                motor: None,
            }));
            sc.add_light(Light::directional(
                Color::from_hex(0xFFFFFF),
                0.8,
                [1.0, 1.0, 1.0],
            ));
            sc
        };
        let mut cam = PerspectiveCamera::new(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);

        // 同一个 Renderer 连渲三帧：第一帧现传，后两帧走缓存。
        let mut r = Renderer::new(48, 36, 1, 3);
        let f1 = r.render(scene(), cam);
        let f2 = r.render(scene(), cam);
        let f3 = r.render(scene(), cam);
        // 一次性渲染器（无跨帧复用机会）作为对照。
        let fresh = Renderer::new(48, 36, 1, 3).render(scene(), cam);
        for img in [&f1, &f2, &f3, &fresh] {
            img.eval().unwrap();
        }
        let (a, b, c, d) = (
            f1.as_slice::<f32>().to_vec(),
            f2.as_slice::<f32>().to_vec(),
            f3.as_slice::<f32>().to_vec(),
            fresh.as_slice::<f32>().to_vec(),
        );
        assert_eq!(a, b, "缓存命中帧必须与首帧逐位一致");
        assert_eq!(a, c, "第二次缓存命中帧必须逐位一致");
        assert_eq!(a, d, "缓存不得改变渲染结果（与一次性渲染逐位一致）");
        assert_eq!(r.tex_cache.len(), 1, "同一纹理只登记一次");
    }
}
