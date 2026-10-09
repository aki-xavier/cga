//! GPU 侧纹理采样（MLX）。CPU 纹理（`cga_scene::Texture`，Vec<f32> 像素）
//! 与 PNG 编解码已抽至 cga-scene；本模块保留 Array 侧的活：`GpuTexture`
//! （CPU → GPU 的转换 + 双线性采样）。`GpuTexture::from_cpu` 用
//! `Array::from_slice`（惰性，eval 才上传）；每次调用现场转换——纹理重型
//! 交互场景若成为瓶颈，跨帧缓存另行立项。

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
pub struct GpuTexture {
    pub pixels: Array,
    pub height: i32,
    pub width: i32,
    pub is_linear: bool,
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
}
