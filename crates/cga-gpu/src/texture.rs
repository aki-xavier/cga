// Immutable linear RGBA texture sampled on the MLX device.  Source images are
// decoded from PNG (sRGB -> linear) before entering the renderer.
//
// Port note: V decoded arbitrary image bytes via stb_image (`stbi`); the Rust
// port uses the `image` crate instead (PNG / JPEG / ...).

use mlx_rs::{ops, Array};

use crate::geometry_ops::col;
use crate::image_io::{decode_png_rgba, load_png_rgba};
use crate::mlxops::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WrapMode {
    Repeat,
    Clamp,
}

#[derive(Clone, Debug)]
pub struct Texture {
    pub pixels: Array, // (height, width, 4) float32 raw texels in [0,1]
    pub height: i32,
    pub width: i32,
    pub is_linear: bool, // true for data maps (metallicRoughness/normal), false for sRGB colour maps
}

// sRGB -> linear on an (N,3) encoded float array.
fn srgb_to_linear_arr(rgb: &Array) -> Array {
    ck(ops::select(
        s_le(rgb, 0.04045),
        s_div(rgb, 12.92),
        s_pow(&s_div(&s_add(rgb, 0.055), 1.055), 2.4),
    ))
}

// texture_from_rgba builds a texture from raw sRGB RGBA floats (0..1).
pub fn texture_from_rgba(rgba: &[Vec<Vec<f64>>]) -> Texture {
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
        pixels: Array::from_slice(&flat, &[h as i32, w as i32, 4]),
        height: h as i32,
        width: w as i32,
        is_linear: false,
    }
}

// texture_from_u8_rgba builds a Texture from raw sRGB RGBA bytes (row-major).
pub fn texture_from_u8_rgba(rgba: &[u8], w: i32, h: i32) -> Texture {
    texture_from_raw(rgba, w, h, false)
}

fn texture_from_raw(rgba: &[u8], w: i32, h: i32, is_linear: bool) -> Texture {
    let mut flat = vec![0f32; rgba.len()];
    for (i, b) in rgba.iter().enumerate() {
        flat[i] = *b as f32 / 255.0;
    }
    Texture {
        pixels: Array::from_slice(&flat, &[h, w, 4]),
        height: h,
        width: w,
        is_linear,
    }
}

// texture_load loads a PNG file and decodes it to linear RGBA.
pub fn texture_load(path: &str) -> Result<Texture, String> {
    let (rgba, w, h) = load_png_rgba(path)?;
    Ok(texture_from_u8_rgba(&rgba, w, h))
}

// texture_from_png_bytes decodes a PNG (raw bytes) to a linear RGBA texture.
pub fn texture_from_png_bytes(data: &[u8]) -> Result<Texture, String> {
    let (rgba, w, h) = decode_png_rgba(data)?;
    Ok(texture_from_u8_rgba(&rgba, w, h))
}

// texture_from_bytes decodes image bytes (PNG / JPEG / ...) via stb_image to a
// linear RGBA texture (used for glTF-embedded textures, which are often JPEG).
pub fn texture_from_bytes(data: &[u8]) -> Result<Texture, String> {
    let img = image::load_from_memory(data).map_err(|e| format!("stbi decode failed: {e}"))?;
    let rgba = img.to_rgba8();
    let (w, h) = rgba.dimensions();
    Ok(texture_from_u8_rgba(&rgba.into_raw(), w as i32, h as i32))
}

fn wrap_value(value: &Array, mode: WrapMode) -> Array {
    match mode {
        WrapMode::Repeat => ck(value.subtract(ck(value.floor()))),
        WrapMode::Clamp => s_clip(value, 0.0, 1.0),
    }
}

impl Texture {
    // sample returns bilinearly interpolated RGBA texels for an (N,2) UV array.
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
        let res = ck(ck(top.multiply(&omfy)).add(ck(bot.multiply(&fy)))); // (count,4) raw interpolated
        let rgb = ck(res.take_axis(Array::from_slice(&[0i32, 1, 2], &[3]), 1));
        let lin = if t.is_linear {
            rgb
        } else {
            srgb_to_linear_arr(&rgb)
        };
        let a = ck(ck(res.take_axis(Array::from_int(3), 1)).expand_dims(1));
        ck(ops::concatenate(&[&lin, &a], 1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image_io::save_frame_png;
    use crate::{
        ambient_light, color_hex, directional_light, mesh, perspective_camera, render_frame, scene,
        standard_material, MaterialParams, MeshParams,
    };
    use cga_core::{box_geometry, Geometry};

    #[test]
    fn test_png_decode_and_sample() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/cgs/assets/brick.png"
        );
        let (rgba, w, h) = load_png_rgba(path).unwrap();
        assert!(w == 256 && h == 256);
        assert_eq!(rgba.len(), 256 * 256 * 4);
        let tex = texture_load(path).unwrap();
        assert!(tex.width == 256 && tex.height == 256);
        let uv = Array::from_slice(&[0.5f32, 0.5], &[1, 2]);
        let s = tex.sample(&uv, WrapMode::Repeat, WrapMode::Repeat);
        s.eval().unwrap();
        let data = s.as_slice::<f32>();
        assert_eq!(data.len(), 4);
        assert!(data[3] == 1.0); // opaque alpha
    }

    #[test]
    fn test_textured_render() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/cgs/assets/brick.png"
        );
        let tex = texture_load(path).unwrap();
        let mut sc = scene(None);
        let mut mat = standard_material(MaterialParams {
            color: color_hex(0xFFFFFF),
            roughness: 0.5,
            metalness: 0.0,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        });
        mat.map = Some(tex);
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::BoxGeometry(box_geometry(1.0, 1.0, 1.0)),
            material: mat,
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]));
        sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.2));
        let mut cam = perspective_camera(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = render_frame(sc, cam, 80, 80, 1);
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../artifacts/tests");
        std::fs::create_dir_all(dir).ok();
        save_frame_png(&format!("{dir}/textured_box.png"), &img);
        img.eval().unwrap();
        let data = img.as_slice::<f32>();
        // centre should be brick-ish (red channel notably above the sky-blue 135)
        let idx = 40 * 80 * 4 + 40 * 4;
        assert!(data[idx] > data[idx + 2]);
    }
}
