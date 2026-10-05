use mlx_rs::{ops, Array};

use crate::geometry_ops::col;
use crate::image_io::{decode_png_rgba, load_png_rgba};
use crate::mlxops::*;

pub mod wrap_mode;
pub use self::wrap_mode::*;
pub mod texture;
pub use self::texture::*;

fn srgb_to_linear_arr(rgb: &Array) -> Array {
    ck(ops::select(
        s_le(rgb, 0.04045),
        s_div(rgb, 12.92),
        s_pow(&s_div(&s_add(rgb, 0.055), 1.055), 2.4),
    ))
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

fn wrap_value(value: &Array, mode: WrapMode) -> Array {
    match mode {
        WrapMode::Repeat => ck(value.subtract(ck(value.floor()))),
        WrapMode::Clamp => s_clip(value, 0.0, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image_io::save_frame_png;
    use crate::scene_graph::Color;
    use crate::{
        Light, Material, MaterialParams, Mesh, MeshParams, PerspectiveCamera, Renderer, Scene,
    };
    use cga_core::{BoxGeometry, Geometry};

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
        assert!(data[3] == 1.0);
    }

    #[test]
    fn test_textured_render() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/cgs/assets/brick.png"
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
        sc.add_mesh(Mesh::new(MeshParams {
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
