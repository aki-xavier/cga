//! cga-scene — 场景数据模型（docs/module-review.md 问题 1：场景表示 ≠ 渲染器）。
//!
//! 从 cga-gpu 抽出的纯 CPU 场景层：场景图（Scene/Object/Object3D/Color）、
//! 相机（PerspectiveCamera/OrbitControls）、灯光与材质的数据结构（Light/
//! Material）、CPU 纹理（Texture + PNG 编解码）。**不含任何 MLX/Metal 依赖**——
//! 纹理像素是 `Vec<f32>`，GPU 采样（MLX Array 转换）在 cga-gpu 的
//! `texture::GpuTexture`；灯光的批量光照方法（`direction_at`/`far`）在
//! cga-gpu 的 `shading::{light_direction_at, light_far}`。

use cga_core::Mat3;

// 场景模型的公开 API 里嵌着这些类型（Object.geometry、PerspectiveCamera.motor、
// Light::to_camera…），随本 crate 一并重导出。
pub use cga_core::{clamp01, vec3_cross, Geometry, Multivector};

pub mod color;
pub use self::color::*;
pub mod object3d;
pub use self::object3d::*;
pub mod light_kind;
pub use self::light_kind::*;
pub mod light;
pub use self::light::*;
pub mod material_kind;
pub use self::material_kind::*;
pub mod material;
pub use self::material::*;
pub mod material_params;
pub use self::material_params::*;
pub mod wrap_mode;
pub use self::wrap_mode::*;
pub mod texture;
pub use self::texture::*;
pub mod png;
pub use self::png::*;
pub mod camera;
pub use self::camera::*;
pub mod orbit_controls;
pub use self::orbit_controls::*;
pub mod object;
pub use self::object::*;
pub mod object_params;
pub use self::object_params::*;
pub mod scene;
pub use self::scene::*;

pub fn vec3_unit(a: [f64; 3]) -> [f64; 3] {
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    if n < 1e-12 {
        return [0.0, 0.0, 1.0];
    }
    [a[0] / n, a[1] / n, a[2] / n]
}

pub fn vec3_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        return c / 12.92;
    }
    ((c + 0.055) / 1.055).powf(2.4)
}

pub fn identity3() -> Mat3 {
    let mut m: Mat3 = [[0.0; 3]; 3];
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
    m
}

pub fn is_identity3(m: Mat3) -> bool {
    for (i, row) in m.iter().enumerate() {
        for (j, &v) in row.iter().enumerate() {
            let want = if i == j { 1.0 } else { 0.0 };
            if (v - want).abs() > 1e-12 {
                return false;
            }
        }
    }
    true
}
