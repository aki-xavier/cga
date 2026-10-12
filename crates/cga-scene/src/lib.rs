//! cga-scene — 场景数据模型（docs/module-review.md 问题 1：场景表示 ≠ 渲染器）。
//!
//! 从 cga-gpu 抽出的纯 CPU 场景层：场景图（Scene/Object/Object3D/Color）、
//! 相机（PerspectiveCamera/OrbitControls）、灯光与材质的数据结构（Light/
//! Material）、CPU 纹理（Texture + PNG 编解码）。**不含任何 MLX/Metal 依赖**——
//! 纹理像素是 `Vec<f32>`，GPU 采样（MLX Array 转换）在 cga-gpu 的
//! `texture::GpuTexture`；灯光的批量光照方法（`direction_at`/`far`）在
//! cga-gpu 的 `shading::{light_direction_at, light_far}`。

// 场景模型的公开 API 里嵌着这些类型（Object.geometry、PerspectiveCamera.motor、
// Light::to_camera…），随本 crate 一并重导出。
pub use cga_core::{clamp01, vec3_cross, vec3_dot, vec3_len, Geometry, Multivector};

pub mod color;
pub use self::color::*;
pub mod light;
pub use self::light::*;
pub mod material;
pub use self::material::*;
pub mod texture;
pub use self::texture::*;
pub mod camera;
pub use self::camera::*;
pub mod orbit_controls;
pub use self::orbit_controls::*;
pub mod object;
pub use self::object::*;
pub mod scene;
pub use self::scene::*;

pub fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        return c / 12.92;
    }
    ((c + 0.055) / 1.055).powf(2.4)
}
