//! 颜色/Object3D/小工具已抽至 `cga-scene`（docs/module-review.md）；本模块是
//! 路径兼容层（`cga_gpu::scene_graph::*` 旧路径不变）。

pub use cga_scene::{identity3, is_identity3, srgb_to_linear, vec3_dot, vec3_unit, Color, Object3D};
