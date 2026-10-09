//! 场景图类型已抽至 `cga-scene`（docs/module-review.md 问题 1）；本模块是
//! 路径兼容层（`cga_gpu::scene::*` 旧路径不变）。

pub use cga_scene::{Object, ObjectParams, OrbitControls, PerspectiveCamera, Scene};
