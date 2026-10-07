//! 三角网格化与网格编码（纯 f64 CPU，只依赖 cga-core）。
//!
//! - [`bake`]：`GeometryParams` → 水密三角网（marching tetrahedra + 认证）。
//! - [`stl`]：STL 二进制/ASCII 编码（导出方向；不解析）。

pub mod bake;
pub use bake::*;

pub mod stl;
pub use stl::*;
