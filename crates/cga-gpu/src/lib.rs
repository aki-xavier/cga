pub mod mlxops;
pub use mlxops::*;

pub mod scene_graph;
pub use scene_graph::*;

pub mod scene;
pub use scene::*;

pub mod geometry_ops;
pub use geometry_ops::*;

pub mod geometry_extra;
pub use geometry_extra::*;

pub mod geom_kernels;
pub use geom_kernels::*;

pub mod shading;
pub use shading::*;

pub mod texture;
pub use texture::*;

pub mod image_io;
pub use image_io::*;

pub mod renderer;
pub use renderer::*;

pub mod csg;
pub use csg::*;

pub mod tol;

pub mod certify;

pub(crate) mod fallback;
pub(crate) mod bvh_trace;

#[cfg(test)]
mod degenerate;

/// 测试期落盘的调试产物（给人眼看的 PNG，**测试不依赖它们**）。
///
/// 之前各测试直接 `create_dir_all(..).unwrap()` + `save_frame_png(..)`，后者
/// 内部对写入失败是 `panic!` —— 于是只读源码树（容器挂载、CI 只读 checkout）
/// 上会仅因写不动这个给人看的目录而测试失败。统一成**best-effort**：写不出
/// 去就跳过，不影响断言。
#[cfg(test)]
pub(crate) fn save_artifact(name: &str, img: &mlx_rs::Array) {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../artifacts/tests");
    if std::fs::create_dir_all(dir).is_err() {
        return;
    }
    let _ = std::fs::write(
        format!("{dir}/{name}"),
        crate::image_io::frame_to_png_bytes(img),
    );
}
