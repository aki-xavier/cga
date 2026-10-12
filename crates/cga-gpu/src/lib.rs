pub mod mlxops;
pub use mlxops::*;

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

pub(crate) mod bvh_trace;
pub(crate) mod fallback;

#[cfg(test)]
mod degenerate;

/// 阶段 3 漂移防线：f32 数组内核 ↔ f64 标量定义（`docs/geometry-unification.md`）。
#[cfg(test)]
mod ray_drift;

/// 测试用：`[x, y, z]` → 1×3 的 f32 数组（光线原点/方向都用它）。
#[cfg(test)]
pub(crate) fn test_ray(x: f64, y: f64, z: f64) -> mlx_rs::Array {
    mlx_rs::Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
}

/// 测试用：`geom_contains` 的逐点包装，相机空间（`m` = 刚体/仿射变换）。
#[cfg(test)]
pub(crate) fn test_contains(
    g: &cga_core::Geometry,
    m: &cga_core::Multivector,
    pts: &[[f64; 3]],
) -> Vec<bool> {
    let p = g.params_in_frame(m);
    let flat: Vec<f32> = pts.iter().flat_map(|q| q.map(|v| v as f32)).collect();
    let pos = mlx_rs::Array::from_slice(&flat, &[pts.len() as i32, 3]);
    let got = geom_contains(&p, &pos);
    got.eval().unwrap();
    got.as_slice::<bool>().to_vec()
}

/// 测试用：单条相机空间光线的最近命中 `(t, 是否命中)`。
#[cfg(test)]
pub(crate) fn test_hit(g: &cga_core::Geometry, o: [f64; 3], d: [f64; 3]) -> (f32, bool) {
    let p = g.params_in_frame(&cga_core::Multivector::identity());
    let (t, _, mask) = geom_intersect(&p, &test_ray(o[0], o[1], o[2]), &test_ray(d[0], d[1], d[2]));
    t.eval().unwrap();
    mask.eval().unwrap();
    (t.item_cast::<f32>(), mask.as_slice::<bool>()[0])
}

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
