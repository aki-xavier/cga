use crate::geom_kernels::geom_to_camera;
use crate::geometry_ops::{geom_bounds, geom_intersect, geom_shadow, geom_uv};
use crate::mlxops::*;
use crate::shading::shade_batched;
use cga_core::{vec3_cross, GeometryParams, Multivector};
use cga_scene::{
    vec3_dot, vec3_unit, Light, LightKind, Object, PerspectiveCamera, Scene, WrapMode,
};
use mlx_rs::{ops, Array};

pub mod truth;
pub use self::truth::*;
// 模块名与父模块同名：renderer.rs 是渲染管线本体（Renderer/IncrementalRenderer），
// mod.rs 只是入口与视图工具。该路径被 docs/ 引用，改名的收益不抵 churn。
#[allow(clippy::module_inception)]
pub mod renderer;
pub use self::renderer::*;

fn view_frame(camera: &PerspectiveCamera) -> ([[f64; 3]; 3], [f64; 3]) {
    let forward = vec3_unit([
        camera.target[0] - camera.position[0],
        camera.target[1] - camera.position[1],
        camera.target[2] - camera.position[2],
    ]);
    let right = vec3_unit(vec3_cross(forward, camera.up));
    let up = vec3_cross(right, forward);
    let basis = [right, [-up[0], -up[1], -up[2]], forward];
    let offset = [
        vec3_dot(basis[0], camera.position),
        vec3_dot(basis[1], camera.position),
        vec3_dot(basis[2], camera.position),
    ];
    (basis, offset)
}

fn outward(points: &Array, basis: &[[f64; 3]; 3], offset: &[f64; 3]) -> Array {
    let q = ck(points.add(arr3v(*offset)));
    let mut out = ck(ops::zeros::<f32>(&[1, 3]));
    for (axis, b) in basis.iter().enumerate() {
        let at = ck(ck(q.take_axis(Array::from_int(axis as i32), 1)).expand_dims(1));
        let term = ck(at.multiply(arr3v(*b)));
        out = if axis == 0 { term } else { ck(out.add(&term)) };
    }
    out
}

#[cfg(test)]
mod tests;
