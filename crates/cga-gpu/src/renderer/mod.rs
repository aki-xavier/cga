use crate::geom_kernels::geom_to_camera;
use crate::geometry_ops::{geom_bounds, geom_intersect, geom_shadow, geom_uv};
use crate::mlxops::*;
use crate::scene::{Object, PerspectiveCamera, Scene};
use crate::scene_graph::{vec3_dot, vec3_unit};
use crate::shading::{shade_batched, Light, LightKind};
use crate::texture::WrapMode;
use cga_core::{vec3_cross, GeometryParams, Multivector};
use mlx_rs::{ops, Array};

pub mod truth;
pub use self::truth::*;
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
    let q = ck(points.add(&arr3v(*offset)));
    let mut out = ck(ops::zeros::<f32>(&[1, 3]));
    for axis in 0..3usize {
        let at = ck(ck(q.take_axis(Array::from_int(axis as i32), 1)).expand_dims(1));
        let term = ck(at.multiply(&arr3v(basis[axis])));
        out = if axis == 0 { term } else { ck(out.add(&term)) };
    }
    out
}

#[cfg(test)]
mod tests;
