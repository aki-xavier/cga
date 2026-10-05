use crate::mlxops::*;
use crate::scene_graph::{vec3_unit, Color};
use crate::texture::Texture;
use cga_core::{clamp01, Multivector};
use mlx_rs::{ops, Array};

pub mod material_kind;
pub use self::material_kind::*;
pub mod material;
pub use self::material::*;
pub mod material_params;
pub use self::material_params::*;
pub mod light_kind;
pub use self::light_kind::*;
pub mod light;
pub use self::light::*;

#[allow(clippy::too_many_arguments)]
pub fn shade_batched(
    emissive: &Array,
    diff: &Array,
    spec: &Array,
    expo: &Array,
    p: &Array,
    n: &Array,
    d: &Array,
    lights: &[Light],
    ambient: Option<Light>,
    vis: &[Array],
) -> Array {
    let v = ck(d.negative());
    let mut out = emissive.clone();
    if let Some(amb) = ambient {
        let ambc = s_mul(&arr3v(amb.color.rgb()), amb.intensity);
        out = ck(out.add(ck(ck(ops::broadcast_to(&ambc, p.shape())).multiply(diff))));
    }
    let ndv = s_max(&ck(ck(n.multiply(&v)).sum_axes(&[-1], true)), 0.0);
    for (i, light) in lights.iter().enumerate() {
        let lc = arr3v(light.color.rgb());
        let (ld, atten) = light.direction_at(p);
        let nl = s_max(&ck(ck(n.multiply(&ld)).sum_axes(&[-1], true)), 0.0);
        let mut h = ck(ld.add(&v));
        let hn = ck(ck(ck(h.multiply(&h)).sum_axes(&[-1], true)).sqrt());
        h = ck(h.divide(s_max(&hn, 1e-12)));
        let spec_t = ck(s_max(&ck(ck(n.multiply(&h)).sum_axes(&[-1], true)), 0.0).power(expo));
        let mut contrib = ck(ck(lc.multiply(&atten)).multiply(ck(
            ck(diff.multiply(&nl)).add(ck(ck(spec.multiply(&spec_t)).multiply(&ndv)))
        )));
        if !vis.is_empty() {
            contrib = ck(contrib.multiply(ck(vis[i].expand_dims(1))));
        }
        out = ck(out.add(&contrib));
    }
    out
}
