// Materials and lights plus the batched Blinn-Phong shading kernel.

use crate::mlxops::*;
use crate::scene_graph::{color_rgb, dir3, vec3_unit, Color};
use crate::texture::Texture;
use cga_core::{clamp01, mv_vector, point, Multivector};
use mlx_rs::{ops, Array};

// --- Material ---------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MaterialKind {
    Standard,
    Basic,
}

#[derive(Clone, Debug)]
pub struct Material {
    pub kind: MaterialKind,
    pub color: Color,
    pub roughness: f64,
    pub metalness: f64,
    pub emissive: Color,
    pub opacity: f64,
    pub ior: f64,
    pub absorption: f64,
    pub map: Option<Texture>,
}

// MaterialParams configures a standard material.
pub struct MaterialParams {
    pub color: Color,
    pub roughness: f64,
    pub metalness: f64,
    pub emissive: Color,
    pub opacity: f64,
    pub ior: f64,
    pub absorption: f64,
}

// standard_material builds a MeshStandardMaterial (Lambert + Blinn-Phong).
pub fn standard_material(p: MaterialParams) -> Material {
    Material {
        kind: MaterialKind::Standard,
        color: p.color,
        roughness: clamp01(p.roughness),
        metalness: clamp01(p.metalness),
        emissive: p.emissive,
        opacity: clamp01(p.opacity),
        ior: p.ior.max(1.0),
        absorption: p.absorption.max(0.0),
        map: None,
    }
}

// basic_material builds a MeshBasicMaterial (unlit flat colour).
pub fn basic_material(color: Color, opacity: f64) -> Material {
    Material {
        kind: MaterialKind::Basic,
        color,
        roughness: 0.0,
        metalness: 0.0,
        emissive: color_rgb(0.0, 0.0, 0.0),
        opacity: clamp01(opacity),
        ior: 1.5, // matches the Python Material base default (used for Fresnel)
        absorption: 0.0,
        map: None,
    }
}

impl Material {
    // shade_params returns (emissive, diff, spec, expo) as linear-space f64 triples.
    pub fn shade_params(&self) -> ([f64; 3], [f64; 3], [f64; 3], f64) {
        let crgb = self.color.rgb();
        if self.kind == MaterialKind::Basic {
            let zero = [0.0, 0.0, 0.0];
            return (crgb, zero, zero, 1.0);
        }
        let inv = 1.0 - self.metalness;
        let diff = [crgb[0] * inv, crgb[1] * inv, crgb[2] * inv];
        let spec = [
            inv + crgb[0] * self.metalness,
            inv + crgb[1] * self.metalness,
            inv + crgb[2] * self.metalness,
        ];
        let em = self.emissive.rgb();
        let k = 1.0 - self.roughness;
        let expo = 4.0 + 196.0 * k * k;
        (em, diff, spec, expo)
    }
}

// --- Lights -----------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LightKind {
    Ambient,
    Directional,
    Point,
}

#[derive(Clone, Copy, Debug)]
pub struct Light {
    pub kind: LightKind,
    pub color: Color,
    pub intensity: f64,
    pub direction: [f64; 3],
    pub position: [f64; 3],
}

pub fn ambient_light(color: Color, intensity: f64) -> Light {
    Light {
        kind: LightKind::Ambient,
        color,
        intensity,
        direction: [0.0; 3],
        position: [0.0; 3],
    }
}

pub fn directional_light(color: Color, intensity: f64, direction: [f64; 3]) -> Light {
    Light {
        kind: LightKind::Directional,
        color,
        intensity,
        direction: vec3_unit(direction),
        position: [0.0; 3],
    }
}

pub fn point_light(color: Color, intensity: f64, position: [f64; 3]) -> Light {
    Light {
        kind: LightKind::Point,
        color,
        intensity,
        direction: [0.0; 3],
        position,
    }
}

// light_to_camera conjugates a light into camera space (ambient unchanged).
pub fn light_to_camera(l: Light, m: Multivector) -> Light {
    match l.kind {
        LightKind::Directional => {
            let d = m.apply(&mv_vector(
                l.direction[0],
                l.direction[1],
                l.direction[2],
                0.0,
                0.0,
            ));
            directional_light(l.color, l.intensity, dir3(d))
        }
        LightKind::Point => {
            let c = m
                .apply(&point(l.position[0], l.position[1], l.position[2]))
                .coords();
            point_light(l.color, l.intensity, c)
        }
        LightKind::Ambient => l,
    }
}

// light_direction_at returns (unit light direction (N,3), attenuation).
// Attenuation is a (N,1) array for point lights, a 0-d scalar for directional.
pub fn light_direction_at(l: Light, p: &Array) -> (Array, Array) {
    match l.kind {
        LightKind::Directional => {
            let ld = ck(ops::broadcast_to(arr3v(l.direction), p.shape()));
            (ld, fs(l.intensity))
        }
        LightKind::Point => {
            let lv = ck(ops::broadcast_to(arr3v(l.position), p.shape())).subtract(p);
            let lv = ck(lv);
            let dist2 = ck(ck(lv.multiply(&lv)).sum_axes(&[-1], true));
            let ld = ck(lv.divide(ck(dist2.sqrt())));
            let atten = s_rdiv(&s_add(&s_div(&dist2, 8.0), 1.0), l.intensity);
            (ld, atten)
        }
        LightKind::Ambient => {
            panic!("ambient light is not part of the per-light loop")
        }
    }
}

// light_far returns the shadow-ray maximum distance (a 0-d scalar for
// directional lights, a (N,) array for point lights).
pub fn light_far(l: Light, p: &Array) -> Array {
    if l.kind == LightKind::Point {
        let lv = ck(ops::broadcast_to(arr3v(l.position), p.shape())).subtract(p);
        let lv = ck(lv);
        return ck(ck(ck(lv.multiply(&lv)).sum_axes(&[-1], false)).sqrt());
    }
    fs(f64::INFINITY)
}

// --- batched shading --------------------------------------------------------

// shade_batched computes per-pixel Blinn-Phong colour in linear space.
// emissive/diff/spec are (N,3); expo is (N,1); p/n/d are (N,3); vis is a list
// of per-light (N,) visibility arrays (or empty).
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
        let (ld, atten) = light_direction_at(*light, p);
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
