use super::*;

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
impl Material {
    pub fn standard(p: MaterialParams) -> Material {
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

    pub fn basic(color: Color, opacity: f64) -> Material {
        Material {
            kind: MaterialKind::Basic,
            color,
            roughness: 0.0,
            metalness: 0.0,
            emissive: Color::from_rgb(0.0, 0.0, 0.0),
            opacity: clamp01(opacity),
            ior: 1.5,
            absorption: 0.0,
            map: None,
        }
    }

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
