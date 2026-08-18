module cga

// Materials and lights plus the batched Blinn-Phong shading kernel.
import mlx
import math

// --- Material ---------------------------------------------------------------

pub enum MaterialKind {
	standard
	basic
}

pub struct Material {
pub:
	kind       MaterialKind
	color      Color
	roughness  f64
	metalness  f64
	emissive   Color
	opacity    f64
	ior        f64
	absorption f64
pub mut:
	map ?Texture
}

// MaterialParams configures a standard material.
pub struct MaterialParams {
pub:
	color      Color
	roughness  f64
	metalness  f64
	emissive   Color
	opacity    f64
	ior        f64
	absorption f64
}

// standard_material builds a MeshStandardMaterial (Lambert + Blinn-Phong).
pub fn standard_material(p MaterialParams) Material {
	return Material{
		kind:       .standard
		color:      p.color
		roughness:  clamp01(p.roughness)
		metalness:  clamp01(p.metalness)
		emissive:   p.emissive
		opacity:    clamp01(p.opacity)
		ior:        math.max(1.0, p.ior)
		absorption: math.max(0.0, p.absorption)
	}
}

// basic_material builds a MeshBasicMaterial (unlit flat colour).
pub fn basic_material(color Color, opacity f64) Material {
	return Material{
		kind:       .basic
		color:      color
		roughness:  0.0
		metalness:  0.0
		emissive:   color_rgb(0.0, 0.0, 0.0)
		opacity:    clamp01(opacity)
		ior:        1.5 // matches the Python Material base default (used for Fresnel)
		absorption: 0.0
	}
}

// shade_params returns (emissive, diff, spec, expo) as linear-space f64 triples.
pub fn (m Material) shade_params() ([3]f64, [3]f64, [3]f64, f64) {
	crgb := m.color.rgb()
	if m.kind == .basic {
		zero := [0.0, 0.0, 0.0]!
		return crgb, zero, zero, 1.0
	}
	inv := 1.0 - m.metalness
	diff := [crgb[0] * inv, crgb[1] * inv, crgb[2] * inv]!
	spec := [inv + crgb[0] * m.metalness, inv + crgb[1] * m.metalness, inv + crgb[2] * m.metalness]!
	em := m.emissive.rgb()
	k := 1.0 - m.roughness
	expo := 4.0 + 196.0 * k * k
	return em, diff, spec, expo
}

// --- Lights -----------------------------------------------------------------

pub enum LightKind {
	ambient
	directional
	point
}

pub struct Light {
pub:
	kind      LightKind
	color     Color
	intensity f64
	direction [3]f64
	position  [3]f64
}

pub fn ambient_light(color Color, intensity f64) Light {
	return Light{
		kind:      .ambient
		color:     color
		intensity: intensity
	}
}

pub fn directional_light(color Color, intensity f64, direction [3]f64) Light {
	return Light{
		kind:      .directional
		color:     color
		intensity: intensity
		direction: vec3_unit(direction)
	}
}

pub fn point_light(color Color, intensity f64, position [3]f64) Light {
	return Light{
		kind:      .point
		color:     color
		intensity: intensity
		position:  position
	}
}

// light_to_camera conjugates a light into camera space (ambient unchanged).
pub fn light_to_camera(l Light, m Multivector) Light {
	match l.kind {
		.directional {
			d := m.apply(mv_vector(l.direction[0], l.direction[1], l.direction[2], 0.0, 0.0))
			return directional_light(l.color, l.intensity, dir3(d))
		}
		.point {
			c := m.apply(point(l.position[0], l.position[1], l.position[2])).coords()
			return point_light(l.color, l.intensity, c)
		}
		.ambient {
			return l
		}
	}
}

// light_direction_at returns (unit light direction (N,3), attenuation).
// Attenuation is a (N,1) array for point lights, a 0-d scalar for directional.
pub fn light_direction_at(l Light, p mlx.Array) (mlx.Array, mlx.Array) {
	match l.kind {
		.directional {
			ld := mlx.arr3v(l.direction).broadcast_to(p.shape())
			return ld, mlx.fs(l.intensity)
		}
		.point {
			lv := mlx.arr3v(l.position).broadcast_to(p.shape()).subtract(p)
			dist2 := lv.multiply(lv).sum_axis(-1, true)
			ld := lv.divide(dist2.sqrt())
			atten := mlx.s_rdiv(mlx.s_add(mlx.s_div(dist2, 8.0), 1.0), l.intensity)
			return ld, atten
		}
		.ambient {
			panic('ambient light is not part of the per-light loop')
		}
	}
}

// light_far returns the shadow-ray maximum distance (a 0-d scalar for
// directional lights, a (N,) array for point lights).
pub fn light_far(l Light, p mlx.Array) mlx.Array {
	if l.kind == .point {
		lv := mlx.arr3v(l.position).broadcast_to(p.shape()).subtract(p)
		return lv.multiply(lv).sum_axis(-1, false).sqrt()
	}
	return mlx.fs(math.inf(1))
}

// --- batched shading --------------------------------------------------------

// shade_batched computes per-pixel Blinn-Phong colour in linear space.
// emissive/diff/spec are (N,3); expo is (N,1); p/n/d are (N,3); vis is a list
// of per-light (N,) visibility arrays (or empty).
pub fn shade_batched(emissive mlx.Array, diff mlx.Array, spec mlx.Array, expo mlx.Array, p mlx.Array, n mlx.Array, d mlx.Array, lights []Light, ambient ?Light, vis []mlx.Array) mlx.Array {
	v := d.negative()
	mut out := emissive
	if amb := ambient {
		ambc := mlx.s_mul(mlx.arr3v(amb.color.rgb()), amb.intensity)
		out = out.add(ambc.broadcast_to(p.shape()).multiply(diff))
	}
	ndv := mlx.s_max(n.multiply(v).sum_axis(-1, true), 0.0)
	for i, light in lights {
		lc := mlx.arr3v(light.color.rgb())
		ld, atten := light_direction_at(light, p)
		nl := mlx.s_max(n.multiply(ld).sum_axis(-1, true), 0.0)
		mut h := ld.add(v)
		hn := h.multiply(h).sum_axis(-1, true).sqrt()
		h = h.divide(mlx.s_max(hn, 1e-12))
		spec_t := mlx.s_max(n.multiply(h).sum_axis(-1, true), 0.0).power(expo)
		mut contrib :=
			lc.multiply(atten).multiply(diff.multiply(nl).add(spec.multiply(spec_t).multiply(ndv)))
		if vis.len > 0 {
			contrib = contrib.multiply(vis[i].expand_dims(1))
		}
		out = out.add(contrib)
	}
	return out
}
