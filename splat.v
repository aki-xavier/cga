module cga

// Gaussian splats attached to CGA surfaces (Route A): sample flattened 3D
// Gaussian ellipsoids on a Dupin cyclide, with each splat's local frame aligned
// to the surface (two tangent axes + the outward normal), and convert them to
// translucent EllipsoidGeometry meshes so the ray tracer can render them.
//
// Frame convention: the rotation matrix R has COLUMNS (t1, t2, n) — i.e. the
// rotor maps local e1 -> t1, e2 -> t2, e3 -> n (world surface normal).  The
// splat scale is (sigma_tangent, sigma_tangent, sigma_normal), so the thin
// axis is local e3.  Route B (true splatting) must reuse this convention.
import math

// Gaussian is one 3D Gaussian splat: centre, orientation (unit quaternion),
// per-axis standard deviations, opacity and colour.
pub struct Gaussian {
pub mut:
	mean    [3]f64
	quat    Quaternion
	scale   [3]f64
	opacity f64
	color   Color
}

// Gaussians is a splat set plus the template material used by to_meshes.
pub struct Gaussians {
	material Material
pub mut:
	splats []Gaussian
}

// sample_gaussians_on_cyclide samples an (n_u x n_v) grid over u, v in
// [0, 2*pi) and attaches one flattened Gaussian per grid point.  Samples with
// a degenerate (zero) surface gradient are skipped.  `mat` supplies the
// template colour/opacity (per-splat overridable afterwards).
pub fn sample_gaussians_on_cyclide(cyc DupinCyclide, n_u int, n_v int, sigma_tangent f64, sigma_normal f64, mat Material) Gaussians {
	if n_u <= 0 || n_v <= 0 {
		panic('need n_u, n_v > 0, got (${n_u}, ${n_v})')
	}
	if sigma_tangent <= 0.0 || sigma_normal <= 0.0 {
		panic('need sigmas > 0, got (${sigma_tangent}, ${sigma_normal})')
	}
	mut g := Gaussians{
		material: mat
		splats:   []Gaussian{cap: n_u * n_v}
	}
	tau := 2.0 * math.pi
	for i in 0 .. n_u {
		u := tau * f64(i) / f64(n_u)
		for j in 0 .. n_v {
			v := tau * f64(j) / f64(n_v)
			p := cyc.surface(u, v)
			gr := cyc.gradient(p[0], p[1], p[2])
			if vec3_dot(gr, gr) < 1e-24 {
				continue // degenerate normal (cusp); skip this sample
			}
			g.splats << gaussian_at(p, vec3_unit(gr), sigma_tangent, sigma_normal, mat)
		}
	}
	return g
}

// to_meshes converts every splat to a translucent EllipsoidGeometry mesh
// placed by a motor (translator x rotor-from-quaternion), reusing the template
// material with the per-splat colour and opacity.
pub fn (g Gaussians) to_meshes() []Mesh {
	mut out := []Mesh{cap: g.splats.len}
	for s in g.splats {
		m := translator(s.mean).gp(rotor_from_quaternion(s.quat))
		out << mesh(MeshParams{
			geometry:       ellipsoid_geometry(s.scale[0], s.scale[1], s.scale[2])
			material:       Material{
				...g.material
				opacity: s.opacity
				color:   s.color
			}
			position:       s.mean
			rotation_axis:  [0.0, 0.0, 1.0]!
			rotation_angle: 0.0
			motor:          m
		})
	}
	return out
}
