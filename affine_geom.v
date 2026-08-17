module cga

// AffineGeometry: wraps any geometry with an invertible 3x3 linear block
// (scale/mirror/shear), applied by the ray-inverse transform.  The inner
// geometry stays in its local canonical form.

import mlx

pub struct AffineGeometry {
pub:
	// Single-element heap-backed slice.  A `&Geometry` field dangles for large
	// sumtypes (V's escape analysis misses heap allocation), so we store the
	// inner geometry in a slice whose backing array is always heap-allocated.
	inner  []Geometry
	linear Mat3
	motor  Multivector
}

// affine_geometry wraps `inner` with a linear block (identity linear = no-op).
pub fn affine_geometry(inner Geometry, linear Mat3) AffineGeometry {
	return AffineGeometry{
		inner: [inner]
		linear: linear
		motor: motor_identity()
	}
}

// transformed_geometry bakes a motor + linear into the geometry (CSG children,
// glTF node transforms).
pub fn transformed_geometry(inner Geometry, motor Multivector, linear Mat3) AffineGeometry {
	return AffineGeometry{
		inner: [inner]
		linear: linear
		motor: motor
	}
}

pub struct AffineParams {
pub:
	inner  GeometryParams
	a_inv3 Mat3
	t_inv  [3]f64
	a_fwd  [16]f64
}

// affine_to_camera computes the inner (local) params plus the affine inverse.
pub fn affine_to_camera(g AffineGeometry, m Multivector) AffineParams {
	ip := geom_to_camera(g.inner[0], motor_identity())
	full := m.gp(g.motor)
	ai, ti, af := affine_from_motor(full, g.linear)
	return AffineParams{
		inner: ip
		a_inv3: ai
		t_inv: ti
		a_fwd: af
	}
}

pub fn affine_intersect(p AffineParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	t_l, n_l, mask := geom_intersect(p.inner, o_l, d_u)
	t := t_l.divide(col(lam, 0))
	mut n := affine_normal(n_l, p.a_inv3)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn affine_shadow(p AffineParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	t_l, mask := geom_shadow(p.inner, o_l, d_u)
	return t_l.divide(col(lam, 0)), mask
}

pub fn affine_uv(p AffineParams, pos mlx.Array, n mlx.Array) mlx.Array {
	p_l := affine_point_to_local(p.a_inv3, p.t_inv, pos)
	return geom_uv(p.inner, p_l, n)
}

pub fn affine_crossings(p AffineParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, mut ns, valid := geom_crossings(p.inner, o_l, d_u)
	ns = affine_normal(ns, p.a_inv3)
	return ts.divide(col(lam, 0).expand_dims(1)), ns, valid
}

pub fn affine_contains(p AffineParams, pos mlx.Array) mlx.Array {
	p_l := affine_point_to_local(p.a_inv3, p.t_inv, pos)
	return geom_contains(p.inner, p_l)
}
