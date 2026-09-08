module cga

// AffineGeometry: wraps any geometry with an invertible 3x3 linear block
// (scale/mirror/shear), applied by the ray-inverse transform.  The inner
// geometry stays in its local canonical form.

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
		inner:  [inner]
		linear: linear
		motor:  motor_identity()
	}
}

// transformed_geometry bakes a motor + linear into the geometry (CSG children,
// glTF node transforms).
pub fn transformed_geometry(inner Geometry, motor Multivector, linear Mat3) AffineGeometry {
	return AffineGeometry{
		inner:  [inner]
		linear: linear
		motor:  motor
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





