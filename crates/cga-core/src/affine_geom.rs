// AffineGeometry: wraps any geometry with an invertible 3x3 linear block
// (scale/mirror/shear), applied by the ray-inverse transform.  The inner
// geometry stays in its local canonical form.

use crate::geometry::{Geometry, GeometryParams};
use crate::motors::{motor_identity, Mat3};
use crate::Multivector;

#[derive(Clone, Debug)]
pub struct AffineGeometry {
    // Single-element heap-backed slice.  A `&Geometry` field dangles for large
    // sumtypes (V's escape analysis misses heap allocation), so we store the
    // inner geometry in a slice whose backing array is always heap-allocated.
    pub inner: Vec<Geometry>,
    pub linear: Mat3,
    pub motor: Multivector,
}

// affine_geometry wraps `inner` with a linear block (identity linear = no-op).
pub fn affine_geometry(inner: Geometry, linear: Mat3) -> AffineGeometry {
    AffineGeometry {
        inner: vec![inner],
        linear,
        motor: motor_identity(),
    }
}

// transformed_geometry bakes a motor + linear into the geometry (CSG children,
// glTF node transforms).
pub fn transformed_geometry(inner: Geometry, motor: Multivector, linear: Mat3) -> AffineGeometry {
    AffineGeometry {
        inner: vec![inner],
        linear,
        motor,
    }
}

#[derive(Clone, Debug)]
pub struct AffineParams {
    pub inner: Box<GeometryParams>,
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
}

// affine_to_camera computes the inner (local) params plus the affine inverse.
