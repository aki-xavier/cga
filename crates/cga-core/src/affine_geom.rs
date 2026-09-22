use crate::geometry::{Geometry, GeometryParams};
use crate::motors::{motor_identity, Mat3};
use crate::Multivector;

#[derive(Clone, Debug)]
pub struct AffineGeometry {
    pub inner: Vec<Geometry>,
    pub linear: Mat3,
    pub motor: Multivector,
}

pub fn affine_geometry(inner: Geometry, linear: Mat3) -> AffineGeometry {
    AffineGeometry {
        inner: vec![inner],
        linear,
        motor: motor_identity(),
    }
}

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
