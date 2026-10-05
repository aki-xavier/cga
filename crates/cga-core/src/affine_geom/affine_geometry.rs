use super::*;

#[derive(Clone, Debug)]
pub struct AffineGeometry {
    pub inner: Vec<Geometry>,
    pub linear: Mat3,
    pub motor: Multivector,
}
impl AffineGeometry {
    pub fn new(inner: Geometry, linear: Mat3) -> AffineGeometry {
        AffineGeometry {
            inner: vec![inner],
            linear,
            motor: Multivector::identity(),
        }
    }

    pub fn with_motor(inner: Geometry, motor: Multivector, linear: Mat3) -> AffineGeometry {
        AffineGeometry {
            inner: vec![inner],
            linear,
            motor,
        }
    }
}
