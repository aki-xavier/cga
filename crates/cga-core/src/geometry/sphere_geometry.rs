use super::*;

#[derive(Clone, Copy, Debug)]
pub struct SphereGeometry {
    pub radius: f64,
    pub blade: Multivector,
}
impl SphereGeometry {
    pub fn new(radius: f64) -> SphereGeometry {
        if radius <= 0.0 {
            panic!("sphere radius must be > 0, got {}", radius);
        }
        SphereGeometry {
            radius,
            blade: Multivector::sphere([0.0, 0.0, 0.0], radius),
        }
    }
}
