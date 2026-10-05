use super::*;

#[derive(Clone, Copy, Debug)]
pub struct CircleGeometry {
    pub radius: f64,
    pub blade: Multivector,
}
impl CircleGeometry {
    pub fn new(radius: f64) -> CircleGeometry {
        if radius <= 0.0 {
            panic!("circle radius must be > 0, got {}", radius);
        }
        CircleGeometry {
            radius,
            blade: Multivector::circle([0.0, 0.0, 0.0], radius, [0.0, 0.0, 1.0]),
        }
    }
}
