use super::*;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct CircleParams {
    pub c: [f64; 3],
    pub n: [f64; 3],
    pub r: f64,
}

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
