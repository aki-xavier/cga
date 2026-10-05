use super::*;

#[derive(Clone, Copy, Debug)]
pub struct PlaneGeometry {
    pub blade: Multivector,
}
impl PlaneGeometry {
    pub fn new(normal: [f64; 3], distance: f64) -> PlaneGeometry {
        PlaneGeometry {
            blade: Multivector::plane(normal, distance),
        }
    }
}
