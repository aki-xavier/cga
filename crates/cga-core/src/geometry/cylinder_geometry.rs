use super::*;

#[derive(Clone, Copy, Debug)]
pub struct CylinderGeometry {
    pub radius: f64,
    pub half: f64,
    pub blade: Cylinder,
}
impl CylinderGeometry {
    pub fn new(radius: f64, length: f64) -> CylinderGeometry {
        if radius <= 0.0 {
            panic!("cylinder radius must be > 0, got {}", radius);
        }
        if length > 0.0 {
            return CylinderGeometry {
                radius,
                half: length / 2.0,
                blade: Cylinder::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], radius),
            };
        }
        CylinderGeometry {
            radius,
            half: -1.0,
            blade: Cylinder::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], radius),
        }
    }
}
