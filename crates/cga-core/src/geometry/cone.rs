use super::*;

#[derive(Clone, Copy, Debug)]
pub struct ConeParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub r: f64,
    pub h: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ConeGeometry {
    pub radius: f64,
    pub height: f64,
}
impl ConeGeometry {
    pub fn new(radius: f64, height: f64) -> ConeGeometry {
        if radius <= 0.0 || height <= 0.0 {
            panic!(
                "cone radius/height must be > 0, got ({}, {})",
                radius, height
            );
        }
        ConeGeometry { radius, height }
    }
}
