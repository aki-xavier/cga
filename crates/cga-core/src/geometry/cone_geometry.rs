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
