#[derive(Clone, Copy, Debug)]
pub struct EllipsoidGeometry {
    pub radii: [f64; 3],
}
impl EllipsoidGeometry {
    pub fn new(rx: f64, ry: f64, rz: f64) -> EllipsoidGeometry {
        if rx.min(ry.min(rz)) <= 0.0 {
            panic!("ellipsoid radii must be > 0, got ({}, {}, {})", rx, ry, rz);
        }
        EllipsoidGeometry {
            radii: [rx, ry, rz],
        }
    }
}
