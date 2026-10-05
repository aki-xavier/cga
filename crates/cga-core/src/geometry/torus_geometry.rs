#[derive(Clone, Copy, Debug)]
pub struct TorusGeometry {
    pub major: f64,
    pub minor: f64,
    pub arc: f64,
}
impl TorusGeometry {
    pub fn new(major: f64, minor: f64) -> TorusGeometry {
        if major <= 0.0 || minor <= 0.0 {
            panic!("torus radii must be > 0, got ({}, {})", major, minor);
        }
        TorusGeometry {
            major,
            minor,
            arc: std::f64::consts::TAU,
        }
    }

    pub fn tube(major: f64, minor: f64, arc: f64) -> TorusGeometry {
        if major <= 0.0 || minor <= 0.0 {
            panic!("tube radii must be > 0, got ({}, {})", major, minor);
        }
        if arc <= 0.0 || arc >= std::f64::consts::TAU {
            panic!("tube arc must be in (0, 2π), got {}", arc);
        }
        TorusGeometry { major, minor, arc }
    }
}
