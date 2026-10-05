#[derive(Clone, Copy, Debug)]
pub struct CyclideGeometry {
    pub a: f64,
    pub b: f64,
    pub d: f64,
    pub shift: [f64; 3],
}
impl CyclideGeometry {
    pub fn new(a: f64, b: f64, d: f64, shift: [f64; 3]) -> CyclideGeometry {
        if !(a > b && b > 0.0) {
            panic!("cyclide needs a > b > 0, got ({}, {})", a, b);
        }
        if d <= 0.0 {
            panic!("cyclide needs d > 0, got {}", d);
        }
        CyclideGeometry { a, b, d, shift }
    }
}
