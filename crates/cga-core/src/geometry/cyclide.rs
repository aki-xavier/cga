use super::*;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct CyclideParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub a: f64,
    pub b: f64,
    pub d: f64,
    pub c: f64,
    pub shift: [f64; 3],
}

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
