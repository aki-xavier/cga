use super::*;

#[derive(Clone, Copy, Debug)]
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
