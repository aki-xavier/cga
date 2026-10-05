use super::*;

#[derive(Clone, Copy, Debug)]
pub struct TorusParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub major: f64,
    pub minor: f64,
    pub arc: f64,
}
