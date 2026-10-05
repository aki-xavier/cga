use super::*;

#[derive(Clone, Copy, Debug)]
pub struct ConeParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub r: f64,
    pub h: f64,
}
