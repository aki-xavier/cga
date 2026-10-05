use super::*;

#[derive(Clone, Debug)]
pub struct TrimeshParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub v0: Vec<[f64; 3]>,
    pub e1: Vec<[f64; 3]>,
    pub e2: Vec<[f64; 3]>,
    pub nrm: Vec<[f64; 3]>,
    pub lo: [f64; 3],
    pub hi: [f64; 3],
}
