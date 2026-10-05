use super::*;

#[derive(Clone, Debug)]
pub struct AffineParams {
    pub inner: Box<GeometryParams>,
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
}
