use super::*;

#[derive(Clone, Debug)]
pub struct CsgParams {
    pub op: CsgOp,
    pub children: Vec<GeometryParams>,
}
