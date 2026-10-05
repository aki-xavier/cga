use super::*;

#[derive(Clone, Debug)]
pub(crate) struct Inst {
    pub(crate) geo: Geometry,
    pub(crate) world: [f64; 16],
    pub(crate) rel: [f64; 16],
}
