use super::*;

pub(crate) struct RastFace {
    pub(crate) a: [f64; 3],
    pub(crate) b: [f64; 3],
    pub(crate) c: [f64; 3],
    pub(crate) n: [f64; 3],
    pub(crate) uv: TriUvs,
    pub(crate) has_uv: bool,
}
