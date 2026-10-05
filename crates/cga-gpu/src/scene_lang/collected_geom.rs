use super::*;

pub(crate) struct CollectedGeom {
    pub(crate) geo: Geometry,
    pub(crate) m4: [f64; 16],
}
