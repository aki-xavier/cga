use super::*;

#[derive(Clone, Debug)]
pub struct GeomVal {
    pub geo: Geometry,
    pub m4: [f64; 16],
}
