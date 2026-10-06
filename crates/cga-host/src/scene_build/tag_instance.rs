use super::*;

#[derive(Clone, Debug)]
pub struct TagInstance {
    pub geo: Geometry,
    pub world: [f64; 16],
    pub rel: [f64; 16],
}
