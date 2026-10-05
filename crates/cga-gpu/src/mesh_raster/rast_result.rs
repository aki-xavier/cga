use super::*;

#[derive(Clone, Debug)]
pub struct RastResult {
    pub depth: Array,
    pub color: Array,
    pub hit: Array,
}
