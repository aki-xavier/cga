use super::*;

#[derive(Default, Debug)]
pub(crate) struct GltfMaterial {
    pub(crate) pbr: GltfPbr,
    pub(crate) emissive_factor: Vec<f64>,
}
