#[derive(Default, Debug)]
pub(crate) struct GltfPbr {
    pub(crate) base_color_factor: Vec<f64>,
    pub(crate) metallic_factor: f64,
    pub(crate) roughness_factor: f64,
}
