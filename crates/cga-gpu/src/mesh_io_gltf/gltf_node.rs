#[derive(Default, Debug)]
pub(crate) struct GltfNode {
    pub(crate) mesh: Option<i32>,
    pub(crate) matrix: Vec<f64>,
    pub(crate) translation: Vec<f64>,
    pub(crate) rotation: Vec<f64>,
    pub(crate) scale: Vec<f64>,
    pub(crate) children: Vec<i32>,
}
