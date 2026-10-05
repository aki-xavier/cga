#[derive(Default, Debug)]
pub(crate) struct GltfPrimitive {
    pub(crate) attributes: std::collections::HashMap<String, i32>,
    pub(crate) indices: Option<i32>,
    pub(crate) mode: i32,
    pub(crate) material: Option<i32>,
}
