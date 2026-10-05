use super::*;

#[derive(Default, Debug)]
pub(crate) struct GltfMesh {
    pub(crate) primitives: Vec<GltfPrimitive>,
}
