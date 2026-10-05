use super::*;

#[derive(Default, Debug)]
pub(crate) struct GltfRoot {
    pub(crate) buffers: Vec<GltfBuffer>,
    pub(crate) buffer_views: Vec<GltfBufferView>,
    pub(crate) accessors: Vec<GltfAccessor>,
    pub(crate) meshes: Vec<GltfMesh>,
    pub(crate) nodes: Vec<GltfNode>,
    pub(crate) scenes: Vec<GltfScene>,
    pub(crate) scene: i32,
    pub(crate) materials: Vec<GltfMaterial>,
}
