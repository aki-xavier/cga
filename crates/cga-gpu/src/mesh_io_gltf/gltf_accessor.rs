#[derive(Default, Debug)]
pub(crate) struct GltfAccessor {
    pub(crate) buffer_view: i32,
    pub(crate) component_type: i32,
    pub(crate) count: i32,
    pub(crate) typ: String,
    pub(crate) byte_offset: i32,
}
impl GltfAccessor {
    pub(crate) fn component_type_bytes(&self) -> i32 {
        match self.component_type {
            5121 => 1,
            5123 => 2,
            5125 => 4,
            5126 => 4,
            _ => panic!("unsupported componentType {}", self.component_type),
        }
    }
}
