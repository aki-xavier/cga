#[derive(Default, Debug)]
pub(crate) struct GltfBufferView {
    pub(crate) buffer: i32,
    pub(crate) byte_offset: i32,
    #[allow(dead_code)]
    pub(crate) byte_length: i32,
    pub(crate) byte_stride: i32,
}
