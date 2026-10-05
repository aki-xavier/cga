#[derive(Clone, Debug)]
pub struct GltfMeshIn {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[i32; 3]>,
    pub transform: Option<[f64; 16]>,
    pub color: Option<[f64; 3]>,
}
