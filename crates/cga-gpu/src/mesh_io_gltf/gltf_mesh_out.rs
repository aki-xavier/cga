use super::*;

#[derive(Clone, Debug)]
pub struct GltfMeshOut {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[i32; 3]>,
    pub world: [f64; 16],
    pub uv: Vec<[f64; 2]>,
    pub color: [f64; 3],
    pub metalness: f64,
    pub roughness: f64,
    pub emissive: [f64; 3],
}
impl GltfMeshOut {
    pub fn material(&self) -> Material {
        Material::standard(MaterialParams {
            color: Color::from_rgb(self.color[0], self.color[1], self.color[2]),
            roughness: self.roughness,
            metalness: self.metalness,
            emissive: Color::from_rgb(self.emissive[0], self.emissive[1], self.emissive[2]),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        })
    }
}
