use super::*;

pub struct MeshParams {
    pub geometry: Geometry,
    pub material: Material,
    pub position: [f64; 3],
    pub rotation_axis: [f64; 3],
    pub rotation_angle: f64,
    pub motor: Option<Multivector>,
}
