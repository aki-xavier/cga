use super::*;

#[derive(Clone, Debug)]
pub struct Mesh {
    pub base: Object3D,
    pub geometry: Geometry,
    pub material: Material,
}
impl Mesh {
    pub fn new(p: MeshParams) -> Mesh {
        Mesh {
            base: Object3D::new(
                p.position,
                p.rotation_axis,
                p.rotation_angle,
                p.motor,
                identity3(),
            ),
            geometry: p.geometry,
            material: p.material,
        }
    }
}
impl std::ops::Deref for Mesh {
    type Target = Object3D;

    fn deref(&self) -> &Object3D {
        &self.base
    }
}
impl std::ops::DerefMut for Mesh {
    fn deref_mut(&mut self) -> &mut Object3D {
        &mut self.base
    }
}
