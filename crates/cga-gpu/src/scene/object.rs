use super::*;

#[derive(Clone, Debug)]
pub struct Object {
    pub base: Object3D,
    pub geometry: Geometry,
    pub material: Material,
}
impl Object {
    pub fn new(p: ObjectParams) -> Object {
        Object {
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
impl std::ops::Deref for Object {
    type Target = Object3D;

    fn deref(&self) -> &Object3D {
        &self.base
    }
}
impl std::ops::DerefMut for Object {
    fn deref_mut(&mut self) -> &mut Object3D {
        &mut self.base
    }
}
