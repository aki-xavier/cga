use super::*;

#[derive(Clone, Debug)]
pub struct Object {
    pub base: Object3D,
    pub geometry: Geometry,
    pub material: Material,
    /// 分组 id（`SceneRun::groups` 的下标，0 = 未分组）。分组是缓存/失效与拾取
    /// 的元数据单元，**不是** z-index：对象前后关系永远由求交决定。
    pub group: u32,
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
            group: 0,
        }
    }

    /// 链式设定分组 id。
    pub fn with_group(mut self, group: u32) -> Object {
        self.group = group;
        self
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
