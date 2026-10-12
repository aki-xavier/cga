use super::*;
use cga_core::{identity3, Mat3};

#[derive(Clone, Copy, Debug)]
pub struct Object3D {
    pub position: [f64; 3],
    pub rotation_axis: [f64; 3],
    pub rotation_angle: f64,
    pub motor_override: Option<Multivector>,
    pub linear: Mat3,
}
impl Object3D {
    pub fn new(
        position: [f64; 3],
        rotation_axis: [f64; 3],
        rotation_angle: f64,
        motor: Option<Multivector>,
        linear: Mat3,
    ) -> Object3D {
        let mut o = Object3D {
            position: [0.0; 3],
            rotation_axis: [0.0; 3],
            rotation_angle: 0.0,
            motor_override: None,
            linear,
        };
        if let Some(m) = motor {
            o.motor_override = Some(m);
            let mtx = m.to_matrix();
            o.position = [mtx[3], mtx[7], mtx[11]];
            o.rotation_axis = [0.0, 0.0, 1.0];
            o.rotation_angle = 0.0;
        } else {
            o.position = position;
            o.rotation_axis = rotation_axis;
            o.rotation_angle = rotation_angle;
        }
        o
    }

    pub fn motor(&self) -> Multivector {
        if let Some(m) = self.motor_override {
            return m;
        }
        Multivector::translator(self.position)
            .gp(&Multivector::rotor(self.rotation_axis, self.rotation_angle))
    }
}

pub struct ObjectParams {
    pub geometry: Geometry,
    pub material: Material,
    pub position: [f64; 3],
    pub rotation_axis: [f64; 3],
    pub rotation_angle: f64,
    pub motor: Option<Multivector>,
}

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
