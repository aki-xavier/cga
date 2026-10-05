use super::*;

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
