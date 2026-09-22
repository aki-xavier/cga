use cga_core::{motor_rotor, translator, Mat3, Multivector};

pub fn vec3_unit(a: [f64; 3]) -> [f64; 3] {
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    if n < 1e-12 {
        return [0.0, 0.0, 1.0];
    }
    [a[0] / n, a[1] / n, a[2] / n]
}

pub fn vec3_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn dir3(a: Multivector) -> [f64; 3] {
    a.euclidean_vector()
}

#[derive(Clone, Copy, Debug)]
pub struct Color {
    pub r: f64,
    pub g: f64,
    pub b: f64,
}

pub fn color_hex(c: i32) -> Color {
    Color {
        r: f64::from((c >> 16) & 0xFF) / 255.0,
        g: f64::from((c >> 8) & 0xFF) / 255.0,
        b: f64::from(c & 0xFF) / 255.0,
    }
}

pub fn color_rgb(r: f64, g: f64, b: f64) -> Color {
    Color { r, g, b }
}

pub fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        return c / 12.92;
    }
    ((c + 0.055) / 1.055).powf(2.4)
}

impl Color {
    pub fn rgb(&self) -> [f64; 3] {
        [
            srgb_to_linear(self.r),
            srgb_to_linear(self.g),
            srgb_to_linear(self.b),
        ]
    }
}

pub fn identity3() -> Mat3 {
    let mut m: Mat3 = [[0.0; 3]; 3];
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
    m
}

pub fn is_identity3(m: Mat3) -> bool {
    for (i, row) in m.iter().enumerate() {
        for (j, &v) in row.iter().enumerate() {
            let want = if i == j { 1.0 } else { 0.0 };
            if (v - want).abs() > 1e-12 {
                return false;
            }
        }
    }
    true
}

#[derive(Clone, Copy, Debug)]
pub struct Object3D {
    pub position: [f64; 3],
    pub rotation_axis: [f64; 3],
    pub rotation_angle: f64,
    pub motor_override: Option<Multivector>,
    pub linear: Mat3,
}

pub fn object3d(
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

impl Object3D {
    pub fn motor(&self) -> Multivector {
        if let Some(m) = self.motor_override {
            return m;
        }
        translator(self.position).gp(&motor_rotor(self.rotation_axis, self.rotation_angle))
    }
}
