// Scene-graph layer: Vec3 helpers, Color, Object3D, Mesh, Scene,
// PerspectiveCamera and OrbitControls (the three.js-style surface).

use cga_core::{motor_rotor, translator, Mat3, Multivector};

// --- Vec3 -------------------------------------------------------------------

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

// dir3 returns the euclidean (e1,e2,e3) part of a grade-1 vector.
pub fn dir3(a: Multivector) -> [f64; 3] {
    a.euclidean_vector()
}

// --- Color ------------------------------------------------------------------

// Color is an sRGB colour (0-1 encoded components).  rgb() returns the linear
// components used for shading; the Renderer re-encodes on output.
#[derive(Clone, Copy, Debug)]
pub struct Color {
    pub r: f64,
    pub g: f64,
    pub b: f64,
}

// color_hex builds a Color from a 0xRRGGBB integer.
pub fn color_hex(c: i32) -> Color {
    Color {
        r: f64::from((c >> 16) & 0xFF) / 255.0,
        g: f64::from((c >> 8) & 0xFF) / 255.0,
        b: f64::from(c & 0xFF) / 255.0,
    }
}

// color_rgb builds a Color from three sRGB-encoded 0-1 components.
pub fn color_rgb(r: f64, g: f64, b: f64) -> Color {
    Color { r, g, b }
}

// srgb_to_linear decodes one sRGB component to linear (IEC 61966-2-1).
pub fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        return c / 12.92;
    }
    ((c + 0.055) / 1.055).powf(2.4)
}

impl Color {
    // rgb returns the linear-space components.
    pub fn rgb(&self) -> [f64; 3] {
        [
            srgb_to_linear(self.r),
            srgb_to_linear(self.g),
            srgb_to_linear(self.b),
        ]
    }
}

// --- Object3D ---------------------------------------------------------------

// identity3 is the 3x3 identity matrix.
pub fn identity3() -> Mat3 {
    let mut m: Mat3 = [[0.0; 3]; 3];
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
    m
}

// is_identity3 reports whether a 3x3 matrix is the identity (within tolerance).
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

// object3d builds an Object3D node (local pose = Motor, optional linear block).
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
    // motor returns the local pose motor (full motor if given, else T . R).
    pub fn motor(&self) -> Multivector {
        if let Some(m) = self.motor_override {
            return m;
        }
        translator(self.position).gp(&motor_rotor(self.rotation_axis, self.rotation_angle))
    }
}
