use cga_core::{Mat3, Multivector};

pub mod color;
pub use self::color::*;
pub mod object3d;
pub use self::object3d::*;

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

pub fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        return c / 12.92;
    }
    ((c + 0.055) / 1.055).powf(2.4)
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
