// CGA Motor — the rigid-body transform (versor) in conformal space.
//
// A motor M (an even-grade versor) acts on any object O by the sandwich product
//   O' = M O M~   (M~ = reverse(M))
// Basic versors: rotor R = exp(-theta/2 B) (rotation), translator
// T = 1 - (t/2) v ^ einf (translation), motor M = T R.
//
// A motor is just a multivector with even-grade components; in this port the
// motor functions below operate on plain `Multivector` values.

use crate::{e0, einf, mv_scalar, mv_vector, Multivector};
use std::f64::consts::PI;

/// Quaternion is an (w, x, y, z) quaternion (MJCF convention).
#[derive(Clone, Copy, Debug)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// motor_identity returns the identity motor.
pub fn motor_identity() -> Multivector {
    mv_scalar(1.0)
}

/// motor_rotor builds a rotor for rotation by `angle` about `axis` (radians).
pub fn motor_rotor(axis: [f64; 3], angle: f64) -> Multivector {
    let mut ax = axis[0];
    let mut ay = axis[1];
    let mut az = axis[2];
    let norm_ax = (ax * ax + ay * ay + az * az).sqrt();
    if norm_ax < 1e-12 {
        return motor_identity();
    }
    ax /= norm_ax;
    ay /= norm_ax;
    az /= norm_ax;
    let half = angle / 2.0;
    let s = half.cos();
    let sf = half.sin();
    let mut vals = [0.0; 32];
    vals[0] = s;
    vals[6] = -sf * az; // e12
    vals[7] = sf * ay; // e13
    vals[10] = -sf * ax; // e23
    Multivector { values: vals }
}

/// rotor_from_quaternion builds a rotor from an (w, x, y, z) quaternion.
/// The quaternion need not be unit: the axis is normalised and the angle uses
/// the scale-invariant form 2*atan2(|xyz|, w).
pub fn rotor_from_quaternion(q: Quaternion) -> Multivector {
    let w = q.w;
    let x = q.x;
    let y = q.y;
    let z = q.z;
    let n = (w * w + x * x + y * y + z * z).sqrt();
    if n < 1e-12 {
        return motor_identity();
    }
    let angle = 2.0 * (x * x + y * y + z * z).sqrt().atan2(w);
    motor_rotor([x / n, y / n, z / n], angle)
}

/// translator builds the translator T = 1 - (t ^ einf) / 2.
pub fn translator(displacement: [f64; 3]) -> Multivector {
    let tv = mv_vector(displacement[0], displacement[1], displacement[2], 0.0, 0.0);
    mv_scalar(1.0).sub(&tv.op(&einf()).mul_scalar(0.5))
}

// --- 3x3 matrix helpers -----------------------------------------------------

pub type Mat3 = [[f64; 3]; 3];

fn mat3_identity() -> Mat3 {
    let mut m = [[0.0; 3]; 3];
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
    m
}

pub fn mat3_new(r0: [f64; 3], r1: [f64; 3], r2: [f64; 3]) -> Mat3 {
    [r0, r1, r2]
}

// (pub(crate): shared between motors.rs and affine.rs.)
pub(crate) fn mat3_mul(a: Mat3, b: Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                s += a[i][k] * b[k][j];
            }
            r[i][j] = s;
        }
    }
    r
}

fn mat3_vec(a: Mat3, v: [f64; 3]) -> [f64; 3] {
    let mut r = [0.0; 3];
    for i in 0..3 {
        r[i] = a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2];
    }
    r
}

fn mat3_add_scaled(a: Mat3, s: f64, b: Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][j] + s * b[i][j];
        }
    }
    r
}

/// matrix_to_quaternion converts a 3x3 rotation matrix to an (w, x, y, z)
/// quaternion.
pub fn matrix_to_quaternion(m: Mat3) -> Quaternion {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        return Quaternion {
            w: 0.25 * s,
            x: (m[2][1] - m[1][2]) / s,
            y: (m[0][2] - m[2][0]) / s,
            z: (m[1][0] - m[0][1]) / s,
        };
    }
    if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        return Quaternion {
            w: (m[2][1] - m[1][2]) / s,
            x: 0.25 * s,
            y: (m[0][1] + m[1][0]) / s,
            z: (m[0][2] + m[2][0]) / s,
        };
    }
    if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        return Quaternion {
            w: (m[0][2] - m[2][0]) / s,
            x: (m[0][1] + m[1][0]) / s,
            y: 0.25 * s,
            z: (m[1][2] + m[2][1]) / s,
        };
    }
    let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
    Quaternion {
        w: (m[1][0] - m[0][1]) / s,
        x: (m[0][2] + m[2][0]) / s,
        y: (m[1][2] + m[2][1]) / s,
        z: 0.25 * s,
    }
}

/// motor_from_matrix builds M = T(t) . R from a 3x3 rotation and translation.
pub fn motor_from_matrix(r: Mat3, t: [f64; 3]) -> Multivector {
    translator(t).gp(&rotor_from_quaternion(matrix_to_quaternion(r)))
}

// --- motor operations (on Multivector) --------------------------------------

impl Multivector {
    /// apply returns M . obj . M~ (the versor conjugation).  The result keeps
    /// the object's blade structure; call `.coords()` / `.euclidean_vector()`
    /// on it.
    pub fn apply(&self, obj: &Multivector) -> Multivector {
        self.gp(obj).gp(&self.reverse())
    }

    /// compose returns self . other (apply other first, then self).
    pub fn compose(&self, other: &Multivector) -> Multivector {
        self.gp(other)
    }

    /// interpolate returns M(t) = self . exp(t . log(self^-1 . other)).
    pub fn interpolate(&self, other: &Multivector, t: f64) -> Multivector {
        let delta = self.reverse().gp(other);
        self.gp(&motor_exp(&delta.log(), t))
    }

    /// to_matrix returns the equivalent 4x4 homogeneous transform [R|t],
    /// flattened row-major into 16 components (row r, col c is at index
    /// 4*r + c).
    pub fn to_matrix(&self) -> [f64; 16] {
        let origin_t = self.apply(&e0());
        let tx = origin_t.values[1];
        let ty = origin_t.values[2];
        let tz = origin_t.values[3];

        let px_t = self.apply(&mv_vector(1.0, 0.0, 0.0, 1.0, 0.5));
        let py_t = self.apply(&mv_vector(0.0, 1.0, 0.0, 1.0, 0.5));
        let pz_t = self.apply(&mv_vector(0.0, 0.0, 1.0, 1.0, 0.5));

        [
            px_t.values[1] - tx,
            py_t.values[1] - tx,
            pz_t.values[1] - tx,
            tx,
            px_t.values[2] - ty,
            py_t.values[2] - ty,
            pz_t.values[2] - ty,
            ty,
            px_t.values[3] - tz,
            py_t.values[3] - tz,
            pz_t.values[3] - tz,
            tz,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    }

    /// log returns the bivector Bv with exp(-Bv) = self (SE(3) matrix
    /// logarithm).
    pub fn log(&self) -> Multivector {
        let t = self.to_matrix();
        let r = mat3_new([t[0], t[1], t[2]], [t[4], t[5], t[6]], [t[8], t[9], t[10]]);
        let tv = [t[3], t[7], t[11]];

        let trace = r[0][0] + r[1][1] + r[2][2];
        let cos_theta = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0);
        let antisym = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
        let sin_theta_abs = 0.5
            * (antisym[0] * antisym[0] + antisym[1] * antisym[1] + antisym[2] * antisym[2]).sqrt();
        let theta = sin_theta_abs.atan2(cos_theta);

        let mut w_bar = [0.0; 3];

        let v_bar: [f64; 3] = if theta < 1e-9 {
            // pure translation
            tv
        } else {
            let sin_theta = theta.sin();
            if theta < PI - 1e-3 {
                let c = theta / (2.0 * sin_theta);
                w_bar = [c * antisym[0], c * antisym[1], c * antisym[2]];
            } else {
                // theta ~ pi: recover axis from the symmetric part
                let mut axis = [0.0; 3];
                for i in 0..3 {
                    let val = (r[i][i] + 1.0) / 2.0;
                    axis[i] = if val > 0.0 { val.sqrt() } else { 0.0 };
                }
                let mut rf = 0;
                for i in 1..3 {
                    if axis[i].abs() > axis[rf].abs() {
                        rf = i;
                    }
                }
                if rf == 0 {
                    axis[1] = axis[1].copysign(r[0][1]);
                    axis[2] = axis[2].copysign(r[0][2]);
                } else if rf == 1 {
                    axis[0] = axis[0].copysign(r[0][1]);
                    axis[2] = axis[2].copysign(r[1][2]);
                } else {
                    axis[0] = axis[0].copysign(r[0][2]);
                    axis[1] = axis[1].copysign(r[1][2]);
                }
                w_bar = [axis[0] * theta, axis[1] * theta, axis[2] * theta];
            }
            // SO(3) left-Jacobian inverse
            let bx = w_bar[0];
            let by = w_bar[1];
            let bz = w_bar[2];
            let wxm = mat3_new([0.0, -bz, by], [bz, 0.0, -bx], [-by, bx, 0.0]);
            let wx2 = mat3_mul(wxm, wxm);
            let theta2 = theta * theta;
            let coeff = 1.0 / theta2 - (1.0 + cos_theta) / (2.0 * theta * sin_theta);
            let v_inv = mat3_add_scaled(mat3_add_scaled(mat3_identity(), -0.5, wxm), coeff, wx2);
            mat3_vec(v_inv, tv)
        };
        velocity_bivector(
            [w_bar[0] / 2.0, w_bar[1] / 2.0, w_bar[2] / 2.0],
            [v_bar[0] / 2.0, v_bar[1] / 2.0, v_bar[2] / 2.0],
        )
    }
}

/// velocity_bivector builds the twist bivector V = w + v ^ einf.
pub fn velocity_bivector(angular: [f64; 3], linear: [f64; 3]) -> Multivector {
    let wx = angular[0];
    let wy = angular[1];
    let wz = angular[2];
    let mut vals = [0.0; 32];
    vals[6] = wz; // e12
    vals[7] = -wy; // e13
    vals[10] = wx; // e23
    let rot = Multivector { values: vals };
    let tv = mv_vector(linear[0], linear[1], linear[2], 0.0, 0.0);
    rot.add(&tv.op(&einf()))
}

/// motor_exp computes exp(-scale . B), where B is a bivector (half-twist
/// convention B = 1/2 (w_bivector + v ^ einf)).
pub fn motor_exp(b: &Multivector, scale: f64) -> Multivector {
    let bv = b.mul_scalar(scale);
    let vals = bv.values;
    let wx = vals[10];
    let wy = -vals[7];
    let wz = vals[6];
    let vx = vals[9];
    let vy = vals[12];
    let vz = vals[14];

    let w_bar = [2.0 * wx, 2.0 * wy, 2.0 * wz];
    let v_bar = [2.0 * vx, 2.0 * vy, 2.0 * vz];
    let theta = (w_bar[0] * w_bar[0] + w_bar[1] * w_bar[1] + w_bar[2] * w_bar[2]).sqrt();
    let v_norm = (v_bar[0] * v_bar[0] + v_bar[1] * v_bar[1] + v_bar[2] * v_bar[2]).sqrt();
    if theta < 1e-12 {
        if v_norm < 1e-12 {
            return motor_identity();
        }
        // pure translation: Bv is nilpotent, series truncates
        return mv_scalar(1.0).sub(&bv);
    }
    if v_norm < 1e-12 {
        // pure rotation through the origin
        return motor_rotor(
            [w_bar[0] / theta, w_bar[1] / theta, w_bar[2] / theta],
            theta,
        );
    }
    // general screw: Rodrigues + SO(3) left Jacobian
    let bx = w_bar[0];
    let by = w_bar[1];
    let bz = w_bar[2];
    let w = mat3_new([0.0, -bz, by], [bz, 0.0, -bx], [-by, bx, 0.0]);
    let ww = mat3_mul(w, w);
    let theta2 = theta * theta;
    let sin_t = theta.sin();
    let cos_t = theta.cos();
    let a_r = sin_t / theta;
    let b_r = (1.0 - cos_t) / theta2;
    let a_v = (1.0 - cos_t) / theta2;
    let b_v = (theta - sin_t) / (theta2 * theta);
    let eye = mat3_identity();
    let r = mat3_add_scaled(mat3_add_scaled(eye, a_r, w), b_r, ww);
    let v = mat3_add_scaled(mat3_add_scaled(eye, a_v, w), b_v, ww);
    let t = mat3_vec(v, v_bar);
    motor_from_matrix(r, t)
}

/// extract_velocity derives angular/linear velocity from two adjacent motors.
pub fn extract_velocity(
    m_curr: &Multivector,
    m_prev: &Multivector,
    dt: f64,
) -> ([f64; 3], [f64; 3]) {
    if dt <= 0.0 {
        panic!("dt must be > 0, got {dt}");
    }
    let delta = m_prev.reverse().gp(m_curr);
    let v = delta.log().mul_scalar(2.0 / dt);
    let vals = v.values;
    let wx = vals[10];
    let wy = -vals[7];
    let wz = vals[6];
    let vx = vals[9];
    let vy = vals[12];
    let vz = vals[14];
    ([wx, wy, wz], [vx, vy, vz])
}

#[cfg(test)]
mod tests {
    use crate::*;
    use std::f64::consts::PI;

    fn point_mv(x: f64, y: f64, z: f64) -> Multivector {
        mv_vector(x, y, z, 1.0, 0.5 * (x * x + y * y + z * z))
    }

    #[test]
    fn test_translator() {
        let t = translator([1.0, 2.0, 3.0]);
        let q = t.apply(&point_mv(0.0, 0.0, 0.0)).coords();
        assert!(q[0] == 1.0 && q[1] == 2.0 && q[2] == 3.0);
    }

    #[test]
    fn test_rotor_z_quarter() {
        // rotate (1,0,0) by 90 deg about +z -> (0,1,0)
        let r = motor_rotor([0.0, 0.0, 1.0], PI / 2.0);
        let q = r.apply(&point_mv(1.0, 0.0, 0.0)).coords();
        assert!((q[0] - 0.0).abs() < 1e-6);
        assert!((q[1] - 1.0).abs() < 1e-6);
        assert!((q[2] - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_to_matrix_roundtrip() {
        let m = translator([1.0, 2.0, 3.0]).gp(&motor_rotor([0.0, 1.0, 0.0], PI / 2.0));
        let mtx = m.to_matrix();
        assert!((mtx[3] - 1.0).abs() < 1e-6);
        assert!((mtx[7] - 2.0).abs() < 1e-6);
        assert!((mtx[11] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_exp_log_roundtrip() {
        let m = translator([0.3, -0.4, 0.5]).gp(&motor_rotor([0.1, 0.6, -0.2], 0.7));
        let b = m.log();
        let m2 = motor_exp(&b, 1.0);
        assert!(m.approx_eq(&m2));
    }

    #[test]
    fn test_interpolate_endpoints() {
        let a = motor_identity();
        let b = translator([1.0, 0.0, 0.0]).gp(&motor_rotor([0.0, 0.0, 1.0], 0.5));
        assert!(a.interpolate(&b, 0.0).approx_eq(&a));
        assert!(a.interpolate(&b, 1.0).approx_eq(&b));
    }

    #[test]
    fn test_rotor_from_quaternion_scaled() {
        // 2x the quaternion for 60 deg about +z must give the same rotor as the
        // unit quaternion (scale-invariant angle): e1 -> (cos60, sin60, 0)
        let c = (PI / 6.0).cos(); // half-angle 30 deg
        let s = (PI / 6.0).sin();
        let q = Quaternion {
            w: 2.0 * c,
            x: 0.0,
            y: 0.0,
            z: 2.0 * s,
        };
        let p = rotor_from_quaternion(q)
            .apply(&point_mv(1.0, 0.0, 0.0))
            .coords();
        assert!((p[0] - 0.5).abs() < 1e-9);
        assert!((p[1] - 3.0_f64.sqrt() / 2.0).abs() < 1e-9);
        assert!(p[2].abs() < 1e-9);
        // and the axis stays +z (not distorted by the scale)
        let p2 = rotor_from_quaternion(q)
            .apply(&point_mv(0.0, 0.0, 1.0))
            .coords();
        assert!((p2[2] - 1.0).abs() < 1e-9);
        assert!(p2[0].abs() < 1e-9);
        assert!(p2[1].abs() < 1e-9);
    }

    #[test]
    fn test_extract_velocity() {
        // pure rotation about z at 1 rad/s
        let dt = 0.1;
        let m0 = motor_rotor([0.0, 0.0, 1.0], 0.0);
        let m1 = motor_rotor([0.0, 0.0, 1.0], 0.1);
        let (ang, lin) = extract_velocity(&m1, &m0, dt);
        assert!((ang[2] - 1.0).abs() < 1e-3);
        assert!(lin[0].abs() < 1e-6);
        assert!(lin[1].abs() < 1e-6);
        assert!(lin[2].abs() < 1e-6);
    }
}
