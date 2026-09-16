// Affine geometry helpers: the ray-inverse-transform used by non-blade
// primitives (cone / torus / ellipsoid / cyclide / trimesh) and the general
// scale/mirror/shear wrapper.  Pure-CPU matrix helpers + the MLX `vecmat`
// row-vector × matrix contraction.

use crate::mesh_io::mat4_mul;
use crate::motors::{mat3_mul, mat3_new, motor_from_matrix, Mat3};
use crate::Multivector;

// mat3_transpose returns m^T.
pub fn mat3_transpose(m: Mat3) -> Mat3 {
    mat3_new(
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    )
}

// mat3_inv returns the inverse of a 3x3 matrix (adjugate / det).
pub fn mat3_inv(m: Mat3) -> Mat3 {
    let a = m[0][0];
    let b = m[0][1];
    let c = m[0][2];
    let d = m[1][0];
    let e = m[1][1];
    let f = m[1][2];
    let g = m[2][0];
    let h = m[2][1];
    let i = m[2][2];
    let ca = e * i - f * h;
    let cb = -(d * i - f * g);
    let cc = d * h - e * g;
    let det = a * ca + b * cb + c * cc;
    if det.abs() < 1e-15 {
        panic!("affine linear part is singular (det={})", det);
    }
    mat3_new(
        [ca / det, -(b * i - c * h) / det, (b * f - c * e) / det],
        [cb / det, (a * i - c * g) / det, -(a * f - c * d) / det],
        [cc / det, -(a * h - b * g) / det, (a * e - b * d) / det],
    )
}

// mat3_to_mat4 embeds a 3x3 linear block in a row-major 4x4 matrix.
pub fn mat3_to_mat4(l: Mat3) -> [f64; 16] {
    [
        l[0][0], l[0][1], l[0][2], 0.0, l[1][0], l[1][1], l[1][2], 0.0, l[2][0], l[2][1], l[2][2],
        0.0, 0.0, 0.0, 0.0, 1.0,
    ]
}

// mat3_to_mlx builds a (3,3) float32 array from a Mat3.

// vecmat computes v (...,3) · m (3,3) -> (...,3) (row-vector convention),
// preserving full float32 precision (mlx matmul drops small-matrix precision).

// affine_from_motor computes A = M·L and A^-1 = L^-1·M^-1, returning the 3x3
// inverse block a_inv3, the inverse translation t_inv and the full forward 4x4.
pub fn affine_from_motor(m: Multivector, linear: Mat3) -> (Mat3, [f64; 3], [f64; 16]) {
    let m4 = m.to_matrix();
    let minv4 = m.reverse().to_matrix();
    let linv = mat3_inv(linear);
    let a_fwd = mat4_mul(m4, mat3_to_mat4(linear));
    let a_inv = mat4_mul(mat3_to_mat4(linv), minv4);
    let a_inv3 = mat3_new(
        [a_inv[0], a_inv[1], a_inv[2]],
        [a_inv[4], a_inv[5], a_inv[6]],
        [a_inv[8], a_inv[9], a_inv[10]],
    );
    let t_inv = [a_inv[3], a_inv[7], a_inv[11]];
    (a_inv3, t_inv, a_fwd)
}

// affine_to_local transforms rays into the local canonical frame:
// o_l = o·a_inv3^T + t_inv, d_l = d·a_inv3^T, returns unit d and |d_l|.

// affine_normal maps a local normal to camera space and normalises.

// decompose_rigid factors a 4x4 affine into (motor, linear): A = motor . linear
// via Newton polar decomposition (reflections absorbed into linear).
pub fn decompose_rigid(m4: [f64; 16]) -> (Multivector, Mat3) {
    let b = mat3_new(
        [m4[0], m4[1], m4[2]],
        [m4[4], m4[5], m4[6]],
        [m4[8], m4[9], m4[10]],
    );
    let t = [m4[3], m4[7], m4[11]];
    mat3_inv(b); // singularity check (panics if det ~ 0)
    let mut x = b;
    for _ in 0..30 {
        let xit = mat3_transpose(mat3_inv(x));
        x = mat3_new(
            [
                0.5 * (x[0][0] + xit[0][0]),
                0.5 * (x[0][1] + xit[0][1]),
                0.5 * (x[0][2] + xit[0][2]),
            ],
            [
                0.5 * (x[1][0] + xit[1][0]),
                0.5 * (x[1][1] + xit[1][1]),
                0.5 * (x[1][2] + xit[1][2]),
            ],
            [
                0.5 * (x[2][0] + xit[2][0]),
                0.5 * (x[2][1] + xit[2][1]),
                0.5 * (x[2][2] + xit[2][2]),
            ],
        );
    }
    let mut q = x;
    let det_q = q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1])
        - q[0][1] * (q[1][0] * q[2][2] - q[1][2] * q[2][0])
        + q[0][2] * (q[1][0] * q[2][1] - q[1][1] * q[2][0]);
    let mut lq = mat3_mul(mat3_transpose(q), b);
    if det_q < 0.0 {
        let flip = mat3_new([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]);
        q = mat3_mul(q, flip);
        lq = mat3_mul(flip, lq);
    }
    (motor_from_matrix(q, t), lq)
}
