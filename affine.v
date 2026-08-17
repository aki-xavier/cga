module cga

// Affine geometry helpers: the ray-inverse-transform used by non-blade
// primitives (cone / torus / ellipsoid / cyclide / trimesh) and the general
// scale/mirror/shear wrapper.  Pure-CPU matrix helpers + the MLX `vecmat`
// row-vector × matrix contraction.
import mlx
import math

// mat3_transpose returns m^T.
pub fn mat3_transpose(m Mat3) Mat3 {
	return mat3_new([m[0][0], m[1][0], m[2][0]]!, [m[0][1], m[1][1], m[2][1]]!, [m[0][2], m[1][2],
		m[2][2]]!)
}

// mat3_inv returns the inverse of a 3x3 matrix (adjugate / det).
pub fn mat3_inv(m Mat3) Mat3 {
	a := m[0][0]
	b := m[0][1]
	c := m[0][2]
	d := m[1][0]
	e := m[1][1]
	f := m[1][2]
	g := m[2][0]
	h := m[2][1]
	i := m[2][2]
	ca := e * i - f * h
	cb := -(d * i - f * g)
	cc := d * h - e * g
	det := a * ca + b * cb + c * cc
	if math.abs(det) < 1e-15 {
		panic('affine linear part is singular (det=${det})')
	}
	return mat3_new([ca / det, -(b * i - c * h) / det, (b * f - c * e) / det]!, [
		cb / det,
		(a * i - c * g) / det,
		-(a * f - c * d) / det,
	]!, [cc / det, -(a * h - b * g) / det, (a * e - b * d) / det]!)
}

// mat3_to_mat4 embeds a 3x3 linear block in a row-major 4x4 matrix.
pub fn mat3_to_mat4(l Mat3) [16]f64 {
	return [l[0][0], l[0][1], l[0][2], 0.0, l[1][0], l[1][1], l[1][2], 0.0, l[2][0], l[2][1], l[2][2],
		0.0, 0.0, 0.0, 0.0, 1.0]!
}

// mat3_to_mlx builds a (3,3) float32 array from a Mat3.
pub fn mat3_to_mlx(m Mat3) mlx.Array {
	return mlx.array_f32([f32(m[0][0]), f32(m[0][1]), f32(m[0][2]), f32(m[1][0]), f32(m[1][1]),
		f32(m[1][2]), f32(m[2][0]), f32(m[2][1]), f32(m[2][2])], [3, 3])
}

// vecmat computes v (...,3) · m (3,3) -> (...,3) (row-vector convention),
// preserving full float32 precision (mlx matmul drops small-matrix precision).
pub fn vecmat(v mlx.Array, m Mat3) mlx.Array {
	mm := mat3_to_mlx(m)
	return v.expand_dims(-1).multiply(mm).sum_axis(-2, false)
}

// affine_from_motor computes A = M·L and A^-1 = L^-1·M^-1, returning the 3x3
// inverse block a_inv3, the inverse translation t_inv and the full forward 4x4.
pub fn affine_from_motor(m Multivector, linear Mat3) (Mat3, [3]f64, [16]f64) {
	m4 := m.to_matrix()
	minv4 := m.reverse().to_matrix()
	linv := mat3_inv(linear)
	a_fwd := mat4_mul(m4, mat3_to_mat4(linear))
	a_inv := mat4_mul(mat3_to_mat4(linv), minv4)
	a_inv3 := mat3_new([a_inv[0], a_inv[1], a_inv[2]]!, [a_inv[4], a_inv[5], a_inv[6]]!, [
		a_inv[8],
		a_inv[9],
		a_inv[10],
	]!)
	t_inv := [a_inv[3], a_inv[7], a_inv[11]]!
	return a_inv3, t_inv, a_fwd
}

// affine_to_local transforms rays into the local canonical frame:
// o_l = o·a_inv3^T + t_inv, d_l = d·a_inv3^T, returns unit d and |d_l|.
pub fn affine_to_local(a_inv3 Mat3, t_inv [3]f64, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	a3t := mat3_transpose(a_inv3)
	t3 := arr3v(t_inv)
	o_l := vecmat(o, a3t).add(t3)
	d_l := vecmat(d, a3t)
	mut lam := d_l.multiply(d_l).sum_axis(-1, true).sqrt()
	lam = mlx.where(s_gt(lam, 1e-12), lam, mlx.ones_like(lam))
	return o_l, d_l.divide(lam), lam
}

// affine_normal maps a local normal to camera space and normalises.
pub fn affine_normal(n_l mlx.Array, a_inv3 Mat3) mlx.Array {
	n := vecmat(n_l, a_inv3)
	norm := n.multiply(n).sum_axis(-1, true).sqrt()
	return n.divide(mlx.where(s_gt(norm, 1e-12), norm, mlx.ones_like(norm)))
}

// decompose_rigid factors a 4x4 affine into (motor, linear): A = motor . linear
// via Newton polar decomposition (reflections absorbed into linear).
pub fn decompose_rigid(m4 [16]f64) (Multivector, Mat3) {
	mut b := mat3_new([m4[0], m4[1], m4[2]]!, [m4[4], m4[5], m4[6]]!, [m4[8], m4[9], m4[10]]!)
	t := [m4[3], m4[7], m4[11]]!
	mat3_inv(b) // singularity check (panics if det ~ 0)
	mut x := b
	for _ in 0 .. 30 {
		xit := mat3_transpose(mat3_inv(x))
		x = mat3_new([0.5 * (x[0][0] + xit[0][0]), 0.5 * (x[0][1] + xit[0][1]),
			0.5 * (x[0][2] + xit[0][2])]!, [0.5 * (x[1][0] + xit[1][0]), 0.5 * (x[1][1] + xit[1][1]),
			0.5 * (x[1][2] + xit[1][2])]!, [0.5 * (x[2][0] + xit[2][0]), 0.5 * (x[2][1] + xit[2][1]),
			0.5 * (x[2][2] + xit[2][2])]!)
	}
	mut q := x
	det_q := q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1]) -
		q[0][1] * (q[1][0] * q[2][2] - q[1][2] * q[2][0]) +
		q[0][2] * (q[1][0] * q[2][1] - q[1][1] * q[2][0])
	mut lq := mat3_mul(mat3_transpose(q), b)
	if det_q < 0.0 {
		flip := mat3_new([1.0, 0.0, 0.0]!, [0.0, 1.0, 0.0]!, [0.0, 0.0, -1.0]!)
		q = mat3_mul(q, flip)
		lq = mat3_mul(flip, lq)
	}
	return motor_from_matrix(q, t), lq
}
