module cga

// CGA Motor — the rigid-body transform (versor) in conformal space.
//
// A motor M (an even-grade versor) acts on any object O by the sandwich product
//   O' = M O M~   (M~ = reverse(M))
// Basic versors: rotor R = exp(-theta/2 B) (rotation), translator
// T = 1 - (t/2) v ^ einf (translation), motor M = T R.
//
// A motor is just a multivector with even-grade components; in this port the
// motor functions below operate on plain `Multivector` values.
import math

// Quaternion is an (w, x, y, z) quaternion (MJCF convention).
pub struct Quaternion {
pub:
	w f64
	x f64
	y f64
	z f64
}

// motor_identity returns the identity motor.
pub fn motor_identity() Multivector {
	return mv_scalar(1.0)
}

// motor_rotor builds a rotor for rotation by `angle` about `axis` (radians).
pub fn motor_rotor(axis [3]f64, angle f64) Multivector {
	mut ax := axis[0]
	mut ay := axis[1]
	mut az := axis[2]
	norm_ax := math.sqrt(ax * ax + ay * ay + az * az)
	if norm_ax < 1e-12 {
		return motor_identity()
	}
	ax /= norm_ax
	ay /= norm_ax
	az /= norm_ax
	half := angle / 2.0
	s := math.cos(half)
	sf := math.sin(half)
	mut vals := [32]f64{}
	vals[0] = s
	vals[6] = -sf * az // e12
	vals[7] = sf * ay // e13
	vals[10] = -sf * ax // e23
	return Multivector{vals}
}

// rotor_from_quaternion builds a rotor from an (w, x, y, z) quaternion.
pub fn rotor_from_quaternion(q Quaternion) Multivector {
	w := q.w
	x := q.x
	y := q.y
	z := q.z
	n := math.sqrt(w * w + x * x + y * y + z * z)
	if n < 1e-12 {
		return motor_identity()
	}
	angle := 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), w / n)
	return motor_rotor([x / n, y / n, z / n]!, angle)
}

// translator builds the translator T = 1 - (t ^ einf) / 2.
pub fn translator(displacement [3]f64) Multivector {
	tv := mv_vector(displacement[0], displacement[1], displacement[2], 0.0, 0.0)
	return mv_scalar(1.0).sub(tv.op(einf()).mul_scalar(0.5))
}

// --- 3x3 matrix helpers -----------------------------------------------------

type Mat3 = [3][3]f64

fn mat3_identity() Mat3 {
	mut m := Mat3{}
	m[0][0] = 1.0
	m[1][1] = 1.0
	m[2][2] = 1.0
	return m
}

fn mat3_new(r0 [3]f64, r1 [3]f64, r2 [3]f64) Mat3 {
	mut m := Mat3{}
	m[0] = r0
	m[1] = r1
	m[2] = r2
	return m
}

fn mat3_mul(a Mat3, b Mat3) Mat3 {
	mut r := Mat3{}
	for i in 0 .. 3 {
		for j in 0 .. 3 {
			mut s := 0.0
			for k in 0 .. 3 {
				s += a[i][k] * b[k][j]
			}
			r[i][j] = s
		}
	}
	return r
}

fn mat3_vec(a Mat3, v [3]f64) [3]f64 {
	mut r := [3]f64{}
	for i in 0 .. 3 {
		r[i] = a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2]
	}
	return r
}

fn mat3_add_scaled(a Mat3, s f64, b Mat3) Mat3 {
	mut r := Mat3{}
	for i in 0 .. 3 {
		for j in 0 .. 3 {
			r[i][j] = a[i][j] + s * b[i][j]
		}
	}
	return r
}

// matrix_to_quaternion converts a 3x3 rotation matrix to an (w, x, y, z)
// quaternion.
pub fn matrix_to_quaternion(m Mat3) Quaternion {
	trace := m[0][0] + m[1][1] + m[2][2]
	if trace > 0.0 {
		s := math.sqrt(trace + 1.0) * 2.0
		return Quaternion{
			w: 0.25 * s
			x: (m[2][1] - m[1][2]) / s
			y: (m[0][2] - m[2][0]) / s
			z: (m[1][0] - m[0][1]) / s
		}
	}
	if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
		s := math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
		return Quaternion{
			w: (m[2][1] - m[1][2]) / s
			x: 0.25 * s
			y: (m[0][1] + m[1][0]) / s
			z: (m[0][2] + m[2][0]) / s
		}
	}
	if m[1][1] > m[2][2] {
		s := math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
		return Quaternion{
			w: (m[0][2] - m[2][0]) / s
			x: (m[0][1] + m[1][0]) / s
			y: 0.25 * s
			z: (m[1][2] + m[2][1]) / s
		}
	}
	s := math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
	return Quaternion{
		w: (m[1][0] - m[0][1]) / s
		x: (m[0][2] + m[2][0]) / s
		y: (m[1][2] + m[2][1]) / s
		z: 0.25 * s
	}
}

// motor_from_matrix builds M = T(t) . R from a 3x3 rotation and translation.
pub fn motor_from_matrix(r Mat3, t [3]f64) Multivector {
	return translator(t).gp(rotor_from_quaternion(matrix_to_quaternion(r)))
}

// --- motor operations (on Multivector) --------------------------------------

// apply returns M . obj . M~ (the versor conjugation).  The result keeps the
// object's blade structure; call `.coords()` / `.euclidean_vector()` on it.
pub fn (m Multivector) apply(obj Multivector) Multivector {
	return m.gp(obj).gp(m.reverse())
}

// compose returns self . other (apply other first, then self).
pub fn (m Multivector) compose(other Multivector) Multivector {
	return m.gp(other)
}

// interpolate returns M(t) = self . exp(t . log(self^-1 . other)).
pub fn (m Multivector) interpolate(other Multivector, t f64) Multivector {
	delta := m.reverse().gp(other)
	return m.gp(motor_exp(delta.log(), t))
}

// to_matrix returns the equivalent 4x4 homogeneous transform [R|t], flattened
// row-major into 16 components (row r, col c is at index 4*r + c).
pub fn (m Multivector) to_matrix() [16]f64 {
	origin_t := m.apply(e0())
	tx := origin_t.values[1]
	ty := origin_t.values[2]
	tz := origin_t.values[3]

	px_t := m.apply(mv_vector(1.0, 0.0, 0.0, 1.0, 0.5))
	py_t := m.apply(mv_vector(0.0, 1.0, 0.0, 1.0, 0.5))
	pz_t := m.apply(mv_vector(0.0, 0.0, 1.0, 1.0, 0.5))

	return [
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
	]!
}

// velocity_bivector builds the twist bivector V = w + v ^ einf.
pub fn velocity_bivector(angular [3]f64, linear [3]f64) Multivector {
	wx := angular[0]
	wy := angular[1]
	wz := angular[2]
	mut vals := [32]f64{}
	vals[6] = wz // e12
	vals[7] = -wy // e13
	vals[10] = wx // e23
	rot := Multivector{vals}
	tv := mv_vector(linear[0], linear[1], linear[2], 0.0, 0.0)
	return rot.add(tv.op(einf()))
}

// motor_exp computes exp(-scale . B), where B is a bivector (half-twist
// convention B = 1/2 (w_bivector + v ^ einf)).
pub fn motor_exp(b Multivector, scale f64) Multivector {
	bv := b.mul_scalar(scale)
	vals := bv.values
	wx := vals[10]
	wy := -vals[7]
	wz := vals[6]
	vx := vals[9]
	vy := vals[12]
	vz := vals[14]

	w_bar := [2.0 * wx, 2.0 * wy, 2.0 * wz]!
	v_bar := [2.0 * vx, 2.0 * vy, 2.0 * vz]!
	theta := math.sqrt(w_bar[0] * w_bar[0] + w_bar[1] * w_bar[1] + w_bar[2] * w_bar[2])
	v_norm := math.sqrt(v_bar[0] * v_bar[0] + v_bar[1] * v_bar[1] + v_bar[2] * v_bar[2])
	if theta < 1e-12 {
		if v_norm < 1e-12 {
			return motor_identity()
		}
		// pure translation: Bv is nilpotent, series truncates
		return mv_scalar(1.0).sub(bv)
	}
	if v_norm < 1e-12 {
		// pure rotation through the origin
		return motor_rotor([w_bar[0] / theta, w_bar[1] / theta, w_bar[2] / theta]!, theta)
	}
	// general screw: Rodrigues + SO(3) left Jacobian
	bx := w_bar[0]
	by := w_bar[1]
	bz := w_bar[2]
	w := mat3_new([0.0, -bz, by]!, [bz, 0.0, -bx]!, [-by, bx, 0.0]!)
	ww := mat3_mul(w, w)
	theta2 := theta * theta
	sin_t := math.sin(theta)
	cos_t := math.cos(theta)
	a_r := sin_t / theta
	b_r := (1.0 - cos_t) / theta2
	a_v := (1.0 - cos_t) / theta2
	b_v := (theta - sin_t) / (theta2 * theta)
	eye := mat3_identity()
	r := mat3_add_scaled(mat3_add_scaled(eye, a_r, w), b_r, ww)
	v := mat3_add_scaled(mat3_add_scaled(eye, a_v, w), b_v, ww)
	t := mat3_vec(v, v_bar)
	return motor_from_matrix(r, t)
}

// log returns the bivector Bv with exp(-Bv) = self (SE(3) matrix logarithm).
pub fn (m Multivector) log() Multivector {
	t := m.to_matrix()
	r := mat3_new([t[0], t[1], t[2]]!, [t[4], t[5], t[6]]!, [t[8], t[9], t[10]]!)
	mut tv := [t[3], t[7], t[11]]!

	trace := r[0][0] + r[1][1] + r[2][2]
	mut cos_theta := (trace - 1.0) / 2.0
	if cos_theta > 1.0 {
		cos_theta = 1.0
	} else if cos_theta < -1.0 {
		cos_theta = -1.0
	}
	antisym := [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]]!
	sin_theta_abs := 0.5 * math.sqrt(antisym[0] * antisym[0] + antisym[1] * antisym[1] +
		antisym[2] * antisym[2])
	theta := math.atan2(sin_theta_abs, cos_theta)

	mut w_bar := [3]f64{}
	mut v_bar := [3]f64{}
	if theta < 1e-9 {
		// pure translation
		v_bar = tv
	} else {
		sin_theta := math.sin(theta)
		if theta < math.pi - 1e-3 {
			c := theta / (2.0 * sin_theta)
			w_bar = [c * antisym[0], c * antisym[1], c * antisym[2]]!
		} else {
			// theta ~ pi: recover axis from the symmetric part
			mut axis := [3]f64{}
			for i in 0 .. 3 {
				val := (r[i][i] + 1.0) / 2.0
				axis[i] = math.sqrt(if val > 0.0 { val } else { 0.0 })
			}
			mut ref := 0
			for i in 1 .. 3 {
				if math.abs(axis[i]) > math.abs(axis[ref]) {
					ref = i
				}
			}
			if ref == 0 {
				axis[1] = math.copysign(axis[1], r[0][1])
				axis[2] = math.copysign(axis[2], r[0][2])
			} else if ref == 1 {
				axis[0] = math.copysign(axis[0], r[0][1])
				axis[2] = math.copysign(axis[2], r[1][2])
			} else {
				axis[0] = math.copysign(axis[0], r[0][2])
				axis[1] = math.copysign(axis[1], r[1][2])
			}
			w_bar = [axis[0] * theta, axis[1] * theta, axis[2] * theta]!
		}
		// SO(3) left-Jacobian inverse
		bx := w_bar[0]
		by := w_bar[1]
		bz := w_bar[2]
		wxm := mat3_new([0.0, -bz, by]!, [bz, 0.0, -bx]!, [-by, bx, 0.0]!)
		wx2 := mat3_mul(wxm, wxm)
		theta2 := theta * theta
		coeff := 1.0 / theta2 - (1.0 + cos_theta) / (2.0 * theta * sin_theta)
		v_inv := mat3_add_scaled(mat3_add_scaled(mat3_identity(), -0.5, wxm), coeff, wx2)
		v_bar = mat3_vec(v_inv, tv)
	}
	return velocity_bivector([w_bar[0] / 2.0, w_bar[1] / 2.0, w_bar[2] / 2.0]!, [
		v_bar[0] / 2.0,
		v_bar[1] / 2.0,
		v_bar[2] / 2.0,
	]!)
}

// extract_velocity derives angular/linear velocity from two adjacent motors.
pub fn extract_velocity(m_curr Multivector, m_prev Multivector, dt f64) ([3]f64, [3]f64) {
	if dt <= 0.0 {
		panic('dt must be > 0, got ${dt}')
	}
	delta := m_prev.reverse().gp(m_curr)
	v := delta.log().mul_scalar(2.0 / dt)
	vals := v.values
	wx := vals[10]
	wy := -vals[7]
	wz := vals[6]
	vx := vals[9]
	vy := vals[12]
	vz := vals[14]
	return [wx, wy, wz]!, [vx, vy, vz]!
}
