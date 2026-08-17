module cga

import math

fn point_mv(x f64, y f64, z f64) Multivector {
	return mv_vector(x, y, z, 1.0, 0.5 * (x * x + y * y + z * z))
}

fn test_translator() {
	t := translator([1.0, 2.0, 3.0]!)
	q := t.apply(point_mv(0.0, 0.0, 0.0)).coords()
	assert q[0] == 1.0 && q[1] == 2.0 && q[2] == 3.0
}

fn test_rotor_z_quarter() {
	// rotate (1,0,0) by 90 deg about +z -> (0,1,0)
	r := motor_rotor([0.0, 0.0, 1.0]!, math.pi / 2.0)
	q := r.apply(point_mv(1.0, 0.0, 0.0)).coords()
	assert math.abs(q[0] - 0.0) < 1e-6
	assert math.abs(q[1] - 1.0) < 1e-6
	assert math.abs(q[2] - 0.0) < 1e-6
}

fn test_to_matrix_roundtrip() {
	m := translator([1.0, 2.0, 3.0]!).gp(motor_rotor([0.0, 1.0, 0.0]!, math.pi / 2.0))
	mtx := m.to_matrix()
	assert math.abs(mtx[3] - 1.0) < 1e-6
	assert math.abs(mtx[7] - 2.0) < 1e-6
	assert math.abs(mtx[11] - 3.0) < 1e-6
}

fn test_exp_log_roundtrip() {
	m := translator([0.3, -0.4, 0.5]!).gp(motor_rotor([0.1, 0.6, -0.2]!, 0.7))
	b := m.log()
	m2 := motor_exp(b, 1.0)
	assert m.eq(m2)
}

fn test_interpolate_endpoints() {
	a := motor_identity()
	b := translator([1.0, 0.0, 0.0]!).gp(motor_rotor([0.0, 0.0, 1.0]!, 0.5))
	assert a.interpolate(b, 0.0).eq(a)
	assert a.interpolate(b, 1.0).eq(b)
}

fn test_extract_velocity() {
	// pure rotation about z at 1 rad/s
	dt := 0.1
	m0 := motor_rotor([0.0, 0.0, 1.0]!, 0.0)
	m1 := motor_rotor([0.0, 0.0, 1.0]!, 0.1)
	ang, lin := extract_velocity(m1, m0, dt)
	assert math.abs(ang[2] - 1.0) < 1e-3
	assert math.abs(lin[0]) < 1e-6
	assert math.abs(lin[1]) < 1e-6
	assert math.abs(lin[2]) < 1e-6
}
