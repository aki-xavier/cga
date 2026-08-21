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

fn test_rotor_from_quaternion_scaled() {
	// 2x the quaternion for 60 deg about +z must give the same rotor as the
	// unit quaternion (scale-invariant angle): e1 -> (cos60, sin60, 0)
	c := math.cos(math.pi / 6.0) // half-angle 30 deg
	s := math.sin(math.pi / 6.0)
	q := Quaternion{
		w: 2.0 * c
		x: 0.0
		y: 0.0
		z: 2.0 * s
	}
	p := rotor_from_quaternion(q).apply(point_mv(1.0, 0.0, 0.0)).coords()
	assert math.abs(p[0] - 0.5) < 1e-9
	assert math.abs(p[1] - math.sqrt(3.0) / 2.0) < 1e-9
	assert math.abs(p[2]) < 1e-9
	// and the axis stays +z (not distorted by the scale)
	p2 := rotor_from_quaternion(q).apply(point_mv(0.0, 0.0, 1.0)).coords()
	assert math.abs(p2[2] - 1.0) < 1e-9
	assert math.abs(p2[0]) < 1e-9
	assert math.abs(p2[1]) < 1e-9
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
