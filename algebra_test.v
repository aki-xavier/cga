module cga

import math

fn close(a f64, b f64, tol f64) bool {
	return math.abs(a - b) < tol
}

fn test_point_null_and_dist() {
	p1 := point(0.0, 0.0, 0.0)
	assert close(p1.gp(p1).values[0], 0.0, 1e-4)
	assert close(point_dist(p1, point(3.0, 4.0, 0.0)), 5.0, 1e-4)
}

fn test_line_incidence() {
	l := line(point(0.0, 0.0, 0.0), point(1.0, 0.0, 0.0))
	assert close(point(2.0, 0.0, 0.0).op(l).vmax(), 0.0, 1e-4)
	assert point(0.0, 1.0, 0.0).op(l).vmax() > 1e-3
}

fn test_plane_incidence() {
	pi := plane([0.0, 0.0, 1.0]!, 2.0)
	assert close(point(0.3, -0.7, 2.0).ip(pi).vmax(), 0.0, 1e-4)
	assert point(0.0, 0.0, 0.0).ip(pi).vmax() > 1e-3
}

fn test_sphere_incidence() {
	s := sphere([1.0, 2.0, 3.0]!, 2.0)
	assert close(point(3.0, 2.0, 3.0).ip(s).vmax(), 0.0, 1e-4)
	assert point(0.0, 0.0, 0.0).ip(s).vmax() > 1e-3
}

fn test_circle_incidence() {
	c := circle([0.0, 0.0, 0.0]!, 1.0, [0.0, 0.0, 1.0]!)
	assert close(point(0.0, 1.0, 0.0).ip(c).vmax(), 0.0, 1e-4)
	assert point(0.0, 0.0, 1.0).ip(c).vmax() > 1e-3
	cnu := circle([1.0, 2.0, 3.0]!, 2.0, [0.0, 0.0, 2.0]!)
	assert close(point(3.0, 2.0, 3.0).ip(cnu).vmax(), 0.0, 1e-4)
}

fn test_distances() {
	pi := plane([0.0, 0.0, 1.0]!, 2.0)
	s := sphere([1.0, 2.0, 3.0]!, 2.0)
	assert close(plane_dist(pi, point(0.0, 0.0, 5.0)), 3.0, 1e-4)
	assert close(sphere_dist(s, point(3.0, 2.0, 3.0)), 0.0, 1e-4)
	assert sphere_dist(s, point(5.0, 2.0, 3.0)) > 0.0
	assert sphere_dist(s, point(1.0, 2.0, 3.0)) < 0.0
}

fn test_meet_plane_plane() {
	pi := plane([0.0, 0.0, 1.0]!, 2.0)
	pi2 := plane([0.0, 1.0, 0.0]!, 1.0)
	lm := pi.dual().meet(pi2.dual())
	assert close(point(0.0, 1.0, 2.0).op(lm).vmax(), 0.0, 1e-4)
	assert close(point(5.0, 1.0, 2.0).op(lm).vmax(), 0.0, 1e-4)
}

fn test_meet_line_sphere() {
	lz := line(point(0.0, 0.0, -2.0), point(0.0, 0.0, 2.0))
	ppm := lz.meet(sphere([0.0, 0.0, 0.0]!, 1.0).dual())
	assert close(point(0.0, 0.0, 1.0).op(ppm).vmax(), 0.0, 1e-4)
	assert close(point(0.0, 0.0, -1.0).op(ppm).vmax(), 0.0, 1e-4)
}

fn test_far_from_origin_dist() {
	assert close(point_dist(point(1000.0, 0.0, 0.0), point(1001.0, 0.0, 0.0)), 1.0, 1e-2)
}

fn test_cylinder_distances() {
	cy := cylinder([0.0, 0.0, 2.0]!, [0.0, 1.0, 0.0]!, 1.0)
	assert close(cylinder_dist(cy, point(1.0, 5.0, 2.0)), 0.0, 1e-4)
	assert close(cylinder_dist(cy, point(0.2, 0.0, 2.0)), -0.8, 1e-4)
	assert close(cylinder_dist(cy, point(3.0, -2.0, 2.0)), 2.0, 1e-4)
	assert close(cylinder_dist(cy, point(-1.0, 5.0, 2.0)), 0.0, 1e-4)
}

fn test_from_dual_after_motor() {
	s_cam := translator([1.0, 2.0, 3.0]!).apply(sphere([0.0, 0.0, 0.0]!, 0.5))
	c, rho := sphere_from_dual(s_cam)
	assert close(c[0], 1.0, 1e-4) && close(c[1], 2.0, 1e-4) && close(c[2], 3.0, 1e-4)
	assert close(rho, 0.5, 1e-4)
	pi_cam := translator([1.0, 2.0, 3.0]!).apply(plane([0.0, 1.0, 0.0]!, 0.0))
	assert close(pi_cam.einf_coeff(), 2.0, 1e-4)
}
