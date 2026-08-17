module cga

import mlx
import math

// Dupin cyclide tests: blade construction (sphere-family envelope / versor
// inversion) + engine analytic intersection.
// Gate: ring cyclide (a=1, b=0.98, d=0.3, c=sqrt(a^2-b^2) ~= 0.199) with c<d<a.

const cga_a = 1.0
const cga_b = 0.98
const cga_d = 0.3
const cga_c = math.sqrt(cga_a * cga_a - cga_b * cga_b)

// canonical ring cyclide x-axis intersections (descending)
const cga_x1 = cga_a + cga_d - cga_c
const cga_x2 = cga_a - cga_d + cga_c
const cga_x3 = cga_d + cga_c - cga_a
const cga_x4 = -cga_a - cga_d - cga_c

fn cga_cy() DupinCyclide {
	return dupin_cyclide(cga_a, cga_b, cga_d, [0.0, 0.0, 0.0]!)
}

fn mod2pi(x f64) f64 {
	tau := 2.0 * math.pi
	r := x - tau * math.floor(x / tau)
	return math.min(r, tau - r)
}

fn ray3(x f64, y f64, z f64) mlx.Array {
	return mlx.array_f32([f32(x), f32(y), f32(z)], [1, 3])
}

fn f32s(vals []f64) []f32 {
	mut out := []f32{len: vals.len}
	for i, v in vals {
		out[i] = f32(v)
	}
	return out
}

fn cyc_hit(g Geometry, o [3]f64, d [3]f64) (f32, bool) {
	p := geom_to_camera(g, motor_identity())
	oa := ray3(o[0], o[1], o[2])
	da := ray3(d[0], d[1], d[2])
	t, _, mask := geom_intersect(p, oa, da)
	return t.item_f32(), mask.data_bool()[0]
}

// --- algebra model -----------------------------------------------------------

fn test_cyclide_param_implicit_consistency() {
	cy := cga_cy()
	for uv in [[0.0, 0.0]!, [1.2, 2.3]!, [3.1, 5.4]!, [math.pi, 0.5]!] {
		s := cy.surface(uv[0], uv[1])
		assert math.abs(cy.implicit(s[0], s[1], s[2])) < 1e-10
	}
}

fn test_cyclide_kind() {
	assert cga_cy().kind() == 'ring' // c < d < a
	assert dupin_cyclide(1.0, 0.6, 1.5, [0.0, 0.0, 0.0]!).kind() == 'spindle' // d > a
	assert dupin_cyclide(1.0, 0.6, 0.3, [0.0, 0.0, 0.0]!).kind() == 'horn' // d < c
}

fn test_cyclide_focal_sphere_tangency() {
	cy := cga_cy()
	for u in [0.0, 1.0, 2.0, 4.0] {
		r := cy.tangency_residual(u)
		assert math.abs(r[0]) < 1e-10
		assert math.abs(r[1]) < 1e-10
	}
}

fn test_cyclide_generator_sphere_is_blade() {
	cy := cga_cy()
	s := cy.generator_sphere(0.7)
	ctr, r := sphere_from_dual(s)
	sp := cy.spine(0.7)
	rad := cy.radius(0.7)
	assert math.abs(ctr[0] - sp[0]) < 1e-5
	assert math.abs(ctr[1] - sp[1]) < 1e-5
	assert math.abs(ctr[2] - sp[2]) < 1e-5
	assert math.abs(r - rad) < 1e-5
}

fn test_cyclide_focal_spheres_are_blades() {
	cy := cga_cy()
	s1, s2 := cy.focal_spheres()
	c1, r1 := sphere_from_dual(s1)
	c2, r2 := sphere_from_dual(s2)
	assert math.abs(c1[0] - cga_c) < 1e-5
	assert math.abs(c1[1]) < 1e-5
	assert math.abs(c1[2]) < 1e-5
	assert math.abs(r1 - (cga_a - cga_d)) < 1e-5
	assert math.abs(c2[0] + cga_c) < 1e-5
	assert math.abs(c2[1]) < 1e-5
	assert math.abs(c2[2]) < 1e-5
	assert math.abs(r2 - (cga_a + cga_d)) < 1e-5
}

fn test_cyclide_characteristic_circle_on_surface() {
	cy := cga_cy()
	u := 0.9
	e := cy.spine(u)
	ep := [-cy.a * math.sin(u), cy.b * math.cos(u), 0.0]!
	r := cy.radius(u)
	rp := cy.c() * math.sin(u)
	ep2 := ep[0] * ep[0] + ep[1] * ep[1] + ep[2] * ep[2]
	lam := r * rp / ep2
	center := [e[0] - lam * ep[0], e[1] - lam * ep[1], e[2] - lam * ep[2]]!
	radius := math.sqrt(r * r - lam * lam * ep2)
	nn := math.sqrt(ep2)
	n := [ep[0] / nn, ep[1] / nn, ep[2] / nn]!
	e1v := [n[1], -n[0], 0.0]!
	e2v := [n[1] * e1v[2] - n[2] * e1v[1], n[2] * e1v[0] - n[0] * e1v[2], n[0] *
		e1v[1] - n[1] * e1v[0]]!
	cc := cy.characteristic_circle(u)
	for t in [0.0, 1.3, 2.5] {
		p := [center[0] + radius * (e1v[0] * math.cos(t) + e2v[0] * math.sin(t)),
			center[1] + radius * (e1v[1] * math.cos(t) + e2v[1] * math.sin(t)),
			center[2] + radius * (e1v[2] * math.cos(t) + e2v[2] * math.sin(t))]!
		assert math.abs(cy.implicit(p[0], p[1], p[2])) < 1e-8
		assert math.abs(point(p[0], p[1], p[2]).ip(cc).scalar_part()) < 1e-6
	}
}

fn test_cyclide_uv_roundtrip() {
	cy := cga_cy()
	for uv in [[0.5, 1.0]!, [2.0, 4.0]!, [5.0, 0.2]!] {
		s := cy.surface(uv[0], uv[1])
		r := cy.uv(s[0], s[1], s[2])
		assert mod2pi(r[0] - uv[0]) < 1e-6
		assert mod2pi(r[1] + uv[1]) < 1e-6
	}
}

fn test_cyclide_from_torus_inversion() {
	cy := from_torus_inversion(2.0, 0.5, 1.0)
	assert cy.kind() == 'ring'
	r_maj := 2.0
	r_min := 0.5
	s := 1.0
	for uv in [[0.7, 1.9]!, [2.1, 0.3]!] {
		u := uv[0]
		v := uv[1]
		tx := s + (r_maj + r_min * math.cos(v)) * math.cos(u)
		ty := (r_maj + r_min * math.cos(v)) * math.sin(u)
		tz := r_min * math.sin(v)
		n2 := tx * tx + ty * ty + tz * tz
		assert math.abs(cy.implicit(tx / n2, ty / n2, tz / n2)) < 1e-9
	}
}

fn test_cyclide_inversion_versor() {
	cy := cga_cy()
	for p in [[2.0, 0.0, 0.0]!, [1.0, 2.0, 3.0]!, [-0.5, 0.25, 1.5]!] {
		q := cy.invert_point(point(p[0], p[1], p[2]))
		r2 := p[0] * p[0] + p[1] * p[1] + p[2] * p[2]
		c := q.coords()
		assert math.abs(c[0] - p[0] / r2) < 1e-6
		assert math.abs(c[1] - p[1] / r2) < 1e-6
		assert math.abs(c[2] - p[2] / r2) < 1e-6
	}
}

// --- engine ------------------------------------------------------------------

fn test_cyclide_ray_hits_implicit() {
	g := cyclide_geometry(cga_a, cga_b, cga_d, [0.0, 0.0, 0.0]!)
	t, m := cyc_hit(g, [3.0, 0.0, 0.0]!, [-1.0, 0.0, 0.0]!)
	assert m
	cy := cga_cy()
	assert math.abs(cy.implicit(3.0 - f64(t), 0.0, 0.0)) < 1e-3
}

fn test_cyclide_axis_ray_four_crossings() {
	g := cyclide_geometry(cga_a, cga_b, cga_d, [0.0, 0.0, 0.0]!)
	p := geom_to_camera(g, motor_identity())
	ts, _, valid := geom_crossings(p, ray3(3.0, 0.0, 0.0), ray3(-1.0, 0.0, 0.0))
	v := valid.data_bool()
	assert v.len == 4
	assert v[0] && v[1] && v[2] && v[3]
	td := ts.data_f32()
	mut exp := [3.0 - cga_x1, 3.0 - cga_x2, 3.0 - cga_x3, 3.0 - cga_x4]
	exp.sort()
	for i in 0 .. 4 {
		assert math.abs(f64(td[i]) - exp[i]) < 1e-2
	}
}

fn test_cyclide_center_hole_ray_misses() {
	g := cyclide_geometry(cga_a, cga_b, cga_d, [0.0, 0.0, 0.0]!)
	_, m := cyc_hit(g, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert !m
}

fn test_cyclide_contains() {
	g := cyclide_geometry(cga_a, cga_b, cga_d, [0.0, 0.0, 0.0]!)
	p := geom_to_camera(g, motor_identity())
	pos := mlx.array_f32(f32s([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0]), [3, 3])
	assert geom_contains(p, pos).data_bool() == [true, false, false]
}

fn test_cyclide_shift() {
	g := cyclide_geometry(cga_a, cga_b, cga_d, [1.0, 0.0, 0.0]!)
	p := geom_to_camera(g, motor_identity())
	pos := mlx.array_f32(f32s([2.0, 0.0, 0.0, 1.0, 0.0, 0.0]), [2, 3])
	assert geom_contains(p, pos).data_bool() == [true, false]
}

fn test_cyclide_bounds_contain_surface() {
	g := cyclide_geometry(cga_a, cga_b, cga_d, [0.0, 0.0, 0.0]!)
	p := geom_to_camera(g, motor_identity())
	b := geom_bounds(p) or { panic('no bounds') }
	lo := b[0]
	hi := b[1]
	cy := cga_cy()
	for i in 0 .. 7 {
		for j in 0 .. 7 {
			s := cy.surface(f64(i) * 2.0 * math.pi / 6.0, f64(j) * 2.0 * math.pi / 6.0)
			assert s[0] >= lo[0] - 1e-6 && s[0] <= hi[0] + 1e-6
			assert s[1] >= lo[1] - 1e-6 && s[1] <= hi[1] + 1e-6
			assert s[2] >= lo[2] - 1e-6 && s[2] <= hi[2] + 1e-6
		}
	}
}

fn test_cyclide_csg_combines() {
	csg := csg_geometry('union', [cyclide_geometry(cga_a, cga_b, cga_d, [0.0, 0.0,
		0.0]!), cyclide_geometry(cga_a, cga_b, cga_d, [0.0, 0.0, 0.0]!)])
	t, m := cyc_hit(csg, [3.0, 0.0, 0.0]!, [-1.0, 0.0, 0.0]!)
	assert m
	assert t > 0.0
}
