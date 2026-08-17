module cga

// Dupin cyclide — a non-blade quartic surface, plus its blade constructions
// (a family of spheres whose envelope is the cyclide; versor inversion of a
// torus).  This is a computable geometric model (not a Multivector subclass).
//
// Canonical form (design parameters a, b, d; c = sqrt(a^2 - b^2), a > b > 0):
//   directrix ellipse   E(u) = (a cos u, b sin u, 0)          (xy plane)
//   focal hyperbola     H(v) = (c/cos v, 0, b tan v)          (xz plane)
//   implicit: (x^2+y^2+z^2+b^2-d^2)^2 - 4(a x - c d)^2 - 4 b^2 y^2 = 0
//   d classification: c<d<a ring | d>a spindle | 0<d<c horn
//   degenerate a=b (c=0): torus (major radius a, minor radius d, axis z).

import math

pub struct DupinCyclide {
pub:
	a     f64
	b     f64
	d     f64
	shift [3]f64
}

// dupin_cyclide builds an elliptic Dupin cyclide from design parameters a, b, d.
pub fn dupin_cyclide(a f64, b f64, d f64, shift [3]f64) DupinCyclide {
	if !(a > b && b > 0.0) {
		panic('need a > b > 0, got a=${a}, b=${b}')
	}
	if d <= 0.0 {
		panic('need d > 0, got d=${d}')
	}
	return DupinCyclide{
		a: a
		b: b
		d: d
		shift: shift
	}
}

// c returns the ellipse linear eccentricity sqrt(a^2 - b^2).
pub fn (cy DupinCyclide) c() f64 {
	return math.sqrt(cy.a * cy.a - cy.b * cy.b)
}

// kind returns "ring", "spindle" or "horn" according to d vs c, a.
pub fn (cy DupinCyclide) kind() string {
	c := cy.c()
	if c < cy.d && cy.d < cy.a {
		return 'ring'
	}
	if cy.d > cy.a {
		return 'spindle'
	}
	return 'horn'
}

// spine returns the sphere centre E(u) on the directrix ellipse.
pub fn (cy DupinCyclide) spine(u f64) [3]f64 {
	return [cy.a * math.cos(u), cy.b * math.sin(u), 0.0]!
}

// radius returns the generating-sphere radius r(u) = d - c cos u.
pub fn (cy DupinCyclide) radius(u f64) f64 {
	return cy.d - cy.c() * math.cos(u)
}

// generator_sphere returns one sphere S(u) of the one-parameter family.
pub fn (cy DupinCyclide) generator_sphere(u f64) Multivector {
	sp := cy.spine(u)
	return sphere(sp, cy.radius(u))
}

// focal_spheres returns the two fixed focal spheres (Maxwell property).
pub fn (cy DupinCyclide) focal_spheres() (Multivector, Multivector) {
	c := cy.c()
	return sphere([c, 0.0, 0.0]!, cy.a - cy.d), sphere([-c, 0.0, 0.0]!, cy.a + cy.d)
}

// tangency_residual returns the tangency residuals (should be ~0).
pub fn (cy DupinCyclide) tangency_residual(u f64) [2]f64 {
	sp := cy.spine(u)
	r := cy.radius(u)
	c := cy.c()
	d1 := math.hypot(sp[0] - c, sp[1])
	d2 := math.hypot(sp[0] + c, sp[1])
	return [d1 - r - (cy.a - cy.d), d2 + r - (cy.a + cy.d)]!
}

// characteristic_circle returns the curvature-line circle = S(u) meet S(u+du).
pub fn (cy DupinCyclide) characteristic_circle(u f64) Multivector {
	c := cy.c()
	cu := math.cos(u)
	su := math.sin(u)
	e := [cy.a * cu, cy.b * su, 0.0]!
	ep := [-cy.a * su, cy.b * cu, 0.0]!
	r := cy.d - c * cu
	rp := c * su
	ep2 := cy.a * cy.a * su * su + cy.b * cy.b * cu * cu
	if ep2 < 1e-18 {
		panic('characteristic circle degenerates (zero tangent)')
	}
	lam := (r * rp) / ep2
	center := [e[0] - lam * ep[0], e[1] - lam * ep[1], e[2] - lam * ep[2]]!
	rho2 := r * r - lam * lam * ep2
	if rho2 <= 0.0 {
		panic('characteristic circle radius non-positive (cusp/degenerate)')
	}
	return circle(center, math.sqrt(rho2), ep)
}

// surface parametrises the surface point at (u, v).
pub fn (cy DupinCyclide) surface(u f64, v f64) [3]f64 {
	a := cy.a
	b := cy.b
	c := cy.c()
	d := cy.d
	cu := math.cos(u)
	cv := math.cos(v)
	su := math.sin(u)
	sv := math.sin(v)
	den := a - c * cu * cv
	x := (d * (c - a * cu * cv) + b * b * cu) / den
	y := (b * su * (a - d * cv)) / den
	z := (b * sv * (c * cu - d)) / den
	return [x + cy.shift[0], y + cy.shift[1], z + cy.shift[2]]!
}

// implicit evaluates F at (x, y, z) in world coordinates (F < 0 = inside).
pub fn (cy DupinCyclide) implicit(x f64, y f64, z f64) f64 {
	a := cy.a
	b := cy.b
	c := cy.c()
	d := cy.d
	sx := x - cy.shift[0]
	sy := y - cy.shift[1]
	sz := z - cy.shift[2]
	bb := b * b - d * d
	rho := sx * sx + sy * sy + sz * sz
	return (rho + bb) * (rho + bb) - 4.0 * (a * sx - c * d) * (a * sx - c * d) - 4.0 *
		b * b * sy * sy
}

// gradient returns grad F (shift-independent direction).
pub fn (cy DupinCyclide) gradient(x f64, y f64, z f64) [3]f64 {
	a := cy.a
	b := cy.b
	c := cy.c()
	d := cy.d
	sx := x - cy.shift[0]
	sy := y - cy.shift[1]
	sz := z - cy.shift[2]
	bb := b * b - d * d
	rho := sx * sx + sy * sy + sz * sz
	g := rho + bb
	return [4.0 * sx * g - 8.0 * a * (a * sx - c * d), 4.0 * sy * g - 8.0 * b * b * sy,
		4.0 * sz * g]!
}

// normal returns the unit normal (gradient direction, pointing outside).
pub fn (cy DupinCyclide) normal(x f64, y f64, z f64) [3]f64 {
	g := cy.gradient(x, y, z)
	n := math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
	if n < 1e-12 {
		return [0.0, 0.0, 1.0]!
	}
	return [g[0] / n, g[1] / n, g[2] / n]!
}

// contains reports whether the point is inside (F < 0).
pub fn (cy DupinCyclide) contains(x f64, y f64, z f64) bool {
	return cy.implicit(x, y, z) < 0.0
}

// uv recovers the (u, v) parameters of a surface point.
pub fn (cy DupinCyclide) uv(x f64, y f64, z f64) [2]f64 {
	a := cy.a
	b := cy.b
	c := cy.c()
	d := cy.d
	sx := x - cy.shift[0]
	sy := y - cy.shift[1]
	sz := z - cy.shift[2]
	rho := sx * sx + sy * sy + sz * sz
	u := math.atan2(2.0 * b * sy, 2.0 * (a * sx - c * d))
	v := math.atan2(2.0 * b * sz, d * d + b * b - rho)
	return [u, v]!
}

// inversion_versor returns the unit-sphere inversion versor s = e0 - 0.5 einf.
pub fn (cy DupinCyclide) inversion_versor() Multivector {
	return sphere([0.0, 0.0, 0.0]!, 1.0)
}

// invert_point inverts a point through the unit sphere: x -> x / |x|^2.
pub fn (cy DupinCyclide) invert_point(p Multivector) Multivector {
	s := cy.inversion_versor()
	out := s.gp(p).gp(s)
	c := out.coords()
	return point(c[0], c[1], c[2])
}

// from_torus_inversion recovers cyclide parameters from a torus inverted through
// the unit sphere (torus major radius `major`, minor radius `minor`, axis z,
// translated by shift_x along x).
pub fn from_torus_inversion(major f64, minor f64, shift_x f64) DupinCyclide {
	r := major
	rr := minor
	if !(r > rr && rr > 0.0) {
		panic('need major > minor > 0, got major=${major}, minor=${minor}')
	}
	s := shift_x
	xs := [s + r + rr, s + r - rr, s - r + rr, s - r - rr]!
	for x in xs {
		if math.abs(x) < 1e-12 {
			panic('torus passes through inversion centre; result not a ring')
		}
	}
	mut ys := [1.0 / xs[0], 1.0 / xs[1], 1.0 / xs[2], 1.0 / xs[3]]
	ys.sort() // ascending
	y1 := ys[3]
	y2 := ys[2]
	y3 := ys[1]
	y4 := ys[0]
	a := 0.25 * (y1 + y2 - y3 - y4)
	d := 0.25 * (y1 - y2 + y3 - y4)
	c := 0.25 * (-y1 + y2 + y3 - y4)
	m0 := 0.25 * (y1 + y2 + y3 + y4)
	if math.abs(c) < 1e-12 * math.max(1.0, math.abs(a)) {
		panic('torus centred at inversion centre stays a torus (c=0); need shift_x != 0')
	}
	b := math.sqrt(a * a - c * c)
	return dupin_cyclide(a, b, d, [m0, 0.0, 0.0]!)
}
