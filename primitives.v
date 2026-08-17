module cga

// CGA primitives (point / point-pair / line / plane / sphere / circle /
// cylinder) and their distance / incidence helpers.
//
// Representation convention (same as the Python reference):
//   - Point / PointPair / Line are direct (join) form; incidence is p.op(X) = 0.
//   - Plane / Sphere / Circle are dual form; incidence is p.ip(X) = 0.
//
// In V the primitives are plain `Multivector` values (a motor is also just a
// multivector), so all algebra (gp/ip/op/dual/meet) composes freely.
import math

// point returns the conformal point p = e0 + x e1 + y e2 + z e3 + 0.5 r^2 einf.
pub fn point(x f64, y f64, z f64) Multivector {
	r2 := x * x + y * y + z * z
	return mv_vector(x, y, z, 1.0, 0.5 * r2)
}

// point_pair returns the point pair Pp = p1 ^ p2 (grade-2 direct form).
pub fn point_pair(p1 Multivector, p2 Multivector) Multivector {
	return p1.op(p2)
}

// line returns the line L = p1 ^ p2 ^ einf (grade-3 direct form).
pub fn line(p1 Multivector, p2 Multivector) Multivector {
	return p1.op(p2).op(einf())
}

// plane returns the dual plane pi = n + d einf (n unit normal).
pub fn plane(normal [3]f64, distance f64) Multivector {
	mut nx := normal[0]
	mut ny := normal[1]
	mut nz := normal[2]
	nl := math.sqrt(nx * nx + ny * ny + nz * nz)
	if nl <= 1e-12 {
		panic('plane normal vector is zero or degenerate')
	}
	nx /= nl
	ny /= nl
	nz /= nl
	return mv_vector(nx, ny, nz, 0.0, distance)
}

// sphere returns the dual sphere s = up(c) - 0.5 rho^2 einf.
pub fn sphere(center [3]f64, radius f64) Multivector {
	half := 0.5 * radius * radius
	return point(center[0], center[1], center[2]).sub(mv_vector(0.0, 0.0, 0.0, 0.0, half))
}

// sphere_from_dual extracts (center, radius) from a dual sphere blade.
pub fn sphere_from_dual(s Multivector) ([3]f64, f64) {
	w := s.e0_coeff()
	if math.abs(w) < 1e-12 {
		panic('sphere multivector has no e0 component')
	}
	v := s.euclidean_vector()
	f := s.einf_coeff()
	cx := v[0] / w
	cy := v[1] / w
	cz := v[2] / w
	mut rho_sq := (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / (w * w) - 2.0 * f / w
	if rho_sq < 0.0 {
		rho_sq = 0.0
	}
	return [cx, cy, cz]!, math.sqrt(rho_sq)
}

// circle returns the dual circle = sphere ^ plane.
pub fn circle(center [3]f64, radius f64, normal [3]f64) Multivector {
	s := sphere(center, radius)
	nl := math.sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2])
	mut d := 0.0
	if nl > 1e-12 {
		d = (center[0] * normal[0] + center[1] * normal[1] + center[2] * normal[2]) / nl
	}
	p := plane(normal, d)
	return s.op(p)
}

// Cylinder carries an axis Line blade plus radius/axis metadata (a rebuilt
// primitive, not a single blade algebraic object).
pub struct Cylinder {
pub:
	blade      Multivector
	radius     f64
	axis_dir   [3]f64
	axis_point [3]f64
}

// cylinder builds a cylinder from an axis point, axis direction (unitised) and
// radius.
pub fn cylinder(axis_point [3]f64, axis_dir [3]f64, radius f64) Cylinder {
	mut ax := axis_dir[0]
	mut ay := axis_dir[1]
	mut az := axis_dir[2]
	al := math.sqrt(ax * ax + ay * ay + az * az)
	if al <= 1e-12 {
		panic('cylinder axis is degenerate')
	}
	ux := ax / al
	uy := ay / al
	uz := az / al
	q := point(axis_point[0], axis_point[1], axis_point[2])
	q2 := point(axis_point[0] + ux, axis_point[1] + uy, axis_point[2] + uz)
	return Cylinder{
		blade:      line(q, q2)
		radius:     radius
		axis_dir:   [ux, uy, uz]!
		axis_point: axis_point
	}
}

// --- distances --------------------------------------------------------------

// point_dist returns the euclidean distance between two conformal points.
pub fn point_dist(a Multivector, b Multivector) f64 {
	c1 := a.coords()
	c2 := b.coords()
	dx := c1[0] - c2[0]
	dy := c1[1] - c2[1]
	dz := c1[2] - c2[2]
	return math.sqrt(dx * dx + dy * dy + dz * dz)
}

// plane_dist returns the signed distance from point p to plane pi
// ((n . x - d) / |n|).
pub fn plane_dist(pi Multivector, p Multivector) f64 {
	c := p.coords()
	v := pi.euclidean_vector()
	d := pi.einf_coeff()
	nl := math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
	if nl < 1e-12 {
		return 1e300
	}
	return (v[0] * c[0] + v[1] * c[1] + v[2] * c[2] - d) / nl
}

// sphere_dist returns the signed distance from point p to sphere s
// (positive outside, negative inside).
pub fn sphere_dist(s Multivector, p Multivector) f64 {
	c, r := sphere_from_dual(s)
	pc := p.coords()
	dx := pc[0] - c[0]
	dy := pc[1] - c[1]
	dz := pc[2] - c[2]
	return math.sqrt(dx * dx + dy * dy + dz * dz) - r
}

// cylinder_dist returns the signed distance from point p to the cylinder
// surface (positive outside, negative inside).
pub fn cylinder_dist(cy Cylinder, p Multivector) f64 {
	c := p.coords()
	dx := c[0] - cy.axis_point[0]
	dy := c[1] - cy.axis_point[1]
	dz := c[2] - cy.axis_point[2]
	ux := cy.axis_dir[0]
	uy := cy.axis_dir[1]
	uz := cy.axis_dir[2]
	dot := dx * ux + dy * uy + dz * uz
	ex := dx - dot * ux
	ey := dy - dot * uy
	ez := dz - dot * uz
	d := math.sqrt(ex * ex + ey * ey + ez * ez)
	return d - cy.radius
}
