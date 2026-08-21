module cga

// General surface sampling for Gaussian splats (Extension 2): attach flattened
// Gaussians to any tractable Geometry variant, in the geometry's LOCAL space
// (wrap in SplatsGeometry and pose via the mesh Object3D — see
// render_scene_with_splats in splat_render.v).
//
// All samplers are deterministic (grids / Fibonacci / golden-spiral) and use
// EXACT point + normal evaluation:
//   sphere     Fibonacci directions;            normal = direction
//   ellipsoid  Fibonacci directions x radii;    normal = unit gradient
//   torus      (u, v) grid;                     normal = analytic
//   cone       equal-area rings + base disc;    normal = analytic gradient
//   cylinder   side grid + cap spirals;         normal = analytic
//   box        area-weighted face grids;        normal = face axis
//   plane      bounded square patch (default half-size 1.0, or explicit via
//              sample_gaussians_on_plane);      normal = plane normal
//   cyclide    delegates to sample_gaussians_on_cyclide (u, v grid)
//   affine     unwraps the child and maps through the affine block, which must
//              be rigid x uniform scale (panics otherwise: a general linear
//              map does not preserve the rotation x diag scale splat form)
//   trimesh / csg / circle / splats            NOT supported (panic)
//
// Sample counts are approximate: grid rounding may produce slightly more or
// fewer than n splats; samples with degenerate normals are skipped.
import math

// frame_from_normal builds the Route A local-frame quaternion for a unit
// normal n: R has columns (t1, t2, n) with t1 x t2 = n (thin axis local e3).
fn frame_from_normal(n [3]f64) Quaternion {
	mut ref := [1.0, 0.0, 0.0]!
	if math.abs(n[0]) >= 0.9 {
		ref = [0.0, 1.0, 0.0]!
	}
	t1 := vec3_unit(vec3_cross(n, ref))
	t2 := vec3_cross(n, t1)
	// R has columns (t1, t2, n); mat3_new takes rows
	r := mat3_new([t1[0], t2[0], n[0]]!, [t1[1], t2[1], n[1]]!, [t1[2], t2[2], n[2]]!)
	return matrix_to_quaternion(r)
}

// gaussian_at builds one flattened splat at p with its frame aligned to the
// unit normal n.
fn gaussian_at(p [3]f64, n [3]f64, sigma_tangent f64, sigma_normal f64, mat Material) Gaussian {
	return Gaussian{
		mean:    p
		quat:    frame_from_normal(n)
		scale:   [sigma_tangent, sigma_tangent, sigma_normal]!
		opacity: mat.opacity
		color:   mat.color
	}
}

// fibonacci_dirs returns n approximately uniform unit directions (Fibonacci
// sphere).
fn fibonacci_dirs(n int) [][3]f64 {
	golden := math.pi * (3.0 - math.sqrt(5.0))
	mut out := [][3]f64{cap: n}
	for i in 0 .. n {
		y := 1.0 - (2.0 * f64(i) + 1.0) / f64(n)
		r := math.sqrt(math.max(0.0, 1.0 - y * y))
		phi := golden * f64(i)
		out << [r * math.cos(phi), y, r * math.sin(phi)]!
	}
	return out
}

// igrid splits a sample budget into (rows, cols) with cols/rows ~= aspect.
fn igrid(n int, aspect f64) (int, int) {
	mut rows := int(math.sqrt(f64(n) / aspect))
	if rows < 1 {
		rows = 1
	}
	mut cols := int(math.ceil(f64(n) / f64(rows)))
	if cols < 1 {
		cols = 1
	}
	return rows, cols
}

// --- per-primitive samplers (local space) ------------------------------------

fn sample_sphere(sp SphereGeometry, n int, st f64, sn f64, mat Material) []Gaussian {
	mut out := []Gaussian{cap: n}
	for w in fibonacci_dirs(n) {
		p := [sp.radius * w[0], sp.radius * w[1], sp.radius * w[2]]!
		out << gaussian_at(p, w, st, sn, mat)
	}
	return out
}

fn sample_ellipsoid(e EllipsoidGeometry, n int, st f64, sn f64, mat Material) []Gaussian {
	rx, ry, rz := e.radii[0], e.radii[1], e.radii[2]
	mut out := []Gaussian{cap: n}
	for w in fibonacci_dirs(n) {
		p := [rx * w[0], ry * w[1], rz * w[2]]!
		// gradient of x^2/rx^2 + y^2/ry^2 + z^2/rz^2 - 1 at p (== 2*w_i/r_i)
		gr := [w[0] / rx, w[1] / ry, w[2] / rz]!
		if vec3_dot(gr, gr) < 1e-24 {
			continue
		}
		out << gaussian_at(p, vec3_unit(gr), st, sn, mat)
	}
	return out
}

fn sample_torus(t TorusGeometry, n int, st f64, sn f64, mat Material) []Gaussian {
	n_v, n_u := igrid(n, (t.major + t.minor) / t.minor)
	mut out := []Gaussian{cap: n_u * n_v}
	tau := 2.0 * math.pi
	for i in 0 .. n_u {
		u := tau * f64(i) / f64(n_u)
		cu, su := math.cos(u), math.sin(u)
		for j in 0 .. n_v {
			v := tau * f64(j) / f64(n_v)
			cv, sv := math.cos(v), math.sin(v)
			p := [(t.major + t.minor * cv) * cu, (t.major + t.minor * cv) * su, t.minor * sv]!
			nrm := [cv * cu, cv * su, sv]!
			out << gaussian_at(p, nrm, st, sn, mat)
		}
	}
	return out
}

// disc_spiral returns m golden-spiral points on the disc of radius r in the
// plane z = zz (local frame).
fn disc_spiral(r f64, zz f64, m int) [][3]f64 {
	golden := math.pi * (3.0 - math.sqrt(5.0))
	mut out := [][3]f64{cap: m}
	for k in 0 .. m {
		rho := r * math.sqrt((f64(k) + 0.5) / f64(m))
		phi := golden * f64(k)
		out << [rho * math.cos(phi), rho * math.sin(phi), zz]!
	}
	return out
}

fn sample_cone(c ConeGeometry, n int, st f64, sn f64, mat Material) []Gaussian {
	// canonical cone: apex z = +h/2, base radius r at z = -h/2
	r, h := c.radius, c.height
	slant := math.sqrt(r * r + h * h)
	n_side := int(math.round(f64(n) * slant / (slant + r))) // area ratio pi*r*slant : pi*r^2
	n_cap := n - n_side
	mut out := []Gaussian{cap: n}
	if n_side > 0 {
		// equal-area rings (t = sqrt spacing), fixed count per ring
		rings, cols := igrid(n_side, math.pi * r / slant)
		for j in 0 .. rings {
			t := math.sqrt((f64(j) + 0.5) / f64(rings)) // 0 = apex, 1 = base
			rho := t * r
			z := h / 2.0 - t * h
			for i in 0 .. cols {
				phi := 2.0 * math.pi * (f64(i) + 0.5) / f64(cols)
				cu, su := math.cos(phi), math.sin(phi)
				p := [rho * cu, rho * su, z]!
				nrm := vec3_unit([cu, su, r / h]!)
				out << gaussian_at(p, nrm, st, sn, mat)
			}
		}
	}
	for p in disc_spiral(r, -h / 2.0, n_cap) {
		out << gaussian_at(p, [0.0, 0.0, -1.0]!, st, sn, mat)
	}
	return out
}

fn sample_cylinder(cy CylinderGeometry, n int, st f64, sn f64, mat Material) []Gaussian {
	r := cy.radius
	if cy.half < 0.0 {
		// infinite cylinder: sample the side section z in [-1, 1] (documented
		// default; use a finite cylinder for explicit extent)
		rows, cols := igrid(n, math.pi * r)
		mut out := []Gaussian{cap: rows * cols}
		for j in 0 .. rows {
			z := -1.0 + 2.0 * (f64(j) + 0.5) / f64(rows)
			for i in 0 .. cols {
				phi := 2.0 * math.pi * (f64(i) + 0.5) / f64(cols)
				cu, su := math.cos(phi), math.sin(phi)
				out << gaussian_at([r * cu, r * su, z]!, [cu, su, 0.0]!, st, sn, mat)
			}
		}
		return out
	}
	h := cy.half
	// area ratio side : caps = 4*pi*r*h : 2*pi*r^2
	n_side := int(math.round(f64(n) * 2.0 * h / (2.0 * h + r)))
	n_cap := (n - n_side) / 2
	mut out := []Gaussian{cap: n}
	if n_side > 0 {
		rows, cols := igrid(n_side, math.pi * r / h)
		for j in 0 .. rows {
			z := -h + 2.0 * h * (f64(j) + 0.5) / f64(rows)
			for i in 0 .. cols {
				phi := 2.0 * math.pi * (f64(i) + 0.5) / f64(cols)
				cu, su := math.cos(phi), math.sin(phi)
				out << gaussian_at([r * cu, r * su, z]!, [cu, su, 0.0]!, st, sn, mat)
			}
		}
	}
	for p in disc_spiral(r, h, n_cap) {
		out << gaussian_at(p, [0.0, 0.0, 1.0]!, st, sn, mat)
	}
	for p in disc_spiral(r, -h, n - n_side - n_cap) {
		out << gaussian_at(p, [0.0, 0.0, -1.0]!, st, sn, mat)
	}
	return out
}

fn sample_box(b BoxGeometry, n int, st f64, sn f64, mat Material) []Gaussian {
	hx, hy, hz := b.half[0], b.half[1], b.half[2]
	// (face area, side lengths along the other two axes)
	areas := [4.0 * hy * hz, 4.0 * hx * hz, 4.0 * hx * hy]
	sides := [[hy, hz]!, [hx, hz]!, [hx, hy]!]
	other := [[1, 2]!, [0, 2]!, [0, 1]!]
	halfs := [hx, hy, hz]!
	a_tot := areas[0] + areas[1] + areas[2]
	mut out := []Gaussian{cap: n}
	for axis in 0 .. 3 {
		for sgn in [-1.0, 1.0] {
			m := int(math.round(f64(n) * (areas[axis] / 2.0) / a_tot))
			if m < 1 {
				continue
			}
			rows, cols := igrid(m, sides[axis][0] / sides[axis][1])
			mut nrm := [0.0, 0.0, 0.0]!
			nrm[axis] = sgn
			for j in 0 .. rows {
				for i in 0 .. cols {
					u1 := sides[axis][0] * (2.0 * (f64(j) + 0.5) / f64(rows) - 1.0)
					u2 := sides[axis][1] * (2.0 * (f64(i) + 0.5) / f64(cols) - 1.0)
					mut p := [0.0, 0.0, 0.0]!
					p[axis] = sgn * halfs[axis]
					p[other[axis][0]] = u1
					p[other[axis][1]] = u2
					out << gaussian_at(p, nrm, st, sn, mat)
				}
			}
		}
	}
	return out
}

// sample_gaussians_on_plane samples a bounded square patch (half-size `half`,
// centred on the plane origin n*d) of the infinite plane, on a uniform grid.
pub fn sample_gaussians_on_plane(pl PlaneGeometry, half f64, n int, sigma_tangent f64, sigma_normal f64, mat Material) Gaussians {
	if half <= 0.0 {
		panic('need half > 0, got ${half}')
	}
	nrm := vec3_unit(pl.blade.euclidean_vector())
	d := pl.blade.einf_coeff()
	p0 := [nrm[0] * d, nrm[1] * d, nrm[2] * d]!
	// in-plane axes
	mut ref := [1.0, 0.0, 0.0]!
	if math.abs(nrm[0]) >= 0.9 {
		ref = [0.0, 1.0, 0.0]!
	}
	t1 := vec3_unit(vec3_cross(nrm, ref))
	t2 := vec3_cross(nrm, t1)
	rows, cols := igrid(n, 1.0)
	mut g := Gaussians{
		material: mat
		splats:   []Gaussian{cap: rows * cols}
	}
	for j in 0 .. rows {
		for i in 0 .. cols {
			u1 := half * (2.0 * (f64(j) + 0.5) / f64(rows) - 1.0)
			u2 := half * (2.0 * (f64(i) + 0.5) / f64(cols) - 1.0)
			p := [p0[0] + t1[0] * u1 + t2[0] * u2, p0[1] + t1[1] * u1 + t2[1] * u2,
				p0[2] + t1[2] * u1 + t2[2] * u2]!
			g.splats << gaussian_at(p, nrm, sigma_tangent, sigma_normal, mat)
		}
	}
	return g
}

// uniform_rotation_factor returns (true, s) when m == s * R with R a PROPER
// rotation (det > 0) — the only linear maps that preserve the rotation x
// diag(scale) splat covariance form.
fn uniform_rotation_factor(m Mat3) (bool, f64) {
	at := mat3_transpose(m)
	b := mat3_mul(at, m)
	s2 := (b[0][0] + b[1][1] + b[2][2]) / 3.0
	// relative to the scale, with a small absolute floor for near-zero scales
	tol := 1e-9 * s2 + 1e-24
	if math.abs(b[0][1]) > tol || math.abs(b[0][2]) > tol || math.abs(b[1][2]) > tol
		|| math.abs(b[0][0] - s2) > tol || math.abs(b[1][1] - s2) > tol
		|| math.abs(b[2][2] - s2) > tol {
		return false, 0.0
	}
	det := m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
		m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
		m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
	if det <= 0.0 {
		return false, 0.0 // reflection: the frame would be improper
	}
	return true, math.sqrt(s2)
}

// sample_affine samples the child geometry, then maps the splats through the
// affine block A = motor * linear.  Only rigid x uniform-scale blocks preserve
// the rotation x diag(scale) splat form; anything else panics.
fn sample_affine(a AffineGeometry, n int, st f64, sn f64, mat Material) []Gaussian {
	inner := sample_gaussians_on_surface(a.inner[0], n, st, sn, mat)
	m4 := a.motor.to_matrix()
	ro := mat3_new([m4[0], m4[1], m4[2]]!, [m4[4], m4[5], m4[6]]!, [m4[8], m4[9], m4[10]]!)
	a3 := mat3_mul(ro, a.linear)
	t := [m4[3], m4[7], m4[11]]!
	ok, s := uniform_rotation_factor(a3)
	if !ok {
		panic('affine splat sampling supports only rigid + uniform-scale transforms')
	}
	q := mat3_new([a3[0][0] / s, a3[0][1] / s, a3[0][2] / s]!,
		[a3[1][0] / s, a3[1][1] / s, a3[1][2] / s]!, [a3[2][0] / s, a3[2][1] / s, a3[2][2] / s]!)
	mut out := []Gaussian{cap: inner.splats.len}
	for sp in inner.splats {
		mp := mat3_vec(a3, sp.mean)
		rm := rotor_from_quaternion(sp.quat).to_matrix()
		rw := mat3_new([rm[0], rm[1], rm[2]]!, [rm[4], rm[5], rm[6]]!, [rm[8], rm[9], rm[10]]!)
		out << Gaussian{
			mean:    [mp[0] + t[0], mp[1] + t[1], mp[2] + t[2]]!
			quat:    matrix_to_quaternion(mat3_mul(q, rw))
			scale:   [s * sp.scale[0], s * sp.scale[1], s * sp.scale[2]]!
			opacity: sp.opacity
			color:   sp.color
		}
	}
	return out
}

// sample_gaussians_on_surface samples ~n flattened Gaussians on any tractable
// Geometry variant, in the geometry's LOCAL space (wrap the result in
// SplatsGeometry and pose it via the mesh Object3D).  See the file header for
// per-primitive methods; unsupported variants panic with a clear message.
pub fn sample_gaussians_on_surface(geom Geometry, n int, sigma_tangent f64, sigma_normal f64, mat Material) Gaussians {
	if n <= 0 {
		panic('need n > 0, got ${n}')
	}
	if sigma_tangent <= 0.0 || sigma_normal <= 0.0 {
		panic('need sigmas > 0, got (${sigma_tangent}, ${sigma_normal})')
	}
	mut g := Gaussians{
		material: mat
		splats:   []Gaussian{cap: n}
	}
	match geom {
		SphereGeometry {
			g.splats = sample_sphere(geom, n, sigma_tangent, sigma_normal, mat)
		}
		EllipsoidGeometry {
			g.splats = sample_ellipsoid(geom, n, sigma_tangent, sigma_normal, mat)
		}
		TorusGeometry {
			g.splats = sample_torus(geom, n, sigma_tangent, sigma_normal, mat)
		}
		ConeGeometry {
			g.splats = sample_cone(geom, n, sigma_tangent, sigma_normal, mat)
		}
		CylinderGeometry {
			g.splats = sample_cylinder(geom, n, sigma_tangent, sigma_normal, mat)
		}
		BoxGeometry {
			g.splats = sample_box(geom, n, sigma_tangent, sigma_normal, mat)
		}
		PlaneGeometry {
			// infinite plane: default to the half-size 1.0 patch (see
			// sample_gaussians_on_plane for explicit extents)
			g.splats = sample_gaussians_on_plane(geom, 1.0, n, sigma_tangent, sigma_normal, mat).splats
		}
		CyclideGeometry {
			// delegate to the exact Route A cyclide sampler (u, v grid)
			n_v, n_u := igrid(n, 2.0)
			g.splats = sample_gaussians_on_cyclide(dupin_cyclide(geom.a, geom.b, geom.d, geom.shift),
				n_u, n_v, sigma_tangent, sigma_normal, mat).splats
		}
		AffineGeometry {
			g.splats = sample_affine(geom, n, sigma_tangent, sigma_normal, mat)
		}
		TrimeshGeometry {
			panic('splat sampling on trimeshes is not supported')
		}
		CsgGeometry {
			panic('splat sampling on CSG geometry is not supported')
		}
		CircleGeometry {
			panic('splat sampling on circles is not supported (not a surface)')
		}
		SplatsGeometry {
			panic('cannot sample splats on a splat set')
		}
	}
	return g
}
