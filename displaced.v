module cga

// Displaced surfaces: a general surface expressed as a base CGA primitive
// plus a residual displacement field,
//   F(x) = d_base(x) - scale * r(uv(x)) = 0
// where d_base is the base signed distance (cyclide: the normalised implicit
// F/|grad F|, a first-order SDF), uv(x) maps an arbitrary point to the base's
// parameter domain and r is a bilinearly-sampled residual grid.  The MLX
// kernels (bracket + march + refine) live in displaced_kernel.v; this file
// has the types, the CPU parametric evaluators and the residual baker.
//
// Supported bases (local frames):
//   sphere    u = atan2(y,x)/(2pi)+0.5 (wrap), v = acos(z/r)/pi in [0,1] (clamp)
//   cylinder  INFINITE, axis +z: u = atan2(y,x)/(2pi)+0.5 (wrap),
//             v = axial/(2r) covering the band [0, 2r]; OUTSIDE the band the
//             surface is suppressed (the marcher sees F > 0), so a displaced
//             cylinder is always bounded to the band
//   plane     n.x = d; in-plane axes b1 = e1 projected on the plane (e2 if
//             e1 nearly parallel), b2 = b1 x n; u = rel.b1, v = rel.b2 in
//             WORLD units, one grid node per unit, tiled (repeat)
//   cyclide   canonical params (a,b,d,shift); u,v in [0,2pi) both periodic
//             (wrap), uv(x) via the closed-form atan2 inversion
// Other bases (ellipsoid/torus/cone/box/trimesh/csg/finite cylinder) panic in
// the constructor.
//
// Grid convention: row-major res_v x res_u, node (i, j) sits exactly at
// u = i/res_u, v = j/(res_v-1) (cyclide: u = 2pi*i/res_u, v = 2pi*j/res_v).
// Periodic domains store res distinct nodes per period (NO duplicated seam
// node); the sampler wraps.  Baked grids are wrap-consistent by construction.
import math
import mlx

// DisplacedGeometry is a base primitive plus a residual displacement grid.
pub struct DisplacedGeometry {
pub:
	base     Geometry // sphere / plane / infinite cylinder / cyclide
	residual []f32    // row-major (res_v * res_u) residual grid
	res_u    int
	res_v    int
	scale    f64 // world units per residual unit
}

// displaced_geometry builds a displaced surface, validating base and grid.
pub fn displaced_geometry(base Geometry, residual []f32, res_u int, res_v int, scale f64) DisplacedGeometry {
	match base {
		SphereGeometry, PlaneGeometry, CyclideGeometry {}
		CylinderGeometry {
			if base.half > 0.0 {
				panic('displaced cylinder base must be infinite (cylinder length <= 0)')
			}
		}
		else {
			panic('displaced base must be sphere/plane/cylinder/cyclide, got ${base.type_name()}')
		}
	}
	if res_u < 2 || res_v < 2 {
		panic('residual grid must be >= 2x2, got ${res_u}x${res_v}')
	}
	if residual.len != res_u * res_v {
		panic('residual grid has ${residual.len} entries, expected ${res_u * res_v}')
	}
	if scale <= 0.0 {
		panic('need scale > 0, got ${scale}')
	}
	return DisplacedGeometry{
		base:     base
		residual: residual
		res_u:    res_u
		res_v:    res_v
		scale:    scale
	}
}

// max_abs_residual returns max |scale * r| over the grid (bracket inflation).
fn (g DisplacedGeometry) max_abs_residual() f64 {
	mut m := 0.0
	for r in g.residual {
		m = math.max(m, math.abs(f64(r)))
	}
	return g.scale * m
}

// base_node evaluates the base surface point and outward normal at grid node
// (i, j) in LOCAL space, matching the MLX uv conventions EXACTLY (same u=0
// origin, same pole placement).
pub fn (g DisplacedGeometry) base_node(i int, j int) ([3]f64, [3]f64) {
	u := f64(i) / f64(g.res_u)
	v := f64(j) / f64(g.res_v - 1)
	phi := 2.0 * math.pi * (u - 0.5) // atan2 argument at node u
	match g.base {
		SphereGeometry {
			theta := math.pi * v
			st := math.sin(theta)
			dir := [st * math.cos(phi), st * math.sin(phi), math.cos(theta)]!
			r := g.base.radius
			return [r * dir[0], r * dir[1], r * dir[2]]!, dir
		}
		CylinderGeometry {
			r := g.base.radius
			return [r * math.cos(phi), r * math.sin(phi), 2.0 * r * v]!, [
				math.cos(phi),
				math.sin(phi),
				0.0,
			]!
		}
		PlaneGeometry {
			n, d, b1, b2 := plane_frame(g.base.blade)
			// one node per world unit, tiled: node (i, j) at (i, j)
			return [n[0] * d + b1[0] * f64(i) + b2[0] * f64(j),
				n[1] * d + b1[1] * f64(i) + b2[1] * f64(j),
				n[2] * d + b1[2] * f64(i) +
					b2[2] * f64(j)]!, n
		}
		CyclideGeometry {
			cy := dupin_cyclide(g.base.a, g.base.b, g.base.d, g.base.shift)
			p := cy.surface(2.0 * math.pi * u, 2.0 * math.pi * v)
			return p, cy.normal(p[0], p[1], p[2])
		}
		else {
			panic('displaced base must be sphere/plane/cylinder/cyclide')
		}
	}
}

// plane_frame extracts (unit normal, distance, in-plane axes b1/b2) from a
// plane blade.  b1 = e1 projected onto the plane (e2 when e1 is nearly
// parallel to n), b2 = b1 x n.  Shared by the baker and geom_to_camera.
fn plane_frame(blade Multivector) ([3]f64, f64, [3]f64, [3]f64) {
	n := vec3_unit(blade.euclidean_vector())
	d := blade.einf_coeff()
	mut seed := [1.0, 0.0, 0.0]!
	if math.abs(n[0]) >= 0.9 {
		seed = [0.0, 1.0, 0.0]!
	}
	sn := vec3_dot(seed, n)
	b1 := vec3_unit([seed[0] - sn * n[0], seed[1] - sn * n[1], seed[2] - sn * n[2]]!)
	b2 := vec3_cross(b1, n)
	return n, d, b1, b2
}

// residual_at samples the residual grid on the CPU (bilinear, same wrap rules
// as the MLX sampler) — for tests and diagnostics.
pub fn (g DisplacedGeometry) residual_at(u f64, v f64) f64 {
	// periodic u for sphere/cylinder/cyclide; plane tiles in world units
	mut x := u * f64(g.res_u)
	if g.base is PlaneGeometry {
		x = u
	}
	x -= math.floor(x / f64(g.res_u)) * f64(g.res_u) // wrap into [0, res_u)
	i0 := int(math.floor(x)) % g.res_u
	i1 := (i0 + 1) % g.res_u
	fx := x - math.floor(x)
	mut r00 := g.residual[0]
	mut r10 := r00
	mut r01 := r00
	mut r11 := r00
	match g.base {
		CyclideGeometry {
			// v periodic too
			mut y := v * f64(g.res_v)
			y -= math.floor(y / f64(g.res_v)) * f64(g.res_v)
			j0 := int(math.floor(y)) % g.res_v
			j1 := (j0 + 1) % g.res_v
			fy := y - math.floor(y)
			r00 = g.residual[j0 * g.res_u + i0]
			r10 = g.residual[j0 * g.res_u + i1]
			r01 = g.residual[j1 * g.res_u + i0]
			r11 = g.residual[j1 * g.res_u + i1]
			return f64(r00) * (1.0 - fx) * (1.0 - fy) + f64(r10) * fx * (1.0 - fy) +
				f64(r01) * (1.0 - fx) * fy + f64(r11) * fx * fy
		}
		PlaneGeometry {
			// v tiles in world units as well
			mut y := v
			y -= math.floor(y / f64(g.res_v)) * f64(g.res_v)
			j0 := int(math.floor(y)) % g.res_v
			j1 := (j0 + 1) % g.res_v
			fy := y - math.floor(y)
			r00 = g.residual[j0 * g.res_u + i0]
			r10 = g.residual[j0 * g.res_u + i1]
			r01 = g.residual[j1 * g.res_u + i0]
			r11 = g.residual[j1 * g.res_u + i1]
			return f64(r00) * (1.0 - fx) * (1.0 - fy) + f64(r10) * fx * (1.0 - fy) +
				f64(r01) * (1.0 - fx) * fy + f64(r11) * fx * fy
		}
		else {
			// sphere v: clamp to [0, 1]; cylinder v: the surface is suppressed
			// outside the band (the MLX kernel substitutes a huge negative
			// residual); residual_at reports 0 there (no grid samples outside)
			y := v * f64(g.res_v - 1)
			if g.base is CylinderGeometry {
				if y < 0.0 || y > f64(g.res_v - 1) {
					return 0.0
				}
			}
			yc := math.min(math.max(y, 0.0), f64(g.res_v - 1))
			j0 := math.min(int(math.floor(yc)), g.res_v - 2)
			fy := yc - f64(j0)
			r00 = g.residual[j0 * g.res_u + i0]
			r10 = g.residual[j0 * g.res_u + i1]
			r01 = g.residual[(j0 + 1) * g.res_u + i0]
			r11 = g.residual[(j0 + 1) * g.res_u + i1]
			return f64(r00) * (1.0 - fx) * (1.0 - fy) + f64(r10) * fx * (1.0 - fy) +
				f64(r01) * (1.0 - fx) * fy + f64(r11) * fx * fy
		}
	}
}

// --- residual baking ----------------------------------------------------------

// bake_residual bakes the residual of `target` onto `base` by casting rays
// along the base's node normals, in LOCAL space (both geometries unpossessed;
// pose the displaced mesh afterwards via its Object3D motor).
//
// Convention: for node (i, j) with base point p0 and normal n, the ray
// p0 + eps*n + t*n hits the target at t > 0  =>  residual = +(eps + t)
// (outward bump); if that misses, the reverse ray hits at t  =>
// residual = eps - t (negative = dimple); if both miss, residual = 0.
// residual = displacement / scale, so pick scale >= the max expected
// |displacement|.  The target is intersected through the standard MLX
// geom_intersect machinery with the identity motor.
pub fn bake_residual(base Geometry, target Geometry, res_u int, res_v int, scale f64) DisplacedGeometry {
	probe := displaced_geometry(base, []f32{len: res_u * res_v, init: f32(0.0)}, res_u, res_v,
		scale)
	n_nodes := res_u * res_v
	eps := 1e-3 * base_char_len(base)
	mut o_flat := []f32{len: 3 * n_nodes}
	mut d_flat := []f32{len: 3 * n_nodes}
	mut n_flat := []f32{len: 3 * n_nodes}
	for j in 0 .. res_v {
		for i in 0 .. res_u {
			p0, n := probe.base_node(i, j)
			k := 3 * (j * res_u + i)
			o_flat[k] = f32(p0[0] + eps * n[0])
			o_flat[k + 1] = f32(p0[1] + eps * n[1])
			o_flat[k + 2] = f32(p0[2] + eps * n[2])
			n_flat[k] = f32(n[0])
			n_flat[k + 1] = f32(n[1])
			n_flat[k + 2] = f32(n[2])
			d_flat[k] = f32(n[0])
			d_flat[k + 1] = f32(n[1])
			d_flat[k + 2] = f32(n[2])
		}
	}
	o := mlx.array_f32(o_flat, [n_nodes, 3])
	d := mlx.array_f32(d_flat, [n_nodes, 3])
	dn := mlx.array_f32(n_flat, [n_nodes, 3]).negative()
	tp := geom_to_camera(target, motor_identity())
	t_out, _, m_out := geom_intersect(tp, o, d)
	t_in, _, m_in := geom_intersect(tp, o, dn)
	ts_o := t_out.data_f32()
	ms_o := m_out.data_bool()
	ts_i := t_in.data_f32()
	ms_i := m_in.data_bool()
	mut grid := []f32{len: n_nodes}
	for k in 0 .. n_nodes {
		mut r := f32(0.0)
		if ms_o[k] {
			r = f32(eps) + ts_o[k]
		} else if ms_i[k] {
			r = f32(eps) - ts_i[k]
		}
		grid[k] = r / f32(scale)
	}
	o.free()
	d.free()
	dn.free()
	return displaced_geometry(base, grid, res_u, res_v, scale)
}

// base_char_len returns a characteristic length of the base (baker epsilon).
fn base_char_len(base Geometry) f64 {
	return match base {
		SphereGeometry { base.radius }
		CylinderGeometry { base.radius }
		CyclideGeometry { base.a }
		PlaneGeometry { 1.0 }
		else { 1.0 }
	}
}

// world_step returns the approximate world-space grid spacing (for the
// finite-difference eps and the bracket margin).
fn (g DisplacedGeometry) world_step() f64 {
	base := g.base
	return match base {
		SphereGeometry {
			base.radius * math.min(2.0 * math.pi / f64(g.res_u), math.pi / f64(g.res_v - 1))
		}
		CylinderGeometry {
			base.radius * math.min(2.0 * math.pi / f64(g.res_u), 2.0 / f64(g.res_v - 1))
		}
		CyclideGeometry {
			math.min(2.0 * math.pi * base.a / f64(g.res_u), 2.0 * math.pi * base.d / f64(g.res_v))
		}
		PlaneGeometry {
			1.0 // one node per world unit
		}
		else {
			1.0
		}
	}
}
