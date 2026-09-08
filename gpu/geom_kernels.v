// geom_kernels.v — mlx batch kernels extracted from the cga root package.
module cga_gpu
import mlx
import mlx_ops

import math
import cga { AffineGeometry, AffineParams, BoxGeometry, BoxParams, CircleGeometry, CircleParams, ConeGeometry, ConeParams, CsgGeometry, CsgParams, CyclideGeometry, CyclideParams, CylinderGeometry, CylinderParams, EllipsoidGeometry, EllipsoidParams, Geometry, GeometryParams, Mat3, Multivector, PlaneGeometry, PlaneParams, SphereGeometry, SphereParams, TorusGeometry, TorusParams, TrimeshGeometry, TrimeshParams, affine_from_motor, e1, e2, e3, mat3_new, mat3_transpose, motor_identity, point, sphere_from_dual }

pub fn mat3_to_mlx(m Mat3) mlx.Array {
	return mlx.array_f32([f32(m[0][0]), f32(m[0][1]), f32(m[0][2]), f32(m[1][0]), f32(m[1][1]),
		f32(m[1][2]), f32(m[2][0]), f32(m[2][1]), f32(m[2][2])], [3, 3])
}

pub fn vecmat(v mlx.Array, m Mat3) mlx.Array {
	mm := mat3_to_mlx(m)
	return v.expand_dims(-1).multiply(mm).sum_axis(-2, false)
}

pub fn affine_to_local(a_inv3 Mat3, t_inv [3]f64, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	a3t := mat3_transpose(a_inv3)
	t3 := mlx_ops.arr3v(t_inv)
	o_l := vecmat(o, a3t).add(t3)
	d_l := vecmat(d, a3t)
	mut lam := d_l.multiply(d_l).sum_axis(-1, true).sqrt()
	lam = mlx.where(mlx_ops.s_gt(lam, 1e-12), lam, mlx.ones_like(lam))
	return o_l, d_l.divide(lam), lam
}

pub fn affine_normal(n_l mlx.Array, a_inv3 Mat3) mlx.Array {
	n := vecmat(n_l, a_inv3)
	norm := n.multiply(n).sum_axis(-1, true).sqrt()
	return n.divide(mlx.where(mlx_ops.s_gt(norm, 1e-12), norm, mlx.ones_like(norm)))
}

pub fn affine_intersect(p AffineParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	t_l, n_l, mask := geom_intersect(p.inner, o_l, d_u)
	t := t_l.divide(col(lam, 0))
	mut n := affine_normal(n_l, p.a_inv3)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn affine_shadow(p AffineParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	t_l, mask := geom_shadow(p.inner, o_l, d_u)
	return t_l.divide(col(lam, 0)), mask
}

pub fn affine_uv(p AffineParams, pos mlx.Array, n mlx.Array) mlx.Array {
	p_l := affine_point_to_local(p.a_inv3, p.t_inv, pos)
	return geom_uv(p.inner, p_l, n)
}

pub fn affine_crossings(p AffineParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, mut ns, valid := geom_crossings(p.inner, o_l, d_u)
	ns = affine_normal(ns, p.a_inv3)
	return ts.divide(col(lam, 0).expand_dims(1)), ns, valid
}

pub fn affine_contains(p AffineParams, pos mlx.Array) mlx.Array {
	p_l := affine_point_to_local(p.a_inv3, p.t_inv, pos)
	return geom_contains(p.inner, p_l)
}

fn csg_crossings(p CsgParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	mut ts_l := []mlx.Array{}
	mut ns_l := []mlx.Array{}
	mut vs_l := []mlx.Array{}
	for cp in p.children {
		t, n, v := geom_crossings(cp, o, d)
		ts_l << t
		ns_l << n
		vs_l << v
	}
	return mlx.concatenate(ts_l, 1), mlx.concatenate(ns_l, 1), mlx.concatenate(vs_l, 1)
}

fn csg_contains(p CsgParams, pos mlx.Array) mlx.Array {
	if p.op == .difference {
		first := geom_contains(p.children[0], pos)
		mut rest := mlx.zeros_like(first)
		for cp in p.children[1..] {
			rest = rest.logical_or(geom_contains(cp, pos))
		}
		return first.logical_and(rest.logical_not())
	}
	mut acc := geom_contains(p.children[0], pos)
	for cp in p.children[1..] {
		cc := geom_contains(cp, pos)
		acc = if p.op == .union { acc.logical_or(cc) } else { acc.logical_and(cc) }
	}
	return acc
}

fn csg_nearest_surface(p CsgParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	ts, ns, _ := csg_crossings(p, o, d)
	order := ts.argsort_axis(1)
	ts_s := ts.take_along_axis(order, 1)
	ns_s := ns.take_along_axis(order.expand_dims(2), 1)
	ts_f := mlx.where(ts_s.isfinite(), ts_s, mlx.zeros_like(ts_s))
	delta := 1e-4
	p_plus := o.expand_dims(1).add(mlx_ops.s_add(ts_f, delta).expand_dims(2).multiply(d.expand_dims(1)))
	p_minus :=
		o.expand_dims(1).add(mlx_ops.s_sub(ts_f, delta).expand_dims(2).multiply(d.expand_dims(1)))
	in_plus := csg_contains(p, p_plus)
	in_minus := csg_contains(p, p_minus)
	flip :=
		in_plus.not_equal(in_minus).logical_and(mlx_ops.s_gt(ts_s, 1e-6)).logical_and(ts_s.isfinite())
	cand := mlx.where(flip, ts_s, inf_like(ts_s))
	t := cand.min_axis(1, false)
	mask := t.isfinite()
	idx := cand.argmin_axis(1, false)
	mut n := ns_s.take_along_axis(idx.expand_dims(1).expand_dims(2).broadcast_to([ns_s.shape()[0],
		1, 3]), 1).take_axis(mlx.int_scalar(0), 1)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn csg_intersect(p CsgParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	return csg_nearest_surface(p, o, d)
}

pub fn csg_shadow(p CsgParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	t, _, mask := csg_nearest_surface(p, o, d)
	return t, mask
}

pub fn csg_uv(p CsgParams, pos mlx.Array, n mlx.Array) mlx.Array {
	mut uv := mlx.zeros([pos.shape()[0], 2], .float32)
	mut found := mlx.zeros([pos.shape()[0]], .bool_)
	delta := 1e-4
	for cp in p.children {
		bp := pos.add(mlx_ops.s_mul(n, delta))
		bm := pos.subtract(mlx_ops.s_mul(n, delta))
		boundary := geom_contains(cp, bp).not_equal(geom_contains(cp, bm))
		pick := boundary.logical_and(found.logical_not())
		uv_c := geom_uv(cp, pos, n)
		uv = mlx.where(pick.expand_dims(1), uv_c, uv)
		found = found.logical_or(pick)
	}
	return uv
}

pub fn geom_to_camera(g Geometry, m Multivector) GeometryParams {
	return match g {
		SphereGeometry {
			s := m.apply(g.blade)
			c, r := sphere_from_dual(s)
			SphereParams{
				c:    c
				r:    r
				axes: [vec3_unit(dir3(m.apply(e1()))), vec3_unit(dir3(m.apply(e2()))),
					vec3_unit(dir3(m.apply(e3())))]!
			}
		}
		PlaneGeometry {
			pi := m.apply(g.blade)
			PlaneParams{
				n: vec3_unit(dir3(pi))
				d: pi.einf_coeff()
			}
		}
		CylinderGeometry {
			CylinderParams{
				q: m.apply(point(0.0, 0.0, 0.0)).coords()
				u: vec3_unit(dir3(m.apply(e3())))
				r: g.radius
				h: g.half
			}
		}
		BoxGeometry {
			hf := g.half
			BoxParams{
				c:    m.apply(point(0.0, 0.0, 0.0)).coords()
				axes: [vec3_unit(dir3(m.apply(e1()))), vec3_unit(dir3(m.apply(e2()))),
					vec3_unit(dir3(m.apply(e3())))]!
				half: hf
			}
		}
		CircleGeometry {
			CircleParams{
				c: m.apply(point(0.0, 0.0, 0.0)).coords()
				n: vec3_unit(dir3(m.apply(e3())))
				r: g.radius
			}
		}
		ConeGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			ConeParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				r:      g.radius
				h:      g.height
			}
		}
		TorusGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			TorusParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				major:  g.major
				minor:  g.minor
			}
		}
		EllipsoidGeometry {
			rr := g.radii
			diag := mat3_new([rr[0], 0.0, 0.0]!, [0.0, rr[1], 0.0]!, [0.0, 0.0, rr[2]]!)
			ai, ti, af := affine_from_motor(m, diag)
			EllipsoidParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
			}
		}
		CyclideGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			sh := g.shift
			CyclideParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				a:      g.a
				b:      g.b
				d:      g.d
				c:      math.sqrt(g.a * g.a - g.b * g.b)
				shift:  sh
			}
		}
		TrimeshGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			glo := g.lo
			ghi := g.hi
			TrimeshParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				v0:     g.v0
				e1:     g.e1
				e2:     g.e2
				nrm:    g.nrm
				lo:     glo
				hi:     ghi
			}
		}
		CsgGeometry {
			mut ch := []GeometryParams{}
			for c in g.children {
				ch << geom_to_camera(c, m)
			}
			CsgParams{
				op:       g.op
				children: ch
			}
		}
		AffineGeometry {
			affine_to_camera(g, m)
		}
	}
}



pub fn affine_to_camera(g cga.AffineGeometry, m cga.Multivector) cga.AffineParams {
	ip := geom_to_camera(g.inner[0], cga.motor_identity())
	full := m.gp(g.motor)
	ai, ti, af := affine_from_motor(full, g.linear)
	return AffineParams{
		inner:  ip
		a_inv3: ai
		t_inv:  ti
		a_fwd:  af
	}
}

pub fn csg_bounds(p cga.CsgParams) ?[2][3]f64 {
	mut bnds := []?[2][3]f64{}
	for cp in p.children {
		bnds << geom_bounds(cp)
	}
	if p.op == .difference {
		return bnds[0]
	}
	mut bounded := []?[2][3]f64{}
	for b in bnds {
		if b != none {
			bounded << b
		}
	}
	if bounded.len == 0 {
		return none
	}
	if p.op == .union {
		mut bmin := [3]f64{}
		mut bmax := [3]f64{}
		mut first := true
		for b in bounded {
			bb := b or { return none }
			if first {
				bmin = bb[0]
				bmax = bb[1]
				first = false
			} else {
				for i in 0 .. 3 {
					if bb[0][i] < bmin[i] {
						bmin[i] = bb[0][i]
					}
					if bb[1][i] > bmax[i] {
						bmax[i] = bb[1][i]
					}
				}
			}
		}
		return [bmin, bmax]!
	}
	// intersection
	mut bmin := [3]f64{}
	mut bmax := [3]f64{}
	mut first := true
	for b in bounded {
		bb := b or { return none }
		if first {
			bmin = bb[0]
			bmax = bb[1]
			first = false
		} else {
			for i in 0 .. 3 {
				if bb[0][i] > bmin[i] {
					bmin[i] = bb[0][i]
				}
				if bb[1][i] < bmax[i] {
					bmax[i] = bb[1][i]
				}
			}
		}
	}
	return [bmin, bmax]!
}

fn to_f32_3(v [][3]f64) []f32 {
	mut out := []f32{len: v.len * 3}
	for i in 0 .. v.len {
		out[i * 3] = f32(v[i][0])
		out[i * 3 + 1] = f32(v[i][1])
		out[i * 3 + 2] = f32(v[i][2])
	}
	return out
}

// tri_mlx converts the CPU TrimeshParams fields to mlx arrays for the kernels.
fn tri_mlx(p cga.TrimeshParams) (mlx.Array, mlx.Array, mlx.Array, mlx.Array) {
	return mlx.array_f32(to_f32_3(p.v0), [p.v0.len, 3]), mlx.array_f32(to_f32_3(p.e1), [p.e1.len, 3]), mlx.array_f32(to_f32_3(p.e2), [p.e2.len, 3]), mlx.array_f32(to_f32_3(p.nrm), [p.nrm.len, 3])
}
