module cga

// Intersection kernels for the non-blade primitives (cone / torus / ellipsoid /
// cyclide), which use the affine ray-inverse transform (see affine.v).

import mlx
import math

// dk_roots solves a monic quartic t^4 + c3 t^3 + c2 t^2 + c1 t + c0 by
// Durand-Kerner iteration (50 rounds, complex64).  Returns (N,4) sorted real
// roots (invalid = +inf).
fn dk_roots(c3 mlx.Array, c2 mlx.Array, c1 mlx.Array, c0 mlx.Array) mlx.Array {
	rad := s_add(mlx.stack([c3.abs(), c2.abs(), c1.abs(), c0.abs()], -1).max_axis(-1,
		true), 1.0)
	seed := mlx.array_with([mlx.Complex64{
		real: 0.4
		imag: 0.9
	}, mlx.Complex64{
		real: -0.65
		imag: 0.72
	}, mlx.Complex64{
		real: -0.74
		imag: -0.67
	}, mlx.Complex64{
		real: 0.73
		imag: -0.68
	}], [4], .complex64)
	mut z := rad.astype(.complex64).multiply(seed.expand_dims(0))
	c3c := c3.astype(.complex64).expand_dims(1)
	c2c := c2.astype(.complex64).expand_dims(1)
	c1c := c1.astype(.complex64).expand_dims(1)
	c0c := c0.astype(.complex64).expand_dims(1)
	idx4 := mlx.arange(0, 4, 1, .float32)
	eye := idx4.expand_dims(1).equal(idx4.expand_dims(0)).astype(.float32).expand_dims(0)
	for _ in 0 .. 50 {
		pz := c0c.add(z.multiply(c1c.add(z.multiply(c2c.add(z.multiply(c3c.add(z)))))))
		diff := z.expand_dims(2).subtract(z.expand_dims(1)).add(eye)
		denom := diff.prod_axis(2, false)
		z = z.subtract(pz.divide(denom))
	}
	re := z.real()
	im := z.imag()
	real_ok := im.abs().less_equal(s_mul(s_max(re.abs(), 1.0), 1e-3))
	roots := mlx.where(real_ok, re, inf_like(re))
	return roots.sort_axis(-1)
}

fn inf_like(a mlx.Array) mlx.Array {
	return mlx.full_like(a, mlx.f32_scalar(f32(math.inf(1))), .float32)
}

// affine_bounds transforms a local AABB's 8 corners by a_fwd (row-major 4x4).
fn affine_bounds(lo [3]f64, hi [3]f64, a_fwd [16]f64) [2][3]f64 {
	mut minp := [3]f64{}
	mut maxp := [3]f64{}
	mut first := true
	for i in [lo[0], hi[0]] {
		for j in [lo[1], hi[1]] {
			for k in [lo[2], hi[2]] {
				x := a_fwd[0] * i + a_fwd[1] * j + a_fwd[2] * k + a_fwd[3]
				y := a_fwd[4] * i + a_fwd[5] * j + a_fwd[6] * k + a_fwd[7]
				z := a_fwd[8] * i + a_fwd[9] * j + a_fwd[10] * k + a_fwd[11]
				if first {
					minp = [x, y, z]!
					maxp = [x, y, z]!
					first = false
				} else {
					if x < minp[0] {
						minp[0] = x
					}
					if y < minp[1] {
						minp[1] = y
					}
					if z < minp[2] {
						minp[2] = z
					}
					if x > maxp[0] {
						maxp[0] = x
					}
					if y > maxp[1] {
						maxp[1] = y
					}
					if z > maxp[2] {
						maxp[2] = z
					}
				}
			}
		}
	}
	return [minp, maxp]!
}

// --- cone -------------------------------------------------------------------

fn cone_local_interval(r f64, h f64, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array, mlx.Array, mlx.Array) {
	k := r / h
	k2 := 1.0 + k * k
	wz := s_sub(col(o, 2), h / 2.0)
	dz := col(d, 2)
	wd := o.multiply(d).sum_axis(-1, false).subtract(s_mul(dz, h / 2.0))
	ww := s_add(o.multiply(o).sum_axis(-1, false).subtract(s_mul(col(o, 2), h)), h * h /
		4.0)
	a := fs(1.0).subtract(s_mul(dz.multiply(dz), k2))
	b := s_mul(wd.subtract(s_mul(wz.multiply(dz), k2)), 2.0)
	c := ww.subtract(s_mul(wz.multiply(wz), k2))
	mut side_ok := s_gt(a.abs(), 1e-12)
	a_s := mlx.where(side_ok, a, mlx.full_like(a, mlx.f32_scalar(f32(1e-12)), .float32))
	disc := b.multiply(b).subtract(s_mul(a_s.multiply(c), 4.0))
	side_ok = side_ok.logical_and(s_gt(disc, 1e-12))
	sq := s_max(disc, 0.0).sqrt()
	r_lo := b.negative().subtract(sq).divide(s_mul(a_s, 2.0))
	r_hi := b.negative().add(sq).divide(s_mul(a_s, 2.0))
	st0 := r_lo.minimum(r_hi)
	st1 := r_lo.maximum(r_hi)
	safe_dz := mlx.where(s_gt(dz.abs(), 1e-9), dz, mlx.full_like(dz, mlx.f32_scalar(f32(1e-9)),
		.float32))
	t_top := wz.negative().divide(safe_dz)
	t_bot := s_add(wz, h).negative().divide(safe_dz)
	at0 := t_top.minimum(t_bot)
	at1 := t_top.maximum(t_bot)
	pos_a := s_gt(a, 1e-12)
	c1e := mlx.where(pos_a, st0.maximum(at0), at0)
	c1x := mlx.where(pos_a, st1.minimum(at1), st0.minimum(at1))
	v1 := side_ok.logical_and(c1e.less(c1x))
	c2e := st1.maximum(at0)
	c2x := at1
	v2 := side_ok.logical_and(pos_a.logical_not()).logical_and(c2e.less(c2x))
	enter := mlx.where(v1, c1e, c2e)
	exit_ := mlx.where(v1, c1x, c2x)
	valid := v1.logical_or(v2)

	n_s0 := cone_side_n(h, k2, o, d, enter)
	n_s1 := cone_side_n(h, k2, o, d, exit_)
	top_first := t_top.less(t_bot).expand_dims(1)
	ez := arr3(0.0, 0.0, 1.0).expand_dims(0)
	n_cap0 := mlx.where(top_first, ez, ez.negative())
	n_cap1 := mlx.where(top_first, ez.negative(), ez)
	enter_cap := enter.equal(at0).expand_dims(1)
	exit_cap := exit_.equal(at1).expand_dims(1)
	n0 := mlx.where(enter_cap, n_cap0, n_s0)
	n1 := mlx.where(exit_cap, n_cap1, n_s1)
	return enter, exit_, valid, n0, n1
}

fn cone_side_n(h f64, k2 f64, o mlx.Array, d mlx.Array, t mlx.Array) mlx.Array {
	p := o.add(t.expand_dims(1).multiply(d))
	s := s_sub(col(p, 2), h / 2.0)
	g := mlx.stack([col(p, 0), col(p, 1), s_mul(s, 1.0 - k2)], -1)
	norm := g.multiply(g).sum_axis(-1, true).sqrt()
	return g.divide(mlx.where(s_gt(norm, 1e-12), norm, mlx.ones_like(norm)))
}

fn cone_local_intersect(r f64, h f64, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	enter, exit_, valid, n0, n1 := cone_local_interval(r, h, o, d)
	hit_enter := valid.logical_and(s_gt(enter, 1e-6))
	hit_exit := valid.logical_and(hit_enter.logical_not()).logical_and(s_gt(exit_, 1e-6))
	t := mlx.where(hit_enter, enter, exit_)
	mut n := mlx.where(hit_enter.expand_dims(1), n0, n1.negative())
	mask := hit_enter.logical_or(hit_exit)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn cone_intersect(p ConeParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	t_l, n_l, mask := cone_local_intersect(p.r, p.h, o_l, d_u)
	t := t_l.divide(col(lam, 0))
	mut n := affine_normal(n_l, p.a_inv3)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn cone_shadow(p ConeParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	enter, exit_, valid, _, _ := cone_local_interval(p.r, p.h, o_l, d_u)
	hit_enter := valid.logical_and(s_gt(enter, 1e-6))
	t := mlx.where(hit_enter, enter, exit_)
	mask := hit_enter.logical_or(valid.logical_and(s_gt(exit_, 1e-6)))
	return t.divide(col(lam, 0)), mask
}

pub fn cone_uv(p ConeParams, pos mlx.Array, n mlx.Array) mlx.Array {
	p_l := vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv))
	u := s_add(s_div(col(p_l, 1).arctan2(col(p_l, 0)), 2.0 * math.pi), 0.5)
	v := s_div(s_rsub(col(p_l, 2), p.h / 2.0), p.h)
	return mlx.stack([u, v], -1)
}

// --- ellipsoid --------------------------------------------------------------

pub fn ellipsoid_intersect(p EllipsoidParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	b := s_mul(o_l.multiply(d_u).sum_axis(-1, false), 2.0)
	cq := s_sub(o_l.multiply(o_l).sum_axis(-1, false), 1.0)
	disc := b.multiply(b).subtract(s_mul(cq, 4.0))
	valid := s_gt(disc, 1e-12)
	sq := s_max(disc, 0.0).sqrt()
	t1 := s_div(b.negative().subtract(sq), 2.0)
	t2 := s_div(b.negative().add(sq), 2.0)
	t_l := mlx.where(valid.logical_and(s_gt(t1, 1e-6)), t1, t2)
	mask := valid.logical_and(s_gt(t_l, 1e-6))
	mut n_l := o_l.add(t_l.expand_dims(1).multiply(d_u))
	inside := mask.logical_and(s_le(t1, 1e-6))
	n_l = mlx.where(inside.expand_dims(1), n_l.negative(), n_l)
	t := t_l.divide(col(lam, 0))
	mut n := affine_normal(n_l, p.a_inv3)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn ellipsoid_shadow(p EllipsoidParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	b := s_mul(o_l.multiply(d_u).sum_axis(-1, false), 2.0)
	cq := s_sub(o_l.multiply(o_l).sum_axis(-1, false), 1.0)
	disc := b.multiply(b).subtract(s_mul(cq, 4.0))
	valid := s_gt(disc, 1e-12)
	sq := s_max(disc, 0.0).sqrt()
	t1 := s_div(b.negative().subtract(sq), 2.0)
	t2 := s_div(b.negative().add(sq), 2.0)
	t_l := mlx.where(valid.logical_and(s_gt(t1, 1e-6)), t1, t2)
	return t_l.divide(col(lam, 0)), valid.logical_and(s_gt(t_l, 1e-6))
}

pub fn ellipsoid_uv(p EllipsoidParams, pos mlx.Array, n mlx.Array) mlx.Array {
	p_l := vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv))
	u := s_add(s_div(col(p_l, 1).arctan2(col(p_l, 0)), 2.0 * math.pi), 0.5)
	v := s_div(s_clip(col(p_l, 2), -1.0, 1.0).arccos(), math.pi)
	return mlx.stack([u, v], -1)
}

// --- torus ------------------------------------------------------------------

fn torus_local_crossings(major f64, minor f64, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	r2 := major * major
	oo := o.multiply(o).sum_axis(-1, false)
	od := o.multiply(d).sum_axis(-1, false)
	g := s_add(oo, r2 - minor * minor)
	c3 := s_mul(od, 4.0)
	c2 := s_mul(g, 2.0).add(s_mul(od.multiply(od), 4.0)).subtract(s_mul(col(d, 0).multiply(col(d, 0)).add(col(d, 1).multiply(col(d, 1))), 4.0 * r2))
	c1 := s_mul(od.multiply(g), 4.0).subtract(s_mul(col(o, 0).multiply(col(d,
		0)).add(col(o, 1).multiply(col(d, 1))), 8.0 * r2))
	c0 := g.multiply(g).subtract(s_mul(col(o, 0).multiply(col(o, 0)).add(col(o,
		1).multiply(col(o, 1))), 4.0 * r2))
	ts := dk_roots(c3, c2, c1, c0)
	valid := ts.isfinite()
	safe_t := mlx.where(valid, ts, mlx.zeros_like(ts))
	p := o.expand_dims(1).add(safe_t.expand_dims(2).multiply(d.expand_dims(1)))
	s := s_add(p.multiply(p).sum_axis(-1, true), r2 - minor * minor)
	s1 := col(s, 0)
	p0 := p.take_axis(mlx.int_scalar(0), 2)
	p1 := p.take_axis(mlx.int_scalar(1), 2)
	p2 := p.take_axis(mlx.int_scalar(2), 2)
	fac := s_sub(s1, 2.0 * r2)
	grad := mlx.stack([fac.multiply(p0), fac.multiply(p1), s1.multiply(p2)], -1)
	norm := grad.multiply(grad).sum_axis(-1, true).sqrt()
	mut ns := grad.divide(mlx.where(s_gt(norm, 1e-12), norm, mlx.ones_like(norm)))
	ns = mlx.where(valid.expand_dims(2), ns, mlx.zeros_like(ns))
	return ts, ns, valid
}

fn torus_local_contains(major f64, minor f64, p mlx.Array) mlx.Array {
	r2 := major * major
	f := s_add(p.multiply(p).sum_axis(-1, false), r2 - minor * minor).square().subtract(s_mul(col(p,
		0).multiply(col(p, 0)).add(col(p, 1).multiply(col(p, 1))), 4.0 * r2))
	return s_lt(f, 0.0)
}

pub fn torus_intersect(p TorusParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, ns, valid := torus_local_crossings(p.major, p.minor, o_l, d_u)
	pos := valid.logical_and(s_gt(ts, 1e-6))
	cand := mlx.where(pos, ts, inf_like(ts))
	t_l := cand.min_axis(-1, false)
	mask := t_l.isfinite()
	idx := cand.argmin_axis(-1, false)
	mut n_l := ns.take_along_axis(idx.expand_dims(1).expand_dims(2).broadcast_to([ns.shape()[0], 1, 3]), 1).take_axis(mlx.int_scalar(0), 1)
	inside := mask.logical_and(torus_local_contains(p.major, p.minor, o_l))
	n_l = mlx.where(inside.expand_dims(1), n_l.negative(), n_l)
	t := t_l.divide(col(lam, 0))
	mut n := affine_normal(n_l, p.a_inv3)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn torus_shadow(p TorusParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, _, valid := torus_local_crossings(p.major, p.minor, o_l, d_u)
	pos := valid.logical_and(s_gt(ts, 1e-6))
	cand := mlx.where(pos, ts, inf_like(ts))
	t_l := cand.min_axis(-1, false)
	return t_l.divide(col(lam, 0)), t_l.isfinite()
}

pub fn torus_uv(p TorusParams, pos mlx.Array, n mlx.Array) mlx.Array {
	p_l := vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv))
	rho := col(p_l, 0).square().add(col(p_l, 1).square()).sqrt()
	u := s_add(s_div(col(p_l, 1).arctan2(col(p_l, 0)), 2.0 * math.pi), 0.5)
	v := s_add(s_div(col(p_l, 2).arctan2(s_sub(rho, p.major)), 2.0 * math.pi), 0.5)
	return mlx.stack([u, v], -1)
}

// --- cyclide ----------------------------------------------------------------

fn cyclide_local_crossings(a f64, b f64, dd f64, c f64, shift [3]f64, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	ox := s_sub(col(o, 0), shift[0])
	oy := s_sub(col(o, 1), shift[1])
	oz := s_sub(col(o, 2), shift[2])
	dx := col(d, 0)
	dy := col(d, 1)
	dz := col(d, 2)
	big_a := ox.multiply(ox).add(oy.multiply(oy)).add(oz.multiply(oz))
	b1 := ox.multiply(dx).add(oy.multiply(dy)).add(oz.multiply(dz))
	bb := b * b - dd * dd
	g := s_add(big_a, bb)
	p0 := s_sub(s_mul(ox, a), c * dd)
	p1 := s_mul(dx, a)
	c3 := s_mul(b1, 4.0)
	c2 := s_mul(g, 2.0).add(s_mul(b1.multiply(b1), 4.0)).subtract(s_mul(p1.multiply(p1), 4.0)).subtract(s_mul(dy.multiply(dy), 4.0 * b * b))
	c1 := s_mul(b1.multiply(g), 4.0).subtract(s_mul(p0.multiply(p1), 8.0)).subtract(s_mul(oy.multiply(dy),
		8.0 * b * b))
	c0 := g.multiply(g).subtract(s_mul(p0.multiply(p0), 4.0)).subtract(s_mul(oy.multiply(oy),
		4.0 * b * b))
	ts := dk_roots(c3, c2, c1, c0)
	valid := ts.isfinite()
	safe_t := mlx.where(valid, ts, mlx.zeros_like(ts))
	p := o.expand_dims(1).subtract(arr3v(shift).expand_dims(0).expand_dims(0)).add(safe_t.expand_dims(2).multiply(d.expand_dims(1)))
	mut ns := cyclide_normal(a, b, dd, c, p)
	ns = mlx.where(valid.expand_dims(2), ns, mlx.zeros_like(ns))
	return ts, ns, valid
}

fn cyclide_normal(a f64, b f64, dd f64, c f64, p mlx.Array) mlx.Array {
	bb := b * b - dd * dd
	rho := p.multiply(p).sum_axis(-1, true)
	g := s_add(rho, bb)
	g0 := col(g, 0)
	p0 := p.take_axis(mlx.int_scalar(0), 2)
	p1 := p.take_axis(mlx.int_scalar(1), 2)
	p2 := p.take_axis(mlx.int_scalar(2), 2)
	grad := mlx.stack([s_mul(p0.multiply(g0), 4.0).subtract(s_mul(s_sub(s_mul(p0,
		a), c * dd), 8.0 * a)), s_mul(p1.multiply(g0), 4.0).subtract(s_mul(p1,
		8.0 * b * b)), s_mul(p2.multiply(g0), 4.0)], -1)
	norm := grad.multiply(grad).sum_axis(-1, true).sqrt()
	return grad.divide(mlx.where(s_gt(norm, 1e-12), norm, mlx.ones_like(norm)))
}

fn cyclide_local_contains(a f64, b f64, dd f64, c f64, shift [3]f64, p mlx.Array) mlx.Array {
	x := s_sub(col(p, 0), shift[0])
	y := s_sub(col(p, 1), shift[1])
	z := s_sub(col(p, 2), shift[2])
	bb := b * b - dd * dd
	rho := x.multiply(x).add(y.multiply(y)).add(z.multiply(z))
	f := s_add(rho, bb).square().subtract(s_mul(s_sub(s_mul(x, a), c * dd).square(),
		4.0)).subtract(s_mul(y.multiply(y), 4.0 * b * b))
	return s_lt(f, 0.0)
}

pub fn cyclide_intersect(p CyclideParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, ns, valid := cyclide_local_crossings(p.a, p.b, p.d, p.c, p.shift, o_l, d_u)
	pos := valid.logical_and(s_gt(ts, 1e-6))
	cand := mlx.where(pos, ts, inf_like(ts))
	t_l := cand.min_axis(-1, false)
	mask := t_l.isfinite()
	idx := cand.argmin_axis(-1, false)
	mut n_l := ns.take_along_axis(idx.expand_dims(1).expand_dims(2).broadcast_to([ns.shape()[0], 1, 3]), 1).take_axis(mlx.int_scalar(0), 1)
	inside := mask.logical_and(cyclide_local_contains(p.a, p.b, p.d, p.c, p.shift, o_l))
	n_l = mlx.where(inside.expand_dims(1), n_l.negative(), n_l)
	t := t_l.divide(col(lam, 0))
	mut n := affine_normal(n_l, p.a_inv3)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn cyclide_shadow(p CyclideParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, _, valid := cyclide_local_crossings(p.a, p.b, p.d, p.c, p.shift, o_l, d_u)
	pos := valid.logical_and(s_gt(ts, 1e-6))
	cand := mlx.where(pos, ts, inf_like(ts))
	t_l := cand.min_axis(-1, false)
	return t_l.divide(col(lam, 0)), t_l.isfinite()
}

pub fn cyclide_uv(p CyclideParams, pos mlx.Array, n mlx.Array) mlx.Array {
	p_l := vecmat(pos, mat3_transpose(p.a_inv3)).add(arr3v(p.t_inv))
	x := s_sub(col(p_l, 0), p.shift[0])
	y := s_sub(col(p_l, 1), p.shift[1])
	z := s_sub(col(p_l, 2), p.shift[2])
	rho := x.multiply(x).add(y.multiply(y)).add(z.multiply(z))
	u := s_add(s_div(s_mul(y, 2.0 * p.b).arctan2(s_mul(s_sub(s_mul(x, p.a), p.c *
		p.d), 2.0)), 2.0 * math.pi), 0.5)
	v := s_add(s_div(s_mul(z, 2.0 * p.b).arctan2(s_rsub(rho, p.d * p.d + p.b *
		p.b)), 2.0 * math.pi), 0.5)
	return mlx.stack([u, v], -1)
}
