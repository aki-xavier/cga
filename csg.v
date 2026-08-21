module cga

// CSG booleans: the solid protocol (crossings / contains) plus the recursive
// CsgGeometry combinator.
import mlx
import math

// last_col extracts the i-th component along the last axis.
@[inline]
fn last_col(a mlx.Array, i int) mlx.Array {
	return a.take_axis(mlx.int_scalar(i), -1)
}

// affine_point_to_local maps a point into the local canonical frame.
fn affine_point_to_local(a_inv3 Mat3, t_inv [3]f64, pos mlx.Array) mlx.Array {
	return vecmat(pos, mat3_transpose(a_inv3)).add(mlx.arr3v(t_inv))
}

// --- crossings / contains for blade solids ----------------------------------

fn sphere_crossings(p SphereParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	c := mlx.arr3v(p.c)
	oc := o.subtract(c)
	b := mlx.s_mul(oc.multiply(d).sum_axis(-1, false), 2.0)
	cq := mlx.s_sub(oc.multiply(oc).sum_axis(-1, false), p.r * p.r)
	disc := b.multiply(b).subtract(mlx.s_mul(cq, 4.0))
	valid := mlx.s_gt(disc, 1e-12)
	sq := mlx.s_max(disc, 0.0).sqrt()
	t1 := mlx.s_div(b.negative().subtract(sq), 2.0)
	t2 := mlx.s_div(b.negative().add(sq), 2.0)
	p1 := o.add(t1.expand_dims(1).multiply(d))
	p2 := o.add(t2.expand_dims(1).multiply(d))
	n1 := mlx.s_div(p1.subtract(c), p.r)
	n2 := mlx.s_div(p2.subtract(c), p.r)
	ts := mlx.stack([
		mlx.where(valid, t1, mlx.full_like(t1, mlx.f32_scalar(f32(math.inf(1))), .float32)),
		mlx.where(valid, t2, mlx.full_like(t2, mlx.f32_scalar(f32(math.inf(1))), .float32)),
	], -1)
	return ts, mlx.stack([n1, n2], 1), mlx.stack([valid, valid], -1)
}

fn sphere_contains(p SphereParams, pos mlx.Array) mlx.Array {
	c := mlx.arr3v(p.c)
	q := pos.subtract(c)
	return mlx.s_lt(q.multiply(q).sum_axis(-1, false), p.r * p.r)
}

fn plane_crossings(p PlaneParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	n := mlx.arr3v(p.n)
	denom := n.multiply(d).sum_axis(-1, false)
	safe := mlx.where(mlx.s_gt(denom.abs(), 1e-9), denom, mlx.full_like(denom,
		mlx.f32_scalar(f32(1e-9)), .float32))
	t := mlx.s_rsub(n.multiply(o).sum_axis(-1, false), p.d).divide(safe)
	valid := mlx.s_gt(denom.abs(), 1e-9).expand_dims(1)
	ts := mlx.where(valid, t.expand_dims(1), mlx.full([o.shape()[0], 1],
		mlx.f32_scalar(f32(math.inf(1))), .float32))
	ns := n.expand_dims(0).expand_dims(0).broadcast_to([o.shape()[0], 1, 3])
	return ts, ns, valid
}

fn plane_contains(p PlaneParams, pos mlx.Array) mlx.Array {
	n := mlx.arr3v(p.n)
	return mlx.s_lt(pos.multiply(n).sum_axis(-1, false), p.d)
}

fn cylinder_crossings(p CylinderParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	q := mlx.arr3v(p.q)
	u := mlx.arr3v(p.u)
	oc := o.subtract(q)
	d_par := d.multiply(u).sum_axis(-1, true)
	o_par := oc.multiply(u).sum_axis(-1, true)
	d_p := d.subtract(d_par.multiply(u))
	o_p := oc.subtract(o_par.multiply(u))
	a := d_p.multiply(d_p).sum_axis(-1, false)
	b := mlx.s_mul(o_p.multiply(d_p).sum_axis(-1, false), 2.0)
	cq := mlx.s_sub(o_p.multiply(o_p).sum_axis(-1, false), p.r * p.r)
	disc := b.multiply(b).subtract(mlx.s_mul(a.multiply(cq), 4.0))
	side_valid := mlx.s_gt(a, 1e-12).logical_and(mlx.s_gt(disc, 1e-12))
	sq := mlx.s_max(disc, 0.0).sqrt()
	st0 := b.negative().subtract(sq).divide(mlx.s_mul(a, 2.0))
	st1 := b.negative().add(sq).divide(mlx.s_mul(a, 2.0))
	p0 := o_p.add(st0.expand_dims(1).multiply(d_p))
	p1 := o_p.add(st1.expand_dims(1).multiply(d_p))
	n_s0 := mlx.s_div(p0, p.r)
	n_s1 := mlx.s_div(p1, p.r)
	inf := mlx.full_like(st0, mlx.f32_scalar(f32(math.inf(1))), .float32)
	if p.h < 0.0 {
		ts := mlx.stack([mlx.where(side_valid, st0, inf), mlx.where(side_valid, st1, inf)], -1)
		return ts, mlx.stack([n_s0, n_s1], 1), mlx.stack([side_valid, side_valid], -1)
	}
	h := p.h
	denom := col(d_par, 0)
	safe := mlx.where(mlx.s_gt(denom.abs(), 1e-9), denom, mlx.full_like(denom,
		mlx.f32_scalar(f32(1e-9)), .float32))
	t_plus := mlx.s_rsub(col(o_par, 0), h).divide(safe)
	t_minus := mlx.s_rsub(col(o_par, 0), -h).divide(safe)
	at0 := t_plus.minimum(t_minus)
	at1 := t_plus.maximum(t_minus)
	enter := st0.maximum(at0)
	exit_ := st1.minimum(at1)
	valid := side_valid.logical_and(enter.less(exit_))
	enter_cap := at0.greater(st0)
	exit_cap := at1.less(st1)
	n_cap0 := mlx.where(t_plus.less(t_minus).expand_dims(1), u.expand_dims(0),
		u.negative().expand_dims(0))
	n_cap1 := mlx.where(t_plus.greater(t_minus).expand_dims(1), u.expand_dims(0),
		u.negative().expand_dims(0))
	n0 := mlx.where(enter_cap.expand_dims(1), n_cap0, n_s0)
	n1 := mlx.where(exit_cap.expand_dims(1), n_cap1, n_s1)
	ts := mlx.stack([mlx.where(valid, enter, inf), mlx.where(valid, exit_, inf)], -1)
	return ts, mlx.stack([n0, n1], 1), mlx.stack([valid, valid], -1)
}

fn cylinder_contains(p CylinderParams, pos mlx.Array) mlx.Array {
	q := mlx.arr3v(p.q)
	u := mlx.arr3v(p.u)
	rel := pos.subtract(q)
	s := rel.multiply(u).sum_axis(-1, true)
	radial := rel.subtract(s.multiply(u))
	mut inside := mlx.s_lt(radial.multiply(radial).sum_axis(-1, false), p.r * p.r)
	if p.h >= 0.0 {
		inside = inside.logical_and(mlx.s_le(last_col(s, 0).abs(), p.h))
	}
	return inside
}

fn box_crossings(p BoxParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	c := mlx.arr3v(p.c)
	oc := o.subtract(c)
	ax0 := mlx.arr3v(p.axes[0])
	ax1 := mlx.arr3v(p.axes[1])
	ax2 := mlx.arr3v(p.axes[2])
	op := mlx.stack([oc.multiply(ax0).sum_axis(-1, false), oc.multiply(ax1).sum_axis(-1, false),
		oc.multiply(ax2).sum_axis(-1, false)], -1)
	dp := mlx.stack([d.multiply(ax0).sum_axis(-1, false), d.multiply(ax1).sum_axis(-1, false),
		d.multiply(ax2).sum_axis(-1, false)], -1)
	inv := mlx.s_rdiv(dp, 1.0)
	half_a := mlx.arr3v(p.half)
	t0 := inv.negative().multiply(op.add(half_a))
	t1 := inv.negative().multiply(op.subtract(half_a))
	tmin := t0.minimum(t1)
	tmax := t0.maximum(t1)
	t_entry := tmin.max_axis(-1, false)
	t_exit := tmax.min_axis(-1, false)
	valid := t_entry.less(t_exit)
	i_e := tmin.argmax_axis(-1, false)
	i_x := tmax.argmin_axis(-1, false)
	eye := mlx.eye(3, 3, 0, .float32)
	dp_e := dp.take_along_axis(i_e.expand_dims(1), -1)
	dp_x := dp.take_along_axis(i_x.expand_dims(1), -1)
	mut n_e := eye.take_axis(i_e, 0).multiply(dp_e.sign().negative())
	mut n_x := eye.take_axis(i_x, 0).multiply(dp_x.sign())
	rot := mlx.stack([ax0, ax1, ax2], 0)
	n_e = vecmat(n_e, mat3_from_mlx(rot))
	n_x = vecmat(n_x, mat3_from_mlx(rot))
	inf := mlx.full_like(t_entry, mlx.f32_scalar(f32(math.inf(1))), .float32)
	ts := mlx.stack([mlx.where(valid, t_entry, inf), mlx.where(valid, t_exit, inf)], -1)
	return ts, mlx.stack([n_e, n_x], 1), mlx.stack([valid, valid], -1)
}

// mat3_from_mlx reads a (3,3) array back into a Mat3 (CPU, small).
fn mat3_from_mlx(m mlx.Array) Mat3 {
	d := m.data_f32()
	return mat3_new([f64(d[0]), f64(d[1]), f64(d[2])]!, [f64(d[3]), f64(d[4]), f64(d[5])]!, [
		f64(d[6]),
		f64(d[7]),
		f64(d[8]),
	]!)
}

fn box_contains(p BoxParams, pos mlx.Array) mlx.Array {
	c := mlx.arr3v(p.c)
	q := pos.subtract(c)
	ax0 := mlx.arr3v(p.axes[0])
	ax1 := mlx.arr3v(p.axes[1])
	ax2 := mlx.arr3v(p.axes[2])
	mut inside := mlx.ones(q.shape()[..q.shape().len - 1], .bool_)
	q0 := q.multiply(ax0).sum_axis(-1, false)
	q1 := q.multiply(ax1).sum_axis(-1, false)
	q2 := q.multiply(ax2).sum_axis(-1, false)
	inside = inside.logical_and(mlx.s_le(q0.abs(), p.half[0])).logical_and(mlx.s_le(q1.abs(),
		p.half[1])).logical_and(mlx.s_le(q2.abs(), p.half[2]))
	return inside
}

// --- affine-solid crossings / contains --------------------------------------

pub fn cone_crossings(p ConeParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	enter, exit_, valid, n0, n1 := cone_local_interval(p.r, p.h, o_l, d_u)
	inf := mlx.full_like(enter, mlx.f32_scalar(f32(math.inf(1))), .float32)
	ts := mlx.stack([mlx.where(valid, enter, inf), mlx.where(valid, exit_, inf)], -1)
	mut ns := mlx.stack([n0, n1], 1)
	ns = affine_normal(ns, p.a_inv3)
	return ts.divide(col(lam, 0).expand_dims(1)), ns, mlx.stack([valid, valid], -1)
}

fn cone_contains(p ConeParams, pos mlx.Array) mlx.Array {
	k2 := 1.0 + (p.r / p.h) * (p.r / p.h)
	s := mlx.s_sub(last_col(pos, 2), p.h / 2.0)
	f :=
		last_col(pos, 0).square().add(last_col(pos, 1).square()).add(s.multiply(s)).subtract(mlx.s_mul(s.multiply(s), k2))
	return mlx.s_le(f, 0.0).logical_and(mlx.s_ge(s, -p.h)).logical_and(mlx.s_le(s, 0.0))
}

fn ellipsoid_crossings(p EllipsoidParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	// local unit sphere crossings, then affine transform
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	b := mlx.s_mul(o_l.multiply(d_u).sum_axis(-1, false), 2.0)
	cq := mlx.s_sub(o_l.multiply(o_l).sum_axis(-1, false), 1.0)
	disc := b.multiply(b).subtract(mlx.s_mul(cq, 4.0))
	valid := mlx.s_gt(disc, 1e-12)
	sq := mlx.s_max(disc, 0.0).sqrt()
	t1 := mlx.s_div(b.negative().subtract(sq), 2.0)
	t2 := mlx.s_div(b.negative().add(sq), 2.0)
	p1 := o_l.add(t1.expand_dims(1).multiply(d_u))
	p2 := o_l.add(t2.expand_dims(1).multiply(d_u))
	inf := mlx.full_like(t1, mlx.f32_scalar(f32(math.inf(1))), .float32)
	ts :=
		mlx.stack([mlx.where(valid, t1, inf), mlx.where(valid, t2, inf)], -1).divide(col(lam, 0).expand_dims(1))
	mut ns := mlx.stack([p1, p2], 1)
	ns = affine_normal(ns, p.a_inv3)
	return ts, ns, mlx.stack([valid, valid], -1)
}

fn ellipsoid_contains(p EllipsoidParams, pos mlx.Array) mlx.Array {
	p_l := affine_point_to_local(p.a_inv3, p.t_inv, pos)
	return mlx.s_lt(p_l.multiply(p_l).sum_axis(-1, false), 1.0)
}

pub fn torus_crossings(p TorusParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, mut ns, valid := torus_local_crossings(p.major, p.minor, o_l, d_u)
	ns = affine_normal(ns, p.a_inv3)
	return ts.divide(col(lam, 0).expand_dims(1)), ns, valid
}

fn torus_contains(p TorusParams, pos mlx.Array) mlx.Array {
	p_l := affine_point_to_local(p.a_inv3, p.t_inv, pos)
	return torus_local_contains(p.major, p.minor, p_l)
}

pub fn cyclide_crossings(p CyclideParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	ts, mut ns, valid := cyclide_local_crossings(p.a, p.b, p.d, p.c, p.shift, o_l, d_u)
	ns = affine_normal(ns, p.a_inv3)
	return ts.divide(col(lam, 0).expand_dims(1)), ns, valid
}

fn cyclide_contains(p CyclideParams, pos mlx.Array) mlx.Array {
	p_l := affine_point_to_local(p.a_inv3, p.t_inv, pos)
	return cyclide_local_contains(p.a, p.b, p.d, p.c, p.shift, p_l)
}

fn trimesh_crossings(p TrimeshParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	tall, nall, _ := trimesh_mt_all(p.v0, p.e1, p.e2, p.nrm, o_l, d_u)
	order := tall.argsort_axis(1).take_axis(mlx.array_i32([i32(0), 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
		11, 12, 13, 14, 15], [16]), 1)
	ts := tall.take_along_axis(order, 1).divide(col(lam, 0).expand_dims(1))
	mut ns := nall.take_along_axis(order.expand_dims(2), 1)
	ns = affine_normal(ns, p.a_inv3)
	return ts, ns, ts.isfinite()
}

fn trimesh_contains(p TrimeshParams, pos mlx.Array) mlx.Array {
	shape := pos.shape()[..pos.shape().len - 1]
	pts := pos.reshape([-1, 3])
	d := mlx.arr3(1.0, 0.0, 0.0).broadcast_to(pts.shape())
	tall, _, _ := trimesh_mt_all(p.v0, p.e1, p.e2, p.nrm, pts, d)
	count := tall.isfinite().astype(.int32).sum_axis(-1, false)
	return count.remainder(mlx.int_scalar(2)).equal(mlx.int_scalar(1)).reshape(shape)
}

// --- dispatch ---------------------------------------------------------------

pub fn geom_crossings(p GeometryParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	return match p {
		SphereParams { sphere_crossings(p, o, d) }
		PlaneParams { plane_crossings(p, o, d) }
		CylinderParams { cylinder_crossings(p, o, d) }
		BoxParams { box_crossings(p, o, d) }
		ConeParams { cone_crossings(p, o, d) }
		EllipsoidParams { ellipsoid_crossings(p, o, d) }
		TorusParams { torus_crossings(p, o, d) }
		CyclideParams { cyclide_crossings(p, o, d) }
		TrimeshParams { trimesh_crossings(p, o, d) }
		CircleParams { panic('circle is not a solid (no crossings)') }
		CsgParams { csg_crossings(p, o, d) }
		AffineParams { affine_crossings(p, o, d) }
		DisplacedParams { panic('displaced surface is not a solid (no crossings)') }
	}
}

pub fn geom_contains(p GeometryParams, pos mlx.Array) mlx.Array {
	return match p {
		SphereParams { sphere_contains(p, pos) }
		PlaneParams { plane_contains(p, pos) }
		CylinderParams { cylinder_contains(p, pos) }
		BoxParams { box_contains(p, pos) }
		ConeParams { cone_contains(p, pos) }
		EllipsoidParams { ellipsoid_contains(p, pos) }
		TorusParams { torus_contains(p, pos) }
		CyclideParams { cyclide_contains(p, pos) }
		TrimeshParams { trimesh_contains(p, pos) }
		CircleParams { panic('circle is not a solid (no contains)') }
		CsgParams { csg_contains(p, pos) }
		AffineParams { affine_contains(p, pos) }
		DisplacedParams { panic('displaced surface is not a solid (no contains)') }
	}
}
