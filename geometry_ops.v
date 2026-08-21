module cga

// Per-pixel ray-intersection kernels (MLX batch, float32).  Each geometry
// provides intersect / intersect_shadow / uv_at / bounds_camera over the
// camera-space parameters computed in geometry.v.
import mlx
import math

// col extracts column i of a (...,N) array as a (...,) array (last-axis index).
@[inline]
fn col(a mlx.Array, i int) mlx.Array {
	return a.take_axis(mlx.int_scalar(i), -1)
}

fn inf_array_like(a mlx.Array) mlx.Array {
	return mlx.full_like(a, mlx.f32_scalar(f32(math.inf(1))), .float32)
}

// --- sphere -----------------------------------------------------------------

pub fn sphere_intersect(p SphereParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	c := mlx.arr3v(p.c)
	oc := o.subtract(c)
	b := mlx.s_mul(oc.multiply(d).sum_axis(-1, false), 2.0)
	cq := mlx.s_sub(oc.multiply(oc).sum_axis(-1, false), p.r * p.r)
	disc := b.multiply(b).subtract(mlx.s_mul(cq, 4.0))
	valid := mlx.s_gt(disc, 1e-12)
	sq := mlx.s_max(disc, 0.0).sqrt()
	t1 := mlx.s_div(b.negative().subtract(sq), 2.0)
	t2 := mlx.s_div(b.negative().add(sq), 2.0)
	t := mlx.where(valid.logical_and(mlx.s_gt(t1, 1e-6)), t1, t2)
	mask := valid.logical_and(mlx.s_gt(t, 1e-6))
	hit := o.add(t.expand_dims(1).multiply(d))
	mut n := mlx.s_div(hit.subtract(c), p.r)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	inside := mask.logical_and(mlx.s_le(t1, 1e-6))
	n = mlx.where(inside.expand_dims(1), n.negative(), n)
	return t, n, mask
}

pub fn sphere_shadow(p SphereParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	c := mlx.arr3v(p.c)
	oc := o.subtract(c)
	b := mlx.s_mul(oc.multiply(d).sum_axis(-1, false), 2.0)
	cq := mlx.s_sub(oc.multiply(oc).sum_axis(-1, false), p.r * p.r)
	disc := b.multiply(b).subtract(mlx.s_mul(cq, 4.0))
	valid := mlx.s_gt(disc, 1e-12)
	sq := mlx.s_max(disc, 0.0).sqrt()
	t1 := mlx.s_div(b.negative().subtract(sq), 2.0)
	t2 := mlx.s_div(b.negative().add(sq), 2.0)
	t := mlx.where(valid.logical_and(mlx.s_gt(t1, 1e-6)), t1, t2)
	return t, valid.logical_and(mlx.s_gt(t, 1e-6))
}

pub fn sphere_uv(p SphereParams, pos mlx.Array, n mlx.Array) mlx.Array {
	c := mlx.arr3v(p.c)
	q := pos.subtract(c)
	x := mlx.s_div(q.multiply(mlx.arr3v(p.axes[0])).sum_axis(-1, false), p.r)
	y := mlx.s_div(q.multiply(mlx.arr3v(p.axes[1])).sum_axis(-1, false), p.r)
	z := mlx.s_clip(mlx.s_div(q.multiply(mlx.arr3v(p.axes[2])).sum_axis(-1, false), p.r), -1.0, 1.0)
	u := mlx.s_add(mlx.s_div(y.arctan2(x), 2.0 * math.pi), 0.5)
	v := mlx.s_div(z.arccos(), math.pi)
	return mlx.stack([u, v], -1)
}

// --- plane ------------------------------------------------------------------

pub fn plane_intersect(p PlaneParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	n := mlx.arr3v(p.n)
	denom := n.multiply(d).sum_axis(-1, false)
	t := mlx.s_rsub(n.multiply(o).sum_axis(-1, false), p.d).divide(denom)
	mask := mlx.s_gt(denom.abs(), 1e-9).logical_and(mlx.s_gt(t, 1e-6))
	n_rep := n.broadcast_to(o.shape())
	return t, mlx.where(mask.expand_dims(1), n_rep, mlx.zeros_like(n_rep)), mask
}

pub fn plane_shadow(p PlaneParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	n := mlx.arr3v(p.n)
	denom := n.multiply(d).sum_axis(-1, false)
	t := mlx.s_rsub(n.multiply(o).sum_axis(-1, false), p.d).divide(denom)
	mask := mlx.s_gt(denom.abs(), 1e-9).logical_and(mlx.s_gt(t, 1e-6))
	return t, mask
}

pub fn plane_uv(p PlaneParams, pos mlx.Array, n mlx.Array) mlx.Array {
	return mlx.stack([col(pos, 0), col(pos, 2)], -1)
}

// --- cylinder ---------------------------------------------------------------

fn cylinder_side(p CylinderParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array, mlx.Array) {
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
	valid := mlx.s_gt(a, 1e-12).logical_and(mlx.s_gt(disc, 1e-12))
	sq := mlx.s_max(disc, 0.0).sqrt()
	t1 := b.negative().subtract(sq).divide(mlx.s_mul(a, 2.0))
	t2 := b.negative().add(sq).divide(mlx.s_mul(a, 2.0))
	t := mlx.where(valid.logical_and(mlx.s_gt(t1, 1e-6)), t1, t2)
	mask := valid.logical_and(mlx.s_gt(t, 1e-6))
	hit := o_p.add(t.expand_dims(1).multiply(d_p))
	mut n := mlx.s_div(hit, p.r)
	inside := mask.logical_and(mlx.s_le(t1, 1e-6))
	n = mlx.where(inside.expand_dims(1), n.negative(), n)
	return t, n, mask, o_par
}

pub fn cylinder_intersect(p CylinderParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	t, mut n, mask, o_par := cylinder_side(p, o, d)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	if p.h < 0.0 {
		return t, n, mask
	}
	h := p.h
	u := mlx.arr3v(p.u)
	q := mlx.arr3v(p.q)
	d_par := d.multiply(u).sum_axis(-1, true)
	s := o_par.add(t.expand_dims(1).multiply(d_par))
	side_ok := mask.logical_and(mlx.s_le(col(s.abs(), 0), h))
	denom := col(d_par, 0)
	cap_t := mlx.stack([mlx.s_rsub(col(o_par, 0), h).divide(denom),
		mlx.s_rsub(col(o_par, 0), -h).divide(denom)], -1)
	mut cap_ok := mlx.s_gt(denom.abs(), 1e-9).expand_dims(1).broadcast_to([o.shape()[0], 2])
	cap_ok = cap_ok.logical_and(mlx.s_gt(cap_t, 1e-6))
	p_cap := o.expand_dims(1).add(cap_t.expand_dims(2).multiply(d.expand_dims(1)))
	rel := p_cap.subtract(q.expand_dims(0).expand_dims(0))
	lat :=
		rel.subtract(rel.multiply(u.expand_dims(0).expand_dims(0)).sum_axis(-1, true).multiply(u.expand_dims(0).expand_dims(0)))
	cap_ok = cap_ok.logical_and(mlx.s_le(lat.multiply(lat).sum_axis(-1, false), p.r * p.r))
	mut n_cap := denom.sign().negative().expand_dims(1).multiply(u.expand_dims(0))
	n_cap = mlx.stack([n_cap, n_cap], 1)
	t_all := mlx.stack([t, col(cap_t, 0), col(cap_t, 1)], -1)
	ok_all := mlx.stack([side_ok, col(cap_ok, 0), col(cap_ok, 1)], -1)
	t_eff := mlx.where(ok_all, t_all, inf_array_like(t_all))
	t_min := t_eff.min_axis(-1, false)
	idx := t_eff.argmin_axis(-1, false)
	n_all := mlx.stack([n, n_cap.take_axis(mlx.int_scalar(0), 1),
		n_cap.take_axis(mlx.int_scalar(1), 1)], 1)
	n_fin := n_all.take_along_axis(idx.expand_dims(1).expand_dims(2).broadcast_to([
		n.shape()[0],
		1,
		3,
	]), 1).take_axis(mlx.int_scalar(0), 1)
	fin := t_min.isfinite().logical_and(mlx.s_gt(t_min, 1e-6))
	return mlx.where(fin, t_min, t), mlx.where(fin.expand_dims(1), n_fin, mlx.zeros_like(n_fin)), fin
}

pub fn cylinder_shadow(p CylinderParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	t, _, mask, o_par := cylinder_side(p, o, d)
	if p.h < 0.0 {
		return t, mask
	}
	h := p.h
	u := mlx.arr3v(p.u)
	q := mlx.arr3v(p.q)
	d_par := d.multiply(u).sum_axis(-1, true)
	s := o_par.add(t.expand_dims(1).multiply(d_par))
	side_ok := mask.logical_and(mlx.s_le(col(s.abs(), 0), h))
	denom := col(d_par, 0)
	cap_t := mlx.stack([mlx.s_rsub(col(o_par, 0), h).divide(denom),
		mlx.s_rsub(col(o_par, 0), -h).divide(denom)], -1)
	mut cap_ok := mlx.s_gt(denom.abs(), 1e-9).expand_dims(1).broadcast_to([o.shape()[0], 2])
	cap_ok = cap_ok.logical_and(mlx.s_gt(cap_t, 1e-6))
	p_cap := o.expand_dims(1).add(cap_t.expand_dims(2).multiply(d.expand_dims(1)))
	rel := p_cap.subtract(q.expand_dims(0).expand_dims(0))
	lat :=
		rel.subtract(rel.multiply(u.expand_dims(0).expand_dims(0)).sum_axis(-1, true).multiply(u.expand_dims(0).expand_dims(0)))
	cap_ok = cap_ok.logical_and(mlx.s_le(lat.multiply(lat).sum_axis(-1, false), p.r * p.r))
	t_all := mlx.stack([t, col(cap_t, 0), col(cap_t, 1)], -1)
	ok_all := mlx.stack([side_ok, col(cap_ok, 0), col(cap_ok, 1)], -1)
	t_eff := mlx.where(ok_all, t_all, inf_array_like(t_all))
	t_min := t_eff.min_axis(-1, false)
	fin := t_min.isfinite().logical_and(mlx.s_gt(t_min, 1e-6))
	return mlx.where(fin, t_min, t), fin
}

pub fn cylinder_uv(p CylinderParams, pos mlx.Array, n mlx.Array) mlx.Array {
	q := mlx.arr3v(p.q)
	axis := mlx.arr3v(p.u)
	rel := pos.subtract(q)
	axial := rel.multiply(axis).sum_axis(-1, true).multiply(axis)
	radial := rel.subtract(axial)
	seed := mlx.arr3(1.0, 0.0, 0.0)
	alt := mlx.arr3(0.0, 1.0, 0.0)
	mut b1 := mlx.where(mlx.s_lt(axis.take_axis(mlx.int_scalar(0), 0).abs(), 0.9), seed, alt)
	b1 = b1.subtract(b1.multiply(axis).sum().multiply(axis))
	b1 = b1.divide(b1.multiply(b1).sum().sqrt())
	b2 := mlx.stack([
		axis.take_axis(mlx.int_scalar(1), 0).multiply(b1.take_axis(mlx.int_scalar(2), 0)).subtract(axis.take_axis(mlx.int_scalar(2), 0).multiply(b1.take_axis(mlx.int_scalar(1), 0))),
		axis.take_axis(mlx.int_scalar(2), 0).multiply(b1.take_axis(mlx.int_scalar(0), 0)).subtract(axis.take_axis(mlx.int_scalar(0), 0).multiply(b1.take_axis(mlx.int_scalar(2), 0))),
		axis.take_axis(mlx.int_scalar(0), 0).multiply(b1.take_axis(mlx.int_scalar(1), 0)).subtract(axis.take_axis(mlx.int_scalar(1), 0).multiply(b1.take_axis(mlx.int_scalar(0), 0))),
	], 0)
	u := mlx.s_add(mlx.s_div(radial.multiply(b2).sum_axis(-1, false).arctan2(radial.multiply(b1).sum_axis(-1,
		false)), 2.0 * math.pi), 0.5)
	v := mlx.s_div(rel.multiply(axis).sum_axis(-1, false), 2.0 * p.r)
	return mlx.stack([u, v], -1)
}

// --- box --------------------------------------------------------------------

pub fn box_intersect(p BoxParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
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
	valid := t_entry.less(t_exit).logical_and(mlx.s_gt(t_exit, 1e-6))
	i_entry := tmin.argmax_axis(-1, false)
	i_exit := tmax.argmin_axis(-1, false)
	inside_hit := valid.logical_and(mlx.s_le(t_entry, 1e-6))
	t := mlx.where(valid.logical_and(inside_hit.logical_not()), t_entry, t_exit)
	idx := mlx.where(inside_hit, i_exit, i_entry)
	eye := mlx.eye(3, 3, 0, .float32)
	mut n :=
		eye.take_axis(idx, 0).multiply(dp.take_along_axis(idx.expand_dims(1), -1).squeeze_axis(-1).sign().negative().expand_dims(1))
	n = mlx.where(valid.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, valid
}

pub fn box_shadow(p BoxParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
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
	valid := t_entry.less(t_exit).logical_and(mlx.s_gt(t_exit, 1e-6))
	inside_hit := valid.logical_and(mlx.s_le(t_entry, 1e-6))
	t := mlx.where(valid.logical_and(inside_hit.logical_not()), t_entry, t_exit)
	return t, valid
}

pub fn box_uv(p BoxParams, pos mlx.Array, n mlx.Array) mlx.Array {
	c := mlx.arr3v(p.c)
	q := pos.subtract(c)
	ax0 := mlx.arr3v(p.axes[0])
	ax1 := mlx.arr3v(p.axes[1])
	ax2 := mlx.arr3v(p.axes[2])
	local := mlx.stack([q.multiply(ax0).sum_axis(-1, false), q.multiply(ax1).sum_axis(-1, false),
		q.multiply(ax2).sum_axis(-1, false)], -1)
	half_a := mlx.arr3v(p.half)
	face := local.divide(half_a).abs().argmax_axis(-1, false)
	l0 := col(local, 0)
	l1 := col(local, 1)
	l2 := col(local, 2)
	x := mlx.where(mlx.s_eq(face, 0.0), l2, l0)
	y := mlx.where(mlx.s_eq(face, 2.0), l1, l2)
	sx := mlx.where(mlx.s_eq(face, 0.0), half_a.take_axis(mlx.int_scalar(2), 0),
		half_a.take_axis(mlx.int_scalar(0), 0))
	sy := mlx.where(mlx.s_eq(face, 2.0), half_a.take_axis(mlx.int_scalar(1), 0),
		half_a.take_axis(mlx.int_scalar(2), 0))
	return mlx.stack([mlx.s_add(x.divide(mlx.s_mul(sx, 2.0)), 0.5),
		mlx.s_add(y.divide(mlx.s_mul(sy, 2.0)), 0.5)], -1)
}

// --- circle -----------------------------------------------------------------

pub fn circle_intersect(p CircleParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	c := mlx.arr3v(p.c)
	n := mlx.arr3v(p.n)
	denom := n.multiply(d).sum_axis(-1, false)
	t := n.multiply(c.subtract(o)).sum_axis(-1, false).divide(denom)
	front := mlx.s_lt(denom, 0.0)
	hit := o.add(t.expand_dims(1).multiply(d))
	diff := hit.subtract(c)
	in_disc := mlx.s_le(diff.multiply(diff).sum_axis(-1, false), p.r * p.r)
	mask := mlx.s_gt(denom.abs(), 1e-9).logical_and(mlx.s_gt(t, 1e-6)).logical_and(in_disc)
	n2 := mlx.where(front.expand_dims(1), n, n.negative())
	return t, mlx.where(mask.expand_dims(1), n2, mlx.zeros_like(n2)), mask
}

pub fn circle_shadow(p CircleParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	c := mlx.arr3v(p.c)
	n := mlx.arr3v(p.n)
	denom := n.multiply(d).sum_axis(-1, false)
	t := n.multiply(c.subtract(o)).sum_axis(-1, false).divide(denom)
	hit := o.add(t.expand_dims(1).multiply(d))
	diff := hit.subtract(c)
	in_disc := mlx.s_le(diff.multiply(diff).sum_axis(-1, false), p.r * p.r)
	mask := mlx.s_gt(denom.abs(), 1e-9).logical_and(mlx.s_gt(t, 1e-6)).logical_and(in_disc)
	return t, mask
}

pub fn circle_uv(p CircleParams, pos mlx.Array, n mlx.Array) mlx.Array {
	c := mlx.arr3v(p.c)
	q := mlx.s_div(pos.subtract(c), 2.0 * p.r)
	return mlx.stack([mlx.s_add(col(q, 0), 0.5), mlx.s_add(col(q, 1), 0.5)], -1)
}

// --- dispatch ---------------------------------------------------------------

pub fn geom_intersect(p GeometryParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	return match p {
		SphereParams {
			sphere_intersect(p, o, d)
		}
		PlaneParams {
			plane_intersect(p, o, d)
		}
		CylinderParams {
			cylinder_intersect(p, o, d)
		}
		BoxParams {
			box_intersect(p, o, d)
		}
		CircleParams {
			circle_intersect(p, o, d)
		}
		ConeParams {
			cone_intersect(p, o, d)
		}
		TorusParams {
			torus_intersect(p, o, d)
		}
		EllipsoidParams {
			ellipsoid_intersect(p, o, d)
		}
		CyclideParams {
			cyclide_intersect(p, o, d)
		}
		TrimeshParams {
			trimesh_intersect(p, o, d)
		}
		CsgParams {
			csg_intersect(p, o, d)
		}
		AffineParams {
			affine_intersect(p, o, d)
		}
		SplatsParams {
			// splat clouds have no surface: never hit
			n_rays := o.shape()[0]
			t := mlx.full([n_rays], mlx.f32_scalar(f32(math.inf(1))), .float32)
			n := mlx.zeros_like(o)
			mask := mlx.zeros([n_rays], .bool_)
			t, n, mask
		}
	}
}

pub fn geom_shadow(p GeometryParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	return match p {
		SphereParams {
			sphere_shadow(p, o, d)
		}
		PlaneParams {
			plane_shadow(p, o, d)
		}
		CylinderParams {
			cylinder_shadow(p, o, d)
		}
		BoxParams {
			box_shadow(p, o, d)
		}
		CircleParams {
			circle_shadow(p, o, d)
		}
		ConeParams {
			cone_shadow(p, o, d)
		}
		TorusParams {
			torus_shadow(p, o, d)
		}
		EllipsoidParams {
			ellipsoid_shadow(p, o, d)
		}
		CyclideParams {
			cyclide_shadow(p, o, d)
		}
		TrimeshParams {
			trimesh_shadow(p, o, d)
		}
		CsgParams {
			csg_shadow(p, o, d)
		}
		AffineParams {
			affine_shadow(p, o, d)
		}
		SplatsParams {
			// splats cast no ray-traced shadows
			n_rays := o.shape()[0]
			t := mlx.full([n_rays], mlx.f32_scalar(f32(math.inf(1))), .float32)
			mask := mlx.zeros([n_rays], .bool_)
			t, mask
		}
	}
}

pub fn geom_uv(p GeometryParams, pos mlx.Array, n mlx.Array) mlx.Array {
	return match p {
		SphereParams { sphere_uv(p, pos, n) }
		PlaneParams { plane_uv(p, pos, n) }
		CylinderParams { cylinder_uv(p, pos, n) }
		BoxParams { box_uv(p, pos, n) }
		CircleParams { circle_uv(p, pos, n) }
		ConeParams { cone_uv(p, pos, n) }
		TorusParams { torus_uv(p, pos, n) }
		EllipsoidParams { ellipsoid_uv(p, pos, n) }
		CyclideParams { cyclide_uv(p, pos, n) }
		TrimeshParams { trimesh_uv(p, pos, n) }
		CsgParams { csg_uv(p, pos, n) }
		AffineParams { affine_uv(p, pos, n) }
		SplatsParams { mlx.zeros([pos.shape()[0], 2], .float32) }
	}
}

// geom_bounds returns the camera-space AABB (lo, hi), or none if unbounded.
pub fn geom_bounds(p GeometryParams) ?[2][3]f64 {
	match p {
		SphereParams {
			return [lo(p.c, p.r), hi(p.c, p.r)]!
		}
		PlaneParams {
			return none
		}
		CylinderParams {
			if p.h < 0.0 {
				return none
			}
			return capsule_bounds(p.q, p.u, p.r, p.h)
		}
		BoxParams {
			return box_bounds(p.c, p.axes, p.half)
		}
		CircleParams {
			return [lo(p.c, p.r), hi(p.c, p.r)]!
		}
		ConeParams {
			return affine_bounds([-p.r, -p.r, -p.h / 2.0]!, [p.r, p.r, p.h / 2.0]!, p.a_fwd)
		}
		TorusParams {
			e := p.major + p.minor
			return affine_bounds([-e, -e, -p.minor]!, [e, e, p.minor]!, p.a_fwd)
		}
		EllipsoidParams {
			return affine_bounds([-1.0, -1.0, -1.0]!, [1.0, 1.0, 1.0]!, p.a_fwd)
		}
		CyclideParams {
			r := p.d + p.c
			return affine_bounds([p.shift[0] - p.a - r, p.shift[1] - p.b - r, p.shift[2] - r]!, [
				p.shift[0] + p.a + r,
				p.shift[1] + p.b + r,
				p.shift[2] + r,
			]!, p.a_fwd)
		}
		TrimeshParams {
			return affine_bounds(p.lo, p.hi, p.a_fwd)
		}
		CsgParams {
			return csg_bounds(p)
		}
		AffineParams {
			return affine_bounds_from_inner(p)
		}
		SplatsParams {
			// never culled in Renderer.nearest (and never hit anyway)
			return none
		}
	}
}

fn affine_bounds_from_inner(p AffineParams) ?[2][3]f64 {
	b := geom_bounds(p.inner) or { return none }
	return affine_bounds(b[0], b[1], p.a_fwd)
}

fn lo(c [3]f64, r f64) [3]f64 {
	return [c[0] - r, c[1] - r, c[2] - r]!
}

fn hi(c [3]f64, r f64) [3]f64 {
	return [c[0] + r, c[1] + r, c[2] + r]!
}

fn capsule_bounds(q [3]f64, u [3]f64, r f64, h f64) [2][3]f64 {
	mut lo3 := [3]f64{}
	mut hi3 := [3]f64{}
	for i in 0 .. 3 {
		e := math.abs(u[i]) * h + r
		lo3[i] = q[i] - e
		hi3[i] = q[i] + e
	}
	return [lo3, hi3]!
}

fn box_bounds(c [3]f64, axes [3][3]f64, half [3]f64) [2][3]f64 {
	mut lo3 := [3]f64{}
	mut hi3 := [3]f64{}
	for i in 0 .. 3 {
		mut e := 0.0
		for j in 0 .. 3 {
			e += math.abs(axes[j][i]) * half[j]
		}
		lo3[i] = c[i] - e
		hi3[i] = c[i] + e
	}
	return [lo3, hi3]!
}
