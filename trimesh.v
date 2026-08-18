module cga

// Triangle-mesh primitive: brute-force Möller–Trumbore over all faces (no BVH),
// with the affine ray-inverse transform (see affine.v).
import mlx
import math

// cross3 is the (...,3) row-wise cross product (mlx has no cross).
fn cross3(a mlx.Array, b mlx.Array) mlx.Array {
	a0 := a.take_axis(mlx.int_scalar(0), -1)
	a1 := a.take_axis(mlx.int_scalar(1), -1)
	a2 := a.take_axis(mlx.int_scalar(2), -1)
	b0 := b.take_axis(mlx.int_scalar(0), -1)
	b1 := b.take_axis(mlx.int_scalar(1), -1)
	b2 := b.take_axis(mlx.int_scalar(2), -1)
	return mlx.stack([a1.multiply(b2).subtract(a2.multiply(b1)),
		a2.multiply(b0).subtract(a0.multiply(b2)), a0.multiply(b1).subtract(a1.multiply(b0))], -1)
}

fn trimesh_mt_all(v0 mlx.Array, e1 mlx.Array, e2 mlx.Array, nrm mlx.Array, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	n := o.shape()[0]
	f := v0.shape()[0]
	v0c := v0.expand_dims(0)
	e1c := e1.expand_dims(0)
	e2c := e2.expand_dims(0)
	dc := d.expand_dims(1)
	p := cross3(dc, e2c)
	det := e1c.multiply(p).sum_axis(-1, false)
	ok := mlx.s_gt(det.abs(), 1e-10)
	inv := mlx.s_rdiv(mlx.where(ok, det, mlx.ones_like(det)), 1.0)
	sv := o.expand_dims(1).subtract(v0c)
	u := sv.multiply(p).sum_axis(-1, false).multiply(inv)
	q := cross3(sv, e1c)
	v := dc.multiply(q).sum_axis(-1, false).multiply(inv)
	t := e2c.multiply(q).sum_axis(-1, false).multiply(inv)
	mut hit := ok.logical_and(mlx.s_ge(u, -1e-9)).logical_and(mlx.s_ge(v, -1e-9))
	hit = hit.logical_and(mlx.s_le(u.add(v), 1.0 + 1e-9)).logical_and(mlx.s_gt(t, 1e-6))
	tall := mlx.where(hit, t, mlx.full_like(t, mlx.f32_scalar(f32(math.inf(1))), .float32))
	valid := tall.isfinite()
	nall := nrm.expand_dims(0).broadcast_to([n, f, 3])
	return tall, nall, valid
}

pub fn trimesh_intersect(p TrimeshParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	tall, nall, _ := trimesh_mt_all(p.v0, p.e1, p.e2, p.nrm, o_l, d_u)
	t_l := tall.min_axis(-1, false)
	mask := t_l.isfinite()
	idx := tall.argmin_axis(-1, false)
	mut n_l := nall.take_along_axis(idx.expand_dims(1).expand_dims(2).broadcast_to([
		nall.shape()[0],
		1,
		3,
	]), 1).take_axis(mlx.int_scalar(0), 1)
	cos_i := d_u.multiply(n_l).sum_axis(-1, true).negative()
	n_l = mlx.where(mlx.s_lt(cos_i, 0.0), n_l.negative(), n_l)
	t := t_l.divide(col(lam, 0))
	mut n := affine_normal(n_l, p.a_inv3)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn trimesh_shadow(p TrimeshParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	o_l, d_u, lam := affine_to_local(p.a_inv3, p.t_inv, o, d)
	tall, _, _ := trimesh_mt_all(p.v0, p.e1, p.e2, p.nrm, o_l, d_u)
	t_l := tall.min_axis(-1, false)
	return t_l.divide(col(lam, 0)), t_l.isfinite()
}

pub fn trimesh_uv(p TrimeshParams, pos mlx.Array, n mlx.Array) mlx.Array {
	// v1: no mesh texture coordinates
	return mlx.zeros([pos.shape()[0], 2], .float32)
}
