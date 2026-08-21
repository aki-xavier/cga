module cga

// MLX kernels for DisplacedGeometry (see displaced.v for conventions).
//
// Intersection = smart bracketing + marching, no whole-ray marching: the BASE
// is intersected analytically first (sphere/cylinder: quadratic with the
// radius inflated by max|scale*r|, so silhouette bumps up to the max residual
// are still bracketed; plane: [t-M, t+M]; cyclide: the quartic crossings,
// min..max, inflated) and F(x) = d_base - scale*r(uv) is sampled at 128 fixed
// steps along the bracket looking for the first + -> - sign change, then
// refined by 8 bisection iterations.  128 steps resolve residual features
// down to ~bracket/128 wide; 8 bisections give ~bracket/128/256 accuracy.
// Rays with F < 0 at the bracket start count as hits at the bracket start
// (camera inside the displaced shell).
//
// Normals: central differences of F with eps = max(1e-4, 0.25 * world grid
// step) — a pragmatic first version (no analytic d(uv)/dx chain rule).
//
// Known limitation: a cyclide base is bracketed only by its analytic
// crossings, so a ray that misses the bare cyclide never marches — a bump
// whose silhouette reaches beyond the bare cyclide by less than the march
// step can be missed (sphere/cylinder brackets inflate the radius and do not
// have this gap).
import mlx
import math

// DisplacedParams is the camera-space state: base params + MLX-resident grid.
pub struct DisplacedParams {
pub:
	base    GeometryParams
	grid    mlx.Array // (res_v * res_u) float32, row-major
	res_u   int
	res_v   int
	scale   f64
	inflate f64    // max |scale * r| + a small margin
	eps     f64    // finite-difference step for normals
	b1      [3]f64 // cylinder azimuth frame / plane in-plane axes
	b2      [3]f64
}

// displaced_to_camera converts a DisplacedGeometry: the base converts as
// usual, the residual grid uploads to MLX (once per object per frame, like
// TrimeshGeometry's vertex arrays), and the aux frames for the cylinder /
// plane bases rotate with the motor.
fn displaced_to_camera(g DisplacedGeometry, m Multivector) DisplacedParams {
	mut b1 := [1.0, 0.0, 0.0]!
	mut b2 := [0.0, 1.0, 0.0]!
	if g.base is CylinderGeometry {
		// local axis is e3, so the local azimuth frame is e1/e2
		b1 = vec3_unit(dir3(m.apply(e1())))
		b2 = vec3_unit(dir3(m.apply(e2())))
	}
	if g.base is PlaneGeometry {
		_, _, t1, t2 := plane_frame(g.base.blade)
		b1 = vec3_unit(dir3(m.apply(mv_vector(t1[0], t1[1], t1[2], 0.0, 0.0))))
		b2 = vec3_unit(dir3(m.apply(mv_vector(t2[0], t2[1], t2[2], 0.0, 0.0))))
	}
	step := g.world_step()
	return DisplacedParams{
		base:    geom_to_camera(g.base, m)
		grid:    mlx.array_f32(g.residual, [g.res_v * g.res_u])
		res_u:   g.res_u
		res_v:   g.res_v
		scale:   g.scale
		inflate: g.max_abs_residual() + 0.05 * step + 1e-4
		eps:     math.max(1e-4, 0.25 * step)
		b1:      b1
		b2:      b2
	}
}

// --- per-base signed distance + uv (camera space) -----------------------------

fn displaced_base_dist_uv(p DisplacedParams, x mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	base := p.base
	match base {
		SphereParams {
			c := mlx.arr3v(base.c)
			q := x.subtract(c)
			dist := q.multiply(q).sum_axis(-1, false).sqrt().subtract(mlx.fs(base.r))
			// sphere_uv convention: u = atan2(y,x)/(2pi)+0.5, v = acos(z/r)/pi
			xq := mlx.s_div(q.multiply(mlx.arr3v(base.axes[0])).sum_axis(-1, false), base.r)
			yq := mlx.s_div(q.multiply(mlx.arr3v(base.axes[1])).sum_axis(-1, false), base.r)
			zq := mlx.s_clip(mlx.s_div(q.multiply(mlx.arr3v(base.axes[2])).sum_axis(-1, false),
				base.r), -1.0, 1.0)
			u := mlx.s_add(mlx.s_div(yq.arctan2(xq), 2.0 * math.pi), 0.5)
			v := mlx.s_div(zq.arccos(), math.pi)
			return dist, u, v
		}
		CylinderParams {
			q := mlx.arr3v(base.q)
			ax := mlx.arr3v(base.u)
			rel := x.subtract(q)
			axial := rel.multiply(ax).sum_axis(-1, true)
			perp := rel.subtract(axial.multiply(ax))
			dist := perp.multiply(perp).sum_axis(-1, false).sqrt().subtract(mlx.fs(base.r))
			u := mlx.s_add(mlx.s_div(perp.multiply(mlx.arr3v(p.b2)).sum_axis(-1, false).arctan2(perp.multiply(mlx.arr3v(p.b1)).sum_axis(-1,
				false)), 2.0 * math.pi), 0.5)
			v := mlx.s_div(col(axial, 0), 2.0 * base.r)
			return dist, u, v
		}
		PlaneParams {
			n := mlx.arr3v(base.n)
			dist := x.multiply(n).sum_axis(-1, false).subtract(mlx.fs(base.d))
			rel := x.subtract(n.multiply(mlx.fs(base.d)))
			// world-unit tiling: one node per unit -> normalise by res
			u := mlx.s_div(rel.multiply(mlx.arr3v(p.b1)).sum_axis(-1, false), f64(p.res_u))
			v := mlx.s_div(rel.multiply(mlx.arr3v(p.b2)).sum_axis(-1, false), f64(p.res_v))
			return dist, u, v
		}
		CyclideParams {
			xl := affine_point_to_local(base.a_inv3, base.t_inv, x)
			sx := mlx.s_sub(col(xl, 0), base.shift[0])
			sy := mlx.s_sub(col(xl, 1), base.shift[1])
			sz := mlx.s_sub(col(xl, 2), base.shift[2])
			bb := base.b * base.b - base.d * base.d
			rho := sx.multiply(sx).add(sy.multiply(sy)).add(sz.multiply(sz))
			g := mlx.s_add(rho, bb)
			acd := mlx.s_sub(mlx.s_mul(sx, base.a), base.c * base.d)
			imp := g.multiply(g).subtract(mlx.s_mul(acd.multiply(acd), 4.0)).subtract(mlx.s_mul(sy.multiply(sy),
				4.0 * base.b * base.b))
			// first-order SDF: implicit / |grad implicit|
			gx := mlx.s_mul(sx.multiply(g), 4.0).subtract(mlx.s_mul(acd, 8.0 * base.a))
			gy := mlx.s_mul(sy.multiply(g), 4.0).subtract(mlx.s_mul(sy, 8.0 * base.b * base.b))
			gz := mlx.s_mul(sz.multiply(g), 4.0)
			gn := gx.multiply(gx).add(gy.multiply(gy)).add(gz.multiply(gz)).sqrt()
			dist := imp.divide(mlx.where(mlx.s_gt(gn, 1e-12), gn, mlx.ones_like(gn)))
			// closed-form uv inversion (cyclide.v uv()); both periodic
			u := mlx.s_div(sy.multiply(mlx.fs(2.0 * base.b)).arctan2(mlx.s_mul(acd, 2.0)),
				2.0 * math.pi)
			v := mlx.s_div(sz.multiply(mlx.fs(2.0 * base.b)).arctan2(mlx.fs(base.d * base.d +
				base.b * base.b).subtract(rho)), 2.0 * math.pi)
			return dist, u, v
		}
		else {
			panic('displaced base must be sphere/plane/cylinder/cyclide')
		}
	}
}

// residual_sample evaluates the bilinear residual grid at (u, v), both (N,).
// Wrap rules per base: sphere u wrap + v clamp; cylinder u wrap + v zero
// outside the [0,1] band; plane both tile; cyclide both wrap.
fn residual_sample(p DisplacedParams, u mlx.Array, v mlx.Array) mlx.Array {
	ru := p.res_u
	rv := p.res_v
	// u always periodic (plane pre-normalised to [0,1) per tile)
	uw := u.subtract(u.floor())
	x := mlx.s_mul(uw, f64(ru))
	x0 := x.floor().astype(.int32)
	fx := x.subtract(x0.astype(.float32))
	i0 := x0.remainder(mlx.int_scalar(ru))
	i1 := x0.add(mlx.int_scalar(1)).remainder(mlx.int_scalar(ru))
	mut r00 := mlx.Array{}
	mut r10 := mlx.Array{}
	mut r01 := mlx.Array{}
	mut r11 := mlx.Array{}
	if p.base is CyclideParams || p.base is PlaneParams {
		// v periodic (cyclide) / tiling (plane)
		vw := v.subtract(v.floor())
		y := mlx.s_mul(vw, f64(rv))
		y0 := y.floor().astype(.int32)
		fy := y.subtract(y0.astype(.float32))
		j0 := y0.remainder(mlx.int_scalar(rv))
		j1 := y0.add(mlx.int_scalar(1)).remainder(mlx.int_scalar(rv))
		r00 = p.grid.take_axis(j0.multiply(mlx.int_scalar(ru)).add(i0), 0)
		r10 = p.grid.take_axis(j0.multiply(mlx.int_scalar(ru)).add(i1), 0)
		r01 = p.grid.take_axis(j1.multiply(mlx.int_scalar(ru)).add(i0), 0)
		r11 = p.grid.take_axis(j1.multiply(mlx.int_scalar(ru)).add(i1), 0)
		top := r00.multiply(mlx.fs(1.0).subtract(fx)).add(r10.multiply(fx))
		bot := r01.multiply(mlx.fs(1.0).subtract(fx)).add(r11.multiply(fx))
		return top.multiply(mlx.fs(1.0).subtract(fy)).add(bot.multiply(fy))
	}
	// sphere: clamp; cylinder: SUPPRESS the surface outside the [0,1] band
	// (huge negative residual => F > 0 everywhere => no surface; keeps an
	// infinite base cylinder from showing through above/below the band)
	y := mlx.s_mul(v, f64(rv - 1))
	mut zero_mask := mlx.zeros([u.shape()[0]], .bool_)
	if p.base is CylinderParams {
		zero_mask = mlx.s_lt(y, 0.0).logical_or(mlx.s_gt(y, f64(rv - 1)))
	}
	yc := mlx.s_clip(y, 0.0, f64(rv - 1))
	y0 := yc.floor().astype(.int32).clip(mlx.int_scalar(0), mlx.int_scalar(rv - 2))
	fy := yc.subtract(y0.astype(.float32))
	j1 := y0.add(mlx.int_scalar(1))
	r00 = p.grid.take_axis(y0.multiply(mlx.int_scalar(ru)).add(i0), 0)
	r10 = p.grid.take_axis(y0.multiply(mlx.int_scalar(ru)).add(i1), 0)
	r01 = p.grid.take_axis(j1.multiply(mlx.int_scalar(ru)).add(i0), 0)
	r11 = p.grid.take_axis(j1.multiply(mlx.int_scalar(ru)).add(i1), 0)
	top := r00.multiply(mlx.fs(1.0).subtract(fx)).add(r10.multiply(fx))
	bot := r01.multiply(mlx.fs(1.0).subtract(fx)).add(r11.multiply(fx))
	res := top.multiply(mlx.fs(1.0).subtract(fy)).add(bot.multiply(fy))
	return mlx.where(zero_mask, mlx.full_like(res, mlx.f32_scalar(f32(-1e9)), .float32), res)
}

// displaced_f evaluates F(x) = d_base(x) - scale * r(uv(x)) at (N,3) points.
fn displaced_f(p DisplacedParams, x mlx.Array) mlx.Array {
	dist, u, v := displaced_base_dist_uv(p, x)
	res := residual_sample(p, u, v)
	return dist.subtract(mlx.s_mul(res, p.scale))
}

// --- bracket + march ----------------------------------------------------------

// displaced_bracket returns (t0, t1, has): the ray segment that can contain
// the displaced surface (base crossings inflated by the max residual).
fn displaced_bracket(p DisplacedParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	n := o.shape()[0]
	inf := mlx.full([n], mlx.f32_scalar(f32(math.inf(1))), .float32)
	base := p.base
	match base {
		SphereParams {
			c := mlx.arr3v(base.c)
			oc := o.subtract(c)
			bb := mlx.s_mul(oc.multiply(d).sum_axis(-1, false), 2.0)
			rbig := base.r + p.inflate
			cq := mlx.s_sub(oc.multiply(oc).sum_axis(-1, false), rbig * rbig)
			disc := bb.multiply(bb).subtract(mlx.s_mul(cq, 4.0))
			valid := mlx.s_gt(disc, 1e-12)
			sq := mlx.s_max(disc, 0.0).sqrt()
			t0 := bb.negative().subtract(sq).divide(mlx.fs(2.0))
			t1 := bb.negative().add(sq).divide(mlx.fs(2.0))
			has := valid.logical_and(mlx.s_gt(t1, 1e-4))
			return mlx.where(has, mlx.s_max(t0, 1e-4), inf), mlx.where(has, t1, inf), has
		}
		CylinderParams {
			q := mlx.arr3v(base.q)
			ax := mlx.arr3v(base.u)
			oc := o.subtract(q)
			d_par := d.multiply(ax).sum_axis(-1, true)
			o_par := oc.multiply(ax).sum_axis(-1, true)
			d_p := d.subtract(d_par.multiply(ax))
			o_p := oc.subtract(o_par.multiply(ax))
			aa := d_p.multiply(d_p).sum_axis(-1, false)
			bb := mlx.s_mul(o_p.multiply(d_p).sum_axis(-1, false), 2.0)
			rbig := base.r + p.inflate
			cq := mlx.s_sub(o_p.multiply(o_p).sum_axis(-1, false), rbig * rbig)
			disc := bb.multiply(bb).subtract(mlx.s_mul(aa.multiply(cq), 4.0))
			valid := mlx.s_gt(aa, 1e-12).logical_and(mlx.s_gt(disc, 1e-12))
			sq := mlx.s_max(disc, 0.0).sqrt()
			t0 := bb.negative().subtract(sq).divide(mlx.s_mul(aa, 2.0))
			t1 := bb.negative().add(sq).divide(mlx.s_mul(aa, 2.0))
			has := valid.logical_and(mlx.s_gt(t1, 1e-4))
			return mlx.where(has, mlx.s_max(t0, 1e-4), inf), mlx.where(has, t1, inf), has
		}
		PlaneParams {
			nn := mlx.arr3v(base.n)
			denom := nn.multiply(d).sum_axis(-1, false)
			t := mlx.s_rsub(nn.multiply(o).sum_axis(-1, false), base.d).divide(mlx.where(mlx.s_gt(denom.abs(), 1e-9),
				denom, mlx.full_like(denom, mlx.f32_scalar(f32(1e-9)), .float32)))
			has := mlx.s_gt(denom.abs(), 1e-9).logical_and(mlx.s_gt(mlx.s_add(t, p.inflate), 1e-4))
			return mlx.where(has, mlx.s_max(mlx.s_sub(t, p.inflate), 1e-4), inf), mlx.where(has, mlx.s_add(t,
				p.inflate), inf), has
		}
		CyclideParams {
			ts, _, valid := geom_crossings(base, o, d)
			fin := valid.logical_and(ts.isfinite())
			far := mlx.where(fin, ts,
				mlx.full_like(ts, mlx.f32_scalar(f32(-math.inf(1))), .float32))
			near := mlx.where(fin, ts, inf.expand_dims(1).broadcast_to(ts.shape()))
			t1 := far.max_axis(-1, false)
			t0 := near.min_axis(-1, false)
			has := t1.isfinite().logical_and(mlx.s_gt(t1, 1e-4))
			return mlx.where(has, mlx.s_max(mlx.s_sub(t0, p.inflate), 1e-4), inf), mlx.where(has, mlx.s_add(t1,
				p.inflate), inf), has
		}
		else {
			panic('displaced base must be sphere/plane/cylinder/cyclide')
		}
	}
}

// displaced_march marches the bracket for the first entering crossing of F
// and refines it by bisection.  Returns (t, mask).
fn displaced_march(p DisplacedParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	steps := 128
	t0, t1, has := displaced_bracket(p, o, d)
	dt := t1.subtract(t0).divide(mlx.fs(f64(steps)))
	mut pt := t0
	mut pf := displaced_f(p, o.add(t0.expand_dims(1).multiply(d)))
	// rays starting inside the displaced shell hit at the bracket start
	mut found := has.logical_and(mlx.s_le(pf, 0.0))
	mut ta := t0
	mut tb := mlx.where(found, t0, t1)
	for k in 1 .. steps + 1 {
		tk := t0.add(mlx.s_mul(dt, f64(k)))
		fk := displaced_f(p, o.add(tk.expand_dims(1).multiply(d)))
		cross :=
			found.logical_not().logical_and(has).logical_and(mlx.s_gt(pf, 0.0)).logical_and(mlx.s_le(fk, 0.0))
		ta = mlx.where(cross, pt, ta)
		tb = mlx.where(cross, tk, tb)
		found = found.logical_or(cross)
		pf = fk
		pt = tk
		if k % 32 == 0 {
			// materialise periodically to bound the lazy graph
			pf.eval()
			ta.eval()
			tb.eval()
			found.eval()
		}
	}
	for _ in 0 .. 8 {
		mid := ta.add(tb).divide(mlx.fs(2.0))
		fm := displaced_f(p, o.add(mid.expand_dims(1).multiply(d)))
		pos := mlx.s_gt(fm, 0.0) // F(mid) > 0 -> root in [mid, tb]
		ta = mlx.where(pos, mid, ta)
		tb = mlx.where(pos, tb, mid)
	}
	t := ta.add(tb).divide(mlx.fs(2.0))
	return t, found
}

// displaced_normal estimates grad F by central differences (outward).
fn displaced_normal(p DisplacedParams, pos mlx.Array) mlx.Array {
	mut comps := []mlx.Array{}
	for axis in 0 .. 3 {
		mut off := [0.0, 0.0, 0.0]!
		off[axis] = p.eps
		ov := mlx.arr3v(off)
		fp := displaced_f(p, pos.add(ov))
		fm := displaced_f(p, pos.subtract(ov))
		comps << fp.subtract(fm)
	}
	g := mlx.stack(comps, -1)
	n := g.multiply(g).sum_axis(-1, true).sqrt()
	return g.divide(mlx.where(mlx.s_gt(n, 1e-12), n, mlx.ones_like(n)))
}

// --- dispatcher protocol ------------------------------------------------------

pub fn displaced_intersect(p DisplacedParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	t, mask := displaced_march(p, o, d)
	pos := o.add(t.expand_dims(1).multiply(d))
	mut n := displaced_normal(p, pos)
	n = mlx.where(mask.expand_dims(1), n, mlx.zeros_like(n))
	return t, n, mask
}

pub fn displaced_shadow(p DisplacedParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array) {
	return displaced_march(p, o, d)
}

pub fn displaced_uv(p DisplacedParams, pos mlx.Array, n mlx.Array) mlx.Array {
	_, u, v := displaced_base_dist_uv(p, pos)
	return mlx.stack([u, v], -1)
}

pub fn displaced_bounds(p DisplacedParams) ?[2][3]f64 {
	b := geom_bounds(p.base) or { return none }
	m := p.inflate
	return [[b[0][0] - m, b[0][1] - m, b[0][2] - m]!, [b[1][0] + m, b[1][1] + m, b[1][2] + m]!]!
}
