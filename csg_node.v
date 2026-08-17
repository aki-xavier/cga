module cga

// CsgGeometry: recursive boolean combinator over solid primitives.
import mlx

// CsgGeometry combines solid children via union / intersection / difference.
pub struct CsgGeometry {
pub:
	op       string
	children []Geometry
}

// csg_geometry builds a CSG node (difference = children[0] - union(children[1:])).
pub fn csg_geometry(op string, children []Geometry) CsgGeometry {
	if op != 'union' && op != 'intersection' && op != 'difference' {
		panic('csg op must be union/intersection/difference, got ${op}')
	}
	if children.len < 2 {
		panic('csg ${op} needs >= 2 children, got ${children.len}')
	}
	for c in children {
		match c {
			CircleGeometry { panic('circle is not a solid (no crossings/contains)') }
			else {}
		}
	}
	return CsgGeometry{
		op:       op
		children: children
	}
}

pub struct CsgParams {
pub:
	op       string
	children []GeometryParams
}

// csg_crossings concatenates all children's boundary crossings.
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

// csg_contains is the whole-tree membership test.
fn csg_contains(p CsgParams, pos mlx.Array) mlx.Array {
	if p.op == 'difference' {
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
		acc = if p.op == 'union' { acc.logical_or(cc) } else { acc.logical_and(cc) }
	}
	return acc
}

// csg_nearest_surface finds the nearest membership-flip crossing.
fn csg_nearest_surface(p CsgParams, o mlx.Array, d mlx.Array) (mlx.Array, mlx.Array, mlx.Array) {
	ts, ns, _ := csg_crossings(p, o, d)
	order := ts.argsort_axis(1)
	ts_s := ts.take_along_axis(order, 1)
	ns_s := ns.take_along_axis(order.expand_dims(2), 1)
	ts_f := mlx.where(ts_s.isfinite(), ts_s, mlx.zeros_like(ts_s))
	delta := 1e-4
	p_plus := o.expand_dims(1).add(s_add(ts_f, delta).expand_dims(2).multiply(d.expand_dims(1)))
	p_minus := o.expand_dims(1).add(s_sub(ts_f, delta).expand_dims(2).multiply(d.expand_dims(1)))
	in_plus := csg_contains(p, p_plus)
	in_minus := csg_contains(p, p_minus)
	flip := in_plus.not_equal(in_minus).logical_and(s_gt(ts_s, 1e-6)).logical_and(ts_s.isfinite())
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
		bp := pos.add(s_mul(n, delta))
		bm := pos.subtract(s_mul(n, delta))
		boundary := geom_contains(cp, bp).not_equal(geom_contains(cp, bm))
		pick := boundary.logical_and(found.logical_not())
		uv_c := geom_uv(cp, pos, n)
		uv = mlx.where(pick.expand_dims(1), uv_c, uv)
		found = found.logical_or(pick)
	}
	return uv
}

pub fn csg_bounds(p CsgParams) ?[2][3]f64 {
	mut bnds := []?[2][3]f64{}
	for cp in p.children {
		bnds << geom_bounds(cp)
	}
	if p.op == 'difference' {
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
	if p.op == 'union' {
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
