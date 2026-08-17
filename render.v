module cga

// Inverse rendering: render a CGA primitive scene (plane / sphere / cylinder
// blades) back to a 2D depth image + visualisation RGB.
import mlx
import math

// RenderPrimitive is one renderable primitive (metre-space blade + region/alpha).
pub struct RenderPrimitive {
pub:
	kind   string // "plane" | "sphere" | "cylinder"
	blade  Multivector
	region int
	alpha  f64
pub mut:
	cylinder_data ?Cylinder // set for "cylinder"
}

pub fn render_primitive(kind string, blade Multivector, region int, alpha f64) RenderPrimitive {
	return RenderPrimitive{
		kind:   kind
		blade:  blade
		region: region
		alpha:  alpha
	}
}

// RenderResult is a depth image plus a visualisation RGB image.
pub struct RenderResult {
pub:
	depth mlx.Array // (H,W) float32 camera Z (0 = no hit)
	rgb   mlx.Array // (H,W,3) float32 (0..255)
}

const palette_rgb = [
	[244.0, 67.0, 54.0]!,
	[33.0, 150.0, 243.0]!,
	[76.0, 175.0, 80.0]!,
	[255.0, 193.0, 7.0]!,
	[156.0, 39.0, 176.0]!,
	[0.0, 188.0, 212.0]!,
	[255.0, 87.0, 34.0]!,
	[139.0, 195.0, 74.0]!,
	[63.0, 81.0, 181.0]!,
	[255.0, 235.0, 59.0]!,
	[0.0, 150.0, 136.0]!,
	[233.0, 30.0, 99.0]!,
]

// render_scene renders primitives to depth + rgb.
pub fn render_scene(prims []RenderPrimitive, fx f64, fy f64, cx f64, cy f64, h int, w int, regions ?mlx.Array, motor ?Multivector, near f64, far f64) RenderResult {
	if prims.len == 0 {
		return RenderResult{
			depth: mlx.zeros([h, w], .float32)
			rgb:   mlx.zeros([h, w, 3], .float32)
		}
	}
	yy := mlx.arange(0, h, 1, .float32).expand_dims(1).broadcast_to([h, w])
	xx := mlx.arange(0, w, 1, .float32).expand_dims(0).broadcast_to([h, w])
	dirs := mlx.stack([s_div(s_sub(xx, cx), fx), s_div(s_sub(yy, cy), fy),
		mlx.ones([h, w], .float32)], -1)
	mut light := arr3(0.3, 0.6, 1.0)
	light = light.divide(light.multiply(light).sum().sqrt())
	mut best := mlx.full([h, w], mlx.f32_scalar(f32(math.inf(1))), .float32)
	mut hits_t := []mlx.Array{}
	mut hits_rgb := []mlx.Array{}
	mut hits_a := []mlx.Array{}
	for p in prims {
		b := if m := motor { m.apply(p.blade) } else { p.blade }
		sel := if r := regions {
			r.equal(mlx.int_scalar(p.region))
		} else {
			mlx.ones([h, w], .bool_)
		}
		mut t := mlx.Array{}
		mut nrm := mlx.Array{}
		if p.kind == 'plane' {
			n := arr3v(b.euclidean_vector())
			d := b.einf_coeff()
			mut denom := dirs.multiply(n).sum_axis(-1, false)
			denom = mlx.where(s_gt(denom.abs(), 1e-8), denom, mlx.full_like(denom,
				mlx.f32_scalar(f32(math.inf(1))), .float32))
			t = fs(d).divide(denom)
			nrm = n.broadcast_to([h, w, 3])
		} else if p.kind == 'sphere' {
			c, r := sphere_from_dual(b)
			ca := arr3v(c)
			a := dirs.multiply(dirs).sum_axis(-1, false)
			bb := s_mul(dirs.multiply(ca).sum_axis(-1, false), -2.0)
			cc := s_sub(ca.multiply(ca).sum(), r * r)
			disc := bb.multiply(bb).subtract(s_mul(a.multiply(cc), 4.0))
			t = mlx.where(s_gt(disc, 0.0),
				bb.negative().subtract(s_max(disc, 0.0).sqrt()).divide(s_mul(a, 2.0)), mlx.full_like(a,
				mlx.f32_scalar(f32(math.inf(1))), .float32))
			hit := t.expand_dims(2).multiply(dirs).subtract(ca)
			nrm = hit.divide(s_max(hit.multiply(hit).sum_axis(-1, true).sqrt(), 1e-8))
		} else {
			cyl := p.cylinder_data or { panic('cylinder render needs cylinder_data') }
			q := arr3v(cyl.axis_point)
			n := arr3v(cyl.axis_dir)
			r := cyl.radius
			dn := dirs.multiply(n).sum_axis(-1, false)
			qn := q.multiply(n).sum()
			aq := dirs.multiply(dirs).sum_axis(-1, false).subtract(dn.multiply(dn))
			bq := dn.multiply(qn).subtract(dirs.multiply(q).sum_axis(-1, false))
			cq := s_sub(q.multiply(q).sum().subtract(qn.multiply(qn)), r * r)
			disc := bq.multiply(bq).subtract(aq.multiply(cq))
			t = mlx.where(s_gt(aq, 1e-8).logical_and(s_gt(disc, 0.0)),
				bq.negative().subtract(s_max(disc, 0.0).sqrt()).divide(aq), mlx.full_like(aq,
				mlx.f32_scalar(f32(math.inf(1))), .float32))
			hit := t.expand_dims(2).multiply(dirs).subtract(q)
			rad := hit.subtract(hit.multiply(n).sum_axis(-1, true).multiply(n))
			nrm = rad.divide(s_max(rad.multiply(rad).sum_axis(-1, true).sqrt(), 1e-8))
		}
		t = mlx.where(s_gt(t, near).logical_and(s_lt(t, far)), t, mlx.full_like(t,
			mlx.f32_scalar(f32(math.inf(1))), .float32))
		t = mlx.where(sel, t, mlx.full_like(t, mlx.f32_scalar(f32(math.inf(1))), .float32))
		best = best.minimum(t)
		col := arr3v(palette_rgb[p.region % 12])
		sh := s_max(nrm.multiply(light).sum_axis(-1, false), 0.0)
		rgb_p :=
			col.expand_dims(0).expand_dims(0).multiply(s_add(s_mul(sh, 0.65), 0.35).expand_dims(2))
		valid1 := s_lt(t, math.inf(1))
		hits_t << t
		hits_rgb << mlx.where(valid1.expand_dims(2), rgb_p, mlx.zeros_like(rgb_p))
		hits_a << mlx.where(valid1, fs(p.alpha), fs(0.0))
	}
	ts := mlx.stack(hits_t, 0)
	order := ts.argsort_axis(0)
	rgbs := mlx.stack(hits_rgb, 0).take_along_axis(order.expand_dims(3), 0)
	alphas := mlx.stack(hits_a, 0).take_along_axis(order, 0)
	mut acc := mlx.zeros([h, w, 3], .float32)
	mut trans := mlx.ones([h, w], .float32)
	for i in 0 .. prims.len {
		a_i := alphas.take_axis(mlx.int_scalar(i), 0)
		acc =
			acc.add(trans.multiply(a_i).expand_dims(2).multiply(rgbs.take_axis(mlx.int_scalar(i), 0)))
		trans = trans.multiply(fs(1.0).subtract(a_i))
	}
	depth := mlx.where(s_lt(best, far), best, fs(0.0))
	return RenderResult{
		depth: depth
		rgb:   acc
	}
}
