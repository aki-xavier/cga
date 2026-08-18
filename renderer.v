module cga

// Off-screen ray tracing renderer (MLX batch).  render(scene, camera) returns
// an (H, W, 4) float32 RGBA frame (0..255).  Supports opaque + transparent
// materials (Whitted Fresnel reflection / Beer refraction), hard shadows, SSAA
// and sRGB encode.
import mlx
import math

pub struct Renderer {
pub mut:
	width     int
	height    int
	aa        int
	max_depth int
	cam       ?PerspectiveCamera
}

// renderer builds a Renderer (aa = supersampling factor).
pub fn renderer(width int, height int, aa int, max_depth int) Renderer {
	if aa < 1 {
		panic('aa must be >= 1, got ${aa}')
	}
	return Renderer{
		width:     width
		height:    height
		aa:        aa
		max_depth: max_depth
	}
}

// build_rays builds the (aa^2 * H * W, 3) camera-space unit ray directions.
fn (mut r Renderer) build_rays() mlx.Array {
	hh := r.height
	ww := r.width
	cam := r.cam or { panic('no camera') }
	fy := f64(hh) / (2.0 * math.tan(math.radians(cam.fov) / 2.0))
	fx := fy * cam.aspect
	cx := f64(ww - 1) / 2.0
	cy := f64(hh - 1) / 2.0
	u0 := mlx.s_div(mlx.s_sub(mlx.arange(0, ww, 1, .float32), cx), fx)
	v0 := mlx.s_div(mlx.s_sub(mlx.arange(0, hh, 1, .float32), cy), fy)
	z := mlx.ones([hh, ww], .float32)
	mut dirs := []mlx.Array{}
	k := r.aa
	for j in 0 .. k {
		for i in 0 .. k {
			off_u := (f64(i) + 0.5) / f64(k) - 0.5
			off_v := (f64(j) + 0.5) / f64(k) - 0.5
			du := off_u / fx
			dv := off_v / fy
			u := mlx.s_add(u0, du).expand_dims(0).broadcast_to([hh, ww])
			v := mlx.s_add(v0, dv).expand_dims(1).broadcast_to([hh, ww])
			dirs << mlx.stack([u, v, z], -1)
		}
	}
	rays := mlx.concatenate(dirs, 0).reshape([-1, 3])
	n := rays.multiply(rays).sum_axis(-1, true).sqrt()
	return rays.divide(n)
}

// render produces the (H, W, 4) uint8 RGBA frame.
pub fn (mut r Renderer) render(scene Scene, camera PerspectiveCamera) mlx.Array {
	r.cam = camera
	rays := r.build_rays()
	o := mlx.zeros_like(rays)
	n_rays := o.shape()[0]
	bg := mlx.arr3v(scene.background.rgb()).broadcast_to([n_rays, 3])
	mut lit := []Light{}
	mut ambient := ?Light(none)
	for light in scene.lights {
		if light.kind == .ambient {
			ambient = light
		} else {
			lit << light_to_camera(light, camera.motor)
		}
	}
	in_medium := mlx.zeros([n_rays], .bool_)
	sigma := mlx.zeros([n_rays], .float32)
	mut rgb := r.trace(scene, o, rays, lit, ambient, bg, in_medium, sigma, 0)
	s := r.aa * r.aa
	if s > 1 {
		rgb = rgb.reshape([s, n_rays / s, 3]).mean_axis(0, false)
	}
	rgb = mlx.s_clip(rgb, 0.0, 1.0)
	rgb = mlx.where(mlx.s_le(rgb, 0.0031308), mlx.s_mul(rgb, 12.92), mlx.s_sub(mlx.s_mul(mlx.s_pow(rgb, 1.0 / 2.4),
		1.055), 0.055))
	mut rgba := mlx.concatenate([rgb, mlx.ones([n_rays / s, 1], .float32)], -1)
	rgba = mlx.s_clip(mlx.s_add(mlx.s_mul(rgba, 255.0), 0.5), 0.0, 255.0)
	return rgba.reshape([r.height, r.width, 4])
}

// trace returns the (N,3) linear colour for a ray bundle.  Transparent hits
// split into Fresnel reflection + refraction (Beer absorption) up to max_depth.
fn (r Renderer) trace(scene Scene, o mlx.Array, d mlx.Array, lit []Light, ambient ?Light, bg mlx.Array, in_medium mlx.Array, sigma mlx.Array, depth int) mlx.Array {
	hit, t, n0, local, op, ior, abso := r.nearest(scene, o, d, lit, ambient, depth == 0)
	mut cos_i := d.multiply(n0).sum_axis(-1, true).negative()
	n := mlx.where(mlx.s_lt(cos_i, 0.0), n0.negative(), n0)
	cos_i = cos_i.abs()
	mut result := mlx.where(hit.expand_dims(1), local, bg)
	if depth < r.max_depth {
		need := hit.logical_and(mlx.s_lt(op, 1.0))
		if need.sum().item_f32() > 0.0 {
			eta := mlx.where(in_medium.expand_dims(1), ior.expand_dims(1),
				mlx.fs(1.0).divide(ior.expand_dims(1)))
			k :=
				mlx.fs(1.0).subtract(eta.multiply(eta).multiply(mlx.fs(1.0).subtract(cos_i.multiply(cos_i))))
			cos_t := mlx.s_max(k, 0.0).sqrt()
			g := mlx.fs(1.0).divide(eta)
			rs := cos_i.subtract(g.multiply(cos_t)).divide(mlx.s_max(cos_i.add(g.multiply(cos_t)),
				1e-12))
			rp := cos_t.subtract(g.multiply(cos_i)).divide(mlx.s_max(cos_t.add(g.multiply(cos_i)),
				1e-12))
			mut fres := mlx.s_mul(rs.multiply(rs).add(rp.multiply(rp)), 0.5)
			fres = mlx.where(mlx.s_le(k, 0.0), mlx.ones_like(fres), fres)
			p := o.add(t.expand_dims(1).multiply(d))
			d_r := d.add(n.multiply(mlx.s_mul(cos_i, 2.0)))
			d_t := d.multiply(eta).add(n.multiply(eta.multiply(cos_i).subtract(cos_t)))
			entering := in_medium.logical_not()
			sig_next := mlx.where(entering, abso, mlx.fs(0.0))
			refl := r.trace(scene, p.add(mlx.s_mul(n, 1e-3)), d_r, lit, ambient, bg, in_medium, sigma,

				depth + 1)
			refr := r.trace(scene, p.subtract(mlx.s_mul(n, 1e-3)), d_t, lit, ambient, bg, entering,
				sig_next, depth + 1)
			body :=
				op.expand_dims(1).multiply(local).add(mlx.fs(1.0).subtract(op.expand_dims(1)).multiply(refr))
			glass := fres.multiply(refl).add(mlx.fs(1.0).subtract(fres).multiply(body))
			result = mlx.where(need.expand_dims(1), glass, result)
		}
	}
	att := mlx.where(in_medium.logical_and(hit).expand_dims(1),
		sigma.negative().multiply(t).expand_dims(1).exp(), mlx.fs(1.0))
	return result.multiply(att)
}

// nearest intersects a ray bundle against all objects and shades the nearest hit.
fn (r Renderer) nearest(scene Scene, o mlx.Array, d mlx.Array, lit []Light, ambient ?Light, primary bool) (mlx.Array, mlx.Array, mlx.Array, mlx.Array, mlx.Array, mlx.Array, mlx.Array) {
	n_rays := o.shape()[0]
	mut best_t := mlx.full([n_rays], mlx.f32_scalar(f32(math.inf(1))), .float32)
	mut best_n := mlx.zeros([n_rays, 3], .float32)
	mut best_uv := mlx.zeros([n_rays, 2], .float32)
	mut best_idx := mlx.zeros([n_rays], .int32)
	objs := scene.objects
	cam := r.cam or { panic('no camera') }
	mut params_list := []GeometryParams{}
	for i, obj in objs {
		wm := cam.motor.compose(obj.motor())
		params := geom_to_camera(obj.geometry, wm)
		params_list << params
		if primary {
			if b := geom_bounds(params) {
				if b[1][2] <= 1e-6 {
					continue
				}
			}
		}
		t, n_i, mask := geom_intersect(params, o, d)
		uv_i := geom_uv(params, o.add(t.expand_dims(1).multiply(d)), n_i)
		nearer := mask.logical_and(t.less(best_t))
		best_t = mlx.where(nearer, t, best_t)
		best_n = mlx.where(nearer.expand_dims(1), n_i, best_n)
		best_uv = mlx.where(nearer.expand_dims(1), uv_i, best_uv)
		best_idx = mlx.where(nearer, mlx.full([n_rays], mlx.int_scalar(i), .int32), best_idx)
	}
	hit := best_t.isfinite()
	// gather per-object material scalars
	mut op := mlx.ones([n_rays], .float32)
	mut ior := mlx.full([n_rays], mlx.f32_scalar(1.5), .float32)
	mut abso := mlx.zeros([n_rays], .float32)
	if objs.len > 0 {
		mut op_arr := []mlx.Array{}
		mut ior_arr := []mlx.Array{}
		mut abso_arr := []mlx.Array{}
		for obj in objs {
			op_arr << mlx.fs(obj.material.opacity)
			ior_arr << mlx.fs(obj.material.ior)
			abso_arr << mlx.fs(obj.material.absorption)
		}
		ops := mlx.stack(op_arr, 0)
		iors := mlx.stack(ior_arr, 0)
		absos := mlx.stack(abso_arr, 0)
		op = ops.take_axis(best_idx, 0)
		ior = iors.take_axis(best_idx, 0)
		abso = absos.take_axis(best_idx, 0)
	}
	cos_i := d.multiply(best_n).sum_axis(-1, true).negative()
	best_n = mlx.where(mlx.s_lt(cos_i, 0.0), best_n.negative(), best_n)
	p := o.add(best_t.expand_dims(1).multiply(d))
	// shadow rays
	p_s := p.add(mlx.s_mul(best_n, 1e-3))
	mut vis := []mlx.Array{}
	for light in lit {
		ld, _ := light_direction_at(light, p)
		far := light_far(light, p)
		mut v := mlx.ones([n_rays], .float32)
		for j, obj in objs {
			st, m := geom_shadow(params_list[j], p_s, ld)
			occ := if far.ndim() == 1 { m.logical_and(st.less(far)) } else { m }
			v = v.multiply(mlx.where(occ, mlx.fs(1.0 - obj.material.opacity), mlx.fs(1.0)))
		}
		vis << v
	}
	// batched shading
	mut acc := mlx.zeros([n_rays, 3], .float32)
	if objs.len > 0 {
		mut em_arr := []mlx.Array{}
		mut diff_arr := []mlx.Array{}
		mut spec_arr := []mlx.Array{}
		mut expo_arr := []mlx.Array{}
		for obj in objs {
			em, diff, spec, expo := obj.material.shade_params()
			em_arr << mlx.arr3v(em)
			diff_arr << mlx.arr3v(diff)
			spec_arr << mlx.arr3v(spec)
			expo_arr << mlx.fs(expo)
		}
		emissive := mlx.stack(em_arr, 0).take_axis(best_idx, 0)
		diff := mlx.stack(diff_arr, 0).take_axis(best_idx, 0)
		spec := mlx.stack(spec_arr, 0).take_axis(best_idx, 0)
		expo := mlx.stack(expo_arr, 0).take_axis(best_idx, 0).expand_dims(1)
		acc = shade_batched(emissive, diff, spec, expo, p, best_n, d, lit, ambient, vis)
		for i, obj in objs {
			if tex := obj.material.map {
				sampled := tex.sample(best_uv, .repeat, .repeat).take_axis(mlx.array_i32([
					i32(0),
					1,
					2,
				], [3]), 1)
				acc = mlx.where(best_idx.equal(mlx.int_scalar(i)).expand_dims(1),
					acc.multiply(sampled), acc)
			}
		}
	}
	return hit, best_t, best_n, acc, op, ior, abso
}

// render_frame is the single-frame convenience entry point.
pub fn render_frame(scene Scene, camera PerspectiveCamera, width int, height int, aa int) mlx.Array {
	mut r := renderer(width, height, aa, 3)
	return r.render(scene, camera)
}
