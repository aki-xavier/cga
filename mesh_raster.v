module cga

// Software rasterizer for explicit triangle meshes.
//
// The hybrid renderer ray-traces CGA analytic primitives (plane/sphere/
// cylinder/.../CSG) and rasterizes `TrimeshGeometry` meshes — fast, with no
// O(rays x faces) memory blowup — then composites the two paths by camera-space
// depth.  Rasterized meshes are shaded with direct lighting (no shadows /
// refraction), and faces are back-face culled.  A per-mesh base-colour texture
// (Material.map) is sampled and post-multiplied, mirroring the ray-traced path.
import mlx
import math

// RastResult is the rasterized mesh output (linear float32 colour + depth).
pub struct RastResult {
pub:
	depth mlx.Array // [H,W] float32 camera-space z, +inf where no mesh hit
	color mlx.Array // [H,W,3] float32 linear colour where a mesh is nearest
	hit   mlx.Array // [H,W] bool mask of mesh-covered pixels
}

// transform_normal rotates a 3-vector by the 3x3 part of a row-major 4x4
// (correct for motor transforms; affine scale would need the inverse transpose).
fn transform_normal(m [16]f64, p [3]f64) [3]f64 {
	return [m[0] * p[0] + m[1] * p[1] + m[2] * p[2], m[4] * p[0] + m[5] * p[1] + m[6] * p[2],
		m[8] * p[0] + m[9] * p[1] + m[10] * p[2]]!
}

// rasterize_meshes rasterizes the given (direct TrimeshGeometry) mesh objects
// and shades them, returning a depth + colour buffer composited over all meshes.
pub fn rasterize_meshes(objs []Mesh, camera PerspectiveCamera, w int, h int, fx f64, fy f64, cx f64, cy f64, lit []Light, ambient ?Light) RastResult {
	// CPU buffers (H*W)
	pixel_count := w * h
	mut depth := []f32{len: pixel_count, init: f32(math.inf(1))}
	mut pos := []f32{len: pixel_count * 3}
	mut nrm := []f32{len: pixel_count * 3}
	mut uvbuf := []f32{len: pixel_count * 2}
	mut matidx := []int{len: pixel_count, init: -1}

	// face list per mesh (camera space)
	mut face_data := [][]RastFace{}
	for mi, obj in objs {
		g := obj.geometry
		if g !is TrimeshGeometry {
			continue
		}
		geom := g as TrimeshGeometry
		wm := camera.motor.compose(obj.motor())
		_, _, af := affine_from_motor(wm, identity3())
		mut faces := []RastFace{len: geom.n_faces}
		for i in 0 .. geom.n_faces {
			a := transform_point(af, geom.v0[i])
			b := transform_point(af, [geom.v0[i][0] + geom.e1[i][0],
				geom.v0[i][1] + geom.e1[i][1], geom.v0[i][2] + geom.e1[i][2]]!)
			c := transform_point(af, [geom.v0[i][0] + geom.e2[i][0],
				geom.v0[i][1] + geom.e2[i][1], geom.v0[i][2] + geom.e2[i][2]]!)
			n := transform_normal(af, geom.nrm[i])
			has := geom.uv.len > 0
			uv := if has {
				geom.uv[i]
			} else {
				TriUvs{}
			}
			faces[i] = RastFace{a: a, b: b, c: c, n: n, uv: uv, has_uv: has}
		}
		face_data << faces
		_ = mi
	}

	// rasterize
	for mi, faces in face_data {
		for f in faces {
			// back-face cull: normal must face the camera (origin)
			center := [(f.a[0] + f.b[0] + f.c[0]) / 3.0, (f.a[1] + f.b[1] + f.c[1]) / 3.0,
				(f.a[2] + f.b[2] + f.c[2]) / 3.0]!
			if f.n[0] * center[0] + f.n[1] * center[1] + f.n[2] * center[2] >= 0.0 {
				continue
			}
			// must be in front of the near plane
			if f.a[2] <= 1e-4 || f.b[2] <= 1e-4 || f.c[2] <= 1e-4 {
				continue
			}
			sax := fx * f.a[0] / f.a[2] + cx
			say := fy * f.a[1] / f.a[2] + cy
			sbx := fx * f.b[0] / f.b[2] + cx
			sby := fy * f.b[1] / f.b[2] + cy
			scx := fx * f.c[0] / f.c[2] + cx
			scy := fy * f.c[1] / f.c[2] + cy
			xmin := int(math.max(0.0, math.floor(math.min(sax, math.min(sbx, scx)))))
			xmax := int(math.min(f64(w - 1), math.ceil(math.max(sax, math.max(sbx, scx)))))
			ymin := int(math.max(0.0, math.floor(math.min(say, math.min(sby, scy)))))
			ymax := int(math.min(f64(h - 1), math.ceil(math.max(say, math.max(sby, scy)))))
			denom := (sbx - sax) * (scy - say) - (sby - say) * (scx - sax)
			if math.abs(denom) < 1e-12 {
				continue
			}
			for py in ymin .. ymax + 1 {
				pyc := f64(py) + 0.5
				for px in xmin .. xmax + 1 {
					pxc := f64(px) + 0.5
					// barycentric (edge) weights
					wa := ((sbx - pxc) * (scy - pyc) - (sby - pyc) * (scx - pxc)) / denom
					wb := ((scx - pxc) * (say - pyc) - (scy - pyc) * (sax - pxc)) / denom
					wc := 1.0 - wa - wb
					if wa < 0.0 || wb < 0.0 || wc < 0.0 {
						continue
					}
					// perspective-correct depth + position
					za := f.a[2]
					zb := f.b[2]
					zc := f.c[2]
					inv_w := wa / za + wb / zb + wc / zc
					if inv_w <= 1e-12 {
						continue
					}
					z := 1.0 / inv_w
					off := py * w + px
					if z >= f64(depth[off]) {
						continue
					}
					depth[off] = f32(z)
					ix := (wa * f.a[0] / za + wb * f.b[0] / zb + wc * f.c[0] / zc) / inv_w
					iy := (wa * f.a[1] / za + wb * f.b[1] / zb + wc * f.c[1] / zc) / inv_w
					iz := (wa * f.a[2] / za + wb * f.b[2] / zb + wc * f.c[2] / zc) / inv_w
					pos[off * 3] = f32(ix)
					pos[off * 3 + 1] = f32(iy)
					pos[off * 3 + 2] = f32(iz)
					nrm[off * 3] = f32(f.n[0])
					nrm[off * 3 + 1] = f32(f.n[1])
					nrm[off * 3 + 2] = f32(f.n[2])
					// perspective-correct UVs (if the mesh has them)
					if f.has_uv {
						uu := (wa * f.uv.u0x / za + wb * f.uv.u1x / zb + wc * f.uv.u2x /
							zc) / inv_w
						vv := (wa * f.uv.u0y / za + wb * f.uv.u1y / zb + wc * f.uv.u2y /
							zc) / inv_w
						uvbuf[off * 2] = f32(uu)
						uvbuf[off * 2 + 1] = f32(vv)
					}
					matidx[off] = mi
				}
			}
		}
	}

	// gather rasterized pixels, shade in a batch
	mut idxs := []int{}
	for i in 0 .. pixel_count {
		if matidx[i] >= 0 {
			idxs << i
		}
	}
	mut hits := mlx.zeros([h, w], .bool_)
	mut rast_hit_mask := []f32{len: pixel_count, init: 0.0}
	for oi in idxs {
		rast_hit_mask[oi] = 1.0
	}
	mut color := mlx.zeros([h, w, 3], .float32)

	if idxs.len > 0 {
		k := idxs.len
		pp := mlx.array_f32(f32s_from_pos(pos, idxs), [k, 3])
		nn := mlx.array_f32(f32s_from_nrm(nrm, idxs), [k, 3])
		// view direction = -normalize(pos) (camera at origin)
		vv := pp.negative().divide(pp.multiply(pp).sum_axis(-1, true).sqrt())
		// aggregate per-mesh materials
		mut em_arr := []mlx.Array{}
		mut diff_arr := []mlx.Array{}
		mut spec_arr := []mlx.Array{}
		mut expo_arr := []mlx.Array{}
		mut valid_objs := []Mesh{}
		for _, o in objs {
			if o.geometry is TrimeshGeometry {
				valid_objs << o
			}
		}
		for o in valid_objs {
			em, diff, spec, expo := o.material.shade_params()
			em_arr << mlx.arr3v(em)
			diff_arr << mlx.arr3v(diff)
			spec_arr << mlx.arr3v(spec)
			expo_arr << mlx.fs(expo)
		}
		// map each pixel's matidx (mesh index in `objs`, 0-based) to material params
		mut mi_arr := []i32{len: k}
		for oi in 0 .. k {
			mi_arr[oi] = i32(matidx[idxs[oi]])
		}
		midx := mlx.array_i32(mi_arr, [k])
		emissive := mlx.stack(em_arr, 0).take_axis(midx, 0)
		diff := mlx.stack(diff_arr, 0).take_axis(midx, 0)
		spec := mlx.stack(spec_arr, 0).take_axis(midx, 0)
		expo := mlx.stack(expo_arr, 0).take_axis(midx, 0).expand_dims(1)
		mut vis := []mlx.Array{}
		for _ in lit {
			vis << mlx.ones([k], .float32)
		}
		shaded := shade_batched(emissive, diff, spec, expo, pp, nn, vv, lit, ambient, vis)
		mut sdata := shaded.data_f32()
		// apply the base-colour texture (post-multiply, like the ray-traced map)
		for mi, o in valid_objs {
			if t := o.material.map {
				mut uv_list := []f32{}
				mut bp := []int{}
				for oi in 0 .. k {
					if matidx[idxs[oi]] == mi {
						bp << oi
						uv_list << uvbuf[idxs[oi] * 2]
						uv_list << uvbuf[idxs[oi] * 2 + 1]
					}
				}
				if bp.len > 0 {
					uvarr := mlx.array_f32(uv_list, [bp.len, 2])
					samp := t.sample(uvarr, .repeat, .repeat).take_axis(mlx.array_i32([
						i32(0),
						1,
						2,
					], [3]), 1)
					td := samp.data_f32()
					for j in 0 .. bp.len {
						o3 := bp[j] * 3
						sdata[o3] *= td[j * 3]
						sdata[o3 + 1] *= td[j * 3 + 1]
						sdata[o3 + 2] *= td[j * 3 + 2]
					}
				}
			}
		}
		// scatter back
		mut col_flat := []f32{len: pixel_count * 3, init: 0.0}
		for oi in 0 .. k {
			o := idxs[oi]
			col_flat[o * 3] = sdata[oi * 3]
			col_flat[o * 3 + 1] = sdata[oi * 3 + 1]
			col_flat[o * 3 + 2] = sdata[oi * 3 + 2]
		}
		color = mlx.array_f32(col_flat, [h, w, 3])
	}

	hits = mlx.array_f32(rast_hit_mask, [h, w]).greater(mlx.fs(0.0))
	return RastResult{
		depth: mlx.array_f32(depth, [h, w])
		color: color
		hit:   hits
	}
}

// RastFace is one camera-space triangle.
struct RastFace {
	a      [3]f64
	b      [3]f64
	c      [3]f64
	n      [3]f64
	uv     TriUvs
	has_uv bool
}

fn f32s_from_pos(pos []f32, idxs []int) []f32 {
	mut out := []f32{len: idxs.len * 3}
	for oi, o in idxs {
		out[oi * 3] = pos[o * 3]
		out[oi * 3 + 1] = pos[o * 3 + 1]
		out[oi * 3 + 2] = pos[o * 3 + 2]
	}
	return out
}

fn f32s_from_nrm(nrm []f32, idxs []int) []f32 {
	mut out := []f32{len: idxs.len * 3}
	for oi, o in idxs {
		out[oi * 3] = nrm[o * 3]
		out[oi * 3 + 1] = nrm[o * 3 + 1]
		out[oi * 3 + 2] = nrm[o * 3 + 2]
	}
	return out
}
