module cga

import math
import mlx
import os

fn disp_test_material() Material {
	return standard_material(MaterialParams{
		color:      color_hex(0xC0392B)
		roughness:  0.3
		metalness:  0.1
		emissive:   color_hex(0x000000)
		opacity:    1.0
		ior:        1.5
		absorption: 0.0
	})
}

fn disp_test_scene(geo Geometry) Scene {
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       geo
		material:       disp_test_material()
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	return sc
}

fn disp_test_cam() PerspectiveCamera {
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	return cam
}

// pix_diff returns (max abs channel diff, mean abs channel diff) of two
// (H, W, 4) 0..255 frames.
fn pix_diff(a mlx.Array, b mlx.Array) (f64, f64) {
	da := a.data_f32()
	db := b.data_f32()
	mut mx := 0.0
	mut sum := 0.0
	for i := 0; i < da.len; i += 4 {
		for k in 0 .. 3 {
			d := math.abs(f64(da[i + k]) - f64(db[i + k]))
			mx = math.max(mx, d)
			sum += d
		}
	}
	return mx, sum / f64(da.len)
}

fn sinusoid_grid(res_u int, res_v int, amp f64, periods int) []f32 {
	mut g := []f32{len: res_u * res_v}
	for j in 0 .. res_v {
		for i in 0 .. res_u {
			u := f64(i) / f64(res_u)
			g[j * res_u + i] = f32(amp * math.sin(2.0 * math.pi * f64(periods) * u))
		}
	}
	return g
}

fn test_displaced_zero_residual_matches_sphere() {
	cam := disp_test_cam()
	clean := render_frame(disp_test_scene(sphere_geometry(1.0)), cam, 120, 120, 1)
	zero :=
		displaced_geometry(sphere_geometry(1.0), []f32{len: 48 * 24, init: f32(0.0)}, 48, 24, 1.0)
	disp := render_frame(disp_test_scene(zero), cam, 120, 120, 1)
	mx, mean := pix_diff(clean, disp)
	assert mean < 0.5
	assert mx < 3.0
}

fn test_displaced_zero_residual_matches_plane() {
	// tilted view of a y-up plane, residual zero -> identical to the plane
	mut sc := scene(none)
	geo := plane_geometry([0.0, 1.0, 0.0]!, 0.0)
	sc.add_mesh(mesh(MeshParams{
		geometry:       geo
		material:       disp_test_material()
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(50.0, 1.0, 0.1, 100.0, [0.0, 2.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	clean := render_frame(sc, cam, 120, 120, 1)
	zero := displaced_geometry(geo, []f32{len: 16 * 16, init: f32(0.0)}, 16, 16, 0.25)
	mut sc2 := scene(none)
	sc2.add_mesh(mesh(MeshParams{
		geometry:       zero
		material:       disp_test_material()
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc2.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc2.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	disp := render_frame(sc2, cam, 120, 120, 1)
	mx, mean := pix_diff(clean, disp)
	assert mean < 0.5
	assert mx < 3.0
}

fn test_displaced_sinusoid_grows_silhouette() {
	cam := disp_test_cam()
	clean := render_frame(disp_test_scene(sphere_geometry(1.0)), cam, 120, 120, 1)
	grid := sinusoid_grid(96, 48, 0.1, 6)
	disp := render_frame(disp_test_scene(displaced_geometry(sphere_geometry(1.0), grid, 96, 48, 1.0)),
		cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/displaced_sphere.png', disp)
	mx, mean := pix_diff(clean, disp)
	// shading differs (bumps) and the silhouette grows by ~max|r| = 0.1
	assert mean > 0.5
	assert mx > 10.0
	cnt := fn (img mlx.Array) int {
		d := img.data_f32()
		mut n := 0
		for i := 0; i < d.len; i += 4 {
			if d[i] > d[i + 2] + 20.0 {
				n++
			}
		}
		return n
	}
	assert cnt(disp) > cnt(clean) + 50
}

fn test_displaced_seam_continuity() {
	// CPU: the residual sampler wraps seamlessly at u = 0/1 (same wrapped
	// point must give the same texel blend, exactly)
	grid := sinusoid_grid(96, 48, 0.1, 6)
	g := displaced_geometry(sphere_geometry(1.0), grid, 96, 48, 1.0)
	assert g.residual_at(1.0, 0.5) == g.residual_at(0.0, 0.5)
	assert g.residual_at(-0.001, 0.5) == g.residual_at(0.999, 0.5)
	assert math.abs(g.residual_at(0.0, 0.5)) < 1e-6 // node 0 is sin(0) = 0
	// GPU: rotate the sphere so the u=0 seam faces the camera; the centre
	// columns must shade smoothly (no crack line)
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       g
		material:       disp_test_material()
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 1.0, 0.0]!
		rotation_angle: math.pi / 2.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	cam := disp_test_cam()
	img := render_frame(sc, cam, 120, 120, 1)
	// the column-to-column shading jump must stay SMOOTH across the seam
	// (a crack would show as an isolated spike in the jump profile)
	data := img.data_f32()
	mut jumps := []f64{}
	for c in 52 .. 68 {
		i0 := (60 * 120 + c) * 4
		i1 := (60 * 120 + c + 1) * 4
		jumps << math.abs(f64(data[i0]) - f64(data[i1])) + math.abs(f64(data[i0 + 1]) -
			f64(data[i1 + 1]))
	}
	jumps.sort()
	median := jumps[jumps.len / 2]
	worst := jumps[jumps.len - 1]
	assert worst < 2.0 * median // measured ratio ~1.7 (legit bump-flank slope)
}

fn test_displaced_cyclide_render() {
	// sinusoid bump band on a ring cyclide (both uv directions exercised)
	mut grid := []f32{len: 96 * 48}
	for j in 0 .. 48 {
		for i in 0 .. 96 {
			grid[j * 96 + i] = f32(0.12 * math.sin(2.0 * math.pi * 8.0 * f64(i) / 96.0) * math.sin(2.0 * math.pi * 3.0 * f64(j) / 48.0))
		}
	}
	geo := displaced_geometry(cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!), grid, 96, 48, 1.0)
	cam := disp_test_cam()
	img := render_frame(disp_test_scene(geo), cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/displaced_cyclide.png', img)
	data := img.data_f32()
	mut red := 0
	for i := 0; i < data.len; i += 4 {
		if data[i] > data[i + 1] + 20.0 && data[i] > data[i + 2] + 20.0 {
			red++
		}
	}
	assert red > 200
	// zero-residual displaced cyclide == the plain cyclide (within marcher
	// accuracy).  A sampled marcher can flip single silhouette pixels (the
	// F<0 dip is thinner than a march step there), so assert on the COUNT of
	// disagreeing pixels plus the mean, not the max.
	clean := render_frame(disp_test_scene(cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!)), cam,
		120, 120, 1)
	zero := displaced_geometry(cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!),
		[]f32{len: 96 * 48, init: f32(0.0)}, 96, 48, 1.0)
	disp := render_frame(disp_test_scene(zero), cam, 120, 120, 1)
	da := clean.data_f32()
	db := disp.data_f32()
	mut bad := 0
	mut sum := 0.0
	for i := 0; i < da.len; i += 4 {
		mut d := 0.0
		for k in 0 .. 3 {
			d += math.abs(f64(da[i + k]) - f64(db[i + k]))
		}
		sum += d / 3.0
		if d > 5.0 {
			bad++
		}
	}
	assert sum / f64(da.len / 4) < 0.5
	assert bad <= 8 // measured 2 (grazing silhouette pixels)
}

// cube_trimesh returns a cube of half-side h centred at the origin.
fn cube_trimesh(h f64) TrimeshGeometry {
	mut v := [][3]f64{len: 8}
	mut k := 0
	for sx in [-1.0, 1.0] {
		for sy in [-1.0, 1.0] {
			for sz in [-1.0, 1.0] {
				v[k] = [sx * h, sy * h, sz * h]!
				k++
			}
		}
	}
	// vertex index = 4*x + 2*y + z bits
	f := [
		[0, 1, 3]!,
		[0, 3, 2]!, // x = -h
		[4, 6, 7]!,
		[4, 7, 5]!, // x = +h
		[0, 4, 5]!,
		[0, 5, 1]!, // y = -h
		[2, 3, 7]!,
		[2, 7, 6]!, // y = +h
		[0, 2, 6]!,
		[0, 6, 4]!, // z = -h
		[1, 5, 7]!,
		[1, 7, 3]!, // z = +h
	]
	return trimesh_geometry(v, f)
}

fn test_bake_cube_onto_sphere() {
	base := sphere_geometry(1.3)
	target := cube_trimesh(0.9)
	baked := bake_residual(base, target, 96, 48, 1.0)
	// node at u=0.5, v=0.5 is the +x point: the cube face is at x=0.9, so the
	// residual is 0.9 - 1.3 = -0.4
	assert math.abs(baked.residual_at(0.5, 0.5) + 0.4) < 2e-3
	cam := disp_test_cam()
	img_d := render_frame(disp_test_scene(baked), cam, 120, 120, 1)
	img_t := render_frame(disp_test_scene(target), cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/displaced_bake.png', img_d)
	save_frame_png('artifacts/tests/displaced_bake_target.png', img_t)
	// diff image (amplified 4x)
	da := img_d.data_f32()
	db := img_t.data_f32()
	mut diff := []f32{len: da.len}
	mut mse := 0.0
	mut nch := 0
	for i := 0; i < da.len; i += 4 {
		for k in 0 .. 3 {
			d := f64(da[i + k]) - f64(db[i + k])
			mse += d * d
			nch++
			diff[i + k] = f32(math.min(255.0, 4.0 * math.abs(d)))
		}
		diff[i + 3] = 255.0
	}
	mse /= f64(nch)
	psnr := 10.0 * math.log10(255.0 * 255.0 / mse)
	println('bake PSNR: ${psnr:.2f} dB')
	save_frame_png('artifacts/tests/displaced_bake_diff.png', mlx.array_f32(diff, [
		120,
		120,
		4,
	]))
	assert psnr > 15.0 // measured ~19-20 dB (FD normals vs flat shading)
}
