module cga

import math
import mlx
import os

fn bake_test_material(col int) Material {
	return standard_material(MaterialParams{
		color:      color_hex(col)
		roughness:  0.3
		metalness:  0.1
		emissive:   color_hex(0x000000)
		opacity:    1.0
		ior:        1.5
		absorption: 0.0
	})
}

fn bake_test_scene(geo Geometry, col int) Scene {
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       geo
		material:       bake_test_material(col)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	return sc
}

fn bake_test_cam() PerspectiveCamera {
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	return cam
}

// frame_psnr returns the PSNR (dB, rgb channels, 0..255) of two frames.
fn frame_psnr(a mlx.Array, b mlx.Array) f64 {
	da := a.data_f32()
	db := b.data_f32()
	mut mse := 0.0
	mut nch := 0
	for i := 0; i < da.len; i += 4 {
		for k in 0 .. 3 {
			d := f64(da[i + k]) - f64(db[i + k])
			mse += d * d
			nch++
		}
	}
	mse /= f64(nch)
	return 10.0 * math.log10(255.0 * 255.0 / mse)
}

// save_diff writes the 4x-amplified per-pixel difference of two frames.
fn save_diff(path string, a mlx.Array, b mlx.Array) {
	da := a.data_f32()
	db := b.data_f32()
	mut diff := []f32{len: da.len}
	for i := 0; i < da.len; i += 4 {
		for k in 0 .. 3 {
			diff[i + k] = f32(math.min(255.0, 4.0 * math.abs(f64(da[i + k]) - f64(db[i + k]))))
		}
		diff[i + 3] = 255.0
	}
	save_frame_png(path, mlx.array_f32(diff, [a.shape()[0], a.shape()[1], 4]))
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

fn test_bake_cube_onto_sphere_vertices() {
	// 96x49 grid: node (48, 24) is the +x equator point (u=0.5 -> phi=0,
	// v=24/48=0.5); the cube face x=0.9 sits at residual 0.9-1.3 = -0.4
	b := bake_surface(sphere_geometry(1.3), cube_trimesh(0.9), 96, 49)
	assert b.vertices.len == 96 * 49
	p := b.vertices[24 * 96 + 48]
	assert math.abs(p[0] - 0.9) < 2e-3
	assert math.abs(p[1]) < 2e-3
	assert math.abs(p[2]) < 2e-3
	// quads = 96 * 48, two tris each, minus 2 * 96 skipped pole degenerates
	assert b.faces.len == 2 * 96 * 48 - 2 * 96
	// the -x node wraps: (i=0) is phi=-pi -> -x, same cube face distance
	q := b.vertices[24 * 96 + 0]
	assert math.abs(q[0] + 0.9) < 2e-3
}

fn test_bake_cube_render_matches_target() {
	cam := bake_test_cam()
	// 48x25: keeps the brute-force trimesh shadow render fast (~2.2k faces)
	baked := bake_surface_mesh(sphere_geometry(1.3), cube_trimesh(0.9), 48, 25)
	img_d := render_frame(bake_test_scene(baked, 0xC0392B), cam, 120, 120, 1)
	img_t := render_frame(bake_test_scene(cube_trimesh(0.9), 0xC0392B), cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/bake_mesh.png', img_d)
	save_frame_png('artifacts/tests/bake_mesh_target.png', img_t)
	save_diff('artifacts/tests/bake_mesh_diff.png', img_d, img_t)
	psnr := frame_psnr(img_d, img_t)
	println('bake mesh PSNR: ${psnr:.2f} dB')
	assert psnr > 16.5 // measured ~17.5 dB at 48x25 with the grazing guards
	// (20.8 dB unguarded at 96x49); the guards cost some cube-edge fidelity
}

fn test_bake_glb_roundtrip() {
	cam := bake_test_cam()
	b := bake_surface(sphere_geometry(1.3), cube_trimesh(0.9), 40, 21)
	os.mkdir_all('artifacts/tests') or {}
	save_glb('artifacts/tests/bake_roundtrip.glb', [
		GltfMeshIn{
			vertices: b.vertices
			faces:    b.faces
			color:    [0.75, 0.22, 0.17]!
		},
	])
	loaded := load_gltf('artifacts/tests/bake_roundtrip.glb') or { panic(err) }
	assert loaded.len == 1
	assert loaded[0].vertices.len == b.vertices.len
	assert loaded[0].faces.len == b.faces.len
	img_direct := render_frame(bake_test_scene(trimesh_geometry(b.vertices, b.faces), 0xC0392B),
		cam, 120, 120, 1)
	img_glb := render_frame(bake_test_scene(gltf_to_geometry(loaded), 0xC0392B), cam, 120, 120, 1)
	psnr := frame_psnr(img_direct, img_glb)
	println('glb roundtrip PSNR: ${psnr:.2f} dB')
	assert psnr > 40.0 // f32 round-trip only
}

fn test_bake_seam_shared_vertices() {
	// concentric sphere target -> perfectly smooth baked sphere; no duplicate
	// seam vertices and no shading crack when the seam faces the camera
	b := bake_surface(sphere_geometry(1.0), sphere_geometry(1.15), 48, 25)
	assert b.vertices.len == 48 * 25 // shared wrap column, no duplicates
	// every vertex at radius ~1.15 (outward hit everywhere)
	for p in b.vertices {
		r := math.sqrt(vec3_dot(p, p))
		assert math.abs(r - 1.15) < 2e-3
	}
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       trimesh_geometry(b.vertices, b.faces)
		material:       bake_test_material(0xC0392B)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 1.0, 0.0]!
		rotation_angle: math.pi / 2.0 // u=0 seam faces the camera
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	img := render_frame(sc, bake_test_cam(), 120, 120, 1)
	data := img.data_f32()
	mut jumps := []f64{}
	for c in 52 .. 68 {
		i0 := (60 * 120 + c) * 4
		i1 := (60 * 120 + c + 1) * 4
		jumps << math.abs(f64(data[i0]) - f64(data[i1])) + math.abs(f64(data[i0 + 1]) -
			f64(data[i1 + 1]))
	}
	jumps.sort()
	// a seam crack would be a big absolute jump; the smooth baked sphere
	// shades slowly (measured worst ~3.4)
	assert jumps[jumps.len - 1] < 8.0
}

fn test_bake_cyclide_wraps() {
	// cyclide base: both uv directions wrap; no degenerate faces expected
	b :=
		bake_surface(cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!), sphere_geometry(1.6), 48, 24)
	assert b.vertices.len == 48 * 24
	assert b.faces.len == 2 * 48 * 24
	img := render_frame(bake_test_scene(trimesh_geometry(b.vertices, b.faces), 0xC0392B),
		bake_test_cam(), 120, 120, 1)
	data := img.data_f32()
	mut red := 0
	for i := 0; i < data.len; i += 4 {
		if data[i] > data[i + 1] + 20.0 && data[i] > data[i + 2] + 20.0 {
			red++
		}
	}
	assert red > 200
}

// count_boundary_edges returns the number of edges used by exactly one
// triangle (0 = closed mesh).
fn count_boundary_edges(faces [][3]int) int {
	mut edge_count := map[u64]int{}
	for f in faces {
		for k in 0 .. 3 {
			a := f[k]
			b := f[(k + 1) % 3]
			key := u64(math.min(a, b)) << 32 | u64(math.max(a, b))
			edge_count[key] = edge_count[key] + 1
		}
	}
	mut n := 0
	for _, cnt in edge_count {
		if cnt == 1 {
			n++
		}
	}
	return n
}

fn test_bake_cylinder_band_is_capped() {
	// the cylinder band has two open loops (z=0 and z=2r); capping must close
	// them with outward-facing fills
	b := bake_surface(cylinder_geometry(0.8, -1.0), cylinder_geometry(0.6, 1.0), 48, 16)
	assert count_boundary_edges(b.faces) == 0
	mut top := 0
	mut bot := 0
	for f in b.faces {
		pa := b.vertices[f[0]]
		pb := b.vertices[f[1]]
		pc := b.vertices[f[2]]
		zmin := math.min(pa[2], math.min(pb[2], pc[2]))
		zmax := math.max(pa[2], math.max(pb[2], pc[2]))
		ea := [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]]!
		eb := [pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]]!
		cz := ea[0] * eb[1] - ea[1] * eb[0] // cross(ea, eb).z
		if zmin > 1.59 {
			assert cz > 0.0 // top cap faces +z (outward)
			top++
		}
		if zmax < 0.01 {
			assert cz < 0.0 // bottom cap faces -z (outward)
			bot++
		}
	}
	// a 48-vertex loop caps into 46 triangles
	assert top == 46
	assert bot == 46
}

fn test_bake_capped_top_render() {
	// looking straight down the tube axis, the capped top is a filled disc
	// (an open tube would show the hollow interior)
	b := bake_surface(cylinder_geometry(0.8, -1.0), cylinder_geometry(0.6, 1.0), 48, 16)
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       trimesh_geometry(b.vertices, b.faces)
		material:       bake_test_material(0xC0392B)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.2, 0.5, 1.0]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.5]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	img := render_frame(sc, cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/bake_capped_top.png', img)
	data := img.data_f32()
	// centre 10x10 block: all cap (red-dominant), no background pixels
	for dy in -5 .. 5 {
		for dx in -5 .. 5 {
			idx := ((60 + dy) * 120 + (60 + dx)) * 4
			assert data[idx] > data[idx + 2] + 20.0
		}
	}
}
