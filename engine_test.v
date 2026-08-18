module cga

import os

fn render_center(geom Geometry, pos [3]f64, name string) [3]f32 {
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{ geometry: geom, material: standard_material(MaterialParams{
		color:      color_hex(0xC0392B)
		roughness:  0.3
		metalness:  0.1
		emissive:   color_hex(0x000000)
		opacity:    1.0
		ior:        1.5
		absorption: 0.0
	}), position: pos, rotation_axis: [0.0, 0.0, 1.0]!, rotation_angle: 0.0, motor: none }))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [pos[0], pos[1], pos[2] + 4.0]!, pos, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at(pos, none)
	img := render_frame(sc, cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/${name}.png', img)
	data := img.data_f32()
	idx := 60 * 120 * 4 + 60 * 4
	return [data[idx], data[idx + 1], data[idx + 2]]!
}

fn test_render_sphere_hits() {
	c := render_center(sphere_geometry(1.0), [0.0, 0.0, 0.0]!, 'sphere')
	assert c[0] > c[2]
	assert c[0] < 250.0
}

fn test_render_cone_hits() {
	c := render_center(cone_geometry(0.8, 2.0), [0.0, 0.0, 0.0]!, 'cone')
	assert c[0] > c[2]
}

fn test_render_ellipsoid_hits() {
	c := render_center(ellipsoid_geometry(1.0, 0.6, 0.8), [0.0, 0.0, 0.0]!, 'ellipsoid')
	assert c[0] > c[2]
}

fn test_render_cyclide_nonempty() {
	// ring cyclide (c < d < a): a=1, b=0.98, d=0.3 gives c~=0.199, a proper
	// torus-like ring.  (d == c would be a degenerate horn with a cusp that
	// fills the frame.)  The centre pixel is the ring's hole, so count
	// red-dominant pixels instead of sampling the centre.
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.1
			emissive:   color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	img := render_frame(sc, cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/cyclide.png', img)
	data := img.data_f32()
	mut hit := 0
	for i in 0 .. 120 * 120 {
		if data[i * 4] > data[i * 4 + 2] + 20.0 {
			hit++
		}
	}
	assert hit > 100
}

fn test_render_torus_nonempty() {
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       torus_geometry(1.0, 0.3)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.1
			emissive:   color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	img := render_frame(sc, cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/torus.png', img)
	data := img.data_f32()
	mut nonbg := 0
	for i in 0 .. 120 * 120 {
		if data[i * 4] < 200.0 {
			nonbg++
		}
	}
	assert nonbg > 100
}

fn test_render_trimesh_nonempty() {
	mut sc := scene(none)
	verts := [[0.0, 0.0, 1.0]!, [1.0, 0.0, 0.0]!, [-0.5, 0.866, 0.0]!,
		[0.0, 0.0, -1.0]!]
	faces := [[0, 1, 2]!, [0, 2, 3]!, [0, 3, 1]!, [1, 3, 2]!]
	sc.add_mesh(mesh(MeshParams{
		geometry:       trimesh_geometry(verts, faces)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.1
			emissive:   color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	img := render_frame(sc, cam, 120, 120, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/trimesh.png', img)
	data := img.data_f32()
	mut hit := 0
	for i in 0 .. 120 * 120 {
		if data[i * 4] > data[i * 4 + 2] + 20.0 {
			hit++
		}
	}
	assert hit > 100
}
