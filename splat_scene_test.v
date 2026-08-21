module cga

import math
import os

// green splat cloud with one splat at a local position
fn one_splat_cloud(pos [3]f64, scale f64, opacity f64) SplatsGeometry {
	return splats_geometry(Gaussians{
		splats: [
			Gaussian{
				mean:    pos
				quat:    Quaternion{
					w: 1.0
					x: 0.0
					y: 0.0
					z: 0.0
				}
				scale:   [scale, scale, scale]!
				opacity: opacity
				color:   color_hex(0x00FF00)
			},
		]
	})
}

fn scene_test_camera(w int, h int) PerspectiveCamera {
	mut cam := perspective_camera(40.0, f64(w) / f64(h), 0.1, 100.0, [0.0, 0.0, 4.0]!, [
		0.0,
		0.0,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	return cam
}

// opaque red sphere + red cyclide + lights, NO splats
fn mixed_opaque_meshes() []Mesh {
	mut objs := []Mesh{}
	objs << mesh(MeshParams{
		geometry:       sphere_geometry(1.0)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xC0392B)
			roughness:  0.5
			metalness:  0.0
			emissive:   color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	})
	objs << mesh(MeshParams{
		geometry:       cyclide_geometry(1.6, 1.5, 0.8, [0.0, 0.0, -3.0]!)
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
	})
	return objs
}

fn mixed_lights() []Light {
	return [directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!),
		ambient_light(color_hex(0xFFFFFF), 0.3)]
}

fn test_splat_mesh_mixed_scene_occlusion() {
	w := 128
	h := 128
	cam := scene_test_camera(w, h)
	mut sc := scene(color_rgb(0.0, 0.0, 0.0))
	for m in mixed_opaque_meshes() {
		sc.add_mesh(m)
	}
	for l in mixed_lights() {
		sc.add_light(l)
	}
	// splat mesh: one splat hidden behind the sphere, one in front of it
	mut g := Gaussians{
		splats: []Gaussian{}
	}
	g.splats << Gaussian{
		mean:    [0.0, 0.0, -2.0]!
		quat:    Quaternion{
			w: 1.0
			x: 0.0
			y: 0.0
			z: 0.0
		}
		scale:   [0.6, 0.6, 0.6]!
		opacity: 1.0
		color:   color_hex(0x00FF00)
	}
	g.splats << Gaussian{
		mean:    [0.0, 0.8, 1.5]!
		quat:    Quaternion{
			w: 1.0
			x: 0.0
			y: 0.0
			z: 0.0
		}
		scale:   [0.2, 0.2, 0.2]!
		opacity: 0.9
		color:   color_hex(0x00FF00)
	}
	sc.add_mesh(mesh(MeshParams{
		geometry:       splats_geometry(g)
		material:       basic_material(color_hex(0x00FF00), 1.0)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	mut r := renderer(w, h, 1, 3)
	img := render_scene_with_splats(sc, mut r, cam)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/splat_mixed_scene.png', img)
	data := img.data_f32()
	// sphere hides the splat behind it: centre pixel stays red-dominant
	ci := (64 * w + 64) * 4
	assert data[ci] > data[ci + 1] + 20.0
	// the front splat covers whatever is behind it: green-dominant there
	fcol, frow := project_point(cam, [0.0, 0.8, 1.5]!, w, h)
	fi := (int(frow) * w + int(fcol)) * 4
	assert data[fi + 1] > data[fi] + 20.0
	assert data[fi + 1] > data[fi + 2] + 20.0
	// the cyclide behind the sphere renders too (mixed opaque scene works)
	mut red := 0
	for i := 0; i < data.len; i += 4 {
		if data[i] > data[i + 1] + 20.0 && data[i] > data[i + 2] + 20.0 {
			red++
		}
	}
	assert red > 200
}

fn test_splat_mesh_pose_moves_splats() {
	w := 128
	h := 128
	cam := scene_test_camera(w, h)
	mut sc := scene(color_rgb(0.0, 0.0, 0.0))
	// local splat at (0.2, 0, 0); mesh pose: translate (0.3, 0.2, 0) then
	// rotate 90° about z -> world (0.3 - 0, 0.2 + 0.2, 0) = (0.3, 0.4, 0)
	// (Object3D motor = T . R: rotation first, then translation)
	sc.add_mesh(mesh(MeshParams{
		geometry:       one_splat_cloud([0.2, 0.0, 0.0]!, 0.08, 1.0)
		material:       basic_material(color_hex(0x00FF00), 1.0)
		position:       [0.3, 0.2, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: math.pi / 2.0
		motor:          none
	}))
	mut r := renderer(w, h, 1, 3)
	img := render_scene_with_splats(sc, mut r, cam)
	data := img.data_f32()
	mut best := f32(-1.0)
	mut best_col := 0
	mut best_row := 0
	for row in 0 .. h {
		for col in 0 .. w {
			idx := (row * w + col) * 4
			b := data[idx] + data[idx + 1] + data[idx + 2]
			if b > best {
				best = b
				best_col = col
				best_row = row
			}
		}
	}
	assert best > 200.0
	ex_col, ex_row := project_point(cam, [0.3, 0.4, 0.0]!, w, h)
	dcol := f64(best_col) - ex_col
	drow := f64(best_row) - ex_row
	assert dcol > -2.0 && dcol < 2.0
	assert drow > -2.0 && drow < 2.0
}

fn test_plain_renderer_ignores_splat_meshes() {
	w := 128
	h := 128
	cam := scene_test_camera(w, h)
	mut sc := scene(color_rgb(0.0, 0.0, 0.0))
	for m in mixed_opaque_meshes() {
		sc.add_mesh(m)
	}
	for l in mixed_lights() {
		sc.add_light(l)
	}
	// a big green splat right in front of the sphere: the plain ray tracer
	// must NOT show it (splats render only via render_scene_with_splats)
	sc.add_mesh(mesh(MeshParams{
		geometry:       one_splat_cloud([0.0, 0.0, 2.0]!, 0.8, 1.0)
		material:       basic_material(color_hex(0x00FF00), 1.0)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	mut r := renderer(w, h, 1, 3)
	img := r.render(sc, cam) // must not crash
	data := img.data_f32()
	// sphere centre still red, and NO green-dominant pixel anywhere
	ci := (64 * w + 64) * 4
	assert data[ci] > data[ci + 1] + 20.0
	for i := 0; i < data.len; i += 4 {
		assert !(data[i + 1] > data[i] + 20.0 && data[i + 1] > data[i + 2] + 20.0)
	}
}

fn test_splat_mesh_rotation_turns_ellipse() {
	// anisotropic splat with its long axis along local e1, under a 90° z
	// rotation in the Object3D pose: the on-screen ellipse must be VERTICAL
	// (catches a transposed mat3_mul(ro, rw) in transform_gaussians)
	w := 128
	h := 128
	cam := scene_test_camera(w, h)
	mut sc := scene(color_rgb(0.0, 0.0, 0.0))
	sc.add_mesh(mesh(MeshParams{
		geometry:       splats_geometry(Gaussians{
			splats: [
				Gaussian{
					mean:    [0.0, 0.0, 0.0]!
					quat:    Quaternion{
						w: 1.0
						x: 0.0
						y: 0.0
						z: 0.0
					}
					scale:   [0.3, 0.05, 0.05]!
					opacity: 1.0
					color:   color_hex(0xFFFFFF)
				},
			]
		})
		material:       basic_material(color_hex(0xFFFFFF), 1.0)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: math.pi / 2.0
		motor:          none
	}))
	mut r := renderer(w, h, 1, 3)
	img := render_scene_with_splats(sc, mut r, cam)
	c0, c1, r0, r1 := bright_bbox(img.data_f32(), w, h, 100.0)
	col_span := c1 - c0 + 1
	row_span := r1 - r0 + 1
	assert row_span > 40
	assert col_span <= 15
	assert row_span > 3 * col_span
}
