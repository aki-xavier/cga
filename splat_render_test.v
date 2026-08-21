module cga

import math
import os

// (the pinhole projection helper is project_point in splat_render.v)

fn single_splat(pos [3]f64, scale f64, c Color, opacity f64) Gaussians {
	return Gaussians{
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
				color:   c
			},
		]
	}
}

fn test_splat_projection_lands_on_expected_pixel() {
	w := 128
	h := 128
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	pos := [0.3, 0.2, 0.0]!
	g := single_splat(pos, 0.08, color_hex(0xFFFFFF), 1.0)
	img := render_splats(g, cam, w, h, color_rgb(0.0, 0.0, 0.0))
	data := img.data_f32()
	// argmax of brightness
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
	assert best > 200.0 // the splat peak is bright on a black background
	ex_col, ex_row := project_point(cam, pos, w, h)
	assert math.abs(f64(best_col) - ex_col) <= 2.0
	assert math.abs(f64(best_row) - ex_row) <= 2.0
}

fn splat_occlusion_scene() Scene {
	mut sc := scene(color_rgb(0.0, 0.0, 0.0))
	sc.add_mesh(mesh(MeshParams{
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
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	return sc
}

fn test_splat_occlusion_by_opaque_sphere() {
	w := 128
	h := 128
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	// green splat fully behind the sphere (must be hidden), green splat in
	// front of the sphere silhouette (must be visible)
	g := Gaussians{
		splats: [
			Gaussian{
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
			},
			Gaussian{
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
			},
		]
	}
	mut r := renderer(w, h, 1, 3)
	img := render_splats_over(g, splat_occlusion_scene(), mut r, cam)
	data := img.data_f32()
	// centre pixel: the sphere, NOT the hidden splat (red-dominant)
	ci := (64 * w + 64) * 4
	assert data[ci] > data[ci + 1] + 20.0
	// the front splat's pixel: green-dominant
	fcol, frow := project_point(cam, [0.0, 0.8, 1.5]!, w, h)
	fi := (int(frow) * w + int(fcol)) * 4
	assert data[fi + 1] > data[fi] + 20.0
	assert data[fi + 1] > data[fi + 2] + 20.0
	// sanity: the hidden splat alone WOULD cover the centre pixel
	only := render_splats(Gaussians{
		splats: [g.splats[0]]
	}, cam, w, h, color_rgb(0.0, 0.0, 0.0))
	od := only.data_f32()
	assert od[ci + 1] > 100.0
}

fn test_render_splats_dense_cyclide() {
	cyc := dupin_cyclide(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!)
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
	// 48x24 = 1152 attached splats
	g := sample_gaussians_on_cyclide(cyc, 48, 24, 0.06, 0.015, standard_material(MaterialParams{
		color:      color_hex(0xECF0F1)
		roughness:  0.4
		metalness:  0.0
		emissive:   color_hex(0x000000)
		opacity:    0.6
		ior:        1.5
		absorption: 0.0
	}))
	assert g.splats.len == 1152
	mut cam := perspective_camera(40.0, 160.0 / 120.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [
		0.0,
		0.0,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	mut r := renderer(160, 120, 1, 3)
	img := render_splats_over(g, sc, mut r, cam)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/splat_render_cyclide.png', img)
	// red cyclide pixels and pale splat pixels must both be present; the
	// far-side splats are occluded so the ring hole stays sky-blue
	data := img.data_f32()
	mut red := 0
	mut pale := 0
	mut sky_hole := 0
	for i := 0; i < data.len; i += 4 {
		rr := data[i]
		gg := data[i + 1]
		bb := data[i + 2]
		if rr > gg + 20.0 && rr > bb + 20.0 {
			red++
		}
		if rr > 180.0 && gg > 180.0 && bb > 180.0 {
			pale++
		}
	}
	// ring hole ~ centre: a small disc around the middle should keep the sky
	cx := 80
	cy := 60
	for dy in -4 .. 5 {
		for dx in -4 .. 5 {
			idx := ((cy + dy) * 160 + (cx + dx)) * 4
			if data[idx + 2] > data[idx] && data[idx + 2] > 120.0 {
				sky_hole++
			}
		}
	}
	assert red > 100
	assert pale > 100
	assert sky_hole > 20
}
