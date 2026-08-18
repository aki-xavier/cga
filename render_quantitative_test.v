module cga

import mlx
import math

// Render pipeline quantitative checks: sRGB roundtrip, ior=1 invisibility,
// Beer absorption, shadow umbra.  Expected values derive from first principles.

fn rq_linear_to_srgb255(lum f64) int {
	mut l := lum
	if l < 0.0 {
		l = 0.0
	}
	if l > 1.0 {
		l = 1.0
	}
	s := if l <= 0.0031308 { 12.92 * l } else { 1.055 * math.pow(l, 1.0 / 2.4) - 0.055 }
	v := int(s * 255.0 + 0.5)
	if v < 0 {
		return 0
	}
	if v > 255 {
		return 255
	}
	return v
}

fn rq_wall_scene() Scene {
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       plane_geometry([0.0, 0.0, -1.0]!, -4.0)
		material:       basic_material(color_hex(0xCC3333), 1.0)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	return sc
}

fn rq_head_on_cam() PerspectiveCamera {
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 0.0]!, [0.0, 0.0, 1.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 1.0]!, none)
	return cam
}

fn rq_px(img mlx.Array, w int, row int, col int) [3]f32 {
	d := img.data_f32()
	idx := (row * w + col) * 4
	return [d[idx], d[idx + 1], d[idx + 2]]!
}

fn test_render_srgb_roundtrip() {
	mut r := renderer(64, 64, 1, 3)
	img := r.render(rq_wall_scene(), rq_head_on_cam())
	p := rq_px(img, 64, 32, 32)
	assert int(p[0]) == 204
	assert int(p[1]) == 51
	assert int(p[2]) == 51
}

fn test_render_ior1_invisible() {
	mut sc := rq_wall_scene()
	// ior=1 & opacity=0 & absorption=0 -> exactly invisible (F=0, no bending)
	sc.add_mesh(mesh(MeshParams{
		geometry:       sphere_geometry(0.8)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xAAD4FF)
			roughness:  0.5
			metalness:  0.0
			emissive:   color_hex(0x000000)
			opacity:    0.0
			ior:        1.0
			absorption: 0.0
		})
		position:       [0.0, 0.0, 2.2]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	mut r := renderer(64, 64, 1, 3)
	cam := rq_head_on_cam()
	a := rq_px(r.render(sc, cam), 64, 32, 32)
	b := rq_px(r.render(rq_wall_scene(), cam), 64, 32, 32)
	for i in 0 .. 3 {
		assert math.abs(f64(a[i]) - f64(b[i])) <= 1.0
	}
}

fn rq_slab(depth f64, absorption f64) [3]f32 {
	mut sc := rq_wall_scene()
	sc.add_mesh(mesh(MeshParams{
		geometry:       box_geometry(3.0, 3.0, depth)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xFFFFFF)
			roughness:  0.5
			metalness:  0.0
			emissive:   color_hex(0x000000)
			opacity:    0.0
			ior:        1.0
			absorption: absorption
		})
		position:       [0.0, 0.0, 2.5]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	mut r := renderer(64, 64, 1, 3)
	return rq_px(r.render(sc, rq_head_on_cam()), 64, 32, 32)
}

fn test_render_beer_absorption() {
	wall := [204.0 / 255.0, 51.0 / 255.0, 51.0 / 255.0]!
	for depth in [0.2, 1.0] {
		px := rq_slab(depth, 0.8)
		trans := math.exp(-0.8 * depth)
		for i in 0 .. 3 {
			want := rq_linear_to_srgb255(trans * srgb_to_linear(wall[i]))
			assert math.abs(f64(px[i]) - f64(want)) <= 2.0
		}
	}
	// sigma=0 -> no attenuation (exact wall colour)
	px0 := rq_slab(1.0, 0.0)
	assert int(px0[0]) == 204 && int(px0[1]) == 51 && int(px0[2]) == 51
}

fn rq_shadow_scene(opacity ?f64) Scene {
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       plane_geometry([0.0, 1.0, 0.0]!, 0.0)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xFFFFFF)
			roughness:  1.0
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
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.0, 1.0, 0.0]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.2))
	if o := opacity {
		sc.add_mesh(mesh(MeshParams{
			geometry:       sphere_geometry(0.5)
			material:       standard_material(MaterialParams{
				color:      color_hex(0xFFFFFF)
				roughness:  1.0
				metalness:  0.0
				emissive:   color_hex(0x000000)
				opacity:    o
				ior:        1.0
				absorption: 0.0
			})
			position:       [2.0, 1.5, 3.0]!
			rotation_axis:  [0.0, 0.0, 1.0]!
			rotation_angle: 0.0
			motor:          none
		}))
	}
	return sc
}

fn rq_shadow_cam() PerspectiveCamera {
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.8, -1.0]!, [2.0, 0.0, 3.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([2.0, 0.0, 3.0]!, none)
	return cam
}

fn test_render_shadow_umbra_is_ambient_only() {
	// umbra = ambient only: white ground dec=1.0, L = 0.2
	mut r := renderer(96, 96, 1, 3)
	img := r.render(rq_shadow_scene(1.0), rq_shadow_cam())
	p := rq_px(img, 96, 48, 48)
	want := rq_linear_to_srgb255(0.2)
	assert math.abs(f64(p[0]) - f64(want)) <= 2.0
}
