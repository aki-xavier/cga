module cga

import mlx
import os

fn test_png_decode_and_sample() {
	rgba, w, h := load_png_rgba('examples/assets/brick.png')
	assert w == 256 && h == 256
	assert rgba.len == 256 * 256 * 4
	tex := texture_load('examples/assets/brick.png')
	assert tex.width == 256 && tex.height == 256
	uv := mlx.array_f32([f32(0.5), 0.5], [1, 2])
	s := tex.sample(uv, .repeat, .repeat)
	data := s.data_f32()
	assert data.len == 4
	assert data[3] == 1.0 // opaque alpha
}

fn test_textured_render() {
	tex := texture_load('examples/assets/brick.png')
	mut sc := scene(none)
	mut mat := standard_material(MaterialParams{
		color:      color_hex(0xFFFFFF)
		roughness:  0.5
		metalness:  0.0
		emissive:   color_hex(0x000000)
		opacity:    1.0
		ior:        1.5
		absorption: 0.0
	})
	mat.map = tex
	sc.add_mesh(mesh(MeshParams{ geometry: box_geometry(1.0, 1.0, 1.0), material: mat, position: [
		0.0,
		0.0,
		0.0,
	]!, rotation_axis: [0.0, 0.0, 1.0]!, rotation_angle: 0.0, motor: none }))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.2))
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	img := render_frame(sc, cam, 80, 80, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/textured_box.png', img)
	data := img.data_f32()
	// centre should be brick-ish (red channel notably above the sky-blue 135)
	idx := 40 * 80 * 4 + 40 * 4
	assert data[idx] > data[idx + 2]
}
