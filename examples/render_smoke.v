module main

import cga
import os

fn main() {
	mut scene := cga.scene(none)
	// ground plane (y = 0)
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.plane_geometry([0.0, 1.0, 0.0]!, 0.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xB0B0B0)
			roughness:  0.7
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	// red sphere
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.sphere_geometry(1.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC0392B)
			roughness:  0.25
			metalness:  0.25
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 1.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	// gold cylinder (finite)
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.cylinder_geometry(0.25, 2.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xD4AF37)
			roughness:  0.3
			metalness:  0.6
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [1.5, 0.5, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	// green box
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.box_geometry(0.9, 0.9, 0.9)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x27AE60)
			roughness:  0.6
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.8, 0.45, 1.8]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	// purple circle (disc)
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.circle_geometry(0.7)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x8E44AD)
			roughness:  0.5
			metalness:  0.1
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [-1.2, 0.7, 1.2]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.38, [0.4, 1.0, 0.35]!))
	scene.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.7, [0.0, 4.0, 3.5]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.34))

	mut camera := cga.perspective_camera(50.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 2.4, 6.2]!, [
		0.0,
		0.8,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	camera.look_at([0.0, 0.8, 0.0]!, none)

	img := cga.render_frame(scene, camera, 320, 240, 1)
	data := img.data_f32()
	defer {
		img.free()
	}
	println('frame shape: ${img.shape()}  pixels: ${data.len / 4}')

	// write PPM (simple format for visual check)
	mut lines := []string{}
	lines << 'P3\n320 240\n255'
	mut idx := 0
	for _ in 0 .. 240 {
		mut row := []string{}
		for _ in 0 .. 320 {
			r := int(data[idx] + 0.5)
			g := int(data[idx + 1] + 0.5)
			b := int(data[idx + 2] + 0.5)
			row << '${r} ${g} ${b}'
			idx += 4
		}
		lines << row.join('  ')
	}
	os.write_file('examples/artifacts/render_smoke.ppm', lines.join('\n')) or {
		panic('write failed')
	}
	println('wrote examples/artifacts/render_smoke.ppm')

	// sample a few pixels to confirm non-background content
	center := 120 * 320 * 4 + 160 * 4
	println('center pixel rgba = ${data[center]}, ${data[center + 1]}, ${data[center + 2]}, ${data[
		center + 3]}')
}
