module main

// CSG boolean demo: difference / intersection / union side by side.
// Run: v run examples/demo_csg.v

import cga
import os

fn main() {
	mut scene := cga.scene(none)

	// ground plane
	scene.add_mesh(cga.mesh(cga.plane_geometry([0.0, 1.0, 0.0]!, 0.0), cga.standard_material(cga.color_hex(0x888888),
		0.8, 0.0, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 0.0, 0.0]!,
		[0.0, 0.0, 1.0]!, 0.0, none))

	// difference: box with a spherical hole (box half 0.8, sphere 0.55)
	diff := cga.csg_geometry('difference', [cga.box_geometry(1.6, 1.6, 1.6),
		cga.sphere_geometry(0.55)])
	scene.add_mesh(cga.mesh(diff, cga.standard_material(cga.color_hex(0xC0392B), 0.3,
		0.1, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [-2.2, 0.8, 0.0]!, [0.0, 0.0,
		1.0]!, 0.0, none))

	// intersection: box clipped by a larger sphere -> rounded box
	inter := cga.csg_geometry('intersection', [cga.box_geometry(1.6, 1.6, 1.6),
		cga.sphere_geometry(1.0)])
	scene.add_mesh(cga.mesh(inter, cga.standard_material(cga.color_hex(0x2980B9),
		0.25, 0.2, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 0.8, 0.0]!, [0.0,
		0.0, 1.0]!, 0.0, none))

	// union: box + sphere
	un := cga.csg_geometry('union', [cga.box_geometry(1.4, 1.4, 1.4), cga.sphere_geometry(0.9)])
	scene.add_mesh(cga.mesh(un, cga.standard_material(cga.color_hex(0x27AE60), 0.35,
		0.05, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [2.2, 0.8, 0.0]!, [0.0, 0.0,
		1.0]!, 0.0, none))

	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.7, [0.5, 1.0,
		0.4]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.35))

	mut camera := cga.perspective_camera(45.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 2.4,
		7.0]!, [0.0, 0.7, 0.0]!, [0.0, 1.0, 0.0]!)
	camera.look_at([0.0, 0.7, 0.0]!, none)
	mut renderer := cga.renderer(480, 360, 2, 3)
	img := renderer.render(scene, camera)

	os.mkdir_all('examples/artifacts') or {}
	cga.save_frame_png('examples/artifacts/demo_csg.png', img)
	println('saved examples/artifacts/demo_csg.png')
}
