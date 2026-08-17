module main

// Orbit demo: three.js-style scene + orbit animation -> PNG frames.
// Run: v run examples/demo_engine.v [frames] [outdir]

import cga
import os
import math

fn build_scene() cga.Scene {
	mut scene := cga.scene(none)
	// ground plane
	scene.add_mesh(cga.mesh(cga.plane_geometry([0.0, 1.0, 0.0]!, 0.0), cga.standard_material(cga.color_hex(0xB0B0B0),
		0.7, 0.0, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 0.0, 0.0]!,
		[0.0, 0.0, 1.0]!, 0.0, none))
	// red sphere
	scene.add_mesh(cga.mesh(cga.sphere_geometry(1.0), cga.standard_material(cga.color_hex(0xC0392B),
		0.25, 0.25, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 1.0, 0.0]!,
		[0.0, 0.0, 1.0]!, 0.0, none))
	// blue sphere
	scene.add_mesh(cga.mesh(cga.sphere_geometry(0.6), cga.standard_material(cga.color_hex(0x2980B9),
		0.15, 0.35, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [-2.2, 0.6, 0.5]!,
		[0.0, 0.0, 1.0]!, 0.0, none))
	// gold cylinder (infinite)
	scene.add_mesh(cga.mesh(cga.cylinder_geometry(0.7, -1.0), cga.standard_material(cga.color_hex(0xD4AC0D),
		0.4, 0.3, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [2.2, 0.7, -0.5]!,
		[0.0, 0.0, 1.0]!, 0.0, none))
	// green box
	scene.add_mesh(cga.mesh(cga.box_geometry(0.9, 0.9, 0.9), cga.standard_material(cga.color_hex(0x27AE60),
		0.6, 0.0, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.8, 0.45, 1.8]!,
		[0.0, 0.0, 1.0]!, 0.0, none))
	// purple disc (tilted)
	scene.add_mesh(cga.mesh(cga.circle_geometry(0.9), cga.standard_material(cga.color_hex(0x8E44AD),
		0.3, 0.0, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [-2.4, 2.2, 0.8]!,
		[1.0, 0.0, 0.0]!, -0.4, none))
	// refractive glass sphere
	scene.add_mesh(cga.mesh(cga.sphere_geometry(0.8), cga.standard_material(cga.color_hex(0xAAD4FF),
		0.05, 0.0, cga.color_hex(0x000000), 0.08, 1.5, 0.2), [0.4, 1.5, 2.6]!,
		[0.0, 0.0, 1.0]!, 0.0, none))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.38, [0.4, 1.0,
		0.35]!))
	scene.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.7, [0.0, 4.0, 3.5]!))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.18, [0.0, 0.35,
		0.9]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.34))
	return scene
}

fn main() {
	frames := 90
	out_dir := 'artifacts'
	os.mkdir_all(out_dir) or {}

	scene := build_scene()
	mut camera := cga.perspective_camera(50.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 2.4,
		6.2]!, [0.0, 0.8, 0.0]!, [0.0, 1.0, 0.0]!)
	camera.look_at([0.0, 0.8, 0.0]!, none)
	controls := cga.orbit_controls([0.0, 0.8, 0.0]!, 0.0, 0.42, 6.6)
	mut renderer := cga.renderer(360, 270, 2, 3)

	for i in 0 .. frames {
		mut ctrl := controls
		ctrl.azimuth = 2.0 * math.pi * f64(i) / f64(frames)
		ctrl.elevation = 0.42 + 0.12 * math.sin(4.0 * math.pi * f64(i) / f64(frames))
		ctrl.update(mut camera)
		img := renderer.render(scene, camera)
		p := '${out_dir}/frame_${i:03d}.png'
		cga.save_frame_png(p, img)
		println('frame ${i + 1}/${frames} saved ${p}')
	}
}
