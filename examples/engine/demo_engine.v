module main

// Orbit demo: three.js-style scene + orbit animation -> animated GIF
// (examples/artifacts/orbit.gif).  Run: v -gc boehm run examples/demo_engine.v
import cga
import os
import math

fn build_scene() cga.Scene {
	mut scene := cga.scene(none)
	// ground plane
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
	// blue sphere
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.sphere_geometry(0.6)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x2980B9)
			roughness:  0.15
			metalness:  0.35
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [-2.2, 0.6, 0.5]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	// gold cylinder (infinite)
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.cylinder_geometry(0.7, -1.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xD4AC0D)
			roughness:  0.4
			metalness:  0.3
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [2.2, 0.7, -0.5]!
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
	// purple disc (tilted)
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.circle_geometry(0.9)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x8E44AD)
			roughness:  0.3
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [-2.4, 2.2, 0.8]!
		rotation_axis:  [1.0, 0.0, 0.0]!
		rotation_angle: -0.4
		motor:          none
	}))
	// refractive glass sphere
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.sphere_geometry(0.8)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xAAD4FF)
			roughness:  0.05
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    0.08
			ior:        1.5
			absorption: 0.2
		})
		position:       [0.4, 1.5, 2.6]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.38, [0.4, 1.0, 0.35]!))
	scene.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.7, [0.0, 4.0, 3.5]!))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.18, [0.0, 0.35, 0.9]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.34))
	return scene
}

fn main() {
	frames := 90
	out_dir := 'examples/artifacts'
	os.mkdir_all(out_dir) or {}

	scene := build_scene()
	mut camera := cga.perspective_camera(50.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 2.4, 6.2]!, [
		0.0,
		0.8,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	camera.look_at([0.0, 0.8, 0.0]!, none)
	controls := cga.orbit_controls([0.0, 0.8, 0.0]!, 0.0, 0.42, 6.6)
	mut renderer := cga.renderer(360, 270, 2, 3)

	mut gif_frames := [][]u8{}
	for i in 0 .. frames {
		mut ctrl := controls
		ctrl.azimuth = 2.0 * math.pi * f64(i) / f64(frames)
		ctrl.elevation = 0.42 + 0.12 * math.sin(4.0 * math.pi * f64(i) / f64(frames))
		ctrl.update(mut camera)
		img := renderer.render(scene, camera)
		gif_frames << cga.f32_rgba_to_u8(img.data_f32())
		img.free()
		println('frame ${i + 1}/${frames} rendered')
	}
	cga.save_gif('${out_dir}/orbit.gif', gif_frames, 360, 270, 3)
	println('saved ${out_dir}/orbit.gif')
}
