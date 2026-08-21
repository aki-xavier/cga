module main

// CSG boolean demo: difference / intersection / union side by side.
// Run: v run examples/demo_csg.v
import cga
import os

fn main() {
	mut scene := cga.scene(none)

	// ground plane
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.plane_geometry([0.0, 1.0, 0.0]!, 0.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x888888)
			roughness:  0.8
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

	// difference: box with a spherical hole (box half 0.8, sphere 0.55)
	diff := cga.csg_geometry(.difference, [cga.box_geometry(1.6, 1.6, 1.6),
		cga.sphere_geometry(0.55)])
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       diff
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.1
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [-2.2, 0.8, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))

	// intersection: box clipped by a larger sphere -> rounded box
	inter := cga.csg_geometry(.intersection, [cga.box_geometry(1.6, 1.6, 1.6),
		cga.sphere_geometry(1.0)])
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       inter
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x2980B9)
			roughness:  0.25
			metalness:  0.2
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.8, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))

	// union: box + sphere
	un := cga.csg_geometry(.union, [cga.box_geometry(1.4, 1.4, 1.4),
		cga.sphere_geometry(0.9)])
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       un
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x27AE60)
			roughness:  0.35
			metalness:  0.05
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [2.2, 0.8, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))

	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.7, [0.5, 1.0, 0.4]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.35))

	mut camera := cga.perspective_camera(45.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 2.4, 7.0]!, [
		0.0,
		0.7,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	camera.look_at([0.0, 0.7, 0.0]!, none)
	mut renderer := cga.renderer(480, 360, 2, 3)
	img := renderer.render(scene, camera)

	out := os.dir(@FILE)

	cga.save_frame_png('${out}/demo_csg.png', img)
}
