module main

// DamagedHelmet demo: render the Khronos GLTF helmet as a solid-colour material
// (its base-color factor; no textures) alongside CGA analytic primitives (sphere
// / cylinder / ground plane).  The hybrid renderer rasterizes the triangle-mesh
// helmet and ray-traces the analytic primitives, then composites by depth.
// Run: v -gc boehm run examples/helmet/demo_helmet.v
import cga
import os

fn main() {
	out := os.dir(@FILE)
	loaded := cga.load_gltf('${out}/DamagedHelmet.glb')!

	mut scene := cga.scene(none)
	// helmet: triangle mesh (rasterized); rendered as a solid grey
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry: cga.gltf_to_geometry(loaded)
		material: cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x9E9E9E)
			roughness:  0.6
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
	// CGA analytic primitives (ray traced): a red sphere and a gold cylinder
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry: cga.sphere_geometry(0.6)
		material: cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.1
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [2.2, 0.6, 0.6]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry: cga.cylinder_geometry(0.4, 1.5)
		material: cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xD4AF37)
			roughness:  0.4
			metalness:  0.4
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [-2.2, 0.75, 0.5]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	// ground plane below everything (analytic -> ray traced)
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry: cga.plane_geometry([0.0, 1.0, 0.0]!, -0.95)
		material: cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x8A8A8A)
			roughness:  0.85
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

	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.8, [0.5, 1.0,
		0.6]!))
	scene.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.5, [-2.2, 2.4, 2.2]!))
	scene.add_light(cga.point_light(cga.color_hex(0xFFE0B0), 0.3, [2.2, 1.2, -1.0]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.22))

	mut camera := cga.perspective_camera(45.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 2.6,
		6.6]!, [0.0, 0.3, -0.1]!, [0.0, 1.0, 0.0]!)
	camera.look_at([0.0, 0.3, -0.1]!, none)
	mut renderer := cga.renderer(480, 360, 1, 3)
	img := renderer.render(scene, camera)

	cga.save_frame_png('${out}/demo_helmet.png', img)
	println('saved ${out}/demo_helmet.png')
}
