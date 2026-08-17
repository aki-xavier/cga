module main

// GLB mesh round-trip demo: extrude an L-shape, save it as .glb (with a node
// transform), reload it, and render the loaded mesh.
// Run: v run examples/demo_gltf.v

import cga
import os

fn main() {
	os.mkdir_all('artifacts') or {}

	// L-shaped extrusion
	verts, faces := cga.extrude([[0.0, 0.0]!, [2.0, 0.0]!, [2.0, 1.0]!, [1.0, 1.0]!,
		[1.0, 2.0]!, [0.0, 2.0]!], 0.8)
	// row-major transform: translate y by 0.6
	t := [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.6, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		0.0, 1.0]!
	cga.save_glb('artifacts/demo_gltf.glb', [cga.GltfMeshIn{
		vertices: verts
		faces: faces
		transform: t
		color: [0.9, 0.45, 0.13]!
	}])

	// reload and render the loaded mesh(es) (world transform baked in)
	loaded := cga.load_gltf('artifacts/demo_gltf.glb')
	mut scene := cga.scene(none)
	scene.add_mesh(cga.mesh(cga.gltf_to_geometry(loaded), cga.standard_material(cga.color_hex(0xE67E22),
		0.3, 0.15, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 0.0, 0.0]!, [0.0,
		0.0, 1.0]!, 0.0, none))
	scene.add_mesh(cga.mesh(cga.plane_geometry([0.0, 1.0, 0.0]!, 0.0), cga.standard_material(cga.color_hex(0x888888),
		0.8, 0.0, cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 0.0, 0.0]!, [0.0,
		0.0, 1.0]!, 0.0, none))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.7, [0.4, 1.0,
		0.4]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.35))

	mut camera := cga.perspective_camera(40.0, 4.0 / 3.0, 0.1, 100.0, [3.0, 2.6,
		3.6]!, [1.0, 1.0, 0.4]!, [0.0, 1.0, 0.0]!)
	camera.look_at([1.0, 1.0, 0.4]!, none)
	mut renderer := cga.renderer(480, 360, 2, 3)
	img := renderer.render(scene, camera)

	cga.save_frame_png('artifacts/demo_gltf.png', img)
	println('saved artifacts/demo_gltf.png')
}
