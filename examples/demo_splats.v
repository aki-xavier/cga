module main

// Gaussian-splat attachment demo (Route B + Extensions 1-2): a Dupin cyclide
// and an ellipsoid, each wearing a dense layer of flattened Gaussians sampled
// on its surface via the general sampler (sample_gaussians_on_surface) and
// carried as SplatsGeometry meshes in the scene.  render_scene_with_splats
// ray-traces the opaque objects and splats the clouds with correct occlusion,
// orbited into an animated GIF (examples/artifacts/splats.gif).
// Run: v -gc boehm run examples/demo_splats.v
import cga
import os
import math

const splat_mat = cga.standard_material(cga.MaterialParams{
	color:      cga.color_hex(0xECF0F1)
	roughness:  0.4
	metalness:  0.0
	emissive:   cga.color_hex(0x000000)
	opacity:    0.6
	ior:        1.5
	absorption: 0.0
})

fn opaque_mesh(geom cga.Geometry, pos [3]f64) cga.Mesh {
	return cga.mesh(cga.MeshParams{
		geometry:       geom
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.1
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       pos
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	})
}

// splat_mesh samples ~n splats on geom (local space) and wraps them in a
// SplatsGeometry mesh sharing the opaque mesh's pose.
fn splat_mesh(geom cga.Geometry, n int, st f64, sn f64, pos [3]f64) cga.Mesh {
	return cga.mesh(cga.MeshParams{
		geometry:       cga.splats_geometry(cga.sample_gaussians_on_surface(geom, n, st, sn,
			splat_mat))
		material:       cga.basic_material(cga.color_hex(0xECF0F1), 0.6)
		position:       pos
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	})
}

fn build_scene() cga.Scene {
	mut scene := cga.scene(none)
	// ground plane
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.plane_geometry([0.0, 1.0, 0.0]!, -1.2)
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
	// ring cyclide + splat layer
	cyc := cga.cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!)
	scene.add_mesh(opaque_mesh(cyc, [0.0, 0.0, 0.0]!))
	scene.add_mesh(splat_mesh(cyc, 1152, 0.06, 0.015, [0.0, 0.0, 0.0]!))
	// blue ellipsoid + splat layer
	ell := cga.ellipsoid_geometry(0.9, 0.6, 0.7)
	ell_pos := [2.3, 0.1, 0.4]!
	mut em := opaque_mesh(ell, ell_pos)
	em.material = cga.standard_material(cga.MaterialParams{
		color:      cga.color_hex(0x2980B9)
		roughness:  0.25
		metalness:  0.25
		emissive:   cga.color_hex(0x000000)
		opacity:    1.0
		ior:        1.5
		absorption: 0.0
	})
	scene.add_mesh(em)
	scene.add_mesh(splat_mesh(ell, 500, 0.07, 0.02, ell_pos))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.38, [0.4, 1.0, 0.35]!))
	scene.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.7, [0.0, 4.0, 3.5]!))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.18, [0.0, 0.35, 0.9]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.34))
	return scene
}

fn main() {
	frames := 24
	out_dir := 'examples/artifacts'
	os.mkdir_all(out_dir) or {}

	scene := build_scene()
	mut camera := cga.perspective_camera(50.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 2.4, 6.2]!, [
		0.0,
		0.0,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	camera.look_at([0.0, 0.0, 0.0]!, none)
	controls := cga.orbit_controls([0.0, 0.0, 0.0]!, 0.0, 0.35, 6.0)
	mut renderer := cga.renderer(240, 180, 1, 3)

	mut gif_frames := [][]u8{}
	for i in 0 .. frames {
		mut ctrl := controls
		ctrl.azimuth = 2.0 * math.pi * f64(i) / f64(frames)
		ctrl.elevation = 0.35 + 0.1 * math.sin(4.0 * math.pi * f64(i) / f64(frames))
		ctrl.update(mut camera)
		img := cga.render_scene_with_splats(scene, mut renderer, camera)
		gif_frames << cga.f32_rgba_to_u8(img.data_f32())
		img.free()
		println('frame ${i + 1}/${frames} rendered')
	}
	cga.save_gif('${out_dir}/splats.gif', gif_frames, 240, 180, 3)
	println('saved ${out_dir}/splats.gif')
}
