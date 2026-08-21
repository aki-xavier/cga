module main

// Displaced-surface demo: a star-profiled lofted vase (trimesh) is BAKED onto
// an infinite-cylinder base as a residual grid, then the displaced cylinder
// renders through the analytic ray tracer (bracket + march, no ray marching
// of free space).  Saves a side-by-side comparison:
//   examples/artifacts/displace_target.png    (the lofted trimesh)
//   examples/artifacts/displace_displaced.png (cylinder + baked residual)
//   examples/artifacts/displace_diff.png      (4x amplified difference)
// Run: v -gc boehm run examples/demo_displace.v
import cga
import mlx
import os
import math

// star_vase builds a lofted vase with a 5-point star cross-section around the
// z axis (local frame of the cylinder base: residual band z in [0, 2r]).
// The outer star radius equals the base radius at both ends, so the baked
// residual blends smoothly into the zero rows at the band edges (no ledge).
fn star_vase() cga.TrimeshGeometry {
	mut profiles := [][][2]f64{}
	mut zs := []f64{}
	for j in 0 .. 6 {
		z := 1.6 * f64(j) / 5.0 // span the full residual band
		outer := 0.8 * (1.0 + 0.22 * math.sin(math.pi * f64(j) / 5.0))
		mut prof := [][2]f64{}
		for k in 0 .. 10 {
			ang := 2.0 * math.pi * f64(k) / 10.0
			r := if k % 2 == 0 { outer } else { 0.78 * outer }
			prof << [r * math.cos(ang), r * math.sin(ang)]!
		}
		profiles << prof
		zs << z
	}
	v, f := cga.loft(profiles, zs)
	return cga.trimesh_geometry(v, f)
}

fn vase_mesh(geo cga.Geometry, col int) cga.Mesh {
	return cga.mesh(cga.MeshParams{
		geometry:       geo
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(col)
			roughness:  0.3
			metalness:  0.1
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, -1.0, 0.0]! // band z in [0, 1.6] -> world y in [-1, 0.6]
		rotation_axis:  [1.0, 0.0, 0.0]!
		rotation_angle: -math.pi / 2.0 // cylinder axis z -> world y
		motor:          none
	})
}

fn build_scene(geo cga.Geometry, col int) cga.Scene {
	mut scene := cga.scene(none)
	scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.plane_geometry([0.0, 1.0, 0.0]!, -1.0)
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
	scene.add_mesh(vase_mesh(geo, col))
	scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.5, [0.4, 1.0, 0.35]!))
	scene.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.6, [0.0, 3.0, 3.0]!))
	scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.35))
	return scene
}

fn main() {
	out_dir := os.dir(@FILE)
	os.mkdir_all(out_dir) or {}

	target := star_vase()
	base := cga.cylinder_geometry(0.8, -1.0) // infinite cylinder
	println('baking residual (128x48 grid)...')
	baked := cga.bake_residual(base, target, 128, 48, 1.0)

	mut cam := cga.perspective_camera(45.0, 4.0 / 3.0, 0.1, 100.0, [0.0, 1.6, 4.6]!, [
		0.0,
		0.0,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	mut r := cga.renderer(320, 240, 2, 3)
	img_t := r.render(build_scene(target, 0x2980B9), cam)
	img_d := r.render(build_scene(baked, 0x2980B9), cam)
	cga.save_frame_png('${out_dir}/displace_target.png', img_t)
	cga.save_frame_png('${out_dir}/displace_displaced.png', img_d)

	// 4x amplified difference
	da := img_d.data_f32()
	db := img_t.data_f32()
	mut diff := []f32{len: da.len}
	mut mse := 0.0
	mut nch := 0
	for i := 0; i < da.len; i += 4 {
		for k in 0 .. 3 {
			d := f64(da[i + k]) - f64(db[i + k])
			mse += d * d
			nch++
			diff[i + k] = f32(math.min(255.0, 4.0 * math.abs(d)))
		}
		diff[i + 3] = 255.0
	}
	mse /= f64(nch)
	cga.save_frame_png('${out_dir}/displace_diff.png', mlx.array_f32(diff, [240, 320, 4]))
	println('bake PSNR vs trimesh: ${10.0 * math.log10(255.0 * 255.0 / mse):.2f} dB')
	println('saved ${out_dir}/displace_{target,displaced,diff}.png')
}
