module cga

import math
import os

// ring cyclide (c < d < a): a=1, b=0.98, d=0.3, c ~= 0.199
fn splat_test_cyclide() DupinCyclide {
	return dupin_cyclide(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!)
}

fn splat_test_material() Material {
	return standard_material(MaterialParams{
		color:      color_hex(0xECF0F1)
		roughness:  0.4
		metalness:  0.0
		emissive:   color_hex(0x000000)
		opacity:    0.5
		ior:        1.5
		absorption: 0.0
	})
}

fn test_splat_count_matches_grid() {
	cyc := splat_test_cyclide()
	g := sample_gaussians_on_cyclide(cyc, 8, 8, 0.1, 0.025, splat_test_material())
	assert g.splats.len == 64
}

fn test_splat_means_on_surface() {
	cyc := splat_test_cyclide()
	g := sample_gaussians_on_cyclide(cyc, 12, 9, 0.1, 0.025, splat_test_material())
	assert g.splats.len == 108
	for s in g.splats {
		f := cyc.implicit(s.mean[0], s.mean[1], s.mean[2])
		assert math.abs(f) < 1e-6
	}
}

fn test_splat_frames_aligned_to_normal() {
	cyc := splat_test_cyclide()
	g := sample_gaussians_on_cyclide(cyc, 8, 8, 0.1, 0.025, splat_test_material())
	for s in g.splats {
		n := cyc.normal(s.mean[0], s.mean[1], s.mean[2])
		r := rotor_from_quaternion(s.quat)
		// local e3 (thin axis) maps to the world surface normal
		z := vec3_unit(dir3(r.apply(e3())))
		assert vec3_dot(z, n) > 0.999
		// local e1/e2 are tangent: perpendicular to the normal
		x := vec3_unit(dir3(r.apply(e1())))
		y := vec3_unit(dir3(r.apply(e2())))
		assert math.abs(vec3_dot(x, n)) < 1e-9
		assert math.abs(vec3_dot(y, n)) < 1e-9
		// flattened along the normal axis
		assert s.scale[2] < s.scale[0]
		assert s.scale[0] == s.scale[1]
	}
}

fn test_splat_to_meshes_places_ellipsoids() {
	cyc := splat_test_cyclide()
	g := sample_gaussians_on_cyclide(cyc, 6, 5, 0.1, 0.025, splat_test_material())
	meshes := g.to_meshes()
	assert meshes.len == g.splats.len
	for i, m in meshes {
		s := g.splats[i]
		// position is recovered from the motor matrix (object3d), so compare
		// with a tolerance rather than bit-exact
		assert math.abs(m.position[0] - s.mean[0]) < 1e-12
		assert math.abs(m.position[1] - s.mean[1]) < 1e-12
		assert math.abs(m.position[2] - s.mean[2]) < 1e-12
		assert m.material.opacity == s.opacity
		match m.geometry {
			EllipsoidGeometry {
				assert m.geometry.radii == s.scale
			}
			else {
				assert false, 'expected EllipsoidGeometry'
			}
		}
	}
}

fn test_render_cyclide_splats() {
	cyc := splat_test_cyclide()
	mut sc := scene(none)
	sc.add_mesh(mesh(MeshParams{
		geometry:       cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!)
		material:       standard_material(MaterialParams{
			color:      color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.1
			emissive:   color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	g := sample_gaussians_on_cyclide(cyc, 8, 8, 0.1, 0.025, splat_test_material())
	for m in g.to_meshes() {
		sc.add_mesh(m)
	}
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(40.0, 1.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [0.0, 0.0, 0.0]!, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	img := render_frame(sc, cam, 128, 128, 1)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/splat_cyclide.png', img)
	// the cyclide shows as red-dominant pixels, the pale splats as bright
	// near-white pixels; both must be present (attachment renders)
	data := img.data_f32()
	mut red := 0
	mut pale := 0
	for i := 0; i < data.len; i += 4 {
		r := data[i]
		gg := data[i + 1]
		b := data[i + 2]
		if r > gg + 20.0 && r > b + 20.0 {
			red++
		}
		if r > 200.0 && gg > 200.0 && b > 200.0 {
			pale++
		}
	}
	assert red > 50
	assert pale > 20
}
