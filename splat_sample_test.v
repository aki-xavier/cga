module cga

import math
import os

fn sample_test_material() Material {
	return standard_material(MaterialParams{
		color:      color_hex(0xECF0F1)
		roughness:  0.4
		metalness:  0.0
		emissive:   color_hex(0x000000)
		opacity:    0.6
		ior:        1.5
		absorption: 0.0
	})
}

// assert_frame_e3 checks the Route A frame contract: the rotor maps local e3
// to the given analytic normal.
fn assert_frame_e3(s Gaussian, n [3]f64) {
	z := vec3_unit(dir3(rotor_from_quaternion(s.quat).apply(e3())))
	assert vec3_dot(z, n) > 0.999
	assert s.scale[2] < s.scale[0]
}

fn test_sample_sphere_on_surface() {
	g := sample_gaussians_on_surface(sphere_geometry(1.3), 200, 0.1, 0.02, sample_test_material())
	assert g.splats.len == 200
	for s in g.splats {
		r := math.sqrt(vec3_dot(s.mean, s.mean))
		assert math.abs(r - 1.3) < 1e-9
		assert_frame_e3(s, [s.mean[0] / 1.3, s.mean[1] / 1.3, s.mean[2] / 1.3]!)
	}
}

fn test_sample_ellipsoid_on_surface() {
	e := ellipsoid_geometry(1.0, 0.6, 0.8)
	g := sample_gaussians_on_surface(e, 200, 0.1, 0.02, sample_test_material())
	assert g.splats.len == 200
	for s in g.splats {
		f := math.pow(s.mean[0] / 1.0, 2.0) + math.pow(s.mean[1] / 0.6, 2.0) +
			math.pow(s.mean[2] / 0.8, 2.0)
		assert math.abs(f - 1.0) < 1e-9
		gr := [2.0 * s.mean[0] / 1.0, 2.0 * s.mean[1] / 0.36, 2.0 * s.mean[2] / 0.64]!
		assert_frame_e3(s, vec3_unit(gr))
	}
}

fn test_sample_torus_on_surface() {
	major := 1.2
	minor := 0.4
	g := sample_gaussians_on_surface(torus_geometry(major, minor), 300, 0.1, 0.02,
		sample_test_material())
	assert g.splats.len >= 290
	for s in g.splats {
		q := math.hypot(s.mean[0], s.mean[1])
		f := (q - major) * (q - major) + s.mean[2] * s.mean[2] - minor * minor
		assert math.abs(f) < 1e-9
		// analytic normal: from the tube centre to the point
		nrm := vec3_unit([s.mean[0] - major * s.mean[0] / q, s.mean[1] - major * s.mean[1] / q,
			s.mean[2]]!)
		assert_frame_e3(s, nrm)
	}
}

fn test_sample_box_on_surface() {
	hx, hy, hz := 0.9, 0.5, 0.7
	g := sample_gaussians_on_surface(box_geometry(2.0 * hx, 2.0 * hy, 2.0 * hz), 240, 0.1, 0.02,
		sample_test_material())
	assert g.splats.len > 200
	for s in g.splats {
		// each point lies on some face: one coordinate at +-half
		on_x := math.abs(math.abs(s.mean[0]) - hx) < 1e-12
		on_y := math.abs(math.abs(s.mean[1]) - hy) < 1e-12
		on_z := math.abs(math.abs(s.mean[2]) - hz) < 1e-12
		assert on_x || on_y || on_z
		// inside the box on the other axes
		assert math.abs(s.mean[0]) <= hx + 1e-12
		assert math.abs(s.mean[1]) <= hy + 1e-12
		assert math.abs(s.mean[2]) <= hz + 1e-12
	}
}

fn test_sample_cylinder_on_surface() {
	r := 0.7
	h := 1.1
	g := sample_gaussians_on_surface(cylinder_geometry(r, 2.0 * h), 240, 0.1, 0.02,
		sample_test_material())
	assert g.splats.len > 200
	for s in g.splats {
		q := math.hypot(s.mean[0], s.mean[1])
		on_side := math.abs(q - r) < 1e-12 && math.abs(s.mean[2]) <= h + 1e-12
		on_cap := math.abs(math.abs(s.mean[2]) - h) < 1e-12 && q <= r + 1e-12
		assert on_side || on_cap
	}
}

fn test_sample_cone_on_surface() {
	r := 0.8
	h := 1.6
	g := sample_gaussians_on_surface(cone_geometry(r, h), 240, 0.1, 0.02, sample_test_material())
	assert g.splats.len > 200
	k := r / h
	for s in g.splats {
		q := math.hypot(s.mean[0], s.mean[1])
		sz := s.mean[2] - h / 2.0
		on_side := math.abs(q * q - k * k * sz * sz) < 1e-12
		on_cap := math.abs(s.mean[2] + h / 2.0) < 1e-12 && q <= r + 1e-12
		assert on_side || on_cap
	}
}

fn test_sample_plane_on_surface() {
	pl := plane_geometry([0.0, 0.0, 1.0]!, 0.4)
	g := sample_gaussians_on_surface(pl, 100, 0.1, 0.02, sample_test_material())
	assert g.splats.len >= 100
	for s in g.splats {
		assert math.abs(s.mean[2] - 0.4) < 1e-12
		// default patch half-size 1.0 around the plane origin
		assert math.abs(s.mean[0]) <= 1.0 + 1e-12
		assert math.abs(s.mean[1]) <= 1.0 + 1e-12
		assert_frame_e3(s, [0.0, 0.0, 1.0]!)
	}
	// explicit extent via the plane helper
	g2 := sample_gaussians_on_plane(pl, 2.5, 25, 0.1, 0.02, sample_test_material())
	mut saw_wide := false
	for s in g2.splats {
		if math.abs(s.mean[0]) > 1.5 || math.abs(s.mean[1]) > 1.5 {
			saw_wide = true
		}
	}
	assert saw_wide
}

fn test_sample_cyclide_via_general_surface() {
	cyc := dupin_cyclide(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!)
	g := sample_gaussians_on_surface(cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0]!), 300, 0.1,
		0.02, sample_test_material())
	assert g.splats.len >= 290
	for s in g.splats {
		assert math.abs(cyc.implicit(s.mean[0], s.mean[1], s.mean[2])) < 1e-6
	}
}

fn test_sample_affine_uniform_scale() {
	// sphere of radius 1 under uniform scale 2 + shift: |p - t| == 2 and the
	// splat scales double
	lin := mat3_new([2.0, 0.0, 0.0]!, [0.0, 2.0, 0.0]!, [0.0, 0.0, 2.0]!)
	af := transformed_geometry(sphere_geometry(1.0), translator([0.5, 0.0, 0.0]!), lin)
	g := sample_gaussians_on_surface(af, 100, 0.1, 0.02, sample_test_material())
	assert g.splats.len == 100
	for s in g.splats {
		dx := s.mean[0] - 0.5
		r := math.sqrt(dx * dx + s.mean[1] * s.mean[1] + s.mean[2] * s.mean[2])
		assert math.abs(r - 2.0) < 1e-9
		assert math.abs(s.scale[0] - 0.2) < 1e-12
		assert math.abs(s.scale[2] - 0.04) < 1e-12
		assert_frame_e3(s, [dx / 2.0, s.mean[1] / 2.0, s.mean[2] / 2.0]!)
	}
}

fn test_render_ellipsoid_with_splat_mesh() {
	w := 160
	h := 120
	mut sc := scene(none)
	geo := ellipsoid_geometry(1.2, 0.8, 0.9)
	sc.add_mesh(mesh(MeshParams{
		geometry:       geo
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
	// attached splat layer via the general sampler + SplatsGeometry (posed)
	spl := sample_gaussians_on_surface(geo, 400, 0.1, 0.025, sample_test_material())
	assert spl.splats.len == 400
	sc.add_mesh(mesh(MeshParams{
		geometry:       splats_geometry(spl)
		material:       basic_material(color_hex(0xECF0F1), 0.6)
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]!))
	sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3))
	mut cam := perspective_camera(40.0, 160.0 / 120.0, 0.1, 100.0, [0.0, 0.0, 4.0]!, [
		0.0,
		0.0,
		0.0,
	]!, [0.0, 1.0, 0.0]!)
	cam.look_at([0.0, 0.0, 0.0]!, none)
	mut r := renderer(w, h, 1, 3)
	img := render_scene_with_splats(sc, mut r, cam)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/splat_ellipsoid.png', img)
	data := img.data_f32()
	mut red := 0
	mut pale := 0
	for i := 0; i < data.len; i += 4 {
		if data[i] > data[i + 1] + 20.0 && data[i] > data[i + 2] + 20.0 {
			red++
		}
		if data[i] > 180.0 && data[i + 1] > 180.0 && data[i + 2] > 180.0 {
			pale++
		}
	}
	assert red > 100
	assert pale > 50
}
