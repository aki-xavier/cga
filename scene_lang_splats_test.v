module cga

import math
import os

// CGS splats(...) modifier: two-mesh emission, pose sharing, clean errors,
// end-to-end render.

fn splat_cloud_of(m Mesh) Gaussians {
	return match m.geometry {
		SplatsGeometry { m.geometry.gaussians }
		else { panic('expected SplatsGeometry') }
	}
}

fn test_cgs_splats_emit_opaque_plus_splat_mesh() {
	sc, _ :=
		cgs_load('material(color=0xC0392B) splats(n=100, sigma_tangent=0.1, sigma_normal=0.02) sphere(r=1);', '')
	assert sc.objects.len == 2
	// opaque sphere first, splat mesh second, both at the same pose
	assert sl_is_sphere(sc.objects[0].geometry)
	g := splat_cloud_of(sc.objects[1])
	assert g.splats.len == 100 // sphere Fibonacci sampling is exact
	for i in 0 .. 3 {
		assert math.abs(sc.objects[0].position[i] - sc.objects[1].position[i]) < 1e-12
	}
	// means on the sphere, per-splat colour inherited from the material
	for s in g.splats {
		assert math.abs(math.sqrt(vec3_dot(s.mean, s.mean)) - 1.0) < 1e-9
		assert s.color.r > 0.7
		assert s.color.g < 0.3
		assert math.abs(s.opacity - 0.6) < 1e-12 // default
	}
}

fn sl_is_sphere(g Geometry) bool {
	return match g {
		SphereGeometry { true }
		else { false }
	}
}

fn test_cgs_splats_share_transform() {
	sc, _ := cgs_load('translate([1, 2, 3]) splats(n=64) sphere(r=0.5);', '')
	assert sc.objects.len == 2
	// both meshes share the pose; splat means are LOCAL (posed by the motor)
	for i in 0 .. 3 {
		assert math.abs(sc.objects[1].position[i] - f64(i) - 1.0) < 1e-9
	}
	// world space = local means through the mesh motor (the render path)
	g := transform_gaussians(splat_cloud_of(sc.objects[1]), sc.objects[1].motor())
	mut c := [0.0, 0.0, 0.0]!
	for s in g.splats {
		d := [s.mean[0] - 1.0, s.mean[1] - 2.0, s.mean[2] - 3.0]!
		assert math.abs(math.sqrt(vec3_dot(d, d)) - 0.5) < 1e-9
		c = [c[0] + s.mean[0], c[1] + s.mean[1], c[2] + s.mean[2]]!
	}
	// and the cloud centroid is close to it (Fibonacci lattice residual)
	n := f64(g.splats.len)
	assert math.abs(c[0] / n - 1.0) < 5e-3
	assert math.abs(c[1] / n - 2.0) < 5e-3
	assert math.abs(c[2] / n - 3.0) < 5e-3
}

fn test_cgs_splats_colour_and_opacity_override() {
	sc, _ :=
		cgs_load('material(color=0xC0392B) splats(n=16, opacity=0.9, color=0x00FF00) sphere(r=1);', '')
	g := splat_cloud_of(sc.objects[1])
	assert math.abs(g.splats[0].opacity - 0.9) < 1e-12
	assert g.splats[0].color.g > 0.9
	assert g.splats[0].color.r < 0.1
}

fn test_cgs_splats_cyclide_approx_count() {
	sc, _ := cgs_load('splats(n=300) cyclide(a=1.0, b=0.98, d=0.3);', '')
	assert sc.objects.len == 2
	g := splat_cloud_of(sc.objects[1])
	// grid rounding: approximately 300
	assert g.splats.len >= 290 && g.splats.len <= 330
}

fn test_cgs_splats_unsupported_kinds_are_clean_errors() {
	mut saw := false
	cgs_load_result('splats(n=10) circle(r=1);', '') or {
		assert err.msg().contains('splats are not supported')
		saw = true
	}
	assert saw
	// CSG blocks reject splats too (no raw sampler panic)
	saw = false
	cgs_load_result('difference() { splats(n=10) sphere(r=1); sphere(r=0.5); }', '') or {
		assert err.msg().contains('splats')
		saw = true
	}
	assert saw
	// non-uniform scale is a clean error, not a panic
	saw = false
	cgs_load_result('scale([2, 1, 1]) splats(n=10) sphere(r=1);', '') or {
		assert err.msg().contains('non-uniform scale')
		saw = true
	}
	assert saw
	// invalid splat params
	saw = false
	cgs_load_result('splats(n=0) sphere(r=1);', '') or {
		assert err.msg().contains('splats needs n > 0')
		saw = true
	}
	assert saw
}

fn test_cgs_splats_end_to_end_render() {
	text := 'background(color=0x87CEEB);\n' +
		'camera(fov=40, aspect=1.0, position=[0, 0, 4], target=[0, 0, 0]);\n' +
		'material(color=0xC0392B) splats(n=200, sigma_tangent=0.12, sigma_normal=0.03, color=0x00FF00) sphere(r=1);\n' +
		'directional_light(direction=[0.5, 1.0, 0.5], intensity=0.8);\n' +
		'ambient_light(intensity=0.3);\n'
	sc, mut cam := cgs_load(text, '')
	assert sc.objects.len == 2
	mut r := renderer(128, 128, 1, 3)
	img := render_scene_with_splats(sc, mut r, cam)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/cgs_splats.png', img)
	// green splats must cover the sphere centre (red opaque under green layer)
	data := img.data_f32()
	mut green := 0
	for i := 0; i < data.len; i += 4 {
		if data[i + 1] > data[i] + 15.0 && data[i + 1] > data[i + 2] + 15.0 {
			green++
		}
	}
	assert green > 200
}
