module cga

import os

fn test_cgs_orbit() {
	text := os.read_file('examples/orbit.cgs') or { panic('read orbit.cgs') }
	sc, _ := cgs_load(text, 'examples')
	assert sc.objects.len == 7
	assert sc.lights.len == 4
}

fn test_cgs_grid_module() {
	text := os.read_file('examples/grid.cgs') or { panic('read grid.cgs') }
	sc, _ := cgs_load(text, 'examples')
	assert sc.objects.len == 10
	assert sc.lights.len == 2
}

fn test_cgs_csg_and_scale() {
	text := 'scale([2,1,1]) sphere(r=1);'
	sc, _ := cgs_load(text, '')
	assert sc.objects.len == 1
	text2 := 'difference() { sphere(r=1); box(s=[1,1,1]); }'
	sc2, _ := cgs_load(text2, '')
	assert sc2.objects.len == 1
}

fn test_cgs_render() {
	text := os.read_file('examples/orbit.cgs') or { panic('read orbit.cgs') }
	sc, cam := cgs_load(text, 'examples')
	mut r := renderer(120, 90, 1, 3)
	img := r.render(sc, cam)
	os.mkdir_all('artifacts/tests') or {}
	save_frame_png('artifacts/tests/cgs_orbit.png', img)
	data := img.data_f32()
	idx := 45 * 120 * 4 + 60 * 4
	// centre is not the sky-blue background
	assert data[idx + 2] < 200.0
}
