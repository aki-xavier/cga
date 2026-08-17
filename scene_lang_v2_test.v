module cga

import math

// CGS v2: variables, expressions, math functions, for+range, module, if-else,
// echo, union grouping (OpenSCAD-aligned semantics).

fn sl_sphere_radius(g Geometry) f64 {
	return match g {
		SphereGeometry { g.radius }
		else { panic('expected sphere geometry') }
	}
}

fn test_cgs_variables_and_expressions() {
	sc, _ := cgs_load('r = 0.4 + 0.1;\n' + 'x = 2 * (r + 0.5);\n' +
		'translate([x, 0, 0]) sphere(r=r);', '')
	assert sc.objects.len == 1
	assert sl_sphere_radius(sc.objects[0].geometry) == 0.5
	assert math.abs(sc.objects[0].position[0] - 2.0) < 1e-9
}

fn test_cgs_unary_minus_and_vector_arith() {
	sc, _ := cgs_load('base = [1, 2, 3];\ntranslate([-1, 0, 0] + base * 2) sphere(r=0.5);', '')
	assert sc.objects.len == 1
	p := sc.objects[0].position
	assert math.abs(p[0] - 1.0) < 1e-9
	assert math.abs(p[1] - 4.0) < 1e-9
	assert math.abs(p[2] - 6.0) < 1e-9
}

fn test_cgs_math_functions() {
	sc, _ := cgs_load('translate([sin(pi / 2), sqrt(16), max(3, 7)]) sphere(r=1);', '')
	assert sc.objects.len == 1
	p := sc.objects[0].position
	assert math.abs(p[0] - 1.0) < 1e-6
	assert math.abs(p[1] - 4.0) < 1e-6
	assert math.abs(p[2] - 7.0) < 1e-6
}

fn test_cgs_for_range() {
	sc, _ := cgs_load('for (i = [0:2]) translate([i, 0, 0]) sphere(r=0.1);' +
		'for (j = [0:0.5:1]) translate([0, j, 0]) sphere(r=0.1);', '')
	assert sc.objects.len == 6 // [0:2] -> 3 + [0:0.5:1] -> 3
	assert math.abs(sc.objects[0].position[0]) < 1e-9
	assert math.abs(sc.objects[1].position[0] - 1.0) < 1e-9
	assert math.abs(sc.objects[2].position[0] - 2.0) < 1e-9
	assert math.abs(sc.objects[3].position[1]) < 1e-9
	assert math.abs(sc.objects[4].position[1] - 0.5) < 1e-9
	assert math.abs(sc.objects[5].position[1] - 1.0) < 1e-9
}

fn test_cgs_module_with_defaults() {
	sc, _ := cgs_load('module bead(r, gap=1.0) { translate([r * gap, 0, 0]) sphere(r=r); }\n' +
		'bead(0.5);\n' + 'bead(0.5, gap=4.0);\n' + 'bead(r=0.25);', '')
	assert sc.objects.len == 3
	assert math.abs(sc.objects[0].position[0] - 0.5) < 1e-9
	assert math.abs(sc.objects[1].position[0] - 2.0) < 1e-9
	assert math.abs(sc.objects[2].position[0] - 0.25) < 1e-9
}

fn test_cgs_module_inherits_transform() {
	sc, _ := cgs_load('module ball() { sphere(r=1); }\ntranslate([3, 0, 0]) ball();', '')
	assert sc.objects.len == 1
	assert math.abs(sc.objects[0].position[0] - 3.0) < 1e-9
}

fn test_cgs_if_else() {
	sc, _ := cgs_load('which = 2;\n' + 'if (which == 1) { sphere(r=1); } else { sphere(r=2); }\n' +
		'if (which > 1) sphere(r=3);', '')
	assert sc.objects.len == 2
	assert math.abs(sl_sphere_radius(sc.objects[0].geometry) - 2.0) < 1e-9
	assert math.abs(sl_sphere_radius(sc.objects[1].geometry) - 3.0) < 1e-9
}

fn test_cgs_echo() {
	// echo runs without error and emits "ECHO: 3.0 [2.0, 4.0]"
	sc, _ := cgs_load('x = 1 + 2;\necho(x, [1, 2] * 2);', '')
	assert sc.objects.len == 0
}

fn test_cgs_union_is_grouping() {
	sc, _ := cgs_load('union() { sphere(r=1); translate([2, 0, 0]) sphere(r=1); }', '')
	assert sc.objects.len == 2
}
