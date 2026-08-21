module main

// params_test.v — tests for CGS numeric-parameter extraction.

fn param_labels(ps []RawParam) []string {
	mut out := []string{}
	for p in ps {
		out << p.label
	}
	return out
}

fn test_params_named_scalar() {
	ps := extract_params('sphere(r=1);')
	assert ps.len == 1
	assert ps[0].label == 'sphere.r'
	assert ps[0].value == 1.0
	assert replace_range('sphere(r=1);', ps[0].start, ps[0].end, format_number(2.5)) == 'sphere(r=2.5);'
}

fn test_params_vector_elements() {
	ps := extract_params('translate([0, 1, 0]) sphere(r=0.5);')
	assert param_labels(ps) == ['translate.t[0]', 'translate.t[1]', 'translate.t[2]', 'sphere.r']
}

fn test_params_skips_hex_colors() {
	ps := extract_params('background(color=0x87CEEB);')
	assert ps.len == 0
}

fn test_params_range_loop_bounds() {
	ps := extract_params('for (i = [0:2]) sphere(r=1);')
	assert param_labels(ps) == ['for.i.start', 'for.i.stop', 'sphere.r']
}

fn test_params_material_named_args() {
	ps := extract_params('material(color=0xB0B0B0, roughness=0.7, metalness=0.25) sphere(r=1);')
	assert param_labels(ps) == ['material.roughness', 'material.metalness', 'sphere.r']
}

fn test_params_format_number_trims() {
	assert format_number(1.0) == '1'
	assert format_number(0.7) == '0.7'
	assert format_number(6.1999998) == '6.2'
	assert format_number(-0.0) == '0'
}

fn test_params_splats_positional_and_named() {
	// positional args map to the splats signature names
	ps := extract_params('splats(200, 0.1, 0.02) sphere(r=1);')
	assert param_labels(ps) == ['splats.n', 'splats.sigma_tangent', 'splats.sigma_normal', 'sphere.r']
	// keyword args self-label
	ps2 := extract_params('splats(n=200, opacity=0.6) sphere(r=1);')
	assert param_labels(ps2) == ['splats.n', 'splats.opacity', 'sphere.r']
}
