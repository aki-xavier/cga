module cga

import mlx

// CSG solid-protocol membership (contains) for union / difference /
// intersection, including a nested tree.

fn f32s_cc(vals []f64) []f32 {
	mut out := []f32{len: vals.len}
	for i, v in vals {
		out[i] = f32(v)
	}
	return out
}

fn cc_contains(g Geometry, pts [][3]f64) []bool {
	p := geom_to_camera(g, motor_identity())
	mut flat := []f64{cap: pts.len * 3}
	for q in pts {
		flat << q[0]
		flat << q[1]
		flat << q[2]
	}
	pos := mlx.array_f32(f32s_cc(flat), [pts.len, 3])
	return geom_contains(p, pos).data_bool()
}

fn test_csg_union_contains() {
	g := csg_geometry('union', [sphere_geometry(1.0), box_geometry(4.0, 4.0, 4.0)])
	// inside sphere | inside box | outside both
	assert cc_contains(g, [[0.9, 0.0, 0.0]!, [1.5, 0.0, 0.0]!,
		[3.0, 0.0, 0.0]!]) == [true, true, false]
}

fn test_csg_difference_contains() {
	g := csg_geometry('difference', [box_geometry(4.0, 4.0, 4.0),
		sphere_geometry(1.0)])
	// in box not sphere | in sphere | outside box
	assert cc_contains(g, [[1.5, 0.0, 0.0]!, [0.0, 0.0, 0.0]!,
		[3.0, 0.0, 0.0]!]) == [true, false, false]
}

fn test_csg_intersection_contains() {
	g := csg_geometry('intersection', [box_geometry(4.0, 4.0, 4.0),
		sphere_geometry(1.0)])
	// in both | in box not sphere | outside both
	assert cc_contains(g, [[0.5, 0.0, 0.0]!, [1.5, 0.0, 0.0]!,
		[3.0, 0.0, 0.0]!]) == [true, false, false]
}

fn test_csg_nested_contains() {
	inner := csg_geometry('difference', [box_geometry(4.0, 4.0, 4.0),
		sphere_geometry(1.0)])
	// half-space y<0
	g := csg_geometry('intersection', [inner, plane_geometry([0.0, 1.0, 0.0]!, 0.0)])
	assert cc_contains(g, [[1.5, -0.5, 0.0]!, [1.5, 0.5, 0.0]!]) == [true, false]
}
