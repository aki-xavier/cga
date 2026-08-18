module cga

import mlx
import math

fn csg_hit(g Geometry, o [3]f64, d [3]f64) (f32, bool) {
	p := geom_to_camera(g, motor_identity())
	oa := mlx.array_f32([f32(o[0]), f32(o[1]), f32(o[2])], [1, 3])
	da := mlx.array_f32([f32(d[0]), f32(d[1]), f32(d[2])], [1, 3])
	t, _, mask := geom_intersect(p, oa, da)
	return t.item_f32(), mask.data_bool()[0]
}

fn test_csg_difference() {
	g := csg_geometry(.difference, [sphere_geometry(1.0), box_geometry(1.0, 1.0, 1.0)])
	t, m := csg_hit(g, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(t - 4.0) < 1e-4
}

fn test_csg_intersection() {
	g := csg_geometry(.intersection, [sphere_geometry(1.0), box_geometry(1.0, 1.0, 1.0)])
	t, m := csg_hit(g, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(t - 4.5) < 1e-4
}

fn test_csg_halfspace() {
	g := csg_geometry(.intersection, [sphere_geometry(1.0), plane_geometry([0.0, 1.0, 0.0]!, 0.0)])
	t, m := csg_hit(g, [0.0, 5.0, 0.0]!, [0.0, -1.0, 0.0]!)
	assert m
	assert math.abs(t - 5.0) < 1e-4
}

fn test_csg_nested() {
	inner := csg_geometry(.difference, [sphere_geometry(1.0),
		box_geometry(1.0, 1.0, 1.0)])
	g := csg_geometry(.intersection, [inner, plane_geometry([0.0, 1.0, 0.0]!, 0.0)])
	t, m := csg_hit(g, [0.0, -0.2, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(t - (5.0 - math.sqrt(0.96))) < 1e-3
}

fn test_csg_union_scaled() {
	g := csg_geometry(.union, [sphere_geometry(1.0), ellipsoid_geometry(1.0, 1.0, 3.0)])
	t, m := csg_hit(g, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(t - 2.0) < 1e-3
}
