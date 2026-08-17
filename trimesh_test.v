module cga

import mlx
import math

// Möller–Trumbore triangle-mesh intersection correctness (no BVH; brute force).

fn tm_ray(x f64, y f64, z f64) mlx.Array {
	return mlx.array_f32([f32(x), f32(y), f32(z)], [1, 3])
}

fn tm_hit(g Geometry, o [3]f64, d [3]f64) (f32, [3]f32, bool) {
	p := geom_to_camera(g, motor_identity())
	t, n, mask := geom_intersect(p, tm_ray(o[0], o[1], o[2]), tm_ray(d[0], d[1], d[2]))
	nd := n.data_f32()
	return t.item_f32(), [nd[0], nd[1], nd[2]]!, mask.data_bool()[0]
}

fn tm_triangle() Geometry {
	// triangle in the z=1 plane, centroid (2/3, 2/3, 1), normal +z
	return trimesh_geometry([[0.0, 0.0, 1.0]!, [2.0, 0.0, 1.0]!, [0.0, 2.0, 1.0]!],
		[[0, 1, 2]!])
}

fn test_trimesh_center_hit() {
	g := tm_triangle()
	t, n, m := tm_hit(g, [2.0 / 3.0, 2.0 / 3.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(f64(t) - 4.0) < 1e-3
	assert math.abs(f64(n[0])) < 1e-2
	assert math.abs(f64(n[1])) < 1e-2
	assert math.abs(f64(n[2]) - 1.0) < 1e-2
}

fn test_trimesh_miss_outside() {
	g := tm_triangle()
	_, _, m := tm_hit(g, [3.0, 3.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert !m
}

fn test_trimesh_parallel_ray_miss() {
	g := tm_triangle()
	_, _, m := tm_hit(g, [0.5, 0.5, 5.0]!, [1.0, 0.0, 0.0]!)
	assert !m
}

fn test_trimesh_backface_normal_flips() {
	g := tm_triangle()
	t, n, m := tm_hit(g, [2.0 / 3.0, 2.0 / 3.0, -5.0]!, [0.0, 0.0, 1.0]!)
	assert m
	assert math.abs(f64(t) - 6.0) < 1e-3
	assert math.abs(f64(n[0])) < 1e-2
	assert math.abs(f64(n[1])) < 1e-2
	assert math.abs(f64(n[2]) + 1.0) < 1e-2
}

fn test_trimesh_nearest_of_two() {
	g := trimesh_geometry([[0.0, 0.0, 1.0]!, [2.0, 0.0, 1.0]!, [0.0, 2.0, 1.0]!,
		[0.0, 0.0, 2.0]!, [2.0, 0.0, 2.0]!, [0.0, 2.0, 2.0]!], [[0, 1, 2]!,
		[3, 4, 5]!])
	t, _, m := tm_hit(g, [2.0 / 3.0, 2.0 / 3.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(f64(t) - 3.0) < 1e-3 // nearest face is at z=2
}

fn test_trimesh_translated_by_motor() {
	// local triangle at z=1, motor translates by +z=2 -> world z=3
	g := tm_triangle()
	p := geom_to_camera(g, translator([0.0, 0.0, 2.0]!))
	t, _, mask := geom_intersect(p, tm_ray(2.0 / 3.0, 2.0 / 3.0, 5.0), tm_ray(0.0,
		0.0, -1.0))
	assert mask.data_bool()[0]
	assert math.abs(f64(t.item_f32()) - 2.0) < 1e-3
}
