// non-OOP — test file (exempt from the OOP rule)
module cga

// Regression tests for geometry under non-identity transforms:
//   - cone_contains / trimesh_contains now map query points back to the
//     local frame (they used to ignore a_inv3/t_inv);
//   - box_intersect now rotates normals into camera space and flips the
//     exit-face normal for inside starts (matching box_crossings).
import mlx
import math

fn tf_ray(x f64, y f64, z f64) mlx.Array {
	return mlx.array_f32([f32(x), f32(y), f32(z)], [1, 3])
}

fn tf_contains(g Geometry, m Multivector, pts [][3]f64) []bool {
	p := geom_to_camera(g, m)
	mut flat := []f32{cap: pts.len * 3}
	for q in pts {
		flat << f32(q[0])
		flat << f32(q[1])
		flat << f32(q[2])
	}
	pos := mlx.array_f32(flat, [pts.len, 3])
	return geom_contains(p, pos).data_bool()
}

fn tf_hit(g Geometry, m Multivector, o [3]f64, d [3]f64) (f32, [3]f32, bool) {
	p := geom_to_camera(g, m)
	t, n, mask := geom_intersect(p, tf_ray(o[0], o[1], o[2]), tf_ray(d[0], d[1], d[2]))
	nd := n.data_f32()
	return t.item_f32(), [nd[0], nd[1], nd[2]]!, mask.data_bool()[0]
}

fn test_moved_cone_contains() {
	g := cone_geometry(1.0, 2.0)
	// cone translated +1 in x: (1.2,0,0) -> local (0.2,0,0), inside;
	// (1.6,0,0) -> local (0.6,0,0), outside (radius at s=-1 is 0.5)
	assert tf_contains(g, translator([1.0, 0.0, 0.0]!), [[1.2, 0.0, 0.0]!, [1.6, 0.0, 0.0]!]) == [
		true,
		false,
	]
	// the identity case the old code already got right
	assert tf_contains(g, motor_identity(), [[0.0, 0.0, 0.0]!, [0.6, 0.0, 0.0]!]) == [
		true,
		false,
	]
}

fn test_moved_mesh_contains() {
	verts, faces := extrude([[0.0, 0.0]!, [1.0, 0.0]!, [1.0, 1.0]!, [0.0, 1.0]!], 1.0)
	g := trimesh_geometry(verts, faces)
	// mesh translated +5 in x: (5.5,0.4,0.5) inside; (5.5,1.5,0.5) outside.
	// (y=0.4 avoids the side-quad diagonal, where the parity ray would
	// double-count the two triangles sharing the edge.)
	assert tf_contains(g, translator([5.0, 0.0, 0.0]!), [[5.5, 0.4, 0.5]!, [5.5, 1.5, 0.5]!]) == [
		true,
		false,
	]
	assert tf_contains(g, motor_identity(), [[0.5, 0.4, 0.5]!, [1.5, 0.5, 0.5]!]) == [
		true,
		false,
	]
}

fn test_csg_moved_cone_contains() {
	// realistic path: cone as a CSG child, the whole node moved
	g := csg_geometry(.union, [cone_geometry(1.0, 2.0), cone_geometry(1.0, 2.0)])
	assert tf_contains(g, translator([1.0, 0.0, 0.0]!), [[1.2, 0.0, 0.0]!, [1.6, 0.0, 0.0]!]) == [
		true,
		false,
	]
}

fn test_csg_moved_mesh_contains() {
	verts, faces := extrude([[0.0, 0.0]!, [1.0, 0.0]!, [1.0, 1.0]!, [0.0, 1.0]!], 1.0)
	g := csg_geometry(.union, [trimesh_geometry(verts, faces), trimesh_geometry(verts, faces)])
	assert tf_contains(g, translator([5.0, 0.0, 0.0]!), [[5.5, 0.4, 0.5]!, [5.5, 1.5, 0.5]!]) == [
		true,
		false,
	]
}

fn test_rotated_box_normal() {
	// box rotated 30 deg about x; a ray straight down -z hits the tilted local
	// +z face (not an edge at this angle): plane n.x = 0.5 with
	// n = R_x(30) . [0,0,1] = [0, -sin(30), cos(30)], so
	// t = (0.866 * 5 - 0.5) / 0.866 = 4.4226
	g := box_geometry(1.0, 1.0, 1.0)
	m := motor_rotor([1.0, 0.0, 0.0]!, math.pi / 6.0)
	t, n, hit := tf_hit(g, m, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert hit
	assert math.abs(f64(t) - 4.4226) < 1e-3
	assert math.abs(f64(n[0])) < 1e-2
	assert math.abs(f64(n[1]) + 0.5) < 1e-2
	assert math.abs(f64(n[2]) - math.sqrt(3.0) / 2.0) < 1e-2
}

fn test_box_inside_exit_normal() {
	// ray starts inside an axis-aligned box and exits through the +x face:
	// the outward normal is +x (used to be flipped to -x)
	g := box_geometry(1.0, 1.0, 1.0)
	t, n, hit := tf_hit(g, motor_identity(), [0.0, 0.0, 0.0]!, [1.0, 0.0, 0.0]!)
	assert hit
	assert math.abs(f64(t) - 0.5) < 1e-3
	assert math.abs(f64(n[0]) - 1.0) < 1e-2
	assert math.abs(f64(n[1])) < 1e-2
	assert math.abs(f64(n[2])) < 1e-2
}

fn test_box_entry_normal_unchanged() {
	// axis-aligned box from outside: identical behaviour before/after the fix
	g := box_geometry(1.0, 1.0, 1.0)
	t, n, hit := tf_hit(g, motor_identity(), [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert hit
	assert math.abs(f64(t) - 4.5) < 1e-3
	assert math.abs(f64(n[0])) < 1e-2
	assert math.abs(f64(n[1])) < 1e-2
	assert math.abs(f64(n[2]) - 1.0) < 1e-2
}
