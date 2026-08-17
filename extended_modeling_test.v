module cga

import mlx
import math
import os

// Extended modeling: affine wrapper, new primitives, builders, mesh IO, CGS v3.

fn em_ray(x f64, y f64, z f64) mlx.Array {
	return mlx.array_f32([f32(x), f32(y), f32(z)], [1, 3])
}

fn f32s(vals []f64) []f32 {
	mut out := []f32{len: vals.len}
	for i, v in vals {
		out[i] = f32(v)
	}
	return out
}

fn em_hit(g Geometry, o [3]f64, d [3]f64) (f32, [3]f32, bool) {
	p := geom_to_camera(g, motor_identity())
	t, n, mask := geom_intersect(p, em_ray(o[0], o[1], o[2]), em_ray(d[0], d[1], d[2]))
	nd := n.data_f32()
	return t.item_f32(), [nd[0], nd[1], nd[2]]!, mask.data_bool()[0]
}

fn em_contains(g Geometry, pts [][3]f64) []bool {
	p := geom_to_camera(g, motor_identity())
	mut flat := []f64{cap: pts.len * 3}
	for q in pts {
		flat << q[0]
		flat << q[1]
		flat << q[2]
	}
	pos := mlx.array_f32(f32s(flat), [pts.len, 3])
	return geom_contains(p, pos).data_bool()
}

fn geom_kind(g Geometry) string {
	return match g {
		CsgGeometry { 'csg' }
		ConeGeometry { 'cone' }
		TorusGeometry { 'torus' }
		EllipsoidGeometry { 'ellipsoid' }
		TrimeshGeometry { 'mesh' }
		SphereGeometry { 'sphere' }
		BoxGeometry { 'box' }
		CylinderGeometry { 'cylinder' }
		PlaneGeometry { 'plane' }
		else { 'other' }
	}
}

// --- affine ------------------------------------------------------------------

fn test_affine_scaled_sphere() {
	g := affine_geometry(sphere_geometry(1.0), mat3_new([2.0, 0.0, 0.0]!, [0.0, 1.0,
		0.0]!, [0.0, 0.0, 1.0]!))
	t, n, m := em_hit(g, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(f64(t) - 4.0) < 1e-5
	assert math.abs(f64(n[0])) < 1e-4
	assert math.abs(f64(n[1])) < 1e-4
	assert math.abs(f64(n[2]) - 1.0) < 1e-4
	_, _, m2 := em_hit(g, [3.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert !m2
}

fn test_affine_decompose_rigid_roundtrip() {
	m := translator([1.0, 2.0, 3.0]!).gp(motor_rotor([0.0, 1.0, 0.0]!, 0.7))
	motor2, lin := decompose_rigid(m.to_matrix())
	mut err := 0.0
	for i in 0 .. 3 {
		for j in 0 .. 3 {
			want := if i == j { 1.0 } else { 0.0 }
			e := math.abs(lin[i][j] - want)
			if e > err {
				err = e
			}
		}
	}
	assert err < 1e-4
	tm := motor2.to_matrix()
	assert math.abs(tm[3] - 1.0) < 1e-4
	assert math.abs(tm[7] - 2.0) < 1e-4
	assert math.abs(tm[11] - 3.0) < 1e-4
}

// --- new primitives ----------------------------------------------------------

fn test_cone_side_ray() {
	// r=1 h=2 (k=0.5): at z=-0.5 (s=-1.5) the radius is 0.75 -> t = 4.25
	g := cone_geometry(1.0, 2.0)
	t, _, m := em_hit(g, [5.0, 0.0, -0.5]!, [-1.0, 0.0, 0.0]!)
	assert m
	assert math.abs(f64(t) - 4.25) < 1e-4
}

fn test_cone_contains() {
	g := cone_geometry(1.0, 2.0)
	assert em_contains(g, [[0.0, 0.0, 0.0]!, [0.6, 0.0, 0.0]!]) == [true, false]
}

fn test_torus_rays() {
	g := torus_geometry(1.0, 0.3)
	_, _, m := em_hit(g, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert !m
	t2, n2, m2 := em_hit(g, [1.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m2
	assert math.abs(f64(t2) - 4.7) < 1e-3
	assert math.abs(f64(n2[0])) < 1e-2
	assert math.abs(f64(n2[1])) < 1e-2
	assert math.abs(f64(n2[2]) - 1.0) < 1e-2
}

fn test_torus_contains() {
	g := torus_geometry(1.0, 0.3)
	assert em_contains(g, [[1.0, 0.0, 0.1]!, [0.0, 0.0, 0.0]!]) == [true, false]
}

fn test_ellipsoid_is_scaled_sphere() {
	g := ellipsoid_geometry(2.0, 1.0, 1.0)
	t, _, m := em_hit(g, [0.0, 0.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(f64(t) - 4.0) < 1e-4 // z semi-axis = 1
}

// --- modeling builders -------------------------------------------------------

fn test_earclip_l_shape() {
	l := [[0.0, 0.0]!, [4.0, 0.0]!, [4.0, 2.0]!, [2.0, 2.0]!, [2.0, 4.0]!, [0.0,
		4.0]!]
	assert triangulate(l).len == 4 // 6-vertex L -> 4 triangles
}

fn test_extrude_hit_and_contains() {
	verts, faces := extrude([[0.0, 0.0]!, [4.0, 0.0]!, [4.0, 2.0]!, [2.0, 2.0]!, [2.0,
		4.0]!, [0.0, 4.0]!], 1.5)
	g := trimesh_geometry(verts, faces)
	t, _, m := em_hit(g, [1.0, 1.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(f64(t) - 3.5) < 1e-5 // top cap z = 1.5
	_, _, m2 := em_hit(g, [3.0, 3.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert !m2
	assert em_contains(g, [[1.0, 0.9, 0.6]!, [3.0, 3.0, 0.75]!]) == [true, false]
}

fn test_loft_between_squares() {
	verts, faces := loft([[[0.0, 0.0]!, [2.0, 0.0]!, [2.0, 2.0]!, [0.0, 2.0]!],
		[[0.4, 0.4]!, [1.6, 0.4]!, [1.6, 1.6]!, [0.4, 1.6]!]], [0.0, 1.0])
	assert verts.len == 8
	g := trimesh_geometry(verts, faces)
	t, _, m := em_hit(g, [1.0, 1.0, 5.0]!, [0.0, 0.0, -1.0]!)
	assert m
	assert math.abs(f64(t) - 4.0) < 1e-4 // top cap z = 1
}

// --- mesh IO -----------------------------------------------------------------

fn test_obj_roundtrip() {
	verts, faces := extrude([[0.0, 0.0]!, [2.0, 0.0]!, [2.0, 2.0]!, [0.0, 2.0]!], 1.0)
	save_obj('/tmp/cga_em_obj.obj', [ObjMesh{vertices: verts, faces: faces}])
	v2, f2 := load_obj('/tmp/cga_em_obj.obj')
	assert v2.len == verts.len
	assert f2.len == faces.len
	for i in 0 .. verts.len {
		assert math.abs(v2[i][0] - verts[i][0]) < 1e-9
		assert math.abs(v2[i][1] - verts[i][1]) < 1e-9
		assert math.abs(v2[i][2] - verts[i][2]) < 1e-9
	}
	for i in 0 .. faces.len {
		assert f2[i] == faces[i]
	}
	os.rm('/tmp/cga_em_obj.obj') or {}
}

fn test_glb_roundtrip_with_transform() {
	verts, faces := extrude([[0.0, 0.0]!, [2.0, 0.0]!, [2.0, 2.0]!, [0.0, 2.0]!], 1.0)
	t4 := [1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0,
		0.0, 1.0]!
	save_glb('/tmp/cga_em.glb', [GltfMeshIn{
		vertices: verts
		faces: faces
		transform: t4
		color: [0.8, 0.2, 0.2]!
	}])
	loaded := load_gltf('/tmp/cga_em.glb')
	assert loaded.len == 1
	assert loaded[0].faces.len == faces.len
	for i in 0 .. faces.len {
		assert loaded[0].faces[i] == faces[i]
	}
	assert math.abs(loaded[0].world[3] - 1.0) < 1e-6
	assert math.abs(loaded[0].world[7] - 2.0) < 1e-6
	assert math.abs(loaded[0].world[11] - 3.0) < 1e-6
	assert loaded[0].vertices.len == verts.len
	os.rm('/tmp/cga_em.glb') or {}
}

// --- CGS v3 ------------------------------------------------------------------

fn test_cgs_modifier_ordering() {
	sc, _ := cgs_load('translate([10,0,0]) scale(2) sphere(r=1);', '')
	assert sc.objects.len == 1
	p := sc.objects[0].position
	assert math.abs(p[0] - 10.0) < 1e-6
	assert math.abs(p[1]) < 1e-6
	assert math.abs(p[2]) < 1e-6
	sc2, _ := cgs_load('mirror(axis=[1,0,0]) translate([2.5,0,0]) sphere(r=1);', '')
	p2 := sc2.objects[0].position
	assert math.abs(p2[0] + 2.5) < 1e-6
	assert math.abs(p2[1]) < 1e-6
	assert math.abs(p2[2]) < 1e-6
}

fn test_cgs_csg_block_and_new_primitives() {
	text := 'difference() { box(s=[2,2,2]); cylinder(r=0.5, h=4); }\n' +
		'cone(r=1, h=2);\n' +
		'torus(R=1, r=0.3);\n' +
		'ellipsoid(radii=[1,2,3]);\n' +
		'extrude(profile=[[0,0],[1,0],[1,1],[0,1]], h=0.5);\n' +
		'p1 = [[0,0],[1,0],[1,1],[0,1]];\n' +
		'p2 = [[0.2,0.2],[0.8,0.2],[0.8,0.8],[0.2,0.8]];\n' +
		'loft(profiles=[p1, p2], zs=[0, 0.5]);'
	sc, _ := cgs_load(text, '')
	assert sc.objects.len == 6
	assert geom_kind(sc.objects[0].geometry) == 'csg'
	assert geom_kind(sc.objects[1].geometry) == 'cone'
	assert geom_kind(sc.objects[2].geometry) == 'torus'
	assert geom_kind(sc.objects[3].geometry) == 'ellipsoid'
	assert geom_kind(sc.objects[4].geometry) == 'mesh'
	assert geom_kind(sc.objects[5].geometry) == 'mesh'
}

fn test_cgs_precision_statement() {
	sc, _ := cgs_load('precision("float64"); sphere(r=1);', '')
	assert sc.objects.len == 1
	set_precision('float32')
}

fn test_cgs_gltf_mesh() {
	// save a GLB then load it back through the CGS mesh() primitive
	verts, faces := extrude([[0.0, 0.0]!, [2.0, 0.0]!, [2.0, 2.0]!, [0.0, 2.0]!], 1.0)
	save_glb('/tmp/cga_cgs.glb', [GltfMeshIn{
		vertices: verts
		faces: faces
	}])
	sc, _ := cgs_load('mesh(file="cga_cgs.glb");', '/tmp')
	assert sc.objects.len == 1
	assert geom_kind(sc.objects[0].geometry) == 'mesh'
	os.rm('/tmp/cga_cgs.glb') or {}
}
