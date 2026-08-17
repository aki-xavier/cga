module cga

import math

fn test_signed_area() {
	sq := [[0.0, 0.0]!, [1.0, 0.0]!, [1.0, 1.0]!, [0.0, 1.0]!]
	assert signed_area(sq) > 0.0
	// reversed is negative
	mut rev := [][2]f64{}
	for i in 0 .. sq.len {
		rev << [sq[sq.len - 1 - i][0], sq[sq.len - 1 - i][1]]!
	}
	assert signed_area(rev) < 0.0
}

fn test_triangulate_square() {
	sq := [[0.0, 0.0]!, [1.0, 0.0]!, [1.0, 1.0]!, [0.0, 1.0]!]
	tris := triangulate(sq)
	assert tris.len == 2
	for t in tris {
		assert t[0] != t[1] && t[1] != t[2] && t[2] != t[0]
	}
}

fn test_triangulate_concave() {
	// L-shaped (concave) polygon, 6 vertices -> 4 triangles
	l := [[0.0, 0.0]!, [2.0, 0.0]!, [2.0, 1.0]!, [1.0, 1.0]!, [1.0, 2.0]!, [0.0,
		2.0]!]
	tris := triangulate(l)
	assert tris.len == 4
}

fn test_extrude_watertight() {
	sq := [[0.0, 0.0]!, [1.0, 0.0]!, [1.0, 1.0]!, [0.0, 1.0]!]
	verts, faces := extrude(sq, 2.0)
	assert verts.len == 8
	// 4 side quads (8 tris) + 2 caps (4 tris) = 12
	assert faces.len == 12
	for f in faces {
		for i in f {
			assert i >= 0 && i < verts.len
		}
	}
}

fn test_loft_watertight() {
	sq1 := [[0.0, 0.0]!, [1.0, 0.0]!, [1.0, 1.0]!, [0.0, 1.0]!]
	sq2 := [[0.5, 0.5]!, [1.5, 0.5]!, [1.5, 1.5]!, [0.5, 1.5]!]
	verts, faces := loft([sq1, sq2], [0.0, 3.0])
	assert verts.len == 8
	assert faces.len == 12
	for f in faces {
		for i in f {
			assert i >= 0 && i < verts.len
		}
	}
}

fn test_transform_point() {
	m := from_trs([1.0, 2.0, 3.0]!, [0.0, 0.0, 0.0, 1.0]!, [1.0, 1.0, 1.0]!)
	p := transform_point(m, [0.0, 0.0, 0.0]!)
	assert math.abs(p[0] - 1.0) < 1e-12
	assert math.abs(p[1] - 2.0) < 1e-12
	assert math.abs(p[2] - 3.0) < 1e-12
}

fn test_obj_roundtrip() {
	verts := [[0.0, 0.0, 0.0]!, [1.0, 0.0, 0.0]!, [0.0, 1.0, 0.0]!, [0.0, 0.0,
		1.0]!]
	faces := [[0, 1, 2]!, [0, 2, 3]!, [0, 3, 1]!, [1, 3, 2]!]
	save_obj('/tmp/cga_obj_roundtrip.obj', [ObjMesh{vertices: verts, faces: faces}])
	v2, f2 := load_obj('/tmp/cga_obj_roundtrip.obj')
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
}
