module cga

import os
import math

fn test_glb_roundtrip() {
	verts := [[0.0, 0.0, 0.0]!, [1.0, 0.0, 0.0]!, [0.0, 1.0, 0.0]!, [0.0, 0.0, 1.0]!]
	faces := [[0, 1, 2]!, [0, 2, 3]!, [0, 3, 1]!, [1, 3, 2]!]
	save_glb('/tmp/cga_roundtrip.glb', [GltfMeshIn{
		vertices: verts
		faces: faces
	}])
	out := load_gltf('/tmp/cga_roundtrip.glb')
	assert out.len == 1
	assert out[0].vertices.len == 4
	assert out[0].faces.len == 4
	for i in 0 .. 4 {
		assert math.abs(out[0].vertices[i][0] - verts[i][0]) < 1e-5
		assert math.abs(out[0].vertices[i][1] - verts[i][1]) < 1e-5
		assert math.abs(out[0].vertices[i][2] - verts[i][2]) < 1e-5
	}
	for i in 0 .. 4 {
		assert out[0].faces[i] == faces[i]
	}
	os.rm('/tmp/cga_roundtrip.glb') or {}
}
