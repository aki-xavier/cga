module cga

import os
import math

fn test_glb_roundtrip() {
	verts := [[0.0, 0.0, 0.0]!, [1.0, 0.0, 0.0]!, [0.0, 1.0, 0.0]!,
		[0.0, 0.0, 1.0]!]
	faces := [[0, 1, 2]!, [0, 2, 3]!, [0, 3, 1]!, [1, 3, 2]!]
	save_glb('/tmp/cga_roundtrip.glb', [
		GltfMeshIn{
			vertices: verts
			faces:    faces
		},
	])
	out := load_gltf('/tmp/cga_roundtrip.glb')!
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

fn test_gltf_json_roundtrip() {
	// plain .gltf (JSON) with an external .bin buffer, like a typical exporter
	verts := [[0.0, 0.0, 0.0]!, [1.0, 0.0, 0.0]!, [0.0, 1.0, 0.0]!,
		[0.0, 0.0, 1.0]!]
	faces := [[0, 1, 2]!, [0, 2, 3]!, [0, 3, 1]!, [1, 3, 2]!]
	mut bin := []u8{}
	for p in verts {
		push_f32(mut bin, f32(p[0]))
		push_f32(mut bin, f32(p[1]))
		push_f32(mut bin, f32(p[2]))
	}
	pos_len := bin.len
	for f in faces {
		push_u32(mut bin, u32(f[0]))
		push_u32(mut bin, u32(f[1]))
		push_u32(mut bin, u32(f[2]))
	}
	idx_len := bin.len - pos_len
	os.write_file('/tmp/cga_tetra.bin', bin.bytestr()) or { panic('write bin') }
	json_str := '{"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1,"mode":4}]}],"buffers":[{"uri":"cga_tetra.bin","byteLength":${bin.len}}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":${pos_len}},{"buffer":0,"byteOffset":${pos_len},"byteLength":${idx_len}}],"accessors":[{"bufferView":0,"componentType":5126,"count":${verts.len},"type":"VEC3"},{"bufferView":1,"componentType":5125,"count":${faces.len * 3},"type":"SCALAR"}]}'
	os.write_file('/tmp/cga_tetra.gltf', json_str) or { panic('write gltf') }
	out := load_gltf('/tmp/cga_tetra.gltf')!
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
	os.rm('/tmp/cga_tetra.gltf') or {}
	os.rm('/tmp/cga_tetra.bin') or {}
}

fn test_glb_color_material() {
	verts := [[0.0, 0.0, 0.0]!, [1.0, 0.0, 0.0]!, [0.0, 1.0, 0.0]!,
		[0.0, 0.0, 1.0]!]
	faces := [[0, 1, 2]!, [0, 2, 3]!, [0, 3, 1]!, [1, 3, 2]!]
	save_glb('/tmp/cga_color.glb', [
		GltfMeshIn{
			vertices: verts
			faces:    faces
			color:    [0.8, 0.2, 0.2]!
		},
	])
	// the color must be written as a baseColorFactor material
	data := os.read_bytes('/tmp/cga_color.glb') or { panic('read glb') }
	assert data.bytestr().contains('baseColorFactor')
	assert data.bytestr().contains('0.8,0.2,0.2')
	// and the file must still load back
	out := load_gltf('/tmp/cga_color.glb')!
	assert out.len == 1
	assert out[0].vertices.len == 4
	os.rm('/tmp/cga_color.glb') or {}
}
