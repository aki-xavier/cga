module cga

// glTF 2.0 read + GLB write (geometry only: TRIANGLES primitives).
import encoding.binary
import encoding.base64
import json2
import os
import math

struct GltfAccessor {
pub:
	buffer_view    int @[json: 'bufferView']
	component_type int @[json: 'componentType']
	count          int
	typ            string @[json: 'type']
	byte_offset    int    @[json: 'byteOffset']
}

struct GltfBufferView {
pub:
	buffer      int
	byte_offset int @[json: 'byteOffset']
	byte_length int @[json: 'byteLength']
	byte_stride int @[json: 'byteStride']
}

struct GltfBuffer {
pub:
	uri string
}

struct GltfPrimitive {
pub:
	attributes map[string]int
	indices    ?int @[json: 'indices']
	mode       int
}

struct GltfMesh {
pub:
	primitives []GltfPrimitive
}

struct GltfNode {
pub:
	mesh        ?int @[json: 'mesh']
	matrix      []f64
	translation []f64
	rotation    []f64
	scale       []f64
	children    []int
}

struct GltfScene {
pub:
	nodes []int
}

struct GltfRoot {
pub:
	buffers      []GltfBuffer
	buffer_views []GltfBufferView @[json: 'bufferViews']
	accessors    []GltfAccessor
	meshes       []GltfMesh
	nodes        []GltfNode
	scenes       []GltfScene
	scene        int
}

// GltfMeshOut is one loaded mesh (vertices, faces, world transform).
pub struct GltfMeshOut {
pub:
	vertices [][3]f64
	faces    [][3]int
	world    [16]f64
}

// GltfMeshIn is one input entry for save_glb.
pub struct GltfMeshIn {
pub:
	vertices  [][3]f64
	faces     [][3]int
	transform ?[16]f64
	color     ?[3]f64
}

fn push_f32(mut b []u8, v f32) {
	push_u32(mut b, math.f32_bits(v))
}

fn push_u32(mut b []u8, v u32) {
	b << u8(v & 0xff)
	b << u8((v >> 8) & 0xff)
	b << u8((v >> 16) & 0xff)
	b << u8((v >> 24) & 0xff)
}

fn align4(mut b []u8) {
	for b.len % 4 != 0 {
		b << u8(0)
	}
}

fn f32_at(b []u8, o int) f32 {
	return math.f32_from_bits(binary.little_endian_u32_at(b, o))
}

fn node_local_matrix(n GltfNode) [16]f64 {
	if n.matrix.len == 16 {
		mut m := [16]f64{}
		for i in 0 .. 16 {
			m[i] = n.matrix[i]
		}
		return from_column_major(m)
	}
	t := if n.translation.len == 3 { [n.translation[0], n.translation[1], n.translation[2]]! } else { [
			0.0,
			0.0,
			0.0,
		]! }
	r := if n.rotation.len == 4 { [n.rotation[0], n.rotation[1], n.rotation[2], n.rotation[3]]! } else { [
			0.0,
			0.0,
			0.0,
			1.0,
		]! }
	sc := if n.scale.len == 3 { [n.scale[0], n.scale[1], n.scale[2]]! } else { [1.0, 1.0, 1.0]! }
	return from_trs(t, r, sc)
}

// save_glb writes meshes as a GLB file (each mesh = one TRIANGLES primitive).
pub fn save_glb(path string, meshes []GltfMeshIn) {
	mut blob := []u8{}
	mut nodes := []string{}
	mut meshes_j := []string{}
	mut accessors := []string{}
	mut views := []string{}
	mut materials := []string{}
	for mi, m in meshes {
		if m.vertices.len == 0 || m.faces.len == 0 {
			panic('mesh ${mi}: empty vertices/faces')
		}
		align4(mut blob)
		pos_off := blob.len
		for p in m.vertices {
			push_f32(mut blob, f32(p[0]))
			push_f32(mut blob, f32(p[1]))
			push_f32(mut blob, f32(p[2]))
		}
		pos_len := blob.len - pos_off
		align4(mut blob)
		idx_off := blob.len
		for f in m.faces {
			push_u32(mut blob, u32(f[0]))
			push_u32(mut blob, u32(f[1]))
			push_u32(mut blob, u32(f[2]))
		}
		idx_len := blob.len - idx_off
		views << '{"buffer":0,"byteOffset":${pos_off},"byteLength":${pos_len}}'
		views << '{"buffer":0,"byteOffset":${idx_off},"byteLength":${idx_len}}'
		mut mn := [1e30, 1e30, 1e30]!
		mut mx := [-1e30, -1e30, -1e30]!
		for p in m.vertices {
			for i in 0 .. 3 {
				if p[i] < mn[i] {
					mn[i] = p[i]
				}
				if p[i] > mx[i] {
					mx[i] = p[i]
				}
			}
		}
		pa := accessors.len
		iaa := accessors.len + 1
		accessors << '{"bufferView":${views.len - 2},"componentType":5126,"count":${m.vertices.len},"type":"VEC3","min":[${mn[0]},${mn[1]},${mn[2]}],"max":[${mx[0]},${mx[1]},${mx[2]}]}'
		accessors << '{"bufferView":${views.len - 1},"componentType":5125,"count":${m.faces.len * 3},"type":"SCALAR"}'
		mut mat_ref := ''
		if c := m.color {
			mat_idx := materials.len
			materials << '{"pbrMetallicRoughness":{"baseColorFactor":[${c[0]},${c[1]},${c[2]},1.0]}}'
			mat_ref = ',"material":${mat_idx}'
		}
		meshes_j << '{"primitives":[{"attributes":{"POSITION":${pa}},"indices":${iaa},"mode":4${mat_ref}}]}'
		mut node_extra := ''
		if tr := m.transform {
			cm := to_column_major(tr)
			node_extra = ',"matrix":[${cm[0]},${cm[1]},${cm[2]},${cm[3]},${cm[4]},${cm[5]},${cm[6]},${cm[7]},${cm[8]},${cm[9]},${cm[10]},${cm[11]},${cm[12]},${cm[13]},${cm[14]},${cm[15]}]'
		}
		nodes << '{"mesh":${mi},"name":"mesh_${mi}"${node_extra}}'
	}
	mat_json := if materials.len > 0 { ',"materials":[${materials.join(',')}]' } else { '' }
	json_str := '{"asset":{"version":"2.0","generator":"cga.mesh_io"},"scene":0,"scenes":[{"nodes":[${nodes.join(',')}]}],"nodes":[${nodes.join(',')}],"meshes":[${meshes_j.join(',')}],"accessors":[${accessors.join(',')}],"bufferViews":[${views.join(',')}],"buffers":[{"byteLength":${blob.len}}]${mat_json}}'
	mut out := []u8{}
	push_u32(mut out, u32(0x46546C67))
	push_u32(mut out, u32(2))
	mut jc := json_str.bytes()
	for jc.len % 4 != 0 {
		jc << u8(0x20)
	}
	mut bc := blob.clone()
	for bc.len % 4 != 0 {
		bc << u8(0)
	}
	push_u32(mut out, u32(12 + 8 + jc.len + 8 + bc.len))
	push_u32(mut out, u32(jc.len))
	push_u32(mut out, u32(0x4E4F534A))
	out << jc
	push_u32(mut out, u32(bc.len))
	push_u32(mut out, u32(0x004E4942))
	out << bc
	os.write_file(path, out.bytestr()) or { panic('cannot write ${path}') }
}

fn read_accessor(gltf &GltfRoot, bins [][]u8, idx int) []f64 {
	acc := gltf.accessors[idx]
	bv := gltf.buffer_views[acc.buffer_view]
	if bv.buffer < 0 || bv.buffer >= bins.len {
		panic('accessor ${idx} references missing buffer ${bv.buffer}')
	}
	buf := bins[bv.buffer]
	ncomp := match acc.typ {
		'SCALAR' { 1 }
		'VEC2' { 2 }
		'VEC3' { 3 }
		'VEC4' { 4 }
		else { panic('unsupported accessor type ${acc.typ}') }
	}
	base := bv.byte_offset + acc.byte_offset
	stride := if bv.byte_stride > 0 { bv.byte_stride } else { acc.component_type_bytes() * ncomp }
	mut out := []f64{cap: acc.count * ncomp}
	for i in 0 .. acc.count {
		o := base + i * stride
		for c in 0 .. ncomp {
			off := o + c * acc.component_type_bytes()
			v := match acc.component_type {
				5126 { f64(f32_at(buf, off)) }
				5125 { f64(binary.little_endian_u32_at(buf, off)) }
				5123 { f64(binary.little_endian_u16_at(buf, off)) }
				5121 { f64(buf[off]) }
				else { panic('unsupported componentType ${acc.component_type}') }
			}
			out << v
		}
	}
	return out
}

fn (a GltfAccessor) component_type_bytes() int {
	return match a.component_type {
		5121 { 1 }
		5123 { 2 }
		5125 { 4 }
		5126 { 4 }
		else { panic('unsupported componentType ${a.component_type}') }
	}
}

fn load_gltf_visit(gltf &GltfRoot, bins [][]u8, idx int, parent [16]f64, mut out []GltfMeshOut) {
	node := gltf.nodes[idx]
	world := mat4_mul(parent, node_local_matrix(node))
	if m := node.mesh {
		mesh := gltf.meshes[m]
		for prim in mesh.primitives {
			if prim.mode != 4 && prim.mode != 0 {
				continue
			}
			pos_idx := prim.attributes['POSITION'] or { panic('primitive missing POSITION') }
			pos := read_accessor(gltf, bins, pos_idx)
			mut verts := [][3]f64{}
			for i := 0; i < pos.len; i += 3 {
				verts << [pos[i], pos[i + 1], pos[i + 2]]!
			}
			mut raw_idx := []int{}
			if ii := prim.indices {
				idx_acc := read_accessor(gltf, bins, ii)
				for v in idx_acc {
					raw_idx << int(v)
				}
			} else {
				for i in 0 .. verts.len {
					raw_idx << i
				}
			}
			if raw_idx.len % 3 != 0 {
				panic('index count not a multiple of 3')
			}
			mut faces := [][3]int{}
			for i := 0; i < raw_idx.len; i += 3 {
				faces << [raw_idx[i], raw_idx[i + 1], raw_idx[i + 2]]!
			}
			out << GltfMeshOut{
				vertices: verts
				faces:    faces
				world:    world
			}
		}
	}
	for child in node.children {
		load_gltf_visit(gltf, bins, child, world, mut out)
	}
}

// gltf_to_geometry bakes loaded glTF meshes (world transforms applied) into a
// single Geometry (a union when the file holds several meshes).
pub fn gltf_to_geometry(outs []GltfMeshOut) Geometry {
	mut kids := []Geometry{}
	for o in outs {
		mut vv := [][3]f64{}
		for p in o.vertices {
			vv << transform_point(o.world, p)
		}
		kids << trimesh_geometry(vv, o.faces)
	}
	if kids.len == 0 {
		panic('glTF contains no meshes')
	}
	if kids.len == 1 {
		return kids[0]
	}
	return csg_geometry('union', kids)
}

// resolve_gltf_buffer returns the bytes for one glTF buffer.  `uri` may be empty
// (GLB embedded BIN chunk), a data URI (base64), or a path relative to the
// containing .gltf file.
fn resolve_gltf_buffer(path string, uri string, embedded []u8) []u8 {
	if uri == '' {
		return embedded
	}
	if uri.starts_with('data:') {
		comma := uri.index(',') or { panic('malformed data URI in glTF buffer') }
		return base64.decode(uri[comma + 1..])
	}
	dir := os.dir(path)
	full := if dir == '' || dir == '.' { uri } else { dir + '/' + uri }
	return os.read_bytes(full) or { panic('cannot read glTF buffer ${full}') }
}

// load_gltf reads a .glb (binary) or .gltf (JSON) file and returns
// [(vertices, faces, world_transform)].
pub fn load_gltf(path string) []GltfMeshOut {
	data := os.read_bytes(path) or { panic('cannot read ${path}') }
	mut json_text := ''
	mut bin_chunk := []u8{}
	if data.len >= 4 && binary.little_endian_u32(data) == 0x46546C67 {
		// GLB binary container: 12-byte header + JSON + optional BIN chunks.
		if binary.little_endian_u32_at(data, 4) != 2 {
			panic('${path}: only glTF 2.0 supported')
		}
		mut offset := 12
		for offset + 8 <= data.len {
			clen := int(binary.little_endian_u32_at(data, offset))
			ctype := binary.little_endian_u32_at(data, offset + 4)
			chunk := data[offset + 8..offset + 8 + clen]
			if ctype == 0x4E4F534A {
				json_text = chunk.bytestr()
			} else if ctype == 0x004E4942 {
				bin_chunk = chunk.clone()
			}
			offset += 8 + clen
		}
	} else {
		// Plain .gltf JSON document.
		json_text = data.bytestr()
	}
	gltf := json2.decode[GltfRoot](json_text) or { panic('bad glTF JSON: ${err}') }
	if gltf.scenes.len == 0 {
		return []
	}
	// Resolve every buffer once (bufferView.buffer indexes into this list).
	mut bins := [][]u8{len: gltf.buffers.len}
	for i, buf in gltf.buffers {
		bins[i] = resolve_gltf_buffer(path, buf.uri, bin_chunk)
	}
	scene_idx := if gltf.scene < gltf.scenes.len { gltf.scene } else { 0 }
	mut out := []GltfMeshOut{}
	for root in gltf.scenes[scene_idx].nodes {
		load_gltf_visit(&gltf, bins, root, mat4_identity(), mut out)
	}
	return out
}
