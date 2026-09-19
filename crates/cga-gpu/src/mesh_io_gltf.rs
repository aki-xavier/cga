// glTF 2.0 read + GLB write (geometry: TRIANGLES primitives; materials: solid
// colour from the PBR base color / metalness / roughness / emissive factors —
// no texturing).
//
// JSON is walked as `serde_json::Value`; missing fields default to zero values.
// Base64 data URIs use the `base64` crate.

use base64::{engine::general_purpose::STANDARD, Engine};
use serde_json::Value;

use crate::{color_rgb, standard_material, Material, MaterialParams};
use cga_core::{
    csg_geometry, from_column_major, from_trs, mat4_identity, mat4_mul, to_column_major,
    transform_point, trimesh_geometry, trimesh_geometry_uv, CsgOp, Geometry,
};

#[derive(Default, Debug)]
struct GltfAccessor {
    buffer_view: i32,
    component_type: i32,
    count: i32,
    typ: String,
    byte_offset: i32,
}

#[derive(Default, Debug)]
struct GltfBufferView {
    buffer: i32,
    byte_offset: i32,
    #[allow(dead_code)]
    // kept for schema completeness; read_accessor uses offsets/stride only
    byte_length: i32,
    byte_stride: i32,
}

#[derive(Default, Debug)]
struct GltfBuffer {
    uri: String,
}

#[derive(Default, Debug)]
struct GltfPrimitive {
    attributes: std::collections::HashMap<String, i32>,
    indices: Option<i32>,
    mode: i32,
    material: Option<i32>,
}

#[derive(Default, Debug)]
struct GltfMesh {
    primitives: Vec<GltfPrimitive>,
}

#[derive(Default, Debug)]
struct GltfNode {
    mesh: Option<i32>,
    matrix: Vec<f64>,
    translation: Vec<f64>,
    rotation: Vec<f64>,
    scale: Vec<f64>,
    children: Vec<i32>,
}

#[derive(Default, Debug)]
struct GltfScene {
    nodes: Vec<i32>,
}

#[derive(Default, Debug)]
struct GltfPbr {
    base_color_factor: Vec<f64>,
    metallic_factor: f64,
    roughness_factor: f64,
}

#[derive(Default, Debug)]
struct GltfMaterial {
    pbr: GltfPbr,
    emissive_factor: Vec<f64>,
}

#[derive(Default, Debug)]
struct GltfRoot {
    buffers: Vec<GltfBuffer>,
    buffer_views: Vec<GltfBufferView>,
    accessors: Vec<GltfAccessor>,
    meshes: Vec<GltfMesh>,
    nodes: Vec<GltfNode>,
    scenes: Vec<GltfScene>,
    scene: i32,
    materials: Vec<GltfMaterial>,
}

// GltfMeshOut is one loaded mesh (vertices, faces, world transform) plus its
// solid-colour PBR material (base colour + metalness / roughness / emissive) and
// per-vertex UVs.
#[derive(Clone, Debug)]
pub struct GltfMeshOut {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[i32; 3]>,
    pub world: [f64; 16],
    pub uv: Vec<[f64; 2]>, // parallel to vertices; empty when none
    pub color: [f64; 3],
    pub metalness: f64,
    pub roughness: f64,
    pub emissive: [f64; 3],
}

// GltfMeshIn is one input entry for save_glb.
#[derive(Clone, Debug)]
pub struct GltfMeshIn {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[i32; 3]>,
    pub transform: Option<[f64; 16]>,
    pub color: Option<[f64; 3]>,
}

fn push_f32(b: &mut Vec<u8>, v: f32) {
    push_u32(b, v.to_bits());
}

fn push_u32(b: &mut Vec<u8>, v: u32) {
    b.push((v & 0xff) as u8);
    b.push(((v >> 8) & 0xff) as u8);
    b.push(((v >> 16) & 0xff) as u8);
    b.push(((v >> 24) & 0xff) as u8);
}

fn align4(b: &mut Vec<u8>) {
    while !b.len().is_multiple_of(4) {
        b.push(0);
    }
}

fn f32_at(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}

fn node_local_matrix(n: &GltfNode) -> [f64; 16] {
    if n.matrix.len() == 16 {
        let mut m = [0.0f64; 16];
        m.copy_from_slice(&n.matrix);
        return from_column_major(m);
    }
    let t = if n.translation.len() == 3 {
        [n.translation[0], n.translation[1], n.translation[2]]
    } else {
        [0.0, 0.0, 0.0]
    };
    let r = if n.rotation.len() == 4 {
        [n.rotation[0], n.rotation[1], n.rotation[2], n.rotation[3]]
    } else {
        [0.0, 0.0, 0.0, 1.0]
    };
    let sc = if n.scale.len() == 3 {
        [n.scale[0], n.scale[1], n.scale[2]]
    } else {
        [1.0, 1.0, 1.0]
    };
    from_trs(t, r, sc)
}

// save_glb writes meshes as a GLB file (each mesh = one TRIANGLES primitive).
pub fn save_glb(path: &str, meshes: &[GltfMeshIn]) {
    let mut blob: Vec<u8> = Vec::new();
    let mut nodes: Vec<String> = Vec::new();
    let mut meshes_j: Vec<String> = Vec::new();
    let mut accessors: Vec<String> = Vec::new();
    let mut views: Vec<String> = Vec::new();
    let mut materials: Vec<String> = Vec::new();
    for (mi, m) in meshes.iter().enumerate() {
        if m.vertices.is_empty() || m.faces.is_empty() {
            panic!("mesh {mi}: empty vertices/faces");
        }
        align4(&mut blob);
        let pos_off = blob.len();
        for p in &m.vertices {
            push_f32(&mut blob, p[0] as f32);
            push_f32(&mut blob, p[1] as f32);
            push_f32(&mut blob, p[2] as f32);
        }
        let pos_len = blob.len() - pos_off;
        align4(&mut blob);
        let idx_off = blob.len();
        for f in &m.faces {
            push_u32(&mut blob, f[0] as u32);
            push_u32(&mut blob, f[1] as u32);
            push_u32(&mut blob, f[2] as u32);
        }
        let idx_len = blob.len() - idx_off;
        views.push(format!(
            "{{\"buffer\":0,\"byteOffset\":{pos_off},\"byteLength\":{pos_len}}}"
        ));
        views.push(format!(
            "{{\"buffer\":0,\"byteOffset\":{idx_off},\"byteLength\":{idx_len}}}"
        ));
        let mut mn = [1e30, 1e30, 1e30];
        let mut mx = [-1e30, -1e30, -1e30];
        for p in &m.vertices {
            for i in 0..3 {
                if p[i] < mn[i] {
                    mn[i] = p[i];
                }
                if p[i] > mx[i] {
                    mx[i] = p[i];
                }
            }
        }
        let pa = accessors.len();
        let iaa = accessors.len() + 1;
        accessors.push(format!(
            "{{\"bufferView\":{},\"componentType\":5126,\"count\":{},\"type\":\"VEC3\",\"min\":[{},{},{}],\"max\":[{},{},{}]}}",
            views.len() - 2,
            m.vertices.len(),
            mn[0],
            mn[1],
            mn[2],
            mx[0],
            mx[1],
            mx[2]
        ));
        accessors.push(format!(
            "{{\"bufferView\":{},\"componentType\":5125,\"count\":{},\"type\":\"SCALAR\"}}",
            views.len() - 1,
            m.faces.len() * 3
        ));
        let mut mat_ref = String::new();
        if let Some(c) = m.color {
            let mat_idx = materials.len();
            materials.push(format!(
                "{{\"pbrMetallicRoughness\":{{\"baseColorFactor\":[{},{},{},1.0]}}}}",
                c[0], c[1], c[2]
            ));
            mat_ref = format!(",\"material\":{mat_idx}");
        }
        meshes_j.push(format!(
            "{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{pa}}},\"indices\":{iaa},\"mode\":4{mat_ref}}}]}}"
        ));
        let mut node_extra = String::new();
        if let Some(tr) = m.transform {
            let cm = to_column_major(tr);
            node_extra = format!(
                ",\"matrix\":[{}]",
                cm.iter()
                    .map(|v| v.to_string())
                    .collect::<Vec<_>>()
                    .join(",")
            );
        }
        nodes.push(format!(
            "{{\"mesh\":{mi},\"name\":\"mesh_{mi}\"{node_extra}}}"
        ));
    }
    // The scene's `nodes` must be node *indices*, not the node objects.
    let mut node_indices: Vec<String> = Vec::with_capacity(nodes.len());
    for i in 0..nodes.len() {
        node_indices.push(i.to_string());
    }
    let mat_json = if !materials.is_empty() {
        format!(",\"materials\":[{}]", materials.join(","))
    } else {
        String::new()
    };
    let json_str = format!(
        "{{\"asset\":{{\"version\":\"2.0\",\"generator\":\"cga.mesh_io\"}},\"scene\":0,\"scenes\":[{{\"nodes\":[{}]}}],\"nodes\":[{}],\"meshes\":[{}],\"accessors\":[{}],\"bufferViews\":[{}],\"buffers\":[{{\"byteLength\":{}}}]{}}}",
        node_indices.join(","),
        nodes.join(","),
        meshes_j.join(","),
        accessors.join(","),
        views.join(","),
        blob.len(),
        mat_json
    );
    let mut out: Vec<u8> = Vec::new();
    push_u32(&mut out, 0x46546C67);
    push_u32(&mut out, 2);
    let mut jc = json_str.into_bytes();
    while jc.len() % 4 != 0 {
        jc.push(0x20);
    }
    let mut bc = blob.clone();
    while !bc.len().is_multiple_of(4) {
        bc.push(0);
    }
    push_u32(&mut out, (12 + 8 + jc.len() + 8 + bc.len()) as u32);
    push_u32(&mut out, jc.len() as u32);
    push_u32(&mut out, 0x4E4F534A);
    out.extend_from_slice(&jc);
    push_u32(&mut out, bc.len() as u32);
    push_u32(&mut out, 0x004E4942);
    out.extend_from_slice(&bc);
    std::fs::write(path, out).unwrap_or_else(|_| panic!("cannot write {path}"));
}

fn read_accessor(gltf: &GltfRoot, bins: &[Vec<u8>], idx: usize) -> Vec<f64> {
    let acc = &gltf.accessors[idx];
    let bv = &gltf.buffer_views[acc.buffer_view as usize];
    if bv.buffer < 0 || bv.buffer as usize >= bins.len() {
        panic!("accessor {idx} references missing buffer {}", bv.buffer);
    }
    let buf = &bins[bv.buffer as usize];
    let ncomp = match acc.typ.as_str() {
        "SCALAR" => 1,
        "VEC2" => 2,
        "VEC3" => 3,
        "VEC4" => 4,
        _ => panic!("unsupported accessor type {}", acc.typ),
    };
    let base = (bv.byte_offset + acc.byte_offset) as usize;
    let stride = if bv.byte_stride > 0 {
        bv.byte_stride as usize
    } else {
        acc.component_type_bytes() as usize * ncomp
    };
    let mut out: Vec<f64> = Vec::with_capacity(acc.count as usize * ncomp);
    for i in 0..acc.count as usize {
        let o = base + i * stride;
        for c in 0..ncomp {
            let off = o + c * acc.component_type_bytes() as usize;
            let v = match acc.component_type {
                5126 => f32_at(buf, off) as f64,
                5125 => u32::from_le_bytes(buf[off..off + 4].try_into().unwrap()) as f64,
                5123 => u16::from_le_bytes(buf[off..off + 2].try_into().unwrap()) as f64,
                5121 => buf[off] as f64,
                _ => panic!("unsupported componentType {}", acc.component_type),
            };
            out.push(v);
        }
    }
    out
}

impl GltfAccessor {
    fn component_type_bytes(&self) -> i32 {
        match self.component_type {
            5121 => 1,
            5123 => 2,
            5125 => 4,
            5126 => 4,
            _ => panic!("unsupported componentType {}", self.component_type),
        }
    }
}

fn load_gltf_visit(
    gltf: &GltfRoot,
    bins: &[Vec<u8>],
    _path: &str,
    idx: usize,
    parent: [f64; 16],
    out: &mut Vec<GltfMeshOut>,
) {
    let node = &gltf.nodes[idx];
    let world = mat4_mul(parent, node_local_matrix(node));
    if let Some(m) = node.mesh {
        let mob = &gltf.meshes[m as usize];
        for prim in &mob.primitives {
            if prim.mode != 4 && prim.mode != 0 {
                continue;
            }
            let pos_idx = *prim
                .attributes
                .get("POSITION")
                .unwrap_or_else(|| panic!("primitive missing POSITION"));
            let pos = read_accessor(gltf, bins, pos_idx as usize);
            let mut verts: Vec<[f64; 3]> = Vec::new();
            let mut i = 0;
            while i < pos.len() {
                verts.push([pos[i], pos[i + 1], pos[i + 2]]);
                i += 3;
            }
            // per-vertex UVs (TEXCOORD_0, VEC2) when present
            let mut uvs: Vec<[f64; 2]> = Vec::new();
            if let Some(uv_idx) = prim.attributes.get("TEXCOORD_0") {
                let uvdata = read_accessor(gltf, bins, *uv_idx as usize);
                let mut i = 0;
                while i + 1 < uvdata.len() {
                    uvs.push([uvdata[i], uvdata[i + 1]]);
                    i += 2;
                }
            }
            let mut raw_idx: Vec<i32> = Vec::new();
            if let Some(ii) = prim.indices {
                let idx_acc = read_accessor(gltf, bins, ii as usize);
                for v in idx_acc {
                    raw_idx.push(v as i32);
                }
            } else {
                for i in 0..verts.len() {
                    raw_idx.push(i as i32);
                }
            }
            if !raw_idx.len().is_multiple_of(3) {
                panic!("index count not a multiple of 3");
            }
            let mut faces: Vec<[i32; 3]> = Vec::new();
            let mut i = 0;
            while i < raw_idx.len() {
                faces.push([raw_idx[i], raw_idx[i + 1], raw_idx[i + 2]]);
                i += 3;
            }
            // material: solid colour from the glTF PBR params (no textures).  The
            // base colour comes from baseColorFactor, with a small default metalness
            // so the mesh still shades visibly under direct lighting.
            let mut color = [1.0, 1.0, 1.0];
            let mut metalness = 0.05;
            let mut roughness = 0.5;
            let mut emissive = [0.0, 0.0, 0.0];
            if let Some(mat_ptr) = prim.material {
                if (mat_ptr as usize) < gltf.materials.len() {
                    let mat = &gltf.materials[mat_ptr as usize];
                    if mat.pbr.base_color_factor.len() >= 3 {
                        color = [
                            mat.pbr.base_color_factor[0],
                            mat.pbr.base_color_factor[1],
                            mat.pbr.base_color_factor[2],
                        ];
                    }
                    metalness = 0.2_f64.min(mat.pbr.metallic_factor);
                    roughness = if mat.pbr.roughness_factor > 0.05 {
                        mat.pbr.roughness_factor
                    } else {
                        0.5
                    };
                    if mat.emissive_factor.len() >= 3 {
                        emissive = [
                            mat.emissive_factor[0],
                            mat.emissive_factor[1],
                            mat.emissive_factor[2],
                        ];
                    }
                }
            }
            out.push(GltfMeshOut {
                vertices: verts,
                faces,
                world,
                uv: uvs,
                color,
                metalness,
                roughness,
                emissive,
            });
        }
    }
    for child in &node.children {
        load_gltf_visit(gltf, bins, _path, *child as usize, world, out);
    }
}

// gltf_to_geometry bakes loaded glTF meshes (world transforms applied) into a
// single Geometry (a union when the file holds several meshes).
pub fn gltf_to_geometry(outs: &[GltfMeshOut]) -> Geometry {
    let mut kids: Vec<Geometry> = Vec::new();
    for o in outs {
        let mut vv: Vec<[f64; 3]> = Vec::new();
        for p in &o.vertices {
            vv.push(transform_point(o.world, *p));
        }
        kids.push(if !o.uv.is_empty() {
            Geometry::TrimeshGeometry(trimesh_geometry_uv(&vv, &o.faces, &o.uv))
        } else {
            Geometry::TrimeshGeometry(trimesh_geometry(&vv, &o.faces))
        });
    }
    if kids.is_empty() {
        panic!("glTF contains no meshes");
    }
    if kids.len() == 1 {
        return kids.into_iter().next().unwrap();
    }
    Geometry::CsgGeometry(csg_geometry(CsgOp::Union, kids))
}

// gltf_material builds a solid-colour Material from a loaded GltfMeshOut using
// its base colour + metalness / roughness / emissive (no textures are loaded).
pub fn gltf_material(o: GltfMeshOut) -> Material {
    standard_material(MaterialParams {
        color: color_rgb(o.color[0], o.color[1], o.color[2]),
        roughness: o.roughness,
        metalness: o.metalness,
        emissive: color_rgb(o.emissive[0], o.emissive[1], o.emissive[2]),
        opacity: 1.0,
        ior: 1.5,
        absorption: 0.0,
    })
}

// resolve_gltf_buffer returns the bytes for one glTF buffer.  `uri` may be empty
// (GLB embedded BIN chunk), a data URI (base64), or a path relative to the
// containing .gltf file.
fn resolve_gltf_buffer(path: &str, uri: &str, embedded: &[u8]) -> Result<Vec<u8>, String> {
    if uri.is_empty() {
        return Ok(embedded.to_vec());
    }
    if uri.starts_with("data:") {
        let comma = uri
            .find(',')
            .ok_or_else(|| "malformed data URI in glTF buffer".to_string())?;
        return STANDARD
            .decode(&uri[comma + 1..])
            .map_err(|e| e.to_string());
    }
    let dir = match path.rfind('/') {
        Some(i) => &path[..i],
        None => ".",
    };
    let full = if dir.is_empty() || dir == "." {
        uri.to_string()
    } else {
        format!("{dir}/{uri}")
    };
    std::fs::read(&full).map_err(|_| format!("cannot read glTF buffer {full}"))
}

// --- JSON parsing (serde_json::Value walk, zero-value defaults) ---------------

fn j_int(v: &Value, key: &str) -> i32 {
    v.get(key).and_then(|x| x.as_i64()).unwrap_or(0) as i32
}

fn j_f64(v: &Value, key: &str) -> f64 {
    v.get(key).and_then(|x| x.as_f64()).unwrap_or(0.0)
}

fn j_opt_int(v: &Value, key: &str) -> Option<i32> {
    v.get(key).and_then(|x| x.as_i64()).map(|x| x as i32)
}

fn j_string(v: &Value, key: &str) -> String {
    v.get(key)
        .and_then(|x| x.as_str())
        .unwrap_or("")
        .to_string()
}

fn j_f64_array(v: &Value, key: &str) -> Vec<f64> {
    match v.get(key).and_then(|x| x.as_array()) {
        Some(a) => a.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect(),
        None => Vec::new(),
    }
}

fn j_int_array(v: &Value, key: &str) -> Vec<i32> {
    match v.get(key).and_then(|x| x.as_array()) {
        Some(a) => a.iter().map(|x| x.as_i64().unwrap_or(0) as i32).collect(),
        None => Vec::new(),
    }
}

fn j_array<'a>(v: &'a Value, key: &str) -> &'a [Value] {
    match v.get(key).and_then(|x| x.as_array()) {
        Some(a) => a.as_slice(),
        None => &[],
    }
}

fn parse_gltf_root(json_text: &str) -> Result<GltfRoot, String> {
    let v: Value = serde_json::from_str(json_text).map_err(|e| e.to_string())?;
    let mut root = GltfRoot {
        scene: j_int(&v, "scene"),
        ..Default::default()
    };
    for b in j_array(&v, "buffers") {
        root.buffers.push(GltfBuffer {
            uri: j_string(b, "uri"),
        });
    }
    for bv in j_array(&v, "bufferViews") {
        root.buffer_views.push(GltfBufferView {
            buffer: j_int(bv, "buffer"),
            byte_offset: j_int(bv, "byteOffset"),
            byte_length: j_int(bv, "byteLength"),
            byte_stride: j_int(bv, "byteStride"),
        });
    }
    for a in j_array(&v, "accessors") {
        root.accessors.push(GltfAccessor {
            buffer_view: j_int(a, "bufferView"),
            component_type: j_int(a, "componentType"),
            count: j_int(a, "count"),
            typ: j_string(a, "type"),
            byte_offset: j_int(a, "byteOffset"),
        });
    }
    for m in j_array(&v, "meshes") {
        let mut mesh = GltfMesh::default();
        for p in j_array(m, "primitives") {
            let mut attributes = std::collections::HashMap::new();
            if let Some(obj) = p.get("attributes").and_then(|x| x.as_object()) {
                for (k, val) in obj {
                    attributes.insert(k.clone(), val.as_i64().unwrap_or(0) as i32);
                }
            }
            mesh.primitives.push(GltfPrimitive {
                attributes,
                indices: j_opt_int(p, "indices"),
                mode: j_int(p, "mode"),
                material: j_opt_int(p, "material"),
            });
        }
        root.meshes.push(mesh);
    }
    for n in j_array(&v, "nodes") {
        root.nodes.push(GltfNode {
            mesh: j_opt_int(n, "mesh"),
            matrix: j_f64_array(n, "matrix"),
            translation: j_f64_array(n, "translation"),
            rotation: j_f64_array(n, "rotation"),
            scale: j_f64_array(n, "scale"),
            children: j_int_array(n, "children"),
        });
    }
    for s in j_array(&v, "scenes") {
        root.scenes.push(GltfScene {
            nodes: j_int_array(s, "nodes"),
        });
    }
    for m in j_array(&v, "materials") {
        let null = Value::Null;
        let pbr_v = m.get("pbrMetallicRoughness").unwrap_or(&null);
        root.materials.push(GltfMaterial {
            pbr: GltfPbr {
                base_color_factor: j_f64_array(pbr_v, "baseColorFactor"),
                metallic_factor: j_f64(pbr_v, "metallicFactor"),
                roughness_factor: j_f64(pbr_v, "roughnessFactor"),
            },
            emissive_factor: j_f64_array(m, "emissiveFactor"),
        });
    }
    Ok(root)
}

// load_gltf reads a .glb (binary) or .gltf (JSON) file and returns
// [(vertices, faces, world_transform)].
pub fn load_gltf(path: &str) -> Result<Vec<GltfMeshOut>, String> {
    let data = std::fs::read(path).map_err(|_| format!("cannot read {path}"))?;
    let mut json_text = String::new();
    let mut bin_chunk: Vec<u8> = Vec::new();
    if data.len() >= 4 && u32::from_le_bytes(data[0..4].try_into().unwrap()) == 0x46546C67 {
        // GLB binary container: 12-byte header + JSON + optional BIN chunks.
        if u32::from_le_bytes(data[4..8].try_into().unwrap()) != 2 {
            return Err(format!("{path}: only glTF 2.0 supported"));
        }
        let mut offset = 12usize;
        while offset + 8 <= data.len() {
            let clen = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as usize;
            let ctype = u32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap());
            let chunk = &data[offset + 8..offset + 8 + clen];
            if ctype == 0x4E4F534A {
                json_text = String::from_utf8_lossy(chunk).into_owned();
            } else if ctype == 0x004E4942 {
                bin_chunk = chunk.to_vec();
            }
            offset += 8 + clen;
        }
    } else {
        // Plain .gltf JSON document.
        json_text = String::from_utf8_lossy(&data).into_owned();
    }
    let gltf = parse_gltf_root(&json_text).map_err(|e| format!("bad glTF JSON: {e}"))?;
    if gltf.scenes.is_empty() {
        return Ok(Vec::new());
    }
    // Resolve every buffer once (bufferView.buffer indexes into this list).
    let mut bins: Vec<Vec<u8>> = vec![Vec::new(); gltf.buffers.len()];
    for (i, buf) in gltf.buffers.iter().enumerate() {
        bins[i] = resolve_gltf_buffer(path, &buf.uri, &bin_chunk)?;
    }
    let scene_idx = if (gltf.scene as usize) < gltf.scenes.len() {
        gltf.scene as usize
    } else {
        0
    };
    let mut out: Vec<GltfMeshOut> = Vec::new();
    for root in gltf.scenes[scene_idx].nodes.clone() {
        load_gltf_visit(&gltf, &bins, path, root as usize, mat4_identity(), &mut out);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tetra() -> (Vec<[f64; 3]>, Vec<[i32; 3]>) {
        let verts = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let faces = vec![[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]];
        (verts, faces)
    }

    #[test]
    fn test_glb_roundtrip() {
        let (verts, faces) = tetra();
        save_glb(
            "/tmp/cga_roundtrip.glb",
            &[GltfMeshIn {
                vertices: verts.clone(),
                faces: faces.clone(),
                transform: None,
                color: None,
            }],
        );
        let out = load_gltf("/tmp/cga_roundtrip.glb").unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].vertices.len(), 4);
        assert_eq!(out[0].faces.len(), 4);
        for (ov, v) in out[0].vertices.iter().zip(&verts) {
            assert!((ov[0] - v[0]).abs() < 1e-5);
            assert!((ov[1] - v[1]).abs() < 1e-5);
            assert!((ov[2] - v[2]).abs() < 1e-5);
        }
        for (of, f) in out[0].faces.iter().zip(&faces) {
            assert_eq!(of, f);
        }
        std::fs::remove_file("/tmp/cga_roundtrip.glb").ok();
    }

    #[test]
    fn test_gltf_json_roundtrip() {
        // plain .gltf (JSON) with an external .bin buffer, like a typical exporter
        let (verts, faces) = tetra();
        let mut bin: Vec<u8> = Vec::new();
        for p in &verts {
            push_f32(&mut bin, p[0] as f32);
            push_f32(&mut bin, p[1] as f32);
            push_f32(&mut bin, p[2] as f32);
        }
        let pos_len = bin.len();
        for f in &faces {
            push_u32(&mut bin, f[0] as u32);
            push_u32(&mut bin, f[1] as u32);
            push_u32(&mut bin, f[2] as u32);
        }
        let idx_len = bin.len() - pos_len;
        std::fs::write("/tmp/cga_tetra.bin", &bin).expect("write bin");
        let json_str = format!(
            "{{\"asset\":{{\"version\":\"2.0\"}},\"scene\":0,\"scenes\":[{{\"nodes\":[0]}}],\"nodes\":[{{\"mesh\":0}}],\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":0}},\"indices\":1,\"mode\":4}}]}}],\"buffers\":[{{\"uri\":\"cga_tetra.bin\",\"byteLength\":{}}}],\"bufferViews\":[{{\"buffer\":0,\"byteOffset\":0,\"byteLength\":{pos_len}}},{{\"buffer\":0,\"byteOffset\":{pos_len},\"byteLength\":{idx_len}}}],\"accessors\":[{{\"bufferView\":0,\"componentType\":5126,\"count\":{},\"type\":\"VEC3\"}},{{\"bufferView\":1,\"componentType\":5125,\"count\":{},\"type\":\"SCALAR\"}}]}}",
            bin.len(),
            verts.len(),
            faces.len() * 3
        );
        std::fs::write("/tmp/cga_tetra.gltf", json_str).expect("write gltf");
        let out = load_gltf("/tmp/cga_tetra.gltf").unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].vertices.len(), 4);
        assert_eq!(out[0].faces.len(), 4);
        for (ov, v) in out[0].vertices.iter().zip(&verts) {
            assert!((ov[0] - v[0]).abs() < 1e-5);
            assert!((ov[1] - v[1]).abs() < 1e-5);
            assert!((ov[2] - v[2]).abs() < 1e-5);
        }
        for (of, f) in out[0].faces.iter().zip(&faces) {
            assert_eq!(of, f);
        }
        std::fs::remove_file("/tmp/cga_tetra.gltf").ok();
        std::fs::remove_file("/tmp/cga_tetra.bin").ok();
    }

    #[test]
    fn test_glb_color_material() {
        let (verts, faces) = tetra();
        save_glb(
            "/tmp/cga_color.glb",
            &[GltfMeshIn {
                vertices: verts,
                faces,
                transform: None,
                color: Some([0.8, 0.2, 0.2]),
            }],
        );
        // the color must be written as a baseColorFactor material
        let data = std::fs::read("/tmp/cga_color.glb").expect("read glb");
        let text = String::from_utf8_lossy(&data);
        assert!(text.contains("baseColorFactor"));
        assert!(text.contains("0.8,0.2,0.2"));
        // and the file must still load back
        let out = load_gltf("/tmp/cga_color.glb").unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].vertices.len(), 4);
        std::fs::remove_file("/tmp/cga_color.glb").ok();
    }

    // test_gltf_solid_color_material loads a real asset and checks the loader no
    // longer decodes textures — the resulting Material is a plain solid colour built
    // from the PBR factor (no `map`), so meshes render uniformly.
    #[test]
    fn test_gltf_solid_color_material() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/helmet/DamagedHelmet.glb"
        );
        let out = load_gltf(path).unwrap();
        assert_eq!(out.len(), 1);
        let m = out[0].clone();
        // the mesh still carries geometry + per-vertex UVs
        assert!(!m.uv.is_empty());
        // and the Material the renderer consumes is untextured solid colour
        let mat = gltf_material(m);
        assert!(mat.map.is_none());
        assert!(mat.metalness >= 0.0 && mat.metalness <= 1.0);
    }
}
