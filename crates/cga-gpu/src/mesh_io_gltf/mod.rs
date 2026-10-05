use crate::scene_graph::Color;
use crate::{Material, MaterialParams};
use cga_core::{
    from_column_major, mat4_identity, mat4_mul, to_column_major, transform_point, CsgGeometry,
    CsgOp, Geometry, TrimeshGeometry,
};

pub mod gltf_mesh_out;
pub use self::gltf_mesh_out::*;
pub mod gltf_mesh_in;
pub use self::gltf_mesh_in::*;

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

fn node_world_matrix(node: &gltf::Node, parent: [f64; 16]) -> [f64; 16] {
    let local = match node.transform() {
        gltf::scene::Transform::Matrix { matrix } => {
            let mut flat = [0.0f64; 16];
            for c in 0..4 {
                for r in 0..4 {
                    flat[c * 4 + r] = matrix[c][r] as f64;
                }
            }
            from_column_major(flat)
        }
        gltf::scene::Transform::Decomposed {
            translation,
            rotation,
            scale,
        } => cga_core::from_trs(
            [
                translation[0] as f64,
                translation[1] as f64,
                translation[2] as f64,
            ],
            [
                rotation[0] as f64,
                rotation[1] as f64,
                rotation[2] as f64,
                rotation[3] as f64,
            ],
            [scale[0] as f64, scale[1] as f64, scale[2] as f64],
        ),
    };
    mat4_mul(parent, local)
}

fn load_gltf_visit(
    document: &gltf::Document,
    buffers: &Vec<Vec<u8>>,
    idx: usize,
    parent: [f64; 16],
    out: &mut Vec<GltfMeshOut>,
) {
    let node = document.nodes().nth(idx).unwrap();
    let world = node_world_matrix(&node, parent);
    if let Some(mesh) = node.mesh() {
        for prim in mesh.primitives() {
            if prim.mode() != gltf::mesh::Mode::Triangles {
                continue;
            }
            let reader = prim.reader(|b| buffers.get(b.index()).map(|data| &data[..]));
            let pos_reader = reader
                .read_positions()
                .unwrap_or_else(|| panic!("primitive missing POSITION"));
            let mut verts: Vec<[f64; 3]> = Vec::new();
            for p in pos_reader {
                verts.push([p[0] as f64, p[1] as f64, p[2] as f64]);
            }
            let mut uvs: Vec<[f64; 2]> = Vec::new();
            if let Some(gltf::mesh::util::ReadTexCoords::F32(tc)) = reader.read_tex_coords(0) {
                for uv in tc {
                    uvs.push([uv[0] as f64, uv[1] as f64]);
                }
            }
            let mut raw_idx: Vec<i32> = Vec::new();
            if let Some(indices) = reader.read_indices() {
                use gltf::mesh::util::ReadIndices::*;
                match indices {
                    U8(it) => raw_idx.extend(it.map(|i| i as i32)),
                    U16(it) => raw_idx.extend(it.map(|i| i as i32)),
                    U32(it) => raw_idx.extend(it.map(|i| i as i32)),
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

            let mat = prim.material();
            let pbr = mat.pbr_metallic_roughness();
            let base = pbr.base_color_factor();
            let color = [base[0] as f64, base[1] as f64, base[2] as f64];
            let metalness = 0.2_f64.min(pbr.metallic_factor() as f64);
            let rf = pbr.roughness_factor() as f64;
            let roughness = if rf > 0.05 { rf } else { 0.5 };
            let em = mat.emissive_factor();
            let emissive = [em[0] as f64, em[1] as f64, em[2] as f64];
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
    for child in node.children() {
        load_gltf_visit(document, buffers, child.index(), world, out);
    }
}

pub fn gltf_to_geometry(outs: &[GltfMeshOut]) -> Geometry {
    let mut kids: Vec<Geometry> = Vec::new();
    for o in outs {
        let mut vv: Vec<[f64; 3]> = Vec::new();
        for p in &o.vertices {
            vv.push(transform_point(o.world, *p));
        }
        kids.push(if !o.uv.is_empty() {
            Geometry::TrimeshGeometry(TrimeshGeometry::with_uv(&vv, &o.faces, &o.uv))
        } else {
            Geometry::TrimeshGeometry(TrimeshGeometry::new(&vv, &o.faces))
        });
    }
    if kids.is_empty() {
        panic!("glTF contains no meshes");
    }
    if kids.len() == 1 {
        return kids.into_iter().next().unwrap();
    }
    Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Union, kids))
}

pub fn load_gltf(path: &str) -> Result<Vec<GltfMeshOut>, String> {
    let (document, buffers, _images) = gltf::import(path).map_err(|e| format!("bad glTF: {e}"))?;
    let buffers: Vec<Vec<u8>> = buffers.into_iter().map(|b| b.0).collect();
    let scene = document
        .default_scene()
        .or_else(|| document.scenes().next());
    let mut out: Vec<GltfMeshOut> = Vec::new();
    let Some(scene) = scene else {
        return Ok(out);
    };
    for root in scene.nodes() {
        load_gltf_visit(&document, &buffers, root.index(), mat4_identity(), &mut out);
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

        let data = std::fs::read("/tmp/cga_color.glb").expect("read glb");
        let text = String::from_utf8_lossy(&data);
        assert!(text.contains("baseColorFactor"));
        assert!(text.contains("0.8,0.2,0.2"));

        let out = load_gltf("/tmp/cga_color.glb").unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].vertices.len(), 4);
        std::fs::remove_file("/tmp/cga_color.glb").ok();
    }

    #[test]
    fn test_gltf_solid_color_material() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/helmet/DamagedHelmet.glb"
        );
        let out = load_gltf(path).unwrap();
        assert_eq!(out.len(), 1);
        let m = out[0].clone();

        assert!(!m.uv.is_empty());

        let mat = m.material();
        assert!(mat.map.is_none());
        assert!(mat.metalness >= 0.0 && mat.metalness <= 1.0);
    }
}
