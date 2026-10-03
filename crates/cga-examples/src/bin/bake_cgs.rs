//! bake_cgs: CGS file -> triangle soup -> OBJ or GLB (world frame).
//!
//! usage: bake_cgs <file.cgs> [out.obj|out.glb] [step]
//! Unbounded meshes (ground planes) are skipped with a warning.

use cga_core::{bake, bounds_of, mesh_volume, save_obj, BakedMesh, ObjMesh};
use cga_gpu::{geom_to_camera, GltfMeshIn};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: bake_cgs <file.cgs> [out.obj|out.glb] [step]");
        std::process::exit(1);
    }
    let src = &args[1];
    let out = if args.len() > 2 {
        args[2].clone()
    } else {
        match src.rfind('.') {
            Some(i) => format!("{}.obj", &src[..i]),
            None => format!("{src}.obj"),
        }
    };
    let step: f64 = if args.len() > 3 {
        args[3].parse().unwrap_or(0.1)
    } else {
        0.1
    };
    let text = std::fs::read_to_string(src).unwrap_or_else(|_| panic!("cannot read {src}"));
    let asset_root = match src.rfind('/') {
        Some(i) => &src[..i],
        None => ".",
    };
    let (scene, _) = cga_gpu::cgs_load(&text, asset_root);

    let mut verts: Vec<[f64; 3]> = Vec::new();
    let mut faces: Vec<[i32; 3]> = Vec::new();
    let mut skipped = 0u32;
    for m in &scene.objects {
        // world-frame params, same resolution as the renderer uses for UVs
        let params = geom_to_camera(&m.geometry, &m.motor());
        if bounds_of(&params).is_none() {
            skipped += 1;
            continue;
        }
        let b: BakedMesh = match bake(&params, step) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("bake: skip mesh ({e})");
                skipped += 1;
                continue;
            }
        };
        let base = verts.len() as i32;
        verts.extend_from_slice(&b.vertices);
        for f in &b.faces {
            faces.push([f[0] + base, f[1] + base, f[2] + base]);
        }
    }
    let v = mesh_volume(&verts, &faces).abs();
    if out.ends_with(".glb") {
        cga_gpu::save_glb(
            &out,
            &[GltfMeshIn {
                vertices: verts,
                faces,
                transform: None,
                color: None,
            }],
        );
    } else {
        save_obj(
            &out,
            &[ObjMesh {
                vertices: verts,
                faces,
                has_transform: false,
                transform: cga_core::mat4_identity(),
            }],
        );
    }
    println!("saved {out} volume~{v:.3} skipped={skipped}");
}
