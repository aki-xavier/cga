use cga_core::{save_obj, BakedMesh, ObjMesh};
use cga_gpu::{geom_to_camera, GltfMeshIn};
use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Input .jsx scene (imports of ./x.css are followed)
    src: String,
    /// Output .obj|.glb (default: <src> minus extension + .obj)
    out: Option<String>,
    /// Bake step size
    #[arg(default_value_t = 0.1)]
    step: f64,
}

fn main() {
    let args = Args::parse();
    let src = &args.src;
    let out = args.out.unwrap_or_else(|| match src.rfind('.') {
        Some(i) => format!("{}.obj", &src[..i]),
        None => format!("{src}.obj"),
    });
    let step = args.step;
    let text = std::fs::read_to_string(src).unwrap_or_else(|_| panic!("cannot read {src}"));
    let asset_root = match src.rfind('/') {
        Some(i) => &src[..i],
        None => ".",
    };
    let scene = cga_gpu::run_jsx(&text, None, asset_root)
        .unwrap_or_else(|e| panic!("{e}"))
        .scene;

    let mut verts: Vec<[f64; 3]> = Vec::new();
    let mut faces: Vec<[i32; 3]> = Vec::new();
    let mut skipped = 0u32;
    for m in &scene.objects {
        let params = geom_to_camera(&m.geometry, &m.motor());
        if params.bounds().is_none() {
            skipped += 1;
            continue;
        }
        let b: BakedMesh = match params.bake(step) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("bake: skip mesh({e})");
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
    let v = BakedMesh::volume_of(&verts, &faces).abs();
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
