// GLB mesh round-trip demo: extrude an L-shape, save it as .glb (with a node
// transform), reload it, and render the loaded mesh.
// Run: cargo run -p cga-examples --bin demo_gltf
use cga_core::*;
use cga_gpu::*;

fn main() {
    // V: os.dir(@FILE) — Rust 版以仓库根目录为 CWD 运行
    let out = "examples/gltf";
    // L-shaped extrusion
    let (verts, faces) = extrude(
        &[
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [0.0, 2.0],
        ],
        0.8,
    );
    // row-major transform: translate y by 0.6
    let t: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.6, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];
    save_glb(
        &format!("{out}/demo_gltf.glb"),
        &[GltfMeshIn {
            vertices: verts,
            faces,
            transform: Some(t),
            color: Some([0.9, 0.45, 0.13]),
        }],
    );

    // reload and render the loaded mesh(es) (world transform baked in)
    let loaded = load_gltf(&format!("{out}/demo_gltf.glb")).unwrap();
    let mut sc = scene(None);
    sc.add_mesh(mesh(MeshParams {
        geometry: gltf_to_geometry(&loaded),
        material: standard_material(MaterialParams {
            color: color_hex(0xE67E22),
            roughness: 0.3,
            metalness: 0.15,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.0, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::PlaneGeometry(plane_geometry([0.0, 1.0, 0.0], 0.0)),
        material: standard_material(MaterialParams {
            color: color_hex(0x888888),
            roughness: 0.8,
            metalness: 0.0,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.0, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_light(directional_light(color_hex(0xFFFFFF), 0.7, [0.4, 1.0, 0.4]));
    sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.35));

    let mut camera = perspective_camera(
        40.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [3.0, 2.6, 3.6],
        [1.0, 1.0, 0.4],
        [0.0, 1.0, 0.0],
    );
    camera.look_at([1.0, 1.0, 0.4], None);
    let mut r = renderer(480, 360, 2, 3);
    let img = r.render(sc, camera);

    save_frame_png(&format!("{out}/demo_gltf.png"), &img);
}
