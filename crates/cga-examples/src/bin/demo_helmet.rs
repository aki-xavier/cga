// DamagedHelmet demo: render the Khronos GLTF helmet as a solid-colour material
// (its base-color factor; no textures) alongside CGA analytic primitives (sphere
// / cylinder / ground plane).  The hybrid renderer rasterizes the triangle-mesh
// helmet and ray-traces the analytic primitives, then composites by depth.
// Run: cargo run -p cga-examples --bin demo_helmet
use cga_core::*;
use cga_gpu::*;

fn main() {
    // 以仓库根目录为 CWD 运行
    let out = "examples/helmet";
    let loaded = load_gltf(&format!("{out}/DamagedHelmet.glb")).unwrap();

    let mut sc = scene(None);
    // helmet: triangle mesh (rasterized); rendered as a solid grey
    sc.add_mesh(mesh(MeshParams {
        geometry: gltf_to_geometry(&loaded),
        material: standard_material(MaterialParams {
            color: color_hex(0x9E9E9E),
            roughness: 0.6,
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
    // CGA analytic primitives (ray traced): a red sphere and a gold cylinder
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::SphereGeometry(sphere_geometry(0.6)),
        material: standard_material(MaterialParams {
            color: color_hex(0xC0392B),
            roughness: 0.3,
            metalness: 0.1,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [2.2, 0.6, 0.6],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::CylinderGeometry(cylinder_geometry(0.4, 1.5)),
        material: standard_material(MaterialParams {
            color: color_hex(0xD4AF37),
            roughness: 0.4,
            metalness: 0.4,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [-2.2, 0.75, 0.5],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    // ground plane below everything (analytic -> ray traced)
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::PlaneGeometry(plane_geometry([0.0, 1.0, 0.0], -0.95)),
        material: standard_material(MaterialParams {
            color: color_hex(0x8A8A8A),
            roughness: 0.85,
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

    sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.6]));
    sc.add_light(point_light(color_hex(0xFFFFFF), 0.5, [-2.2, 2.4, 2.2]));
    sc.add_light(point_light(color_hex(0xFFE0B0), 0.3, [2.2, 1.2, -1.0]));
    sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.22));

    let mut camera = perspective_camera(
        45.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [0.0, 2.6, 6.6],
        [0.0, 0.3, -0.1],
        [0.0, 1.0, 0.0],
    );
    camera.look_at([0.0, 0.3, -0.1], None);
    let mut r = renderer(480, 360, 1, 3);
    let img = r.render(sc, camera);

    save_frame_png(&format!("{out}/demo_helmet.png"), &img);
    println!("saved {out}/demo_helmet.png");
}
