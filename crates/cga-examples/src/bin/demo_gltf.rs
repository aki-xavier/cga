use cga_core::*;
use cga_gpu::*;

fn main() {
    let out = "examples/gltf";

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

    let t: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.6, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
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

    let loaded = load_gltf(&format!("{out}/demo_gltf.glb")).unwrap();
    let mut sc = Scene::new(None);
    sc.add_mesh(Mesh::new(MeshParams {
        geometry: gltf_to_geometry(&loaded),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0xE67E22),
            roughness: 0.3,
            metalness: 0.15,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.0, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_mesh(Mesh::new(MeshParams {
        geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0x888888),
            roughness: 0.8,
            metalness: 0.0,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.0, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_light(Light::directional(
        Color::from_hex(0xFFFFFF),
        0.7,
        [0.4, 1.0, 0.4],
    ));
    sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.35));

    let mut camera = PerspectiveCamera::new(
        40.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [3.0, 2.6, 3.6],
        [1.0, 1.0, 0.4],
        [0.0, 1.0, 0.0],
    );
    camera.look_at([1.0, 1.0, 0.4], None);
    let mut r = Renderer::new(480, 360, 2, 3);
    let img = r.render(sc, camera);

    save_frame_png(&format!("{out}/demo_gltf.png"), &img);
}
