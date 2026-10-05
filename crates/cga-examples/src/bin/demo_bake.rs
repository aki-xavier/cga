use cga_core::*;
use cga_gpu::*;

fn std_mat(color: i32, roughness: f64, metalness: f64) -> Material {
    Material::standard(MaterialParams {
        color: Color::from_hex(color),
        roughness,
        metalness,
        emissive: Color::from_hex(0x000000),
        opacity: 1.0,
        ior: 1.5,
        absorption: 0.0,
    })
}

fn place(geometry: Geometry, color: i32, roughness: f64, metalness: f64, x: f64) -> Mesh {
    Mesh::new(MeshParams {
        geometry,
        material: std_mat(color, roughness, metalness),
        position: [x, 0.0, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    })
}

fn main() {
    // 隐式 CSG 原件: 盒 − 球(顶面开碟) − 通孔
    let csg = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Difference,
        vec![
            Geometry::BoxGeometry(BoxGeometry::new(2.2, 1.4, 1.4)),
            Geometry::SphereGeometry(SphereGeometry::new(0.95)),
            Geometry::CylinderGeometry(CylinderGeometry::new(0.28, 3.0)),
        ],
    ));

    // 烘焙: 有符号场 → marching tetrahedra → 位精确焊接 + 一致定向(纯 CPU f64)
    let baked = csg.identity_params().bake(0.04).expect("bake");
    let rep = baked.topology_report();
    println!("topology: {rep:?}");
    assert!(rep.is_watertight(), "baked mesh must be watertight");
    assert!(rep.is_consistently_oriented());
    println!("volume: {:.4}", baked.volume().abs());
    let mesh_geo = Geometry::TrimeshGeometry(TrimeshGeometry::new(&baked.vertices, &baked.faces));

    let mut sc = Scene::new(Some(Color::from_hex(0x2B3138)));
    sc.add_mesh(place(
        Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], -0.71)),
        0x3A4046,
        0.9,
        0.0,
        0.0,
    ));
    sc.add_mesh(place(csg, 0xC0392B, 0.3, 0.1, -1.5));
    sc.add_mesh(place(mesh_geo, 0x2980B9, 0.45, 0.1, 1.5));
    sc.add_light(Light::directional(
        Color::from_hex(0xFFFFFF),
        0.55,
        [0.4, 1.0, 0.35],
    ));
    sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.35));

    let mut camera = PerspectiveCamera::new(
        45.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [0.0, 2.6, 7.6],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    );
    camera.look_at([0.0, 0.0, 0.0], None);
    let mut renderer = Renderer::new(640, 480, 2, 3);
    let img = renderer.render(sc, camera);

    let out = "examples/bake";
    let _ = std::fs::create_dir_all(out);
    save_frame_png(&format!("{out}/demo_bake.png"), &img);
    save_obj(
        &format!("{out}/demo_bake.obj"),
        &[ObjMesh {
            vertices: baked.vertices,
            faces: baked.faces,
            has_transform: false,
            transform: mat4_identity(),
        }],
    );
    println!("saved {out}/demo_bake.png + demo_bake.obj");
}
