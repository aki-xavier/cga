use cga_core::*;
use cga_gpu::*;

fn main() {
    let mut sc = scene(None);

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

    let diff = Geometry::CsgGeometry(csg_geometry(
        CsgOp::Difference,
        vec![
            Geometry::BoxGeometry(box_geometry(1.6, 1.6, 1.6)),
            Geometry::SphereGeometry(sphere_geometry(0.55)),
        ],
    ));
    sc.add_mesh(mesh(MeshParams {
        geometry: diff,
        material: standard_material(MaterialParams {
            color: color_hex(0xC0392B),
            roughness: 0.3,
            metalness: 0.1,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [-2.2, 0.8, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    let inter = Geometry::CsgGeometry(csg_geometry(
        CsgOp::Intersection,
        vec![
            Geometry::BoxGeometry(box_geometry(1.6, 1.6, 1.6)),
            Geometry::SphereGeometry(sphere_geometry(1.0)),
        ],
    ));
    sc.add_mesh(mesh(MeshParams {
        geometry: inter,
        material: standard_material(MaterialParams {
            color: color_hex(0x2980B9),
            roughness: 0.25,
            metalness: 0.2,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.8, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    let un = Geometry::CsgGeometry(csg_geometry(
        CsgOp::Union,
        vec![
            Geometry::BoxGeometry(box_geometry(1.4, 1.4, 1.4)),
            Geometry::SphereGeometry(sphere_geometry(0.9)),
        ],
    ));
    sc.add_mesh(mesh(MeshParams {
        geometry: un,
        material: standard_material(MaterialParams {
            color: color_hex(0x27AE60),
            roughness: 0.35,
            metalness: 0.05,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [2.2, 0.8, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    sc.add_light(directional_light(color_hex(0xFFFFFF), 0.7, [0.5, 1.0, 0.4]));
    sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.35));

    let mut camera = perspective_camera(
        45.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [0.0, 2.4, 7.0],
        [0.0, 0.7, 0.0],
        [0.0, 1.0, 0.0],
    );
    camera.look_at([0.0, 0.7, 0.0], None);
    let mut renderer = renderer(480, 360, 2, 3);
    let img = renderer.render(sc, camera);

    let out = "examples/csg";

    save_frame_png(&format!("{out}/demo_csg.png"), &img);
}
