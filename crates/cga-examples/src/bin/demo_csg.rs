use cga_core::*;
use cga_gpu::*;

fn main() {
    let mut sc = Scene::new(None);

    sc.add_object(Object::new(ObjectParams {
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

    let diff = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Difference,
        vec![
            Geometry::BoxGeometry(BoxGeometry::new(1.6, 1.6, 1.6)),
            Geometry::SphereGeometry(SphereGeometry::new(0.55)),
        ],
    ));
    sc.add_object(Object::new(ObjectParams {
        geometry: diff,
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0xC0392B),
            roughness: 0.3,
            metalness: 0.1,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [-2.2, 0.8, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    let inter = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Intersection,
        vec![
            Geometry::BoxGeometry(BoxGeometry::new(1.6, 1.6, 1.6)),
            Geometry::SphereGeometry(SphereGeometry::new(1.0)),
        ],
    ));
    sc.add_object(Object::new(ObjectParams {
        geometry: inter,
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0x2980B9),
            roughness: 0.25,
            metalness: 0.2,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.8, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    let un = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Union,
        vec![
            Geometry::BoxGeometry(BoxGeometry::new(1.4, 1.4, 1.4)),
            Geometry::SphereGeometry(SphereGeometry::new(0.9)),
        ],
    ));
    sc.add_object(Object::new(ObjectParams {
        geometry: un,
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0x27AE60),
            roughness: 0.35,
            metalness: 0.05,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [2.2, 0.8, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    sc.add_light(Light::directional(
        Color::from_hex(0xFFFFFF),
        0.7,
        [0.5, 1.0, 0.4],
    ));
    sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.35));

    let mut camera = PerspectiveCamera::new(
        45.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [0.0, 2.4, 7.0],
        [0.0, 0.7, 0.0],
        [0.0, 1.0, 0.0],
    );
    camera.look_at([0.0, 0.7, 0.0], None);
    let mut renderer = Renderer::new(480, 360, 2, 3);
    let img = renderer.render(sc, camera);

    let out = "examples/csg";

    save_frame_png(&format!("{out}/demo_csg.png"), &img);
}
