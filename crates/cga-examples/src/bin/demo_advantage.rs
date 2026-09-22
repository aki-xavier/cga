use cga_core::*;
use cga_gpu::*;

fn render_panel(sc: Scene, cam_pos: [f64; 3], target: [f64; 3], name: &str) {
    let mut cam = perspective_camera(
        50.0,
        400.0 / 300.0,
        0.1,
        100.0,
        cam_pos,
        target,
        [0.0, 1.0, 0.0],
    );
    cam.look_at(target, None);
    let mut r = renderer(400, 300, 2, 3);
    let img = r.render(sc, cam);

    let out_dir = "examples/advantage";
    let _ = std::fs::create_dir_all(out_dir);
    save_frame_png(&format!("{out_dir}/{name}.png"), &img);
}

fn panel_a() -> Scene {
    let mut sc = scene(Some(color_hex(0x101418)));
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::SphereGeometry(sphere_geometry(1.2)),
        material: standard_material(MaterialParams {
            color: color_hex(0xC0392B),
            roughness: 0.18,
            metalness: 0.2,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.15, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::CylinderGeometry(cylinder_geometry(0.4, -1.0)),
        material: standard_material(MaterialParams {
            color: color_hex(0xD4AC0D),
            roughness: 0.3,
            metalness: 0.25,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [1.55, 0.15, 0.1],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_light(directional_light(color_hex(0xFFFFFF), 0.5, [0.3, 0.9, 0.4]));
    sc.add_light(point_light(color_hex(0xFFFFFF), 0.9, [0.6, 2.0, 1.5]));
    sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.25));
    sc
}

fn panel_b() -> Scene {
    let mut sc = scene(None);
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::PlaneGeometry(plane_geometry([0.0, 1.0, 0.0], 0.0)),
        material: standard_material(MaterialParams {
            color: color_hex(0xB0B0B0),
            roughness: 0.75,
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
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::CylinderGeometry(cylinder_geometry(0.5, -1.0)),
        material: standard_material(MaterialParams {
            color: color_hex(0xD4AC0D),
            roughness: 0.35,
            metalness: 0.2,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [1.3, 0.0, -0.5],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::SphereGeometry(sphere_geometry(0.8)),
        material: standard_material(MaterialParams {
            color: color_hex(0xC0392B),
            roughness: 0.3,
            metalness: 0.0,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [-1.4, 0.8, 0.6],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_light(directional_light(
        color_hex(0xFFFFFF),
        0.42,
        [0.4, 1.0, 0.35],
    ));
    sc.add_light(point_light(color_hex(0xFFFFFF), 0.5, [0.0, 3.0, 2.5]));
    sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.32));
    sc
}

fn panel_c() -> Scene {
    let mut sc = scene(Some(color_hex(0x101418)));
    let m0 = translator([-2.2, 0.75, -0.6]).gp(&motor_rotor([0.0, 1.0, 0.0], -0.5));
    let m1 = translator([2.0, 0.75, 1.0]).gp(&motor_rotor([0.0, 1.0, 0.0], 1.1));
    for i in 0..6 {
        let m = m0.interpolate(&m1, f64::from(i) / 5.0);
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::SphereGeometry(sphere_geometry(0.16)),
            material: standard_material(MaterialParams {
                color: color_hex(0x95A5A6),
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
            motor: Some(m),
        }));
    }
    sc.add_mesh(mesh(MeshParams {
        geometry: Geometry::BoxGeometry(box_geometry(0.65, 0.65, 0.65)),
        material: standard_material(MaterialParams {
            color: color_hex(0x27AE60),
            roughness: 0.55,
            metalness: 0.0,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.0, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: Some(m1),
    }));
    sc.add_light(directional_light(
        color_hex(0xFFFFFF),
        0.45,
        [0.4, 1.0, 0.35],
    ));
    sc.add_light(point_light(color_hex(0xFFFFFF), 0.6, [-1.0, 2.5, 3.0]));
    sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.38));
    sc
}

fn main() {
    render_panel(panel_a(), [0.0, 0.15, 3.2], [0.0, 0.15, 0.0], "advantage_a");
    render_panel(panel_b(), [0.0, 1.6, 5.5], [0.0, 0.7, 0.0], "advantage_b");
    render_panel(panel_c(), [0.2, 2.6, 5.8], [0.0, 0.75, 0.2], "advantage_c");
}
