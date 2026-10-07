use cga_core::*;
use cga_examples::{data_f32, mlx_frame_gc, save_gif};
use cga_gpu::*;
use std::f64::consts::PI;

fn build_scene() -> Scene {
    let mut sc = Scene::new(None);

    sc.add_object(Object::new(ObjectParams {
        geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0xB0B0B0),
            roughness: 0.7,
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

    sc.add_object(Object::new(ObjectParams {
        geometry: Geometry::SphereGeometry(SphereGeometry::new(1.0)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0xC0392B),
            roughness: 0.25,
            metalness: 0.25,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 1.0, 0.0],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    sc.add_object(Object::new(ObjectParams {
        geometry: Geometry::SphereGeometry(SphereGeometry::new(0.6)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0x2980B9),
            roughness: 0.15,
            metalness: 0.35,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [-2.2, 0.6, 0.5],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    sc.add_object(Object::new(ObjectParams {
        geometry: Geometry::CylinderGeometry(CylinderGeometry::new(0.7, -1.0)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0xD4AC0D),
            roughness: 0.4,
            metalness: 0.3,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [2.2, 0.7, -0.5],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    sc.add_object(Object::new(ObjectParams {
        geometry: Geometry::BoxGeometry(BoxGeometry::new(0.9, 0.9, 0.9)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0x27AE60),
            roughness: 0.6,
            metalness: 0.0,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.8, 0.45, 1.8],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));

    sc.add_object(Object::new(ObjectParams {
        geometry: Geometry::CircleGeometry(CircleGeometry::new(0.9)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0x8E44AD),
            roughness: 0.3,
            metalness: 0.0,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [-2.4, 2.2, 0.8],
        rotation_axis: [1.0, 0.0, 0.0],
        rotation_angle: -0.4,
        motor: None,
    }));

    sc.add_object(Object::new(ObjectParams {
        geometry: Geometry::SphereGeometry(SphereGeometry::new(0.8)),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0xAAD4FF),
            roughness: 0.05,
            metalness: 0.0,
            emissive: Color::from_hex(0x000000),
            opacity: 0.08,
            ior: 1.5,
            absorption: 0.2,
        }),
        position: [0.4, 1.5, 2.6],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    sc.add_light(Light::directional(
        Color::from_hex(0xFFFFFF),
        0.38,
        [0.4, 1.0, 0.35],
    ));
    sc.add_light(Light::point(
        Color::from_hex(0xFFFFFF),
        0.7,
        [0.0, 4.0, 3.5],
    ));
    sc.add_light(Light::directional(
        Color::from_hex(0xFFFFFF),
        0.18,
        [0.0, 0.35, 0.9],
    ));
    sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.34));
    sc
}

fn main() {
    let frames: i32 = {
        use clap::Parser;
        #[derive(Parser)]
        struct Args {
            /// Number of orbit frames
            #[arg(default_value_t = 90)]
            frames: i32,
        }
        Args::parse().frames
    };

    let out_dir = "examples/engine";

    let sc = build_scene();
    let mut camera = PerspectiveCamera::new(
        50.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [0.0, 2.4, 6.2],
        [0.0, 0.8, 0.0],
        [0.0, 1.0, 0.0],
    );
    camera.look_at([0.0, 0.8, 0.0], None);
    let controls = OrbitControls::new([0.0, 0.8, 0.0], 0.0, 0.42, 6.6);
    let mut r = Renderer::new(360, 270, 2, 3);

    let mut gif_frames: Vec<Vec<u8>> = Vec::new();
    for i in 0..frames {
        let mut ctrl = controls;
        ctrl.azimuth = 2.0 * PI * f64::from(i) / f64::from(frames);
        ctrl.elevation = 0.42 + 0.12 * (4.0 * PI * f64::from(i) / f64::from(frames)).sin();
        ctrl.update(&mut camera);
        let img = r.render(sc.clone(), camera);
        gif_frames.push(f32_rgba_to_u8(&data_f32(&img)));
        drop(img);

        mlx_frame_gc();
        println!("frame {}/{} rendered", i + 1, frames);
    }
    save_gif(&format!("{out_dir}/orbit.gif"), &gif_frames, 360, 270, 3);
    println!("saved {out_dir}/orbit.gif");
}
