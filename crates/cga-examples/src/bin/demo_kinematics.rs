// Kinematics demo: gears + crank-slider + spiral trajectory driven by Motor.
// Saves PNG frames and an animated GIF (assembled in V).
use cga_core::*;
use cga_examples::{data_f32, mlx_frame_gc};
use cga_gpu::*;
use std::f64::consts::PI;

struct KinScene {
    scene: Scene,
    big: Vec<usize>,
    small: Vec<usize>,
    rod: usize,
    slider: usize,
    ball: usize,
    m0: Multivector,
    twist: Multivector,
}

fn gear_meshes(
    ks: &mut KinScene,
    r_hub: f64,
    r_tooth: f64,
    n_teeth: i32,
    thick: f64,
    color: Color,
) -> Vec<usize> {
    let mat = standard_material(MaterialParams {
        color,
        roughness: 0.35,
        metalness: 0.7,
        emissive: color_hex(0x000000),
        opacity: 1.0,
        ior: 1.5,
        absorption: 0.0,
    });
    let mut idx: Vec<usize> = Vec::new();
    ks.scene.add_mesh(mesh(MeshParams {
        geometry: Geometry::CylinderGeometry(cylinder_geometry(r_hub, thick)),
        material: mat.clone(),
        position: [0.0, 0.0, 0.0],
        rotation_axis: [1.0, 0.0, 0.0],
        rotation_angle: -PI / 2.0,
        motor: None,
    }));
    idx.push(ks.scene.objects.len() - 1);
    let r_mid = r_hub + (r_tooth - r_hub) / 2.0;
    for i in 0..n_teeth {
        let a = f64::from(i) * 2.0 * PI / f64::from(n_teeth);
        let m = motor_rotor([0.0, 1.0, 0.0], a).gp(&translator([r_mid, 0.0, 0.0]));
        ks.scene.add_mesh(mesh(MeshParams {
            geometry: Geometry::BoxGeometry(box_geometry(r_tooth - r_hub + 0.06, thick, 0.16)),
            material: mat.clone(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: Some(m),
        }));
        idx.push(ks.scene.objects.len() - 1);
    }
    idx
}

fn frame_motor(axis: [f64; 3], angle: f64, point: [f64; 3]) -> Multivector {
    translator(point)
        .gp(&motor_rotor(axis, angle))
        .gp(&translator([-point[0], -point[1], -point[2]]))
}

fn rod_motor(p: [f64; 3], q: [f64; 3]) -> (Multivector, f64) {
    let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
    let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let u = [d[0] / length, d[1] / length, d[2] / length];
    let ax = [-u[1], u[0]];
    let an = (ax[0] * ax[0] + ax[1] * ax[1]).sqrt();
    let rot = if an < 1e-12 {
        motor_rotor([1.0, 0.0, 0.0], if u[2] > 0.0 { 0.0 } else { PI })
    } else {
        let angle = u[2].clamp(-1.0, 1.0).acos();
        motor_rotor([ax[0] / an, ax[1] / an, 0.0], angle)
    };
    let mid = [
        (p[0] + q[0]) / 2.0,
        (p[1] + q[1]) / 2.0,
        (p[2] + q[2]) / 2.0,
    ];
    (translator(mid).gp(&rot), length)
}

fn build_scene() -> KinScene {
    let mut ks = KinScene {
        scene: scene(None),
        big: Vec::new(),
        small: Vec::new(),
        rod: 0,
        slider: 0,
        ball: 0,
        m0: motor_identity(),
        twist: motor_identity(),
    };
    ks.scene.add_mesh(mesh(MeshParams {
        geometry: Geometry::PlaneGeometry(plane_geometry([0.0, 1.0, 0.0], -0.05)),
        material: standard_material(MaterialParams {
            color: color_hex(0x3A4046),
            roughness: 0.9,
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
    ks.scene
        .add_light(directional_light(color_hex(0xFFFFFF), 0.5, [0.4, 1.0, 0.5]));
    ks.scene
        .add_light(point_light(color_hex(0xFFFFFF), 0.5, [-4.0, 6.0, 4.0]));
    ks.scene.add_light(ambient_light(color_hex(0xFFFFFF), 0.4));
    // gears: big (16 teeth) at origin, small (8 teeth) at x=2.4
    ks.big = gear_meshes(&mut ks, 1.28, 1.6, 16, 0.4, color_hex(0xC8A24A));
    ks.small = gear_meshes(&mut ks, 0.64, 0.8, 8, 0.4, color_hex(0x9BA1A6));
    for &idx in &ks.small.clone() {
        let m = translator([2.4, 0.0, 0.0]).gp(&ks.scene.objects[idx].motor());
        ks.scene.objects[idx].motor_override = Some(m);
    }
    // crank-slider (rear z=2.6 plane)
    let crank_c = [-1.6, 0.6, 2.6];
    ks.scene.add_mesh(mesh(MeshParams {
        geometry: Geometry::CylinderGeometry(cylinder_geometry(0.55, 0.25)),
        material: standard_material(MaterialParams {
            color: color_hex(0x4A4F54),
            roughness: 0.5,
            metalness: 0.6,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: crank_c,
        rotation_axis: [1.0, 0.0, 0.0],
        rotation_angle: -PI / 2.0,
        motor: None,
    }));
    ks.scene.add_mesh(mesh(MeshParams {
        geometry: Geometry::CylinderGeometry(cylinder_geometry(0.09, 1.0)),
        material: standard_material(MaterialParams {
            color: color_hex(0xC8A24A),
            roughness: 0.3,
            metalness: 0.8,
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
    ks.rod = ks.scene.objects.len() - 1;
    ks.scene.add_mesh(mesh(MeshParams {
        geometry: Geometry::BoxGeometry(box_geometry(0.5, 0.4, 0.35)),
        material: standard_material(MaterialParams {
            color: color_hex(0x9BA1A6),
            roughness: 0.35,
            metalness: 0.75,
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
    ks.slider = ks.scene.objects.len() - 1;
    ks.scene.add_mesh(mesh(MeshParams {
        geometry: Geometry::BoxGeometry(box_geometry(2.6, 0.08, 0.5)),
        material: standard_material(MaterialParams {
            color: color_hex(0x4A4F54),
            roughness: 0.5,
            metalness: 0.6,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        }),
        position: [0.0, 0.4, 2.6],
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    }));
    // spiral ball
    ks.scene.add_mesh(mesh(MeshParams {
        geometry: Geometry::SphereGeometry(sphere_geometry(0.18)),
        material: standard_material(MaterialParams {
            color: color_hex(0xC0392B),
            roughness: 0.3,
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
    ks.ball = ks.scene.objects.len() - 1;
    let m0 = translator([-3.2, 0.5, 2.2]);
    let m1 = translator([3.6, 1.8, 3.4]).gp(&motor_rotor([0.0, 1.0, 0.0], 1.5 * PI));
    ks.m0 = m0;
    ks.twist = m0.reverse().gp(&m1).log();
    for k in 0..9 {
        ks.scene.add_mesh(mesh(MeshParams {
            geometry: Geometry::SphereGeometry(sphere_geometry(0.05)),
            material: standard_material(MaterialParams {
                color: color_hex(0x7F8C8D),
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
            motor: Some(m0.gp(&motor_exp(&ks.twist, f64::from(k) / 8.0))),
        }));
    }
    ks
}

fn main() {
    let mut ks = build_scene();
    let n_frames = 24;
    let mut big_local: Vec<Multivector> = Vec::new();
    for &idx in &ks.big {
        big_local.push(ks.scene.objects[idx].motor());
    }
    let mut small_local: Vec<Multivector> = Vec::new();
    for &idx in &ks.small {
        small_local.push(ks.scene.objects[idx].motor());
    }
    let mut cam = perspective_camera(
        48.0,
        4.0 / 3.0,
        0.1,
        100.0,
        [0.4, 5.6, 8.8],
        [0.4, 0.2, 1.2],
        [0.0, 1.0, 0.0],
    );
    cam.look_at([0.4, 0.2, 1.2], None);
    let mut r = renderer(360, 270, 2, 3);
    let mut gif_frames: Vec<Vec<u8>> = Vec::new();
    let big = ks.big.clone();
    let small = ks.small.clone();
    for f in 0..n_frames {
        let s = f64::from(f) / f64::from(n_frames);
        let g1 = frame_motor([0.0, 1.0, 0.0], 2.0 * PI * s, [0.0, 0.0, 0.0]);
        for (i, &idx) in big.iter().enumerate() {
            ks.scene.objects[idx].motor_override = Some(g1.gp(&big_local[i]));
        }
        let g2 = frame_motor([0.0, 1.0, 0.0], -4.0 * PI * s + PI / 8.0, [2.4, 0.0, 0.0]);
        for (i, &idx) in small.iter().enumerate() {
            ks.scene.objects[idx].motor_override = Some(g2.gp(&small_local[i]));
        }
        let th = 2.0 * PI * s;
        let p = [-1.6 + 0.35 * th.cos(), 0.6, 2.6 + 0.35 * th.sin()];
        let xs = p[0] + (1.35 * 1.35 - (2.6 - p[2]) * (2.6 - p[2])).sqrt();
        let q = [xs, 0.6, 2.6];
        let (rm, rl) = rod_motor(p, q);
        ks.scene.objects[ks.rod].motor_override = Some(rm);
        ks.scene.objects[ks.rod].geometry = Geometry::CylinderGeometry(cylinder_geometry(0.09, rl));
        ks.scene.objects[ks.slider].position = q;
        ks.scene.objects[ks.slider].motor_override = None;
        ks.scene.objects[ks.ball].motor_override = Some(ks.m0.gp(&motor_exp(&ks.twist, s)));
        let img = r.render(ks.scene.clone(), cam);
        gif_frames.push(f32_rgba_to_u8(&data_f32(&img)));
        drop(img);
        // 每帧收尾：释放死 Metal buffer（Rust drop），再把 MLX 不复用的
        // cache 还给 OS（见 editor/server.v；V 里是 gc_collect + clear_cache）
        mlx_frame_gc();
    }
    // V: os.dir(@FILE) — Rust 版以仓库根目录为 CWD 运行
    let out = "examples/kinematics";
    save_gif(&format!("{out}/kinematics.gif"), &gif_frames, 360, 270, 5);
}
