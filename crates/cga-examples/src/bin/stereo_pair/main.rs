use cga_core::*;
use cga_gpu::*;
use std::fs;

#[path = "rng.rs"]
pub mod rng;
pub(crate) use self::rng::*;
#[path = "placement.rs"]
pub mod placement;
pub(crate) use self::placement::*;

const FOV: f64 = 50.0;

const CAM_Y: f64 = 2.5;
const CAM_Z: f64 = 6.0;

const NEAR: f64 = 5.5;
const FAR: f64 = 11.5;

const ROOM_X: f64 = 6.0;
const ROOM_Y: f64 = 5.0;

const BACK_Z: f64 = -6.0;

const OBJECTS: usize = 14;

fn block_texture(size: i32, base_cells: i32, octaves: i32, rng: &mut Rng) -> Texture {
    let n = size as usize;
    let mut field = vec![0.0f64; n * n];
    let mut amp = 1.0;
    let mut total = 0.0;
    for k in 0..octaves {
        let cells = (base_cells << k) as usize;
        if cells > n {
            break;
        }
        let per = (n / cells).max(1);
        let mut tone = vec![0.0f64; cells * cells];
        for t in tone.iter_mut() {
            *t = if rng.unit() < 0.5 { amp } else { 0.0 };
        }
        for y in 0..n {
            for x in 0..n {
                field[y * n + x] += tone[(y / per % cells) * cells + (x / per % cells)];
            }
        }
        total += amp;
        amp *= 0.6;
    }
    let mut rgba = vec![0u8; n * n * 4];
    for i in 0..n * n {
        let v = 0.12 + 0.76 * (field[i] / total).clamp(0.0, 1.0);
        let s = (v * 255.0 + 0.5) as u8;
        rgba[i * 4] = s;
        rgba[i * 4 + 1] = s;
        rgba[i * 4 + 2] = s;
        rgba[i * 4 + 3] = 255;
    }
    Texture::from_u8_rgba(&rgba, size, size)
}

fn mat(color: i32, roughness: f64, map: Option<Texture>) -> Material {
    let mut m = Material::standard(MaterialParams {
        color: Color::from_hex(color),
        roughness,
        metalness: 0.0,
        emissive: Color::from_hex(0x000000),
        opacity: 1.0,
        ior: 1.5,
        absorption: 0.0,
    });
    m.map = map;
    m
}

fn build_scene(rng: &mut Rng) -> (Scene, Vec<Placement>) {
    let mut sc = Scene::new(None);

    let room = [
        ("floor", [0.0, 1.0, 0.0], 0.0),
        ("ceiling", [0.0, -1.0, 0.0], -ROOM_Y),
        ("back wall", [0.0, 0.0, 1.0], BACK_Z),
        ("left wall", [1.0, 0.0, 0.0], -ROOM_X),
        ("right wall", [-1.0, 0.0, 0.0], -ROOM_X),
    ];
    for (i, (_, normal, distance)) in room.iter().enumerate() {
        let color = if i == 1 { 0xBFC5CC } else { 0x9AA0A6 };
        sc.add_mesh(Mesh::new(MeshParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new(*normal, *distance)),
            material: mat(color, 0.85, Some(block_texture(256, 2, 4, rng))),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
    }

    let palette = [
        0xC0392B, 0x2980B9, 0x27AE60, 0xF39C12, 0x8E44AD, 0xE67E22, 0x16A085, 0xB03A2E, 0x2C3E50,
        0x7F8C8D,
    ];
    let mut placed = Vec::new();
    for _ in 0..OBJECTS {
        let depth = rng.range(NEAR, FAR);
        let z = CAM_Z - depth;

        let x = rng.range(-0.5, 0.5) * 0.62 * depth;
        let size = rng.range(0.20, 0.45);
        let color = palette[rng.below(palette.len() as u64) as usize];
        let map = Some(block_texture(256, 2, 4, rng));

        let (kind, geometry, y, axis, angle) = match rng.below(3) {
            0 => (
                "sphere",
                Geometry::SphereGeometry(SphereGeometry::new(size)),
                size,
                [0.0, 0.0, 1.0],
                0.0,
            ),
            1 => {
                let h = size * rng.range(1.6, 3.2);
                (
                    "box",
                    Geometry::BoxGeometry(BoxGeometry::new(size * 1.8, h, size * 1.8)),
                    h / 2.0,
                    [0.0, 1.0, 0.0],
                    rng.range(0.0, std::f64::consts::PI),
                )
            }
            _ => {
                let len = size * rng.range(2.0, 4.0);
                (
                    "cylinder",
                    Geometry::CylinderGeometry(CylinderGeometry::new(size, len)),
                    len / 2.0,
                    [1.0, 0.0, 0.0],
                    std::f64::consts::FRAC_PI_2,
                )
            }
        };
        let at = [x, y, z];
        sc.add_mesh(Mesh::new(MeshParams {
            geometry,
            material: mat(color, rng.range(0.45, 0.85), map),
            position: at,
            rotation_axis: axis,
            rotation_angle: angle,
            motor: None,
        }));
        placed.push(Placement { kind, at, size });
    }

    sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.45));
    sc.add_light(Light::directional(
        Color::from_hex(0xFFF6E8),
        0.70,
        [rng.range(-0.5, 0.5), 1.0, rng.range(-0.4, 0.4)],
    ));
    sc.add_light(Light::point(
        Color::from_hex(0xFFFFFF),
        0.55,
        [rng.range(-2.0, 2.0), 2.6, CAM_Z - rng.range(3.0, 6.0)],
    ));

    (sc, placed)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let seed: u64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
    let out_dir = args
        .get(2)
        .cloned()
        .unwrap_or_else(|| "artifacts/stereo".to_string());
    let w: i32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(640);
    let h: i32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(480);
    let baseline: f64 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(0.10);
    if w <= 0 || h <= 0 || baseline <= 0.0 {
        eprintln!("usage: stereo_pair [seed] [out_dir] [w] [h] [baseline]");
        std::process::exit(2);
    }

    let mut rng = Rng::new(seed);
    let (sc, placed) = build_scene(&mut rng);

    let aspect = f64::from(w) / f64::from(h);
    let focal_px = f64::from(h) / (2.0 * (FOV.to_radians() / 2.0).tan());
    let half = baseline / 2.0;

    let mut cam_left = PerspectiveCamera::new(
        FOV,
        aspect,
        0.05,
        200.0,
        [-half, CAM_Y, CAM_Z],
        [-half, CAM_Y, CAM_Z - 1.0],
        [0.0, 1.0, 0.0],
    );
    cam_left.look_at([-half, CAM_Y, CAM_Z - 1.0], Some([0.0, 1.0, 0.0]));
    let mut cam_right = PerspectiveCamera::new(
        FOV,
        aspect,
        0.05,
        200.0,
        [half, CAM_Y, CAM_Z],
        [half, CAM_Y, CAM_Z - 1.0],
        [0.0, 1.0, 0.0],
    );
    cam_right.look_at([half, CAM_Y, CAM_Z - 1.0], Some([0.0, 1.0, 0.0]));

    fs::create_dir_all(&out_dir).unwrap_or_else(|e| panic!("cannot make {out_dir}: {e}"));
    let mut r = Renderer::new(w, h, 2, 3);
    let left = r.render(sc.clone(), cam_left);
    let left_path = format!("{out_dir}/left.png");
    save_frame_png(&left_path, &left);
    let right = r.render(sc, cam_right);
    let right_path = format!("{out_dir}/right.png");
    save_frame_png(&right_path, &right);

    let mut depths: Vec<f64> = placed.iter().map(|p| CAM_Z - p.at[2]).collect();
    depths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let (near, far) = (depths[0], depths[depths.len() - 1]);

    let furthest = far.max(CAM_Z - BACK_Z);

    let mut truth = String::new();
    truth.push_str(&format!(
        "# seed {seed} · frame {w}x{h} · fov {FOV}° (vertical)\n"
    ));
    truth.push_str(&format!(
        "# focal_px {focal_px:.4} · baseline {baseline:.6} · camera y {CAM_Y} z {CAM_Z} · aspect {aspect:.6}\n"
    ));
    truth.push_str("# kind x y z size depth_z disparity_px\n");
    for p in &placed {
        let depth = CAM_Z - p.at[2];
        truth.push_str(&format!(
            "{} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}\n",
            p.kind,
            p.at[0],
            p.at[1],
            p.at[2],
            p.size,
            depth,
            focal_px * baseline / depth
        ));
    }
    fs::write(format!("{out_dir}/truth.txt"), &truth).unwrap();

    let px_per_deg = focal_px * 1.0f64.to_radians().tan();
    let d_near = focal_px * baseline / near;
    let d_far = focal_px * baseline / far;

    let floor_depth = CAM_Y / (FOV.to_radians() / 2.0).tan();
    let d_floor = focal_px * baseline / floor_depth;
    let search = d_near.max(d_floor).ceil().max(1.0);

    println!("stereo_pair · seed {seed} · {w}x{h} · baseline {baseline}");
    println!("saved {left_path}");
    println!("saved {right_path}");
    println!("saved {out_dir}/truth.txt");
    println!("focal_px   {focal_px:.4}  (vertical fov {FOV}°, aspect w/h = {aspect:.6})");
    println!("px_per_deg {px_per_deg:.4}");
    println!(
        "objects    depth {near:.2} .. {far:.2} (camera-space Z), disparity {d_near:.2} .. {d_far:.2} px"
    );
    println!(
        "furthest   {furthest:.2} (the room's back wall), disparity {:.2} px",
        focal_px * baseline / furthest
    );
    println!(
        "floor      bottom edge at depth {floor_depth:.2} (disparity {d_floor:.2} px) — --search {search:.0} covers it"
    );
    println!();
    println!("# 直接可跑（--search 取最近处的视差，`--furthest` 取最远处深度）:");
    println!(
        "cargo run --release -p r3d-cli -- stereo {left_path} {right_path} --px-per-deg {px_per_deg:.4} --search {search:.0} --out {out_dir}/stereo"
    );
    println!(
        "cargo run --release -p r3d-cli -- depth {left_path} {right_path} --focal {focal_px:.4} --baseline {baseline:.6} --furthest {furthest:.2} --px-per-deg {px_per_deg:.4} --search {search:.0} --out {out_dir}/depth"
    );
}
