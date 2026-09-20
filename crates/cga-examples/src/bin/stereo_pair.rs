// stereo_pair.rs — 随机三维场景 + 双相机双目渲染，产出 r3d 可直接读的一对图。
//
// 为什么这样摆相机：r3d 把一对图当作**已校正**的左右目，左图在前、右图在后
// （`r3d-pathways/src/depth/distance.rs` 的头注释）。所以两只相机必须**朝向完全相同**，
// 只沿相机 x 轴平移一个基线 —— 做法是让两个相机的 target 各自正下方一点，
// 于是 f = (0,0,−1) 对两者都精确成立，r = cross(f, up) = +x，得到一对严格校正的图。
// 此时近处结构在左图中投影在更大的 x 上，位移为负，正是 r3d 的 `Depth` 读的符号。
//
// 相机空间约定（cga renderer）：X 右 / Y 下 / Z 前；`fov` 是**垂直**视场，
// fx = fy · aspect，主点在 ((w−1)/2, (h−1)/2)。要给 r3d 一个单一焦距，aspect 必须等于
// w/h，于是 focal_px = h / (2·tan(fov/2))。
//
// 输出 <out_dir>/left.png · <out_dir>/right.png · <out_dir>/truth.txt，并打印可直接粘贴的
// r3d 命令行。渲染输出是 sRGB 编码的 8 位 RGBA（renderer.rs 出口做 sRGB encode），
// 正是 `RgbImage::load` 期望的输入。
//
// 用法：cargo run --release -p cga-examples --bin stereo_pair -- [seed] [out_dir] [w] [h] [baseline]

use cga_core::*;
use cga_gpu::*;
use std::fs;

/// FOV 是所有帧共用的垂直视场角，度。
const FOV: f64 = 50.0;

/// CAM_Y 与 CAM_Z 是两个相机共同的高度与离原点的距离：都看 −z，只差 x 上的半个基线。
///
/// CAM_Y 取 2.5 而不是贴地的高度：画面**底边**落在地面上的深度是 CAM_Y / tan(fov/2)，
/// 也就是 5.36，视差 9.6 像素——这样整幅地面都落在 `--search 10` 之内，
/// 不留一条"太近而读不出"的边。抬相机不会破坏校正（两相机仍只差 x）。
const CAM_Y: f64 = 2.5;
const CAM_Z: f64 = 6.0;

/// NEAR and FAR bound the depth a random object is placed at, in the camera's own Z: the disparity
/// `focal·baseline/Z` then runs about 9.4 down to 4.5 pixels at a baseline of a tenth of a unit, which
/// stands above the band's finest wavelength (3 px) and inside half of its coarsest (24 px).
const NEAR: f64 = 5.5;
const FAR: f64 = 11.5;

/// ROOM_X, ROOM_Y and BACK_Z state the room the scene is drawn in.  BACK_Z matters beyond the look of
/// it: the back wall is the farthest surface the pair has, so it — not the farthest object — is the
/// bound a run's `--furthest` must state, or the wall's own distance is clamped away.
const ROOM_X: f64 = 6.0;
const ROOM_Y: f64 = 5.0;
const BACK_Z: f64 = -6.0;

/// OBJECTS 是随机场景里物件的数量。
const OBJECTS: usize = 14;

// --- 随机数 ------------------------------------------------------------------
//
// splitmix64：无外部依赖，同一种子在任何机器上给出同一个场景，这是"可复现"的全部要求。

struct Rng {
    s: u64,
}

impl Rng {
    fn new(seed: u64) -> Rng {
        Rng { s: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.s = self.s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// unit is a uniform draw in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / 9_007_199_254_740_992.0
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next_u64() % n
    }
}

// --- 程序化纹理 --------------------------------------------------------------
//
// 多倍频块状噪声：2、4、8、16 格每世界单位的随机块图案按半幅权重相加。
// 单一频率的棋盘只能和自己混叠，掠射角上立刻出摩尔纹，而且约十像素的周期正好和视差同量级，
// 相位读出会绕卷；一个带通滤波器组要的是**多个尺度上的边**。语料为同一个理由画的是
// 方波的倍频叠加（`ROADMAP.md` 步骤 23）。

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
        // 压到 0.12 .. 0.88：留开纯黑纯白，掠射角上不留过硬的黑白跳变
        let v = 0.12 + 0.76 * (field[i] / total).clamp(0.0, 1.0);
        let s = (v * 255.0 + 0.5) as u8;
        rgba[i * 4] = s;
        rgba[i * 4 + 1] = s;
        rgba[i * 4 + 2] = s;
        rgba[i * 4 + 3] = 255;
    }
    texture_from_u8_rgba(&rgba, size, size)
}

// --- 材质与场景 --------------------------------------------------------------

fn mat(color: i32, roughness: f64, map: Option<Texture>) -> Material {
    let mut m = standard_material(MaterialParams {
        color: color_hex(color),
        roughness,
        metalness: 0.0,
        emissive: color_hex(0x000000),
        opacity: 1.0,
        ior: 1.5,
        absorption: 0.0,
    });
    m.map = map;
    m
}

/// Placement is one object the scene was drawn with: what it is, where it stands and how big it is —
/// the world's own statement beside the picture, so a read can be checked against it.
struct Placement {
    kind: &'static str,
    at: [f64; 3],
    size: f64,
}

fn build_scene(rng: &mut Rng) -> (Scene, Vec<Placement>) {
    let mut sc = scene(None);

    // 房间：地面 + 天花板 + 后墙 + 两片侧墙，把画面填满结构。
    // 只有地面的话，天空与掠射角上的远地面都读不出位移，整幅距离场会被钳到最远，
    // 拿到的一对图就只在几个物件上可用；房间把每个方向都换成一段可读的深度。
    let room = [
        ("floor", [0.0, 1.0, 0.0], 0.0),
        ("ceiling", [0.0, -1.0, 0.0], -ROOM_Y),
        ("back wall", [0.0, 0.0, 1.0], BACK_Z),
        ("left wall", [1.0, 0.0, 0.0], -ROOM_X),
        ("right wall", [-1.0, 0.0, 0.0], -ROOM_X),
    ];
    for (i, (_, normal, distance)) in room.iter().enumerate() {
        // 天花板拿不到上面的直射光，给亮一点的自发色；其余用中灰
        let color = if i == 1 { 0xBFC5CC } else { 0x9AA0A6 };
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::PlaneGeometry(plane_geometry(*normal, *distance)),
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
        // 深度 Z ∈ [NEAR, FAR]：视差 = focal·baseline/Z，取 baseline 0.1 时约 9.4 .. 4.5 像素，
        // 落在 3 px 最细尺度之上、24 px 最粗尺度的半波长之内 —— 读数不绕卷的那个区间。
        let depth = rng.range(NEAR, FAR);
        let z = CAM_Z - depth;
        // 横向随深度收窄：水平半视场 = Z·tan(hfov/2) = 0.622·Z，取它的一半留出边界余量
        let x = rng.range(-0.5, 0.5) * 0.62 * depth;
        let size = rng.range(0.20, 0.45);
        let color = palette[rng.below(palette.len() as u64) as usize];
        let map = Some(block_texture(256, 2, 4, rng));

        // 三种实体：球 / 盒 / 柱。三类都落在地面上，都带随机偏航，柱另转成竖直。
        let (kind, geometry, y, axis, angle) = match rng.below(3) {
            0 => (
                "sphere",
                Geometry::SphereGeometry(sphere_geometry(size)),
                size,
                [0.0, 0.0, 1.0],
                0.0,
            ),
            1 => {
                let h = size * rng.range(1.6, 3.2);
                (
                    "box",
                    Geometry::BoxGeometry(box_geometry(size * 1.8, h, size * 1.8)),
                    h / 2.0,
                    [0.0, 1.0, 0.0],
                    rng.range(0.0, std::f64::consts::PI),
                )
            }
            _ => {
                let len = size * rng.range(2.0, 4.0);
                (
                    "cylinder",
                    Geometry::CylinderGeometry(cylinder_geometry(size, len)),
                    len / 2.0,
                    [1.0, 0.0, 0.0],
                    std::f64::consts::FRAC_PI_2,
                )
            }
        };
        let at = [x, y, z];
        sc.add_mesh(mesh(MeshParams {
            geometry,
            material: mat(color, rng.range(0.45, 0.85), map),
            position: at,
            rotation_axis: axis,
            rotation_angle: angle,
            motor: None,
        }));
        placed.push(Placement { kind, at, size });
    }

    // 光照：环境 + 一盏方向光 + 一盏点光。方向光的 y 分量为正表示光从上方来。
    sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.45));
    sc.add_light(directional_light(
        color_hex(0xFFF6E8),
        0.70,
        [rng.range(-0.5, 0.5), 1.0, rng.range(-0.4, 0.4)],
    ));
    sc.add_light(point_light(
        color_hex(0xFFFFFF),
        0.55,
        [rng.range(-2.0, 2.0), 2.6, CAM_Z - rng.range(3.0, 6.0)],
    ));

    (sc, placed)
}

// --- 入口 -------------------------------------------------------------------

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

    // aspect = w / h 而非 fov 推出的横纵比：只有这样 fx 才等于 fy，
    // r3d 的 Calibration 才有一个单一焦距可用。
    let aspect = f64::from(w) / f64::from(h);
    let focal_px = f64::from(h) / (2.0 * (FOV.to_radians() / 2.0).tan());
    let half = baseline / 2.0;

    // 两个相机朝向完全相同：target 都在各自正下方一点，于是 f 与 r 逐分量相同，
    // 两者只差 x 上的半个基线 —— 严格校正的一对。
    let mut cam_left = perspective_camera(
        FOV,
        aspect,
        0.05,
        200.0,
        [-half, CAM_Y, CAM_Z],
        [-half, CAM_Y, CAM_Z - 1.0],
        [0.0, 1.0, 0.0],
    );
    cam_left.look_at([-half, CAM_Y, CAM_Z - 1.0], Some([0.0, 1.0, 0.0]));
    let mut cam_right = perspective_camera(
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
    let mut r = renderer(w, h, 2, 3);
    let left = r.render(sc.clone(), cam_left);
    let left_path = format!("{out_dir}/left.png");
    save_frame_png(&left_path, &left);
    let right = r.render(sc, cam_right);
    let right_path = format!("{out_dir}/right.png");
    save_frame_png(&right_path, &right);

    // 真值：场景自己的声明。深度是相机系下的 Z = CAM_Z − z。
    let mut depths: Vec<f64> = placed.iter().map(|p| CAM_Z - p.at[2]).collect();
    depths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let (near, far) = (depths[0], depths[depths.len() - 1]);
    // 房间后墙比任何物件都远，它才是这一对图最远的那个面
    let furthest = far.max(CAM_Z - BACK_Z);

    let mut truth = String::new();
    truth.push_str(&format!("# seed {seed} · frame {w}x{h} · fov {FOV}° (vertical)\n"));
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

    // 像素每度：中心处的局部尺度 f·tan(1°)，fovea 的落点用的就是它。
    let px_per_deg = focal_px * 1.0f64.to_radians().tan();
    let d_near = focal_px * baseline / near;
    let d_far = focal_px * baseline / far;
    // 画面底边落在地面上的那条带是最近的深度，搜索范围要把它一起盖住，否则近处地面读不出位移
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
