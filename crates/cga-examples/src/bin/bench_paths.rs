//! 光线路径 vs 光栅路径实测对比（一次性基准，不入测试套件）。
use cga_core::*;
use cga_gpu::*;
use mlx_rs::{ops, Array};
use std::time::Instant;

fn uv_sphere(stacks: usize, slices: usize) -> TrimeshGeometry {
    let mut verts: Vec<[f64; 3]> = Vec::new();
    let mut faces: Vec<[i32; 3]> = Vec::new();
    for i in 0..=stacks {
        let phi = std::f64::consts::PI * i as f64 / stacks as f64;
        for j in 0..slices {
            let th = 2.0 * std::f64::consts::PI * j as f64 / slices as f64;
            verts.push([
                1.5 * phi.sin() * th.cos(),
                1.5 * phi.cos(),
                1.5 * phi.sin() * th.sin(),
            ]);
        }
    }
    for i in 0..stacks {
        for j in 0..slices {
            let a = (i * slices + j) as i32;
            let b = (i * slices + (j + 1) % slices) as i32;
            let c = ((i + 1) * slices + j) as i32;
            let d = ((i + 1) * slices + (j + 1) % slices) as i32;
            faces.push([a, c, b]);
            faces.push([b, c, d]);
        }
    }
    // 过滤极点处的零面积三角形
    faces.retain(|f| {
        let (a, b, c) = (verts[f[0] as usize], verts[f[1] as usize], verts[f[2] as usize]);
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let cx = e1[1] * e2[2] - e1[2] * e2[1];
        let cy = e1[2] * e2[0] - e1[0] * e2[2];
        let cz = e1[0] * e2[1] - e1[1] * e2[0];
        cx * cx + cy * cy + cz * cz > 1e-20
    });
    TrimeshGeometry::new(&verts, &faces)
}

fn cam(w: i32, h: i32) -> PerspectiveCamera {
    let mut c = PerspectiveCamera::new(
        40.0,
        w as f64 / h as f64,
        0.1,
        100.0,
        [0.0, 0.0, 4.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    );
    c.look_at([0.0, 0.0, 0.0], None);
    c
}

fn mesh_scene(g: TrimeshGeometry) -> Scene {
    let mut sc = Scene::new(None);
    sc.add_mesh(Mesh::new(MeshParams {
        geometry: Geometry::TrimeshGeometry(g),
        material: Material::standard(MaterialParams {
            color: Color::from_hex(0xC0392B),
            roughness: 0.3,
            metalness: 0.1,
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
    sc.add_light(Light::directional(Color::from_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]));
    sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.3));
    sc
}

fn bench_raster_end_to_end(faces: usize, w: i32, h: i32) {
    let s = ((faces / 2) as f64).sqrt().max(2.0) as usize;
    let sl = faces / (2 * s);
    let g = uv_sphere(s, sl);
    let real_faces = g.n_faces;
    let sc = mesh_scene(g);
    let t0 = Instant::now();
    let mut r = Renderer::new(w, h, 1, 3);
    let img = r.render(sc, cam(w, h));
    img.eval().unwrap();
    println!(
        "raster  {:>7} faces {:>4}x{:<4}  {:>8.1} ms",
        real_faces,
        w,
        h,
        t0.elapsed().as_secs_f64() * 1e3
    );
}

fn bench_ray_trimesh(faces: usize, w: i32, h: i32) {
    let s = ((faces / 2) as f64).sqrt().max(2.0) as usize;
    let sl = faces / (2 * s);
    let g = uv_sphere(s, sl);
    let real_faces = g.n_faces;
    let geom = Geometry::TrimeshGeometry(g);
    let c = cam(w, h);
    let fy = h as f64 / (2.0 * (40.0_f64.to_radians() / 2.0).tan());
    let fx = fy * c.aspect;
    let cx = (w - 1) as f64 / 2.0;
    let cy = (h - 1) as f64 / 2.0;
    let n = (w * h) as usize;
    let mut dirs = vec![0f32; n * 3];
    for y in 0..h {
        for x in 0..w {
            let u = (x as f64 - cx) / fx;
            let v = (y as f64 - cy) / fy;
            let l = (u * u + v * v + 1.0).sqrt();
            let o = (y * w + x) as usize * 3;
            dirs[o] = (u / l) as f32;
            dirs[o + 1] = (v / l) as f32;
            dirs[o + 2] = (1.0 / l) as f32;
        }
    }
    let d = Array::from_slice(&dirs, &[w * h, 3]);
    let o = ops::zeros::<f32>(&[w * h, 3]).unwrap();
    let wm = c.motor;
    let t0 = Instant::now();
    let params = geom_to_camera(&geom, &wm);
    let (t, _n, _m) = geom_intersect(&params, &o, &d);
    t.eval().unwrap();
    println!(
        "ray     {:>7} faces {:>4}x{:<4}  {:>8.1} ms",
        real_faces,
        w,
        h,
        t0.elapsed().as_secs_f64() * 1e3
    );
}

fn main() {
    // warmup: mlx pipeline 初始化
    let mut r = Renderer::new(32, 32, 1, 3);
    r.render(mesh_scene(uv_sphere(4, 8)), cam(32, 32)).eval().unwrap();

    println!("== 光线路径（trimesh 求交，GPU）==");
    for &(f, w, h) in &[
        (1_000usize, 640i32, 480i32),
        (2_000, 640, 480),
        (8_000, 320, 240),
        (32_000, 160, 120),
    ] {
        bench_ray_trimesh(f, w, h);
    }
    println!("== 光栅路径（端到端 Renderer，CPU 扫描 + GPU 着色）==");
    for &(f, w, h) in &[
        (1_000usize, 640i32, 480i32),
        (8_000, 640, 480),
        (32_000, 640, 480),
        (128_000, 640, 480),
        (512_000, 640, 480),
    ] {
        bench_raster_end_to_end(f, w, h);
    }
}
