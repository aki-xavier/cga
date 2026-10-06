//! 纯解析光线追踪基准：成本模型各因子（对象数 / 屏内占比 / 光源数 / 分辨率 /
//! 抗锯齿 / 图元类型 / CSG 子节点数 / 折射递归）与真实画廊场景。
//!
//! 用法：cargo run --release --bin bench_ray [gallery]
use cga_core::*;
use cga_gpu::*;
use std::time::Instant;

fn mat_red() -> Material {
    Material::standard(MaterialParams {
        color: Color::from_hex(0xC0392B),
        roughness: 0.4,
        metalness: 0.1,
        emissive: Color::from_hex(0x000000),
        opacity: 1.0,
        ior: 1.5,
        absorption: 0.0,
    })
}

fn mat_glass() -> Material {
    Material::standard(MaterialParams {
        color: Color::from_hex(0xAAD4FF),
        roughness: 0.05,
        metalness: 0.0,
        emissive: Color::from_hex(0x000000),
        opacity: 0.08,
        ior: 1.5,
        absorption: 0.2,
    })
}

fn obj(geometry: Geometry, pos: [f64; 3], material: Material) -> Object {
    Object::new(ObjectParams {
        geometry,
        material,
        position: pos,
        rotation_axis: [0.0, 0.0, 1.0],
        rotation_angle: 0.0,
        motor: None,
    })
}

fn cam(w: i32, h: i32) -> PerspectiveCamera {
    let mut c = PerspectiveCamera::new(
        45.0,
        w as f64 / h as f64,
        0.1,
        100.0,
        [0.0, 4.0, 9.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    );
    c.look_at([0.0, 0.0, 0.0], None);
    c
}

/// n 个图元铺在画面内（全部可见）。
fn visible(n: usize, mut mk: impl FnMut([f64; 3]) -> Object) -> Scene {
    let mut sc = Scene::new(None);
    let cols = (n as f64).sqrt().ceil().max(1.0) as usize;
    for i in 0..n {
        let (r, c) = (i / cols, i % cols);
        let x = -2.0 + 4.0 * (c as f64 + 0.5) / cols as f64;
        let z = -2.0 + 4.0 * (r as f64 + 0.5) / cols as f64;
        sc.add_object(mk([x, 0.3, z]));
    }
    sc
}

/// n 个图元，但只有第 1 个在视野内。
fn mostly_offscreen(n: usize, mut mk: impl FnMut([f64; 3]) -> Object) -> Scene {
    let mut sc = Scene::new(None);
    sc.add_object(mk([0.0, 0.3, 0.0]));
    for i in 1..n {
        sc.add_object(mk([-80.0 - i as f64, 0.0, -60.0]));
    }
    sc
}

fn lights(sc: &mut Scene, n: u8) {
    match n {
        0 => {}
        1 => sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.4, 1.0, 0.4],
        )),
        _ => {
            sc.add_light(Light::directional(
                Color::from_hex(0xFFFFFF),
                0.7,
                [0.4, 1.0, 0.4],
            ));
            sc.add_light(Light::point(
                Color::from_hex(0xFFFFFF),
                0.6,
                [-5.0, 6.0, 4.0],
            ));
        }
    }
    sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.3));
}

fn time(sc: Scene, w: i32, h: i32, aa: i32, label: &str) {
    time_cam(sc, cam(w, h), w, h, aa, label)
}

fn time_cam(sc: Scene, camera: PerspectiveCamera, w: i32, h: i32, aa: i32, label: &str) {
    let n = sc.objects.len();
    let mut r = Renderer::new(w, h, aa, 3);
    let t0 = Instant::now();
    let img = r.render(sc, camera);
    img.eval().unwrap();
    println!(
        "{label:<28} 对象 {n:>4}  {w}x{h} aa={aa}  {:>9.1} ms",
        t0.elapsed().as_secs_f64() * 1e3
    );
}

fn main() {
    let gallery = std::env::args().any(|a| a == "gallery");
    // 预热管线
    let mut r = Renderer::new(32, 32, 1, 3);
    let mut warm = visible(1, |p| {
        obj(
            Geometry::SphereGeometry(SphereGeometry::new(0.3)),
            p,
            mat_red(),
        )
    });
    lights(&mut warm, 1);
    r.render(warm, cam(32, 32)).eval().unwrap();

    if gallery {
        println!("== 画廊场景（真实图元场景）");
        let scenes = [
            "orbit",
            "grid",
            "building",
            "mechanical",
            "primitives",
            "affine",
            "assembly",
        ];
        for name in scenes {
            let jsx = std::fs::read_to_string(format!("examples/jsx/{name}.jsx")).unwrap();
            let css =
                std::fs::read_to_string(format!("examples/jsx/{name}.css")).unwrap_or_default();
            let mut run = cga_host::run_jsx(&jsx, Some(&css), "examples/jsx").expect("run_jsx");
            for (w, h) in [(640i32, 480i32), (1280, 720)] {
                run.camera.aspect = f64::from(w) / f64::from(h);
                time_cam(run.scene.clone(), run.camera, w, h, 2, &format!("{name}"));
            }
        }
        return;
    }

    println!("== 对象数（全部可见，640x480 aa=1，2 光源）");
    for n in [1usize, 10, 50, 200, 500, 1000] {
        let mut sc = visible(n, |p| {
            obj(
                Geometry::SphereGeometry(SphereGeometry::new(0.06)),
                p,
                mat_red(),
            )
        });
        lights(&mut sc, 2);
        time(sc, 640, 480, 1, "sphere visible");
    }

    println!("== 屏内占比（640x480 aa=1，2 光源）");
    for n in [200usize, 500] {
        let mut sc = mostly_offscreen(n, |p| {
            obj(
                Geometry::SphereGeometry(SphereGeometry::new(0.06)),
                p,
                mat_red(),
            )
        });
        lights(&mut sc, 2);
        time(sc, 640, 480, 1, "sphere mostly offscreen");
    }

    println!("== 光源数（200 球可见，640x480 aa=1）");
    for nl in [0u8, 1, 2] {
        let mut sc = visible(200, |p| {
            obj(
                Geometry::SphereGeometry(SphereGeometry::new(0.06)),
                p,
                mat_red(),
            )
        });
        lights(&mut sc, nl);
        time(sc, 640, 480, 1, &format!("sphere visible {nl} light"));
    }

    println!("== 分辨率 / 抗锯齿（50 球可见，2 光源）");
    for (w, h, aa) in [
        (320i32, 240i32, 1i32),
        (640, 480, 1),
        (1280, 720, 1),
        (640, 480, 2),
        (1280, 720, 2),
    ] {
        let mut sc = visible(50, |p| {
            obj(
                Geometry::SphereGeometry(SphereGeometry::new(0.25)),
                p,
                mat_red(),
            )
        });
        lights(&mut sc, 2);
        time(sc, w, h, aa, "sphere visible");
    }

    println!("== 图元类型（100 个同类，640x480 aa=1）");
    let kinds: Vec<(&str, fn([f64; 3]) -> Object)> = vec![
        ("sphere", |p| {
            obj(
                Geometry::SphereGeometry(SphereGeometry::new(0.06)),
                p,
                mat_red(),
            )
        }),
        ("box", |p| {
            obj(
                Geometry::BoxGeometry(BoxGeometry::new(0.1, 0.1, 0.1)),
                p,
                mat_red(),
            )
        }),
        ("cylinder", |p| {
            obj(
                Geometry::CylinderGeometry(CylinderGeometry::new(0.06, 0.12)),
                p,
                mat_red(),
            )
        }),
        ("cone", |p| {
            obj(
                Geometry::ConeGeometry(ConeGeometry::new(0.06, 0.2)),
                p,
                mat_red(),
            )
        }),
        ("torus", |p| {
            obj(
                Geometry::TorusGeometry(TorusGeometry::new(0.07, 0.025)),
                p,
                mat_red(),
            )
        }),
        ("ellipsoid", |p| {
            obj(
                Geometry::EllipsoidGeometry(EllipsoidGeometry::new(0.06, 0.09, 0.05)),
                p,
                mat_red(),
            )
        }),
        ("cyclide", |p| {
            obj(
                Geometry::CyclideGeometry(CyclideGeometry::new(0.1, 0.08, 0.03, [0.0, 0.0, 0.0])),
                p,
                mat_red(),
            )
        }),
    ];
    for (name, mk) in kinds {
        let mut sc = visible(100, mk);
        lights(&mut sc, 2);
        time(sc, 640, 480, 1, name);
    }

    println!("== CSG 子节点数（box 差 N 个球，640x480 aa=1）");
    for n in [1usize, 4, 16, 64] {
        let mut kids = vec![Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0))];
        for i in 0..n {
            let a = i as f64 * 0.7;
            kids.push(Geometry::AffineGeometry(AffineGeometry::new(
                Geometry::SphereGeometry(SphereGeometry::new(0.25)),
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            )));
            let _ = a;
        }
        let mut sc = Scene::new(None);
        sc.add_object(obj(
            Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Difference, kids)),
            [0.0, 0.3, 0.0],
            mat_red(),
        ));
        lights(&mut sc, 1);
        time(sc, 640, 480, 1, &format!("csg box-{n}sphere"));
    }

    println!("== 折射递归（玻璃球 vs 不透明球，640x480 aa=1，3 光源）");
    for (label, m) in [("opaque", mat_red()), ("glass", mat_glass())] {
        let mut sc = Scene::new(None);
        sc.add_object(obj(
            Geometry::SphereGeometry(SphereGeometry::new(1.0)),
            [0.0, 0.0, 0.0],
            m,
        ));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.7,
            [0.4, 1.0, 0.4],
        ));
        sc.add_light(Light::point(
            Color::from_hex(0xFFFFFF),
            0.6,
            [-5.0, 6.0, 4.0],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.3));
        time(sc, 640, 480, 1, label);
    }
}
