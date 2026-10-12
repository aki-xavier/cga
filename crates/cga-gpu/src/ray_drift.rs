//! 漂移防线（阶段 3，见 `docs/geometry-unification.md` §4）。
//!
//! 图元求交有三条**实现**：f32 MLX 数组内核、手写 MSL（BVH 内核）、f64 标量（`cga_core`
//! 的 `ray` 系数/法线 + `certify` 的精确求根）。几何定义只有一份，这个模块负责证明另外
//! 两条没有漂移：命中/未命中一致、最近 `t` 一致、命中点落在 `cga_core` 定义的同一张曲面上、
//! 法线同向。
//!
//! 歧义带（f32 判别式落在相消带内、f32 判不出符号）按设计**跳过**——那正是 f64 回退存在
//! 的理由；这里只要求"能判的地方必须判成一模一样"。

use crate::fallback::{test_rows, RowHit};
use crate::geom_intersect;
use cga_core::{
    vec3_add, vec3_dot, vec3_len, vec3_scale, vec3_sub, BoxGeometry, ConeGeometry, CyclideGeometry,
    CylinderGeometry, EllipsoidGeometry, Geometry, GeometryParams, PlaneGeometry, SphereGeometry,
    TorusGeometry,
};
use mlx_rs::Array;

/// 固定种子 LCG（仓库惯例：确定性、无 RNG 依赖）。
struct Lcg(u64);
impl Lcg {
    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

fn ray_batch(n: usize) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    // 一半是"命中型"光线（从球壳出发打向图元内部的抖动目标），一半是随机方向的
    // "未命中型"光线——两条都要覆盖，且命中率足够高才有比较价值。
    let mut rng = Lcg(0x5eed_1234_abcd_0001);
    let mut os = Vec::with_capacity(n);
    let mut ds = Vec::with_capacity(n);
    while os.len() < n {
        let mut dir = [
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ];
        let l = vec3_len(dir);
        if l < 1e-6 {
            continue;
        }
        dir = vec3_scale(dir, 1.0 / l);
        let r = rng.range(2.0, 3.0);
        let o = vec3_scale(dir, r);
        let mut d = if os.len() % 2 == 0 {
            let target = [
                rng.range(-0.5, 0.5),
                rng.range(-0.5, 0.5),
                rng.range(-0.5, 0.5),
            ];
            vec3_sub(target, o)
        } else {
            [
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
            ]
        };
        let dl = vec3_len(d);
        if dl < 1e-6 {
            continue;
        }
        d = vec3_scale(d, 1.0 / dl);
        os.push(o);
        ds.push(d);
    }
    (os, ds)
}

fn batch_array(v: &[[f64; 3]]) -> Array {
    let flat: Vec<f32> = v.iter().flat_map(|p| p.map(|x| x as f32)).collect();
    Array::from_slice(&flat, &[v.len() as i32, 3])
}

fn read_f32(a: &Array) -> Vec<f32> {
    a.eval().unwrap();
    a.as_slice::<f32>().to_vec()
}

fn read_mask(a: &Array) -> Vec<bool> {
    a.eval().unwrap();
    a.as_slice::<bool>().to_vec()
}

fn read_i32(a: &Array) -> Vec<i32> {
    a.eval().unwrap();
    a.as_slice::<i32>().to_vec()
}

/// 歧义判定：`|disc| ≤ 8·ε_f32·max(b², 4|ac|)`（与内核 `quad_ambig` 同一规则）。
fn ambiguous_quad(a: f64, b: f64, c: f64) -> bool {
    let scale = (b * b).max(4.0 * (a * c).abs()).max(1e-30);
    (b * b - 4.0 * a * c).abs() <= scale * 8.0 * (f32::EPSILON as f64)
}

/// 四次式的"近切"判定（f32 求根在近重根处不可靠）：`[-2, 20]` 上采样，
/// 出现 |f| 与 |f'| 同时很小的地方即视为近切。
fn near_tangent_quartic(c: [f64; 4]) -> bool {
    let f = |t: f64| (((t + c[0]) * t + c[1]) * t + c[2]) * t + c[3];
    let df = |t: f64| ((4.0 * t + 3.0 * c[0]) * t + 2.0 * c[1]) * t + c[2];
    for k in 0..=2200 {
        let x = -2.0 + k as f64 * 0.01;
        if f(x).abs() < 1e-4 && df(x).abs() < 5e-3 {
            return true;
        }
    }
    false
}

/// f64 最近交叉：返回 `(t, 法线, 入射根)`——`t ≥ 0` 的近根、以及最小的有限根（判定
/// 光线是否从实体内部射出，内核按此翻转法线朝向）。`None` = 未命中。
fn nearest(rows: Vec<RowHit>) -> Option<(f64, [f64; 3], f64)> {
    let entry = rows
        .iter()
        .filter(|h| h.t.is_finite())
        .map(|h| h.t)
        .fold(f64::INFINITY, f64::min);
    rows.into_iter()
        .filter(|h| h.t.is_finite() && h.t >= 0.0)
        .min_by(|a, b| a.t.total_cmp(&b.t))
        .map(|h| (h.t, h.n, entry))
}

struct Drift {
    compared: usize,
    hits: usize,
    skipped: usize,
}

/// 核对比：f32 数组内核（性能实现）vs f64 标量定义（core 系数/法线 + certify 求根）。
///
/// `band`：该光线是否落在 f32 判不出符号的歧义带内（是则跳过比较）；
/// `canon`：逐光线的 f64 最近交叉（`None` = 未命中）。
fn compare(
    name: &str,
    params: &GeometryParams,
    o64: &[[f64; 3]],
    d64: &[[f64; 3]],
    band: impl Fn([f64; 3], [f64; 3]) -> bool,
    canon: impl Fn([f64; 3], [f64; 3]) -> Option<(f64, [f64; 3], f64)>,
) -> Drift {
    let o = batch_array(o64);
    let d = batch_array(d64);
    let n = o64.len();
    let (ts_f32, ns_f32, vs_f32) = geom_intersect(params, &o, &d);
    let t32 = read_f32(&ts_f32);
    let v32 = read_mask(&vs_f32);
    ns_f32.eval().unwrap();
    let n32 = ns_f32.as_slice::<f32>();

    let mut out = Drift {
        compared: 0,
        hits: 0,
        skipped: 0,
    };
    for i in 0..n {
        let (o, d) = (o64[i], d64[i]);
        if band(o, d) {
            out.skipped += 1;
            continue;
        }
        out.compared += 1;
        let hit32 = v32[i] && t32[i].is_finite();
        let got64 = canon(o, d);
        assert_eq!(
            hit32,
            got64.is_some(),
            "{name} 光线 {i}: 命中不一致（数组 {hit32} t={} vs f64 {got64:?}）",
            t32[i]
        );
        let Some((t64, n64, entry)) = got64 else {
            continue;
        };
        out.hits += 1;
        let a = f64::from(t32[i]);
        let rel = (a - t64).abs() / t64.abs().max(1.0);
        assert!(
            rel < 1e-3,
            "{name} 光线 {i}: t 偏差 {rel}（数组 {a} vs f64 {t64}）"
        );
        // 命中点必须落在 core 定义的那张曲面上（core 的精确距离，存在时）
        let p = vec3_add(o, vec3_scale(d, t64));
        if let Some(sep) = params.distance(p) {
            assert!(
                sep.abs() < 1e-4,
                "{name} 光线 {i}: f64 命中点不在曲面上（distance={sep}）"
            );
        }
        // 法线：数组侧有限且与 f64 同向
        let na = [
            f64::from(n32[i * 3]),
            f64::from(n32[i * 3 + 1]),
            f64::from(n32[i * 3 + 2]),
        ];
        assert!(
            na.iter().all(|x| x.is_finite()),
            "{name} 光线 {i}: 数组法线非有限"
        );
        let dp = vec3_dot(na, n64);
        if entry > 1e-6 {
            // 外侧命中：法线必须同向（真正的公式漂移会在这里暴露）
            assert!(
                dp > 0.99,
                "{name} 光线 {i}: 法线不同向（数组 {na:?} vs f64 {n64:?}）"
            );
        } else {
            // 内侧命中：内核按渲染约定把法线翻向入射方向（geometry_ops.rs 的 `inside` 分支）
            assert!(
                dp.abs() > 0.99,
                "{name} 光线 {i}: 法线不同轴（数组 {na:?} vs f64 {n64:?}）"
            );
        }
    }
    out
}

#[test]
fn array_kernel_agrees_with_the_f64_canonical_definition() {
    let (o, d) = ray_batch(320);

    // 球
    let sp = Geometry::SphereGeometry(SphereGeometry::new(1.2)).identity_params();
    let r = compare(
        "sphere",
        &sp,
        &o,
        &d,
        |o, d| {
            let (a, b, c) = cga_core::sphere_quad(o, d, [0.0; 3], 1.2);
            ambiguous_quad(a, b, c)
        },
        |o, d| {
            if let GeometryParams::SphereParams(p) = &sp {
                nearest(test_rows::sphere(p, o, d))
            } else {
                unreachable!()
            }
        },
    );
    assert!(
        r.hits > 60 && r.compared > 200,
        "球：hits={} compared={}",
        r.hits,
        r.compared
    );

    // 平面（跳过近平行：内核在 |n·d| ≤ 1e-9 时判无命中）
    let pl = Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, -0.2], 0.5)).identity_params();
    let r = compare(
        "plane",
        &pl,
        &o,
        &d,
        |_o, d| {
            if let GeometryParams::PlaneParams(p) = &pl {
                vec3_dot(p.n, d).abs() <= 1e-9
            } else {
                false
            }
        },
        |o, d| {
            if let GeometryParams::PlaneParams(p) = &pl {
                nearest(test_rows::plane(p, o, d))
            } else {
                unreachable!()
            }
        },
    );
    assert!(r.hits > 60, "平面：hits={}", r.hits);

    // 柱（有限长）
    let cy = Geometry::CylinderGeometry(CylinderGeometry::new(0.8, 2.4)).identity_params();
    let r = compare(
        "cylinder",
        &cy,
        &o,
        &d,
        |o, d| {
            if let GeometryParams::CylinderParams(p) = &cy {
                let (a, b, c) = cga_core::cylinder_quad(o, d, p.q, p.u, p.r);
                (a != 0.0 || b != 0.0) && ambiguous_quad(a, b, c)
            } else {
                false
            }
        },
        |o, d| {
            if let GeometryParams::CylinderParams(p) = &cy {
                nearest(test_rows::cylinder(p, o, d))
            } else {
                unreachable!()
            }
        },
    );
    assert!(r.hits > 40, "柱：hits={}", r.hits);

    // 锥
    let co = Geometry::ConeGeometry(ConeGeometry::new(0.7, 1.8)).identity_params();
    let r = compare(
        "cone",
        &co,
        &o,
        &d,
        |o, d| {
            if let GeometryParams::ConeParams(p) = &co {
                // 与内核 `cone_local_interval` 的 amb 一致：|a| 退化 或 判别式相消带
                let (a, b, c) = cga_core::cone_quad(p.r, p.h, o, d);
                let k2 = 1.0 + (p.r / p.h) * (p.r / p.h);
                let a_band = 1.0f64.max(k2 * d[2] * d[2]) * 8.0 * (f32::EPSILON as f64);
                a.abs() <= a_band || ambiguous_quad(a, b, c)
            } else {
                false
            }
        },
        |o, d| {
            if let GeometryParams::ConeParams(p) = &co {
                nearest(test_rows::cone(p, o, d))
            } else {
                unreachable!()
            }
        },
    );
    assert!(r.hits > 20, "锥：hits={}", r.hits);

    // 椭球
    let el = Geometry::EllipsoidGeometry(EllipsoidGeometry::new(1.1, 0.6, 0.9)).identity_params();
    let r = compare(
        "ellipsoid",
        &el,
        &o,
        &d,
        |o, d| {
            if let GeometryParams::EllipsoidParams(p) = &el {
                match cga_core::local_ray(p.a_inv3, p.t_inv, o, d) {
                    Some((o_l, d_u, _)) => {
                        let (a, b, c) = cga_core::ellipsoid_quad(o_l, d_u);
                        ambiguous_quad(a, b, c)
                    }
                    None => true,
                }
            } else {
                false
            }
        },
        |o, d| {
            if let GeometryParams::EllipsoidParams(p) = &el {
                nearest(test_rows::ellipsoid(p, o, d))
            } else {
                unreachable!()
            }
        },
    );
    assert!(r.hits > 40, "椭球：hits={}", r.hits);

    // 环面（四次式：近切光线交给歧义判定跳过）
    let to = Geometry::TorusGeometry(TorusGeometry::new(1.4, 0.45)).identity_params();
    let r = compare(
        "torus",
        &to,
        &o,
        &d,
        |o, d| {
            if let GeometryParams::TorusParams(p) = &to {
                near_tangent_quartic(cga_core::torus_quartic(p.major, p.minor, o, d))
            } else {
                false
            }
        },
        |o, d| {
            if let GeometryParams::TorusParams(p) = &to {
                nearest(test_rows::torus(p, o, d))
            } else {
                unreachable!()
            }
        },
    );
    assert!(r.hits > 8, "环面：hits={}", r.hits);

    // 环纹面
    let cyc =
        Geometry::CyclideGeometry(CyclideGeometry::new(1.0, 0.5, 0.25, [0.0; 3])).identity_params();
    let r = compare(
        "cyclide",
        &cyc,
        &o,
        &d,
        |o, d| {
            if let GeometryParams::CyclideParams(p) = &cyc {
                near_tangent_quartic(cga_core::cyclide_quartic(p.a, p.b, p.d, p.c, p.shift, o, d))
            } else {
                false
            }
        },
        |o, d| {
            if let GeometryParams::CyclideParams(p) = &cyc {
                nearest(test_rows::cyclide(p, o, d))
            } else {
                unreachable!()
            }
        },
    );
    assert!(r.hits > 3, "环纹面：hits={}", r.hits);
}

// ---- MSL（BVH 内核）的逐图元漂移 ------------------------------------------

use crate::bvh_trace::BvhTrace;
use crate::RenderMode;
use cga_scene::{Color, Material, MaterialParams, Object, ObjectParams, Scene};

fn red_material() -> Material {
    Material::standard(MaterialParams {
        color: Color::from_hex(0xC0392B),
        roughness: 0.6,
        metalness: 0.0,
        emissive: Color::from_hex(0x000000),
        opacity: 1.0,
        ior: 1.5,
        absorption: 0.0,
    })
}

/// MSL 内核覆盖的 5 个图元（内核 `switch` 只认这 5 种）。
fn msl_targets() -> Vec<(&'static str, Geometry)> {
    vec![
        ("sphere", Geometry::SphereGeometry(SphereGeometry::new(0.6))),
        (
            "box",
            Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
        ),
        (
            "cylinder",
            Geometry::CylinderGeometry(CylinderGeometry::new(0.5, 1.4)),
        ),
        ("cone", Geometry::ConeGeometry(ConeGeometry::new(0.55, 1.2))),
        (
            "ellipsoid",
            Geometry::EllipsoidGeometry(EllipsoidGeometry::new(0.7, 0.5, 0.6)),
        ),
    ]
}

/// 33 个同类型对象：1 个目标在原点（带一点旋转），32 个诱饵在远处环上
/// （对象数 ≥ 32 才走 BVH 内核）。
fn scene_with_target(target: &Geometry) -> Scene {
    let mut sc = Scene::new(None);
    sc.add_object(Object::new(ObjectParams {
        geometry: target.clone(),
        material: red_material(),
        position: [0.0, 0.0, 0.0],
        rotation_axis: [0.0, 1.0, 0.0],
        rotation_angle: 0.35,
        motor: None,
    }));
    for k in 0..32 {
        let a = k as f64 * 0.21;
        sc.add_object(Object::new(ObjectParams {
            geometry: target.clone(),
            material: red_material(),
            position: [a.cos() * 30.0, 5.0 + (k % 5) as f64, a.sin() * 30.0],
            rotation_axis: [0.0, 1.0, 0.0],
            rotation_angle: 0.0,
            motor: None,
        }));
    }
    sc
}

/// 瞄准目标内部固定抖动的光线（从半径 6..9 的球壳出发）⇒ 绝大多数命中目标本身。
fn aim_rays(n: usize, jitter: f64) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let mut rng = Lcg(0x9e37_79b9_7f4a_7c15);
    let mut os = Vec::with_capacity(n);
    let mut ds = Vec::with_capacity(n);
    while os.len() < n {
        let mut u = [
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ];
        let l = vec3_len(u);
        if l < 1e-6 {
            continue;
        }
        u = vec3_scale(u, 1.0 / l);
        let o = vec3_scale(u, rng.range(6.0, 9.0));
        let t = [
            rng.range(-jitter, jitter),
            rng.range(-jitter, jitter),
            rng.range(-jitter, jitter),
        ];
        let d = vec3_sub(t, o);
        let dl = vec3_len(d);
        if dl < 1e-6 {
            continue;
        }
        os.push(o);
        ds.push(vec3_scale(d, 1.0 / dl));
    }
    (os, ds)
}

/// MSL（BVH 内核）**逐图元**漂移测试：瞄准目标的光线，其最近命中必须与
/// `cga_core` 定义的曲面一致，并与另外两条实现一致（f64 标量定义、f32 数组内核）。
///
/// 盒没有 f64 行解算器（slab 判定在 f32 下本身就精确，不需要认证回退），
/// 因此盒的对照是数组内核；其余四个图元三者都要一致。
#[test]
fn msl_bvh_kernel_agrees_per_primitive() {
    for (name, target) in msl_targets() {
        let sc = scene_with_target(&target);
        let params_list: Vec<GeometryParams> = sc
            .objects
            .iter()
            .map(|o| o.geometry.params_in_frame(&o.motor()))
            .collect();
        let Some(bvh) = BvhTrace::build(&sc, &params_list, RenderMode::Normal) else {
            eprintln!("SKIP msl_drift {name}: MLX/Metal 内核不可用");
            continue;
        };
        // 全部对象都必须真的走内核（否则测的就不是 MSL 了）
        assert!(
            bvh.legacy.iter().all(|&l| !l),
            "{name}: 有对象落回旧路径，MSL 覆盖不完整"
        );
        let (o64, d64) = aim_rays(64, 0.22);
        let o = batch_array(&o64);
        let d = batch_array(&d64);
        // MSL 内核返回 (t, 对象下标, 法线)：miss ⇒ t = +∞、idx = −1
        let (ts, idxs, ns) = bvh.trace_nearest(&o, &d).expect("trace_nearest");
        let t_msl = read_f32(&ts);
        let idx_msl = read_i32(&idxs);
        ns.eval().unwrap();
        let n_msl = ns.as_slice::<f32>();
        // 第二参照：数组内核（只对目标对象）
        let (ats, _ans, avs) = geom_intersect(&params_list[0], &o, &d);
        let t_arr = read_f32(&ats);
        let v_arr = read_mask(&avs);

        let mut hits = 0;
        for i in 0..o64.len() {
            let hit_msl = t_msl[i].is_finite() && t_msl[i] > 1e-6;
            let hit_arr = v_arr[i] && t_arr[i].is_finite();
            assert_eq!(
                hit_msl, hit_arr,
                "{name} 光线 {i}: MSL {hit_msl}(t={}, idx={}) 与数组 {hit_arr}(t={}) 不一致",
                t_msl[i], idx_msl[i], t_arr[i]
            );
            if !hit_msl {
                continue;
            }
            hits += 1;
            // 命中必须是目标对象（0 号）——同时验证 BVH 遍历挑对了叶
            assert_eq!(
                idx_msl[i], 0,
                "{name} 光线 {i}: MSL 命中了 {} 号对象（诱饵）而非目标",
                idx_msl[i]
            );
            let rel = f64::from((t_msl[i] - t_arr[i]).abs()) / f64::from(t_arr[i].abs().max(1.0));
            assert!(rel < 1e-3, "{name} 光线 {i}: MSL/数组 t 偏差 {rel}");
            // 命中点落在 core 定义的曲面上
            let p = vec3_add(o64[i], vec3_scale(d64[i], f64::from(t_msl[i])));
            if let Some(sep) = params_list[0].distance(p) {
                assert!(
                    sep.abs() < 1e-3,
                    "{name} 光线 {i}: MSL 命中点不在 core 曲面上（distance={sep}）"
                );
            }
            // 与 f64 标量定义（有行解算器的图元）
            let got64 = match (&params_list[0], name) {
                (GeometryParams::SphereParams(p), _) => {
                    nearest(test_rows::sphere(p, o64[i], d64[i]))
                }
                (GeometryParams::CylinderParams(p), _) => {
                    nearest(test_rows::cylinder(p, o64[i], d64[i]))
                }
                (GeometryParams::ConeParams(p), _) => nearest(test_rows::cone(p, o64[i], d64[i])),
                (GeometryParams::EllipsoidParams(p), _) => {
                    nearest(test_rows::ellipsoid(p, o64[i], d64[i]))
                }
                _ => None, // 盒：无 f64 行解算器，数组内核即对照
            };
            if let Some((t64, n64, entry)) = got64 {
                let rel = (f64::from(t_msl[i]) - t64).abs() / t64.abs().max(1.0);
                assert!(rel < 1e-3, "{name} 光线 {i}: MSL/f64 t 偏差 {rel}");
                let na = [
                    f64::from(n_msl[i * 3]),
                    f64::from(n_msl[i * 3 + 1]),
                    f64::from(n_msl[i * 3 + 2]),
                ];
                let dp = vec3_dot(na, n64);
                if entry > 1e-6 {
                    assert!(dp > 0.99, "{name} 光线 {i}: MSL 法线不同向（{dp}）");
                } else {
                    assert!(dp.abs() > 0.99, "{name} 光线 {i}: MSL 法线不同轴（{dp}）");
                }
            }
        }
        assert!(hits > 40, "{name}: 有效命中太少（{hits}）");
    }
}
