//! 场景级碰撞扫描（计划 `docs/collision-plan.md` C1）：broad phase（包围盒 +
//! 同组免检）+ 窄相（`cga_core::collision`）+ 指纹缓存（帧间没变的对象对不重算，
//! 与增量渲染同一套失效哲学）。

use cga_core::collision::{probe, Contact, Hit};
use cga_gpu::scene::{Object, ObjectParams, Scene};
use cga_gpu::scene_graph::Color;
use cga_gpu::shading::{Material, MaterialParams};
use std::collections::HashMap;

/// 一个对象对的结果。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PairHit {
    /// `scene.objects` 下标。
    pub a: usize,
    pub b: usize,
    pub hit: Hit,
    /// 分离距离（相离正 / 相切 0 / 穿入负）；`None` = Unknown。
    pub separation: Option<f64>,
}

/// 接触命中：对象对 + 接触点列表（`None` = Unknown 对）。
#[derive(Clone, Debug, PartialEq)]
pub struct ContactHit {
    pub a: usize,
    pub b: usize,
    /// `Some(vec![])` = 确切无接触；`Some(非空)` = 接触/穿入；`None` = Unknown。
    pub contacts: Option<Vec<Contact>>,
}

/// 碰撞扫描器：持有帧间指纹缓存。
#[derive(Default)]
pub struct CollisionScan {
    cache: HashMap<(u64, u64), (Hit, Option<f64>)>,
    /// 上次 scan 的缓存命中数（测试/调优用）。
    pub cache_hits: usize,
    /// 上次 scan 的窄相计算数。
    pub computed: usize,
}

fn fp<T: std::fmt::Debug>(x: &T) -> u64 {
    use std::hash::Hasher;
    let mut h = std::collections::hash_map::DefaultHasher::new();
    std::hash::Hash::hash(&format!("{x:?}"), &mut h);
    h.finish()
}

impl CollisionScan {
    pub fn new() -> Self {
        Self::default()
    }

    /// 全对扫描。同组（`group` 非 0 且相等）的对象对免检跳过：同组语义是
    /// 同一连杆/装配体的刚体绑定，互相重叠是预期而非干涉。
    ///
    /// 缓存键是对象对的指纹（几何 + 世界变换），与对象在表中的位置无关。
    pub fn scan(&mut self, scene: &cga_gpu::scene::Scene) -> Vec<PairHit> {
        self.cache_hits = 0;
        self.computed = 0;
        let objs = &scene.objects;
        let fps: Vec<u64> = objs.iter().map(|o| fp(&(o.motor(), &o.geometry))).collect();
        let mut out = Vec::new();
        for i in 0..objs.len() {
            for j in i + 1..objs.len() {
                let (gi, gj) = (objs[i].group, objs[j].group);
                if gi != 0 && gi == gj {
                    continue;
                }
                let key = (fps[i], fps[j]);
                let (hit, sep) = match self.cache.get(&key) {
                    Some(&r) => {
                        self.cache_hits += 1;
                        r
                    }
                    None => {
                        let wa = objs[i].motor().to_matrix();
                        let wb = objs[j].motor().to_matrix();
                        let r = probe(&objs[i].geometry, wa, &objs[j].geometry, wb);
                        self.cache.insert(key, r);
                        self.computed += 1;
                        r
                    }
                };
                out.push(PairHit {
                    a: i,
                    b: j,
                    hit,
                    separation: sep,
                });
            }
        }
        out
    }

    /// 扫描并对 Yes 对解接触点（C2）。接触解只跑在接触对上；Unknown 对的
    /// `contacts` 为 None。
    pub fn scan_contacts(&mut self, scene: &Scene) -> Vec<ContactHit> {
        let hits = self.scan(scene);
        hits.into_iter()
            .map(|h| {
                let contacts = match h.hit {
                    Hit::Yes => {
                        let wa = scene.objects[h.a].motor().to_matrix();
                        let wb = scene.objects[h.b].motor().to_matrix();
                        cga_core::collision::contacts(
                            &scene.objects[h.a].geometry,
                            wa,
                            &scene.objects[h.b].geometry,
                            wb,
                        )
                    }
                    Hit::No => Some(Vec::new()),
                    Hit::Unknown => None,
                };
                ContactHit {
                    a: h.a,
                    b: h.b,
                    contacts,
                }
            })
            .collect()
    }
}

/// 接触标记（C2 可视化）：每个接触点一个红色发光小球，法向一根黄色细柱——
/// 普通对象通道，直接 `scene.add_object` 即可渲染。`size` = 标记球半径
/// （按场景尺度取，如包围盒对角线的 1–2%）。
pub fn contact_markers(hits: &[ContactHit], size: f64) -> Vec<Object> {
    let mut out = Vec::new();
    let dot_mat = || {
        Material::standard(MaterialParams {
            color: Color::from_hex(0xFF2222),
            roughness: 1.0,
            metalness: 0.0,
            emissive: Color::from_hex(0xFF2222),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        })
    };
    let stick_mat = || {
        Material::standard(MaterialParams {
            color: Color::from_hex(0xFFCC00),
            roughness: 1.0,
            metalness: 0.0,
            emissive: Color::from_hex(0xFFCC00),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        })
    };
    for h in hits {
        let Some(cs) = &h.contacts else { continue };
        for c in cs {
            out.push(
                Object::new(ObjectParams {
                    geometry: cga_core::Geometry::SphereGeometry(cga_core::SphereGeometry::new(
                        size,
                    )),
                    material: dot_mat(),
                    position: c.point,
                    rotation_axis: [0.0, 0.0, 1.0],
                    rotation_angle: 0.0,
                    motor: None,
                })
                .with_group(0),
            );
            // 法向细柱：从接触点沿 normal 伸出 4·size
            let n = c.normal;
            let len = 4.0 * size;
            let m = motor_from_z_to(n);
            let center = [
                c.point[0] + n[0] * len / 2.0,
                c.point[1] + n[1] * len / 2.0,
                c.point[2] + n[2] * len / 2.0,
            ];
            let tm = cga_core::Multivector::translator(center);
            out.push(Object::new(ObjectParams {
                geometry: cga_core::Geometry::CylinderGeometry(cga_core::CylinderGeometry::new(
                    size / 4.0,
                    len,
                )),
                material: stick_mat(),
                position: [0.0; 3],
                rotation_axis: [0.0, 0.0, 1.0],
                rotation_angle: 0.0,
                motor: Some(tm.compose(&m)),
            }));
        }
    }
    out
}

/// 把局部 +z 转到方向 n 的 motor（n 单位）。
fn motor_from_z_to(n: [f64; 3]) -> cga_core::Multivector {
    let z = [0.0, 0.0, 1.0];
    let d = z[0] * n[0] + z[1] * n[1] + z[2] * n[2];
    if d > 1.0 - 1e-12 {
        return cga_core::Multivector::identity();
    }
    if d < -1.0 + 1e-12 {
        return cga_core::Multivector::rotor([1.0, 0.0, 0.0], std::f64::consts::PI);
    }
    let axis = [
        z[1] * n[2] - z[2] * n[1],
        z[2] * n[0] - z[0] * n[2],
        z[0] * n[1] - z[1] * n[0],
    ];
    cga_core::Multivector::rotor(axis, d.clamp(-1.0, 1.0).acos())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{Geometry, SphereGeometry};
    use cga_gpu::scene::{Object, ObjectParams, Scene};
    use cga_gpu::scene_graph::Color;
    use cga_gpu::shading::{Material, MaterialParams};

    fn sphere_obj(x: f64, r: f64, group: u32) -> Object {
        Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(r)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xFFFFFF),
                roughness: 0.5,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [x, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        })
        .with_group(group)
    }

    #[test]
    fn scan_pairs_and_group_skip() {
        let mut sc = Scene::new(None);
        sc.add_object(sphere_obj(0.0, 1.0, 1)); // 0
        sc.add_object(sphere_obj(1.5, 1.0, 1)); // 1：与 0 重叠，但同组 → 免检
        sc.add_object(sphere_obj(5.0, 1.0, 2)); // 2：远
        sc.add_object(sphere_obj(6.2, 1.0, 0)); // 3：与 2 重叠（sep = 6.2−5−2 = −0.8）
        let hits = CollisionScan::new().scan(&sc);
        assert_eq!(hits.len(), 5, "同组对 (0,1) 被跳过: {hits:?}");
        let find = |a: usize, b: usize| hits.iter().find(|h| h.a == a && h.b == b).copied();
        let h23 = find(2, 3).expect("pair 2-3");
        assert_eq!(h23.hit, Hit::Yes);
        assert!((h23.separation.unwrap() + 0.8).abs() < 1e-9, "{h23:?}");
        assert_eq!(find(0, 2).unwrap().hit, Hit::No);
    }

    fn fnv1a(bytes: &[u8]) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }

    #[test]
    fn scan_cache_reuse_across_scans() {
        let mut sc = Scene::new(None);
        sc.add_object(sphere_obj(0.0, 1.0, 0));
        sc.add_object(sphere_obj(5.0, 1.0, 0));
        sc.add_object(sphere_obj(9.0, 1.0, 0));
        let mut scan = CollisionScan::new();
        let first = scan.scan(&sc);
        assert_eq!(scan.computed, 3);
        assert_eq!(scan.cache_hits, 0);
        let second = scan.scan(&sc);
        assert_eq!(scan.computed, 0, "全缓存命中");
        assert_eq!(scan.cache_hits, 3);
        assert_eq!(first, second, "缓存结果与计算结果一致");
        // 动一个对象：只有含它的两对重算
        sc.objects[1].position = [5.5, 0.0, 0.0];
        scan.scan(&sc);
        assert_eq!(scan.computed, 2);
        assert_eq!(scan.cache_hits, 1);
    }

    #[test]
    fn contact_markers_render_golden() {
        use cga_core::{BoxGeometry, PlaneGeometry};
        use cga_gpu::scene::PerspectiveCamera;
        use cga_gpu::shading::Light;

        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0x7F8C8D),
                roughness: 0.9,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        // 落地的盒（恰好接触地面）+ 两个相交的球（sep = −0.4）
        let mut bx_ = sphere_obj(0.0, 1.0, 0);
        bx_.geometry = Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0));
        bx_.position = [0.0, 0.5, 0.0];
        sc.add_object(bx_);
        sc.add_object(sphere_obj(1.4, 0.5, 0));
        sc.objects[2].position = [1.4, 0.5, 0.0];
        sc.add_object(sphere_obj(2.0, 0.5, 0));
        sc.objects[3].position = [2.0, 0.5, 0.0];
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.5, 1.0, 0.5],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.35));

        let hits = CollisionScan::new().scan_contacts(&sc);
        // 接触对：地面-盒、地面-球×2、球-球
        let yes = hits
            .iter()
            .filter(|h| h.contacts.as_ref().is_some_and(|c| !c.is_empty()))
            .count();
        assert_eq!(yes, 4, "接触对数: {hits:?}");
        for m in contact_markers(&hits, 0.04) {
            sc.add_object(m);
        }

        let mut cam = PerspectiveCamera::new(
            45.0,
            96.0 / 72.0,
            0.1,
            100.0,
            [3.2, 2.4, 4.5],
            [0.8, 0.3, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.8, 0.3, 0.0], None);
        let mut r = cga_gpu::Renderer::new(96, 72, 1, 3);
        let img = r.render(sc, cam);
        let png = cga_gpu::frame_to_png_bytes(&img);
        assert_eq!(
            fnv1a(&png),
            0x9e5e_200a_aa7e_b143,
            "接触标记渲染金标不符（{} 字节）",
            png.len()
        );
    }
}
