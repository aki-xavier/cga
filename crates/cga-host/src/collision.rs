//! 场景级碰撞扫描（计划 `docs/collision-plan.md` C1）：broad phase（包围盒 +
//! 同组免检）+ 窄相（`cga_core::collision`）+ 指纹缓存（帧间没变的对象对不重算，
//! 与增量渲染同一套失效哲学）。

use cga_core::collision::{probe, Hit};
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
}
