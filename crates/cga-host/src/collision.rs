//! 场景级碰撞扫描（计划 `docs/collision-plan.md` C1）：broad phase（包围盒 +
//! 同组免检）+ 窄相（`cga_collision`）+ 指纹缓存（帧间没变的对象对不重算，
//! 与增量渲染同一套失效哲学）。

use cga_collision::{probe, Contact, Hit};
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
                        cga_collision::contacts(
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

/// 关节行程干涉扫描（C4）：joint 的 q 从行程低端扫到高端（continuous 扫一整圈），
/// 其连杆（含嵌套子关节的 meshes，刚体随动）对场景其余对象逐一求螺旋 TOI。
///
/// v1 边界：只扫 1-DOF 关节（revolute/continuous/prismatic/helical）；其余关节
/// **冻结**（齿轮/凸轮耦合的联动扫描另行立项）；多 DOF / fixed / 无 limit 的
/// revolute → `skipped` 注明原因。
pub struct JointSweepOutcome {
    pub joint: String,
    /// 最早确切干涉：(对象下标, q*)。`q*` 在行程区间内。
    pub first: Option<(usize, f64)>,
    /// 无法判定的对象（三值诚实）。
    pub unknown: Vec<usize>,
    /// 非 None = 未执行（原因）。
    pub skipped: Option<String>,
}

/// 见 [`JointSweepOutcome`]。`joint_name` 实际是 **pair 名**（图模型；
/// 函数名保留以免调用点改动，文档以 pair 为准）。
pub fn sweep_joint(run: &crate::SceneRun, joint_name: &str) -> JointSweepOutcome {
    let kin = &run.kinematics;
    let Some(j) = kin
        .pairs
        .iter()
        .find(|p| p.name.as_deref() == Some(joint_name))
    else {
        return JointSweepOutcome {
            joint: joint_name.to_string(),
            first: None,
            unknown: Vec::new(),
            skipped: Some("no such pair".to_string()),
        };
    };
    let skip = |reason: &str| JointSweepOutcome {
        joint: joint_name.to_string(),
        first: None,
        unknown: Vec::new(),
        skipped: Some(reason.to_string()),
    };
    if !j.kind.is_1dof() {
        return skip("not a 1-DOF pair (cylindrical/spherical/planar/fixed)");
    }
    let q_cur = j.q.first().copied().unwrap_or(0.0);
    let (lo, hi) = match j.limit {
        Some([lo, hi]) => (lo, hi),
        None if j.kind == crate::scene_build::JointKind::Continuous => {
            (q_cur, q_cur + std::f64::consts::TAU)
        }
        None => return skip("no limit (revolute needs limit; continuous sweeps one turn)"),
    };
    if hi <= lo {
        return skip("empty limit range");
    }

    // 关节 frame（运动前）的世界矩阵 = fa_world（F_a 的世界位姿，求解器已给）。
    use crate::scene_build::{joint_motion, mat4_inv};
    use cga_core::mat4_mul;
    let pitch = j.pitch.unwrap_or(0.0);
    let mq = joint_motion(&j.kind, j.axis, &j.q, pitch);
    let f = j.fa_world;
    let o = [f[3], f[7], f[11]];
    let axis_w = {
        let a = [
            f[0] * j.axis[0] + f[1] * j.axis[1] + f[2] * j.axis[2],
            f[4] * j.axis[0] + f[5] * j.axis[1] + f[6] * j.axis[2],
            f[8] * j.axis[0] + f[9] * j.axis[1] + f[10] * j.axis[2],
        ];
        let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
        if n < 1e-12 {
            return skip("degenerate pair axis");
        }
        [a[0] / n, a[1] / n, a[2] / n]
    };

    // q(t) = lo + t·(hi−lo) 的物理螺旋速度（sweep_toi 要物理量纲）。
    let rate = hi - lo;
    let (w, v) = match j.kind {
        crate::scene_build::JointKind::Prismatic => ([0.0; 3], v3scale(axis_w, rate)),
        _ => {
            let w = v3scale(axis_w, rate);
            // ṗ = ω×(p−o) ⇒ v = o×ω；helical 再加节距平移 pitch·rate 沿轴。
            let mut v = v3cross(o, w);
            if j.kind == crate::scene_build::JointKind::Helical {
                v = v3add(v, v3scale(axis_w, pitch * rate));
            }
            (w, v)
        }
    };
    let xi = [w[0], w[1], w[2], v[0], v[1], v[2]];

    // 运动集合：tree_down 子树的全部连杆的 meshes（刚体随动）。
    let mut member_links: Vec<&str> = vec![j.tree_down.as_str()];
    let mut i = 0;
    while i < member_links.len() {
        let name = member_links[i];
        for p in kin.pairs.iter().filter(|p| p.tree_up == name) {
            member_links.push(p.tree_down.as_str());
        }
        i += 1;
    }
    let mut members: Vec<usize> = Vec::new();
    for l in kin.links.iter() {
        if member_links.contains(&l.name.as_str()) {
            members.extend_from_slice(&l.meshes);
        }
    }

    // 各 mesh 从当前姿态退到 q=lo：wa_lo = (F·M(lo)·M(q)⁻¹·F⁻¹)·wa_cur。
    let m_lo = joint_motion(&j.kind, j.axis, &[lo], pitch);
    let delta_lo = mat4_mul(f, mat4_mul(mat4_mul(m_lo, mat4_inv(mq)), mat4_inv(f)));
    let mut out = JointSweepOutcome {
        joint: joint_name.to_string(),
        first: None,
        unknown: Vec::new(),
        skipped: None,
    };
    let scene = &run.scene;
    for &mi in &members {
        let obj = &scene.objects[mi];
        let wa = mat4_mul(delta_lo, obj.motor().to_matrix());
        for (k, other) in scene.objects.iter().enumerate() {
            if members.contains(&k) {
                continue; // 同一运动集合：刚体随动，不算干涉
            }
            let (gi, gk) = (obj.group, other.group);
            if gi != 0 && gi == gk {
                continue; // 同组免检
            }
            let (h, t) = cga_collision::sweep_toi(
                xi,
                &obj.geometry,
                wa,
                &other.geometry,
                other.motor().to_matrix(),
                1.0,
            );
            match h {
                Hit::Yes => {
                    let q = lo + t.unwrap_or(0.0) * rate;
                    if out.first.is_none_or(|(_, bq)| q < bq) {
                        out.first = Some((k, q));
                    }
                }
                Hit::Unknown => out.unknown.push(k),
                Hit::No => {}
            }
        }
    }
    out
}

fn v3scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn v3add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn v3cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
/// 螺旋扫掠场景版（C3）：对象 `a` 以螺旋速度 `xi`（物理角速度 + 线速度）运动，
/// 对场景其余对象逐一求 TOI。同组（`group` 非 0 且相等）跳过——同组是刚体
/// 绑定，理应一起动。
pub struct SweepOutcome {
    /// 最早的确切接触（对象下标, toi）。
    pub first: Option<(usize, f64)>,
    /// 无法判定的对象（三值诚实：它们可能在 first 之前接触，由调用方决定如何处理）。
    pub unknown: Vec<usize>,
}

/// 见 [`SweepOutcome`]。
pub fn sweep_scene(xi: [f64; 6], a: usize, scene: &Scene, t_max: f64) -> SweepOutcome {
    let obj = &scene.objects[a];
    let wa = obj.motor().to_matrix();
    let mut out = SweepOutcome {
        first: None,
        unknown: Vec::new(),
    };
    for (j, o) in scene.objects.iter().enumerate() {
        if j == a || (obj.group != 0 && obj.group == o.group) {
            continue;
        }
        let (h, t) = cga_collision::sweep_toi(
            xi,
            &obj.geometry,
            wa,
            &o.geometry,
            o.motor().to_matrix(),
            t_max,
        );
        match h {
            Hit::Yes => {
                let t = t.unwrap_or(0.0);
                if out.first.is_none_or(|(_, bt)| t < bt) {
                    out.first = Some((j, t));
                }
            }
            Hit::Unknown => out.unknown.push(j),
            Hit::No => {}
        }
    }
    out
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
    fn sweep_scene_first_contact() {
        // 旋转臂（对象 0：球 r=0.5 在 (0,3,0)，绕原点 z 顺时针 ω=1）：
        // 先撞对象 1（(3,0,0) 的球，t* = π/2 − 2asin(1/6) ≈ 1.2359），
        // 后撞对象 2（(0,-3,0) 的球，t = π − Δφ* ≈ 2.807）。对象 3 与臂同组 → 跳过。
        let mut sc = Scene::new(None);
        sc.add_object(sphere_obj(0.0, 0.5, 7));
        sc.objects[0].position = [0.0, 3.0, 0.0];
        sc.add_object(sphere_obj(3.0, 0.5, 0));
        sc.add_object(sphere_obj(0.0, 0.5, 0));
        sc.objects[2].position = [0.0, -3.0, 0.0];
        sc.add_object(sphere_obj(1.5, 0.5, 7)); // 同组（7）：在路径附近但免检
        sc.objects[3].position = [1.5, 1.5, 0.0];
        let out = sweep_scene([0.0, 0.0, -1.0, 0.0, 0.0, 0.0], 0, &sc, 3.0);
        let (j, t) = out.first.expect("最早接触");
        assert_eq!(j, 1, "先撞 (3,0,0) 的立柱");
        let want = std::f64::consts::FRAC_PI_2 - 2.0 * (1.0f64 / 6.0).asin();
        assert!((t - want).abs() < 1e-9, "t={t} want={want}");
        assert!(out.unknown.is_empty(), "{:?}", out.unknown);
    }

    fn run_of(src: &str) -> crate::SceneRun {
        crate::run_jsx(src, None, ".").expect("run")
    }

    #[test]
    fn sweep_joint_closed_form() {
        // 关节臂：revolute 绕原点 z 转，臂上球 r=0.2 在半径 1.2；立柱球 r=0.5
        // 在 (1.5, 1.0, 0)。行程 [0, 1] 从 q=0 出发。
        // 接触：|c(q) − p| = 0.7 ⇒ cosΔ = (ρ²+d²−s²)/(2ρd)，q* = β − Δ。
        let src = r#"
export default (
  <scene>
    <camera />
    <translate t={[1.5, 1.0, 0]}><sphere r={0.5} /></translate>
    <link name="base" />
    <link name="arm_link"><translate t={[1.2, 0, 0]}><sphere r={0.2} /></translate></link>
    <pair kind="revolute" name="arm" a="base" b="arm_link" axis={[0, 0, 1]} at={[0, 0, 0]} limit={[0, 1.0]} />
    <anchor link="base" />
  </scene>
);
"#;
        let out = sweep_joint(&run_of(src), "arm");
        assert!(out.skipped.is_none(), "{:?}", out.skipped);
        let (idx, q) = out.first.expect("应撞到立柱");
        assert_eq!(idx, 0, "立柱是对象 0");
        let beta = 1.0f64.atan2(1.5);
        let d = (1.5f64 * 1.5 + 1.0).sqrt();
        let rho = 1.2;
        let want = beta - ((rho * rho + d * d - 0.7 * 0.7) / (2.0 * rho * d)).acos();
        assert!((q - want).abs() < 1e-9, "q={q} want={want}");
        assert!(out.unknown.is_empty());
    }

    #[test]
    fn sweep_joint_nested_closure() {
        // 嵌套关节：扫 base 时 elbow 的连杆球刚体随动（半径 2.4 绕原点）。
        // 立柱球 r=0.5 在 (2.0, 1.5, 0)。
        let src = r#"
export default (
  <scene>
    <camera />
    <translate t={[2.0, 1.5, 0]}><sphere r={0.5} /></translate>
    <link name="base" />
    <link name="upper" />
    <link name="fore"><translate t={[1.2, 0, 0]}><sphere r={0.2} /></translate></link>
    <pair kind="revolute" name="base" a="base" b="upper" axis={[0, 0, 1]} at={[0, 0, 0]} limit={[0, 1.0]} />
    <pair kind="revolute" name="elbow" a="upper" b="fore" axis={[0, 0, 1]} at={[1.2, 0, 0]} limit={[0, 1.0]} />
    <anchor link="base" />
  </scene>
);
"#;
        let run = run_of(src);
        // 扫 base：球绕原点半径 2.4
        let out = sweep_joint(&run, "base");
        let (idx, q) = out.first.expect("base 应撞到立柱");
        assert_eq!(idx, 0);
        let beta = 1.5f64.atan2(2.0);
        let d = 2.5f64;
        let want = beta - ((2.4 * 2.4 + d * d - 0.7 * 0.7) / (2.0 * 2.4 * d)).acos();
        assert!((q - want).abs() < 1e-9, "base: q={q} want={want}");
        // 扫 elbow：球绕 (1.2, 0) 半径 1.2
        let out2 = sweep_joint(&run, "elbow");
        let (idx2, q2) = out2.first.expect("elbow 应撞到立柱");
        assert_eq!(idx2, 0);
        let beta2 = 1.5f64.atan2(0.8);
        let d2 = (0.8f64 * 0.8 + 1.5 * 1.5).sqrt();
        let want2 = beta2 - ((1.44 + d2 * d2 - 0.49) / (2.0 * 1.2 * d2)).acos();
        assert!((q2 - want2).abs() < 1e-9, "elbow: q={q2} want={want2}");
    }

    #[test]
    fn sweep_joint_skips_and_clean() {
        // 多 DOF / fixed / 无 limit / 未知关节 → skipped；无干涉 → first=None
        let src = r#"
export default (
  <scene>
    <camera />
    <translate t={[10.0, 0.0, 0]}><sphere r={0.5} /></translate>
    <link name="base" />
    <link name="l_fix"><sphere r={0.2} /></link>
    <link name="l_ball"><sphere r={0.2} /></link>
    <link name="l_free"><sphere r={0.2} /></link>
    <link name="l_arm"><translate t={[1.2, 0, 0]}><sphere r={0.2} /></translate></link>
    <pair kind="fixed" name="fix" a="base" b="l_fix" at={[0, 0, 0]} />
    <pair kind="spherical" name="ball" a="base" b="l_ball" axis={[0, 0, 1]} at={[0, 0, 0]} />
    <pair kind="revolute" name="free" a="base" b="l_free" axis={[0, 0, 1]} at={[0, 0, 0]} />
    <pair kind="revolute" name="arm" a="base" b="l_arm" axis={[0, 0, 1]} at={[0, 0, 0]} limit={[0, 1.0]} />
    <anchor link="base" />
  </scene>
);
"#;
        let run = run_of(src);
        assert_eq!(
            sweep_joint(&run, "fix").skipped.as_deref(),
            Some("not a 1-DOF pair (cylindrical/spherical/planar/fixed)")
        );
        assert!(sweep_joint(&run, "ball").skipped.is_some());
        assert!(
            sweep_joint(&run, "free").skipped.is_some(),
            "revolute 无 limit"
        );
        assert_eq!(
            sweep_joint(&run, "nope").skipped.as_deref(),
            Some("no such pair")
        );
        let clean = sweep_joint(&run, "arm");
        assert!(clean.skipped.is_none());
        assert_eq!(clean.first, None, "立柱在 x=10，行程内碰不到");
        // 运动集合不互相误报：arm 自己的球不在干涉里
        assert!(clean.unknown.is_empty());
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
