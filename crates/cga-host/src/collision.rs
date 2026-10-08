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
    /// 分离距离（相离正 / 相切 0 / 穿入负）；`None` = Unknown，或 AABB 宽相
    /// 认证相离（D3 大场景：判定是精确的 No，只是没算距离）。
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
    /// D1：Yes 对的接触解缓存（同一指纹键）；接触解比 probe 贵得多，是帧间大头。
    contact_cache: HashMap<(u64, u64), Option<Vec<Contact>>>,
    /// 上次 scan 的缓存命中数（测试/调优用）。
    pub cache_hits: usize,
    /// 上次 scan 的窄相计算数。
    pub computed: usize,
    /// 上次 scan_contacts 的接触解缓存命中数。
    pub contact_cache_hits: usize,
}

fn fingerprints(objs: &[Object]) -> Vec<u64> {
    objs.iter().map(|o| fp(&(o.motor(), &o.geometry))).collect()
}

/// 空间索引阈值：对象数超过此值启用 AABB 宽相（小场景保持全对精确距离）。
pub const SPATIAL_INDEX_THRESHOLD: usize = 64;

/// 对象的世界 AABB（局部包围盒 8 角点变换后取 min/max）；无界几何（平面）→ None。
fn world_aabb(o: &Object) -> Option<[[f64; 3]; 2]> {
    let [bmin, bmax] = cga_mesh::BakeExt::bounds(&o.geometry.identity_params())?;
    let m = o.motor().to_matrix();
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for ci in 0..8 {
        let p = [
            if ci & 1 == 0 { bmin[0] } else { bmax[0] },
            if ci & 2 == 0 { bmin[1] } else { bmax[1] },
            if ci & 4 == 0 { bmin[2] } else { bmax[2] },
        ];
        let pw = cga_core::transform_point(m, p);
        for k in 0..3 {
            mn[k] = mn[k].min(pw[k]);
            mx[k] = mx[k].max(pw[k]);
        }
    }
    Some([mn, mx])
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
    ///
    /// 空间索引（D3）：对象数 > [`SPATIAL_INDEX_THRESHOLD`] 时启用世界 AABB 宽相——
    /// AABB 不相交 ⇒ 几何不相交（认证的 No，`separation = None` 未算精确距离），
    /// 跳过指纹缓存查询与窄相 probe。小场景保持全对精确距离（报告/金标不变）。
    /// 无界几何（平面）的 AABB 为 None，宽相中与一切相交（不剪）。
    pub fn scan(&mut self, scene: &cga_gpu::scene::Scene) -> Vec<PairHit> {
        self.cache_hits = 0;
        self.computed = 0;
        let objs = &scene.objects;
        let fps = fingerprints(objs);
        let aabbs: Vec<Option<[[f64; 3]; 2]>> = if objs.len() > SPATIAL_INDEX_THRESHOLD {
            objs.iter().map(world_aabb).collect()
        } else {
            Vec::new()
        };
        let mut out = Vec::new();
        for i in 0..objs.len() {
            for j in i + 1..objs.len() {
                let (gi, gj) = (objs[i].group, objs[j].group);
                if gi != 0 && gi == gj {
                    continue;
                }
                // 宽相：AABB 不相交 ⇒ 认证相离（精确的 Hit::No，距离未算）
                if !aabbs.is_empty() {
                    if let (Some(a), Some(b)) = (aabbs[i], aabbs[j]) {
                        if (0..3).any(|k| a[0][k] > b[1][k] || b[0][k] > a[1][k]) {
                            out.push(PairHit {
                                a: i,
                                b: j,
                                hit: Hit::No,
                                separation: None,
                            });
                            continue;
                        }
                    }
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
    /// `contacts` 为 None。接触解按同一指纹键帧间缓存（D1）。
    pub fn scan_contacts(&mut self, scene: &Scene) -> Vec<ContactHit> {
        self.contact_cache_hits = 0;
        let fps = fingerprints(&scene.objects);
        let hits = self.scan(scene);
        hits.into_iter()
            .map(|h| {
                let contacts = match h.hit {
                    Hit::Yes => {
                        let key = (fps[h.a], fps[h.b]);
                        match self.contact_cache.get(&key) {
                            Some(c) => {
                                self.contact_cache_hits += 1;
                                c.clone()
                            }
                            None => {
                                let wa = scene.objects[h.a].motor().to_matrix();
                                let wb = scene.objects[h.b].motor().to_matrix();
                                let c = cga_collision::contacts(
                                    &scene.objects[h.a].geometry,
                                    wa,
                                    &scene.objects[h.b].geometry,
                                    wb,
                                );
                                self.contact_cache.insert(key, c.clone());
                                c
                            }
                        }
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

/// 关节行程干涉扫描（C4 + D2）：joint 的 q 从行程低端扫到高端（continuous 扫一整圈），
/// 其连杆（含嵌套子关节的 meshes，刚体随动）对场景其余对象逐一求螺旋 TOI。
///
/// D2 联动：被扫 pair 经 gear 关系（q_b = ratio·q_a + offset）带动的 pair 一并随动，
/// 各自子树按自己的螺旋走闭式 TOI（兄弟分支型联动精确）。边界（诚实拒绝）：
/// - 只扫 1-DOF 关节（revolute/continuous/prismatic/helical）；
/// - cam 联动 → `skipped`（从动 q 非线性，不是螺旋）；
/// - 嵌套联动（一个随动 pair 在另一个的子树里：复合运动不是单螺旋）→ `skipped`；
/// - 随动对之间互碰：同轴 → 相对螺旋闭式精确；不同轴 → 间隔 + Lipschitz 速度上界
///   认证不碰，认证不了进 `unknown`（绝不假装）。
pub struct JointSweepOutcome {
    pub joint: String,
    /// 最早确切干涉：(对象下标, q*)。`q*` 在行程区间内。
    /// 随动对互碰时下标是后一个 mesh。
    pub first: Option<(usize, f64)>,
    /// 无法判定的对象（三值诚实）。
    pub unknown: Vec<usize>,
    /// 随动的 pair 名（gear 联动，不含被扫 pair 本身；D2）。
    pub coupled: Vec<String>,
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
            coupled: Vec::new(),
            skipped: Some("no such pair".to_string()),
        };
    };
    let skip = |reason: &str| JointSweepOutcome {
        joint: joint_name.to_string(),
        first: None,
        unknown: Vec::new(),
        coupled: Vec::new(),
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

    use crate::scene_build::{joint_motion, mat4_inv};
    use cga_core::mat4_mul;
    let rate_x = hi - lo;

    // ---- D2: gear 联动集合（q_b = ratio·q_a + offset → dq_b = ratio·dq_a 的 BFS）----
    let x_idx = kin
        .pairs
        .iter()
        .position(|p| p.name.as_deref() == Some(joint_name))
        .unwrap();
    let mut deltas: Vec<(usize, f64)> = vec![(x_idx, 1.0)]; // (pair 下标, dq_p/dq_x)
    let mut qi = 0;
    while qi < deltas.len() {
        let (pi, di) = deltas[qi];
        qi += 1;
        let pname = kin.pairs[pi].name.clone().unwrap_or_default();
        for g in &kin.gears {
            let (partner, d) = if g.a == pname {
                (g.b.clone(), di * g.ratio)
            } else if g.b == pname {
                if g.ratio.abs() < 1e-12 {
                    return skip("gear ratio=0：被扫 pair 被锁死在 offset");
                }
                (g.a.clone(), di / g.ratio)
            } else {
                continue;
            };
            let Some(pj) = kin
                .pairs
                .iter()
                .position(|p| p.name.as_deref() == Some(partner.as_str()))
            else {
                continue;
            };
            if let Some(&(_, d0)) = deltas.iter().find(|&&(k, _)| k == pj) {
                if (d0 - d).abs() > 1e-9 {
                    return skip("gear 环路比率冲突");
                }
                continue;
            }
            deltas.push((pj, d));
        }
    }
    // |Δ|≈0 的从动端锁在 offset，不动（ratio=0 的正向情形）
    deltas.retain(|&(_, d)| d.abs() > 1e-12);
    for &(pi, _) in &deltas {
        if !kin.pairs[pi].kind.is_1dof() {
            return skip("gear 联动的 pair 不是 1-DOF");
        }
    }
    // cam 边界：cam 的 a/b 是 LINK 名；关联 link 落在任一随动子树里 → 从动 q
    // 非线性（接触解），不是螺旋 → 拒绝
    let subtree_links = |root: &str| -> Vec<String> {
        let mut links: Vec<String> = vec![root.to_string()];
        let mut i = 0;
        while i < links.len() {
            let name = links[i].clone();
            for p in kin.pairs.iter().filter(|p| p.tree_up == name) {
                links.push(p.tree_down.clone());
            }
            i += 1;
        }
        links
    };
    for c in &kin.cams {
        for &(pi, _) in &deltas {
            let links = subtree_links(&kin.pairs[pi].tree_down);
            if links.contains(&c.a) || links.contains(&c.b) {
                return skip("cam 联动的行程扫描不支持（从动 q 非线性）");
            }
        }
    }
    // 嵌套检查：任一随动 pair 在另一个的子树里 → 复合运动非单螺旋，拒绝
    let moving_names: Vec<String> = deltas
        .iter()
        .filter_map(|&(pi, _)| kin.pairs[pi].name.clone())
        .collect();
    let parent_pair =
        |link: &str| -> Option<usize> { kin.pairs.iter().position(|p| p.tree_down == link) };
    let is_ancestor_pair = |anc: usize, desc: usize| -> bool {
        let mut cur = kin.pairs[desc].tree_up.clone();
        for _ in 0..1000 {
            let Some(pp) = parent_pair(&cur) else {
                return false;
            };
            if pp == anc {
                return true;
            }
            cur = kin.pairs[pp].tree_up.clone();
        }
        false
    };
    for a in 0..deltas.len() {
        for b in 0..deltas.len() {
            if a != b && is_ancestor_pair(deltas[a].0, deltas[b].0) {
                return skip("嵌套 gear 联动：复合运动不是单螺旋，无法闭式 TOI");
            }
        }
    }

    // ---- 每个随动 pair：世界螺旋 + 退位矩阵 + 子树 meshes ----
    struct Moving {
        o: [f64; 3],      // 轴上一点（世界，当前姿态）
        dir: [f64; 3],    // 单位轴（世界）
        xi: [f64; 6],     // 物理螺旋 dq/dt（t∈[0,1] 映射 q_x = lo + t·rate_x）
        delta: [f64; 16], // 当前姿态 → q(t=0) 的退位
        meshes: Vec<usize>,
    }
    let subtree_meshes = |root: &str| -> Vec<usize> {
        let mut links: Vec<&str> = vec![root];
        let mut i = 0;
        while i < links.len() {
            let name = links[i];
            for p in kin.pairs.iter().filter(|p| p.tree_up == name) {
                links.push(p.tree_down.as_str());
            }
            i += 1;
        }
        let mut out = Vec::new();
        for l in kin.links.iter() {
            if links.contains(&l.name.as_str()) {
                out.extend_from_slice(&l.meshes);
            }
        }
        out
    };
    let mut movings: Vec<Moving> = Vec::new();
    for &(pi, dp) in &deltas {
        let p = &kin.pairs[pi];
        let pitch = p.pitch.unwrap_or(0.0);
        let f = p.fa_world;
        let o = [f[3], f[7], f[11]];
        let dir = {
            let a = [
                f[0] * p.axis[0] + f[1] * p.axis[1] + f[2] * p.axis[2],
                f[4] * p.axis[0] + f[5] * p.axis[1] + f[6] * p.axis[2],
                f[8] * p.axis[0] + f[9] * p.axis[1] + f[10] * p.axis[2],
            ];
            let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
            if n < 1e-12 {
                return skip("degenerate pair axis");
            }
            [a[0] / n, a[1] / n, a[2] / n]
        };
        let rate_p = dp * rate_x;
        let (w, v) = match p.kind {
            crate::scene_build::JointKind::Prismatic => ([0.0; 3], v3scale(dir, rate_p)),
            _ => {
                let w = v3scale(dir, rate_p);
                // ṗ = ω×(p−o) ⇒ v = o×ω；helical 再加节距平移 pitch·rate 沿轴。
                let mut v = v3cross(o, w);
                if p.kind == crate::scene_build::JointKind::Helical {
                    v = v3add(v, v3scale(dir, pitch * rate_p));
                }
                (w, v)
            }
        };
        // q_p(t=0) = q_p_cur + Δp·(lo − q_x_cur)
        let q_p0 = p.q.first().copied().unwrap_or(0.0) + dp * (lo - q_cur);
        let mq = joint_motion(&p.kind, p.axis, &p.q, pitch);
        let m0 = joint_motion(&p.kind, p.axis, &[q_p0], pitch);
        let delta = mat4_mul(f, mat4_mul(mat4_mul(m0, mat4_inv(mq)), mat4_inv(f)));
        movings.push(Moving {
            o,
            dir,
            xi: [w[0], w[1], w[2], v[0], v[1], v[2]],
            delta,
            meshes: subtree_meshes(&p.tree_down),
        });
    }

    let mut members: Vec<usize> = movings
        .iter()
        .flat_map(|m| m.meshes.iter().copied())
        .collect();
    members.sort_unstable();
    members.dedup();

    let mut out = JointSweepOutcome {
        joint: joint_name.to_string(),
        first: None,
        unknown: Vec::new(),
        coupled: moving_names[1..].to_vec(),
        skipped: None,
    };
    let note = |out: &mut JointSweepOutcome, idx: usize, t: f64| {
        let q = lo + t * rate_x;
        if out.first.is_none_or(|(_, bq)| q < bq) {
            out.first = Some((idx, q));
        }
    };
    let scene = &run.scene;

    // ---- 随动 vs 静止：每个 mesh 按自己驱动 pair 的螺旋走闭式 TOI ----
    for mv in &movings {
        for &mi in &mv.meshes {
            let obj = &scene.objects[mi];
            let wa = mat4_mul(mv.delta, obj.motor().to_matrix());
            for (k, other) in scene.objects.iter().enumerate() {
                if members.contains(&k) {
                    continue; // 运动集合内部另行处理（见下）
                }
                let (gi, gk) = (obj.group, other.group);
                if gi != 0 && gi == gk {
                    continue; // 同组免检
                }
                let (h, t) = cga_collision::sweep_toi(
                    mv.xi,
                    &obj.geometry,
                    wa,
                    &other.geometry,
                    other.motor().to_matrix(),
                    1.0,
                );
                match h {
                    Hit::Yes => note(&mut out, k, t.unwrap_or(0.0)),
                    Hit::Unknown => out.unknown.push(k),
                    Hit::No => {}
                }
            }
        }
    }

    // ---- 随动 vs 随动：同轴 → 相对螺旋闭式；不同轴 → 认证不碰或 Unknown ----
    for a in 0..movings.len() {
        for b in a + 1..movings.len() {
            let (ma, mb) = (&movings[a], &movings[b]);
            let coaxial = {
                let dp = ma.dir[0] * mb.dir[0] + ma.dir[1] * mb.dir[1] + ma.dir[2] * mb.dir[2];
                let oo = [mb.o[0] - ma.o[0], mb.o[1] - ma.o[1], mb.o[2] - ma.o[2]];
                let cr = v3cross(oo, ma.dir);
                dp.abs() > 1.0 - 1e-9 && cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2] < 1e-12
            };
            let xi_rel = [
                ma.xi[0] - mb.xi[0],
                ma.xi[1] - mb.xi[1],
                ma.xi[2] - mb.xi[2],
                ma.xi[3] - mb.xi[3],
                ma.xi[4] - mb.xi[4],
                ma.xi[5] - mb.xi[5],
            ];
            for &mi in &ma.meshes {
                for &mj in &mb.meshes {
                    let (oa, ob) = (&scene.objects[mi], &scene.objects[mj]);
                    let (gi, gj) = (oa.group, ob.group);
                    if gi != 0 && gi == gj {
                        continue;
                    }
                    let wa = mat4_mul(ma.delta, oa.motor().to_matrix());
                    let wb = mat4_mul(mb.delta, ob.motor().to_matrix());
                    if coaxial {
                        let (h, t) = cga_collision::sweep_toi(
                            xi_rel,
                            &oa.geometry,
                            wa,
                            &ob.geometry,
                            wb,
                            1.0,
                        );
                        match h {
                            Hit::Yes => note(&mut out, mj, t.unwrap_or(0.0)),
                            Hit::Unknown => {
                                out.unknown.push(mi);
                                out.unknown.push(mj);
                            }
                            Hit::No => {}
                        }
                    } else {
                        // 保守认证不碰：t=0 的间隔 > 全程相对接近速度上界
                        match probe(&oa.geometry, wa, &ob.geometry, wb) {
                            (Hit::Yes, _) => note(&mut out, mj, 0.0),
                            (Hit::No, Some(d0)) => {
                                let sa = speed_bound(ma.o, ma.xi, oa, wa);
                                let sb = speed_bound(mb.o, mb.xi, ob, wb);
                                if !(d0 > sa + sb) {
                                    out.unknown.push(mi);
                                    out.unknown.push(mj);
                                }
                            }
                            _ => {
                                out.unknown.push(mi);
                                out.unknown.push(mj);
                            }
                        }
                    }
                }
            }
        }
    }
    out.unknown.sort_unstable();
    out.unknown.dedup();
    out
}

/// 螺旋运动下物体点的全程速度上界（保守认证用）：|v| + |ω|·(轴到中心距 + 局部包围半径)。
/// 无局部包围盒 → ∞（认证不了）。
fn speed_bound(mv_o: [f64; 3], mv_xi: [f64; 6], obj: &Object, w: [f64; 16]) -> f64 {
    let Some([bmin, bmax]) = cga_mesh::BakeExt::bounds(&obj.geometry.identity_params()) else {
        return f64::INFINITY;
    };
    let c_local = [
        (bmin[0] + bmax[0]) / 2.0,
        (bmin[1] + bmax[1]) / 2.0,
        (bmin[2] + bmax[2]) / 2.0,
    ];
    let r = ((0..3)
        .map(|i| (bmax[i] - bmin[i]) * (bmax[i] - bmin[i]))
        .sum::<f64>()
        .sqrt())
        / 2.0;
    let c_w = cga_core::transform_point(w, c_local);
    let oc = [c_w[0] - mv_o[0], c_w[1] - mv_o[1], c_w[2] - mv_o[2]];
    let om = [mv_xi[0], mv_xi[1], mv_xi[2]];
    let vv = [mv_xi[3], mv_xi[4], mv_xi[5]];
    let om_n = (om[0] * om[0] + om[1] * om[1] + om[2] * om[2]).sqrt();
    let vv_n = (vv[0] * vv[0] + vv[1] * vv[1] + vv[2] * vv[2]).sqrt();
    // 轴到点距 = |oc × dir|，dir = ω/|ω|（纯平移 |ω|=0 时无转动项）
    let d_axis = if om_n > 1e-12 {
        let cr = v3cross(oc, [om[0] / om_n, om[1] / om_n, om[2] / om_n]);
        (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt()
    } else {
        0.0
    };
    vv_n + om_n * (d_axis + r)
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
    <sphere r={0.5} t={[1.5, 1.0, 0]} />
    <link name="base" />
    <link name="arm_link"><sphere r={0.2} t={[1.2, 0, 0]} /></link>
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
    <sphere r={0.5} t={[2.0, 1.5, 0]} />
    <link name="base" />
    <link name="upper" />
    <link name="fore"><sphere r={0.2} t={[1.2, 0, 0]} /></link>
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
    <sphere r={0.5} t={[10.0, 0.0, 0]} />
    <link name="base" />
    <link name="l_fix"><sphere r={0.2} /></link>
    <link name="l_ball"><sphere r={0.2} /></link>
    <link name="l_free"><sphere r={0.2} /></link>
    <link name="l_arm"><sphere r={0.2} t={[1.2, 0, 0]} /></link>
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

    #[test]
    fn scan_spatial_index_matches_brute_force() {
        // D3：100 球网格（>64 触发宽相）+ 一个无界平面 + 一对相交。
        // 宽相的认证 No 必须与逐对 probe 的参考判定一致；相交对走窄相、
        // separation 精确。
        let mut sc = Scene::new(None);
        for i in 0..5 {
            for j in 0..5 {
                for k in 0..4 {
                    let mut o = sphere_obj(0.0, 1.0, 0);
                    o.position = [i as f64 * 3.0, j as f64 * 3.0, k as f64 * 3.0];
                    sc.add_object(o);
                }
            }
        }
        // 相交对：球 100 贴着球 0（球心距 0.75 < 2；距 (3,0,0) 的球 2.25 > 2 不误触）
        let mut ov = sphere_obj(0.0, 1.0, 0);
        ov.position = [0.75, 0.0, 0.0];
        sc.add_object(ov);
        // 无界平面（宽相不剪）
        let mut pl = sphere_obj(0.0, 1.0, 0);
        pl.geometry = Geometry::PlaneGeometry(cga_core::PlaneGeometry::new([0.0, 1.0, 0.0], -30.0));
        pl.position = [0.0, 0.0, 0.0];
        sc.add_object(pl);
        let n = sc.objects.len();
        assert!(n > SPATIAL_INDEX_THRESHOLD as usize, "{n}");

        let mut scan = CollisionScan::new();
        let hits = scan.scan(&sc);
        // 参考：逐对直接 probe（跳过同组规则外的全对）
        let total_pairs = n * (n - 1) / 2;
        assert_eq!(hits.len(), total_pairs, "发射仍是全矩阵");
        let mut mismatches = 0;
        for h in &hits {
            let wa = sc.objects[h.a].motor().to_matrix();
            let wb = sc.objects[h.b].motor().to_matrix();
            let (bh, _bsep) =
                cga_collision::probe(&sc.objects[h.a].geometry, wa, &sc.objects[h.b].geometry, wb);
            if h.hit != bh {
                mismatches += 1;
            }
        }
        assert_eq!(mismatches, 0, "宽相判定必须与窄相参考一致");
        // 宽相确实剪了绝大多数对（网格球互不相交）
        assert!(
            scan.computed < total_pairs / 10,
            "computed={} total={total_pairs}",
            scan.computed
        );
        // 相交对走了窄相：Yes + 精确分离 −0.5
        let yes: Vec<_> = hits.iter().filter(|h| h.hit == Hit::Yes).collect();
        assert_eq!(yes.len(), 1, "只有球-球相交对（平面在远处地面）");
        assert!(
            (yes[0].separation.unwrap() + 1.25).abs() < 1e-9,
            "{:?}",
            yes[0]
        );
        // 无界平面（y=-30 地面，实体侧朝下）与所有球相离：AABB=None 不剪，
        // 逐对走窄相得精确 No——mismatches==0 已覆盖；认证 No 的 separation 是
        // None（诚实未算）：
        let certified = hits
            .iter()
            .filter(|h| h.hit == Hit::No && h.separation.is_none())
            .count();
        assert!(certified > total_pairs * 9 / 10, "certified={certified}");
    }

    #[test]
    fn scan_contacts_cache_across_scans() {
        // D1：Yes 对的接触解按同一指纹键缓存——第二帧零接触解、结果逐位一致。
        let mut sc = Scene::new(None);
        sc.add_object(sphere_obj(0.0, 1.0, 0));
        sc.add_object(sphere_obj(1.5, 1.0, 0)); // 与 0 相交（Yes）
        sc.add_object(sphere_obj(9.0, 1.0, 0)); // 远离（No）
        let mut scan = CollisionScan::new();
        let first = scan.scan_contacts(&sc);
        assert_eq!(scan.contact_cache_hits, 0);
        assert!(first[0].contacts.as_ref().is_some_and(|c| !c.is_empty()));
        let second = scan.scan_contacts(&sc);
        assert_eq!(scan.computed, 0, "probe 全缓存命中");
        assert_eq!(scan.contact_cache_hits, 1, "接触解缓存命中");
        assert_eq!(first, second, "缓存的接触解与重算逐位一致");
        // 动相交对的一个：接触解重算
        sc.objects[1].position = [1.6, 0.0, 0.0];
        let third = scan.scan_contacts(&sc);
        assert_eq!(scan.contact_cache_hits, 0);
        assert_ne!(first[0].contacts, third[0].contacts, "位姿变了接触点必须变");
    }

    #[test]
    fn sweep_joint_gear_coupled_static() {
        // D2：平行轴齿轮联动（ratio=-1，反向）。A 臂球 r=0.1 在半径 1（绕原点 z）；
        // B 臂绕 (2.5,0,0) 的 z 轴，球在半径 1。立柱球 r=0.1 在 (2.5,-1,0)。
        // A 扫 [0,2]：B 反向转，球到 (2.5+cos q, -sin q)；触柱 ⇒ 2−2sin q = 0.04
        // ⇒ q* = asin(0.98)。A 自己的球够不到立柱（距原点 √7.25 > 1.2）。
        let src = r#"
export default (
  <scene>
    <camera />
    <sphere r={0.1} t={[2.5, -1.0, 0]} />
    <link name="base" />
    <link name="armA"><sphere r={0.1} t={[1, 0, 0]} /></link>
    <link name="armB"><sphere r={0.1} t={[1, 0, 0]} /></link>
    <pair kind="revolute" name="A" a="base" b="armA" axis={[0,0,1]} at={[0,0,0]} q={0} limit={[0,2.0]} />
    <pair kind="revolute" name="B" a="base" b="armB" axis={[0,0,1]} at={[2.5,0,0]} q={0} />
    <gear a="A" b="B" ratio={-1} />
    <anchor link="base" />
  </scene>
);
"#;
        let run = run_of(src);
        let out = sweep_joint(&run, "A");
        assert!(out.skipped.is_none(), "{:?}", out.skipped);
        assert_eq!(out.coupled, vec!["B".to_string()], "B 被 gear 带动");
        let (idx, q) = out.first.expect("B 臂应撞柱");
        assert_eq!(idx, 0, "撞到的是立柱（对象 0）");
        let want = 0.98f64.asin();
        assert!((q - want).abs() < 1e-9, "q={q} want={want}");
        // 随动对互碰：两球圆心最小距 0.5（半径 0.2 碰不到），但保守界
        // （速度 2.2+2.2 > 间隔 0.3）认证不了 → 诚实进 unknown
        assert_eq!(out.unknown.len(), 2, "{:?}", out.unknown);
    }

    #[test]
    fn sweep_joint_gear_coaxial_mutual() {
        // D2：同轴联动互碰闭式。两臂都绕原点 z（同轴），ratio=-1：A 球在角 q，
        // B 球在角 π−q（local (-1,0,0)）。间距 = 2cos q，触 ⇒ q* = acos(0.1)。
        // 相对运动是同轴转动（相对螺旋）→ 精确 TOI，不进 unknown。
        let src = r#"
export default (
  <scene>
    <camera />
    <link name="base" />
    <link name="armA"><sphere r={0.1} t={[1, 0, 0]} /></link>
    <link name="armB"><sphere r={0.1} t={[-1, 0, 0]} /></link>
    <pair kind="revolute" name="A" a="base" b="armA" axis={[0,0,1]} at={[0,0,0]} q={0} limit={[0,1.6]} />
    <pair kind="revolute" name="B" a="base" b="armB" axis={[0,0,1]} at={[0,0,0]} q={0} />
    <gear a="A" b="B" ratio={-1} />
    <anchor link="base" />
  </scene>
);
"#;
        let run = run_of(src);
        let out = sweep_joint(&run, "A");
        assert!(out.skipped.is_none(), "{:?}", out.skipped);
        let (_idx, q) = out.first.expect("两臂球应互碰");
        let want = 0.1f64.acos();
        assert!((q - want).abs() < 1e-9, "q={q} want={want}");
        assert!(out.unknown.is_empty(), "同轴互碰是闭式: {:?}", out.unknown);
    }

    #[test]
    fn sweep_joint_gear_nested_skipped() {
        // D2 边界：B 在 A 的子树里且 gear 联动 → 复合运动非单螺旋 → skipped
        let src = r#"
export default (
  <scene>
    <camera />
    <link name="base" />
    <link name="upper" />
    <link name="fore"><sphere r={0.2} /></link>
    <pair kind="revolute" name="A" a="base" b="upper" axis={[0,0,1]} at={[0,0,0]} limit={[0,1.0]} />
    <pair kind="revolute" name="B" a="upper" b="fore" axis={[0,0,1]} at={[1,0,0]} />
    <gear a="A" b="B" ratio={-1} />
    <anchor link="base" />
  </scene>
);
"#;
        let run = run_of(src);
        let out = sweep_joint(&run, "A");
        assert_eq!(
            out.skipped.as_deref(),
            Some("嵌套 gear 联动：复合运动不是单螺旋，无法闭式 TOI"),
            "{:?}",
            out.skipped
        );
    }

    #[test]
    fn sweep_joint_cam_coupled_skipped() {
        // D2 边界：cam 关联 link 在随动子树里 → skipped（从动 q 非线性）
        let src = r#"
export default (
  <scene>
    <camera />
    <link name="base" />
    <link name="cam_driver_link"><cylinder r={0.4} h={0.2} /></link>
    <link name="cam_follower_link"><sphere r={0.12} /></link>
    <pair kind="revolute" name="cam_driver" a="base" b="cam_driver_link" axis={[0,0,1]} at={[0,3,0]} q={0.3} limit={[0,1.0]} />
    <pair kind="prismatic" name="cam_follower" a="base" b="cam_follower_link" axis={[1,0,0]} at={[0.9,3,0]} limit={[-0.4,-0.1]} />
    <cam a="cam_driver_link" b="cam_follower_link"
         aProfile={{kind:"circle", c:[0.12,0,0], n:[0,0,1], r:0.4}}
         bProfile={{kind:"circle", c:[0,0,0], n:[0,0,1], r:0.12}} />
    <anchor link="base" />
  </scene>
);
"#;
        let run = run_of(src);
        let out = sweep_joint(&run, "cam_driver");
        assert_eq!(
            out.skipped.as_deref(),
            Some("cam 联动的行程扫描不支持（从动 q 非线性）"),
            "{:?}",
            out.skipped
        );
    }

    #[test]
    fn sweep_joint_gear_certified_miss() {
        // D2：相距 10 的两臂联动，小行程 [0,0.5]——间隔 9.8 > 速度上界和 1.1
        // → 认证不碰：first=None 且 unknown 为空（不是 Unknown 兜底）。
        let src = r#"
export default (
  <scene>
    <camera />
    <link name="base" />
    <link name="armA"><sphere r={0.1} t={[1, 0, 0]} /></link>
    <link name="armB"><sphere r={0.1} t={[1, 0, 0]} /></link>
    <pair kind="revolute" name="A" a="base" b="armA" axis={[0,0,1]} at={[0,0,0]} q={0} limit={[0,0.5]} />
    <pair kind="revolute" name="B" a="base" b="armB" axis={[0,0,1]} at={[10,0,0]} q={0} />
    <gear a="A" b="B" ratio={-1} />
    <anchor link="base" />
  </scene>
);
"#;
        let run = run_of(src);
        let out = sweep_joint(&run, "A");
        assert!(out.skipped.is_none(), "{:?}", out.skipped);
        assert_eq!(out.coupled, vec!["B".to_string()]);
        assert_eq!(out.first, None);
        assert!(
            out.unknown.is_empty(),
            "认证不碰不该进 unknown: {:?}",
            out.unknown
        );
    }
}
