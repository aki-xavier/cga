use super::*;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Continuous,
    Prismatic,
    Helical,
    Cylindrical,
    Spherical,
    Planar,
    Fixed,
}
impl JointKind {
    pub fn parse(s: &str) -> Option<JointKind> {
        match s {
            "revolute" => Some(JointKind::Revolute),
            "continuous" => Some(JointKind::Continuous),
            "prismatic" => Some(JointKind::Prismatic),
            "helical" => Some(JointKind::Helical),
            "cylindrical" => Some(JointKind::Cylindrical),
            "spherical" => Some(JointKind::Spherical),
            "planar" => Some(JointKind::Planar),
            "fixed" => Some(JointKind::Fixed),
            _ => None,
        }
    }
    pub fn name(&self) -> &'static str {
        match self {
            JointKind::Revolute => "revolute",
            JointKind::Continuous => "continuous",
            JointKind::Prismatic => "prismatic",
            JointKind::Helical => "helical",
            JointKind::Cylindrical => "cylindrical",
            JointKind::Spherical => "spherical",
            JointKind::Planar => "planar",
            JointKind::Fixed => "fixed",
        }
    }
    pub fn is_1dof(&self) -> bool {
        matches!(
            self,
            JointKind::Revolute | JointKind::Continuous | JointKind::Prismatic | JointKind::Helical
        )
    }
    pub fn q_arity(&self) -> usize {
        match self {
            JointKind::Revolute
            | JointKind::Continuous
            | JointKind::Prismatic
            | JointKind::Helical => 1,
            JointKind::Cylindrical => 2,
            JointKind::Spherical | JointKind::Planar => 3,
            JointKind::Fixed => 0,
        }
    }
    /// Text used in the q-arity error messages.
    pub fn q_shape(&self) -> &'static str {
        match self {
            JointKind::Cylindrical => "[qr, qp]",
            JointKind::Spherical => "[rx, ry, rz]",
            JointKind::Planar => "[x, y, theta]",
            _ => "a number",
        }
    }
}

/// 无因果运动副（docs/kinematics-graph.md）：a/b 对称的约束边。
/// 求解器从 anchor 定向生成树后填充 `tree_up`/`tree_down`。
#[derive(Clone, Debug)]
pub struct PairDef {
    pub name: Option<String>,
    pub kind: JointKind,
    pub a: String,
    pub b: String,
    pub at: [f64; 3],
    pub axis: [f64; 3],
    pub rpy: [f64; 3],
    pub q: Vec<f64>,
    pub pitch: Option<f64>,
    pub limit: Option<[f64; 2]>,
    /// 解出后：F_a 的世界矩阵（报告 / URDF / 扫描用）。
    pub fa_world: [f64; 16],
    /// 生成树定向（求解器填充）：上游 / 下游连杆。
    pub tree_up: String,
    pub tree_down: String,
}

/// 连杆（体图节点）。
#[derive(Clone, Debug)]
pub struct LinkDef {
    pub name: String,
    /// Scene mesh indices emitted under this link's frame.
    pub meshes: Vec<usize>,
    /// 解出的世界位姿（图求解注入 walk）。
    pub world: [f64; 16],
}

/// 齿轮耦合（q 图上的方程，无方向）：`q_b = ratio·q_a + offset`，谁解谁由求解器定。
#[derive(Clone, Debug)]
pub struct GearRel {
    pub a: String,
    pub b: String,
    pub ratio: f64,
    pub offset: f64,
}

#[derive(Clone, Debug)]
pub enum CamProfile {
    Circle { c: [f64; 3], n: [f64; 3], r: f64 },
    Plane { n: [f64; 3], d: f64 },
}

#[derive(Clone, Debug)]
pub struct CamRel {
    pub a: String,
    pub b: String,
    pub a_profile: CamProfile,
    pub b_profile: CamProfile,
}

#[derive(Clone, Debug)]
pub struct CamSolved {
    pub a: String,
    pub b: String,
    pub q: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Kinematics {
    pub links: Vec<LinkDef>,
    pub pairs: Vec<PairDef>,
    pub anchor: Option<String>,
    pub gears: Vec<GearRel>,
    pub cams: Vec<CamSolved>,
    /// Pose overrides that took effect (sorted by name), for the report.
    pub pose: Vec<(String, f64)>,
}

/// URDF fixed-axis roll-pitch-yaw: R = Rz(yaw)·Ry(pitch)·Rx(roll).
pub(crate) fn rpy4(rpy: [f64; 3]) -> [f64; 16] {
    if rpy == [0.0, 0.0, 0.0] {
        return mat4_identity();
    }
    mat4_mul(
        Multivector::rotor([0.0, 0.0, 1.0], rpy[2]).to_matrix(),
        mat4_mul(
            Multivector::rotor([0.0, 1.0, 0.0], rpy[1]).to_matrix(),
            Multivector::rotor([1.0, 0.0, 0.0], rpy[0]).to_matrix(),
        ),
    )
}

fn v3_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn v3_scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn v3_norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn v3_unit(a: [f64; 3]) -> [f64; 3] {
    let n = v3_norm(a);
    if n > 1e-300 {
        v3_scale(a, 1.0 / n)
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// Any unit vector perpendicular to `u`.
fn v3_perp(u: [f64; 3]) -> [f64; 3] {
    let seed = if u[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let c = [
        u[1] * seed[2] - u[2] * seed[1],
        u[2] * seed[0] - u[0] * seed[2],
        u[0] * seed[1] - u[1] * seed[0],
    ];
    v3_unit(c)
}

fn v3_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Joint motion M(q) as a 4×4 matrix, about the anchor (axis in parent frame).
pub(crate) fn joint_motion(kind: &JointKind, axis: [f64; 3], q: &[f64], pitch: f64) -> [f64; 16] {
    match kind {
        JointKind::Revolute | JointKind::Continuous => Multivector::rotor(axis, q[0]).to_matrix(),
        JointKind::Prismatic => translate4(v3_scale(axis, q[0])),
        JointKind::Helical => mat4_mul(
            translate4(v3_scale(axis, pitch * q[0])),
            Multivector::rotor(axis, q[0]).to_matrix(),
        ),
        JointKind::Cylindrical => mat4_mul(
            translate4(v3_scale(axis, q[1])),
            Multivector::rotor(axis, q[0]).to_matrix(),
        ),
        JointKind::Spherical => mat4_mul(
            Multivector::rotor([1.0, 0.0, 0.0], q[0]).to_matrix(),
            mat4_mul(
                Multivector::rotor([0.0, 1.0, 0.0], q[1]).to_matrix(),
                Multivector::rotor([0.0, 0.0, 1.0], q[2]).to_matrix(),
            ),
        ),
        JointKind::Planar => {
            let e1 = v3_perp(axis);
            let e2 = v3_cross(axis, e1);
            mat4_mul(
                translate4(v3_add(v3_scale(e1, q[0]), v3_scale(e2, q[1]))),
                Multivector::rotor(axis, q[2]).to_matrix(),
            )
        }
        JointKind::Fixed => mat4_identity(),
    }
}

struct ProfilePose {
    // circle: center/normal/radius; plane: normal/d (r = 0 marker unused)
    c: [f64; 3],
    n: [f64; 3],
    r: f64,
    d: f64,
    is_plane: bool,
}

fn pose_profile(p: &CamProfile, world: [f64; 16]) -> ProfilePose {
    let rot_dir = |v: [f64; 3]| -> [f64; 3] {
        v3_unit([
            world[0] * v[0] + world[1] * v[1] + world[2] * v[2],
            world[4] * v[0] + world[5] * v[1] + world[6] * v[2],
            world[8] * v[0] + world[9] * v[1] + world[10] * v[2],
        ])
    };
    match p {
        CamProfile::Circle { c, n, r } => ProfilePose {
            c: transform_point(world, *c),
            n: rot_dir(*n),
            r: *r,
            d: 0.0,
            is_plane: false,
        },
        CamProfile::Plane { n, d } => {
            let nw = rot_dir(*n);
            let pw = transform_point(world, v3_scale(*n, *d));
            ProfilePose {
                c: pw,
                n: nw,
                r: 0.0,
                d: vec3_dot(nw, pw),
                is_plane: true,
            }
        }
    }
}

/// Signed separations of the posed profiles: 0 at contact, > 0 apart.
/// circle–circle: |c₂−c₁| (in the shared plane) − (r₁+r₂) — one branch.
/// circle–plane: the cam contact is one-sided, so both sides of the plane
/// are separate branches: δ−r and −δ−r with δ = c_circle·n_plane − d_plane.
fn separation_branches(a: &ProfilePose, b: &ProfilePose) -> Vec<f64> {
    match (a.is_plane, b.is_plane) {
        (false, false) => {
            // Both normals verified parallel by the caller; project out the
            // normal component so skew-in-normal does not fake separation.
            let n = a.n;
            let dc = [b.c[0] - a.c[0], b.c[1] - a.c[1], b.c[2] - a.c[2]];
            let along = vec3_dot(dc, n);
            let in_plane = [
                dc[0] - along * n[0],
                dc[1] - along * n[1],
                dc[2] - along * n[2],
            ];
            vec![v3_norm(in_plane) - (a.r + b.r)]
        }
        (true, false) => {
            let delta = vec3_dot(b.c, a.n) - a.d;
            vec![delta - b.r, -delta - b.r]
        }
        (false, true) => {
            let delta = vec3_dot(a.c, b.n) - b.d;
            vec![delta - a.r, -delta - a.r]
        }
        (true, true) => vec![f64::INFINITY], // two planes never make a cam contact
    }
}

/// Solve the driven joint coordinate so the two cam profiles touch.
/// Search over [lo, hi]: sample 256 points, require exactly one sign bracket,
/// then bisect to 1e-12. No root or several roots are explicit errors.
#[allow(clippy::too_many_arguments)]
pub(crate) fn cam_solve(
    rel: &CamRel,
    driver_world: [f64; 16],
    driver_kind: &JointKind,
    driver_axis: [f64; 3],
    driven_ctx: [f64; 16],
    driven_kind: &JointKind,
    driven_axis: [f64; 3],
    driven_at: [f64; 3],
    pitch: f64,
    lo: f64,
    hi: f64,
    _line: i32,
) -> Result<f64, String> {
    // Planar-mechanism check: circle normals must be parallel to each other
    // (for a revolute/helical/cylindrical joint the profile normal must be
    // parallel to its own joint axis, else the plane precesses).
    let d_world = driver_world;
    let t2 = |q: f64| {
        mat4_mul(
            mat4_mul(driven_ctx, translate4(driven_at)),
            joint_motion(driven_kind, driven_axis, &[q], pitch),
        )
    };
    let pa = pose_profile(&rel.a_profile, d_world);
    let pb0 = pose_profile(&rel.b_profile, t2(0.0));
    let parallel = |u: [f64; 3], v: [f64; 3]| v3_norm(v3_cross(u, v)) < 1e-9;
    let err_axis = || "build: cam profile normal must be parallel to its joint axis".to_string();
    if !pa.is_plane
        && matches!(
            driver_kind.clone(),
            JointKind::Revolute | JointKind::Helical | JointKind::Cylindrical
        )
    {
        // driver axis in world = R(driver.world) · axis. The 3×3 part of
        // driver.world already includes M(q), but M(q) rotates ABOUT the
        // axis, so the axis is invariant either way.
        let aw = {
            let a = driver_axis;
            v3_unit([
                d_world[0] * a[0] + d_world[1] * a[1] + d_world[2] * a[2],
                d_world[4] * a[0] + d_world[5] * a[1] + d_world[6] * a[2],
                d_world[8] * a[0] + d_world[9] * a[1] + d_world[10] * a[2],
            ])
        };
        if !parallel(pa.n, aw) {
            return Err(err_axis());
        }
    }

    if !pb0.is_plane && !parallel(pa.n, pb0.n) {
        return Err(err_axis());
    }
    if !pb0.is_plane
        && matches!(
            driven_kind,
            JointKind::Revolute | JointKind::Helical | JointKind::Cylindrical
        )
    {
        // driven profile normal must be invariant under its own motion.
        let local_axis_parallel = {
            let CamProfile::Circle { n, .. } = &rel.b_profile else {
                unreachable!()
            };
            parallel(*n, driven_axis)
        };
        if !local_axis_parallel {
            return Err(err_axis());
        }
    }

    let branches = separation_branches(&pa, &pose_profile(&rel.b_profile, t2(0.0))).len();
    let f = |q: f64, k: usize| separation_branches(&pa, &pose_profile(&rel.b_profile, t2(q)))[k];
    const N: usize = 256;
    let mut brackets: Vec<(f64, f64, usize)> = Vec::new();
    for k in 0..branches {
        let mut q0 = lo;
        let mut f0 = f(q0, k);
        for i in 1..=N {
            let q1 = lo + (hi - lo) * (i as f64) / (N as f64);
            let f1 = f(q1, k);
            if f0.is_finite() && f1.is_finite() {
                // An exact zero on a sample point is a root; a strict sign
                // change is a bracket. Skip the sign test when the previous
                // sample was itself the recorded zero.
                if f1 == 0.0 {
                    brackets.push((q1, q1, k));
                } else if f0 != 0.0 && f0 * f1 < 0.0 {
                    brackets.push((q0, q1, k));
                }
            }
            q0 = q1;
            f0 = f1;
        }
    }
    if brackets.is_empty() {
        return Err(format!(
            "build: cam found no contact in limit [{}, {}]",
            fmt_f64(lo),
            fmt_f64(hi)
        ));
    }
    if brackets.len() > 1 {
        return Err(format!(
            "build: cam contact is not unique in limit [{}, {}]",
            fmt_f64(lo),
            fmt_f64(hi)
        ));
    }
    let (mut a, mut b, k) = brackets[0];
    let mut fa = f(a, k);
    for _ in 0..100 {
        let m = 0.5 * (a + b);
        let fm = f(m, k);
        if (b - a).abs() < 1e-12 {
            return Ok(m);
        }
        if fa * fm <= 0.0 {
            b = m;
        } else {
            a = m;
            fa = fm;
        }
    }
    Ok(0.5 * (a + b))
}

// ---- 图模型求解（docs/kinematics-graph.md）--------------------------------

/// 声明期的图输入（Builder 的 pass 1 收集）。
#[derive(Clone, Debug, Default)]
pub struct GraphDecl {
    pub links: Vec<String>,
    pub pairs: Vec<PairDecl>,
    pub anchor: Option<String>,
    pub gears: Vec<GearDecl>,
    pub cams: Vec<CamDecl>,
}

#[derive(Clone, Debug)]
pub struct PairDecl {
    pub name: Option<String>,
    pub kind: JointKind,
    pub a: String,
    pub b: String,
    pub at: [f64; 3],
    pub axis: [f64; 3],
    pub rpy: [f64; 3],
    /// q 初值（未给 = 空 vec）。
    pub q_init: Vec<f64>,
    /// q 是否显式给出（gear/cam 的"已固定"判定与 pose 冲突检查用）。
    pub q_given: bool,
    pub pitch: Option<f64>,
    pub limit: Option<[f64; 2]>,
}

#[derive(Clone, Debug)]
pub struct GearDecl {
    pub a: String,
    pub b: String,
    pub ratio: f64,
    pub offset: f64,
}

#[derive(Clone, Debug)]
pub struct CamDecl {
    pub a: String,
    pub b: String,
    pub a_profile: CamProfile,
    pub b_profile: CamProfile,
}

/// 图求解输出：注入 walk 的连杆位姿 + 求解后的 pair/cam 记录。
pub struct GraphSolution {
    /// 与 `GraphDecl::links` 同序的连杆世界位姿。
    pub link_world: Vec<[f64; 16]>,
    pub pairs: Vec<PairDef>,
    pub cams: Vec<CamSolved>,
}

/// 校验（D8）→ q 赋值 → gear 不动点 → BFS 定向生成树 → 前向传播 → cam 逐个解。
/// 阶段 G1 只覆盖树机构：图有环直接报错（D7）。
pub(crate) fn solve_graph(
    decl: &GraphDecl,
    pose: &HashMap<String, f64>,
) -> Result<GraphSolution, String> {
    use std::collections::{HashMap, HashSet, VecDeque};
    let pname = |p: &PairDecl| p.name.clone().unwrap_or_else(|| "<unnamed>".to_string());

    // ---- 1. 校验 ----
    let mut link_idx: HashMap<&str, usize> = HashMap::new();
    for (i, l) in decl.links.iter().enumerate() {
        if link_idx.insert(l.as_str(), i).is_some() {
            return Err(format!("JSX: duplicate link name {l}"));
        }
    }
    let n_links = decl.links.len();
    if n_links > 0 && decl.anchor.is_none() {
        return Err("JSX: 图里有 link 但缺 <anchor>（位姿需要世界锚定）".to_string());
    }
    let anchor = match &decl.anchor {
        Some(a) => {
            if !link_idx.contains_key(a.as_str()) {
                return Err(format!("JSX: anchor 引用未知 link {a}"));
            }
            a.clone()
        }
        None => String::new(),
    };
    let mut pair_names: HashSet<&str> = HashSet::new();
    for p in &decl.pairs {
        if let Some(n) = &p.name {
            if !pair_names.insert(n.as_str()) {
                return Err(format!("JSX: duplicate pair name {n}"));
            }
        }
        for r in [&p.a, &p.b] {
            if !link_idx.contains_key(r.as_str()) {
                return Err(format!("JSX: pair {} 引用未知 link {r}", pname(p)));
            }
        }
        if p.a == p.b {
            return Err(format!(
                "JSX: pair {} 的 a 与 b 是同一 link {}",
                pname(p),
                p.a
            ));
        }
    }
    let named_pair = |name: &str, what: &str| -> Result<usize, String> {
        decl.pairs
            .iter()
            .position(|p| p.name.as_deref() == Some(name))
            .ok_or_else(|| format!("JSX: {what} 引用未知 pair {name}"))
    };
    for g in &decl.gears {
        named_pair(&g.a, "gear")?;
        named_pair(&g.b, "gear")?;
    }
    for c in &decl.cams {
        for r in [&c.a, &c.b] {
            if !link_idx.contains_key(r.as_str()) {
                return Err(format!("JSX: cam 引用未知 link {r}"));
            }
        }
    }

    // ---- 2. q 赋值：pose 覆盖 > prop q > 0 ----
    let check_limit = |name: &str, limit: Option<[f64; 2]>, x: f64| -> Result<(), String> {
        if let Some([lo, hi]) = limit {
            if x < lo || x > hi {
                return Err(format!("JSX: pair {name} q={x} outside limit [{lo}, {hi}]"));
            }
        }
        Ok(())
    };
    let mut qs: Vec<Vec<f64>> = Vec::with_capacity(decl.pairs.len());
    let mut pinned: Vec<bool> = Vec::with_capacity(decl.pairs.len());
    for p in &decl.pairs {
        let arity = p.kind.q_arity();
        let mut q = p.q_init.clone();
        let mut pin = p.q_given;
        if let Some(n) = &p.name {
            if let Some(&o) = pose.get(n) {
                if p.q_given {
                    return Err(format!(
                        "JSX: pair {n} has a pose override, q must be omitted"
                    ));
                }
                if !p.kind.is_1dof() {
                    return Err(format!(
                        "JSX: pose override needs a 1-DOF pair, got {}",
                        p.kind.name()
                    ));
                }
                q = vec![o];
                pin = true;
            }
        }
        if q.is_empty() && arity > 0 {
            q = vec![0.0; arity];
        }
        if q.len() != arity {
            return Err(format!(
                "JSX: {} pair q must be {}",
                p.kind.name(),
                p.kind.q_shape()
            ));
        }
        // 限位预检只针对**已固定**的 q（给定的初值/pose）；默认 0 值可能由
        // gear/cam 解出，最终再统一检查（cam_follower 这类先写限位后解 q 的场景）。
        if p.kind.is_1dof() && pin {
            check_limit(&pname(p), p.limit, q[0])?;
        }
        qs.push(q);
        pinned.push(pin);
    }

    // ---- 3. gear 方程不动点（无因果：哪侧已知解哪侧） ----
    let mut rounds = 0;
    loop {
        rounds += 1;
        let mut progress = false;
        for g in &decl.gears {
            let ia = named_pair(&g.a, "gear")?;
            let ib = named_pair(&g.b, "gear")?;
            if !decl.pairs[ia].kind.is_1dof() || !decl.pairs[ib].kind.is_1dof() {
                return Err(format!("JSX: gear needs 1-DOF pairs, got {}-{}", g.a, g.b));
            }
            match (pinned[ia], pinned[ib]) {
                (true, true) => {
                    if (qs[ib][0] - (g.ratio * qs[ia][0] + g.offset)).abs() > 1e-9 {
                        return Err(format!("JSX: gear {}-{} 矛盾", g.a, g.b));
                    }
                }
                (true, false) => {
                    qs[ib][0] = g.ratio * qs[ia][0] + g.offset;
                    pinned[ib] = true;
                    check_limit(&pname(&decl.pairs[ib]), decl.pairs[ib].limit, qs[ib][0])?;
                    progress = true;
                }
                (false, true) => {
                    if g.ratio.abs() < 1e-12 {
                        return Err(format!("JSX: gear {}-{} ratio=0 且只能反解", g.a, g.b));
                    }
                    qs[ia][0] = (qs[ib][0] - g.offset) / g.ratio;
                    pinned[ia] = true;
                    check_limit(&pname(&decl.pairs[ia]), decl.pairs[ia].limit, qs[ia][0])?;
                    progress = true;
                }
                (false, false) => {
                    // 两侧都未固定：取 a 侧默认 0 为种子前向解（与旧模型
                    // "driver 的默认 q 作种子"一致，确定性）。
                    pinned[ia] = true;
                    progress = true;
                }
            }
        }
        if !progress || rounds > decl.gears.len() + 1 {
            break;
        }
    }

    // ---- 4. BFS 定向生成树 + 孤岛/闭环检查 ----
    let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n_links];
    for (pi, p) in decl.pairs.iter().enumerate() {
        let (ia, ib) = (link_idx[p.a.as_str()], link_idx[p.b.as_str()]);
        adj[ia].push((pi, ib));
        adj[ib].push((pi, ia));
    }
    let mut parent: Vec<Option<(usize, usize, bool)>> = vec![None; n_links]; // (pair_idx, parent_link, forward)
    let mut order: Vec<usize> = Vec::new();
    if n_links > 0 {
        let aidx = link_idx[anchor.as_str()];
        let mut seen = vec![false; n_links];
        let mut qd = VecDeque::from([aidx]);
        seen[aidx] = true;
        while let Some(u) = qd.pop_front() {
            for &(pi, v) in &adj[u] {
                if !seen[v] {
                    seen[v] = true;
                    let fwd = decl.pairs[pi].a == decl.links[u];
                    parent[v] = Some((pi, u, fwd));
                    order.push(v);
                    qd.push_back(v);
                }
            }
        }
        if let Some(i) = seen.iter().position(|s| !s) {
            return Err(format!(
                "JSX: link {} 与 anchor 不连通（孤岛）",
                decl.links[i]
            ));
        }
        if decl.pairs.len() > n_links.saturating_sub(1) {
            return Err(
                "JSX: 机构有闭环但没有 <closure>（闭链另行立项，见 docs/kinematics-graph.md）"
                    .to_string(),
            );
        }
    }

    // ---- 5. 前向传播（闭包：cam 解出后重放） ----
    let propagate = |qs: &[Vec<f64>],
                     world: &mut Vec<[f64; 16]>,
                     parent: &[Option<(usize, usize, bool)>],
                     order: &[usize]| {
        for w in world.iter_mut() {
            *w = mat4_identity();
        }
        for &v in order {
            let (pi, u, fwd) = parent[v].unwrap();
            let p = &decl.pairs[pi];
            let fa = mat4_mul(translate4(p.at), rpy4(p.rpy));
            let m = joint_motion(&p.kind, p.axis, &qs[pi], p.pitch.unwrap_or(0.0));
            world[v] = if fwd {
                mat4_mul(world[u], mat4_mul(fa, m))
            } else {
                mat4_mul(world[u], mat4_inv(mat4_mul(fa, m)))
            };
        }
    };
    let mut world = vec![mat4_identity(); n_links.max(1)];
    propagate(&qs, &mut world, &parent, &order);

    // ---- 6. cam 逐个解（自由 q = anchor→b 路径上第一个未固定 pair） ----
    let mut cams = Vec::new();
    for cd in &decl.cams {
        // b 到 anchor 的树路径（pair 下标，从 b 侧向上）
        let mut path: Vec<usize> = Vec::new();
        let mut cur = link_idx[cd.b.as_str()];
        while let Some((pi, u, _)) = parent[cur] {
            path.push(pi);
            cur = u;
        }
        let Some(free_pi) = path.iter().copied().find(|&pi| !pinned[pi]) else {
            return Err(format!(
                "JSX: cam {}-{} 找不到可解的自由 q（路径上的 pair 都被固定）",
                cd.a, cd.b
            ));
        };
        let fp = &decl.pairs[free_pi];
        if !fp.kind.is_1dof() {
            return Err(format!(
                "JSX: cam 的自由 q 需要 1-DOF pair，got {}",
                fp.kind.name()
            ));
        }
        let Some([lo, hi]) = fp.limit else {
            return Err(format!("JSX: cam 的自由 pair {} 需要 limit", pname(fp)));
        };
        // driver（profile a 所在的 link 的上游 pair；a 是 anchor 时无需轴平行检查）
        let (d_kind, d_axis) = match parent[link_idx[cd.a.as_str()]] {
            Some((dpi, _, _)) => (decl.pairs[dpi].kind.clone(), decl.pairs[dpi].axis),
            None => (JointKind::Fixed, [0.0, 0.0, 1.0]),
        };
        let rel = CamRel {
            a: cd.a.clone(),
            b: cd.b.clone(),
            a_profile: cd.a_profile.clone(),
            b_profile: cd.b_profile.clone(),
        };
        let q = cam_solve(
            &rel,
            world[link_idx[cd.a.as_str()]],
            &d_kind,
            d_axis,
            world[link_idx[fp.a.as_str()]],
            &fp.kind,
            fp.axis,
            fp.at,
            fp.pitch.unwrap_or(0.0),
            lo,
            hi,
            0,
        )?;
        qs[free_pi] = vec![q];
        pinned[free_pi] = true;
        propagate(&qs, &mut world, &parent, &order);
        cams.push(CamSolved {
            a: cd.a.clone(),
            b: cd.b.clone(),
            q,
        });
    }

    // ---- 7. 输出（限位终检：默认 0 值在 gear/cam 都没管到的 pair 上越限也要报） ----
    for (pi, p) in decl.pairs.iter().enumerate() {
        if p.kind.is_1dof() {
            check_limit(&pname(p), p.limit, qs[pi][0])?;
        }
    }
    let mut out_pairs = Vec::with_capacity(decl.pairs.len());
    for (pi, p) in decl.pairs.iter().enumerate() {
        let fa = mat4_mul(translate4(p.at), rpy4(p.rpy));
        let (up, down) = if parent[link_idx[p.b.as_str()]].is_some_and(|x| x.0 == pi) {
            (p.a.clone(), p.b.clone())
        } else {
            (p.b.clone(), p.a.clone())
        };
        out_pairs.push(PairDef {
            name: p.name.clone(),
            kind: p.kind.clone(),
            a: p.a.clone(),
            b: p.b.clone(),
            at: p.at,
            axis: p.axis,
            rpy: p.rpy,
            q: qs[pi].clone(),
            pitch: p.pitch,
            limit: p.limit,
            fa_world: mat4_mul(world[link_idx[p.a.as_str()]], fa),
            tree_up: up,
            tree_down: down,
        });
    }
    Ok(GraphSolution {
        link_world: world,
        pairs: out_pairs,
        cams,
    })
}
