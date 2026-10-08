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
    /// q 是否由作者/pose 给定（活动度计数的 pin；gear/cam/closure 解出的不算）。
    pub given: bool,
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
    pub closures: Vec<ClosureSolved>,
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
    pub closures: Vec<ClosureDecl>,
}

/// 环路闭包约束（G5，Modelica cut-joint 同构）：link a 上 `at` 点必须与 link b 上
/// `b_at` 点重合，且 closure 轴对齐。它不是树边（生成树只用 `<pair>`），
/// 是位置级约束方程；它自己的 q 不是未知量，是闭合物。
#[derive(Clone, Debug)]
pub struct ClosureDecl {
    pub a: String,
    pub b: String,
    /// a 侧 frame 里的闭合点（F_a 原点）。
    pub at: [f64; 3],
    /// b 侧 frame 里的闭合点（默认 b 的 frame 原点）。
    pub b_at: [f64; 3],
    /// 闭合轴（a 侧 frame；q=0 闭合同一轴）。revolute 闭环必给。
    pub axis: [f64; 3],
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
    /// 求解器初值（`guess` prop）：只是 LM/闭链求解的种子，不是值——不参与构建，
    /// 不被求解器触碰的 pair 上 guess 无效（报错）。
    pub q_guess: Vec<f64>,
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
    pub closures: Vec<ClosureSolved>,
}

/// 求解后的闭环记录（报告用）。
#[derive(Clone, Debug)]
pub struct ClosureSolved {
    pub a: String,
    pub b: String,
    pub at: [f64; 3],
    pub b_at: [f64; 3],
    /// 求解到的 pair（名 + q）。
    pub solved: Vec<(String, f64)>,
    /// 独立约束数 = 残差雅可比在解处的秩（活动度计数用）。
    pub rank: usize,
    /// 收敛后的残差范数。
    pub residual: f64,
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
    for c in &decl.closures {
        if c.a == c.b {
            return Err(format!("JSX: closure 的 a 与 b 是同一 link {}", c.a));
        }
        for r in [&c.a, &c.b] {
            if !link_idx.contains_key(r.as_str()) {
                return Err(format!("JSX: closure 引用未知 link {r}"));
            }
        }
        let n = c.axis[0] * c.axis[0] + c.axis[1] * c.axis[1] + c.axis[2] * c.axis[2];
        if n < 1e-12 {
            return Err("JSX: closure.axis must be nonzero".to_string());
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
    let mut author_pinned: Vec<bool> = Vec::with_capacity(decl.pairs.len());
    for p in &decl.pairs {
        let arity = p.kind.q_arity();
        let mut q = p.q_init.clone();
        let mut pin = p.q_given;
        let mut author_pin = p.q_given;
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
                author_pin = true;
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
        author_pinned.push(author_pin);
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

    // ---- 6b. closure 逐个解（G5：闭链，位置级约束 + LM） ----
    let mut closures = Vec::new();
    let mut guess_used = vec![false; decl.pairs.len()];
    for cd in &decl.closures {
        // 树路径 a → b（两端已在同一棵生成树里——全图连通性在孤岛检查时成立）。
        let ia = link_idx[cd.a.as_str()];
        let ib = link_idx[cd.b.as_str()];
        let path = tree_path(&parent, ia, ib);
        if path.is_empty() {
            return Err(format!("JSX: closure {}-{} 两端不在同一棵树", cd.a, cd.b));
        }
        // 自由变量：路径上未固定 pair 的全部 q 分量（G5+：多自由度副按分量展开；
        // 雅可比是 B 螺旋列的解析导数，FD 对拍见测试）。
        let free: Vec<(usize, usize)> = path
            .iter()
            .filter(|&&pi| !pinned[pi])
            .flat_map(|&pi| (0..decl.pairs[pi].kind.q_arity()).map(move |c| (pi, c)))
            .collect();
        if free.is_empty() {
            return Err(format!(
                "JSX: closure {}-{} 路径上没有可解的自由 q",
                cd.a, cd.b
            ));
        }
        // 残差：闭合点重合（3）+ 闭合轴对齐（3，叉积分量）。
        let resid = |qs: &[Vec<f64>], world: &mut Vec<[f64; 16]>| -> Vec<f64> {
            propagate(qs, world, &parent, &order);
            closure_residual(cd, ia, ib, world)
        };
        // 初值：guess prop > 当前 q（默认 0），按分量取。
        let mut x: Vec<f64> = free
            .iter()
            .map(|&(pi, c)| {
                if let Some(&g) = decl.pairs[pi].q_guess.get(c) {
                    guess_used[pi] = true;
                    g
                } else {
                    qs[pi][c]
                }
            })
            .collect();
        // 解析雅可比：自由变量 k=(pi,c) 的螺旋列 s=[w;v] 作用在它驱动的一侧端点上。
        // 残差 r = [pa−pb; ûa×ûb]；d(pa)=w×pa+v，d(ûa)=w×ûa（旋转保持单位长）。
        let qs0 = qs.clone();
        let mut wtmp_r = vec![mat4_identity(); n_links.max(1)];
        let mut rfn = |x: &[f64]| {
            let mut qq = qs0.clone();
            for (k, &(pi, c)) in free.iter().enumerate() {
                qq[pi][c] = x[k];
            }
            resid(&qq, &mut wtmp_r)
        };
        let mut wtmp_j = vec![mat4_identity(); n_links.max(1)];
        let mut jfn = |x: &[f64]| -> Vec<Vec<f64>> {
            let mut qq = qs0.clone();
            for (k, &(pi, c)) in free.iter().enumerate() {
                qq[pi][c] = x[k];
            }
            propagate(&qq, &mut wtmp_j, &parent, &order);
            closure_jacobian(decl, cd, &free, ia, ib, &link_idx, &parent, &qq, &wtmp_j)
        };
        let j_final = lm_solve_j(&mut x, &mut rfn, &mut jfn)
            .map_err(|e| format!("JSX: closure {}-{} 未收敛: {e}", cd.a, cd.b))?;
        let mut wtmp = vec![mat4_identity(); n_links.max(1)];
        // 先把解写回 qs 再验残差（rfn/jfn 在克隆上工作，外层的 qs 未被更新）。
        for (k, &(pi, c)) in free.iter().enumerate() {
            qs[pi][c] = x[k];
        }
        let r = resid(&qs, &mut wtmp);
        let rnorm = r.iter().map(|v| v * v).sum::<f64>().sqrt();
        if rnorm > 1e-6 {
            return Err(format!(
                "JSX: closure {}-{} 收敛后残差 {rnorm:e} 仍超界（机构可能装不上）",
                cd.a, cd.b
            ));
        }
        let mut solved = Vec::new();
        for (k, &(pi, c)) in free.iter().enumerate() {
            qs[pi][c] = x[k];
            pinned[pi] = true;
            if decl.pairs[pi].kind.is_1dof() {
                check_limit(&pname(&decl.pairs[pi]), decl.pairs[pi].limit, x[k])?;
            }
            // 名字：1-DOF 保持裸名（报告与既有消费者不变），多自由度按分量展开。
            let nm = if decl.pairs[pi].kind.is_1dof() {
                pname(&decl.pairs[pi])
            } else {
                format!("{}[{c}]", pname(&decl.pairs[pi]))
            };
            solved.push((nm, x[k]));
        }
        propagate(&qs, &mut world, &parent, &order);
        closures.push(ClosureSolved {
            a: cd.a.clone(),
            b: cd.b.clone(),
            at: cd.at,
            b_at: cd.b_at,
            solved,
            // 独立约束数 = 残差雅可比的秩（平面闭环的退化行自动排除）。
            // closure 自身还有一个被确定的旋转 DOF（+1 在 mobility 里算）。
            rank: mat_rank(&j_final),
            residual: rnorm,
        });
    }

    // ---- 7. 输出（限位终检：默认 0 值在 gear/cam 都没管到的 pair 上越限也要报） ----
    for (pi, p) in decl.pairs.iter().enumerate() {
        if p.kind.is_1dof() {
            check_limit(&pname(p), p.limit, qs[pi][0])?;
        }
        if !p.q_guess.is_empty() && !guess_used[pi] {
            return Err(format!(
                "JSX: pair {} 的 guess 没有被任何求解器使用（它不在闭链上）",
                pname(p)
            ));
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
            given: author_pinned[pi],
            fa_world: mat4_mul(world[link_idx[p.a.as_str()]], fa),
            tree_up: up,
            tree_down: down,
        });
    }
    Ok(GraphSolution {
        link_world: world,
        pairs: out_pairs,
        cams,
        closures,
    })
}

/// 生成树上 a → b 的 pair 路径（parent 数组由 BFS 填充）。
fn tree_path(parent: &[Option<(usize, usize, bool)>], mut a: usize, mut b: usize) -> Vec<usize> {
    // 先爬到同一深度，再同步上爬到 LCA。
    let depth = |mut v: usize| {
        let mut d = 0;
        while let Some((_, u, _)) = parent[v] {
            d += 1;
            v = u;
        }
        d
    };
    let (mut da, mut db) = (depth(a), depth(b));
    let mut out: Vec<usize> = Vec::new();
    while da > db {
        let (pi, u, _) = parent[a].unwrap();
        out.push(pi);
        a = u;
        da -= 1;
    }
    while db > da {
        let (pi, u, _) = parent[b].unwrap();
        out.push(pi);
        b = u;
        db -= 1;
    }
    while a != b {
        let (pi, u, _) = parent[a].unwrap();
        out.push(pi);
        a = u;
        let (pj, v, _) = parent[b].unwrap();
        out.push(pj);
        b = v;
    }
    out
}

/// 小规模 Levenberg–Marquardt：min |r(x)|，有限差分雅可比。
/// 返回收敛处的雅可比（活动度的约束秩用）。不收敛（100 轮）→ Err。
/// 用于闭链的位置级约束求解。
#[cfg(test)] // FD 版是解析版 lm_solve_j 的裁判（closure_lm_fd_crosscheck），生产路径不用
fn lm_solve(x: &mut [f64], r: &mut dyn FnMut(&[f64]) -> Vec<f64>) -> Result<Vec<Vec<f64>>, String> {
    let n = x.len();
    let mut lambda = 1e-3;
    let mut cur = r(x);
    let mut cur_norm = cur.iter().map(|v| v * v).sum::<f64>().sqrt();
    let mut j: Vec<Vec<f64>> = Vec::new();
    for _ in 0..100 {
        if cur_norm < 1e-9 {
            return Ok(j);
        }
        let m = cur.len();
        // 有限差分雅可比
        j = vec![vec![0.0; n]; m];
        for (k, xk) in x.iter().enumerate() {
            let h = 1e-7 * (1.0 + xk.abs());
            let mut xp = x.to_vec();
            xp[k] += h;
            let rp = r(&xp);
            for i in 0..m {
                j[i][k] = (rp[i] - cur[i]) / h;
            }
        }
        // 解 (JᵀJ + λI)δ = −Jᵀr（小规模高斯消元）
        let mut a = vec![vec![0.0; n]; n];
        let mut b = vec![0.0; n];
        for i in 0..m {
            for u in 0..n {
                b[u] -= j[i][u] * cur[i];
                for v in 0..n {
                    a[u][v] += j[i][u] * j[i][v];
                }
            }
        }
        let mut solved = false;
        for _ in 0..50 {
            let mut aa = a.clone();
            for u in 0..n {
                aa[u][u] += lambda * (a[u][u].abs() + 1e-12);
            }
            if let Some(delta) = gauss_solve(&aa, &b) {
                let xn: Vec<f64> = x.iter().zip(delta.iter()).map(|(x, d)| x + d).collect();
                let rn = r(&xn);
                let rn_norm = rn.iter().map(|v| v * v).sum::<f64>().sqrt();
                if rn_norm < cur_norm {
                    x.copy_from_slice(&xn);
                    cur = rn;
                    cur_norm = rn_norm;
                    lambda = (lambda / 3.0).max(1e-12);
                    solved = true;
                    break;
                }
            }
            lambda *= 10.0;
            if lambda > 1e12 {
                break;
            }
        }
        if !solved {
            return Err(format!("LM 停滞（残差 {cur_norm:e}）"));
        }
    }
    if cur_norm < 1e-7 {
        return Ok(j);
    }
    Err(format!("LM 100 轮未收敛（残差 {cur_norm:e}）"))
}

/// 闭合残差（G5/G5+）：闭合点重合（3）+ 闭合轴对齐（3，叉积分量）。
/// `world` 必须是当前 q 下 propagate 过的。
pub(crate) fn closure_residual(
    cd: &ClosureDecl,
    ia: usize,
    ib: usize,
    world: &[[f64; 16]],
) -> Vec<f64> {
    let rot = |m: &[f64; 16], v: [f64; 3]| {
        [
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[4] * v[0] + m[5] * v[1] + m[6] * v[2],
            m[8] * v[0] + m[9] * v[1] + m[10] * v[2],
        ]
    };
    let pa = cga_core::transform_point(world[ia], cd.at);
    let pb = cga_core::transform_point(world[ib], cd.b_at);
    let ua = v3_unit(rot(&world[ia], cd.axis));
    let ub = v3_unit(rot(&world[ib], cd.axis));
    let cr = v3_cross(ua, ub);
    vec![
        pa[0] - pb[0],
        pa[1] - pb[1],
        pa[2] - pb[2],
        cr[0],
        cr[1],
        cr[2],
    ]
}

/// 闭合残差的解析雅可比（G5+）：自由变量 (pair, 分量) 的螺旋列 s=[w;v] 作用于
/// 它驱动的一侧端点：d(pa) = w×pa+v，d(û) = w×û（旋转保持单位长）。
/// 测试纪律：FD 对拍（`closure_jacobian_fd_check`）。
pub(crate) fn closure_jacobian(
    decl: &GraphDecl,
    cd: &ClosureDecl,
    free: &[(usize, usize)],
    ia: usize,
    ib: usize,
    link_idx: &HashMap<&str, usize>,
    parent: &[Option<(usize, usize, bool)>],
    qs: &[Vec<f64>],
    world: &[[f64; 16]],
) -> Vec<Vec<f64>> {
    let rot = |m: &[f64; 16], v: [f64; 3]| {
        [
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[4] * v[0] + m[5] * v[1] + m[6] * v[2],
            m[8] * v[0] + m[9] * v[1] + m[10] * v[2],
        ]
    };
    let is_down = |pi: usize, mut v: usize| -> bool {
        while let Some((pj, u, _)) = parent[v] {
            if pj == pi {
                return true;
            }
            v = u;
        }
        false
    };
    let pa = cga_core::transform_point(world[ia], cd.at);
    let pb = cga_core::transform_point(world[ib], cd.b_at);
    let ua = v3_unit(rot(&world[ia], cd.axis));
    let ub = v3_unit(rot(&world[ib], cd.axis));
    let mut j = vec![vec![0.0; free.len()]; 6];
    for (k, &(pi, c)) in free.iter().enumerate() {
        let p = &decl.pairs[pi];
        let fa = mat4_mul(
            world[link_idx[p.a.as_str()]],
            mat4_mul(translate4(p.at), rpy4(p.rpy)),
        );
        let s = screws_world_at(fa, &p.kind, p.axis, p.pitch.unwrap_or(0.0), &qs[pi])[c];
        // 螺旋列 s 是「b 侧相对 a 侧」的运动。生成树正向边（b 侧下游）下游子树
        // 随 +s；反向边（a 侧下游）下游子树随 −s。符号只取决于生成树方向，
        // 与被驱动的是闭合的哪一端无关（反向边 FD 对拍：
        // closure_jacobian_fd_check_reversed_edges）。
        let sgn = if is_down(pi, link_idx[p.b.as_str()]) {
            1.0
        } else {
            -1.0
        };
        let (w, v) = (
            [sgn * s[0], sgn * s[1], sgn * s[2]],
            [sgn * s[3], sgn * s[4], sgn * s[5]],
        );
        let (da, db) = (is_down(pi, ia), is_down(pi, ib));
        let d_pa = if da {
            v3_add(v3_cross(w, pa), v)
        } else {
            [0.0; 3]
        };
        let d_pb = if db {
            v3_add(v3_cross(w, pb), v)
        } else {
            [0.0; 3]
        };
        let d_ua = if da { v3_cross(w, ua) } else { [0.0; 3] };
        let d_ub = if db { v3_cross(w, ub) } else { [0.0; 3] };
        let d_cr = v3_add(v3_cross(d_ua, ub), v3_cross(ua, d_ub));
        for i in 0..3 {
            j[i][k] = d_pa[i] - d_pb[i];
            j[3 + i][k] = d_cr[i];
        }
    }
    j
}

/// 解析雅可比版 LM（G5+：闭链残差的螺旋列解析导数；`lm_solve` 的 FD 版保留
/// 给对拍测试）。收敛判据与停滞语义同 `lm_solve`。
fn lm_solve_j(
    x: &mut [f64],
    r: &mut dyn FnMut(&[f64]) -> Vec<f64>,
    jac: &mut dyn FnMut(&[f64]) -> Vec<Vec<f64>>,
) -> Result<Vec<Vec<f64>>, String> {
    let n = x.len();
    let mut lambda = 1e-3;
    let mut cur = r(x);
    let mut cur_norm = cur.iter().map(|v| v * v).sum::<f64>().sqrt();
    let mut j: Vec<Vec<f64>> = Vec::new();
    for _ in 0..100 {
        if cur_norm < 1e-9 {
            return Ok(j);
        }
        let m = cur.len();
        j = jac(x);
        debug_assert_eq!(j.len(), m);
        let mut a = vec![vec![0.0; n]; n];
        let mut b = vec![0.0; n];
        for i in 0..m {
            for u in 0..n {
                b[u] -= j[i][u] * cur[i];
                for v in 0..n {
                    a[u][v] += j[i][u] * j[i][v];
                }
            }
        }
        let mut solved = false;
        for _ in 0..50 {
            let mut aa = a.clone();
            for u in 0..n {
                aa[u][u] += lambda * (a[u][u].abs() + 1e-12);
            }
            if let Some(delta) = gauss_solve(&aa, &b) {
                let xn: Vec<f64> = x.iter().zip(delta.iter()).map(|(x, d)| x + d).collect();
                let rn = r(&xn);
                let rn_norm = rn.iter().map(|v| v * v).sum::<f64>().sqrt();
                if rn_norm < cur_norm {
                    x.copy_from_slice(&xn);
                    cur = rn;
                    cur_norm = rn_norm;
                    lambda = (lambda / 3.0).max(1e-12);
                    solved = true;
                    break;
                }
            }
            lambda *= 10.0;
            if lambda > 1e12 {
                break;
            }
        }
        if !solved {
            return Err(format!("LM 停滞（残差 {cur_norm:e}）"));
        }
    }
    if cur_norm < 1e-7 {
        return Ok(j);
    }
    Err(format!("LM 100 轮未收敛（残差 {cur_norm:e}）"))
}

/// 矩阵的秩（高斯消元，列主元）。
fn mat_rank(m: &[Vec<f64>]) -> usize {
    if m.is_empty() || m[0].is_empty() {
        return 0;
    }
    let (rows, cols) = (m.len(), m[0].len());
    let mut a: Vec<Vec<f64>> = m.to_vec();
    let mut rank = 0;
    let mut row = 0;
    for c in 0..cols {
        // 列主元：第一个非零行
        let Some(pr) = (row..rows).find(|&r| a[r][c].abs() > 1e-9) else {
            continue;
        };
        a.swap(row, pr);
        let d = a[row][c];
        for v in a[row].iter_mut().take(cols).skip(c) {
            *v /= d;
        }
        for r in 0..rows {
            if r != row {
                let f = a[r][c];
                let (src, dst) = (a[row].clone(), &mut a[r]);
                for (k, dv) in dst.iter_mut().enumerate().take(cols).skip(c) {
                    *dv -= f * src[k];
                }
            }
        }
        row += 1;
        rank += 1;
    }
    rank
}

/// 小规模高斯消元（部分主元），奇异 → None。
fn gauss_solve(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = a.len();
    let mut m: Vec<Vec<f64>> = a.to_vec();
    let mut x = b.to_vec();
    for c in 0..n {
        // 部分主元
        let mut piv = c;
        for (r, row_r) in m.iter().enumerate().skip(c + 1) {
            if row_r[c].abs() > m[piv][c].abs() {
                piv = r;
            }
        }
        if m[piv][c].abs() < 1e-14 {
            return None;
        }
        m.swap(c, piv);
        x.swap(c, piv);
        let pivot_val = m[c][c];
        let (top, rest) = m.split_at_mut(c + 1);
        for (ri, row_r) in rest.iter_mut().enumerate() {
            let r = c + 1 + ri;
            let f = row_r[c] / pivot_val;
            let src = top[c].clone();
            for (k, mv) in row_r.iter_mut().enumerate().take(n).skip(c) {
                *mv -= f * src[k];
            }
            x[r] -= f * x[c];
        }
    }
    let mut out = vec![0.0; n];
    for c in (0..n).rev() {
        let mut s = x[c];
        for k in c + 1..n {
            s -= m[c][k] * out[k];
        }
        out[c] = s / m[c][c];
    }
    Some(out)
}

// ---- 速度级运动学（docs/roadmap.md §3.B）----------------------------------
// twist 约定与 sweep_toi 一致：物理量纲 [ω; v]，ṗ = ω×(p−o) + v
// （旋转副 v = o×ω）。注意 Multivector::velocity 存二重矢量系数（物理角速度
// = 2ω），本模块输出一律物理量纲。全部雅可比列都有有限差分对拍看守。

/// 副在当前位姿的世界螺旋轴列（多 DOF 副多列，列序 = q 的分量序）。
/// `fa_world` 是运动前的副 frame（F_a 的世界位姿，solve_graph 已填）。
pub fn pair_screws_world(p: &PairDef) -> Vec<[f64; 6]> {
    screws_world_at(p.fa_world, &p.kind, p.axis, p.pitch.unwrap_or(0.0), &p.q)
}

/// 世界系螺旋轴列（给定 F_a 世界位姿与当前 q）。closure 求解的解析雅可比每轮
/// 用新 fa 调它（`pair_screws_world` 是本函数在 PairDef 上的封装）。
pub fn screws_world_at(
    fa: [f64; 16],
    kind: &JointKind,
    axis: [f64; 3],
    pitch: f64,
    q: &[f64],
) -> Vec<[f64; 6]> {
    let f = fa;
    let r = |v: [f64; 3]| {
        [
            f[0] * v[0] + f[1] * v[1] + f[2] * v[2],
            f[4] * v[0] + f[5] * v[1] + f[6] * v[2],
            f[8] * v[0] + f[9] * v[1] + f[10] * v[2],
        ]
    };
    let o0 = [f[3], f[7], f[11]];
    let axis_w = v3_unit(r(axis));
    let rot_col = |o: [f64; 3]| {
        let w = axis_w;
        let v = v3_cross(o, w);
        [w[0], w[1], w[2], v[0], v[1], v[2]]
    };
    let slide_col = [0.0, 0.0, 0.0, axis_w[0], axis_w[1], axis_w[2]];
    match kind {
        JointKind::Revolute | JointKind::Continuous => vec![rot_col(o0)],
        JointKind::Prismatic => vec![slide_col],
        JointKind::Helical => {
            // M = T(axis·p·q)·R(axis,q)：旋转心随平移走。
            let q = q.first().copied().unwrap_or(0.0);
            let o = v3_add(o0, v3_scale(axis_w, pitch * q));
            let v = v3_add(v3_cross(o, axis_w), v3_scale(axis_w, pitch));
            vec![[axis_w[0], axis_w[1], axis_w[2], v[0], v[1], v[2]]]
        }
        JointKind::Cylindrical => {
            // M = T(axis·qp)·R(axis,qr)：旋转心在平移后的点。
            let qp = q.get(1).copied().unwrap_or(0.0);
            let o = v3_add(o0, v3_scale(axis_w, qp));
            vec![rot_col(o), slide_col]
        }
        JointKind::Spherical => {
            // M = Rx·Ry·Rz（先 z 后 y 后 x）：瞬时轴 = x, Rx·y, Rx·Ry·z（F_a 系）。
            let q = q;
            let (rx, ry) = (
                q.first().copied().unwrap_or(0.0),
                q.get(1).copied().unwrap_or(0.0),
            );
            let rx_m = Multivector::rotor([1.0, 0.0, 0.0], rx).to_matrix();
            let ry_m = Multivector::rotor([0.0, 1.0, 0.0], ry).to_matrix();
            let m3 = |m: [f64; 16], v: [f64; 3]| {
                [
                    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
                    m[4] * v[0] + m[5] * v[1] + m[6] * v[2],
                    m[8] * v[0] + m[9] * v[1] + m[10] * v[2],
                ]
            };
            let axes = [
                [1.0, 0.0, 0.0],
                m3(rx_m, [0.0, 1.0, 0.0]),
                m3(mat4_mul(rx_m, ry_m), [0.0, 0.0, 1.0]),
            ];
            axes.iter()
                .map(|a| {
                    let w = v3_unit(r(*a));
                    let v = v3_cross(o0, w);
                    [w[0], w[1], w[2], v[0], v[1], v[2]]
                })
                .collect()
        }
        JointKind::Planar => {
            // M = T(x·e1 + y·e2)·R(axis,θ)：平移在 F_a 系（不经 θ 旋转！），
            // 旋转轴过当前平移后的点。
            let axis_l = axis;
            let e1 = v3_perp(axis_l);
            let e2 = v3_cross(axis_l, e1);
            let q = q;
            let th = q.get(2).copied().unwrap_or(0.0);
            let _ = th;
            let (x, y) = (
                q.first().copied().unwrap_or(0.0),
                q.get(1).copied().unwrap_or(0.0),
            );
            let o = v3_add(o0, v3_add(v3_scale(r(e1), x), v3_scale(r(e2), y)));
            let rotc = rot_col(o);
            vec![
                [0.0, 0.0, 0.0, r(e1)[0], r(e1)[1], r(e1)[2]],
                [0.0, 0.0, 0.0, r(e2)[0], r(e2)[1], r(e2)[2]],
                rotc,
            ]
        }
        JointKind::Fixed => vec![],
    }
}

/// 从 anchor 到 link 的树路径上的副（根→叶序）。
pub fn link_path_pairs(kin: &Kinematics, link: &str) -> Result<Vec<usize>, String> {
    let mut out = Vec::new();
    let mut cur = link.to_string();
    let anchor = kin.anchor.clone().unwrap_or_default();
    while cur != anchor {
        let Some(pi) = kin.pairs.iter().position(|p| p.tree_down == cur) else {
            return Err(format!(
                "kinematics: link {link} 到 anchor {anchor} 没有树路径（未知 link 或未连通）"
            ));
        };
        out.push(pi);
        cur = kin.pairs[pi].tree_up.clone();
    }
    out.reverse();
    Ok(out)
}

/// 雅可比的列：`(pair 名, q 分量下标, 世界螺旋轴)`。
pub type JacobianCol = (Option<String>, usize, [f64; 6]);

/// 连杆末端雅可比：列 = 路径上各副的世界螺旋轴（列序 = 树路径序，多 DOF 副多列）。
/// 螺旋列是「b 侧相对 a 侧」的运动：正向边（tree_up == a）取 +s，反向边取 −s
/// （下游是 a 侧）。路径只含树边，方向由 PairDef.tree_up/tree_down 判定。
pub fn jacobian(kin: &Kinematics, link: &str) -> Result<Vec<JacobianCol>, String> {
    let mut cols = Vec::new();
    for pi in link_path_pairs(kin, link)? {
        let p = &kin.pairs[pi];
        let sgn = if p.tree_up == p.a { 1.0 } else { -1.0 };
        for (k, s) in pair_screws_world(p).into_iter().enumerate() {
            cols.push((
                p.name.clone(),
                k,
                [
                    sgn * s[0],
                    sgn * s[1],
                    sgn * s[2],
                    sgn * s[3],
                    sgn * s[4],
                    sgn * s[5],
                ],
            ));
        }
    }
    Ok(cols)
}

/// 连杆 twist（世界系）：Σ 螺旋列 · q̇。`qd` 按 pair 名给速率（无名/未给 = 0）。
pub fn link_twist(
    kin: &Kinematics,
    link: &str,
    qd: &HashMap<String, f64>,
) -> Result<[f64; 6], String> {
    let mut v = [0.0; 6];
    for (name, _, s) in jacobian(kin, link)? {
        let q = name
            .as_deref()
            .and_then(|n| qd.get(n))
            .copied()
            .unwrap_or(0.0);
        for i in 0..6 {
            v[i] += s[i] * q;
        }
    }
    Ok(v)
}

/// 连杆上一点的世界系速度：ṗ = ω×(p − o) + v，由 twist [ω; v] 计算
/// （twist 的 v 定义在螺旋轴点 o 上；这里换到原点参考：ṗ = ω×p + (v − ω×o)…
/// 约定统一为 ṗ = ω×(p−o)+v ⇒ 用世界螺旋 [ω; v] 时 ṗ = ω×p + v 当且仅当 v 是
/// 原点处的线速度。pair_screws_world 的 v = o×ω 正是"绕 o 转"的原点线速度）。
pub fn point_velocity(
    kin: &Kinematics,
    link: &str,
    point_world: [f64; 3],
    qd: &HashMap<String, f64>,
) -> Result<[f64; 3], String> {
    let t = link_twist(kin, link, qd)?;
    let w = [t[0], t[1], t[2]];
    let v = [t[3], t[4], t[5]];
    // ṗ = ω×p + v（v 是原点参考的线速度：v = o×ω 时 ω×p + o×ω = ω×(p−o)...
    // 标准螺旋：ṗ = ω×p + v，v = −ω×o。我们的列给的是 v = o×ω？看 rot_col：
    // v = o×ω ⇒ ṗ = ω×(p−o) = ω×p − ω×o = ω×p + o×ω ✓ 所以 ṗ = ω×p + v 直接成立。
    let c = v3_cross(w, point_world);
    Ok([c[0] + v[0], c[1] + v[1], c[2] + v[2]])
}

/// 列空间的秩（高斯消元，列主元）。
fn cols_rank(cols: &[[f64; 6]]) -> usize {
    let n = cols.len();
    if n == 0 {
        return 0;
    }
    let mut m: Vec<Vec<f64>> = (0..6)
        .map(|r| cols.iter().map(|c| c[r]).collect())
        .collect();
    let mut rank = 0;
    let mut row = 0;
    for c in 0..n {
        let Some(pr) = (row..6).find(|&r| m[r][c].abs() > 1e-9) else {
            continue;
        };
        m.swap(row, pr);
        let d = m[row][c];
        for v in m[row].iter_mut().take(n).skip(c) {
            *v /= d;
        }
        let src: Vec<f64> = m[row].clone();
        for (r, m_r) in m.iter_mut().enumerate() {
            if r != row {
                let f = m_r[c];
                for (k, mv) in m_r.iter_mut().enumerate().take(n).skip(c) {
                    *mv -= f * src[k];
                }
            }
        }
        row += 1;
        rank += 1;
    }
    rank
}

/// 奇异位形：路径雅可比列秩 < 路径自由度数（如平面 2R 臂伸直）。
pub fn is_singular(kin: &Kinematics, link: &str) -> Result<bool, String> {
    let cols = jacobian(kin, link)?;
    Ok(cols_rank(&cols.iter().map(|c| c.2).collect::<Vec<_>>()) < cols.len())
}

/// Grübler 活动度（构图期可知）：机构固有活动度 = 不钉任何输入时的值。
/// 口径：gross = Σ 副的 q 维数 + Σ closure（每个 revolute 闭环自带 1 个旋转
/// 自由度）；pins = 作者给定/pose 的 q 数 + gear 方程数 + cam 数 +
/// Σ closure（解出的 q 数 + 1（自身被确定的 DOF））。
#[derive(Clone, Debug)]
pub struct MobilityReport {
    pub gross_dofs: usize,
    pub author_pins: usize,
    pub gears: usize,
    pub cams: usize,
    pub closure_pins: usize,
    pub dof: i64,
}

pub fn mobility(kin: &Kinematics) -> MobilityReport {
    let gross: usize = kin.pairs.iter().map(|p| p.q.len()).sum::<usize>() + kin.closures.len();
    let author_pins = kin.pairs.iter().filter(|p| p.given).count();
    let closure_pins: usize = kin.closures.iter().map(|c| c.rank + 1).sum();
    MobilityReport {
        gross_dofs: gross,
        author_pins,
        gears: kin.gears.len(),
        cams: kin.cams.len(),
        closure_pins,
        dof: gross as i64 - (author_pins + kin.gears.len() + kin.cams.len() + closure_pins) as i64,
    }
}

// ---- 准静态平衡（docs/roadmap.md §3.C，C2）---------------------------------
// 重力势能 U = −Σ_links m·g·cog；平衡 ⇔ 每个自由 q 上 ∂U/∂q = 0。
// ∂cog/∂q_j = 该副螺旋列在该点的线速度（雅可比的物理用法）。LM 复用 G5 的实现，
// 无时间积分——这是静力学不是动力学。

/// 连杆质量：`(link 名, 质量, 连杆局部系质心)`。
pub type LinkMass = (String, f64, [f64; 3]);

/// 在给定 q 覆盖下重算全部连杆位姿（生成树前向传播，与 solve_graph 同规则：
/// 正向边 world(B)=world(A)∘F_a∘M(q)，反向边取逆）。
fn propagate_with_q(kin: &Kinematics, q_override: &HashMap<String, f64>) -> Vec<[f64; 16]> {
    let anchor = kin.anchor.clone().unwrap_or_default();
    let mut world: HashMap<String, [f64; 16]> = HashMap::new();
    world.insert(anchor.clone(), mat4_identity());
    // BFS 顺序：反复扫 pair，上游已知就算下游（小图，清晰度优先）
    let mut pending: Vec<&PairDef> = kin.pairs.iter().collect();
    let mut guard = 0;
    while !pending.is_empty() && guard < 1000 {
        guard += 1;
        pending.retain(|p| {
            let (up, down) = (p.tree_up.as_str(), p.tree_down.as_str());
            let Some(&wu) = world.get(up) else {
                return true;
            };
            let q = q_override
                .get(p.name.as_deref().unwrap_or(""))
                .copied()
                .map(|x| vec![x])
                .unwrap_or_else(|| p.q.clone());
            let fa = mat4_mul(translate4(p.at), rpy4(p.rpy));
            let m = joint_motion(&p.kind, p.axis, &q, p.pitch.unwrap_or(0.0));
            let fam = mat4_mul(fa, m);
            let w = if up == p.a {
                mat4_mul(wu, fam)
            } else {
                mat4_mul(wu, mat4_inv(fam))
            };
            world.insert(down.to_string(), w);
            false
        });
    }
    kin.links
        .iter()
        .map(|l| world.get(&l.name).copied().unwrap_or_else(mat4_identity))
        .collect()
}

/// 重力下的准静态平衡：自由 q（1-DOF pair 名）上重力势能最小。
/// 势能 U = −Σ m·g·cog；求解 = 带阻尼牛顿（FD 梯度 + FD Hessian + 步长截断），
/// 步接受以 U 下降为准——不是平衡方程的迭代，这是静力学不是动力学。
/// `free` 为空 = 所有未给定的 1-DOF pair。返回平衡 q（按 `free` 序）。
/// 不收敛 → Err（不许给错姿态）。
pub fn settle(
    kin: &Kinematics,
    masses: &[LinkMass],
    gravity: [f64; 3],
    free: &[String],
    guess: Option<&[f64]>,
) -> Result<Vec<(String, f64)>, String> {
    let free_names: Vec<String> = if free.is_empty() {
        kin.pairs
            .iter()
            .filter(|p| p.kind.is_1dof() && !p.given)
            .filter_map(|p| p.name.clone())
            .collect()
    } else {
        free.to_vec()
    };
    if free_names.is_empty() {
        return Ok(Vec::new());
    }
    for n in &free_names {
        let Some(p) = kin
            .pairs
            .iter()
            .find(|p| p.name.as_deref() == Some(n.as_str()))
        else {
            return Err(format!("JSX: settle 引用未知 pair {n}"));
        };
        if !p.kind.is_1dof() {
            return Err(format!(
                "JSX: settle 需要 1-DOF pair，{} 是 {}",
                n,
                p.kind.name()
            ));
        }
    }
    // U(x) = −Σ m·g·cog(x)
    let potential = |x: &[f64]| -> f64 {
        let qov: HashMap<String, f64> = free_names.iter().cloned().zip(x.iter().copied()).collect();
        let worlds = propagate_with_q(kin, &qov);
        let mut u = 0.0;
        for (li, l) in kin.links.iter().enumerate() {
            let Some((_, m, cog_l)) = masses.iter().find(|(ln, _, _)| ln == &l.name) else {
                continue;
            };
            let cog_w = cga_core::transform_point(worlds[li], *cog_l);
            u -= m * (gravity[0] * cog_w[0] + gravity[1] * cog_w[1] + gravity[2] * cog_w[2]);
        }
        u
    };
    let mut x: Vec<f64> = match guess {
        Some(g) if g.len() == free_names.len() => g.to_vec(),
        _ => free_names
            .iter()
            .map(|n| {
                kin.pairs
                    .iter()
                    .find(|p| p.name.as_deref() == Some(n.as_str()))
                    .map(|p| p.q.first().copied().unwrap_or(0.0))
                    .unwrap_or(0.0)
            })
            .collect(),
    };
    newton_minimize(&potential, &mut x).map_err(|e| format!("JSX: settle 未收敛: {e}"))?;
    Ok(free_names.into_iter().zip(x).collect())
}

/// 带阻尼与步长截断的牛顿最小化（小规模；FD 梯度 + FD Hessian + 部分主元消元）。
/// 收敛：|∇U|∞ < 1e-7·(1+|U|)。不收敛 → Err。
fn newton_minimize(f: &dyn Fn(&[f64]) -> f64, x: &mut [f64]) -> Result<(), String> {
    let n = x.len();
    let grad = |x: &[f64]| -> Vec<f64> {
        let f0 = f(x);
        (0..n)
            .map(|k| {
                let h = 1e-6 * (1.0 + x[k].abs());
                let mut xp = x.to_vec();
                xp[k] += h;
                (f(&xp) - f0) / h
            })
            .collect()
    };
    let mut u = f(x);
    for _ in 0..200 {
        let g = grad(x);
        let ginf = g.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
        if ginf < 1e-7 * (1.0 + u.abs()) {
            return Ok(());
        }
        // FD Hessian（对梯度再差分）
        let mut h = vec![vec![0.0; n]; n];
        for k in 0..n {
            let hh = 1e-4 * (1.0 + x[k].abs());
            let mut xp = x.to_vec();
            xp[k] += hh;
            let gp = grad(&xp);
            for i in 0..n {
                h[i][k] = (gp[i] - g[i]) / hh;
            }
        }
        // 对称化（FD 噪声）
        for i in 0..n {
            for j in i + 1..n {
                let s = 0.5 * (h[i][j] + h[j][i]);
                h[i][j] = s;
                h[j][i] = s;
            }
        }
        let mut lambda = 1e-6;
        let mut stepped = false;
        for _ in 0..50 {
            let mut aa = h.clone();
            for u2 in 0..n {
                aa[u2][u2] += lambda;
            }
            let b: Vec<f64> = g.iter().map(|v| -v).collect();
            if let Some(mut d) = gauss_solve(&aa, &b) {
                // 步长截断（信任域）：|δ|∞ ≤ 1.0 rad/m
                let dmax = d.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
                if dmax > 1.0 {
                    for v in d.iter_mut() {
                        *v /= dmax;
                    }
                }
                let xn: Vec<f64> = x.iter().zip(d.iter()).map(|(a, b)| a + b).collect();
                let un = f(&xn);
                if un < u {
                    x.copy_from_slice(&xn);
                    u = un;
                    stepped = true;
                    break;
                }
            }
            lambda *= 10.0;
            if lambda > 1e10 {
                break;
            }
        }
        if !stepped {
            return Err(format!("牛顿步停滞（U={u:e}）"));
        }
    }
    Err("牛顿 200 轮未收敛".to_string())
}

/// 由 SceneRun 计算各连杆的（名, 质量, 连杆局部系质心）。无质量的连杆不出现。
pub fn link_masses(run: &crate::SceneRun) -> Vec<LinkMass> {
    let mut out = Vec::new();
    for l in &run.kinematics.links {
        let mut m_total = 0.0;
        let mut acc = [0.0; 3];
        for &mi in &l.meshes {
            if let Some(Some(mp)) = run.mass_props.get(mi) {
                m_total += mp.mass;
                acc = v3_add(acc, v3_scale(mp.cog, mp.mass));
            }
        }
        if m_total > 0.0 {
            let cog_w = v3_scale(acc, 1.0 / m_total);
            let inv = mat4_inv(l.world);
            out.push((
                l.name.clone(),
                m_total,
                cga_core::transform_point(inv, cog_w),
            ));
        }
    }
    out
}

// ---- 抓取驱动（U1，docs/ue58-inspirations.md §3.U1）-----------------------
// 抓住 link 上一点拖到世界目标点 = 位置级约束（3 行残差：抓取点 − target），
// 与闭链同构但残差来自外部目标。自由变量 = anchor→link 树路径上未固定的
// 1-DOF pair。雅可比 = 路径螺旋列的解析点导数（ṗ = ω×p + v），方向符号规则
// 与闭链一致：正向边 fa_w = W_up∘F_a、列取 +s；反向边 fa_w = W_up∘M(q)⁻¹、
// 列取 −s（推导：反向边下游是 a 侧，其运动螺旋 = b 侧相对螺旋的负；
// FD 对拍见 drag_jacobian_fd_check）。

/// 抓取拖拽的求解结果。
#[derive(Clone, Debug)]
pub struct DragSolved {
    pub link: String,
    /// 求解后的抓取点世界位置（≈ target）。
    pub grab: [f64; 3],
    pub target: [f64; 3],
    /// 求解到的 pair（名 + q；V1 自由变量都是 1-DOF，裸名）。
    pub solved: Vec<(String, f64)>,
    /// 收敛后的残差范数 |grab − target|。
    pub residual: f64,
}

/// 拖拽路径传播（pub(crate) 可测）：自由变量 x 下被抓 link 的世界位姿 +
/// 各路径 pair 的 F_a 世界矩阵（雅可比用）。反向边 F_a 世界 = W_up∘M(q)⁻¹。
pub(crate) fn drag_path_walk(
    kin: &Kinematics,
    path: &[usize],
    free: &[usize],
    x: &[f64],
) -> ([f64; 16], Vec<[f64; 16]>) {
    let mut world = mat4_identity();
    let mut fa_ws = Vec::with_capacity(path.len());
    for &pi in path {
        let p = &kin.pairs[pi];
        let mut q = p.q.clone();
        if let Some(k) = free.iter().position(|&f| f == pi) {
            q[0] = x[k];
        }
        let fa = mat4_mul(translate4(p.at), rpy4(p.rpy));
        let m = joint_motion(&p.kind, p.axis, &q, p.pitch.unwrap_or(0.0));
        if p.tree_up == p.a {
            fa_ws.push(mat4_mul(world, fa));
            world = mat4_mul(world, mat4_mul(fa, m));
        } else {
            fa_ws.push(mat4_mul(world, mat4_inv(m)));
            world = mat4_mul(world, mat4_inv(mat4_mul(fa, m)));
        }
    }
    (world, fa_ws)
}

/// 抓取点雅可比（解析螺旋列；方向符号规则见模块注释）。返回 3×n（n = free 数）。
pub(crate) fn drag_jacobian(
    kin: &Kinematics,
    path: &[usize],
    free: &[usize],
    x: &[f64],
    grab_local: [f64; 3],
) -> Vec<Vec<f64>> {
    let (world, fa_ws) = drag_path_walk(kin, path, free, x);
    let p_grab = cga_core::transform_point(world, grab_local);
    let mut j = vec![vec![0.0; free.len()]; 3];
    for (k, &pi) in free.iter().enumerate() {
        let p = &kin.pairs[pi];
        let pos = path
            .iter()
            .position(|&u| u == pi)
            .expect("drag: 自由 pair 必在路径上");
        let mut q = p.q.clone();
        q[0] = x[k];
        let s = screws_world_at(fa_ws[pos], &p.kind, p.axis, p.pitch.unwrap_or(0.0), &q)[0];
        let sgn = if p.tree_up == p.a { 1.0 } else { -1.0 };
        let w = [sgn * s[0], sgn * s[1], sgn * s[2]];
        let v = [sgn * s[3], sgn * s[4], sgn * s[5]];
        let d = v3_add(v3_cross(w, p_grab), v);
        for i in 0..3 {
            j[i][k] = d[i];
        }
    }
    j
}

/// 拖拽的自由变量选择（抓点/抓位姿共用）：(anchor→link 路径 pair 下标, 自由 pair 下标)。
/// V1 边界全部在这里拒绝：cam / 闭链相交 / gear / 多 DOF / 无名自由 pair。
fn drag_free_pairs(
    kin: &Kinematics,
    link: &str,
    extra_free: &std::collections::HashSet<String>,
) -> Result<(Vec<usize>, Vec<usize>), String> {
    if !kin.links.iter().any(|l| l.name == link) {
        return Err(format!("JSX: drag 引用未知 link {link}"));
    }
    if !kin.cams.is_empty() {
        return Err("JSX: drag 暂不支持含 cam 的机构（cam 解出的 q 未在图里标记，拖拽会静默违反接触；联动拖拽另行立项）".to_string());
    }
    let path = link_path_pairs(kin, link)?;
    let path_names: std::collections::HashSet<&str> = path
        .iter()
        .filter_map(|&pi| kin.pairs[pi].name.as_deref())
        .collect();
    for cl in &kin.closures {
        for (n, _) in &cl.solved {
            // 多 DOF 副在 solved 名单里按 name[c] 展开，比较时去分量后缀。
            let base = n.split('[').next().unwrap_or(n);
            if path_names.contains(base) {
                return Err(format!(
                    "JSX: drag 的路径经过闭链求解的 pair {base}（闭链机构拖拽 = 拖动+闭链联解，另行立项）"
                ));
            }
        }
    }
    let geared: std::collections::HashSet<&str> = kin
        .gears
        .iter()
        .flat_map(|g| [g.a.as_str(), g.b.as_str()])
        .collect();
    let mut free: Vec<usize> = Vec::new();
    for &pi in &path {
        let p = &kin.pairs[pi];
        let redrag = p.name.as_ref().is_some_and(|n| extra_free.contains(n));
        if p.given && !redrag {
            continue;
        }
        if p.name.as_deref().is_some_and(|n| geared.contains(n)) {
            continue;
        }
        if !p.kind.is_1dof() {
            continue;
        }
        free.push(pi);
    }
    if free.is_empty() {
        return Err(format!(
            "JSX: drag 路径上没有可解的自由 1-DOF pair（link {link}；给定/gear/多 DOF 副在拖拽中固定）"
        ));
    }
    if free.iter().any(|&pi| kin.pairs[pi].name.is_none()) {
        return Err("JSX: drag 需要路径上的自由 pair 有名字（q 要经 pose 写回）".to_string());
    }
    Ok((path, free))
}

/// 抓住 link 上一点（link 局部系 `grab_local`）拖到世界系 `target`：
/// 解 anchor→link 树路径上自由 1-DOF pair 的 q（LM + 解析雅可比，与闭链同一实现）。
/// `extra_free`：上轮拖拽经 pose 写回钉住的 pair 名——它们本轮仍是自由变量。
///
/// V1 边界（诚实，不静默违反任何约束，全部在 `drag_free_pairs` 拒绝）：
/// 多 DOF 副保持当前 q；gear 耦合 pair 固定；路径触碰闭链求解 pair → Err；
/// 含 cam 机构 → Err；不收敛 / 收敛后残差超界 / 解出 q 越限 → Err（不许假装跟随）。
pub fn drag_solve(
    kin: &Kinematics,
    link: &str,
    grab_local: [f64; 3],
    target: [f64; 3],
    extra_free: &std::collections::HashSet<String>,
) -> Result<DragSolved, String> {
    let (path, free) = drag_free_pairs(kin, link, extra_free)?;
    let mut x: Vec<f64> = free.iter().map(|&pi| kin.pairs[pi].q[0]).collect();
    let resid = |x: &[f64]| -> Vec<f64> {
        let (world, _) = drag_path_walk(kin, &path, &free, x);
        let p = cga_core::transform_point(world, grab_local);
        vec![p[0] - target[0], p[1] - target[1], p[2] - target[2]]
    };
    let mut rfn = |x: &[f64]| resid(x);
    let mut jfn = |x: &[f64]| drag_jacobian(kin, &path, &free, x, grab_local);
    lm_solve_j(&mut x, &mut rfn, &mut jfn).map_err(|e| format!("JSX: drag {link} 未收敛: {e}"))?;
    let r = resid(&x);
    let rnorm = r.iter().map(|v| v * v).sum::<f64>().sqrt();
    if rnorm > 1e-6 {
        return Err(format!(
            "JSX: drag {link} 收敛后残差 {rnorm:e} 仍超界（目标不可达）"
        ));
    }
    let mut solved = Vec::new();
    for (k, &pi) in free.iter().enumerate() {
        let p = &kin.pairs[pi];
        let name = p.name.clone().expect("drag: 上面已查无名");
        if let Some([lo, hi]) = p.limit {
            if x[k] < lo || x[k] > hi {
                return Err(format!(
                    "JSX: drag 解出的 pair {name} q={} outside limit [{lo}, {hi}]（目标在限位外）",
                    x[k]
                ));
            }
        }
        solved.push((name, x[k]));
    }
    let (world, _) = drag_path_walk(kin, &path, &free, &x);
    Ok(DragSolved {
        link: link.to_string(),
        grab: cga_core::transform_point(world, grab_local),
        target,
        solved,
        residual: rnorm,
    })
}

// ---- 位姿抓取（U1 后续，docs/ue58-inspirations.md §3.U1 借清单第 1 件）------
// 把 link 的 frame（位姿，不只是点）拖到目标位姿。残差 6 行 = 位置差（3）+
// rotvec(R_c·R_tᵀ)（3，rotor 对数形式——对照 control-ga-pid 的 rotvec_between；
// 不用轴叉积，叉积在对跖朝向退化）。雅可比 = 路径螺旋列：位置行 w×p+v、朝向行 w。
// 雅可比在 r=0 处精确（FD 对拍），远离时是标准几何 IK 一阶模型——LM 接受准则
// 仍是残差下降，收敛由终检钉死，与闭链同一诚实形态。

/// 位姿拖拽的求解结果。
#[derive(Clone, Debug)]
pub struct DragPoseSolved {
    pub link: String,
    /// 求解后的 link 位姿（≈ target）。
    pub pose: [f64; 16],
    pub target: [f64; 16],
    /// 求解到的 pair（名 + q；V1 自由变量都是 1-DOF，裸名）。
    pub solved: Vec<(String, f64)>,
    /// 收敛后的残差范数（6 行：米 + 弧度混合范数，仅供判收敛用）。
    pub residual: f64,
}

/// mat4（行主序）的旋转部分。
fn mat4_rot(m: &[f64; 16]) -> [[f64; 3]; 3] {
    [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]]
}

fn mat3_transpose(m: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

fn mat3_mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

/// rotvec（轴·角）：R 的对数。θ→0 一阶分支；θ→π 对跖分支（从对角元取轴，
/// 叉积形式在那里退化——这正是残差不用轴叉积的原因）。
fn rotvec_from_mat3(r: [[f64; 3]; 3]) -> [f64; 3] {
    let tr = r[0][0] + r[1][1] + r[2][2];
    let c = ((tr - 1.0) / 2.0).clamp(-1.0, 1.0);
    let th = c.acos();
    if th < 1e-9 {
        return [
            (r[2][1] - r[1][2]) / 2.0,
            (r[0][2] - r[2][0]) / 2.0,
            (r[1][0] - r[0][1]) / 2.0,
        ];
    }
    if std::f64::consts::PI - th < 1e-6 {
        // 对跖：R = 2aaᵀ − I ⇒ 对角元给出 |a| 分量，取最大者定号。
        let mut a = [
            ((r[0][0] + 1.0) / 2.0).max(0.0).sqrt(),
            ((r[1][1] + 1.0) / 2.0).max(0.0).sqrt(),
            ((r[2][2] + 1.0) / 2.0).max(0.0).sqrt(),
        ];
        let k = if a[0] >= a[1] && a[0] >= a[2] {
            0
        } else if a[1] >= a[2] {
            1
        } else {
            2
        };
        if a[k] > 1e-12 {
            let (i, j) = ((k + 1) % 3, (k + 2) % 3);
            // 非对角元定另两个分量的号（a_i·a_k = R_ik/2）。
            for &m in &[i, j] {
                if r[m][k] + r[k][m] < 0.0 {
                    a[m] = -a[m];
                }
            }
            a = v3_unit(a);
        }
        return v3_scale(a, th);
    }
    let s = 2.0 * th.sin();
    [
        th * (r[2][1] - r[1][2]) / s,
        th * (r[0][2] - r[2][0]) / s,
        th * (r[1][0] - r[0][1]) / s,
    ]
}

/// SO(3) 对数的左雅可比之逆（闭式）：rel 受世界系扰动 exp(ŵ·h)·rel 时
/// d rotvec(rel)/dh = J_l(r)⁻¹·w。
/// J_l(r)⁻¹ = I − ½r̂ + c₂·r̂²，c₂ = 1/θ² − (1+cosθ)/(2θ·sinθ)。
/// 极限都可去：θ→0 时 c₂→1/12（级数分支）；θ→π 时 (1+cosθ)/(2θ sinθ)→u/(4θ)
/// （u=π−θ，避免 0/0）。有了它朝向行雅可比在**任意**残差处精确，不只是 r=0
/// （FD 对拍：drag_pose_jacobian_fd_check 含非零残差构型）。
fn rotvec_left_jac_inv(r: [f64; 3]) -> [[f64; 3]; 3] {
    let th = v3_norm(r);
    let skew = [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]];
    let skew2 = mat3_mul(skew, skew);
    let c2 = if th < 1e-9 {
        1.0 / 12.0
    } else if th > std::f64::consts::PI - 1e-3 {
        let u = std::f64::consts::PI - th;
        1.0 / (th * th) - u / (4.0 * th)
    } else {
        1.0 / (th * th) - (1.0 + th.cos()) / (2.0 * th * th.sin())
    };
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = if i == j { 1.0 } else { 0.0 } - 0.5 * skew[i][j] + c2 * skew2[i][j];
        }
    }
    out
}

fn mat3_vec(m: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// 位姿残差（pub(crate) 可测）：位置差（3）+ rotvec(R_c·R_tᵀ)（3）。
pub(crate) fn drag_pose_residual(
    kin: &Kinematics,
    path: &[usize],
    free: &[usize],
    x: &[f64],
    target: &[f64; 16],
) -> Vec<f64> {
    let (world, _) = drag_path_walk(kin, path, free, x);
    let rc = mat4_rot(&world);
    let rt = mat4_rot(target);
    let rel = mat3_mul(rc, mat3_transpose(rt));
    let rv = rotvec_from_mat3(rel);
    vec![
        world[3] - target[3],
        world[7] - target[7],
        world[11] - target[11],
        rv[0],
        rv[1],
        rv[2],
    ]
}

/// 位姿雅可比（pub(crate) 可测）：位置行 w×p+v（任意位形精确）；朝向行
/// J_l(r_rot)⁻¹·w（左雅可比逆闭式，任意残差处精确——不只是 r=0）。
pub(crate) fn drag_pose_jacobian(
    kin: &Kinematics,
    path: &[usize],
    free: &[usize],
    x: &[f64],
    target: &[f64; 16],
) -> Vec<Vec<f64>> {
    let (world, fa_ws) = drag_path_walk(kin, path, free, x);
    let p = [world[3], world[7], world[11]];
    let rel = mat3_mul(mat4_rot(&world), mat3_transpose(mat4_rot(target)));
    let jl_inv = rotvec_left_jac_inv(rotvec_from_mat3(rel));
    let mut j = vec![vec![0.0; free.len()]; 6];
    for (k, &pi) in free.iter().enumerate() {
        let pd = &kin.pairs[pi];
        let pos = path
            .iter()
            .position(|&u| u == pi)
            .expect("drag_pose: 自由 pair 必在路径上");
        let mut q = pd.q.clone();
        q[0] = x[k];
        let s = screws_world_at(fa_ws[pos], &pd.kind, pd.axis, pd.pitch.unwrap_or(0.0), &q)[0];
        let sgn = if pd.tree_up == pd.a { 1.0 } else { -1.0 };
        let w = [sgn * s[0], sgn * s[1], sgn * s[2]];
        let v = [sgn * s[3], sgn * s[4], sgn * s[5]];
        let dp = v3_add(v3_cross(w, p), v);
        let dw = mat3_vec(jl_inv, w);
        for i in 0..3 {
            j[i][k] = dp[i];
            j[3 + i][k] = dw[i];
        }
    }
    j
}

/// 把 link 的 frame 拖到目标位姿 `target`（4×4 齐次矩阵，旋转部分必须正规，
/// 否则显式 Err）。自由变量/边界与 [`drag_solve`] 相同（`drag_free_pairs`）。
/// 自由 DOF < 6 时 LM 求最小二乘——目标不可达由终检残差显式拒绝，不假装跟随。
pub fn drag_pose_solve(
    kin: &Kinematics,
    link: &str,
    target: [f64; 16],
    extra_free: &std::collections::HashSet<String>,
) -> Result<DragPoseSolved, String> {
    // target 正规性：RᵀR ≈ I（不许拿缩放/剪切矩阵当位姿）。
    let rt = mat4_rot(&target);
    let gram = mat3_mul(mat3_transpose(rt), rt);
    let ident = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for i in 0..3 {
        for j in 0..3 {
            if (gram[i][j] - ident[i][j]).abs() > 1e-9 {
                return Err("JSX: drag_pose 的 target 旋转部分不正规（RᵀR ≠ I）".to_string());
            }
        }
    }
    let (path, free) = drag_free_pairs(kin, link, extra_free)?;
    let mut x: Vec<f64> = free.iter().map(|&pi| kin.pairs[pi].q[0]).collect();
    let mut rfn = |x: &[f64]| drag_pose_residual(kin, &path, &free, x, &target);
    let mut jfn = |x: &[f64]| drag_pose_jacobian(kin, &path, &free, x, &target);
    lm_solve_j(&mut x, &mut rfn, &mut jfn)
        .map_err(|e| format!("JSX: drag_pose {link} 未收敛: {e}"))?;
    let r = drag_pose_residual(kin, &path, &free, &x, &target);
    let rnorm = r.iter().map(|v| v * v).sum::<f64>().sqrt();
    if rnorm > 1e-6 {
        return Err(format!(
            "JSX: drag_pose {link} 收敛后残差 {rnorm:e} 仍超界（目标不可达）"
        ));
    }
    let mut solved = Vec::new();
    for (k, &pi) in free.iter().enumerate() {
        let p = &kin.pairs[pi];
        let name = p.name.clone().expect("drag_pose: 上面已查无名");
        if let Some([lo, hi]) = p.limit {
            if x[k] < lo || x[k] > hi {
                return Err(format!(
                    "JSX: drag_pose 解出的 pair {name} q={} outside limit [{lo}, {hi}]（目标在限位外）",
                    x[k]
                ));
            }
        }
        solved.push((name, x[k]));
    }
    let (world, _) = drag_path_walk(kin, &path, &free, &x);
    Ok(DragPoseSolved {
        link: link.to_string(),
        pose: world,
        target,
        solved,
        residual: rnorm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// G5+ 测试场景（手工 decl）：base —cyl(z)→ l1（臂端 [1,0,0]）—sph→ l2，
    /// 闭合 l2 的 [0,1,0] 到 base 的 [1,1,0]，轴 z。零位形天然闭合：
    /// q 全 0 时 l2 端点 = (1,0,0)+(0,1,0) = (1,1,0) 且轴对齐。
    fn multi_dof_decl(guess_cyl: [f64; 2], guess_sph: [f64; 3]) -> GraphDecl {
        GraphDecl {
            links: vec!["base".into(), "l1".into(), "l2".into()],
            pairs: vec![
                PairDecl {
                    name: Some("cyl".into()),
                    kind: JointKind::Cylindrical,
                    a: "base".into(),
                    b: "l1".into(),
                    at: [0.0; 3],
                    axis: [0.0, 0.0, 1.0],
                    rpy: [0.0; 3],
                    q_init: vec![],
                    q_guess: guess_cyl.to_vec(),
                    q_given: false,
                    pitch: None,
                    limit: None,
                },
                PairDecl {
                    name: Some("sph".into()),
                    kind: JointKind::Spherical,
                    a: "l1".into(),
                    b: "l2".into(),
                    at: [1.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    rpy: [0.0; 3],
                    q_init: vec![],
                    q_guess: guess_sph.to_vec(),
                    q_given: false,
                    pitch: None,
                    limit: None,
                },
            ],
            anchor: Some("base".into()),
            gears: vec![],
            cams: vec![],
            closures: vec![ClosureDecl {
                a: "l2".into(),
                b: "base".into(),
                at: [0.0, 1.0, 0.0],
                b_at: [1.0, 1.0, 0.0],
                axis: [0.0, 0.0, 1.0],
            }],
        }
    }

    #[test]
    fn multi_dof_closure_recovers_known_assembly() {
        // 自由变量 5（cyl 2 + sph 3），残差秩 5 → 局部唯一解 = 零位形。
        // 从偏移 guess 收敛回零 = 空间多自由度闭链的闭式验证。
        let decl = multi_dof_decl([0.15, 0.08], [0.12, -0.09, 0.2]);
        let sol = solve_graph(&decl, &HashMap::new()).expect("solve");
        assert_eq!(sol.closures.len(), 1);
        let cl = &sol.closures[0];
        assert!(cl.residual < 1e-8, "残差 {}", cl.residual);
        assert_eq!(cl.rank, 5, "秩 5（点 3 + 轴叉积 2）");
        assert_eq!(cl.solved.len(), 5, "{:?}", cl.solved);
        for (n, q) in &cl.solved {
            assert!(q.abs() < 1e-6, "{n} 应回到 0，got {q}");
        }
        // 名字按分量展开
        assert!(cl.solved.iter().any(|(n, _)| n == "cyl[0]"));
        assert!(cl.solved.iter().any(|(n, _)| n == "sph[2]"));
    }

    #[test]
    fn closure_lm_fd_crosscheck() {
        // FD 版 LM（lm_solve）作裁判：同一闭链系统用 FD 雅可比解一遍，
        // 必须与解析版（solve_graph 内）收敛到同一根。
        let decl = multi_dof_decl([0.15, 0.08], [0.12, -0.09, 0.2]);
        let cd = &decl.closures[0];
        let parent: [Option<(usize, usize, bool)>; 3] =
            [None, Some((0, 0, true)), Some((1, 1, true))];
        let order = [1usize, 2usize];
        let fwd = |qs: &Vec<Vec<f64>>| -> Vec<[f64; 16]> {
            let mut world = vec![mat4_identity(); 3];
            for &v in &order {
                let (pi, u, is_fwd) = parent[v].unwrap();
                let p = &decl.pairs[pi];
                let fa = mat4_mul(translate4(p.at), rpy4(p.rpy));
                let m = joint_motion(&p.kind, p.axis, &qs[pi], p.pitch.unwrap_or(0.0));
                world[v] = if is_fwd {
                    mat4_mul(world[u], mat4_mul(fa, m))
                } else {
                    mat4_mul(world[u], mat4_inv(mat4_mul(fa, m)))
                };
            }
            world
        };
        let mut x = vec![0.15, 0.08, 0.12, -0.09, 0.2];
        lm_solve(&mut x, &mut |x: &[f64]| {
            let qs = vec![vec![x[0], x[1]], vec![x[2], x[3], x[4]]];
            closure_residual(cd, 2, 0, &fwd(&qs))
        })
        .expect("FD LM 收敛");
        for (i, v) in x.iter().enumerate() {
            assert!(v.abs() < 1e-6, "FD 根[{i}] 应回到 0，got {v}");
        }
    }

    #[test]
    fn closure_jacobian_fd_check() {
        // 解析雅可比 vs FD（裁判）。任意非零位形，手工 parent/order/前向传播。
        let decl = multi_dof_decl([0.0, 0.0], [0.0, 0.0, 0.0]);
        let cd = &decl.closures[0];
        let link_idx: HashMap<&str, usize> =
            [("base", 0), ("l1", 1), ("l2", 2)].into_iter().collect();
        // parent[v] = (pair, 上游 link, fwd)
        let parent: [Option<(usize, usize, bool)>; 3] =
            [None, Some((0, 0, true)), Some((1, 1, true))];
        let order = [1usize, 2usize];
        let qs: Vec<Vec<f64>> = vec![vec![0.3, 0.2], vec![0.1, -0.2, 0.15]];
        let free: Vec<(usize, usize)> = vec![(0, 0), (0, 1), (1, 0), (1, 1), (1, 2)];
        let fwd = |qs: &Vec<Vec<f64>>| -> Vec<[f64; 16]> {
            let mut world = vec![mat4_identity(); 3];
            for &v in &order {
                let (pi, u, is_fwd) = parent[v].unwrap();
                let p = &decl.pairs[pi];
                let fa = mat4_mul(translate4(p.at), rpy4(p.rpy));
                let m = joint_motion(&p.kind, p.axis, &qs[pi], p.pitch.unwrap_or(0.0));
                world[v] = if is_fwd {
                    mat4_mul(world[u], mat4_mul(fa, m))
                } else {
                    mat4_mul(world[u], mat4_inv(mat4_mul(fa, m)))
                };
            }
            world
        };
        let world = fwd(&qs);
        let ja = closure_jacobian(&decl, cd, &free, 2, 0, &link_idx, &parent, &qs, &world);
        // FD 雅可比
        for (k, &(pi, c)) in free.iter().enumerate() {
            let h = 1e-7;
            let mut qp = qs.clone();
            qp[pi][c] += h;
            let rp = closure_residual(cd, 2, 0, &fwd(&qp));
            let r0 = closure_residual(cd, 2, 0, &fwd(&qs));
            for i in 0..6 {
                let fd = (rp[i] - r0[i]) / h;
                assert!(
                    (ja[i][k] - fd).abs() < 1e-5 + 1e-5 * fd.abs(),
                    "J[{i}][{k}] 解析 {} vs FD {fd}",
                    ja[i][k]
                );
            }
        }
    }

    /// 反向声明的 G5+ 场景：pair 朝 anchor 写（a 侧在生成树下游，BFS 反向边）。
    /// 几何与 multi_dof_decl 同构：零位形下 l2 的 [0,1,0] 落在 base 的 [−1,1,0]。
    fn multi_dof_decl_reversed() -> GraphDecl {
        GraphDecl {
            links: vec!["base".into(), "l1".into(), "l2".into()],
            pairs: vec![
                PairDecl {
                    name: Some("cyl".into()),
                    kind: JointKind::Cylindrical,
                    a: "l1".into(),
                    b: "base".into(),
                    at: [0.0; 3],
                    axis: [0.0, 0.0, 1.0],
                    rpy: [0.0; 3],
                    q_init: vec![],
                    q_guess: vec![],
                    q_given: false,
                    pitch: None,
                    limit: None,
                },
                PairDecl {
                    name: Some("sph".into()),
                    kind: JointKind::Spherical,
                    a: "l2".into(),
                    b: "l1".into(),
                    at: [1.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    rpy: [0.0; 3],
                    q_init: vec![],
                    q_guess: vec![],
                    q_given: false,
                    pitch: None,
                    limit: None,
                },
            ],
            anchor: Some("base".into()),
            gears: vec![],
            cams: vec![],
            closures: vec![ClosureDecl {
                a: "l2".into(),
                b: "base".into(),
                at: [0.0, 1.0, 0.0],
                b_at: [-1.0, 1.0, 0.0],
                axis: [0.0, 0.0, 1.0],
            }],
        }
    }

    #[test]
    fn closure_jacobian_fd_check_reversed_edges() {
        // 探针（U1 前期）：反向边的下游是 a 侧，其运动螺旋 = −s（b 侧相对螺旋取负）。
        // closure_jacobian 必须与 FD 一致——不一致就是反向边符号漏处理的实证。
        let decl = multi_dof_decl_reversed();
        let cd = &decl.closures[0];
        let link_idx: HashMap<&str, usize> =
            [("base", 0), ("l1", 1), ("l2", 2)].into_iter().collect();
        // parent[v] = (pair, 上游 link, fwd)：两条边都是反向（fwd=false）。
        let parent: [Option<(usize, usize, bool)>; 3] =
            [None, Some((0, 0, false)), Some((1, 1, false))];
        let order = [1usize, 2usize];
        let qs: Vec<Vec<f64>> = vec![vec![0.3, 0.2], vec![0.1, -0.2, 0.15]];
        let free: Vec<(usize, usize)> = vec![(0, 0), (0, 1), (1, 0), (1, 1), (1, 2)];
        let fwd = |qs: &Vec<Vec<f64>>| -> Vec<[f64; 16]> {
            let mut world = vec![mat4_identity(); 3];
            for &v in &order {
                let (pi, u, is_fwd) = parent[v].unwrap();
                let p = &decl.pairs[pi];
                let fa = mat4_mul(translate4(p.at), rpy4(p.rpy));
                let m = joint_motion(&p.kind, p.axis, &qs[pi], p.pitch.unwrap_or(0.0));
                world[v] = if is_fwd {
                    mat4_mul(world[u], mat4_mul(fa, m))
                } else {
                    mat4_mul(world[u], mat4_inv(mat4_mul(fa, m)))
                };
            }
            world
        };
        let world = fwd(&qs);
        let ja = closure_jacobian(&decl, cd, &free, 2, 0, &link_idx, &parent, &qs, &world);
        for (k, &(pi, c)) in free.iter().enumerate() {
            let h = 1e-7;
            let mut qp = qs.clone();
            qp[pi][c] += h;
            let rp = closure_residual(cd, 2, 0, &fwd(&qp));
            let r0 = closure_residual(cd, 2, 0, &fwd(&qs));
            for i in 0..6 {
                let fd = (rp[i] - r0[i]) / h;
                assert!(
                    (ja[i][k] - fd).abs() < 1e-5 + 1e-5 * fd.abs(),
                    "反向边 J[{i}][{k}] 解析 {} vs FD {fd}",
                    ja[i][k]
                );
            }
        }
    }

    // ---- 抓取驱动（U1）------------------------------------------------------

    /// 由声明求解并组装 Kinematics（drag_solve 的输入形态）。
    fn solve_kin(decl: &GraphDecl) -> Kinematics {
        let sol = solve_graph(decl, &HashMap::new()).expect("solve");
        let links = decl
            .links
            .iter()
            .enumerate()
            .map(|(i, n)| LinkDef {
                name: n.clone(),
                meshes: vec![],
                world: sol.link_world[i],
            })
            .collect();
        Kinematics {
            links,
            pairs: sol.pairs,
            anchor: decl.anchor.clone(),
            gears: decl
                .gears
                .iter()
                .map(|g| GearRel {
                    a: g.a.clone(),
                    b: g.b.clone(),
                    ratio: g.ratio,
                    offset: g.offset,
                })
                .collect(),
            cams: sol.cams,
            closures: sol.closures,
            pose: vec![],
        }
    }

    fn pair_decl(name: &str, kind: JointKind, a: &str, b: &str, at: [f64; 3]) -> PairDecl {
        PairDecl {
            name: Some(name.into()),
            kind,
            a: a.into(),
            b: b.into(),
            at,
            axis: [0.0, 0.0, 1.0],
            rpy: [0.0; 3],
            q_init: vec![],
            q_guess: vec![],
            q_given: false,
            pitch: None,
            limit: None,
        }
    }

    /// 单摆：base —rev(z)→ l1，抓取点 l1 局部 [1,0,0]。
    fn pend_decl(a: &str, b: &str) -> GraphDecl {
        GraphDecl {
            links: vec!["base".into(), "l1".into()],
            pairs: vec![pair_decl("j", JointKind::Revolute, a, b, [0.0; 3])],
            anchor: Some("base".into()),
            gears: vec![],
            cams: vec![],
            closures: vec![],
        }
    }

    #[test]
    fn drag_pendulum_closed_form() {
        // 闭式：抓取点 (1,0,0) 拖到 (0,1,0) → q = π/2（单位圆上的旋转）。
        let kin = solve_kin(&pend_decl("base", "l1"));
        let d = drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .expect("drag");
        assert_eq!(d.solved.len(), 1);
        assert_eq!(d.solved[0].0, "j");
        assert!(
            (d.solved[0].1 - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "q 应为 π/2，got {}",
            d.solved[0].1
        );
        assert!(d.residual < 1e-9, "残差 {}", d.residual);
        for i in 0..3 {
            assert!((d.grab[i] - [0.0, 1.0, 0.0][i]).abs() < 1e-9);
        }
    }

    #[test]
    fn drag_pendulum_reversed_edge_closed_form() {
        // 反向声明（a 侧在生成树下游）：world(l1) = R(z,−q)，同一目标 → q = −π/2。
        // 与正向用例同几何、异号解——专门看守方向符号。
        let kin = solve_kin(&pend_decl("l1", "base"));
        let d = drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .expect("drag");
        assert!(
            (d.solved[0].1 + std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "反向边 q 应为 −π/2，got {}",
            d.solved[0].1
        );
        assert!(d.residual < 1e-9);
    }

    #[test]
    fn drag_unreachable_is_error() {
        // 目标 (1,1,0) 距原点 √2 ≠ 1：不在单摆可达圆上 → Err，不假装跟随。
        let kin = solve_kin(&pend_decl("base", "l1"));
        let e = drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .unwrap_err();
        assert!(e.contains("drag"), "{e}");
    }

    #[test]
    fn drag_limit_violation_is_error() {
        // 限位 [−1,1] 的单摆拖到需要 q=π/2 的目标 → Err（目标在限位外）。
        let mut decl = pend_decl("base", "l1");
        decl.pairs[0].limit = Some([-1.0, 1.0]);
        let kin = solve_kin(&decl);
        let e = drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .unwrap_err();
        assert!(e.contains("outside limit"), "{e}");
    }

    #[test]
    fn drag_prismatic_closed_form() {
        // 滑动副（x 轴）：抓取点随 q 平移，(1,0,0) → (2.5,0,0) 即 q = 1.5。
        let mut decl = pend_decl("base", "l1");
        decl.pairs[0].kind = JointKind::Prismatic;
        decl.pairs[0].axis = [1.0, 0.0, 0.0];
        let kin = solve_kin(&decl);
        let d = drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [2.5, 0.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .expect("drag");
        assert!(
            (d.solved[0].1 - 1.5).abs() < 1e-8,
            "q 应为 1.5，got {}",
            d.solved[0].1
        );
        // 滑动副到不了横向目标 → Err
        assert!(drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [2.5, 0.1, 0.0],
            &std::collections::HashSet::new(),
        )
        .is_err());
    }

    /// 平面 2R 臂：base —j1(z)→ l1 —j2(z, at=[1,0,0])→ l2，抓取 l2 局部 [1,0,0]。
    fn two_r_decl(q1: f64, q2: f64) -> GraphDecl {
        let mut j1 = pair_decl("j1", JointKind::Revolute, "base", "l1", [0.0; 3]);
        j1.q_init = vec![q1];
        let mut j2 = pair_decl("j2", JointKind::Revolute, "l1", "l2", [1.0, 0.0, 0.0]);
        j2.q_init = vec![q2];
        GraphDecl {
            links: vec!["base".into(), "l1".into(), "l2".into()],
            pairs: vec![j1, j2],
            anchor: Some("base".into()),
            gears: vec![],
            cams: vec![],
            closures: vec![],
        }
    }

    #[test]
    fn drag_two_r_arm_fk_roundtrip() {
        // 闭式往返：q*=(0.9,1.1) 正解出末端点 p*，从种子 (0.3,0.5) 拖到 p*
        // 应收回 q*（局部最近根；肘上/肘下两支离种子不等距）。
        let kin_star = solve_kin(&two_r_decl(0.9, 1.1));
        let l2w = kin_star
            .links
            .iter()
            .find(|l| l.name == "l2")
            .unwrap()
            .world;
        let p_star = cga_core::transform_point(l2w, [1.0, 0.0, 0.0]);
        let kin = solve_kin(&two_r_decl(0.3, 0.5));
        let d = drag_solve(
            &kin,
            "l2",
            [1.0, 0.0, 0.0],
            p_star,
            &std::collections::HashSet::new(),
        )
        .expect("drag");
        assert!((d.solved[0].1 - 0.9).abs() < 1e-6, "q1: {}", d.solved[0].1);
        assert!((d.solved[1].1 - 1.1).abs() < 1e-6, "q2: {}", d.solved[1].1);
        assert!(d.residual < 1e-9);
    }

    #[test]
    fn drag_jacobian_fd_check() {
        // 解析雅可比 vs FD（裁判）：j1 正向 + j2 反向（a="l2", b="l1"）混合链，
        // 专门看守反向边的 −s 与 fa_w = W_up∘M(q)⁻¹。
        let mut j2 = pair_decl("j2", JointKind::Revolute, "l2", "l1", [0.5, 0.0, 0.0]);
        j2.q_init = vec![0.3];
        let mut j1 = pair_decl("j1", JointKind::Revolute, "base", "l1", [0.0; 3]);
        j1.q_init = vec![0.4];
        let decl = GraphDecl {
            links: vec!["base".into(), "l1".into(), "l2".into()],
            pairs: vec![j1, j2],
            anchor: Some("base".into()),
            gears: vec![],
            cams: vec![],
            closures: vec![],
        };
        let kin = solve_kin(&decl);
        let path = link_path_pairs(&kin, "l2").expect("path");
        let free: Vec<usize> = path.clone();
        let grab_local = [0.5, 0.0, 0.0];
        let x: Vec<f64> = free.iter().map(|&pi| kin.pairs[pi].q[0]).collect();
        let ja = drag_jacobian(&kin, &path, &free, &x, grab_local);
        let point = |x: &[f64]| {
            let (w, _) = drag_path_walk(&kin, &path, &free, x);
            cga_core::transform_point(w, grab_local)
        };
        let p0 = point(&x);
        for k in 0..free.len() {
            let h = 1e-7;
            let mut xp = x.clone();
            xp[k] += h;
            let p1 = point(&xp);
            for i in 0..3 {
                let fd = (p1[i] - p0[i]) / h;
                assert!(
                    (ja[i][k] - fd).abs() < 1e-5 + 1e-5 * fd.abs(),
                    "drag J[{i}][{k}] 解析 {} vs FD {fd}",
                    ja[i][k]
                );
            }
        }
    }

    #[test]
    fn drag_multi_dof_pair_stays_fixed() {
        // 路径上的球副（未给定、多 DOF）在拖拽中保持当前 q：只有 rev 是自由变量。
        // rev 单变量可达的目标 (0,2,0)：q = π/2（臂长 2 的圆）。
        let j1 = pair_decl("j1", JointKind::Revolute, "base", "l1", [0.0; 3]);
        let s = pair_decl("s", JointKind::Spherical, "l1", "l2", [1.0, 0.0, 0.0]);
        let decl = GraphDecl {
            links: vec!["base".into(), "l1".into(), "l2".into()],
            pairs: vec![j1, s],
            anchor: Some("base".into()),
            gears: vec![],
            cams: vec![],
            closures: vec![],
        };
        let kin = solve_kin(&decl);
        let d = drag_solve(
            &kin,
            "l2",
            [1.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .expect("drag");
        assert_eq!(d.solved.len(), 1, "球副不得成为自由变量: {:?}", d.solved);
        assert_eq!(d.solved[0].0, "j1");
        assert!((d.solved[0].1 - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn drag_gear_pair_not_free() {
        // gear 耦合的 pair 在拖拽中固定：路径上只有 gear 副 → 无自由变量 → Err。
        let mut decl = pend_decl("base", "l1");
        decl.links.push("l2".into());
        decl.pairs
            .push(pair_decl("j2", JointKind::Revolute, "base", "l2", [0.0; 3]));
        decl.gears.push(GearDecl {
            a: "j".into(),
            b: "j2".into(),
            ratio: -2.0,
            offset: 0.0,
        });
        let kin = solve_kin(&decl);
        let e = drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .unwrap_err();
        assert!(e.contains("可解的自由"), "{e}");
    }

    #[test]
    fn drag_closure_path_rejected() {
        // 闭链机构（multi_dof_decl 的 closure 解出 cyl[*]/sph[*]）：路径与之相交 → Err。
        let decl = multi_dof_decl([0.0, 0.0], [0.0, 0.0, 0.0]);
        let kin = solve_kin(&decl);
        let e = drag_solve(
            &kin,
            "l2",
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .unwrap_err();
        assert!(e.contains("闭链"), "{e}");
    }

    #[test]
    fn drag_cam_rejected() {
        // 含 cam 的机构：cam 解出的 q 未在图里标记，拖拽会静默违反接触 → Err。
        let mut kin = solve_kin(&pend_decl("base", "l1"));
        kin.cams.push(CamSolved {
            a: "base".into(),
            b: "l1".into(),
            q: 0.0,
        });
        let e = drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .unwrap_err();
        assert!(e.contains("cam"), "{e}");
    }

    #[test]
    fn drag_extra_free_allows_redrag() {
        // 上轮拖拽经 pose 写回的 pair（given=true）仍是本轮自由变量（extra_free）。
        let mut decl = pend_decl("base", "l1");
        decl.pairs[0].q_init = vec![0.2];
        decl.pairs[0].q_given = true;
        let kin = solve_kin(&decl);
        assert!(drag_solve(
            &kin,
            "l1",
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            &std::collections::HashSet::new(),
        )
        .is_err());
        let extra: std::collections::HashSet<String> = ["j".to_string()].into_iter().collect();
        let d = drag_solve(&kin, "l1", [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], &extra).expect("re-drag");
        assert!((d.solved[0].1 - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    // ---- 位姿抓取（U1 借清单第 1 件）---------------------------------------

    #[test]
    fn drag_pose_pendulum_closed_form() {
        // link frame 从 I 拖到 R_z(π/2)（位置不动）：q = π/2。纯朝向目标——
        // 抓点版够不到，由 rotvec 残差承载。
        let kin = solve_kin(&pend_decl("base", "l1"));
        let target = Multivector::rotor([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2).to_matrix();
        let d = drag_pose_solve(&kin, "l1", target, &std::collections::HashSet::new())
            .expect("drag_pose");
        assert_eq!(d.solved.len(), 1);
        assert!(
            (d.solved[0].1 - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "q 应为 π/2，got {}",
            d.solved[0].1
        );
        assert!(d.residual < 1e-9, "残差 {}", d.residual);
    }

    #[test]
    fn drag_pose_pendulum_reversed_closed_form() {
        // 反向声明：world(l1) = R_z(−q)，同一目标 → q = −π/2。
        let kin = solve_kin(&pend_decl("l1", "base"));
        let target = Multivector::rotor([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2).to_matrix();
        let d = drag_pose_solve(&kin, "l1", target, &std::collections::HashSet::new())
            .expect("drag_pose");
        assert!(
            (d.solved[0].1 + std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "反向边 q 应为 −π/2，got {}",
            d.solved[0].1
        );
    }

    #[test]
    fn drag_pose_orientation_unreachable_is_error() {
        // z 轴单摆够不到绕 x 的旋转 → Err，不假装跟随。
        let kin = solve_kin(&pend_decl("base", "l1"));
        let target = Multivector::rotor([1.0, 0.0, 0.0], 0.5).to_matrix();
        let e = drag_pose_solve(&kin, "l1", target, &std::collections::HashSet::new()).unwrap_err();
        assert!(e.contains("drag_pose"), "{e}");
    }

    #[test]
    fn drag_pose_two_r_fk_roundtrip() {
        // 位姿 FK 往返：q*=(0.9,1.1) 的位姿作目标，从种子 (0.3,0.5) 收回 q*。
        // 2R 臂的位姿流形是 2 维（平面位置 + θ=q1+q2），FK 目标在流形上 → 精确。
        let kin_star = solve_kin(&two_r_decl(0.9, 1.1));
        let target = kin_star
            .links
            .iter()
            .find(|l| l.name == "l2")
            .unwrap()
            .world;
        let kin = solve_kin(&two_r_decl(0.3, 0.5));
        let d = drag_pose_solve(&kin, "l2", target, &std::collections::HashSet::new())
            .expect("drag_pose");
        assert!((d.solved[0].1 - 0.9).abs() < 1e-6, "q1: {}", d.solved[0].1);
        assert!((d.solved[1].1 - 1.1).abs() < 1e-6, "q2: {}", d.solved[1].1);
        assert!(d.residual < 1e-9);
    }

    #[test]
    fn drag_pose_jacobian_fd_check() {
        // 朝向行有左雅可比逆后在**任意**残差处精确：两组构型对拍——r=0（target =
        // 当前位姿）与非零残差（target 偏移 [0.05,−0.04]）。FD 是裁判。
        // 链与 drag_jacobian_fd_check 相同（j1 正向 + j2 反向）。
        let mut j2 = pair_decl("j2", JointKind::Revolute, "l2", "l1", [0.5, 0.0, 0.0]);
        j2.q_init = vec![0.3];
        let mut j1 = pair_decl("j1", JointKind::Revolute, "base", "l1", [0.0; 3]);
        j1.q_init = vec![0.4];
        let decl = GraphDecl {
            links: vec!["base".into(), "l1".into(), "l2".into()],
            pairs: vec![j1, j2],
            anchor: Some("base".into()),
            gears: vec![],
            cams: vec![],
            closures: vec![],
        };
        let kin = solve_kin(&decl);
        let path = link_path_pairs(&kin, "l2").expect("path");
        let free: Vec<usize> = path.clone();
        let x: Vec<f64> = free.iter().map(|&pi| kin.pairs[pi].q[0]).collect();
        let (world0, _) = drag_path_walk(&kin, &path, &free, &x);
        let mut x_off = x.clone();
        x_off[0] += 0.05;
        x_off[1] -= 0.04;
        let (world_off, _) = drag_path_walk(&kin, &path, &free, &x_off);
        for (tag, target, want_zero) in [("r=0", world0, true), ("非零残差", world_off, false)]
        {
            let ja = drag_pose_jacobian(&kin, &path, &free, &x, &target);
            let r0 = drag_pose_residual(&kin, &path, &free, &x, &target);
            if want_zero {
                assert!(r0.iter().all(|v| v.abs() < 1e-12), "r=0 前提: {r0:?}");
            } else {
                assert!(r0.iter().any(|v| v.abs() > 1e-3), "非零残差前提: {r0:?}");
            }
            for k in 0..free.len() {
                let h = 1e-7;
                let mut xp = x.clone();
                xp[k] += h;
                let rp = drag_pose_residual(&kin, &path, &free, &xp, &target);
                for i in 0..6 {
                    let fd = (rp[i] - r0[i]) / h;
                    assert!(
                        (ja[i][k] - fd).abs() < 1e-5 + 1e-5 * fd.abs(),
                        "{tag} drag_pose J[{i}][{k}] 解析 {} vs FD {fd}",
                        ja[i][k]
                    );
                }
            }
        }
    }

    #[test]
    fn drag_pose_rejects_non_rotation_target() {
        // 旋转部分带缩放 → 显式 Err（不拿非正规矩阵当位姿）。
        let kin = solve_kin(&pend_decl("base", "l1"));
        let mut target = mat4_identity();
        target[0] = 2.0;
        let e = drag_pose_solve(&kin, "l1", target, &std::collections::HashSet::new()).unwrap_err();
        assert!(e.contains("不正规"), "{e}");
    }

    #[test]
    fn drag_pose_shares_v1_boundaries() {
        // 自由变量选择与抓点版同一条路径（drag_free_pairs 共用）：gear 拒绝。
        let mut decl = pend_decl("base", "l1");
        decl.links.push("l2".into());
        decl.pairs
            .push(pair_decl("j2", JointKind::Revolute, "base", "l2", [0.0; 3]));
        decl.gears.push(GearDecl {
            a: "j".into(),
            b: "j2".into(),
            ratio: -2.0,
            offset: 0.0,
        });
        let kin = solve_kin(&decl);
        let e = drag_pose_solve(
            &kin,
            "l1",
            mat4_identity(),
            &std::collections::HashSet::new(),
        )
        .unwrap_err();
        assert!(e.contains("可解的自由"), "{e}");
    }
}
