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

#[derive(Clone, Debug)]
pub struct JointDef {
    pub name: String,
    pub kind: JointKind,
    pub axis: [f64; 3],
    pub at: [f64; 3],
    pub rpy: [f64; 3],
    pub q: Vec<f64>,
    pub pitch: Option<f64>,
    pub limit: Option<[f64; 2]>,
    pub parent: Option<String>,
    /// Scene mesh indices emitted by this joint's body (link geometry).
    pub meshes: Vec<usize>,
    /// World transform of the child frame: ctx · T(at) · R(rpy) · M(q).
    pub world: [f64; 16],
}

#[derive(Clone, Debug)]
pub struct GearRel {
    pub driver: String,
    pub driven: String,
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
    pub driver: String,
    pub driven: String,
    pub driver_profile: CamProfile,
    pub driven_profile: CamProfile,
}

#[derive(Clone, Debug)]
pub struct CamSolved {
    pub driver: String,
    pub driven: String,
    pub q: f64,
}

/// What a later `joint` statement is driven by (exactly one may exist per name).
#[derive(Clone, Debug)]
pub(crate) enum Driven {
    Gear(GearRel),
    Cam(CamRel),
}

#[derive(Clone, Debug, Default)]
pub struct Kinematics {
    pub joints: Vec<JointDef>,
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
pub(crate) fn cam_solve(
    rel: &CamRel,
    driver: &JointDef,
    driven_ctx: [f64; 16],
    driven_kind: &JointKind,
    driven_axis: [f64; 3],
    driven_at: [f64; 3],
    pitch: f64,
    lo: f64,
    hi: f64,
    line: i32,
) -> Result<f64, String> {
    // Planar-mechanism check: circle normals must be parallel to each other
    // (for a revolute/helical/cylindrical joint the profile normal must be
    // parallel to its own joint axis, else the plane precesses).
    let d_world = driver.world;
    let t2 = |q: f64| {
        mat4_mul(
            mat4_mul(driven_ctx, translate4(driven_at)),
            joint_motion(driven_kind, driven_axis, &[q], pitch),
        )
    };
    let pa = pose_profile(&rel.driver_profile, d_world);
    let pb0 = pose_profile(&rel.driven_profile, t2(0.0));
    let parallel = |u: [f64; 3], v: [f64; 3]| v3_norm(v3_cross(u, v)) < 1e-9;
    let err_axis = || format!("build: cam profile normal must be parallel to its joint axis");
    if !pa.is_plane {
        if matches!(
            driver.kind,
            JointKind::Revolute | JointKind::Helical | JointKind::Cylindrical
        ) {
            // driver axis in world = R(driver.world) · axis. The 3×3 part of
            // driver.world already includes M(q), but M(q) rotates ABOUT the
            // axis, so the axis is invariant either way.
            let aw = {
                let a = driver.axis;
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
            let CamProfile::Circle { n, .. } = &rel.driven_profile else {
                unreachable!()
            };
            parallel(*n, driven_axis)
        };
        if !local_axis_parallel {
            return Err(err_axis());
        }
    }

    let branches = separation_branches(&pa, &pose_profile(&rel.driven_profile, t2(0.0))).len();
    let f =
        |q: f64, k: usize| separation_branches(&pa, &pose_profile(&rel.driven_profile, t2(q)))[k];
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
