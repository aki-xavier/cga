//! 碰撞检测（计划见 `docs/collision-plan.md`，C0：点包含 + 凸对分离距离 + 三值重叠）。
//!
//! 约定：
//! - **平面是半空间实体**：`n·x ≤ d` 为内部（地面 `n=+y, d=0` ⇒ 大地在 y≤0 一侧）。
//!   球落到地面下方 = 穿入，符合"地面是实体"的直觉。
//! - **分离距离**：相离时为表面最近距离，穿入时为负（凸对给出最小平移深度），
//!   相切 = 0。`overlap` 视相切为接触（`Yes`）。
//! - **三值诚实**（D2）：`Hit::Unknown` 只在给不出确切答案时出现——CSG 差/交的
//!   重叠、非刚体仿射下的分离、部分环面（arc < 2π）、圆片/环纹面参与的对、
//!   以及未实现的两两组合。绝不假装精确。
//! - 圆片是曲面不是实体：`contains_point` → `Unknown`；分离只对球有定义。
//! - 窄相是纯 CPU f64 标量数学（D3），与渲染器的 MLX 批处理解耦。

use cga_core::affine::{mat3_inv, mat3_to_mat4};
use cga_core::geometry::Geometry;
use cga_core::mat4::{mat4_mul, transform_point};

mod contacts;
mod pairs;
mod shape;
mod sweep;

pub use contacts::contacts;
pub use sweep::sweep_toi;

/// 三值判定（D2）：确切命中 / 确切不命中 / 认证路径给不出确切答案。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hit {
    Yes,
    No,
    Unknown,
}

impl Hit {
    fn and(self, o: Hit) -> Hit {
        match (self, o) {
            (Hit::No, _) | (_, Hit::No) => Hit::No,
            (Hit::Yes, Hit::Yes) => Hit::Yes,
            _ => Hit::Unknown,
        }
    }
    fn or(self, o: Hit) -> Hit {
        match (self, o) {
            (Hit::Yes, _) | (_, Hit::Yes) => Hit::Yes,
            (Hit::No, Hit::No) => Hit::No,
            _ => Hit::Unknown,
        }
    }
    fn not(self) -> Hit {
        match self {
            Hit::Yes => Hit::No,
            Hit::No => Hit::Yes,
            Hit::Unknown => Hit::Unknown,
        }
    }
}

/// 一个接触：点、法向（从 b 指向 a）、带符号分离度（< 0 = 穿入）。C2 填充。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub point: [f64; 3],
    pub normal: [f64; 3],
    pub separation: f64,
}

// ---- 小向量工具（本模块内的全部计算都是 CPU f64 标量） ----
//
// 定义全在 cga-core（canonical，唯一一份）；本模块的几何公式很密（`sub`/`add`/
// `norm` 共 130+ 处），故以短别名引入，而不是把调用点全改成 `vec3_*`——纯粹为
// 可读性。`unit` 保留为本地派生助手：它没有近零兜底（除以零得 inf），与
// `cga_scene::vec3_unit` 的 `[0,0,1]` 兜底不是同一语义。

pub(crate) use cga_core::{
    transform_dir, vec3_add as add, vec3_cross, vec3_dot, vec3_len as norm, vec3_scale,
    vec3_sub as sub,
};

pub(crate) fn unit(a: [f64; 3]) -> [f64; 3] {
    vec3_scale(a, 1.0 / norm(a))
}

/// 仿射 4×4 的逆（decompose_rigid 的极分解 + mat3_inv；奇异线性部分会 panic，
/// 与渲染管线的行为一致）。
pub(crate) fn inv_affine4(m: [f64; 16]) -> [f64; 16] {
    let (motor, lin) = cga_core::affine::decompose_rigid(m);
    mat4_mul(mat3_to_mat4(mat3_inv(lin)), motor.reverse().to_matrix())
}

// ---- 点包含 -------------------------------------------------------------

/// 点是否在实体内（三值）。对任意仿射变换精确（把点逆变换到局部空间），
/// CSG 按三值逻辑递归。
pub fn contains_point(g: &Geometry, world: [f64; 16], p: [f64; 3]) -> Hit {
    match g {
        Geometry::CsgGeometry(c) => {
            let kids = c
                .children
                .iter()
                .map(|k| contains_point(k, world, p))
                .collect::<Vec<_>>();
            match c.op {
                cga_core::csg_node::CsgOp::Union => {
                    kids.into_iter().reduce(Hit::or).unwrap_or(Hit::No)
                }
                cga_core::csg_node::CsgOp::Intersection => {
                    kids.into_iter().reduce(Hit::and).unwrap_or(Hit::Unknown)
                }
                cga_core::csg_node::CsgOp::Difference => kids
                    .into_iter()
                    .reduce(|acc, h| acc.and(h.not()))
                    .unwrap_or(Hit::Unknown),
            }
        }
        Geometry::AffineGeometry(a) => {
            if a.inner.len() != 1 {
                return Hit::Unknown; // 多内件的仿射包装：v1 不支持
            }
            let m = mat4_mul(world, mat4_mul(a.motor.to_matrix(), mat3_to_mat4(a.linear)));
            contains_point(&a.inner[0], m, p)
        }
        prim => {
            // 图元内外归属的唯一实现在 `cga_core::query`（三值：不可判定 → Unknown）。
            let pl = transform_point(inv_affine4(world), p);
            match prim.identity_params().contains(pl) {
                cga_core::Membership::Inside => Hit::Yes,
                cga_core::Membership::Outside => Hit::No,
                cga_core::Membership::Unknown => Hit::Unknown,
            }
        }
    }
}

// ---- 分离距离与重叠 ------------------------------------------------------

/// 两个实体的分离距离：相离为正（表面最近距离），穿入为负（凸对 = 最小平移
/// 深度），相切为 0。CSG union 分解为子件最小值（全部可算才给）。
/// `None` = Unknown（未实现的对 / 非刚体仿射 / CSG 差与交 / 部分环面 /
/// 圆片 / 环纹面 / 无限长柱与非球非平面的对）。
pub fn separation(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> Option<f64> {
    // CSG union：到并集的距离 = 到各子件距离的最小值（有子件算不出则整体 Unknown）。
    if let Geometry::CsgGeometry(c) = a {
        if c.op == cga_core::csg_node::CsgOp::Union {
            let mut best: Option<f64> = Some(f64::INFINITY);
            for k in &c.children {
                best = match (best, separation(k, wa, b, wb)) {
                    (Some(x), Some(d)) => Some(x.min(d)),
                    _ => None,
                };
            }
            return best;
        }
    }
    if let Geometry::CsgGeometry(c) = b {
        if c.op == cga_core::csg_node::CsgOp::Union {
            return separation(b, wb, a, wa);
        }
    }
    let sa = shape::to_shape(a, wa)?;
    let sb = shape::to_shape(b, wb)?;
    pairs::pair_separation(&sa, &sb)
}

/// 两个实体是否重叠（三值）。见 [`probe`]。
pub fn overlap(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> Hit {
    probe(a, wa, b, wb).0
}

/// `overlap` + `separation` 一次算（避免重复求值）。
///
/// 判定阶梯（保守，逐步降级）：
/// 1. 世界包围盒相离 ⇒ 确切 `No`（分离距离照算，不受 broad phase 影响）；
/// 2. 两两分离距离表能算 ⇒ 按符号给 `Yes`/`No`；
/// 3. CSG union 分解：`overlap(∪ᵢ aᵢ, b) = orᵢ overlap(aᵢ, b)`（三值 or；
///    分离距离 = 子件最小值，全可算才给）；
/// 4. 差/交与其余 ⇒ `Unknown`。
pub fn probe(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> (Hit, Option<f64>) {
    if let (Some(x), Some(y)) = (world_aabb(a, wa), world_aabb(b, wb)) {
        if !aabb_overlap(x, y) {
            return (Hit::No, separation(a, wa, b, wb));
        }
    }
    if let Some(d) = separation(a, wa, b, wb) {
        let hit = if d <= 0.0 { Hit::Yes } else { Hit::No };
        return (hit, Some(d));
    }
    if let Geometry::CsgGeometry(c) = a {
        if c.op == cga_core::csg_node::CsgOp::Union {
            return probe_union(c, wa, b, wb);
        }
    }
    if let Geometry::CsgGeometry(c) = b {
        if c.op == cga_core::csg_node::CsgOp::Union {
            let (h, s) = probe_union(c, wb, a, wa);
            return (h, s);
        }
    }
    (Hit::Unknown, None)
}

fn probe_union(
    c: &cga_core::CsgGeometry,
    wc: [f64; 16],
    other: &Geometry,
    wo: [f64; 16],
) -> (Hit, Option<f64>) {
    let mut hit = Hit::No;
    let mut seps: Option<f64> = Some(f64::INFINITY);
    for k in &c.children {
        let (h, s) = probe(k, wc, other, wo);
        hit = hit.or(h);
        seps = match (seps, s) {
            (Some(acc), Some(d)) => Some(acc.min(d)),
            _ => None,
        };
    }
    (hit, seps)
}

/// 世界空间 AABB（保守：部分环面按完整环面取，圆片取零厚度板）。
/// `None` = 无界（平面、无限长柱）或 v1 不支持的环纹面。
pub fn world_aabb(g: &Geometry, world: [f64; 16]) -> Option<[[f64; 3]; 2]> {
    match g {
        Geometry::AffineGeometry(a) => {
            if a.inner.len() != 1 {
                return None;
            }
            let m = mat4_mul(world, mat4_mul(a.motor.to_matrix(), mat3_to_mat4(a.linear)));
            world_aabb(&a.inner[0], m)
        }
        Geometry::CsgGeometry(c) => match c.op {
            cga_core::csg_node::CsgOp::Union => {
                let mut out: Option<[[f64; 3]; 2]> = None;
                for k in &c.children {
                    let b = world_aabb(k, world)?;
                    out = Some(match out {
                        None => b,
                        Some(o) => [
                            [
                                o[0][0].min(b[0][0]),
                                o[0][1].min(b[0][1]),
                                o[0][2].min(b[0][2]),
                            ],
                            [
                                o[1][0].max(b[1][0]),
                                o[1][1].max(b[1][1]),
                                o[1][2].max(b[1][2]),
                            ],
                        ],
                    });
                }
                out
            }
            cga_core::csg_node::CsgOp::Intersection => {
                let mut out: Option<[[f64; 3]; 2]> = None;
                for k in &c.children {
                    let b = world_aabb(k, world)?;
                    out = Some(match out {
                        None => b,
                        Some(o) => [
                            [
                                o[0][0].max(b[0][0]),
                                o[0][1].max(b[0][1]),
                                o[0][2].max(b[0][2]),
                            ],
                            [
                                o[1][0].min(b[1][0]),
                                o[1][1].min(b[1][1]),
                                o[1][2].min(b[1][2]),
                            ],
                        ],
                    });
                }
                out
            }
            cga_core::csg_node::CsgOp::Difference => world_aabb(&c.children[0], world),
        },
        prim => {
            let b = local_bounds(prim)?;
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for c in 0..8 {
                let p = [
                    if c & 1 == 0 { b[0][0] } else { b[1][0] },
                    if c & 2 == 0 { b[0][1] } else { b[1][1] },
                    if c & 4 == 0 { b[0][2] } else { b[1][2] },
                ];
                let w = transform_point(world, p);
                for i in 0..3 {
                    lo[i] = lo[i].min(w[i]);
                    hi[i] = hi[i].max(w[i]);
                }
            }
            Some([lo, hi])
        }
    }
}

/// 局部包围盒（碰撞侧的口径）：图元的几何包围盒来自 `cga_core::query`
/// （唯一实现），只有两处是**碰撞的保守策略**而非几何知识：
/// 圆片不是实体（网格化/`bounds` 给 `None`）但碰撞要一个零厚度板；
/// 环纹面的保守包围盒在碰撞侧不给（走 `Unknown` 路径）。
fn local_bounds(g: &Geometry) -> Option<[[f64; 3]; 2]> {
    match g {
        Geometry::CircleGeometry(c) => {
            Some([[-c.radius, -c.radius, 0.0], [c.radius, c.radius, 0.0]])
        }
        Geometry::CyclideGeometry(_) => None,
        Geometry::CsgGeometry(_) | Geometry::AffineGeometry(_) => unreachable!(),
        prim => prim.identity_params().bounds(),
    }
}

fn aabb_overlap(a: [[f64; 3]; 2], b: [[f64; 3]; 2]) -> bool {
    (0..3).all(|i| a[0][i] <= b[1][i] && b[0][i] <= a[1][i])
}

#[cfg(test)]
mod tests;
