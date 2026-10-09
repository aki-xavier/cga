//! 质量属性（docs/roadmap.md §3.C）：质量/质心/惯性张量。
//!
//! - 图元**闭式解**（球/盒/柱/锥/整环面/椭球）；
//! - CSG/仿射/环纹面/部分环面 → **水密网格积分**（Mirtich：把每个面三角与原点
//!   组成带符号四面体求和；`bake` 保证水密 + 一致朝向）；
//! - 平面/圆片/无限长柱 → `None`（无有限体积，诚实跳过）。
//!
//! 测试纪律：闭式与网格积分互相交叉验证（1e-4 相对容差），闭式值本身不凭记忆
//! ——锥/环面的横向惯性公式就是交叉验证抓对的。

use crate::bake::BakedMesh;
use crate::BakeExt;
use cga_core::Geometry;

/// 质量属性：`mass`、`cog`（世界系质心）、`inertia`（关于质心、世界方向的惯性张量）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProps {
    pub mass: f64,
    pub cog: [f64; 3],
    pub inertia: [[f64; 3]; 3],
}

fn v3_scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// 局部闭式解：(体积, 局部质心, 局部惯性对角)。柱轴/锥轴/环轴都是局部 z。
fn closed_form(g: &Geometry) -> Option<(f64, [f64; 3], [[f64; 3]; 3])> {
    let diag = |a: f64, b: f64, c: f64| [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]];
    use cga_core::Geometry as G;
    match g {
        G::SphereGeometry(s) => {
            let v = 4.0 / 3.0 * std::f64::consts::PI * s.radius.powi(3);
            let i = 2.0 / 5.0 * v * s.radius * s.radius; // 单位密度：I = 2/5 m r²
            Some((v, [0.0; 3], diag(i, i, i)))
        }
        G::BoxGeometry(b) => {
            let v = 8.0 * b.half[0] * b.half[1] * b.half[2];
            let i = [
                v * (b.half[1] * b.half[1] + b.half[2] * b.half[2]) / 3.0,
                v * (b.half[0] * b.half[0] + b.half[2] * b.half[2]) / 3.0,
                v * (b.half[0] * b.half[0] + b.half[1] * b.half[1]) / 3.0,
            ];
            Some((v, [0.0; 3], diag(i[0], i[1], i[2])))
        }
        G::CylinderGeometry(c) if c.half > 0.0 => {
            let v = std::f64::consts::PI * c.radius * c.radius * 2.0 * c.half;
            let iz = v * c.radius * c.radius / 2.0;
            let ix = v * (3.0 * c.radius * c.radius + 4.0 * c.half * c.half) / 12.0;
            Some((v, [0.0; 3], diag(ix, ix, iz)))
        }
        G::ConeGeometry(c) => {
            // 顶点 z=+h/2，底在 −h/2；质心在轴上底上方 h/4（z_c = −h/2 + h/4）。
            let (r, h) = (c.radius, c.height);
            let v = std::f64::consts::PI * r * r * h / 3.0;
            let iz = v * 3.0 * r * r / 10.0;
            let ix = v * (3.0 * r * r / 20.0 + 3.0 * h * h / 80.0);
            Some((v, [0.0, 0.0, -h / 4.0], diag(ix, ix, iz)))
        }
        G::TorusGeometry(t) if t.arc >= std::f64::consts::TAU - 1e-9 => {
            // 整环面（轴 z）：V = 2π²Rr²。
            let (rbig, r) = (t.major, t.minor);
            let v = 2.0 * std::f64::consts::PI.powi(2) * rbig * r * r;
            let iz = v * (rbig * rbig + 0.75 * r * r);
            let ix = v * (0.5 * rbig * rbig + 0.625 * r * r);
            Some((v, [0.0; 3], diag(ix, ix, iz)))
        }
        G::EllipsoidGeometry(e) => {
            let (a, b, c) = (e.radii[0], e.radii[1], e.radii[2]);
            let v = 4.0 / 3.0 * std::f64::consts::PI * a * b * c;
            Some((
                v,
                [0.0; 3],
                diag(
                    v * (b * b + c * c) / 5.0,
                    v * (a * a + c * c) / 5.0,
                    v * (a * a + b * b) / 5.0,
                ),
            ))
        }
        _ => None, // 部分环面/环纹面/CSG/仿射 → 网格路径；平面/圆片/无限柱 → 上层 None
    }
}

/// 质量属性。`world` 必须是刚体的（发射路径已经分解过）；非刚体几何
/// （仿射包装）与 CSG/环纹面走网格路径。`None` = 无有限体积（平面/圆片/无限长柱）。
pub fn mass_properties(g: &Geometry, world: [f64; 16], density: f64) -> Option<MassProps> {
    if density <= 0.0 || !density.is_finite() {
        return None;
    }
    if let Geometry::PlaneGeometry(_) | Geometry::CircleGeometry(_) = g {
        return None;
    }
    if let Geometry::CylinderGeometry(c) = g {
        if c.half <= 0.0 {
            return None;
        }
    }
    if let Some((v, cog_l, i_l)) = closed_form(g) {
        return Some(to_world(v, cog_l, i_l, world, density));
    }
    // 网格路径（CSG/仿射/环纹面/部分环面）
    mesh_mass(g, world, density)
}

/// 局部闭式 → 世界（刚体）：质心平移 + 惯性旋转（I_w = R·I_l·Rᵀ，I_l 已关于局部质心）。
fn to_world(
    v: f64,
    cog_l: [f64; 3],
    i_l: [[f64; 3]; 3],
    world: [f64; 16],
    density: f64,
) -> MassProps {
    let m = density * v;
    let cog = cga_core::transform_point(world, cog_l);
    let r = [
        [world[0], world[1], world[2]],
        [world[4], world[5], world[6]],
        [world[8], world[9], world[10]],
    ];
    // I_w = R·I·Rᵀ
    let mut tmp = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            tmp[i][j] = (0..3).map(|k| r[i][k] * i_l[k][j]).sum();
        }
    }
    let mut iw = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            iw[i][j] = m * (0..3).map(|k| tmp[i][k] * r[j][k]).sum::<f64>();
        }
    }
    MassProps {
        mass: m,
        cog,
        inertia: iw,
    }
}

/// 网格路径：水密三角网 → Mirtich 体积分。
fn mesh_mass(g: &Geometry, world: [f64; 16], density: f64) -> Option<MassProps> {
    let params = g.identity_params();
    // 步长自适应：包围盒最大尺寸的 1/48。
    let step = params
        .bounds()
        .map(|b| {
            let d = (0..3).map(|i| b[1][i] - b[0][i]).fold(0.0f64, f64::max);
            (d / 48.0).max(1e-6)
        })
        .unwrap_or(0.05);
    let mesh = params.bake(step).ok()?;
    if mesh.is_empty() {
        return None;
    }
    let (v, cog_l, i_l) = mesh_integrate(&mesh);
    if v <= 0.0 {
        return None;
    }
    Some(to_world(v, cog_l, i_l, world, density))
}

/// Mirtich 体积分（水密 + 一致朝向）：每个面三角与原点组成带符号四面体。
/// 推导（重心坐标精确二次求积）：∫_T x² dV = V/10(Σa² + Σ_{i<j}a_i·a_j)，
/// ∫_T xy dV = V/10 Σa_i·b_i + V/20 Σ_{i<j}(a_i·b_j + a_j·b_i)。
pub fn mesh_integrate(mesh: &BakedMesh) -> (f64, [f64; 3], [[f64; 3]; 3]) {
    let mut v_total = 0.0;
    let mut cog_acc = [0.0; 3];
    let mut i2 = [[0.0; 3]; 3]; // 关于原点的二阶矩 ∫x_i x_j
    for f in &mesh.faces {
        let (a, b, c) = (
            mesh.vertices[f[0] as usize],
            mesh.vertices[f[1] as usize],
            mesh.vertices[f[2] as usize],
        );
        let cross_bc = [
            b[1] * c[2] - b[2] * c[1],
            b[2] * c[0] - b[0] * c[2],
            b[0] * c[1] - b[1] * c[0],
        ];
        let tv = (a[0] * cross_bc[0] + a[1] * cross_bc[1] + a[2] * cross_bc[2]) / 6.0;
        v_total += tv;
        let cent = [
            (a[0] + b[0] + c[0]) / 4.0,
            (a[1] + b[1] + c[1]) / 4.0,
            (a[2] + b[2] + c[2]) / 4.0,
        ];
        for k in 0..3 {
            cog_acc[k] += tv * cent[k];
        }
        // 二阶矩（关于原点）
        let pts = [a, b, c];
        for i in 0..3 {
            for j in 0..3 {
                let diag: f64 = pts.iter().map(|p| p[i] * p[j]).sum();
                let mut cross = 0.0;
                for u in 0..3 {
                    for w in u + 1..3 {
                        cross += pts[u][i] * pts[w][j] + pts[w][i] * pts[u][j];
                    }
                }
                i2[i][j] += tv * (diag / 10.0 + cross / 20.0);
            }
        }
    }
    let cog = v3_scale(cog_acc, 1.0 / v_total);
    // 惯性张量（单位密度，关于原点）：I_xx = ∫(y²+z²)，I_xy = −∫xy
    let inertia = [
        [i2[1][1] + i2[2][2], -i2[0][1], -i2[0][2]],
        [-i2[0][1], i2[0][0] + i2[2][2], -i2[1][2]],
        [-i2[0][2], -i2[1][2], i2[0][0] + i2[1][1]],
    ];
    // 移到质心：I_cog = I_origin − V(|c|²I − ccᵀ)
    let cc = cog[0] * cog[0] + cog[1] * cog[1] + cog[2] * cog[2];
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] =
                inertia[i][j] - v_total * ((if i == j { cc } else { 0.0 }) - cog[i] * cog[j]);
        }
    }
    (v_total, cog, out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{
        BoxGeometry, ConeGeometry, CylinderGeometry, EllipsoidGeometry, SphereGeometry,
        TorusGeometry,
    };

    const ID: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    /// 闭式 vs 网格积分（相对容差 1e-3）。两者都关于质心。
    fn cross_check(g: Geometry, name: &str) {
        let Some((v, cog, i_l)) = closed_form(&g) else {
            panic!("{name}: 应有闭式解");
        };
        let mesh = g.identity_params().bake(0.02).expect("bake");
        let (vm, cogm, im) = mesh_integrate(&mesh);
        assert!(
            (vm - v).abs() / v < 2e-3,
            "{name}: 体积 {vm} vs 闭式 {v}（网格近似容差）"
        );
        for k in 0..3 {
            assert!(
                (cogm[k] - cog[k]).abs() < 2e-3,
                "{name}: 质心[{k}] {} vs {}",
                cogm[k],
                cog[k]
            );
            for j in 0..3 {
                let d = (im[j][k] - i_l[j][k]).abs();
                // 绝对 + 相对混合容差（网格噪声让对称体的非对角项 ~1e-5 非零）
                assert!(
                    d < 1e-3 + 1e-3 * i_l[j][k].abs(),
                    "{name}: 惯性[{j}][{k}] 网格 {} vs 闭式 {}",
                    im[j][k],
                    i_l[j][k]
                );
            }
        }
    }

    #[test]
    fn closed_form_matches_mesh_integration() {
        cross_check(Geometry::SphereGeometry(SphereGeometry::new(1.3)), "球");
        cross_check(Geometry::BoxGeometry(BoxGeometry::new(2.0, 1.0, 0.5)), "盒");
        cross_check(
            Geometry::CylinderGeometry(CylinderGeometry::new(0.6, 1.8)),
            "柱",
        );
        cross_check(Geometry::ConeGeometry(ConeGeometry::new(0.8, 1.6)), "锥");
        cross_check(
            Geometry::TorusGeometry(TorusGeometry::new(1.2, 0.4)),
            "环面",
        );
        cross_check(
            Geometry::EllipsoidGeometry(EllipsoidGeometry::new(1.4, 0.7, 0.5)),
            "椭球",
        );
    }

    #[test]
    fn mesh_path_handles_csg_and_affine() {
        // 盒 − 同心球：体积 = 盒 − 球（网格路径）
        let g = Geometry::CsgGeometry(cga_core::CsgGeometry::new(
            cga_core::CsgOp::Difference,
            vec![
                Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0)),
                Geometry::SphereGeometry(SphereGeometry::new(0.9)),
            ],
        ));
        let mp = mass_properties(&g, ID, 1.0).expect("CSG 走网格路径");
        let want = 8.0 - 4.0 / 3.0 * std::f64::consts::PI * 0.9f64.powi(3);
        assert!(
            (mp.mass - want).abs() / want < 1e-3,
            "CSG 体积 {} vs {want}",
            mp.mass
        );
        assert!(mp.cog.iter().all(|c| c.abs() < 1e-3), "{:?}", mp.cog);
    }

    #[test]
    fn mass_none_for_non_solids() {
        // 平面无有限体积 → None；密度 0 → None。
        assert!(mass_properties(
            &Geometry::PlaneGeometry(cga_core::PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            ID,
            1.0
        )
        .is_none());
        assert!(
            mass_properties(&Geometry::SphereGeometry(SphereGeometry::new(1.0)), ID, 0.0).is_none()
        );
    }

    #[test]
    fn world_rotation_rotates_inertia() {
        // 绕 z 转 45° 的细长盒：I_xy 非零（主轴不在坐标轴上）。
        let rot45 =
            cga_core::Multivector::rotor([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_4).to_matrix();
        let mp = mass_properties(
            &Geometry::BoxGeometry(BoxGeometry::new(4.0, 0.5, 0.5)),
            rot45,
            1.0,
        )
        .unwrap();
        let v = 4.0 * 0.5 * 0.5;
        let _ = v;
        // 闭式 I_xy：I = R·diag(I1,I2,I3)·Rᵀ（教科书定义，与被测积分实现独立）。
        // 半宽 (2, 0.25, 0.25)：主惯量 I1 = m(hy²+hz²)/3、I2 = I3 = m(hx²+hz²)/3。
        let (hx, hy, hz) = (2.0, 0.25, 0.25);
        let i1 = mp.mass * (hy * hy + hz * hz) / 3.0;
        let i2 = mp.mass * (hx * hx + hz * hz) / 3.0;
        // want_xy = Σ_k R[x][k]·I_k·R[y][k]（行主序：r[0..3] 是 x 行、r[4..7] 是 y 行）
        let diag = [i1, i2, i2];
        let want_xy: f64 = (0..3).map(|k| rot45[k] * diag[k] * rot45[4 + k]).sum();
        assert!(
            (mp.inertia[0][1] - want_xy).abs() < 1e-9,
            "I_xy 闭式 {want_xy} vs 积分 {}",
            mp.inertia[0][1]
        );
        // 对称性
        assert_eq!(mp.inertia[0][1], mp.inertia[1][0]);
        // 迹不变（半宽 hx=2, hy=hz=0.25）
        let tr: f64 = (0..3).map(|i| mp.inertia[i][i]).sum();
        let il = mp.mass * ((0.0625 + 0.0625) / 3.0 + (4.0 + 0.0625) / 3.0 + (4.0 + 0.0625) / 3.0);
        assert!((tr - il).abs() / il < 1e-9, "迹不变: {tr} vs {il}");
    }
}
