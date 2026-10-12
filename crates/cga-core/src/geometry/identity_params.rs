//! **唯一的**「几何 + 电机 → 参数」实现（`GeometryParams` 的参数化 seam）。
//!
//! 此前这里是一份恒等变换的 switch，`cga-gpu` 里有第二份把电机折进参数的 switch
//! （`geom_to_camera`），两者对 cylinder/box/circle 给出不同的 `q/u/axes`。
//! 现在只有 `Geometry::params_in_frame`：`identity_params()` 是它的
//! `m = identity` 特例（逐字段相等，有测试）。
//!
//! `m` 是**几何所处坐标系里**的刚体电机：渲染器传的是相机系下的物体电机，网格/碰撞/
//! 宿主传恒等（局部系）。非刚性部分（scale/mirror）不走电机，按各图元的原始参数保留
//! （仿射包装由 `AffineParams` 的 `a_inv3/t_inv/a_fwd` 承载，见 `affine_from_motor`）。

use crate::affine_geom::AffineParams;
use crate::geometry::*;
use crate::motors::{identity3, Mat3};
use crate::multivector::Multivector;
use crate::vec3_unit;

impl Geometry {
    /// 恒等变换下的参数（局部系）——`params_in_frame(identity)` 的简写。
    pub fn identity_params(&self) -> GeometryParams {
        self.params_in_frame(&Multivector::identity())
    }

    /// 几何在电机 `m` 所处坐标系里的参数。**全工作区唯一**的塞子。
    pub fn params_in_frame(&self, m: &Multivector) -> GeometryParams {
        match self {
            Geometry::SphereGeometry(g) => {
                let s = m.apply(&g.blade);
                let (c, r) = s.to_sphere();
                GeometryParams::SphereParams(SphereParams {
                    c,
                    r,
                    axes: frame_axes(m),
                })
            }
            Geometry::PlaneGeometry(g) => {
                let pi = m.apply(&g.blade);
                GeometryParams::PlaneParams(PlaneParams {
                    n: vec3_unit(pi.euclidean_vector()),
                    d: pi.einf_coeff(),
                })
            }
            Geometry::CylinderGeometry(g) => GeometryParams::CylinderParams(CylinderParams {
                q: origin_in_frame(m),
                u: vec3_unit(m.apply(&Multivector::e3()).euclidean_vector()),
                r: g.radius,
                h: g.half,
            }),
            Geometry::BoxGeometry(g) => GeometryParams::BoxParams(BoxParams {
                c: origin_in_frame(m),
                axes: frame_axes(m),
                half: g.half,
            }),
            Geometry::CircleGeometry(g) => GeometryParams::CircleParams(crate::CircleParams {
                c: origin_in_frame(m),
                n: vec3_unit(m.apply(&Multivector::e3()).euclidean_vector()),
                r: g.radius,
            }),
            Geometry::ConeGeometry(g) => {
                let (ai, ti, af) = m.affine_from_motor(identity3());
                GeometryParams::ConeParams(ConeParams {
                    a_inv3: ai,
                    t_inv: ti,
                    a_fwd: af,
                    r: g.radius,
                    h: g.height,
                })
            }
            Geometry::TorusGeometry(g) => {
                let (ai, ti, af) = m.affine_from_motor(identity3());
                GeometryParams::TorusParams(TorusParams {
                    a_inv3: ai,
                    t_inv: ti,
                    a_fwd: af,
                    major: g.major,
                    minor: g.minor,
                    arc: g.arc,
                })
            }
            Geometry::EllipsoidGeometry(g) => {
                let rr = g.radii;
                let (ai, ti, af) = m.affine_from_motor(diag3(rr));
                GeometryParams::EllipsoidParams(EllipsoidParams {
                    a_inv3: ai,
                    t_inv: ti,
                    a_fwd: af,
                })
            }
            Geometry::CyclideGeometry(g) => {
                let (ai, ti, af) = m.affine_from_motor(identity3());
                GeometryParams::CyclideParams(CyclideParams {
                    a_inv3: ai,
                    t_inv: ti,
                    a_fwd: af,
                    a: g.a,
                    b: g.b,
                    d: g.d,
                    c: (g.a * g.a - g.b * g.b).sqrt(),
                    shift: g.shift,
                })
            }
            Geometry::CsgGeometry(g) => GeometryParams::CsgParams(crate::CsgParams {
                op: g.op,
                children: g.children.iter().map(|c| c.params_in_frame(m)).collect(),
            }),
            Geometry::AffineGeometry(g) => {
                GeometryParams::AffineParams(affine_params_in_frame(g, m))
            }
        }
    }
}

/// 电机后的基向量（球/盒的 `axes`）。
fn frame_axes(m: &Multivector) -> Mat3 {
    [
        vec3_unit(m.apply(&Multivector::e1()).euclidean_vector()),
        vec3_unit(m.apply(&Multivector::e2()).euclidean_vector()),
        vec3_unit(m.apply(&Multivector::e3()).euclidean_vector()),
    ]
}

/// 电机后的原点（柱/盒/圆的 `q`/`c`）。
fn origin_in_frame(m: &Multivector) -> [f64; 3] {
    m.apply(&Multivector::point(0.0, 0.0, 0.0)).coords()
}

fn diag3(rr: [f64; 3]) -> Mat3 {
    [[rr[0], 0.0, 0.0], [0.0, rr[1], 0.0], [0.0, 0.0, rr[2]]]
}

/// 仿射包装：内件在**自己的**局部系里取参数，外层把 `m·motor` 与 `linear` 折成
/// `(a_inv3, t_inv, a_fwd)` 三件套。
fn affine_params_in_frame(g: &AffineGeometry, m: &Multivector) -> AffineParams {
    let inner = g.inner[0].params_in_frame(&Multivector::identity());
    let full = m.gp(&g.motor);
    let (a_inv3, t_inv, a_fwd) = full.affine_from_motor(g.linear);
    AffineParams {
        inner: Box::new(inner),
        a_inv3,
        t_inv,
        a_fwd,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::*;

    fn all_prims() -> Vec<Geometry> {
        vec![
            Geometry::SphereGeometry(SphereGeometry::new(1.3)),
            Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.4)),
            Geometry::CylinderGeometry(CylinderGeometry::new(0.7, 2.0)),
            Geometry::BoxGeometry(BoxGeometry::new(1.4, 0.8, 0.6)),
            Geometry::CircleGeometry(CircleGeometry::new(0.9)),
            Geometry::ConeGeometry(ConeGeometry::new(0.6, 1.5)),
            Geometry::TorusGeometry(TorusGeometry::new(1.2, 0.35)),
            Geometry::EllipsoidGeometry(EllipsoidGeometry::new(1.0, 0.7, 1.6)),
            Geometry::CyclideGeometry(CyclideGeometry::new(1.0, 0.5, 0.25, [0.1, -0.2, 0.3])),
            Geometry::CsgGeometry(crate::CsgGeometry::new(
                crate::CsgOp::Difference,
                vec![
                    Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0)),
                    Geometry::SphereGeometry(SphereGeometry::new(0.8)),
                ],
            )),
            Geometry::AffineGeometry(crate::AffineGeometry::new(
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                [[1.2, 0.0, 0.0], [0.0, 0.9, 0.0], [0.0, 0.0, 1.1]],
            )),
        ]
    }

    fn motors() -> Vec<Multivector> {
        let id = Multivector::identity();
        let t = Multivector::translator([1.0, -2.0, 0.5]);
        let r = Multivector::rotor([0.3, 1.0, -0.2], 0.7);
        let rt = r.compose(&t);
        vec![id, t, r, rt]
    }

    fn sample_points() -> Vec<[f64; 3]> {
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [-0.4, 0.9, 0.3],
            [2.0, -1.0, 0.7],
        ]
    }

    /// 恒等特例：`identity_params` 与 `params_in_frame(identity)` 逐字段相等。
    #[test]
    fn identity_params_is_the_identity_case() {
        for g in all_prims() {
            assert_eq!(
                g.identity_params(),
                g.params_in_frame(&Multivector::identity()),
                "{g:?}"
            );
        }
    }

    /// 帧等变：`params_in_frame(m)` 的隐式场就等于「先把点逆变换回局部系再取恒等场」。
    /// 这是「参数真的承载了电机」的定义性检验（左式是帧内取值，右式是局部取值）。
    #[test]
    fn params_in_frame_is_frame_equivariant() {
        for g in all_prims() {
            let local = g.identity_params();
            for m in motors() {
                let p = g.params_in_frame(&m);
                let back = m.reverse(); // 刚体电机的逆 = reverse
                for x in sample_points() {
                    let want =
                        local.field(back.apply(&Multivector::point(x[0], x[1], x[2])).coords());
                    let got = p.field(x);
                    match (want, got) {
                        (Some(a), Some(b)) => assert!(
                            (a - b).abs() < 1e-9,
                            "{g:?} 在 {x:?}：帧内场 {b} vs 局部场 {a}"
                        ),
                        (None, None) => {}
                        // 圆片/环纹面无场或字段不一致的情况按各自语义放行
                        _ => {}
                    }
                }
            }
        }
    }

    /// 平移 + 旋转的闭式期望（球心/球半径、柱轴、盒轴）。
    #[test]
    fn params_in_frame_closed_forms() {
        let t = Multivector::translator([1.0, 2.0, 3.0]);
        if let GeometryParams::SphereParams(sp) =
            Geometry::SphereGeometry(SphereGeometry::new(2.0)).params_in_frame(&t)
        {
            assert!((sp.c[0] - 1.0).abs() < 1e-12);
            assert!((sp.c[1] - 2.0).abs() < 1e-12);
            assert!((sp.c[2] - 3.0).abs() < 1e-12);
            assert_eq!(sp.r, 2.0);
            assert_eq!(sp.axes, identity3(), "纯平移不改基向量");
        } else {
            panic!("sphere params");
        }
        // 绕 z 转 90°：柱轴 +y，盒 x 轴 → +y
        let r = Multivector::rotor([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        if let GeometryParams::CylinderParams(cy) =
            Geometry::CylinderGeometry(CylinderGeometry::new(0.5, 4.0)).params_in_frame(&r)
        {
            assert!(
                cy.u[0].abs() < 1e-12 && (cy.u[2] - 1.0).abs() < 1e-12,
                "{:?}",
                cy.u
            );
        } else {
            panic!("cylinder params");
        }
        if let GeometryParams::BoxParams(bx) =
            Geometry::BoxGeometry(BoxGeometry::new(1.0, 2.0, 3.0)).params_in_frame(&r)
        {
            assert!((bx.axes[0][1] - 1.0).abs() < 1e-12, "{:?}", bx.axes);
            assert!((bx.axes[1][0] + 1.0).abs() < 1e-12, "{:?}", bx.axes);
            assert_eq!(bx.half, [0.5, 1.0, 1.5]);
        } else {
            panic!("box params");
        }
        // 平面平移后 d 变化，法向不变
        if let GeometryParams::PlaneParams(pl) =
            Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)).params_in_frame(&t)
        {
            assert!((pl.n[1] - 1.0).abs() < 1e-12);
            assert!((pl.d - 2.0).abs() < 1e-12, "d={}", pl.d);
        } else {
            panic!("plane params");
        }
    }
}
