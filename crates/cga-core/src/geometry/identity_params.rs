//! 恒等变换下的几何参数（原在 bake.rs，随烘焙移除后独立保留：
//! 供局部包围盒 / 标签 / 查询使用，不含网格与曲面）。

use crate::affine_geom::AffineParams;
use crate::geometry::*;
use crate::motors::mat3_new;
use crate::multivector::Multivector;

impl Geometry {
    pub fn identity_params(&self) -> GeometryParams {
        let id = Multivector::identity();
        let eye: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let eye3 = mat3_new(eye[0], eye[1], eye[2]);
        match self {
            Geometry::SphereGeometry(g) => {
                let (c, r) = g.blade.to_sphere();
                GeometryParams::SphereParams(SphereParams { c, r, axes: eye })
            }
            Geometry::PlaneGeometry(g) => GeometryParams::PlaneParams(PlaneParams {
                n: g.blade.euclidean_vector(),
                d: g.blade.einf_coeff(),
            }),
            Geometry::CylinderGeometry(g) => GeometryParams::CylinderParams(CylinderParams {
                q: [0.0; 3],
                u: [0.0, 0.0, 1.0],
                r: g.radius,
                h: g.half,
            }),
            Geometry::BoxGeometry(g) => GeometryParams::BoxParams(BoxParams {
                c: [0.0; 3],
                axes: eye,
                half: g.half,
            }),
            Geometry::CircleGeometry(g) => GeometryParams::CircleParams(crate::CircleParams {
                c: [0.0; 3],
                n: [0.0, 0.0, 1.0],
                r: g.radius,
            }),
            Geometry::ConeGeometry(g) => {
                let (ai, ti, af) = id.affine_from_motor(eye3);
                GeometryParams::ConeParams(ConeParams {
                    a_inv3: ai,
                    t_inv: ti,
                    a_fwd: af,
                    r: g.radius,
                    h: g.height,
                })
            }
            Geometry::TorusGeometry(g) => {
                let (ai, ti, af) = id.affine_from_motor(eye3);
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
                let diag = mat3_new([rr[0], 0.0, 0.0], [0.0, rr[1], 0.0], [0.0, 0.0, rr[2]]);
                let (ai, ti, af) = id.affine_from_motor(diag);
                GeometryParams::EllipsoidParams(EllipsoidParams {
                    a_inv3: ai,
                    t_inv: ti,
                    a_fwd: af,
                })
            }
            Geometry::CyclideGeometry(g) => {
                let (ai, ti, af) = id.affine_from_motor(eye3);
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
                children: g.children.iter().map(|c| c.identity_params()).collect(),
            }),
            Geometry::AffineGeometry(g) => {
                let inner = g.inner[0].identity_params();
                let (ai, ti, af) = g.motor.affine_from_motor(g.linear);
                GeometryParams::AffineParams(AffineParams {
                    inner: Box::new(inner),
                    a_inv3: ai,
                    t_inv: ti,
                    a_fwd: af,
                })
            }
        }
    }
}
