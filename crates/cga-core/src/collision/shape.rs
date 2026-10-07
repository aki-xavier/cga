//! 世界空间形状参数：分离距离的求解域。只接受刚体（含镜像）变换——非均匀
//! 缩放会改变欧氏距离，给不出诚实的分离值（返回 None ⇒ Unknown）。

use super::*;
use crate::geometry::Geometry;

/// 世界空间形状（刚体变换后的参数）。
#[derive(Clone, Copy, Debug)]
pub(crate) enum Shape {
    Sphere {
        c: [f64; 3],
        r: f64,
    },
    /// 半空间实体：n·x ≤ d 为内部（n 单位）。
    Plane {
        n: [f64; 3],
        d: f64,
    },
    Box {
        c: [f64; 3],
        axes: [[f64; 3]; 3],
        half: [f64; 3],
    },
    /// 轴 u（单位）、中心 q、半径 r；half ≤ 0 表示无限长（只支持与球/平面的配对，
    /// 且平面配对要求有限长）。
    Cylinder {
        q: [f64; 3],
        u: [f64; 3],
        r: f64,
        half: f64,
    },
    /// 顶点在 c + axis·h/2，底圆半径 r 在 c − axis·h/2。
    Cone {
        c: [f64; 3],
        axis: [f64; 3],
        r: f64,
        h: f64,
    },
    /// 完整环面（arc = 2π；部分环面 v1 不支持）。
    Torus {
        c: [f64; 3],
        axis: [f64; 3],
        major: f64,
        minor: f64,
    },
    Ellipsoid {
        c: [f64; 3],
        axes: [[f64; 3]; 3],
        radii: [f64; 3],
    },
}

/// 刚体（含镜像）判定：3×3 部分的列向量单位且两两正交。
fn is_rigid(m: [f64; 16]) -> bool {
    let col = |j: usize| [m[j], m[4 + j], m[8 + j]];
    let (c0, c1, c2) = (col(0), col(1), col(2));
    let unit_ok = [c0, c1, c2].iter().all(|c| (norm(*c) - 1.0).abs() < 1e-9);
    unit_ok && dot(c0, c1).abs() < 1e-9 && dot(c0, c2).abs() < 1e-9 && dot(c1, c2).abs() < 1e-9
}

/// (几何, 世界变换) → 世界空间形状。`None` = CSG / 圆片 / 环纹面 / 部分环面 /
/// 非刚体仿射（分离距离对这些一律 Unknown）。
pub(crate) fn to_shape(g: &Geometry, world: [f64; 16]) -> Option<Shape> {
    let m = match g {
        Geometry::AffineGeometry(a) => {
            if a.inner.len() != 1 {
                return None;
            }
            let m2 = mat4_mul(world, mat4_mul(a.motor.to_matrix(), mat3_to_mat4(a.linear)));
            return to_shape(&a.inner[0], m2);
        }
        Geometry::CsgGeometry(_) => return None,
        _ => world,
    };
    if !is_rigid(m) {
        return None;
    }
    let e3 = |m: [f64; 16]| unit(xform_dir(m, [0.0, 0.0, 1.0]));
    let axes = |m: [f64; 16]| {
        [
            unit(xform_dir(m, [1.0, 0.0, 0.0])),
            unit(xform_dir(m, [0.0, 1.0, 0.0])),
            unit(xform_dir(m, [0.0, 0.0, 1.0])),
        ]
    };
    match g {
        Geometry::SphereGeometry(s) => Some(Shape::Sphere {
            c: transform_point(m, [0.0; 3]),
            r: s.radius,
        }),
        Geometry::PlaneGeometry(p) => {
            let n = p.blade.euclidean_vector();
            let nl = norm(n);
            let (n, d) = (scale(n, 1.0 / nl), p.blade.einf_coeff() / nl);
            let p0 = transform_point(m, scale(n, d));
            let nw = unit(xform_dir(m, n));
            Some(Shape::Plane {
                n: nw,
                d: dot(nw, p0),
            })
        }
        Geometry::BoxGeometry(b) => Some(Shape::Box {
            c: transform_point(m, [0.0; 3]),
            axes: axes(m),
            half: b.half,
        }),
        Geometry::CylinderGeometry(c) => Some(Shape::Cylinder {
            q: transform_point(m, [0.0; 3]),
            u: e3(m),
            r: c.radius,
            half: c.half,
        }),
        Geometry::ConeGeometry(c) => Some(Shape::Cone {
            c: transform_point(m, [0.0; 3]),
            axis: e3(m),
            r: c.radius,
            h: c.height,
        }),
        Geometry::TorusGeometry(t) => {
            if t.arc < std::f64::consts::TAU - 1e-9 {
                return None;
            }
            Some(Shape::Torus {
                c: transform_point(m, [0.0; 3]),
                axis: e3(m),
                major: t.major,
                minor: t.minor,
            })
        }
        Geometry::EllipsoidGeometry(e) => Some(Shape::Ellipsoid {
            c: transform_point(m, [0.0; 3]),
            axes: axes(m),
            radii: e.radii,
        }),
        Geometry::CircleGeometry(_) | Geometry::CyclideGeometry(_) => None,
        Geometry::CsgGeometry(_) | Geometry::AffineGeometry(_) => unreachable!(),
    }
}
