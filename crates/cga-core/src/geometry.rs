use crate::affine_geom::{AffineGeometry, AffineParams};
use crate::csg_node::{CsgGeometry, CsgParams};
use crate::primitives::Cylinder;
use crate::{Mat3, Multivector};

/// 场景几何：所有几何 kernel 的封闭枚举（含 CSG 节点与仿射包装）。
#[derive(Clone, Debug)]
pub enum Geometry {
    AffineGeometry(AffineGeometry),
    CsgGeometry(CsgGeometry),
    ConeGeometry(ConeGeometry),
    CyclideGeometry(CyclideGeometry),
    EllipsoidGeometry(EllipsoidGeometry),
    TorusGeometry(TorusGeometry),
    SphereGeometry(SphereGeometry),
    PlaneGeometry(PlaneGeometry),
    CylinderGeometry(CylinderGeometry),
    BoxGeometry(BoxGeometry),
    CircleGeometry(CircleGeometry),
}

pub mod params;
pub use self::params::*;
pub mod sphere;
pub use self::sphere::*;
pub mod plane;
pub use self::plane::*;
pub mod cylinder;
pub use self::cylinder::*;
pub mod r#box;
pub use self::r#box::*;
pub mod circle;
pub use self::circle::*;
pub mod cone;
pub use self::cone::*;
pub mod torus;
pub use self::torus::*;
pub mod ellipsoid;
pub use self::ellipsoid::*;
pub mod cyclide;
pub use self::cyclide::*;
pub mod identity_params;
