use crate::affine_geom::{AffineGeometry, AffineParams};
use crate::csg_node::{CsgGeometry, CsgParams};
use crate::primitives::Cylinder;
use crate::{Mat3, Multivector};

pub mod sphere_params;
pub use self::sphere_params::*;
pub mod plane_params;
pub use self::plane_params::*;
pub mod cylinder_params;
pub use self::cylinder_params::*;
pub mod box_params;
pub use self::box_params::*;
pub mod circle_params;
pub use self::circle_params::*;
pub mod cone_params;
pub use self::cone_params::*;
pub mod torus_params;
pub use self::torus_params::*;
pub mod ellipsoid_params;
pub use self::ellipsoid_params::*;
pub mod cyclide_params;
pub use self::cyclide_params::*;
pub mod trimesh_params;
pub use self::trimesh_params::*;
pub mod geometry_params;
pub use self::geometry_params::*;
pub mod sphere_geometry;
pub use self::sphere_geometry::*;
pub mod plane_geometry;
pub use self::plane_geometry::*;
pub mod cylinder_geometry;
pub use self::cylinder_geometry::*;
pub mod box_geometry;
pub use self::box_geometry::*;
pub mod circle_geometry;
pub use self::circle_geometry::*;
pub mod geometry;
pub use self::geometry::*;
pub mod tri_uvs;
pub use self::tri_uvs::*;
pub mod trimesh_geometry;
pub use self::trimesh_geometry::*;
pub mod cone_geometry;
pub use self::cone_geometry::*;
pub mod torus_geometry;
pub use self::torus_geometry::*;
pub mod ellipsoid_geometry;
pub use self::ellipsoid_geometry::*;
pub mod cyclide_geometry;
pub use self::cyclide_geometry::*;

pub fn vec3_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
