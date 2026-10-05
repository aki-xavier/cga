use crate::scene_graph::{identity3, vec3_dot, vec3_unit, Color, Object3D};
use crate::shading::{Light, Material};
use cga_core::{vec3_cross, Geometry, Multivector};

pub mod mesh;
pub use self::mesh::*;
pub mod mesh_params;
pub use self::mesh_params::*;
pub mod scene;
pub use self::scene::*;
pub mod perspective_camera;
pub use self::perspective_camera::*;
pub mod orbit_controls;
pub use self::orbit_controls::*;
