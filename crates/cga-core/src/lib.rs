//! cga-core — 5D Conformal Geometric Algebra core.
//!
//! Pure f64 CPU. All submodules are re-exported flat at the crate root.

pub mod util;
pub use util::*;

// GP product table + grade masks are an implementation detail of the
// multivector algebra — not part of the public API.
// (dead_code: the generated table is kept complete for fidelity with the
// generator even where the algebra doesn't consult every column.)
#[allow(dead_code)]
pub(crate) mod tables;

pub mod multivector;
pub use multivector::*;

pub mod motors;
pub use motors::*;

pub mod primitives;
pub use primitives::*;

pub mod cyclide;
pub use cyclide::*;

pub mod geometry;
pub use geometry::*;

pub mod affine;
pub use affine::*;

pub mod affine_geom;
pub use affine_geom::*;

pub mod csg_node;
pub use csg_node::*;

pub mod modeling;
pub use modeling::*;

pub mod mesh_io;
pub use mesh_io::*;

pub mod gif;
pub use gif::*;
