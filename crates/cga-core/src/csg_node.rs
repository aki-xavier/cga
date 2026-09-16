// CsgGeometry: recursive boolean combinator over solid primitives.

use crate::geometry::{Geometry, GeometryParams};

// CsgOp is the boolean operation of a CSG node.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CsgOp {
    Union,
    Difference,
    Intersection,
}

// CsgGeometry combines solid children via union / intersection / difference.
#[derive(Clone, Debug)]
pub struct CsgGeometry {
    pub op: CsgOp,
    pub children: Vec<Geometry>,
}

// csg_geometry builds a CSG node (difference = children[0] - union(children[1:])).
pub fn csg_geometry(op: CsgOp, children: Vec<Geometry>) -> CsgGeometry {
    if children.len() < 2 {
        panic!("csg {:?} needs >= 2 children, got {}", op, children.len());
    }
    for c in &children {
        if let Geometry::CircleGeometry(_) = c {
            panic!("circle is not a solid (no crossings/contains)")
        }
    }
    CsgGeometry { op, children }
}

#[derive(Clone, Debug)]
pub struct CsgParams {
    pub op: CsgOp,
    pub children: Vec<GeometryParams>,
}

// csg_crossings concatenates all children's boundary crossings.

// csg_contains is the whole-tree membership test.

// csg_nearest_surface finds the nearest membership-flip crossing.
