use crate::geometry::{Geometry, GeometryParams};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CsgOp {
    Union,
    Difference,
    Intersection,
}

#[derive(Clone, Debug)]
pub struct CsgGeometry {
    pub op: CsgOp,
    pub children: Vec<Geometry>,
}

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
