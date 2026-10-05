use super::*;

#[derive(Clone, Debug)]
pub struct CsgGeometry {
    pub op: CsgOp,
    pub children: Vec<Geometry>,
}
impl CsgGeometry {
    pub fn new(op: CsgOp, children: Vec<Geometry>) -> CsgGeometry {
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
}
