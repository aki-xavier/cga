module cga

// CsgGeometry: recursive boolean combinator over solid primitives.

// CsgOp is the boolean operation of a CSG node.
pub enum CsgOp {
	union
	difference
	intersection
}

// CsgGeometry combines solid children via union / intersection / difference.
pub struct CsgGeometry {
pub:
	op       CsgOp
	children []Geometry
}

// csg_geometry builds a CSG node (difference = children[0] - union(children[1:])).
pub fn csg_geometry(op CsgOp, children []Geometry) CsgGeometry {
	if children.len < 2 {
		panic('csg ${op} needs >= 2 children, got ${children.len}')
	}
	for c in children {
		match c {
			CircleGeometry { panic('circle is not a solid (no crossings/contains)') }
			else {}
		}
	}
	return CsgGeometry{
		op:       op
		children: children
	}
}

pub struct CsgParams {
pub:
	op       CsgOp
	children []GeometryParams
}

// csg_crossings concatenates all children's boundary crossings.

// csg_contains is the whole-tree membership test.

// csg_nearest_surface finds the nearest membership-flip crossing.




