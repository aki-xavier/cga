#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CsgOp {
    Union,
    Difference,
    Intersection,
}
