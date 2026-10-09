#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LightKind {
    Ambient,
    Directional,
    Point,
}
