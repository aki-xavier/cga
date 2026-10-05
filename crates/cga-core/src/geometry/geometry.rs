use super::*;

#[derive(Clone, Debug)]
pub enum Geometry {
    AffineGeometry(AffineGeometry),
    CsgGeometry(CsgGeometry),
    TrimeshGeometry(TrimeshGeometry),
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
