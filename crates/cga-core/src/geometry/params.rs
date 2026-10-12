use super::*;

#[derive(Clone, Debug)]
pub enum GeometryParams {
    AffineParams(AffineParams),
    CsgParams(CsgParams),
    CircleParams(CircleParams),
    ConeParams(ConeParams),
    CyclideParams(CyclideParams),
    EllipsoidParams(EllipsoidParams),
    TorusParams(TorusParams),
    BoxParams(BoxParams),
    CylinderParams(CylinderParams),
    PlaneParams(PlaneParams),
    SphereParams(SphereParams),
}
