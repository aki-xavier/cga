use super::*;

#[derive(Clone, Debug)]
pub enum GeometryParams {
    AffineParams(AffineParams),
    CsgParams(CsgParams),
    TrimeshParams(TrimeshParams),
    CircleParams(CircleParams),
    ConeParams(ConeParams),
    CyclideParams(CyclideParams),
    EllipsoidParams(EllipsoidParams),
    TorusParams(TorusParams),
    BoxParams(BoxParams),
    CylinderParams(CylinderParams),
    BezierParams(BezierParams),
    PlaneParams(PlaneParams),
    SphereParams(SphereParams),
}
