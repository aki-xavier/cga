"""椭球图元 = 单位球的仿射缩放 (组合, 零新求交数学)。

椭球不是 5D CGA blade (球 blade 经非均匀缩放后不再是球),
由 AffineGeometry 的射线逆变换获得精确解析求交, 并继承
实体协议 (crossings/contains)、UV 与包围盒。
"""

from cga.engine.affine_geometry import AffineGeometry
from cga.engine.sphere_geometry import SphereGeometry


class EllipsoidGeometry(AffineGeometry):
    """椭球 (半轴 rx, ry, rz, 局部中心在原点)。"""

    def __init__(self, rx: float, ry: float, rz: float):
        if min(rx, ry, rz) <= 0:
            raise ValueError(f"ellipsoid radii must be > 0, got {(rx, ry, rz)}")
        super().__init__(
            SphereGeometry(1.0), ((rx, 0.0, 0.0), (0.0, ry, 0.0), (0.0, 0.0, rz))
        )
        self.radii = (float(rx), float(ry), float(rz))
