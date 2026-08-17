from cga.engine.affine_geometry import AffineGeometry, is_identity3
from cga.engine.geometry_base import GeometryBase
from cga.engine.material import Material
from cga.engine.mesh_standard_material import MeshStandardMaterial
from cga.engine.object3d import Object3D
from cga.motors import Motor

_Mat3 = tuple[tuple[float, float, float], ...]


class Mesh(Object3D):
    """几何 + 材质 + pose (three.js Mesh)。

    linear (3x3, 默认恒等): scale/mirror 等线性块; 非恒等时 geometry
    被 AffineGeometry 包装 (世界变换 = motor · linear), 渲染器透明。
    """

    def __init__(
        self,
        geometry: GeometryBase,
        material: Material | None = None,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rotation_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        rotation_angle: float = 0.0,
        motor: Motor | None = None,
        linear: _Mat3 | None = None,
    ):
        super().__init__(position, rotation_axis, rotation_angle, motor, linear)
        if not is_identity3(self.linear):
            geometry = AffineGeometry(geometry, self.linear)
        self.geometry = geometry
        self.material = material if material is not None else MeshStandardMaterial()
