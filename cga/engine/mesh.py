from cga.engine.geometry_base import GeometryBase
from cga.engine.material import Material
from cga.engine.mesh_standard_material import MeshStandardMaterial
from cga.engine.object3d import Object3D
from cga.motors import Motor


class Mesh(Object3D):
    """几何 + 材质 + pose (three.js Mesh)。"""

    def __init__(
        self,
        geometry: GeometryBase,
        material: Material | None = None,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rotation_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        rotation_angle: float = 0.0,
        motor: Motor | None = None,
    ):
        super().__init__(position, rotation_axis, rotation_angle, motor)
        self.geometry = geometry
        self.material = material if material is not None else MeshStandardMaterial()
