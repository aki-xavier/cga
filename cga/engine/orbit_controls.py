import math

from cga.engine.perspective_camera import PerspectiveCamera


class OrbitControls:
    """球面轨道 (three.js OrbitControls 的静态版): 改属性后 update()。

    azimuth/elevation 弧度, radius 距离, target 注视点。
    """

    def __init__(
        self,
        camera: PerspectiveCamera,
        target: tuple[float, float, float] = (0.0, 0.0, 0.0),
        azimuth: float = 0.0,
        elevation: float = 0.4,
        radius: float = 8.0,
    ):
        self.camera = camera
        self.target = target
        self.azimuth = float(azimuth)
        self.elevation = float(elevation)
        self.radius = float(radius)

    def update(self) -> None:
        ce = math.cos(self.elevation)
        x = self.radius * ce * math.sin(self.azimuth)
        y = self.radius * math.sin(self.elevation)
        z = self.radius * ce * math.cos(self.azimuth)
        self.camera.position = (
            self.target[0] + x,
            self.target[1] + y,
            self.target[2] + z,
        )
        self.camera.look_at(self.target)
