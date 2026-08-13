import mlx.core as mx

from cga.engine.color import Color
from cga.engine.light_base import LightBase
from cga.engine.vec3 import Vec3
from cga.motors import Motor
from cga.multivector import Multivector


class DirectionalLight(LightBase):
    """平行光。direction 是"光来的方向" (指向光源), 世界系单位向量。

    direction 以 e∞ 系数为 0 的 grade-1 向量存, motor 共轭只吃旋转、
    translator 天然不变 —— 相机平移不会改变平行光着色 (自检覆盖)。
    """

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        intensity: float = 1.0,
        direction: tuple[float, float, float] = (0.0, -1.0, 0.0),
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.intensity = float(intensity)
        self.direction = Vec3.unit(direction)

    def to_camera(self, motor: Motor) -> DirectionalLight:
        """方向只吃旋转 (translator 不改方向, 自检覆盖)。"""
        d_world = Multivector.vector(*self.direction)
        return DirectionalLight(
            self.color, self.intensity, Vec3.dir3(motor.apply(d_world))
        )

    def direction_at(self, p: mx.array) -> tuple[mx.array, float]:
        """方向恒定, 衰减 = 强度 (无距离概念)。"""
        ld = mx.broadcast_to(mx.array(self.direction, dtype=mx.float32), p.shape)
        return ld, self.intensity
