import mlx.core as mx

from cga.algebra import Point
from cga.engine.color import Color
from cga.engine.light_base import LightBase
from cga.motors import Motor


class PointLight(LightBase):
    """点光源: 位置 + 强度, 距离衰减 1/(1 + d²/8) (软衰减, v1 声明)。"""

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        intensity: float = 1.0,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.intensity = float(intensity)
        self.position = position

    def to_camera(self, motor: Motor) -> PointLight:
        """位置吃平移+旋转 (点共轭)。"""
        pos_cam = motor.apply(Point(*self.position)).coords()
        return PointLight(self.color, self.intensity, pos_cam)

    def far(self, p: mx.array) -> mx.array | None:
        lv = (
            mx.broadcast_to(mx.array(self.position, dtype=mx.float32), p.shape) - p
        )
        return mx.sqrt(mx.sum(lv * lv, axis=-1))

    def direction_at(self, p: mx.array) -> tuple[mx.array, float]:
        """方向 = 位置→命中点, 距离平方衰减。"""
        lv = (
            mx.broadcast_to(mx.array(self.position, dtype=mx.float32), p.shape) - p
        )
        dist2 = mx.sum(lv * lv, axis=-1, keepdims=True)
        return lv / mx.sqrt(dist2), self.intensity / (1.0 + dist2 / 8.0)
