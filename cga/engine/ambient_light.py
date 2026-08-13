import mlx.core as mx

from cga.engine.color import Color
from cga.engine.light_base import LightBase
from cga.motors import Motor


class AmbientLight(LightBase):
    """环境光: 不吃变换 (to_camera 原样返回); 不进 per-light 循环
    (render 注册式路由, direction_at 不会被调用)。"""

    def __init__(self, color: Color | int = 0xFFFFFF, intensity: float = 0.3):
        self.color = Color(color) if isinstance(color, int) else color
        self.intensity = float(intensity)

    def to_camera(self, motor: Motor) -> AmbientLight:
        return self

    def direction_at(self, p: mx.array) -> tuple[mx.array, float]:
        raise NotImplementedError("ambient 光不进 per-light 循环")
