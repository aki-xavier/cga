import mlx.core as mx

from cga.motors import Motor


class LightBase:
    """灯光基类: to_camera 共轭进相机空间; direction_at 提供着色方向/衰减。"""

    def to_camera(self, motor: Motor) -> LightBase:
        raise NotImplementedError

    def direction_at(self, p: mx.array) -> tuple[mx.array, float]:
        """(光方向 (N,3) 单位向量, 衰减系数) —— 着色循环多态分发。"""
        raise NotImplementedError

    def far(self, p: mx.array) -> mx.array | None:
        """阴影射线最大距离 (None = 无限; 点光源 = 到光源的距离)。"""
        return None
