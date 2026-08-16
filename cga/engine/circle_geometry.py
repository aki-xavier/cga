import mlx.core as mx

from cga.algebra import Circle, Point
from cga.engine.geometry_base import GeometryBase
from cga.engine.vec3 import Vec3
from cga.motors import Motor
from cga.multivector import Multivector


class CircleGeometry(GeometryBase):
    """圆盘 (CGA Circle blade = 对偶球∧对偶平面; 解析存储中心/法向/半径)。"""

    def __init__(self, radius: float):
        if radius <= 0:
            raise ValueError(f"circle radius must be > 0, got {radius}")
        self.blade = Circle((0.0, 0.0, 0.0), radius, (0.0, 0.0, 1.0))  # 局部法向 +Z
        self.radius = float(radius)

    def to_camera(self, motor: Motor) -> tuple:
        c = motor.apply(Point(0.0, 0.0, 0.0)).coords()
        n = Vec3.unit(Vec3.dir3(motor.apply(Multivector.E3)))
        return (c, n, self.radius)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        # 圆盘 ⊂ 其外接球 (保守)
        c = params[0]
        r = params[2]
        return (
            tuple(c[i] - r for i in range(3)),
            tuple(c[i] + r for i in range(3)),
        )

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        n = mx.array(params[1], dtype=mx.float32)
        r = params[2]
        denom = mx.sum(n * d, axis=-1)
        t = mx.sum(n * (c - o), axis=-1) / denom
        front = denom < 0.0  # 法向面向相机的一侧
        p = o + t[:, None] * d
        in_disc = mx.sum((p - c) * (p - c), axis=-1) <= r * r
        mask = mx.logical_and(mx.logical_and(mx.abs(denom) > 1e-9, t > 1e-6), in_disc)
        n = mx.where(front[:, None], n, -n)  # 背面可见时翻向相机
        return t, mx.where(mask[:, None], n, mx.zeros_like(n)), mask

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        c = mx.array(params[0], dtype=mx.float32)
        r = params[2]
        q = (p - c) / (2.0 * r)
        return mx.stack([q[:, 0] + 0.5, q[:, 1] + 0.5], axis=-1)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        """阴影射线: 只算 (t, mask), 与 intersect 逐位一致。"""
        c = mx.array(params[0], dtype=mx.float32)
        n = mx.array(params[1], dtype=mx.float32)
        r = params[2]
        denom = mx.sum(n * d, axis=-1)
        t = mx.sum(n * (c - o), axis=-1) / denom
        p = o + t[:, None] * d
        in_disc = mx.sum((p - c) * (p - c), axis=-1) <= r * r
        mask = mx.logical_and(mx.logical_and(mx.abs(denom) > 1e-9, t > 1e-6), in_disc)
        return t, mask
