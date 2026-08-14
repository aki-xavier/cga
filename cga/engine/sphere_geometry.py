import mlx.core as mx

from cga.algebra import Sphere
from cga.engine.geometry_base import GeometryBase
from cga.motors import Motor


class SphereGeometry(GeometryBase):
    """球 (CGA Sphere blade)。构造参数只有半径, 尺寸建模的核心。"""

    def __init__(self, radius: float):
        if radius <= 0:
            raise ValueError(f"sphere radius must be > 0, got {radius}")
        self.blade = Sphere((0.0, 0.0, 0.0), radius)  # 局部系原点

    def to_camera(self, motor: Motor) -> tuple:
        s = motor.apply(self.blade)  # 对偶球 blade 共轭 (类型降级为 Multivector)
        return Sphere.from_dual(s)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        c, r = params
        return (
            tuple(c[i] - r for i in range(3)),
            tuple(c[i] + r for i in range(3)),
        )

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        r = params[1]
        oc = o - c
        b = 2.0 * mx.sum(oc * d, axis=-1)
        cq = mx.sum(oc * oc, axis=-1) - r * r
        disc = b * b - 4.0 * cq
        valid = disc > 1e-12
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        t1 = (-b - sq) / 2.0
        t2 = (-b + sq) / 2.0
        t = mx.where(mx.logical_and(valid, t1 > 1e-6), t1, t2)
        mask = mx.logical_and(valid, t > 1e-6)
        p = o + t[:, None] * d
        n = (p - c) / r
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        # 相机在球内: t1<0 取 t2, 法向翻向相机
        inside = mx.logical_and(mask, t1 <= 1e-6)
        n = mx.where(inside[:, None], -n, n)
        return t, n, mask

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        """阴影射线: 只算 (t, mask), 与 intersect 逐位一致。"""
        c = mx.array(params[0], dtype=mx.float32)
        r = params[1]
        oc = o - c
        b = 2.0 * mx.sum(oc * d, axis=-1)
        cq = mx.sum(oc * oc, axis=-1) - r * r
        disc = b * b - 4.0 * cq
        valid = disc > 1e-12
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        t1 = (-b - sq) / 2.0
        t2 = (-b + sq) / 2.0
        t = mx.where(mx.logical_and(valid, t1 > 1e-6), t1, t2)
        mask = mx.logical_and(valid, t > 1e-6)
        return t, mask
