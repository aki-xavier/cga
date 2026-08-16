import mlx.core as mx

from cga.algebra import Plane
from cga.engine.geometry_base import GeometryBase
from cga.engine.vec3 import Vec3
from cga.motors import Motor


class PlaneGeometry(GeometryBase):
    """无限平面 (CGA Plane blade, 对偶形式 n + d·e∞)。"""

    def __init__(
        self,
        normal: tuple[float, float, float] = (0.0, 0.0, 1.0),
        distance: float = 0.0,
    ):
        self.blade = Plane(normal, distance)

    def to_camera(self, motor: Motor) -> tuple:
        pi = motor.apply(self.blade)
        n = Vec3.unit(Vec3.dir3(pi))
        d = float(pi.einf_coeff())
        return (n, d)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        n = mx.array(params[0], dtype=mx.float32)
        dist = params[1]
        denom = mx.sum(n * d, axis=-1)
        t = (dist - mx.sum(n * o, axis=-1)) / denom
        mask = mx.logical_and(mx.abs(denom) > 1e-9, t > 1e-6)
        n_rep = mx.broadcast_to(n, o.shape)
        return t, mx.where(mask[:, None], n_rep, mx.zeros_like(n_rep)), mask

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        # A deterministic planar projection. Infinite planes repeat in scene units.
        return mx.stack([p[:, 0], p[:, 2]], axis=-1)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        """阴影射线: 只算 (t, mask), 与 intersect 逐位一致。"""
        n = mx.array(params[0], dtype=mx.float32)
        dist = params[1]
        denom = mx.sum(n * d, axis=-1)
        t = (dist - mx.sum(n * o, axis=-1)) / denom
        mask = mx.logical_and(mx.abs(denom) > 1e-9, t > 1e-6)
        return t, mask
