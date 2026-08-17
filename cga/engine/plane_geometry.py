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

    # ── 实体协议 (CSG 叶子 = 半空间) ───────────────────────────────
    # CSG 语境下平面定义为"实体侧 = 法向负侧"的半空间 (n·x ≤ dist),
    # 用于剖切 (intersection 裁剪) 与砍除 (difference)。平面无界,
    # 单独作 CSG 根节点无可见表面意义, 需与有界实体组合。

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越: ts (N,1), ns (N,1,3) 外法向 = n, valid (N,1)。"""
        n = mx.array(params[0], dtype=mx.float32)
        dist = params[1]
        denom = mx.sum(n * d, axis=-1)
        t = (dist - mx.sum(n * o, axis=-1)) / mx.where(
            mx.abs(denom) > 1e-9, denom, mx.full_like(denom, 1e-9)
        )
        valid = (mx.abs(denom) > 1e-9)[:, None]
        ts = mx.where(
            valid, t[:, None], mx.full((o.shape[0], 1), float("inf"), dtype=mx.float32)
        )
        ns = mx.broadcast_to(n[None, None, :], (o.shape[0], 1, 3))
        return ts, ns, valid

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试: n·p < dist (实体侧 = 法向负侧, 任意前导维度)。"""
        n = mx.array(params[0], dtype=mx.float32)
        dist = params[1]
        return mx.sum(p * n, axis=-1) < dist
