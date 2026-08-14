import mlx.core as mx

from cga.algebra import Cylinder, Point
from cga.engine.geometry_base import GeometryBase
from cga.engine.vec3 import Vec3
from cga.motors import Motor
from cga.multivector import Multivector


class CylinderGeometry(GeometryBase):
    """圆柱 (CGA Cylinder = 轴 Line blade + 半径; 解析槽手动变换)。

    轴点走完整 motor (Point 共轭), 轴方向是 e∞ 系数为 0 的方向向量
    (translator 不变), 半径/长度在刚体运动下不变。
    length=None → CGA 无限圆柱 (v1 语义); length 给定 → 有限圆柱
    (端盖圆盘, 轴段 [−h, +h], 中心在原点)。
    """

    def __init__(self, radius: float, length: float | None = None):
        if radius <= 0:
            raise ValueError(f"cylinder radius must be > 0, got {radius}")
        if length is not None and length <= 0:
            raise ValueError(f"cylinder length must be > 0, got {length}")
        self.blade = Cylinder((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), radius)  # 局部轴 = +Z
        self.radius = float(radius)
        self.half = float(length) / 2.0 if length is not None else None

    def to_camera(self, motor: Motor) -> tuple:
        q = motor.apply(Point(0.0, 0.0, 0.0)).coords()
        # 方向只吃旋转, translator 天然不变
        u = Vec3.unit(Vec3.dir3(motor.apply(Multivector.E3)))
        return (q, u, self.radius, self.half)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        q = mx.array(params[0], dtype=mx.float32)
        u = mx.array(params[1], dtype=mx.float32)
        r = params[2]
        h = params[3]
        oc = o - q
        d_par = mx.sum(d * u, axis=-1, keepdims=True)
        o_par = mx.sum(oc * u, axis=-1, keepdims=True)
        d_p = d - d_par * u
        o_p = oc - o_par * u
        a = mx.sum(d_p * d_p, axis=-1)
        b = 2.0 * mx.sum(o_p * d_p, axis=-1)
        cq = mx.sum(o_p * o_p, axis=-1) - r * r
        disc = b * b - 4.0 * a * cq
        valid = mx.logical_and(a > 1e-12, disc > 1e-12)
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        t1 = (-b - sq) / (2.0 * a)
        t2 = (-b + sq) / (2.0 * a)
        t = mx.where(mx.logical_and(valid, t1 > 1e-6), t1, t2)
        mask = mx.logical_and(valid, t > 1e-6)
        hit = o_p + t[:, None] * d_p
        n = hit / r
        inside = mx.logical_and(mask, t1 <= 1e-6)
        n = mx.where(inside[:, None], -n, n)
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        if h is None:
            return t, n, mask  # 无限圆柱: 原 v1 路径
        # ── 有限圆柱: 侧面限制在轴段 |s| ≤ h + 两个端盖圆盘 ──
        s = o_par + t[:, None] * d_par  # 命中点的轴投影 (从 q 起)
        # 注意: (N,) 与 (N,1) 逐位与会把 (N,) 广播成 (1,N) → (N,N) 错位,
        # 必须先把 s 压成 (N,)
        side_ok = mx.logical_and(mask, mx.abs(s)[:, 0] <= h)
        # 端盖: 圆心 q ± h·u, 法向 ±u; 出射法向 = −sign(d·u)·u (朝相机侧)
        denom = d_par[:, 0]
        cap_t = mx.stack(
            [(h - o_par[:, 0]) / denom, (-h - o_par[:, 0]) / denom], axis=-1
        )  # (N,2)
        cap_ok = mx.broadcast_to((mx.abs(denom) > 1e-9)[:, None], (o.shape[0], 2))
        cap_ok = mx.logical_and(cap_ok, cap_t > 1e-6)  # (N,2)
        p_cap = o[:, None, :] + cap_t[:, :, None] * d[:, None, :]  # (N,2,3)
        lat = (
            p_cap
            - q[None, None, :]
            - mx.sum(
                (p_cap - q[None, None, :]) * u[None, None, :], axis=-1, keepdims=True
            )
            * u[None, None, :]
        )
        cap_ok = mx.logical_and(cap_ok, mx.sum(lat * lat, axis=-1) <= r * r)
        n_cap = -mx.sign(denom)[:, None] * u[None, :]  # 两个端盖同一出射法向 (N,3)
        n_cap = mx.stack([n_cap, n_cap], axis=1)  # (N,2,3)
        # 侧面 + 两端盖取最小 t; 法向随 argmin 选取
        t_all = mx.stack([t, cap_t[:, 0], cap_t[:, 1]], axis=-1)  # (N,3)
        ok_all = mx.stack([side_ok, cap_ok[:, 0], cap_ok[:, 1]], axis=-1)
        t_eff = mx.where(ok_all, t_all, mx.full_like(t_all, float("inf")))
        t_min = mx.min(t_eff, axis=-1)
        idx = mx.argmin(t_eff, axis=-1)
        n_all = mx.stack([n, n_cap[:, 0, :], n_cap[:, 1, :]], axis=1)  # (N,3,3)
        n_fin = mx.take_along_axis(
            n_all, mx.broadcast_to(idx[:, None, None], (n.shape[0], 1, 3)), axis=1
        )[:, 0, :]
        fin = mx.logical_and(mx.isfinite(t_min), t_min > 1e-6)
        return (
            mx.where(fin, t_min, t),  # 未命中时返回原 t (调用方按 mask 忽略)
            mx.where(fin[:, None], n_fin, mx.zeros_like(n_fin)),
            fin,
        )
