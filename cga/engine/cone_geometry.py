"""圆锥图元 —— 局部规范形 + 凸体区间裁剪 (非 CGA blade)。

圆锥在 5D CGA 中没有 blade 表示, 走与 AffineGeometry 相同的射线
逆变换路径: 射线变换进局部规范形 (轴 +Z, 顶点 z=+h/2, 底面
z=−h/2 半径 r), 侧二次面与轴向区间 [−h, 0] (自顶点计) 裁剪。

k = r/h (半角正切), 隐式: F(x) = ρ² − (1+k²)s² = 0, s = z − h/2
实体: F ≤ 0 且 −h ≤ s ≤ 0 (凸体)。
"""

from __future__ import annotations

import math

import mlx.core as mx

from cga.engine.affine_geometry import AffineGeometry
from cga.engine.geometry_base import GeometryBase

_IDENTITY3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class _ConeLocal(GeometryBase):
    """局部规范圆锥: params = (r, h)。射线须为单位方向。"""

    def __init__(self, radius: float, height: float):
        if radius <= 0 or height <= 0:
            raise ValueError(f"cone radius/height must be > 0, got {(radius, height)}")
        self.radius = float(radius)
        self.height = float(height)

    def to_camera(self, motor) -> tuple:  # 仅被 AffineGeometry 以 identity 调用
        return (self.radius, self.height)

    def _interval(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array, mx.array, mx.array]:
        """(enter, exit, valid, n_enter, n_exit): 侧面 ∩ 轴向区间。"""
        r, h = params
        k = r / h
        k2 = 1.0 + k * k
        # w = o − apex, apex = (0,0,h/2)
        wz = o[:, 2] - h / 2.0
        dz = d[:, 2]
        wd = mx.sum(o * d, axis=-1) - h / 2.0 * dz  # w·d
        ww = mx.sum(o * o, axis=-1) - h * o[:, 2] + h * h / 4.0  # w·w
        a = 1.0 - k2 * dz * dz
        b = 2.0 * (wd - k2 * wz * dz)
        c = ww - k2 * wz * wz
        side_ok = mx.abs(a) > 1e-12
        a_s = mx.where(side_ok, a, mx.full_like(a, 1e-12))
        disc = b * b - 4.0 * a_s * c
        side_ok = mx.logical_and(side_ok, disc > 1e-12)
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        r_lo = (-b - sq) / (2.0 * a_s)
        r_hi = (-b + sq) / (2.0 * a_s)
        # 排序 (a<0 时求根公式两解互换)
        st0 = mx.minimum(r_lo, r_hi)
        st1 = mx.maximum(r_lo, r_hi)
        # 轴向区间 s ∈ [−h, 0]
        safe_dz = mx.where(mx.abs(dz) > 1e-9, dz, mx.full_like(dz, 1e-9))
        t_top = (0.0 - wz) / safe_dz  # s=0 (顶点平面)
        t_bot = (-h - wz) / safe_dz  # s=−h (底面)
        at0 = mx.minimum(t_top, t_bot)
        at1 = mx.maximum(t_top, t_bot)
        # 双锥内域 = F ≤ 0: a>0 时是根区间 [st0,st1]; a<0 时是其补集
        # (−∞,st0] ∪ [st1,+∞) —— 与轴向区间裁剪得两个候选区间。
        # 单瓣锥是凸体, 候选至多一个非空。
        pos_a = a > 1e-12
        # 候选 1 (a>0: 根区间; a<0: 左支)
        c1e = mx.where(pos_a, mx.maximum(st0, at0), at0)
        c1x = mx.where(pos_a, mx.minimum(st1, at1), mx.minimum(st0, at1))
        v1 = side_ok & (c1e < c1x)
        # 候选 2 (仅 a<0: 右支)
        c2e = mx.maximum(st1, at0)
        c2x = at1
        v2 = mx.logical_and(side_ok, mx.logical_not(pos_a)) & (c2e < c2x)
        enter = mx.where(v1, c1e, c2e)
        exit_ = mx.where(v1, c1x, c2x)
        valid = mx.logical_or(v1, v2)

        # 法向: 侧面梯度 ∇F = 2(w + t·d) − 2k2·s·ẑ, w = p − apex
        def side_n(t: mx.array) -> mx.array:
            p = o + t[:, None] * d
            s = p[:, 2] - h / 2.0
            g = mx.stack([p[:, 0], p[:, 1], s * (1.0 - k2)], axis=-1)
            norm = mx.sqrt(mx.sum(g * g, axis=-1, keepdims=True))
            return g / mx.where(norm > 1e-12, norm, mx.ones_like(norm))

        n_s0 = side_n(enter)
        n_s1 = side_n(exit_)
        # 端面法向: 先穿过的轴向平面为进入面 (t_top < t_bot → +ẑ 面)
        top_first = (t_top < t_bot)[:, None]
        ez = mx.array([0.0, 0.0, 1.0], dtype=mx.float32)[None, :]
        n_cap0 = mx.where(top_first, ez, -ez)
        n_cap1 = mx.where(top_first, -ez, ez)
        # 边界归属: enter/exit 取自 max/min 的哪一侧 (复制值相等比较精确)
        enter_cap = (enter == at0)[:, None]
        exit_cap = (exit_ == at1)[:, None]
        n0 = mx.where(enter_cap, n_cap0, n_s0)
        n1 = mx.where(exit_cap, n_cap1, n_s1)
        return enter, exit_, valid, n0, n1

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越: ts (N,2) 升序, ns (N,2,3) 外法向, valid (N,2)。"""
        enter, exit_, valid, n0, n1 = self._interval(params, o, d)
        inf = mx.full_like(enter, float("inf"))
        ts = mx.stack(
            [mx.where(valid, enter, inf), mx.where(valid, exit_, inf)], axis=-1
        )
        return ts, mx.stack([n0, n1], axis=1), mx.stack([valid, valid], axis=-1)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        enter, exit_, valid, n0, n1 = self._interval(params, o, d)
        hit_enter = mx.logical_and(valid, enter > 1e-6)
        hit_exit = mx.logical_and(valid, mx.logical_not(hit_enter))
        hit_exit = mx.logical_and(hit_exit, exit_ > 1e-6)
        t = mx.where(hit_enter, enter, exit_)
        n = mx.where(hit_enter[:, None], n0, -n1)  # 内起点: 出射面法向取反
        mask = mx.logical_or(hit_enter, hit_exit)
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        return t, n, mask

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试: F ≤ 0 且 −h ≤ s ≤ 0 (任意前导维度)。"""
        r, h = params
        k2 = 1.0 + (r / h) ** 2
        s = p[..., 2] - h / 2.0
        f = p[..., 0] ** 2 + p[..., 1] ** 2 + s**2 - k2 * s**2
        return mx.logical_and(f <= 0.0, mx.logical_and(s >= -h, s <= 0.0))

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        _r, h = params
        u = mx.atan2(p[:, 1], p[:, 0]) / (2.0 * math.pi) + 0.5
        v = (h / 2.0 - p[:, 2]) / h  # 顶点 0 → 底面 1
        return mx.stack([u, v], axis=-1)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        enter, exit_, valid, _n0, _n1 = self._interval(params, o, d)
        hit_enter = mx.logical_and(valid, enter > 1e-6)
        t = mx.where(hit_enter, enter, exit_)
        mask = mx.logical_or(hit_enter, mx.logical_and(valid, exit_ > 1e-6))
        return t, mask

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        r, h = params
        return (-r, -r, -h / 2.0), (r, r, h / 2.0)


class ConeGeometry(AffineGeometry):
    """圆锥 (底面半径 r, 高 h)。局部轴 +Z, 中心在原点
    (顶点 +h/2, 底面 −h/2, 与 cylinder 的居中约定一致)。
    """

    def __init__(self, radius: float, height: float):
        super().__init__(_ConeLocal(radius, height), _IDENTITY3)
        self.radius = float(radius)
        self.height = float(height)
