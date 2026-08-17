"""环面图元 —— 局部规范形 + Durand-Kerner 四次求根 (非 CGA blade)。

环面在 5D CGA 中没有 blade 表示 (需 DCGA 双共形扩张), 故走与
AffineGeometry 相同的射线逆变换路径: 射线变换进局部规范形
(轴 +Z, 主半径 R, 副半径 r), 在局部空间解隐式四次方程

  F(x) = (x·x + R² − r²)² − 4R²(x² + y²) = 0

射线代入得 t 的首一四次多项式 (c4 = 1, 单位方向), 用 Durand-Kerner
复数迭代批量求全部 4 根 (MLX complex64, (N,4) 固定 50 轮), 滤出实根。

设计后果 (如实标注):
  - DK 对重根 (相切光线) 线性收敛, 50 轮后残余虚部可能偏大 ——
    相切是零测集, 过滤阈值 1e-3·max(1,|re|) 吸收。
  - 环面是"渲染层解析图元", 不参与代数层 meet/关联判据。
"""

from __future__ import annotations

import math

import mlx.core as mx

from cga.engine.affine_geometry import AffineGeometry
from cga.engine.geometry_base import GeometryBase

_DK_ITERS = 50
_IDENTITY3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class _TorusLocal(GeometryBase):
    """局部规范环面 (轴 +Z 过原点): params = (R, r)。射线须为单位方向。"""

    def __init__(self, major: float, minor: float):
        if major <= 0 or minor <= 0:
            raise ValueError(f"torus radii must be > 0, got {(major, minor)}")
        self.major = float(major)
        self.minor = float(minor)

    def to_camera(self, motor) -> tuple:  # 仅被 AffineGeometry 以 identity 调用
        return (self.major, self.minor)

    # ── 四次方程 Durand-Kerner 求根 ────────────────────────────────

    def _roots(self, params: tuple, o: mx.array, d: mx.array) -> mx.array:
        """(N,) 射线 → (N,4) 实根 (升序, 无效 = inf)。"""
        R, r = params
        oo = mx.sum(o * o, axis=-1)
        od = mx.sum(o * d, axis=-1)
        G = oo + R * R - r * r
        c3 = 4.0 * od
        c2 = 2.0 * G + 4.0 * od * od - 4.0 * R * R * (d[:, 0] ** 2 + d[:, 1] ** 2)
        c1 = 4.0 * od * G - 8.0 * R * R * (o[:, 0] * d[:, 0] + o[:, 1] * d[:, 1])
        c0 = G * G - 4.0 * R * R * (o[:, 0] ** 2 + o[:, 1] ** 2)
        # Cauchy 界定初始根半径
        rad = 1.0 + mx.max(
            mx.stack([mx.abs(c3), mx.abs(c2), mx.abs(c1), mx.abs(c0)], axis=-1),
            axis=-1,
            keepdims=True,
        )  # (N,1)
        seed = mx.array(
            [0.4 + 0.9j, -0.65 + 0.72j, -0.74 - 0.67j, 0.73 - 0.68j],
            dtype=mx.complex64,
        )
        z = rad.astype(mx.complex64) * seed[None, :]  # (N,4)
        c3c = c3.astype(mx.complex64)[:, None]
        c2c = c2.astype(mx.complex64)[:, None]
        c1c = c1.astype(mx.complex64)[:, None]
        c0c = c0.astype(mx.complex64)[:, None]
        for _ in range(_DK_ITERS):
            pz = c0c + z * (c1c + z * (c2c + z * (c3c + z)))  # Horner, c4=1
            diff = z[:, :, None] - z[:, None, :]  # (N,4,4)
            # 对角置 1: mx.eye 的 complex64 走 GPU scatter (不支持),
            # 用相等比较构造 (float32 → 加法提升 complex64)
            idx4 = mx.arange(4)
            eye = (idx4[:, None] == idx4[None, :]).astype(mx.float32)[None, :, :]
            diff = diff + eye
            denom = mx.prod(diff, axis=2)
            z = z - pz / denom
        re = mx.real(z)
        im = mx.imag(z)
        real_ok = mx.abs(im) <= 1e-3 * mx.maximum(1.0, mx.abs(re))
        roots = mx.where(real_ok, re, mx.full_like(re, float("inf")))
        return mx.sort(roots, axis=-1)

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越: ts (N,4) 升序, ns (N,4,3) 外法向, valid (N,4)。"""
        R, r = params
        ts = self._roots(params, o, d)
        valid = mx.isfinite(ts)
        safe_t = mx.where(valid, ts, mx.zeros_like(ts))
        p = o[:, None, :] + safe_t[:, :, None] * d[:, None, :]  # (N,4,3)
        s = mx.sum(p * p, axis=-1, keepdims=True) + R * R - r * r
        grad = s * p
        grad = grad.at[:, :, 0].add(-2.0 * R * R * p[:, :, 0])
        grad = grad.at[:, :, 1].add(-2.0 * R * R * p[:, :, 1])
        norm = mx.sqrt(mx.sum(grad * grad, axis=-1, keepdims=True))
        ns = grad / mx.where(norm > 1e-12, norm, mx.ones_like(norm))
        ns = mx.where(valid[:, :, None], ns, mx.zeros_like(ns))
        return ts, ns, valid

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        ts, ns, valid = self.crossings(params, o, d)
        pos = mx.logical_and(valid, ts > 1e-6)
        cand = mx.where(pos, ts, mx.full_like(ts, float("inf")))
        t = mx.min(cand, axis=-1)
        mask = mx.isfinite(t)
        idx = mx.argmin(cand, axis=-1)
        n = mx.take_along_axis(ns, idx[:, None, None], axis=1)[:, 0, :]
        # 起点在实体内部 → 命中的是出射面, 法向取反 (与 sphere 同约定)
        inside = mx.logical_and(mask, self.contains(params, o))
        n = mx.where(inside[:, None], -n, n)
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        return t, n, mask

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试: F(p) < 0 (任意前导维度)。"""
        R, r = params
        f = (mx.sum(p * p, axis=-1) + R * R - r * r) ** 2 - 4.0 * R * R * (
            p[..., 0] ** 2 + p[..., 1] ** 2
        )
        return f < 0.0

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        R, _r = params
        rho = mx.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
        u = mx.atan2(p[:, 1], p[:, 0]) / (2.0 * math.pi) + 0.5
        v = mx.atan2(p[:, 2], rho - R) / (2.0 * math.pi) + 0.5
        return mx.stack([u, v], axis=-1)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        ts, _ns, valid = self.crossings(params, o, d)
        pos = mx.logical_and(valid, ts > 1e-6)
        cand = mx.where(pos, ts, mx.full_like(ts, float("inf")))
        t = mx.min(cand, axis=-1)
        return t, mx.isfinite(t)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        R, r = params
        return (-(R + r), -(R + r), -r), (R + r, R + r, r)


class TorusGeometry(AffineGeometry):
    """环面 (主半径 R = 环心线半径, 副半径 r = 截面半径)。

    局部轴 +Z (与 cylinder 一致); 经 AffineGeometry 刚性逆变换进
    局部空间求交, motor 正常生效。
    """

    def __init__(self, major: float, minor: float):
        super().__init__(_TorusLocal(major, minor), _IDENTITY3)
        self.major = float(major)
        self.minor = float(minor)
