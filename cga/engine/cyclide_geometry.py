"""Dupin cyclide 图元 —— 局部规范形 + Durand-Kerner 四次求根 (非 CGA blade)。

cyclide 是四次曲面 (两族曲率线都是圆), 在 5D CGA 中没有 blade 表示,
走与 torus/cone 相同的射线逆变换路径: 射线变换进局部规范形, 在局部
空间解隐式四次方程

  F(x) = (x²+y²+z²+b²−d²)² − 4(ax−cd)² − 4b²y² = 0   (c=√(a²−b²))

射线代入得 t 的首一四次多项式 (c4=1, 单位方向), 用 Durand-Kerner
复数迭代批量求全部 4 根 (MLX complex64, (N,4) 固定 50 轮), 滤出实根。
系数推导见 cga/algebra/cyclide.py 与测试。

设计后果 (如实标注):
  - 与 torus 相同: DK 对重根 (相切光线) 线性收敛, 过滤阈值
    1e-3·max(1,|re|) 吸收相切零测集。
  - cyclide 是渲染层解析图元, 不参与代数层 meet/关联判据; 其 blade
    构造 (球族包络 / versor 反演) 见 cga.algebra.cyclide。
  - 实体内部 = F < 0, 仅对环型/纺锤型 (不自交) 有全局意义; 尖型自交,
    CSG 成员性语义退化 (如实标注)。
"""

from __future__ import annotations

import math

import mlx.core as mx

from cga.engine.affine_geometry import AffineGeometry
from cga.engine.geometry_base import GeometryBase

_DK_ITERS = 50
_IDENTITY3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class _CyclideLocal(GeometryBase):
    """局部规范 cyclide: params = (a, b, d, sx, sy, sz)。

    规范形 (设计参数 a, b, d, c=√(a²−b²), 直接rix 椭圆在 xy 平面);
    (sx,sy,sz) = 焦锥中心平移 (from_torus_inversion 非零)。射线须单位方向。
    """

    def __init__(
        self,
        a: float,
        b: float,
        d: float,
        shift: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        if not (a > b > 0.0):
            raise ValueError(f"cyclide needs a > b > 0, got {(a, b)}")
        if d <= 0.0:
            raise ValueError(f"cyclide needs d > 0, got {d}")
        self.a = float(a)
        self.b = float(b)
        self.d = float(d)
        self.c = math.sqrt(a * a - b * b)
        self.shift = (float(shift[0]), float(shift[1]), float(shift[2]))

    def to_camera(self, motor) -> tuple:  # 仅被 AffineGeometry 以 identity 调用
        return (self.a, self.b, self.d, *self.shift)

    # ── 四次方程 Durand-Kerner 求根 ────────────────────────────────

    def _roots(self, params: tuple, o: mx.array, d: mx.array) -> mx.array:
        """(N,) 射线 → (N,4) 实根 (升序, 无效 = inf)。"""
        a, b, dd, sx, sy, sz = params
        c = self.c
        ox = o[:, 0] - sx
        oy = o[:, 1] - sy
        oz = o[:, 2] - sz
        dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
        A = ox * ox + oy * oy + oz * oz
        B1 = ox * dx + oy * dy + oz * dz
        B = b * b - dd * dd
        G = A + B
        P0 = a * ox - c * dd
        P1 = a * dx
        c3 = 4.0 * B1
        c2 = 2.0 * G + 4.0 * B1 * B1 - 4.0 * P1 * P1 - 4.0 * b * b * dy * dy
        c1 = 4.0 * B1 * G - 8.0 * P0 * P1 - 8.0 * b * b * oy * dy
        c0 = G * G - 4.0 * P0 * P0 - 4.0 * b * b * oy * oy
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

    def _normal(self, params: tuple, p: mx.array) -> mx.array:
        """梯度 ∇F 归一 (p 已平移进规范形, (…,3))。"""
        a, b, _dd, _sx, _sy, _sz = params
        c = self.c
        B = b * b - self.d * self.d
        rho = mx.sum(p * p, axis=-1, keepdims=True)
        g = rho + B
        grad = mx.stack(
            [
                p[..., 0] * g[..., 0] * 4.0 - 8.0 * a * (a * p[..., 0] - c * self.d),
                p[..., 1] * g[..., 0] * 4.0 - 8.0 * b * b * p[..., 1],
                p[..., 2] * g[..., 0] * 4.0,
            ],
            axis=-1,
        )
        norm = mx.sqrt(mx.sum(grad * grad, axis=-1, keepdims=True))
        return grad / mx.where(norm > 1e-12, norm, mx.ones_like(norm))

    def _shift(self, params: tuple) -> mx.array:
        return mx.array(params[3:6], dtype=mx.float32)

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越: ts (N,4) 升序, ns (N,4,3) 外法向, valid (N,4)。"""
        ts = self._roots(params, o, d)
        valid = mx.isfinite(ts)
        safe_t = mx.where(valid, ts, mx.zeros_like(ts))
        sh = self._shift(params)
        p = o[:, None, :] - sh[None, None, :] + safe_t[:, :, None] * d[:, None, :]
        ns = self._normal(params, p)  # (N,4,3)
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
        inside = mx.logical_and(mask, self.contains(params, o))
        n = mx.where(inside[:, None], -n, n)
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        return t, n, mask

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试: F(p−shift) < 0 (任意前导维度)。"""
        a, b, _dd, sx, sy, sz = params
        c = self.c
        B = b * b - self.d * self.d
        x = p[..., 0] - sx
        y = p[..., 1] - sy
        z = p[..., 2] - sz
        rho = x * x + y * y + z * z
        f = (rho + B) ** 2 - 4.0 * (a * x - c * self.d) ** 2 - 4.0 * b * b * y * y
        return f < 0.0

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        a, b, _dd, sx, sy, sz = params
        c = self.c
        x = p[:, 0] - sx
        y = p[:, 1] - sy
        z = p[:, 2] - sz
        rho = x * x + y * y + z * z
        u = mx.atan2(2.0 * b * y, 2.0 * (a * x - c * self.d))
        v = mx.atan2(2.0 * b * z, self.d * self.d + b * b - rho)
        return mx.stack([u / (2.0 * math.pi) + 0.5, v / (2.0 * math.pi) + 0.5], axis=-1)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        ts, _ns, valid = self.crossings(params, o, d)
        pos = mx.logical_and(valid, ts > 1e-6)
        cand = mx.where(pos, ts, mx.full_like(ts, float("inf")))
        t = mx.min(cand, axis=-1)
        return t, mx.isfinite(t)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        """保守 AABB: 曲面在直接rix 椭圆的 (d+c) 距离邻域内。"""
        _a, b, _dd, sx, sy, sz = params
        r = self.d + self.c  # |r(u)| ≤ d+c
        return (
            (sx - self.a - r, sy - b - r, sz - r),
            (sx + self.a + r, sy + b + r, sz + r),
        )


class CyclideGeometry(AffineGeometry):
    """Dupin cyclide (设计参数 a, b, d; 可选焦锥中心平移 shift)。

    局部直接rix 椭圆在 xy 平面, 焦双曲线在 xz 平面; 经 AffineGeometry
    刚性逆变换进局部空间求交, motor 正常生效。环型 (c<d<a) 是光滑
    亏格-1 曲面 (CAD 圆角/混合/变径管的自然图元)。
    """

    def __init__(
        self,
        a: float,
        b: float,
        d: float,
        shift: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        super().__init__(_CyclideLocal(a, b, d, shift), _IDENTITY3)
        self.a = float(a)
        self.b = float(b)
        self.d = float(d)
        self.shift = (float(shift[0]), float(shift[1]), float(shift[2]))
