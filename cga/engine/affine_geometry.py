"""仿射几何包装器 —— 非 versor 可达变换 (scale/mirror/shear) 的引擎扩展。

CGA motor 是刚体 versor: 非均匀缩放与镜像不在 SE(3) 内, 无法用
blade 共轭表示 (缩放后的球不再是 Sphere blade, 而是椭球)。本模块用
经典射线逆变换把任意可逆线性映射 L 接到任意图元上, 而不改动各图元
的 blade 实现:

  世界变换 A = M · L (M = motor, 刚体; L = 3x3 线性, 可含镜像/剪切)
  局部空间:  o_l = A⁻¹o, d_l = A⁻¹d (逐射线归一化 d_u = d_l/|d_l|)
  命中:      t = t_local / |d_l|  (同一射线参数, 相机空间语义不变)
  法向:      n_cam = A⁻ᵀ · n_local  (逆置变换对 det<0 镜像同样正确)

设计后果 (如实标注):
  - 局部空间是各图元的规范形 (球心在原点、盒轴对齐), inner.intersect
    的实现被原样复用, 行为与未包装时逐位一致 (刚体部分不改变方向模长)。
  - 非均匀缩放引入剪切时, 法向不再垂直于视觉切面的情形由逆置变换
    正确处理; 但 blade 语义 (meet/关联判据) 不适用于仿射形变后的图元
    —— 本包装器只服务渲染管线 (求交/UV/包围盒), 不进代数层。
"""

from __future__ import annotations

import mlx.core as mx

from cga.engine.geometry_base import GeometryBase, Solid, vecmat
from cga.motors import Motor

# ── 纯 Python 3x3/4x4 助手 (CPU, 每帧每对象一次, 量小) ──────────────

_Mat3 = tuple[tuple[float, ...], ...]


def _as_mat4(l3: _Mat3) -> tuple[tuple[float, ...], ...]:
    """3x3 线性块 → 4x4 齐次矩阵 (零平移)。"""
    return (
        (l3[0][0], l3[0][1], l3[0][2], 0.0),
        (l3[1][0], l3[1][1], l3[1][2], 0.0),
        (l3[2][0], l3[2][1], l3[2][2], 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _mat3_inv(m: _Mat3) -> _Mat3:
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    ca = e * i - f * h
    cb = -(d * i - f * g)
    cc = d * h - e * g
    det = a * ca + b * cb + c * cc
    if abs(det) < 1e-15:
        raise ValueError(f"affine linear part is singular (det={det})")
    # 伴随矩阵 / det (行 = 代数余子式的转置)
    return (
        (ca / det, -(b * i - c * h) / det, (b * f - c * e) / det),
        (cb / det, (a * i - c * g) / det, -(a * f - c * d) / det),
        (cc / det, -(a * h - b * g) / det, (a * e - b * d) / det),
    )


def _mat4_mul(a: tuple[tuple[float, ...], ...], b: tuple[tuple[float, ...], ...]):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
        for i in range(4)
    )


def _to_mat4(m: list[list[float]]) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(float(v) for v in row) for row in m)


def is_identity3(m: _Mat3, eps: float = 1e-12) -> bool:
    return all(
        abs(m[i][j] - (1.0 if i == j else 0.0)) < eps
        for i in range(3)
        for j in range(3)
    )


class AffineGeometry(GeometryBase):
    """对任意图元施加可逆线性变换 L 的包装器 (scale/mirror/shear)。

    inner 保持 blade 语义 (局部规范形 + motor 共轭); 本类只把
    L 折叠进射线变换。L 必须可逆; det<0 (镜像) 由逆置法向自动处理。
    """

    def __init__(self, inner: GeometryBase, linear: _Mat3):
        if len(linear) != 3 or any(len(row) != 3 for row in linear):
            raise ValueError(f"linear must be 3x3, got {linear}")
        self.inner = inner
        self.linear: _Mat3 = tuple(tuple(float(v) for v in row) for row in linear)
        self._linv = _mat3_inv(self.linear)
        self._local_params: tuple | None = None  # 局部规范参数 (惰性缓存)

    # ── 每帧: 组合相机空间仿射 ─────────────────────────────────────

    def to_camera(self, motor: Motor) -> tuple:
        if self._local_params is None:
            # 局部规范参数: inner 在恒等 motor 下的共轭 (不变量, 缓存)
            self._local_params = self.inner.to_camera(Motor.identity())
        m4 = _to_mat4(motor.to_matrix())
        minv4 = _to_mat4(motor.inverse().to_matrix())
        # A = M·L;  A⁻¹ = L⁻¹·M⁻¹
        a_fwd = _mat4_mul(m4, _as_mat4(self.linear))
        a_inv = _mat4_mul(_as_mat4(self._linv), minv4)
        a_inv3 = tuple(tuple(a_inv[i][j] for j in range(3)) for i in range(3))
        t_inv = tuple(a_inv[i][3] for i in range(3))
        return (self._local_params, a_inv3, t_inv, a_fwd)

    # ── 射线逆变换核心 ─────────────────────────────────────────────

    @staticmethod
    def _to_local(
        params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        _lp, a_inv3, t_inv, _af = params
        a3 = mx.array(a_inv3, dtype=mx.float32)
        t3 = mx.array(t_inv, dtype=mx.float32)
        o_l = vecmat(o, mx.transpose(a3)) + t3
        d_l = vecmat(d, mx.transpose(a3))
        lam = mx.sqrt(mx.sum(d_l * d_l, axis=-1, keepdims=True))
        lam = mx.where(lam > 1e-12, lam, mx.ones_like(lam))
        return o_l, d_l / lam, lam

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        local_params, a_inv3, _t_inv, _af = params
        o_l, d_u, lam = self._to_local(params, o, d)
        t_l, n_l, mask = self.inner.intersect(local_params, o_l, d_u)
        t = t_l / lam.squeeze(-1)
        a3 = mx.array(a_inv3, dtype=mx.float32)
        n = vecmat(n_l, a3)  # 行向量形式: n_cam = n_l · A⁻¹ (即 A⁻ᵀ·n_l)
        norm = mx.sqrt(mx.sum(n * n, axis=-1, keepdims=True))
        n = n / mx.where(norm > 1e-12, norm, mx.ones_like(norm))
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        return t, n, mask

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        local_params = params[0]
        o_l, d_u, lam = self._to_local(params, o, d)
        t_l, mask = self.inner.intersect_shadow(local_params, o_l, d_u)
        return t_l / lam.squeeze(-1), mask

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        local_params, a_inv3, t_inv, a_fwd = params
        a3 = mx.array(a_inv3, dtype=mx.float32)
        t3 = mx.array(t_inv, dtype=mx.float32)
        p_l = vecmat(p, mx.transpose(a3)) + t3
        # 法向反推局部: n_l ∝ Aᵀ · n_cam (uv 只用方向比例, 无需归一)
        a_f3 = mx.array(
            tuple(tuple(a_fwd[i][j] for j in range(3)) for i in range(3)),
            dtype=mx.float32,
        )
        n_l = vecmat(n, a_f3)
        return self.inner.uv_at(local_params, p_l, n_l)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple] | None:
        local_params, _a_inv3, _t_inv, a_fwd = params
        bnd = self.inner.bounds_camera(local_params)
        if bnd is None:
            return None
        lo, hi = bnd
        xs = []
        ys = []
        zs = []
        for i in (lo[0], hi[0]):
            for j in (lo[1], hi[1]):
                for k in (lo[2], hi[2]):
                    x = (
                        a_fwd[0][0] * i
                        + a_fwd[0][1] * j
                        + a_fwd[0][2] * k
                        + a_fwd[0][3]
                    )
                    y = (
                        a_fwd[1][0] * i
                        + a_fwd[1][1] * j
                        + a_fwd[1][2] * k
                        + a_fwd[1][3]
                    )
                    z = (
                        a_fwd[2][0] * i
                        + a_fwd[2][1] * j
                        + a_fwd[2][2] * k
                        + a_fwd[2][3]
                    )
                    xs.append(x)
                    ys.append(y)
                    zs.append(z)
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    # ── 实体协议 (CSG 叶子, 委托 inner) ────────────────────────────

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越 (射线逆变换后委托 inner, t 重标定, 逆置法向)。"""
        local_params, a_inv3, _t_inv, _af = params
        if not isinstance(self.inner, Solid):
            raise TypeError(f"{type(self.inner).__name__} 不支持 CSG (非实体图元)")
        o_l, d_u, lam = self._to_local(params, o, d)
        ts, ns, valid = self.inner.crossings(local_params, o_l, d_u)
        a3 = mx.array(a_inv3, dtype=mx.float32)
        ns = vecmat(ns, a3)
        norm = mx.sqrt(mx.sum(ns * ns, axis=-1, keepdims=True))
        ns = ns / mx.where(norm > 1e-12, norm, mx.ones_like(norm))
        return ts / lam, ns, valid

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试 (点逆变换后委托 inner, 任意前导维度)。"""
        local_params, a_inv3, t_inv, _af = params
        if not isinstance(self.inner, Solid):
            raise TypeError(f"{type(self.inner).__name__} 不支持 CSG (非实体图元)")
        a3 = mx.array(a_inv3, dtype=mx.float32)
        t3 = mx.array(t_inv, dtype=mx.float32)
        p_l = vecmat(p, mx.transpose(a3)) + t3
        return self.inner.contains(local_params, p_l)


# ── 变换烘焙 (CSG 子节点定架 / glTF 导入) ────────────────────────────


def _mat3_transpose(m: _Mat3) -> _Mat3:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def _mat3_mul(a: _Mat3, b: _Mat3) -> _Mat3:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def decompose_rigid(m4: tuple[tuple[float, ...], ...]) -> tuple[Motor, _Mat3]:
    """4x4 仿射 → (motor, linear): A = T·R·L (Newton 极分解)。

    旋转因子必须是真旋转 (det=+1, versor 可达); det(B)<0 的反射分量
    吸收进 linear (F = diag(1,1,−1))。奇异 B 抛 ValueError。
    """
    b: _Mat3 = tuple(tuple(float(m4[i][j]) for j in range(3)) for i in range(3))
    t = (float(m4[0][3]), float(m4[1][3]), float(m4[2][3]))
    _mat3_inv(b)  # 奇异性检查 (det≈0 抛错)
    x = b
    for _ in range(30):
        xit = _mat3_transpose(_mat3_inv(x))
        x = tuple(
            tuple(0.5 * (x[i][j] + xit[i][j]) for j in range(3)) for i in range(3)
        )
    q = x  # 极分解正交因子
    det_q = (
        q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1])
        - q[0][1] * (q[1][0] * q[2][2] - q[1][2] * q[2][0])
        + q[0][2] * (q[1][0] * q[2][1] - q[1][1] * q[2][0])
    )
    lq = _mat3_mul(_mat3_transpose(q), b)  # B = Q·lq
    if det_q < 0:
        # 反射分量吸收进 linear: B = (Q·F)·(F·lq), F = diag(1,1,−1)
        flip: _Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0))
        q = _mat3_mul(q, flip)
        lq = _mat3_mul(flip, lq)
    motor = Motor.from_matrix([list(row) for row in q], list(t))
    return motor, lq


class TransformedGeometry(GeometryBase):
    """把 (motor, linear) 烘进任意几何的包装器。

    用途: CSG 子节点各自持有独立变换 (CsgGeometry 的孩子共享节点
    motor, 差异变换必须烘进几何); glTF 导入网格的节点矩阵。
    """

    def __init__(
        self,
        inner: GeometryBase,
        motor: Motor | None = None,
        linear: _Mat3 | None = None,
    ):
        if linear is not None and not is_identity3(linear):
            inner = AffineGeometry(inner, linear)
        self.inner = inner
        self.motor = motor if motor is not None else Motor.identity()

    def to_camera(self, motor: Motor) -> tuple:
        return self.inner.to_camera(motor.compose(self.motor))

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        return self.inner.intersect(params, o, d)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        return self.inner.intersect_shadow(params, o, d)

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        return self.inner.uv_at(params, p, n)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple] | None:
        return self.inner.bounds_camera(params)

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        if not isinstance(self.inner, Solid):
            raise TypeError(f"{type(self.inner).__name__} 不支持 CSG (非实体图元)")
        return self.inner.crossings(params, o, d)

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        if not isinstance(self.inner, Solid):
            raise TypeError(f"{type(self.inner).__name__} 不支持 CSG (非实体图元)")
        return self.inner.contains(params, p)
