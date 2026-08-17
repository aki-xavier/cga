"""三角网格图元 —— 局部规范形 + Möller–Trumbore 批量求交 (非 CGA blade)。

三角网格不是 5D CGA blade, 走与 torus/cone 相同的射线逆变换路径:
射线变换进局部空间 (顶点定义处), 对全部三角形做 Möller–Trumbore
求交 (按射线数 N 自适应分块, 控制 (N,C,3) 中间张量显存)。

设计后果 (如实标注):
  - 暴力 O(N·F): 无 BVH —— 每帧每网格代价随面数线性 (ponytail:
    BVH/ kd-tree 加速结构)。定位: 挤出/放样/导入的中小网格 (~10³ 面)。
  - 平坦着色 (面法向); 顶点法向平滑插值留升级路径。
  - contains (CSG 成员测试) 用 +X 方向奇偶投射, 要求水密网格;
    射线恰穿边/顶点时奇偶可能抖动 (零测集, 不特殊处理)。
  - uv_at v1 返回 (0,0): 网格纹理坐标留升级路径。
"""

from __future__ import annotations

import mlx.core as mx

from cga.engine.affine_geometry import AffineGeometry
from cga.engine.geometry_base import GeometryBase

_IDENTITY3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_MAX_CROSSINGS = 16  # CSG 穿越点上限/射线 (超出截断, 如实标注)
_BUDGET = 32 * 2**20  # 中间张量显存预算 (字节)


def _cross(a: mx.array, b: mx.array) -> mx.array:
    """(…,3) 逐行叉积 (mlx 无 mx.cross)。"""
    return mx.stack(
        [
            a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1],
            a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2],
            a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0],
        ],
        axis=-1,
    )


class _TrimeshLocal(GeometryBase):
    """局部三角网格: params = 预计算数组元组。射线须为单位方向。"""

    def __init__(self, vertices: list, faces: list):
        if not faces:
            raise ValueError("trimesh needs >= 1 face")
        nv = len(vertices)
        for f in faces:
            if len(f) != 3 or min(f) < 0 or max(f) >= nv:
                raise ValueError(f"bad face {f} (vertices={nv})")
        verts = mx.array(vertices, dtype=mx.float32).reshape(nv, 3)
        fidx = mx.array(faces, dtype=mx.int32).reshape(-1, 3)
        v0 = verts[fidx[:, 0]]
        e1 = verts[fidx[:, 1]] - v0
        e2 = verts[fidx[:, 2]] - v0
        nrm = _cross(e1, e2)
        ln = mx.sqrt(mx.sum(nrm * nrm, axis=-1, keepdims=True))
        if float(mx.min(ln)) < 1e-12:
            raise ValueError("trimesh has degenerate (zero-area) faces")
        nrm = nrm / ln
        self._params = (v0, e1, e2, nrm)
        lo = mx.min(verts, axis=0).tolist()
        hi = mx.max(verts, axis=0).tolist()
        self._aabb = (tuple(lo), tuple(hi))
        self.n_faces = len(faces)

    def to_camera(self, motor) -> tuple:  # 仅被 AffineGeometry 以 identity 调用
        return self._params

    # ── Möller–Trumbore (分块) ────────────────────────────────────

    def _mt_all(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部 (射线 × 三角形) 命中: t (N,F), n (N,F,3), valid (N,F)。"""
        v0, e1, e2, nrm = params
        n = o.shape[0]
        f = v0.shape[0]
        chunk = max(4, min(256, _BUDGET // (n * 12 * 6)))
        ts = []
        for s in range(0, f, chunk):
            e = min(s + chunk, f)
            v0c = v0[s:e][None, :, :]  # (1,C,3)
            e1c = e1[s:e][None, :, :]
            e2c = e2[s:e][None, :, :]
            dc = d[:, None, :]  # (N,1,3)
            p = _cross(dc, e2c)  # (N,C,3)
            det = mx.sum(e1c * p, axis=-1)  # (N,C)
            ok = mx.abs(det) > 1e-10
            inv = 1.0 / mx.where(ok, det, mx.ones_like(det))
            sv = o[:, None, :] - v0c
            u = mx.sum(sv * p, axis=-1) * inv
            q = _cross(sv, e1c)
            v = mx.sum(dc * q, axis=-1) * inv
            t = mx.sum(e2c * q, axis=-1) * inv
            hit = mx.logical_and(
                ok,
                mx.logical_and(u >= -1e-9, v >= -1e-9),
            )
            hit = mx.logical_and(hit, u + v <= 1.0 + 1e-9)
            hit = mx.logical_and(hit, t > 1e-6)
            ts.append(mx.where(hit, t, mx.full_like(t, float("inf"))))
        tall = mx.concatenate(ts, axis=1)  # (N,F)
        valid = mx.isfinite(tall)
        nall = mx.broadcast_to(nrm[None, :, :], (n, f, 3))
        return tall, nall, valid

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越: ts (N,K) 升序 (K≤16 截断), ns, valid。"""
        tall, nall, _v = self._mt_all(params, o, d)
        order = mx.argsort(tall, axis=1)[:, :_MAX_CROSSINGS]
        ts = mx.take_along_axis(tall, order, axis=1)
        ns = mx.take_along_axis(nall, order[:, :, None], axis=1)
        return ts, ns, mx.isfinite(ts)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        tall, nall, _v = self._mt_all(params, o, d)
        t = mx.min(tall, axis=-1)
        mask = mx.isfinite(t)
        idx = mx.argmin(tall, axis=-1)
        n = mx.take_along_axis(nall, idx[:, None, None], axis=1)[:, 0, :]
        # 背面命中: 法向翻向射线侧 (网格无内外约定, 与渲染器一致)
        cos_i = -mx.sum(d * n, axis=-1, keepdims=True)
        n = mx.where(cos_i < 0.0, -n, n)
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        return t, n, mask

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试: +X 奇偶投射 (要求水密, 任意前导维度)。"""
        shape = p.shape[:-1]
        pts = p.reshape(-1, 3)
        d = mx.broadcast_to(mx.array([1.0, 0.0, 0.0], dtype=mx.float32), pts.shape)
        tall, _n, _v = self._mt_all(params, pts, d)
        count = mx.sum(mx.isfinite(tall).astype(mx.int32), axis=-1)
        return (count % 2 == 1).reshape(shape)

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        # v1: 网格无纹理坐标 (升级路径: 导入 glTF UV + 重心插值)
        return mx.zeros((p.shape[0], 2), dtype=mx.float32)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        tall, _n, _v = self._mt_all(params, o, d)
        t = mx.min(tall, axis=-1)
        return t, mx.isfinite(t)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        return self._aabb


class MeshGeometry(AffineGeometry):
    """三角网格 (vertices, faces 三角形索引)。局部空间即顶点定义空间,
    motor/linear 正常生效 (经 AffineGeometry 射线逆变换)。"""

    def __init__(self, vertices: list, faces: list):
        super().__init__(_TrimeshLocal(vertices, faces), _IDENTITY3)
        self.n_faces = len(faces)
