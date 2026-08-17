"""CSG 组合实体 —— 渲染层的射线区间布尔 (递归, 任意嵌套)。

每个实体叶子 (sphere/box/cylinder/plane 半空间/affine 包装) 提供两个
实体原语:

  crossings(params, o, d) -> (ts (N,K), ns (N,K,3), valid (N,K))
      射线与实体边界的全部穿越点 (升序 t + 该处外法向)
  contains(params, p) -> (N,) bool
      点成员测试 (任意前导维度)

CSG 节点 = 纯组合器: 收集子树全部穿越点 → 逐像素排序 → 在每个穿越点
两侧 δ 处做整树成员测试, 成员性发生翻转的最近穿越点即可见表面。
difference(A, B, C, ...) 语义 = A − (B ∪ C ∪ ...); intersection =
全部子树的交; union = 全部子树的并 (单一材质合并)。

设计后果 (如实标注):
  - 非实体图元 (circle 圆盘) 不能作 CSG 叶子 (TypeError)。
  - δ 双侧采样 (1e-4 场景单位) 对相切/共面退化配置可能漏翻转
    (两表面间距 < 2δ 时); ponytail: δ 随 |t| 自适应。
  - 法向取穿越点所在叶子的外法向, 进出方向由渲染器的朝向翻转
    统一处理 (与既有图元一致)。
  - 单材质: 整个 CSG 节点共享 Mesh 的材质 (多材质 CSG 需按叶子
    选材质, 留升级路径)。
"""

from __future__ import annotations

import mlx.core as mx

from cga.engine.geometry_base import GeometryBase, Solid
from cga.motors import Motor

_OPS = ("union", "intersection", "difference")
_DELTA = 1e-4  # 成员性双侧采样间距 (场景单位)


class CsgGeometry(GeometryBase):
    """CSG 组合实体: op(children) 的递归布尔。

    children: 实体图元 (实现 crossings/contains 协议) 列表, 可嵌套
    CsgGeometry。difference 语义 = children[0] − ∪ children[1:]。
    """

    def __init__(self, op: str, children: list[GeometryBase]):
        if op not in _OPS:
            raise ValueError(f"csg op must be one of {_OPS}, got {op!r}")
        if len(children) < 2:
            raise ValueError(f"csg {op} needs >= 2 children, got {len(children)}")
        solids: list[Solid] = []
        for child in children:
            if not isinstance(child, Solid):
                raise TypeError(
                    f"{type(child).__name__} 不能作 CSG 叶子 "
                    "(需要实体协议 crossings/contains)"
                )
            solids.append(child)
        self.op = op
        self.children = solids

    def to_camera(self, motor: Motor) -> tuple:
        return tuple(child.to_camera(motor) for child in self.children)

    # ── 实体协议 (节点 = 子树组合) ─────────────────────────────────

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """子树全部叶子的穿越点拼接 (非边界穿越在翻转判定中自然抵消)。"""
        ts_l = []
        ns_l = []
        vs_l = []
        for child, cp in zip(self.children, params):
            ts, ns, vs = child.crossings(cp, o, d)
            ts_l.append(ts)
            ns_l.append(ns)
            vs_l.append(vs)
        return (
            mx.concatenate(ts_l, axis=1),
            mx.concatenate(ns_l, axis=1),
            mx.concatenate(vs_l, axis=1),
        )

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """整树成员测试: union=任一, intersection=全部, difference=首减余。"""
        if self.op == "difference":
            first = self.children[0].contains(params[0], p)
            rest = mx.zeros_like(first)
            for child, cp in zip(self.children[1:], params[1:]):
                rest = mx.logical_or(rest, child.contains(cp, p))
            return mx.logical_and(first, mx.logical_not(rest))
        acc = self.children[0].contains(params[0], p)
        for child, cp in zip(self.children[1:], params[1:]):
            cc = child.contains(cp, p)
            acc = (
                mx.logical_or(acc, cc)
                if self.op == "union"
                else mx.logical_and(acc, cc)
            )
        return acc

    # ── 求交 (最近实体表面) ────────────────────────────────────────

    def _nearest_surface(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array, mx.array]:
        """(t, n, mask, owner_idx): 最近成员性翻转穿越点。"""
        ts, ns, _vs = self.crossings(params, o, d)
        order = mx.argsort(ts, axis=1)
        ts_s = mx.take_along_axis(ts, order, axis=1)
        ns_s = mx.take_along_axis(ns, order[:, :, None], axis=1)
        # 无效穿越 (t=inf) 的采样点会是 ±inf/NaN —— 污染下游 contains
        # 与仿射变换 (matmul 中 inf×0=NaN, 实测同行数据亦被腐蚀),
        # 替换为有限占位 0: 这些穿越本就被候选过滤 (isfinite) 排除。
        ts_f = mx.where(mx.isfinite(ts_s), ts_s, mx.zeros_like(ts_s))
        # 每个穿越点两侧 δ 处的整树成员性
        p_plus = o[:, None, :] + (ts_f + _DELTA)[:, :, None] * d[:, None, :]
        p_minus = o[:, None, :] + (ts_f - _DELTA)[:, :, None] * d[:, None, :]
        in_plus = self.contains(params, p_plus)
        in_minus = self.contains(params, p_minus)
        flip = mx.logical_and(
            in_plus != in_minus, mx.logical_and(ts_s > 1e-6, mx.isfinite(ts_s))
        )
        cand = mx.where(flip, ts_s, mx.full_like(ts_s, float("inf")))
        t = mx.min(cand, axis=1)
        mask = mx.isfinite(t)
        idx = mx.argmin(cand, axis=1)
        n = mx.take_along_axis(ns_s, idx[:, None, None], axis=1)[:, 0, :]
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        return t, n, mask, idx

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        t, n, mask, _idx = self._nearest_surface(params, o, d)
        return t, n, mask

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        """阴影射线: 与 intersect 同一最近表面 (逐位一致)。"""
        t, _n, mask, _idx = self._nearest_surface(params, o, d)
        return t, mask

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        """UV 委托给边界所属子节点 (δ 双侧成员性翻转判定归属)。"""
        uv = mx.zeros((p.shape[0], 2), dtype=mx.float32)
        found = mx.zeros((p.shape[0],), dtype=mx.bool_)
        for child, cp in zip(self.children, params):
            boundary = child.contains(cp, p + _DELTA * n) != child.contains(
                cp, p - _DELTA * n
            )
            pick = mx.logical_and(boundary, mx.logical_not(found))
            if hasattr(child, "uv_at"):
                uv_c = child.uv_at(cp, p, n)
                uv = mx.where(pick[:, None], uv_c, uv)
            found = mx.logical_or(found, pick)
        return uv

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple] | None:
        """union=并集, intersection=交集, difference=首子的界。"""
        bnds = [child.bounds_camera(cp) for child, cp in zip(self.children, params)]
        if self.op == "difference":
            return bnds[0]
        if self.op == "union":
            if any(b is None for b in bnds):
                return None
            bounded_u = [b for b in bnds if b is not None]
            lo = tuple(min(b[0][i] for b in bounded_u) for i in range(3))
            hi = tuple(max(b[1][i] for b in bounded_u) for i in range(3))
            return lo, hi
        # intersection: 有界子的交集 (全无界 → None)
        bounded = [b for b in bnds if b is not None]
        if not bounded:
            return None
        lo = tuple(max(b[0][i] for b in bounded) for i in range(3))
        hi = tuple(min(b[1][i] for b in bounded) for i in range(3))
        return lo, hi
