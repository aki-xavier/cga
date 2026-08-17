from typing import Protocol, runtime_checkable

import mlx.core as mx

from cga.motors import Motor


def vecmat(v: mx.array, m: mx.array) -> mx.array:
    """(…,3) 行向量 × (3,3) 矩阵 → (…,3), 广播-求和实现。

    不用 mx.matmul: 其 GPU float32 路径 (Metal GEMM) 小矩阵降精度到
    ~bfloat16 (1.0001 → 1.0, 误差 1e-4 级), 坐标/法向变换不可用;
    逐元素乘 + 求和保持全精度 (motors.py 的 3x3 matmul 走 CPU stream,
    不受影响)。
    """
    return mx.sum(v[..., :, None] * m, axis=-2)


@runtime_checkable
class Solid(Protocol):
    """实体协议 (CSG 叶子): 几何接口 + 边界穿越点 + 点成员测试。

    实体图元 (sphere/box/cylinder/cone/torus/ellipsoid/plane 半空间/
    网格/affine 包装/CSG 节点) 隐式实现; circle 圆盘非实体 (无 crossings,
    isinstance 校验为 False)。
    """

    def to_camera(self, motor: Motor) -> tuple:
        """见 GeometryBase.to_camera。"""
        ...

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        """见 GeometryBase.uv_at。"""
        ...

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple] | None:
        """见 GeometryBase.bounds_camera。"""
        ...

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越: ts (N,K) 升序, ns (N,K,3) 外法向, valid (N,K)。"""
        ...

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试 (任意前导维度) → bool 数组。"""
        ...


class GeometryBase:
    """几何基类: CGA blade + 每帧共轭进相机空间 + 批量求交。

    to_camera(motor) -> params:  CPU, 每帧一次, versor 共轭出相机空间参数。
    intersect(params, o, d) -> (t, n, mask):  GPU, mlx 批量 (N,) 数组。
      t    最近命中距离 (inf = 未命中)
      n    (N,3) 单位法向 (指向相机侧)
      mask (N,) 命中掩码 (1/0 浮点)
    """

    def to_camera(self, motor: Motor) -> tuple:
        raise NotImplementedError

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        raise NotImplementedError

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        """Return stable local UV coordinates for camera-space hit points.

        ``p`` and ``n`` are (N, 3) arrays. Implementations derive coordinates
        from the geometry's local frame carried in ``params``; a Motor therefore
        moves the texture rigidly with its Mesh.
        """
        raise NotImplementedError

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        """阴影射线专用求交: 只返回 (t, mask), 跳过法向 (阴影不需要)。

        t/mask 必须与 intersect 逐位一致 (最近表面 + 命中掩码)。
        """
        raise NotImplementedError

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple] | None:
        """相机空间 AABB (lo, hi) (各为 3 元 float 的保守包围盒)。

        None = 无界 (无限平面/圆柱), 无法剔除。用于保守剔除: AABB 整体在
        相机后方 (hi.z <= 0) 时主射线 t>0 不可能命中, 跳过该对象求交。
        """
        return None
