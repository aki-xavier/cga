import mlx.core as mx

from cga.algebra import Point
from cga.engine.geometry_base import GeometryBase
from cga.engine.vec3 import Vec3
from cga.motors import Motor
from cga.multivector import Multivector


class BoxGeometry(GeometryBase):
    """轴对齐盒 (3 对平面 slab, 解析存储, 局部系中心在原点)。"""

    def __init__(self, width: float, height: float, depth: float):
        if min(width, height, depth) <= 0:
            raise ValueError(
                f"box dimensions must be > 0, got {(width, height, depth)}"
            )
        self.half = (width / 2.0, height / 2.0, depth / 2.0)

    def to_camera(self, motor: Motor) -> tuple:
        c = motor.apply(Point(0.0, 0.0, 0.0)).coords()
        # 局部轴方向经 motor 旋转 (translator 不变), 半尺寸不变
        axes = []
        for ax in (Multivector.E1, Multivector.E2, Multivector.E3):
            axes.append(Vec3.unit(Vec3.dir3(motor.apply(ax))))
        return (c, axes, self.half)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        axes = params[1]
        half = params[2]
        # 把射线变换进盒局部系: o' = Rᵀ(o−c), d' = Rᵀ·d
        oc = o - c
        op = mx.stack(
            [
                mx.sum(oc * mx.array(axes[i], dtype=mx.float32), axis=-1)
                for i in range(3)
            ],
            axis=-1,
        )
        dp = mx.stack(
            [
                mx.sum(d * mx.array(axes[i], dtype=mx.float32), axis=-1)
                for i in range(3)
            ],
            axis=-1,
        )
        inv = 1.0 / dp
        t0 = -inv * (op + mx.array(half, dtype=mx.float32))
        t1 = -inv * (op - mx.array(half, dtype=mx.float32))
        tmin = mx.minimum(t0, t1)
        tmax = mx.maximum(t0, t1)
        t_entry = mx.max(tmin, axis=-1)
        t_exit = mx.min(tmax, axis=-1)
        valid = mx.logical_and(t_entry < t_exit, t_exit > 1e-6)
        # 外命中: 可见面 = tmin 最大的轴 (进入面); 相机在内: 可见面 = 出口面
        i_entry = mx.argmax(tmin, axis=-1)
        i_exit = mx.argmin(tmax, axis=-1)
        inside_hit = mx.logical_and(valid, t_entry <= 1e-6)
        t = mx.where(mx.logical_and(valid, ~inside_hit), t_entry, t_exit)
        idx = mx.where(inside_hit, i_exit, i_entry)
        # 法向 = −sign(d'_{idx})·e_{idx} (两种命中同式, 恒指向相机侧)
        e = mx.eye(3, dtype=mx.float32)
        n = (
            mx.take(e, idx, axis=0)
            * (-mx.sign(mx.take_along_axis(dp, idx[:, None], axis=-1).squeeze(-1)))[
                :, None
            ]
        )
        n = mx.where(valid[:, None], n, mx.zeros_like(n))
        return t, n, valid
