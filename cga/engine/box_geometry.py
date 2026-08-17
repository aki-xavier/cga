import mlx.core as mx

from cga.algebra import Point
from cga.engine.geometry_base import GeometryBase, vecmat
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

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        c, axes, half = params
        ext = [sum(abs(axes[j][i]) * half[j] for j in range(3)) for i in range(3)]
        return (
            tuple(c[i] - ext[i] for i in range(3)),
            tuple(c[i] + ext[i] for i in range(3)),
        )

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

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        c = mx.array(params[0], dtype=mx.float32)
        axes = [mx.array(axis, dtype=mx.float32) for axis in params[1]]
        half = mx.array(params[2], dtype=mx.float32)
        q = p - c
        local = mx.stack([mx.sum(q * axis, axis=-1) for axis in axes], axis=-1)
        face = mx.argmax(mx.abs(local / half), axis=-1)
        # Cube projection: choose the two tangent local axes of the hit face.
        x = mx.where(face == 0, local[:, 2], local[:, 0])
        y = mx.where(face == 2, local[:, 1], local[:, 2])
        sx = mx.where(face == 0, half[2], half[0])
        sy = mx.where(face == 2, half[1], half[2])
        return mx.stack([x / (2.0 * sx) + 0.5, y / (2.0 * sy) + 0.5], axis=-1)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        """阴影射线: 只算 (t, mask), 与 intersect 逐位一致 (跳过法向)。"""
        c = mx.array(params[0], dtype=mx.float32)
        axes = params[1]
        half = params[2]
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
        inside_hit = mx.logical_and(valid, t_entry <= 1e-6)
        t = mx.where(mx.logical_and(valid, ~inside_hit), t_entry, t_exit)
        return t, valid

    # ── 实体协议 (CSG 叶子) ────────────────────────────────────────

    def crossings(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        """全部边界穿越: ts (N,2), ns (N,2,3) 相机空间外法向, valid (N,2)。"""
        c = mx.array(params[0], dtype=mx.float32)
        axes = params[1]
        half = params[2]
        oc = o - c
        ax = [mx.array(a, dtype=mx.float32) for a in axes]
        op = mx.stack([mx.sum(oc * a, axis=-1) for a in ax], axis=-1)
        dp = mx.stack([mx.sum(d * a, axis=-1) for a in ax], axis=-1)
        inv = 1.0 / dp
        half_a = mx.array(half, dtype=mx.float32)
        t0 = -inv * (op + half_a)
        t1 = -inv * (op - half_a)
        tmin = mx.minimum(t0, t1)
        tmax = mx.maximum(t0, t1)
        t_entry = mx.max(tmin, axis=-1)
        t_exit = mx.min(tmax, axis=-1)
        valid = t_entry < t_exit
        i_e = mx.argmax(tmin, axis=-1)
        i_x = mx.argmin(tmax, axis=-1)
        eye = mx.eye(3, dtype=mx.float32)
        dp_e = mx.take_along_axis(dp, i_e[:, None], axis=-1)
        dp_x = mx.take_along_axis(dp, i_x[:, None], axis=-1)
        # 盒框架外法向: 进入面 −sign(dp)·e_i, 退出面 +sign(dp)·e_i
        n_e = mx.take(eye, i_e, axis=0) * (-mx.sign(dp_e))
        n_x = mx.take(eye, i_x, axis=0) * mx.sign(dp_x)
        # 盒框架 → 相机空间: n_cam = Σ n_box_i · axes_i
        rot = mx.stack(ax, axis=0)  # (3,3) 行 = 盒轴
        n_e = vecmat(n_e, rot)
        n_x = vecmat(n_x, rot)
        inf = mx.full_like(t_entry, float("inf"))
        ts = mx.stack(
            [mx.where(valid, t_entry, inf), mx.where(valid, t_exit, inf)], axis=-1
        )
        return ts, mx.stack([n_e, n_x], axis=1), mx.stack([valid, valid], axis=-1)

    def contains(self, params: tuple, p: mx.array) -> mx.array:
        """点成员测试: 盒框架逐轴 |q_i| ≤ half_i (任意前导维度)。"""
        c = mx.array(params[0], dtype=mx.float32)
        axes = params[1]
        half = params[2]
        q = p - c
        inside = mx.ones(q.shape[:-1], dtype=mx.bool_)
        for i in range(3):
            a = mx.array(axes[i], dtype=mx.float32)
            qi = mx.sum(q * a, axis=-1)
            inside = mx.logical_and(inside, mx.abs(qi) <= half[i])
        return inside
