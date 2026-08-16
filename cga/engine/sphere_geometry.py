import math

import mlx.core as mx

from cga.algebra import Sphere
from cga.engine.geometry_base import GeometryBase
from cga.engine.vec3 import Vec3
from cga.motors import Motor
from cga.multivector import Multivector


class SphereGeometry(GeometryBase):
    """球 (CGA Sphere blade) with spherical local UV coordinates."""

    def __init__(self, radius: float):
        if radius <= 0:
            raise ValueError(f"sphere radius must be > 0, got {radius}")
        self.blade = Sphere((0.0, 0.0, 0.0), radius)
        self.radius = float(radius)

    def to_camera(self, motor: Motor) -> tuple:
        s = motor.apply(self.blade)
        c, r = Sphere.from_dual(s)
        axes = tuple(
            Vec3.unit(Vec3.dir3(motor.apply(axis)))
            for axis in (Multivector.E1, Multivector.E2, Multivector.E3)
        )
        return (c, r, axes)

    def bounds_camera(self, params: tuple) -> tuple[tuple, tuple]:
        c, r, _axes = params
        return (
            tuple(c[i] - r for i in range(3)),
            tuple(c[i] + r for i in range(3)),
        )

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        r = params[1]
        oc = o - c
        b = 2.0 * mx.sum(oc * d, axis=-1)
        cq = mx.sum(oc * oc, axis=-1) - r * r
        disc = b * b - 4.0 * cq
        valid = disc > 1e-12
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        t1 = (-b - sq) / 2.0
        t2 = (-b + sq) / 2.0
        t = mx.where(mx.logical_and(valid, t1 > 1e-6), t1, t2)
        mask = mx.logical_and(valid, t > 1e-6)
        p = o + t[:, None] * d
        n = (p - c) / r
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        inside = mx.logical_and(mask, t1 <= 1e-6)
        n = mx.where(inside[:, None], -n, n)
        return t, n, mask

    def uv_at(self, params: tuple, p: mx.array, n: mx.array) -> mx.array:
        c = mx.array(params[0], dtype=mx.float32)
        r = params[1]
        axes = [mx.array(axis, dtype=mx.float32) for axis in params[2]]
        q = p - c
        x = mx.sum(q * axes[0], axis=-1) / r
        y = mx.sum(q * axes[1], axis=-1) / r
        z = mx.clip(mx.sum(q * axes[2], axis=-1) / r, -1.0, 1.0)
        u = mx.atan2(y, x) / (2.0 * math.pi) + 0.5
        v = mx.acos(z) / math.pi
        return mx.stack([u, v], axis=-1)

    def intersect_shadow(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        r = params[1]
        oc = o - c
        b = 2.0 * mx.sum(oc * d, axis=-1)
        cq = mx.sum(oc * oc, axis=-1) - r * r
        disc = b * b - 4.0 * cq
        valid = disc > 1e-12
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        t1 = (-b - sq) / 2.0
        t2 = (-b + sq) / 2.0
        t = mx.where(mx.logical_and(valid, t1 > 1e-6), t1, t2)
        return t, mx.logical_and(valid, t > 1e-6)
