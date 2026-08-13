"""共形点 (直接形式, grade 1)。"""

import math

from cga.multivector import Multivector


class Point(Multivector):
    """共形点 (grade 1, 直接形式), null 向量 p·p = 0。

    p = e0 + x·e1 + y·e2 + z·e3 + ½(x²+y²+z²)·e∞
    """

    __slots__ = ()

    def __init__(self, x: float, y: float, z: float):
        """由欧氏坐标构造 (p·p = 0 自动满足)。"""
        r2 = x * x + y * y + z * z
        super().__init__(Multivector.vector(x, y, z, 1.0, 0.5 * r2).values)

    def coords(self) -> tuple[float, float, float]:
        """权重归一欧氏坐标 (e0 系数 = 齐次权重), float64。"""
        w = float(self.values[4])  # e0 coefficient
        if abs(w) < 1e-12:
            raise ValueError("multivector has no e0 component; not a finite point")
        return (
            float(self.values[1]) / w,
            float(self.values[2]) / w,
            float(self.values[3]) / w,
        )

    def dist(self, other: Multivector) -> float:
        """到 Point/Plane/Sphere 的 (带号) 欧氏距离, float64。"""
        # 函数级 import: Plane/Sphere 的构造又依赖 Point, 防循环
        from cga.algebra.plane import Plane
        from cga.algebra.sphere import Sphere

        if isinstance(other, Point):
            x1, y1, z1 = self.coords()
            x2, y2, z2 = other.coords()
            dx, dy, dz = x1 - x2, y1 - y2, z1 - z2
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        if isinstance(other, (Plane, Sphere)):
            return other.dist(self)
        raise TypeError(f"Point.dist: unsupported target {type(other).__name__}")
