"""圆 (对偶形式, grade 2)。"""

import math

from cga.algebra.plane import Plane
from cga.algebra.sphere import Sphere
from cga.multivector import Multivector


class Circle(Multivector):
    """圆 (grade 2, 对偶形式): C = 对偶球 ∧ 对偶平面。"""

    __slots__ = ()

    def __init__(
        self,
        center: tuple[float, float, float],
        radius: float,
        normal: tuple[float, float, float],
    ):
        """由球心/半径/所在平面法向构造 (球 ∧ 平面的交)。"""
        s = Sphere(center, radius)
        # Plane 归一化法向但不缩放 d, 故 d 须用单位法向计算 (零法向由 Plane 抛错)
        nl = math.sqrt(sum(n * n for n in normal))
        d_raw = sum(c * n for c, n in zip(center, normal, strict=True))
        d = d_raw / nl if nl > 1e-12 else 0.0
        p = Plane(normal, d)  # Plane 负责法向归一化与退化检查
        super().__init__(s.op(p).values)
