"""球 (对偶形式, grade 1)。"""

import math

from cga.algebra.point import Point
from cga.multivector import Multivector


class Sphere(Multivector):
    """球 (grade 1, 对偶形式): s = up(c) − ½ρ²·e∞。"""

    __slots__ = ()

    def __init__(self, center: tuple[float, float, float], radius: float):
        """由球心与半径构造。"""
        cx, cy, cz = center
        half_r2 = 0.5 * radius * radius
        s = Point(cx, cy, cz) - Multivector.vector(0, 0, 0, 0, half_r2)
        super().__init__(s.values)

    @classmethod
    def from_dual(cls, s: Multivector) -> tuple[tuple[float, float, float], float]:
        """对偶球 blade → (球心, 半径)。

        s = w·(up(c) − ½ρ²e∞): c = v/w, ρ² = |c|² − 2f/w (v = 欧氏部分,
        w = e0 系数, f = e∞ 系数)。motor 共轭后 blade 类型降级为普通
        Multivector, 实例方法不可用 —— 做成类方法 (同 Motor.from_matrix
        惯例), 引擎/render 共用同一公式。
        """
        w = float(s.e0_coeff())
        if abs(w) < 1e-12:
            raise ValueError("sphere multivector has no e0 component")
        v1, v2, v3 = s.euclidean_vector()
        f = float(s.einf_coeff())
        cx, cy, cz = v1 / w, v2 / w, v3 / w
        rho_sq = (v1 * v1 + v2 * v2 + v3 * v3) / (w * w) - 2.0 * f / w
        return (cx, cy, cz), math.sqrt(max(0.0, rho_sq))

    def dist(self, p: Point) -> float:
        """点 p 到球面的带号距离 (正=外, 负=内), float64。"""
        (cx, cy, cz), r = Sphere.from_dual(self)
        x, y, z = p.coords()
        dx, dy, dz = x - cx, y - cy, z - cz
        return math.sqrt(dx * dx + dy * dy + dz * dz) - r
