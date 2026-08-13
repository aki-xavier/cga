"""平面 (对偶形式, grade 1)。"""

import math

from cga.multivector import Multivector


class Plane(Multivector):
    """平面 (grade 1, 对偶形式): π = n + d·e∞, n 单位法向, d 到原点距离。"""

    __slots__ = ()

    def __init__(self, normal_vec: tuple[float, float, float], distance: float):
        """法向自动归一化; 零法向抛 ValueError。"""
        nx, ny, nz = normal_vec
        nl = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nl <= 1e-12:
            raise ValueError(f"plane normal vector is zero or degenerate: {normal_vec}")
        super().__init__(
            Multivector.vector(nx / nl, ny / nl, nz / nl, 0.0, distance).values
        )

    def dist(self, p) -> float:
        """点 p 到平面的带号距离: (n·x − d)/|n|, float64。"""
        x, y, z = p.coords()
        nx, ny, nz = (
            float(self.values[1]),
            float(self.values[2]),
            float(self.values[3]),
        )
        d = float(self.values[5])  # e∞ coefficient
        nl = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nl < 1e-12:
            return float("inf")
        return (nx * x + ny * y + nz * z - d) / nl
