"""圆柱 (重建图元: 轴 Line blade + 半径槽)。"""

import math

from cga.algebra.line import Line
from cga.algebra.point import Point
from cga.multivector import Multivector


class Cylinder(Multivector):
    """圆柱 (轴 Line + 半径槽): 重建图元, 非单 blade 代数对象。

    内部 = 轴 Line blade (grade 3 直接形式) + 半径。距离走解析
    公式 (float64, 同 house 判例不碰 conformal ip):
    dist(p) = |n̂×(p−q)| − ρ, q = 轴上点, n̂ = 单位方向。

    axis_dir / axis_point 是公开属性; motor 共轭会丢 slots
    (apply 返回纯 Multivector) —— 圆柱暂不支持 motor 视角。
    """

    __slots__ = ("radius", "axis_dir", "axis_point")

    def __init__(
        self,
        axis_point: tuple[float, float, float],
        axis_dir: tuple[float, float, float],
        radius: float,
    ):
        """轴点 + 轴方向 (自动单位化) + 半径。"""
        ax, ay, az = axis_dir
        al = math.sqrt(ax * ax + ay * ay + az * az)
        if al <= 1e-12:
            raise ValueError(f"cylinder axis is degenerate: {axis_dir}")
        ux, uy, uz = ax / al, ay / al, az / al
        q = Point(*axis_point)
        q2 = Point(axis_point[0] + ux, axis_point[1] + uy, axis_point[2] + uz)
        super().__init__(Line(q, q2).values)
        self.radius = float(radius)
        self.axis_dir = (ux, uy, uz)
        self.axis_point = (
            float(axis_point[0]),
            float(axis_point[1]),
            float(axis_point[2]),
        )

    def dist(self, p: Point) -> float:
        """点 p 到柱面的带号距离 (正=外, 负=内), float64。"""
        x, y, z = p.coords()
        qx, qy, qz = self.axis_point
        ux, uy, uz = self.axis_dir
        dx, dy, dz = x - qx, y - qy, z - qz
        # n̂×(p−q) 的模 = |p−q − (p−q)·n̂·n̂|
        dot = dx * ux + dy * uy + dz * uz
        ex, ey, ez = dx - dot * ux, dy - dot * uy, dz - dot * uz
        d = math.sqrt(ex * ex + ey * ey + ez * ez)
        return d - self.radius
