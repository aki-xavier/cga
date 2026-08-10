"""CGA 图元类与距离 (OOP 表面)。

代数运算 (gp/ip/op/dual/meet/norm/...) 是 Multivector 的方法,
实现见 cga.multivector (积表与 grade 掩码也在那里)。

表示约定:
  - 点/点对/线为**直接 (join) 形式**: Point (grade 1) / PointPair
    (grade 2) / Line (grade 3 = p1∧p2∧e∞); 关联判据 p.op(X) = 0。
  - 平面/球/圆为**对偶形式**: Plane, Sphere (grade 1 向量) / Circle
    (grade 2); 关联判据 p.ip(X) = 0。
  距离方法直接读取对偶形式的系数 (float64); meet 接受直接形式输入。

距离公式不走 float32 的 conformal 内积: sqrt(-2·p1·p2) 在远原点时
灾难性抵消 (实测 (1000,0,0)-(1001,0,0) 得 0.0)。null 基下 conformal
权重即 e0 系数 (槽 4), 显式存储, 无基换算抵消 —— 故距离用权重归一
欧氏坐标的 float64 欧氏公式。
"""

import math

import mlx.core as mx

from cga.multivector import Multivector

# ── 基向量 ────────────────────────────────────────────────────────

E1 = Multivector.vector(1, 0, 0, 0, 0)
E2 = Multivector.vector(0, 1, 0, 0, 0)
E3 = Multivector.vector(0, 0, 1, 0, 0)
E0 = Multivector.vector(0, 0, 0, 1, 0)
EINF = Multivector.vector(0, 0, 0, 0, 1)

# 基向量常量物化 (后台线程使用安全, 见 multivector.py 末尾注)
mx.eval(E1.values, E2.values, E3.values, E0.values, EINF.values)


# ── 图元类 ────────────────────────────────────────────────────────


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
        if isinstance(other, Point):
            x1, y1, z1 = self.coords()
            x2, y2, z2 = other.coords()
            dx, dy, dz = x1 - x2, y1 - y2, z1 - z2
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        if isinstance(other, (Plane, Sphere)):
            return other.dist(self)
        raise TypeError(f"Point.dist: unsupported target {type(other).__name__}")


class PointPair(Multivector):
    """点对 / 0-球 (grade 2, 直接形式): Pp = p1 ∧ p2。"""

    __slots__ = ()

    def __init__(self, p1: Multivector, p2: Multivector):
        """由两个共形点构造: Pp = p1 ∧ p2。"""
        super().__init__(p1.op(p2).values)


class Line(Multivector):
    """线 (grade 3, 直接形式): L = p1 ∧ p2 ∧ e∞。"""

    __slots__ = ()

    def __init__(self, p1: Multivector, p2: Multivector):
        """由两个共形点构造: L = p1 ∧ p2 ∧ e∞。"""
        super().__init__(p1.op(p2).op(EINF).values)


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

    def dist(self, p: Point) -> float:
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


class Sphere(Multivector):
    """球 (grade 1, 对偶形式): s = up(c) − ½ρ²·e∞。"""

    __slots__ = ()

    def __init__(self, center: tuple[float, float, float], radius: float):
        """由球心与半径构造。"""
        cx, cy, cz = center
        half_r2 = 0.5 * radius * radius
        s = Point(cx, cy, cz) - Multivector.vector(0, 0, 0, 0, half_r2)
        super().__init__(s.values)

    def dist(self, p: Point) -> float:
        """点 p 到球面的带号距离 (正=外, 负=内), float64。

        对偶球 s = w·(up(c) − ½ρ²e∞): 球心 c = v/w, ρ² = |c|² − 2f/w
        (v = 欧氏部分, w = e0 系数, f = e∞ 系数)。
        """
        w = float(self.values[4])  # e0 系数
        if abs(w) < 1e-12:
            raise ValueError("sphere multivector has no e0 component")
        v1, v2, v3 = (
            float(self.values[1]),
            float(self.values[2]),
            float(self.values[3]),
        )
        f = float(self.values[5])  # e∞ coefficient
        cx, cy, cz = v1 / w, v2 / w, v3 / w
        rho_sq = (v1 * v1 + v2 * v2 + v3 * v3) / (w * w) - 2.0 * f / w
        r = math.sqrt(max(0.0, rho_sq))
        x, y, z = p.coords()
        dx, dy, dz = x - cx, y - cy, z - cz
        return math.sqrt(dx * dx + dy * dy + dz * dz) - r


class Cylinder(Multivector):
    """圆柱 (轴 Line + 半径槽): 重建图元, 非单 blade 代数对象。

    内部 = 轴 Line blade (grade 3 直接形式) + 半径。距离走解析
    公式 (float64, 同 house 判例不碰 conformal ip):
    dist(p) = |n̂×(p−q)| − ρ, q = 轴上点, n̂ = 单位方向。"""

    __slots__ = ("radius", "_axis_dir", "_axis_point")

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
        self._axis_dir = (ux, uy, uz)
        self._axis_point = tuple(float(x) for x in axis_point)

    def dist(self, p: Point) -> float:
        """点 p 到柱面的带号距离 (正=外, 负=内), float64。"""
        x, y, z = p.coords()
        qx, qy, qz = self._axis_point
        ux, uy, uz = self._axis_dir
        dx, dy, dz = x - qx, y - qy, z - qz
        # n̂×(p−q) 的模 = |p−q − (p−q)·n̂·n̂|
        dot = dx * ux + dy * uy + dz * uz
        ex, ey, ez = dx - dot * ux, dy - dot * uy, dz - dot * uz
        d = math.sqrt(ex * ex + ey * ey + ez * ez)
        return d - self.radius


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
