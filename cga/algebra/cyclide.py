"""Dupin cyclide —— 非 blade 的四次曲面, 及它的 blade 构造。

(理论地位)
5D CGA 的 blade (IPNS) 是广义球 (二次曲面): 球/平面/点, meet 得圆/点对。
Dupin cyclide 是四次曲面 (canal surface, 两族曲率线都是圆), 不是 blade
—— 但它恰好是两种 blade 构造的产物:

  1. 一参数球族包络: 一族球 S(u) = Sphere(E(u), r(u)) (Sphere blade),
     球心 E(u) 沿直接rix 椭圆, 半径 r(u) = d − c·cos u; 每颗球与两颗固定
     焦球相切 (Maxwell 性质), 包络即 cyclide。特征圆 (曲率线) =
     相邻球 S(u) ∧ S'(u) 的 meet (两球之交 = Circle blade)。

  2. versor 反演: cyclide = 环面 (或圆柱/双锥) 在球反演下的像。球反演
     是一个 versor (单位球 blade s = e0 − ½e∞ 的 sandwich: x ↦ s·x·s),
     Möbius 变换保持球与相切关系, 故把环面的一族相切球映成 cyclide
     的一族相切球。

本模块是可计算的几何模型 (非 Multivector 子类): 给出球族/焦球/特征圆
(返回真实 Sphere/Circle blade)、曲面参数化与隐式方程, 供
cga.engine.CyclideGeometry 做解析射线求交 (四次方程)。

规范形 (设计参数 a, b, d; 恒有 c = √(a²−b²), a > b > 0):
  直接rix 椭圆   E(u) = (a·cos u, b·sin u, 0)        (xy 平面)
  焦双曲线        H(v) = (c/cos v, 0, b·tan v)        (xz 平面)
  隐式:  (x²+y²+z²+b²−d²)² − 4(ax−cd)² − 4b²y² = 0
  d 分型:  c<d<a 环型(ring) | d>a 纺锤型(spindle) | 0<d<c 尖型(horn)
  退化 a=b (c=0): 环面 (主半径 a, 副半径 d, 轴 z)。

参考: https://en.wikipedia.org/wiki/Dupin_cyclide
"""

from __future__ import annotations

import math

from cga.algebra.circle import Circle
from cga.algebra.point import Point
from cga.algebra.sphere import Sphere
from cga.multivector import Multivector


def _coords(mv: Multivector) -> tuple[float, float, float]:
    """权重归一欧氏坐标 (同 Point.coords, 但不要求 Point 类型)。"""
    w = float(mv.values[4])
    if abs(w) < 1e-12:
        raise ValueError("multivector has no e0 component; not a finite point")
    return (
        float(mv.values[1]) / w,
        float(mv.values[2]) / w,
        float(mv.values[3]) / w,
    )


class DupinCyclide:
    """椭圆型 Dupin cyclide 的几何模型 (设计参数 a, b, d)。

    shift: 焦锥中心相对规范原点的平移 (from_torus_inversion 会给出非零
    shift; 直接构造的规范 cyclide shift = 0)。曲面/隐式/梯度方法都在
    世界坐标 (含 shift) 下工作。
    """

    def __init__(
        self,
        a: float,
        b: float,
        d: float,
        shift: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        if not (a > b > 0.0):
            raise ValueError(f"需要 a > b > 0, 得到 a={a!r}, b={b!r}")
        if d <= 0.0:
            raise ValueError(f"需要 d > 0, 得到 d={d!r}")
        self.a = float(a)
        self.b = float(b)
        self.d = float(d)
        self.shift = (float(shift[0]), float(shift[1]), float(shift[2]))

    @property
    def c(self) -> float:
        """椭圆线性离心率 c = √(a²−b²)。"""
        return math.sqrt(self.a * self.a - self.b * self.b)

    @property
    def kind(self) -> str:
        c = self.c
        if c < self.d < self.a:
            return "ring"
        if self.d > self.a:
            return "spindle"
        return "horn"  # 0 < d < c (含 d == c 的尖点极限)

    # ── 球族包络 (blade 构造 1) ────────────────────────────────────

    def spine(self, u: float) -> tuple[float, float, float]:
        """直接rix 椭圆上的球心 E(u) = (a·cos u, b·sin u, 0)。"""
        return (self.a * math.cos(u), self.b * math.sin(u), 0.0)

    def radius(self, u: float) -> float:
        """生成球半径 r(u) = d − c·cos u (尖型可负 = 有向球)。"""
        return self.d - self.c * math.cos(u)

    def generator_sphere(self, u: float) -> Sphere:
        """一参数球族的一颗球 S(u) —— 真实的 Sphere blade。"""
        return Sphere(self.spine(u), self.radius(u))

    def focal_spheres(self) -> tuple[Sphere, Sphere]:
        """两颗固定焦球 (每颗 S(u) 都与它们相切, Maxwell 性质)。

        S₊ 与 S(u) 外切: 心 (c,0,0), 半径 a−d;  S₋ 与 S(u) 内切:
        心 (−c,0,0), 半径 a+d。环型时二者半径皆正。
        """
        return (
            Sphere((self.c, 0.0, 0.0), self.a - self.d),
            Sphere((-self.c, 0.0, 0.0), self.a + self.d),
        )

    def tangency_residual(self, u: float) -> tuple[float, float]:
        """相切残差 (应≈0): (|E−F₊| − r − (a−d), |E−F₋| + r − (a+d))。"""
        ex, ey, ez = self.spine(u)
        r = self.radius(u)
        d1 = math.dist((ex, ey, ez), (self.c, 0.0, 0.0))
        d2 = math.dist((ex, ey, ez), (-self.c, 0.0, 0.0))
        return (d1 - r - (self.a - self.d), d2 + r - (self.a + self.d))

    def characteristic_circle(self, u: float) -> Circle:
        """特征圆 (曲率线) = 相邻球 S(u) 与 S(u+du) 的 meet, 解析闭式。

        圆 = 球 S(u) ∩ 平面 (p−E)·E' = −r·r' (对 |p−E|²=r² 求导)。
        圆心 C = E − (r·r'/|E'|²)·E', 半径 ρ = √(r² − (r·r')²/|E'|²),
        平面法向 = E'。
        """
        c = self.c
        cu, su = math.cos(u), math.sin(u)
        E = (self.a * cu, self.b * su, 0.0)
        Ep = (-self.a * su, self.b * cu, 0.0)
        r = self.d - c * cu
        rp = c * su
        ep2 = self.a * self.a * su * su + self.b * self.b * cu * cu
        if ep2 < 1e-18:
            raise ValueError("特征圆退化 (直接rix 切向为零)")
        lam = (r * rp) / ep2
        center = (E[0] - lam * Ep[0], E[1] - lam * Ep[1], E[2] - lam * Ep[2])
        rho2 = r * r - lam * lam * ep2
        if rho2 <= 0.0:
            raise ValueError("特征圆半径非正 (尖点/退化)")
        return Circle(center, math.sqrt(rho2), Ep)

    # ── 曲面参数化 / 隐式 ──────────────────────────────────────────

    def surface(self, u: float, v: float) -> tuple[float, float, float]:
        """参数化 (u, v ∈ [0, 2π)) → 曲面点 (世界坐标, 含 shift)。"""
        a, b, c, d = self.a, self.b, self.c, self.d
        cu, cv = math.cos(u), math.cos(v)
        su, sv = math.sin(u), math.sin(v)
        den = a - c * cu * cv
        x = (d * (c - a * cu * cv) + b * b * cu) / den
        y = (b * su * (a - d * cv)) / den
        z = (b * sv * (c * cu - d)) / den
        return (x + self.shift[0], y + self.shift[1], z + self.shift[2])

    def implicit(self, x: float, y: float, z: float) -> float:
        """隐式 F(规范形) 在 (x,y,z) 处的值 (世界坐标, 含 shift)。

        F < 0 为实体内部 (环型/纺锤型); 尖型自交, 内/外无全局意义。
        """
        a, b, c, d = self.a, self.b, self.c, self.d
        x = x - self.shift[0]
        y = y - self.shift[1]
        z = z - self.shift[2]
        B = b * b - d * d
        rho = x * x + y * y + z * z
        return (rho + B) ** 2 - 4.0 * (a * x - c * d) ** 2 - 4.0 * b * b * y * y

    def gradient(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """∇F (世界坐标; 平移不变, 故 shift 不影响方向)。"""
        a, b, c, d = self.a, self.b, self.c, self.d
        x = x - self.shift[0]
        y = y - self.shift[1]
        z = z - self.shift[2]
        B = b * b - d * d
        rho = x * x + y * y + z * z
        g = rho + B
        return (
            4.0 * x * g - 8.0 * a * (a * x - c * d),
            4.0 * y * g - 8.0 * b * b * y,
            4.0 * z * g,
        )

    def normal(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """单位法向 (∇F 归一, 指向实体外侧)。"""
        gx, gy, gz = self.gradient(x, y, z)
        n = math.sqrt(gx * gx + gy * gy + gz * gz)
        if n < 1e-12:
            return (0.0, 0.0, 1.0)
        return (gx / n, gy / n, gz / n)

    def contains(self, x: float, y: float, z: float) -> bool:
        """点成员测试: F < 0。"""
        return self.implicit(x, y, z) < 0.0

    def uv(self, x: float, y: float, z: float) -> tuple[float, float]:
        """曲面上点 → (u, v) (双球族包络的反解, 闭式)。

        曲面上点同时落在椭圆族与双曲线族各一颗球上; 由相切 (重根) 条件
        消去歧义得: u = atan2(2by, 2(ax−cd)), v = atan2(2bz, d²+b²−|p|²)
        (v 相对参数化相差一个镜像 −v, 无碍纹理)。
        """
        a, b, c, d = self.a, self.b, self.c, self.d
        x = x - self.shift[0]
        y = y - self.shift[1]
        z = z - self.shift[2]
        rho = x * x + y * y + z * z
        u = math.atan2(2.0 * b * y, 2.0 * (a * x - c * d))
        v = math.atan2(2.0 * b * z, d * d + b * b - rho)
        return u, v

    # ── versor 反演 (blade 构造 2) ─────────────────────────────────

    @staticmethod
    def inversion_versor() -> Multivector:
        """单位球反演的 versor: 单位球 blade s = e0 − ½e∞ (s² = 1)。"""
        return Sphere((0.0, 0.0, 0.0), 1.0)

    @staticmethod
    def invert_point(p: Point) -> Point:
        """单位球反演 x ↦ x/|x|², 经 versor sandwich: p' = s·p·s。

        (s⁻¹ = s 因 s² = 1; 全局符号对 null 点无意义。)
        """
        s = DupinCyclide.inversion_versor()
        out = s.gp(p).gp(s)
        return Point(*_coords(out))

    @classmethod
    def from_torus_inversion(
        cls, major: float, minor: float, shift_x: float = 0.0
    ) -> DupinCyclide:
        """单位球反演把 (沿 x 平移 shift_x 的) 环面映成 cyclide, 反解参数。

        环面 (主半径 major, 副半径 minor, 轴 z) 与 x 轴的 4 交点
        x_i = shift_x ± (major ± minor), 反演后 y_i = 1/x_i; 由 4 点公式
        (Wikipedia) 反解设计参数 (a, b, c, d) 与焦锥中心 m0:
          a = ¼(y1+y2−y3−y4), d = ¼(y1−y2+y3−y4),
          c = ¼(−y1+y2+y3−y4), m0 = ¼(y1+y2+y3+y4), b = √(a²−c²)。

        返回规范 (a, b, d) 加上 shift = (m0, 0, 0); 降序排序保证环型。
        """
        R, r = float(major), float(minor)
        if not (R > r > 0.0):
            raise ValueError(f"需要 major > minor > 0, 得到 {(major, minor)}")
        s = float(shift_x)
        xs = [s + R + r, s + R - r, s - R + r, s - R - r]
        if any(abs(x) < 1e-12 for x in xs):
            raise ValueError("环面穿过反演球心 (x 交点过 0), 反演非环型")
        y1, y2, y3, y4 = sorted((1.0 / x for x in xs), reverse=True)
        a = 0.25 * (y1 + y2 - y3 - y4)
        d = 0.25 * (y1 - y2 + y3 - y4)
        c = 0.25 * (-y1 + y2 + y3 - y4)
        m0 = 0.25 * (y1 + y2 + y3 + y4)
        if abs(c) < 1e-12 * max(1.0, abs(a)):
            raise ValueError(
                "环面球心过反演中心 (shift_x=0) 时反演仍是环面 (c=0), "
                "需 shift_x ≠ 0 才得非退化 cyclide"
            )
        b = math.sqrt(a * a - c * c)
        return cls(a, b, d, shift=(m0, 0.0, 0.0))
