"""共形几何代数 (CGA) 核心包。

5D CGA 把欧氏 3D 空间嵌入共形空间, 基为 {e1, e2, e3, e0, e∞},
其中 e0 是原点, e∞ 是无穷远点。

由此点/线/面/圆/球与刚体变换 (motor) 得以统一表示。

OOP 表面:
  - Multivector: 32 分量多重向量, 代数运算全是方法
    (gp/ip/op/reverse/dual/meet/norm/...)。
  - 图元类: Point / PointPair / Line (直接形式, 关联判据 p.op(X) = 0),
    Plane / Sphere / Circle (对偶形式, 关联判据 p.ip(X) = 0)。
  - Motor: 刚体变换 versor, O' = M.apply(O); exp/log/插值/速度提取
    都是类方法或方法。
"""

from cga.algebra import (
    E0,
    E1,
    E2,
    E3,
    EINF,
    Circle,
    Cylinder,
    Line,
    Plane,
    Point,
    PointPair,
    Sphere,
)
from cga.motors import Motor
from cga.multivector import Multivector
from cga.render import RenderPrimitive, RenderResult, render_scene

__all__ = [
    "E0",
    "E1",
    "E2",
    "E3",
    "EINF",
    "Circle",
    "Cylinder",
    "Line",
    "Motor",
    "Multivector",
    "Plane",
    "Point",
    "PointPair",
    "RenderPrimitive",
    "RenderResult",
    "Sphere",
    "render_scene",
]
