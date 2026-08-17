"""CGA 图元类 (OOP 表面), 每图元一个文件。

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

基向量 (E1/E2/E3/E0/EINF) 是 Multivector 的类属性。
"""

from cga.algebra.circle import Circle
from cga.algebra.cyclide import DupinCyclide
from cga.algebra.cylinder import Cylinder
from cga.algebra.line import Line
from cga.algebra.plane import Plane
from cga.algebra.point import Point
from cga.algebra.point_pair import PointPair
from cga.algebra.sphere import Sphere
from cga.multivector import wrap_cpu_f64

# 图元自有方法 (coords/from_dual/distance 等直接索引 .values) 在
# float64 模式下同样需 CPU stream —— 与 Multivector/Motor 同一守卫。
for _cls in (Point, PointPair, Line, Plane, Sphere, Circle, Cylinder):
    wrap_cpu_f64(_cls)
del _cls

__all__ = [
    "Circle",
    "Cylinder",
    "DupinCyclide",
    "Line",
    "Plane",
    "Point",
    "PointPair",
    "Sphere",
]
