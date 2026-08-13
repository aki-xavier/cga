"""三维向量小工具 (静态方法集合)。"""

import math

from cga.multivector import Multivector


class Vec3:
    """欧氏三元组 / multivector 向量部的常用操作。"""

    @staticmethod
    def dir3(a: Multivector) -> tuple[float, float, float]:
        """grade-1 向量 → 欧氏三元组 (走公开访问器, 忽略 e∞ 槽)。

        方向向量共轭后 e∞ 槽会混入 (t·u) 杂散项 (translator 写入), 方向
        语义只看向量部分 —— euclidean_vector() 只读 e1..e3 槽。"""
        return a.euclidean_vector()

    @staticmethod
    def unit(a: tuple[float, float, float]) -> tuple[float, float, float]:
        """单位化 (近零向量退回 +Z)。"""
        n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
        if n < 1e-12:
            return (0.0, 0.0, 1.0)
        return (a[0] / n, a[1] / n, a[2] / n)

    @staticmethod
    def cross(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """叉积。"""
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        """点积。"""
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
