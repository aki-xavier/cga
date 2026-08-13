"""测试共享断言/换算助手 (静态方法集合)。"""

import mlx.core as mx

from cga import Multivector


class Checks:
    """浮点比较与 sRGB 管线换算 (供 Test* 类继承)。"""

    @staticmethod
    def close(a: float, b: float, tol: float = 1e-4) -> bool:
        """|a−b| < tol。"""
        return abs(float(a) - float(b)) < tol

    @staticmethod
    def vmax(mv: Multivector) -> float:
        """分量的最大绝对值。"""
        return float(mx.abs(mv.values).max().item())

    @staticmethod
    def diff_max(a: Multivector, b: Multivector) -> float:
        """两 multivector 逐分量差的最大绝对值。"""
        return float(mx.abs(a.values - b.values).max().item())

    @staticmethod
    def srgb_to_linear(c: float) -> float:
        """sRGB 编码值 → 线性 (与引擎 Color 同式)。"""
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    @staticmethod
    def linear_to_srgb255(lum: float) -> int:
        """线性 → sRGB uint8 (与引擎 Renderer 输出端同式, 四舍五入)。"""
        lum = min(1.0, max(0.0, lum))
        s = 12.92 * lum if lum <= 0.0031308 else 1.055 * lum ** (1 / 2.4) - 0.055
        return min(255, max(0, int(s * 255 + 0.5)))
