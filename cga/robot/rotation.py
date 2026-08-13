"""旋转换算 (URDF rpy 约定 ↔ 旋转矩阵)。"""

import math


class Rotation:
    """外旋 X-Y-Z (URDF 约定): R = Rz(γ)·Ry(β)·Rx(α)。"""

    @staticmethod
    def rpy_to_matrix(rpy: tuple[float, float, float]) -> list[list[float]]:
        """rpy → 旋转矩阵 R = Rz(γ)·Ry(β)·Rx(α)。"""
        a, b, c = rpy
        ca, sa = math.cos(a), math.sin(a)
        cb, sb = math.cos(b), math.sin(b)
        cc, sc = math.cos(c), math.sin(c)
        # Rx(a) 后 Ry(b) 后 Rz(c):
        return [
            [cb * cc, sa * sb * cc - ca * sc, ca * sb * cc + sa * sc],
            [cb * sc, sa * sb * sc + ca * cc, ca * sb * sc - sa * cc],
            [-sb, sa * cb, ca * cb],
        ]

    @staticmethod
    def matrix_to_rpy(R) -> tuple[float, float, float]:
        """旋转矩阵 → rpy (R = Rz·Ry·Rx), 逆函数; 万向节锁 (cosβ≈0) 用 γ=0 约定。"""
        (r00, r01, _), (r10, r11, _), (r20, r21, r22) = (R[0], R[1], R[2])
        cb = math.sqrt(r00 * r00 + r10 * r10)
        if cb > 1e-9:
            b = math.atan2(-r20, cb)
            a = math.atan2(r21, r22)
            c = math.atan2(r10, r00)
        else:  # 万向节锁: β = ±π/2, 取 γ = 0
            b = math.atan2(-r20, cb)
            a = math.atan2(-r01, r11)
            c = 0.0
        return (a, b, c)
