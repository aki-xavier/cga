"""CRDF 惯量。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Inertial:
    """link 惯量 (全 6 分量张量, 质心 com 在 link frame)。"""

    mass: float
    com: tuple[float, float, float]
    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0
