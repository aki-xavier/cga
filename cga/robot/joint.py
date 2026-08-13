"""CRDF 关节 (类型常量是类属性)。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from cga.motors import Motor


@dataclass(frozen=True)
class Joint:
    """关节。类型常量: Joint.REVOLUTE / PRISMATIC / CONTINUOUS / FIXED;
    Joint.MOVABLE = 三者可动类型。"""

    REVOLUTE: ClassVar[str] = "revolute"
    PRISMATIC: ClassVar[str] = "prismatic"
    CONTINUOUS: ClassVar[str] = "continuous"
    FIXED: ClassVar[str] = "fixed"
    MOVABLE: ClassVar[tuple[str, ...]] = (REVOLUTE, PRISMATIC, CONTINUOUS)

    name: str
    type: str
    parent: str
    child: str
    origin: Motor
    axis: tuple[float, float, float] | None = None
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    damping: float | None = None
