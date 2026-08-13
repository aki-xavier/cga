"""CRDF 几何 (link frame 里的一个 blade)。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from cga.motors import Motor


@dataclass(frozen=True)
class Geometry:
    """link frame 里的一个几何 blade (可多角色复用)。

    blade 决定哪些参数字段有效:
      cylinder: radius, length (轴 = 局部 +Z, 与 URDF/engine 一致)
      box:      size = (w, h, d), 中心在原点
      sphere:   radius
      plane:    normal + distance (平面 n·x = d, CGA 扩展)
      circle:   radius (圆盘, 局部法向 +Z, CGA 扩展)
      mesh:     file (不透明字符串, 可含 package:// URI) + scale ——
                像 URDF 一样的文件引用, 引擎不渲染 (interop 用)
    """

    BLADES: ClassVar[tuple[str, ...]] = (
        "cylinder", "box", "sphere", "plane", "circle", "mesh",
    )
    ROLES: ClassVar[tuple[str, ...]] = ("visual", "collision")

    blade: str
    origin: Motor
    role: tuple[str, ...]
    material: str | None = None
    radius: float | None = None
    length: float | None = None
    size: tuple[float, float, float] | None = None
    normal: tuple[float, float, float] | None = None
    distance: float | None = None
    file: str | None = None
    scale: tuple[float, float, float] | None = None
