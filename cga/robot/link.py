"""CRDF 链接。"""

from __future__ import annotations

from dataclasses import dataclass

from cga.robot.geometry import Geometry
from cga.robot.inertial import Inertial


@dataclass(frozen=True)
class Link:
    name: str
    geometry: tuple[Geometry, ...] = ()
    inertial: Inertial | None = None
