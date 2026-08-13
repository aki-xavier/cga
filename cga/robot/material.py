"""CRDF 材质。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Material:
    name: str
    color: tuple[float, float, float, float]  # rgba, 0-1
