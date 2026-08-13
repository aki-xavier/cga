from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import mlx.core as mx

from cga.engine.color import Color
from cga.engine.material import Material

if TYPE_CHECKING:
    from cga.engine.ambient_light import AmbientLight


class MeshBasicMaterial(Material):
    """不接光照, 直接输出颜色 (three.js MeshBasicMaterial)。"""

    def __init__(self, color: Color | int = 0xFFFFFF, opacity: float = 1.0):
        self.color = Color(color) if isinstance(color, int) else color
        self.opacity = float(min(1.0, max(0.0, opacity)))

    def shade(
        self,
        p: mx.array,
        n: mx.array,
        d: mx.array,
        lights: Sequence,
        ambient: AmbientLight | None = None,
        vis: Sequence | None = None,
    ) -> mx.array:
        """无光照平涂: 每像素 = 材质颜色。"""
        return mx.broadcast_to(
            mx.array(self.color.rgb(), dtype=mx.float32), (p.shape[0], 3)
        )
