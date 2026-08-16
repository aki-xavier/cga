from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import mlx.core as mx

from cga.engine.color import Color
from cga.engine.material import Material
from cga.engine.texture import Texture

if TYPE_CHECKING:
    from cga.engine.ambient_light import AmbientLight


class MeshBasicMaterial(Material):
    """不接光照, 直接输出颜色 (three.js MeshBasicMaterial)。"""

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        opacity: float = 1.0,
        map: Texture | None = None,
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.opacity = float(min(1.0, max(0.0, opacity)))
        self.map = map

    def shade_params(self) -> tuple[mx.array, mx.array, mx.array, float]:
        """无光照平涂: emissive=材质色, diff/spec=0 → 直出颜色。

        expo 取 1.0 (spec_c=0, 高光项恒 0, 避免 pow(0,0) 边界)。
        """
        c = mx.array(self.color.rgb(), dtype=mx.float32)
        zero = mx.zeros(3, dtype=mx.float32)
        return c, zero, zero, 1.0

    def shade(
        self,
        p: mx.array,
        n: mx.array,
        d: mx.array,
        lights: Sequence,
        ambient: AmbientLight | None = None,
        vis: Sequence | None = None,
    ) -> mx.array:
        """无光照平涂: 每像素 = 材质颜色 (经共享批量着色, diff/spec=0)。"""
        emissive, diff_c, spec_c, expo = self.shade_params()
        n_px = p.shape[0]
        return Material.shade_batched(
            mx.broadcast_to(emissive, (n_px, 3)),
            mx.broadcast_to(diff_c, (n_px, 3)),
            mx.broadcast_to(spec_c, (n_px, 3)),
            mx.broadcast_to(mx.array(expo, dtype=mx.float32), (n_px, 1)),
            p,
            n,
            d,
            lights,
            ambient,
            vis,
        )
