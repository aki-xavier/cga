from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import mlx.core as mx

from cga.engine.color import Color
from cga.engine.material import Material

if TYPE_CHECKING:
    from cga.engine.ambient_light import AmbientLight


class MeshStandardMaterial(Material):
    """标准材质: 环境 + Lambert 漫反射 + Blinn-Phong 高光。

    metalness: 1 → 高光色 = 自身颜色 (金属), 0 → 高光白。
    roughness: 0..1 → 高光指数 200→4 (指数越低越糊)。
    v1 不做 PBR 能量守恒/IBL, 是廉价近似 (与 three.js 的差距如实标注)。
    """

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        roughness: float = 0.5,
        metalness: float = 0.0,
        emissive: Color | int = 0x000000,
        opacity: float = 1.0,
        ior: float = 1.5,
        absorption: float = 0.0,
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.roughness = float(min(1.0, max(0.0, roughness)))
        self.metalness = float(min(1.0, max(0.0, metalness)))
        self.emissive = Color(emissive) if isinstance(emissive, int) else emissive
        self.opacity = float(min(1.0, max(0.0, opacity)))
        self.ior = float(max(1.0, ior))
        self.absorption = float(max(0.0, absorption))

    def shade_params(self) -> tuple[mx.array, mx.array, mx.array, float]:
        """(emissive, diff, spec, expo) 标量参数 (线性空间)。"""
        diff_c = mx.array(self.color.rgb(), dtype=mx.float32) * (1.0 - self.metalness)
        spec_c = mx.array(
            tuple(
                m * (1.0 - self.metalness) + c * self.metalness
                for m, c in zip((1.0, 1.0, 1.0), self.color.rgb())
            ),
            dtype=mx.float32,
        )
        emissive = mx.array(self.emissive.rgb(), dtype=mx.float32)
        k = 1.0 - self.roughness
        expo = 4.0 + 196.0 * k * k  # roughness 0→200, 1→4
        return emissive, diff_c, spec_c, expo

    def shade(
        self,
        p: mx.array,
        n: mx.array,
        d: mx.array,
        lights: Sequence,
        ambient: AmbientLight | None = None,
        vis: Sequence | None = None,
    ) -> mx.array:
        """Blinn-Phong 着色 (p = 命中点, n = 相机空间法向, d = 射线方向)。

        单对象入口: 把自身标量参数广播到逐像素, 交给共享批量着色。
        """
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
